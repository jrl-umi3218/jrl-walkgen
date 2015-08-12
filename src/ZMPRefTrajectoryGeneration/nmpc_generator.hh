/*
 * Copyright 2009, 2010, 2014
 *
 * Maximilien Naveau
 * Olivier  Stasse,
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/*! \file nmpc_generator.cpp
  \brief implement an SQP method to generate online stable walking motion */

#ifndef NMPC_GENERATOR_H
#define NMPC_GENERATOR_H

#include <jrl/walkgen/pgtypes.hh>
#include <Mathematics/relative-feet-inequalities.hh>
#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
#include <iomanip>
#include <cmath>
#include <qpOASES.hpp>

namespace PatternGeneratorJRL
{
  class NMPCgenerator
  {
  public:
    NMPCgenerator(SimplePluginManager *aSPM, CjrlHumanoidDynamicRobot *aHDR);
    ~NMPCgenerator();
    void initNMPCgenerator();
    void solve();

  private:

    //////////////////////
    // Solve the Problem :
    //////////////////////
    void preprocess_solution() ;
    void solve_qp()            ;
    void postprocess_solution();

    ///////////////////
    // Build Matrices :
    ///////////////////

    // Build Time Variant Matrices
    //////////////////////////////
    void initializeTimeVariantMatrix();
    void updateTimeVariantMatrix();
    void computeInitialGuess();

    void initializeFootSelectionMatrix();
    void updateFootSelectionMatrix();
    void computeSupportOrder();

    // build the constraints :
    void initializeCoPConstraint();
    void updateCoPConstraint();
    void initializeFootPoseConstraint();
    void updateFootPoseConstraint();
    void initializeFootVelIneqConstraint();
    void updateFootVelIneqConstraint();
    void initializeRotIneqConstraint();
    void updateRotIneqConstraint();
    void initializeObstacleConstraint();
    void updateObstacleConstraint();

    // build the cost function
    void initializeCostFunction();
    void updateCostFunction();

    // Build Constant Matrices
    //////////////////////////
    // construct the constant matrix depending
    // on the Euler integration scheme and the com height
    void buildConstantMatrix();
    void buildCoMIntegrationMatrix();
    void buildCoPIntegrationMatrix(); // depend on c_k_z_
    void buildConvexHullSystems(); // depend on the robot

  public:
    // Getter and Setter
    ////////////////////
    void setLocalVelocityReference(reference_t local_vel_ref);
    void setGlobalVelocityReference(reference_t local_vel_ref);

    inline support_state_t const & currentSupport() const
    { return currentSupport_; }
    inline support_state_t & currentSupport()
    { return currentSupport_; }
    inline void setNbStepsLeft(unsigned NbStepsLeft)
    { currentSupport_.NbStepsLeft=NbStepsLeft; }

    void getSolution(std::vector<double> JerkX,
                     std::vector<double> JerkY,
                     std::vector<double> FootStepX,
                     std::vector<double> FootStepY,
                     std::vector<double> FootStepYaw);
    inline std::deque<support_state_t> const & SupportStates_deq() const
    { return SupportStates_deq_ ; }

  private:
    SimplePluginManager * SPM_ ;
    CjrlHumanoidDynamicRobot * HDR_ ;

    // Time variant parameter
    /////////////////////////

    // [x, dx, ddx], com (pos, vel, acc) at instant t_k on axis X
    MAL_VECTOR_TYPE(double) c_k_x_;
    // [y, dy, ddy], com (pos, vel, acc) at instant t_k on axis Y
    MAL_VECTOR_TYPE(double) c_k_y_;

    /**
    NOTE number of foot steps in prediction horizon changes between
    nf and nf+1, because when robot takes first step nf steps are
    planned on the prediction horizon, which makes a total of nf+1 steps.
    */
    MAL_VECTOR_TYPE(double) v_kp1_ ;
    MAL_MATRIX_TYPE(double) V_kp1_ ;

    // Usefull for managing the PG

    // currentSupport_.x, support foot at time t_k on axis X
    // currentSupport_.y, support foot at time t_k on axis Y
    // currentSupport_.theta, support foot at time t_k around axis Z
    support_state_t currentSupport_ ;
    std::deque<support_state_t> SupportStates_deq_ ;

    // Constraint Matrix
    // Center of Pressure constraint
    unsigned nc_cop_ ;
    MAL_MATRIX_TYPE(double) Acop_xy_, Acop_theta_ ;
    MAL_VECTOR_TYPE(double) UBcop_, LBcop_ ;
    MAL_MATRIX_TYPE(double) D_kp1_xy_, D_kp1_theta_, Pzuv_, derv_Acop_map_  ;
    MAL_VECTOR_TYPE(double) b_kp1_, Pzsc_, Pzsc_x_, Pzsc_y_, v_kp1f_, v_kp1f_x_, v_kp1f_y_ ;

    // Foot position constraint
    unsigned nc_foot_ ;
    MAL_MATRIX_TYPE(double) Afoot_xy_, Afoot_theta_  ;
    MAL_VECTOR_TYPE(double) UBfoot_, LBfoot_ ;
    MAL_MATRIX_TYPE(double) SelecMat_, rotMat1_, rotMat2_, drotMat1_, drotMat2_, derv_Afoot_map_ ;
    MAL_MATRIX_TYPE(double) ASx_xy_, ASy_xy_, ASx_theta_, ASy_theta_ , AS_theta_;

    // Foot Velocity constraint
    unsigned nc_vel_ ;
    MAL_MATRIX_TYPE(double) Avel_ ;
    MAL_VECTOR_TYPE(double) UBvel_, LBvel_ ;
    // Rotation linear constraint
    unsigned nc_rot_ ;
    MAL_MATRIX_TYPE(double) Arot_ ;
    MAL_VECTOR_TYPE(double) UBrot_, LBrot_ ;
    // Obstacle constraint
    unsigned nc_obs_ ;
    std::vector< std::vector<MAL_MATRIX_TYPE(double)> > Hobs_ ;
    std::vector< std::vector<MAL_VECTOR_TYPE(double)> > Aobs_ ;
    std::vector< MAL_VECTOR_TYPE(double) > UBobs_, LBobs_ ;
    std::vector<Circle> obstacles_ ;

    // Cost Function
    unsigned nv_ ; // number of degrees of freedom
    // initial problem matrix
    MAL_MATRIX_TYPE(double) Q_x_, Q_theta_ , I_NN_ ;
    MAL_VECTOR_TYPE(double) p_x_, p_y_, p_theta_ ;
    // decomposition of p_xy_
    // p_xy_ = ( p_xy_X_, p_xy_Fx_, p_xy_Y_, p_xy_Fy_ )
    MAL_VECTOR_TYPE(double) p_xy_X_, p_xy_Fx_, p_xy_Y_, p_xy_Fy_ ;
    MAL_VECTOR_TYPE(double) Pvsc_x_ , Pvsc_y_ ;

    // decomposition of Q_x_=Q_y_
    // Q_x = ( Q_x_XX Q_x_XF ) = Q_y
    //       ( Q_x_FX Q_x_FF )
    MAL_MATRIX_TYPE(double) Q_x_XX_, Q_x_XF_, Q_x_FX_, Q_x_FF_ ;

    // Gauss-Newton Hessian approximation
    unsigned nc_ ;
    MAL_MATRIX_TYPE(double) qp_H_   ;
    MAL_VECTOR_TYPE(double) qp_g_   ;
    MAL_MATRIX_TYPE(double) qp_J_   ; //constraint Jacobian
    MAL_VECTOR_TYPE(double) qp_lbJ_ ; //constraint Jacobian
    MAL_VECTOR_TYPE(double) qp_ubJ_ ; //constraint Jacobian
    MAL_VECTOR_TYPE(double) qp_lb_  ;
    MAL_VECTOR_TYPE(double) qp_ub_  ;
    // temporary usefull variable for matrix manipulation
    MAL_VECTOR_TYPE(double) qp_g_x_, qp_g_y_, qp_g_theta_ ;
    MAL_MATRIX_TYPE(double) qp_J_cop_, qp_J_foot_, qp_J_vel_, qp_J_obs_, qp_J_rot_ ;
    MAL_VECTOR_TYPE(double) qp_lbJ_cop_, qp_lbJ_foot_, qp_lbJ_vel_, qp_lbJ_obs_, qp_lbJ_rot_ ;
    MAL_VECTOR_TYPE(double) qp_ubJ_cop_, qp_ubJ_foot_, qp_ubJ_vel_, qp_ubJ_obs_, qp_ubJ_rot_ ;

    // Free variable of the system
    // U_       = [C_kp1_x_ F_kp1_x_ C_kp1_y_ F_kp1_y_ F_kp1_theta_]^T
    // U_xy     = [C_kp1_x_ F_kp1_x_ C_kp1_y_ F_kp1_y_]^T
    // U_x      = [C_kp1_x_ F_kp1_x_]^T
    // U_y      = [C_kp1_y_ F_kp1_y_]^T
    // U_theta_ = [F_kp1_theta_]^T
    MAL_VECTOR_TYPE(double) U_           ;
    MAL_VECTOR_TYPE(double) U_xy_        ;
    MAL_VECTOR_TYPE(double) U_x_         ;
    MAL_VECTOR_TYPE(double) U_y_         ;
    MAL_VECTOR_TYPE(double) F_kp1_x_     ;
    MAL_VECTOR_TYPE(double) F_kp1_y_     ;
    MAL_VECTOR_TYPE(double) F_kp1_theta_ ;

    // Time constant parameter
    //////////////////////////

    // Sampling period of the SQP preview
    double T_ ;
    // Number of sample in the SQP preview
    unsigned N_ ;
    // Number of foot step planned in the preview
    unsigned nf_ ;
    // Step Period :
    double T_step_ ;
    // com height
    double c_k_z_ ;
    // security margin for the foot Hull in X and Y
    double SecurityMarginX_ ;
    double SecurityMarginY_ ;

    // Gain of the cost function :
    double alpha_ ;
    double beta_  ;
    double gamma_ ;

    // Integration Matrices

    // C_k+1_%    = position     of the CoM on the whole preview
    // dC_k+1_%   = velocity     of the CoM on the whole preview
    // ddC_k+1_%  = acceleration of the CoM on the whole preview
    // dddC_k+1_% = jerk         of the CoM on the whole preview (free variable)
    // Z_k+1_%    = position     of the ZMP on the whole preview (free variable)

    // C_k+1_% = Pps * c_k_% + Ppu dddC_k+1_%  (%={x,y})
    MAL_MATRIX_TYPE(double) Pps_ ;
    MAL_MATRIX_TYPE(double) Ppu_ ;
    // dC_k+1_% = Pvs * c_k_% + Pvu dddC_k+1_%
    MAL_MATRIX_TYPE(double) Pvs_ ;
    MAL_MATRIX_TYPE(double) Pvu_ ;
    // ddC_k+1_% = Pas * c_k_% + Pau dddC_k+1_%
    MAL_MATRIX_TYPE(double) Pas_ ;
    MAL_MATRIX_TYPE(double) Pau_ ;
    // Z_k+1_% = Pzs * c_k_% + Pzu dddC_k+1_%
    MAL_MATRIX_TYPE(double) Pzs_ ;
    MAL_MATRIX_TYPE(double) Pzu_ ;

    // Convex Hulls for ZMP and FootStep constraints :
    RelativeFeetInequalities * RFI_;
    // linear system corresponding to the foot step constraints
    // right foot polygonal hull
    MAL_MATRIX_TYPE(double) A0r_  ;
    MAL_VECTOR_TYPE(double) ubB0r_;
    // left foot polygonal hull
    MAL_MATRIX_TYPE(double) A0l_  ;
    MAL_VECTOR_TYPE(double) ubB0l_;

    // linear systems corresponding to the CoP constraints
    // right foot hull minus security margin
    MAL_MATRIX_TYPE(double) A0rf_    ;
    MAL_VECTOR_TYPE(double) ubB0rf_  ;
    // left foot hull minus security margin
    MAL_MATRIX_TYPE(double) A0lf_    ;
    MAL_VECTOR_TYPE(double) ubB0lf_  ;
    // foot hull minus security margin for standing still
    // or dealing with the switching mass phase
    MAL_MATRIX_TYPE(double) A0ds_   ;
    MAL_VECTOR_TYPE(double) ubB0ds_ ;

    // [Vx, Vy, Vtheta], reference velocity express in local frame
    // [Vx, Vy, Vtheta], reference velocity express in global frame
    // contain the vectors for the reference velocity over the
    // entire horizon
    reference_t vel_ref_ ;

    // QPoases data structure
    bool isQPinitialized_ ;
    qpOASES::SQProblem * QP_ ;
    qpOASES::Options options_ ;
    qpOASES::real_t* qpOases_H_  ;
    qpOASES::real_t* qpOases_g_  ;
    qpOASES::real_t* qpOases_J_  ;
    qpOASES::real_t* qpOases_lbJ ;
    qpOASES::real_t* qpOases_ubJ ;
    qpOASES::real_t* qpOases_lb_ ;
    qpOASES::real_t* qpOases_ub_ ;
    int nwsr_ ;
    qpOASES::real_t* deltaU_  ;
    qpOASES::real_t* cput_ ;


  };


// See if a derivation of a constraint class can simplify the code
//  class Constraint
//  {
//  public:
//    Constraint();
//    void buildConstantMatrix();
//    void buildUnconstantPart();
//    void jacobian();
//  };

}//End Namespace

#endif // NMPC_GENERATOR_H

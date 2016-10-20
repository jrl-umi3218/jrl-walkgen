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
#include <jrl/walkgen/pinocchiorobot.hh>
#include <iomanip>
#include <cmath>
#include <qpOASES.hpp>

namespace PatternGeneratorJRL
{
  class NMPCgenerator
  {
  public:
    NMPCgenerator(SimplePluginManager *aSPM, PinocchioRobot *aPR);

    ~NMPCgenerator();

    void initNMPCgenerator(bool useLineSearch, support_state_t &currentSupport,
                           COMState & lStartingCOMState,
                           reference_t & local_vel_ref,
                           unsigned N, unsigned nf, double T, double T_step);
    void updateInitialCondition(double time,
                                FootAbsolutePosition &currentLeftFootAbsolutePosition,
                                FootAbsolutePosition &currentRightFootAbsolutePosition,
                                COMState & currentCOMState,
                                reference_t & local_vel_ref
                                );
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
//    void updateFinalStateMachine(double time,
//        FootAbsolutePosition &FinalLeftFoot,
//        FootAbsolutePosition &FinalRightFoot);
    void updateCurrentSupport(double time,
        FootAbsolutePosition &FinalLeftFoot,
        FootAbsolutePosition &FinalRightFoot);
    void updateSupportdeque(double time,
        FootAbsolutePosition &FinalLeftFoot,
        FootAbsolutePosition &FinalRightFoot);
    void computeFootSelectionMatrix();

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
    void initializeStandingConstraint();
    void updateStandingConstraint();
    void initializeFootExactPositionConstraint();
    void updateFootExactPositionConstraint();

    // tools
    void computeAbsolutePositionFromRelative(
        support_state_t currentSupport,
        const RelativeFootPosition & relativePosition,
        support_state_t & nextSupport);

    // tools to check if foot is close to land
    void updateIterationBeforeLanding();
    bool isFootCloseToLand()
    {
//      std::cout << itBeforeLanding_ << std::endl;
      return (itBeforeLanding_ < 2 );
//      return false ;
    }

    // build the cost function
    void initializeCostFunction();
    void updateCostFunction();

    // tools for line search
    void initializeLineSearch();
    void lineSearch();
    void evalConstraint(MAL_VECTOR_TYPE(double) & U);
    double evalMeritFunctionJacobian();
    double evalMeritFunction();

    // Build Constant Matrices
    //////////////////////////

    // construct the constant matrix depending
    // on the Euler integration scheme and the com height
    void buildCoMCoPIntegrationMatrix(); // depend on c_k_z_
    void updateCoMCoPIntegrationMatrix();
    void buildConvexHullSystems(); // depend on the robot

  public:
    // Getter and Setter
    ////////////////////
    void setLocalVelocityReference(reference_t local_vel_ref);
    void setGlobalVelocityReference(reference_t global_vel_ref);

    inline support_state_t const & currentSupport() const
    { return currentSupport_; }
    inline support_state_t & currentSupport()
    { return currentSupport_; }
    inline void setNbStepsLeft(unsigned NbStepsLeft)
    { currentSupport_.NbStepsLeft=NbStepsLeft; }

    void getSolution(std::vector<double> &JerkX,
                     std::vector<double> &JerkY,
                     std::vector<double> &FootStepX,
                     std::vector<double> &FootStepY,
                     std::vector<double> &FootStepYaw);
    inline std::deque<support_state_t> const & SupportStates_deq() const
    { return SupportStates_deq_ ; }

    inline void addOneObstacle(double x, double y, double r)
    {
      Circle newObstacle ;
      newObstacle.x_0 = x ;
      newObstacle.y_0 = y ;
      newObstacle.r   = r ;
      newObstacle.margin = 0.4 ;
      obstacles_.push_back(newObstacle);
    }

    inline void deleteAllObstacles()
    {obstacles_.clear();}

    inline void updateOneObstacle(unsigned int id, double x, double y, double r)
    {
      if(id <= obstacles_.size())
      {
        obstacles_[id-1].x_0 = x ;
        obstacles_[id-1].y_0 = y ;
        obstacles_[id-1].r   = r ;
      }
    }

    RelativeFeetInequalities * RFI()
    {return RFI_;}

    // Sampling period of the SQP preview
    inline double T()
    {return T_;}
    inline void T(double T)
    {T_=T;}

    inline double Tfirst()
    {return Tfirst_ ;}

    // Number of sample in the SQP preview
    inline double N()
    {return N_;}
    inline void N(unsigned N)
    {N_=N;}

    // Number of planned foot step in the SQP preview
    inline double nf()
    {return nf_;}
    inline void nf(unsigned nf)
    {nf_=nf;}

    // Step Period :
    inline double T_step()
    {return T_step_;}
    inline void T_step(double T_step)
    {T_step_=T_step;}

    // cpu time consumption for one SQP
    inline double cput()
    {return *cput_ ;}

    // number of active set recalculation
    inline int nwsr()
    {return nwsr_ ;}

    std::deque <RelativeFootPosition> & relativeSupportDeque()
    {return desiredNextSupportFootRelativePosition;}


  private:
    SimplePluginManager * SPM_ ;
    PinocchioRobot * PR_ ;

    // Time variant parameter
    /////////////////////////

    double time_;

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
    FootAbsolutePosition currentLeftFootAbsolutePosition_;
    FootAbsolutePosition currentRightFootAbsolutePosition_;
    SupportFSM * FSM_ ;

    // Constraint Matrix
    // Center of Pressure constraint
    unsigned nc_cop_ ;
    MAL_MATRIX_TYPE(double) Acop_xy_, Acop_theta_ ;
    MAL_VECTOR_TYPE(double) UBcop_, LBcop_ ;
    MAL_MATRIX_TYPE(double) D_kp1_xy_, D_kp1_theta_, Pzuv_, derv_Acop_map_  ;
    MAL_MATRIX_TYPE(double) derv_Acop_map2_ ;
    MAL_VECTOR_TYPE(double) b_kp1_, Pzsc_, Pzsc_x_, Pzsc_y_, v_kp1f_, v_kp1f_x_, v_kp1f_y_ ;
    MAL_VECTOR_TYPE(double) v_kf_x_, v_kf_y_ ;
    MAL_MATRIX_TYPE(double) diffMat_ ;
    MAL_MATRIX_TYPE(double) rotMat_xy_, rotMat_theta_, rotMat_;
    MAL_MATRIX_TYPE(double) A0_xy_, A0_theta_;
    MAL_VECTOR_TYPE(double) B0_;
    MAL_MATRIX_TYPE(double) Acop_theta_dummy0_;
    MAL_VECTOR_TYPE(double) Acop_theta_dummy1_;

    // Foot position constraint
    unsigned nc_foot_ ;
    unsigned n_vertices_ ;
    unsigned itBeforeLanding_ ;
    int itMax_;
    std::vector<MAL_MATRIX_TYPE(double)> Afoot_xy_, Afoot_theta_  ;
    std::vector<MAL_VECTOR_TYPE(double)> UBfoot_, LBfoot_ ;
    std::vector<MAL_MATRIX_TYPE(double)> SelecMat_;
    std::vector<MAL_MATRIX_TYPE(double)> A0f_xy_, A0f_theta_ ;
    std::vector<MAL_VECTOR_TYPE(double)> B0f_;
    std::vector<MAL_MATRIX_TYPE(double)> rotMat_vec_, drotMat_vec_ ;
    MAL_MATRIX_TYPE(double) tmpRotMat_;
    std::vector<MAL_VECTOR_TYPE(double)> deltaF_ ;
    std::vector<MAL_VECTOR_TYPE(double)> AdRdF_ ;
    MAL_MATRIX_TYPE(double) Afoot_xy_full_, Afoot_theta_full_  ;
    MAL_VECTOR_TYPE(double) UBfoot_full_, LBfoot_full_ ;

    // Foot Velocity constraint
    unsigned nc_vel_ ;
    std::deque <RelativeFootPosition> desiredNextSupportFootRelativePosition ;
    std::vector<support_state_t> desiredNextSupportFootAbsolutePosition ;
    MAL_MATRIX_TYPE(double) Avel_ ;
    MAL_VECTOR_TYPE(double) UBvel_, LBvel_ ;

    // Foot Position constraint
    unsigned nc_pos_ ;
    MAL_MATRIX_TYPE(double) Apos_ ;
    MAL_VECTOR_TYPE(double) UBpos_, LBpos_ ;

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
    MAL_VECTOR_TYPE(double) qp_J_obs_i_ ;
    // Standing constraint :
    unsigned nc_stan_ ;
    MAL_MATRIX_TYPE(double) Astan_ ;
    MAL_VECTOR_TYPE(double) UBstan_, LBstan_ ;

    // evaluate constraint
    // real problem bounds : lb_ < g(U) < ub_
    MAL_VECTOR_TYPE(double) lb_  ;
    MAL_VECTOR_TYPE(double) ub_  ;
    MAL_VECTOR_TYPE(double) gU_  ;
    MAL_VECTOR_TYPE(double) Uxy_  ;
    MAL_VECTOR_TYPE(double) gU_cop_, gU_vel_ ;
    MAL_VECTOR_TYPE(double) gU_foot_ ;
    MAL_VECTOR_TYPE(double) gU_obs_, gU_rot_ , gU_stan_ ;

    // Cost Function
    unsigned nv_ ; // number of degrees of freedom
    // initial problem matrix
    MAL_MATRIX_TYPE(double) Q_theta_, I_NN_, I_FF_ ;

    // decomposition of p_xy_
    // p_xy_ = ( p_xy_X_, p_xy_Fx_, p_xy_Y_, p_xy_Fy_ )
    MAL_VECTOR_TYPE(double) p_xy_X_, p_xy_Fx_, p_xy_Y_, p_xy_Fy_ ;
    MAL_VECTOR_TYPE(double) Pvsc_x_ , Pvsc_y_ ;

    // decomposition of Q_x_=Q_y_
    // Q_x = ( Q_x_XX Q_x_XF ) = Q_y
    //       ( Q_x_FX Q_x_FF )
    MAL_MATRIX_TYPE(double) Q_x_XX_, Q_x_XF_, Q_x_FX_, Q_x_FF_ ;
    MAL_MATRIX_TYPE(double) Q_y_XX_;// Q_x_XX_ != Q_y_XX_

    // Line Search
    bool useLineSearch_ ;
    MAL_VECTOR_TYPE(double) p_ , U_n_, selectActiveConstraint ;
    MAL_VECTOR_TYPE(double) JdU_, contraintValue ;
    MAL_VECTOR_TYPE(double) HUn_ ;
    double lineStep_, lineStep0_, stepParam_ ; // step searched
    double mu_ ; // weight between cost function and constraints
    double cm_, c_ ; // Merit Function Jacobian
    double L_n_, L_ ; // Merit function of the next step and Merit function
    unsigned maxLineSearchIteration_ ;
    qpOASES::Constraints constraints_;
    qpOASES::Indexlist * indexActiveConstraints_ ;
    bool oneMoreStep_ ;
    unsigned maxSolverIteration_ ;

    // Gauss-Newton Hessian
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
    //MAL_MATRIX_TYPE(double) qp_J_cop_, qp_J_foot_, qp_J_vel_, qp_J_obs_, qp_J_rot_ , qp_J_stan_;
    //MAL_VECTOR_TYPE(double) qp_lbJ_cop_, qp_lbJ_foot_, qp_lbJ_vel_, qp_lbJ_obs_, qp_lbJ_rot_, qp_lbJ_stan_ ;
    //MAL_VECTOR_TYPE(double) qp_ubJ_cop_, qp_ubJ_foot_, qp_ubJ_vel_, qp_ubJ_obs_, qp_ubJ_rot_, qp_ubJ_stan_ ;

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
    // Sampling period of the first SQP preview iteration
    double Tfirst_ ;
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
    double alpha_x_ ;
    double alpha_y_ ;
    double alpha_theta_ ;
    double beta_  ;
    double gamma_ ;
    double delta_ ;
    double kappa_ ;

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
    support_state_t dummySupp_ ;
    convex_hull_t hull4_, hull5_ ;
    RelativeFeetInequalities * RFI_;
    double FeetDistance_;
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
    bool isQPlandinginitialized_ ;
    qpOASES::SQProblem * QP_ ;
    qpOASES::SQProblem * QP_landing_ ;
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
    MAL_VECTOR_TYPE(double) deltaU_thresh_ ;
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

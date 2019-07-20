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
#include <eigen-quadprog/QuadProg.h>

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
    void updateInitialConditionDependentMatrices();
    void guessWarmStart();

    // build the constraints :
    void initializeConstraint();
    void updateConstraint();
    void evalConstraint(Eigen::VectorXd & U);

    void initializeCoPConstraint();
    void evalCoPconstraint(Eigen::VectorXd & U);
    void updateCoPconstraint(Eigen::VectorXd & U);
    void initializeFootPoseConstraint();
    void evalFootPoseConstraint(Eigen::VectorXd & U);
    void updateFootPoseConstraint(Eigen::VectorXd & U);
    void initializeFootVelIneqConstraint();
    void updateFootVelIneqConstraint();
    void initializeRotIneqConstraint();
    void updateRotIneqConstraint();
    void initializeObstacleConstraint();
    void updateObstacleConstraint();
    void initializeStandingConstraint();
    void updateStandingConstraint();    

    // tools to check if foot is close to land
    void updateIterationBeforeLanding();
    bool isFootCloseToLand()
    {
//      std::cout << itBeforeLanding_ << std::endl;
      return ((itBeforeLanding_ < 2) && useItBeforeLanding_);
//      return false ;
    }

    // build the cost function
    void initializeCostFunction();
    void updateCostFunction();

    // tools for line search
    void initializeLineSearch();
    void lineSearch();
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

    std::deque <RelativeFootPosition> & relativeSupportDeque()
    {return desiredNextSupportFootRelativePosition;}


  private:
    SimplePluginManager * SPM_ ;
    PinocchioRobot * PR_ ;

    // Time variant parameter
    /////////////////////////

    double time_;

    // [x, dx, ddx], com (pos, vel, acc) at instant t_k on axis X
    Eigen::VectorXd c_k_x_;
    // [y, dy, ddy], com (pos, vel, acc) at instant t_k on axis Y
    Eigen::VectorXd c_k_y_;

    /**
    NOTE number of foot steps in prediction horizon changes between
    nf and nf+1, because when robot takes first step nf steps are
    planned on the prediction horizon, which makes a total of nf+1 steps.
    */
    Eigen::VectorXd v_kp1_ ;
    Eigen::MatrixXd V_kp1_ ;

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
    std::size_t nc_cop_ ;

    Eigen::MatrixXd Acop_xy_, Acop_theta_ ;
    Eigen::VectorXd UBcop_ ;
    Eigen::MatrixXd D_kp1_xy_, D_kp1_theta_, Pzuv_, derv_Acop_map_  ;
    Eigen::MatrixXd derv_Acop_map2_ ;
    Eigen::VectorXd b_kp1_, Pzsc_, Pzsc_x_, Pzsc_y_, v_kp1f_, v_kp1f_x_, v_kp1f_y_ ;
    Eigen::VectorXd v_kf_x_, v_kf_y_ ;
    Eigen::MatrixXd diffMat_ ;
    Eigen::MatrixXd rotMat_xy_, rotMat_theta_, rotMat_;
    Eigen::MatrixXd A0_xy_, A0_theta_;
    Eigen::VectorXd B0_;
    Eigen::MatrixXd Acop_theta_dummy0_;
    Eigen::VectorXd Acop_theta_dummy1_;

    // Foot position constraint
    unsigned nc_foot_ ;
    unsigned n_vertices_ ;
    unsigned int itBeforeLanding_ ;
    bool useItBeforeLanding_ ;
    int itMax_;
    std::vector<Eigen::MatrixXd> Afoot_xy_, Afoot_theta_  ;
    std::vector<Eigen::VectorXd> UBfoot_ ;
    std::vector<Eigen::MatrixXd> SelecMat_;
    std::vector<Eigen::MatrixXd> A0f_xy_, A0f_theta_ ;
    std::vector<Eigen::VectorXd> B0f_;
    std::vector<Eigen::MatrixXd> rotMat_vec_, drotMat_vec_ ;
    Eigen::MatrixXd tmpRotMat_;
    std::vector<Eigen::VectorXd> deltaF_ ;
    std::vector<Eigen::VectorXd> AdRdF_ ;
    Eigen::MatrixXd Afoot_xy_full_, Afoot_theta_full_  ;
    Eigen::VectorXd UBfoot_full_ ;

    // Foot Velocity constraint
    unsigned nc_vel_ ;
    std::deque <RelativeFootPosition> desiredNextSupportFootRelativePosition ;
    std::vector<support_state_t> desiredNextSupportFootAbsolutePosition ;
    Eigen::MatrixXd Avel_ ;
    Eigen::VectorXd Bvel_ ;

    // Foot Position constraint
    unsigned nc_pos_ ;
    Eigen::MatrixXd Apos_ ;
    Eigen::VectorXd Bpos_ ;

    // Rotation linear constraint
    unsigned nc_rot_ ;
    Eigen::MatrixXd Arot_ ;
    Eigen::VectorXd UBrot_,LBrot_ ;

    // Obstacle constraint
    unsigned nc_obs_ ;
    std::vector< std::vector<Eigen::MatrixXd> > Hobs_ ;
    std::vector< std::vector<Eigen::VectorXd> > Aobs_ ;
    std::vector< Eigen::VectorXd > UBobs_ ;
    std::vector<Circle> obstacles_ ;
    Eigen::VectorXd qp_J_obs_i_ ;
    // Standing constraint :
    unsigned nc_stan_ ;
    Eigen::MatrixXd Astan_ ;
    Eigen::VectorXd UBstan_, LBstan_ ;

    // evaluate constraint
    // real problem bounds : lb_ < g(U) < ub_
    Eigen::VectorXd ub_  ;
    Eigen::VectorXd gU_  ;
    Eigen::VectorXd Uxy_  ;
    Eigen::VectorXd gU_cop_, gU_vel_ ;
    Eigen::VectorXd gU_foot_ ;
    Eigen::VectorXd gU_obs_, gU_rot_ , gU_stan_ ;

    // Cost Function
    unsigned nv_ ; // number of degrees of freedom
    // initial problem matrix
    Eigen::MatrixXd Q_theta_, I_NN_, I_FF_ ;

    // decomposition of p_xy_
    // p_xy_ = ( p_xy_X_, p_xy_Fx_, p_xy_Y_, p_xy_Fy_ )
    Eigen::VectorXd p_xy_X_, p_xy_Fx_, p_xy_Y_, p_xy_Fy_ ;
    Eigen::VectorXd Pvsc_x_ , Pvsc_y_ ;

    // decomposition of Q_x_=Q_y_
    // Q_x = ( Q_x_XX Q_x_XF ) = Q_y
    //       ( Q_x_FX Q_x_FF )
    Eigen::MatrixXd Q_x_XX_, Q_x_XF_, Q_x_FX_, Q_x_FF_ ;
    Eigen::MatrixXd Q_y_XX_;// Q_x_XX_ != Q_y_XX_

    // Line Search
    bool useLineSearch_ ;
    Eigen::VectorXd p_ , U_n_, selectActiveConstraint ;
    Eigen::VectorXd JdU_, contraintValue ;
    Eigen::VectorXd HUn_ ;
    double lineStep_, lineStep0_, stepParam_ ; // step searched
    double mu_ ; // weight between cost function and constraints
    double cm_, c_ ; // Merit Function Jacobian
    double L_n_, L_ ; // Merit function of the next step and Merit function
    unsigned maxLineSearchIteration_ ;
    bool oneMoreStep_ ;
    unsigned maxSolverIteration_ ;

    // Gauss-Newton Hessian
    unsigned nceq_ ;
    unsigned ncineq_ ;
    unsigned nc_ ;
    Eigen::MatrixXd qp_H_   ;
    Eigen::VectorXd qp_g_   ;
    Eigen::MatrixXd qp_J_   ; //constraint Jacobian
    Eigen::VectorXd qp_ubJ_ ; //constraint Jacobian
    // temporary usefull variable for matrix manipulation
    Eigen::VectorXd qp_g_x_, qp_g_y_, qp_g_theta_ ;

    // Free variable of the system
    // U_       = [C_kp1_x_ F_kp1_x_ C_kp1_y_ F_kp1_y_ F_kp1_theta_]^T
    // U_xy     = [C_kp1_x_ F_kp1_x_ C_kp1_y_ F_kp1_y_]^T
    // U_x      = [C_kp1_x_ F_kp1_x_]^T
    // U_y      = [C_kp1_y_ F_kp1_y_]^T
    // U_theta_ = [F_kp1_theta_]^T
    Eigen::VectorXd U_           ;
    Eigen::VectorXd U_xy_        ;
    Eigen::VectorXd U_x_         ;
    Eigen::VectorXd U_y_         ;
    Eigen::VectorXd F_kp1_x_     ;
    Eigen::VectorXd F_kp1_y_     ;
    Eigen::VectorXd F_kp1_theta_ ;

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
    double minjerk_ ;
    double delta_ ;
    double kappa_ ;

    // Integration Matrices

    // C_k+1_%    = position     of the CoM on the whole preview
    // dC_k+1_%   = velocity     of the CoM on the whole preview
    // ddC_k+1_%  = acceleration of the CoM on the whole preview
    // dddC_k+1_% = jerk         of the CoM on the whole preview (free variable)
    // Z_k+1_%    = position     of the ZMP on the whole preview (free variable)

    // C_k+1_% = Pps * c_k_% + Ppu dddC_k+1_%  (%={x,y})
    Eigen::MatrixXd Pps_ ;
    Eigen::MatrixXd Ppu_ ;
    // dC_k+1_% = Pvs * c_k_% + Pvu dddC_k+1_%
    Eigen::MatrixXd Pvs_ ;
    Eigen::MatrixXd Pvu_ ;
    // ddC_k+1_% = Pas * c_k_% + Pau dddC_k+1_%
    Eigen::MatrixXd Pas_ ;
    Eigen::MatrixXd Pau_ ;
    // Z_k+1_% = Pzs * c_k_% + Pzu dddC_k+1_%
    Eigen::MatrixXd Pzs_ ;
    Eigen::MatrixXd Pzu_ ;

    // Convex Hulls for ZMP and FootStep constraints :
    support_state_t dummySupp_ ;
    convex_hull_t hull4_, hull5_ ;
    RelativeFeetInequalities * RFI_;
    double FeetDistance_;
    // linear system corresponding to the foot step constraints
    // right foot polygonal hull
    Eigen::MatrixXd A0r_  ;
    Eigen::VectorXd ubB0r_;
    // left foot polygonal hull
    Eigen::MatrixXd A0l_  ;
    Eigen::VectorXd ubB0l_;

    // linear systems corresponding to the CoP constraints
    // right foot hull minus security margin
    Eigen::MatrixXd A0rf_    ;
    Eigen::VectorXd ubB0rf_  ;
    // left foot hull minus security margin
    Eigen::MatrixXd A0lf_    ;
    Eigen::VectorXd ubB0lf_  ;
    // foot hull minus security margin for standing still
    // or dealing with the switching mass phase
    Eigen::MatrixXd A0ds_   ;
    Eigen::VectorXd ubB0ds_ ;

    // [Vx, Vy, Vtheta], reference velocity express in local frame
    // [Vx, Vy, Vtheta], reference velocity express in global frame
    // contain the vectors for the reference velocity over the
    // entire horizon
    reference_t vel_ref_ ;

    // QPoases data structure
    bool isQPinitialized_ ;
    bool isQPlandinginitialized_ ;
    Eigen::QuadProgDense * QP_ ;
    Eigen::MatrixXd QuadProg_H_, QuadProg_J_eq_, QuadProg_J_ineq_;
    Eigen::VectorXd QuadProg_g_, QuadProg_bJ_eq_, QuadProg_lbJ_ineq_, deltaU_;
    Eigen::VectorXd deltaU_thresh_ ;

    /// Exit on error.
    bool exit_on_error_;
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

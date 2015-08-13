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

#include <ZMPRefTrajectoryGeneration/nmpc_generator.hh>
#include <cmath>
#include <Debug.hh>

#define DEBUG

using namespace std;
using namespace PatternGeneratorJRL;

NMPCgenerator::NMPCgenerator(SimplePluginManager * aSPM, CjrlHumanoidDynamicRobot * aHDR)
{
  T_ = 0.0 ;
  N_ = 0 ;
  nf_ = 0 ;
  T_step_ = 0.0 ;
  c_k_z_ = 0.0 ;
  vel_ref_.Local.X    = 0.0 ;
  vel_ref_.Local.Y    = 0.0 ;
  vel_ref_.Local.Yaw  = 0.0 ;
  vel_ref_.Global.X   = 0.0 ;
  vel_ref_.Global.Y   = 0.0 ;
  vel_ref_.Global.Yaw = 0.0 ;
  SecurityMarginX_ = 0.0 ;
  SecurityMarginY_ = 0.0 ;

  alpha_=0.0;
  beta_ =0.0;
  gamma_=0.0;

  SPM_ = aSPM ;
  HDR_ = aHDR ;
  RFI_=NULL;
  QP_ = NULL ;
  qpOases_H_ =NULL;
  qpOases_J_ =NULL;
  qpOases_lbJ=NULL;
  qpOases_ubJ=NULL;
  qpOases_lb_=NULL;
  qpOases_ub_=NULL;
  nwsr_ = 0;
  cput_ = NULL;
  isQPinitialized_ = false;

  SupportStates_deq_.clear();
}

NMPCgenerator::~NMPCgenerator()
{
  if (cput_ !=NULL)
  {
    delete cput_;
    cput_ = NULL ;
  }
  if (QP_ !=NULL)
  {
    delete QP_;
    QP_ = NULL ;
  }
  if (RFI_ !=NULL)
  {
    delete RFI_;
    RFI_ = NULL ;
  }
}

void NMPCgenerator::initNMPCgenerator(double time,
    support_state_t & currentSupport,
    FootAbsolutePosition &InitLeftFootAbsolutePosition,
    FootAbsolutePosition &InitRightFootAbsolutePosition,
    COMState &lStartingCOMState,
    reference_t &local_vel_ref)
{
  N_ = 16 ;
  nf_ = 2 ;
  // number of degrees of freedom
  nv_ = 2*N_+3*nf_;

  MAL_MATRIX_RESIZE(Pps_,N_,3);                  MAL_MATRIX_FILL(Pps_,0.0);
  MAL_MATRIX_RESIZE(Ppu_,N_,N_);                 MAL_MATRIX_FILL(Ppu_,0.0);
  MAL_MATRIX_RESIZE(Pvs_,N_,3);                  MAL_MATRIX_FILL(Pvs_,0.0);
  MAL_MATRIX_RESIZE(Pvu_,N_,N_);                 MAL_MATRIX_FILL(Pvu_,0.0);
  MAL_MATRIX_RESIZE(Pas_,N_,3);                  MAL_MATRIX_FILL(Pas_,0.0);
  MAL_MATRIX_RESIZE(Pau_,N_,N_);                 MAL_MATRIX_FILL(Pau_,0.0);
  MAL_MATRIX_RESIZE(Pzs_,N_,3);                  MAL_MATRIX_FILL(Pzs_,0.0);
  MAL_MATRIX_RESIZE(Pzu_,N_,N_);                 MAL_MATRIX_FILL(Pzu_,0.0);
  MAL_VECTOR_RESIZE(v_kp1_,N_) ;                 MAL_VECTOR_FILL(v_kp1_,0.0) ;
  MAL_MATRIX_RESIZE(V_kp1_,N_,nf_) ;             MAL_MATRIX_FILL(V_kp1_,0.0) ;
  MAL_VECTOR_RESIZE(U_           , 2*N_+3*nf_);  MAL_VECTOR_FILL(U_          ,0.0);
  MAL_VECTOR_RESIZE(U_xy_        , 2*(N_+nf_));  MAL_VECTOR_FILL(U_xy_       ,0.0);
  MAL_VECTOR_RESIZE(U_x_         , N_+nf_);      MAL_VECTOR_FILL(U_x_        ,0.0);
  MAL_VECTOR_RESIZE(U_y_         , N_+nf_);      MAL_VECTOR_FILL(U_y_        ,0.0);
  MAL_VECTOR_RESIZE(F_kp1_x_     , nf_);         MAL_VECTOR_FILL(F_kp1_x_    ,0.0);
  MAL_VECTOR_RESIZE(F_kp1_y_     , nf_);         MAL_VECTOR_FILL(F_kp1_y_    ,0.0);
  MAL_VECTOR_RESIZE(F_kp1_theta_ , nf_);         MAL_VECTOR_FILL(F_kp1_theta_,0.0);
  MAL_VECTOR_RESIZE(c_k_x_,3);                   MAL_VECTOR_FILL(c_k_x_ ,0.0);
  MAL_VECTOR_RESIZE(c_k_y_,3);                   MAL_VECTOR_FILL(c_k_y_ ,0.0);
  MAL_MATRIX_RESIZE(A0r_   ,5,2) ;               MAL_MATRIX_FILL(A0r_   ,0.0);
  MAL_VECTOR_RESIZE(ubB0r_ ,5) ;                 MAL_VECTOR_FILL(ubB0r_ ,0.0);
  MAL_MATRIX_RESIZE(A0l_   ,5,2) ;               MAL_MATRIX_FILL(A0l_   ,0.0);
  MAL_VECTOR_RESIZE(ubB0l_ ,5) ;                 MAL_VECTOR_FILL(ubB0l_ ,0.0);
  MAL_MATRIX_RESIZE(A0rf_  ,4,2) ;               MAL_MATRIX_FILL(A0rf_  ,0.0);
  MAL_VECTOR_RESIZE(ubB0rf_,4) ;                 MAL_VECTOR_FILL(ubB0rf_,0.0);
  MAL_MATRIX_RESIZE(A0lf_  ,4,2) ;               MAL_MATRIX_FILL(A0lf_  ,0.0);
  MAL_VECTOR_RESIZE(ubB0lf_,4) ;                 MAL_VECTOR_FILL(ubB0lf_,0.0);
  MAL_MATRIX_RESIZE(A0ds_  ,4,2) ;               MAL_MATRIX_FILL(A0ds_  ,0.0);
  MAL_VECTOR_RESIZE(ubB0ds_,4) ;                 MAL_VECTOR_FILL(ubB0ds_,0.0);

  T_ = 0.1 ;
  T_step_ = 0.8 ;
  alpha_ = 1.0   ; // weight for CoM velocity tracking  : 0.5 * a
  beta_  = 1e+03 ; // weight for ZMP reference tracking : 0.5 * b
  gamma_ = 5e-04 ; // weight for jerk minimization      : 0.5 * c
  SecurityMarginX_ = 0.09 ;
  SecurityMarginY_ = 0.05 ;

  setLocalVelocityReference(local_vel_ref);
  c_k_x_(0) = lStartingCOMState.x[0] ;
  c_k_x_(1) = lStartingCOMState.x[1] ;
  c_k_x_(2) = lStartingCOMState.x[2] ;
  c_k_y_(0) = lStartingCOMState.y[0] ;
  c_k_y_(1) = lStartingCOMState.y[1] ;
  c_k_y_(2) = lStartingCOMState.y[2] ;
  c_k_z_ = lStartingCOMState.z[0] ;

  // start with Left foot as support
  // the right foot start moving
  currentSupport_ = currentSupport ;
  SupportStates_deq_.resize(N_,currentSupport_);

  RFI_ = new RelativeFeetInequalities(SPM_,HDR_) ;
  ostringstream oss(std::ostringstream::ate);
  oss << ":setfeetconstraint XY " << SecurityMarginX_ << " " << SecurityMarginY_ ;
  istringstream strm(oss.str());
  string cmd ;
  strm >> cmd ;
  RFI_->CallMethod(cmd,strm);

  // build constant matrices
  buildCoMIntegrationMatrix();
  buildCoPIntegrationMatrix();
  buildConvexHullSystems();

  // initialize time dependant matrices
  initializeCoPConstraint();
  initializeFootPoseConstraint();
  initializeFootVelIneqConstraint();
  initializeRotIneqConstraint();
  initializeObstacleConstraint();
  initializeCostFunction();

  // initialize the solver
  QP_ = new qpOASES::SQProblem(nv_,nc_,qpOASES::HST_SEMIDEF) ;
  //options_.printLevel = qpOASES::PL_NONE ;
  options_.setToMPC();
  QP_->setOptions(options_);
  nwsr_ = 1e+8 ;
  cput_ = new double[1] ;
  deltaU_ = new double[nv_];
  cput_[0] = 1e+8;

  FSM_ = new SupportFSM();
  FSM_->StepPeriod( T_step_ );
  FSM_->DSPeriod( 1e9 ); // period during the robot move at 0.0 com speed
  FSM_->DSSSPeriod( T_step_ );
  FSM_->NbStepsSSDS( 2 ); // number of previw step
  FSM_->SamplingPeriod( T_ );

  updateFinalStateMachine(time,
                          InitLeftFootAbsolutePosition,
                          InitRightFootAbsolutePosition);
  computeFootSelectionMatrix();
}

void NMPCgenerator::solve()
{
  /* Process and solve problem, s.t. pattern generator data is consistent */
  preprocess_solution() ;
  solve_qp()            ;
  postprocess_solution();
}

void NMPCgenerator::preprocess_solution()
{
  //computeInitialGuess();
  updateCoPConstraint();
  updateFootPoseConstraint();
  updateFootVelIneqConstraint();
  updateRotIneqConstraint();
  updateObstacleConstraint();
  updateCostFunction();
  qpOases_H_ = MRAWDATA(qp_H_  ) ;
  qpOases_g_ = MRAWDATA(qp_g_  ) ;
  qpOases_J_ = MRAWDATA(qp_J_  ) ;
  qpOases_lbJ= MRAWDATA(qp_lbJ_) ;
  qpOases_ubJ= MRAWDATA(qp_ubJ_) ;
  qpOases_lb_= MRAWDATA(qp_lb_ ) ;
  qpOases_ub_= MRAWDATA(qp_ub_ ) ;
  return ;
}

void NMPCgenerator::solve_qp(){
  /**
  """
  Solve QP first run with init functionality and other runs with warmstart
  """
  */
  cput_[0] = 1e+8; // force the solver to use the maximum time for computing the solution
  nwsr_ = 100 ;
  //qpOASES::returnValue ret, error ;
  if (!isQPinitialized_)
  {
    /*ret =*/ QP_->init(
            qpOases_H_, qpOases_g_, qpOases_J_,
            qpOases_lb_, qpOases_ub_,
            qpOases_lbJ, qpOases_ubJ,
            nwsr_, cput_
          );
    isQPinitialized_ = true ;
  }
  else
  {
    /*ret = */QP_->hotstart(
      qpOases_H_, qpOases_g_, qpOases_J_,
      qpOases_lb_, qpOases_ub_,
      qpOases_lbJ, qpOases_ubJ,
      nwsr_, cput_
    );
  }

  // primal SQP solution
  /*error = */QP_->getPrimalSolution(deltaU_) ;

  // save qp solver data
  cput_[0] = cput_[0]*1000. ;// in milliseconds TODO verify the behaviour of this

  return ;
}

void NMPCgenerator::postprocess_solution()
{
  /**
  """ Get solution and put it back into generator data structures """
  # extract dofs
  # dofs = ( dddC_k_x ) N
  #        (    F_k_x ) nf
  #        ( dddC_k_y ) N
  #        (    F_k_y ) nf
  #        (    F_k_q ) nf
  */

  // NOTE this time we add an increment to the existing values
  // data(k+1) = data(k) + alpha * dofs

  // TODO add line search when problematic
  double alpha = 1.0 ;

  for(unsigned i=0 ; i<nv_ ; ++i)
    U_(i) += alpha * deltaU_[i];

  for(unsigned i=0 ; i<2*N_+2*nf_ ; ++i)
    U_xy_(i)=U_(i);

  for(unsigned i=0 ; i<N_+nf_ ; ++i)
  {
    U_x_(i) = U_(i);
    U_y_(i) = U_(N_+nf_+i);
  }
  for(unsigned i=0 ; i<nf_ ; ++i)
  {
    F_kp1_x_(i)     = U_(N_+i);
    F_kp1_y_(i)     = U_(2*N_+nf_+i);
    F_kp1_theta_(i) = U_(2*N_+2*nf_+i);
  }

  // integrate the jerks to get the current CoM state
  c_k_x_[0] = c_k_x_[0] + T_*c_k_x_[1] + 0.5*T_*T_*c_k_x_[2] + T_*T_*T_/6*U_x_(0) ;
  c_k_x_[1] = c_k_x_[1] + T_*c_k_x_[2] + 0.5*T_*T_*U_x_(0) ;
  c_k_x_[2] = c_k_x_[2] + T_*U_x_(0) ;

  c_k_y_[0] = c_k_y_[0] + T_*c_k_y_[1] + 0.5*T_*T_*c_k_y_[2] + T_*T_*T_/6*U_y_(0) ;
  c_k_y_[1] = c_k_y_[1] + T_*c_k_y_[2] + 0.5*T_*T_*U_y_(0) ;
  c_k_y_[2] = c_k_y_[2] + T_*U_y_(0) ;

  return ;
}

void NMPCgenerator::getSolution(std::vector<double> JerkX,
                                std::vector<double> JerkY,
                                std::vector<double> FootStepX,
                                std::vector<double> FootStepY,
                                std::vector<double> FootStepYaw)
{
  JerkX.resize(N_);
  JerkY.resize(N_);
  for(unsigned i=0 ; i<N_ ; ++i)
  {
    JerkX[i] = U_x_(i);
    JerkY[i] = U_y_(i);
  }
  unsigned nf = 0 ;
  if(currentSupport_.StateChanged)
    nf=nf_-1 ;
  else
    nf=nf_;
  FootStepX  .resize(nf+1);
  FootStepY  .resize(nf+1);
  FootStepYaw.resize(nf+1);
  for(unsigned i=0 ; i<nf ; ++i)
  {
    FootStepX  [i] = F_kp1_x_(i)    ;
    FootStepY  [i] = F_kp1_y_(i)    ;
    FootStepYaw[i] = F_kp1_theta_(i);
  }
  double c_kpN_x(0.0), dc_kpN_x(0.0), c_kpN_y (0.0), dc_kpN_y(0.0) ;
  c_kpN_x  += Pps_(N_-1,0)*c_k_x_(1) + Pps_(N_-1,0)*c_k_x_(1) + Pps_(N_-1,0)*c_k_x_(1) ;
  dc_kpN_x += Pvsc_x_(N_-1) ;
  c_kpN_y  += Pps_(N_-1,0)*c_k_y_(1) + Pps_(N_-1,0)*c_k_y_(1) + Pps_(N_-1,0)*c_k_y_(1) ;
  dc_kpN_y += Pvsc_y_(N_-1) ;
  for(unsigned i=0 ; i<N_ ; ++i)
  {
    c_kpN_x  += Pps_(N_-1,i)*U_x_(i) ;
    dc_kpN_x += Pvs_(N_-1,i)*U_x_(i) ;
    c_kpN_y  += Pps_(N_-1,i)*U_y_(i) ;
    dc_kpN_y += Pvs_(N_-1,i)*U_y_(i) ;
  }
  const double GRAVITY = 9.81 ;
  // step on the capture point at the end of the preview
  FootStepX  [nf] = c_kpN_x + sqrt(c_k_z_/GRAVITY) * dc_kpN_x ;
  FootStepY  [nf] = c_kpN_y + sqrt(c_k_z_/GRAVITY) * dc_kpN_y ;
  FootStepYaw[nf] = FootStepYaw[nf-1] + vel_ref_.Global.Yaw*T_ ;
}

void NMPCgenerator::updateFinalStateMachine(
    double time,
    FootAbsolutePosition & FinalLeftFootTraj,
    FootAbsolutePosition & FinalRightFootTraj)
{
  FSM_->update_vel_reference(vel_ref_,currentSupport_);
  const FootAbsolutePosition * FAP = NULL;

  // DETERMINE CURRENT SUPPORT STATE:
  // --------------------------------
  FSM_->set_support_state( time, 0, currentSupport_, vel_ref_ );
  if( currentSupport_.StateChanged == true )
  {
    if( currentSupport_.Foot == LEFT )
      FAP = & FinalLeftFootTraj;
    else
      FAP = & FinalRightFootTraj;
    currentSupport_.X = FAP->x;
    currentSupport_.Y = FAP->y;
    currentSupport_.Yaw = FAP->theta*M_PI/180.0;
    currentSupport_.StartTime = time;
  }
  SupportStates_deq_[0] = currentSupport_ ;

  // PREVIEW SUPPORT STATES:
  // -----------------------
  // initialize the previewed support state before previewing
  support_state_t PreviewedSupport = currentSupport_;
  PreviewedSupport.StepNumber  = 0;
  for( unsigned pi=1 ; pi<=N_ ; pi++ )
  {
    FSM_->set_support_state( time, pi, PreviewedSupport, vel_ref_ );
    if( PreviewedSupport.StateChanged )
    {
      if( pi == 1  )//Foot down
      {
        if( PreviewedSupport.Foot == LEFT )
          FAP = & FinalLeftFootTraj;
        else
          FAP = & FinalRightFootTraj;
        PreviewedSupport.X = FAP->x;
        PreviewedSupport.Y = FAP->y;
        PreviewedSupport.Yaw = FAP->theta*M_PI/180.0;
        PreviewedSupport.StartTime = time+pi*T_;
      }
      if( /*pi > 1 &&*/ PreviewedSupport.StepNumber > 0 )
      {
        PreviewedSupport.X = 0.0;
        PreviewedSupport.Y = 0.0;
      }
    }
    SupportStates_deq_[pi] = PreviewedSupport ;
  }
}

void NMPCgenerator::computeFootSelectionMatrix()
{
  std::deque<support_state_t>::const_iterator SS_it;
  SS_it = SupportStates_deq_.begin();//points at the cur. sup. st.
  ++SS_it;
  MAL_VECTOR_FILL(v_kp1_,0.0);
  MAL_MATRIX_FILL(V_kp1_,0.0);
  for(unsigned i=0;i<N_;++i, ++SS_it)
  {
    if(SS_it->StepNumber==0)
      v_kp1_(i)=1.0;
    if(SS_it->StepNumber==1)
      V_kp1_(i,0)=1.0;
    if(SS_it->StepNumber==2)
      V_kp1_(i,1)=1.0;
  }
#ifdef DEBUG
  cout << "v_kp1_ = " << v_kp1_ << endl ;
  cout << "V_kp1_ = " << V_kp1_ << endl ;
#endif
  return ;
}

void NMPCgenerator::buildCoMIntegrationMatrix()
{
  for (unsigned i = 0 , k = 1 ; i < N_ ; ++i , k=i+1)
  {
    Pps_(i,0) = 1.0 ; Pps_(i,1) = k*T_ ; Pps_(i,2) = k*k*T_*T_*0.5 ;
    Pvs_(i,0) = 0.0 ; Pvs_(i,1) = 1.0  ; Pvs_(i,2) = k*T_          ;
    Pas_(i,0) = 0.0 ; Pas_(i,1) = 0.0  ; Pas_(i,2) = 1.0           ;

    for (unsigned j = 0 ; j <= i ; ++j)
    {
      double id = (double)i ;
      double jd = (double)j ;
      Ppu_(i,j) = 3.0*(id-jd)*(id-jd)*T_*T_*T_*1/6.0 + 3.0*(id-jd)*T_*T_*T_*1/6.0 + T_*T_*T_*1/6.0 ;
      Pvu_(i,j) = 2.0*(id-jd)*T_*T_*0.5 + T_*T_*0.5 ;
      Pau_(i,j) = T_ ;
    }
  }
#ifdef DEBUG
  DumpMatrix("Pps_",Pps_);
  DumpMatrix("Pvs_",Pvs_);
  DumpMatrix("Pas_",Pas_);
  DumpMatrix("Ppu_",Ppu_);
  DumpMatrix("Pvu_",Pvu_);
  DumpMatrix("Pau_",Pau_);
#endif
  return ;
}

void NMPCgenerator::buildCoPIntegrationMatrix()
{
  const double GRAVITY = 9.81;
  MAL_MATRIX_FILL(Pzu_,0.0);
  MAL_MATRIX_FILL(Pzs_,0.0);
  for (unsigned i = 0 , k = 1 ; i < N_ ; ++i , k=i+1)
  {
    Pzs_(i,0) = 1.0 ; Pzs_(i,1) = k*T_ ; Pzs_(i,2) = k*k*T_*T_*0.5 - c_k_z_/GRAVITY ;

    for (unsigned j = 0 ; j <= i ; ++j)
    {
      Pzu_(i,j) = (3.0*(double)(i-j)*(double)(i-j) + 3.0*(double)(i-j)+1.0)*T_*T_*T_/6.0 - T_*c_k_z_/GRAVITY ;
    }
  }
#ifdef DEBUG
  DumpMatrix("Pzs_",Pzs_);
  DumpMatrix("Pzu_",Pzu_);
#endif
  return ;
}

void NMPCgenerator::buildConvexHullSystems()
{
  support_state_t dummySupp ;
  unsigned nbEdges = 0;
  unsigned nbIneq = 0;
  convex_hull_t hull ;

  // CoP hulls
  ////////////////////////////////////////////////////////
  nbEdges = 4 ;
  nbIneq = 4 ;
  hull = convex_hull_t(nbEdges, nbIneq);
  // RIGHT FOOT
  dummySupp.Foot = RIGHT ;
  dummySupp.Phase = SS ;
  RFI_->set_vertices( hull, dummySupp, INEQ_COP );
  RFI_->compute_linear_system( hull, dummySupp );
  for(unsigned i = 0 ; i < hull.A_vec.size() ; ++i)
  {
    A0rf_(i,0) = hull.A_vec[i] ;
    A0rf_(i,1) = hull.B_vec[i] ;
    ubB0rf_(i) = hull.D_vec[i] ;
  }
  // LEFT FOOT
  dummySupp.Foot = LEFT ;
  dummySupp.Phase = SS ;
  RFI_->set_vertices( hull, dummySupp, INEQ_COP );
  RFI_->compute_linear_system( hull, dummySupp );
  for(unsigned i = 0 ; i < hull.A_vec.size() ; ++i)
  {
    A0lf_(i,0) = hull.A_vec[i] ;
    A0lf_(i,1) = hull.B_vec[i] ;
    ubB0lf_(i) = hull.D_vec[i] ;
  }
  // "SWITCHING MASS"/"DOUBLE SUPPORT" HULL
  dummySupp.Foot = LEFT ; // foot by default, it shouldn't cause any trouble
  dummySupp.Phase = SS ;
  RFI_->set_vertices( hull, dummySupp, INEQ_COP );
  RFI_->compute_linear_system( hull, dummySupp );
  for(unsigned i = 0 ; i < hull.A_vec.size() ; ++i)
  {
    A0ds_(i,0) = hull.A_vec[i] ;
    A0ds_(i,1) = hull.B_vec[i] ;
    ubB0ds_(i) = hull.D_vec[i] ;
  }
#ifdef DEBUG
  DumpMatrix("A0lf_",A0lf_);
  DumpMatrix("A0rf_",A0rf_);
  DumpMatrix("A0ds_",A0ds_);
  DumpVector("ubB0rf_",ubB0rf_);
  DumpVector("ubB0lf_",ubB0lf_);
  DumpVector("ubB0ds_",ubB0ds_);
#endif
  // Polygonal hulls for feasible foot placement
  ///////////////////////////////////////////////////////
  nbEdges = 5 ;
  nbIneq = 5 ;
  hull = convex_hull_t(nbEdges, nbIneq);
  // RIGHT FOOT
  dummySupp.Foot = RIGHT ;
  dummySupp.Phase = SS ;
  hull.X_vec[0] = -0.28 ; hull.Y_vec[0] = -0.2 ;
  hull.X_vec[1] = -0.2  ; hull.Y_vec[1] = -0.3 ;
  hull.X_vec[2] =  0.0  ; hull.Y_vec[2] = -0.4 ;
  hull.X_vec[3] =  0.2  ; hull.Y_vec[3] = -0.3 ;
  hull.X_vec[4] =  0.28 ; hull.Y_vec[4] = -0.2 ;
  RFI_->compute_linear_system( hull, dummySupp );
  for(unsigned i = 0 ; i < hull.A_vec.size() ; ++i)
  {
    A0r_(i,0) = hull.A_vec[i] ;
    A0r_(i,1) = hull.B_vec[i] ;
    ubB0r_(i) = hull.D_vec[i] ;
  }
  // LEFT FOOT
  dummySupp.Foot = LEFT ;
  dummySupp.Phase = SS ;
  hull.X_vec[0] = -0.28 ; hull.Y_vec[0] = 0.2 ;
  hull.X_vec[1] = -0.2  ; hull.Y_vec[1] = 0.3 ;
  hull.X_vec[2] =  0.0  ; hull.Y_vec[2] = 0.4 ;
  hull.X_vec[3] =  0.2  ; hull.Y_vec[3] = 0.3 ;
  hull.X_vec[4] =  0.28 ; hull.Y_vec[4] = 0.2 ;
  RFI_->compute_linear_system( hull, dummySupp );
  for(unsigned i = 0 ; i < hull.A_vec.size() ; ++i)
  {
    A0l_(i,0) = hull.A_vec[i] ;
    A0l_(i,1) = hull.B_vec[i] ;
    ubB0l_(i) = hull.D_vec[i] ;
  }
#ifdef DEBUG
  DumpMatrix("A0r_",A0r_);
  DumpMatrix("A0l_",A0l_);
  DumpVector("ubB0r_",ubB0r_);
  DumpVector("ubB0l_",ubB0l_);
#endif

  return ;
}

void NMPCgenerator::initializeCoPConstraint()
{
  /**
  build linear inequality constraints to keep the CoP
  inside the support polygone

  NOTE: needs actual SupportStates_deq_ to work properly
  """
  # inequality constraint on both feet A u + B <= 0
  # A0 R(theta) [Zx_k - Fx_k] <= UB
  #             [Zy_k - Fy_k]
  */
  nc_cop_ = N_*MAL_VECTOR_SIZE(ubB0rf_) ;
  MAL_MATRIX_RESIZE(Acop_xy_   ,nc_cop_,2*(N_+nf_));
  MAL_MATRIX_RESIZE(Acop_theta_,nc_cop_,nf_);
  MAL_VECTOR_RESIZE(UBcop_     ,nc_cop_);
  MAL_VECTOR_RESIZE(LBcop_     ,nc_cop_);

  MAL_MATRIX_RESIZE(D_kp1_xy_,nc_cop_,2*N_);
  MAL_MATRIX_RESIZE(D_kp1_theta_,nc_cop_,2*N_);
  MAL_VECTOR_RESIZE(b_kp1_,nc_cop_);
  MAL_MATRIX_RESIZE(Pzuv_,2*N_,2*(N_+nf_));
  MAL_VECTOR_RESIZE(Pzsc_,2*N_);
  MAL_VECTOR_RESIZE(Pzsc_x_,N_);
  MAL_VECTOR_RESIZE(Pzsc_y_,N_);
  MAL_VECTOR_RESIZE(v_kp1f_,2*N_);
  MAL_VECTOR_RESIZE(v_kp1f_x_,N_);
  MAL_VECTOR_RESIZE(v_kp1f_y_,N_);

  MAL_MATRIX_RESIZE(derv_Acop_map_,nc_cop_,N_);

  MAL_MATRIX_FILL(Acop_xy_      ,0.0);
  MAL_MATRIX_FILL(Acop_theta_   ,0.0);
  MAL_VECTOR_FILL(UBcop_        ,0.0);
  MAL_VECTOR_FILL(LBcop_        ,-1e+08);
  MAL_MATRIX_FILL(D_kp1_xy_     ,0.0);
  MAL_MATRIX_FILL(D_kp1_theta_  ,0.0);
  MAL_VECTOR_FILL(b_kp1_        ,0.0);
  MAL_MATRIX_FILL(Pzuv_         ,0.0);
  MAL_VECTOR_FILL(Pzsc_         ,0.0);
  MAL_VECTOR_FILL(Pzsc_x_       ,0.0);
  MAL_VECTOR_FILL(Pzsc_y_       ,0.0);
  MAL_VECTOR_FILL(v_kp1f_       ,0.0);
  MAL_VECTOR_FILL(v_kp1f_x_     ,0.0);
  MAL_VECTOR_FILL(v_kp1f_y_     ,0.0);
  MAL_MATRIX_FILL(derv_Acop_map_,0.0);

  for(unsigned i=0 ; i<MAL_MATRIX_NB_ROWS(Pzu_) ; ++i)
  {
    for(unsigned j=0 ; j<MAL_MATRIX_NB_COLS(Pzu_) ; ++j)
    {
      Pzuv_(i,j)=Pzu_(i,j);
      Pzuv_(i+N_,j+N_+nf_)=Pzu_(i,j);
    }
  }

  for(unsigned j=0 , k=0; j<N_ ; ++j, k+=MAL_MATRIX_NB_ROWS(A0rf_))
    for(unsigned i=0 ; i<MAL_MATRIX_NB_ROWS(A0rf_) ; ++i )
      derv_Acop_map_(i+k,j) = 1.0 ;
#ifdef DEBUG
  //cout << derv_Acop_map_ << endl ;
  DumpMatrix("derv_Acop_map_",derv_Acop_map_);
  DumpMatrix("Pzu_",Pzu_);
#endif
  updateCoPConstraint();

  return ;
}

void NMPCgenerator::updateCoPConstraint()
{
  // Compute D_kp1_, it depends on the feet hulls
  vector<double>theta_vec(3);
  theta_vec[0]=currentSupport_.Yaw;
  theta_vec[1]=F_kp1_theta_(0);
  theta_vec[2]=F_kp1_theta_(1);
  // every time instant in the pattern generator constraints
  // depend on the support order
  for (unsigned i=0 ; i<N_ ; ++i)
  {
    double theta = theta_vec[SupportStates_deq_[i].StepNumber] ;
    MAL_MATRIX_DIM(rotMat_xy,double,2,2);
    MAL_MATRIX_DIM(rotMat_theta,double,2,2);
    rotMat_xy(0,0)= cos(theta) ; rotMat_xy(0,1)= sin(theta) ;
    rotMat_xy(1,0)=-sin(theta) ; rotMat_xy(1,1)= cos(theta) ;
    rotMat_theta(0,0)=-sin(theta) ; rotMat_theta(0,1)= cos(theta) ;
    rotMat_theta(1,0)=-cos(theta) ; rotMat_theta(1,1)=-sin(theta) ;

    MAL_MATRIX_TYPE(double) A0_xy, A0_theta;
    MAL_VECTOR_TYPE(double) B0;
    if (SupportStates_deq_[i].NbStepsLeft == 0)
    {
      A0_xy    = MAL_RET_A_by_B(A0ds_,rotMat_xy   ) ;
      A0_theta = MAL_RET_A_by_B(A0ds_,rotMat_theta) ;
      B0 = ubB0ds_ ;
    }
    else if (SupportStates_deq_[i].Foot == LEFT)
    {
      A0_xy    = MAL_RET_A_by_B(A0lf_,rotMat_xy   ) ;
      A0_theta = MAL_RET_A_by_B(A0lf_,rotMat_theta) ;
      B0 = ubB0lf_ ;
    }else{
      A0_xy    = MAL_RET_A_by_B(A0rf_,rotMat_xy   ) ;
      A0_theta = MAL_RET_A_by_B(A0rf_,rotMat_theta) ;
      B0 = ubB0rf_ ;
    }
    for (unsigned k=0 ; k<MAL_MATRIX_NB_ROWS(A0_xy) ; ++k)
    {
      // get d_i+1^x(f^theta)
      D_kp1_xy_(i*MAL_MATRIX_NB_ROWS(A0_xy)+k, i) = A0_xy(k,0);
      // get d_i+1^y(f^theta)
      D_kp1_xy_(i*MAL_MATRIX_NB_ROWS(A0_xy)+k, i+N_) = A0_xy(k,1);

      // get d_i+1^x(f^'dtheta/dt')
      D_kp1_theta_(i*MAL_MATRIX_NB_ROWS(A0_theta)+k, i) = A0_theta(k,0);
      // get d_i+1^y(f^'dtheta/dt')
      D_kp1_theta_(i*MAL_MATRIX_NB_ROWS(A0_theta)+k, i+N_) = A0_theta(k,1);

      // get right hand side of equation
      b_kp1_(i*MAL_MATRIX_NB_ROWS(A0_xy)+k) = B0(k) ;
    }
  }

  // Compute Pzuv, it depends on the feet hulls
  for(unsigned i=0 ; i<N_ ; ++i)
  {
    for(unsigned j=0 ; j<nf_ ; ++j)
    {
      Pzuv_(i,N_+j) = -V_kp1_(i,j) ;
      Pzuv_(i+N_,2*N_+nf_+j) = -V_kp1_(i,j) ;
    }
  }

  // build Pzsc_ and v_kp1f_
  Pzsc_x_ = MAL_RET_A_by_B(Pzs_,c_k_x_);
  Pzsc_y_ = MAL_RET_A_by_B(Pzs_,c_k_y_);
  v_kp1f_x_ = v_kp1_ * currentSupport_.X ;
  v_kp1f_y_ = v_kp1_ * currentSupport_.Y ;
  for(unsigned i=0 ; i<N_ ; ++i)
  {
    Pzsc_(i)      = Pzsc_x_(i) ;
    Pzsc_(i+N_)   = Pzsc_y_(i) ;
    v_kp1f_(i)    = v_kp1f_x_(i);
    v_kp1f_(i+N_) = v_kp1f_y_(i);
  }

  // build Acop_xy_
  Acop_xy_ = MAL_RET_A_by_B(D_kp1_xy_,Pzuv_);

  // build Acop_theta_ TODO VERIFY THE EQUATION !!!!!
  MAL_MATRIX_TYPE(double) dummy1 = MAL_RET_A_by_B(D_kp1_theta_,Pzuv_);
  MAL_VECTOR_TYPE(double) dummy = MAL_RET_A_by_B(dummy1,U_xy_);
  dummy = MAL_RET_A_by_B(dummy,derv_Acop_map_);
  dummy = MAL_RET_A_by_B(dummy,V_kp1_);
  for(unsigned i=0 ; i<MAL_MATRIX_NB_ROWS(Acop_theta_) ; ++i)
    for(unsigned j=0 ; j<MAL_MATRIX_NB_COLS(Acop_theta_) ; ++j)
      Acop_theta_(i,j) = dummy(j);

  UBcop_ = b_kp1_ + MAL_RET_A_by_B(D_kp1_xy_,v_kp1f_-Pzsc_) ;
#ifdef DEBUG
  DumpMatrix("Pzuv_",Pzuv_);
  DumpMatrix("D_kp1_xy_",D_kp1_xy_);
  DumpMatrix("D_kp1_theta_",D_kp1_theta_);
  DumpMatrix("Acop_xy_",Acop_xy_);
  DumpMatrix("Acop_theta_",Acop_theta_);
  DumpVector("UBcop_",UBcop_);
#endif
  return ;
}

void NMPCgenerator::initializeFootPoseConstraint()
{
  /**
  build linear inequality constraints for the placement of the feet

  NOTE: needs actual SupportStates_deq_ to work properly
  """
  # inequality constraint on both feet A u + B <= 0
  # A0 R(theta) [Fx_k+1 - Fx_k] <= ubB0
  #             [Fy_k+1 - Fy_k]
  */
  nc_foot_ = nf_*MAL_VECTOR_SIZE(ubB0r_) ;
  MAL_MATRIX_RESIZE(Afoot_xy_   ,nc_foot_,2*(N_+nf_));
  MAL_MATRIX_RESIZE(Afoot_theta_,nc_foot_,nf_);
  MAL_MATRIX_RESIZE(AS_theta_   ,nc_foot_,2*(N_+nf_));
  MAL_VECTOR_RESIZE(UBfoot_     ,nc_foot_);
  MAL_VECTOR_RESIZE(LBfoot_     ,nc_foot_);

  MAL_MATRIX_RESIZE(SelecMat_,2,2);
  SelecMat_(0,0) =  1; SelecMat_(0,1) = 0;
  SelecMat_(1,0) = -1; SelecMat_(1,1) = 1;

  MAL_MATRIX_RESIZE(rotMat1_,2,2);
  MAL_MATRIX_RESIZE(rotMat2_,2,2);
  MAL_MATRIX_RESIZE(drotMat1_,2,2);
  MAL_MATRIX_RESIZE(drotMat2_,2,2);

  MAL_MATRIX_RESIZE(ASx_xy_,nc_foot_,nf_);
  MAL_MATRIX_RESIZE(ASy_xy_,nc_foot_,nf_);
  MAL_MATRIX_RESIZE(ASx_theta_,nc_foot_,nf_) ;
  MAL_MATRIX_RESIZE(ASy_theta_,nc_foot_,nf_) ;

  MAL_MATRIX_FILL(Afoot_xy_   ,0.0);
  MAL_MATRIX_FILL(Afoot_theta_,0.0);
  MAL_MATRIX_FILL(AS_theta_   ,0.0);
  MAL_VECTOR_FILL(UBfoot_     ,0.0);
  MAL_VECTOR_FILL(LBfoot_     ,-1e+08);

  MAL_MATRIX_RESIZE(derv_Afoot_map_,nc_foot_,N_);

  updateFootPoseConstraint() ;

  return ;
}

void NMPCgenerator::updateFootPoseConstraint()
{
  // rotation matrice from F_k+1 to F_k
  rotMat1_(0,0)= cos(currentSupport_.Yaw) ; rotMat1_(0,1)= sin(currentSupport_.Yaw) ;
  rotMat1_(1,0)=-sin(currentSupport_.Yaw) ; rotMat1_(1,1)= cos(currentSupport_.Yaw) ;
  rotMat2_(0,0)= cos(F_kp1_theta_(0))    ; rotMat2_(0,1)= sin(F_kp1_theta_(0)) ;
  rotMat2_(1,0)=-sin(F_kp1_theta_(0))    ; rotMat2_(1,1)= cos(F_kp1_theta_(0)) ;

  drotMat1_(0,0)=-sin(currentSupport_.Yaw) ; drotMat1_(0,1)= cos(currentSupport_.Yaw) ;
  drotMat1_(1,0)=-cos(currentSupport_.Yaw) ; drotMat1_(1,1)=-sin(currentSupport_.Yaw) ;
  drotMat2_(0,0)=-sin(F_kp1_theta_(0))    ; drotMat2_(0,1)= cos(F_kp1_theta_(0)) ;
  drotMat2_(1,0)=-cos(F_kp1_theta_(0))    ; drotMat2_(1,1)=-sin(F_kp1_theta_(0)) ;
#ifdef DEBUG
  cout << "yaw = " << currentSupport_.Yaw << endl;
#endif
  // every time instant in the pattern generator constraints
  // depend on the support order
  MAL_MATRIX_TYPE(double) Af1_xy, Af1_theta, Af2_xy, Af2_theta ;
  MAL_VECTOR_TYPE(double) Bf1 , Bf2 ;
  // reset matrix
  MAL_MATRIX_FILL(ASx_xy_ ,0.0);
  MAL_MATRIX_FILL(ASy_xy_ ,0.0);
  MAL_MATRIX_FILL(ASx_theta_,0.0);
  MAL_MATRIX_FILL(ASy_theta_,0.0);
  if (currentSupport_.Foot == LEFT)
  {
    Af1_xy    = MAL_RET_A_by_B(A0r_,rotMat1_) ;
    Af2_xy    = MAL_RET_A_by_B(A0l_,rotMat2_) ;
    Af1_theta = MAL_RET_A_by_B(A0r_,drotMat1_) ;
    Af2_theta = MAL_RET_A_by_B(A0l_,drotMat2_) ;
    Bf1       = ubB0r_ ;
    Bf2       = ubB0l_ ;
  }else{
    Af1_xy    = MAL_RET_A_by_B(A0l_,rotMat1_) ;
    Af2_xy    = MAL_RET_A_by_B(A0r_,rotMat2_) ;
    Af1_theta = MAL_RET_A_by_B(A0l_,drotMat1_) ;
    Af2_theta = MAL_RET_A_by_B(A0r_,drotMat2_) ;
    Bf1       = ubB0l_ ;
    Bf2       = ubB0r_ ;
  }
#ifdef DEBUG
  cout << "Af1_xy" << Af1_xy << endl ;
  cout << "Af2_xy" << Af2_xy << endl ;
#endif
  unsigned nbEdges = MAL_MATRIX_NB_ROWS(Af1_xy);
  for (unsigned i=0 ; i<nbEdges ; ++i)
  {
    ASx_xy_(i,0) = Af1_xy(i,0) ;
    ASx_xy_(i+nbEdges,1) = Af2_xy(i,0) ;

    ASy_xy_(i,0) = Af1_xy(i,1) ;
    ASy_xy_(i+nbEdges,1) = Af2_xy(i,1) ;

    ASx_theta_(i,0) = Af1_theta(i,0) ;
    ASx_theta_(i+nbEdges,1) = Af2_theta(i,0) ;

    ASy_theta_(i,0) = Af1_theta(i,1) ;
    ASy_theta_(i+nbEdges,1) = Af2_theta(i,1) ;
  }

  for (unsigned i=0 ; i<nbEdges ; ++i)
  {
    UBfoot_(i)         = Bf1(i);
    UBfoot_(i+nbEdges) = Bf2(i);
  }

  MAL_VECTOR_DIM(SfootX,double,2);
  SfootX(0) = currentSupport_.X ; SfootX(1) = 0.0 ;
  MAL_VECTOR_DIM(SfootY,double,2);
  SfootY(0) = currentSupport_.Y ; SfootY(1) = 0.0 ;

  UBfoot_ = UBfoot_ + MAL_RET_A_by_B(ASx_xy_,SfootX)  + MAL_RET_A_by_B(ASy_xy_,SfootY) ;
#ifdef DEBUG
  DumpVector("UBfoot_",UBfoot_);
  DumpMatrix("Ax_xy_",ASx_xy_);
  DumpMatrix("Ay_xy_",ASy_xy_);
#endif
  ASx_xy_    = MAL_RET_A_by_B(ASx_xy_   ,SelecMat_);
  ASy_xy_    = MAL_RET_A_by_B(ASy_xy_   ,SelecMat_);
  ASx_theta_ = MAL_RET_A_by_B(ASx_theta_,SelecMat_);
  ASy_theta_ = MAL_RET_A_by_B(ASy_theta_,SelecMat_);
#ifdef DEBUG
  DumpMatrix("ASx_xy_",ASx_xy_);
  DumpMatrix("ASy_xy_",ASy_xy_);
#endif
  for (unsigned i=0 ; i<nc_foot_ ; ++i)
  {
    for (unsigned j=0 ; j<nf_ ; ++j)
    {
      Afoot_xy_(i,N_+j)       = ASx_xy_(i,j);
      Afoot_xy_(i,2*N_+nf_+j) = ASy_xy_(i,j);
      AS_theta_(i,N_+j)       = ASx_theta_(i,j);
      AS_theta_(i,2*N_+nf_+j) = ASy_theta_(i,j);
    }
  }

  MAL_MATRIX_FILL(derv_Afoot_map_,0.0);
  for (unsigned k=0 , n=nbEdges ; k<nf_-1 ;++k , n+=nbEdges)
  {
    for (unsigned j=0 ; j<N_ ; ++j )
    {
      if (V_kp1_(j,k) == 1)
      {
        for (unsigned i=0 ; i<nbEdges ; ++i )
          derv_Afoot_map_(n+i,j) = 1.0 ;
        break;
      }
    }
  }

  MAL_VECTOR_TYPE(double) dummy = MAL_RET_A_by_B(AS_theta_,U_xy_);
  dummy = MAL_RET_A_by_B(dummy,derv_Afoot_map_);
  dummy = MAL_RET_A_by_B(dummy,V_kp1_);
  for(unsigned i=0 ; i<MAL_MATRIX_NB_ROWS(Afoot_theta_) ; ++i)
    for(unsigned j=0 ; j<MAL_MATRIX_NB_COLS(Afoot_theta_) ; ++j)
      Afoot_theta_(i,j) = dummy(j);
#ifdef DEBUG
  DumpMatrix("Afoot_xy_",Afoot_xy_);
  DumpMatrix("Afoot_theta_",Afoot_theta_);
#endif
  return ;
}

void NMPCgenerator::initializeFootVelIneqConstraint()
{
  /**
  """
  create constraints that freezes foot position optimization when swing
  foot comes close to foot step in preview window. Needed for proper
  interpolation of trajectory.
  """
  # B ( f_x_kp1 - f_x_k ) <= (t_touchdown - t) v_max
  #   ( f_y_kp1 - f_y_k )
  #
  # -inf <= velref / ||velref|| S x <= (t_touchdown - t) v_max_x
  #
  # with S a selection matrix selecting the f_x_k+1 and f_y_k+1 accordingly.
  # and B the direction vector of vel_ref : vel_ref / || vel_ref ||
  #*/
  nc_vel_ = 3 ;
  MAL_MATRIX_RESIZE(Avel_ ,nc_vel_,2*N_+3*nf_)  ;
  MAL_VECTOR_RESIZE(UBvel_,nc_vel_)  ;
  MAL_VECTOR_RESIZE(LBvel_,nc_vel_)  ;

  MAL_MATRIX_FILL(Avel_ ,0.0)  ;
  MAL_VECTOR_FILL(UBvel_,0.0)  ;
  MAL_VECTOR_FILL(LBvel_,-1e+08)  ;

  updateFootVelIneqConstraint();
  return ;
}

void NMPCgenerator::updateFootVelIneqConstraint()
{
  unsigned itBeforeLanding = 0;
  for(unsigned i=0 ; i<MAL_VECTOR_SIZE(v_kp1_) ; ++i)
    if(v_kp1_(i))
      ++itBeforeLanding;
    else
      break;

  double dt = (double)itBeforeLanding * T_ ;// (t_touchdown - t)

  double vref_x = vel_ref_.Global.X ;
  double vref_y = vel_ref_.Global.Y ;
  double norm_vref = sqrt(vref_x*vref_x + vref_y*vref_y ) ;
  double dyaw = vel_ref_.Global.Yaw ;
  double signq = dyaw>=0.0?1:-1;

  double xvmax(0.4), yvmax(0.4), yawvmax(1.5) ;// [m/s,m/s,rad/s]

  UBvel_(0) = (dt+T_) * xvmax + vref_x * F_kp1_x_(0) / norm_vref ;
  UBvel_(1) = (dt+T_) * yvmax + vref_y * F_kp1_y_(0) / norm_vref ;

  Avel_(0, N_      ) = vref_x / norm_vref ;
  Avel_(1, 2*N_+nf_) = vref_y / norm_vref ;

  Avel_ (2,2*N_+2*nf_) = signq ;
  UBvel_(2) = (dt+T_) * yawvmax + signq * F_kp1_theta_(0) ;
#ifdef DEBUG
  DumpMatrix("Avel_",Avel_);
  DumpVector("UBvel_",UBvel_);
#endif
  return ;
}

void NMPCgenerator::initializeRotIneqConstraint()
{
  /**""" constraints on relative angular velocity """
  # build foot angle constraints
  # || F_kp1_q - F_k_q ||_2^2 <= 0.09 ~ 5 degrees
  # <=>
  # -0.09 + f_k_q <= [  1 0 ] [f_kp1_q] <= 0.09 + f_k_q
  # -0.09         <= [ -1 1 ] [f_kp2_q] <= 0.09
  */
  nc_rot_ = nf_ ;
  MAL_MATRIX_RESIZE(Arot_ ,nc_rot_,2*N_+3*nf_);
  MAL_VECTOR_RESIZE(UBrot_,nc_rot_);
  MAL_VECTOR_RESIZE(LBrot_,nc_rot_);

  MAL_MATRIX_FILL(Arot_ ,0.0);
  MAL_VECTOR_FILL(UBrot_,0.0);
  MAL_VECTOR_FILL(LBrot_,0.0);

  Arot_(0,2*N_+2*nf_  ) = 1.0 ;
  Arot_(0,2*N_+2*nf_+1) = 0.0 ;
  Arot_(1,2*N_+2*nf_  ) = -1.0 ;
  Arot_(1,2*N_+2*nf_+1) =  1.0 ;
  UBrot_(1) =  0.09 ;
  LBrot_(1) = -0.09 ;

  updateRotIneqConstraint();
}

void NMPCgenerator::updateRotIneqConstraint()
{
  UBrot_(0) =  0.09 + currentSupport_.Yaw ;
  LBrot_(0) = -0.09 + currentSupport_.Yaw ;
#ifdef DEBUG
  DumpMatrix("Arot_", Arot_);
  DumpVector("UBrot_",UBrot_);
  DumpVector("LBrot_",LBrot_);
#endif
  return;
}

void NMPCgenerator::initializeObstacleConstraint()
{
  /**
  """ constraints coming from obstacles """
  #
  # inf > X Hobs X + Aobs X > Bobs
  #
  */
  obstacles_.clear();
  UBobs_.clear();
  LBobs_.clear();
  Hobs_.clear();
  Aobs_.clear();
  nc_obs_ = obstacles_.size();

  Circle obstacle ;
  obstacle.x_0    = -1.0 ;
  obstacle.y_0    = 0.3 ;
  obstacle.r      = 0.5/2 ;
  obstacle.margin = 0.4172 ;
  obstacles_.push_back(obstacle);

  updateObstacleConstraint();
}

void NMPCgenerator::updateObstacleConstraint()
{
  nc_obs_ = nf_*obstacles_.size() ;
  MAL_MATRIX_DIM(H,double,2*(N_+nf_),2*(N_+nf_));
  MAL_VECTOR_DIM(A,double,2*(N_+nf_));
  MAL_VECTOR_DIM(B,double,nc_obs_);

  MAL_MATRIX_FILL(H,0.0);
  MAL_VECTOR_FILL(A,0.0);
  MAL_VECTOR_FILL(B,0.0);

  Hobs_.resize(obstacles_.size() , std::vector<MAL_MATRIX_TYPE(double)>(nc_obs_,H) );
  Aobs_.resize(obstacles_.size() , std::vector<MAL_VECTOR_TYPE(double)>(nc_obs_,A) );
  LBobs_.resize(obstacles_.size() , B );
  UBobs_.resize(obstacles_.size() , B );

  unsigned nc = nf_ ;
  for(unsigned obs=0 ; obs<obstacles_.size() ; ++obs)
  {
    for (unsigned i=0 ; i<nc ; ++i)
    {
      Hobs_ [obs][i](N_+i,N_+i)             = 1.0 ;
      Hobs_ [obs][i](2*N_+nf_+i,2*N_+nf_+i) = 1.0 ;
      Aobs_ [obs][i](N_+i)       = -2*obstacles_[obs].x_0 ;
      Aobs_ [obs][i](2*N_+nf_+i) = -2*obstacles_[obs].y_0 ;
      LBobs_[obs](i) = - obstacles_[obs].x_0*obstacles_[obs].x_0
                       - obstacles_[obs].y_0*obstacles_[obs].y_0
                       + (obstacles_[obs].r+obstacles_[obs].margin)
                        *(obstacles_[obs].r+obstacles_[obs].margin) ;
      MAL_VECTOR_FILL(UBobs_[obs],1e+8);
    }
  }
#ifdef DEBUG
  DumpMatrix("Hobs_0" ,Hobs_ [0][0]);
  DumpVector("Aobs_0" ,Aobs_ [0][0]);
  DumpMatrix("Hobs_1" ,Hobs_ [0][1]);
  DumpVector("Aobs_1" ,Aobs_ [0][1]);
  DumpVector("LBobs_",LBobs_[0]);
#endif

  return ;
}

void NMPCgenerator::initializeCostFunction()
{
  // number of constraint
  nc_ = nc_cop_+nc_foot_+nc_vel_+nc_rot_+nc_obs_ ;

  MAL_MATRIX_RESIZE(qp_H_       ,nv_,nv_);        MAL_MATRIX_SET_IDENTITY(qp_H_);
  MAL_VECTOR_RESIZE(qp_g_       ,nv_);            MAL_VECTOR_FILL(qp_g_         ,0.0);
  MAL_VECTOR_RESIZE(qp_g_x_     ,N_+nf_);         MAL_VECTOR_FILL(qp_g_x_       ,0.0);
  MAL_VECTOR_RESIZE(qp_g_y_     ,N_+nf_);         MAL_VECTOR_FILL(qp_g_y_       ,0.0);
  MAL_VECTOR_RESIZE(qp_g_theta_ ,nf_);            MAL_VECTOR_FILL(qp_g_theta_   ,0.0);
  MAL_MATRIX_RESIZE(qp_J_       ,nc_,nv_);        MAL_MATRIX_FILL(qp_J_         ,0.0);
  MAL_MATRIX_RESIZE(qp_J_cop_   ,nc_cop_,nv_);    MAL_MATRIX_FILL(qp_J_cop_     ,0.0);
  MAL_MATRIX_RESIZE(qp_J_foot_  ,nc_foot_,nv_);   MAL_MATRIX_FILL(qp_J_foot_    ,0.0);
  MAL_MATRIX_RESIZE(qp_J_vel_   ,nc_vel_,nv_);    MAL_MATRIX_FILL(qp_J_vel_     ,0.0);
  MAL_MATRIX_RESIZE(qp_J_obs_   ,nc_obs_,nv_);    MAL_MATRIX_FILL(qp_J_obs_     ,0.0);
  MAL_MATRIX_RESIZE(qp_J_rot_   ,nc_rot_,nv_);    MAL_MATRIX_FILL(qp_J_rot_     ,0.0);
  MAL_VECTOR_RESIZE(qp_lbJ_     ,nc_);            MAL_VECTOR_FILL(qp_lbJ_       ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_     ,nc_);            MAL_VECTOR_FILL(qp_ubJ_       ,0.0);
  MAL_VECTOR_RESIZE(qp_lbJ_cop_ ,nc_cop_);        MAL_VECTOR_FILL(qp_lbJ_cop_   ,0.0);
  MAL_VECTOR_RESIZE(qp_lbJ_foot_,nc_foot_);       MAL_VECTOR_FILL(qp_lbJ_foot_  ,0.0);
  MAL_VECTOR_RESIZE(qp_lbJ_vel_ ,nc_vel_);        MAL_VECTOR_FILL(qp_lbJ_vel_   ,0.0);
  MAL_VECTOR_RESIZE(qp_lbJ_obs_ ,nc_obs_);        MAL_VECTOR_FILL(qp_lbJ_obs_   ,0.0);
  MAL_VECTOR_RESIZE(qp_lbJ_rot_ ,nc_rot_);        MAL_VECTOR_FILL(qp_lbJ_rot_   ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_cop_ ,nc_cop_);        MAL_VECTOR_FILL(qp_ubJ_cop_   ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_foot_,nc_foot_);       MAL_VECTOR_FILL(qp_ubJ_foot_  ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_vel_ ,nc_vel_);        MAL_VECTOR_FILL(qp_ubJ_vel_   ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_obs_ ,nc_obs_);        MAL_VECTOR_FILL(qp_ubJ_obs_   ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_rot_ ,nc_rot_);        MAL_VECTOR_FILL(qp_ubJ_rot_   ,0.0);
  MAL_VECTOR_RESIZE(qp_lb_      ,nv_);            MAL_VECTOR_FILL(qp_lb_        ,-1e+8);
  MAL_VECTOR_RESIZE(qp_ub_      ,nv_);            MAL_VECTOR_FILL(qp_ub_        , 1e+8);
  MAL_MATRIX_RESIZE(Q_x_        ,N_+nf_, N_+nf_); MAL_MATRIX_FILL(Q_x_          ,0.0);
  MAL_MATRIX_RESIZE(Q_x_XX_     ,N_,N_);          MAL_MATRIX_FILL(Q_x_XX_       ,0.0);
  MAL_MATRIX_RESIZE(Q_x_XF_     ,N_,nf_);         MAL_MATRIX_FILL(Q_x_XF_       ,0.0);
  MAL_MATRIX_RESIZE(Q_x_FX_     ,nf_,N_);         MAL_MATRIX_FILL(Q_x_FX_       ,0.0);
  MAL_MATRIX_RESIZE(Q_x_FF_     ,nf_,nf_);        MAL_MATRIX_FILL(Q_x_FF_       ,0.0);
  MAL_MATRIX_RESIZE(Q_theta_    ,nf_,nf_);        MAL_MATRIX_SET_IDENTITY(Q_theta_);
  MAL_VECTOR_RESIZE(p_x_        ,N_+nf_);         MAL_VECTOR_FILL(p_x_          , 0.0);
  MAL_VECTOR_RESIZE(p_y_        ,N_+nf_);         MAL_VECTOR_FILL(p_y_          , 0.0);
  MAL_VECTOR_RESIZE(p_xy_X_     ,N_);             MAL_VECTOR_FILL(p_xy_X_       , 0.0);
  MAL_VECTOR_RESIZE(p_xy_Fx_    ,nf_);            MAL_VECTOR_FILL(p_xy_Fx_      , 0.0);
  MAL_VECTOR_RESIZE(p_xy_Y_     ,N_);             MAL_VECTOR_FILL(p_xy_Y_       , 0.0);
  MAL_VECTOR_RESIZE(p_xy_Fy_    ,nf_);            MAL_VECTOR_FILL(p_xy_Fy_      , 0.0);
  MAL_VECTOR_RESIZE(p_theta_    ,nf_);            MAL_VECTOR_FILL(p_theta_      , 0.0);
  MAL_MATRIX_RESIZE(I_NN_       ,N_,N_);          MAL_MATRIX_SET_IDENTITY(I_NN_);
  MAL_VECTOR_RESIZE(Pvsc_x_     ,N_);             MAL_VECTOR_FILL(Pvsc_x_       , 0.0);
  MAL_VECTOR_RESIZE(Pvsc_y_     ,N_);             MAL_VECTOR_FILL(Pvsc_y_       , 0.0);

  // Q_xXX = (  0.5 * a * Pvu^T   * Pvu + b * Pzu^T * Pzu + c * I )
  // Q_xXF = ( -0.5 * b * Pzu^T   * V_kp1 )
  // Q_xFX = ( -0.5 * b * V_kp1^T * Pzu )^T
  // Q_xFF = (  0.5 * b * V_kp1^T * V_kp1 )
  Q_x_XX_ = alpha_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pvu_),Pvu_)
          + beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_),Pzu_)
          + gamma_ * I_NN_ ;
  // Q_x_XF_, Q_x_FX_, Q_x_FF_ are time dependant matrices so they
  // are computed in the update function

  // Q_q = 0.5 * a * ( 1 0 )
  //                   ( 0 1 )
  Q_theta_ *= alpha_ ;

  // p_xy_ =  ( p_xy_X_, p_xy_Fx_, p_xy_Y_, p_xy_Fy_ )
  // p_theta_ = ( 0.5 * a * (-2) * [ f_k_theta+T_step*dTheta^ref  f_k_theta+2*T_step*dTheta^ref ] )
  // Those are time dependant matrices so they are computed in the update function

  updateCostFunction();
}

void NMPCgenerator::updateCostFunction()
{
  nc_ = nc_cop_+nc_foot_+nc_vel_+nc_rot_+nc_obs_ ;
  // Q_xXX = (  0.5 * a * Pvu^T   * Pvu + b * Pzu^T * Pzu + c * I )
  // Q_xXF = ( -0.5 * b * Pzu^T   * V_kp1 )
  // Q_xFX = ( -0.5 * b * V_kp1^T * Pzu )^T
  // Q_xFF = (  0.5 * b * V_kp1^T * V_kp1 )
  Q_x_XF_ = - beta_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_),V_kp1_);
  Q_x_FX_ = - beta_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(V_kp1_),Pzu_);
  Q_x_FF_ =   beta_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(V_kp1_),V_kp1_);

  // Q_x = ( Q_xXX Q_xXF ) = Q_y
  //       ( Q_xFX Q_xFF )
  for(unsigned i=0 ; i<N_ ; ++i)
    for(unsigned j=0 ; j<N_ ; ++j)
      Q_x_(i,j) = Q_x_XX_(i,j) ;
  for(unsigned i=0 ; i<N_ ; ++i)
    for(unsigned j=0 ; j<nf_ ; ++j)
      Q_x_(i,N_+j) = Q_x_XF_(i,j) ;
  for(unsigned i=0 ; i<nf_ ; ++i)
    for(unsigned j=0 ; j<N_ ; ++j)
      Q_x_(N_+i,j) = Q_x_FX_(i,j) ;
  for(unsigned i=0 ; i<nf_ ; ++i)
    for(unsigned j=0 ; j<nf_ ; ++j)
      Q_x_(N_+i,N_+j) = Q_x_FF_(i,j) ;

  Pvsc_x_ = MAL_RET_A_by_B(Pvs_ , c_k_x_) ;
  Pvsc_y_ = MAL_RET_A_by_B(Pvs_ , c_k_y_) ;
  // Pzsc_x_, Pzsc_y_ , v_kp1f_x_ and v_kp1f_y_ alreday up to date
  //from the CoP constraint building function

  // p_xy_ =  ( p_xy_X_, p_xy_Fx_, p_xy_Y_, p_xy_Fy_ )
  // p_xy_X  =   0.5 * a * Pvu^T   * ( Pvs * c_k_x - dX^ref )
  //           + 0.5 * b * Pzu^T   * ( Pzs * c_k_x - v_kp1 * f_k_x )
  // p_xy_Fx = - 0.5 * b * V_kp1^T * ( Pzs * c_k_x - v_kp1 * f_k_x )
  // p_xy_Y  =   0.5 * a * Pvu^T   * ( Pvs * c_k_y - dY^ref )
  //           + 0.5 * b * Pzu^T   * ( Pzs * c_k_y - v_kp1 * f_k_y )
  // p_xy_Fy = - 0.5 * b * V_kp1^T * ( Pzs * c_k_y - v_kp1 * f_k_y )
  p_xy_X_ =    alpha_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pvu_  ), Pvsc_x_ - vel_ref_.Global.X_vec)
             + beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_  ), Pzsc_x_ - v_kp1f_x_                 );
#ifdef DEBUG
  DumpVector("Pvsc_x_"    , Pvsc_x_                    ) ;
  DumpVector("RefVectorX" , global_vel_ref_.RefVectorX ) ;
  DumpVector("Pzsc_x_"    , Pzsc_x_                    ) ;
  DumpVector("v_kp1f_x_"  , v_kp1f_x_                  ) ;
#endif
  p_xy_Fx_ = - beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(V_kp1_), Pzsc_x_ - v_kp1f_x_                 );

  p_xy_Y_  =   alpha_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pvu_  ), Pvsc_y_ - vel_ref_.Global.Y_vec)
             + beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_  ), Pzsc_y_ - v_kp1f_y_                 );

  p_xy_Fy_ = - beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(V_kp1_), Pzsc_y_ - v_kp1f_y_                 );
  for(unsigned i=0 ; i<N_ ; ++i)
    p_x_(i) = p_xy_X_(i) ;
  for(unsigned i=0 ; i<nf_ ; ++i)
    p_x_(N_+i) = p_xy_Fx_(i) ;
  for(unsigned i=0 ; i<N_ ; ++i)
    p_y_(i) = p_xy_Y_(i) ;
  for(unsigned i=0 ; i<nf_ ; ++i)
    p_y_(N_+i) = p_xy_Fy_(i) ;


  // p_theta_ = ( 0.5 * a * [ f_k_theta+T_step*dTheta^ref  f_k_theta+2*T_step*dTheta^ref ] )
  p_theta_(0) = - (alpha_ * currentSupport_.Yaw +     T_step_* vel_ref_.Global.Yaw) ;
  p_theta_(1) = - (alpha_ * currentSupport_.Yaw + 2 * T_step_* vel_ref_.Global.Yaw) ;
#ifdef DEBUG
  DumpMatrix("Q_theta_",Q_theta_);
  DumpMatrix("Q_x_"    ,Q_x_    );
  DumpVector("p_x_"   ,p_x_   );
  DumpVector("p_y_"   ,p_y_   );
  DumpVector("p_theta_",p_theta_);
#endif
  // define QP matrices
  // Gauss-Newton Hessian approximation
  //                                dim :
  // H = (Q_x_     0          0   ) N_+nf_
  //     ( 0      Q_x_        0   ) N_+nf_
  //     ( 0       0      Q_theta_) nf_
  //dim : N_+nf_  N_+nf_    nf_     = nv_
  //
  unsigned n_x = N_+nf_;
  unsigned n_xy = 2*(N_+nf_);
  unsigned n_theta = nf_;
  for(unsigned i=0 ; i<n_x ;++i)
  {
    for(unsigned j=0 ; j<n_x ;++j)
    {
      // Q_x_(i,j)=Q_y_(i,j)
      qp_H_(i,j)=Q_x_(i,j) ;
      qp_H_(i+n_x,j+n_x)=Q_x_(i,j) ;
    }
  }
  for(unsigned i=0 ; i<n_theta ;++i)
    for(unsigned j=0 ; j<n_theta ;++j)
      qp_H_(i+n_xy,j+n_xy)=Q_theta_(i,j) ;

  // Gradient of Objective
  // qp_g_ = (gx    )
  //         (gy    )
  //         (gtheta)
#ifdef DEBUG
  DumpVector( "U_x_" , U_x_ );
#endif
  qp_g_x_ = MAL_RET_A_by_B(U_x_,Q_x_) + p_x_ ;


  qp_g_y_ = MAL_RET_A_by_B(U_y_,Q_x_/*=Q_y_*/) + p_y_ ;
  qp_g_theta_ = MAL_RET_A_by_B(F_kp1_theta_,Q_theta_) + p_theta_ ;
  for(unsigned i=0 ; i<n_x ; ++i)
    qp_g_(i)=qp_g_x_(i) ;
  for(unsigned i=0 ; i<n_x ; ++i)
    qp_g_(i+n_x)=qp_g_y_(i) ;
  for(unsigned i=0 ; i<n_theta ; ++i)
    qp_g_(i+2*n_x)=qp_g_theta_(i) ;

  // Constraints
  // qp_lbJ_ = (qp_lbJ_cop_  ) < qp_J_ * X = (qp_J_cop_  ) * X < qp_ubJ_ = (qp_ubJ_cop_  )
  //           (qp_lbJ_foot_ ) <             (qp_J_foot_ )     <           (qp_ubJ_foot_ )
  //           (qp_lbJ_vel_  ) <             (qp_J_vel_  )     <           (qp_ubJ_vel_  )
  //           (qp_lbJ_obs_  ) <             (qp_J_obs_  )     <           (qp_ubJ_obs_  )
  //           (qp_lbJ_theta_) <             (qp_J_theta_)     <           (qp_ubJ_theta_)
  //
  //  Jacobians
  // qp_J_cop_ = (Acop_xy_  , Acop_theta_ )
  for(unsigned i=0; i<nc_cop_ ; ++i)
  {
    for(unsigned j=0; j<n_xy ; ++j)
      qp_J_cop_(i,j)=Acop_xy_(i,j);
    for(unsigned j=0; j<nf_ ; ++j)
      qp_J_cop_(i,j+n_xy)=Acop_theta_(i,j);
  }
  // qp_J_foot_ = (Afoot_xy_ , Afoot_theta_)
  for(unsigned i=0; i<nc_foot_ ; ++i)
  {
    for(unsigned j=0; j<n_xy ; ++j)
      qp_J_foot_(i,j)=Afoot_xy_(i,j);
    for(unsigned j=0; j<nf_ ; ++j)
      qp_J_foot_(i,j+n_xy)=Afoot_theta_(i,j);
  }
  // qp_J_vel_ = Avel_
  qp_J_vel_ = Avel_ ;
  // qp_J_obs_ = (                 ...                  )
  //             (  Hobs_[obs][nf] * X + Aobs_[obs][nf] )
  //             (                 ...                  )
  MAL_VECTOR_DIM(qp_J_obs_i, double, nv_);
  for(unsigned obs=0 ; obs<obstacles_.size() ; ++obs)
  {
    for(unsigned n=0 ; n<nf_ ; ++n)
    {
      qp_J_obs_i = 2 * MAL_RET_A_by_B(U_xy_,Hobs_[obs][n]) + Aobs_[obs][n] ;
      for(unsigned j=0 ; j<n_xy ; ++j)
        qp_J_obs_((obs+1)*n,j) = qp_J_obs_i(j) ;
    }
  }
  // qp_J_rot_ = Arot_
  qp_J_rot_ = Arot_ ;
  //
  //  Boundaries
  //    CoP
  qp_lbJ_cop_ = LBcop_ - MAL_RET_A_by_B(Acop_xy_,U_xy_);
  qp_ubJ_cop_ = UBcop_ - MAL_RET_A_by_B(Acop_xy_,U_xy_);
  //    Foot
  qp_lbJ_foot_ = LBfoot_ - MAL_RET_A_by_B(Afoot_xy_,U_xy_);
  qp_ubJ_foot_ = UBfoot_ - MAL_RET_A_by_B(Afoot_xy_,U_xy_);
  //    Velocity
  qp_lbJ_vel_ = LBvel_ - MAL_RET_A_by_B(Avel_,U_);
  qp_ubJ_vel_ = UBvel_ - MAL_RET_A_by_B(Avel_,U_);
  //    Obstacle
  for(unsigned obs=0 ; obs<obstacles_.size() ; ++obs)
  {
    for(unsigned n=0 ; n<nf_ ; ++n)
    {
      MAL_VECTOR_TYPE(double) HobsUxy = MAL_RET_A_by_B(Hobs_[obs][n],U_xy_);
      double deltaObs = 0 ;
      for(unsigned i=0 ; i<MAL_VECTOR_SIZE(HobsUxy) ; ++i)
          deltaObs += U_xy_(i) * (HobsUxy(i) + Aobs_[obs][n](i)) ;
      qp_lbJ_obs_((obs+1)*n) = LBobs_[obs](n) - deltaObs ;
      qp_ubJ_obs_((obs+1)*n) = UBobs_[obs](n) - deltaObs ;
    }
  }
  //    Rotation
  qp_lbJ_rot_ = LBrot_ - MAL_RET_A_by_B(Arot_,U_);
  qp_ubJ_rot_ = UBrot_ - MAL_RET_A_by_B(Arot_,U_);

  // Fill up qp_lbJ_, qp_ubJ_ and qp_J_
  MAL_MATRIX_RESIZE(qp_J_  ,nc_,nv_);
  MAL_VECTOR_RESIZE(qp_lbJ_,nc_);
  MAL_VECTOR_RESIZE(qp_ubJ_,nc_);
  MAL_MATRIX_FILL(qp_J_  ,0.0);
  MAL_VECTOR_FILL(qp_lbJ_,0.0);
  MAL_VECTOR_FILL(qp_ubJ_,0.0);
  unsigned index = 0 ;
  for(unsigned i=0 ; i<nc_cop_ ; ++i)
  {
    qp_lbJ_(i) = qp_lbJ_cop_(i) ;
    qp_ubJ_(i) = qp_ubJ_cop_(i) ;
    for(unsigned j=0 ; j<nv_ ; ++j)
      qp_J_(i,j) = qp_J_cop_(i,j);
  }
  index = nc_cop_ ;
  for(unsigned i=0 ; i<nc_foot_ ; ++i)
  {
    qp_lbJ_(i+index) = qp_lbJ_foot_(i) ;
    qp_ubJ_(i+index) = qp_ubJ_foot_(i) ;
    for(unsigned j=0 ; j<nv_ ; ++j)
      qp_J_(i+index,j) = qp_J_foot_(i,j);
  }
  index += nc_foot_ ;
  for(unsigned i=0 ; i<nc_vel_ ; ++i)
  {
    qp_lbJ_(i+index) = qp_lbJ_vel_(i) ;
    qp_ubJ_(i+index) = qp_ubJ_vel_(i) ;
    for(unsigned j=0 ; j<nv_ ; ++j)
      qp_J_(i+index,j) = qp_J_vel_(i,j);
  }
  index += nc_vel_ ;
  for(unsigned i=0 ; i<nc_obs_ ; ++i)
  {
    qp_lbJ_(i+index) = qp_lbJ_obs_(i) ;
    qp_ubJ_(i+index) = qp_ubJ_obs_(i) ;
    for(unsigned j=0 ; j<nv_ ; ++j)
      qp_J_(i+index,j) = qp_J_obs_(i,j);
  }
  index += nc_obs_ ;
  for(unsigned i=0 ; i<nc_rot_ ; ++i)
  {
    qp_lbJ_(i+index) = qp_lbJ_rot_(i) ;
    qp_ubJ_(i+index) = qp_ubJ_rot_(i) ;
    for(unsigned j=0 ; j<nv_ ; ++j)
      qp_J_(i+index,j) = qp_J_rot_(i,j);
  }
#ifdef DEBUG
  DumpMatrix("qp_H_",qp_H_);
  DumpVector("qp_g_",qp_g_);
  DumpMatrix("qp_J_",qp_J_);
  DumpVector("qp_lbJ_",qp_lbJ_);
  DumpVector("qp_ubJ_",qp_ubJ_);
#endif
  return ;
}

void NMPCgenerator::setLocalVelocityReference(reference_t local_vel_ref)
{
  vel_ref_.Local = local_vel_ref.Local ;
  vel_ref_.Global.X   = vel_ref_.Local.X * cos(currentSupport_.Yaw) - vel_ref_.Local.Y * sin(currentSupport_.Yaw) ;
  vel_ref_.Global.Y   = vel_ref_.Local.X * sin(currentSupport_.Yaw) + vel_ref_.Local.Y * cos(currentSupport_.Yaw) ;
  vel_ref_.Global.Yaw = vel_ref_.Local.Yaw ;
  MAL_VECTOR_RESIZE(vel_ref_.Global.X_vec , N_) ;
  MAL_VECTOR_RESIZE(vel_ref_.Global.Y_vec , N_) ;
  MAL_VECTOR_FILL(vel_ref_.Global.X_vec   , vel_ref_.Global.X  ) ;
  MAL_VECTOR_FILL(vel_ref_.Global.Y_vec   , vel_ref_.Global.Y  ) ;
#ifdef DEBUG
  DumpVector("RefVectorX"    ,vel_ref_.Global.X_vec  );
  DumpVector("RefVectorY"    ,vel_ref_.Global.Y_vec  );
#endif
  return ;
}

void NMPCgenerator::setGlobalVelocityReference(reference_t global_vel_ref)
{
  vel_ref_.Global = global_vel_ref.Global ;
  vel_ref_.Local.X   =  vel_ref_.Global.X * cos(currentSupport_.Yaw) + vel_ref_.Global.Y * sin(currentSupport_.Yaw) ;
  vel_ref_.Local.Y   = -vel_ref_.Global.X * sin(currentSupport_.Yaw) + vel_ref_.Global.Y * cos(currentSupport_.Yaw) ;
  vel_ref_.Local.Yaw = vel_ref_.Global.Yaw ;
  MAL_VECTOR_RESIZE(vel_ref_.Global.X_vec , N_) ;
  MAL_VECTOR_RESIZE(vel_ref_.Global.Y_vec , N_) ;
  MAL_VECTOR_FILL(vel_ref_.Global.X_vec   , vel_ref_.Global.X  ) ;
  MAL_VECTOR_FILL(vel_ref_.Global.Y_vec   , vel_ref_.Global.Y  ) ;
#ifdef DEBUG
  DumpVector("RefVectorX"    ,vel_ref_.Global.X_vec  );
  DumpVector("RefVectorY"    ,vel_ref_.Global.Y_vec  );
#endif
  return ;
}

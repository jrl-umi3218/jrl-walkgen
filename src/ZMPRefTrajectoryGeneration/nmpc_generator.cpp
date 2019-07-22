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

#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen/Dense>

#include <ZMPRefTrajectoryGeneration/nmpc_generator.hh>
#include <cmath>
#include <Debug.hh>

//#define DEBUG
//#define DEBUG_COUT

//#ifdef DEBUG


void DumpMatrix(std::string fileName, Eigen::MatrixXd & M)
{
  std::ofstream aof;
  std::ostringstream oss(std::ostringstream::ate);
  aof.open(fileName.c_str(),std::ofstream::out);
  aof.close();
  aof.open(fileName.c_str(),std::ofstream::app);
  aof.precision(18);
  aof.setf(std::ios::scientific, std::ios::floatfield);
  for (unsigned int i = 0 ; i < M.rows() ; ++i)
  {
    for (unsigned int j = 0 ; j < M.cols()-1 ; ++j)
    {
      if(M(i,j)*M(i,j) < 1e-10)
      {
        aof << 0.0 << " " ;
      }else
      {
        aof << M(i,j) << " " ;
      }
    }
    if(M(i,M.cols()-1)*M(i,M.cols()-1) < 1e-10)
    {
      aof << 0.0 << std::endl ;
    }else
    {
      aof << M(i,M.cols()-1) << std::endl ;
    }
  }
}

void DumpVector(std::string fileName, Eigen::VectorXd & M)
{
  std::ofstream aof;
  std::ostringstream oss(std::ostringstream::ate);
  aof.open(fileName.c_str(),std::ofstream::out);
  aof.close();
  aof.open(fileName.c_str(),std::ofstream::app);
  aof.precision(18);
  aof.setf(std::ios::scientific, std::ios::floatfield);
  for (unsigned int i = 0 ; i < M.size() ; ++i)
  {
    if(M(i)*M(i) > 0.000001)
    {
      aof << M(i) << std::endl ;
    }else
    {
      aof << 0.0 << std::endl ;
    }
  }
}
//#endif // DEBUG

using namespace std;
using namespace PatternGeneratorJRL;

NMPCgenerator::
NMPCgenerator
(SimplePluginManager * aSPM, PinocchioRobot *aPR)
{
  exit_on_error_ = true;
  time_=0.0;
  T_ = 0.0 ;
  Tfirst_ = 0.0 ;
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

  nceq_    = 0 ;
  ncineq_  = 0 ;
  nc_ = ncineq_+nceq_;
  nc_cop_  = 0 ;
  nc_foot_ = 0 ;
  nc_vel_  = 0 ;
  nc_rot_  = 0 ;
  nc_obs_  = 0 ;
  nc_stan_ = 0 ;

  alpha_x_     = 0.0 ;
  alpha_y_     = 0.0 ;
  alpha_theta_ = 0.0 ;
  beta_ =0.0;
  minjerk_=0.0;
  delta_=0.0;
  kappa_=0.0;

  SPM_ = aSPM ;
  PR_ = aPR ;

  FSM_ = new SupportFSM();
  RFI_ = new RelativeFeetInequalities(SPM_,PR_) ;

  QP_=NULL;
  QuadProg_H_.resize(1,1);
  QuadProg_J_eq_.resize(1,1);
  QuadProg_J_ineq_.resize(1,1);
  QuadProg_g_.resize(1);
  QuadProg_bJ_eq_.resize(1);
  QuadProg_lbJ_ineq_.resize(1);
  deltaU_.resize(1);

  isQPinitialized_ = false;
  useItBeforeLanding_ = false ;
  useLineSearch_ = false;
  itBeforeLanding_ = 0;

  SupportStates_deq_.clear();

  exit_on_error_ = true;
}

NMPCgenerator::~NMPCgenerator()
{
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

void NMPCgenerator::initNMPCgenerator(
    bool useLineSearch,
    support_state_t & currentSupport,
    COMState &lStartingCOMState,
    reference_t &local_vel_ref,
    unsigned N, unsigned nf, double T, double T_step)
{
  N_ = N ;
  nf_ = nf ;
  // number of degrees of freedom
  nv_ = 2*N_+3*nf_;
  Pps_.resize(N_,3);  Pps_.setZero();
  Ppu_.resize(N_,N_); Ppu_.setZero();
  Pvs_.resize(N_,3);  Pvs_.setZero();
  Pvu_.resize(N_,N_); Pvu_.setZero();
  Pas_.resize(N_,3);  Pas_.setZero();

  Pau_.resize(N_,N_); Pau_.setZero();
  Pzs_.resize(N_,3);  Pzs_.setZero();
  Pzu_.resize(N_,N_); Pzu_.setZero();
  v_kp1_.resize(N_) ; v_kp1_.setZero();
  V_kp1_.resize(N_,nf_) ; V_kp1_.setZero();
  U_.resize(2*N_+3*nf_);  U_.setZero();
  deltaU_thresh_.resize(2*N_+3*nf_);
  deltaU_thresh_.setZero();
  U_xy_ .resize(2*(N_+nf_));
  deltaU_thresh_.setZero();
  U_x_.resize(N_);
  U_x_.setZero();
  U_y_.resize(N_);
  U_y_.setZero();
  F_kp1_x_.resize(nf_);
  F_kp1_x_.setZero();
  F_kp1_y_.resize(nf_);
  F_kp1_y_.setZero();
  F_kp1_theta_.resize(nf_);
  F_kp1_theta_.setZero();
  c_k_x_.resize(3);
  c_k_x_.setZero();
  c_k_y_.resize(3);
  c_k_y_.setZero();
  A0r_.resize(5,2) ;
  A0r_.Zero(5,2);
  ubB0r_.resize(5) ;
  ubB0r_.setZero();
  A0l_.resize(5,2) ;
  A0l_.setZero();
  ubB0l_.resize(5) ;
  ubB0l_.setZero();
  A0rf_.resize(4,2) ;
  A0rf_.setZero();
  ubB0rf_.resize(4) ;
  ubB0rf_.setZero();
  A0lf_.resize(4,2) ;
  A0lf_.setZero();
  ubB0lf_.resize(4) ;
  ubB0lf_.setZero();
  A0ds_.resize(4,2) ;
  A0ds_.setZero();
  ubB0ds_.resize(4) ;
  ubB0ds_.setZero();
  A0_xy_.resize(4,2) ;
  A0_xy_.setZero();
  A0_theta_.resize(4,2) ;
  A0_theta_.setZero();
  B0_.resize(4) ;
  B0_.setZero();
  vel_ref_.Global.X_vec.resize(N_) ;
  vel_ref_.Global.Y_vec.resize(N_) ;
  vel_ref_.Local .X_vec.resize( N_) ;
  vel_ref_.Local .Y_vec.resize( N_) ;
  rotMat_xy_.resize(2,2);
  rotMat_xy_.setZero();
  rotMat_theta_.resize(2,2);
  rotMat_theta_.setZero();
  rotMat_.resize(2,2);
  rotMat_.setZero();
  tmpRotMat_.resize(2,2);
  tmpRotMat_.setZero();
  qp_J_obs_i_.resize(nv_);
  qp_J_obs_i_.setZero();
  { for(unsigned int i=0;i<F_kp1_x_.size();F_kp1_x_[i++]=currentSupport.X);};
  { for(unsigned int i=0;i<F_kp1_y_.size();F_kp1_y_[i++]=currentSupport.Y);};

  T_ = T ;
  Tfirst_ = T ;
  T_step_ = T_step ;
  useLineSearch_=useLineSearch;

  alpha_x_     = 5 ; // weight for CoM velocity X tracking  : 0.5 * a ; 2.5
  alpha_y_     = 5 ; // weight for CoM velocity Y tracking  : 0.5 * a ; 2.5
  alpha_theta_ = 1e+06 ; // weight for CoM velocity Yaw tracking : 0.5 * a ; 2.5
  beta_  = 1e+03 ; // weight for ZMP reference tracking : 0.5 * b ; 1e+03
  minjerk_ = 1e-08 ; // weight for jerk minimization      : 0.5 * c ; 1e-04
  delta_ = 1e-06 ; // weight for foot position evolution: 0.5 * d ; 1e-04
  kappa_ = 0.0 ;   // weight for foot distance from support: 0.5 * d ; 1e-04

  SecurityMarginX_ = 0.095 ;
  SecurityMarginY_ = 0.055 ;
  maxSolverIteration_=1;
  oneMoreStep_=false;

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
  SupportStates_deq_.resize(N_+1,currentSupport_);
  FSM_->StepPeriod( T_step_ );
  FSM_->DSPeriod( 1e9 ); // period during the robot move at 0.0 com speed
  FSM_->DSSSPeriod( T_step_ );
  FSM_->NbStepsSSDS( 2 ); // number of previw step
  FSM_->SamplingPeriod( T_ );

  ostringstream oss(std::ostringstream::ate);
  oss << ":setfeetconstraint XY " << SecurityMarginX_
      << " " << SecurityMarginY_ ;
  istringstream strm(oss.str());
  string cmd ;
  strm >> cmd ;
  RFI_->CallMethod(cmd,strm);
  FeetDistance_ = RFI_->DSFeetDistance();

  itBeforeLanding_=(int)round(T_step_/T_);
  itMax_=itBeforeLanding_;
  useItBeforeLanding_ = false ;

  initializeConstraint();
  initializeCostFunction();
  initializeLineSearch();

  // initialize the solver
  // we assume 0 equality constraint at the beginning
  // call QP_->problem((int)nv_,(int)nceq_,(int)ncineq_) before using it
  QP_ = new Eigen::QuadProgDense((int)nv_,(int)nceq_,(int)ncineq_) ;
  QuadProg_H_       .resize(nv_,nv_);
  QuadProg_g_       .resize(nv_);
  QuadProg_J_eq_    .resize(nceq_,nv_);
  QuadProg_bJ_eq_   .resize(nceq_);
  QuadProg_J_ineq_  .resize(ncineq_,nv_);
  QuadProg_lbJ_ineq_.resize(ncineq_);
  deltaU_  .resize(nv_);

  QuadProg_H_       .fill(0.0);
  QuadProg_g_       .fill(0.0);
  QuadProg_J_eq_    .fill(0.0);
  QuadProg_bJ_eq_   .fill(0.0);
  QuadProg_J_ineq_  .fill(0.0);
  QuadProg_lbJ_ineq_.fill(0.0);
  deltaU_  .fill(0.0);
}

void NMPCgenerator::initializeConstraint()
{
  qp_J_.resize(nc_,nv_);
  qp_J_.setZero();
  qp_ubJ_.resize(nc_);
  qp_ubJ_.setZero();
  ub_.resize(nc_);
  ub_.setZero();
  gU_.resize(nc_);
  gU_.setZero();
  Uxy_.resize(2*(N_+nf_));
  Uxy_.setZero();

  Pzuv_.resize(2*N_,2*(N_+nf_));
  Pzsc_.resize(2*N_);
  Pzsc_x_.resize(N_);
  Pzsc_y_.resize(N_);
  v_kp1f_.resize(2*N_);
  v_kp1f_x_.resize(N_);
  v_kp1f_y_.resize(N_);

  // build constant matrices
  buildCoMCoPIntegrationMatrix();
  buildConvexHullSystems();

  // initialize time dependant matrices

  initializeCoPConstraint();
  initializeFootPoseConstraint();
  initializeFootVelIneqConstraint();
  initializeRotIneqConstraint();

  //initializeFootExactPositionConstraint();
  //initializeObstacleConstraint();
  //initializeStandingConstraint();

  ncineq_ = (unsigned int)(nc_cop_ + nc_foot_ + nc_rot_ + nc_obs_);
  nceq_ = nc_vel_ ;
  nc_ = nceq_ + ncineq_;

  gU_cop_.resize(nc_cop_);   gU_cop_.setZero();
  gU_foot_.resize(nc_foot_); gU_foot_.setZero();
  gU_obs_.resize(nc_obs_);   gU_obs_.setZero();
  gU_rot_.resize(nc_rot_);   gU_rot_.setZero();
  gU_stan_.resize(nc_stan_); gU_stan_.setZero();

  return ;
}

void NMPCgenerator::updateConstraint()
{
  updateCoPconstraint(U_);
  updateFootPoseConstraint(U_);
  updateFootVelIneqConstraint();
  updateRotIneqConstraint();

  //updateFootExactPositionConstraint();
  //updateObstacleConstraint();
  //updateStandingConstraint();

  // Global Jacobian for all constraints
  // Constraints
  // qp_lbJ_ = (qp_lbJ_cop_  ) < qp_J_ * X = (qp_J_cop_  ) * X < qp_ubJ_ = (qp_ubJ_cop_  )
  //           (qp_lbJ_foot_ ) <             (qp_J_foot_ )     <           (qp_ubJ_foot_ )
  //           (qp_lbJ_vel_  ) <             (qp_J_vel_  )     <           (qp_ubJ_vel_  )
  //           (qp_lbJ_obs_  ) <             (qp_J_obs_  )     <           (qp_ubJ_obs_  )
  //           (qp_lbJ_theta_) <             (qp_J_theta_)     <           (qp_ubJ_theta_)
  //
  //  Jacobians
  // qp_J_cop_ = (Acop_xy_  , Acop_theta_ )

  // number of constraint
  ncineq_ = (unsigned int)(nc_cop_ + nc_foot_ + nc_rot_ + nc_obs_ + nc_stan_ );
  nceq_ = nc_vel_ ;
  nc_ = ncineq_ + nceq_ ;

  unsigned N2nf2 = 2*(N_+nf_) ;
  qp_J_.resize(nc_,nv_); qp_J_.setZero();

  // Fill up qp_J_
  unsigned index = 0 ;
  for(unsigned i=0 ; i<nc_vel_ ; ++i)
  {
    for(unsigned j=0 ; j<nv_ ; ++j)
      qp_J_(i+index,j) = Avel_(i,j);
  }
  index += nc_vel_ ;
  for(unsigned i=0 ; i<nc_cop_ ; ++i)
  {
    for(unsigned j=0; j<N2nf2 ; ++j)
      qp_J_(i+index,j)=Acop_xy_(i,j);
    for(unsigned j=0; j<nf_ ; ++j)
      qp_J_(i+index,j+N2nf2)=Acop_theta_(i,j);
  }
  index = (unsigned int)nc_cop_ ;
  for(unsigned i=0 ; i<nc_foot_ ; ++i)
  {
    for(unsigned j=0; j<N2nf2 ; ++j)
      qp_J_(i+index,j)=Afoot_xy_full_(i,j);
    for(unsigned j=0; j<nf_ ; ++j)
      qp_J_(i+index,j+N2nf2)=Afoot_theta_full_(i,j);
  }
  index += nc_foot_ ;
  for(unsigned i=0 ; i<nc_rot_ ;++i)
  {
    for (unsigned j=0 ; j<nf_ ; ++j)
    {
      qp_J_(index+i,2*N_+2*nf_+j) = Arot_(i,2*N_+2*nf_+j);
    }
  }
  index += nc_rot_ ;
  for(unsigned obs=0 ; obs<obstacles_.size() ; ++obs)
  {
    for(unsigned n=0 ; n<nf_ ; ++n)
    {

      qp_J_obs_i_ = 2 * U_xy_.transpose()*Hobs_[obs][n]
	+ Aobs_[obs][n];
      for(unsigned j=0 ; j<N2nf2 ; ++j)
        qp_J_((obs+1)*n,j) = qp_J_obs_i_(j) ;
    }
  }
  //  for(unsigned i=0 ; i<nc_stan_ ; ++i)
  //  {
  //    for(unsigned j=0 ; j<nv_ ; ++j)
  //      qp_J_(i+index,j) = qp_J_stan_(i,j);
  //  }
  //  index += nc_stan_ ;

  //  Boundaries
  // compute the constraint value
  evalConstraint(U_);
  qp_ubJ_ = ub_ - gU_ ;

#ifdef DEBUG
  DumpMatrix("qp_J_",qp_J_);
  DumpVector("qp_lbJ_",qp_lbJ_);
  DumpVector("qp_ubJ_",qp_ubJ_);
#endif
  return ;
}


void NMPCgenerator::evalConstraint(Eigen::VectorXd & U)
{
  //
  //  Eval real problem bounds : lb_ < g(U) < ub_
  for(unsigned i=0 ; i<2*(N_+nf_) ; ++i)
  {Uxy_(i)=U(i);}

  //    CoP
  evalCoPconstraint(U);
  gU_cop_ = Acop_xy_*Uxy_;
  //    Foot
  evalFootPoseConstraint(U);
  gU_foot_ = Afoot_xy_full_*Uxy_;
  //    Velocity
  { for(unsigned int i=0;i<gU_vel_.size();gU_vel_[i++]=0.0);};
  //    Rotation
  gU_rot_ = Arot_*U;
  //    Obstacle
  gU_obs_.resize(nc_obs_);
  for(unsigned obs=0 ; obs<obstacles_.size() ; ++obs)
  {
    for(unsigned n=0 ; n<nf_ ; ++n)
    {
      Eigen::VectorXd HobsUxy = Hobs_[obs][n]*Uxy_;
      double deltaObs = 0 ;
      for(unsigned i=0 ; i<HobsUxy.size() ; ++i)
          deltaObs += Uxy_(i) * (HobsUxy(i) + Aobs_[obs][n](i)) ;
      gU_obs_(obs*nf_ + n) = deltaObs ;
    }
  }
  // Standing
  //gU_stan_ = MAL_RET_A_by_B(Astan_,U) ;

  ub_.resize(nc_);
  gU_.resize(nc_);

  // Fill up lb_, ub_ and gU_
  unsigned index = 0 ;
  for(unsigned i=0 ; i<nc_vel_ ; ++i)
  {
    ub_(index+i) = Bvel_ (i);
    gU_(index+i) = gU_vel_(i);
  }
  index += nc_vel_ ;
  for(unsigned i=0 ; i<nc_cop_ ; ++i)
  {
    ub_(index+i) = UBcop_ (i) ;
    gU_(index+i) = gU_cop_(i);
  }
  index += (unsigned int)nc_cop_ ;
  for(unsigned i=0 ; i<nc_foot_ ; ++i)
  {
    ub_(index+i) = UBfoot_full_ (i);
    gU_(index+i) = gU_foot_(i);
  }
  index += nc_foot_ ;
  for(unsigned i=0 ; i<nc_rot_ ; ++i)
  {
    ub_(index+i) = UBrot_ (i);
    gU_(index+i) = gU_rot_(i);
  }
  index += nc_rot_ ;
  for(unsigned obs=0 ; obs<obstacles_.size() ; ++obs)
  {
    for(unsigned n=0 ; n<nf_ ; ++n)
    {
      ub_(index + obs*nf_+n) = UBobs_[obs](n);
      gU_(index + obs*nf_+n) = gU_obs_(obs*nf_+n) ;
    }
  }
  return ;
}

void NMPCgenerator::updateInitialCondition(double time,
    FootAbsolutePosition & currentLeftFootAbsolutePosition,
    FootAbsolutePosition & currentRightFootAbsolutePosition,
    COMState & currentCOMState,
    reference_t & local_vel_ref)
{
  time_ = time ;

  currentLeftFootAbsolutePosition_  = currentLeftFootAbsolutePosition;
  currentRightFootAbsolutePosition_ = currentRightFootAbsolutePosition;

  setLocalVelocityReference(local_vel_ref);
  //setGlobalVelocityReference(local_vel_ref);

  c_k_x_(0) = currentCOMState.x[0] ;
  c_k_x_(1) = currentCOMState.x[1] ;
  c_k_x_(2) = currentCOMState.x[2] ;
  c_k_y_(0) = currentCOMState.y[0] ;
  c_k_y_(1) = currentCOMState.y[1] ;
  c_k_y_(2) = currentCOMState.y[2] ;
#ifdef DEBUG_COUT
  cout << "currentCOMState = " << currentCOMState ;
  cout << "c_k_x_ = " << c_k_x_ << endl;
  cout << "c_k_y_ = " << c_k_y_ << endl;
#endif
#ifdef DEBUG
  DumpVector("c_k_x_", c_k_x_);
  DumpVector("c_k_y_", c_k_y_);
#endif

  c_k_z_ = currentCOMState.z[0] ;

  updateCurrentSupport(time_,
                       currentLeftFootAbsolutePosition,
                       currentRightFootAbsolutePosition);

  if(currentSupport_.StateChanged &&
     desiredNextSupportFootRelativePosition.size()!=0)
  {
    desiredNextSupportFootRelativePosition.pop_front();
  }

#ifdef DEBUG_COUT
  cout << time_ << "  "
       << currentSupport_.StartTime << "  "
       << currentSupport_.TimeLimit << "  "
       << currentSupport_.NbStepsLeft << "  "
       << endl ;
#endif
  if(currentSupport_.TimeLimit>1e+8)
  {
    Tfirst_ = T_ ;
  }
  else if (currentSupport_.StartTime==0.0)
  {
    Tfirst_ = (time_+T_-currentSupport_.TimeLimit)
        - ((double)(int)((time_-currentSupport_.TimeLimit)/T_) * T_) ;
#ifdef DEBUG_COUT
    cout << Tfirst_ << " " ;
#endif
    Tfirst_ = T_ - Tfirst_;
  }else
  {
    Tfirst_ = (time_-currentSupport_.StartTime)
        - ((double)(int)((time_-currentSupport_.StartTime)/T_) * T_) ;
    Tfirst_ = T_ - Tfirst_;
  }
  if(Tfirst_<0.0001)
    Tfirst_=T_;

#ifdef DEBUG_COUT
  cout << time_ << " " ;
  cout << currentSupport_.TimeLimit << " " ;
  cout << currentSupport_.StartTime << " " ;
  cout << Tfirst_ << endl ;
#endif
  updateCoMCoPIntegrationMatrix();

  updateSupportdeque(time_,
                     currentLeftFootAbsolutePosition,
                     currentRightFootAbsolutePosition);
  computeFootSelectionMatrix();
  updateIterationBeforeLanding();
  updateInitialConditionDependentMatrices();
  guessWarmStart();
  return ;
}

void NMPCgenerator::guessWarmStart()
{
  if(Tfirst_==T_ && currentSupport_.StateChanged)
  {
    // we need to plan the last step
    double sign = 1.0 ;
    if(currentSupport_.Foot==LEFT)
      sign = 1.0;
    else
      sign = -1.0;

    if((nf_%2==0))
      sign = -sign ;

    if(nf_==1)
    {
      F_kp1_x_[0] = currentSupport_.X  +
	sign*sin(currentSupport_.Yaw)*1.2*FeetDistance_ ;
      F_kp1_y_[0] = currentSupport_.Y  -
	sign*cos(currentSupport_.Yaw)*1.2*FeetDistance_ ;
    }
    else
    {
      for(unsigned i=1 ; i<nf_ ; ++i)
      {
        F_kp1_x_[i-1] = F_kp1_x_[i] ;
        F_kp1_y_[i-1] = F_kp1_y_[i] ;
        F_kp1_theta_[i-1] = F_kp1_theta_[i] ;
      }
      F_kp1_x_[nf_-1] = F_kp1_x_[nf_-2] +
	sign*sin(F_kp1_theta_[nf_-2])*1.2*FeetDistance_ ;
      F_kp1_y_[nf_-1] = F_kp1_y_[nf_-2] -
	sign*cos(F_kp1_theta_[nf_-2])*1.2*FeetDistance_ ;
    }
    for(unsigned i=1 ; i<N_ ; ++i)
    {
      U_x_(i) = U_x_(i-1);
      U_y_(i) = U_y_(i-1);
    }

    // Fill up U_ again
    for(unsigned i=0 ; i<nf_ ; ++i)
    {
      U_(N_+i)         = F_kp1_x_(i)     ;
      U_(2*N_+nf_+i)   = F_kp1_y_(i)     ;
      U_(2*N_+2*nf_+i) = F_kp1_theta_(i) ;
    }
    for(unsigned i=0 ; i<N_ ; ++i)
    {
      U_(i)        = U_x_(i) ;
      U_(N_+nf_+i) = U_y_(i) ;
    }

    // update U_xy_
    for(unsigned i=0 ; i<2*N_+2*nf_ ; ++i)
      U_xy_(i)=U_(i);
  }
  return ;
}

void NMPCgenerator::updateInitialConditionDependentMatrices()
{
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
  Pzsc_x_ = Pzs_*c_k_x_;
  Pzsc_y_ = Pzs_*c_k_y_;

  for (unsigned i=0 ; i<N_ ; ++i)
  {
    if(SupportStates_deq_[i+1].Phase==DS)
    {
      if(SupportStates_deq_[i+1].Foot==LEFT)
      {
        v_kp1f_x_(i) = v_kp1_(i) *
            (SupportStates_deq_[i+1].X +
	     sin(SupportStates_deq_[i+1].Yaw)*
	     FeetDistance_*0.5);
        v_kp1f_y_(i) = v_kp1_(i) *
            (SupportStates_deq_[i+1].Y -
	     cos(SupportStates_deq_[i+1].Yaw)*
	     FeetDistance_*0.5);
      }else
      {
        v_kp1f_x_(i) = v_kp1_(i) *
            (SupportStates_deq_[i+1].X -
	     sin(SupportStates_deq_[i+1].Yaw)
	     *FeetDistance_*0.5);
        v_kp1f_y_(i) = v_kp1_(i) *
            (SupportStates_deq_[i+1].Y +
	     cos(SupportStates_deq_[i+1].Yaw)
	     *FeetDistance_*0.5);
      }
    }
    else
    {
      v_kp1f_x_(i) = v_kp1_(i) * SupportStates_deq_[i+1].X ;
      v_kp1f_y_(i) = v_kp1_(i) * SupportStates_deq_[i+1].Y ;
    }
  }

  for(unsigned i=0 ; i<N_ ; ++i)
  {
    Pzsc_(i)      = Pzsc_x_(i) ;
    Pzsc_(i+N_)   = Pzsc_y_(i) ;
    v_kp1f_(i)    = v_kp1f_x_(i);
    v_kp1f_(i+N_) = v_kp1f_y_(i);
  }

  return ;
}

void NMPCgenerator::solve()
{
  if(currentSupport_.Phase==DS && currentSupport_.NbStepsLeft == 0)
    return;
  Eigen::internal::set_is_malloc_allowed(false);
  /* Process and solve problem, s.t. pattern generator data is consistent */
  unsigned iter = 0 ;
  oneMoreStep_ = true;
  double normDeltaU = 0.0 ;
  while(iter < maxSolverIteration_ && oneMoreStep_ == true)
  {
    preprocess_solution() ;
    solve_qp()            ;
    postprocess_solution();

    normDeltaU = 0.0 ;
    for(unsigned i=0 ; i<nv_ ; ++i)
      normDeltaU += sqrt(deltaU_[i]*deltaU_[i]);
    //cout << "normDeltaU = " << normDeltaU << endl;

    if(normDeltaU > 1e-5)
      oneMoreStep_=true;
    else
      oneMoreStep_=false;

    ++iter;
  }
#ifdef DEBUG
  static unsigned iteration_solver_file = 0 ;
  if(iteration_solver_file == 0)
  {
    ofstream os;
    os.open("iteration_solver.dat",ios::out);
    ++iteration_solver_file ;
  }
  Eigen::internal::set_is_malloc_allowed(true);
  ofstream os("iteration_solver.dat",ios::app);
  os << time_ << " "
     << iter-1  << " "
     << normDeltaU << endl ;
#endif // DEBUG
#ifdef DEBUG_COUT
  //cout << "solver number of iteration = " << iter << endl ;
#endif // DEBUG_COUT
}

void NMPCgenerator::preprocess_solution()
{
  updateConstraint();
  updateCostFunction();
  QP_->problem((int)nv_,(int)nceq_,(int)ncineq_);
  QuadProg_H_       .resize(nv_,nv_);
  QuadProg_g_       .resize(nv_);
  QuadProg_J_eq_    .resize(nceq_,nv_);
  QuadProg_bJ_eq_   .resize(nceq_);
  QuadProg_J_ineq_  .resize(ncineq_,nv_);
  QuadProg_lbJ_ineq_.resize(ncineq_);
  deltaU_  .resize(nv_);
  deltaU_thresh_.resize(nv_);

  for(unsigned i=0 ; i<nv_ ; ++i)
  {
    for(unsigned j=0 ; j<nv_ ; ++j)
    {
      QuadProg_H_(i,j) = qp_H_(i,j) ;
    }
    QuadProg_g_(i) = qp_g_(i) ;
  }
  for(unsigned i=0 ; i<nceq_ ; ++i)
  {
    for(unsigned j=0 ; j<nv_ ; ++j)
    {
      QuadProg_J_eq_(i,j) = qp_J_(i,j) ;
    }
    QuadProg_bJ_eq_(i) = qp_ubJ_(i) ;
  }

  for(unsigned i=0 ; i<ncineq_ ; ++i)
  {
    for(unsigned j=0 ; j<nv_ ; ++j)
    {
      QuadProg_J_ineq_(i,j) = qp_J_(i+nceq_,j) ;
    }
    QuadProg_lbJ_ineq_(i) = qp_ubJ_(i+nceq_) ;
  }
  return ;
}

void NMPCgenerator::solve_qp()
{
  // primal SQP solution
  QP_->solve(QuadProg_H_,QuadProg_g_,
             QuadProg_J_eq_,QuadProg_bJ_eq_,
             QuadProg_J_ineq_,QuadProg_lbJ_ineq_,false);
  //  if(QP_->fail()==0)
  //    cerr << "qp solveur succeded" << endl ;
  if(QP_->fail()==1)
    {
      cerr << "qp solveur failed : problem has no solution" << endl ;
      if (exit_on_error_)
	exit(-1);
    }
  if(QP_->fail()==2)
    {
      cerr << "qp solveur failed : problems with decomposing H" << endl ;
      if (exit_on_error_)
	exit(-1);
    }

  deltaU_ = QP_->result() ;
  //cout << deltaU_.transpose() << endl ;

#ifdef DEBUG_COUT
  bool endline = false;
  if(*cput_ >= 0.0007)
  {
    cout << *cput_ * 1000 << " " ;
    endline = true;
  }
  if(nwsr_ > 2)
  {
    cout << nwsr_ << " " ;
    endline = true;
  }
  if(*cput_>= 0.001)
  {
    cout << " : warning on cpu time" ;
    endline = true;
  }
  if(endline)
  {
    cout << "----" << endl;
  }
#endif
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
  for(unsigned i=0 ; i<nv_ ; ++i)
    deltaU_thresh_(i) = deltaU_(i);

  lineSearch();

  if(true)
  {
    for(unsigned i=2*N_+2*nf_ ; i<2*N_+3*nf_ ; ++i)
      if(deltaU_[i]*deltaU_[i]<1e-06)
        deltaU_thresh_(i)=0.0;
      else
        deltaU_thresh_(i)=deltaU_[i];
  }

  U_ += lineStep_ * deltaU_thresh_ ;

  for(unsigned i=0 ; i<2*N_+2*nf_ ; ++i)
    U_xy_(i)=U_(i);

  for(unsigned i=0 ; i<N_ ; ++i)
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
#ifdef DEBUG
  DumpVector("U_",U_);
#endif
#ifdef DEBUG_COUT
  cout << "U_ = " << U_ << endl ;
  cout << "delta_F_X = [";
  for(unsigned i=N_ ; i<N_+nf_ ; ++i)
    cout << deltaU_thresh_[i] << " , " ;
  cout << "]" << endl;

  cout << "delta_F_Y = [";
  for(unsigned i=2*N_+nf_ ; i<2*N_+2*nf_ ; ++i)
    cout << deltaU_thresh_[i] << " , " ;
  cout << "]" << endl;

  cout << "delta_F_theta = [";
  for(unsigned i=2*N_+2*nf_ ; i<2*N_+3*nf_ ; ++i)
    cout << deltaU_thresh_[i] << " , " ;
  cout << "]" << endl;
#endif

  return ;
}

void NMPCgenerator::
getSolution
(std::vector<double> & JerkX,
 std::vector<double> & JerkY,
 std::vector<double> & FootStepX,
 std::vector<double> & FootStepY,
 std::vector<double> & FootStepYaw)
{
  for(unsigned i=0 ; i<N_ ; ++i)
  {
    JerkX[i] = U_x_(i);
    JerkY[i] = U_y_(i);
  }
  unsigned nf = 0 ;
  if(currentSupport_.StateChanged  || currentSupport_.Phase == DS)
    nf=nf_-1 ;
  else
    nf=nf_;

  for(unsigned i=0 ; i<nf ; ++i)
  {
    FootStepX  [i] = F_kp1_x_(i)    ;
    FootStepY  [i] = F_kp1_y_(i)    ;
    FootStepYaw[i] = F_kp1_theta_(i);
  }
  double sign = 1.0 ;
  // we need to plan the third step
  if(currentSupport_.Foot==LEFT)
    sign = 1.0;
  else
    sign = -1.0;

  // to be done : step on/around the capture point at the end of the preview
#ifdef DEBUG_COUT
  cout << currentSupport_.NbStepsLeft << " "
        << SupportStates_deq_.back().StepNumber << endl;
#endif //DEBUG_COUT
  if(currentSupport_.NbStepsLeft <= 0 &&
     SupportStates_deq_.back().StepNumber <= 0)
  {
    FootStepX  [0] = currentSupport_.X  + sign*sin(currentSupport_.Yaw)
      *FeetDistance_ ;
    FootStepY  [0] = currentSupport_.Y  - sign*cos(currentSupport_.Yaw)
      *FeetDistance_ ;
    FootStepYaw[0] = currentSupport_.Yaw ;
    for(unsigned i=1 ; i<nf ; ++i)
    {
      sign = -sign ;
      FootStepX  [i] = FootStepX[i-1] + sign*sin(FootStepYaw[i-1])
	*FeetDistance_ ;
      FootStepY  [i] = FootStepY[i-1] - sign*cos(FootStepYaw[i-1])
	*FeetDistance_ ;
      FootStepYaw[i] = FootStepYaw[i-1] ;
    }
  }else
  {
    // warning "if StateChanged" we need to plan the second step
    //    if(currentSupport_.StateChanged)
    //      sign = -sign ;
    double vx = vel_ref_.Global.X ;
    double vy = vel_ref_.Global.Y ;
    double vtheta = vel_ref_.Global.Yaw ;
    double thresh = 0.17 ;
    if(vx > thresh)
      vx = thresh ;
    if(vx < -thresh)
      vx = -thresh ;

    if(vy > thresh)
      vy = thresh ;
    if(vy < -thresh)
      vy = -thresh ;

    if(vtheta > thresh)
      vtheta =  thresh ;
    if(vtheta < -thresh)
      vtheta = -thresh ;

    double fx = FootStepX[nf-1] ;
    double fy = FootStepY[nf-1] ;
    double ftheta = FootStepYaw[nf-1] ;

    FootStepX  [nf] = fx + vx*T_ + sign*sin(ftheta)*FeetDistance_ ;
    FootStepY  [nf] = fy + vy*T_ - sign*cos(ftheta)*FeetDistance_ ;
    FootStepYaw[nf] = ftheta + vtheta*T_ ;
    //    FootStepX  [nf] = FootStepX[nf-1] + vel_ref_.Global.X*T_
    // + sign*sin(FootStepYaw[nf-1])*FeetDistance_ ;
    //    FootStepY  [nf] = FootStepY[nf-1] + vel_ref_.Global.Y*T_
    // - sign*cos(FootStepYaw[nf-1])*FeetDistance_ ;
    //    FootStepYaw[nf] = FootStepYaw[nf-1] + vel_ref_.Global.Yaw*T_ ;
  }
}

void NMPCgenerator::updateCurrentSupport(double time,
    FootAbsolutePosition &FinalLeftFoot,
    FootAbsolutePosition &FinalRightFoot)
{
#ifdef DEBUG_COUT
  cout << "previous support : \n"
       << currentSupport_.Phase        << " "
       << currentSupport_.Foot         << " "
       << currentSupport_.StepNumber   << " "
       << currentSupport_.StateChanged << " "
       << currentSupport_.X   << " "
       << currentSupport_.Y   << " "
       << currentSupport_.Yaw << " "
       << currentSupport_.NbStepsLeft << " "
       << endl ;
#endif
  const FootAbsolutePosition * FAP = NULL;
  reference_t vel = vel_ref_;
  //vel.Local.X=1;
  // DETERMINE CURRENT SUPPORT STATE:
  // --------------------------------
  FSM_->set_support_state( time, 0, currentSupport_, vel );
  if( currentSupport_.StateChanged == true )
  {
    if( currentSupport_.Foot == LEFT )
      FAP = & FinalLeftFoot;
    else
      FAP = & FinalRightFoot;
    currentSupport_.X = FAP->x;
    currentSupport_.Y = FAP->y;
    currentSupport_.Yaw = FAP->theta*M_PI/180.0;
    currentSupport_.StartTime = time;
  }
}

void NMPCgenerator::updateSupportdeque(double time,
    FootAbsolutePosition &FinalLeftFoot,
    FootAbsolutePosition &FinalRightFoot)
{
  SupportStates_deq_[0] = currentSupport_ ;
  const FootAbsolutePosition * FAP = NULL;
  reference_t vel = vel_ref_;
  // PREVIEW SUPPORT STATES:
  // -----------------------
  // initialize the previewed support state before previewing
  support_state_t PreviewedSupport = currentSupport_;
  PreviewedSupport.StepNumber  = 0;
  double currentTime = time - (T_-Tfirst_);
  for( unsigned pi=1 ; pi<=N_ ; pi++ )
  {
    FSM_->set_support_state( currentTime, pi, PreviewedSupport, vel );
    if( PreviewedSupport.StateChanged )
    {
      if( pi == 1 || SupportStates_deq_[pi-1].Phase==DS )//Foot down
      {
        if( PreviewedSupport.Foot == LEFT )
          FAP = & FinalLeftFoot;
        else
          FAP = & FinalRightFoot;
        PreviewedSupport.X = FAP->x;
        PreviewedSupport.Y = FAP->y;
        PreviewedSupport.Yaw = FAP->theta*M_PI/180.0;
      }
      if( /*pi > 1 &&*/ PreviewedSupport.StepNumber > 0 )
      {
        PreviewedSupport.X = 0.0;
        PreviewedSupport.Y = 0.0;
      }
      PreviewedSupport.StartTime = currentTime+pi*T_;
    }
    SupportStates_deq_[pi] = PreviewedSupport ;
  }
#ifdef DEBUG_COUT
  for(unsigned i=0;i<SupportStates_deq_.size();++i)
  {
    cout << SupportStates_deq_[i].Phase        << " "
         << SupportStates_deq_[i].Foot         << " "
         << SupportStates_deq_[i].StepNumber   << " "
         << SupportStates_deq_[i].StateChanged << " "
         << SupportStates_deq_[i].X            << " "
         << SupportStates_deq_[i].Y            << " "
         << SupportStates_deq_[i].Yaw          << " "
         << SupportStates_deq_[i].NbStepsLeft  << " "
         << endl ;
  }
#endif
}

void NMPCgenerator::computeFootSelectionMatrix()
{
  std::deque<support_state_t>::const_iterator SS_it;
  SS_it = SupportStates_deq_.begin();//points at the cur. sup. st.
  ++SS_it;
  v_kp1_.setZero();
  V_kp1_.setZero();

  //double local_time = time_ ;
  for(unsigned i=0;i<N_;++i, ++SS_it)
  {
    if(SS_it->StepNumber>0)
    {
      V_kp1_(i,SS_it->StepNumber-1)=1.0;
    }
    else
    {
      v_kp1_(i) = 1.0 ;
    }
  }
#ifdef DEBUG
  DumpMatrix("V_kp1_",V_kp1_);
  DumpVector("v_kp1_",v_kp1_);
#endif
#ifdef DEBUG_COUT
  cout << "V_kp1_ = " << V_kp1_ << endl ;
  cout << "v_kp1_ = " << v_kp1_ << endl ;
#endif
  return ;
}

void NMPCgenerator::buildCoMCoPIntegrationMatrix()
{
  double T1 = Tfirst_ ;
  double k = 0.0 ;
  double i_j = 0.0 ;
  const double GRAVITY = 9.81;
  double T1kT = 0.0;
  for (unsigned i = 0 ; i < N_ ; ++i)
  {
    k = (double) i ;
    T1kT = T1+k*T_ ;
    Pps_(i,0) = 1.0 ; Pps_(i,1) = T1kT ;
    Pps_(i,2) = (T1kT)*(T1kT)*0.5;

    Pvs_(i,0) = 0.0 ; Pvs_(i,1) = 1.0 ; Pvs_(i,2) = T1kT ;

    Pas_(i,0) = 0.0 ; Pas_(i,1) = 0.0 ; Pas_(i,2) = 1.0 ;

    Pzs_(i,0) = 1.0 ; Pzs_(i,1) = T1kT ;
    Pzs_(i,2) = (T1kT)*(T1kT)*0.5 - c_k_z_/GRAVITY ;

    for (unsigned j = 0 ; j <= i ; ++j)
    {
      i_j = (double)(i-j) ;
      if(j==0)
      {
        Ppu_(i,j) = (T1*T1*T1 + 3*i_j*T_*T1*T1 + 3*i_j*i_j*T_*T_*T1)/6.0 ;
        Pvu_(i,j) = T1*T1*0.5 + i_j*T_*T1 ;
        Pau_(i,j) = T1 ;
        Pzu_(i,j) = (T1*T1*T1 + 3*i_j*T_*T1*T1 + 3*i_j*i_j*T_*T_*T1)/6.0
           - T1*c_k_z_/GRAVITY ;
      }else{
        Ppu_(i,j) = (3.0*i_j*i_j + 3.0*i_j + 1)*T_*T_*T_/6.0 ;
        Pvu_(i,j) = (2.0*i_j+1)*T_*T_*0.5 ;
        Pau_(i,j) = T_ ;
        Pzu_(i,j) = (3.0*i_j*i_j + 3.0*i_j + 1)*T_*T_*T_/6.0
            - T_*c_k_z_/GRAVITY ;
      }
    }
  }
#ifdef DEBUG
  DumpMatrix("Pps_",Pps_);
  DumpMatrix("Pvs_",Pvs_);
  DumpMatrix("Pas_",Pas_);
  DumpMatrix("Ppu_",Ppu_);
  DumpMatrix("Pvu_",Pvu_);
  DumpMatrix("Pau_",Pau_);
  DumpMatrix("Pzs_",Pzs_);
  DumpMatrix("Pzu_",Pzu_);
#endif
  return ;
}

void NMPCgenerator::updateCoMCoPIntegrationMatrix()
{
  double T1 = Tfirst_ ;
  double k = 0.0 ;
  const double GRAVITY = 9.81;
  double T1kT = 0.0;
  for (unsigned i = 0 ; i < N_ ; ++i)
  {
    k = (double) i ;
    T1kT = T1+k*T_ ;
    Pps_(i,1) = T1kT ;
    Pps_(i,2) = (T1kT)*(T1kT)*0.5;
    Pvs_(i,2) = T1kT ;
    Pzs_(i,1) = T1kT ;
    Pzs_(i,2) = (T1kT)*(T1kT)*0.5 - c_k_z_/GRAVITY ;

    Ppu_(i,0) = (T1*T1*T1 + 3*i*T_*T1*T1 + 3*i*i*T_*T_*T1)/6.0 ;
    Pvu_(i,0) = T1*T1*0.5 + i*T_*T1 ;
    Pau_(i,0) = T1 ;
    Pzu_(i,0) = (T1*T1*T1 + 3*i*T_*T1*T1 + 3*i*i*T_*T_*T1)/6.0
       - T1*c_k_z_/GRAVITY ;
  }

  // update the part PzuV depending on Pzu
  for(unsigned i=0 ; i<Pzu_.rows() ; ++i)
  {
    for(unsigned j=0 ; j<Pzu_.cols() ; ++j)
    {
      Pzuv_(i,j)=Pzu_(i,j);
      Pzuv_(i+N_,j+N_+nf_)=Pzu_(i,j);
    }
  }
}


void NMPCgenerator::buildConvexHullSystems()
{
  unsigned nbEdges = 4 ;
  unsigned nbIneq = 4 ;
  hull4_ = convex_hull_t(nbEdges, nbIneq);
  nbEdges = 5 ;
  nbIneq = 5 ;
  hull5_ = convex_hull_t(nbEdges, nbIneq);

  // CoP hulls
  ////////////////////////////////////////////////////////

  // RIGHT FOOT
  dummySupp_.Foot = RIGHT ;
  dummySupp_.Phase = SS ;
  RFI_->set_vertices( hull4_, dummySupp_, INEQ_COP );
  RFI_->compute_linear_system( hull4_, dummySupp_ );
  for(unsigned i = 0 ; i < hull4_.A_vec.size() ; ++i)
  {
    A0rf_(i,0) = hull4_.A_vec[i] ;
    A0rf_(i,1) = hull4_.B_vec[i] ;
    ubB0rf_(i) = hull4_.D_vec[i] ;
  }
  // LEFT FOOT
  dummySupp_.Foot = LEFT ;
  dummySupp_.Phase = SS ;
  RFI_->set_vertices( hull4_, dummySupp_, INEQ_COP );
  RFI_->compute_linear_system( hull4_, dummySupp_ );
  for(unsigned i = 0 ; i < hull4_.A_vec.size() ; ++i)
  {
    A0lf_(i,0) = hull4_.A_vec[i] ;
    A0lf_(i,1) = hull4_.B_vec[i] ;
    ubB0lf_(i) = hull4_.D_vec[i] ;
  }
  // "SWITCHING MASS"/"DOUBLE SUPPORT" HULL
  dummySupp_.Foot = LEFT ; // foot by default, it shouldn't cause any trouble
  dummySupp_.Phase = SS ;
  RFI_->set_vertices( hull4_, dummySupp_, INEQ_COP );
  RFI_->compute_linear_system( hull4_, dummySupp_ );
  for(unsigned i = 0 ; i < hull4_.A_vec.size() ; ++i)
  {
    A0ds_(i,0) = hull4_.A_vec[i] ;
    A0ds_(i,1) = hull4_.B_vec[i] ;
    ubB0ds_(i) = hull4_.D_vec[i] ;
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

  // RIGHT FOOT
  dummySupp_.Foot = RIGHT ;
  dummySupp_.Phase = SS ;
  hull5_.X_vec[0] = -0.28 ; hull5_.Y_vec[0] = -0.162 ; // -0.2
  hull5_.X_vec[1] = -0.2  ; hull5_.Y_vec[1] = -0.3 ;   // -0.3
  hull5_.X_vec[2] =  0.0  ; hull5_.Y_vec[2] = -0.4 ;   // -0.4
  hull5_.X_vec[3] =  0.2  ; hull5_.Y_vec[3] = -0.3 ;   // -0.3
  hull5_.X_vec[4] =  0.28 ; hull5_.Y_vec[4] = -0.162 ; // -0.162
  RFI_->compute_linear_system( hull5_, dummySupp_ );
  for(unsigned i = 0 ; i < hull5_.A_vec.size() ; ++i)
  {
    A0r_(i,0) = hull5_.A_vec[i] ;
    A0r_(i,1) = hull5_.B_vec[i] ;
    ubB0r_(i) = hull5_.D_vec[i] ;
  }
  // LEFT FOOT
  dummySupp_.Foot = LEFT ;
  dummySupp_.Phase = SS ;
  hull5_.X_vec[0] = -0.28 ; hull5_.Y_vec[0] = 0.162 ; // 0.2
  hull5_.X_vec[1] = -0.2  ; hull5_.Y_vec[1] = 0.3 ; // 0.3
  hull5_.X_vec[2] =  0.0  ; hull5_.Y_vec[2] = 0.4 ; // 0.4
  hull5_.X_vec[3] =  0.2  ; hull5_.Y_vec[3] = 0.3 ; // 0.3
  hull5_.X_vec[4] =  0.28 ; hull5_.Y_vec[4] = 0.162 ; // 0.2
  RFI_->compute_linear_system( hull5_, dummySupp_ );
  for(unsigned i = 0 ; i < hull5_.A_vec.size() ; ++i)
  {
    A0l_(i,0) = hull5_.A_vec[i] ;
    A0l_(i,1) = hull5_.B_vec[i] ;
    ubB0l_(i) = hull5_.D_vec[i] ;
  }
#ifdef DEBUG
  DumpMatrix("A0r_",A0r_);
  DumpMatrix("A0l_",A0l_);
  DumpVector("ubB0r_",ubB0r_);
  DumpVector("ubB0l_",ubB0l_);
#endif

  Eigen::Matrix<double,5,2> tmp52d; ;
  tmp52d.setZero();

  Eigen::Matrix<double,5,1> tmp5d ;
  tmp5d.setZero();

  A0f_xy_.resize(nf_,tmp52d);
  A0f_theta_.resize(nf_,tmp52d);
  B0f_.resize(nf_,tmp5d);

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
  nc_cop_ = N_*ubB0rf_.size() ;

  Acop_xy_.resize(nc_cop_,2*(N_+nf_));
  Acop_theta_.resize(nc_cop_,nf_);
  UBcop_.resize(nc_cop_);
  D_kp1_xy_.resize(nc_cop_,2*N_);
  D_kp1_theta_.resize(nc_cop_,2*N_);
  b_kp1_.resize(nc_cop_);
  derv_Acop_map_.resize(nc_cop_,N_);

  Acop_xy_.setZero();
  Acop_theta_.setZero();
  UBcop_.setZero();
  D_kp1_xy_.setZero();
  D_kp1_theta_.setZero();
  b_kp1_.setZero();
  Pzuv_.setZero();
  Pzsc_.setZero();
  Pzsc_x_.setZero();
  Pzsc_y_.setZero();
  v_kp1f_.setZero();
  v_kp1f_x_.setZero();
  v_kp1f_y_.setZero();
  derv_Acop_map_.setZero();

  // mapping matrix to compute the gradient_theta of the CoP Constraint Jacobian
  for(unsigned j=0 , k=0; j<N_ ; ++j, k+= (unsigned int)A0rf_.rows())
    for(unsigned i=0 ; i<A0rf_.rows() ; ++i )
      derv_Acop_map_(i+k,j) = 1.0 ;
#ifdef DEBUG
  DumpMatrix("derv_Acop_map_",derv_Acop_map_);
  DumpMatrix("Pzu_",Pzu_);
#endif
  return ;
}

void NMPCgenerator::updateCoPconstraint(Eigen::VectorXd &U)
{
  if(nc_cop_==0)
    return ;

  // buils Acop_xy_ + UBcop_
  evalCoPconstraint(U);

  // build Acop_theta_
  vector<double>theta_vec(nf_+1);
  theta_vec[0]=SupportStates_deq_[1].Yaw;
  for(unsigned i=0 ; i<nf_ ; ++i)
  {
    theta_vec[i+1]=U(2*N_+2*nf_+i); //F_kp1_theta_(i);
  }
  Eigen::MatrixXd U_xy(2*N_+2*nf_,1);
  for(unsigned i=0; i<U_xy.size() ; ++i)
    U_xy(i)=U(i);
  // every time instant in the pattern generator constraints
  // depend on the support order
  for (unsigned i=0 ; i<N_ ; ++i)
  {
    double theta = theta_vec[SupportStates_deq_[i+1].StepNumber] ;
    rotMat_theta_(0,0)=-sin(theta) ; rotMat_theta_(0,1)= cos(theta) ;
    rotMat_theta_(1,0)=-cos(theta) ; rotMat_theta_(1,1)=-sin(theta) ;
    if (SupportStates_deq_[i+1].Phase == DS)
    {
      A0_theta_ = A0ds_*rotMat_theta_ ;
    }
    else if (SupportStates_deq_[i+1].Foot == LEFT)
    {
      A0_theta_ = A0lf_*rotMat_theta_ ;
    }else{
      A0_theta_ = A0rf_*rotMat_theta_ ;
    }

    for (unsigned k=0 ; k<A0_theta_.rows() ; ++k)
    {
      // get d_i+1^x(f^'dtheta/dt')
      D_kp1_theta_(i*A0_theta_.rows()+k, i) = A0_theta_(k,0);
      // get d_i+1^y(f^'dtheta/dt')
      D_kp1_theta_(i*A0_theta_.rows()+k, i+N_) = A0_theta_(k,1);
    }
  }
  derv_Acop_map2_ = derv_Acop_map_*V_kp1_;
  Acop_theta_dummy0_ = D_kp1_theta_*Pzuv_;
  Acop_theta_dummy1_ = Acop_theta_dummy0_*U_xy;
  for(unsigned i=0 ; i<Acop_theta_.rows() ; ++i)
  {
    for(unsigned j=0 ; j<Acop_theta_.cols() ; ++j)
    {
      Acop_theta_(i,j) = derv_Acop_map2_(i,j) * Acop_theta_dummy1_(i); // warning this is the real jacobian
      //Acop_theta_(i,j) = 0.0 ; // WARNING this is not the real jacobian !
    }
  }
  return ;
}

void NMPCgenerator::evalCoPconstraint(Eigen::VectorXd & U)
{
  if(nc_cop_==0)
    return ;

  // Compute D_kp1_, it depends on the feet hulls
  vector<double>theta_vec(nf_+1);
  theta_vec[0]=SupportStates_deq_[1].Yaw;
  for(unsigned i=0 ; i<nf_ ; ++i)
  {
    theta_vec[i+1]=U(2*N_+2*nf_+i); //F_kp1_theta_(i);
  }
  // every time instant in the pattern generator constraints
  // depend on the support order
  for (unsigned i=0 ; i<N_ ; ++i)
  {
    double theta = theta_vec[SupportStates_deq_[i+1].StepNumber] ;
    rotMat_xy_(0,0)= cos(theta) ; rotMat_xy_(0,1)= sin(theta) ;
    rotMat_xy_(1,0)=-sin(theta) ; rotMat_xy_(1,1)= cos(theta) ;
    if (SupportStates_deq_[i+1].Phase == DS)
    {
      A0_xy_    = A0ds_*rotMat_xy_ ;
      B0_ = ubB0ds_ ;
    }
    else if (SupportStates_deq_[i+1].Foot == LEFT)
    {
      A0_xy_    = A0lf_*rotMat_xy_ ;
      B0_ = ubB0lf_ ;
    }else{
      A0_xy_    = A0rf_*rotMat_xy_ ;
      B0_ = ubB0rf_ ;
    }
    for (unsigned k=0 ; k<A0_xy_.rows() ; ++k)
    {
      // get d_i+1^x(f^theta)
      D_kp1_xy_(i*A0_xy_.rows()+k, i) = A0_xy_(k,0);
      // get d_i+1^y(f^theta)
      D_kp1_xy_(i*A0_xy_.rows()+k, i+N_) = A0_xy_(k,1);

      // get right hand side of equation
      b_kp1_(i*A0_xy_.rows()+k) = B0_(k) ;
    }
  }

  // build Acop_xy_
  Acop_xy_ = D_kp1_xy_*Pzuv_;
  // build UBcop_
  UBcop_ = b_kp1_ + D_kp1_xy_*(v_kp1f_-Pzsc_);

#ifdef DEBUG
  DumpMatrix("Pzuv_",Pzuv_);
  DumpMatrix("D_kp1_xy_",D_kp1_xy_);
  DumpVector("b_kp1_",b_kp1_);
  DumpMatrix("D_kp1_theta_",D_kp1_theta_);
  DumpMatrix("Acop_xy_",Acop_xy_);
  DumpMatrix("Acop_theta_",Acop_theta_);
  DumpVector("UBcop_",UBcop_);
  DumpVector("v_kp1f_",v_kp1f_);
  DumpVector("Pzsc_",Pzsc_);
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
  # A0 R(Ftheta_k+1) [Fx_k+1 - Fx_k] <= ubB0
  #                  [Fy_k+1 - Fy_k]
  */
  n_vertices_ = (unsigned int )ubB0r_.size() ;
  nc_foot_ = nf_*n_vertices_ ;
  Afoot_xy_   .resize(nf_);
  Afoot_theta_.resize(nf_);
  UBfoot_     .resize(nf_);
  SelecMat_   .resize(nf_);
  AdRdF_      .resize(nf_);
  deltaF_     .resize(nf_);
  for(unsigned i=0 ; i< nf_ ; ++i)
  {
    Afoot_xy_   [i].resize(n_vertices_,2*(N_+nf_));
    Afoot_theta_[i].resize(n_vertices_,nf_);
    UBfoot_     [i].resize(n_vertices_);
    SelecMat_   [i].resize(2,2*(N_+nf_));
    AdRdF_      [i].resize(n_vertices_);
    deltaF_     [i].resize(2);//2 as [deltaFx,deltaFy]
  }
  rotMat_vec_ .resize(nf_, tmpRotMat_ );
  drotMat_vec_.resize(nf_, tmpRotMat_ );
  Afoot_xy_full_.resize(nc_foot_,2*(N_+nf_));
  Afoot_theta_full_.resize(nc_foot_,nf_);
  UBfoot_full_.resize(nc_foot_);

  for(unsigned n=0 ; n < nf_ ; ++n)
  {
    Afoot_xy_   [n].setZero();
    Afoot_theta_[n].setZero();
    UBfoot_     [n].setZero();
    SelecMat_   [n].setZero();
    deltaF_     [n].setZero();
    SelecMat_[n](0,N_+n) = 1 ;
    SelecMat_[n](1,2*N_+nf_+n) = 1 ;
    if(n>0)
    {
      SelecMat_[n](0,N_+n-1) = -1 ;
      SelecMat_[n](1,2*N_+nf_+n-1) = -1 ;
    }
  }
  Afoot_xy_full_.setZero();
  Afoot_theta_full_.setZero();
  UBfoot_full_.setZero();
  return ;
}

void NMPCgenerator::updateFootPoseConstraint(Eigen::VectorXd &U)
{
//  if(nc_foot_==0)
//    return ;

  int ignoreFirstStep = 0 ;
  if(isFootCloseToLand())
    ignoreFirstStep=nf_;

  nc_foot_ = (nf_-ignoreFirstStep)*n_vertices_ ;

//  if( desiredNextSupportFootRelativePosition.size()!=0 )
//  {
//    unsigned nbFoot = std::min(desiredNextSupportFootRelativePosition.size(),nf_);
//    ignoreFirstStep = nbFoot ;
//  }

  Afoot_xy_full_.resize(nc_foot_,2*(N_+nf_));
  Afoot_theta_full_.resize(nc_foot_,nf_);
  UBfoot_full_.resize(nc_foot_);
  Afoot_xy_full_.setZero();
  Afoot_theta_full_.setZero();
  UBfoot_full_.setZero();

  // compute Afoot_xy_full_, UBfoot_full_
  evalFootPoseConstraint(U);

  // compute Afoot_theta_full_
  // rotation matrice from F_k+1 to F_k
  vector<support_state_t>support_state(nf_);
  support_state[0]=SupportStates_deq_[1];
  bool done = false ;
  for(unsigned i=0, n=1; n<nf_&&i<N_ ; ++i)
  {
    if(support_state[n-1].Foot != SupportStates_deq_[i+1].Foot)
    {
      support_state[n]=SupportStates_deq_[i+1];
      if(support_state[n].StepNumber!=0)
      {
        support_state[n].Yaw = U(2*N_+2*nf_+n-1); //F_kp1_theta_(n-1);
      }
      ++n;
      done = true ;
    }
  }
  if(!done)
  {
    for(unsigned n=1;n<support_state.size();++n)
    {
      support_state[n]=support_state[0];
    }
  }

  for(unsigned n=ignoreFirstStep ; n<nf_ ; ++n)
  {
    drotMat_vec_[n](0,0)=-sin(support_state[n].Yaw) ;
    drotMat_vec_[n](0,1)= cos(support_state[n].Yaw) ;
    drotMat_vec_[n](1,0)=-cos(support_state[n].Yaw) ;
    drotMat_vec_[n](1,1)=-sin(support_state[n].Yaw) ;

    if (support_state[n].Foot == LEFT)
    {
      A0f_theta_[n] = A0r_*drotMat_vec_[n] ;
    }else{
      A0f_theta_[n] = A0l_*drotMat_vec_[n] ;
    }
    if(n!=0)
    {
      deltaF_[n](0)=U(N_+n)-U(N_+n-1) ;//F_kp1_x_[n]-F_kp1_x_[n-1];
      deltaF_[n](1)=U(2*N_+nf_+n)-U(2*N_+nf_+n-1) ;//F_kp1_y_[n]-F_kp1_y_[n-1];
      AdRdF_[n] = A0f_theta_[n]*deltaF_[n];
      double sum = 0.0 ;
      for (unsigned j=0 ; j<n_vertices_ ; ++j)
      {
        sum += AdRdF_[n](j);
      }
      for (unsigned j=0 ; j<n_vertices_ ; ++j)
      {
        //Afoot_theta_[n](j,n-1) = sum;
        Afoot_theta_[n](j,n-1) = AdRdF_[n](j); // This is the real jacobian!
        //Afoot_theta_[n](j,n-1) = 0.0; // WARNING this is not the real jacobian!
      }
      for(unsigned i=0 ; i<n_vertices_ ;++i)
        for(unsigned j=0 ; j<nf_ ; ++j)
          Afoot_theta_full_((n-ignoreFirstStep)*n_vertices_+i,j)
              = Afoot_theta_[n](i,j);

    }

#ifdef DEBUG
  ostringstream os ("") ;
  os << "Afoot_xy_" << n << "_" ;
  DumpMatrix(os.str() ,Afoot_xy_   [n]);
  os.str("");
  os << "Afoot_theta_" << n << "_" ;
  DumpMatrix(os.str() ,Afoot_theta_[n]);
#endif // DEBUG
  }
#ifdef DEBUG_COUT
  cout << Afoot_xy_full_ << endl ;
  cout << Afoot_theta_full_ << endl ;
#endif // DEBUG_COUT
  return ;
}

void NMPCgenerator::evalFootPoseConstraint(Eigen::VectorXd & U)
{
  if(nc_foot_==0)
    return ;

  int ignoreFirstStep = 0 ;
  if(isFootCloseToLand())
    ignoreFirstStep=1;

  // rotation matrice from F_k+1 to F_k
  vector<support_state_t>support_state(nf_);
  support_state[0]=SupportStates_deq_[1];
  bool done = false ;
  for(unsigned i=0, n=1; n<nf_&&i<N_ ; ++i)
  {
    if(support_state[n-1].Foot != SupportStates_deq_[i+1].Foot)
    {
      support_state[n]=SupportStates_deq_[i+1];
      if(support_state[n].StepNumber!=0)
      {
        support_state[n].Yaw = U(2*N_+2*nf_+n-1); //F_kp1_theta_(n-1);
      }
      ++n;
      done = true ;
    }
  }
  if(!done)
  {
    for(unsigned n=1;n<support_state.size();++n)
    {
      support_state[n]=support_state[0];
    }
  }

  for(unsigned n=ignoreFirstStep ; n<nf_ ; ++n)
  {
    rotMat_vec_[n](0,0)= cos(support_state[n].Yaw) ;
    rotMat_vec_[n](0,1)= sin(support_state[n].Yaw) ;
    rotMat_vec_[n](1,0)=-sin(support_state[n].Yaw) ;
    rotMat_vec_[n](1,1)= cos(support_state[n].Yaw) ;

    if (support_state[n].Foot == LEFT)
    {
      A0f_xy_   [n] = A0r_*rotMat_vec_ [n] ;
      B0f_      [n] = ubB0r_ ;
    }else{
      A0f_xy_   [n] = A0l_*rotMat_vec_ [n] ;
      B0f_      [n] = ubB0l_ ;
    }
    Afoot_xy_   [n].setZero();
    Afoot_xy_[n] = A0f_xy_[n]*SelecMat_[n];
    for(unsigned i=0 ; i<n_vertices_ ;++i)
      for(unsigned j=0 ; j<2*(N_+nf_) ; ++j)
        Afoot_xy_full_((n-ignoreFirstStep)*n_vertices_+i,j)
            = Afoot_xy_[n](i,j);

    { for(unsigned int i=0;i<UBfoot_[n].size();UBfoot_[n][i++]=0.0);};
    UBfoot_[n]=B0f_[n];
    if(n==0)
    {
      for(unsigned i=0 ; i<n_vertices_ ; ++i)
      {
        UBfoot_[0](i) += A0f_xy_[0](i,0)*currentSupport_.X +
                         A0f_xy_[0](i,1)*currentSupport_.Y ;
      }
    }
    for(unsigned i=0 ; i<n_vertices_ ;++i)
      UBfoot_full_((n-ignoreFirstStep)*n_vertices_+i) = UBfoot_[n](i);
  }
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
  nc_vel_=3*nf_;
  Avel_.resize(nc_vel_,2*N_+3*nf_)  ;
  Bvel_.resize(nc_vel_)  ;
  gU_vel_.resize(nc_vel_);

  Avel_.setZero();
  Bvel_.setZero();

  for(unsigned i=0 ; i<nf_ ; ++i)
  {
    Avel_(0+i*3,   N_+i)       =  1.0 ;
    Avel_(1+i*3, 2*N_+nf_+i)   =  1.0 ;
    Avel_(2+i*3, 2*N_+2*nf_+i) =  1.0 ;
  }
  nc_vel_=0;
  useItBeforeLanding_=true;
  return ;
}

void NMPCgenerator::updateFootVelIneqConstraint()
{
  if( isFootCloseToLand() )
  {
    nc_vel_=3*nf_;
  }
  else
  {
    nc_vel_=0;
  }

#ifdef DEBUG
  DumpMatrix("Avel_",Avel_);
  DumpVector("UBvel_",UBvel_);
#endif
#ifdef DEBUG_COUT
  cout << "Avel_ = " << Avel_  << endl ;
  cout << "Bvel_ = " << Bvel_ << endl ;
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
  nc_rot_ = 2*nf_ ;
  Arot_.resize(nc_rot_,2*N_+3*nf_);
  UBrot_.resize(nc_rot_);
  LBrot_.resize(nc_rot_);

  Arot_.setZero();
  UBrot_.setZero();
  LBrot_.setZero();

  for(unsigned i=0 ; i<nf_ ;++i)
  {
    for (unsigned j=0 ; j<nf_ ; ++j)
    {
      if(i==j)
        Arot_(i,2*N_+2*nf_+j) =  1.0 ;
      if((i-1)==j)
        Arot_(i,2*N_+2*nf_+j) = -1.0 ;
    }
    UBrot_(i) =  0.17 ;
  }

  for(unsigned i=nf_ ; i<2*nf_ ;++i)
  {
    for (unsigned j=0 ; j<nf_ ; ++j)
    {
      if((i-nf_)==j)
        Arot_(i,2*N_+2*nf_+j) = -1.0 ;
      if((i-nf_-1)==j)
        Arot_(i,2*N_+2*nf_+j) =  1.0 ;
    }
    UBrot_(i) =  0.17 ;
  }
}

void NMPCgenerator::updateRotIneqConstraint()
{
  if(nc_rot_==0)
    return ;

  UBrot_(0) =  0.17 + currentSupport_.Yaw ;
  UBrot_(nf_) =  0.17 - currentSupport_.Yaw ;
  LBrot_(0) = -0.17 + currentSupport_.Yaw ;
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
  Hobs_.clear();
  Aobs_.clear();
  nc_obs_ = (unsigned int)obstacles_.size();

  //  Circle obstacle ;
  //obstacle.x_0    = 1.0 ;
  //obstacle.y_0    = 0.5 ;
  //obstacle.r      = 0.23 ;
  //obstacle.margin = 0.40 ;
  //  obstacle.x_0    = 1.5 ;
  //  obstacle.y_0    = -1.5 ;
  //  obstacle.r      = 0.23 ;
  //  obstacle.margin = 0.40 ;
  //  obstacles_.push_back(obstacle);

}

void NMPCgenerator::updateObstacleConstraint()
{
  nc_obs_ = (unsigned int)(nf_*obstacles_.size()) ;
  Eigen::MatrixXd H(2*(N_+nf_),2*(N_+nf_));
  Eigen::VectorXd A(2*(N_+nf_));
  Eigen::VectorXd B(nc_obs_);

  H.setZero();
  A.setZero();
  B.setZero();

  Hobs_.resize(obstacles_.size() , std::vector<Eigen::MatrixXd>(nc_obs_,H) );
  Aobs_.resize(obstacles_.size() , std::vector<Eigen::VectorXd>(nc_obs_,A) );
  UBobs_.resize(obstacles_.size() , B );

  unsigned nc = nf_ ;
  for(unsigned obs=0 ; obs<obstacles_.size() ; ++obs)
  {
    for (unsigned i=0 ; i<nc ; ++i)
    {
      Hobs_ [obs][i](N_+i,N_+i)             = -1.0 ;
      Hobs_ [obs][i](2*N_+nf_+i,2*N_+nf_+i) = -1.0 ;
      Aobs_ [obs][i](N_+i)       = +2*obstacles_[obs].x_0 ;
      Aobs_ [obs][i](2*N_+nf_+i) = +2*obstacles_[obs].y_0 ;
      UBobs_[obs](i) = -(-  obstacles_[obs].x_0*obstacles_[obs].x_0
                         -  obstacles_[obs].y_0*obstacles_[obs].y_0
                         + (obstacles_[obs].r+obstacles_[obs].margin)
                         * (obstacles_[obs].r+obstacles_[obs].margin) ) ;
    }
#ifdef DEBUG_COUT
    cout << "prepare the obstacle : " << obstacles_[obs] << endl ;
#endif
  }
#ifdef DEBUG
//  DumpMatrix("Hobs_0" ,Hobs_ [0][0]);
//  DumpVector("Aobs_0" ,Aobs_ [0][0]);
//  DumpMatrix("Hobs_1" ,Hobs_ [0][1]);
//  DumpVector("Aobs_1" ,Aobs_ [0][1]);
//  DumpVector("LBobs_",LBobs_[0]);
#endif

  return ;
}

void NMPCgenerator::initializeStandingConstraint()
{
  // constraint on the foot position we force them to be next to each other
  nc_stan_ = 3*nf_ ; // 3 because we constraint x,y and theta
  Astan_.resize(nc_stan_,nv_);
  Astan_.setZero();
  UBstan_.resize(nc_stan_);
  UBstan_.setZero();
  LBstan_.resize(nc_stan_);
  LBstan_.setZero();

  for(unsigned i=0 ; i<nf_ ; ++i)
  {
    Astan_ (i*3  ,N_+i)         = 1.0 ;
    Astan_ (i*3+1,2*N_+nf_+i)   = 1.0 ;
    Astan_ (i*3+2,2*N_+2*nf_+i) = 1.0 ;
  }
  nc_stan_=0;
  return ;
}

void NMPCgenerator::updateStandingConstraint()
{
  // this contraint is valid only if the robot is standing still
  if(/*currentSupport_.Phase==SS*/true)
  {
    nc_stan_=0;
    return;
  }else
  {
    nc_stan_ = 3*nf_ ;
  }
  double oddPos_x(0.0), oddPos_y(0.0), oddPos_theta(0.0) ;
  double evenPos_x(0.0), evenPos_y(0.0), evenPos_theta(0.0) ;
  if(currentSupport_.Foot==LEFT)
  {
    evenPos_x     = currentSupport_.X +
      sin(currentSupport_.Yaw)*RFI_->DSFeetDistance() ;
    evenPos_y     = currentSupport_.Y -
      cos(currentSupport_.Yaw)*RFI_->DSFeetDistance() ;
    evenPos_theta = currentSupport_.Yaw ;
  }else
  {
    evenPos_x     = currentSupport_.X -
      sin(currentSupport_.Yaw)*RFI_->DSFeetDistance() ;
    evenPos_y     = currentSupport_.Y +
      cos(currentSupport_.Yaw)*RFI_->DSFeetDistance() ;
    evenPos_theta = currentSupport_.Yaw ;
  }
  oddPos_x     = currentSupport_.X ;
  oddPos_y     = currentSupport_.Y ;
  oddPos_theta = currentSupport_.Yaw ;
  for(unsigned i=0 ; i<nf_ ; ++i)
  {
    if(i%2==0)
    {
      UBstan_(i*3  )     = evenPos_x     ;
      UBstan_(i*3+1)     = evenPos_y     ;
      UBstan_(i*3+2)     = evenPos_theta ;
      LBstan_(i*3  )     = evenPos_x     ;
      LBstan_(i*3+1)     = evenPos_y     ;
      LBstan_(i*3+2)     = evenPos_theta ;
    }else
    {
      UBstan_(i*3  )     = oddPos_x     ;
      UBstan_(i*3+1)     = oddPos_y     ;
      UBstan_(i*3+2)     = oddPos_theta ;
      LBstan_(i*3  )     = oddPos_x     ;
      LBstan_(i*3+1)     = oddPos_y     ;
      LBstan_(i*3+2)     = oddPos_theta ;
    }
  }
  return ;
}

void NMPCgenerator::initializeCostFunction()
{
  // number of constraint
  nc_ = (unsigned int)(nc_cop_+nc_foot_+nc_vel_+nc_rot_+nc_obs_+nc_stan_ );

  qp_H_.resize(nv_,nv_);    qp_H_.setIdentity();
  qp_g_.resize(nv_); qp_g_.setZero();
  qp_g_x_.resize(N_+nf_); qp_g_x_.setZero();
  qp_g_y_.resize(N_+nf_); qp_g_y_.setZero();
  qp_g_theta_.resize(nf_); qp_g_theta_.setZero();
  Q_x_XX_.resize(N_,N_); Q_x_XX_.setZero();
  Q_x_XF_.resize(N_,nf_); Q_x_XF_.setZero();
  Q_x_FX_.resize(nf_,N_); Q_x_FX_.setZero();
  Q_x_FF_.resize(nf_,nf_); Q_x_FF_.setZero();
  Q_y_XX_.resize(N_,N_); Q_y_XX_.setZero();
  Q_theta_.resize(nf_,nf_);   Q_theta_.setIdentity();
  p_xy_X_.resize(N_); p_xy_X_.setZero();
  p_xy_Fx_.resize(nf_); p_xy_Fx_.setZero();
  p_xy_Y_.resize(N_); p_xy_Y_.setZero();
  p_xy_Fy_.resize(nf_); p_xy_Fy_.setZero();
  p_.resize(nv_); p_.setZero();
  I_NN_.resize(N_,N_);     I_NN_.setIdentity();
  I_FF_.resize(nf_,nf_);   I_FF_.setZero();
  I_FF_(nf_-1,nf_-1)=1.0;
  I_FF_.setIdentity();
  Pvsc_x_.resize(N_);
  Pvsc_x_.setZero();
  Pvsc_y_.resize(N_);
  Pvsc_y_.setZero();
  Pvsc_y_.resize(N_);
  Pvsc_y_.setZero();
  Pvsc_y_.resize(N_);
  Pvsc_y_.setZero();
  v_kf_x_.resize(nf_);
  v_kf_x_.setZero();
  v_kf_y_.resize(nf_);
  v_kf_y_.setZero();
  diffMat_.resize(nf_,nf_);      diffMat_.setIdentity();
  for(unsigned i=0 ; i<nf_ ; ++i)
    for(unsigned j=0 ; j<nf_ ; ++j)
      if((i-j)==1)
        diffMat_(i,j)=-1;
#ifdef DEBUG_COUT
  cout << diffMat_ << endl ;
#endif

  // Q_q = 0.5 * a * ( 1 0 )
  //                 ( 0 1 )
  Q_theta_ *= (alpha_theta_) ;

  // Q_x_XF_, Q_x_FX_, Q_x_FF_ are time dependant matrices so they
  // are computed in the update function

  // p_xy_ =  ( p_xy_X_, p_xy_Fx_, p_xy_Y_, p_xy_Fy_ )
  // p_theta_ = ( 0.5 * a * (-2) * [ f_k_theta+T_step*dTheta^ref  f_k_theta+2*T_step*dTheta^ref ] )
  // Those are time dependant matrices so they are computed in the update function
}

void NMPCgenerator::updateCostFunction()
{
  // Q_xXX = (  0.5 * a * Pvu^T   * Pvu + b * Pzu^T * Pzu + c * I )
  // Q_xXF = ( -0.5 * b * Pzu^T   * V_kp1 )
  // Q_xFX = ( -0.5 * b * V_kp1^T * Pzu )^T
  // Q_xFF = (  0.5 * b * V_kp1^T * V_kp1 )
  Q_x_XX_ = alpha_x_ * Pvu_.transpose() * Pvu_
          + beta_    * Pzu_.transpose() * Pzu_
          + minjerk_ * I_NN_ ;

  // Q_xXX = (  0.5 * a * Pvu^T   * Pvu + b * Pzu^T * Pzu + c * I )
  // Q_xXF = ( -0.5 * b * Pzu^T   * V_kp1 )
  // Q_xFX = ( -0.5 * b * V_kp1^T * Pzu ) = Q_xXF^T
  // Q_xFF = (  0.5 * b * V_kp1^T * V_kp1 - 0.5 * d * I_FF_)
  Q_x_XF_ = - beta_ * Pzu_.transpose() *V_kp1_;
  Q_x_FX_ =   Q_x_XF_.transpose();
  Q_x_FF_ =   beta_ * V_kp1_.transpose() *V_kp1_
            + delta_ * I_FF_
            + kappa_ * diffMat_.transpose() * diffMat_;

  // Q_yXX = (  0.5 * a * Pvu^T   * Pvu + b * Pzu^T * Pzu + c * I )
  Q_y_XX_ = alpha_y_ * Pvu_.transpose() *Pvu_
          + beta_    * Pzu_.transpose() *Pzu_
          + minjerk_   * I_NN_ ;


  // define QP matrices
  // Gauss-Newton Hessian
  //                                     dim :
  // H = (( Q_xXX  Q_xXF   0      0       0     ) N_
  //      ( Q_xFX  Q_xFF   0      0       0     ) nf_
  //      (   0      0   Q_yXX  Q_xXF     0     ) N_
  //      (   0      0   Q_xFX  Q_xFF     0     ) nf_
  //      (   0      0     0      0    Q_theta_ ) nf_
  //dim :     N_     nf_   N_     nf_    nf_     = nv_
  //
  unsigned Nnf = N_+nf_ ;
  unsigned N2nf = 2*N_+nf_ ;
  unsigned N2nf2 = 2*(N_+nf_) ;
  qp_H_.setZero();
  for(unsigned i=0 ; i<N_ ; ++i)
  {
    for(unsigned j=0 ; j<N_ ; ++j)
    {
      qp_H_(i,j) = Q_x_XX_(i,j) ;
      qp_H_(Nnf+i,Nnf+j) = Q_y_XX_(i,j) ;
    }
  }
  for(unsigned i=0 ; i<N_ ; ++i)
  {
    for(unsigned j=0 ; j<nf_ ; ++j)
    {
      qp_H_(i,N_+j) = Q_x_XF_(i,j) ;
      qp_H_(Nnf+i,N2nf+j) = Q_x_XF_(i,j) ;
    }
  }
  for(unsigned i=0 ; i<nf_ ; ++i)
  {
    for(unsigned j=0 ; j<N_ ; ++j)
    {
      qp_H_(N_+i,j) = Q_x_FX_(i,j) ;
      qp_H_(N2nf+i,Nnf+j) = Q_x_FX_(i,j) ;
    }
  }
  for(unsigned i=0 ; i<nf_ ; ++i)
  {
    for(unsigned j=0 ; j<nf_ ; ++j)
    {
      qp_H_(N_+i,N_+j) = Q_x_FF_(i,j) ;
      qp_H_(N2nf+i,N2nf+j) = Q_x_FF_(i,j) ;
    }
  }
  for(unsigned i=0 ; i< nf_ ;++i)
    for(unsigned j=0 ; j<nf_ ;++j)
      qp_H_(i+N2nf2,j+N2nf2)=Q_theta_(i,j) ;

  // p_xy_ =  ( p_xy_X_, p_xy_Fx_, p_xy_Y_, p_xy_Fy_ )
  // p_xy_X  =   0.5 * a * Pvu^T   * ( Pvs * c_k_x - dX^ref )
  //           + 0.5 * b * Pzu^T   * ( Pzs * c_k_x - v_kp1 * f_k_x )
  // p_xy_Fx = - 0.5 * b * V_kp1^T * ( Pzs * c_k_x - v_kp1 * f_k_x )
  // p_xy_Y  =   0.5 * a * Pvu^T   * ( Pvs * c_k_y - dY^ref )
  //           + 0.5 * b * Pzu^T   * ( Pzs * c_k_y - v_kp1 * f_k_y )
  // p_xy_Fy = - 0.5 * b * V_kp1^T * ( Pzs * c_k_y - v_kp1 * f_k_y )
#ifdef DEBUG_COUT
  cout << vel_ref_.Global.X << " "
       << vel_ref_.Global.Y << endl;
#endif
  Pvsc_x_ = Pvs_*c_k_x_ ;
  Pvsc_y_ = Pvs_*c_k_y_ ;
  // Pzsc_x_, Pzsc_y_ , v_kp1f_x_ and v_kp1f_y_ already up to date
  //from the CoP constraint building function

  p_xy_X_ =    alpha_x_ * Pvu_.transpose() *( Pvsc_x_ - vel_ref_.Global.X_vec)
    + beta_    * Pzu_.transpose() *( Pzsc_x_ - v_kp1f_x_            );
#ifdef DEBUG
  DumpVector("Pvsc_x_"    , Pvsc_x_                    ) ;
  DumpVector("RefVectorX" , vel_ref_.Global.X_vec ) ;
  DumpVector("Pzsc_x_"    , Pzsc_x_                    ) ;
  DumpVector("v_kp1f_x_"  , v_kp1f_x_                  ) ;
#endif
  v_kf_x_(0) = currentSupport_.X ;
  p_xy_Fx_ = - beta_  * V_kp1_.transpose()*( Pzsc_x_ - v_kp1f_x_)
    - delta_ * I_FF_*F_kp1_x_ ;
  - kappa_ * diffMat_*v_kf_x_;

  p_xy_Y_  =   alpha_y_ * Pvu_.transpose() * (Pvsc_y_ - vel_ref_.Global.Y_vec)
    + beta_    * Pzu_.transpose() *( Pzsc_y_ - v_kp1f_y_                 );

  v_kf_y_(0) = currentSupport_.Y ;
  p_xy_Fy_ = - beta_  * V_kp1_.transpose() * ( Pzsc_y_ - v_kp1f_y_)
             - delta_ * I_FF_*F_kp1_y_ ;
             - kappa_ * diffMat_*v_kf_y_ ;

#ifdef DEBUG
  DumpVector("Pvsc_y_"    , Pvsc_y_                    ) ;
  DumpVector("RefVectorY" , vel_ref_.Global.Y_vec ) ;
  DumpVector("Pzsc_y_"    , Pzsc_y_                    ) ;
  DumpVector("v_kp1f_y_"  , v_kp1f_y_                  ) ;
#endif
#ifdef DEBUG_COUT
  cout << "costfunctiob\n" ;
  cout << "c_k_x_ = " << c_k_x_ << endl ;
  cout << "c_k_y_ = " << c_k_y_ << endl ;
  cout << "Pvsc_y_ = " << Pvsc_y_ << endl;
#endif
  unsigned index = 0 ;
  for(unsigned i=0 ; i<N_ ; ++i)
    p_(index+i) = p_xy_X_(i) ;
  index+=N_;
  for(unsigned i=0 ; i<nf_ ; ++i)
    p_(index+i) = p_xy_Fx_(i) ;
  index+=nf_;
  for(unsigned i=0 ; i<N_ ; ++i)
    p_(index+i) = p_xy_Y_(i) ;
  index+=N_;
  for(unsigned i=0 ; i<nf_ ; ++i)
    p_(index+i) = p_xy_Fy_(i) ;
  index+=nf_;
  // p_theta_ = ( 0.5 * a * [ f_k_theta+T_step*dTheta^ref  f_k_theta+2*T_step*dTheta^ref ] )
  for(unsigned i=0 ; i<nf_ ; ++i)
    p_(index+i) = - alpha_theta_ * ( currentSupport_.Yaw +
                                    (i+1) * T_step_* vel_ref_.Global.Yaw )
                  - delta_ * F_kp1_theta_(i) ;

  // Gradient of Objective
  // qp_g_ = (gx    )
  //         (gy    )
  //         (gtheta)
#ifdef DEBUG
  DumpVector( "U_x_" , U_x_ );
#endif
  qp_g_ = qp_H_*U_+p_;

#ifdef DEBUG
  DumpMatrix("qp_H_",qp_H_);
  DumpVector("qp_g_",qp_g_);
#endif
  return ;
}

void NMPCgenerator::setLocalVelocityReference(reference_t local_vel_ref)
{
  vel_ref_.Local = local_vel_ref.Local ;
  vel_ref_.Global.X   = vel_ref_.Local.X * cos(currentSupport_.Yaw) -
    vel_ref_.Local.Y * sin(currentSupport_.Yaw) ;
  vel_ref_.Global.Y   = vel_ref_.Local.X * sin(currentSupport_.Yaw) +
    vel_ref_.Local.Y * cos(currentSupport_.Yaw) ;
  vel_ref_.Global.Yaw = vel_ref_.Local.Yaw ;

  if(vel_ref_.Global.X>0.4)
    vel_ref_.Global.X = 0.4;
  if(vel_ref_.Global.X<-0.4)
    vel_ref_.Global.X = -0.4;

  if(vel_ref_.Global.Y>0.3)
    vel_ref_.Global.Y = 0.3;
  if(vel_ref_.Global.Y<-0.3)
    vel_ref_.Global.Y = -0.3;

  if(vel_ref_.Global.Yaw>0.2)
    vel_ref_.Global.Yaw = 0.2 ;
  if(vel_ref_.Global.Yaw<-0.2)
    vel_ref_.Global.Yaw = -0.2 ;
#ifdef DEBUG_COUT
  cout << "velocity = " ;
  cout << vel_ref_.Global.X << " "  ;
  cout << vel_ref_.Global.Y << endl ;
#endif
  vel_ref_.Global.X_vec =
    Eigen::VectorXd::Constant(vel_ref_.Global.X_vec.size(),vel_ref_.Global.X);
  vel_ref_.Global.Y_vec =
    Eigen::VectorXd::Constant(vel_ref_.Global.Y_vec.size(),vel_ref_.Global.Y);
#ifdef DEBUG
  DumpVector("RefVectorX"    ,vel_ref_.Global.X_vec  );
  DumpVector("RefVectorY"    ,vel_ref_.Global.Y_vec  );
#endif
  return ;
}

void NMPCgenerator::setGlobalVelocityReference(reference_t global_vel_ref)
{
  vel_ref_.Global.X = global_vel_ref.Local.X ;
  vel_ref_.Global.Y = global_vel_ref.Local.Y ;
  vel_ref_.Global.Yaw = global_vel_ref.Local.Yaw ;

  if(vel_ref_.Global.X>0.4)
    vel_ref_.Global.X = 0.4;
  if(vel_ref_.Global.X<-0.4)
    vel_ref_.Global.X = -0.4;

  if(vel_ref_.Global.Y>0.3)
    vel_ref_.Global.Y = 0.3;
  if(vel_ref_.Global.Y<-0.3)
    vel_ref_.Global.Y = -0.3;

  if(vel_ref_.Global.Yaw>0.2)
    vel_ref_.Global.Yaw = 0.2 ;
  if(vel_ref_.Global.Yaw<-0.2)
    vel_ref_.Global.Yaw = -0.2 ;

  vel_ref_.Local.X   =  vel_ref_.Global.X * cos(currentSupport_.Yaw) + vel_ref_.Global.Y * sin(currentSupport_.Yaw) ;
  vel_ref_.Local.Y   = -vel_ref_.Global.X * sin(currentSupport_.Yaw) + vel_ref_.Global.Y * cos(currentSupport_.Yaw) ;
  vel_ref_.Local.Yaw = vel_ref_.Global.Yaw ;
  { for(unsigned int i=0;i<vel_ref_.Global.X_vec.size();vel_ref_.Global.X_vec[i++]=vel_ref_.Global.X);} ;
  { for(unsigned int i=0;i<vel_ref_.Global.Y_vec.size();vel_ref_.Global.Y_vec[i++]=vel_ref_.Global.Y);} ;
#ifdef DEBUG
  DumpVector("RefVectorX"    ,vel_ref_.Global.X_vec  );
  DumpVector("RefVectorY"    ,vel_ref_.Global.Y_vec  );
#endif
  return ;
}

void NMPCgenerator::initializeLineSearch()
{
  HUn_.resize(nv_); HUn_.setZero();
  U_n_.resize(nv_); U_n_.setZero();
  JdU_.resize(nc_); JdU_.setZero();
  selectActiveConstraint.resize(nc_);
  selectActiveConstraint.setZero();
  lineStep_=1.0; lineStep0_=1.0 ; // step searched
  cm_=0.0; c_=1.0 ; // Merit Function Jacobian
  mu_ = 1.0 ;
  stepParam_ = 0.8 ;
  L_n_=0.0; L_=0.0; // Merit function of the next step and Merit function
  maxLineSearchIteration_ = 1000 ;
}

void NMPCgenerator::lineSearch()
{
  // TODO make a better line search
  lineStep_ = lineStep0_ ;
  return ;
}

double NMPCgenerator::evalMeritFunctionJacobian()
{
  double meritJac = 0.0 ;
//  for (unsigned i=0; i<nv_ ; ++i)
//    meritJac += qp_g_(i) * deltaU_[i] ;

  //constraintJacobian = mu*sum((sign(qp_J_*deltaU_)*qp_J_*deltaU_)
  //                       *selecActiveConstraint);
  double constrValue = 0.0 ;
  JdU_.resize(nc_); JdU_.setZero();

  for (unsigned i=0; i<nc_ ; ++i)
  {
    if(selectActiveConstraint(i)!=0.0)
    {
      for (unsigned j=0; j<nv_ ; ++j)
      {
        JdU_(i) += qp_J_(i,j) * deltaU_[j];
      }
      if(selectActiveConstraint(i) > 0.0)
      {
        constrValue = gU_(i) - ub_(i) ;
      }

      double sign = 1.0 ;
      if(constrValue<0.0)
        sign=-1.0;

      meritJac += mu_ * sign * JdU_(i) ;
    }
  }
  return meritJac ;
}

double NMPCgenerator::evalMeritFunction()
{
  // evaluation of the cost function
  HUn_ = U_n_*qp_H_;
  double costFunction = 0.0 ;
  for(unsigned i=0 ; i<nv_ ; ++i)
  {
    costFunction += U_n_(i)*HUn_(i) + p_(i)*U_n_(i);
  }
  // cout << "cost = " << costFunction << " ; constrNorm = " ;
  // evaluate constraint norm
  double constrValueNorm = 0.0 ;
  for (unsigned i=0; i<nc_ ; ++i)
  {
    if(selectActiveConstraint(i)!=0.0)
    {
      if(selectActiveConstraint(i)>0.0)
      {
        double tmp = gU_(i) - ub_(i) ;
        if (tmp<0)
          tmp = -tmp;
        constrValueNorm += tmp ;
      }
    }
  }
  //cout << endl ;
  return /*costFunction + mu_ * */constrValueNorm ;
}

void NMPCgenerator::updateIterationBeforeLanding()
{
#ifdef DEBUG_COUT
  cout << "v_kp1_ = " << v_kp1_ << endl ;
#endif
  itBeforeLanding_ = 0;
  for(unsigned i=0 ; i<v_kp1_.size() ; ++i)
    if(v_kp1_(i))
      ++itBeforeLanding_;
    else
      break;
  --itBeforeLanding_;
  if(itBeforeLanding_>(unsigned int)itMax_)
    itBeforeLanding_=itMax_;
#ifdef DEBUG_COUT
  cout << "itBeforeLanding_ = " << itBeforeLanding_ << endl ;
#endif
  return ;
}

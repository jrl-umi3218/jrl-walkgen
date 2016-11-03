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

//#define DEBUG
//#define DEBUG_COUT

//#ifdef DEBUG
void DumpMatrix(std::string fileName, MAL_MATRIX_TYPE(double) & M)
{
  std::ofstream aof;
  std::ostringstream oss(std::ostringstream::ate);
  aof.open(fileName.c_str(),std::ofstream::out);
  aof.close();
  aof.open(fileName.c_str(),std::ofstream::app);
  aof.precision(18);
  aof.setf(std::ios::scientific, std::ios::floatfield);
  for (unsigned int i = 0 ; i < M.size1() ; ++i)
  {
    for (unsigned int j = 0 ; j < M.size2()-1 ; ++j)
    {
      if(M(i,j)*M(i,j) < 1e-10)
      {
        aof << 0.0 << " " ;
      }else
      {
        aof << M(i,j) << " " ;
      }
    }
    if(M(i,M.size2()-1)*M(i,M.size2()-1) < 1e-10)
    {
      aof << 0.0 << std::endl ;
    }else
    {
      aof << M(i,M.size2()-1) << std::endl ;
    }
  }
}

void DumpVector(std::string fileName, MAL_VECTOR_TYPE(double) & M)
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

NMPCgenerator::NMPCgenerator(SimplePluginManager * aSPM, PinocchioRobot *aPR)
{
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
  gamma_=0.0;
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
  useLineSearch_ = false;
  itBeforeLanding_ = 0;

  SupportStates_deq_.clear();
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
  MAL_MATRIX_RESIZE(Pps_,N_,3);              MAL_MATRIX_FILL(Pps_,0.0);
  MAL_MATRIX_RESIZE(Ppu_,N_,N_);             MAL_MATRIX_FILL(Ppu_,0.0);
  MAL_MATRIX_RESIZE(Pvs_,N_,3);              MAL_MATRIX_FILL(Pvs_,0.0);
  MAL_MATRIX_RESIZE(Pvu_,N_,N_);             MAL_MATRIX_FILL(Pvu_,0.0);
  MAL_MATRIX_RESIZE(Pas_,N_,3);              MAL_MATRIX_FILL(Pas_,0.0);
  MAL_MATRIX_RESIZE(Pau_,N_,N_);             MAL_MATRIX_FILL(Pau_,0.0);
  MAL_MATRIX_RESIZE(Pzs_,N_,3);              MAL_MATRIX_FILL(Pzs_,0.0);
  MAL_MATRIX_RESIZE(Pzu_,N_,N_);             MAL_MATRIX_FILL(Pzu_,0.0);
  MAL_VECTOR_RESIZE(v_kp1_,N_) ;             MAL_VECTOR_FILL(v_kp1_,0.0) ;
  MAL_MATRIX_RESIZE(V_kp1_,N_,nf_) ;         MAL_MATRIX_FILL(V_kp1_,0.0) ;
  MAL_VECTOR_RESIZE(U_          ,2*N_+3*nf_);MAL_VECTOR_FILL(U_          ,0.0);
  MAL_VECTOR_RESIZE(deltaU_thresh_,2*N_+3*nf_);MAL_VECTOR_FILL(deltaU_thresh_,0.0);
  MAL_VECTOR_RESIZE(U_xy_       ,2*(N_+nf_));MAL_VECTOR_FILL(U_xy_       ,0.0);
  MAL_VECTOR_RESIZE(U_x_        ,N_+nf_);    MAL_VECTOR_FILL(U_x_        ,0.0);
  MAL_VECTOR_RESIZE(U_y_        ,N_+nf_);    MAL_VECTOR_FILL(U_y_        ,0.0);
  MAL_VECTOR_RESIZE(F_kp1_x_    ,nf_);       MAL_VECTOR_FILL(F_kp1_x_    ,0.0);
  MAL_VECTOR_RESIZE(F_kp1_y_    ,nf_);       MAL_VECTOR_FILL(F_kp1_y_    ,0.0);
  MAL_VECTOR_RESIZE(F_kp1_theta_,nf_);       MAL_VECTOR_FILL(F_kp1_theta_,0.0);
  MAL_VECTOR_RESIZE(c_k_x_,3);               MAL_VECTOR_FILL(c_k_x_ ,0.0);
  MAL_VECTOR_RESIZE(c_k_y_,3);               MAL_VECTOR_FILL(c_k_y_ ,0.0);
  MAL_MATRIX_RESIZE(A0r_   ,5,2) ;           MAL_MATRIX_FILL(A0r_   ,0.0);
  MAL_VECTOR_RESIZE(ubB0r_ ,5) ;             MAL_VECTOR_FILL(ubB0r_ ,0.0);
  MAL_MATRIX_RESIZE(A0l_   ,5,2) ;           MAL_MATRIX_FILL(A0l_   ,0.0);
  MAL_VECTOR_RESIZE(ubB0l_ ,5) ;             MAL_VECTOR_FILL(ubB0l_ ,0.0);
  MAL_MATRIX_RESIZE(A0rf_  ,4,2) ;           MAL_MATRIX_FILL(A0rf_  ,0.0);
  MAL_VECTOR_RESIZE(ubB0rf_,4) ;             MAL_VECTOR_FILL(ubB0rf_,0.0);
  MAL_MATRIX_RESIZE(A0lf_  ,4,2) ;           MAL_MATRIX_FILL(A0lf_  ,0.0);
  MAL_VECTOR_RESIZE(ubB0lf_,4) ;             MAL_VECTOR_FILL(ubB0lf_,0.0);
  MAL_MATRIX_RESIZE(A0ds_  ,4,2) ;           MAL_MATRIX_FILL(A0ds_  ,0.0);
  MAL_VECTOR_RESIZE(ubB0ds_,4) ;             MAL_VECTOR_FILL(ubB0ds_,0.0);
  MAL_MATRIX_RESIZE(A0_xy_   ,4,2) ;         MAL_MATRIX_FILL(A0_xy_   ,0.0);
  MAL_MATRIX_RESIZE(A0_theta_,4,2) ;         MAL_MATRIX_FILL(A0_theta_,0.0);
  MAL_VECTOR_RESIZE(B0_      ,4) ;           MAL_VECTOR_FILL(B0_      ,0.0);
  MAL_VECTOR_RESIZE(vel_ref_.Global.X_vec , N_) ;
  MAL_VECTOR_RESIZE(vel_ref_.Global.Y_vec , N_) ;
  MAL_VECTOR_RESIZE(vel_ref_.Local .X_vec , N_) ;
  MAL_VECTOR_RESIZE(vel_ref_.Local .Y_vec , N_) ;
  MAL_MATRIX_RESIZE(rotMat_xy_,2,2); MAL_MATRIX_FILL(rotMat_xy_ ,0.0);
  MAL_MATRIX_RESIZE(rotMat_theta_,2,2); MAL_MATRIX_FILL(rotMat_theta_ ,0.0);
  MAL_MATRIX_RESIZE(rotMat_,2,2); MAL_MATRIX_FILL(rotMat_ ,0.0);
  MAL_MATRIX_RESIZE(tmpRotMat_,2,2); MAL_MATRIX_FILL(tmpRotMat_ ,0.0);
  MAL_VECTOR_RESIZE(qp_J_obs_i_, nv_); MAL_VECTOR_FILL(qp_J_obs_i_, 0.0);
  MAL_VECTOR_FILL(F_kp1_x_,currentSupport.X);
  MAL_VECTOR_FILL(F_kp1_y_,currentSupport.Y);

  T_ = T ;
  Tfirst_ = T ;
  T_step_ = T_step ;
  useLineSearch_=useLineSearch;

  alpha_x_     = 5.0 ; // weight for CoM velocity X tracking  : 0.5 * a ; 2.5
  alpha_y_     = 5.0 ; // weight for CoM velocity Y tracking  : 0.5 * a ; 2.5
  alpha_theta_ = 1e+06 ; // weight for CoM velocity Yaw tracking  : 0.5 * a ; 2.5
  beta_  = 1e+03 ; // weight for ZMP reference tracking : 0.5 * b ; 1e+03
  gamma_ = 1e-08 ; // weight for jerk minimization      : 0.5 * c ; 1e-04
  delta_ = 1e-05 ; // weight for foot position evolution: 0.5 * d ; 1e-04
  kappa_ = 0.0 ;   // weight for foot distance from support: 0.5 * d ; 1e-04

  SecurityMarginX_ = 0.09 ;
  SecurityMarginY_ = 0.05 ;
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
  oss << ":setfeetconstraint XY " << SecurityMarginX_ << " " << SecurityMarginY_ ;
  istringstream strm(oss.str());
  string cmd ;
  strm >> cmd ;
  RFI_->CallMethod(cmd,strm);
  FeetDistance_ = RFI_->DSFeetDistance();

  itBeforeLanding_=(int)round(T_step_/T_);
  itMax_=itBeforeLanding_;

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
  MAL_MATRIX_RESIZE(qp_J_     ,nc_,nv_);     MAL_MATRIX_FILL(qp_J_      ,0.0);
  MAL_VECTOR_RESIZE(qp_lbJ_   ,nc_);         MAL_VECTOR_FILL(qp_lbJ_    ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_   ,nc_);         MAL_VECTOR_FILL(qp_ubJ_    ,0.0);
  MAL_VECTOR_RESIZE(qp_lb_    ,nv_);       MAL_VECTOR_FILL(qp_lb_   ,-1e+8);
  MAL_VECTOR_RESIZE(qp_ub_    ,nv_);       MAL_VECTOR_FILL(qp_ub_   , 1e+8);

  MAL_VECTOR_RESIZE(lb_  ,      nc_);  MAL_VECTOR_FILL(lb_  , 0.0);
  MAL_VECTOR_RESIZE(ub_  ,      nc_);  MAL_VECTOR_FILL(ub_  , 0.0);
  MAL_VECTOR_RESIZE(gU_  ,      nc_);  MAL_VECTOR_FILL(gU_  , 0.0);
  MAL_VECTOR_RESIZE(Uxy_ ,2*(N_+nf_));  MAL_VECTOR_FILL(Uxy_ , 0.0);

  MAL_MATRIX_RESIZE(Pzuv_,2*N_,2*(N_+nf_));
  MAL_VECTOR_RESIZE(Pzsc_,2*N_);
  MAL_VECTOR_RESIZE(Pzsc_x_,N_);
  MAL_VECTOR_RESIZE(Pzsc_y_,N_);
  MAL_VECTOR_RESIZE(v_kp1f_,2*N_);
  MAL_VECTOR_RESIZE(v_kp1f_x_,N_);
  MAL_VECTOR_RESIZE(v_kp1f_y_,N_);

  // build constant matrices
  buildCoMCoPIntegrationMatrix();
  buildConvexHullSystems();

  // initialize time dependant matrices
  initializeCoPConstraint();
  //initializeFootExactPositionConstraint();
  initializeFootPoseConstraint();
  //initializeFootVelIneqConstraint();
  //initializeRotIneqConstraint();
  //initializeObstacleConstraint();
  //initializeStandingConstraint();

  ncineq_ = nc_cop_ + nc_foot_ + nc_rot_ + nc_obs_ ;
  nceq_ = nc_vel_ ;

  MAL_VECTOR_RESIZE(gU_cop_  ,nc_cop_); MAL_VECTOR_FILL(gU_cop_  , 0.0);
  MAL_VECTOR_RESIZE(gU_foot_ ,nc_foot_); MAL_VECTOR_FILL(gU_foot_  , 0.0);
  MAL_VECTOR_RESIZE(gU_vel_  ,nc_vel_); MAL_VECTOR_FILL(gU_vel_  , 0.0);
  MAL_VECTOR_RESIZE(gU_obs_  ,nc_obs_); MAL_VECTOR_FILL(gU_obs_  , 0.0);
  MAL_VECTOR_RESIZE(gU_rot_  ,nc_rot_); MAL_VECTOR_FILL(gU_rot_  , 0.0);
  MAL_VECTOR_RESIZE(gU_stan_ ,nc_stan_); MAL_VECTOR_FILL(gU_stan_ , 0.0);

  return ;
}

void NMPCgenerator::updateConstraint()
{
  updateCoPconstraint(U_);
  //updateFootExactPositionConstraint();
  updateFootPoseConstraint(U_);
  //updateFootVelIneqConstraint();
  //updateRotIneqConstraint();
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
  nc_ = nc_cop_+nc_foot_+nc_vel_+nc_rot_+nc_obs_+nc_stan_ ;
  ncineq_ = nc_cop_ + nc_foot_ + nc_rot_ + nc_obs_ ;
  nceq_ = nc_vel_ ;

  unsigned N2nf2 = 2*(N_+nf_) ;
  MAL_MATRIX_RESIZE(qp_J_,nc_,nv_);
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
      qp_J_(i,j)=Acop_xy_(i,j);
    for(unsigned j=0; j<nf_ ; ++j)
      qp_J_(i,j+N2nf2)=Acop_theta_(i,j);
  }
  index = nc_cop_ ;
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
      if(i==j)
        qp_J_(index+i,2*N_+2*nf_+j) =  1.0 ;
      if(j==(i-1))
        qp_J_(index+i,2*N_+2*nf_+j) = -1.0 ;
    }
  }
  index += nc_rot_ ;
  for(unsigned obs=0 ; obs<obstacles_.size() ; ++obs)
  {
    for(unsigned n=0 ; n<nf_ ; ++n)
    {
      qp_J_obs_i_ = 2 * MAL_RET_A_by_B(U_xy_,Hobs_[obs][n]) + Aobs_[obs][n] ;
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
  qp_lbJ_ = lb_ - gU_ ;
  qp_ubJ_ = ub_ - gU_ ;

#ifdef DEBUG
  DumpMatrix("qp_J_",qp_J_);
  DumpVector("qp_lbJ_",qp_lbJ_);
  DumpVector("qp_ubJ_",qp_ubJ_);
#endif
  return ;
}


void NMPCgenerator::evalConstraint(MAL_VECTOR_TYPE(double) & U)
{
  //
  //  Eval real problem bounds : lb_ < g(U) < ub_
  for(unsigned i=0 ; i<2*(N_+nf_) ; ++i)
  {Uxy_(i)=U(i);}

  //    CoP
  gU_cop_ = MAL_RET_A_by_B(Acop_xy_,Uxy_);
  //    Foot
  gU_foot_ = MAL_RET_A_by_B(Afoot_xy_full_,Uxy_);
  //    Velocity
  gU_vel_ = MAL_RET_A_by_B(Avel_,U) ;
  //    Rotation
  gU_rot_ = MAL_RET_A_by_B(Arot_,U);
  //    Obstacle
  MAL_VECTOR_RESIZE(gU_obs_ ,nc_obs_);
  for(unsigned obs=0 ; obs<obstacles_.size() ; ++obs)
  {
    for(unsigned n=0 ; n<nf_ ; ++n)
    {
      MAL_VECTOR_TYPE(double) HobsUxy = MAL_RET_A_by_B(Hobs_[obs][n],Uxy_);
      double deltaObs = 0 ;
      for(unsigned i=0 ; i<MAL_VECTOR_SIZE(HobsUxy) ; ++i)
          deltaObs += Uxy_(i) * (HobsUxy(i) + Aobs_[obs][n](i)) ;
      gU_obs_(obs*nf_ + n) = deltaObs ;
    }
  }
  // Standing
  //gU_stan_ = MAL_RET_A_by_B(Astan_,U) ;

  MAL_VECTOR_RESIZE(lb_,nc_);
  MAL_VECTOR_RESIZE(ub_,nc_);
  MAL_VECTOR_RESIZE(gU_,nc_);

  // Fill up lb_, ub_ and gU_
  unsigned index = 0 ;
  for(unsigned i=0 ; i<nc_vel_ ; ++i)
  {
    lb_(index+i) = LBvel_ (i);
    ub_(index+i) = UBvel_ (i);
    gU_(index+i) = gU_vel_(i);
  }
  index += nc_vel_ ;
  for(unsigned i=0 ; i<nc_cop_ ; ++i)
  {
    lb_(index+i) = LBcop_ (i) ;
    ub_(index+i) = UBcop_ (i) ;
    gU_(index+i) = gU_cop_(i);
  }
  index += nc_cop_ ;
  for(unsigned i=0 ; i<nc_foot_ ; ++i)
  {
    lb_(index+i) = LBfoot_full_ (i);
    ub_(index+i) = UBfoot_full_ (i);
    gU_(index+i) = gU_foot_(i);
  }
  index += nc_foot_ ;
  for(unsigned i=0 ; i<nc_rot_ ; ++i)
  {
    lb_(index+i) = LBrot_ (i);
    ub_(index+i) = UBrot_ (i);
    gU_(index+i) = gU_rot_(i);
  }
  index += nc_rot_ ;
  for(unsigned obs=0 ; obs<obstacles_.size() ; ++obs)
  {
    for(unsigned n=0 ; n<nf_ ; ++n)
    {
      lb_(index + obs*nf_+n) = LBobs_[obs](n);
      ub_(index + obs*nf_+n) = UBobs_[obs](n);
      gU_(index + obs*nf_+n) = gU_obs_(obs*nf_+n) ;
    }
  }

//  index += nc_rot_ ;
//  for(unsigned i=0 ; i<nc_stan_ ; ++i)
//  {
//    lb_(index+i) = LBstan_ (i);
//    ub_(index+i) = UBstan_ (i);
//    gU_(index+i) = gU_stan_(i);
//  }
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
  Pzsc_x_ = MAL_RET_A_by_B(Pzs_,c_k_x_);
  Pzsc_y_ = MAL_RET_A_by_B(Pzs_,c_k_y_);

  for (unsigned i=0 ; i<N_ ; ++i)
  {
    if(SupportStates_deq_[i+1].Phase==DS)
    {
      if(SupportStates_deq_[i+1].Foot==LEFT)
      {
        v_kp1f_x_(i) = v_kp1_(i) *
            (SupportStates_deq_[i+1].X + sin(SupportStates_deq_[i+1].Yaw)*FeetDistance_*0.5);
        v_kp1f_y_(i) = v_kp1_(i) *
            (SupportStates_deq_[i+1].Y - cos(SupportStates_deq_[i+1].Yaw)*FeetDistance_*0.5);
      }else
      {
        v_kp1f_x_(i) = v_kp1_(i) *
            (SupportStates_deq_[i+1].X - sin(SupportStates_deq_[i+1].Yaw)*FeetDistance_*0.5);
        v_kp1f_y_(i) = v_kp1_(i) *
            (SupportStates_deq_[i+1].Y + cos(SupportStates_deq_[i+1].Yaw)*FeetDistance_*0.5);
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
  MAL_VECTOR_RESIZE(deltaU_thresh_,nv_);

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

void NMPCgenerator::solve_qp(){
  // primal SQP solution
  QP_->solve(QuadProg_H_,QuadProg_g_,
             QuadProg_J_eq_,QuadProg_bJ_eq_,
             QuadProg_J_ineq_,QuadProg_lbJ_ineq_,false);
//  if(QP_->fail()==0)
//    cerr << "qp solveur succeded" << endl ;
  if(QP_->fail()==1)
    cerr << "qp solveur failed : problem has no solution" << endl ;
  if(QP_->fail()==2)
    cerr << "qp solveur failed : problems with decomposing H" << endl ;

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
//  lineSearch();

  for(unsigned i=0 ; i<nv_ ; ++i)
    deltaU_thresh_(i) = deltaU_(i);

//  if(useLineSearch_)
//  {
//    for(unsigned i=N_ ; i<N_+nf_ ; ++i)
//      if(deltaU_[i]*deltaU_[i]<1e-05)
//        deltaU_thresh_(i)=0.0;
//      else
//        deltaU_thresh_(i)=deltaU_[i];

//    for(unsigned i=2*N_+nf_ ; i<2*N_+2*nf_ ; ++i)
//      if(deltaU_[i]*deltaU_[i]<1e-05)
//        deltaU_thresh_(i)=0.0;
//      else
//        deltaU_thresh_(i)=deltaU_[i];

//    for(unsigned i=2*N_+2*nf_ ; i<2*N_+3*nf_ ; ++i)
//      if(deltaU_[i]*deltaU_[i]<1e-04)
//        deltaU_thresh_(i)=0.0;
//      else
//        deltaU_thresh_(i)=deltaU_[i];
//  }

  U_ += /*lineStep_*/1.0 * deltaU_thresh_ ;

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

void NMPCgenerator::getSolution(std::vector<double> & JerkX,
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
  if(currentSupport_.NbStepsLeft <= 0 && SupportStates_deq_.back().StepNumber <= 0)
  {
    FootStepX  [0] = currentSupport_.X  + sign*sin(currentSupport_.Yaw)*FeetDistance_ ;
    FootStepY  [0] = currentSupport_.Y  - sign*cos(currentSupport_.Yaw)*FeetDistance_ ;
    FootStepYaw[0] = currentSupport_.Yaw ;
    for(unsigned i=1 ; i<nf ; ++i)
    {
      sign = -sign ;
      FootStepX  [i] = FootStepX[i-1] + sign*sin(FootStepYaw[i-1])*FeetDistance_ ;
      FootStepY  [i] = FootStepY[i-1] - sign*cos(FootStepYaw[i-1])*FeetDistance_ ;
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
//    FootStepX  [nf] = FootStepX[nf-1] + vel_ref_.Global.X*T_ + sign*sin(FootStepYaw[nf-1])*FeetDistance_ ;
//    FootStepY  [nf] = FootStepY[nf-1] + vel_ref_.Global.Y*T_ - sign*cos(FootStepYaw[nf-1])*FeetDistance_ ;
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

//void NMPCgenerator::updateFinalStateMachine(
//    double time,
//    FootAbsolutePosition & FinalLeftFoot,
//    FootAbsolutePosition & FinalRightFoot)
//{
//#ifdef DEBUG_COUT
//  cout << "previous support : \n"
//       << currentSupport_.Phase        << " "
//       << currentSupport_.Foot         << " "
//       << currentSupport_.StepNumber   << " "
//       << currentSupport_.StateChanged << " "
//       << currentSupport_.X   << " "
//       << currentSupport_.Y   << " "
//       << currentSupport_.Yaw << " "
//       << currentSupport_.NbStepsLeft << " "
//       << endl ;
//#endif
//  const FootAbsolutePosition * FAP = NULL;
//  reference_t vel = vel_ref_;
//  //vel.Local.X=1;
//  // DETERMINE CURRENT SUPPORT STATE:
//  // --------------------------------
//  FSM_->set_support_state( time, 0, currentSupport_, vel );
//  if( currentSupport_.StateChanged == true )
//  {
//    if( currentSupport_.Foot == LEFT )
//      FAP = & FinalLeftFoot;
//    else
//      FAP = & FinalRightFoot;
//    currentSupport_.X = FAP->x;
//    currentSupport_.Y = FAP->y;
//    currentSupport_.Yaw = FAP->theta*M_PI/180.0;
//    currentSupport_.StartTime = time;
//  }
//  SupportStates_deq_[0] = currentSupport_ ;

//  // PREVIEW SUPPORT STATES:
//  // -----------------------
//  // initialize the previewed support state before previewing
//  support_state_t PreviewedSupport = currentSupport_;
//  PreviewedSupport.StepNumber  = 0;
//  for( unsigned pi=1 ; pi<=N_ ; pi++ )
//  {
//    FSM_->set_support_state( time, pi, PreviewedSupport, vel );
//    if( PreviewedSupport.StateChanged )
//    {
//      if( pi == 1 || SupportStates_deq_[pi-1].Phase==DS )//Foot down
//      {
//        if( PreviewedSupport.Foot == LEFT )
//          FAP = & FinalLeftFoot;
//        else
//          FAP = & FinalRightFoot;
//        PreviewedSupport.X = FAP->x;
//        PreviewedSupport.Y = FAP->y;
//        PreviewedSupport.Yaw = FAP->theta*M_PI/180.0;
//        PreviewedSupport.StartTime = time+pi*T_;
//      }
//      if( /*pi > 1 &&*/ PreviewedSupport.StepNumber > 0 )
//      {
//        PreviewedSupport.X = 0.0;
//        PreviewedSupport.Y = 0.0;
//      }
//    }
//    SupportStates_deq_[pi] = PreviewedSupport ;
//  }
//#ifdef DEBUG_COUT
//  for(unsigned i=0;i<SupportStates_deq_.size();++i)
//  {
//    cout << SupportStates_deq_[i].Phase        << " "
//         << SupportStates_deq_[i].Foot         << " "
//         << SupportStates_deq_[i].StepNumber   << " "
//         << SupportStates_deq_[i].StateChanged << " "
//         << SupportStates_deq_[i].X            << " "
//         << SupportStates_deq_[i].Y            << " "
//         << SupportStates_deq_[i].Yaw          << " "
//         << SupportStates_deq_[i].NbStepsLeft  << " "
//         << endl ;
//  }
//#endif
//}

void NMPCgenerator::computeFootSelectionMatrix()
{
  std::deque<support_state_t>::const_iterator SS_it;
  SS_it = SupportStates_deq_.begin();//points at the cur. sup. st.
  ++SS_it;
  MAL_VECTOR_FILL(v_kp1_,0.0);
  MAL_MATRIX_FILL(V_kp1_,0.0);
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
  for(unsigned i=0 ; i<MAL_MATRIX_NB_ROWS(Pzu_) ; ++i)
  {
    for(unsigned j=0 ; j<MAL_MATRIX_NB_COLS(Pzu_) ; ++j)
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
  hull5_.X_vec[0] = -0.28 ; hull5_.Y_vec[0] = -0.2 ;
  hull5_.X_vec[1] = -0.2  ; hull5_.Y_vec[1] = -0.3 ;
  hull5_.X_vec[2] =  0.0  ; hull5_.Y_vec[2] = -0.4 ;
  hull5_.X_vec[3] =  0.2  ; hull5_.Y_vec[3] = -0.3 ;
  hull5_.X_vec[4] =  0.28 ; hull5_.Y_vec[4] = -0.2 ;
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
  hull5_.X_vec[0] = -0.28 ; hull5_.Y_vec[0] = 0.2 ;
  hull5_.X_vec[1] = -0.2  ; hull5_.Y_vec[1] = 0.3 ;
  hull5_.X_vec[2] =  0.0  ; hull5_.Y_vec[2] = 0.4 ;
  hull5_.X_vec[3] =  0.2  ; hull5_.Y_vec[3] = 0.3 ;
  hull5_.X_vec[4] =  0.28 ; hull5_.Y_vec[4] = 0.2 ;
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

  MAL_MATRIX_DIM(tmp52d,double,5,2) ;
  MAL_MATRIX_FILL(tmp52d,0.0) ;

  MAL_VECTOR_DIM(tmp5d,double,5) ;
  MAL_VECTOR_FILL(tmp5d,0.0) ;

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
  nc_cop_ = N_*MAL_VECTOR_SIZE(ubB0rf_) ;

  MAL_MATRIX_RESIZE(Acop_xy_   ,nc_cop_,2*(N_+nf_));
  MAL_MATRIX_RESIZE(Acop_theta_,nc_cop_,nf_);
  MAL_VECTOR_RESIZE(UBcop_     ,nc_cop_);
  MAL_VECTOR_RESIZE(LBcop_     ,nc_cop_);

  MAL_MATRIX_RESIZE(D_kp1_xy_,nc_cop_,2*N_);
  MAL_MATRIX_RESIZE(D_kp1_theta_,nc_cop_,2*N_);
  MAL_VECTOR_RESIZE(b_kp1_,nc_cop_);
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

  // mapping matrix to compute the gradient_theta of the CoP Constraint Jacobian
  for(unsigned j=0 , k=0; j<N_ ; ++j, k+=MAL_MATRIX_NB_ROWS(A0rf_))
    for(unsigned i=0 ; i<MAL_MATRIX_NB_ROWS(A0rf_) ; ++i )
      derv_Acop_map_(i+k,j) = 1.0 ;
#ifdef DEBUG
  DumpMatrix("derv_Acop_map_",derv_Acop_map_);
  DumpMatrix("Pzu_",Pzu_);
#endif
  return ;
}

void NMPCgenerator::updateCoPconstraint(MAL_VECTOR_TYPE(double) &U)
{
  // buils Acop_xy_ + UBcop_
  evalCoPconstraint(U);

  // build Acop_theta_
  vector<double>theta_vec(nf_+1);
  theta_vec[0]=SupportStates_deq_[1].Yaw;
  for(unsigned i=0 ; i<nf_ ; ++i)
  {
    theta_vec[i+1]=U(2*N_+2*nf_+i); //F_kp1_theta_(i);
  }
  MAL_VECTOR_DIM(U_xy,double,2*N_+2*nf_);
  for(unsigned i=0; i<MAL_VECTOR_SIZE(U_xy) ; ++i)
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
      A0_theta_ = MAL_RET_A_by_B(A0ds_,rotMat_theta_) ;
    }
    else if (SupportStates_deq_[i+1].Foot == LEFT)
    {
      A0_theta_ = MAL_RET_A_by_B(A0lf_,rotMat_theta_) ;
    }else{
      A0_theta_ = MAL_RET_A_by_B(A0rf_,rotMat_theta_) ;
    }

    for (unsigned k=0 ; k<MAL_MATRIX_NB_ROWS(A0_theta_) ; ++k)
    {
      // get d_i+1^x(f^'dtheta/dt')
      D_kp1_theta_(i*MAL_MATRIX_NB_ROWS(A0_theta_)+k, i) = A0_theta_(k,0);
      // get d_i+1^y(f^'dtheta/dt')
      D_kp1_theta_(i*MAL_MATRIX_NB_ROWS(A0_theta_)+k, i+N_) = A0_theta_(k,1);
    }
  }
  derv_Acop_map2_ = MAL_RET_A_by_B(derv_Acop_map_ , V_kp1_);
  Acop_theta_dummy0_ = MAL_RET_A_by_B(D_kp1_theta_,Pzuv_);
  Acop_theta_dummy1_ = MAL_RET_A_by_B(Acop_theta_dummy0_,U_xy);
  for(unsigned i=0 ; i<MAL_MATRIX_NB_ROWS(Acop_theta_) ; ++i)
  {
    for(unsigned j=0 ; j<MAL_MATRIX_NB_COLS(Acop_theta_) ; ++j)
    {
      Acop_theta_(i,j) = derv_Acop_map2_(i,j) * Acop_theta_dummy1_(i); // warning this is the real jacobian
      //Acop_theta_(i,j) = 0.0 ; // WARNING this is not the real jacobian !
    }
  }
  return ;
}

void NMPCgenerator::evalCoPconstraint(MAL_VECTOR_TYPE(double) & U)
{
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
      A0_xy_    = MAL_RET_A_by_B(A0ds_,rotMat_xy_  ) ;
      B0_ = ubB0ds_ ;
    }
    else if (SupportStates_deq_[i+1].Foot == LEFT)
    {
      A0_xy_    = MAL_RET_A_by_B(A0lf_,rotMat_xy_   ) ;
      B0_ = ubB0lf_ ;
    }else{
      A0_xy_    = MAL_RET_A_by_B(A0rf_,rotMat_xy_   ) ;
      B0_ = ubB0rf_ ;
    }
    for (unsigned k=0 ; k<MAL_MATRIX_NB_ROWS(A0_xy_) ; ++k)
    {
      // get d_i+1^x(f^theta)
      D_kp1_xy_(i*MAL_MATRIX_NB_ROWS(A0_xy_)+k, i) = A0_xy_(k,0);
      // get d_i+1^y(f^theta)
      D_kp1_xy_(i*MAL_MATRIX_NB_ROWS(A0_xy_)+k, i+N_) = A0_xy_(k,1);

      // get right hand side of equation
      b_kp1_(i*MAL_MATRIX_NB_ROWS(A0_xy_)+k) = B0_(k) ;
    }
  }

  // build Acop_xy_
  Acop_xy_ = MAL_RET_A_by_B(D_kp1_xy_,Pzuv_);
  // build UBcop_
  UBcop_ = b_kp1_ + MAL_RET_A_by_B(D_kp1_xy_,v_kp1f_-Pzsc_) ;

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
  n_vertices_ = MAL_VECTOR_SIZE(ubB0r_) ;
  nc_foot_ = nf_*n_vertices_ ;
  Afoot_xy_   .resize(nf_);
  Afoot_theta_.resize(nf_);
  UBfoot_     .resize(nf_);
  LBfoot_     .resize(nf_);
  SelecMat_   .resize(nf_);
  AdRdF_      .resize(nf_);
  deltaF_     .resize(nf_);
  for(unsigned i=0 ; i< nf_ ; ++i)
  {
    MAL_MATRIX_RESIZE(Afoot_xy_   [i],n_vertices_,2*(N_+nf_));
    MAL_MATRIX_RESIZE(Afoot_theta_[i],n_vertices_,nf_);
    MAL_VECTOR_RESIZE(UBfoot_     [i],n_vertices_);
    MAL_VECTOR_RESIZE(LBfoot_     [i],n_vertices_);
    MAL_MATRIX_RESIZE(SelecMat_   [i],2,2*(N_+nf_));
    MAL_VECTOR_RESIZE(AdRdF_      [i],n_vertices_);
    MAL_VECTOR_RESIZE(deltaF_     [i],2);//2 as [deltaFx,deltaFy]
  }
  rotMat_vec_ .resize(nf_, tmpRotMat_ );
  drotMat_vec_.resize(nf_, tmpRotMat_ );
  MAL_MATRIX_RESIZE(Afoot_xy_full_   ,nc_foot_,2*(N_+nf_));
  MAL_MATRIX_RESIZE(Afoot_theta_full_,nc_foot_,nf_);
  MAL_VECTOR_RESIZE(UBfoot_full_     ,nc_foot_);
  MAL_VECTOR_RESIZE(LBfoot_full_     ,nc_foot_);

  for(unsigned n=0 ; n < nf_ ; ++n)
  {
    MAL_MATRIX_FILL(Afoot_xy_   [n],0.0);
    MAL_MATRIX_FILL(Afoot_theta_[n],0.0);
    MAL_VECTOR_FILL(UBfoot_     [n],0.0);
    MAL_VECTOR_FILL(LBfoot_     [n],-1e+08);
    MAL_MATRIX_FILL(SelecMat_   [n],0.0);
    MAL_VECTOR_FILL(deltaF_     [n],0.0);
    SelecMat_[n](0,N_+n) = 1 ;
    SelecMat_[n](1,2*N_+nf_+n) = 1 ;
    if(n>0)
    {
      SelecMat_[n](0,N_+n-1) = -1 ;
      SelecMat_[n](1,2*N_+nf_+n-1) = -1 ;
    }
  }
  MAL_MATRIX_FILL(Afoot_xy_full_   ,0.0);
  MAL_MATRIX_FILL(Afoot_theta_full_,0.0);
  MAL_VECTOR_FILL(UBfoot_full_     ,0.0);
  MAL_VECTOR_FILL(LBfoot_full_     ,-1e+08);
  return ;
}

void NMPCgenerator::updateFootPoseConstraint(MAL_VECTOR_TYPE(double) &U)
{
  int ignoreFirstStep = 0 ;
  nc_foot_ = nf_*n_vertices_ ;

//  if( desiredNextSupportFootRelativePosition.size()!=0 )
//  {
//    unsigned nbFoot = std::min(desiredNextSupportFootRelativePosition.size(),nf_);
//    ignoreFirstStep = nbFoot ;
//  }

  MAL_MATRIX_RESIZE(Afoot_xy_full_   ,nc_foot_,2*(N_+nf_));
  MAL_MATRIX_RESIZE(Afoot_theta_full_,nc_foot_,nf_);
  MAL_VECTOR_RESIZE(UBfoot_full_     ,nc_foot_);
  MAL_VECTOR_RESIZE(LBfoot_full_     ,nc_foot_);
  MAL_MATRIX_FILL(Afoot_xy_full_   ,0.0);
  MAL_MATRIX_FILL(Afoot_theta_full_,0.0);
  MAL_VECTOR_FILL(UBfoot_full_     ,0.0);
  MAL_VECTOR_FILL(LBfoot_full_     ,-1e+08);

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
      A0f_theta_[n] = MAL_RET_A_by_B(A0r_,drotMat_vec_[n]) ;
    }else{
      A0f_theta_[n] = MAL_RET_A_by_B(A0l_,drotMat_vec_[n]) ;
    }
    if(n!=0)
    {
      deltaF_[n](0)=U(N_+n)-U(N_+n-1) ;//F_kp1_x_[n]-F_kp1_x_[n-1];
      deltaF_[n](1)=U(2*N_+nf_+n)-U(2*N_+nf_+n-1) ;//F_kp1_y_[n]-F_kp1_y_[n-1];
      AdRdF_[n] = MAL_RET_A_by_B(A0f_theta_[n],deltaF_[n]);
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
#endif
  }
  return ;
}

void NMPCgenerator::evalFootPoseConstraint(MAL_VECTOR_TYPE(double) & U)
{
  int ignoreFirstStep = 0 ;
  nc_foot_ = nf_*n_vertices_ ;
//  if( desiredNextSupportFootRelativePosition.size()!=0 )
//  {
//    unsigned nbFoot = std::min(desiredNextSupportFootRelativePosition.size(),nf_);
//    ignoreFirstStep = nbFoot ;
//  }

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
      A0f_xy_   [n] = MAL_RET_A_by_B(A0r_,rotMat_vec_ [n]) ;
      B0f_      [n] = ubB0r_ ;
    }else{
      A0f_xy_   [n] = MAL_RET_A_by_B(A0l_,rotMat_vec_ [n]) ;
      B0f_      [n] = ubB0l_ ;
    }
    MAL_MATRIX_FILL(Afoot_xy_   [n],0.0);
    Afoot_xy_[n] = MAL_RET_A_by_B(A0f_xy_[n],SelecMat_[n]);
    for(unsigned i=0 ; i<n_vertices_ ;++i)
      for(unsigned j=0 ; j<2*(N_+nf_) ; ++j)
        Afoot_xy_full_((n-ignoreFirstStep)*n_vertices_+i,j)
            = Afoot_xy_[n](i,j);

    MAL_VECTOR_FILL(UBfoot_[n] , 0.0);
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
  //nc_vel_ = 1*nf_ ;
  nc_vel_=3;
  MAL_MATRIX_RESIZE(Avel_ ,nc_vel_,2*N_+3*nf_)  ;
  MAL_VECTOR_RESIZE(UBvel_,nc_vel_)  ;
  MAL_VECTOR_RESIZE(LBvel_,nc_vel_)  ;

  MAL_MATRIX_FILL(Avel_ ,0.0)  ;
  MAL_VECTOR_FILL(UBvel_,0.0)  ;
  MAL_VECTOR_FILL(LBvel_,0.0)  ;

//  for(unsigned i=0;i<nf_;++i)
//  {
////    Avel_(i*3+0, i+N_      )  = 1.0 ;
////    Avel_(i*3+1, i+2*N_+nf_)  = 1.0 ;
////    Avel_(i*3+2, i+2*N_+2*nf_) = 1.0 ;
//    Avel_(i, i+2*N_+2*nf_) = 1.0 ;
//  }

  Avel_(0, N_      )  = 1.0 ;
  Avel_(1, 2*N_+nf_)  = 1.0 ;
  Avel_(2, 2*N_+2*nf_) = 1.0 ;

  return ;
}

void NMPCgenerator::updateFootVelIneqConstraint()
{
  if( isFootCloseToLand() )
  {
//    for(unsigned i=0;i<nf_;++i)
//    {
//      UBvel_(i*3+0) = F_kp1_x_(i)     ;
//      UBvel_(i*3+1) = F_kp1_y_(i)     ;
//      UBvel_(i*3+2) = F_kp1_theta_(i) ;

//      LBvel_(i*3+0) = F_kp1_x_(i)     ;
//      LBvel_(i*3+1) = F_kp1_y_(i)     ;
//      LBvel_(i*3+2) = F_kp1_theta_(i) ;

//      Avel_(i*3+0, i+N_      )  = 1.0 ;
//      Avel_(i*3+1, i+2*N_+nf_)  = 1.0 ;
//      Avel_(i*3+2, i+2*N_+2*nf_) = 1.0 ;
////      UBvel_(i) = F_kp1_theta_(i) + 1e-06 ;
////      LBvel_(i) = F_kp1_theta_(i) - 1e-06 ;
////      Avel_(i, i+2*N_+2*nf_) = 1.0 ;
//    }
    UBvel_(0) = F_kp1_x_(0)     ;
    UBvel_(1) = F_kp1_y_(0)     ;
    UBvel_(2) = F_kp1_theta_(0) ;

    LBvel_(0) = F_kp1_x_(0)     ;
    LBvel_(1) = F_kp1_y_(0)     ;
    LBvel_(2) = F_kp1_theta_(0) ;
  }
  else
  {
    MAL_MATRIX_FILL(Avel_ ,0.0)  ;
    MAL_VECTOR_FILL(UBvel_,0.0)  ;
    MAL_VECTOR_FILL(LBvel_,0.0)  ;
  }

#ifdef DEBUG
  DumpMatrix("Avel_",Avel_);
  DumpVector("UBvel_",UBvel_);
#endif
#ifdef DEBUG_COUT
  cout << "Avel_  = " << Avel_  << endl ;
  cout << "UBvel_ = " << UBvel_ << endl ;
#endif
  return ;
}

void NMPCgenerator::initializeFootExactPositionConstraint()
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
  nc_pos_ = 3*nf_ ;
  MAL_MATRIX_RESIZE( Apos_ ,nc_pos_,2*N_+3*nf_)  ;
  MAL_VECTOR_RESIZE(UBpos_ ,nc_pos_)  ;
  MAL_VECTOR_RESIZE(LBpos_ ,nc_pos_)  ;

  MAL_MATRIX_FILL( Apos_,0.0)  ;
  MAL_VECTOR_FILL(UBpos_,0.0)  ;
  MAL_VECTOR_FILL(LBpos_,0.0)  ;

  nc_pos_ = 0 ;

  desiredNextSupportFootRelativePosition.clear();
  desiredNextSupportFootAbsolutePosition.resize(nf_);

  return ;
}

void NMPCgenerator::computeAbsolutePositionFromRelative(
    support_state_t currentSupport,
    const RelativeFootPosition & relativePosition,
    support_state_t & nextSupport)
{
  double c = cos(currentSupport.Yaw);
  double s = sin(currentSupport.Yaw);

  nextSupport.X = currentSupport.X +
      c * relativePosition.sx - s * relativePosition.sy ;
  nextSupport.Y = currentSupport.Y +
      s * relativePosition.sx + c * relativePosition.sy ;
  nextSupport.Yaw = currentSupport.Yaw + relativePosition.theta ;
}

void NMPCgenerator::updateFootExactPositionConstraint()
{
  if( desiredNextSupportFootRelativePosition.size()!=0 )
  {
    unsigned nbFoot = std::min(desiredNextSupportFootRelativePosition.size(),nf_);
    nc_pos_ = 3*nbFoot ;
    for(unsigned i=0 ; i<nbFoot ; ++i)
    {
      if(i==0)
      {
        computeAbsolutePositionFromRelative(
              currentSupport_,
              desiredNextSupportFootRelativePosition[0],
              desiredNextSupportFootAbsolutePosition[0]);
      }else
      {
        computeAbsolutePositionFromRelative(
              desiredNextSupportFootAbsolutePosition[i-1],
              desiredNextSupportFootRelativePosition[i],
              desiredNextSupportFootAbsolutePosition[i]);
      }

      Avel_(0+i*nf_, N_      )  = 1.0 ;
      Avel_(1+i*nf_, 2*N_+nf_)  = 1.0 ;
      Avel_(2+i*nf_,2*N_+2*nf_) = 1.0 ;

      UBvel_(0+i*nf_) = desiredNextSupportFootAbsolutePosition[i].X   ;
      UBvel_(1+i*nf_) = desiredNextSupportFootAbsolutePosition[i].Y   ;
      UBvel_(2+i*nf_) = desiredNextSupportFootAbsolutePosition[i].Yaw ;

      LBvel_(0+i*nf_) = desiredNextSupportFootAbsolutePosition[i].X   ;
      LBvel_(1+i*nf_) = desiredNextSupportFootAbsolutePosition[i].Y   ;
      LBvel_(2+i*nf_) = desiredNextSupportFootAbsolutePosition[i].Yaw ;
    }
  }else
  {
    nc_pos_ = 0 ;
    return ;
  }

#ifdef DEBUG
  DumpMatrix("Avel_",Avel_);
  DumpVector("UBvel_",UBvel_);
#endif
#ifdef DEBUG_COUT
  cout << "Avel_  = " << Avel_  << endl ;
  cout << "UBvel_ = " << UBvel_ << endl ;
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


  for(unsigned i=0 ; i<nc_rot_ ;++i)
  {
    for (unsigned j=0 ; j<nf_ ; ++j)
    {
      if(i==j)
        Arot_(i,2*N_+2*nf_+j) =  1.0 ;
      if(j==i-1)
        Arot_(i,2*N_+2*nf_+j) = -1.0 ;
    }
    UBrot_(i) =  0.17 ;
    LBrot_(i) = -0.17 ;
  }
}

void NMPCgenerator::updateRotIneqConstraint()
{
  UBrot_(0) =  0.17 + currentSupport_.Yaw ;
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
  LBobs_.clear();
  Hobs_.clear();
  Aobs_.clear();
  nc_obs_ = obstacles_.size();

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
  MAL_MATRIX_RESIZE(Astan_  ,nc_stan_,nv_); MAL_MATRIX_FILL(Astan_  ,0.0);
  MAL_VECTOR_RESIZE(UBstan_ ,nc_stan_);     MAL_VECTOR_FILL(UBstan_ ,0.0);
  MAL_VECTOR_RESIZE(LBstan_ ,nc_stan_);     MAL_VECTOR_FILL(LBstan_ ,0.0);
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
    evenPos_x     = currentSupport_.X + sin(currentSupport_.Yaw)*RFI_->DSFeetDistance() ;
    evenPos_y     = currentSupport_.Y - cos(currentSupport_.Yaw)*RFI_->DSFeetDistance() ;
    evenPos_theta = currentSupport_.Yaw ;
  }else
  {
    evenPos_x     = currentSupport_.X - sin(currentSupport_.Yaw)*RFI_->DSFeetDistance() ;
    evenPos_y     = currentSupport_.Y + cos(currentSupport_.Yaw)*RFI_->DSFeetDistance() ;
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
  nc_ = nc_cop_+nc_foot_+nc_vel_+nc_rot_+nc_obs_+nc_stan_ ;

  MAL_MATRIX_RESIZE(qp_H_      ,nv_,nv_);    MAL_MATRIX_SET_IDENTITY(qp_H_);
  MAL_VECTOR_RESIZE(qp_g_      ,nv_);        MAL_VECTOR_FILL(qp_g_      ,0.0);
  MAL_VECTOR_RESIZE(qp_g_x_    ,N_+nf_);     MAL_VECTOR_FILL(qp_g_x_    ,0.0);
  MAL_VECTOR_RESIZE(qp_g_y_    ,N_+nf_);     MAL_VECTOR_FILL(qp_g_y_    ,0.0);
  MAL_VECTOR_RESIZE(qp_g_theta_,nf_);        MAL_VECTOR_FILL(qp_g_theta_,0.0);
  MAL_MATRIX_RESIZE(Q_x_XX_     ,N_,N_);     MAL_MATRIX_FILL(Q_x_XX_     ,0.0);
  MAL_MATRIX_RESIZE(Q_x_XF_     ,N_,nf_);    MAL_MATRIX_FILL(Q_x_XF_     ,0.0);
  MAL_MATRIX_RESIZE(Q_x_FX_     ,nf_,N_);    MAL_MATRIX_FILL(Q_x_FX_     ,0.0);
  MAL_MATRIX_RESIZE(Q_x_FF_     ,nf_,nf_);   MAL_MATRIX_FILL(Q_x_FF_     ,0.0);
  MAL_MATRIX_RESIZE(Q_y_XX_     ,N_,N_);     MAL_MATRIX_FILL(Q_y_XX_     ,0.0);
  MAL_MATRIX_RESIZE(Q_theta_    ,nf_,nf_);   MAL_MATRIX_SET_IDENTITY(Q_theta_);
  MAL_VECTOR_RESIZE(p_xy_X_     ,N_);        MAL_VECTOR_FILL(p_xy_X_ , 0.0);
  MAL_VECTOR_RESIZE(p_xy_Fx_    ,nf_);       MAL_VECTOR_FILL(p_xy_Fx_, 0.0);
  MAL_VECTOR_RESIZE(p_xy_Y_     ,N_);        MAL_VECTOR_FILL(p_xy_Y_ , 0.0);
  MAL_VECTOR_RESIZE(p_xy_Fy_    ,nf_);       MAL_VECTOR_FILL(p_xy_Fy_, 0.0);
  MAL_VECTOR_RESIZE(p_          ,nv_);       MAL_VECTOR_FILL(p_      , 0.0);
  MAL_MATRIX_RESIZE(I_NN_       ,N_,N_);     MAL_MATRIX_SET_IDENTITY(I_NN_);
  MAL_MATRIX_RESIZE(I_FF_       ,nf_,nf_);   MAL_MATRIX_SET_IDENTITY(I_FF_);
  MAL_VECTOR_RESIZE(Pvsc_x_     ,N_);        MAL_VECTOR_FILL(Pvsc_x_ , 0.0);
  MAL_VECTOR_RESIZE(Pvsc_y_     ,N_);        MAL_VECTOR_FILL(Pvsc_y_ , 0.0);
  MAL_VECTOR_RESIZE(Pvsc_y_     ,N_);        MAL_VECTOR_FILL(Pvsc_y_ , 0.0);
  MAL_VECTOR_RESIZE(Pvsc_y_     ,N_);        MAL_VECTOR_FILL(Pvsc_y_ , 0.0);
  MAL_VECTOR_RESIZE(v_kf_x_, nf_);           MAL_VECTOR_FILL(v_kf_x_ , 0.0);
  MAL_VECTOR_RESIZE(v_kf_y_, nf_);           MAL_VECTOR_FILL(v_kf_y_ , 0.0);
  MAL_MATRIX_RESIZE(diffMat_, nf_,nf_);      MAL_MATRIX_SET_IDENTITY(diffMat_);
  for(unsigned i=0 ; i<nf_ ; ++i)
    for(unsigned j=0 ; j<nf_ ; ++j)
      if((i-j)==1)
        diffMat_(i,j)=-1;
#ifdef DEBUG_COUT
  cout << diffMat_ << endl ;
#endif

  // Q_q = 0.5 * a * ( 1 0 )
  //                 ( 0 1 )
  Q_theta_ *= (alpha_theta_ + gamma_) ;

  // Q_x_XF_, Q_x_FX_, Q_x_FF_ are time dependant matrices so they
  // are computed in the update function

  // p_xy_ =  ( p_xy_X_, p_xy_Fx_, p_xy_Y_, p_xy_Fy_ )
  // p_theta_ = ( 0.5 * a * (-2) * [ f_k_theta+T_step*dTheta^ref  f_k_theta+2*T_step*dTheta^ref ] )
  // Those are time dependant matrices so they are computed in the update function
}

void NMPCgenerator::updateCostFunction()
{
//  delta_ = pow(10,(itMax_-(int)itBeforeLanding_)-itMax_/2) ;
//  if(isFootCloseToLand())
//  {
//    alpha_x_ = 1e-03 ;
//    alpha_y_ = 5e-03 ;
//    kappa_=0;
//  }else
//  {
//    alpha_x_ = 1 ;
//    alpha_y_ = 5 ;
//    kappa_=1e-5;
//  }
//  cout << delta_ << " " << alpha_x_ << " " << alpha_y_ << " " << endl  ;

  // Q_xXX = (  0.5 * a * Pvu^T   * Pvu + b * Pzu^T * Pzu + c * I )
  // Q_xXF = ( -0.5 * b * Pzu^T   * V_kp1 )
  // Q_xFX = ( -0.5 * b * V_kp1^T * Pzu )^T
  // Q_xFF = (  0.5 * b * V_kp1^T * V_kp1 )
  Q_x_XX_ = alpha_x_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pvu_),Pvu_)
          + beta_    * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_),Pzu_)
          + gamma_   * I_NN_ ;

  // Q_xXX = (  0.5 * a * Pvu^T   * Pvu + b * Pzu^T * Pzu + c * I )
  // Q_xXF = ( -0.5 * b * Pzu^T   * V_kp1 )
  // Q_xFX = ( -0.5 * b * V_kp1^T * Pzu ) = Q_xXF^T
  // Q_xFF = (  0.5 * b * V_kp1^T * V_kp1 - 0.5 * d * I_FF_)
  Q_x_XF_ = - beta_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_),V_kp1_);
  Q_x_FX_ =   MAL_RET_TRANSPOSE(Q_x_XF_);
  Q_x_FF_ =   beta_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(V_kp1_),V_kp1_)
            + delta_ * I_FF_
            + kappa_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(diffMat_),diffMat_);

  // Q_yXX = (  0.5 * a * Pvu^T   * Pvu + b * Pzu^T * Pzu + c * I )
  Q_y_XX_ = alpha_y_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pvu_),Pvu_)
          + beta_    * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_),Pzu_)
          + gamma_   * I_NN_ ;


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
  Pvsc_x_ = MAL_RET_A_by_B(Pvs_ , c_k_x_) ;
  Pvsc_y_ = MAL_RET_A_by_B(Pvs_ , c_k_y_) ;
  // Pzsc_x_, Pzsc_y_ , v_kp1f_x_ and v_kp1f_y_ already up to date
  //from the CoP constraint building function

  p_xy_X_ =    alpha_x_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pvu_  ), Pvsc_x_ - vel_ref_.Global.X_vec)
             + beta_    * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_  ), Pzsc_x_ - v_kp1f_x_            );
#ifdef DEBUG
  DumpVector("Pvsc_x_"    , Pvsc_x_                    ) ;
  DumpVector("RefVectorX" , vel_ref_.Global.X_vec ) ;
  DumpVector("Pzsc_x_"    , Pzsc_x_                    ) ;
  DumpVector("v_kp1f_x_"  , v_kp1f_x_                  ) ;
#endif
  v_kf_x_(0) = currentSupport_.X ;
  p_xy_Fx_ = - beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(V_kp1_), Pzsc_x_ - v_kp1f_x_)
             - delta_ * F_kp1_x_ ;
             - kappa_ * MAL_RET_A_by_B(v_kf_x_, diffMat_);

  p_xy_Y_  =   alpha_y_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pvu_  ), Pvsc_y_ - vel_ref_.Global.Y_vec)
             + beta_    * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_  ), Pzsc_y_ - v_kp1f_y_                 );

  v_kf_y_(0) = currentSupport_.Y ;
  p_xy_Fy_ = - beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(V_kp1_), Pzsc_y_ - v_kp1f_y_)
             - delta_ * F_kp1_y_ ;
             - kappa_ * MAL_RET_A_by_B(v_kf_y_, diffMat_) ;

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
    p_(index+i) = - alpha_theta_ * ( currentSupport_.Yaw + (i+1)
                  * T_step_* vel_ref_.Global.Yaw)
                  - delta_ * F_kp1_theta_(i) ;

  // Gradient of Objective
  // qp_g_ = (gx    )
  //         (gy    )
  //         (gtheta)
#ifdef DEBUG
  DumpVector( "U_x_" , U_x_ );
#endif
  qp_g_ = MAL_RET_A_by_B(qp_H_,U_)+p_;

#ifdef DEBUG
  DumpMatrix("qp_H_",qp_H_);
  DumpVector("qp_g_",qp_g_);
#endif
  return ;
}

void NMPCgenerator::setLocalVelocityReference(reference_t local_vel_ref)
{
  vel_ref_.Local = local_vel_ref.Local ;
  vel_ref_.Global.X   = vel_ref_.Local.X * cos(currentSupport_.Yaw) - vel_ref_.Local.Y * sin(currentSupport_.Yaw) ;
  vel_ref_.Global.Y   = vel_ref_.Local.X * sin(currentSupport_.Yaw) + vel_ref_.Local.Y * cos(currentSupport_.Yaw) ;
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
  MAL_VECTOR_FILL(vel_ref_.Global.X_vec   , vel_ref_.Global.X  ) ;
  MAL_VECTOR_FILL(vel_ref_.Global.Y_vec   , vel_ref_.Global.Y  ) ;
#ifdef DEBUG
  DumpVector("RefVectorX"    ,vel_ref_.Global.X_vec  );
  DumpVector("RefVectorY"    ,vel_ref_.Global.Y_vec  );
#endif
  return ;
}

void NMPCgenerator::initializeLineSearch()
{
  MAL_VECTOR_RESIZE(HUn_, nv_); MAL_VECTOR_FILL(HUn_, 0.0);
  MAL_VECTOR_RESIZE(U_n_, nv_); MAL_VECTOR_FILL(U_n_, 0.0);
  MAL_VECTOR_RESIZE(JdU_, nc_); MAL_VECTOR_FILL(JdU_, 0.0);
  MAL_VECTOR_RESIZE(selectActiveConstraint, nc_);
  MAL_VECTOR_FILL(selectActiveConstraint, 0.0);
  lineStep_=1.0; lineStep0_=1.0 ; // step searched
  cm_=0.0; c_=1.0 ; // Merit Function Jacobian
  mu_ = 1.0 ;
  stepParam_ = 0.8 ;
  L_n_=0.0; L_=0.0; // Merit function of the next step and Merit function
  maxLineSearchIteration_ = 5 ;
}

void NMPCgenerator::lineSearch()
{
  if(!useLineSearch_ || nc_vel_!=0)
  {
    lineStep_ = lineStep0_ ;
    return;
  }

////  qpOASES::QProblemStatus status_ = QP_->getStatus();
////  if(status_ < qpOASES::QPS_HOMOTOPYQPSOLVED)
////  {
////    lineStep_ = 0.5;
////    return ;
////  }
//  cout << status_ << endl ;
//  cout << "########################################" << endl ;
  // selection active constraints

  U_n_ = U_ ;
  for(unsigned i=0 ; i<nv_ ; ++i)
    U_n_(i) = U_(i) + lineStep0_*deltaU_[i] ;
  evalConstraint(U_n_);
  MAL_VECTOR_RESIZE(selectActiveConstraint, nc_);
  MAL_VECTOR_FILL(selectActiveConstraint,0.0);
  bool badConstraints = false ;
  for(unsigned i=0 ; i<nc_ ; ++i)
  {
    if(gU_(i)-ub_(i)>1e-3)
    {
      selectActiveConstraint(i)=1.0;
//      cout << "constr nb " << i << " ; dist to constr = " << gU_(i)-ub_(i) << endl ;
      badConstraints = true;
    }
    if(-gU_(i)+lb_(i)>1e-3)
    {
      selectActiveConstraint(i)=-1.0;
//      cout << "constr nb " << i << " ; dist to constr = " << -gU_(i)+lb_(i) << endl ;
      badConstraints = true;
    }
  }

  if(!badConstraints)
  {
    lineStep_ = lineStep0_ ;
    return ;
  }

  evalConstraint(U_);
  L_ = evalMeritFunction();
  cm_ = c_ * evalMeritFunctionJacobian();

  lineStep_ = lineStep0_ ;
  bool find_step = false ;
  for (unsigned it=0 ; it<maxLineSearchIteration_ ; ++it)
  {
    for(unsigned i=0 ; i<nv_ ; ++i)
      U_n_(i) = U_(i) + lineStep_*deltaU_[i] ;
    evalConstraint(U_n_);
    L_n_ = evalMeritFunction();

    if ((L_n_ - L_) <= lineStep_ * cm_)
    {
      find_step = true ;
      break;
    }else
    {
      lineStep_ *= stepParam_ ;
#ifdef DEBUG_COUT
      cout << "L_n = "<< L_n_ << " ; "
           << "L = "  << L_   << " ; "
           << "lineStep_ * cm_ = "  << lineStep_ * cm_ << endl ;
#endif
    }
//    if(it==(maxLineSearchIteration_-1))
//      lineStep_ = 0.0 ;
  }
#ifdef DEBUG_COUT
  if(lineStep_!=lineStep0_)
  {
    cout << "#################################" << endl ;
    cout << "lineStep_ = " << lineStep_ << endl ;
  }
  if(!find_step)
  {
    cout << "step not found" << endl ;
  }
#endif
  if(lineStep_!=lineStep0_)
    cout << "lineStep_ = " << lineStep_ << endl ;
  //assert(find_step);
}

double NMPCgenerator::evalMeritFunctionJacobian()
{
  double meritJac = 0.0 ;
//  for (unsigned i=0; i<nv_ ; ++i)
//    meritJac += qp_g_(i) * deltaU_[i] ;

  //constraintJacobian = mu*sum((sign(qp_J_*deltaU_)*qp_J_*deltaU_)
  //                       *selecActiveConstraint);
  double constrValue = 0.0 ;
  MAL_VECTOR_RESIZE(JdU_, nc_);
  MAL_VECTOR_FILL(JdU_,0.0);
  for (unsigned i=0; i<nc_ ; ++i)
  {
    if(selectActiveConstraint(i)!=0.0)
    {
      for (unsigned j=0; j<nv_ ; ++j)
      {
        JdU_(i) += qp_J_(i,j) * deltaU_[j];
      }
      if(selectActiveConstraint(i) < 0.0)
      {
        constrValue = -gU_(i) + lb_(i) ;
      }
      else if(selectActiveConstraint(i) > 0.0)
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
  HUn_ = MAL_RET_A_by_B(U_n_,qp_H_);
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
      else if (selectActiveConstraint(i)<0.0)
      {
        double tmp = -gU_(i) + lb_(i) ;
        if (tmp<0)
          tmp = -tmp;
        constrValueNorm += tmp ;
      }
      //cout << constrValueNorm << " " ;
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
  for(unsigned i=0 ; i<MAL_VECTOR_SIZE(v_kp1_) ; ++i)
    if(v_kp1_(i))
      ++itBeforeLanding_;
    else
      break;
  --itBeforeLanding_;
  if(itBeforeLanding_>itMax_)
    itBeforeLanding_=itMax_;
#ifdef DEBUG_COUT
  cout << "itBeforeLanding_ = " << itBeforeLanding_ << endl ;
#endif
  return ;
}

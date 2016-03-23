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

#ifdef DEBUG
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
#endif // DEBUG

using namespace std;
using namespace PatternGeneratorJRL;

NMPCgenerator::NMPCgenerator(SimplePluginManager * aSPM, CjrlHumanoidDynamicRobot * aHDR)
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

  alpha_=0.0;
  beta_ =0.0;
  gamma_=0.0;

  SPM_ = aSPM ;
  HDR_ = aHDR ;

  FSM_ = new SupportFSM();
  RFI_ = new RelativeFeetInequalities(SPM_,HDR_) ;

  QP_=NULL;
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
  if (deltaU_ !=NULL)
  {
    delete deltaU_;
    deltaU_ = NULL ;
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

void NMPCgenerator::initNMPCgenerator(support_state_t & currentSupport,
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
  MAL_VECTOR_RESIZE(vel_ref_.Global.X_vec , N_) ;
  MAL_VECTOR_RESIZE(vel_ref_.Global.Y_vec , N_) ;

  MAL_MATRIX_RESIZE(rotMat_xy_,2,2); MAL_MATRIX_FILL(rotMat_xy_ ,0.0);
  MAL_MATRIX_RESIZE(rotMat_theta_,2,2); MAL_MATRIX_FILL(rotMat_theta_ ,0.0);
  MAL_MATRIX_RESIZE(rotMat_,2,2); MAL_MATRIX_FILL(rotMat_ ,0.0);
  MAL_MATRIX_RESIZE(tmpRotMat_,2,2); MAL_MATRIX_FILL(tmpRotMat_ ,0.0);
  MAL_VECTOR_RESIZE(qp_J_obs_i_, nv_); MAL_VECTOR_FILL(qp_J_obs_i_, 0.0);

  T_ = T ;
  Tfirst_ = T ;
  T_step_ = T_step ;
  alpha_ = 5 ;// 1     ; // weight for CoM velocity tracking  : 0.5 * a ; 2.5
  beta_  = 1e+03 ;// 1     ; // weight for ZMP reference tracking : 0.5 * b ; 1e+03
  gamma_ = 1e-05 ;// 1e-05 ; // weight for jerk minimization      : 0.5 * c ; 1e-04
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

  // build constant matrices
  buildCoMCoPIntegrationMatrix();
  buildConvexHullSystems();

  // initialize time dependant matrices
  initializeCoPConstraint();
  initializeFootPoseConstraint();
  initializeFootVelIneqConstraint();
  initializeRotIneqConstraint();
  initializeObstacleConstraint();
  initializeStandingConstraint();
  initializeCostFunction();

  // initialize the solver
  QP_ = new qpOASES::SQProblem((int)nv_,(int)nc_,qpOASES::HST_POSDEF) ;
//  options_.initialStatusBounds = qpOASES::ST_INACTIVE ;

  options_.setToMPC();
  QP_->setOptions(options_);
  QP_->setPrintLevel(qpOASES::PL_NONE);
  nwsr_ = 1e+8 ;
  cput_ = new double[1] ;
  deltaU_ = new double[nv_];
  cput_[0] = 1e+8;
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

#ifdef COUT
  cout << time_ << "  "
       << currentSupport_.StartTime << "  "
       << currentSupport_.TimeLimit << "  "
       << endl ;
#endif
  if(currentSupport_.TimeLimit>1e+8)
  {
    Tfirst_ = 0.1 ;
  }
  else if (currentSupport_.StartTime==0.0)
  {
    Tfirst_ = (time_+T_-currentSupport_.TimeLimit)
        - ((double)(int)((time_-currentSupport_.TimeLimit)/0.1) * 0.1) ;
#ifdef COUT
    cout << Tfirst_ << " " ;
#endif
    Tfirst_ = 0.1 - Tfirst_;
  }else
  {
    Tfirst_ = (time_-currentSupport_.StartTime)
        - ((double)(int)((time_-currentSupport_.StartTime)/0.1) * 0.1) ;
    Tfirst_ = 0.1 - Tfirst_;
  }
  if(Tfirst_<0.0001)
    Tfirst_=0.1;
#ifdef COUT
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
  return ;
}

void NMPCgenerator::solve()
{
  if(currentSupport_.Phase==DS && currentSupport_.NbStepsLeft == 0)
    return;
  /* Process and solve problem, s.t. pattern generator data is consistent */
  preprocess_solution() ;
  solve_qp()            ;
  postprocess_solution();
}

void NMPCgenerator::preprocess_solution()
{
  updateCoPConstraint();
  updateFootPoseConstraint();
  updateFootVelIneqConstraint();
  updateRotIneqConstraint();
  updateObstacleConstraint();
  updateStandingConstraint();
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

  //qpOASES::returnValue ret, error ;
  if (!isQPinitialized_)
  {
    cput_[0] = 1000.0;
    nwsr_ = 100000 ;
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
    // force the solver to use the maximum time for computing the solution
//    cput_[0] = 0.0015;
//    nwsr_ = 20 ;
    cput_[0] = 0.0015;
    nwsr_ = 100 ;
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
#ifdef DEBUG_COUT
  bool endline = false;
  if(*cput_ >= 0.0005)
  {
    cout << *cput_ * 1000<< " " ;
    endline = true;
  }
//  if(nwsr_ >= 10)
//  {
//    cout << nwsr_ << " " ;
//    endline = true;
//  }
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
#ifdef DEBUG
  DumpVector("U_",U_);
#endif
#ifdef DEBUG_COUT
  cout << "U_ = " << U_ << endl ;
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
  if(currentSupport_.StateChanged
  || currentSupport_.Phase == DS)
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
      FootStepX  [i] = FootStepX[nf-1] + sign*sin(FootStepYaw[nf-1])*FeetDistance_ ;
      FootStepY  [i] = FootStepY[nf-1] - sign*cos(FootStepYaw[nf-1])*FeetDistance_ ;
      FootStepYaw[i] = FootStepYaw[nf-1] ;
    }
  }else
  {
    // warning "if StateChanged" we need to plan the second step
//    if(currentSupport_.StateChanged)
//      sign = -sign ;

    FootStepX  [nf] = FootStepX[nf-1] + vel_ref_.Global.X*T_ + sign*sin(FootStepYaw[nf-1])*FeetDistance_ ;
    FootStepY  [nf] = FootStepY[nf-1] + vel_ref_.Global.Y*T_ - sign*cos(FootStepYaw[nf-1])*FeetDistance_ ;
    FootStepYaw[nf] = FootStepYaw[nf-1] + vel_ref_.Global.Yaw*T_ ;
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

void NMPCgenerator::updateFinalStateMachine(
    double time,
    FootAbsolutePosition & FinalLeftFoot,
    FootAbsolutePosition & FinalRightFoot)
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
  SupportStates_deq_[0] = currentSupport_ ;

  // PREVIEW SUPPORT STATES:
  // -----------------------
  // initialize the previewed support state before previewing
  support_state_t PreviewedSupport = currentSupport_;
  PreviewedSupport.StepNumber  = 0;
  for( unsigned pi=1 ; pi<=N_ ; pi++ )
  {
    FSM_->set_support_state( time, pi, PreviewedSupport, vel );
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
  MAL_VECTOR_FILL(v_kp1_,0.0);
  MAL_MATRIX_FILL(V_kp1_,0.0);
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

void NMPCgenerator::updateCoPConstraint()
{
  // update the part PzuV depending on Pzu
  for(unsigned i=0 ; i<MAL_MATRIX_NB_ROWS(Pzu_) ; ++i)
  {
    for(unsigned j=0 ; j<MAL_MATRIX_NB_COLS(Pzu_) ; ++j)
    {
      Pzuv_(i,j)=Pzu_(i,j);
      Pzuv_(i+N_,j+N_+nf_)=Pzu_(i,j);
    }
  }

  // Compute D_kp1_, it depends on the feet hulls
  vector<double>theta_vec(3);
  theta_vec[0]=SupportStates_deq_[1].Yaw;
  theta_vec[1]=F_kp1_theta_(0);
  theta_vec[2]=F_kp1_theta_(1);
  // every time instant in the pattern generator constraints
  // depend on the support order
  for (unsigned i=0 ; i<N_ ; ++i)
  {
    double theta = theta_vec[SupportStates_deq_[i+1].StepNumber] ;
    rotMat_xy_(0,0)= cos(theta) ; rotMat_xy_(0,1)= sin(theta) ;
    rotMat_xy_(1,0)=-sin(theta) ; rotMat_xy_(1,1)= cos(theta) ;
    rotMat_theta_(0,0)=-sin(theta) ; rotMat_theta_(0,1)= cos(theta) ;
    rotMat_theta_(1,0)=-cos(theta) ; rotMat_theta_(1,1)=-sin(theta) ;
    if (SupportStates_deq_[i+1].Phase == DS)
    {
      A0_xy_    = MAL_RET_A_by_B(A0ds_,rotMat_xy_  ) ;
      A0_theta_ = MAL_RET_A_by_B(A0ds_,rotMat_theta_) ;
      B0_ = ubB0ds_ ;
    }
    else if(currentSupport_.Phase==SS && i==0 && time_+T_ > currentSupport_.TimeLimit)
    {
      double x1 (currentLeftFootAbsolutePosition_ .x);
      double y1 (currentLeftFootAbsolutePosition_ .y);
      double x2 (currentRightFootAbsolutePosition_.x);
      double y2 (currentRightFootAbsolutePosition_.y);
      double angle = atan2(y2-y1,x2-x1);

      rotMat_(0,0)= cos(angle) ; rotMat_(0,1)= sin(angle) ;
      rotMat_(1,0)=-sin(angle) ; rotMat_(1,1)= cos(angle) ;

      double l = 0.04;
      double L = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)) ;

      hull4_.X_vec[0] = - L*0.5 ; hull4_.Y_vec[0] = - l*0.5 ;
      hull4_.X_vec[1] = - L*0.5 ; hull4_.Y_vec[1] = + l*0.5 ;
      hull4_.X_vec[2] = + L*0.5 ; hull4_.Y_vec[2] = + l*0.5 ;
      hull4_.X_vec[3] = + L*0.5 ; hull4_.Y_vec[3] = - l*0.5 ;
#ifdef COUT
      double m1=(x1+x2)*0.5;
      double m2=(y1+y2)*0.5;
      cout << angle*180/M_PI << " ; "
           << L-2*0.01 << " ; "
           << m1 << " "  << m2 << " ; "
           << hull.X_vec[0] << " " << hull.Y_vec[0] << " ; "
           << hull.X_vec[1] << " " << hull.Y_vec[1] << " ; "
           << hull.X_vec[2] << " " << hull.Y_vec[2] << " ; "
           << hull.X_vec[3] << " " << hull.Y_vec[3] << endl ;
#endif
      dummySupp_.Foot = LEFT ;
      dummySupp_.Phase = SS ;
      RFI_->compute_linear_system( hull4_, dummySupp_ );
      for(unsigned i = 0 ; i < hull4_.A_vec.size() ; ++i)
      {
        A0ds_(i,0) = hull4_.A_vec[i] ;
        A0ds_(i,1) = hull4_.B_vec[i] ;
        ubB0ds_(i) = hull4_.D_vec[i] ;
      }
      A0_xy_ = MAL_RET_A_by_B(A0ds_,rotMat_) ;
      MAL_MATRIX_FILL(A0_theta_,0.0) ;
      B0_ = ubB0ds_ ;
    }
    else if (SupportStates_deq_[i+1].Foot == LEFT)
    {
      A0_xy_    = MAL_RET_A_by_B(A0lf_,rotMat_xy_   ) ;
      A0_theta_ = MAL_RET_A_by_B(A0lf_,rotMat_theta_) ;
      B0_ = ubB0lf_ ;
    }else{
      A0_xy_    = MAL_RET_A_by_B(A0rf_,rotMat_xy_   ) ;
      A0_theta_ = MAL_RET_A_by_B(A0rf_,rotMat_theta_) ;
      B0_ = ubB0rf_ ;
    }
#ifdef DEBUG_COUT
    if(MAL_MATRIX_NB_ROWS(A0_xy_)!=4 ||
       MAL_MATRIX_NB_COLS(A0_xy_)!=2 ||
       MAL_MATRIX_NB_ROWS(A0_theta_)!=4 ||
       MAL_MATRIX_NB_COLS(A0_theta_)!=2 ||
       MAL_VECTOR_SIZE(B0_)!=4)
    {
    cout << A0_xy_ << endl
         << A0_theta_ << endl
         << B0_ << endl ;
    }
#endif
    for (unsigned k=0 ; k<MAL_MATRIX_NB_ROWS(A0_xy_) ; ++k)
    {
      // get d_i+1^x(f^theta)
      D_kp1_xy_(i*MAL_MATRIX_NB_ROWS(A0_xy_)+k, i) = A0_xy_(k,0);
      // get d_i+1^y(f^theta)
      D_kp1_xy_(i*MAL_MATRIX_NB_ROWS(A0_xy_)+k, i+N_) = A0_xy_(k,1);

      // get d_i+1^x(f^'dtheta/dt')
      D_kp1_theta_(i*MAL_MATRIX_NB_ROWS(A0_theta_)+k, i) = A0_theta_(k,0);
      // get d_i+1^y(f^'dtheta/dt')
      D_kp1_theta_(i*MAL_MATRIX_NB_ROWS(A0_theta_)+k, i+N_) = A0_theta_(k,1);

      // get right hand side of equation
      b_kp1_(i*MAL_MATRIX_NB_ROWS(A0_xy_)+k) = B0_(k) ;
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
    }else if(currentSupport_.Phase==SS && i==0 && time_+T_ > currentSupport_.TimeLimit)
    {
      double x1 (currentLeftFootAbsolutePosition_ .x);
      double y1 (currentLeftFootAbsolutePosition_ .y);
      double x2 (currentRightFootAbsolutePosition_.x);
      double y2 (currentRightFootAbsolutePosition_.y);
      double m1=(x1+x2)*0.5;
      double m2=(y1+y2)*0.5;
      v_kp1f_x_(i) = v_kp1_(i) * m1 ;
      v_kp1f_y_(i) = v_kp1_(i) * m2 ;
    }
    else
    {
      v_kp1f_x_(i) = v_kp1_(i) * SupportStates_deq_[i+1].X ;
      v_kp1f_y_(i) = v_kp1_(i) * SupportStates_deq_[i+1].Y ;
    }
  }
#ifdef DEBUG_COUT
  cout << "v_kp1f_x_ = " << v_kp1f_x_ << endl ;
  cout << "v_kp1f_y_ = " << v_kp1f_y_ << endl ;
#endif

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
  Acop_theta_dummy0_ = MAL_RET_A_by_B(D_kp1_theta_,Pzuv_);
  Acop_theta_dummy1_ = MAL_RET_A_by_B(Acop_theta_dummy0_,U_xy_);
  Acop_theta_dummy1_ = MAL_RET_A_by_B(Acop_theta_dummy1_,derv_Acop_map_);
  Acop_theta_dummy1_ = MAL_RET_A_by_B(Acop_theta_dummy1_,V_kp1_);
  for(unsigned i=0 ; i<MAL_MATRIX_NB_ROWS(Acop_theta_) ; ++i)
    for(unsigned j=0 ; j<MAL_MATRIX_NB_COLS(Acop_theta_) ; ++j)
      Acop_theta_(i,j) = Acop_theta_dummy1_(j);

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
  # A0 R(theta) [Fx_k+1 - Fx_k] <= ubB0
  #             [Fy_k+1 - Fy_k]
  */
  nc_foot_ = nf_*MAL_VECTOR_SIZE(ubB0r_) ;
  MAL_MATRIX_RESIZE(Afoot_xy_   ,nc_foot_,2*(N_+nf_));
  MAL_MATRIX_RESIZE(Afoot_theta_,nc_foot_,nf_);
  MAL_MATRIX_RESIZE(AS_theta_   ,nc_foot_,2*(N_+nf_));
  MAL_VECTOR_RESIZE(UBfoot_     ,nc_foot_);
  MAL_VECTOR_RESIZE(LBfoot_     ,nc_foot_);

  MAL_MATRIX_RESIZE(SelecMat_,nf_,nf_);
  MAL_MATRIX_SET_IDENTITY(SelecMat_);
  for(unsigned i=1 ; i<SelecMat_.size1() ; ++i)
  {
    for(unsigned j=0 ; j<SelecMat_.size2() ; ++j)
    {
      if(i==j)
      {
        SelecMat_(i,j-1)=-1;
      }
    }
  }

  rotMat_vec_ .resize(nf_, tmpRotMat_ );
  drotMat_vec_.resize(nf_, tmpRotMat_ );

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

  MAL_VECTOR_RESIZE(SfootX_,nf_);
  MAL_VECTOR_RESIZE(SfootY_,nf_);
  MAL_VECTOR_FILL(SfootX_,0.0);
  MAL_VECTOR_FILL(SfootY_,0.0);

  return ;
}

void NMPCgenerator::updateFootPoseConstraint()
{
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
        support_state[n].Yaw = F_kp1_theta_(n-1);
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

  for(unsigned i=0 ; i<nf_ ; ++i)
  {
    rotMat_vec_[i](0,0)= cos(support_state[i].Yaw) ;
    rotMat_vec_[i](0,1)= sin(support_state[i].Yaw) ;
    rotMat_vec_[i](1,0)=-sin(support_state[i].Yaw) ;
    rotMat_vec_[i](1,1)= cos(support_state[i].Yaw) ;

    drotMat_vec_[i](0,0)=-sin(support_state[i].Yaw) ;
    drotMat_vec_[i](0,1)= cos(support_state[i].Yaw) ;
    drotMat_vec_[i](1,0)=-cos(support_state[i].Yaw) ;
    drotMat_vec_[i](1,1)=-sin(support_state[i].Yaw) ;
  }

  // every time instant in the pattern generator constraints
  // depend on the support order

  // reset matrix
  MAL_MATRIX_FILL(ASx_xy_ ,0.0);
  MAL_MATRIX_FILL(ASy_xy_ ,0.0);
  MAL_MATRIX_FILL(ASx_theta_,0.0);
  MAL_MATRIX_FILL(ASy_theta_,0.0);

  for(unsigned i=0 ; i<nf_ ; ++i)
  {
    if (support_state[i].Foot == LEFT)
    {
      A0f_xy_   [i] = MAL_RET_A_by_B(A0r_,rotMat_vec_[i]) ;
      A0f_theta_[i] = MAL_RET_A_by_B(A0r_,drotMat_vec_[i]) ;
      B0f_      [i] = ubB0r_ ;
    }else{
      A0f_xy_   [i] = MAL_RET_A_by_B(A0l_,rotMat_vec_[i]) ;
      A0f_theta_[i] = MAL_RET_A_by_B(A0l_,drotMat_vec_[i]) ;
      B0f_      [i] = ubB0l_ ;
    }
  }

  unsigned nbEdges = MAL_MATRIX_NB_ROWS(A0f_xy_[0]);
  for(unsigned n=0 ; n<nf_ ; ++n)
  {
    for (unsigned i=0 ; i<nbEdges ; ++i)
    {
      ASx_xy_(n*nbEdges+i,n) = A0f_xy_[n](i,0) ;
      ASy_xy_(n*nbEdges+i,n) = A0f_xy_[n](i,1) ;

      ASx_theta_(n*nbEdges+i,n) = A0f_theta_[n](i,0) ;
      ASy_theta_(n*nbEdges+i,n) = A0f_theta_[n](i,1) ;

      UBfoot_(n*nbEdges+i) = B0f_[n](i);
    }
  }
#ifdef DEBUG
  DumpVector("UBfoot_",UBfoot_);
  DumpMatrix("Ax_xy_",ASx_xy_);
  DumpMatrix("Ay_xy_",ASy_xy_);
#endif
#ifdef DEBUG_COUT
  cout << "ASx_xy_ = " << ASx_xy_ << endl ;
#endif
  MAL_VECTOR_FILL(SfootX_,0.0);
  MAL_VECTOR_FILL(SfootY_,0.0);
  SfootX_(0) = support_state[0].X ; SfootX_(1) = 0.0 ;
  SfootY_(0) = support_state[0].Y ; SfootY_(1) = 0.0 ;

  UBfoot_ = UBfoot_ +
      MAL_RET_A_by_B(ASx_xy_,SfootX_) +
      MAL_RET_A_by_B(ASy_xy_,SfootY_) ;

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
  --itBeforeLanding;

  double thresh_time_pos = 0.0 ;
  double thresh_time_rot = 0.0 ;
  if(itBeforeLanding < 2)
  {
    thresh_time_pos=1.0;
    thresh_time_rot=1.0;
  }
  Avel_(0, N_      ) = thresh_time_pos ;
  Avel_(1, 2*N_+nf_) = thresh_time_pos ;
  Avel_ (2,2*N_+2*nf_) = thresh_time_rot ;

  UBvel_(0) = F_kp1_x_(0) * thresh_time_pos ;
  UBvel_(1) = F_kp1_y_(0) * thresh_time_pos ;
  UBvel_(2) = F_kp1_theta_(0) * thresh_time_rot;

  LBvel_(0) = F_kp1_x_(0) * thresh_time_pos ;
  LBvel_(1) = F_kp1_y_(0) * thresh_time_pos ;
  LBvel_(2) = F_kp1_theta_(0) * thresh_time_rot;

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
  UBrot_(1) =  0.9 ;
  LBrot_(1) = -0.9 ;

}

void NMPCgenerator::updateRotIneqConstraint()
{
  UBrot_(0) =  0.9 + currentSupport_.Yaw ;
  LBrot_(0) = -0.9 + currentSupport_.Yaw ;
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
#ifdef DEBUG
    cout << "prepare the obstacle : " << obstacles_[obs] << endl ;
#endif
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

  MAL_MATRIX_RESIZE(qp_H_       ,nv_,nv_);        MAL_MATRIX_SET_IDENTITY(qp_H_);
  MAL_VECTOR_RESIZE(qp_g_       ,nv_);            MAL_VECTOR_FILL(qp_g_         ,0.0);
  MAL_VECTOR_RESIZE(qp_g_x_     ,N_+nf_);         MAL_VECTOR_FILL(qp_g_x_       ,0.0);
  MAL_VECTOR_RESIZE(qp_g_y_     ,N_+nf_);         MAL_VECTOR_FILL(qp_g_y_       ,0.0);
  MAL_VECTOR_RESIZE(qp_g_theta_ ,nf_);            MAL_VECTOR_FILL(qp_g_theta_   ,0.0);
  MAL_MATRIX_RESIZE(qp_J_       ,nc_,nv_);        MAL_MATRIX_FILL(qp_J_         ,0.0);
  MAL_MATRIX_RESIZE(qp_J_cop_   ,nc_cop_,nv_);    MAL_MATRIX_FILL(qp_J_cop_     ,0.0);
  MAL_MATRIX_RESIZE(qp_J_foot_  ,nc_foot_,nv_);   MAL_MATRIX_FILL(qp_J_foot_    ,0.0);
  MAL_MATRIX_RESIZE(qp_J_vel_   ,nc_vel_,nv_);    MAL_MATRIX_FILL(qp_J_vel_     ,0.0);
  MAL_MATRIX_RESIZE(qp_J_rot_   ,nc_rot_,nv_);    MAL_MATRIX_FILL(qp_J_rot_     ,0.0);
  MAL_VECTOR_RESIZE(qp_lbJ_     ,nc_);            MAL_VECTOR_FILL(qp_lbJ_       ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_     ,nc_);            MAL_VECTOR_FILL(qp_ubJ_       ,0.0);
  MAL_VECTOR_RESIZE(qp_lbJ_cop_ ,nc_cop_);        MAL_VECTOR_FILL(qp_lbJ_cop_   ,0.0);
  MAL_VECTOR_RESIZE(qp_lbJ_foot_,nc_foot_);       MAL_VECTOR_FILL(qp_lbJ_foot_  ,0.0);
  MAL_VECTOR_RESIZE(qp_lbJ_vel_ ,nc_vel_);        MAL_VECTOR_FILL(qp_lbJ_vel_   ,0.0);
  MAL_VECTOR_RESIZE(qp_lbJ_rot_ ,nc_rot_);        MAL_VECTOR_FILL(qp_lbJ_rot_   ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_cop_ ,nc_cop_);        MAL_VECTOR_FILL(qp_ubJ_cop_   ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_foot_,nc_foot_);       MAL_VECTOR_FILL(qp_ubJ_foot_  ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_vel_ ,nc_vel_);        MAL_VECTOR_FILL(qp_ubJ_vel_   ,0.0);
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


//  MAL_MATRIX_RESIZE(qp_J_  ,nc_,nv_);
//  MAL_VECTOR_RESIZE(qp_lbJ_,nc_);
//  MAL_VECTOR_RESIZE(qp_ubJ_,nc_);
//  MAL_MATRIX_FILL(qp_J_  ,0.0);
//  MAL_VECTOR_FILL(qp_lbJ_,0.0);
//  MAL_VECTOR_FILL(qp_ubJ_,0.0);

  // Q_q = 0.5 * a * ( 1 0 )
  //                 ( 0 1 )
  Q_theta_ *= alpha_ ;

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
  Q_x_XX_ = alpha_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pvu_),Pvu_)
          + beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_),Pzu_)
          + gamma_ * I_NN_ ;

  // number of constraint
  unsigned nc = nc_cop_+nc_foot_+nc_vel_+nc_rot_+nc_obs_+nc_stan_ ;
  if(nc_!=nc)
  {
    MAL_MATRIX_RESIZE(qp_J_, nc_,nv_);
    MAL_VECTOR_RESIZE(qp_lbJ_, nc_);
    MAL_VECTOR_RESIZE(qp_ubJ_, nc_);
  }
  nc_=nc;
  MAL_MATRIX_FILL(qp_J_, 0.0);
  MAL_VECTOR_FILL(qp_lbJ_, 0.0);
  MAL_VECTOR_FILL(qp_ubJ_, 0.0);

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
#ifdef DEBUG
  cout << vel_ref_.Global.X << " "
       << vel_ref_.Global.Y << endl;
#endif
  p_xy_X_ =    alpha_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pvu_  ), Pvsc_x_ - vel_ref_.Global.X_vec)
             + beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_  ), Pzsc_x_ - v_kp1f_x_                 );
#ifdef DEBUG
  DumpVector("Pvsc_x_"    , Pvsc_x_                    ) ;
  DumpVector("RefVectorX" , vel_ref_.Global.X_vec ) ;
  DumpVector("Pzsc_x_"    , Pzsc_x_                    ) ;
  DumpVector("v_kp1f_x_"  , v_kp1f_x_                  ) ;
#endif
  p_xy_Fx_ = - beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(V_kp1_), Pzsc_x_ - v_kp1f_x_                 );

  p_xy_Y_  =   alpha_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pvu_  ), Pvsc_y_ - vel_ref_.Global.Y_vec)
             + beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_  ), Pzsc_y_ - v_kp1f_y_                 );

  p_xy_Fy_ = - beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(V_kp1_), Pzsc_y_ - v_kp1f_y_                 );
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
  for(unsigned i=0 ; i<N_ ; ++i)
    p_x_(i) = p_xy_X_(i) ;
  for(unsigned i=0 ; i<nf_ ; ++i)
    p_x_(N_+i) = p_xy_Fx_(i) ;
  for(unsigned i=0 ; i<N_ ; ++i)
    p_y_(i) = p_xy_Y_(i) ;
  for(unsigned i=0 ; i<nf_ ; ++i)
    p_y_(N_+i) = p_xy_Fy_(i) ;


  // p_theta_ = ( 0.5 * a * [ f_k_theta+T_step*dTheta^ref  f_k_theta+2*T_step*dTheta^ref ] )
  p_theta_(0) = - alpha_ * ( currentSupport_.Yaw +     T_step_* vel_ref_.Global.Yaw) ;
  p_theta_(1) = - alpha_ * ( currentSupport_.Yaw + 2 * T_step_* vel_ref_.Global.Yaw) ;
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
  // qp_J_rot_ = Arot_
  qp_J_rot_ = Arot_ ;
  // qp_J_rot_ = Arot_
  qp_J_stan_ = Astan_ ;
  // qp_J_obs_ = (                 ...                  )
  //             (  Hobs_[obs][nf] * X + Aobs_[obs][nf] )
  //             (                 ...                  )
  MAL_MATRIX_RESIZE(qp_J_obs_   ,nc_obs_,nv_);
  MAL_MATRIX_FILL(qp_J_obs_     ,0.0);
  for(unsigned obs=0 ; obs<obstacles_.size() ; ++obs)
  {
    for(unsigned n=0 ; n<nf_ ; ++n)
    {
      qp_J_obs_i_ = 2 * MAL_RET_A_by_B(U_xy_,Hobs_[obs][n]) + Aobs_[obs][n] ;
      for(unsigned j=0 ; j<n_xy ; ++j)
        qp_J_obs_((obs+1)*n,j) = qp_J_obs_i_(j) ;
    }
  }
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
  //    Rotation
  qp_lbJ_rot_ = LBrot_ - MAL_RET_A_by_B(Arot_,U_);
  qp_ubJ_rot_ = UBrot_ - MAL_RET_A_by_B(Arot_,U_);
  //    Obstacle
  MAL_VECTOR_RESIZE(qp_lbJ_obs_ ,nc_obs_); MAL_VECTOR_FILL(qp_lbJ_obs_   ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_obs_ ,nc_obs_); MAL_VECTOR_FILL(qp_ubJ_obs_   ,0.0);
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
  // Standing
  MAL_VECTOR_RESIZE(qp_lbJ_stan_ ,nc_stan_); MAL_VECTOR_FILL(qp_lbJ_stan_   ,0.0);
  MAL_VECTOR_RESIZE(qp_ubJ_stan_ ,nc_stan_); MAL_VECTOR_FILL(qp_ubJ_stan_   ,0.0);
  qp_lbJ_stan_ = LBstan_ - MAL_RET_A_by_B(Astan_,U_) ;
  qp_ubJ_stan_ = UBstan_ - MAL_RET_A_by_B(Astan_,U_) ;


  // Fill up qp_lbJ_, qp_ubJ_ and qp_J_
//  MAL_MATRIX_RESIZE(qp_J_  ,nc_,nv_);
//  MAL_VECTOR_RESIZE(qp_lbJ_,nc_);
//  MAL_VECTOR_RESIZE(qp_ubJ_,nc_);
//  MAL_MATRIX_FILL(qp_J_  ,0.0);
//  MAL_VECTOR_FILL(qp_lbJ_,0.0);
//  MAL_VECTOR_FILL(qp_ubJ_,0.0);
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
  index += nc_rot_ ;
  for(unsigned i=0 ; i<nc_stan_ ; ++i)
  {
    qp_lbJ_(i+index) = qp_lbJ_stan_(i) ;
    qp_ubJ_(i+index) = qp_ubJ_stan_(i) ;
    for(unsigned j=0 ; j<nv_ ; ++j)
      qp_J_(i+index,j) = qp_J_stan_(i,j);
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
  MAL_VECTOR_FILL(vel_ref_.Global.X_vec   , vel_ref_.Global.X  ) ;
  MAL_VECTOR_FILL(vel_ref_.Global.Y_vec   , vel_ref_.Global.Y  ) ;
#ifdef DEBUG
  DumpVector("RefVectorX"    ,vel_ref_.Global.X_vec  );
  DumpVector("RefVectorY"    ,vel_ref_.Global.Y_vec  );
#endif
  return ;
}

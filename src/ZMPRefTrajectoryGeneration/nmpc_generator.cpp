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

using namespace std;
using namespace PatternGeneratorJRL;

NMPC_generator::NMPC_generator(SimplePluginManager * aSPM,CjrlHumanoidDynamicRobot * aHDR)
{
  T_ = 0.0 ;
  N_ = 0 ;
  nf_ = 0 ;
  T_step_ = 0.0 ;
  c_k_z_ = 0.0 ;
  global_vel_ref_.x = 0.0 ;
  global_vel_ref_.y = 0.0 ;
  global_vel_ref_.dYaw = 0.0 ;
  SecurityMarginX_ = 0.0 ;
  SecurityMarginY_ = 0.0 ;

  alpha_=0.0;
  beta_ =0.0;
  gamma_=0.0;

  MAL_VECTOR_RESIZE(c_k_x_,3);
  MAL_VECTOR_RESIZE(c_k_y_,3);
  MAL_MATRIX_RESIZE(Pps_,N_,3);
  MAL_MATRIX_RESIZE(Ppu_,N_,N_);
  MAL_MATRIX_RESIZE(Pvs_,N_,3);
  MAL_MATRIX_RESIZE(Pvu_,N_,N_);
  MAL_MATRIX_RESIZE(Pas_,N_,3);
  MAL_MATRIX_RESIZE(Pau_,N_,N_);
  MAL_MATRIX_RESIZE(Pzs_,N_,3);
  MAL_MATRIX_RESIZE(Pzu_,N_,N_);
  MAL_VECTOR_RESIZE(v_kp1_,N_) ;
  MAL_MATRIX_RESIZE(V_kp1_,N_,nf_) ;

  MAL_MATRIX_RESIZE(A0r_   ,5,2) ;
  MAL_VECTOR_RESIZE(ubB0r_ ,5) ;
  MAL_MATRIX_RESIZE(A0l_   ,5,2) ;
  MAL_VECTOR_RESIZE(ubB0l_ ,5) ;
  MAL_MATRIX_RESIZE(A0rf_  ,4,2) ;
  MAL_VECTOR_RESIZE(ubB0rf_,4) ;
  MAL_MATRIX_RESIZE(A0lf_  ,4,2) ;
  MAL_VECTOR_RESIZE(ubB0lf_,4) ;
  MAL_MATRIX_RESIZE(A0ds_  ,4,2) ;
  MAL_VECTOR_RESIZE(ubB0ds_,4) ;

  MAL_VECTOR_RESIZE( U_           , 2*N_+3*nf_);
  MAL_VECTOR_RESIZE( U_xy_        , 2*(N_+nf_));
  MAL_VECTOR_RESIZE( U_x_         , N_+nf_);
  MAL_VECTOR_RESIZE( U_y_         , N_+nf_);
  MAL_VECTOR_RESIZE( C_kp1_x_     , N_ );
  MAL_VECTOR_RESIZE( F_kp1_x_     , nf_);
  MAL_VECTOR_RESIZE( C_kp1_y_     , N_ );
  MAL_VECTOR_RESIZE( F_kp1_y_     , nf_);
  MAL_VECTOR_RESIZE( F_kp1_theta_ , nf_);

  SPM_ = aSPM ;
  HDR_ = aHDR ;
  RFI_=NULL;

  SupportStates_deq.clear();
  local_vel_ref_.clear();
}

void NMPC_generator::initNMPC_generator()
{
  N_ = 16 ;
  nf_ = 2 ;

  MAL_MATRIX_RESIZE(Pps_,N_,3);
  MAL_MATRIX_RESIZE(Ppu_,N_,N_);
  MAL_MATRIX_RESIZE(Pvs_,N_,3);
  MAL_MATRIX_RESIZE(Pvu_,N_,N_);
  MAL_MATRIX_RESIZE(Pas_,N_,3);
  MAL_MATRIX_RESIZE(Pau_,N_,N_);
  MAL_MATRIX_RESIZE(Pzs_,N_,3);
  MAL_MATRIX_RESIZE(Pzu_,N_,N_);
  MAL_VECTOR_RESIZE(v_kp1_,N_) ;
  MAL_MATRIX_RESIZE(V_kp1_,N_,nf_) ;
  MAL_VECTOR_RESIZE(U_           , 2*N_+3*nf_);
  MAL_VECTOR_RESIZE(U_xy_        , 2*(N_+nf_));
  MAL_VECTOR_RESIZE(U_x_         , N_+nf_);
  MAL_VECTOR_RESIZE(U_y_         , N_+nf_);
  MAL_VECTOR_RESIZE(C_kp1_x_     , N_ );
  MAL_VECTOR_RESIZE(F_kp1_x_     , nf_);
  MAL_VECTOR_RESIZE(C_kp1_y_     , N_ );
  MAL_VECTOR_RESIZE(F_kp1_y_     , nf_);
  MAL_VECTOR_RESIZE(F_kp1_theta_ , nf_);
  MAL_VECTOR_RESIZE(c_k_x_,3);
  MAL_VECTOR_RESIZE(c_k_y_,3);

  T_ = 0.1 ;
  T_step_ = 0.8 ;
  local_vel_ref_.resize(3);
  local_vel_ref_[0] = 0.2 ;
  local_vel_ref_[1] = 0.0 ;
  local_vel_ref_[2] = 0.02 ;
  setVelocityReference(local_vel_ref_);

  MAL_VECTOR_FILL(c_k_x_,0.0);
  MAL_VECTOR_FILL(c_k_y_,0.0);
  c_k_x_(0) = 0.00949035 ;
  c_k_y_(0) = 0.095 ;
  c_k_z_ = 0.814 ;

  alpha_ = 1.0   ; // weight for CoM velocity tracking  : 0.5 * a
  beta_  = 1e+03 ; // weight for ZMP reference tracking : 0.5 * b
  gamma_ = 5e-04 ; // weight for jerk minimization      : 0.5 * c

  // start with Left foot as support
  // the right foot start moving
  currentSupport.Foot=LEFT;
  currentSupport.NbStepsLeft=2;
  currentSupport.StartTime=0.0;
  currentSupport.TimeLimit=0.0;
  currentSupport.X=0.00949035;
  currentSupport.Y=0.095;
  currentSupport.Yaw=0.0;
  currentSupport.StepNumber=0;

  SupportStates_deq.resize(N_,currentSupport);

  SecurityMarginX_ = 0.09 ;
  SecurityMarginY_ = 0.05 ;

  MAL_MATRIX_FILL(Pps_,0.0);
  MAL_MATRIX_FILL(Ppu_,0.0);
  MAL_MATRIX_FILL(Pvs_,0.0);
  MAL_MATRIX_FILL(Pvu_,0.0);
  MAL_MATRIX_FILL(Pas_,0.0);
  MAL_MATRIX_FILL(Pau_,0.0);
  MAL_MATRIX_FILL(Pzs_,0.0);
  MAL_MATRIX_FILL(Pzu_,0.0);
  MAL_VECTOR_FILL(v_kp1_,0.0) ;
  MAL_MATRIX_FILL(V_kp1_,0.0) ;
  MAL_VECTOR_RESIZE(U_          ,0.0);
  MAL_VECTOR_RESIZE(U_xy_       ,0.0);
  MAL_VECTOR_RESIZE(U_x_        ,0.0);
  MAL_VECTOR_RESIZE(U_y_        ,0.0);
  MAL_VECTOR_RESIZE(C_kp1_x_    ,0.0);
  MAL_VECTOR_RESIZE(F_kp1_x_    ,0.0);
  MAL_VECTOR_RESIZE(C_kp1_y_    ,0.0);
  MAL_VECTOR_RESIZE(F_kp1_y_    ,0.0);
  MAL_VECTOR_RESIZE(F_kp1_theta_,0.0);

  RFI_ = new RelativeFeetInequalities(SPM_,HDR_) ;
  ostringstream oss(std::ostringstream::ate);
  oss << ":setfeetconstraint XY " << SecurityMarginX_ << " " << SecurityMarginY_ ;
  cout << oss.str() << endl ;
  istringstream strm(oss.str());
  string cmd ;
  strm >> cmd ;
  RFI_->CallMethod(cmd,strm);

  MAL_MATRIX_FILL(A0r_   ,0.0) ;
  MAL_VECTOR_FILL(ubB0r_ ,0.0) ;
  MAL_MATRIX_FILL(A0l_   ,0.0) ;
  MAL_VECTOR_FILL(ubB0l_ ,0.0) ;
  MAL_MATRIX_FILL(A0rf_  ,0.0) ;
  MAL_VECTOR_FILL(ubB0rf_,0.0) ;
  MAL_MATRIX_FILL(A0lf_  ,0.0) ;
  MAL_VECTOR_FILL(ubB0lf_,0.0) ;
  MAL_MATRIX_FILL(A0ds_  ,0.0) ;
  MAL_VECTOR_FILL(ubB0ds_,0.0) ;

  buildConstantMatrix();
  initializeTimeVariantMatrix();
}

void NMPC_generator::updateNMPC_generator()
{
  updateTimeVariantMatrix();
  //computeInitialGuess();
  return ;
}

void NMPC_generator::buildConstantMatrix()
{
  buildCoMIntegrationMatrix();
  buildCoPIntegrationMatrix();
  buildConvexHullSystems();
  return ;
}

void NMPC_generator::initializeTimeVariantMatrix()
{
  initializeFootSelectionMatrix();
  initializeCoPConstraint();
  initializeFootPoseConstraint();
  initializeFootVelIneqConstraint();
  initializeRotIneqConstraint();
  initializeObstacleConstraint();
  initializeCostFunction();
  return;
}

void NMPC_generator::updateTimeVariantMatrix()
{
  updateFootSelectionMatrix();
  updateCoPConstraint();
  updateFootPoseConstraint();
  updateFootVelIneqConstraint();
  updateRotIneqConstraint();
  updateObstacleConstraint();
  updateCostFunction();
  return;
}

void NMPC_generator::initializeFootSelectionMatrix()
{
  unsigned nStep = (unsigned)round(T_step_ / T_) ;
  for (unsigned i = 0 ; i < nStep ; ++i )
  {
    v_kp1_(i) = 1.0 ;
  }

  for (unsigned j = 0 ; j < nf_ ; ++j )
  {
    unsigned i_min = min((j+1)*nStep,N_);
    unsigned i_max = min((j+2)*nStep,N_);
    for (unsigned i = i_min ; i < i_max ; ++i )
    {
      V_kp1_(i,j) = 1.0 ;
    }
  }
  //cout << v_kp1_ << endl << V_kp1_ << endl ;
  computeSupportOrder();
  return ;
}

void NMPC_generator::updateFootSelectionMatrix()
{
  unsigned nStep = (unsigned)round(T_step_ / T_) ;
  double v_kp1_0 = v_kp1_(0) ;
  for (unsigned i = 0 ; i < nStep ; ++i )
  {
    v_kp1_(i) = v_kp1_(i+1) ;
  }

  for (unsigned i = 0 ; i < N_-1 ; ++i )
  {
    for (unsigned j = 0 ; j < nf_ ; ++j )
    {
      V_kp1_(i,j) = V_kp1_(i+1,j) ;
    }
  }

  for (unsigned j = 0 ; j < nf_ ; ++j )
    V_kp1_(MAL_MATRIX_NB_ROWS(V_kp1_)-1,j) = 0.0 ;
  V_kp1_(MAL_MATRIX_NB_ROWS(V_kp1_)-1,MAL_MATRIX_NB_COLS(V_kp1_)-1) = v_kp1_0 ;

  bool firstRowOfZeros = (v_kp1_(0) == 0.0) ;
  for (unsigned i = 1 ; i < nStep ; ++i )
  {
    firstRowOfZeros = firstRowOfZeros && (v_kp1_(i) == 0.0) ;
  }

  if(firstRowOfZeros)
  {
    for (unsigned i = 0 ; i < N_ ; ++i )
      v_kp1_(i)=V_kp1_(i,0) ;

    for (unsigned i = 0 ; i < N_ ; ++i )
      for (unsigned j = 0 ; j < nf_-1 ; ++j )
        V_kp1_(i,j) = V_kp1_(i,j+1) ;

    for (unsigned i = 0 ; i < N_ ; ++i )
      V_kp1_(i,MAL_MATRIX_NB_COLS(V_kp1_)-1) = 0.0 ;

    // update the current support in the mean time
    if(currentSupport.Foot==LEFT)
      currentSupport.Foot=RIGHT;
    else
      currentSupport.Foot=LEFT;
    ++currentSupport.StepNumber;
    if( (global_vel_ref_.x*global_vel_ref_.x +
         global_vel_ref_.y*global_vel_ref_.y +
         global_vel_ref_.dYaw*global_vel_ref_.dYaw) < 0.0001 )
    {
      if (currentSupport.NbStepsLeft == 0)
        currentSupport.NbStepsLeft = 0 ;
      else
        --currentSupport.NbStepsLeft ;
    }
    else
      currentSupport.NbStepsLeft = 2 ;

  }
  //cout << v_kp1_ << endl << V_kp1_ << endl ;
  computeSupportOrder();
  return ;
}

void NMPC_generator::computeSupportOrder()
{
  for (unsigned i = 0 ; i < N_ ; ++i)
  {
    // at the first iteration the support foot is the current one
    if(v_kp1_(i)==1.0)
      {SupportStates_deq[i]=currentSupport;}

    foot_type_e odd,even;
    if(currentSupport.Foot==LEFT)
      {odd = LEFT ; even = RIGHT ;}
    else
      {odd = RIGHT ; even = LEFT ;}
    for (unsigned j=0 ; j<nf_ ; ++j)
    {
      if(V_kp1_(i,j)==1.0)
      {
        SupportStates_deq[i].StepNumber = j+1 ;
        if((j%2) == 1)
          SupportStates_deq[i].Foot = odd ;
        else
          SupportStates_deq[i].Foot = even ;
      }
    }
  }

//  for (unsigned i = 0 ; i < N_ ; ++i)
//  {
//    cout << SupportStates_deq[i].Foot << " ; " ;
//  }
//  cout << endl ;

  return ;
}

void NMPC_generator::buildCoMIntegrationMatrix()
{
  for (unsigned i = 0 , k = 1 ; i < N_ ; ++i , k=i+1)
  {
    Pps_(i,0) = 1.0 ; Pps_(i,1) = k*T_ ; Pps_(i,2) = k*k*T_*T_*0.5 ;
    Pvs_(i,0) = 0.0 ; Pvs_(i,1) = 1.0  ; Pvs_(i,2) = k*T_          ;
    Pas_(i,0) = 0.0 ; Pas_(i,1) = 0.0  ; Pas_(i,2) = 1.0           ;

    for (unsigned j = 0 ; j <= i ; ++j)
    {
      Ppu_(i,j) = (3.0*(i-j)*(i-j) + 3.0*(i-j)+1.0)*T_*T_*T_/6.0 ;
      Pvu_(i,j) = (2.0*(i-j) + 1.0)*T_*T_*0.5 ;
      Pau_(i,j) = T_ ;
    }
  }
  //cout << Pps_ << endl << Ppu_ << endl ;
  //cout << Pvs_ << endl << Pvu_ << endl ;
  //cout << Pas_ << endl << Pau_ << endl ;
  return ;
}

void NMPC_generator::buildCoPIntegrationMatrix()
{
  const double GRAVITY = 9.81;
  for (unsigned i = 0 , k = 1 ; i < N_ ; ++i , k=i+1)
  {
    Pzs_(i,0) = 1.0 ; Pzs_(i,1) = k*T_ ; Pzs_(i,2) = k*k*T_*T_*0.5 - c_k_z_/GRAVITY ;

    for (unsigned j = 0 ; j <= i ; ++j)
    {
      Pzu_(i,j) = (3.0*(i-j)*(i-j) + 3.0*(i-j)+1.0)*T_*T_*T_/6.0 - T_*c_k_z_/GRAVITY ;
    }
  }
  //cout << Pzs_ << endl << Pzu_ << endl ;
  return ;
}

void NMPC_generator::buildConvexHullSystems()
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

//  cout << A0rf_ << " " << ubB0rf_ << endl
//       << A0lf_ << " " << ubB0lf_ << endl
//       << A0ds_ << " " << ubB0ds_ << endl ;

  // Polygonal hulls for feasible foot placement
  ///////////////////////////////////////////////////////
  nbEdges = 5 ;
  nbIneq = 5 ;
  hull = convex_hull_t(nbEdges, nbIneq);
  // RIGHT FOOT
  dummySupp.Foot = LEFT ;
  dummySupp.Phase = SS ;
  RFI_->set_vertices( hull, dummySupp, INEQ_FEET );
  RFI_->compute_linear_system( hull, dummySupp );
  for(unsigned i = 0 ; i < hull.A_vec.size() ; ++i)
  {
    A0r_(i,0) = -hull.A_vec[i] ;
    A0r_(i,1) = -hull.B_vec[i] ;
    ubB0r_(i) = -hull.D_vec[i] ;
  }
  // LEFT FOOT
  dummySupp.Foot = RIGHT ;
  dummySupp.Phase = SS ;
  RFI_->set_vertices( hull, dummySupp, INEQ_FEET );
  RFI_->compute_linear_system( hull, dummySupp );
  for(unsigned i = 0 ; i < hull.A_vec.size() ; ++i)
  {
    A0l_(i,0) = -hull.A_vec[i] ;
    A0l_(i,1) = -hull.B_vec[i] ;
    ubB0l_(i) = -hull.D_vec[i] ;
  }

//  cout << endl ;
//  cout << A0r_ << " " << ubB0r_ << endl
//       << A0l_ << " " << ubB0l_ << endl ;


  return ;
}

void NMPC_generator::setVelocityReference(std::vector<double> local_vel_ref)
{
  local_vel_ref_ = local_vel_ref ;
  global_vel_ref_.x = local_vel_ref[0] * cos(currentSupport.Yaw) - local_vel_ref[1] * sin(currentSupport.Yaw) ;
  global_vel_ref_.y = local_vel_ref[0] * sin(currentSupport.Yaw) + local_vel_ref[1] * cos(currentSupport.Yaw) ;
  global_vel_ref_.dYaw = local_vel_ref[2] ;
  MAL_VECTOR_RESIZE(global_vel_ref_.RefVectorX,N_) ;
  MAL_VECTOR_RESIZE(global_vel_ref_.RefVectorY,N_) ;
  MAL_VECTOR_RESIZE(global_vel_ref_.RefVectorTheta,N_) ;

  MAL_VECTOR_FILL(global_vel_ref_.RefVectorX    , global_vel_ref_.x   ) ;
  MAL_VECTOR_FILL(global_vel_ref_.RefVectorY    , global_vel_ref_.y   ) ;
  MAL_VECTOR_FILL(global_vel_ref_.RefVectorTheta, global_vel_ref_.dYaw) ;

  return ;
}

void NMPC_generator::initializeCoPConstraint()
{
  /**
  build linear inequality constraints to keep the CoP
  inside the support polygone

  NOTE: needs actual SupportStates_deq to work properly
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

  //cout << derv_Acop_map_ << endl ;

  updateCoPConstraint();

  return ;
}

void NMPC_generator::updateCoPConstraint()
{
  // Compute D_kp1_, it depends on the feet hulls
  vector<double>theta_vec(3);
  theta_vec[0]=currentSupport.Yaw;
  theta_vec[1]=F_kp1_theta_(0);
  theta_vec[2]=F_kp1_theta_(1);
  // every time instant in the pattern generator constraints
  // depend on the support order
  for (unsigned i=0 ; i<N_ ; ++i)
  {
    double theta = theta_vec[SupportStates_deq[i].StepNumber] ;
    MAL_MATRIX_DIM(rotMat_xy,double,2,2);
    MAL_MATRIX_DIM(rotMat_theta,double,2,2);
    rotMat_xy(0,0)= cos(theta) ; rotMat_xy(0,1)= sin(theta) ;
    rotMat_xy(1,0)=-sin(theta) ; rotMat_xy(1,1)= cos(theta) ;
    rotMat_theta(0,0)=-sin(theta) ; rotMat_theta(0,1)= cos(theta) ;
    rotMat_theta(1,0)=-cos(theta) ; rotMat_theta(1,1)=-sin(theta) ;

    MAL_MATRIX_TYPE(double) A0_xy, A0_theta;
    MAL_VECTOR_TYPE(double) B0;
    if (SupportStates_deq[i].NbStepsLeft == 0)
    {
      A0_xy    = MAL_RET_A_by_B(A0ds_,rotMat_xy   ) ;
      A0_theta = MAL_RET_A_by_B(A0ds_,rotMat_theta) ;
      B0 = ubB0ds_ ;
    }
    else if (SupportStates_deq[i].Foot == LEFT)
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
  v_kp1f_x_ = v_kp1_ * currentSupport.X ;
  v_kp1f_y_ = v_kp1_ * currentSupport.Y ;
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
  return ;
}

void NMPC_generator::initializeFootPoseConstraint()
{
  /**
  build linear inequality constraints for the placement of the feet

  NOTE: needs actual SupportStates_deq to work properly
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

  MAL_MATRIX_FILL(ASx_xy_ ,0.0);
  MAL_MATRIX_FILL(ASy_xy_ ,0.0);
  MAL_MATRIX_FILL(ASx_theta_,0.0);
  MAL_MATRIX_FILL(ASy_theta_,0.0);

  MAL_MATRIX_RESIZE(derv_Afoot_map_,nc_foot_,N_);

  updateFootPoseConstraint() ;

  return ;
}

void NMPC_generator::updateFootPoseConstraint()
{
  // rotation matrice from F_k+1 to F_k
  rotMat1_(0,0)= cos(currentSupport.Yaw) ; rotMat1_(0,1)= sin(currentSupport.Yaw) ;
  rotMat1_(1,0)=-sin(currentSupport.Yaw) ; rotMat1_(1,1)= cos(currentSupport.Yaw) ;
  rotMat2_(0,0)= cos(F_kp1_theta_(0))    ; rotMat2_(0,1)= sin(F_kp1_theta_(0)) ;
  rotMat2_(1,0)=-sin(F_kp1_theta_(0))    ; rotMat2_(1,1)= cos(F_kp1_theta_(0)) ;

  drotMat1_(0,0)=-sin(currentSupport.Yaw) ; drotMat1_(0,1)= cos(currentSupport.Yaw) ;
  drotMat1_(1,0)=-cos(currentSupport.Yaw) ; drotMat1_(1,1)=-sin(currentSupport.Yaw) ;
  drotMat2_(0,0)=-sin(F_kp1_theta_(0))    ; drotMat2_(0,1)= cos(F_kp1_theta_(0)) ;
  drotMat2_(1,0)=-cos(F_kp1_theta_(0))    ; drotMat2_(1,1)=-sin(F_kp1_theta_(0)) ;

  // every time instant in the pattern generator constraints
  // depend on the support order
  MAL_MATRIX_TYPE(double) Af1_xy, Af1_theta, Af2_xy, Af2_theta ;
  MAL_VECTOR_TYPE(double) Bf1 , Bf2 ;
  if (currentSupport.Foot == LEFT)
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
  SfootX(0) = currentSupport.X ; SfootX(1) = 0.0 ;
  MAL_VECTOR_DIM(SfootY,double,2);
  SfootY(0) = currentSupport.Y ; SfootY(1) = 0.0 ;

  UBfoot_ = UBfoot_ + MAL_RET_A_by_B(ASx_xy_,SfootX)  + MAL_RET_A_by_B(ASy_xy_,SfootY) ;
  DumpVector("UBfoot_",UBfoot_);

  ASx_xy_    = MAL_RET_A_by_B(ASx_xy_   ,SelecMat_);
  ASy_xy_    = MAL_RET_A_by_B(ASy_xy_   ,SelecMat_);
  ASx_theta_ = MAL_RET_A_by_B(ASx_theta_,SelecMat_);
  ASy_theta_ = MAL_RET_A_by_B(ASy_theta_,SelecMat_);

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

  return ;
}

void NMPC_generator::initializeFootVelIneqConstraint()
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

void NMPC_generator::updateFootVelIneqConstraint()
{
  unsigned itBeforeLanding = 0;
  for(unsigned i=0 ; i<MAL_VECTOR_SIZE(v_kp1_) ; ++i)
    if(v_kp1_(i))
      ++itBeforeLanding;
    else
      break;

  double dt = (double)itBeforeLanding * T_ ;// (t_touchdown - t)

  double vref_x = global_vel_ref_.x ;
  double vref_y = global_vel_ref_.y ;
  double norm_vref = sqrt(vref_x*vref_x + vref_y*vref_y ) ;
  double dyaw = global_vel_ref_.dYaw ;
  double signq = dyaw>=0.0?1:-1;

  double xvmax(0.4), yvmax(0.4), yawvmax(1.5) ;// [m/s,m/s,rad/s]

  UBvel_(0) = (dt+T_) * xvmax + vref_x * F_kp1_x_(0) / norm_vref ;
  UBvel_(1) = (dt+T_) * yvmax + vref_y * F_kp1_y_(0) / norm_vref ;

  Avel_(0, N_      ) = vref_x / norm_vref ;
  Avel_(1, 2*N_+nf_) = vref_y / norm_vref ;

  Avel_ (2,2*N_+2*nf_) = signq ;
  UBvel_(2) = (dt+T_) * yawvmax + signq * F_kp1_theta_(0) ;
  return ;
}

void NMPC_generator::initializeRotIneqConstraint()
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

void NMPC_generator::updateRotIneqConstraint()
{
  UBrot_(0) =  0.09 + currentSupport.Yaw ;
  LBrot_(0) = -0.09 + currentSupport.Yaw ;
  return;
}

void NMPC_generator::initializeObstacleConstraint()
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
  updateObstacleConstraint();
}

void NMPC_generator::updateObstacleConstraint()
{
  Circle obstacle ;
  obstacle.x_0    = -1.0 ;
  obstacle.y_0    = 0.3 ;
  obstacle.r      = 0.5/2 ;
  obstacle.margin = 0.4172 ;
  obstacles_.push_back(obstacle);

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
  return ;
}

void NMPC_generator::initializeCostFunction()
{
  // number of degrees of freedom
  nv_ = 2*N_+3*nf_;
  // number of constraint
  nc_ = nc_cop_+nc_foot_+nc_vel_+nc_rot_+nc_obs_ ;

  MAL_MATRIX_RESIZE(qp_H_       ,nv_,nv_);
  MAL_VECTOR_RESIZE(qp_g_       ,nv_);
  MAL_VECTOR_RESIZE(qp_g_x_     ,N_+nf_);
  MAL_VECTOR_RESIZE(qp_g_y_     ,N_+nf_);
  MAL_VECTOR_RESIZE(qp_g_theta_ ,nf_);
  MAL_MATRIX_RESIZE(qp_J_       ,nc_,nv_);
  MAL_MATRIX_RESIZE(qp_J_cop_   ,nc_cop_,nv_);
  MAL_MATRIX_RESIZE(qp_J_foot_  ,nc_foot_,nv_);
  MAL_MATRIX_RESIZE(qp_J_vel_   ,nc_vel_,nv_);
  MAL_MATRIX_RESIZE(qp_J_obs_   ,nc_obs_,nv_);
  MAL_MATRIX_RESIZE(qp_J_rot_   ,nc_rot_,nv_);
  MAL_VECTOR_RESIZE(qp_lbJ_     ,nc_);
  MAL_VECTOR_RESIZE(qp_ubJ_     ,nc_);
  MAL_VECTOR_RESIZE(qp_lbJ_cop_ ,nc_cop_);
  MAL_VECTOR_RESIZE(qp_lbJ_foot_,nc_foot_);
  MAL_VECTOR_RESIZE(qp_lbJ_vel_ ,nc_vel_);
  MAL_VECTOR_RESIZE(qp_lbJ_obs_ ,nc_obs_);
  MAL_VECTOR_RESIZE(qp_lbJ_rot_ ,nc_rot_);
  MAL_VECTOR_RESIZE(qp_ubJ_cop_ ,nc_cop_);
  MAL_VECTOR_RESIZE(qp_ubJ_foot_,nc_foot_);
  MAL_VECTOR_RESIZE(qp_ubJ_vel_ ,nc_vel_);
  MAL_VECTOR_RESIZE(qp_ubJ_obs_ ,nc_obs_);
  MAL_VECTOR_RESIZE(qp_ubJ_rot_ ,nc_rot_);
  MAL_VECTOR_RESIZE(qp_lb_      ,nv_);
  MAL_VECTOR_RESIZE(qp_ub_      ,nv_);
  MAL_MATRIX_RESIZE(Q_x_        ,N_+nf_, N_+nf_);
  MAL_MATRIX_RESIZE(Q_x_XX_     ,N_,N_);
  MAL_MATRIX_RESIZE(Q_x_XF_     ,N_,nf_);
  MAL_MATRIX_RESIZE(Q_x_FX_     ,nf_,N_);
  MAL_MATRIX_RESIZE(Q_x_FF_     ,nf_,nf_);
  MAL_MATRIX_RESIZE(Q_theta_    ,nf_,nf_);
  MAL_VECTOR_RESIZE(p_x_        ,N_+nf_);
  MAL_VECTOR_RESIZE(p_y_        ,N_+nf_);
  MAL_VECTOR_RESIZE(p_xy_X_     ,N_);
  MAL_VECTOR_RESIZE(p_xy_Fx_    ,nf_);
  MAL_VECTOR_RESIZE(p_xy_Y_     ,N_);
  MAL_VECTOR_RESIZE(p_xy_Fy_    ,nf_);
  MAL_VECTOR_RESIZE(p_theta_    ,nf_);
  MAL_MATRIX_RESIZE(I_NN_       ,N_,N_);
  MAL_VECTOR_RESIZE(Pvsc_x_     ,N_);
  MAL_VECTOR_RESIZE(Pvsc_y_     ,N_);

  MAL_MATRIX_SET_IDENTITY(qp_H_);
  MAL_MATRIX_SET_IDENTITY(I_NN_);
  MAL_VECTOR_FILL(qp_g_         ,0.0);
  MAL_VECTOR_FILL(qp_g_x_       ,0.0);
  MAL_VECTOR_FILL(qp_g_y_       ,0.0);
  MAL_VECTOR_FILL(qp_g_theta_   ,0.0);
  MAL_MATRIX_FILL(qp_J_         ,0.0);
  MAL_MATRIX_FILL(qp_J_cop_     ,0.0);
  MAL_MATRIX_FILL(qp_J_foot_    ,0.0);
  MAL_MATRIX_FILL(qp_J_vel_     ,0.0);
  MAL_MATRIX_FILL(qp_J_obs_     ,0.0);
  MAL_MATRIX_FILL(qp_J_rot_     ,0.0);
  MAL_VECTOR_FILL(qp_lbJ_       ,0.0);
  MAL_VECTOR_FILL(qp_ubJ_       ,0.0);
  MAL_VECTOR_FILL(qp_lbJ_cop_   ,0.0);
  MAL_VECTOR_FILL(qp_lbJ_foot_  ,0.0);
  MAL_VECTOR_FILL(qp_lbJ_vel_   ,0.0);
  MAL_VECTOR_FILL(qp_lbJ_obs_   ,0.0);
  MAL_VECTOR_FILL(qp_lbJ_rot_   ,0.0);
  MAL_VECTOR_FILL(qp_ubJ_cop_   ,0.0);
  MAL_VECTOR_FILL(qp_ubJ_foot_  ,0.0);
  MAL_VECTOR_FILL(qp_ubJ_vel_   ,0.0);
  MAL_VECTOR_FILL(qp_ubJ_obs_   ,0.0);
  MAL_VECTOR_FILL(qp_ubJ_rot_   ,0.0);
  MAL_VECTOR_FILL(qp_lb_        ,-1e+8);
  MAL_VECTOR_FILL(qp_ub_        , 1e+8);
  MAL_MATRIX_FILL(Q_x_          ,0.0);
  MAL_MATRIX_FILL(Q_x_XX_       ,0.0);
  MAL_MATRIX_FILL(Q_x_XF_       ,0.0);
  MAL_MATRIX_FILL(Q_x_FX_       ,0.0);
  MAL_MATRIX_FILL(Q_x_FF_       ,0.0);
  MAL_MATRIX_SET_IDENTITY(Q_theta_);
  MAL_VECTOR_FILL(p_x_          , 0.0);
  MAL_VECTOR_FILL(p_y_          , 0.0);
  MAL_VECTOR_FILL(p_xy_X_       , 0.0);
  MAL_VECTOR_FILL(p_xy_Fx_      , 0.0);
  MAL_VECTOR_FILL(p_xy_Y_       , 0.0);
  MAL_VECTOR_FILL(p_xy_Fy_      , 0.0);
  MAL_VECTOR_FILL(p_theta_      , 0.0);
  MAL_VECTOR_FILL(Pvsc_x_       , 0.0);
  MAL_VECTOR_FILL(Pvsc_y_       , 0.0);

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

void NMPC_generator::updateCostFunction()
{
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
  p_xy_X_ =    alpha_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pvu_  ), Pvsc_x_ - global_vel_ref_.RefVectorX)
             + beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pzu_  ), Pzsc_x_ - v_kp1f_x_                 );
  p_xy_Fx_ = - beta_  * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(V_kp1_), Pzsc_x_ - v_kp1f_x_                 );
  p_xy_Y_  =   alpha_ * MAL_RET_A_by_B(MAL_RET_TRANSPOSE(Pvu_  ), Pvsc_y_ - global_vel_ref_.RefVectorY)
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
  p_theta_(0) = - (alpha_ * currentSupport.Yaw +     T_step_* global_vel_ref_.dYaw) ;
  p_theta_(1) = - (alpha_ * currentSupport.Yaw + 2 * T_step_* global_vel_ref_.dYaw) ;

//  DumpMatrix("Q_theta_",Q_theta_);
//  DumpMatrix("Q_x_"    ,Q_x_    );
//  DumpVector("p_x_"   ,p_x_   );
//  DumpVector("p_y_"   ,p_y_   );
//  DumpVector("p_theta_",p_theta_);

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
      qp_H_(i+2*n_x,j+2*n_x)=Q_theta_(i,j) ;

  // Gradient of Objective
  // qp_g_ = (gx)
  //         (gx)
  //         (gq)
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
      qp_J_obs_i = 2* MAL_RET_A_by_B(Hobs_[obs][n],U_xy_) + Aobs_[obs][n] ;
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
  qp_lbJ_vel_ = LBvel_ - MAL_RET_A_by_B(Avel_,U_xy_);
  qp_ubJ_vel_ = UBvel_ - MAL_RET_A_by_B(Avel_,U_xy_);
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
  qp_lbJ_rot_ = LBcop_ - MAL_RET_A_by_B(Acop_xy_,U_xy_);
  qp_ubJ_rot_ = UBcop_ - MAL_RET_A_by_B(Acop_xy_,U_xy_);


//  , qp_lbJ_foot_, qp_lbJ_vel_, qp_lbJ_obs_, qp_lbJ_rot_
//  , qp_ubJ_foot_, qp_ubJ_vel_, qp_ubJ_obs_, qp_ubJ_rot_


  // Fill up qp_lbJ_, qp_ubJ_ and qp_J_
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

  DumpMatrix("qp_H_",qp_H_);
  DumpMatrix("qp_J_",qp_J_);
  DumpVector("qp_lbJ_",qp_lbJ_);
  DumpVector("qp_ubJ_",qp_ubJ_);
  return ;
}






































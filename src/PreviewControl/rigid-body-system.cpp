/*
 * Copyright 2011
 *
 * Andrei Herdt
 *
 * JRL, CNRS/AIST, INRIA Grenoble-Rhone-Alpes
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
 */

/// Simulate a rigid body system

#include <PreviewControl/rigid-body-system.hh>

using namespace PatternGeneratorJRL;
using namespace std;

RigidBodySystem::
RigidBodySystem
( SimplePluginManager * SPM,
  PinocchioRobot * aPR,
  SupportFSM * FSM ):
  mass_(0),CoMHeight_(0),T_(0),Tr_(0),Ta_(0),N_(0),multiBody_(false),
  OFTG_(0), FSM_(0)
{
  PR_ = aPR;
  FSM_ = FSM;
  OFTG_ = new OnLineFootTrajectoryGeneration(SPM,PR_->leftFoot());
}


RigidBodySystem::~RigidBodySystem()
{
  if (OFTG_!=0)
    delete OFTG_;
}


void
RigidBodySystem::initialize(  )
{

  // Create and initialize online interpolation of feet trajectories:
  // ----------------------------------------------------------------
  OFTG_->InitializeInternalDataStructures();
  OFTG_->SetSingleSupportTime( 0.7 );
  OFTG_->SetDoubleSupportTime( T_ );
  OFTG_->QPSamplingPeriod( T_ );
  OFTG_->NbSamplingsPreviewed( N_ );
  OFTG_->FeetDistance( 0.2 ); // HRP-2
  //OFTG_->FeetDistance( 0.162 );
  OFTG_->SetStepHeight( 0.03 );

  // Initialize predetermined trajectories:
  // --------------------------------------
  initialize_trajectories();
  LeftFoot_.NbSamplingsPreviewed( N_ );
  LeftFoot_.Trajectory().resize( N_ );
  LeftFoot_.initialize();
  RightFoot_.NbSamplingsPreviewed( N_ );
  RightFoot_.Trajectory().resize( N_ );
  RightFoot_.initialize();

  GRF_deq_.resize( N_ );

  // Resize dynamics matrices:
  // -------------------------
  // TODO: Use of bounded arrays
  // TODO: Remove CoP dynamics
  if(multiBody_)
    {
      LeftFoot_.Dynamics ( COP_POSITION ).S.resize(N_,3);
      RightFoot_.Dynamics( COP_POSITION ).S.resize(N_,3);
      LeftFoot_.Dynamics ( COP_POSITION ).clear();
      RightFoot_.Dynamics( COP_POSITION ).clear();
    }

  CoPDynamicsJerk_.Type = COP_POSITION;

  // Initialize dynamics
  // -------------------
  CoM_.Mass( mass_ );
  CoM_.NbSamplingsPreviewed( N_ );
  CoM_.initialize();
  compute_dyn_cjerk();

}


int
RigidBodySystem::initialize_trajectories()
{

  // Vertical foot trajectory of a stance phase
  // starting from the beginning of the simple support phase
  // and ending at the end of the double support phase.
  // The trajectory is divided in two order 5 polynomials:
  // -----------------------------------------------------
  double SSPeriod = FSM_->StepPeriod()-T_;
  unsigned int NbInstantsSS = (unsigned int)(SSPeriod/T_)+1;

  FlyingFootTrajectory_deq_.resize(NbInstantsSS);
  std::deque<rigid_body_state_t>::iterator FTIt;
  FTIt = FlyingFootTrajectory_deq_.begin();
  OFTG_->SetParameters( FootTrajectoryGenerationStandard::Z_AXIS,
                        SSPeriod/2.0, OFTG_->GetStepHeight() );
  double LocalStartTime = 0.0;
  for( unsigned int i = 0; i < NbInstantsSS; i++)
    {
      if(i*T_>SSPeriod/2.0)
        {
          LocalStartTime = SSPeriod/2.0;
          OFTG_->SetParameters( FootTrajectoryGenerationStandard::Z_AXIS,
                                SSPeriod/2.0, 0.0,
                                OFTG_->GetStepHeight(), 0.0, 0.0 );
        }

      FTIt->Z(0) =
        OFTG_->Compute( FootTrajectoryGenerationStandard::Z_AXIS,
                        i*T_-LocalStartTime );
      FTIt->Z(2) =
        OFTG_->ComputeSecDerivative
        ( FootTrajectoryGenerationStandard::Z_AXIS,i*T_-LocalStartTime );
      FTIt++;
    }


  // Constant CoM height:
  // --------------------
  CoM_.Trajectory().resize(N_);
  std::deque<rigid_body_state_t>::iterator TrajIt;
  for(TrajIt = CoM_.Trajectory().begin();
      TrajIt < CoM_.Trajectory().end();
      ++TrajIt)
    {
      TrajIt->Z(0) = CoMHeight_;
    }


  // Fixed support states:
  // ---------------------
  SupportTrajectory_deq_.resize(40);
  std::deque<support_state_t>::iterator ST_it =
    SupportTrajectory_deq_.begin();
  ST_it->X = 0.257792;
  ST_it->Y = -0.105;
  ST_it++;
  ST_it->X = 0.257792;
  ST_it->Y = -0.105;
  ST_it++;
  ST_it->X = 0.412312;
  ST_it->Y = 0.095;
  ST_it++;
  double PositionX = 0.412312;
  for(; ST_it < (--SupportTrajectory_deq_.end()); ST_it++)
    {
      PositionX += 0.16;
      ST_it->X = PositionX;
      ST_it->Y = -0.105;
      ST_it++;
      PositionX += 0.16;
      ST_it->X = PositionX;
      ST_it->Y = 0.095;
    }


  return 0;

}


int
RigidBodySystem::
precompute_trajectories
( const deque<support_state_t> & SupportStates_deq )
{

  // Precompute vertical foot trajectories
  // The lowest height is the height of the ankle:
  // ---------------------------------------------
  Eigen::Vector3d LocalAnklePosition;
  LocalAnklePosition = PR_->leftFoot()->anklePosition ;

  deque<support_state_t>::const_iterator SS_it
    = SupportStates_deq.begin();
  SS_it++;//First support phase is current support phase
  deque<rigid_body_state_t>::iterator
    LFTraj_it = LeftFoot_.Trajectory().begin();
  deque<rigid_body_state_t>::iterator
    RFTraj_it = RightFoot_.Trajectory().begin();
  deque<rigid_body_state_t>::iterator
    FFTraj_it = FlyingFootTrajectory_deq_.begin();
  for(unsigned int i=0; i<N_; i++)
    {
      FFTraj_it = FlyingFootTrajectory_deq_.begin();
      // Patch :  It happens that  SS_it->NbInstants >
      // FlyingFootTrajectory_deq_.size()
      // which cause a wrong memory access during "FFTraj_it++;"
      // (size of the deque exceedded).
      // Typically, I witnessed during TestHerdt2010
      // a case where SS_it->NbInstants = 9
      // while FlyingFootTrajectory_deq_.size()=8
      // TODO: enhance this correction (find its origin).

      unsigned int endLoop =
        std::min<unsigned>(SS_it->NbInstants,(unsigned int)
                           FlyingFootTrajectory_deq_.size());
      for(unsigned int j = 0; j < endLoop; j++)
        FFTraj_it++;
      if(SS_it->StateChanged)
        {
          FFTraj_it = FlyingFootTrajectory_deq_.begin();
        }
      if(SS_it->Phase == DS)
        {
          LFTraj_it->Z(0) = LocalAnklePosition(2);
          RFTraj_it->Z(0) = LocalAnklePosition(2);
        }
      if(SS_it->Phase == SS && SS_it->Foot == LEFT)
        {
          LFTraj_it->Z(0) = LocalAnklePosition(2);
          RFTraj_it->Z(0) = LocalAnklePosition(2)+FFTraj_it->Z(0);
          RFTraj_it->Z(2) = FFTraj_it->Z(2);
          FFTraj_it++;
        }
      if(SS_it->Phase == SS && SS_it->Foot == RIGHT)
        {
          RFTraj_it->Z(0) = LocalAnklePosition(2);
          LFTraj_it->Z(0) = LocalAnklePosition(2)+FFTraj_it->Z(0);
          LFTraj_it->Z(2) = FFTraj_it->Z(2);
          FFTraj_it++;
        }
      SS_it++;
      RFTraj_it++;
      LFTraj_it++;
    }

  return 0;

}


int
RigidBodySystem::update
( const std::deque<support_state_t> & SupportStates_deq,
  const std::deque<FootAbsolutePosition> & LeftFootTraj_deq,
  const std::deque<FootAbsolutePosition> & RightFootTraj_deq )
{

  unsigned nbStepsPreviewed = SupportStates_deq.back().StepNumber;
  if(multiBody_)
    {

      compute_foot_pol_dynamics
        ( SupportStates_deq, LeftFoot_.Dynamics(POSITION),
          RightFoot_.Dynamics(POSITION) );
      compute_foot_pol_dynamics
        ( SupportStates_deq, LeftFoot_.Dynamics(ACCELERATION),
          RightFoot_.Dynamics(ACCELERATION) );

      precompute_trajectories( SupportStates_deq );
      compute_dyn_cop( nbStepsPreviewed );

      LeftFoot_.State().X[0] = LeftFootTraj_deq.front().x;
      LeftFoot_.State().X[1] = LeftFootTraj_deq.front().dx;
      LeftFoot_.State().X[2] = LeftFootTraj_deq.front().ddx;
      LeftFoot_.State().Y[0] = LeftFootTraj_deq.front().y;
      LeftFoot_.State().Y[1] = LeftFootTraj_deq.front().dy;
      LeftFoot_.State().Y[2] = LeftFootTraj_deq.front().ddy;
      RightFoot_.State().X[0] = RightFootTraj_deq.front().x;
      RightFoot_.State().X[1] = RightFootTraj_deq.front().dx;
      RightFoot_.State().X[2] = RightFootTraj_deq.front().ddx;
      RightFoot_.State().Y[0] = RightFootTraj_deq.front().y;
      RightFoot_.State().Y[1] = RightFootTraj_deq.front().dy;
      RightFoot_.State().Y[2] = RightFootTraj_deq.front().ddy;

    }

  return 0;

}

#if 0
/* TODO : Move this function on another file
 *
 * Matrix inversion routine.
 Uses lu_factorize and lu_substitute in uBLAS to invert a matrix */
template<class T> bool
invertMatrix (const Eigen::Matrix<T,Dynamic,Dynamic>& input,
              Eigen::Matrix<T,Dynamic,Dynamic>& inverse)
{
  using namespace boost::numeric::ublas;
  typedef permutation_matrix<std::size_t> pmatrix;
  // create a working copy of the input
  matrix<T> A(input);
  // create a permutation matrix for the LU-factorization
  pmatrix pm(A.rows());

  // perform LU-factorization
  int res = lu_factorize(A,pm);
  if( res != 0 ) return false;

  // create identity matrix of "inverse"
  inverse.assign(boost_ublas::identity_matrix<T>(A.rows()));

  // backsubstitute to get the inverse
  lu_substitute(A, pm, inverse);

  return true;
}
#endif

int
RigidBodySystem::compute_dyn_cop( unsigned nbSteps )
{

  const double GRAVITY = 9.81;

  if(multiBody_)
    {
      //TODO: Use of bounded_array to avoid dynamic allocation
      LeftFoot_. Dynamics( COP_POSITION ).U.resize(N_,nbSteps);
      LeftFoot_. Dynamics( COP_POSITION ).UT.resize(nbSteps,N_);
      RightFoot_.Dynamics( COP_POSITION ).U.resize(N_,nbSteps);
      RightFoot_.Dynamics( COP_POSITION ).UT.resize(nbSteps,N_);
      LeftFoot_. Dynamics( COP_POSITION ).clear();
      RightFoot_.Dynamics( COP_POSITION ).clear();
    }

  CoPDynamicsJerk_.clear();

  // Add "weighted" dynamic matrices:
  // --------------------------------
  std::deque<rigid_body_state_t>::iterator LFTraj_it =
    LeftFoot_.Trajectory().begin();
  std::deque<rigid_body_state_t>::iterator RFTraj_it =
    RightFoot_.Trajectory().begin();
  std::deque<rigid_body_state_t>::iterator CoMTraj_it =
    CoM_.Trajectory().begin();

  //  std::deque<double>::iterator GRF_it = GRF_deq_.begin();
  double GRF = 0.0;
  for(unsigned int i = 0; i < N_; i++)
    {
      GRF = CoM_.Mass()*(CoMTraj_it->Z[2]+GRAVITY)+
        LeftFoot_.Mass()*(LFTraj_it->Z[2]+GRAVITY)+
        RightFoot_.Mass()*(RFTraj_it->Z[2]+GRAVITY);

      CoPDynamicsJerk_.S.row( i )     +=
        CoM_.Dynamics(POSITION).S.row( i ) * CoM_.Mass()*
        ( CoMTraj_it->Z[2]+GRAVITY )/GRF;
      CoPDynamicsJerk_.U.row(i )      +=
        CoM_.Dynamics(POSITION).U.row( i ) * CoM_.Mass()*
        ( CoMTraj_it->Z[2]+GRAVITY )/GRF;
      CoPDynamicsJerk_.UT.col( i )    +=
        CoM_.Dynamics(POSITION).U.row( i ) * CoM_.Mass()*
        ( CoMTraj_it->Z[2]+GRAVITY )/GRF;
      CoPDynamicsJerk_.S.row( i ) -= CoM_.Dynamics(ACCELERATION).S.row( i )
        * CoM_.Mass()*( CoMTraj_it->Z[0] )/GRF;
      CoPDynamicsJerk_.U.row( i ) -= CoM_.Dynamics(ACCELERATION).U.row(i )
        * CoM_.Mass()*( CoMTraj_it->Z[0] )/GRF;
      CoPDynamicsJerk_.UT.col( i ) -= CoM_.Dynamics(ACCELERATION).U.row( i )
        * CoM_.Mass()*( CoMTraj_it->Z[0] )/GRF;

      if(multiBody_)
        {
          LeftFoot_.Dynamics(COP_POSITION).S.row( i )  +=
            LeftFoot_.Dynamics(POSITION).S.row( i ) *
            LeftFoot_.Mass()*( LFTraj_it->Z[2]+GRAVITY )/GRF;
          LeftFoot_.Dynamics(COP_POSITION).U.row( i )  +=
            LeftFoot_.Dynamics(POSITION).U.row( i ) *
            LeftFoot_.Mass()*( LFTraj_it->Z[2]+GRAVITY )/GRF;
          LeftFoot_.Dynamics(COP_POSITION).UT.col( i ) +=
            LeftFoot_.Dynamics(POSITION).U.row( i ) *
            LeftFoot_.Mass()*( LFTraj_it->Z[2]+GRAVITY )/GRF;
          LeftFoot_.Dynamics(COP_POSITION).S.row( i )  -=
            LeftFoot_.Dynamics(ACCELERATION).S.row( i ) *
            LeftFoot_.Mass()*( LFTraj_it->Z[0] )/GRF;
          LeftFoot_.Dynamics(COP_POSITION).U.row( i )  -=
            LeftFoot_.Dynamics(ACCELERATION).U.row( i ) *
            LeftFoot_.Mass()*( LFTraj_it->Z[0] )/GRF;
          LeftFoot_.Dynamics(COP_POSITION).UT.col( i ) -=
            LeftFoot_.Dynamics(ACCELERATION).U.row( i ) *
            LeftFoot_.Mass()*( LFTraj_it->Z[0] )/GRF;

          RightFoot_.Dynamics(COP_POSITION).S.row( i )  +=
            RightFoot_.Dynamics(POSITION).S.row( i ) *
            RightFoot_.Mass()*( RFTraj_it->Z[2]+GRAVITY )/GRF;
          RightFoot_.Dynamics(COP_POSITION).U.row( i )  +=
            RightFoot_.Dynamics(POSITION).U.row( i ) *
            RightFoot_.Mass()*( RFTraj_it->Z[2]+GRAVITY )/GRF;
          RightFoot_.Dynamics(COP_POSITION).UT.col( i ) +=
            RightFoot_.Dynamics(POSITION).U.row( i ) *
            RightFoot_.Mass()*( RFTraj_it->Z[2]+GRAVITY )/GRF;
          RightFoot_.Dynamics(COP_POSITION).S.row( i )  -=
            RightFoot_.Dynamics(ACCELERATION).S.row( i ) *
            RightFoot_.Mass()*( RFTraj_it->Z[0] )/GRF;
          RightFoot_.Dynamics(COP_POSITION).U.row( i )  -=
            RightFoot_.Dynamics(ACCELERATION).U.row( i ) *
            RightFoot_.Mass()*( RFTraj_it->Z[0] )/GRF;
          RightFoot_.Dynamics(COP_POSITION).UT.col( i ) -=
            RightFoot_.Dynamics(ACCELERATION).U.row( i ) *
            RightFoot_.Mass()*( RFTraj_it->Z[0] )/GRF;
        }

      CoMTraj_it++;
      LFTraj_it++;
      RFTraj_it++;
    }

  CoPDynamicsJerk_.Um1.resize(CoPDynamicsJerk_.U.rows(),
                              CoPDynamicsJerk_.U.cols());
  //    invertMatrix(CoPDynamicsJerk_.U,CoPDynamicsJerk_.Um1);

  return 0;

}


int
RigidBodySystem::compute_dyn_cjerk()
{

  // Initialize dynamics:
  // --------------------
  compute_dyn_cjerk( CoM_.Dynamics(POSITION) );
  compute_dyn_cjerk( CoM_.Dynamics(VELOCITY) );
  compute_dyn_cjerk( CoM_.Dynamics(ACCELERATION) );
  compute_dyn_cjerk( CoM_.Dynamics(JERK) );
  compute_dyn_cjerk( CoPDynamicsJerk_ );

  return 0;

}


int
RigidBodySystem::compute_dyn_cjerk( linear_dynamics_t & Dynamics )
{
  //TODO: This can be moved to RigidBody.
  Dynamics.U.resize(N_,N_);
  Dynamics.U.setZero();
  Dynamics.UT.resize(N_,N_);
  Dynamics.UT.setZero();
  Dynamics.S.resize(N_,3);
  Dynamics.S.setZero();

  switch(Dynamics.Type)
    {
    case POSITION:
      for(unsigned int i=0; i<N_; i++)
        {
          Dynamics.S(i,0) = 1;
          Dynamics.S(i,1) =(i+1)*T_;
          Dynamics.S(i,2) = ((i+1)*T_)*((i+1)*T_)/2;
          for(unsigned int j=0; j<N_; j++)
            if (j<=i)
              Dynamics.U(i,j) = Dynamics.UT(j,i)=
                (1+3*(i-j)+3*(i-j)*(i-j))*(T_*T_*T_)/6 ;
            else
              Dynamics.U(i,j) = Dynamics.UT(j,i) = 0.0;
        }
      break;
    case VELOCITY:
      for(unsigned int i=0; i<N_; i++)
        {
          Dynamics.S(i,0) = 0.0;
          Dynamics.S(i,1) = 1.0;
          Dynamics.S(i,2) = (i+1)*T_;
          for(unsigned int j=0; j<N_; j++)
            if (j<=i)
              Dynamics.U(i,j) = Dynamics.UT(j,i) = (2*(i-j)+1)*T_*T_*0.5 ;
            else
              Dynamics.U(i,j) = Dynamics.UT(j,i) = 0.0;
        }
      break;
    case ACCELERATION:
      for(unsigned int i=0; i<N_; i++)
        {
          Dynamics.S(i,0) = 0.0;
          Dynamics.S(i,1) = 0.0;
          Dynamics.S(i,2) = 1.0;
          for(unsigned int j=0; j<N_; j++)
            if (j<=i)
              Dynamics.U(i,j) = Dynamics.UT(j,i) = T_;
            else
              Dynamics.U(i,j) = Dynamics.UT(j,i) = 0.0;
        }
      break;
    case JERK:
      for(unsigned int i=0; i<N_; i++)
        {
          Dynamics.S(i,0) = 0.0;
          Dynamics.S(i,1) = 0.0;
          Dynamics.S(i,2) = 0.0;
          for(unsigned int j=0; j<N_; j++)
            if (j==i)
              Dynamics.U(i,j) = Dynamics.UT(j,i) = 1.0;
            else
              Dynamics.U(i,j) = Dynamics.UT(j,i) = 0.0;
        }
      break;
    case COP_POSITION:
      for(unsigned int i=0; i<N_; i++)
        {
          Dynamics.S(i,0) = 1.0;
          Dynamics.S(i,1) = (i+1)*T_;
          Dynamics.S(i,2) = (i+1)*(i+1)*T_*T_*0.5-CoMHeight_/9.81;
          for(unsigned int j=0; j<N_; j++)
            if (j<=i)
              Dynamics.U(i,j) =
                Dynamics.UT(j,i) =
                (1 + 3*(i-j) + 3*(i-j)*(i-j)) * T_*T_*T_/6.0 -
                T_*CoMHeight_/9.81;
            else
              Dynamics.U(i,j) = Dynamics.UT(j,i) = 0.0;
        }
      break;
      //    compute_dyn_cop( 0 );
      break;

    }

  return 0;

}


int
RigidBodySystem::
compute_foot_zero_dynamics
( const std::deque<support_state_t> &
  SupportStates_deq,
  linear_dynamics_t & LeftFootDynamics,
  linear_dynamics_t & RightFootDynamics )
{

  // Resize the matrices:
  // --------------------
  unsigned int nbSteps = SupportStates_deq.back().StepNumber;

  LeftFootDynamics.U.resize(N_,nbSteps);
  LeftFootDynamics.U.setZero();
  LeftFootDynamics.UT.resize(nbSteps,N_);
  LeftFootDynamics.UT.setZero();
  LeftFootDynamics.S.setZero();
  RightFootDynamics.U.resize(N_,nbSteps);
  RightFootDynamics.U.setZero();
  RightFootDynamics.UT.resize(nbSteps,N_);
  RightFootDynamics.UT.setZero();
  RightFootDynamics.S.setZero();

  // Fill the matrices:
  // ------------------
  linear_dynamics_t * SFDynamics;
  linear_dynamics_t * FFDynamics;
  double Spbar[3]= {0.0,0.0,0.0}; //, Sabar[3];
  double Upbar[2]= {0.0,0.0}; //, Uabar[2];
  std::deque<support_state_t>::const_iterator SS_it =
    SupportStates_deq.begin();
  SS_it++;
  for(unsigned int i=0; i<N_; i++)
    {
      if(SS_it->Foot == LEFT)
        {
          SFDynamics = & LeftFootDynamics;
          FFDynamics = & RightFootDynamics;
        }
      else
        {
          SFDynamics = & RightFootDynamics;
          FFDynamics = & LeftFootDynamics;
        }
      if(SS_it->Phase == DS)
        {
          // The previous row is copied in DS phase
          if(i==0)
            {
              if(SFDynamics->Type == POSITION)
                {
                  SFDynamics->S(i,0) = 1.0;
                  FFDynamics->S(i,0) = 1.0;
                  SFDynamics->S(i,1) = 0.0;
                  FFDynamics->S(i,1) = 0.0;
                  SFDynamics->S(i,2) = 0.0;
                  FFDynamics->S(i,2) = 0.0;
                }
            }
          if(i>0)
            {
              SFDynamics->S(i,0) = SFDynamics->S(i-1,0);
              FFDynamics->S(i,0) = FFDynamics->S(i-1,0);
              SFDynamics->S(i,1) = SFDynamics->S(i-1,1);
              FFDynamics->S(i,1) = FFDynamics->S(i-1,1);
              SFDynamics->S(i,2) = SFDynamics->S(i-1,2);
              FFDynamics->S(i,2) = FFDynamics->S(i-1,2);
              for(unsigned int SNb = 0; SNb < nbSteps; SNb++)
                {
                  SFDynamics->U(i,SNb)
                    = SFDynamics->UT(SNb,i)
                    = SFDynamics->U(i-1,SNb);
                  FFDynamics->U(i,SNb)
                    = FFDynamics->UT(SNb,i)
                    = FFDynamics->U(i-1,SNb);
                }
            }
        }
      else
        {
          if(SS_it->StepNumber == 0)
            {
              if(FFDynamics->Type == POSITION)
                {
                  SFDynamics->S(i,0) = 1.0;
                  SFDynamics->S(i,1) = 0.0;
                  SFDynamics->S(i,2) = 0.0;
                  FFDynamics->U(i,SS_it->StepNumber) =
                    FFDynamics->UT(SS_it->StepNumber,i) = 0.0;
                  SFDynamics->U(i,SS_it->StepNumber) =
                    SFDynamics->UT(SS_it->StepNumber,i) = 0.0;
                }
            }
          else if(SS_it->StepNumber == 1 && SS_it->StepNumber < nbSteps)
            {
              if(FFDynamics->Type == POSITION)
                {
                  FFDynamics->S(i,0) = Spbar[0];
                  FFDynamics->S(i,1) = Spbar[1];
                  FFDynamics->S(i,2) = Spbar[2];
                  FFDynamics->U(i,SS_it->StepNumber) =
                    FFDynamics->UT(SS_it->StepNumber,i) = Upbar[0];
                  SFDynamics->U(i,SS_it->StepNumber-1) =
                    SFDynamics->UT(SS_it->StepNumber-1,i) = 1.0;
                }
            }
          else if(SS_it->StepNumber == 2)
            {
              FFDynamics->S(i,0) = FFDynamics->S(i-1,0);
              SFDynamics->S(i,0) = SFDynamics->S(i-1,0);
              FFDynamics->S(i,1) = FFDynamics->S(i-1,1);
              SFDynamics->S(i,1) = SFDynamics->S(i-1,1);
              FFDynamics->S(i,2) = FFDynamics->S(i-1,2);
              SFDynamics->S(i,2) = SFDynamics->S(i-1,2);
              for(unsigned int j = 0; j<nbSteps; j++)
                {
                  FFDynamics->U(i,j)
                    = FFDynamics->UT(j,i)
                    = FFDynamics->U(i-1,j);
                  SFDynamics->U(i,j)
                    = SFDynamics->UT(j,i)
                    = SFDynamics->U(i-1,j);
                }
            }
        }
    }

  return 0;

}


int
RigidBodySystem::
compute_foot_pol_dynamics
( const std::deque<support_state_t> &
  SupportStates_deq,
  linear_dynamics_t & LeftFootDynamics,
  linear_dynamics_t & RightFootDynamics )
{

  // Resize the matrices:
  // --------------------
  unsigned int nbSteps = SupportStates_deq.back().StepNumber;

  LeftFootDynamics.U.resize(N_,nbSteps);
  LeftFootDynamics.U.setZero();
  LeftFootDynamics.UT.resize(nbSteps,N_);
  LeftFootDynamics.UT.setZero();
  LeftFootDynamics.S.setZero();
  RightFootDynamics.U.resize(N_,nbSteps);
  RightFootDynamics.U.setZero();
  RightFootDynamics.UT.resize(nbSteps,N_);
  RightFootDynamics.UT.setZero();
  RightFootDynamics.S.setZero();

  // Fill the matrices:
  // ------------------
  linear_dynamics_t * SFDynamics;
  linear_dynamics_t * FFDynamics;
  double Spbar[3], Sabar[3];
  double Upbar[2], Uabar[2];
  std::deque<support_state_t>::const_iterator SS_it =
    SupportStates_deq.begin();
  SS_it++;
  for(unsigned int i=0; i<N_; i++)
    {

      if(SS_it->Foot == LEFT)
        {
          SFDynamics = & LeftFootDynamics;
          FFDynamics = & RightFootDynamics;
        }
      else
        {
          SFDynamics = & RightFootDynamics;
          FFDynamics = & LeftFootDynamics;
        }
      if(SS_it->Phase == DS)
        {
          // The previous row is copied in DS phase
          // which means that the foot state stays the same
          if(i==0)
            {
              if(SFDynamics->Type == POSITION)
                {
                  SFDynamics->S(i,0) = 1.0;
                  FFDynamics->S(i,0) = 1.0;
                  SFDynamics->S(i,1) = 0.0;
                  FFDynamics->S(i,1) = 0.0;
                  SFDynamics->S(i,2) = 0.0;
                  FFDynamics->S(i,2) = 0.0;
                }
            }
          if(i>0)
            {
              SFDynamics->S(i,0) = SFDynamics->S(i-1,0);
              FFDynamics->S(i,0) = FFDynamics->S(i-1,0);
              SFDynamics->S(i,1) = SFDynamics->S(i-1,1);
              FFDynamics->S(i,1) = FFDynamics->S(i-1,1);
              SFDynamics->S(i,2) = SFDynamics->S(i-1,2);
              FFDynamics->S(i,2) = FFDynamics->S(i-1,2);
              for(unsigned int SNb = 0; SNb < nbSteps; SNb++)
                {
                  SFDynamics->U(i,SNb) = SFDynamics->UT(SNb,i) =
                    SFDynamics->U(i-1,SNb);
                  FFDynamics->U(i,SNb) = FFDynamics->UT(SNb,i) =
                    FFDynamics->U(i-1,SNb);
                }
            }
        }
      else
        {

          compute_sbar( Spbar, Sabar, (SS_it->NbInstants)*T_,
                        FSM_->StepPeriod()-T_ );
          compute_ubar( Upbar, Uabar, (SS_it->NbInstants)*T_,
                        FSM_->StepPeriod()-T_ );
          if(SS_it->StepNumber == 0 && SS_it->StepNumber < nbSteps)
            {
              if(FFDynamics->Type == POSITION)
                {
                  FFDynamics->S(i,0) = Spbar[0];
                  SFDynamics->S(i,0) = 1.0;
                  FFDynamics->S(i,1) = Spbar[1];
                  SFDynamics->S(i,1) = 0.0;
                  FFDynamics->S(i,2) = Spbar[2];
                  SFDynamics->S(i,2) = 0.0;
                  FFDynamics->U(i,SS_it->StepNumber) =
                    FFDynamics->UT(SS_it->StepNumber,
                                   i) = Upbar[0];
                  SFDynamics->U(i,SS_it->StepNumber) =
                    SFDynamics->UT(SS_it->StepNumber,i) = 0.0;
                }
              else if(FFDynamics->Type == ACCELERATION)
                {
                  FFDynamics->S(i,0) = Sabar[0];
                  SFDynamics->S(i,0) = 0.0;
                  FFDynamics->S(i,1) = Sabar[1];
                  SFDynamics->S(i,1) = 0.0;
                  FFDynamics->S(i,2) = Sabar[2];
                  SFDynamics->S(i,2) = 1.0;
                  FFDynamics->U(i,SS_it->StepNumber) =
                    FFDynamics->UT(SS_it->StepNumber,
                                   i) = Uabar[0];
                  SFDynamics->U(i,SS_it->StepNumber) =
                    SFDynamics->UT(SS_it->StepNumber,i) = 0.0;
                }
              if(((SS_it->NbInstants)*T_ > FSM_->StepPeriod()-T_) &&
                 (SS_it->StepNumber != 0))
                {
                  // DS phase
                  FFDynamics->S(i,0) = FFDynamics->S(i-1,0);
                  SFDynamics->S(i,0) = SFDynamics->S(i-1,0);
                  FFDynamics->S(i,1) = FFDynamics->S(i-1,1);
                  SFDynamics->S(i,1) = SFDynamics->S(i-1,1);
                  FFDynamics->S(i,2) = FFDynamics->S(i-1,2);
                  SFDynamics->S(i,2) = SFDynamics->S(i-1,2);
                  FFDynamics->U(i,SS_it->StepNumber) =
                    FFDynamics->UT(SS_it->StepNumber,
                                   i) = FFDynamics->U(i-1,SS_it->StepNumber);
                  SFDynamics->U(i,SS_it->StepNumber) =
                    SFDynamics->UT(SS_it->StepNumber,
                                   i) = SFDynamics->U(i-1,SS_it->StepNumber);
                }
            }
          else if(SS_it->StepNumber == 1 && SS_it->StepNumber < nbSteps)
            {
              if(FFDynamics->Type == POSITION)
                {
                  FFDynamics->S(i,0) = Spbar[0];
                  //SFDynamics->S(i,0) = SFDynamics->S(i-1,0);
                  FFDynamics->S(i,1) = Spbar[1];
                  //SFDynamics->S(i,1) = SFDynamics->S(i-1,1);
                  FFDynamics->S(i,2) = Spbar[2];
                  //SFDynamics->S(i,2) = SFDynamics->S(i-1,2);
                  FFDynamics->U(i,SS_it->StepNumber) =
                    FFDynamics->UT(SS_it->StepNumber,
                                   i) = Upbar[0];
                  SFDynamics->U(i,SS_it->StepNumber-1) =
                    SFDynamics->UT(SS_it->StepNumber-1,
                                   i) = 1.0;
                }
              else if(FFDynamics->Type == ACCELERATION)
                {
                  FFDynamics->S(i,0) = Sabar[0];
                  FFDynamics->S(i,1) = Sabar[1];
                  FFDynamics->S(i,2) = Sabar[2];
                  FFDynamics->U(i,SS_it->StepNumber) =
                    FFDynamics->UT(SS_it->StepNumber,
                                   i) = Uabar[0];
                  SFDynamics->U(i,SS_it->StepNumber-1) =
                    SFDynamics->UT(SS_it->StepNumber-1,
                                   i) = 1.0;
                }
              // The foot has touched the ground, the support phase has not
              // switched yet
              if((SS_it->NbInstants)*T_ > FSM_->StepPeriod()-T_)
                {
                  FFDynamics->S(i,0) = FFDynamics->S(i-1,0);
                  SFDynamics->S(i,0) = SFDynamics->S(i-1,0);
                  FFDynamics->S(i,1) = FFDynamics->S(i-1,1);
                  SFDynamics->S(i,1) = SFDynamics->S(i-1,1);
                  FFDynamics->S(i,2) = FFDynamics->S(i-1,2);
                  SFDynamics->S(i,2) = SFDynamics->S(i-1,2);
                  FFDynamics->U(i,SS_it->StepNumber) =
                    FFDynamics->UT(SS_it->StepNumber,
                                   i) = FFDynamics->U(i-1,SS_it->StepNumber);
                  SFDynamics->U(i,SS_it->StepNumber-1) =
                    SFDynamics->UT(SS_it->StepNumber-1,
                                   i) = SFDynamics->U(i-1,SS_it->StepNumber-1);
                }
            }
          else if(SS_it->StepNumber == 2)
            {
              FFDynamics->S(i,0) = FFDynamics->S(i-1,0);
              SFDynamics->S(i,0) = SFDynamics->S(i-1,0);
              FFDynamics->S(i,1) = FFDynamics->S(i-1,1);
              SFDynamics->S(i,1) = SFDynamics->S(i-1,1);
              FFDynamics->S(i,2) = FFDynamics->S(i-1,2);
              SFDynamics->S(i,2) = SFDynamics->S(i-1,2);
              for(unsigned int j = 0; j<nbSteps; j++)
                {
                  FFDynamics->U(i,j) = FFDynamics->UT(j,i) =
                    FFDynamics->U(i-1,j);
                  SFDynamics->U(i,j) = SFDynamics->UT(j,i) =
                    SFDynamics->U(i-1,j);
                }
            }
        }
      SS_it++;
    }

  return 0;

}




int
RigidBodySystem::
generate_trajectories
( double Time,
  const solution_t & Solution,
  const std::deque<support_state_t> & PrwSupportStates_deq,
  const std::deque<double> & PreviewedSupportAngles_deq,
  std::deque<FootAbsolutePosition> & LeftFootTraj_deq,
  std::deque<FootAbsolutePosition> & RightFootTraj_deq )
{
  
  OFTG_->interpolate_feet_positions(Time, PrwSupportStates_deq,
                                    Solution, PreviewedSupportAngles_deq,
                                    LeftFootTraj_deq, RightFootTraj_deq);

  return 0;

}


//
// Private methods:
//

int
RigidBodySystem::compute_sbar( double * Spbar, double * Sabar, double T,
                               double Td )
{

  double Td2 = Td*Td;
  double Td3 = Td*Td*Td;
  double Td4 = Td*Td*Td*Td;
  double Td5 = Td*Td*Td*Td*Td;

  Spbar[0] = 1.0;
  Spbar[1] = T;
  Spbar[2] = 0.0;
  double Ttemp = 0.5*T*T;
  Spbar[2] += Ttemp;
  Ttemp *= T;
  Spbar[0] -= 20.0/Td3*Ttemp;
  Spbar[1] -= 12.0/Td2*Ttemp;
  Spbar[2] -= 3.0/Td*Ttemp;
  Ttemp *= T;
  Spbar[0] += 30.0/Td4*Ttemp;
  Spbar[1] += 16.0/Td3*Ttemp;
  Spbar[2] += 3.0/Td2*Ttemp;
  Ttemp *= T;
  Spbar[0] -= 12.0/Td5*Ttemp;
  Spbar[1] -= 6.0/Td4*Ttemp;
  Spbar[2] -= 1.0/Td3*Ttemp;

  Sabar[0] = Sabar[1] = 0.0;
  Sabar[2] = 1.0;
  Ttemp = 0.5;
  Ttemp *= T;
  Sabar[0] -= 6.0*20.0/Td3*Ttemp;
  Sabar[1] -= 6.0*12.0/Td2*Ttemp;
  Sabar[2] -= 6.0*3.0/Td*Ttemp;
  Ttemp *= T;
  Sabar[0] += 12.0*30.0/Td4*Ttemp;
  Sabar[1] += 12.0*16.0/Td3*Ttemp;
  Sabar[2] += 12.0*3.0/Td2*Ttemp;
  Ttemp *= T;
  Sabar[0] -= 20.0*12.0/Td5*Ttemp;
  Sabar[1] -= 20.0*6.0/Td4*Ttemp;
  Sabar[2] -= 20.0*1.0/Td3*Ttemp;

  return 0;

}


int
RigidBodySystem::compute_ubar( double * Upbar, double * Uabar, double T,
                               double Td )
{

  double Td3 = Td*Td*Td;
  double Td4 = Td*Td*Td*Td;
  double Td5 = Td*Td*Td*Td*Td;

  Upbar[0] =  0.0; //Upbar[1] = 0.0;
  double Ttemp = T*T*T;
  Upbar[0] += 10.0/Td3*Ttemp; //Upbar[1] += 10.0/Td3*Ttemp;
  Ttemp *= T;
  Upbar[0] -= 15.0/Td4*Ttemp; //Upbar[1] -= 15.0/Td4*Ttemp;
  Ttemp *= T;
  Upbar[0] += 6.0/Td5*Ttemp; //Upbar[1] += 6.0/Td5*Ttemp;

  Uabar[0] =  0.0;// Uabar[1] = 0.0;
  Ttemp = T;
  Uabar[0] += 6.0*10.0/Td3*Ttemp;// Uabar[1] += 6.0*10.0/Td3*Ttemp;
  Ttemp *= T;
  Uabar[0] -= 12.0*15.0/Td4*Ttemp;// Uabar[1] -= 12.0*15.0/Td4*Ttemp;
  Ttemp *= T;
  Uabar[0] += 20.0*6.0/Td5*Ttemp;// Uabar[1] += 20.0*6.0/Td5*Ttemp;

  return 0;

}

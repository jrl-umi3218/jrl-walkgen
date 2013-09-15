/*
 * Copyright 2010,
 *
 * Andrei Herdt
 * Francois Keith
 * Olivier Stasse
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

/*! This object generate all the values for the foot trajectories,
  and the desired ZMP based on a sequence of steps following a QP
  formulation and a new QP solver as proposed by Herdt Advanced Robotics 2010.

  Andrei Herdt,
  Olivier Stasse,
 */

#include "portability/gettimeofday.hh"

#ifdef WIN32
# include <Windows.h>
#endif /* WIN32 */

#include <time.h>

#include <iostream>
#include <fstream>

#include <Mathematics/qld.hh>
#include <privatepgtypes.hh>
#include <ZMPRefTrajectoryGeneration/ZMPVelocityReferencedQP.hh>

#include <Debug.hh>
using namespace std;
using namespace PatternGeneratorJRL;



ZMPVelocityReferencedQP::ZMPVelocityReferencedQP(SimplePluginManager *SPM,
    string , CjrlHumanoidDynamicRobot *aHS) :
    ZMPRefTrajectoryGeneration(SPM),
    Robot_(0),SupportFSM_(0),OrientPrw_(0),VRQPGenerator_(0),IntermedData_(0),RFI_(0),Problem_(),Solution_()
{
  Running_ = false;
  TimeBuffer_ = 0.04;
  QP_T_ = 0.1;
  QP_N_ = 16;
  m_SamplingPeriod = 0.005;
  PerturbationOccured_ = false;
  UpperTimeLimitToUpdate_ = 0.0;
  RobotMass_ = aHS->mass();
  Solution_.useWarmStart=false;

  // Create and initialize online interpolation of feet trajectories
  RFI_ = new RelativeFeetInequalities( SPM,aHS );

  // Create and initialize the finite state machine for support sequences
  SupportFSM_ = new SupportFSM();
  SupportFSM_->StepPeriod( 0.8 );
  SupportFSM_->DSPeriod( 1e9 );
  SupportFSM_->DSSSPeriod( 0.8 );
  SupportFSM_->NbStepsSSDS( 2 );
  SupportFSM_->SamplingPeriod( QP_T_ );

  // Create and initialize preview of orientations
  OrientPrw_ = new OrientationsPreview( aHS->rootJoint() );
  OrientPrw_->SamplingPeriod( QP_T_ );
  OrientPrw_->NbSamplingsPreviewed( QP_N_ );
  OrientPrw_->SSLength( SupportFSM_->StepPeriod() );
  COMState CurrentTrunkState;
  OrientPrw_->CurrentTrunkState( CurrentTrunkState );

  // Initialize  the 2D LIPM
  CoM_.SetSimulationControlPeriod( QP_T_ );
  CoM_.SetRobotControlPeriod( m_SamplingPeriod );
  CoM_.InitializeSystem();

  // Create and initialize simplified robot model
  Robot_ = new RigidBodySystem( SPM, aHS, SupportFSM_ );
  Robot_->Mass( aHS->mass() );
  Robot_->LeftFoot().Mass( 0.0 );
  Robot_->RightFoot().Mass( 0.0 );
  Robot_->NbSamplingsPreviewed( QP_N_ );
  Robot_->SamplingPeriodSim( QP_T_ );
  Robot_->SamplingPeriodAct( m_SamplingPeriod );
  Robot_->CoMHeight( 0.814 );
  Robot_->multiBody(false);
  Robot_->initialize( );


  IntermedData_ = new IntermedQPMat();

  VRQPGenerator_ = new GeneratorVelRef( SPM, IntermedData_, Robot_, RFI_ );
  VRQPGenerator_->NbPrwSamplings( QP_N_ );
  VRQPGenerator_->SamplingPeriodPreview( QP_T_ );
  VRQPGenerator_->SamplingPeriodControl( m_SamplingPeriod );
  VRQPGenerator_->ComHeight( 0.814 );
  VRQPGenerator_->initialize_matrices();
  VRQPGenerator_->Ponderation( 1.0, INSTANT_VELOCITY );
  VRQPGenerator_->Ponderation( 0.000001, COP_CENTERING );
  VRQPGenerator_->Ponderation( 0.00001, JERK_MIN );

  // Register method to handle
  const unsigned int NbMethods = 3;
  string aMethodName[NbMethods] =
      {":previewcontroltime",
          ":numberstepsbeforestop",
          ":stoppg"};

  for(unsigned int i=0;i<NbMethods;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
    }
  
}


ZMPVelocityReferencedQP::~ZMPVelocityReferencedQP()
{

  if (VRQPGenerator_!=0)
    delete VRQPGenerator_;

  if (SupportFSM_!=0)
    delete SupportFSM_;

  if (RFI_!=0)
    delete RFI_;

  if (OrientPrw_!=0)
    delete OrientPrw_;

  if (Robot_!=0)
    delete Robot_;

  if (IntermedData_!=0)
    delete IntermedData_;

}


void
ZMPVelocityReferencedQP::setCoMPerturbationForce(istringstream &strm)
{

  MAL_VECTOR_RESIZE(PerturbationAcceleration_,6);

  strm >> PerturbationAcceleration_(2);
  strm >> PerturbationAcceleration_(5);
  PerturbationAcceleration_(2) = PerturbationAcceleration_(2)/RobotMass_;
  PerturbationAcceleration_(5) = PerturbationAcceleration_(5)/RobotMass_;
  PerturbationOccured_ = true;
}

void
ZMPVelocityReferencedQP::setCoMPerturbationForce(double x, double y)
{

  MAL_VECTOR_RESIZE(PerturbationAcceleration_,6);

  PerturbationAcceleration_(2) = x/RobotMass_;
  PerturbationAcceleration_(5) = y/RobotMass_;
  PerturbationOccured_ = true;
}

//--------------------------------------
//
//
//-----------new functions--------------
void ZMPVelocityReferencedQP::CallMethod(std::string & Method, std::istringstream &strm)
{

  if (Method==":previewcontroltime")
    {
      strm >> m_PreviewControlTime;
    }
  if (Method==":numberstepsbeforestop")
    {
      support_state_t & CurrentSupport = IntermedData_->SupportState();
      strm >> CurrentSupport.NbStepsLeft;
      SupportFSM_->NbStepsSSDS(CurrentSupport.NbStepsLeft);
    }
  if (Method==":stoppg")
    {
      EndingPhase_ = true;
    }

  ZMPRefTrajectoryGeneration::CallMethod(Method,strm);

}

int
ZMPVelocityReferencedQP::InitOnLine(deque<ZMPPosition> & FinalZMPTraj_deq,
    deque<COMState> & FinalCoMPositions_deq,
    deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
    deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
    FootAbsolutePosition & InitLeftFootAbsolutePosition,
    FootAbsolutePosition & InitRightFootAbsolutePosition,
    deque<RelativeFootPosition> &, // RelativeFootPositions,
    COMState & lStartingCOMState,
    MAL_S3_VECTOR_TYPE(double) & lStartingZMPPosition)
{

  FootAbsolutePosition CurrentLeftFootAbsPos, CurrentRightFootAbsPos;

  // Set the internal state of the ZMPRefTrajectory object.
  m_OnLineMode = true;
  EndingPhase_ = false;
  TimeToStopOnLineMode_ = -1.0;

  // INITIALIZE FEET POSITIONS:
  // --------------------------
  CurrentLeftFootAbsPos = InitLeftFootAbsolutePosition;
  CurrentLeftFootAbsPos.z = 0.0;
  CurrentLeftFootAbsPos.dz = CurrentLeftFootAbsPos.ddz = 0.0;
  CurrentLeftFootAbsPos.time = 0.0;
  CurrentLeftFootAbsPos.theta = 0.0;

  CurrentRightFootAbsPos = InitRightFootAbsolutePosition;
  CurrentRightFootAbsPos.z = 0.0;
  CurrentRightFootAbsPos.dz = CurrentRightFootAbsPos.ddz = 0.0;
  CurrentRightFootAbsPos.time = 0.0;
  CurrentRightFootAbsPos.theta = 0.0;

  // FILL THE QUEUES:
  // ----------------
  int AddArraySize;
  {
    assert(m_SamplingPeriod > 0);
    double ldAddArraySize = TimeBuffer_/m_SamplingPeriod;
    AddArraySize = (int)ldAddArraySize;
  }

  FinalZMPTraj_deq.resize(AddArraySize);
  FinalCoMPositions_deq.resize(AddArraySize);
  FinalLeftFootTraj_deq.resize(AddArraySize);
  FinalRightFootTraj_deq.resize(AddArraySize);
  int CurrentZMPindex=0;
  for( unsigned int i=0;i<FinalZMPTraj_deq.size();i++ )
    {
      // Smooth ramp
      FinalZMPTraj_deq[CurrentZMPindex].px = lStartingZMPPosition(0);
      FinalZMPTraj_deq[CurrentZMPindex].py = lStartingZMPPosition(1);
      FinalZMPTraj_deq[CurrentZMPindex].pz = lStartingZMPPosition(2);
      FinalZMPTraj_deq[CurrentZMPindex].theta = 0.0;
      FinalZMPTraj_deq[CurrentZMPindex].time = m_CurrentTime;
      FinalZMPTraj_deq[CurrentZMPindex].stepType = 0;

      // Set CoM positions.
      FinalCoMPositions_deq[CurrentZMPindex] = lStartingCOMState;
      // Set Left Foot positions.
      FinalLeftFootTraj_deq[CurrentZMPindex] = CurrentLeftFootAbsPos;
      FinalRightFootTraj_deq[CurrentZMPindex] = CurrentRightFootAbsPos;
      FinalLeftFootTraj_deq[CurrentZMPindex].time =
          FinalRightFootTraj_deq[CurrentZMPindex].time = m_CurrentTime;
      FinalLeftFootTraj_deq[CurrentZMPindex].stepType =
          FinalRightFootTraj_deq[CurrentZMPindex].stepType = 10;

      m_CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
    }

  // INITIAL SUPPORT STATE:
  // ----------------------
  support_state_t CurrentSupport;
  CurrentSupport.Phase = DS;
  CurrentSupport.Foot = LEFT;
  CurrentSupport.TimeLimit = 1000000000;
  CurrentSupport.NbStepsLeft = 1;
  CurrentSupport.StateChanged = false;
  CurrentSupport.X = 0.0;
  CurrentSupport.Y = 0.1;
  CurrentSupport.Yaw = 0.0;
  CurrentSupport.StartTime = 0.0;
  IntermedData_->SupportState(CurrentSupport);

  // INITIALIZE CENTER OF MASS:
  // --------------------------
  com_t CoM;
  CoM.x[0] = lStartingCOMState.x[0];
  CoM.x[1] = lStartingCOMState.x[1];
  CoM.x[2] = lStartingCOMState.x[2];
  CoM.y[0] = lStartingCOMState.y[0];
  CoM.y[1] = lStartingCOMState.y[1];
  CoM.y[2] = lStartingCOMState.y[2];
  CoM.z[0] = lStartingCOMState.z[0];
  CoM.z[1] = lStartingCOMState.z[1];
  CoM.z[2] = lStartingCOMState.z[2];
  CoM_.SetComHeight(lStartingCOMState.z[0]);
  CoM_.InitializeSystem();
  CoM_(CoM);
  IntermedData_->CoM(CoM_());

  // BUILD CONSTANT PART OF THE OBJECTIVE:
  // -------------------------------------
  Problem_.reset();
  Problem_.nbInvariantRows(2*QP_N_);
  Problem_.nbInvariantCols(2*QP_N_);
  VRQPGenerator_->build_invariant_part( Problem_ );

  return 0;
}



void
ZMPVelocityReferencedQP::OnLine(double time,
    deque<ZMPPosition> & FinalZMPTraj_deq,
    deque<COMState> & FinalCOMTraj_deq,
    deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
    deque<FootAbsolutePosition> & FinalRightFootTraj_deq)

{

  // If on-line mode not activated we go out.
  if (!m_OnLineMode)
    return;

  // Test if the end of the online mode has been reached.
  if ((EndingPhase_) &&
      (time>=TimeToStopOnLineMode_))
    { m_OnLineMode = false; }



  // UPDATE WALKING TRAJECTORIES:
  // ----------------------------
  if(time + 0.00001 > UpperTimeLimitToUpdate_)
    {

      // UPDATE INTERNAL DATA:
      // ---------------------
      Problem_.reset_variant();
      Solution_.reset();
      VRQPGenerator_->CurrentTime( time );
      VelRef_=NewVelRef_;
      SupportFSM_->update_vel_reference(VelRef_, IntermedData_->SupportState());
      IntermedData_->Reference( VelRef_ );
      IntermedData_->CoM( CoM_() );


      // PREVIEW SUPPORT STATES FOR THE WHOLE PREVIEW WINDOW:
      // ----------------------------------------------------
      VRQPGenerator_->preview_support_states( time, SupportFSM_,
          FinalLeftFootTraj_deq, FinalRightFootTraj_deq, Solution_.SupportStates_deq );

      // COMPUTE ORIENTATIONS OF FEET FOR WHOLE PREVIEW PERIOD:
      // ------------------------------------------------------
      OrientPrw_->preview_orientations( time, VelRef_,
          SupportFSM_->StepPeriod(),
          FinalLeftFootTraj_deq, FinalRightFootTraj_deq,
          Solution_ );


      // UPDATE THE DYNAMICS:
      // --------------------
      Robot_->update( Solution_.SupportStates_deq,
          FinalLeftFootTraj_deq, FinalRightFootTraj_deq );


      // COMPUTE REFERENCE IN THE GLOBAL FRAME:
      // --------------------------------------
      VRQPGenerator_->compute_global_reference( Solution_ );


      // BUILD VARIANT PART OF THE OBJECTIVE:
      // ------------------------------------
      VRQPGenerator_->update_problem( Problem_, Solution_.SupportStates_deq );


      // BUILD CONSTRAINTS:
      // ------------------
      VRQPGenerator_->build_constraints( Problem_, Solution_ );


      // SOLVE PROBLEM:
      // --------------
      if (Solution_.useWarmStart)
    	  VRQPGenerator_->compute_warm_start( Solution_ );//TODO: Move to update_problem or build_constraints?
      Problem_.solve( QLD, Solution_, NONE );
      if(Solution_.Fail>0)
          Problem_.dump( time );


      // INTERPOLATE THE NEXT COMPUTED COM STATE:
      // ----------------------------------------
      unsigned currentIndex = FinalCOMTraj_deq.size();
      FinalCOMTraj_deq.resize( (unsigned)(QP_T_/m_SamplingPeriod)+currentIndex );
      FinalZMPTraj_deq.resize( (unsigned)(QP_T_/m_SamplingPeriod)+currentIndex );
      if(Solution_.SupportStates_deq.size() &&  Solution_.SupportStates_deq[0].NbStepsLeft == 0)
      {
         double jx = (FinalLeftFootTraj_deq[0].x + FinalRightFootTraj_deq[0].x)/2 - FinalCOMTraj_deq[0].x[0];
         double jy = (FinalLeftFootTraj_deq[0].y + FinalRightFootTraj_deq[0].y)/2 - FinalCOMTraj_deq[0].y[0];
         if(fabs(jx) < 1e-3 && fabs(jy) < 1e-3) { Running_ = false; }
         const double tf = 0.75;
         jx = 6/(tf*tf*tf)*(jx - tf*FinalCOMTraj_deq[0].x[1] - (tf*tf/2)*FinalCOMTraj_deq[0].x[2]);
         jy = 6/(tf*tf*tf)*(jy - tf*FinalCOMTraj_deq[0].y[1] - (tf*tf/2)*FinalCOMTraj_deq[0].y[2]);
         CoM_.Interpolation( FinalCOMTraj_deq, FinalZMPTraj_deq, currentIndex,
             jx, jy);
         CoM_.OneIteration( jx, jy );
      }
      else
      {
         Running_ = true;
         CoM_.Interpolation( FinalCOMTraj_deq, FinalZMPTraj_deq, currentIndex,
             Solution_.Solution_vec[0], Solution_.Solution_vec[QP_N_] );
         CoM_.OneIteration( Solution_.Solution_vec[0],Solution_.Solution_vec[QP_N_] );
      }


      // INTERPOLATE TRUNK ORIENTATION:
      // ------------------------------
      OrientPrw_->interpolate_trunk_orientation( time, currentIndex,
          m_SamplingPeriod, Solution_.SupportStates_deq,
          FinalCOMTraj_deq );


      // INTERPOLATE THE COMPUTED FOOT POSITIONS:
      // ----------------------------------------
      Robot_->generate_trajectories( time, Solution_,
          Solution_.SupportStates_deq, Solution_.SupportOrientations_deq,
          FinalLeftFootTraj_deq, FinalRightFootTraj_deq );

      
      // Specify that we are in the ending phase.
      if (EndingPhase_ == false)
        {
          TimeToStopOnLineMode_ = UpperTimeLimitToUpdate_ + QP_T_ * QP_N_;
        }
      UpperTimeLimitToUpdate_ = UpperTimeLimitToUpdate_ + QP_T_;

    }
  //-----------------------------------
  //
  //
  //----------"Real-time" loop--------

}


// TODO: New parent class needed
void ZMPVelocityReferencedQP::GetZMPDiscretization(deque<ZMPPosition> & ,
    deque<COMState> & ,
    deque<RelativeFootPosition> &,
    deque<FootAbsolutePosition> &,
    deque<FootAbsolutePosition> &,
    double ,
    COMState &,
    MAL_S3_VECTOR(&,double),
    FootAbsolutePosition & ,
    FootAbsolutePosition & )
{
  cout << "To be removed" << endl;
}


void ZMPVelocityReferencedQP::OnLineAddFoot(RelativeFootPosition & ,
    deque<ZMPPosition> & ,
    deque<COMState> & ,
    deque<FootAbsolutePosition> &,
    deque<FootAbsolutePosition> &,
    bool)
{
  cout << "To be removed" << endl;
}

int ZMPVelocityReferencedQP::OnLineFootChange(double ,
    FootAbsolutePosition &,
    deque<ZMPPosition> & ,
    deque<COMState> & ,
    deque<FootAbsolutePosition> &,
    deque<FootAbsolutePosition> &,
    StepStackHandler  *)
{
  cout << "To be removed" << endl;
  return -1;
}

void ZMPVelocityReferencedQP::EndPhaseOfTheWalking(deque<ZMPPosition> &, 
    deque<COMState> &,
    deque<FootAbsolutePosition> &,
    deque<FootAbsolutePosition> &)
{
  cout << "To be removed" << endl;
}

int ZMPVelocityReferencedQP::ReturnOptimalTimeToRegenerateAStep()
{
  int r = (int)(m_PreviewControlTime/m_SamplingPeriod);
  return 2*r;
}


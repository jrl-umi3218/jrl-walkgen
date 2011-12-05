/*
 * Copyright 2011,
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
 
/*! This object composes all the objects necessary for the generation of walking
 * trajectories as proposed by Herdt Advanced Robotics 2010.

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

#include <ZMPRefTrajectoryGeneration/ZMPVelocityReferencedQP.h>

using namespace std;
using namespace PatternGeneratorJRL;



ZMPVelocityReferencedQP::ZMPVelocityReferencedQP(SimplePluginManager *SPM,
                                                 string DataFile, CjrlHumanoidDynamicRobot *aHS) :
  ZMPRefTrajectoryGeneration(SPM),
  Robot_(0),SupportFSM_(0),OrientPrw_(0),VRQPGenerator_(0),IntermedData_(0),RFI_(0),Problem_(),Solution_()
{

  TimeBuffer_ = 0.01;

  QP_T_ = 0.1;
  QP_N_ = 16;
  m_SamplingPeriod = 0.005;
  m_SamplingFeedback = 0.005;
  PerturbationOccured_ = false;
  UpperTimeLimitToUpdate_ = 0.0;
  UpperTimeLimitToFeedback_ = 0.0;
  FirstIterationDynamicsDuration_ = QP_T_;
  RobotMass_ = aHS->mass();

  Solution_.useWarmStart=true;


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
  OrientPrw_-> SimuPeriod(m_SamplingFeedback);
  OrientPrw_->NbSamplingsPreviewed( QP_N_ );
  OrientPrw_->SSLength( SupportFSM_->StepPeriod() );
  COMState CurrentTrunkState;
  OrientPrw_->CurrentTrunkState( CurrentTrunkState );
  
  // Initialize  the 2D LIPM
  CoM_.SetSimulationControlPeriod( m_SamplingFeedback );
  CoM_.SetRobotControlPeriod( m_SamplingPeriod );
  CoM_.InitializeSystem();
  
  // Create and initialize simplified robot model
  Robot_ = new RigidBodySystem( SPM, aHS, SupportFSM_ );
  Robot_->Mass( aHS->mass() );
  Robot_->LeftFoot().Mass( 0.0 );
  Robot_->RightFoot().Mass( 0.0 );
  Robot_->NbSamplingsPreviewed( QP_N_ );
  Robot_->SamplingPeriodSim( m_SamplingFeedback );
  Robot_->SamplingPeriodMatrix( QP_T_ );
  Robot_->SamplingPeriodAct( m_SamplingPeriod );
  Robot_->CoMHeight( 0.814 );
  Robot_->multiBody(false);
  Robot_->FirstIterationDynamicsDuration(FirstIterationDynamicsDuration_);
  Robot_->initialize( );
  
  
  IntermedData_ = new IntermedQPMat();
  
  VRQPGenerator_ = new GeneratorVelRef( SPM, IntermedData_, Robot_, RFI_ );
  VRQPGenerator_->NbPrwSamplings( QP_N_ );
  VRQPGenerator_->SamplingPeriodPreview( QP_T_ );
  VRQPGenerator_->SamplingPeriodControl( m_SamplingPeriod );
  VRQPGenerator_->ComHeight( 0.814 );

  VRQPGenerator_->initialize_matrices();

  VRQPGenerator_->Ponderation( 1, INSTANT_VELOCITY );
  VRQPGenerator_->Ponderation( 10, COP_CENTERING );
  VRQPGenerator_->Ponderation( 0.001, JERK_MIN );

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
ZMPVelocityReferencedQP::setCoMPerturbationForce(double x,double y)
{
  
  MAL_VECTOR_RESIZE(PerturbationAcceleration_,6);
  
  PerturbationAcceleration_(2) = x/RobotMass_;
  PerturbationAcceleration_(5) = y/RobotMass_;
  PerturbationOccured_ = true;
  
}


void
ZMPVelocityReferencedQP::CallMethod(std::string & Method, std::istringstream &strm)
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

  return 0;
}

void
ZMPVelocityReferencedQP::OnLine(double time,
    deque<ZMPPosition> & FinalZMPTraj_deq,
    deque<COMState> & FinalCOMTraj_deq,
    deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
    deque<FootAbsolutePosition> & FinalRightFootTraj_deq)
{
  OnLine(time,
         FinalZMPTraj_deq,
         FinalCOMTraj_deq,
         FinalLeftFootTraj_deq,
         FinalRightFootTraj_deq,
         false);
  // TODO: Changing the support state can be done by a mutator, no?
}


void
ZMPVelocityReferencedQP::OnLine(double time,
    deque<ZMPPosition> & FinalZMPTraj_deq,
    deque<COMState> & FinalCOMTraj_deq,
    deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
    deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
    bool support_state_changed)
{
  
  // If on-line mode not activated we go out.
  if (!m_OnLineMode)
    { return; }

  // Test if the end of the online mode has been reached.
  if ((EndingPhase_) &&
      (time>=TimeToStopOnLineMode_))
    { m_OnLineMode = false; }


  // Apply external forces if occured
  if(PerturbationOccured_ == true)
    {
      com_t com = CoM_();
      com.x(2) = com.x(2)+PerturbationAcceleration_(2);
      com.y(2) = com.y(2)+PerturbationAcceleration_(5);
      PerturbationOccured_ = false;
      CoM_(com);
    }


  // UPDATE WALKING TRAJECTORIES:
  // ----------------------------
  if(time  > UpperTimeLimitToUpdate_ - 0.00001){
      // Specify that we are in the ending phase.
      if (EndingPhase_ == false)
        {
          TimeToStopOnLineMode_ = UpperTimeLimitToUpdate_ + QP_T_ * QP_N_;
        }
      UpperTimeLimitToUpdate_ = UpperTimeLimitToUpdate_ + QP_T_;
      CurrentTime_ = time;
  }


  if(time  >= UpperTimeLimitToFeedback_)
      {



      static double CurrentCPUTime=0.0, CurrentQLDTime=0.0, CurrentinvariantpartTime=0.0,
    		  CurrentConstraintTime=0.0, CurrentObjTime=0.0, CurrentInterpolTime=0.0, CurrentrefTime=0.0;
      static double MaxCPUTime=0.0, MaxQLDTime=0.0, MaxinvariantpartTime=0.0,
    		  MaxObjTime=0.0, MaxConstraintTime=0.0, MaxInterpolTime=0.0, MaxrefTime=0.0;
      static int itt=0;

      struct timeval start,mid1,mid2,mid3,mid4,mid5,mid6,mid7,mid8,mid9,mid10,mid11,mid12,end;
      gettimeofday(&start,0);

      gettimeofday(&mid11,0);


      // Adaptive control of current support state
      if ( support_state_changed ){
		UpperTimeLimitToUpdate_ = time + QP_T_;
		//UpperTimeLimitToFeedback_ = time;
		CurrentTime_ = time;
      }
      

      // UPDATE INTERNAL DATA:
      // ---------------------
      Problem_.reset();
      Solution_.reset();
      VRQPGenerator_->CurrentTime( CurrentTime_ );
      VelRef_=NewVelRef_;

      SupportFSM_->update_vel_reference(VelRef_, IntermedData_->SupportState());
      if (VelRef_.Local.Yaw==0 && VelRef_.Local.X==0 && VelRef_.Local.Y==0){
		  VRQPGenerator_->Ponderation( 1, INSTANT_VELOCITY );
		  VRQPGenerator_->Ponderation( 10, COP_CENTERING );
		  VRQPGenerator_->Ponderation( 0.001, JERK_MIN );
      }else{
		  VRQPGenerator_->Ponderation( 1, INSTANT_VELOCITY );
		  VRQPGenerator_->Ponderation( 0.000001, COP_CENTERING );
		  VRQPGenerator_->Ponderation( 0.001, JERK_MIN );
      }

      IntermedData_->Reference( VelRef_ );
      IntermedData_->CoM( CoM_() );
      
      // update time limits
      FirstIterationDynamicsDuration_ = UpperTimeLimitToUpdate_-UpperTimeLimitToFeedback_;
      UpperTimeLimitToFeedback_ = UpperTimeLimitToFeedback_ + m_SamplingFeedback;


      // Recompute dynamic matrix according to synchronization with DS phase
      Robot_->FirstIterationDynamicsDuration(FirstIterationDynamicsDuration_);
      Robot_->recompute_dynamic_matrix( );


      // PREVIEW SUPPORT STATES FOR THE WHOLE PREVIEW WINDOW:
      // ----------------------------------------------------
      VRQPGenerator_->preview_support_states( CurrentTime_, SupportFSM_,
          Solution_.SupportStates_deq );


      // COMPUTE ORIENTATIONS OF FEET FOR WHOLE PREVIEW PERIOD:
      // ------------------------------------------------------
      OrientPrw_->preview_orientations( CurrentTime_, VelRef_,
          SupportFSM_->StepPeriod(),
          Robot_->LeftFoot().State(),Robot_->RightFoot().State(),
          Solution_ );

      // UPDATE THE DYNAMICS:
      // --------------------
      Robot_->update( Solution_.SupportStates_deq );


      // COMPUTE REFERENCE IN THE GLOBAL FRAME:
      // --------------------------------------
      VRQPGenerator_->compute_global_reference( Solution_ );
      gettimeofday(&mid12,0);

      // BUILD VARIANT PART OF THE OBJECTIVE:
      // ------------------------------------
      gettimeofday(&mid7,0);
      VRQPGenerator_->update_problem( Problem_, Solution_.SupportStates_deq, FirstIterationDynamicsDuration_/QP_T_ );
      gettimeofday(&mid8,0);

      // BUILD CONSTRAINTS:
      // ------------------

      gettimeofday(&mid5,0);
      VRQPGenerator_->build_constraints( Problem_, Solution_);
      gettimeofday(&mid6,0);
      // SOLVE PROBLEM:
      // --------------

      gettimeofday(&mid3,0);
      VRQPGenerator_->compute_warm_start( Solution_, FirstIterationDynamicsDuration_/QP_T_ );

      gettimeofday(&mid4,0);
      gettimeofday(&mid1,0);
      Problem_.solve( LSSOL, Solution_, NONE );
      gettimeofday(&mid2,0);


      if(Solution_.Fail>0)
          Problem_.dump( time );

      VRQPGenerator_->convert_cop_to_jerk_formulation(Solution_);

      VRQPGenerator_->amelif_preview_display(Solution_);

      gettimeofday(&mid9,0);

      // INTERPOLATE THE NEXT COMPUTED COM STATE:
      // ----------------------------------------
      unsigned currentIndex = FinalCOMTraj_deq.size();
      FinalCOMTraj_deq.resize( (unsigned)(m_SamplingFeedback/m_SamplingPeriod)+currentIndex );
      FinalZMPTraj_deq.resize( (unsigned)(m_SamplingFeedback/m_SamplingPeriod)+currentIndex );
      CoM_.Interpolation( FinalCOMTraj_deq, FinalZMPTraj_deq, currentIndex,
          Solution_.Solution_vec[0], Solution_.Solution_vec[QP_N_] );
      CoM_.OneIteration( Solution_.Solution_vec[0],Solution_.Solution_vec[QP_N_] );


      // INTERPOLATE TRUNK ORIENTATION:
      // ------------------------------
      OrientPrw_->interpolate_trunk_orientation( CurrentTime_, currentIndex,
          m_SamplingFeedback, Solution_.SupportStates_deq, FinalCOMTraj_deq );


      // INTERPOLATE THE COMPUTED FOOT POSITIONS:
      // ----------------------------------------
      Robot_->generate_trajectories( CurrentTime_, Solution_,
          Solution_.SupportStates_deq, Solution_.SupportOrientations_deq,
          FinalLeftFootTraj_deq, FinalRightFootTraj_deq );

      gettimeofday(&mid10,0);




      // Compute CPU consumption time.
      gettimeofday(&end,0);








      CurrentCPUTime += end.tv_sec - start.tv_sec +
          0.000001 * (end.tv_usec - start.tv_usec);

      CurrentQLDTime += mid2.tv_sec - mid1.tv_sec +
          0.000001 * (mid2.tv_usec - mid1.tv_usec);
      CurrentinvariantpartTime+= mid4.tv_sec - mid3.tv_sec +
              0.000001 * (mid4.tv_usec - mid3.tv_usec);

      CurrentConstraintTime+= mid6.tv_sec - mid5.tv_sec +
              0.000001 * (mid6.tv_usec - mid5.tv_usec);

      CurrentObjTime+= mid8.tv_sec - mid7.tv_sec +
              0.000001 * (mid8.tv_usec - mid7.tv_usec);

      CurrentInterpolTime+= mid10.tv_sec - mid9.tv_sec +
              0.000001 * (mid10.tv_usec - mid9.tv_usec);

      CurrentrefTime+= mid12.tv_sec - mid11.tv_sec +
              0.000001 * (mid12.tv_usec - mid11.tv_usec);

      if (end.tv_sec - start.tv_sec +0.000001 * (end.tv_usec - start.tv_usec)>MaxCPUTime){
    	  MaxCPUTime=end.tv_sec - start.tv_sec +0.000001 * (end.tv_usec - start.tv_usec);
      }
      if (mid2.tv_sec - mid1.tv_sec +0.000001 * (mid2.tv_usec - mid1.tv_usec)>MaxQLDTime){
    	  MaxQLDTime=mid2.tv_sec - mid1.tv_sec +0.000001 * (mid2.tv_usec - mid1.tv_usec);
      }
      if (mid4.tv_sec - mid3.tv_sec +0.000001 * (mid4.tv_usec - mid3.tv_usec)>MaxinvariantpartTime){
    	  MaxinvariantpartTime=mid4.tv_sec - mid3.tv_sec +0.000001 * (mid4.tv_usec - mid3.tv_usec);
      }
      if (mid6.tv_sec - mid5.tv_sec +0.000001 * (mid6.tv_usec - mid5.tv_usec)>MaxConstraintTime){
    	  MaxConstraintTime=mid6.tv_sec - mid5.tv_sec +0.000001 * (mid6.tv_usec - mid5.tv_usec);
      }
      if (mid8.tv_sec - mid7.tv_sec +0.000001 * (mid8.tv_usec - mid7.tv_usec)>MaxObjTime){
    	  MaxObjTime=mid8.tv_sec - mid7.tv_sec +0.000001 * (mid8.tv_usec - mid7.tv_usec);
      }
      if (mid10.tv_sec - mid9.tv_sec +0.000001 * (mid10.tv_usec - mid9.tv_usec)>MaxInterpolTime){
    	  MaxInterpolTime=mid10.tv_sec - mid9.tv_sec +0.000001 * (mid10.tv_usec - mid9.tv_usec);
      }
      if (mid12.tv_sec - mid11.tv_sec +0.000001 * (mid12.tv_usec - mid11.tv_usec)>MaxrefTime){
    	  MaxrefTime=mid12.tv_sec - mid11.tv_sec +0.000001 * (mid12.tv_usec - mid11.tv_usec);
      }
      CurrentCPUTime=CurrentQLDTime+CurrentinvariantpartTime+CurrentConstraintTime+CurrentObjTime+CurrentInterpolTime+CurrentrefTime;
      MaxCPUTime=MaxQLDTime+MaxinvariantpartTime+MaxConstraintTime+MaxObjTime+MaxInterpolTime+MaxrefTime;
      ++itt;
      if (itt==10){
    	  CurrentCPUTime/=10;
    	  CurrentQLDTime/=10;
    	  CurrentinvariantpartTime/=10;
    	  CurrentConstraintTime/=10;
    	  CurrentObjTime/=10;
    	  CurrentInterpolTime/=10;
    	  CurrentrefTime/=10;
    	  itt=0;
    	  std::cout << "mid loop time : " << CurrentCPUTime*1000000
    			    << " us, whose \n LSSOL :" << CurrentQLDTime*1000000
    			    << " us, \n warmstart :" << CurrentinvariantpartTime*1000000
    			    << " us, \n constraints :" << CurrentConstraintTime*1000000
    			    << " us, \n objective :" << CurrentObjTime*1000000
    			    << " us, \n interpolation :" << CurrentInterpolTime*1000000
    			    << " us, \n ref + preview :" << CurrentrefTime*1000000
    			    << " us " << std::endl<< std::endl;

    	  std::cout << "max loop time : " << MaxCPUTime*1000000
    			    << " us, whose \n whose LSSOL :" << MaxQLDTime*1000000
    			    << " us, \n warmstart :" << MaxinvariantpartTime*1000000
    			    << " us, \n constraints :" << MaxConstraintTime*1000000
    			    << " us, \n objective :" << MaxObjTime*1000000
    			    << " us, \n interpolation :" << MaxInterpolTime*1000000
    			    << " us, \n ref + preview :" << MaxrefTime*1000000
    			    << " us " << std::endl<< std::endl<< std::endl<< std::endl;
    	  CurrentCPUTime=CurrentQLDTime=CurrentinvariantpartTime=CurrentConstraintTime=CurrentObjTime=CurrentInterpolTime=CurrentrefTime=0;
      }


    }


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


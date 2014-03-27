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

#ifndef METAPOD_INCLUDES
#define METAPOD_INCLUDES
  #include <metapod/tools/print.hh>
  #include <metapod/tools/initconf.hh>
  #include <metapod/algos/rnea.hh>
  #include <Eigen/StdVector>
#endif

#include <Debug.hh>
using namespace std;
using namespace PatternGeneratorJRL;
using namespace metapod;

double filterprecision(double adb)
{
  if (fabs(adb)<1e-7)
    return 0.0;

  if (fabs(adb)>1e7)
    return 1e7 ;

  double ladb2 = adb * 1e7;
  double lintadb2 = trunc(ladb2);
  return lintadb2/1e7;
}

ZMPVelocityReferencedQP::ZMPVelocityReferencedQP(SimplePluginManager *SPM,
    string , CjrlHumanoidDynamicRobot *aHS ) :
    ZMPRefTrajectoryGeneration(SPM),
    Robot_(0),SupportFSM_(0),OrientPrw_(0),VRQPGenerator_(0),IntermedData_(0),
    RFI_(0),Problem_(),Solution_(),OFTG_DF_(0),OFTG_control_(0)
{
  Running_ = false ;
  TimeBuffer_ = 0.04 ;
  QP_T_ = 0.1 ;
  QP_N_ = 16 ;
  m_SamplingPeriod = 0.005 ;
  InterpolationPeriod_ = QP_T_/20 ;
  StepPeriod_ = 0.8 ;
  SSPeriod = 0.7 ;
  DSPeriod = 0.1 ;
  FeetDistance = 0.2 ;
  StepHeight = 0.05 ;
  CoMHeight_ = 0.814 ;
  PerturbationOccured_ = false ;
  UpperTimeLimitToUpdate_ = 0.0 ;
  RobotMass_ = aHS->mass() ;
  Solution_.useWarmStart=false ;

  // Create and initialize online interpolation of feet trajectories
  RFI_ = new RelativeFeetInequalities( SPM,aHS );

  // Save the reference to HDR
  HDR_ = aHS ;

  // Create and initialize the finite state machine for support sequences
  SupportFSM_ = new SupportFSM();
  SupportFSM_->StepPeriod( StepPeriod_ );
  SupportFSM_->DSPeriod( 1e9 ); // period during the robot move at 0.0 com speed
  SupportFSM_->DSSSPeriod( StepPeriod_ );
  SupportFSM_->NbStepsSSDS( 2 ); // number of previw step
  SupportFSM_->SamplingPeriod( QP_T_ );

  // Create and initialize preview of orientations
  OrientPrw_ = new OrientationsPreview( aHS->rootJoint() );
  OrientPrw_->SamplingPeriod( QP_T_ );
  OrientPrw_->NbSamplingsPreviewed( QP_N_ );
  OrientPrw_->SSLength( SupportFSM_->StepPeriod() );
  COMState CurrentTrunkState;
  OrientPrw_->CurrentTrunkState( CurrentTrunkState );

  // Initialize  the 2D LIPM
  LIPM_control_.SetSimulationControlPeriod( QP_T_ );
  LIPM_control_.SetRobotControlPeriod( m_SamplingPeriod );
  LIPM_control_.InitializeSystem();

  // Initialize  the 2D LIPM
  LIPM_control_subsampled_.SetSimulationControlPeriod( QP_T_ );
  LIPM_control_subsampled_.SetRobotControlPeriod( InterpolationPeriod_ );
  LIPM_control_subsampled_.InitializeSystem();

  // Initialize  the 2D LIPM
  LIPM_DF_.SetSimulationControlPeriod( QP_T_ );
  LIPM_DF_.SetRobotControlPeriod( m_SamplingPeriod );
  LIPM_DF_.InitializeSystem();

  // Initialize  the 2D LIPM
  LIPM_DF_subsampled_.SetSimulationControlPeriod( QP_T_ );
  LIPM_DF_subsampled_.SetRobotControlPeriod( InterpolationPeriod_ );
  LIPM_DF_subsampled_.InitializeSystem();

  // Create and initialize simplified robot model
  Robot_ = new RigidBodySystem( SPM, aHS, SupportFSM_ );
  Robot_->Mass( aHS->mass() );
  Robot_->LeftFoot().Mass( 0.0 );
  Robot_->RightFoot().Mass( 0.0 );
  Robot_->NbSamplingsPreviewed( QP_N_ );
  Robot_->SamplingPeriodSim( QP_T_ );
  Robot_->SamplingPeriodAct( m_SamplingPeriod );
  Robot_->CoMHeight( CoMHeight_ );
  Robot_->multiBody(false);
  Robot_->initialize( );

  IntermedData_ = new IntermedQPMat();

  VRQPGenerator_ = new GeneratorVelRef( SPM, IntermedData_, Robot_, RFI_ );
  VRQPGenerator_->NbPrwSamplings( QP_N_ );
  VRQPGenerator_->SamplingPeriodPreview( QP_T_ );
  VRQPGenerator_->SamplingPeriodControl( m_SamplingPeriod );
  VRQPGenerator_->ComHeight( CoMHeight_ );
  VRQPGenerator_->initialize_matrices();
  VRQPGenerator_->Ponderation( 1.0, INSTANT_VELOCITY );
  VRQPGenerator_->Ponderation( 0.000001, COP_CENTERING );
  VRQPGenerator_->Ponderation( 0.00001, JERK_MIN );

  // Create and initialize online interpolation of feet trajectories:
  // ----------------------------------------------------------------
  OFTG_DF_ = new OnLineFootTrajectoryGeneration(SPM,HDR_->leftFoot());
  OFTG_DF_->InitializeInternalDataStructures();
  OFTG_DF_->SetSingleSupportTime( SSPeriod );
  OFTG_DF_->SetDoubleSupportTime( DSPeriod );
  OFTG_DF_->SetSamplingPeriod( InterpolationPeriod_ );
  OFTG_DF_->QPSamplingPeriod( QP_T_ );
  OFTG_DF_->NbSamplingsPreviewed( QP_N_ );
  OFTG_DF_->FeetDistance( FeetDistance );
  OFTG_DF_->StepHeight( StepHeight );

  OFTG_control_ = new OnLineFootTrajectoryGeneration(SPM,HDR_->leftFoot());
  OFTG_control_->InitializeInternalDataStructures();
  OFTG_control_->SetSingleSupportTime( SSPeriod );
  OFTG_control_->SetDoubleSupportTime( DSPeriod );
  OFTG_control_->SetSamplingPeriod( m_SamplingPeriod );
  OFTG_control_->QPSamplingPeriod( QP_T_ );
  OFTG_control_->NbSamplingsPreviewed( QP_N_ );
  OFTG_control_->FeetDistance( FeetDistance );
  OFTG_control_->StepHeight( StepHeight );

  // Create and initialize the class that compute the simplify inverse kinematics :
  // ------------------------------------------------------------------------------
  ComAndFootRealization_ = new ComAndFootRealizationByGeometry( (PatternGeneratorInterfacePrivate*) SPM );
  ComAndFootRealization_->setHumanoidDynamicRobot(aHS);
  ComAndFootRealization_->SetHeightOfTheCoM(0.0);// seems weird...
  ComAndFootRealization_->setSamplingPeriod(InterpolationPeriod_);
  ComAndFootRealization_->Initialization();

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

  // Initialization of the Kajita preview controls (ICRA 2003).
  MAL_MATRIX_RESIZE(m_deltax,3,1);  MAL_MATRIX_RESIZE(m_deltay,3,1);
  PC_ = new PreviewControl(SPM, OptimalControllerSolver::MODE_WITH_INITIALPOS, false);
  PC_->SetPreviewControlTime (QP_T_*QP_N_ - QP_T_/m_SamplingPeriod * InterpolationPeriod_);
  PC_->SetSamplingPeriod (InterpolationPeriod_);
  PC_->SetHeightOfCoM(CoMHeight_);

	// init of the buffer for the kajita's dynamic filter

    // number of sample inside one iteration of the preview control
	NbSampleControl_ = (unsigned)(QP_T_/m_SamplingPeriod) ;
  NbSampleInterpolation_ = (unsigned)(QP_T_/InterpolationPeriod_) ;

    // size = numberOfIterationOfThePreviewControl * NumberOfSample + Margin
  ZMPTraj_deq_.resize( QP_N_ * NbSampleInterpolation_+20);
  COMTraj_deq_.resize( QP_N_ * NbSampleInterpolation_+20);
  tmpCoM_.resize(QP_N_ * NbSampleInterpolation_ + 20);
  tmpZMP_.resize(QP_N_ * NbSampleInterpolation_ + 20);

  ConfigurationTraj_.resize( QP_N_ * NbSampleInterpolation_ );
  VelocityTraj_.resize( QP_N_ * NbSampleInterpolation_ );
  AccelerationTraj_.resize( QP_N_ * NbSampleInterpolation_ );

  DeltaZMPMBPositions_.resize ( QP_N_ * NbSampleInterpolation_ );

    // Initialization of the configuration vectors
  PreviousConfiguration_ = aHS->currentConfiguration() ;
  PreviousVelocity_ = aHS->currentVelocity();
  PreviousAcceleration_ = aHS->currentAcceleration();

  ComStateBuffer_.resize(NbSampleControl_);

  Once_ = true ;
  DInitX_ = 0 ;
  DInitY_ = 0 ;
}


ZMPVelocityReferencedQP::~ZMPVelocityReferencedQP()
{

  if (VRQPGenerator_!=0)
  {
    delete VRQPGenerator_;
    VRQPGenerator_ = 0 ;
  }

  if (SupportFSM_!=0)
  {
      delete SupportFSM_;
      SupportFSM_ = 0 ;
  }

  if (RFI_!=0)
  {
    delete RFI_;
    RFI_ = 0 ;
  }

  if (OrientPrw_!=0)
  {
    delete OrientPrw_;
    OrientPrw_ = 0 ;
  }

  if (Robot_!=0)
  {
    delete Robot_;
    Robot_ = 0 ;
  }

  if (IntermedData_!=0)
  {
    delete IntermedData_;
    IntermedData_ = 0 ;
  }

  if (ComAndFootRealization_!=0){
    delete ComAndFootRealization_;
    ComAndFootRealization_ = 0 ;
  }

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
  UpperTimeLimitToUpdate_ = 0.0;

  FootAbsolutePosition CurrentLeftFootAbsPos, CurrentRightFootAbsPos;

  // Set the internal state of the ZMPRefTrajectory object.
  m_OnLineMode = true;
  EndingPhase_ = false;
  TimeToStopOnLineMode_ = -1.0;

  // INITIALIZE FEET POSITIONS:
  // --------------------------
  CurrentLeftFootAbsPos = InitLeftFootAbsolutePosition;
  CurrentRightFootAbsPos = InitRightFootAbsolutePosition;

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
  m_CurrentTime = 0;
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
  //TODO check init with left foot (divergence)
  support_state_t CurrentSupport;
  CurrentSupport.Phase = DS;
  CurrentSupport.Foot = RIGHT;
  CurrentSupport.TimeLimit = 1e9;
  CurrentSupport.NbStepsLeft = 1;
  CurrentSupport.StateChanged = false;
  CurrentSupport.X   = CurrentRightFootAbsPos.x; //0.0 ;
  CurrentSupport.Y   = CurrentRightFootAbsPos.y; //0.1 ;
  CurrentSupport.Yaw = CurrentRightFootAbsPos.theta*M_PI/180; //0.0 ;
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
  LIPM_control_.SetComHeight(lStartingCOMState.z[0]);
  LIPM_control_.InitializeSystem();
  LIPM_control_(CoM);
  //--
  LIPM_control_subsampled_.SetComHeight(lStartingCOMState.z[0]);
  LIPM_control_subsampled_.InitializeSystem();
  LIPM_control_subsampled_(CoM);
  //--
  LIPM_DF_.SetComHeight(lStartingCOMState.z[0]);
  LIPM_DF_.InitializeSystem();
  LIPM_DF_(CoM);
  //--
  LIPM_DF_subsampled_.SetComHeight(lStartingCOMState.z[0]);
  LIPM_DF_subsampled_.InitializeSystem();
  LIPM_DF_subsampled_(CoM);
  //--
  IntermedData_->CoM(LIPM_control_());

  // Initialize preview of orientations
  OrientPrw_->CurrentTrunkState( lStartingCOMState );

  // BUILD CONSTANT PART OF THE OBJECTIVE:
  // -------------------------------------
  Problem_.reset();
  Problem_.nbInvariantRows(2*QP_N_);
  Problem_.nbInvariantCols(2*QP_N_);
  VRQPGenerator_->build_invariant_part( Problem_ );

  // initialize intermed data needed during the interpolation
  InitStateLIPMcontrol_ = LIPM_control_.GetState() ;
  InitStateOrientPrw_ = OrientPrw_->CurrentTrunkState() ;
  FinalStateOrientPrw_ = OrientPrw_->CurrentTrunkState() ;

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
  {
    return;
  }

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
    IntermedData_->CoM( LIPM_control_() );

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
    {
  	  VRQPGenerator_->compute_warm_start( Solution_ );//TODO: Move to update_problem or build_constraints?
    }
    Problem_.solve( QLD, Solution_, NONE );
    if(Solution_.Fail>0)
    {
      Problem_.dump( time );
    }

    // INITIALZE INTERPOLATION:
    // ------------------------
    CurrentIndex_ = FinalCOMTraj_deq.size();
    solution_ = Solution_ ;
    for (unsigned i = 0 ; i < CurrentIndex_ ; i++)
    {
      ZMPTraj_deq_[i] = FinalZMPTraj_deq[i] ;
      COMTraj_deq_[i] = tmpCoM_[i] = FinalCOMTraj_deq[i] ;
    }
    LeftFootTraj_deq_ = FinalLeftFootTraj_deq ;
    RightFootTraj_deq_ = FinalRightFootTraj_deq ;
    FinalZMPTraj_deq.resize( NbSampleControl_ + CurrentIndex_ );
    FinalCOMTraj_deq.resize( NbSampleControl_ + CurrentIndex_ );

    // INTERPOLATION
    ControlInterpolation( FinalZMPTraj_deq, FinalLeftFootTraj_deq,
                          FinalRightFootTraj_deq, time) ;
    DynamicFilterInterpolation( FinalCOMTraj_deq, time) ;

    // DYNAMIC FILTER
    // --------------
    //DynamicFilter( time, FinalCOMTraj_deq );

    // PREPARATION OF THE FOLLOWING STEP
    // ---------------------------------
    LIPM_DF_.setState(FinalCOMTraj_deq[NbSampleControl_ + CurrentIndex_ - 1]);
    LIPM_DF_subsampled_.setState(FinalCOMTraj_deq[NbSampleControl_ + CurrentIndex_ - 1]) ;
    OrientPrw_->CurrentTrunkState(FinalStateOrientPrw_);

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
  //----------"Real-time" loop---------
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

void ZMPVelocityReferencedQP::ControlInterpolation(
          std::deque<ZMPPosition> & FinalZMPTraj_deq,
		      std::deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
		      std::deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
		      double time)
{
  InitStateLIPMcontrol_ = LIPM_control_.GetState() ;
  // INTERPOLATE CoM AND ZMP TRAJECTORIES:
  // -------------------------------------
  CoMZMPInterpolation(FinalZMPTraj_deq, tmpCoM_,
      FinalLeftFootTraj_deq, FinalRightFootTraj_deq,
      &solution_, &LIPM_control_, NbSampleControl_, 0);

  // INTERPOLATE TRUNK ORIENTATION:
  // ------------------------------
  InitStateOrientPrw_ = OrientPrw_->CurrentTrunkState() ;
  OrientPrw_->interpolate_trunk_orientation( time, CurrentIndex_,
        m_SamplingPeriod, solution_.SupportStates_deq,
        tmpCoM_ );
  FinalStateOrientPrw_ = OrientPrw_->CurrentTrunkState();

  // INTERPOLATE THE COMPUTED FOOT POSITIONS:
  // ----------------------------------------
  OFTG_control_->interpolate_feet_positions( time,
          solution_.SupportStates_deq, solution_,
          solution_.SupportOrientations_deq,
          FinalLeftFootTraj_deq, FinalRightFootTraj_deq);

  return ;
}

void ZMPVelocityReferencedQP::DynamicFilterInterpolation(
              std::deque<COMState> & FinalCOMTraj_deq,
              double time)
{
  LIPM_control_subsampled_.setState(InitStateLIPMcontrol_) ;
  for ( int i = 0 ; i < QP_N_ ; i++ )
  {
    // INTERPOLATE ZMP TRAJECTORIES:
    // -----------------------------
    CoMZMPInterpolation(ZMPTraj_deq_, tmpCoM_,
      LeftFootTraj_deq_, RightFootTraj_deq_,
      &solution_, &LIPM_control_subsampled_,
      NbSampleInterpolation_, i);
  }

  // INTERPOLATE COM TRAJECTORIES:
  // -----------------------------
  for ( int i = 0 ; i < QP_N_ ; i++ )
  {
    CoMZMPInterpolation(tmpZMP_, COMTraj_deq_,
      LeftFootTraj_deq_, RightFootTraj_deq_,
      &solution_, &LIPM_DF_subsampled_,
      NbSampleInterpolation_, i);
  }
  CoMZMPInterpolation(tmpZMP_, FinalCOMTraj_deq,
    LeftFootTraj_deq_, RightFootTraj_deq_,
    &solution_, &LIPM_DF_, NbSampleControl_, 0);


  // INTERPOLATE TRUNK ORIENTATION and FEET POSITIONS :
  // --------------------------------------------------
  OrientPrw_->CurrentTrunkState(InitStateOrientPrw_) ;
  OrientPrw_->interpolate_trunk_orientation( time, CurrentIndex_,
        m_SamplingPeriod, solution_.SupportStates_deq,
        FinalCOMTraj_deq );

  OrientPrw_->CurrentTrunkState(InitStateOrientPrw_) ;
  for ( int i = 0 ; i < QP_N_ ; i++ )
  {
    OrientPrw_->interpolate_trunk_orientation( time + i * QP_T_,
        CurrentIndex_ + i * NbSampleInterpolation_, InterpolationPeriod_,
        solution_.SupportStates_deq, COMTraj_deq_ );

    OFTG_DF_->interpolate_feet_positions( time + i * QP_T_,
          solution_.SupportStates_deq, solution_,
          solution_.SupportOrientations_deq,
          LeftFootTraj_deq_, RightFootTraj_deq_);
    solution_.SupportStates_deq.pop_front();
  }
  return ;
}

int ZMPVelocityReferencedQP::DynamicFilter(double time, std::deque<COMState> & FinalCOMTraj_deq)
{
  const unsigned int N = NbSampleInterpolation_ * QP_N_ ;
  // \brief calculate, from the CoM computed by the preview control,
  //    the corresponding articular position, velocity and acceleration
  // ------------------------------------------------------------------
  for(unsigned int i = 0 ; i <  N ; i++ ){
    CallToComAndFootRealization(
      COMTraj_deq_[CurrentIndex_+i],
      LeftFootTraj_deq_ [CurrentIndex_+i],
      RightFootTraj_deq_ [CurrentIndex_+i],
      ConfigurationTraj_[i],
      VelocityTraj_[i],
      AccelerationTraj_[i],
      i);
  }
  /// \brief Debug Purpose
  /// --------------------
  ofstream aof;
  string aFileName;
  ostringstream oss(std::ostringstream::ate);
  static int iteration = 0;
  int iteration100 = (int)iteration/100;
  int iteration10 = (int)(iteration - iteration100*100)/10;
  int iteration1 = (int)(iteration - iteration100*100 - iteration10*10 );

  /// \brief Debug Purpose
  /// --------------------
  oss.str("TestHerdt2010footcom");
  oss << "_" << iteration100 << iteration10 << iteration1 << ".dat";
  aFileName = oss.str();
  aof.open(aFileName.c_str(),ofstream::out);
  aof.close();
  ///----
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (unsigned int i = 0 ; i < N ; ++i )
  {
    aof << filterprecision( COMTraj_deq_[i].x[0] ) << " "         // 1
        << filterprecision( COMTraj_deq_[i].y[0] ) << " "         // 2
        << filterprecision( COMTraj_deq_[i].x[1] ) << " "         // 3
        << filterprecision( COMTraj_deq_[i].y[1] ) << " "         // 4
        << filterprecision( COMTraj_deq_[i].x[2] ) << " "         // 5
        << filterprecision( COMTraj_deq_[i].y[2] ) << " "         // 6
        << filterprecision( LeftFootTraj_deq_[i].x ) << " "       // 7
        << filterprecision( LeftFootTraj_deq_[i].y ) << " "       // 8
        << filterprecision( LeftFootTraj_deq_[i].theta * M_PI / 180 ) << " "   // 9
        << filterprecision( RightFootTraj_deq_[i].x ) << " "      //10
        << filterprecision( RightFootTraj_deq_[i].y ) << " "      //11
        << filterprecision( RightFootTraj_deq_[i].theta * M_PI / 180 ) << " "  //12
        << endl ;
  }
  aof.close() ;

  /// \brief Debug Purpose
  /// --------------------
  oss.str("TestHerdt2010DynamicZMPMB.dat");
  aFileName = oss.str();
  if (iteration == 0 )
  {
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
  }
  ///----
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (unsigned int i = 0 ; i < N ; i++ )
  {
    // Apply the RNEA to the metapod multibody and print the result in a log file.
    for(unsigned int j = 0 ; j < ConfigurationTraj_[i].size() ; j++ )
    {
      m_q(j,0) = ConfigurationTraj_[i](j) ;
      m_dq(j,0) = VelocityTraj_[i](j) ;
      m_ddq(j,0) = AccelerationTraj_[i](j) ;
    }
    metapod::rnea< Robot_Model, true >::run(m_robot, m_q, m_dq, m_ddq);

    Node & node = boost::fusion::at_c<Robot_Model::BODY>(m_robot.nodes);
    m_force = node.body.iX0.applyInv (node.joint.f);
    if (Once_){
      DInitX_ = ZMPTraj_deq_[CurrentIndex_].px - ( - m_force.n()[1] / m_force.f()[2] ) ;
      DInitY_ = ZMPTraj_deq_[CurrentIndex_].py - (   m_force.n()[0] / m_force.f()[2] ) ;
      Once_ = false ;
    }
    DeltaZMPMBPositions_[i].px = ZMPTraj_deq_[CurrentIndex_+i].px - ( - m_force.n()[1] / m_force.f()[2] ) - DInitX_ ;
    DeltaZMPMBPositions_[i].py = ZMPTraj_deq_[CurrentIndex_+i].py - (   m_force.n()[0] / m_force.f()[2] ) - DInitY_ ;
    DeltaZMPMBPositions_[i].pz = 0.0 ;
    DeltaZMPMBPositions_[i].theta = 0.0 ;
    DeltaZMPMBPositions_[i].time = time + i * m_SamplingPeriod ;
    DeltaZMPMBPositions_[i].stepType = ZMPTraj_deq_[CurrentIndex_+i].stepType ;

    if (i==0){
      aof << filterprecision( ( - m_force.n()[1] / m_force.f()[2] ) + DInitX_ ) << " "   // 1
      << filterprecision( (   m_force.n()[0] / m_force.f()[2] ) + DInitY_ ) << " "   // 1
      << filterprecision( ZMPTraj_deq_[CurrentIndex_].px ) << " "   // 1
      << filterprecision( ZMPTraj_deq_[CurrentIndex_].py ) << " "   // 1
      << endl ;
    }
  }
  aof.close();

  /// Preview control on the ZMPMBs computed
  /// --------------------------------------
  //init of the Kajita preview control
  PC_->SetPreviewControlTime (QP_T_*QP_N_ - NbSampleControl_*InterpolationPeriod_);
  PC_->SetSamplingPeriod (InterpolationPeriod_);
  PC_->SetHeightOfCoM(CoMHeight_);
  PC_->ComputeOptimalWeights(OptimalControllerSolver::MODE_WITH_INITIALPOS);
  for(int j=0;j<3;j++)
  {
    m_deltax(j,0) = 0 ;
    m_deltay(j,0) = 0 ;
  }
  double aSxzmp (0) , aSyzmp(0);
  double deltaZMPx (0) , deltaZMPy (0) ;

  // calcul of the preview control along the "ZMPTraj_deq_"
  for (unsigned i = 0 ; i < NbSampleControl_ ; i++ )
  {
    PC_->OneIterationOfPreview(m_deltax,m_deltay,
                                aSxzmp,aSyzmp,
                                DeltaZMPMBPositions_,i,
                                deltaZMPx, deltaZMPy, false);
    for(int j=0;j<3;j++)
    {
      ComStateBuffer_[i].x[j] = m_deltax(j,0);
      ComStateBuffer_[i].y[j] = m_deltay(j,0);
    }
  }

  for (unsigned int i = 0 ; i < NbSampleControl_ ; i++)
  {
    for(int j=0;j<3;j++)
    {
      if ( ComStateBuffer_[i].x[j] == ComStateBuffer_[i].x[j] ||
       ComStateBuffer_[i].y[j] == ComStateBuffer_[i].y[j] )
      {
        FinalCOMTraj_deq[CurrentIndex_+i].x[j] += ComStateBuffer_[i].x[j] ;
        FinalCOMTraj_deq[CurrentIndex_+i].y[j] += ComStateBuffer_[i].y[j] ;
      }
      else
      {
        //cout << "kajita2003 preview control diverged\n" ;
      }
    }
  }

  /// \brief Debug Purpose
  /// --------------------
  oss.str("TestHerdt2010DynamicDZMP");
  oss << "_" << iteration100 << iteration10 << iteration1 << ".dat";
  aFileName = oss.str();
  aof.open(aFileName.c_str(),ofstream::out);
  aof.close();
  ///----
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (unsigned int i = 0 ; i < DeltaZMPMBPositions_.size() ; ++i )
  {
    aof << filterprecision( DeltaZMPMBPositions_[i].px ) << " "   // 1
        << filterprecision( DeltaZMPMBPositions_[i].py ) << " "   // 2
        << endl ;
  }

  /// \brief Debug Purpose
  /// --------------------
  oss.str("TestHerdt2010DynamicDCOM");
  oss << "_" << iteration100 << iteration10 << iteration1 << ".dat";
  aFileName = oss.str();
  aof.open(aFileName.c_str(),ofstream::out);
  aof.close();
  ///----
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (unsigned int i = 0 ; i < ComStateBuffer_.size() ; ++i )
  {
    aof << filterprecision( ComStateBuffer_[i].x[0] ) << " "   // 1
        << filterprecision( ComStateBuffer_[i].y[0] ) << " "   // 2
        << filterprecision( ComStateBuffer_[i].x[1] ) << " "   // 1
        << filterprecision( ComStateBuffer_[i].y[1] ) << " "   // 2
        << filterprecision( ComStateBuffer_[i].x[2] ) << " "   // 1
        << filterprecision( ComStateBuffer_[i].y[2] ) << " "   // 2
        << endl ;
  }
  iteration++;

  return 0;
}


void ZMPVelocityReferencedQP::CallToComAndFootRealization(COMState & acomp,
     FootAbsolutePosition & aLeftFAP,
     FootAbsolutePosition & aRightFAP,
     MAL_VECTOR_TYPE(double) & CurrentConfiguration,
     MAL_VECTOR_TYPE(double) & CurrentVelocity,
     MAL_VECTOR_TYPE(double) & CurrentAcceleration,
     unsigned IterationNumber)
{

  // New scheme for WPG v3.0
  // We call the object in charge of generating the whole body
  // motion  ( for a given CoM and Feet points)  before applying the second filter.
  MAL_VECTOR_DIM(aCOMState,double,6);
  MAL_VECTOR_DIM(aCOMSpeed,double,6);
  MAL_VECTOR_DIM(aCOMAcc,double,6);

  aCOMState(0) = acomp.x[0];
  aCOMState(1) = acomp.y[0];
  aCOMState(2) = acomp.z[0];
  aCOMState(3) = acomp.roll[0];
  aCOMState(4) = acomp.pitch[0];
  aCOMState(5) = acomp.yaw[0];

  aCOMSpeed(0) = acomp.x[1];
  aCOMSpeed(1) = acomp.y[1];
  aCOMSpeed(2) = acomp.z[1];
  aCOMSpeed(3) = acomp.roll[1];
  aCOMSpeed(4) = acomp.roll[1];
  aCOMSpeed(5) = acomp.roll[1];

  aCOMAcc(0) = acomp.x[2];
  aCOMAcc(1) = acomp.y[2];
  aCOMAcc(2) = acomp.z[2];
  aCOMAcc(3) = acomp.roll[2];
  aCOMAcc(4) = acomp.roll[2];
  aCOMAcc(5) = acomp.roll[2];

  MAL_VECTOR_DIM(aLeftFootPosition,double,5);
  MAL_VECTOR_DIM(aRightFootPosition,double,5);

  aLeftFootPosition(0) = aLeftFAP.x;
  aLeftFootPosition(1) = aLeftFAP.y;
  aLeftFootPosition(2) = aLeftFAP.z;
  aLeftFootPosition(3) = aLeftFAP.theta;
  aLeftFootPosition(4) = aLeftFAP.omega;

  aRightFootPosition(0) = aRightFAP.x;
  aRightFootPosition(1) = aRightFAP.y;
  aRightFootPosition(2) = aRightFAP.z;
  aRightFootPosition(3) = aRightFAP.theta;
  aRightFootPosition(4) = aRightFAP.omega;

  if (IterationNumber == 0){
    CurrentConfiguration = HDR_->currentConfiguration();
    CurrentVelocity = HDR_->currentConfiguration();
    CurrentAcceleration = HDR_->currentConfiguration();
  }else{
    CurrentConfiguration = PreviousConfiguration_ ;
    CurrentVelocity = PreviousVelocity_ ;
    CurrentAcceleration = PreviousAcceleration_ ;
  }
  ComAndFootRealization_->ComputePostureForGivenCoMAndFeetPosture(aCOMState, aCOMSpeed, aCOMAcc,
								    aLeftFootPosition,
								    aRightFootPosition,
								    CurrentConfiguration,
								    CurrentVelocity,
								    CurrentAcceleration,
								    IterationNumber,
								    0);
  if(IterationNumber == NbSampleInterpolation_-1 ){
    HDR_->currentConfiguration(CurrentConfiguration);
    HDR_->currentConfiguration(CurrentVelocity);
    HDR_->currentConfiguration(CurrentAcceleration);
  }else{
    PreviousConfiguration_ = CurrentConfiguration ;
    PreviousVelocity_ = CurrentVelocity ;
    PreviousAcceleration_ = CurrentAcceleration ;
  }

  return ;
}

void ZMPVelocityReferencedQP::CoMZMPInterpolation(
          std::deque<ZMPPosition> & ZMPPositions,
		      std::deque<COMState> & COMTraj_deq ,
		      std::deque<FootAbsolutePosition> & LeftFootTraj_deq,
		      std::deque<FootAbsolutePosition> & RightFootTraj_deq,
		      solution_t * Solution,
          LinearizedInvertedPendulum2D * LIPM,
		      unsigned numberOfSample,
		      int IterationNumber)
{
  if(Solution->SupportStates_deq.size() && Solution->SupportStates_deq[0].NbStepsLeft == 0)
  {
     double jx = (LeftFootTraj_deq[0].x + RightFootTraj_deq[0].x)/2 - LIPM->GetState().x[0];
     double jy = (LeftFootTraj_deq[0].y + RightFootTraj_deq[0].y)/2 - LIPM->GetState().y[0];
     if(fabs(jx) < 1e-3 && fabs(jy) < 1e-3) { Running_ = false; }
     const double tf = 0.75;
     jx = 6/(tf*tf*tf)*(jx - tf*LIPM->GetState().x[1] - (tf*tf/2)*LIPM->GetState().x[2]);
     jy = 6/(tf*tf*tf)*(jy - tf*LIPM->GetState().y[1] - (tf*tf/2)*LIPM->GetState().y[2]);
     LIPM->Interpolation( COMTraj_deq, ZMPPositions, CurrentIndex_ + IterationNumber * numberOfSample,
         jx, jy);
     LIPM->OneIteration( jx, jy );
  }
  else
  {
     Running_ = true;
     LIPM->Interpolation( COMTraj_deq, ZMPPositions, CurrentIndex_ + IterationNumber * numberOfSample,
         Solution->Solution_vec[IterationNumber], Solution->Solution_vec[IterationNumber+QP_N_] );
     LIPM->OneIteration( Solution->Solution_vec[IterationNumber],Solution->Solution_vec[IterationNumber+QP_N_] );
  }
  return ;
}


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
  Maximilien Naveau,
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
    Robot_(0),SupportFSM_(0),OrientPrw_(0),OrientPrw_DF_(0),
    VRQPGenerator_(0),IntermedData_(0),RFI_(0),Problem_(),
    Solution_(),OFTG_DF_(0),OFTG_control_(0)
{
  Running_ = false ;
  TimeBuffer_ = 0.04 ;
  QP_T_ = 0.1 ;
  QP_N_ = 16 ;
  m_SamplingPeriod = 0.005 ;
  InterpolationPeriod_ = QP_T_/20;
  StepPeriod_ = 0.8 ;
  SSPeriod_ = 0.7 ;
  DSPeriod_ = 0.1 ;
  FeetDistance_ = 0.2 ;
  StepHeight_ = 0.05 ;
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

  // Create and initialize preview of orientations
  OrientPrw_DF_ = new OrientationsPreview( aHS->rootJoint() );
  OrientPrw_DF_->SamplingPeriod( QP_T_ );
  OrientPrw_DF_->NbSamplingsPreviewed( QP_N_ );
  OrientPrw_DF_->SSLength( SupportFSM_->StepPeriod() );
  OrientPrw_DF_->CurrentTrunkState( CurrentTrunkState );


  // Initialize  the 2D LIPM
  LIPM_.SetSimulationControlPeriod( QP_T_ );
  LIPM_.SetRobotControlPeriod( m_SamplingPeriod );
  LIPM_.InitializeSystem();

  // Initialize  the 2D LIPM
  LIPM_subsampled_.SetSimulationControlPeriod( QP_T_ );
  LIPM_subsampled_.SetRobotControlPeriod( InterpolationPeriod_ );
  LIPM_subsampled_.InitializeSystem();

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
  OFTG_DF_->SetSingleSupportTime( SSPeriod_ );
  OFTG_DF_->SetDoubleSupportTime( DSPeriod_ );
  OFTG_DF_->SetSamplingPeriod( InterpolationPeriod_ );
  OFTG_DF_->QPSamplingPeriod( QP_T_ );
  OFTG_DF_->NbSamplingsPreviewed( QP_N_ );
  OFTG_DF_->FeetDistance( FeetDistance_ );
  OFTG_DF_->StepHeight( StepHeight_ );

  OFTG_control_ = new OnLineFootTrajectoryGeneration(SPM,HDR_->leftFoot());
  OFTG_control_->InitializeInternalDataStructures();
  OFTG_control_->SetSingleSupportTime( SSPeriod_ );
  OFTG_control_->SetDoubleSupportTime( DSPeriod_ );
  OFTG_control_->SetSamplingPeriod( m_SamplingPeriod );
  OFTG_control_->QPSamplingPeriod( QP_T_ );
  OFTG_control_->NbSamplingsPreviewed( QP_N_ );
  OFTG_control_->FeetDistance( FeetDistance_ );
  OFTG_control_->StepHeight( StepHeight_ );

  // Create and initialize the class that compute the simplify inverse kinematics :
  // ------------------------------------------------------------------------------
  ComAndFootRealization_ = new ComAndFootRealizationByGeometry( (PatternGeneratorInterfacePrivate*) SPM );
  ComAndFootRealization_->setHumanoidDynamicRobot(aHS);
  ComAndFootRealization_->SetHeightOfTheCoM(CoMHeight_);// seems weird...
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
  tmpCoM_.resize(QP_N_ * NbSampleControl_ + 20);
  tmpZMP_.resize(QP_N_ * NbSampleControl_ + 20);

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

  if (OrientPrw_DF_!=0)
  {
    delete OrientPrw_DF_;
    OrientPrw_DF_ = 0 ;
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
  CurrentSupport.Foot = LEFT;
  CurrentSupport.TimeLimit = 1e9;
  CurrentSupport.NbStepsLeft = 1;
  CurrentSupport.StateChanged = false;
  CurrentSupport.X   = CurrentLeftFootAbsPos.x; //0.0 ;
  CurrentSupport.Y   = CurrentLeftFootAbsPos.y; //0.1 ;
  CurrentSupport.Yaw = CurrentLeftFootAbsPos.theta*M_PI/180; //0.0 ;
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
  LIPM_.SetComHeight(lStartingCOMState.z[0]);
  LIPM_.InitializeSystem();
  LIPM_(CoM);
  //--
  LIPM_subsampled_.SetComHeight(lStartingCOMState.z[0]);
  LIPM_subsampled_.InitializeSystem();
  LIPM_subsampled_(CoM);
  //--
  IntermedData_->CoM(LIPM_());

  // Initialize preview of orientations
  OrientPrw_->CurrentTrunkState( lStartingCOMState );
  OrientPrw_DF_->CurrentTrunkState( lStartingCOMState );
  // BUILD CONSTANT PART OF THE OBJECTIVE:
  // -------------------------------------
  Problem_.reset();
  Problem_.nbInvariantRows(2*QP_N_);
  Problem_.nbInvariantCols(2*QP_N_);
  VRQPGenerator_->build_invariant_part( Problem_ );

  // initialize intermed data needed during the interpolation
  InitStateLIPM_ = LIPM_.GetState() ;
  InitStateOrientPrw_ = OrientPrw_->CurrentTrunkState() ;
  FinalCurrentStateOrientPrw_ = OrientPrw_->CurrentTrunkState() ;

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
    IntermedData_->CoM( LIPM_() );

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

    // INITIALIZE INTERPOLATION:
    // ------------------------
    CurrentIndex_ = FinalCOMTraj_deq.size();
    solution_ = Solution_ ;
    for (unsigned i = 0 ; i < CurrentIndex_ ; i++)
    {
      ZMPTraj_deq_[i] = FinalZMPTraj_deq[i] ;
      COMTraj_deq_[i] = FinalCOMTraj_deq[i] ;
    }
    LeftFootTraj_deq_ = FinalLeftFootTraj_deq ;
    RightFootTraj_deq_ = FinalRightFootTraj_deq ;
    FinalZMPTraj_deq.resize( NbSampleControl_ + CurrentIndex_ );
    FinalCOMTraj_deq.resize( NbSampleControl_ + CurrentIndex_ );

    static int iteration = 0;
    if (iteration == 11){
      iteration = 11;
    }

    // INTERPOLATION
    ControlInterpolation( FinalCOMTraj_deq, FinalZMPTraj_deq, FinalLeftFootTraj_deq,
                          FinalRightFootTraj_deq, time) ;
    DynamicFilterInterpolation( time) ;

    // DYNAMIC FILTER
    // --------------
    DynamicFilter( time, FinalCOMTraj_deq );

  /// \brief Debug Purpose
  /// --------------------
  ofstream aof;
  string aFileName;
  ostringstream oss(std::ostringstream::ate);

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
  for (unsigned int i = 0 ; i < NbSampleInterpolation_ * QP_N_ ; ++i )
  {
    aof << filterprecision( COMTraj_deq_[/*CurrentIndex_+*/i].x[0] ) << " "         // 1
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].y[0] ) << " "         // 2
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].z[0] ) << " "         // 3
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].x[1] ) << " "         // 4
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].y[1] ) << " "         // 5
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].z[1] ) << " "         // 6
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].x[2] ) << " "         // 7
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].y[2] ) << " "         // 8
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].z[2] ) << " "         // 9
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].yaw[0] ) << " "       // 10
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].yaw[1] ) << " "       // 11
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].yaw[2] ) << " "       // 12
        << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].x ) << " "       // 13
        << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].y ) << " "       // 14
        << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].z ) << " "       // 15
        << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].theta * M_PI / 180 ) << " "   // 16
        << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].omega * M_PI / 180 ) << " "   // 17
        << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].x ) << " "      //18
        << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].y ) << " "      //19
        << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].z ) << " "  //20
        << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].theta * M_PI / 180 ) << " "  //21
        << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].omega * M_PI / 180 ) << " "  //22
        << filterprecision( solution_.Solution_vec[i/NbSampleInterpolation_] ) << " "  //23
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].roll[0] ) << " "       // 24
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].roll[1] ) << " "       // 25
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].roll[2] ) << " "       // 26
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].pitch[0] ) << " "       // 27
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].pitch[1] ) << " "       // 28
        << filterprecision( COMTraj_deq_[CurrentIndex_+i].pitch[2] ) << " "       // 29
        << filterprecision( i * InterpolationPeriod_ ) << " "       // 30
        << endl ;
  }
  aof.close() ;

	oss.str("TestHerdt2010footcom");
  aFileName = oss.str();
	if (iteration == 0 ){
		aof.open(aFileName.c_str(),ofstream::out);
		aof.close();
	}
  ///----
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (unsigned int i = 0 ; i < (FinalCOMTraj_deq.size()-CurrentIndex_) ; ++i )
  {
    aof << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].x[0] ) << " "         // 1
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].y[0] ) << " "         // 2
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].z[0] ) << " "         // 3
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].x[1] ) << " "         // 4
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].y[1] ) << " "         // 5
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].z[1] ) << " "         // 6
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].x[2] ) << " "         // 7
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].y[2] ) << " "         // 8
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].z[2] ) << " "         // 9
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].yaw[0] ) << " "       // 10
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].yaw[1] ) << " "       // 11
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].yaw[2] ) << " "       // 12
        << filterprecision( FinalLeftFootTraj_deq[CurrentIndex_+i].x ) << " "       // 13
        << filterprecision( FinalLeftFootTraj_deq[CurrentIndex_+i].y ) << " "       // 14
        << filterprecision( FinalLeftFootTraj_deq[CurrentIndex_+i].z ) << " "       // 15
        << filterprecision( FinalLeftFootTraj_deq[CurrentIndex_+i].theta * M_PI / 180 ) << " "   // 16
        << filterprecision( FinalLeftFootTraj_deq[CurrentIndex_+i].omega * M_PI / 180 ) << " "   // 17
        << filterprecision( FinalRightFootTraj_deq[CurrentIndex_+i].x ) << " "      //18
        << filterprecision( FinalRightFootTraj_deq[CurrentIndex_+i].y ) << " "      //19
        << filterprecision( FinalRightFootTraj_deq[CurrentIndex_+i].z ) << " "  //20
        << filterprecision( FinalRightFootTraj_deq[CurrentIndex_+i].theta * M_PI / 180 ) << " "  //21
        << filterprecision( FinalRightFootTraj_deq[CurrentIndex_+i].omega * M_PI / 180 ) << " "  //22
				<< filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].roll[0] ) << " "       // 23
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].roll[1] ) << " "       // 24
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].roll[2] ) << " "       // 25
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].pitch[0] ) << " "       // 26
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].pitch[1] ) << " "       // 27
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].pitch[2] ) << " "       // 28
        << endl ;
  }
  aof.close() ;

  iteration++;

		cout << "----------------------------------------------\n" ;
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
          std::deque<COMState> & FinalCOMTraj_deq,                      // OUTPUT
          std::deque<ZMPPosition> & FinalZMPTraj_deq,                   // OUTPUT
		      std::deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,     // OUTPUT
		      std::deque<FootAbsolutePosition> & FinalRightFootTraj_deq,    // OUTPUT
		      double time)                                                  // INPUT
{
  InitStateLIPM_ = LIPM_.GetState() ;
  InitStateOrientPrw_ = OrientPrw_->CurrentTrunkState() ;

  // INTERPOLATE CoM AND ZMP TRAJECTORIES:
  // -------------------------------------
  CoMZMPInterpolation(FinalZMPTraj_deq, FinalCOMTraj_deq,
      FinalLeftFootTraj_deq, FinalRightFootTraj_deq,
      &solution_, &LIPM_, NbSampleControl_, 0);

  // INTERPOLATE TRUNK ORIENTATION:
  // ------------------------------
  InitStateOrientPrw_ = OrientPrw_->CurrentTrunkState() ;
  OrientPrw_->interpolate_trunk_orientation( time, CurrentIndex_,
        m_SamplingPeriod, solution_.SupportStates_deq,
        FinalCOMTraj_deq );
  FinalCurrentStateOrientPrw_ = OrientPrw_->CurrentTrunkState();
  FinalPreviewStateOrientPrw_ = OrientPrw_->PreviewTrunkState();
  // INTERPOLATE THE COMPUTED FOOT POSITIONS:
  // ----------------------------------------
  OFTG_control_->interpolate_feet_positions( time,
          solution_.SupportStates_deq, solution_,
          solution_.SupportOrientations_deq,
          FinalLeftFootTraj_deq, FinalRightFootTraj_deq);
  return ;
}

void ZMPVelocityReferencedQP::DynamicFilterInterpolation(double time)
{
	// interpolation the position of the com along the whole preview
	vector<double> solFoot;
  LIPM_subsampled_.setState(InitStateLIPM_) ;
  OrientPrw_DF_->CurrentTrunkState(InitStateOrientPrw_) ;
	InterpretSolution(solFoot);
  for ( int i = 0 ; i < QP_N_ ; i++ )
  {
    CoMZMPInterpolation(ZMPTraj_deq_, COMTraj_deq_,
      LeftFootTraj_deq_, RightFootTraj_deq_,
      &solution_, &LIPM_subsampled_,
      NbSampleInterpolation_, i);
  }

	// Copy the solution for the orientation interpolation function
	OFTG_DF_->SetSamplingPeriod( InterpolationPeriod_ );
  solution_t aSolution  = solution_ ;

  for ( int i = 0 ; i < QP_N_ ; i++ )
  {
		OrientPrw_DF_->preview_orientations( time + i * QP_T_, VelRef_,
        SupportFSM_->StepPeriod(),
        LeftFootTraj_deq_, RightFootTraj_deq_,
        aSolution );

    OrientPrw_DF_->interpolate_trunk_orientation( time + i * QP_T_,
        CurrentIndex_ + i * NbSampleInterpolation_, InterpolationPeriod_,
        solution_.SupportStates_deq, COMTraj_deq_ );

		// Modify a copy of the solution to allow "OFTG_DF_->interpolate_feet_positions"
		// to use the correcte feet step previewed
		PrepareSolution(i,solFoot);

		if(solution_.SupportStates_deq.front().Phase != DS){
			int vjskbsrtlb = 0;
		}

		OFTG_DF_->interpolate_feet_positions( time + i * QP_T_,
          solution_.SupportStates_deq, solution_,
          aSolution.SupportOrientations_deq,
          LeftFootTraj_deq_, RightFootTraj_deq_);
    solution_.SupportStates_deq.pop_front();
	}

  OrientPrw_DF_->CurrentTrunkState(FinalCurrentStateOrientPrw_);
  OrientPrw_DF_->CurrentTrunkState(FinalPreviewStateOrientPrw_);
  return ;
}

void ZMPVelocityReferencedQP::InterpretSolution(vector<double> &solFoot)
{
	double PosFootX(0),PosFootY(0),Vx(0),Vy(0),cosTheta(0),sinTheta(0),Sign(0);
	int nbSteps (0);

	support_state_t & LastSupport = solution_.SupportStates_deq.back() ;
	support_state_t & FirstSupport = solution_.SupportStates_deq.front() ;
	nbSteps = LastSupport.StepNumber ;
	Vx = VelRef_.Local.X ;
	Vy = VelRef_.Local.Y ;
	cosTheta = cos(LastSupport.Yaw) ;
	sinTheta = sin(LastSupport.Yaw) ;
	cout << "velref X=" << VelRef_.Local.X << " Y=" << VelRef_.Local.Y<< " Yaw=" << VelRef_.Local.Yaw << " Lastfoot.yaw="<< LastSupport.Yaw <<endl ;
	static int tour (0);
	if ( tour == 58 )
	{
		tour = 58 ;
	}
	tour++;

	if (LastSupport.Phase == DS)
	{
		PosFootX = LastSupport.X ;
		PosFootY = LastSupport.Y ;
		cout << "PosFootX = "<< PosFootX << " " << "PosFootY = "<< PosFootY ;
		cout << endl ;
	}
	else
	{
		if(FirstSupport.Foot == LEFT )
			Sign = 1.0 ;
		else
			Sign = -1.0 ;
		if(LastSupport.Phase == SS && FirstSupport.Phase == DS )
			Sign = -Sign ;
		// compute the next foot step
		PosFootX = (Vx * StepPeriod_ + Sign * sin(LastSupport.Yaw) * FeetDistance_ + solution_.Solution_vec[2*QP_N_ + nbSteps -1]) ;
		PosFootY = (Vy * StepPeriod_ - Sign * cos(LastSupport.Yaw) * FeetDistance_ + solution_.Solution_vec[2*QP_N_ + 2*nbSteps -1]) ;
		cout << "PosFootX = "<< PosFootX << " " << "PosFootY = "<< PosFootY ;
		cout << endl ;
		//ProjectionOnConstraints(PosFootX,PosFootY);
	}

	solFoot.resize ((nbSteps+1)*2,0.0);
	for (int i = 0 ; i < nbSteps ; ++i )
	{
	  solFoot[i] = solution_.Solution_vec[2*QP_N_+i] ;
	  solFoot[nbSteps+1+i] = solution_.Solution_vec[2*QP_N_+nbSteps+i] ;
	}
	solFoot[nbSteps] = PosFootX ;
	solFoot[solFoot.size()-1] = PosFootY ;

	for (unsigned int i = 0 ; i < (solFoot.size()) ; ++i )
	{
		cout << "solFoot["<<i<<"] = "<< solFoot[i] << " " ;
	}
	cout << endl ;

	return ;
}

void ZMPVelocityReferencedQP::ProjectionOnConstraints(double & X, double & Y)
{
	// intialization
	vector<int> active_set ;
	boost_ublas::compressed_matrix<double, boost_ublas::row_major> DX ;
	linear_inequality_t IneqFeet = IntermedData_->Inequalities( INEQ_FEET );
	int nbSteps(0),Sign(1);
	double dx(0),dy(0);

	// definition of a fake buffer of support foot to get an appropriate convex hull
	std::deque<support_state_t> SupportStatesDeq = Solution_.SupportStates_deq ;
	support_state_t LastSupport = SupportStatesDeq.back() ;
	support_state_t FirstSupport = SupportStatesDeq.front() ;
	nbSteps = LastSupport.StepNumber ;
	if(FirstSupport.Foot == LEFT )
		Sign = 1.0 ;
	else
		Sign = -1.0 ;
	if(LastSupport.Phase == SS && FirstSupport.Phase == DS )
		Sign = -Sign ;

	if ( Sign == 1 )
		LastSupport.Foot = RIGHT ;
	else
		LastSupport.Foot = LEFT ;
	LastSupport.StateChanged = true ;
	SupportStatesDeq.pop_front();
	for (unsigned i = 0 ; i < SupportStatesDeq.size() ; ++i)
	{
		if ( SupportStatesDeq[i].StateChanged )
			SupportStatesDeq[i].StateChanged = false ;
		SupportStatesDeq[i].StepNumber = 1 ;
	}
	SupportStatesDeq.push_back(LastSupport);

	// collecting the linear inequalities
	VRQPGenerator_->build_inequalities_feet(IneqFeet,SupportStatesDeq);

	// transalating the linear inequalities at the appropriate position
	dx = Sign * sin(LastSupport.Yaw) * FeetDistance_ * 0.5 + solution_.Solution_vec[2*QP_N_ + nbSteps -1] ;
	dy = Sign * cos(LastSupport.Yaw) * FeetDistance_ * 0.5 + solution_.Solution_vec[2*QP_N_ + 2*nbSteps -1] ;

	// Translation
	for (unsigned i = 0 ; i < IneqFeet.Dc_vec.size() ; ++ i)
	{
		IneqFeet.Dc_vec(i) = IneqFeet.Dc_vec(i) + IneqFeet.D.X_mat(i,0) * dx + IneqFeet.D.Y_mat(i,0) * dy ;
	}

	cout << " dx dy : " ;
	cout << dx << " " << dy << endl ;

	// Computing the active set
	DX = (IneqFeet.D.X_mat * X + IneqFeet.D.Y_mat * Y) ;
	for (unsigned i = 0 ; i < DX.size1() ; ++ i)
	{
		if( (DX(i,0) - IneqFeet.Dc_vec(i)) < 0 )
		{
			active_set.push_back(i);
		}
	}
	cout << "active_set.size() = " << active_set.size() << endl ;

	// Projection on the active set if it is not empty
	if ( active_set.size() == 1 )
	{
		double a = IneqFeet.D.X_mat(active_set[0],0);
		double b = IneqFeet.D.Y_mat(active_set[0],0);
		double c = IneqFeet.Dc_vec(active_set[0]);
		double Bx(1),By(1) ;
		double BH(0),normV(0);
		if ( a == 0 && b != 0 )
		{
			By = c/b ;
		}else if ( a != 0 && b == 0 )
		{
			Bx = c/a ;
		}else
		{
			By = (c-a*Bx) / b ;
		}
		normV = sqrt( a*a + b*b );
		BH = ( (X - Bx)*(-b) + (Y - By)*a ) / normV ;

		X = Bx + BH / normV * (-b) ;
		Y = By + BH / normV * a ;
	}else if ( active_set.size() == 2 )
	{
		double a1 = IneqFeet.D.X_mat(active_set[0],0);
		double b1 = IneqFeet.D.Y_mat(active_set[0],0);
		double c1 = IneqFeet.Dc_vec(active_set[0]);
		double a2 = IneqFeet.D.X_mat(active_set[1],0);
		double b2 = IneqFeet.D.Y_mat(active_set[1],0);
		double c2 = IneqFeet.Dc_vec(active_set[1]);

		if (a1!=0 && b1!=0 && a2==0 && b2!=0)
		{
		  Y = c2/b2 ;
		  X = (c1-b1*Y)/a1 ;
		}else if (a1==0 && b1!=0 && a2!=0 && b2!=0)
		{
		  Y = c1/b1 ;
			X = (c2-b2*Y)/a2 ;
		}else if (a1!=0 && b1==0 && a2!=0 && b2!=0)
		{
		  X = c1/a1 ;
		  Y = (c2-a2*X)/b2 ;
		}else if (a1!=0 && b1!=0 && a2!=0 && b2==0)
		{
		  X = c2/a2 ;
		  Y = (c1-a1*X)/b1 ;
		}else if (a1!=0 && b1==0 && a2==0 && b2!=0)
		{
		  X = c1/a1 ;
		  Y = c2/b2 ;
		}else if (a1==0 && b1!=0 && a2!=0 && b2==0)
		{
		  X = c2/a2 ;
		  Y = c1/b1 ;
		}else if (a1!=0 && b1!=0 && a2!=0 && b2!=0)
		{
		  Y = c2/(b1-a1/a2*b2) ;
		  X = (c2-b2*Y)/a2 ;
		}
	}
	return ;
}

// Modify a copy of the solution to allow "OFTG_DF_->interpolate_feet_positions()"
		// to use the correct foot step previewed
void ZMPVelocityReferencedQP::PrepareSolution(int iteration, const vector<double> &solFoot)
{
	static int nbStepChanged = 0 ;
	int nbSteps (0);
	nbSteps = solution_.SupportStates_deq.back().StepNumber ;

	if (iteration > 0)
	{
		nbStepChanged = nbStepChanged + (int)(solution_.SupportStates_deq.front().StateChanged) ;
	}
	if (iteration == 0)
	{
		nbStepChanged = 0 ;
	}

//	if ( solution_.SupportStates_deq.front().Phase == DS )
//	{/*do nothing*/}
/*	else*/ if ( nbSteps == 1 && nbStepChanged == 2 )
	{
		solution_.Solution_vec[2*QP_N_] = solFoot[nbSteps] ;
		solution_.Solution_vec[2*QP_N_+nbSteps] = solFoot.back() ;
	}
	else if ( nbSteps == 2 && nbStepChanged == 2 )
	{
		solution_.Solution_vec[2*QP_N_] = solFoot[nbSteps] ;
		solution_.Solution_vec[2*QP_N_+nbSteps] = solFoot.back() ;
	}else if ( nbSteps == 2 && nbStepChanged == 1 ){
		solution_.Solution_vec[2*QP_N_] = solFoot[nbSteps-1] ;
		solution_.Solution_vec[2*QP_N_+nbSteps] = solFoot[2*nbSteps] ;
	}else{/*do nothing*/}

	if (solution_.SupportStates_deq.front().Foot == LEFT)
			cout << "footsupport = LEFT" ;
		else
			cout << "footsupport = RIGHT" ;
	if(solution_.Solution_vec.size()!=0 && solution_.SupportStates_deq.front().Phase != DS && nbSteps > 0 ){
		 cout << " ; SolX = " << solution_.Solution_vec[2*QP_N_]
		 << " ; SolY = " << solution_.Solution_vec[2*QP_N_+nbSteps]
		 << " ; nbSteps = " << solution_.SupportStates_deq.front().StepNumber
		 <<	" ; nbStepChanged = " << nbStepChanged;

	}else{}
	cout << endl ;
	return ;
}

void ZMPVelocityReferencedQP::DynamicFilter(double time, std::deque<COMState> & FinalCOMTraj_deq)
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
  ofstream aof,aof2;
  string aFileName, aFileName2;
  ostringstream oss(std::ostringstream::ate);
  ostringstream oss2(std::ostringstream::ate);
  static int iteration = 0;
  int iteration100 = (int)iteration/100;
  int iteration10 = (int)(iteration - iteration100*100)/10;
  int iteration1 = (int)(iteration - iteration100*100 - iteration10*10 );

  /// \brief Debug Purpose
  /// --------------------
  oss.str("TestHerdt2010DynamicZMPMB.dat");
  aFileName = oss.str();
  if (iteration == 0 )
  {
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
  }
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

		///----
		oss.str("TestHerdt2010DynamicZMPMB.dat");
		aFileName = oss.str();
		aof.open(aFileName.c_str(),ofstream::app);
		aof.precision(8);
		aof.setf(ios::scientific, ios::floatfield);
    if (i<NbSampleInterpolation_){
      aof << filterprecision( ( - m_force.n()[1] / m_force.f()[2] ) + DInitX_ ) << " "   // 1
      << filterprecision( (   m_force.n()[0] / m_force.f()[2] ) + DInitY_ ) << " "   // 2
      << filterprecision( ZMPTraj_deq_[i].px ) << " "   // 3
      << filterprecision( ZMPTraj_deq_[i].py ) << " "   // 4
      << endl ;
    }
		aof.close();

		oss2.str("TestHerdt2010DynamicDZMP");
		oss2 << "_" << iteration100 << iteration10 << iteration1 << ".dat";
		aFileName2 = oss2.str();
		if ( i == 0 ){
			aof2.open(aFileName2.c_str(),ofstream::out);
			aof2.close();
		}
		///----
		aof2.open(aFileName2.c_str(),ofstream::app);
		aof2.precision(8);
		aof2.setf(ios::scientific, ios::floatfield);
		aof2 << filterprecision( ( - m_force.n()[1] / m_force.f()[2] ) + DInitX_ ) << " "   // 1
      << filterprecision( (   m_force.n()[0] / m_force.f()[2] ) + DInitY_ ) << " "   // 2
      << filterprecision( ZMPTraj_deq_[CurrentIndex_+i].px ) << " "   // 3
      << filterprecision( ZMPTraj_deq_[CurrentIndex_+i].py ) << " "   // 4
      << endl ;
		aof2.close();

  }


  static double ecartmaxX = 0 ;
  static double ecartmaxY = 0 ;

	for (unsigned int i = 0 ; i < N ; i++ )
  {
		if ( abs(DeltaZMPMBPositions_[i].px) > ecartmaxX )
			ecartmaxX = abs(DeltaZMPMBPositions_[i].px) ;
		if ( abs(DeltaZMPMBPositions_[i].py) > ecartmaxY )
			ecartmaxY = abs(DeltaZMPMBPositions_[i].py) ;
	}
  //cout << "ecartmaxX :" << ecartmaxX << endl ;
  //cout << "ecartmaxY :" << ecartmaxY << endl ;

  /// Preview control on the ZMPMBs computed
  /// --------------------------------------
  //init of the Kajita preview control
  PC_->SetPreviewControlTime (QP_T_*QP_N_ - NbSampleControl_*InterpolationPeriod_);
  PC_->SetSamplingPeriod (InterpolationPeriod_);
  PC_->SetHeightOfCoM(CoMHeight_);
  PC_->ComputeOptimalWeights(OptimalControllerSolver::MODE_WITH_INITIALPOS);

  double aSxzmp (0) , aSyzmp(0);
  double deltaZMPx (0) , deltaZMPy (0) ;

  // calcul of the preview control along the "ZMPTraj_deq_"
  for (unsigned i = 0 ; i < NbSampleControl_ ; i++ )
  {
    for(int j=0;j<3;j++)
    {
      m_deltax(j,0) = 0 ;
      m_deltay(j,0) = 0 ;
    }
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
			{}else{cout << "kajita2003 preview control diverged\n" ;}
    }

    for(int j=0;j<3;j++)
		{
			FinalCOMTraj_deq[CurrentIndex_+i].x[j] += ComStateBuffer_[i].x[j] ;
      FinalCOMTraj_deq[CurrentIndex_+i].y[j] += ComStateBuffer_[i].y[j] ;
		}
  }

  /// \brief Debug Purpose
  /// --------------------
  oss.str("TestHerdt2010DynamicART");
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
        << filterprecision( DeltaZMPMBPositions_[i].py ) << " ";   // 2
    for (unsigned int j = 0 ; j < VelocityTraj_[i].size() ; ++j)
    {
      aof << filterprecision(VelocityTraj_[i](j)) << " " ;
    }
    for (unsigned int j = 0 ; j < AccelerationTraj_[i].size() ; ++j)
    {
      aof << filterprecision(AccelerationTraj_[i](j)) << " " ;
    }
    aof << endl ;
  }
	cout << "iteration = " << iteration << endl;
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

  return ;
}


void ZMPVelocityReferencedQP::CallToComAndFootRealization(
    const COMState & acomp,
		const FootAbsolutePosition & aLeftFAP,
    const FootAbsolutePosition & aRightFAP,
    MAL_VECTOR_TYPE(double) & CurrentConfiguration,
    MAL_VECTOR_TYPE(double) & CurrentVelocity,
    MAL_VECTOR_TYPE(double) & CurrentAcceleration,
    const unsigned & IterationNumber)
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
  aCOMSpeed(4) = acomp.pitch[1];
  aCOMSpeed(5) = acomp.yaw[1];

  aCOMAcc(0) = acomp.x[2];
  aCOMAcc(1) = acomp.y[2];
  aCOMAcc(2) = acomp.z[2];
  aCOMAcc(3) = acomp.roll[2];
  aCOMAcc(4) = acomp.pitch[2];
  aCOMAcc(5) = acomp.yaw[2];

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

  if (IterationNumber == 0)
  {
    CurrentConfiguration = HDR_->currentConfiguration();
    CurrentVelocity = HDR_->currentConfiguration();
    CurrentAcceleration = HDR_->currentConfiguration();
  }else
  {
    CurrentConfiguration = PreviousConfiguration_ ;
    CurrentVelocity = PreviousVelocity_ ;
    CurrentAcceleration = PreviousAcceleration_ ;
  }
  ComAndFootRealization_->setSamplingPeriod(InterpolationPeriod_);
  ComAndFootRealization_->ComputePostureForGivenCoMAndFeetPosture(aCOMState, aCOMSpeed, aCOMAcc,
								    aLeftFootPosition,
								    aRightFootPosition,
								    CurrentConfiguration,
								    CurrentVelocity,
								    CurrentAcceleration,
								    IterationNumber,
								    0);

  if(IterationNumber == NbSampleInterpolation_-1 )
  {
    HDR_->currentConfiguration(CurrentConfiguration);
    HDR_->currentConfiguration(CurrentVelocity);
    HDR_->currentConfiguration(CurrentAcceleration);
  }else
  {
    PreviousConfiguration_ = CurrentConfiguration ;
    PreviousVelocity_ = CurrentVelocity ;
    PreviousAcceleration_ = CurrentAcceleration ;
  }

  return ;
}

void ZMPVelocityReferencedQP::CoMZMPInterpolation(
					std::deque<ZMPPosition> & ZMPPositions,                     // OUTPUT
		      std::deque<COMState> & COMTraj_deq ,                        // OUTPUT
		      const std::deque<FootAbsolutePosition> & LeftFootTraj_deq, // INPUT
		      const std::deque<FootAbsolutePosition> & RightFootTraj_deq,// INPUT
		      const solution_t * Solution,                               // INPUT
          LinearizedInvertedPendulum2D * LIPM,                        // INPUT/OUTPUT
		      const unsigned numberOfSample,                             // INPUT
		      const int IterationNumber)                                 // INPUT
{
  if(Solution->SupportStates_deq.size() && Solution->SupportStates_deq[IterationNumber].NbStepsLeft == 0)
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


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
  NbSampleControl_ = (unsigned)(QP_T_/m_SamplingPeriod) ;
  NbSampleInterpolation_ = (unsigned)(QP_T_/InterpolationPeriod_) ;
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
  OrientPrw_ = new OrientationsPreview( aHS );
  OrientPrw_->SamplingPeriod( QP_T_ );
  OrientPrw_->NbSamplingsPreviewed( QP_N_ );
  OrientPrw_->SSLength( SupportFSM_->StepPeriod() );
  COMState CurrentTrunkState;
  OrientPrw_->CurrentTrunkState( CurrentTrunkState );

  // Create and initialize preview of orientations
  OrientPrw_DF_ = new OrientationsPreview( aHS );
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

  // Initialize the 2D LIPM
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

  // Create and initialize the class that compute the simplified inverse kinematics :
  // ------------------------------------------------------------------------------
  ComAndFootRealizationByGeometry_ = new ComAndFootRealizationByGeometry( (PatternGeneratorInterfacePrivate*) SPM );
  ComAndFootRealizationByGeometry_->setHumanoidDynamicRobot(aHS);
  ComAndFootRealizationByGeometry_->SetHeightOfTheCoM(CoMHeight_);
  ComAndFootRealizationByGeometry_->setSamplingPeriod(InterpolationPeriod_);
  ComAndFootRealizationByGeometry_->Initialization();

  // Initialization of the configuration vectors
  PreviousConfiguration_ = aHS->currentConfiguration() ;
  PreviousVelocity_ = aHS->currentVelocity();
  PreviousAcceleration_ = aHS->currentAcceleration();

  ComAndFootRealizationByGeometry_->SetPreviousConfigurationStage0(PreviousConfiguration_);
  ComAndFootRealizationByGeometry_->SetPreviousVelocityStage0(PreviousVelocity_);
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

  // size = numberOfIterationOfThePreviewControl * NumberOfSample + Margin
  ZMPTraj_deq_.resize( QP_N_ * NbSampleInterpolation_+20);
  COMTraj_deq_.resize( QP_N_ * NbSampleInterpolation_+20);
  tmpCoM_.resize(QP_N_ * NbSampleControl_ + 20);
  tmpZMP_.resize(QP_N_ * NbSampleControl_ + 20);

  ConfigurationTraj_.resize( QP_N_ * NbSampleInterpolation_, aHS->currentConfiguration() );
  VelocityTraj_.resize( QP_N_ * NbSampleInterpolation_, aHS->currentVelocity() );
  AccelerationTraj_.resize( QP_N_ * NbSampleInterpolation_, aHS->currentAcceleration() );

  tmpConfigurationTraj_.resize(QP_N_ * NbSampleInterpolation_) ;
  tmpVelocityTraj_.resize(QP_N_ * NbSampleInterpolation_) ;
  tmpAccelerationTraj_.resize(QP_N_ * NbSampleInterpolation_) ;

  DeltaZMPMBPositions_.resize ( QP_N_ * NbSampleInterpolation_ );
  ZMPMBTraj_deq_.resize( QP_N_ * NbSampleInterpolation_, vector<double>(2) );

  ComStateBuffer_.resize(NbSampleControl_);


  m_Jacobian_LF = Jacobian_LF::Jacobian::Zero();
  m_Jacobian_RF = Jacobian_RF::Jacobian::Zero();
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

  if (ComAndFootRealizationByGeometry_!=0){
    delete ComAndFootRealizationByGeometry_;
    ComAndFootRealizationByGeometry_ = 0 ;
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
  CoM_.SetComHeight(lStartingCOMState.z[0]);
  CoM_.InitializeSystem();
  CoM_(CoM);

  IntermedData_->CoM(CoM);

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
    InitStateOrientPrw_ = OrientPrw_->CurrentTrunkState() ;
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

    // INTERPRET THE SOLUTION VECTOR :
    // -------------------------------
    InterpretSolutionVector();

    // INTERPOLATION
    ControlInterpolation( FinalCOMTraj_deq, FinalZMPTraj_deq, FinalLeftFootTraj_deq,
                          FinalRightFootTraj_deq, time) ;

    //DynamicFilterInterpolation(time) ;
    //CallToComAndFootRealization(time);


    //deque<COMState> tmp = FinalCOMTraj_deq ;
    // DYNAMIC FILTER
    // --------------
    //DynamicFilter( time, tmp );












    /// \brief Debug Purpose
    /// --------------------
    ofstream aof;
    string aFileName;
    ostringstream oss(std::ostringstream::ate);
    static int iteration = 0 ;
    int iteration100 = (int)iteration/100;
    int iteration10 = (int)(iteration - iteration100*100)/10;
    int iteration1 = (int)(iteration - iteration100*100 - iteration10*10 );
//
//    /// \brief Debug Purpose
//    /// --------------------
//    oss.str("TestHerdt2010footcom");
//    oss << "_" << iteration100 << iteration10 << iteration1 << ".dat";
//    aFileName = oss.str();
//    aof.open(aFileName.c_str(),ofstream::out);
//    aof.close();
//    ///----
//    aof.open(aFileName.c_str(),ofstream::app);
//    aof.precision(8);
//    aof.setf(ios::scientific, ios::floatfield);
//    for (unsigned int i = 0 ; i < NbSampleInterpolation_ * QP_N_ ; ++i )
//    {
//      aof << filterprecision( i * InterpolationPeriod_ ) << " "       // 0
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].x[0] ) << " "         // 1
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].y[0] ) << " "         // 2
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].z[0] ) << " "         // 3
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].x[1] ) << " "         // 4
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].y[1] ) << " "         // 5
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].z[1] ) << " "         // 6
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].x[2] ) << " "         // 7
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].y[2] ) << " "         // 8
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].z[2] ) << " "         // 9
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].yaw[0] ) << " "       // 10
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].yaw[1] ) << " "       // 11
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].yaw[2] ) << " "       // 12
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].x ) << " "       // 13
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].y ) << " "       // 14
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].z ) << " "       // 15
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].theta * M_PI / 180 ) << " "   // 16
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].omega * M_PI / 180 ) << " "   // 17
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].x ) << " "      //18
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].y ) << " "      //19
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].z ) << " "  //20
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].theta * M_PI / 180 ) << " "  //21
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].omega * M_PI / 180 ) << " "  //22
//          << filterprecision( solution_.Solution_vec[i/NbSampleInterpolation_] ) << " "  //23
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].roll[0] ) << " "       // 24
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].roll[1] ) << " "       // 25
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].roll[2] ) << " "       // 26
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].pitch[0] ) << " "       // 27
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].pitch[1] ) << " "       // 28
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].pitch[2] ) << " "       // 29
//          << endl ;
//    }
//    aof.close() ;

//    /// \brief Debug Purpose
//    /// --------------------
//    oss.str("TestHerdt2010DynamicCoMComparison1.dat");
//    aFileName = oss.str();
//    if(iteration == 0)
//    {
//      aof.open(aFileName.c_str(),ofstream::out);
//      aof.close();
//    }
//    ///----
//    aof.open(aFileName.c_str(),ofstream::app);
//    aof.precision(8);
//    aof.setf(ios::scientific, ios::floatfield);
//    for (unsigned int i = 0 ; i < NbSampleInterpolation_ ; ++i )
//    {
//      aof << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].x ) << " "         // 1
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].y ) << " "         // 2
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].z ) << " "         // 3
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].theta ) << " "     // 4
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].omega ) << " "     // 5
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].dx ) << " "        // 6
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].dy ) << " "        // 7
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].dz ) << " "        // 8
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].dtheta ) << " "    // 9
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].domega ) << " "    // 10
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].ddx ) << " "       // 11
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].ddy ) << " "       // 12
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].ddz ) << " "       // 13
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].ddtheta ) << " "   // 14
//          << filterprecision( LeftFootTraj_deq_[CurrentIndex_+i].ddomega ) << " "   // 15
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].x ) << " "        // 16
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].y ) << " "        // 17
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].z ) << " "        // 18
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].theta ) << " "    // 19
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].omega ) << " "    // 20
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].dx ) << " "       // 21
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].dy ) << " "       // 22
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].dz ) << " "       // 23
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].dtheta ) << " "   // 24
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].domega ) << " "   // 25
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].ddx ) << " "      // 26
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].ddy ) << " "      // 27
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].ddz ) << " "      // 28
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].ddtheta ) << " "  // 29
//          << filterprecision( RightFootTraj_deq_[CurrentIndex_+i].ddomega ) << " "  // 30
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].x[0] ) << " "           // 31
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].y[0] ) << " "           // 32
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].z[0] ) << " "           // 33
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].x[1] ) << " "           // 34
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].y[1] ) << " "           // 35
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].z[1] ) << " "           // 36
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].x[2] ) << " "           // 37
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].y[2] ) << " "           // 38
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].z[2] ) << " "           // 39
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].roll[0] ) << " "        // 40
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].pitch[0] ) << " "       // 41
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].yaw[0] ) << " "         // 42
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].roll[1] ) << " "        // 43
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].pitch[1] ) << " "       // 44
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].yaw[1] ) << " "         // 45
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].roll[2] ) << " "        // 46
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].pitch[2] ) << " "       // 47
//          << filterprecision( COMTraj_deq_[CurrentIndex_+i].yaw[2] ) << " "         // 48
//          << endl ;
//    }
//    aof.close();

    /// \brief Debug Purpose
    /// --------------------
    oss.str("TestHerdt2010DynamicCoMComparison2.dat");
    aFileName = oss.str();
    if(iteration == 0)
    {
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
    ///----
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    for (unsigned int i = CurrentIndex_ ; i < FinalCOMTraj_deq.size() ; ++i )
    {
      aof << filterprecision( FinalLeftFootTraj_deq[i].x ) << " "       // 1
          << filterprecision( FinalLeftFootTraj_deq[i].y ) << " "       // 2
          << filterprecision( FinalLeftFootTraj_deq[i].z ) << " "       // 3
          << filterprecision( FinalLeftFootTraj_deq[i].theta ) << " "   // 4
          << filterprecision( FinalLeftFootTraj_deq[i].omega ) << " "   // 5
          << filterprecision( FinalLeftFootTraj_deq[i].dx ) << " "      // 6
          << filterprecision( FinalLeftFootTraj_deq[i].dy ) << " "      // 7
          << filterprecision( FinalLeftFootTraj_deq[i].dz ) << " "      // 8
          << filterprecision( FinalLeftFootTraj_deq[i].dtheta ) << " "  // 9
          << filterprecision( FinalLeftFootTraj_deq[i].domega ) << " "  // 10
          << filterprecision( FinalLeftFootTraj_deq[i].ddx ) << " "     // 11
          << filterprecision( FinalLeftFootTraj_deq[i].ddy ) << " "     // 12
          << filterprecision( FinalLeftFootTraj_deq[i].ddz ) << " "     // 13
          << filterprecision( FinalLeftFootTraj_deq[i].ddtheta ) << " " // 14
          << filterprecision( FinalLeftFootTraj_deq[i].ddomega ) << " " // 15
          << filterprecision( FinalRightFootTraj_deq[i].x ) << " "      // 16
          << filterprecision( FinalRightFootTraj_deq[i].y ) << " "      // 17
          << filterprecision( FinalRightFootTraj_deq[i].z ) << " "  	// 18
          << filterprecision( FinalRightFootTraj_deq[i].theta ) << " "  // 19
          << filterprecision( FinalRightFootTraj_deq[i].omega ) << " "  // 20
          << filterprecision( FinalRightFootTraj_deq[i].dx ) << " "     // 21
          << filterprecision( FinalRightFootTraj_deq[i].dy ) << " "     // 22
          << filterprecision( FinalRightFootTraj_deq[i].dz ) << " "  	// 23
          << filterprecision( FinalRightFootTraj_deq[i].dtheta ) << " " // 24
          << filterprecision( FinalRightFootTraj_deq[i].domega ) << " " // 25
          << filterprecision( FinalRightFootTraj_deq[i].ddx ) << " "    // 26
          << filterprecision( FinalRightFootTraj_deq[i].ddy ) << " "    // 27
          << filterprecision( FinalRightFootTraj_deq[i].ddz ) << " "  	// 28
          << filterprecision( FinalRightFootTraj_deq[i].ddtheta ) << " "// 29
          << filterprecision( FinalRightFootTraj_deq[i].ddomega ) << " "// 30
          << filterprecision( FinalCOMTraj_deq[i].x[0] ) << " "         // 31
          << filterprecision( FinalCOMTraj_deq[i].y[0] ) << " "         // 32
          << filterprecision( FinalCOMTraj_deq[i].z[0] ) << " "         // 33
          << filterprecision( FinalCOMTraj_deq[i].x[1] ) << " "         // 34
          << filterprecision( FinalCOMTraj_deq[i].y[1] ) << " "         // 35
          << filterprecision( FinalCOMTraj_deq[i].z[1] ) << " "         // 36
          << filterprecision( FinalCOMTraj_deq[i].x[2] ) << " "         // 37
          << filterprecision( FinalCOMTraj_deq[i].y[2] ) << " "         // 38
          << filterprecision( FinalCOMTraj_deq[i].z[2] ) << " "         // 39
          << filterprecision( FinalCOMTraj_deq[i].roll[0] ) << " "      // 40
          << filterprecision( FinalCOMTraj_deq[i].pitch[0] ) << " "     // 41
          << filterprecision( FinalCOMTraj_deq[i].yaw[0] ) << " "       // 42
          << filterprecision( FinalCOMTraj_deq[i].roll[1] ) << " "      // 43
          << filterprecision( FinalCOMTraj_deq[i].pitch[1] ) << " "     // 44
          << filterprecision( FinalCOMTraj_deq[i].yaw[1] ) << " "       // 45
          << filterprecision( FinalCOMTraj_deq[i].roll[2] ) << " "      // 46
          << filterprecision( FinalCOMTraj_deq[i].pitch[2] ) << " "     // 47
          << filterprecision( FinalCOMTraj_deq[i].yaw[2] ) << " "       // 48
          << filterprecision( FinalLeftFootTraj_deq[i].dddx ) << " "       // 49
          << filterprecision( FinalRightFootTraj_deq[i].dddx ) << " "       // 50
          << endl ;
    }
    aof.close();





    iteration++;





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

void ZMPVelocityReferencedQP::ControlInterpolation(
    std::deque<COMState> & FinalCOMTraj_deq,                      // OUTPUT
    std::deque<ZMPPosition> & FinalZMPTraj_deq,                   // OUTPUT
    std::deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,     // OUTPUT
    std::deque<FootAbsolutePosition> & FinalRightFootTraj_deq,    // OUTPUT
    double time)                                                  // INPUT
{
  InitStateLIPM_ = LIPM_.GetState() ;

  // INTERPOLATE CoM AND ZMP TRAJECTORIES:
  // -------------------------------------
  CoMZMPInterpolation(FinalZMPTraj_deq, FinalCOMTraj_deq,
                      FinalLeftFootTraj_deq, FinalRightFootTraj_deq,
                      &Solution_, &LIPM_, NbSampleControl_, 0);

  // INTERPOLATE TRUNK ORIENTATION:
  // ------------------------------
  OrientPrw_->interpolate_trunk_orientation( time, CurrentIndex_,
                                             m_SamplingPeriod, Solution_.SupportStates_deq,
                                             FinalCOMTraj_deq );
  FinalCurrentStateOrientPrw_ = OrientPrw_->CurrentTrunkState();
  FinalPreviewStateOrientPrw_ = OrientPrw_->PreviewTrunkState();
  // INTERPOLATE THE COMPUTED FOOT POSITIONS:
  // ----------------------------------------
  OFTG_control_->interpolate_feet_positions( time,
                                             Solution_.SupportStates_deq, Solution_,
                                             Solution_.SupportOrientations_deq,
                                             FinalLeftFootTraj_deq, FinalRightFootTraj_deq);
//  cout << "FinalLeftFootTraj_deq.ddtheta = " << FinalLeftFootTraj_deq[CurrentIndex_-1].ddtheta << " "
//      << FinalLeftFootTraj_deq[CurrentIndex_+NbSampleControl_-1].ddtheta << endl ;
//  cout << "FinalRightFootTraj_deq.ddtheta = " << FinalRightFootTraj_deq[CurrentIndex_-1].ddtheta << " "
//      << FinalRightFootTraj_deq[CurrentIndex_+NbSampleControl_-1].ddtheta << endl ;
  return ;
}

void ZMPVelocityReferencedQP::DynamicFilterInterpolation(double time)
{
  // interpolation the position of the com along the whole preview
  LIPM_subsampled_.setState(InitStateLIPM_) ;
  OrientPrw_DF_->CurrentTrunkState(InitStateOrientPrw_) ;
  for ( int i = 0 ; i < QP_N_ ; i++ )
  {
    CoMZMPInterpolation(ZMPTraj_deq_, COMTraj_deq_,
                        LeftFootTraj_deq_, RightFootTraj_deq_,
                        &Solution_, &LIPM_subsampled_,
                        NbSampleInterpolation_, i);
  }

  InterpretSolutionVector();

  cout << "support foot\n"
      << "Phase Foot NbStepsLeft StepNumber NbInstants TimeLimit StartTime X Y Yaw StateChanged\n";
  for (unsigned int i = 0 ; i < Solution_.SupportStates_deq.size() ; ++i)
  {
    cout<< Solution_.SupportStates_deq[i].Phase << " "
        << Solution_.SupportStates_deq[i].Foot << " "
        << Solution_.SupportStates_deq[i].NbStepsLeft << " "
        << Solution_.SupportStates_deq[i].StepNumber << " "
        << Solution_.SupportStates_deq[i].NbInstants << " "
        << Solution_.SupportStates_deq[i].TimeLimit << " "
        << Solution_.SupportStates_deq[i].StartTime << " "
        << Solution_.SupportStates_deq[i].X << " "
        << Solution_.SupportStates_deq[i].Y << " "
        << Solution_.SupportStates_deq[i].Yaw << " "
        << Solution_.SupportStates_deq[i].StateChanged << " " << endl ;
  }
  cout << "X solution = " ;
  for (unsigned int i = 0 ; i < FootPrw_vec.size() ; ++i )
    cout << FootPrw_vec[i][0] << " " ;
  cout << " Y solution = " ;
  for (unsigned int i = 0 ; i < FootPrw_vec.size() ; ++i )
    cout << FootPrw_vec[i][1] << " " ;
  cout << endl ;

  // Copy the solution for the orientation interpolation function
  OFTG_DF_->SetSamplingPeriod( InterpolationPeriod_ );
  solution_t aSolution  = Solution_ ;
  //solution_.SupportStates_deq.pop_front();

  for ( int i = 0 ; i < QP_N_ ; i++ )
  {
    aSolution.SupportOrientations_deq.clear();
    OrientPrw_DF_->preview_orientations( time + i * QP_T_, VelRef_,
                                         SupportFSM_->StepPeriod(),
                                         LeftFootTraj_deq_, RightFootTraj_deq_,
                                         aSolution );

    OrientPrw_DF_->interpolate_trunk_orientation( time + i * QP_T_,
                                                  CurrentIndex_ + i * NbSampleInterpolation_, InterpolationPeriod_,
                                                  solution_.SupportStates_deq, COMTraj_deq_ );

    // Modify a copy of the solution to allow "OFTG_DF_->interpolate_feet_positions"
    // to use the correcte feet step previewed
    PrepareSolution();

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

void ZMPVelocityReferencedQP::DynamicFilter(double time, std::deque<COMState> & FinalCOMTraj_deq)
{
  const unsigned int N = NbSampleInterpolation_ * QP_N_ ;
  // \brief calculate, from the CoM computed by the preview control,
  //    the corresponding articular position, velocity and acceleration
  // ------------------------------------------------------------------
  //CallToComAndFootRealization(time);

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

    ZMPMBTraj_deq_[i][0] = - m_force.n()[1] / m_force.f()[2] + DInitX_ ;
    ZMPMBTraj_deq_[i][1] =   m_force.n()[0] / m_force.f()[2]  + DInitY_ ;

    DeltaZMPMBPositions_[i].px = ZMPTraj_deq_[CurrentIndex_+i].px - ZMPMBTraj_deq_[i][0] ;
    DeltaZMPMBPositions_[i].py = ZMPTraj_deq_[CurrentIndex_+i].py - ZMPMBTraj_deq_[i][1] ;
    DeltaZMPMBPositions_[i].pz = 0.0 ;
    DeltaZMPMBPositions_[i].theta = 0.0 ;
    DeltaZMPMBPositions_[i].time = time + i * InterpolationPeriod_ ;
    DeltaZMPMBPositions_[i].stepType = ZMPTraj_deq_[CurrentIndex_+i].stepType ;
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

  // calcul of the preview control along the "DeltaZMPMBPositions_"
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
  ofstream aof;
  string aFileName;
  ostringstream oss(std::ostringstream::ate);
  static int iteration = 0;
  int iteration100 = (int)iteration/100;
  int iteration10 = (int)(iteration - iteration100*100)/10;
  int iteration1 = (int)(iteration - iteration100*100 - iteration10*10 );

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
  for (unsigned int i = 0 ; i < ZMPMBTraj_deq_.size() ; ++i )
  {
    aof << filterprecision( ZMPMBTraj_deq_[i][0] ) << " "   // 1
        << filterprecision( ZMPMBTraj_deq_[i][1] ) << " "   // 2
        << filterprecision( ZMPTraj_deq_[CurrentIndex_+i].px ) << " "   // 3
        << filterprecision( ZMPTraj_deq_[CurrentIndex_+i].py ) << " "   // 4
        << endl ;
  }
  aof.close();

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
    aof << filterprecision( i ) << " "   // 1
        << filterprecision( ComStateBuffer_[i].x[0] ) << " "   // 2
        << filterprecision( ComStateBuffer_[i].y[0] ) << " "   // 3
        << filterprecision( ComStateBuffer_[i].x[1] ) << " "   // 4
        << filterprecision( ComStateBuffer_[i].y[1] ) << " "   // 5
        << filterprecision( ComStateBuffer_[i].x[2] ) << " "   // 6
        << filterprecision( ComStateBuffer_[i].y[2] ) << " "   // 7
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].x[0]-ComStateBuffer_[i].x[0] ) << " "   // 8
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].y[0]-ComStateBuffer_[i].y[0] ) << " "   // 9
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].x[1]-ComStateBuffer_[i].x[1] ) << " "   // 10
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].y[1]-ComStateBuffer_[i].y[1] ) << " "   // 11
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].x[2]-ComStateBuffer_[i].x[2] ) << " "   // 12
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].y[2]-ComStateBuffer_[i].y[2] ) << " "   // 13
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].x[0] ) << " "   // 14
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].y[0] ) << " "   // 15
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].x[1] ) << " "   // 16
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].y[1] ) << " "   // 17
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].x[2] ) << " "   // 18
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].y[2] ) << " "   // 19
        << endl ;
  }
  aof.close();

  /// \brief Debug Purpose
  /// --------------------
  oss.str("TestHerdt2010DynamicCOMXY.dat");
  aFileName = oss.str();
  if(iteration == 0)
  {
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
  }
  ///----
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (unsigned int i = 0 ; i < NbSampleControl_ ; ++i )
  {
    aof << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].x[0] ) << " "   // 1
        << filterprecision( FinalCOMTraj_deq[CurrentIndex_+i].y[0] ) << " "   // 2
        << endl ;
  }
  aof.close();

  /// \brief Debug Purpose
  /// --------------------
  oss.str("TestHerdt2010DynamicZMPMB.dat");
  aFileName = oss.str();
  if(iteration == 0)
  {
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
  }
  ///----
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (unsigned int i = 0 ; i < NbSampleInterpolation_ ; ++i )
  {
    aof << filterprecision( ZMPMBTraj_deq_[i][0] ) << " "   // 1
        << filterprecision( ZMPMBTraj_deq_[i][1] ) << " " ; // 2
    for(unsigned int j = 0 ; j < ConfigurationTraj_[i].size() ; j++ )
      aof << filterprecision( ConfigurationTraj_[i](j) ) << " " ;
    for(unsigned int j = 0 ; j < VelocityTraj_[i].size() ; j++ )
      aof << filterprecision( VelocityTraj_[i](j) ) << " " ;
    for(unsigned int j = 0 ; j < AccelerationTraj_[i].size() ; j++ )
      aof << filterprecision( AccelerationTraj_[i](j) ) << " " ;
    aof << endl ;
  }
  aof.close();

  ++iteration;
  return ;
}


void ZMPVelocityReferencedQP::CallToComAndFootRealization(double time)
{
  const unsigned int N = NbSampleInterpolation_ * QP_N_ ;
  const int stage0 = 0 ;
  int it = (int)(time / QP_T_) ;

  ComAndFootRealizationByGeometry_->SetPreviousConfigurationStage0(PreviousConfiguration_);
  ComAndFootRealizationByGeometry_->SetPreviousVelocityStage0(PreviousVelocity_);

  for(unsigned int i = 0 ; i <  N ; i++ )
  {
    const COMState & acomp = COMTraj_deq_[CurrentIndex_+i] ;
    const FootAbsolutePosition & aLeftFAP = LeftFootTraj_deq_ [CurrentIndex_+i] ;
    const FootAbsolutePosition & aRightFAP = RightFootTraj_deq_ [CurrentIndex_+i] ;

    MAL_VECTOR_DIM(aCOMState,double,6);	MAL_VECTOR_DIM(aLeftFootPosition,double,5);
    MAL_VECTOR_DIM(aCOMSpeed,double,6);	MAL_VECTOR_DIM(aRightFootPosition,double,5);
    MAL_VECTOR_DIM(aCOMAcc,double,6);

    aCOMState(0) = acomp.x[0];      aCOMSpeed(0) = acomp.x[1];			aCOMAcc(0) = acomp.x[2];
    aCOMState(1) = acomp.y[0];      aCOMSpeed(1) = acomp.y[1];      aCOMAcc(1) = acomp.y[2];
    aCOMState(2) = acomp.z[0];      aCOMSpeed(2) = acomp.z[1];      aCOMAcc(2) = acomp.z[2];
    aCOMState(3) = acomp.roll[0];   aCOMSpeed(3) = acomp.roll[1];   aCOMAcc(3) = acomp.roll[2];
    aCOMState(4) = acomp.pitch[0];  aCOMSpeed(4) = acomp.pitch[1];  aCOMAcc(4) = acomp.pitch[2];
    aCOMState(5) = acomp.yaw[0];		aCOMSpeed(5) = acomp.yaw[1];		aCOMAcc(5) = acomp.yaw[2];

    aLeftFootPosition(0) = aLeftFAP.x;			aRightFootPosition(0) = aRightFAP.x;
    aLeftFootPosition(1) = aLeftFAP.y;      aRightFootPosition(1) = aRightFAP.y;
    aLeftFootPosition(2) = aLeftFAP.z;      aRightFootPosition(2) = aRightFAP.z;
    aLeftFootPosition(3) = aLeftFAP.theta;  aRightFootPosition(3) = aRightFAP.theta;
    aLeftFootPosition(4) = aLeftFAP.omega;  aRightFootPosition(4) = aRightFAP.omega;

    ComAndFootRealizationByGeometry_->setSamplingPeriod(InterpolationPeriod_);
    ComAndFootRealizationByGeometry_->ComputePostureForGivenCoMAndFeetPosture(aCOMState, aCOMSpeed, aCOMAcc,
                                                                              aLeftFootPosition,
                                                                              aRightFootPosition,
                                                                              ConfigurationTraj_[i],
                                                                              VelocityTraj_[i],
                                                                              AccelerationTraj_[i],
                                                                              it, stage0);
  }

  tmpConfigurationTraj_[0] = ( ConfigurationTraj_[1]+ConfigurationTraj_[0]+PreviousConfiguration_ )/3;
  tmpVelocityTraj_[0]      = ( VelocityTraj_[1]+VelocityTraj_[0]+PreviousVelocity_ )/3;
  tmpAccelerationTraj_[0]  = ( AccelerationTraj_[1]+AccelerationTraj_[0]+PreviousAcceleration_ )/3;

  // saving the precedent state of the next QP_ computation
  PreviousConfiguration_ = ConfigurationTraj_[NbSampleInterpolation_-1] ;
  PreviousVelocity_ = VelocityTraj_[NbSampleInterpolation_-1] ;
  PreviousAcceleration_ = AccelerationTraj_[NbSampleInterpolation_-1] ;

  for (unsigned int i = 1 ; i < N-1 ; ++i )
  {
    tmpConfigurationTraj_[i] = ( ConfigurationTraj_[i+1] + ConfigurationTraj_[i] + ConfigurationTraj_[i-1] )/3;
    tmpVelocityTraj_[i] = ( VelocityTraj_[i+1] + VelocityTraj_[i] + VelocityTraj_[i-1] )/3;
    tmpAccelerationTraj_[i] = ( AccelerationTraj_[i+1] + AccelerationTraj_[i] + AccelerationTraj_[i-1] )/3;
  }

  tmpConfigurationTraj_[N-1] = ( ConfigurationTraj_[N-1]+ConfigurationTraj_[N-2] )*0.5;
  tmpVelocityTraj_[N-1]      = ( VelocityTraj_[N-1]+VelocityTraj_[N-2] )*0.5;
  tmpAccelerationTraj_[N-1]  = ( AccelerationTraj_[N-1]+AccelerationTraj_[N-2] )*0.5;


  ConfigurationTraj_ = tmpConfigurationTraj_ ;
  VelocityTraj_ = tmpVelocityTraj_ ;
  AccelerationTraj_ = tmpAccelerationTraj_ ;

  /// \brief Debug Purpose
  /// --------------------
  ofstream aof;
  string aFileName;
  ostringstream oss(std::ostringstream::ate);
  static int iteration = 0;
  //int iteration100 = (int)iteration/100;
  //int iteration10 = (int)(iteration - iteration100*100)/10;
  //int iteration1 = (int)(iteration - iteration100*100 - iteration10*10 );

  /// \brief Debug Purpose
  /// --------------------
  oss.str("TestHerdt2010DynamicArtDF.dat");
  aFileName = oss.str();
  if(iteration == 0)
  {
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
  }
  ///----
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (unsigned int i = 0 ; i < NbSampleInterpolation_ ; ++i )
  {
    aof << filterprecision( 0.0 ) << " "   // 1
        << filterprecision( 0.0 ) << " " ; // 2
    for(unsigned int j = 0 ; j < ConfigurationTraj_[i].size() ; j++ )
      aof << filterprecision( ConfigurationTraj_[i](j) ) << " " ;
    for(unsigned int j = 0 ; j < VelocityTraj_[i].size() ; j++ )
      aof << filterprecision( VelocityTraj_[i](j) ) << " " ;
    for(unsigned int j = 0 ; j < AccelerationTraj_[i].size() ; j++ )
      aof << filterprecision( AccelerationTraj_[i](j) ) << " " ;
    aof << endl ;
  }
  aof.close();

  ++iteration;


  return ;
}

void ZMPVelocityReferencedQP::CoMZMPInterpolation(
    std::deque<ZMPPosition> & ZMPPositions,                    // OUTPUT
    std::deque<COMState> & COMTraj_deq ,                       // OUTPUT
    const std::deque<FootAbsolutePosition> & LeftFootTraj_deq, // INPUT
    const std::deque<FootAbsolutePosition> & RightFootTraj_deq,// INPUT
    const solution_t * aSolutionReference,                     // INPUT
    LinearizedInvertedPendulum2D * LIPM,                       // INPUT/OUTPUT
    const unsigned numberOfSample,                             // INPUT
    const int IterationNumber)                                 // INPUT
{
  if(aSolutionReference->SupportStates_deq.size() && aSolutionReference->SupportStates_deq[IterationNumber].NbStepsLeft == 0)
  {
    double jx = (LeftFootTraj_deq[0].x + RightFootTraj_deq[0].x)/2 - COMTraj_deq[0].x[0];
    double jy = (LeftFootTraj_deq[0].y + RightFootTraj_deq[0].y)/2 - COMTraj_deq[0].y[0];
    if(fabs(jx) < 1e-3 && fabs(jy) < 1e-3) { Running_ = false; }
    const double tf = 0.75;
    jx = 6/(tf*tf*tf)*(jx - tf*COMTraj_deq[0].x[1] - (tf*tf/2)*COMTraj_deq[0].x[2]);
    jy = 6/(tf*tf*tf)*(jy - tf*COMTraj_deq[0].y[1] - (tf*tf/2)*COMTraj_deq[0].y[2]);
    LIPM->Interpolation( COMTraj_deq, ZMPPositions, CurrentIndex_ + IterationNumber * numberOfSample,
                         jx, jy);
    LIPM->OneIteration( jx, jy );
  }
  else
  {
    Running_ = true;
    LIPM->Interpolation( COMTraj_deq, ZMPPositions, CurrentIndex_ + IterationNumber * numberOfSample,
                         aSolutionReference->Solution_vec[IterationNumber], aSolutionReference->Solution_vec[IterationNumber+QP_N_] );
    LIPM->OneIteration( aSolutionReference->Solution_vec[IterationNumber],aSolutionReference->Solution_vec[IterationNumber+QP_N_] );
  }
  return ;
}


void ZMPVelocityReferencedQP::InterpretSolutionVector()
{
  double Vx = VelRef_.Local.X ;
  double Vy = VelRef_.Local.Y ;
  std::deque<support_state_t> & SupportStates = solution_.SupportStates_deq ;
  support_state_t & LastSupport = solution_.SupportStates_deq.back() ;
  support_state_t & FirstSupport = solution_.SupportStates_deq[1] ;
  int nbSteps = LastSupport.StepNumber ;
  vector<int> index ;
  FootPrw_vec.resize( nbSteps+2 , vector<double>(2,0.0) );

  // complete the previewed feet position
  FootPrw_vec[0][0] = FirstSupport.X ;
  FootPrw_vec[0][1] = FirstSupport.Y ;
  for (int i = 0 ; i < nbSteps ; ++i )
  {
    FootPrw_vec[i+1][0] = solution_.Solution_vec[2*QP_N_+i] ;
    FootPrw_vec[i+1][1] = solution_.Solution_vec[2*QP_N_+nbSteps+i] ;
  }

  // compute an additional previewed foot position
  {
    int size_vec_sol = FootPrw_vec.size();

    if ( nbSteps > 0 ){
      // center of the feet of the last preview double support phase :
      double middleX = (FootPrw_vec[size_vec_sol-2][0] + FootPrw_vec[size_vec_sol-3][0])*0.5 ;
      double middleY = (FootPrw_vec[size_vec_sol-2][1] + FootPrw_vec[size_vec_sol-3][1])*0.5 ;

      FootPrw_vec[size_vec_sol-1][0] = FootPrw_vec[size_vec_sol-2][0] + 2*( (middleX + Vx*StepPeriod_) - FootPrw_vec[size_vec_sol-2][0] );
      FootPrw_vec[size_vec_sol-1][1] = FootPrw_vec[size_vec_sol-2][1] + 2*( (middleY + Vy*StepPeriod_) - FootPrw_vec[size_vec_sol-2][1] );
    }else
    {
      double Sign;
      if(FirstSupport.Foot == LEFT)
        Sign = 1.0;
      else
        Sign = -1.0;
      FootPrw_vec[size_vec_sol-1][0] = FirstSupport.X + Sign*sin(FirstSupport.Yaw)*FeetDistance_;
      FootPrw_vec[size_vec_sol-1][1] = FirstSupport.Y - Sign*cos(FirstSupport.Yaw)*FeetDistance_;
    }
  }

  for (unsigned int i = 1 ; i < SupportStates.size() ; ++i)
  {
    SupportStates[i].X = FootPrw_vec[SupportStates[i].StepNumber][0] ;
    SupportStates[i].Y = FootPrw_vec[SupportStates[i].StepNumber][1] ;
  }

  return ;
}

// Modify a copy of the solution to allow "OFTG_DF_->interpolate_feet_positions()"
// to use the correct foot step previewed
void ZMPVelocityReferencedQP::PrepareSolution()
{
  int nbSteps = 0 ;
  nbSteps = solution_.SupportStates_deq.back().StepNumber ;
  support_state_t & CurrentSupport = solution_.SupportStates_deq[1] ;

  if(CurrentSupport.Phase!=DS && nbSteps!=0)
  {
    solution_.Solution_vec[2*QP_N_] = FootPrw_vec[CurrentSupport.StepNumber+1][0] ;
    solution_.Solution_vec[2*QP_N_+nbSteps] = FootPrw_vec[CurrentSupport.StepNumber+1][1];
  }
  return ;
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

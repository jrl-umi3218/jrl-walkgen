/*
 * Copyright 2010,
 *
 * Andrei Herdt
 * Francois Keith
 * Olivier Stasse
 * Maximilien Naveau
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
  Olivier Stasse
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
//using namespace metapod;

//#define DEBUG

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
                                                 string , PinocchioRobot *aPR ) :
ZMPRefTrajectoryGeneration(SPM),
Robot_(0),SupportFSM_(0),OrientPrw_(0),OrientPrw_DF_(0),
VRQPGenerator_(0),IntermedData_(0),RFI_(0),Problem_(),
Solution_(),OFTG_DF_(0),OFTG_control_(0),dynamicFilter_(0)
{
  // Save the reference to HDR
  PR_ = aPR ;

  Running_ = false ;
  TimeBuffer_ = 0.04 ;
  QP_T_ = 0.1 ;
  QP_N_ = 16 ;
  m_SamplingPeriod = 0.005 ;
  InterpolationPeriod_ = m_SamplingPeriod*7;
  previewDuration_ = 7*QP_T_ ;
  NbSampleControl_ = (int)round(QP_T_/m_SamplingPeriod) ;
  NbSampleInterpolation_ = (int)round(QP_T_/InterpolationPeriod_) ;
  previewSize_ = QP_N_ ;
  StepPeriod_ = 0.8 ;
  SSPeriod_ = 0.7 ;
  DSPeriod_ = 0.1 ;
  FeetDistance_ = 0.2 ;
  StepHeight_ = 0.05 ;
  CoMHeight_ = 0.814 ;
  PerturbationOccured_ = false ;
  UpperTimeLimitToUpdate_ = 0.0 ;
  RobotMass_ = PR_->mass() ;
  Solution_.useWarmStart=false ;

  CurrentIndex_ = 0 ;

  // Create and initialize online interpolation of feet trajectories
  RFI_ = new RelativeFeetInequalities( SPM,PR_ );

  // Create and initialize the finite state machine for support sequences
  SupportFSM_ = new SupportFSM();
  SupportFSM_->StepPeriod( StepPeriod_ );
  SupportFSM_->DSPeriod( 1e9 ); // period during the robot move at 0.0 com speed
  SupportFSM_->DSSSPeriod( StepPeriod_ );
  SupportFSM_->NbStepsSSDS( 2 ); // number of previw step
  SupportFSM_->SamplingPeriod( QP_T_ );

  // Create and initialize preview of orientations
  OrientPrw_ = new OrientationsPreview( PR_ );
  OrientPrw_->SamplingPeriod( QP_T_ );
  OrientPrw_->NbSamplingsPreviewed( QP_N_ );
  OrientPrw_->SSLength( SupportFSM_->StepPeriod() );
  COMState CurrentTrunkState;
  OrientPrw_->CurrentTrunkState( CurrentTrunkState );

  // Create and initialize preview of orientations
  OrientPrw_DF_ = new OrientationsPreview( PR_ );
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
  LIPM_subsampled_.SetRobotControlPeriod( m_SamplingPeriod );
  LIPM_subsampled_.InitializeSystem();

  // Initialize the 2D LIPM
  CoM_.SetSimulationControlPeriod( QP_T_ );
  CoM_.SetRobotControlPeriod( m_SamplingPeriod );
  CoM_.InitializeSystem();

  // Create and initialize simplified robot model
  Robot_ = new RigidBodySystem( SPM, PR_, SupportFSM_ );
  Robot_->Mass( RobotMass_ );
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
  OFTG_DF_ = new OnLineFootTrajectoryGeneration(SPM,PR_->leftFoot());
  OFTG_DF_->InitializeInternalDataStructures();
  OFTG_DF_->SetSingleSupportTime( SSPeriod_ );
  OFTG_DF_->SetDoubleSupportTime( DSPeriod_ );
  OFTG_DF_->SetSamplingPeriod( m_SamplingPeriod );
  OFTG_DF_->QPSamplingPeriod( QP_T_ );
  OFTG_DF_->NbSamplingsPreviewed( QP_N_ );
  OFTG_DF_->FeetDistance( FeetDistance_ );
  OFTG_DF_->SetStepHeight( StepHeight_ );
  OFTG_DF_->SetStepStairOn(0) ;

  OFTG_control_ = new OnLineFootTrajectoryGeneration(SPM,PR_->leftFoot());
  OFTG_control_->InitializeInternalDataStructures();
  OFTG_control_->SetSingleSupportTime( SSPeriod_ );
  OFTG_control_->SetDoubleSupportTime( DSPeriod_ );
  OFTG_control_->SetSamplingPeriod( m_SamplingPeriod );
  OFTG_control_->QPSamplingPeriod( QP_T_ );
  OFTG_control_->NbSamplingsPreviewed( QP_N_ );
  OFTG_control_->FeetDistance( FeetDistance_ );
  OFTG_control_->SetStepHeight( StepHeight_ );
  OFTG_control_->SetStepStairOn(0) ;

  dynamicFilter_ = new DynamicFilter(SPM,PR_);

  // Register method to handle
  const unsigned int NbMethods = 4;
  const char *lMethodNames[NbMethods] =
  {":previewcontroltime",
   ":numberstepsbeforestop",
   ":stoppg",
   ":setfeetconstraint"};
  RESETDEBUG4("PgDebug2.txt");
  ODEBUG4("Before registering methods for ZMPVelocityReferencedQP",
	  "PgDebug2.txt");
  for(unsigned int i=0;i<NbMethods;i++)
  {
    //#ifdef DEBUG
    //    std::cout << "lMethodNames["<< i << "]="
    // << lMethodNames[i] <<std::endl;
    //#endif
    std::string aMethodName(lMethodNames[i]);
    ODEBUG4("Register method " << aMethodName
	    << "for ZMPVelocityReferencedQP","PgDebug2.txt");
    if (!RegisterMethod(aMethodName))
    {
      std::cerr << "Unable to register " << aMethodName << std::endl;
    }
  }

  // init of the buffer for the kajita's dynamic filter

  // size = numberOfIterationOfThePreviewControl * NumberOfSample + Margin
  ZMPTraj_deq_.resize( QP_N_ * NbSampleInterpolation_+10);
  COMTraj_deq_.resize( QP_N_ * NbSampleInterpolation_+10);
  LeftFootTraj_deq_.resize( QP_N_ * NbSampleInterpolation_+10) ;
  RightFootTraj_deq_.resize( QP_N_ * NbSampleInterpolation_+10) ;

  ZMPTraj_deq_ctrl_.resize( QP_N_ * NbSampleControl_+10) ;
  COMTraj_deq_ctrl_.resize( QP_N_ * NbSampleControl_+10) ;
  deltaCOMTraj_deq_.resize(NbSampleControl_);
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

  if (dynamicFilter_!=0)
  {
    delete dynamicFilter_;
    dynamicFilter_ = 0 ;
  }

}


void ZMPVelocityReferencedQP::setCoMPerturbationForce(istringstream &strm)
{

  PerturbationAcceleration_.resize(6);

  strm >> PerturbationAcceleration_(2);
  strm >> PerturbationAcceleration_(5);
  PerturbationAcceleration_(2) = PerturbationAcceleration_(2)/RobotMass_;
  PerturbationAcceleration_(5) = PerturbationAcceleration_(5)/RobotMass_;
  PerturbationOccured_ = true;
}

void ZMPVelocityReferencedQP::setCoMPerturbationForce(double x, double y)
{

  PerturbationAcceleration_.resize(6);

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
//#ifdef DEBUG
//  std::cout << __PRETTY_FUNCTION__ << " Method:" << Method << std::endl;
//#endif
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
//#ifdef DEBUG
//    std::cout << __PRETTY_FUNCTION__ << " EndingPhase" << std::endl;
//#endif
    EndingPhase_ = true;
  }
  if(Method==":setfeetconstraint")
  {
   RFI_->CallMethod(Method,strm);
  }
  ZMPRefTrajectoryGeneration::CallMethod(Method,strm);
}

std::size_t
ZMPVelocityReferencedQP::
InitOnLine
(deque<ZMPPosition> & FinalZMPTraj_deq,
 deque<COMState> & FinalCoMPositions_deq,
 deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
 deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
 FootAbsolutePosition & InitLeftFootAbsolutePosition,
 FootAbsolutePosition & InitRightFootAbsolutePosition,
 deque<RelativeFootPosition> &, // RelativeFootPositions,
 COMState & lStartingCOMState,
 Eigen::Vector3d & lStartingZMPPosition)
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

  dynamicFilter_->getComAndFootRealization()->ShiftFoot(true);
  dynamicFilter_->init(m_SamplingPeriod,
                       InterpolationPeriod_,
                       QP_T_,
                       previewDuration_ + QP_T_ ,
                       previewDuration_,
                       lStartingCOMState);
  return 0;
}



void ZMPVelocityReferencedQP::OnLine(double time,
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
    Problem_.solve( QLD, Solution_, NONE );
    if(Solution_.Fail>0)
    {
      Problem_.dump( time );
    }
    VRQPGenerator_->LastFootSol(Solution_);
    //OrientPrw_->

    // INITIALIZE INTERPOLATION:
    // ------------------------
    CurrentIndex_ = (unsigned int)FinalCOMTraj_deq.size();
    for (unsigned int i = 0  ; i < CurrentIndex_ ; ++i )
    {
        ZMPTraj_deq_ctrl_[i] = FinalZMPTraj_deq[i] ;
        COMTraj_deq_ctrl_[i] = FinalCOMTraj_deq[i] ;
    }
    LeftFootTraj_deq_ctrl_ = FinalLeftFootTraj_deq ;
    RightFootTraj_deq_ctrl_ = FinalRightFootTraj_deq ;

    solution_ = Solution_ ;
    InterpretSolutionVector();

    // INTERPOLATION
    FinalZMPTraj_deq.resize( NbSampleControl_ + CurrentIndex_ );
    FinalCOMTraj_deq.resize( NbSampleControl_ + CurrentIndex_ );
    ControlInterpolation( FinalCOMTraj_deq, FinalZMPTraj_deq, FinalLeftFootTraj_deq,
                          FinalRightFootTraj_deq, time) ;

    DynamicFilterInterpolation(time);

    unsigned int IndexMax = (int)round((previewDuration_+QP_T_)  / InterpolationPeriod_ );
    ZMPTraj_deq_.resize(IndexMax);
    COMTraj_deq_.resize(IndexMax);
    LeftFootTraj_deq_.resize(IndexMax);
    RightFootTraj_deq_.resize(IndexMax);
    int inc =  (int)round(InterpolationPeriod_ / m_SamplingPeriod) ;
    for (unsigned int i = 0 , j = 0 ; j < IndexMax ; i = i + inc , ++j )
    {
      ZMPTraj_deq_[j] = ZMPTraj_deq_ctrl_[i] ;
      COMTraj_deq_[j] = COMTraj_deq_ctrl_[i] ;
      COMTraj_deq_[j].roll[0]  = 180/M_PI * COMTraj_deq_ctrl_[i].roll[0] ;
      COMTraj_deq_[j].pitch[0] = 180/M_PI * COMTraj_deq_ctrl_[i].pitch[0] ;
      COMTraj_deq_[j].yaw[0]   = 180/M_PI * COMTraj_deq_ctrl_[i].yaw[0] ;
      LeftFootTraj_deq_[j] = LeftFootTraj_deq_ctrl_[i] ;
      RightFootTraj_deq_[j] = RightFootTraj_deq_ctrl_[i] ;
    }


    dynamicFilter_->OnLinefilter(COMTraj_deq_,ZMPTraj_deq_ctrl_,
                                 LeftFootTraj_deq_,
                                 RightFootTraj_deq_,
                                 deltaCOMTraj_deq_);
//#define DEBUG
#ifdef DEBUG
    dynamicFilter_->Debug(COMTraj_deq_ctrl_,
                          LeftFootTraj_deq_ctrl_,
                          RightFootTraj_deq_ctrl_,
                          COMTraj_deq_,ZMPTraj_deq_ctrl_,
                          LeftFootTraj_deq_,
                          RightFootTraj_deq_,
                          deltaCOMTraj_deq_);
#endif

    // Correct the CoM.
    for (unsigned int i = 0 ; i < NbSampleControl_ ; ++i)
    {
      for(int j=0;j<3;j++)
      {
        FinalCOMTraj_deq[i].x[j] += deltaCOMTraj_deq_[i].x[j] ;
        FinalCOMTraj_deq[i].y[j] += deltaCOMTraj_deq_[i].y[j] ;
      }
    }

    // Specify that we are in the ending phase.
    if (time <= m_SamplingPeriod )
      {
        if (EndingPhase_ == false)
        {
          TimeToStopOnLineMode_ = UpperTimeLimitToUpdate_ + QP_T_ * QP_N_ + m_SamplingPeriod;
        }
        UpperTimeLimitToUpdate_ = UpperTimeLimitToUpdate_ + QP_T_ + m_SamplingPeriod ;
      }else{
        if (EndingPhase_ == false)
        {
          TimeToStopOnLineMode_ = UpperTimeLimitToUpdate_ + QP_T_ * QP_N_;
        }
        UpperTimeLimitToUpdate_ = UpperTimeLimitToUpdate_ + QP_T_;
      }


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
                      &Solution_, &LIPM_, NbSampleControl_, 0, CurrentIndex_);

  // INTERPOLATE TRUNK ORIENTATION:
  // ------------------------------
  OrientPrw_->one_iteration(time,Solution_.SupportStates_deq);

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
  return ;
}

void ZMPVelocityReferencedQP::DynamicFilterInterpolation(double time)
{
  // interpolation the position of the com along the whole preview
  LIPM_subsampled_.setState(InitStateLIPM_) ;
  OrientPrw_DF_->CurrentTrunkState(InitStateOrientPrw_) ;
  InterpretSolutionVector();
  OFTG_DF_->SetSamplingPeriod( m_SamplingPeriod );

  for ( int i = 0 ; i < previewSize_ ; i++ )
  {
    CoMZMPInterpolation(ZMPTraj_deq_ctrl_, COMTraj_deq_ctrl_,
                        LeftFootTraj_deq_ctrl_, RightFootTraj_deq_ctrl_,
                        &Solution_, &LIPM_subsampled_,
                        NbSampleControl_, i, CurrentIndex_);

    OrientPrw_->interpolate_trunk_orientation( time + i * QP_T_,
                                               CurrentIndex_ + i * NbSampleControl_, m_SamplingPeriod,
                                               solution_.SupportStates_deq, COMTraj_deq_ctrl_ );

    // Modify a copy of the solution to allow "OFTG_DF_->interpolate_feet_positions"
    // to use the correcte feet step previewed
    PrepareSolution();

    OFTG_DF_->interpolate_feet_positions( time + i * QP_T_,
                                          solution_.SupportStates_deq, solution_,
                                          solution_.SupportOrientations_deq,
                                          LeftFootTraj_deq_ctrl_, RightFootTraj_deq_ctrl_);
    solution_.SupportStates_deq.pop_front();
  }

  OrientPrw_DF_->CurrentTrunkState(FinalCurrentStateOrientPrw_);
  OrientPrw_DF_->CurrentTrunkState(FinalPreviewStateOrientPrw_);

  OFTG_DF_->copyPolynomesFromFTGS(OFTG_control_);

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
    const int IterationNumber,                                 // INPUT
    const unsigned int currentIndex)                            // INPUT
{
  if(aSolutionReference->SupportStates_deq.size() && aSolutionReference->SupportStates_deq[IterationNumber].NbStepsLeft == 0)
  {
    unsigned int i = currentIndex + IterationNumber * numberOfSample ;
    double jx = (LeftFootTraj_deq[i-1].x + RightFootTraj_deq[i-1].x)/2 - COMTraj_deq[i-1].x[0];
    double jy = (LeftFootTraj_deq[i-1].y + RightFootTraj_deq[i-1].y)/2 - COMTraj_deq[i-1].y[0];
    if(fabs(jx) < 1e-3 && fabs(jy) < 1e-3) { Running_ = false; }
    const double tf = 0.75;
    jx = 6/(tf*tf*tf)*(jx - tf*COMTraj_deq[i-1].x[1] - (tf*tf/2)*COMTraj_deq[i-1].x[2]);
    jy = 6/(tf*tf*tf)*(jy - tf*COMTraj_deq[i-1].y[1] - (tf*tf/2)*COMTraj_deq[i-1].y[2]);
    LIPM->Interpolation( COMTraj_deq, ZMPPositions, currentIndex + IterationNumber * numberOfSample,
                         jx, jy);
    LIPM->OneIteration( jx, jy );
  }
  else
  {
    Running_ = true;
    LIPM->Interpolation( COMTraj_deq, ZMPPositions, currentIndex + IterationNumber * numberOfSample,
                         aSolutionReference->Solution_vec[IterationNumber], aSolutionReference->Solution_vec[IterationNumber+QP_N_] );
    LIPM->OneIteration( aSolutionReference->Solution_vec[IterationNumber],aSolutionReference->Solution_vec[IterationNumber+QP_N_] );
  }
  return ;
}


void ZMPVelocityReferencedQP::InterpretSolutionVector()
{
  double Vx = VelRef_.Local.X ;
  double Vy = VelRef_.Local.Y ;
  if (Vx > 0.2 /*ms*/)
    Vx = 0.2 ;
  if (Vy > 0.2 /*ms*/)
    Vy = 0.2 ;
  std::deque<support_state_t> & SupportStates = solution_.SupportStates_deq ;
  support_state_t & LastSupport = solution_.SupportStates_deq.back() ;
  support_state_t & FirstSupport = solution_.SupportStates_deq[1] ;
  support_state_t & CurrentSupport = solution_.SupportStates_deq.front() ;
  int nbSteps = LastSupport.StepNumber ;
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
    int size_vec_sol = (int) FootPrw_vec.size();

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
      FootPrw_vec[size_vec_sol-1][0] = CurrentSupport.X + Sign*sin(FirstSupport.Yaw)*FeetDistance_;
      FootPrw_vec[size_vec_sol-1][1] = CurrentSupport.Y - Sign*cos(FirstSupport.Yaw)*FeetDistance_;
    }
  }
  for(unsigned int i = 0 ; i < SupportStates.size() ; ++i)
    {
      SupportStates[i].NbStepsLeft = 2 ;
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
                                                   Eigen::Vector3d &,
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

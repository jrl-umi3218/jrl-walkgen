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

  // Save the reference to HDR
  HDR_ = aHS ;

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

  // Initialize  the 2D LIPM
  CoM2_.SetSimulationControlPeriod( QP_T_ );
  CoM2_.SetRobotControlPeriod( m_SamplingPeriod );
  CoM2_.InitializeSystem();

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

  // Create and initialize the class that compute the simplify inverse kinematics :
  // ------------------------------------------------------------------------------
  ComAndFootRealization_ = new ComAndFootRealizationByGeometry( (PatternGeneratorInterfacePrivate*) SPM );
  ComAndFootRealization_->setHumanoidDynamicRobot(aHS);
  ComAndFootRealization_->SetHeightOfTheCoM(0.0);
  ComAndFootRealization_->setSamplingPeriod(0.005);
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
  m_PC = new PreviewControl(SPM,
			    OptimalControllerSolver::MODE_WITH_INITIALPOS,
			    true);
  m_PC->SetPreviewControlTime (QP_T_*(QP_N_-1));
  m_PC->SetSamplingPeriod (m_SamplingPeriod);
  m_PC->SetHeightOfCoM(0.814);

  // init of the debug files
  ofstream aof;
  string aFileName = "TestHerdt2010DynamicFilterSolutionTestFGPI.dat";
	aof.open(aFileName.c_str(),ofstream::out);

	// init of the buffer for the kajita's dynamic filter
    // number of sample inside one iteration of the preview control
	m_numberOfSample = (unsigned)(QP_T_/m_SamplingPeriod);
    // size = numberOfIterationOfThePreviewControl * NumberOfSample + Margin
  m_ZMPTraj_deq.resize( QP_N_ * m_numberOfSample + 50 );
  m_COMTraj_deq.resize( QP_N_ * m_numberOfSample + 50 );
  m_configurationTraj.resize( QP_N_ * m_numberOfSample );
  m_velocityTraj.resize( QP_N_ * m_numberOfSample );
  m_accelerationTraj.resize( QP_N_ * m_numberOfSample );
  m_deltaZMPMBPositions.resize ( QP_N_ * m_numberOfSample );
    // Initialization of the configuration vectors
  m_previousConfiguration = aHS->currentConfiguration() ;
  m_previousVelocity = aHS->currentVelocity();
  m_previousAcceleration = aHS->currentAcceleration();
    // Configuration vectors at the end of the previewControl of Herdt
  m_QP_T_Configuration = aHS->currentConfiguration();
  m_QP_T_previousVelocity = aHS->currentVelocity();
  m_QP_T_previousAcceleration = aHS->currentAcceleration();

  m_once = true ;
  m_dInitX = 0 ;
  m_dInitY = 0 ;
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
  CoM_.SetComHeight(lStartingCOMState.z[0]);
  CoM_.InitializeSystem();
  CoM_(CoM);
  IntermedData_->CoM(CoM_());

  // Initialize preview of orientations
  OrientPrw_->CurrentTrunkState( lStartingCOMState );

  // initialisation of a second object that allow the interpolation along 1.6s
  CoM2_.SetComHeight(lStartingCOMState.z[0]);
  CoM2_.InitializeSystem();
  CoM2_(CoM);

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
    {
  	  VRQPGenerator_->compute_warm_start( Solution_ );//TODO: Move to update_problem or build_constraints?
    }
    Problem_.solve( QLD, Solution_, NONE );
    if(Solution_.Fail>0)
    {
      Problem_.dump( time );
    }

    static int iteration = 0 ;
    Solution_.print(cout);

    // INTERPOLATE THE NEXT COMPUTED COM STATE:
    // ----------------------------------------
    m_currentIndex = FinalCOMTraj_deq.size();
    m_solution = Solution_ ;
    for (unsigned i = 0 ; i < m_currentIndex ; i++)
    {
      m_ZMPTraj_deq[i] = FinalZMPTraj_deq[i] ;
      m_COMTraj_deq[i] = FinalCOMTraj_deq[i] ;
    }
    m_LeftFootTraj_deq = FinalLeftFootTraj_deq ;
    m_RightFootTraj_deq = FinalRightFootTraj_deq ;

    for ( int i = 0 ; i < QP_N_ ; i++ )
    {
      if(m_solution.SupportStates_deq.size() &&  m_solution.SupportStates_deq[0].NbStepsLeft == 0)
      {
        double jx = (FinalLeftFootTraj_deq[0].x + FinalRightFootTraj_deq[0].x)/2 - FinalCOMTraj_deq[0].x[0];
        double jy = (FinalLeftFootTraj_deq[0].y + FinalRightFootTraj_deq[0].y)/2 - FinalCOMTraj_deq[0].y[0];
        if(fabs(jx) < 1e-3 && fabs(jy) < 1e-3) { Running_ = false; }
        const double tf = 0.75;
        jx = 6/(tf*tf*tf)*(jx - tf*FinalCOMTraj_deq[0].x[1] - (tf*tf/2)*FinalCOMTraj_deq[0].x[2]);
        jy = 6/(tf*tf*tf)*(jy - tf*FinalCOMTraj_deq[0].y[1] - (tf*tf/2)*FinalCOMTraj_deq[0].y[2]);
        if(i == 0)
        {
          CoM2_.setState(CoM_());
        }
        CoM2_.Interpolation( m_COMTraj_deq, m_ZMPTraj_deq, m_currentIndex + i * m_numberOfSample,
                            jx, jy);
        CoM2_.OneIteration( jx, jy );
      }
      else
      {
        Running_ = true;
        if(i == 0)
        {
          CoM2_.setState(CoM_());
        }
        CoM2_.Interpolation( m_COMTraj_deq, m_ZMPTraj_deq, m_currentIndex + i * m_numberOfSample,
                            m_solution.Solution_vec[i], m_solution.Solution_vec[QP_N_+i] );
        CoM2_.OneIteration( m_solution.Solution_vec[i], m_solution.Solution_vec[QP_N_+i] );
      }
    }

    // INTERPOLATE TRUNK ORIENTATION AND THE COMPUTED FOOT POSITIONS :
    // ---------------------------------------------------------------
    OrientPrw_->interpolate_trunk_orientation( time, m_currentIndex,
          m_SamplingPeriod, m_solution.SupportStates_deq,
          m_COMTraj_deq );
    COMState aCoMState = OrientPrw_->CurrentTrunkState();
    Robot_->generate_trajectories( time, m_solution,
                      m_solution.SupportStates_deq, m_solution.SupportOrientations_deq,
                      m_LeftFootTraj_deq, m_RightFootTraj_deq );
    cout << "support X , Y = " << m_solution.SupportStates_deq.front().X << ", " <<  m_solution.SupportStates_deq.front().Y << endl ;
    m_solution.SupportStates_deq.pop_front();
    for ( int i = 1 ; i < QP_N_ ; i++ ){
      OrientPrw_->interpolate_trunk_orientation( time + i * QP_T_, m_currentIndex + i * m_numberOfSample,
            m_SamplingPeriod, m_solution.SupportStates_deq,
            m_COMTraj_deq );
      Robot_->generate_trajectories( time + i * QP_T_, m_solution,
                      m_solution.SupportStates_deq, m_solution.SupportOrientations_deq,
                      m_LeftFootTraj_deq, m_RightFootTraj_deq );
      cout << "support X , Y = " << m_solution.SupportStates_deq.front().X << ", " <<  m_solution.SupportStates_deq.front().Y << endl ;
      m_solution.SupportStates_deq.pop_front();
    }


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
    oss.str("TestHerdt2010DynamicBuffers");
    oss << "_" << iteration100 << iteration10 << iteration1 << ".dat";
    aFileName = oss.str();
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
    ///----
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    for(unsigned int i = 0 ; i < m_currentIndex + QP_N_ * m_numberOfSample ; i++){
      aof << m_COMTraj_deq[i].roll[0] << " "   // 1
          << m_COMTraj_deq[i].pitch[0] << " "   // 2
          << m_COMTraj_deq[i].yaw[0] << " "   // 3
          << m_COMTraj_deq[i].x[0] << " "   // 4
          << m_COMTraj_deq[i].y[0] << " "   // 5
          << m_ZMPTraj_deq[i].px << " "   // 6
          << m_ZMPTraj_deq[i].py << " "   // 7
          << m_LeftFootTraj_deq[i].theta *M_PI/180 << " "   // 8
          << m_RightFootTraj_deq[i].theta *M_PI/180 << " "   // 9
          << m_LeftFootTraj_deq[i].x << " "   // 10
          << m_RightFootTraj_deq[i].x << " "   // 11
          << m_LeftFootTraj_deq[i].y << " "   // 12
          << m_RightFootTraj_deq[i].y << " "   // 13
          << m_LeftFootTraj_deq[i].z << " "   // 14
          << m_RightFootTraj_deq[i].z << " "   // 15
          << endl ;
    }
    aof.close();

    FinalZMPTraj_deq.resize( m_numberOfSample + m_currentIndex );
    FinalLeftFootTraj_deq.resize( m_numberOfSample + m_currentIndex );
    FinalRightFootTraj_deq.resize( m_numberOfSample + m_currentIndex );
    for (unsigned int i = m_currentIndex ; i < FinalZMPTraj_deq.size() ; i++ )
    {
      FinalZMPTraj_deq[i] = m_ZMPTraj_deq[i] ;
      FinalLeftFootTraj_deq[i] = m_LeftFootTraj_deq[i] ;
      FinalRightFootTraj_deq[i] = m_RightFootTraj_deq[i] ;
    }
    // DYNAMIC FILTER
    // --------------
    if ( Solution_.SupportStates_deq.front().Phase == SS )
    {
      DynamicFilter( m_ZMPTraj_deq, m_COMTraj_deq, m_LeftFootTraj_deq, m_RightFootTraj_deq, m_currentIndex, time );
    }
    CoM_.setState(m_COMTraj_deq[m_numberOfSample + m_currentIndex - 1]);
    OrientPrw_->CurrentTrunkState(m_COMTraj_deq[m_numberOfSample + m_currentIndex - 1]);


    // RECOPIE DU BUFFER
    // -----------------
    FinalCOMTraj_deq.resize( m_numberOfSample + m_currentIndex );
    for (unsigned int i = m_currentIndex ; i < FinalZMPTraj_deq.size() ; i++ )
    {
      FinalCOMTraj_deq[i] = m_COMTraj_deq[i] ;
    }

    /// \brief Debug Purpose
    /// --------------------
    oss.str("TestHerdt2010DynamicFinalBuffers");
    oss << "_" << iteration100 << iteration10 << iteration1 << ".dat";
    aFileName = oss.str();
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
    ///----
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    for(unsigned int i = 0 ; i < FinalZMPTraj_deq.size() ; i++){
      aof << FinalCOMTraj_deq[i].roll[0] << " "   // 1
          << FinalCOMTraj_deq[i].pitch[0] << " "   // 2
          << FinalCOMTraj_deq[i].yaw[0] << " "   // 3
          << FinalCOMTraj_deq[i].x[0] << " "   // 4
          << FinalCOMTraj_deq[i].y[0] << " "   // 5
          << FinalZMPTraj_deq[i].px << " "   // 6
          << FinalZMPTraj_deq[i].py << " "   // 7
          << FinalLeftFootTraj_deq[i].theta *M_PI/180 << " "   // 8
          << FinalRightFootTraj_deq[i].theta *M_PI/180 << " "   // 9
          << FinalLeftFootTraj_deq[i].x << " "   // 10
          << FinalRightFootTraj_deq[i].x << " "   // 11
          << FinalLeftFootTraj_deq[i].y << " "   // 12
          << FinalRightFootTraj_deq[i].y << " "   // 13
          << FinalLeftFootTraj_deq[i].z << " "   // 14
          << FinalRightFootTraj_deq[i].z << " "   // 15
          << endl ;
    }
    aof.close();

    /// \brief Debug Purpose
    /// --------------------
    if ( iteration == 0 )
    {
      oss.str("TestHerdt2010Orientation.dat");
      aFileName = oss.str();
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
    ///----
    oss.str("TestHerdt2010Orientation.dat");
    aFileName = oss.str();
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    aof   << iteration*0.1 << " "   // 1
          << FinalLeftFootTraj_deq[0].theta *M_PI/180 << " "   // 2
          << FinalRightFootTraj_deq[0].theta *M_PI/180 << " "   // 3
          << FinalCOMTraj_deq[0].roll[0] << " "   // 4
          << FinalCOMTraj_deq[0].pitch[0] << " "   // 5
          << FinalCOMTraj_deq[0].yaw[0] << " "   // 6
          << FinalCOMTraj_deq[0].x[0] << " "   // 7
          << FinalCOMTraj_deq[0].y[0] << " "   // 8
          << FinalZMPTraj_deq[0].px << " "   // 9
          << FinalZMPTraj_deq[0].py << " "   // 10
          << filterprecision( m_LeftFootTraj_deq[0].x ) << " "   // 11
          << filterprecision( m_LeftFootTraj_deq[0].y ) << " "   // 12
          << filterprecision( m_RightFootTraj_deq[0].x ) << " "   // 13
          << filterprecision( m_RightFootTraj_deq[0].y ) << " "   // 14

          << endl ;
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

int ZMPVelocityReferencedQP::DynamicFilter(std::deque<ZMPPosition> &ZMPPositions,
		      std::deque<COMState> & COMTraj_deq ,
		      std::deque<FootAbsolutePosition>& LeftFootAbsolutePositions,
		      std::deque<FootAbsolutePosition>& RightFootAbsolutePositions,
		      unsigned currentIndex,
		      double time
		      )
{

  /// \brief Debug Purpose
  /// --------------------
  ofstream aof;
  string aFileName;
  ostringstream oss(std::ostringstream::ate);
  static int iteration = 0 ;
  int iteration100 = (int)iteration/100;
  int iteration10 = (int)(iteration - iteration100*100)/10;
  int iteration1 = (int)(iteration - iteration100*100 - iteration10*10 );

  const unsigned int N = m_numberOfSample * QP_N_ ;
  // \brief calculate, from the CoM computed by the preview control,
  //    the corresponding articular position, velocity and acceleration
  // ------------------------------------------------------------------
  for(unsigned int i = 0 ; i <  N ; i++ ){
    CallToComAndFootRealization(
      COMTraj_deq[currentIndex+i],
      LeftFootAbsolutePositions [currentIndex+i],
      RightFootAbsolutePositions [currentIndex+i],
      m_configurationTraj[i],
      m_velocityTraj[i],
      m_accelerationTraj[i],
      i);
  }

  // \brief rnea, calculation of the multi body ZMP
  // ----------------------------------------------
  double zmpmbX, zmpmbY;
  /// \brief Debug Purpose
  /// --------------------
  oss.str("TestHerdt2010ZMPMB");
  oss << "_" << iteration100 << iteration10 << iteration1 << ".dat";
  aFileName = oss.str();
  aof.open(aFileName.c_str(),ofstream::out);
  aof.close();
  ///----
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);

  for (unsigned int i = 0 ; i < N ; i++ ){
    // Apply the RNEA to the metapod multibody and print the result in a log file.
    for(unsigned int j = 0 ; j < m_configurationTraj[i].size() ; j++ )
    {
      m_q(j,0) = m_configurationTraj[i](j) ;
      m_dq(j,0) = m_velocityTraj[i](j) ;
      m_ddq(j,0) = m_accelerationTraj[i](j) ;
    }
    metapod::rnea< Robot_Model, true >::run(m_robot, m_q, m_dq, m_ddq);

    Node & node = boost::fusion::at_c<Robot_Model::BODY>(m_robot.nodes);
    m_force = node.body.iX0.applyInv (node.joint.f);
    if (m_once){
      m_dInitX = ZMPPositions[currentIndex].px - ( - m_force.n()[1] / m_force.f()[2] ) ;
      m_dInitY = ZMPPositions[currentIndex].py - (   m_force.n()[0] / m_force.f()[2] ) ;
      m_once = false ;
    }
    m_deltaZMPMBPositions[i].px = ZMPPositions[currentIndex+i].px - ( - m_force.n()[1] / m_force.f()[2] ) - m_dInitX ;
    m_deltaZMPMBPositions[i].py = ZMPPositions[currentIndex+i].py - (   m_force.n()[0] / m_force.f()[2] ) - m_dInitY ;
    m_deltaZMPMBPositions[i].pz = 0.0 ;
    m_deltaZMPMBPositions[i].theta = 0.0 ;
    m_deltaZMPMBPositions[i].time = time + i * m_SamplingPeriod ;
    m_deltaZMPMBPositions[i].stepType = ZMPPositions[currentIndex+i].stepType ;

    if ( i == 0 ){
      zmpmbX = - m_force.n()[1] / m_force.f()[2] ;
      zmpmbY = m_force.n()[0] / m_force.f()[2] ;
    }
    aof << filterprecision( - m_force.n()[1] / m_force.f()[2] ) << " "   // 1
        << filterprecision(  m_force.n()[0] / m_force.f()[2] ) << " "   // 2
        << filterprecision( ZMPPositions[currentIndex+i].px ) << " "   // 3
        << filterprecision( ZMPPositions[currentIndex+i].py ) << " "   // 4
        << filterprecision( - m_force.n()[1] / m_force.f()[2] + m_dInitX) << " "   // 5
        << filterprecision(  m_force.n()[0] / m_force.f()[2]  + m_dInitY) << " "   // 6
        << endl ;


  }
  aof.close();

  /// Preview control on the ZMPMBs computed
  /// --------------------------------------
  //init of the Kajita preview control
  m_PC->SetPreviewControlTime (QP_T_*QP_N_ - 20*m_SamplingPeriod);
  m_PC->SetSamplingPeriod (m_SamplingPeriod);
  m_PC->SetHeightOfCoM(0.814);
  m_PC->ComputeOptimalWeights(OptimalControllerSolver::MODE_WITH_INITIALPOS);
  for(int j=0;j<3;j++)
  {
    m_deltax(j,0) = 0 ;
    m_deltay(j,0) = 0 ;
  }
  double aSxzmp (0) , aSyzmp(0);
  double deltaZMPx (0) , deltaZMPy (0) ;
  std::deque<COMState> COMStateBuffer (m_numberOfSample);
  // calcul of the preview control along the "ZMPPositions"
  for (unsigned i = 0 ; i < m_numberOfSample ; i++ )
  {
    m_PC->OneIterationOfPreview(m_deltax,m_deltay,
                                aSxzmp,aSyzmp,
                                m_deltaZMPMBPositions,i,
                                deltaZMPx, deltaZMPy, false);
    for(int j=0;j<3;j++)
    {
      COMStateBuffer[i].x[j] = m_deltax(j,0);
      COMStateBuffer[i].y[j] = m_deltay(j,0);
    }
  }



  for (unsigned int i = 0 ; i < m_numberOfSample ; i++)
  {
    for(int j=0;j<3;j++)
    {
      if ( COMStateBuffer[i].x[j] == COMStateBuffer[i].x[j] )
        COMTraj_deq[currentIndex+i].x[j] += COMStateBuffer[i].x[j] ;
      else
        cout << "PC problem nan in : x[" << j << "] and index : " << i << endl ;
      if ( COMStateBuffer[i].y[j] == COMStateBuffer[i].y[j] )
        COMTraj_deq[currentIndex+i].y[j] += COMStateBuffer[i].y[j] ;
      else
        cout << "PC problem nan in : y[" << j << "] and index : " << i << endl ;
    }
  }

    /// \brief Debug Purpose
    /// --------------------
    oss.str("TestHerdt2010blabla");
    oss << "_" << iteration100 << iteration10 << iteration1 << ".dat";
    aFileName = oss.str();
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
    ///----
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    for(unsigned int i = 0 ; i < COMStateBuffer.size() ; i++){
      aof << filterprecision( COMStateBuffer[i].x[0] ) << " "              // 1
          << filterprecision( COMStateBuffer[i].x[1] ) << " "              // 1
          << filterprecision( COMStateBuffer[i].x[2] ) << " "              // 1
          << filterprecision( COMStateBuffer[i].y[0] ) << " "              // 1
          << filterprecision( COMStateBuffer[i].y[1] ) << " "              // 1
          << filterprecision( COMStateBuffer[i].y[2] ) << " "              // 1
          << endl ;
    }
    aof.close();

  /// \brief Debug Purpose
  /// --------------------
  if ( iteration == 0 )
  {
    oss.str("TestHerdt2010ZMPMB.dat");
    aFileName = oss.str();
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
  }
  ///----
  oss.str("TestHerdt2010ZMPMB.dat");
  aFileName = oss.str();
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  aof << filterprecision( zmpmbX ) << " "              // 1
      << filterprecision( zmpmbY ) << " "              // 2
      << filterprecision( ZMPPositions[currentIndex].px ) << " "  // 3
      << filterprecision( ZMPPositions[currentIndex].py ) << " "  // 4
      << filterprecision( zmpmbX + m_dInitX ) << " "   // 5
      << filterprecision( zmpmbY + m_dInitY ) << " "  // 6
      << filterprecision( COMStateBuffer[currentIndex].x[1] ) << " "   // 7
      << filterprecision( COMStateBuffer[currentIndex].y[1] ) << " "  // 8
      << filterprecision( m_deltaZMPMBPositions[0].px ) << " "   // 9
      << filterprecision( m_deltaZMPMBPositions[0].py ) << " "  // 10
      << endl ;
  aof.close();

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
    CurrentConfiguration = m_previousConfiguration ;
    CurrentVelocity = m_previousVelocity ;
    CurrentAcceleration = m_previousAcceleration ;
  }
  ComAndFootRealization_->ComputePostureForGivenCoMAndFeetPosture(aCOMState, aCOMSpeed, aCOMAcc,
								    aLeftFootPosition,
								    aRightFootPosition,
								    CurrentConfiguration,
								    CurrentVelocity,
								    CurrentAcceleration,
								    IterationNumber,
								    0);
  if(IterationNumber == m_numberOfSample-1 ){
    HDR_->currentConfiguration(CurrentConfiguration);
    HDR_->currentConfiguration(CurrentVelocity);
    HDR_->currentConfiguration(CurrentAcceleration);
  }else{
    m_previousConfiguration = CurrentConfiguration ;
    m_previousVelocity = CurrentVelocity ;
    m_previousAcceleration = CurrentAcceleration ;
  }

  return ;
}

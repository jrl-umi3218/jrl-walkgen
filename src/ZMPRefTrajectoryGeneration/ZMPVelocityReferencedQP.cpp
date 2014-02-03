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
// metapod includes
typedef double LocalFloatType;
#include <metapod/models/hrp2_14/hrp2_14.hh>
#include <metapod/tools/print.hh>
#include <metapod/tools/initconf.hh>
#include "metapod/algos/rnea.hh"
#include <Eigen/StdVector>
typedef metapod::hrp2_14<LocalFloatType> Robot_Model;
#endif

#include <Debug.hh>
using namespace std;
using namespace PatternGeneratorJRL;
using namespace metapod;


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

  ComAndFootRealization_ = 0 ;

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
			    OptimalControllerSolver::MODE_WITHOUT_INITIALPOS,
			    true);

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
      unsigned numberOfSample = (unsigned)(QP_T_/m_SamplingPeriod);

      deque<ZMPPosition> ZMPTraj_deq (FinalZMPTraj_deq);
      deque<COMState> COMTraj_deq (FinalCOMTraj_deq);
      deque<FootAbsolutePosition> LeftFootTraj_deq (FinalLeftFootTraj_deq);
      deque<FootAbsolutePosition> RightFootTraj_deq (FinalRightFootTraj_deq);

      ZMPTraj_deq.resize( QP_N_ * numberOfSample + currentIndex );
      COMTraj_deq.resize( QP_N_ * numberOfSample + currentIndex );
      LeftFootTraj_deq.resize( QP_N_ * numberOfSample + currentIndex );
      RightFootTraj_deq.resize( QP_N_ * numberOfSample + currentIndex );

      for ( int i = 0 ; i < QP_N_ ; i++ )
      {
        if(Solution_.SupportStates_deq.size() &&  Solution_.SupportStates_deq[0].NbStepsLeft == 0)
        {
          double jx = (FinalLeftFootTraj_deq[0].x + FinalRightFootTraj_deq[0].x)/2 - FinalCOMTraj_deq[0].x[0];
          double jy = (FinalLeftFootTraj_deq[0].y + FinalRightFootTraj_deq[0].y)/2 - FinalCOMTraj_deq[0].y[0];
          if(fabs(jx) < 1e-3 && fabs(jy) < 1e-3) { Running_ = false; }
          const double tf = 0.75;
          jx = 6/(tf*tf*tf)*(jx - tf*FinalCOMTraj_deq[0].x[1] - (tf*tf/2)*FinalCOMTraj_deq[0].x[2]);
          jy = 6/(tf*tf*tf)*(jy - tf*FinalCOMTraj_deq[0].y[1] - (tf*tf/2)*FinalCOMTraj_deq[0].y[2]);
          CoM_.Interpolation( COMTraj_deq, ZMPTraj_deq, currentIndex + i * numberOfSample,
                              jx, jy);
          if(i == 0)
          {
            CoM_.OneIteration( jx, jy );
          }
        }
        else
        {
          Running_ = true;
          CoM_.Interpolation( COMTraj_deq, ZMPTraj_deq, currentIndex + i * numberOfSample,
               Solution_.Solution_vec[i], Solution_.Solution_vec[QP_N_+i] );
          if(i == 0)
          {
            CoM_.OneIteration( Solution_.Solution_vec[0],Solution_.Solution_vec[QP_N_] );
          }
        }

        // INTERPOLATE TRUNK ORIENTATION:
        // ------------------------------
        OrientPrw_->interpolate_trunk_orientation( time + i * QP_T_, currentIndex + i * numberOfSample,
            m_SamplingPeriod, Solution_.SupportStates_deq,
            COMTraj_deq );

        // INTERPOLATE THE COMPUTED FOOT POSITIONS:
        // ----------------------------------------
        deque<FootAbsolutePosition> LeftFootTraj (FinalLeftFootTraj_deq);
        deque<FootAbsolutePosition> RightFootTraj (FinalRightFootTraj_deq);
        Robot_->generate_trajectories( time + i * QP_T_ , Solution_,
            Solution_.SupportStates_deq, Solution_.SupportOrientations_deq,
            LeftFootTraj, RightFootTraj );

        for (unsigned int k = 0 ; k < numberOfSample ; k++)
        {
          LeftFootTraj_deq[currentIndex + i * numberOfSample + k] = LeftFootTraj[currentIndex + k] ;
          RightFootTraj_deq[currentIndex + i * numberOfSample + k] = RightFootTraj[currentIndex + k] ;
        }
      }

      // DYNAMIC FILTER
      // --------------
      //DynamicFilter( FinalZMPTraj_deq, FinalCOMTraj_deq, FinalLeftFootTraj_deq, FinalRightFootTraj_deq );

      FinalCOMTraj_deq.resize( numberOfSample + currentIndex );
      FinalZMPTraj_deq.resize( numberOfSample + currentIndex );
      FinalLeftFootTraj_deq.resize( numberOfSample + currentIndex );
      FinalRightFootTraj_deq.resize( numberOfSample + currentIndex );

      for (unsigned int i = 0 ; i < FinalZMPTraj_deq.size() ; i++ )
      {
        FinalZMPTraj_deq[i] = ZMPTraj_deq[i] ;
        FinalCOMTraj_deq[i] = COMTraj_deq[i] ;
        FinalLeftFootTraj_deq[i] = LeftFootTraj_deq[i] ;
        FinalRightFootTraj_deq[i] = RightFootTraj_deq[i] ;
      }

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

double filterprecision(double adb)
{
  if (fabs(adb)<1e-7)
    return 0.0;

  double ladb2 = adb * 1e7;
  double lintadb2 = trunc(ladb2);
  return lintadb2/1e7;
}

int ZMPVelocityReferencedQP::DynamicFilter(std::deque<ZMPPosition> & ZMPPositions,
		      std::deque<COMState> &FinalCOMTraj_deq ,
		      std::deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
		      std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
		      unsigned currentIndex
		      )
{
  const unsigned int N = FinalCOMTraj_deq.size();
  // \brief claculate, from the CoM of computed by the preview control,
  //    the corresponding articular position, velocity and acceleration
  // ------------------------------------------------------------------
  vector <MAL_VECTOR_TYPE(double)> configurationTraj (N) ;
  vector <MAL_VECTOR_TYPE(double)> velocityTraj (N) ;
  vector <MAL_VECTOR_TYPE(double)> accelerationTraj (N) ;
  for(unsigned int i = 0 ; i <  N ; i++ ){
    CallToComAndFootRealization(
      FinalCOMTraj_deq[i],
      LeftFootAbsolutePositions [i],
      RightFootAbsolutePositions [i],
      configurationTraj[i],
      velocityTraj[i],
      accelerationTraj[i],
      i);
  }

  // \brief rnea
  // -----------
  // Initialize the robot with the autogenerated files by MetapodFromUrdf
  Robot_Model robot;
  // Set configuration vectors (q, dq, ddq) to reference values.
  Robot_Model::confVector torques;
  vector < Robot_Model::confVector,Eigen::aligned_allocator<Robot_Model::confVector> > allTorques (N) ;
  Robot_Model::confVector q, dq, ddq;
  for (unsigned int i = 0 ; i < N ; i++ ){
    // Apply the RNEA to the metapod multibody and print the result in a log file.
    for(unsigned int j = 0 ; j < configurationTraj[i].size() ; j++ )
    {
      q(j,0) = configurationTraj[i][j] ;
      dq(j,0) = velocityTraj[i][j] ;
      ddq(j,0) = accelerationTraj[i][j] ;
    }
    metapod::rnea< Robot_Model, true >::run(robot, q, dq, ddq);
    getTorques(robot, torques);
    allTorques[i] = torques ;
  }

  // Projection of the Torques on the ground, the result is the ZMP Multi-Body
  // -------------------------------------------------------------------------
  double factor = 1/(RobotMass_*9.81);
  std::deque<ZMPPosition> deltaZMPMBPositions (N,ZMPPosition());
  for(unsigned int i = 0 ; i <  N ; i++ )
  {
    // Smooth ramp
    deltaZMPMBPositions[i].px = ZMPPositions[i].px + allTorques[i](4,0) /*(1,0)*/  * factor ;
    deltaZMPMBPositions[i].py = ZMPPositions[i].py - allTorques[i](3,0) /*(0,0)*/  * factor ;
    deltaZMPMBPositions[i].pz = 0.0 ;
    deltaZMPMBPositions[i].theta = 0.0;
    deltaZMPMBPositions[i].time = m_CurrentTime;
    deltaZMPMBPositions[i].stepType = ZMPPositions[i].stepType ;
  }

  static int iteration = 0;
  if (iteration == 0)
  {
    ofstream aof;
    string aFileName;
	  aFileName = "TestHerdt2010DynamicFilterPC.dat";
	  aof.open(aFileName.c_str(),ofstream::out);
	  aof.close();
  }
  ofstream aof;
  string aFileName;
	aFileName = "TestHerdt2010DynamicFilterPC.dat";
	aof.open(aFileName.c_str(),ofstream::app);
	aof.precision(8);
	aof.setf(ios::scientific, ios::floatfield);
	aof << filterprecision(iteration*0.005 ) << " "             // 1
	    << filterprecision(deltaZMPMBPositions[0].px ) << " "   // 2
	    << filterprecision(deltaZMPMBPositions[0].py ) << " "   // 3
      << filterprecision(FinalCOMTraj_deq[0].x[0]) << " "     // 4
      << filterprecision(FinalCOMTraj_deq[0].y[0]) << " "     // 5
      << filterprecision(ZMPPositions[0].px) << " "        // 6
      << filterprecision(ZMPPositions[0].py) << " " ;      // 7
  iteration++;

  // Preview control on the ZMPMBs computed
  // --------------------------------------
  // setup
  m_PC->SetPreviewControlTime (QP_T_*(QP_N_*0.5-1));
  m_PC->SetSamplingPeriod (m_SamplingPeriod);
  m_PC->SetHeightOfCoM(Robot_->CoMHeight());
  m_PC->ComputeOptimalWeights(OptimalControllerSolver::MODE_WITHOUT_INITIALPOS);
  double aSxzmp (0) , aSyzmp(0);
  double deltaZMPx (0) , deltaZMPy (0) ;
    // calcul of the preview control along the "ZMPPositions"
  for (int i = 0 ; i < QP_N_ ; i++ )
  {
    m_PC->OneIterationOfPreview(m_deltax,m_deltay,
                                aSxzmp,aSyzmp,
                                deltaZMPMBPositions,i,
                                deltaZMPx, deltaZMPy, true);

    for(int j=0;j<3;j++)
    {
      FinalCOMTraj_deq[i].x[j] += m_deltax(j,0);
      FinalCOMTraj_deq[i].y[j] += m_deltay(j,0);
    }
  }


  aof << filterprecision(m_deltax(0,0)) << " "            // 8
	    << filterprecision(m_deltay(0,0)) << " "            // 9
      << filterprecision(FinalCOMTraj_deq[0].x[0]) << " " // 10
      << filterprecision(FinalCOMTraj_deq[0].y[0]) << " " // 11
      << endl ;
  aof.close();

  return 0;
}


void ZMPVelocityReferencedQP::CallToComAndFootRealization(COMState &acomp,
     FootAbsolutePosition &aLeftFAP,
     FootAbsolutePosition &aRightFAP,
     MAL_VECTOR_TYPE(double) &CurrentConfiguration,
     MAL_VECTOR_TYPE(double) &CurrentVelocity,
     MAL_VECTOR_TYPE(double) &CurrentAcceleration,
     int IterationNumber)
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

  /* Get the current configuration vector */
  CurrentConfiguration = HDR_->currentConfiguration();

  /* Get the current velocity vector */
  CurrentVelocity = HDR_->currentVelocity();

  /* Get the current acceleration vector */
  CurrentAcceleration = HDR_->currentAcceleration();

  static int StageOfTheAlgorithm = 0 ;
  ComAndFootRealization_->ComputePostureForGivenCoMAndFeetPosture(aCOMState, aCOMSpeed, aCOMAcc,
								    aLeftFootPosition,
								    aRightFootPosition,
								    CurrentConfiguration,
								    CurrentVelocity,
								    CurrentAcceleration,
								    IterationNumber,
								    StageOfTheAlgorithm);
 }

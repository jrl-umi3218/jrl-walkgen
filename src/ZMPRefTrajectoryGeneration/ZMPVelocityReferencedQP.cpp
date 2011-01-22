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



ZMPVelocityReferencedQP::ZMPVelocityReferencedQP(SimplePluginManager *lSPM,
						 string DataFile,
						 CjrlHumanoidDynamicRobot *aHS) :
  ZMPRefTrajectoryGeneration(lSPM)
{

  m_TimeBuffer = 0.040;

  m_FullDebug = 0;
  m_FastFormulationMode = QLD;

  m_QP_T = 0.1;
  m_QP_N = 16;

  m_SamplingPeriod = 0.005;

  m_ComHeight = 0.814;

  m_UpperTimeLimitToUpdate = 0.0;

  // Getting the ZMP reference from Kajita's heuristic.
  m_ZMPD = new ZMPDiscretization(lSPM,DataFile,aHS);

  // For computing the equilibrium constraints from the feet positions.
  m_fCALS = new FootConstraintsAsLinearSystemForVelRef(lSPM,aHS);

  m_FTGS = new FootTrajectoryGenerationStandard(lSPM,aHS->leftFoot());
  m_FTGS->InitializeInternalDataStructures();

  //Initialize the support state
  support_state_t CurrentSupport;
  CurrentSupport.Phase = 0;
  CurrentSupport.Foot = 1;
  CurrentSupport.TimeLimit = 1000000000;
  CurrentSupport.StepsLeft = 1;
  CurrentSupport.StateChanged = false;
  CurrentSupport.x = 0.0;
  CurrentSupport.y = 0.1;
  CurrentSupport.yaw = 0.0;
  CurrentSupport.StartTime = 0.0;

  m_Matrices.SupportState(CurrentSupport);

  m_SupportFSM = new SupportFSM(0.1);

  /* Orientations preview algorithm*/
  m_OP = new OrientationsPreview(0.1, 16, m_SupportFSM->m_SSPeriod, aHS->rootJoint());

  m_RobotMass = aHS->mass();
  m_TrunkState.yaw[0]=m_TrunkState.yaw[1]=m_TrunkState.yaw[2]=0.0;

  /// Initialize  the 2D LIPM
  m_CoM.SetSimulationControlPeriod(0.1);
  m_CoM.SetRobotControlPeriod(0.005);
  m_CoM.SetComHeight(0.814);
  m_CoM.InitializeSystem();

  // Register method to handle
  string aMethodName[2] =
    {":previewcontroltime",
     ":numberstepsbeforestop"};

  for(int i=0;i<2;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
    }

  //Feet distance in the DS phase
  m_FeetDistanceDS = 0.2;

  m_PerturbationOccured = false;


  m_GenVR = new GeneratorVelRef(lSPM );

  m_GenVR->setNbPrwSamplings(16);
  m_GenVR->setSamplingPeriodPreview(0.1);

  m_GenVR->setNbVariables(32);
  m_GenVR->setComHeight(0.814);

  m_GenVR->initialize(m_Matrices);
  m_GenVR->setPonderation(m_Matrices, 1.0, IntermedQPMat::INSTANT_VELOCITY);
  m_GenVR->setPonderation(m_Matrices, 0.000001, IntermedQPMat::COP_CENTERING);
  m_GenVR->setPonderation(m_Matrices, 0.00001, IntermedQPMat::JERK_MIN);

  m_InvariantPartInitialized = false;

}

ZMPVelocityReferencedQP::~ZMPVelocityReferencedQP()
{

  if (m_GenVR!=0)
    delete m_GenVR;

  if (m_ZMPD!=0)
    delete m_ZMPD;

  if (m_SupportFSM!=0)
    delete m_SupportFSM;

  if (m_fCALS!=0)
    delete m_fCALS;

  if (m_FTGS!=0)
    delete m_FTGS;

  if (m_OP!=0)
    delete m_OP;

}


void ZMPVelocityReferencedQP::setVelReference(istringstream &strm)
{
  strm >> m_VelRef.local.x;
  strm >> m_VelRef.local.y;
  strm >> m_VelRef.local.yaw;
}

void ZMPVelocityReferencedQP::setVelReference(double dx,
					      double dy,
					      double dyaw)
{
  m_VelRef.local.x = dx;
  m_VelRef.local.y = dy;
  m_VelRef.local.yaw = dyaw;
}

void ZMPVelocityReferencedQP::setCoMPerturbationForce(istringstream &strm)
{
  MAL_VECTOR_RESIZE(m_PerturbationAcceleration,6);

  strm >> m_PerturbationAcceleration(2);
  strm >> m_PerturbationAcceleration(5);
  m_PerturbationAcceleration(2) = m_PerturbationAcceleration(2)/m_RobotMass;
  m_PerturbationAcceleration(5) = m_PerturbationAcceleration(5)/m_RobotMass;
  m_PerturbationOccured = true;
}

void ZMPVelocityReferencedQP::setCoMPerturbationForce(double x,double y)
{
  MAL_VECTOR_RESIZE(m_PerturbationAcceleration,6);

  m_PerturbationAcceleration(2) = x/m_RobotMass;
  m_PerturbationAcceleration(5) = y/m_RobotMass;
  m_PerturbationOccured = true;

}

void ZMPVelocityReferencedQP::interpolateFeet(deque<FootAbsolutePosition> &,
					      deque<FootAbsolutePosition> &)
{

  printf("To be implemented \n");
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
      strm >> m_SupportFSM->m_NbOfStepsSSDS;
    }
  if (Method==":comheight")
    {
      strm >> m_ComHeight;
      m_CoM.SetComHeight(m_ComHeight);
    }

  ZMPRefTrajectoryGeneration::CallMethod(Method,strm);
}


int
ZMPVelocityReferencedQP::InitOnLine(deque<ZMPPosition> & FinalZMPTraj_deq,
					deque<COMState> & FinalCoMPositions,
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
  m_EndingPhase = false;
  m_TimeToStopOnLineMode = -1.0;

  // Initialize position of the feet.
  CurrentLeftFootAbsPos = InitLeftFootAbsolutePosition;
  CurrentLeftFootAbsPos.z = 0.0;//m_FTGS->m_AnklePositionLeft[2];
  CurrentLeftFootAbsPos.time = 0.0;
  CurrentLeftFootAbsPos.theta = 0.0;


  CurrentRightFootAbsPos = InitRightFootAbsolutePosition;
  CurrentRightFootAbsPos.z = 0.0;//m_FTGS->m_AnklePositionRight[2];
  CurrentRightFootAbsPos.time = 0.0;
  CurrentRightFootAbsPos.theta = 0.0;

  // V pre is the difference between
  // the current m_SupportFSM position and the precedent.


  int AddArraySize;
  {
    assert(m_SamplingPeriod > 0);
    double ldAddArraySize = m_TimeBuffer/m_SamplingPeriod;
    AddArraySize = (int) ldAddArraySize;
  }

  FinalZMPTraj_deq.resize(AddArraySize);
  FinalCoMPositions.resize(AddArraySize);
  FinalLeftFootTraj_deq.resize(AddArraySize);
  FinalRightFootTraj_deq.resize(AddArraySize);
  int CurrentZMPindex=0;


  for( unsigned int i=0;i<FinalZMPTraj_deq.size();i++)
    {

      // Smooth ramp
      FinalZMPTraj_deq[CurrentZMPindex].px = lStartingZMPPosition(0);
      FinalZMPTraj_deq[CurrentZMPindex].py = lStartingZMPPosition(1);
      FinalZMPTraj_deq[CurrentZMPindex].pz = lStartingZMPPosition(2);
      FinalZMPTraj_deq[CurrentZMPindex].theta = 0.0;
      FinalZMPTraj_deq[CurrentZMPindex].time = m_CurrentTime;
      FinalZMPTraj_deq[CurrentZMPindex].stepType = 0;

      // Set CoM positions.
      FinalCoMPositions[CurrentZMPindex] = lStartingCOMState;
      // Set Left Foot positions.
      FinalLeftFootTraj_deq[CurrentZMPindex] = CurrentLeftFootAbsPos;
      FinalRightFootTraj_deq[CurrentZMPindex] = CurrentRightFootAbsPos;

      FinalLeftFootTraj_deq[CurrentZMPindex].time =
        FinalRightFootTraj_deq[CurrentZMPindex].time = m_CurrentTime;

      FinalLeftFootTraj_deq[CurrentZMPindex].stepType =
        FinalRightFootTraj_deq[CurrentZMPindex].stepType = 10;



      if(m_FullDebug>0)
	{
	  //Feet coordinates for plot in scilab
	  ofstream aoffeet;
	  aoffeet.open("/tmp/Feet.dat",ios::app);
	  aoffeet<<FinalLeftFootTraj_deq[CurrentZMPindex].time<<"    "
		 <<FinalLeftFootTraj_deq[CurrentZMPindex].x<<"    "
		 <<FinalLeftFootTraj_deq[CurrentZMPindex].y<<"    "
		 <<FinalLeftFootTraj_deq[CurrentZMPindex].z<<"    "
		 <<FinalLeftFootTraj_deq[CurrentZMPindex].stepType<<"    "
		 <<FinalRightFootTraj_deq[CurrentZMPindex].x<<"    "
		 <<FinalRightFootTraj_deq[CurrentZMPindex].y<<"    "
		 <<FinalRightFootTraj_deq[CurrentZMPindex].z<<"    "
		 <<FinalRightFootTraj_deq[CurrentZMPindex].stepType<<"    "<<endl;
	  aoffeet.close();
	}

      m_CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;

    }

  com_t CoM;

  CoM.x[0] = lStartingCOMState.x[0];
  CoM.x[1] = lStartingCOMState.x[1];
  CoM.x[2] = lStartingCOMState.x[2];
  CoM.y[0] = lStartingCOMState.y[0];
  CoM.y[1] = lStartingCOMState.y[1];
  CoM.y[2] = lStartingCOMState.y[2];

  m_CoM(CoM);
  m_Matrices.CoM(m_CoM());

  return 0;
}


void
ZMPVelocityReferencedQP::interpolateTrunkState(double time, int CurrentIndex,
                                                    const support_state_t & CurrentSupport,
						    deque<COMState> & FinalCOMTraj_deq)
{
  if(CurrentSupport.Phase == 1 && time+m_TimeBuffer+3.0/2.0*m_QP_T < CurrentSupport.TimeLimit)
    {
      //Set parameters for trunk interpolation
      m_c = 3.0*(m_TrunkStateT.yaw[1]-m_TrunkState.yaw[1])/(m_QP_T*m_QP_T);
      m_d = -2.0*m_c/(3.0*m_QP_T);
      m_a =  m_TrunkState.yaw[1];

      double tT;
      double Theta = m_TrunkState.yaw[0];
      //double dTheta = m_TrunkState.yaw[1];
      //double ddTheta = m_TrunkState.yaw[2];

      FinalCOMTraj_deq[CurrentIndex].yaw[0] = m_TrunkState.yaw[0];
      //Interpolate the
      for(int k = 1; k<=(int)(m_QP_T/m_SamplingPeriod);k++)
	{
	  tT = (double)k*m_SamplingPeriod;
	  //interpolate the orientation of the trunk
	  if(fabs(m_TrunkStateT.yaw[1]-m_TrunkState.yaw[1])-0.000001 > 0)
	    {
	      m_TrunkState.yaw[0] = (((1.0/4.0*m_d*tT+1.0/3.0*m_c)*
				      tT)*tT+m_a)*tT+Theta;
	      m_TrunkState.yaw[1] = ((m_d*tT+m_c)*tT)*tT+m_a;
	      m_TrunkState.yaw[2] = (3.0*m_d*tT+2.0*m_c)*tT;

	      m_QueueOfTrunkStates.push_back(m_TrunkState);
	    }
	  else
	    {
	      m_TrunkState.yaw[0] += m_SamplingPeriod*m_TrunkStateT.yaw[1];

	      m_QueueOfTrunkStates.push_back(m_TrunkState);
	    }
	  FinalCOMTraj_deq[CurrentIndex+k].yaw[0] = m_TrunkState.yaw[0];
	  if(m_FullDebug>2)
	    {
	      ofstream aof;
	      aof.open("/tmp/Trunk.dat",ofstream::app);
	      aof<<time+k*m_SamplingPeriod<<" "<<m_TrunkState.yaw[0]<<" "<<m_TrunkState.yaw[1]<<" "<<m_TrunkState.yaw[2]<<endl;
	      aof.close();
	    }
	}
    }
  else if (CurrentSupport.Phase == 0 || time+m_TimeBuffer+3.0/2.0*m_QP_T > CurrentSupport.TimeLimit)
    {
      for(int k = 0; k<=(int)(m_QP_T/m_SamplingPeriod);k++)
	{
	  FinalCOMTraj_deq[CurrentIndex+k].yaw[0] = m_TrunkState.yaw[0];
	}
    }
  

}


void ZMPVelocityReferencedQP::interpolateFeetPositions(double time, int CurrentIndex,
                                                       const support_state_t & CurrentSupport,
                                                       const deque<double> & PreviewedSupportAngles_deq,
						       deque<FootAbsolutePosition> &FinalLeftFootTraj_deq,
						       deque<FootAbsolutePosition> &FinalRightFootTraj_deq)
{
  double LocalInterpolationTime = (time+m_TimeBuffer)-(CurrentSupport.TimeLimit-m_SupportFSM->m_SSPeriod);

  double StepHeight = 0.05;
  int StepType = 1;

  if(CurrentSupport.Phase == 1 && time+m_TimeBuffer+3.0/2.0*m_QP_T < CurrentSupport.TimeLimit)
    {
      //determine coefficients of interpolation polynom
      double ModulationSupportCoefficient = 0.9;
      double ModulatedSingleSupportTime = (m_SupportFSM->m_SSPeriod-m_QP_T) * ModulationSupportCoefficient;
      double EndOfLiftOff = ((m_SupportFSM->m_SSPeriod-m_QP_T)-ModulatedSingleSupportTime)*0.5;
      double InterpolationTimePassed = 0.0;
      if(LocalInterpolationTime>EndOfLiftOff)
	InterpolationTimePassed = LocalInterpolationTime-EndOfLiftOff;

      FootAbsolutePosition LastSwingFootPosition;

      if(CurrentSupport.Foot==1)
	{
	  LastSwingFootPosition = FinalRightFootTraj_deq[CurrentIndex];
	}
      else
	{
	  LastSwingFootPosition = FinalLeftFootTraj_deq[CurrentIndex];
	}
      //Set parameters for current polynomial
      m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::X_AXIS,
						ModulatedSingleSupportTime-InterpolationTimePassed,m_FPx,
						LastSwingFootPosition.x,
						LastSwingFootPosition.dx);
      m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::Y_AXIS,
						ModulatedSingleSupportTime-InterpolationTimePassed,m_FPy,
						LastSwingFootPosition.y,
						LastSwingFootPosition.dy);

      if(CurrentSupport.StateChanged==true)
	m_FTGS->SetParameters(FootTrajectoryGenerationStandard::Z_AXIS, m_SupportFSM->m_SSPeriod-m_QP_T,StepHeight);

      m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::THETA_AXIS,
						ModulatedSingleSupportTime-InterpolationTimePassed,
						PreviewedSupportAngles_deq[0]*180.0/M_PI,
						LastSwingFootPosition.theta,
						LastSwingFootPosition.dtheta);
      m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::OMEGA_AXIS,
						ModulatedSingleSupportTime-InterpolationTimePassed,0.0*180.0/M_PI,
						LastSwingFootPosition.omega,
						LastSwingFootPosition.domega);
      m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::OMEGA2_AXIS,
						ModulatedSingleSupportTime-InterpolationTimePassed,2*0.0*180.0/M_PI,
						LastSwingFootPosition.omega2,
						LastSwingFootPosition.domega2);

      for(int k = 1; k<=(int)(m_QP_T/m_SamplingPeriod);k++)
	{
	  if (CurrentSupport.Foot==1)
	    {
	      m_FTGS->UpdateFootPosition(FinalLeftFootTraj_deq,
					 FinalRightFootTraj_deq,
					 CurrentIndex,k,
					 LocalInterpolationTime,
					 ModulatedSingleSupportTime,
					 StepType, -1);
	    }
	  else
	    {
	      m_FTGS->UpdateFootPosition(FinalRightFootTraj_deq,
					 FinalLeftFootTraj_deq,
					 CurrentIndex,k,
					 LocalInterpolationTime,
					 ModulatedSingleSupportTime,
					 StepType, 1);
	    }
	  FinalLeftFootTraj_deq[CurrentIndex+k].time =
	    FinalRightFootTraj_deq[CurrentIndex+k].time = time+m_TimeBuffer+k*m_SamplingPeriod;


	  if(m_FullDebug>0)
	    {
	      ofstream aoffeet;
	      aoffeet.open("/tmp/Feet.dat",ios::app);
	      aoffeet<<time+m_TimeBuffer+k*m_SamplingPeriod<<"    "
		     <<FinalLeftFootTraj_deq[CurrentIndex+k].x<<"    "
		     <<FinalLeftFootTraj_deq[CurrentIndex+k].y<<"    "
		     <<FinalLeftFootTraj_deq[CurrentIndex+k].z<<"    "
		     <<FinalLeftFootTraj_deq[CurrentIndex+k].stepType<<"    "
		     <<FinalRightFootTraj_deq[CurrentIndex+k].x<<"    "
		     <<FinalRightFootTraj_deq[CurrentIndex+k].y<<"    "
		     <<FinalRightFootTraj_deq[CurrentIndex+k].z<<"    "
		     <<FinalRightFootTraj_deq[CurrentIndex+k].stepType<<"    "
		     <<endl;
	      aoffeet.close();
	    }

	}
    }
  else if (CurrentSupport.Phase == 0 || time+m_TimeBuffer+3.0/2.0*m_QP_T > CurrentSupport.TimeLimit)
    {
      for(int k = 0; k<=(int)(m_QP_T/m_SamplingPeriod);k++)
	{
	  FinalRightFootTraj_deq[CurrentIndex+k]=FinalRightFootTraj_deq[CurrentIndex+k-1];
	  FinalLeftFootTraj_deq[CurrentIndex+k]=FinalLeftFootTraj_deq[CurrentIndex+k-1];
	  FinalLeftFootTraj_deq[CurrentIndex+k].time =
	    FinalRightFootTraj_deq[CurrentIndex+k].time = time+m_TimeBuffer+k*m_SamplingPeriod;
	  FinalLeftFootTraj_deq[CurrentIndex+k].stepType =
	    FinalRightFootTraj_deq[CurrentIndex+k].stepType = 10;

	  if(m_FullDebug>0)
	    {
	      ofstream aoffeet;
	      aoffeet.open("/tmp/Feet.dat",ios::app);
	      aoffeet<<time+m_TimeBuffer+k*m_SamplingPeriod<<"    "
		     <<FinalLeftFootTraj_deq[CurrentIndex+k].x<<"    "
		     <<FinalLeftFootTraj_deq[CurrentIndex+k].y<<"    "
		     <<FinalLeftFootTraj_deq[CurrentIndex+k].z<<"    "
		     <<FinalLeftFootTraj_deq[CurrentIndex+k].stepType<<"    "
		     <<FinalRightFootTraj_deq[CurrentIndex+k].x<<"    "
		     <<FinalRightFootTraj_deq[CurrentIndex+k].y<<"    "
		     <<FinalRightFootTraj_deq[CurrentIndex+k].z<<"    "
		     <<FinalRightFootTraj_deq[CurrentIndex+k].stepType<<"    "
		     <<endl;
	      aoffeet.close();
	    }
	}
    }
}


void ZMPVelocityReferencedQP::OnLine(double time,
				     deque<ZMPPosition> & FinalZMPTraj_deq,
				     deque<COMState> & FinalCOMTraj_deq,
				     deque<FootAbsolutePosition> &FinalLeftFootTraj_deq,
				     deque<FootAbsolutePosition> &FinalRightFootTraj_deq)
{


  // If on-line mode not activated we go out.
  if (!m_OnLineMode)
    { return; }

  // Testing if we are reaching the end of the online mode.
  if ((m_EndingPhase) &&
      (time>=m_TimeToStopOnLineMode))
    { m_OnLineMode = false; }


  // Apply external forces if occured
  if(m_PerturbationOccured == true)
    {
      com_t com = m_CoM();
      com.x(2) = com.x(2)+m_PerturbationAcceleration(2);
      com.y(2) = com.y(2)+m_PerturbationAcceleration(5);
      m_PerturbationOccured = false;
      m_CoM(com);
    }


  if(time + 0.00001 > m_UpperTimeLimitToUpdate)
    {
      double TotalAmountOfCPUTime=0.0,CurrentCPUTime=0.0;
      struct timeval start,end;
      gettimeofday(&start,0);


      // UPDATE DATA:
      // ------------
      m_Matrices.Reference(m_VelRef);
      m_GenVR->setCurrentTime(time+m_TimeBuffer);


      // PREVIEW SUPPORT STATES FOR THE WHOLE PREVIEW WINDOW:
      // ----------------------------------------------------
      deque<support_state_t> PrwSupportStates_deq;
      m_GenVR->previewSupportStates(m_Matrices, m_SupportFSM, PrwSupportStates_deq);


      // DETERMINE CURRENT SUPPORT POSITION:
      // -----------------------------------
      support_state_t CurrentSupport = PrwSupportStates_deq.front();
      //Add a new support foot to the support feet history deque
      if(CurrentSupport.StateChanged == true)
        {
          FootAbsolutePosition FAP;
          if(CurrentSupport.Foot==1)
            FAP = FinalLeftFootTraj_deq.back();
          else
            FAP = FinalRightFootTraj_deq.back();
          CurrentSupport.x = FAP.x;
          CurrentSupport.y = FAP.y;
          CurrentSupport.yaw = FAP.theta*M_PI/180.0;
          CurrentSupport.StartTime = m_CurrentTime;
          m_Matrices.SupportState(CurrentSupport);
        }


      // DEFINE ORIENTATIONS OF FEET FOR WHOLE PREVIEW PERIOD:
      // -----------------------------------------------------
      deque<double> PreviewedSupportAngles_deq;
      m_OP->verifyAccelerationOfHipJoint(m_VelRef, m_TrunkState,
					 m_TrunkStateT, CurrentSupport);
      m_OP->previewOrientations(time+m_TimeBuffer,
				PreviewedSupportAngles_deq,
				m_TrunkState,
				m_TrunkStateT,
				m_SupportFSM, CurrentSupport,
				FinalLeftFootTraj_deq,
				FinalRightFootTraj_deq);


      // COMPUTE REFERENCE IN THE GLOBAL FRAME:
      // --------------------------------------
      m_GenVR->computeGlobalReference(m_Matrices, m_TrunkStateT);


      // BUILD CONSTANT PART OF THE OBJECTIVE:
      // -------------------------------------
      m_GenVR->buildInvariantPart(m_Pb, m_Matrices);


      // BUILD VARIANT PART OF THE OBJECTIVE:
      // ------------------------------------
      m_GenVR->updateProblem(m_Pb, m_Matrices, PrwSupportStates_deq);


      // BUILD CONSTRAINTS:
      // ------------------
      m_GenVR->buildConstraints(m_Matrices, m_Pb,
          m_fCALS,
          FinalLeftFootTraj_deq,
          FinalRightFootTraj_deq,
          PrwSupportStates_deq,
          PreviewedSupportAngles_deq);


      // SOLVE PROBLEM:
      // --------------
      QPProblem_s::solution_t Result;
      m_Pb.solve( QPProblem_s::QLD , Result );


      // INTERPOLATE THE NEXT COMPUTED COM STATE:
      // ----------------------------------------
      FinalCOMTraj_deq.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      FinalZMPTraj_deq.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      FinalLeftFootTraj_deq.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      FinalRightFootTraj_deq.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      int CurrentIndex = (int)(m_TimeBuffer/m_SamplingPeriod)-1;
      m_CoM.Interpolation(FinalCOMTraj_deq,
			      FinalZMPTraj_deq,
			      CurrentIndex,
			      Result.vecSolution[0],Result.vecSolution[m_QP_N]);
      m_Matrices.CoM(m_CoM.OneIteration(Result.vecSolution[0],Result.vecSolution[m_QP_N]));


      // INTERPOLATE THE COMPUTED FEET POSITIONS:
      // ----------------------------------------
      if(CurrentSupport.StepsLeft>0)
	{
	  if(fabs(Result.vecSolution[2*m_QP_N])-0.00001<0.0)
	    {
	      cout<<"Previewed foot position zero at time: "<<time<<endl;
	    }
	  else if (CurrentSupport.TimeLimit-time-m_QP_T/2.0>0)
	    {//The landing position is yet determined by the solver because the robot finds himself still in the single support phase
	      m_FPx = Result.vecSolution[2*m_QP_N];
	      m_FPy = Result.vecSolution[2*m_QP_N+PrwSupportStates_deq.back().StepNumber];
	    }
	}
      else
	{//The solver isn't responsible for the feet positions anymore
         //The robot is supposed to stop always with the feet aligned in the lateral plane.
	  m_FPx = CurrentSupport.x + double(CurrentSupport.Foot)*sin(CurrentSupport.yaw)*m_FeetDistanceDS;
	  m_FPy = CurrentSupport.y - double(CurrentSupport.Foot)*cos(CurrentSupport.yaw)*m_FeetDistanceDS;

	  // Specify that we are in the ending phase.
	  if (m_EndingPhase==false)
	    {
	      // This should be done only during the transition EndingPhase=false -> EndingPhase=true
	      m_TimeToStopOnLineMode = m_UpperTimeLimitToUpdate+m_QP_T * m_QP_N;
	      // Set the ZMP reference as very important.
	      // It suppose to work because Gamma appears only during the non-constant 
	    }
	  m_EndingPhase = true;
	}

      interpolateTrunkState(time, CurrentIndex,
                            CurrentSupport,
			    FinalCOMTraj_deq);
      interpolateFeetPositions(time, CurrentIndex,
                               CurrentSupport,
                               PreviewedSupportAngles_deq,
			       FinalLeftFootTraj_deq,
			       FinalRightFootTraj_deq);



      m_UpperTimeLimitToUpdate = m_UpperTimeLimitToUpdate+m_QP_T;




      // Compute CPU consumption time.
      gettimeofday(&end,0);
      CurrentCPUTime = end.tv_sec - start.tv_sec +
        0.000001 * (end.tv_usec - start.tv_usec);
      TotalAmountOfCPUTime += CurrentCPUTime;
    }

}


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
}


void ZMPVelocityReferencedQP::OnLineAddFoot(RelativeFootPosition & ,
					    deque<ZMPPosition> & ,
					    deque<COMState> & ,
					    deque<FootAbsolutePosition> &,
					    deque<FootAbsolutePosition> &,
					    bool)
{
  cout << "To be implemented" << endl;
}

int ZMPVelocityReferencedQP::OnLineFootChange(double ,
					      FootAbsolutePosition &,
					      deque<ZMPPosition> & ,
					      deque<COMState> & ,
					      deque<FootAbsolutePosition> &,
					      deque<FootAbsolutePosition> &,
					      StepStackHandler  *)
{
  cout << "To be implemented" << endl;
  return -1;
}

void ZMPVelocityReferencedQP::EndPhaseOfTheWalking(deque<ZMPPosition> &, 
						   deque<COMState> &,
						   deque<FootAbsolutePosition> &,
						   deque<FootAbsolutePosition> &)
{
  cout << "To be implemented" << endl;
}

int ZMPVelocityReferencedQP::ReturnOptimalTimeToRegenerateAStep()
{
  int r = (int)(m_PreviewControlTime/m_SamplingPeriod);
  return 2*r;
}



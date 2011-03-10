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
  m_CurrentSupport.Phase = 0;
  m_CurrentSupport.Foot = 1;
  m_CurrentSupport.TimeLimit = 1000000000;
  m_CurrentSupport.StepsLeft = 1;
  m_CurrentSupport.SSSS = false;
  m_CurrentSupport.StateChanged = false;
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

  initFeet();

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


  m_GenVR = new GeneratorVelRef(lSPM, DataFile, aHS);

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
ZMPVelocityReferencedQP::initFeet()
{

  //Define the initial coordinates of the feet
  //This might be done when creating SupportState
  supportfoot_t aSFLeft;
  supportfoot_t aSFRight;
  aSFLeft.x = 0.0;
  aSFLeft.y = 0.1;
  aSFLeft.theta = 0.0;
  aSFLeft.StartTime = 0.0;
  aSFLeft.SupportFoot = 1;
  aSFRight.x = 0.0;
  aSFRight.y = -0.1;//TODO:
  aSFRight.theta = 0.0;
  aSFRight.StartTime = 0.0;
  aSFRight.SupportFoot = -1;

  QueueOfSupportFeet.push_back(aSFLeft);
  m_Matrices.SupportFoot(aSFLeft);
  QueueOfSupportFeet.push_back(aSFRight);

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
ZMPVelocityReferencedQP::InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
					deque<COMState> & FinalCoMPositions,
					deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
					deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
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

  FinalZMPPositions.resize(AddArraySize);
  FinalCoMPositions.resize(AddArraySize);
  FinalLeftFootAbsolutePositions.resize(AddArraySize);
  FinalRightFootAbsolutePositions.resize(AddArraySize);
  int CurrentZMPindex=0;


  for( unsigned int i=0;i<FinalZMPPositions.size();i++)
    {

      // Smooth ramp
      FinalZMPPositions[CurrentZMPindex].px = lStartingZMPPosition(0);
      FinalZMPPositions[CurrentZMPindex].py = lStartingZMPPosition(1);
      FinalZMPPositions[CurrentZMPindex].pz = lStartingZMPPosition(2);
      FinalZMPPositions[CurrentZMPindex].theta = 0.0;
      FinalZMPPositions[CurrentZMPindex].time = m_CurrentTime;
      FinalZMPPositions[CurrentZMPindex].stepType = 0;

      // Set CoM positions.
      FinalCoMPositions[CurrentZMPindex] = lStartingCOMState;
      // Set Left Foot positions.
      FinalLeftFootAbsolutePositions[CurrentZMPindex] = CurrentLeftFootAbsPos;
      FinalRightFootAbsolutePositions[CurrentZMPindex] = CurrentRightFootAbsPos;

      FinalLeftFootAbsolutePositions[CurrentZMPindex].time =
        FinalRightFootAbsolutePositions[CurrentZMPindex].time = m_CurrentTime;

      FinalLeftFootAbsolutePositions[CurrentZMPindex].stepType =
        FinalRightFootAbsolutePositions[CurrentZMPindex].stepType = 10;



      if(m_FullDebug>0)
	{
	  //Feet coordinates for plot in scilab
	  ofstream aoffeet;
	  aoffeet.open("/tmp/Feet.dat",ios::app);
	  aoffeet<<FinalLeftFootAbsolutePositions[CurrentZMPindex].time<<"    "
		 <<FinalLeftFootAbsolutePositions[CurrentZMPindex].x<<"    "
		 <<FinalLeftFootAbsolutePositions[CurrentZMPindex].y<<"    "
		 <<FinalLeftFootAbsolutePositions[CurrentZMPindex].z<<"    "
		 <<FinalLeftFootAbsolutePositions[CurrentZMPindex].stepType<<"    "
		 <<FinalRightFootAbsolutePositions[CurrentZMPindex].x<<"    "
		 <<FinalRightFootAbsolutePositions[CurrentZMPindex].y<<"    "
		 <<FinalRightFootAbsolutePositions[CurrentZMPindex].z<<"    "
		 <<FinalRightFootAbsolutePositions[CurrentZMPindex].stepType<<"    "<<endl;
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
						    deque<COMState> & FinalCOMStates)
{
  if(m_CurrentSupport.Phase == 1 && time+m_TimeBuffer+3.0/2.0*m_QP_T < m_CurrentSupport.TimeLimit)
    {
      //Set parameters for trunk interpolation
      m_c = 3.0*(m_TrunkStateT.yaw[1]-m_TrunkState.yaw[1])/(m_QP_T*m_QP_T);
      m_d = -2.0*m_c/(3.0*m_QP_T);
      m_a =  m_TrunkState.yaw[1];

      double tT;
      double Theta = m_TrunkState.yaw[0];
      //double dTheta = m_TrunkState.yaw[1];
      //double ddTheta = m_TrunkState.yaw[2];

      FinalCOMStates[CurrentIndex].yaw[0] = m_TrunkState.yaw[0];
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
	  FinalCOMStates[CurrentIndex+k].yaw[0] = m_TrunkState.yaw[0];
	  if(m_FullDebug>2)
	    {
	      ofstream aof;
	      aof.open("/tmp/Trunk.dat",ofstream::app);
	      aof<<time+k*m_SamplingPeriod<<" "<<m_TrunkState.yaw[0]<<" "<<m_TrunkState.yaw[1]<<" "<<m_TrunkState.yaw[2]<<endl;
	      aof.close();
	    }
	}
    }
  else if (m_CurrentSupport.Phase == 0 || time+m_TimeBuffer+3.0/2.0*m_QP_T > m_CurrentSupport.TimeLimit)
    {
      for(int k = 0; k<=(int)(m_QP_T/m_SamplingPeriod);k++)
	{
	  FinalCOMStates[CurrentIndex+k].yaw[0] = m_TrunkState.yaw[0];
	}
    }
  

}


void ZMPVelocityReferencedQP::interpolateFeetPositions(double time, int CurrentIndex,
						       deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
						       deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)
{
  double LocalInterpolationTime = (time+m_TimeBuffer)-(m_CurrentSupport.TimeLimit-m_SupportFSM->m_SSPeriod);

  double StepHeight = 0.05;
  int StepType = 1;

  if(m_CurrentSupport.Phase == 1 && time+m_TimeBuffer+3.0/2.0*m_QP_T < m_CurrentSupport.TimeLimit)
    {
      //determine coefficients of interpolation polynom
      double ModulationSupportCoefficient = 0.9;
      double ModulatedSingleSupportTime = (m_SupportFSM->m_SSPeriod-m_QP_T) * ModulationSupportCoefficient;
      double EndOfLiftOff = ((m_SupportFSM->m_SSPeriod-m_QP_T)-ModulatedSingleSupportTime)*0.5;
      double InterpolationTimePassed = 0.0;
      if(LocalInterpolationTime>EndOfLiftOff)
	InterpolationTimePassed = LocalInterpolationTime-EndOfLiftOff;

      FootAbsolutePosition LastSwingFootPosition;

      if(m_CurrentSupport.Foot==1)
	{
	  LastSwingFootPosition = FinalRightFootAbsolutePositions[CurrentIndex];
	}
      else
	{
	  LastSwingFootPosition = FinalLeftFootAbsolutePositions[CurrentIndex];
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

      if(m_CurrentSupport.StateChanged==true)
	m_FTGS->SetParameters(FootTrajectoryGenerationStandard::Z_AXIS, m_SupportFSM->m_SSPeriod-m_QP_T,StepHeight);

      m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::THETA_AXIS,
						ModulatedSingleSupportTime-InterpolationTimePassed,
						m_PreviewedSupportAngles[0]*180.0/M_PI,
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
	  if (m_CurrentSupport.Foot==1)
	    {
	      m_FTGS->UpdateFootPosition(FinalLeftFootAbsolutePositions,
					 FinalRightFootAbsolutePositions,
					 CurrentIndex,k,
					 LocalInterpolationTime,
					 ModulatedSingleSupportTime,
					 StepType, -1);
	    }
	  else
	    {
	      m_FTGS->UpdateFootPosition(FinalRightFootAbsolutePositions,
					 FinalLeftFootAbsolutePositions,
					 CurrentIndex,k,
					 LocalInterpolationTime,
					 ModulatedSingleSupportTime,
					 StepType, 1);
	    }
	  FinalLeftFootAbsolutePositions[CurrentIndex+k].time =
	    FinalRightFootAbsolutePositions[CurrentIndex+k].time = time+m_TimeBuffer+k*m_SamplingPeriod;


	  if(m_FullDebug>0)
	    {
	      ofstream aoffeet;
	      aoffeet.open("/tmp/Feet.dat",ios::app);
	      aoffeet<<time+m_TimeBuffer+k*m_SamplingPeriod<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].x<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].y<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].z<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].stepType<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].x<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].y<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].z<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].stepType<<"    "
		     <<endl;
	      aoffeet.close();
	    }

	}
    }
  else if (m_CurrentSupport.Phase == 0 || time+m_TimeBuffer+3.0/2.0*m_QP_T > m_CurrentSupport.TimeLimit)
    {
      for(int k = 0; k<=(int)(m_QP_T/m_SamplingPeriod);k++)
	{
	  FinalRightFootAbsolutePositions[CurrentIndex+k]=FinalRightFootAbsolutePositions[CurrentIndex+k-1];
	  FinalLeftFootAbsolutePositions[CurrentIndex+k]=FinalLeftFootAbsolutePositions[CurrentIndex+k-1];
	  FinalLeftFootAbsolutePositions[CurrentIndex+k].time =
	    FinalRightFootAbsolutePositions[CurrentIndex+k].time = time+m_TimeBuffer+k*m_SamplingPeriod;
	  FinalLeftFootAbsolutePositions[CurrentIndex+k].stepType =
	    FinalRightFootAbsolutePositions[CurrentIndex+k].stepType = 10;

	  if(m_FullDebug>0)
	    {
	      ofstream aoffeet;
	      aoffeet.open("/tmp/Feet.dat",ios::app);
	      aoffeet<<time+m_TimeBuffer+k*m_SamplingPeriod<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].x<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].y<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].z<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].stepType<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].x<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].y<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].z<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].stepType<<"    "
		     <<endl;
	      aoffeet.close();
	    }
	}
    }
}


void ZMPVelocityReferencedQP::OnLine(double time,
				     deque<ZMPPosition> & FinalZMPPositions,
				     deque<COMState> & FinalCOMStates,
				     deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
				     deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)
{


  // If on-line mode not activated we go out.
  if (!m_OnLineMode)
    { return; }

  // Testing if we are reaching the end of the online mode.
  if ((m_EndingPhase) &&
      (time>=m_TimeToStopOnLineMode))
    { m_OnLineMode = false; }

  if(time + 0.00001 > m_UpperTimeLimitToUpdate)
    {
      double TotalAmountOfCPUTime=0.0,CurrentCPUTime=0.0;
      struct timeval start,end;
      gettimeofday(&start,0);

      m_OP->verifyAccelerationOfHipJoint(m_VelRef, m_TrunkState,
					 m_TrunkStateT, m_CurrentSupport);
      m_OP->previewOrientations(time+m_TimeBuffer,
				m_PreviewedSupportAngles,
				m_TrunkState,
				m_TrunkStateT,
				m_SupportFSM, m_CurrentSupport,
				FinalLeftFootAbsolutePositions,
				FinalRightFootAbsolutePositions);

      m_GenVR->computeGlobalReference(m_Matrices, m_TrunkStateT);

      //Apply external forces
      if(m_PerturbationOccured == true)
	{
          com_t com = m_CoM();
	  com.x(2) = com.x(2)+m_PerturbationAcceleration(2);
	  com.y(2) = com.y(2)+m_PerturbationAcceleration(5);
	  m_PerturbationOccured = false;
	  m_CoM(com);
	}

      m_SupportFSM->setSupportState(time+m_TimeBuffer, 0, m_CurrentSupport, m_VelRef);

      //Add a new support foot to the support feet history deque
      if(m_CurrentSupport.StateChanged == true)
	{
	  FootAbsolutePosition FAP;
	  supportfoot_t newSF;
	  if(m_CurrentSupport.Foot==1)
	    FAP = FinalLeftFootAbsolutePositions.back();
	  else
	    FAP = FinalRightFootAbsolutePositions.back();

	  newSF.x = FAP.x;
	  newSF.y = FAP.y;
	  newSF.theta = FAP.theta*M_PI/180.0;
	  newSF.StartTime = time+m_TimeBuffer;
	  newSF.SupportFoot = m_CurrentSupport.Foot;

	  QueueOfSupportFeet.push_back(newSF);

	  m_Matrices.SupportFoot(newSF);
	}

      deque<support_state_t> deqPrwSupportStates;
      deqPrwSupportStates.push_back(m_CurrentSupport);

      m_GenVR->setCurrentTime(time+m_TimeBuffer);
      m_Matrices.Reference(m_VelRef);

      m_GenVR->computeGlobalReference(m_Matrices, m_TrunkStateT);
      m_GenVR->previewSupportStates(m_Matrices, m_SupportFSM, deqPrwSupportStates);

      m_PrwSupport = deqPrwSupportStates.back();



      int NbVariables = 2*(m_QP_N + m_PrwSupport.StepNumber);
      int NbOfConstraints = 4*m_QP_N + 5*m_PrwSupport.StepNumber;
      m_Pb.setDimensions( NbVariables, NbOfConstraints, 0 );

      m_GenVR->buildInvariantPart(m_Pb, m_Matrices);

      m_GenVR->updateProblem(m_Pb, m_Matrices, deqPrwSupportStates);

      m_GenVR->buildConstraints(m_Matrices, m_Pb,
          m_fCALS,
          FinalLeftFootAbsolutePositions,
          FinalRightFootAbsolutePositions,
          deqPrwSupportStates,
          m_PreviewedSupportAngles);



      if ((m_FastFormulationMode==QLDANDLQ)||
	  (m_FastFormulationMode==QLD))
	{

	  m_Pb.solve( QPProblem_s::QLD , m_Result );

	}


      FinalCOMStates.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      FinalZMPPositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      FinalLeftFootAbsolutePositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      FinalRightFootAbsolutePositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));

      int CurrentIndex = (int)(m_TimeBuffer/m_SamplingPeriod)-1;

      m_CoM.Interpolation(FinalCOMStates,
			      FinalZMPPositions,
			      CurrentIndex,
			      m_Result.vecSolution[0],m_Result.vecSolution[m_QP_N]);
      m_Matrices.CoM(m_CoM.OneIteration(m_Result.vecSolution[0],m_Result.vecSolution[m_QP_N]));


      //The robot is supposed to stop always with the feet aligned in the lateral plane.
      if(m_CurrentSupport.StepsLeft>0)
	{
	  if(fabs(m_Result.vecSolution[2*m_QP_N])-0.00001<0.0)
	    {
	      cout<<"Previewed foot position zero at time: "<<time<<endl;
	    }
	  else if (m_CurrentSupport.TimeLimit-time-m_QP_T/2.0>0)
	    {//The landing position is yet determined by the solver because the robot finds himself still in the single support phase
	      m_FPx = m_Result.vecSolution[2*m_QP_N];
	      m_FPy = m_Result.vecSolution[2*m_QP_N+m_PrwSupport.StepNumber];
	    }
	}
      else
	{//The solver isn't responsible for the feet positions anymore
	  deque<supportfoot_t>::iterator CurSF_it;
	  CurSF_it = QueueOfSupportFeet.end();
	  CurSF_it--;
	  while(CurSF_it->SupportFoot!=m_CurrentSupport.Foot)
	    CurSF_it--;
	  m_FPx = CurSF_it->x + double(CurSF_it->SupportFoot)*sin(CurSF_it->theta)*m_FeetDistanceDS;
	  m_FPy = CurSF_it->y - double(CurSF_it->SupportFoot)*cos(CurSF_it->theta)*m_FeetDistanceDS;

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
			    FinalCOMStates);
      interpolateFeetPositions(time, CurrentIndex,
			       FinalLeftFootAbsolutePositions,
			       FinalRightFootAbsolutePositions);








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



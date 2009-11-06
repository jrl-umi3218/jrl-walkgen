/*! \file DoubleStagePreviewControlSrategy.h
  \brief This object defines a global strategy object to generate 
  full body position every 5 ms over a preview window. 
  It implements Kajita's algorithm presented in \ref Kajita2003
  
  
  Copyright (c) 2007, 
  @author Francois Keith, Olivier Stasse
   
  JRL-Japan, CNRS/AIST
  
  All rights reserved.

  Please look at License.txt for details on the license.
  
*/
#include <iostream>
#include <fstream>
using namespace std;

#include <Debug.h>
#include <GlobalStrategyManagers/DoubleStagePreviewControlStrategy.h>

using namespace PatternGeneratorJRL;

DoubleStagePreviewControlStrategy::DoubleStagePreviewControlStrategy(SimplePluginManager * aSPM)
  :GlobalStrategyManager(aSPM)
{
  m_ZMPFrame = ZMPFRAME_WAIST;

  // The object to realize the global stage of preview control.
  m_ZMPpcwmbz = new ZMPPreviewControlWithMultiBodyZMP(aSPM);

  unsigned int NbOfMethods=4;
  std::string aMethodName[4] = 
    {":SetAlgoForZmpTrajectory",
     ":SetZMPFrame",
     ":samplingperiod",
     ":previewcontroltime"};
  
  for(unsigned int i=0;i<NbOfMethods;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr<< "Unable to register " << aMethodName[i]  << endl;
	}
    }
  
  RESETDEBUG4("ZMPRefAndWaist.dat");
  RESETDEBUG4("ZMPRef.dat");
}

DoubleStagePreviewControlStrategy::~DoubleStagePreviewControlStrategy()
{
  if (m_ZMPpcwmbz!=0)
    delete m_ZMPpcwmbz;
}


int DoubleStagePreviewControlStrategy::InitInterObjects(PreviewControl * aPC,
							CjrlHumanoidDynamicRobot *aHDR,
							ComAndFootRealization * aCFR,
							StepStackHandler * aSSH)
{
  m_ZMPpcwmbz->SetPreviewControl(aPC);
  m_PC = aPC;
  m_SamplingPeriod = m_PC->SamplingPeriod();
  m_PreviewControlTime = m_PC->PreviewControlTime();
  m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);
  setHumanoidDynamicRobot(aHDR);
  m_ZMPpcwmbz->setHumanoidDynamicRobot(m_HumanoidDynamicRobot);
  m_ZMPpcwmbz->setComAndFootRealization(aCFR);
  m_StepStackHandler = aSSH;
  return 1;
}

int DoubleStagePreviewControlStrategy::OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
							      FootAbsolutePosition &RightFootPosition,
							      MAL_VECTOR(,double) & ZMPRefPos,
							      COMPosition & finalCOMPosition,
							      MAL_VECTOR(,double) & CurrentConfiguration,
							      MAL_VECTOR(,double) & CurrentVelocity,
							      MAL_VECTOR(,double) & CurrentAcceleration)
{
  // New scheme:
  // Update the queue of ZMP ref
  m_ZMPpcwmbz->UpdateTheZMPRefQueue((*m_ZMPPositions)[2*m_NL]);
  if ((m_StepStackHandler->GetWalkMode()==0) ||
      (m_StepStackHandler->GetWalkMode()==4))
    {
      (*m_COMBuffer)[m_NL].yaw = (*m_ZMPPositions)[m_NL].theta;
    }

  //    COMPositionFromPC1 = m_COMBuffer[m_NL];
  finalCOMPosition =  (*m_COMBuffer)[m_NL];
  
  ODEBUG("ZMP : " << (*m_ZMPPositions)[0].px
	 << " " << (*m_ZMPPositions)[0].py
	 << " " << (*m_ZMPPositions)[2*m_NL].px
	 << " " << (*m_ZMPPositions)[2*m_NL].py );
  ODEBUG4( (*m_ZMPPositions)[0].px << " " << 
	   (*m_ZMPPositions)[0].py << " " <<
	   (*m_ZMPPositions)[0].pz,"ZMPRef.dat");

  ODEBUG(m_count << " before-CurrentConfiguration " << CurrentConfiguration);
  
  m_ZMPpcwmbz->OneGlobalStepOfControl((*m_LeftFootPositions)[2*m_NL],
				      (*m_RightFootPositions)[2*m_NL],
				      (*m_ZMPPositions)[2*m_NL],
				      finalCOMPosition,
				      CurrentConfiguration,
				      CurrentVelocity,
				      CurrentAcceleration);
  ODEBUG4("finalCOMPosition:" <<finalCOMPosition.x[0] << " " 
	  << finalCOMPosition.y[0] ,"DebugData.txt");
  
  (*m_COMBuffer)[0] = finalCOMPosition;
  
  LeftFootPosition = (*m_LeftFootPositions)[0];
  RightFootPosition = (*m_RightFootPositions)[0];
    
  // Compute the waist position in the current motion global reference frame.
  MAL_S4x4_MATRIX( PosOfWaistInCOMF,double);
  PosOfWaistInCOMF = m_ZMPpcwmbz->GetCurrentPositionofWaistInCOMFrame();
  
  COMPosition outWaistPosition;
  outWaistPosition = finalCOMPosition;
  outWaistPosition.x[0] =  CurrentConfiguration(0);
  outWaistPosition.y[0] =  CurrentConfiguration(1);
  outWaistPosition.z[0] =  CurrentConfiguration(2);
  
  // In case we are at the end of the motion
  double CurrentZMPNeutralPosition[2];
  CurrentZMPNeutralPosition[0] = (*m_ZMPPositions)[0].px;
  CurrentZMPNeutralPosition[1] = (*m_ZMPPositions)[0].py;
  
  double temp1;
  double temp2;
  double temp3;
  
  if (m_ZMPFrame==ZMPFRAME_WAIST)
    {
      temp1 = (*m_ZMPPositions)[0].px - outWaistPosition.x[0];
      temp2 = (*m_ZMPPositions)[0].py - outWaistPosition.y[0];
      temp3 = finalCOMPosition.yaw*M_PI/180.0;
      
      ZMPRefPos(0) = cos(temp3)*temp1+sin(temp3)*temp2 ;
      ZMPRefPos(1) = -sin(temp3)*temp1+cos(temp3)*temp2;
      ZMPRefPos(2) = -finalCOMPosition.z[0] - MAL_S4x4_MATRIX_ACCESS_I_J(PosOfWaistInCOMF, 2,3) - (*m_ZMPPositions)[0].pz;
    }
  else if (m_ZMPFrame==ZMPFRAME_WORLD)
    {
      temp1 = (*m_ZMPPositions)[0].px ;
      temp2 = (*m_ZMPPositions)[0].py ;
      temp3 = finalCOMPosition.yaw*M_PI/180.0;

      ZMPRefPos(0) = cos(temp3)*temp1+sin(temp3)*temp2 ;
      ZMPRefPos(1) = -sin(temp3)*temp1+cos(temp3)*temp2;
      ZMPRefPos(2) = 0.0;
	      
    }
  else 
    { ODEBUG3("Problem with the ZMP reference frame set to 0."); }

  ODEBUG4SIMPLE((*m_ZMPPositions)[0].px <<  " " << 
		(*m_ZMPPositions)[0].py << " " <<
		outWaistPosition.x[0] << " " <<
		outWaistPosition.y[0] << " " <<
		ZMPRefPos(0) << " " << ZMPRefPos(1) << " " << ZMPRefPos(2) <<
		LeftFootPosition.stepType << " " <<
		RightFootPosition.stepType, 
		"ZMPRefAndWaist.dat");

  m_ZMPPositions->pop_front();
  m_COMBuffer->pop_front();
  m_LeftFootPositions->pop_front();
  m_RightFootPositions->pop_front();
  
  m_CurrentWaistState = outWaistPosition;

  return 0;
}

int DoubleStagePreviewControlStrategy::EvaluateStartingState(MAL_VECTOR( &,double) BodyAngles,
							     COMPosition & aStartingCOMPosition,
							     MAL_S3_VECTOR(&,double) aStartingZMPPosition,
							     MAL_VECTOR(&,double) aStartingWaistPose,
							     FootAbsolutePosition & InitLeftFootPosition,
							     FootAbsolutePosition & InitRightFootPosition)
{
  MAL_S3_VECTOR(lStartingCOMPosition,double);
  lStartingCOMPosition(0) = aStartingCOMPosition.x[0];
  lStartingCOMPosition(1) = aStartingCOMPosition.y[0];
  lStartingCOMPosition(2) = aStartingCOMPosition.z[0];
  
  m_ZMPpcwmbz->EvaluateStartingState(BodyAngles,
				     lStartingCOMPosition,
				     aStartingZMPPosition,
				     aStartingWaistPose,
				     InitLeftFootPosition,InitRightFootPosition);

  aStartingCOMPosition.x[0] = lStartingCOMPosition(0);
  aStartingCOMPosition.y[0] = lStartingCOMPosition(1);
  aStartingCOMPosition.z[0] = lStartingCOMPosition(2);
  return 0;
}

int DoubleStagePreviewControlStrategy::EndOfMotion()
{
  ODEBUG("m_ZMPPositions->size()  2*m_NL+1 2*m_NL " 
	 << m_ZMPPositions->size() << " "
	 << 2*m_NL+1 << " " 
	 << 2*m_NL << " " );
  if (m_ZMPPositions->size()== 2*m_NL)
    return 0;
  else if (m_ZMPPositions->size()< 2*m_NL+1)
    return -1;

  return 1;

}

void DoubleStagePreviewControlStrategy::SetAlgoForZMPTraj(istringstream &strm)
{
  string ZMPTrajAlgo;
  strm >> ZMPTrajAlgo;
  if (ZMPTrajAlgo=="PBW")
    {
      m_ZMPpcwmbz->SetStrategyForStageActivation(ZMPPreviewControlWithMultiBodyZMP::
						 ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY);
    }
  else if (ZMPTrajAlgo=="Kajita")
    {
      m_ZMPpcwmbz->SetStrategyForStageActivation(ZMPPreviewControlWithMultiBodyZMP::
						 ZMPCOM_TRAJECTORY_FULL);
    }
  else if (ZMPTrajAlgo=="KajitaOneStage")
    {
      m_ZMPpcwmbz->SetStrategyForStageActivation(ZMPPreviewControlWithMultiBodyZMP::
						 ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY);
    }
  else if (ZMPTrajAlgo=="Morisawa")
    {
      ODEBUG("Wrong Global Strategy");
    }
  
}

void DoubleStagePreviewControlStrategy::SetZMPFrame(std::istringstream &astrm)
{
  string aZMPFrame;
  astrm >> aZMPFrame;

  if (aZMPFrame=="waist")
    { 
      m_ZMPFrame = ZMPFRAME_WAIST; 
    }
  else if (aZMPFrame=="world")
    { m_ZMPFrame = ZMPFRAME_WORLD; 
    }
  else
    {ODEBUG3("Mistake wrong keyword" << aZMPFrame);}
}

void DoubleStagePreviewControlStrategy::SetSamplingPeriod(double lSamplingPeriod)
{
  m_SamplingPeriod = lSamplingPeriod;

  m_NL=0.0;
  if (m_SamplingPeriod!=0.0)
    m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);
  
}

void DoubleStagePreviewControlStrategy::SetPreviewControlTime(double lPreviewControlTime)
{
  m_PreviewControlTime = lPreviewControlTime;

  m_NL=0.0;
  if (m_SamplingPeriod!=0.0)
    m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);

}

void DoubleStagePreviewControlStrategy::CallMethod(std::string &Method, std::istringstream &astrm)
{
  ODEBUG("Method: " << Method);

  if (Method==":SetAlgoForZmpTrajectory")
    {
      SetAlgoForZMPTraj(astrm);
    }
  else if (Method==":SetZMPFrame")
    {
      SetZMPFrame(astrm);
    }
  if (Method==":samplingperiod")
    {
      std::string aws;
      if (astrm.good())
	{
	  double lSamplingPeriod;
	  astrm >> lSamplingPeriod;
	  SetSamplingPeriod(lSamplingPeriod);
	}
    }
  else if (Method==":previewcontroltime")
    {
      std::string aws;
      if (astrm.good())
	{
	  double lpreviewcontroltime;
	  astrm >> lpreviewcontroltime;
	  SetPreviewControlTime(lpreviewcontroltime);
	}
    }
}

void DoubleStagePreviewControlStrategy::Setup(deque<ZMPPosition> & aZMPPositions,
					      deque<COMPosition> & aCOMBuffer,
					      deque<FootAbsolutePosition> & aLeftFootAbsolutePositions,
					      deque<FootAbsolutePosition> & aRightFootAbsolutePositions)
{
  m_ZMPpcwmbz->Setup(aZMPPositions,
		     aCOMBuffer,
		     aLeftFootAbsolutePositions,
		     aRightFootAbsolutePositions);
}

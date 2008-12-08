/*! \file DoubleStagePreviewControlSrategy.h
  \brief This object defines a global strategy object to generate 
  full body position every 5 ms over a preview window. 
  It implements Kajita's algorithm presented in \ref Kajita2003
  
  
  Copyright (c) 2007, 
  @author Francois Keith, Olivier Stasse
   
  JRL-Japan, CNRS/AIST
  
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:
  
  * Redistributions of source code must retain the above copyright notice, 
  this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice, 
  this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
  * Neither the name of the CNRS/AIST nor the names of its contributors 
  may be used to endorse or promote products derived from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
  OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <iostream>
#include <fstream>
#include "jrlMathTools/jrlConstants.h"
using namespace std;

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); \
    DebugFile << "DoubleStagePreviewControlStrategy: " << x << endl; \
    DebugFile.close();}
#else
#define RESETDEBUG4(y)
#define ODEBUG4(x,y)
#endif

#define RESETDEBUG6(y)
#define ODEBUG6(x,y)

#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "DoubleStagePreviewControlStrategy: " << x << endl; DebugFile.close();}

#if 1
#define ODEBUG(x)
#else
#define ODEBUG(x)  std::cout << x << endl;
#endif

#define ODEBUG3(x)  std::cout << x << endl;

#include <walkGenJrl/GlobalStrategyManagers/DoubleStagePreviewControlStrategy.h>

using namespace PatternGeneratorJRL;

DoubleStagePreviewControlStrategy::DoubleStagePreviewControlStrategy(SimplePluginManager * aSPM)
  :GlobalStrategyManager(aSPM)
{
  // The object to realize the global stage of preview control.
  m_ZMPpcwmbz = new ZMPPreviewControlWithMultiBodyZMP();

  unsigned int NbOfMethods=1;
  std::string aMethodName[1] = 
    {":SetAlgoForZmpTrajectory"};
  
  for(unsigned int i=0;i<NbOfMethods;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr<< "Unable to register " << aMethodName[i]  << endl;
	}
    }
  
  RESETDEBUG4("ZMPRefAndWaist.dat");
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
  
  ODEBUG(m_count << " before-CurrentConfiguration " << CurrentConfiguration);
  
  m_ZMPpcwmbz->OneGlobalStepOfControl((*m_LeftFootPositions)[m_NL],
				      (*m_RightFootPositions)[m_NL],
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
  //     MAL_S4x4_MATRIX( FinalDesiredCOMPose,double);
  //     FinalDesiredCOMPose = m_ZMPpcwmbz->GetFinalDesiredCOMPose();
  //     ODEBUG3("FinalDesiredCOMPose :" << FinalDesiredCOMPose);
  MAL_S4x4_MATRIX( PosOfWaistInCOMF,double);
  PosOfWaistInCOMF = m_ZMPpcwmbz->GetCurrentPositionofWaistInCOMFrame();
  // MAL_S4x4_MATRIX( AbsWaist,double);
  //     MAL_S4x4_C_eq_A_by_B(AbsWaist ,FinalDesiredCOMPose , PosOfWaistInCOMF);
  //     ODEBUG3("AbsWaist " << AbsWaist);
  //     ODEBUG3("PosOfWaistInCOMF " << PosOfWaistInCOMF);
  //     ODEBUG3("Configuration(0-2): " <<
  //  	    CurrentConfiguration(0) << " " <<
  //  	    CurrentConfiguration(1) << " " <<
  //  	    CurrentConfiguration(2));
  
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

  temp1 = (*m_ZMPPositions)[0].px - outWaistPosition.x[0];
  temp2 = (*m_ZMPPositions)[0].py - outWaistPosition.y[0];
  temp3 = finalCOMPosition.yaw*M_PI/180.0;
  
  ZMPRefPos(0) = cos(temp3)*temp1+sin(temp3)*temp2 ;
  ZMPRefPos(1) = -sin(temp3)*temp1+cos(temp3)*temp2;
  ZMPRefPos(2) = -finalCOMPosition.z[0] - MAL_S4x4_MATRIX_ACCESS_I_J(PosOfWaistInCOMF, 2,3);


  ODEBUG4((*m_ZMPPositions)[0].px <<  " " << 
	  (*m_ZMPPositions)[0].py << " " <<
	  outWaistPosition.x[0] << " " <<
	  outWaistPosition.y[0] << " " <<
	  ZMPRefPos(0) << " " << ZMPRefPos(1) << " " << ZMPRefPos(2), "ZMPRefAndWaist.dat");

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
    else if (ZMPTrajAlgo=="Morisawa")
      {
	ODEBUG("Wrong Global Strategy");
      }

  }


void DoubleStagePreviewControlStrategy::CallMethod(std::string &Method, std::istringstream &astrm)
{
  ODEBUG("Method: " << Method);

  if (Method==":SetAlgoForZmpTrajectory")
    {
      SetAlgoForZMPTraj(astrm);
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

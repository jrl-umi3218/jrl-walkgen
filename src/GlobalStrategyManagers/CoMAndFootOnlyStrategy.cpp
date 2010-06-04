/*! \file CoMAndFootOnlyStrategy.h
  \brief This object defines a global strategy object to generate 
  only foot, ZMP reference and CoM trajectories position every 5 ms.

  Copyright (c) 2007-2009, 
  @author Olivier Stasse
   
  JRL-Japan, CNRS/AIST
  
  All rights reserved.

   Please see License.txt for further information on license.     
*/

#include <Debug.h>
#include <GlobalStrategyManagers/CoMAndFootOnlyStrategy.h>

using namespace PatternGeneratorJRL;

CoMAndFootOnlyStrategy::CoMAndFootOnlyStrategy(SimplePluginManager * aSimplePluginManager)
  : GlobalStrategyManager(aSimplePluginManager)
{
  
}

CoMAndFootOnlyStrategy::~CoMAndFootOnlyStrategy()
{
}

int CoMAndFootOnlyStrategy::InitInterObjects(CjrlHumanoidDynamicRobot *aHDR,
					     ComAndFootRealization * aCFR,
					     StepStackHandler * aSSH)
{
  m_ComAndFootRealization = aCFR;
  return 0;
}

int CoMAndFootOnlyStrategy::OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
						   FootAbsolutePosition &RightFootPosition,
						   MAL_VECTOR(,double) & ZMPRefPos,
						   COMPosition & finalCOMPosition,
						   MAL_VECTOR(,double) & CurrentConfiguration,
						   MAL_VECTOR(,double) & CurrentVelocity,
						   MAL_VECTOR(,double) & CurrentAcceleration)
{
  ODEBUG("Begin OneGlobalStepOfControl " 
	  << m_LeftFootPositions->size() << " " 
	  << m_RightFootPositions->size() << " "
	  << m_COMBuffer->size() << " "
	  << m_ZMPPositions->size());
  
  /* The strategy of this class is simply to pull off values from the buffers. */
  if (m_LeftFootPositions->size()>0)
    {
      LeftFootPosition = (*m_LeftFootPositions)[0];
      m_LeftFootPositions->pop_front();
    }
  else
    {
      ODEBUG3("Problem on the left foot position queue: empty");
      return -2;
    }

  if (m_RightFootPositions->size()>0)
    {
      RightFootPosition = (*m_RightFootPositions)[0];
      m_RightFootPositions->pop_front();
    }
  else
    {
      ODEBUG3("Problem on the right foot position queue: empty");
      return -3;
    }
      
  if (m_COMBuffer->size()>0)
    {
      finalCOMPosition = (*m_COMBuffer)[0];
      m_COMBuffer->pop_front();
    }
  else
    {
      ODEBUG("Problem on the COM queue: empty");
      return -4;
    }

  if(m_ZMPPositions->size()>0)
    {
      ZMPPosition aZMPPosition = (*m_ZMPPositions)[0];
      ZMPRefPos(0) = aZMPPosition.px;
      ZMPRefPos(1) = aZMPPosition.py;
      ZMPRefPos(2) = 0;
      m_ZMPPositions->pop_front();
    }
  else
    {
      ODEBUG("Problem on the ZMP size: empty");
      return -5;
    }

  ODEBUG("End of OneGlobalStepOfControl"
	  << m_LeftFootPositions->size() << " " 
	  << m_RightFootPositions->size() << " "
	  << m_COMBuffer->size() << " "
	  << m_ZMPPositions->size());
  return 0;
}


int CoMAndFootOnlyStrategy::EvaluateStartingState(MAL_VECTOR(&,double) BodyAngles,
						  COMPosition & aStartingCOMPosition,
						  MAL_S3_VECTOR(&,double) aStartingZMPPosition,
						  MAL_VECTOR(&,double) lStartingWaistPose,
						  FootAbsolutePosition & InitLeftFootPosition,
						  FootAbsolutePosition & InitRightFootPosition)
{
  MAL_S3_VECTOR(lStartingCOMPosition,double);

  lStartingCOMPosition(0) = aStartingCOMPosition.x[0];
  lStartingCOMPosition(1) = aStartingCOMPosition.y[0];
  lStartingCOMPosition(2) = aStartingCOMPosition.z[0];

  m_ComAndFootRealization->InitializationCoM(BodyAngles,lStartingCOMPosition,
					     lStartingWaistPose,
					     InitLeftFootPosition, InitRightFootPosition);  

  ODEBUG("EvaluateStartingCOM: m_StartingCOMPosition: " << lStartingCOMPosition);
  aStartingCOMPosition.x[0] = lStartingCOMPosition(0);
  aStartingCOMPosition.y[0] = lStartingCOMPosition(1);
  aStartingCOMPosition.z[0] = lStartingCOMPosition(2);
  aStartingZMPPosition= m_ComAndFootRealization->GetCOGInitialAnkles();

  //  cerr << "YOU SHOULD INITIALIZE PROPERLY aStartingZMPosition in   CoMAndFootOnlyStrategy::EvaluateStartingState" <<endl;
  return 0;
}

int CoMAndFootOnlyStrategy::EndOfMotion()
{
  // Just testing one buffer.
  // They suppose to have the same behavior.

  if (m_LeftFootPositions->size()>m_BufferSizeLimit)
    {
      if (m_LeftFootPositions->size()==m_BufferSizeLimit+1)
	{
	  ODEBUG("LeftFootPositions position ( "<< (*m_LeftFootPositions)[0].x 
		 << " , " << (*m_LeftFootPositions)[0].y << " ) " );
	}

      m_NbOfHitBottom=0;
      return 1;
    }
  else if ((m_LeftFootPositions->size()==m_BufferSizeLimit) &&
	   (m_NbOfHitBottom==0))
    {
      ODEBUG("LeftFootPositions size : "<< m_LeftFootPositions->size() 
	     << "Buffer size limit: " << m_BufferSizeLimit);

      m_NbOfHitBottom++;
      return 0;
    }
  else if ((m_LeftFootPositions->size()==m_BufferSizeLimit) &&
	   (m_NbOfHitBottom>0))
    {
      return -1;
    }
  return 0;
}

void CoMAndFootOnlyStrategy::Setup(deque<ZMPPosition> &aZMPPositions,
				   deque<COMPosition> &aCOMBuffer,
				   deque<FootAbsolutePosition> & aLeftFootAbsolutePositions,
				   deque<FootAbsolutePosition> & aRightFootAbsolutePositions)
{
}

void CoMAndFootOnlyStrategy::CallMethod(std::string &Method, std::istringstream &astrm)
{
}

void CoMAndFootOnlyStrategy::SetTheLimitOfTheBuffer(unsigned int lBufferSizeLimit)
{
  m_BufferSizeLimit = lBufferSizeLimit;
}

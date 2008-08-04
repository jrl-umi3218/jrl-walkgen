/*! \file CoMAndFootOnlyStrategy.h
  \brief This object defines a global strategy object to generate 
  only foot, ZMP reference and CoM trajectories position every 5 ms.

  Copyright (c) 2007, 
  @author Olivier Stasse
   
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
#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); \
    DebugFile << "CoMAndFootOnlyStrategy: " << x << endl; \
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

#define ODEBUG3(x)  std::cout << "CoMAndFootOnlyStrategy:" << x << endl;

#include <walkGenJrl/GlobalStrategyManagers/CoMAndFootOnlyStrategy.h>

using namespace PatternGeneratorJRL;

CoMAndFootOnlyStrategy::CoMAndFootOnlyStrategy(SimplePluginManager * aSimplePluginManager)
  : GlobalStrategyManager(aSimplePluginManager)
{
  
}

CoMAndFootOnlyStrategy::~CoMAndFootOnlyStrategy()
{
}

int CoMAndFootOnlyStrategy::InitInterObjects(PreviewControl * aPC,
							CjrlHumanoidDynamicRobot *aHDR,
							ComAndFootRealization * aCFR,
							StepStackHandler * aSSH)
{
  m_ComAndFootRealization = aCFR;
}

int CoMAndFootOnlyStrategy::OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
						   FootAbsolutePosition &RightFootPosition,
						   MAL_VECTOR(,double) & ZMPRefPos,
						   COMPosition & finalCOMPosition,
						   MAL_VECTOR(,double) & CurrentConfiguration,
						   MAL_VECTOR(,double) & CurrentVelocity)
{
  ODEBUG("Begin OneGlobalStepOfControl");
  
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
      
      m_ZMPPositions->pop_front();
    }
  else
    {
      ODEBUG("Problem on the ZMP size: empty");
      return -5;
    }

  ODEBUG("End of OneGlobalStepOfControl");
}


int CoMAndFootOnlyStrategy::EvaluateStartingState(MAL_VECTOR(&,double) BodyAngles,
						  COMPosition & aStartingCOMPosition,
						  FootAbsolutePosition & InitLeftFootPosition,
						  FootAbsolutePosition & InitRightFootPosition)
{
  MAL_S3_VECTOR(lStartingCOMPosition,double);

  lStartingCOMPosition(0) = aStartingCOMPosition.x[0];
  lStartingCOMPosition(1) = aStartingCOMPosition.y[0];
  lStartingCOMPosition(2) = aStartingCOMPosition.z[0];

  m_ComAndFootRealization->InitializationCoM(BodyAngles,lStartingCOMPosition,
					     InitLeftFootPosition, InitRightFootPosition);  

  ODEBUG("EvaluateStartingCOM: m_StartingCOMPosition: " << lStartingCOMPosition);
  aStartingCOMPosition.x[0] = lStartingCOMPosition(0);
  aStartingCOMPosition.y[0] = lStartingCOMPosition(1);
  aStartingCOMPosition.z[0] = lStartingCOMPosition(2);
  
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

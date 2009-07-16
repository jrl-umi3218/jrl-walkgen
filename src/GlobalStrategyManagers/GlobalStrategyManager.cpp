/*! \file GlobalStrategyManager.cpp
  \brief This object defines a global strategy abstract object to generate an output
  handled by the PatternGeneratorInterface object.
  
  
  Copyright (c) 2007, 
  @author Olivier Stasse
   
  JRL-Japan, CNRS/AIST
  
  All rights reserved.
 
  Please see License.txt for further information on license. 
*/

#include <Debug.h>
#include <deque> 
#include <GlobalStrategyManagers/GlobalStrategyManager.h>

using namespace PatternGeneratorJRL;

GlobalStrategyManager::GlobalStrategyManager(SimplePluginManager *aPluginManager):
  SimplePlugin(aPluginManager)
{
}

  
  
void GlobalStrategyManager::SetBufferPositions(deque<ZMPPosition> * aZMPPositions,
					       deque<COMPosition> * aCOMBuffer,
					       deque<FootAbsolutePosition> *aLeftFootAbsolutePositions,
					       deque<FootAbsolutePosition> *aRightFootAbsolutePositions )
{
  m_ZMPPositions = aZMPPositions;
  m_COMBuffer = aCOMBuffer;
  m_LeftFootPositions = aLeftFootAbsolutePositions;
  m_RightFootPositions = aRightFootAbsolutePositions;
}

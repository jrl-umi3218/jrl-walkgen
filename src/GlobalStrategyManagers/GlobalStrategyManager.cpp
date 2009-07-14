/*! \file GlobalStrategyManager.cpp
  \brief This object defines a global strategy abstract object to generate an output
  handled by the PatternGeneratorInterface object.
  
  
  Copyright (c) 2007, 
  @author Olivier Stasse
   
  JRL-Japan, CNRS/AIST
  
  All rights reserved.
 
  Please see License.txt for further information on license. 
*/

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); \
    DebugFile << "GlobalStrategyManager: " << x << endl; \
    DebugFile.close();}
#else
#define RESETDEBUG4(y)
#define ODEBUG4(x,y)
#endif

#define RESETDEBUG6(y)
#define ODEBUG6(x,y)

#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "GlobalStrategyManager: " << x << endl; DebugFile.close();}
#if 1
#define ODEBUG(x)
#else
#define ODEBUG(x)  std::cout << x << endl;
#endif

#define ODEBUG3(x)  std::cout << x << endl;

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

/* \file ZMPRefTrajectoryGeneration.cpp
   \brief Abstract object for ZMP trajectory generation.

   Copyright (c) 2008, 
   Olivier Stasse,
   Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   All rights reserved.


   For more information on the license please look at License.txt 
   in the root directory.
   
*/
#include <ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.h>

#define ODEBUG2(x)
#define ODEBUG3(x) cout << "ZMPRefTrajectoryGeneration :" << x << endl
#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#if 0
#define ODEBUG(x) cerr << "ZMPRefTrajectoryGeneration :" <<  x << endl
#else
#define ODEBUG(x)
#endif

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); \
    DebugFile << "PGI: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1
#else
#define RESETDEBUG4(y)
#define ODEBUG4(x,y)
#endif

#define ODEBUG6(x,y)

using namespace PatternGeneratorJRL;

ZMPRefTrajectoryGeneration::ZMPRefTrajectoryGeneration(SimplePluginManager *lSPM)
  : SimplePlugin(lSPM)
{

  ODEBUG("Identification: " << this);
  std::string aMethodName[5] = 
    {":omega",
     ":stepheight",
     ":singlesupporttime",
     ":doublesupporttime",
     ":comheight"};
  
  for(int i=0;i<5;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
      else
	{
	  ODEBUG("Succeed in registering " << aMethodName[i]);
	}

    }
};


void ZMPRefTrajectoryGeneration::CallMethod(std::string & Method, std::istringstream &strm)
{
  ODEBUG("Calling me (" << this << ") with Method: " << Method);
  if (Method==":omega")
    {
      strm >> m_Omega;
    }
  else if (Method==":stepheight")
    {
      strm >> m_StepHeight;
      ODEBUG("Value of stepheight " << m_StepHeight << " this:" << this);
    }
  else if (Method==":singlesupporttime")
    {
      strm >> m_Tsingle;
      ODEBUG(":singlesupporttime " << m_Tsingle << " ID: " << this);
    }
  else if (Method==":doublesupporttime")
    {
      strm >> m_Tdble;
      ODEBUG(":doublesupporttime " << m_Tdble << " ID: " << this);
    }
  else if (Method==":comheight")
    {
      strm >> m_ComHeight;
      ODEBUG(":comheight" << m_ComHeight << " ID: " << this);
    }

};

bool ZMPRefTrajectoryGeneration::GetOnLineMode()
{
  return m_OnLineMode;
}

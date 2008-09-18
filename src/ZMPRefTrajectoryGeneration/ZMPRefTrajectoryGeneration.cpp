/* \file ZMPRefTrajectoryGeneration.cpp
   \brief Abstract object for ZMP trajectory generation.

   Copyright (c) 2008, 
   Olivier Stasse,
   Ramzi Sellouati
   
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
#include <walkGenJrl/ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.h>

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "ZMPRefTrajectoryGeneration :" << x << endl
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
  std::string aMethodName[4] = 
    {":omega",":stepheight",":singlesupporttime",":doublesupporttime"};
  
  for(int i=0;i<4;i++)
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
  ODEBUG("Method: " << Method);
  if (Method==":omega")
    {
      strm >> m_Omega;
    }
  else if (Method==":stepheight")
    {
      strm >> m_StepHeight;
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

};

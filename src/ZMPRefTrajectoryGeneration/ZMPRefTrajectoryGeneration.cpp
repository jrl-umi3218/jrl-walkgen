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
#include <Debug.h>
#include <ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.h>


using namespace PatternGeneratorJRL;

ZMPRefTrajectoryGeneration::ZMPRefTrajectoryGeneration(SimplePluginManager *lSPM)
  : SimplePlugin(lSPM)
  , m_Tsingle(0.)
  , m_Tdble(0.)
  , m_SamplingPeriod(0.)
  , m_ModulationSupportCoefficient(0.)
  , m_Omega(0.)
  , m_PreviewControlTime(0.)
  , m_StepHeight(0.)
  , m_CurrentTime(0.)
  , m_OnLineMode(false)
  , m_ComHeight(0.)
{

  ODEBUG("Identification: " << this);
  std::string aMethodName[6] = 
    {":omega",
     ":stepheight",
     ":singlesupporttime",
     ":doublesupporttime",
     ":comheight",
     ":samplingperiod"};
  
  for(int i=0;i<6;i++)
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
  else if (Method==":samplingperiod")
    {
      strm >> m_SamplingPeriod;
      ODEBUG(":samplingperiod" << m_SamplingPeriod << " ID: " << this);
    }
  
};

bool ZMPRefTrajectoryGeneration::GetOnLineMode()
{
  return m_OnLineMode;
}

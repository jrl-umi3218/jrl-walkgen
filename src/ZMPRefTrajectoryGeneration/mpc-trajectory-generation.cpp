/*
 * Copyright 2010, 
 *
 * Andrei Herdt
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
/* \file mpc-trajectory-generation.cpp
   \brief Abstract object for trajectory generation via model predictive control.*/

#include <Debug.h>
#include <ZMPRefTrajectoryGeneration/mpc-trajectory-generation.hh>


using namespace PatternGeneratorJRL;

MPCTrajectoryGeneration::MPCTrajectoryGeneration(SimplePluginManager *lSPM)
  : SimplePlugin(lSPM)
  , m_Tsingle(0.)
  , m_Tdble(0.)
  , m_T_Ctr(0.)
  , m_T_Prw(0.)
  , m_PreviewControlTime(0.)
  , m_StepHeight(0.)
  , m_CurrentTime(0.)
  , m_OnLineMode(false)
  , m_CoMHeight(0.)
  , m_SecurityMargin(0.)
  , m_ModulationSupportCoefficient(0.)
  , m_Omega(0.)
{

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
}


void MPCTrajectoryGeneration::CallMethod(std::string & Method, std::istringstream &strm)
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
      strm >> m_CoMHeight;
      ODEBUG(":comheight" << m_ComHeight << " ID: " << this);
    }
  else if (Method==":samplingperiod")
    {
      strm >> m_T_Prw;
      ODEBUG(":samplingperiod" << m_T_Prw << " ID: " << this);
    }
  
}

bool MPCTrajectoryGeneration::GetOnLineMode()
{
  return m_OnLineMode;
}

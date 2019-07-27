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
   \brief Abstract object for trajectory generation 
   via model predictive control.*/

#include <Debug.hh>
#include <ZMPRefTrajectoryGeneration/mpc-trajectory-generation.hh>


using namespace PatternGeneratorJRL;

MPCTrajectoryGeneration::MPCTrajectoryGeneration(SimplePluginManager *lSPM)
  : SimplePlugin(lSPM)
  , Tsingle_(0.)
  , Tdble_(0.)
  , Tctr_(0.)
  , Tprw_(0.)
  , PreviewControlTime_(0.)
  , N_(0)
  , NbVariables_(0)
  , StepHeight_(0.)
  , CurrentTime_(0.)
  , OnLineMode_(false)
  , CoMHeight_(0.)
  , SecurityMargin_(0.)
  , ModulationSupportCoefficient_(0.)
  , Omega_(0.)
{

  std::string aMethodName[6] =
    {
     ":omega",
     ":stepheight",
     ":singlesupporttime",
     ":doublesupporttime",
     ":comheight",
     ":samplingperiod"
    };

  for(int i=0; i<6; i++)
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


void MPCTrajectoryGeneration::CallMethod(std::string & Method,
                                         std::istringstream &strm)
{
  ODEBUG("Calling me (" << this << ") with Method: " << Method);
  if (Method==":omega")
    {
      strm >> Omega_;
    }
  else if (Method==":omega2")
    {
      strm >> Omega2_;
    }
  else if (Method==":stepheight")
    {
      strm >> StepHeight_;
      ODEBUG("Value of stepheight " << StepHeight_ << " this:" << this);
    }
  else if (Method==":singlesupporttime")
    {
      strm >> Tsingle_;
      ODEBUG(":singlesupporttime " << Tsingle_ << " ID: " << this);
    }
  else if (Method==":doublesupporttime")
    {
      strm >> Tdble_;
      ODEBUG(":doublesupporttime " << Tdble_ << " ID: " << this);
    }
  else if (Method==":comheight")
    {
      strm >> CoMHeight_;
      ODEBUG(":comheight" << CoMHeight_ << " ID: " << this);
    }
  else if (Method==":samplingperiod")
    {
      strm >> Tprw_;
      ODEBUG(":samplingperiod" << Tprw_ << " ID: " << this);
    }

}

bool MPCTrajectoryGeneration::GetOnLineMode()
{
  return OnLineMode_;
}

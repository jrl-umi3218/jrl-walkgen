/*
 * Copyright 2010, 
 *
 * Andrei  Herdt
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

#include <iostream>
#include <fstream>

#include <PreviewControl/SupportFSM.h>
#include <Debug.h>

using namespace PatternGeneratorJRL;
using namespace std;

SupportFSM::SupportFSM()
{

}


SupportFSM::~SupportFSM()
{
}

void SupportFSM::set_support_state(const double &Time, const int &pi,
				 support_state_t & Support, const reference_t & Ref) const
{

  double eps = 1e-6;

  Support.StateChanged = false;

  bool ReferenceGiven = false;
  if(fabs(Ref.Local.x)>eps||fabs(Ref.Local.y)>eps||fabs(Ref.Local.yaw)>eps)
    ReferenceGiven = true;

  // Update time limit for double support phase
  if(ReferenceGiven && Support.Phase == 0 && (Support.TimeLimit-Time-eps) > DSSSPeriod_)
    {
      Support.TimeLimit = Time+DSSSPeriod_;
      Support.NbStepsLeft = NbStepsSSDS_;
    }

  //FSM
  if(Time+eps+pi*T_ >= Support.TimeLimit)
    {

      //SS->DS
      if(Support.Phase == 1  && !ReferenceGiven && Support.NbStepsLeft == 0)
	{
	  Support.Phase = 0;
	  Support.TimeLimit = Time+pi*T_ + DSPeriod_;
	  Support.StateChanged = true;
	}
      //DS->SS
      else if(Support.Phase == 0 && ReferenceGiven || Support.Phase == 0 && Support.NbStepsLeft > 0)
	{
	  Support.Phase = 1;
	  Support.TimeLimit = Time+pi*T_ + StepPeriod_;
	  Support.NbStepsLeft = NbStepsSSDS_;
	  Support.StateChanged = true;
	}
      //SS->SS
      else if(Support.Phase == 1 && Support.NbStepsLeft > 0 ||
	      Support.NbStepsLeft == 0 && ReferenceGiven)
	{
	  Support.Foot = -1*Support.Foot;
	  Support.StateChanged = true;
	  Support.TimeLimit = Time+pi*T_ + StepPeriod_;
	  Support.StepNumber++;
	  if (!ReferenceGiven)
	    Support.NbStepsLeft = Support.NbStepsLeft-1;
	  if (ReferenceGiven)
	    Support.NbStepsLeft = NbStepsSSDS_;
	}
    }

}

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

void SupportFSM::setSupportState(const double &Time, const int &pi,
				 support_state_t & Support, const reference_t & Ref) const
{

  double eps = 1e-6;

  Support.StateChanged = false;

  bool ReferenceGiven = false;
  if(fabs(Ref.local.x)>eps||fabs(Ref.local.y)>eps||fabs(Ref.local.yaw)>eps)
    ReferenceGiven = true;

  if(ReferenceGiven == true && Support.Phase == 0 && (Support.TimeLimit-Time-eps)>DSSSPeriod_)
    {
      Support.TimeLimit = Time+DSSSPeriod_;
    }


  //FSM
  if(Time+eps+pi*T_ >= Support.TimeLimit)
    {
      //SS->DS
      if(Support.Phase == 1  && ReferenceGiven == false && Support.StepsLeft==0)
	{
	  Support.Phase = 0;
	  Support.TimeLimit = Time+pi*T_ + DSPeriod_;
	  Support.StateChanged = true;
	}
      //DS->SS
      else if(Support.Phase == 0 && ReferenceGiven == true)
	{
	  Support.Phase = 1;
	  Support.TimeLimit = Time+pi*T_ + StepPeriod_;
	  Support.StepsLeft = NbStepsSSDS_;
	  Support.StateChanged = true;
	}
      //SS->SS
      else if(((Support.Phase == 1) && (Support.StepsLeft>0)) ||
	      ((Support.StepsLeft==0) && (ReferenceGiven == true)))
	{
	  Support.Foot = -1*Support.Foot;
	  Support.StateChanged = true;
	  Support.TimeLimit = Time+pi*T_ + StepPeriod_;
	  Support.StepNumber++;
	  if (ReferenceGiven == false)
	    Support.StepsLeft = Support.StepsLeft-1;
	}
    }

}

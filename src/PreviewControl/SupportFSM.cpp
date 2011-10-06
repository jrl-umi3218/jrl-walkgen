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
:EPS_(1e-6)
,in_translation_(false)
,in_rotation_(false)
,Current_support_foot_(LEFT)
,nb_steps_after_end_of_rotation_(0)
,start_of_end_rotation_phase_(false)
{

}


SupportFSM::~SupportFSM()
{
}

void SupportFSM::update_vel_reference(reference_t & Ref, const support_state_t & CurrentSupport){
	// Check the reference type of the robot (rotation, translation)
	if(fabs(Ref.Local.x)>2*EPS_||fabs(Ref.Local.y)>2*EPS_){
	    in_translation_ = true;
	}else{
		in_translation_ = false;
	}
	if(fabs(Ref.Local.yaw)>EPS_){
	    in_rotation_ = true;
	}else{
		// make two step to avoid the robot's fall
		if (in_rotation_ && !in_translation_){
			Ref.Local.x=2*EPS_;
			Ref.Local.y=2*EPS_;
			if (!start_of_end_rotation_phase_){
				Current_support_foot_=CurrentSupport.Foot;
				nb_steps_after_end_of_rotation_=0;
				start_of_end_rotation_phase_=true;
			}else{
				if (Current_support_foot_!=CurrentSupport.Foot){
					Current_support_foot_=CurrentSupport.Foot;
					++nb_steps_after_end_of_rotation_;
				}
				if (nb_steps_after_end_of_rotation_>2){
					in_rotation_=false;
					start_of_end_rotation_phase_=false;
				}
			}
		}else{
			in_rotation_=false;
		}
	}
}

void SupportFSM::set_support_state(double Time, unsigned int Pi,
				 support_state_t & Support, const reference_t & Ref) const
{





  Support.StateChanged = false;
  Support.NbInstants++;

  bool ReferenceGiven = false;
  if(fabs(Ref.Local.x)>EPS_||fabs(Ref.Local.y)>EPS_||fabs(Ref.Local.yaw)>EPS_)
    ReferenceGiven = true;

  // Update time limit for double support phase
  if(ReferenceGiven && Support.Phase == DS && (Support.TimeLimit-Time-EPS_) > DSSSPeriod_)
    {
      Support.TimeLimit = Time+DSSSPeriod_-T_/10.0;
      Support.NbStepsLeft = NbStepsSSDS_;
    }

  //FSM
  if(Time+EPS_+Pi*T_ >= Support.TimeLimit)
    {
      //SS->DS
      if(Support.Phase == SS  && !ReferenceGiven && Support.NbStepsLeft == 0)
	{
	  Support.Phase = DS;
	  Support.TimeLimit = Time+Pi*T_+DSPeriod_-T_/10.0;
	  Support.StateChanged = true;
	  Support.NbInstants = 0;
	}
      //DS->SS
      else if((Support.Phase == DS && ReferenceGiven) || (Support.Phase == DS && Support.NbStepsLeft) > 0)
	{
	  Support.Phase = SS;
	  Support.TimeLimit = Time+Pi*T_+StepPeriod_-T_/10.0;
	  Support.NbStepsLeft = NbStepsSSDS_;
	  Support.StateChanged = true;
	  Support.NbInstants = 0;
	}
      //SS->SS
      else if((Support.Phase == SS && Support.NbStepsLeft > 0) ||
	      (Support.NbStepsLeft == 0 && ReferenceGiven))
	{
          if(Support.Foot == LEFT)
            Support.Foot = RIGHT;
          else
            Support.Foot = LEFT;
	  Support.StateChanged = true;
	  Support.NbInstants = 0;
	  Support.TimeLimit = Time+Pi*T_+StepPeriod_-T_/10.0;
	  Support.StepNumber++;
	  if (!ReferenceGiven)
	    Support.NbStepsLeft = Support.NbStepsLeft-1;
	  if (ReferenceGiven)
	    Support.NbStepsLeft = NbStepsSSDS_;
	}
    }

}

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

using namespace PatternGeneratorJRL;
using namespace std;

SupportFSM::SupportFSM()
:EPS_(1e-6)
,InTranslation_(false)
,InRotation_(false)
,NbStepsAfterRotation_(0)
,CurrentSupportFoot_(LEFT)
,PostRotationPhase_(false)
{

}


SupportFSM::~SupportFSM()
{
}


void
SupportFSM::update_vel_reference(reference_t & Ref, const support_state_t & CurrentSupport){
  // Check the reference type of the robot (rotation, translation)
  if(fabs(Ref.Local.X)>2*EPS_||fabs(Ref.Local.Y)>2*EPS_){
      InTranslation_ = true;
  }else{
      InTranslation_ = false;
  }
  if(fabs(Ref.Local.Yaw)>EPS_){
      InRotation_ = true;
  }else{
      // make two step to avoid the robot's fall
      if (InRotation_ && !InTranslation_){
          Ref.Local.X=2*EPS_;
          Ref.Local.Y=0;
          if (!PostRotationPhase_){
              CurrentSupportFoot_ = CurrentSupport.Foot;
              NbStepsAfterRotation_ = 0;
              PostRotationPhase_ = true;
          }else{
              if (CurrentSupportFoot_ != CurrentSupport.Foot){
                  CurrentSupportFoot_ = CurrentSupport.Foot;
                  ++NbStepsAfterRotation_;
              }
              if (NbStepsAfterRotation_>2){
                  InRotation_ = false;
                  PostRotationPhase_ = false;
              }
          }
      }else{
          InRotation_ = false;
      }
  }
}


void
SupportFSM::set_support_state(double Time, unsigned int Pi,
    support_state_t & Support, const reference_t & Ref) const
{

  Support.StateChanged = false;
  Support.NbInstants++;

  bool ReferenceGiven = false;
  if(fabs(Ref.Local.X)>EPS_||fabs(Ref.Local.Y)>EPS_||fabs(Ref.Local.Yaw)>EPS_)
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
      else if( ((Support.Phase == DS) && ReferenceGiven) 
		  ||   ((Support.Phase == DS) && (Support.NbStepsLeft > 0)))
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

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
/* TODO 3: Restructure the class
 *  setSupportState.cpp
 */

#include <iostream>
#include <fstream>

#include <PreviewControl/SupportFSM.hh>
#include <Debug.hh>

using namespace PatternGeneratorJRL;
using namespace std;

SupportFSM::SupportFSM(const double &SamplingPeriod)
{
  m_SSPeriod = 0.8; 	  //Duration of one step
  m_DSDuration = 1e9;       //Duration of the DS phase
  m_DSSSDuration = 0.8;
  //TODO: setNumberOfStepsSSDS
  m_NbOfStepsSSDS = 200;

  m_T = SamplingPeriod;

  m_eps = 0.00000001;

  m_FullDebug = 0;

}
 

SupportFSM::~SupportFSM()
{
}

void setSupportState(const double &Time, const int &pi,
		     SupportState_t & Support, const ReferenceAbsoluteVelocity & RefVel)
{
  bool ReferenceGiven = false;
  if(fabs(RefVel.x)>0||fabs(RefVel.y)>0)
    ReferenceGiven = true;
  setSupportState(Time,pi,Support,ReferenceGiven);

}

void SupportFSM::setSupportState(const double &Time, const int &pi,
		SupportState_t & Support, const bool & ReferenceGiven)
{

  Support.StateChanged = false;
  
  m_ReferenceGiven = false;
  if(ReferenceGiven)
	  m_ReferenceGiven = true;
    
  if(m_ReferenceGiven == true && Support.Phase == 0 && (Support.TimeLimit-Time-m_eps)>m_DSSSDuration)
    {
      Support.TimeLimit = Time+m_DSSSDuration;
    }
 
   
  //FSM
  if(Time+m_eps+pi*m_T >= Support.TimeLimit)
    {
      //SS->DS
      if(Support.Phase == 1  && m_ReferenceGiven == false && Support.StepsLeft==0)
	{
	  Support.Phase = 0;
	  Support.TimeLimit = Time+pi*m_T + m_DSDuration;
	  Support.StateChanged = true;
	}
      //DS->SS
      else if(Support.Phase == 0 && m_ReferenceGiven == true)
	{
	  Support.Phase = 1;
	  Support.TimeLimit = Time+pi*m_T + m_SSPeriod;
	  Support.StepsLeft = m_NbOfStepsSSDS;
	  Support.StateChanged = true;
	}
      //SS->SS
      else if(((Support.Phase == 1) && (Support.StepsLeft>0)) ||
	      ((Support.StepsLeft==0) && (m_ReferenceGiven == true)))
	{
	  Support.Foot = -1*Support.Foot;
	  Support.StateChanged = true;
	  Support.TimeLimit = Time+pi*m_T + m_SSPeriod;
	  Support.StepNumber++;
	  Support.SSSS = 1;
	  if (m_ReferenceGiven == false)
	    Support.StepsLeft = Support.StepsLeft-1;
	}
    }
  
  if(m_FullDebug>0)
    {
      ofstream aof;
      aof.open("SupportStates.dat", ios::app);
      aof << "Time: "<<Time<<" PrwTime: "<<Time+pi*m_T
  	  <<" CSF: "<<Support.Foot<<" CTL: "<<Support.TimeLimit
  	  <<" SL: "<<Support.StepsLeft<<" *SF: "<<Support.Foot
  	  <<" SN: "<<Support.StepNumber;
      aof << endl;
      aof.close();
    }

}

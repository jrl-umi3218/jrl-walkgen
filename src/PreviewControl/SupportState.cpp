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

#include <PreviewControl/SupportState.h>
#include <Debug.h>

using namespace PatternGeneratorJRL;
using namespace std;

SupportState::SupportState(const double &SamplingPeriod)
{
  SSPeriod = 0.8; 	  //Duration of one step
  DSDuration = 1e9;       //Duration of the DS phase
  DSSSDuration = 0.8;
  NbOfStepsSSDS = 200;

  m_T = SamplingPeriod;
  //Initial current state
  CurrentSupportPhase = 0;
  CurrentSupportFoot = 1;
  CurrentTimeLimit = 1000000000;
  CurrentStepsLeft = 1;
  StartSupportFoot = 1;

  eps = 0.00000001;
  m_StateChanged = false;

  m_FullDebug = 0;

}
 

SupportState::~SupportState()
{
}

void SupportState::setSupportState(const double &Time, const int &pi,  const ReferenceAbsoluteVelocity & RefVel)

{

  m_StateChanged = false;

  if(pi==0) {
    m_SupportPhase = &CurrentSupportPhase;
    m_SupportFoot = &CurrentSupportFoot;
    m_SupportStepsLeft = &CurrentStepsLeft;
    m_SupportTimeLimit = &CurrentTimeLimit;
    StepNumber = 0;
    SSSS = 0;
  }
  else {
    m_SupportPhase = &PrwSupportPhase;
    m_SupportFoot = &PrwSupportFoot;
    m_SupportStepsLeft = &PrwStepsLeft;
    m_SupportTimeLimit = &PrwTimeLimit;
  }

  
  ReferenceGiven = -1;
  if(fabs(RefVel.x)>0||fabs(RefVel.y)>0)
    ReferenceGiven = 1;
    
  if(ReferenceGiven == 1 && *m_SupportPhase == 0 && (*m_SupportTimeLimit-Time-eps)>DSSSDuration)
    {
      *m_SupportTimeLimit = Time+DSSSDuration;
    }
 
   
  //FSM
  if(Time+eps+pi*m_T >= *m_SupportTimeLimit)
    {
      //SS->DS
      if(*m_SupportPhase == 1  && ReferenceGiven == -1 && *m_SupportStepsLeft==0)
	{
	  *m_SupportPhase = 0;
	  *m_SupportTimeLimit = Time+pi*m_T + DSDuration;
	  m_StateChanged = true;
	}
      //DS->SS
      else if(*m_SupportPhase == 0 && ReferenceGiven == 1)
	{
	  *m_SupportPhase = 1;
	  *m_SupportTimeLimit = Time+pi*m_T + SSPeriod;
	  *m_SupportStepsLeft = NbOfStepsSSDS;
	  m_StateChanged = true;
	}
      //SS->SS
      else if(((*m_SupportPhase == 1) && (*m_SupportStepsLeft>0)) ||
	      ((*m_SupportStepsLeft==0) && (ReferenceGiven == 1)))
	{
	  *m_SupportFoot = -1**m_SupportFoot;
	  m_StateChanged = true;
	  *m_SupportTimeLimit = Time+pi*m_T + SSPeriod;
	  StepNumber++;
	  SSSS = 1;
	  if (ReferenceGiven == -1)
	    *m_SupportStepsLeft = *m_SupportStepsLeft-1;
	}
    }


  if(pi==0)
    initializePreviewedState();

  
  if(m_FullDebug>0)
    {
      ofstream aof;
      aof.open("SupportStates.dat", ios::app);
      aof << "Time: "<<Time<<" PrwTime: "<<Time+pi*m_T<<" CSP: "<<CurrentSupportPhase
  	  <<" CSF: "<<CurrentSupportFoot<<" CTL: "<<CurrentTimeLimit
  	  <<" CSL: "<<CurrentStepsLeft<<" PrwSP: "<<PrwSupportPhase
  	  <<" PrwSF: "<<PrwSupportFoot<<" PrwTL: "<<PrwTimeLimit
  	  <<" PrwSL: "<<PrwStepsLeft<<" *SF: "<<*m_SupportFoot
  	  <<" *SSL: "<<*m_SupportStepsLeft<<" SN: "<<StepNumber;
      aof << endl;
      aof.close();
    }

}

void SupportState::initializePreviewedState()
{ 
  PrwSupportPhase = CurrentSupportPhase;
  PrwSupportFoot = CurrentSupportFoot;
  PrwStepsLeft = CurrentStepsLeft;
  PrwTimeLimit = CurrentTimeLimit;
  StepNumber = 0;
}

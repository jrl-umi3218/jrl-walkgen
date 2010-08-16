/* TODO 3: Restructure the class
 *  setSupportState.cpp
 *  
 *
 *  Created by Andrei  on 27/01/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
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

  // RESETDEBUG4("DebugSupportState.dat");
  // ofstream aof("SupportStates.dat");

}
 

SupportState::~SupportState()
{
}

void SupportState::setSupportState(const double &Time, const int &pi,  const ReferenceAbsoluteVelocity & RefVel)

{

  m_StateChanged = false;

  if(pi==0) {
    SupportPhase = &CurrentSupportPhase;
    SupportFoot = &CurrentSupportFoot;
    SupportStepsLeft = &CurrentStepsLeft;//to be changed
    SupportTimeLimit = &CurrentTimeLimit;
    StepNumber = 0;
    SSSS = 0;
  }
  else {
    SupportPhase = &PrwSupportPhase;
    SupportFoot = &PrwSupportFoot;
    SupportStepsLeft = &PrwStepsLeft;//to be changed
    SupportTimeLimit = &PrwTimeLimit;
  }

  
  ReferenceGiven = -1;
  if(fabs(RefVel.x)>0||fabs(RefVel.y)>0)//Andremize
    ReferenceGiven = 1;
    
  if(ReferenceGiven == 1 && *SupportPhase == 0 && (*SupportTimeLimit-Time-eps)>DSSSDuration)
    {
      *SupportTimeLimit = Time+DSSSDuration; 
    }
 
   
  //FSM
  if(Time+eps+pi*m_T >= *SupportTimeLimit)
    {
      //SS->DS
      if(*SupportPhase == 1  && ReferenceGiven == -1 && *SupportStepsLeft==0)
	{
	  *SupportPhase = 0;	
	  *SupportTimeLimit = Time+pi*m_T + DSDuration;
	  m_StateChanged = true;
	}
      //DS->SS
      else if(*SupportPhase == 0 && ReferenceGiven == 1)
	{
	  *SupportPhase = 1;
	  //*SupportFoot = CurrentSupportFoot;//StartSupportFoot;
	  *SupportTimeLimit = Time+pi*m_T + SSPeriod;
	  *SupportStepsLeft = NbOfStepsSSDS;
	  m_StateChanged = true;
	}
      //SS->SS
      else if(((*SupportPhase == 1) && (*SupportStepsLeft>0)) || 
	      ((*SupportStepsLeft==0) && (ReferenceGiven == 1)))
	{
	  *SupportFoot = -1**SupportFoot;
	  m_StateChanged = true;
	  *SupportTimeLimit = Time+pi*m_T + SSPeriod;
	  StepNumber++;
	  SSSS = 1;
	  if (ReferenceGiven == -1)
	    *SupportStepsLeft = *SupportStepsLeft-1;
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
  	  <<" PrwSL: "<<PrwStepsLeft<<" *SF: "<<*SupportFoot
  	  <<" *SSL: "<<*SupportStepsLeft<<" SN: "<<StepNumber;
      aof << endl;
      aof.close();
    }

 // ODEBUG4( " " , "DebugSupportState.dat");

}

//Andremize: initialization only necessary when state changes
void SupportState::initializePreviewedState()
{ 
  PrwSupportPhase = CurrentSupportPhase;
  PrwSupportFoot = CurrentSupportFoot;
  PrwStepsLeft = CurrentStepsLeft;
  PrwTimeLimit = CurrentTimeLimit;
  StepNumber = 0;
}

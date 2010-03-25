/*
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

SupportState::SupportState()
{

  printf("Entered SupportState \n");
  SSDuration = 0.8; 	  //Duration of one step
  DSDuration = 1e9;       //Duration of the DS phase
  DSSSDuration = 0.8;
  NumberSteps = 0;

  //Initial current state
  CurrentPhase = 0;
  CurrentSupportFoot = 1;
  CurrentTimeLimit = 1000000000;
  CurrentStepsLeft = 0;
  StartSupportFoot = 1;

  eps = 0.00000001;
  StateChanged = -1;

  s_FullDebug = 1;
  RESETDEBUG4("DebugSupportState.dat");

  printf("Leaving SupportState \n");
}


SupportState::~SupportState()
{
}

void SupportState::setSupportState(const double &Time, const int &pi,  double Ref[3])

{
  printf("Inside setSupportState \n");

  StateChanged = -1;

  if(pi==0) {
    SupportPhase = &CurrentPhase;
    SupportFoot = &CurrentSupportFoot;
    StepNumber = 0;
    SupportStepsLeft = &CurrentStepsLeft;//to be changed
    SupportTimeLimit = &CurrentTimeLimit;
  }
  else {
    SupportPhase = &PrwPhase;
    SupportFoot = &PrwSupportFoot;
    SupportStepsLeft = &PrwStepsLeft;//to be changed
    SupportTimeLimit = &PrwTimeLimit;
  }

  
  ReferenceGiven = -1;
  for(int i = 0; i<3; i++)
    {
      if(fabs(Ref[i])>0)
	ReferenceGiven = 1;
    }
  if(ReferenceGiven == 1 && *SupportPhase == 0 && (*SupportTimeLimit-Time-eps)>DSSSDuration)
    {
      //printf("SupportTimeLimit changed: %f", *SupportTimeLimit);
      *SupportTimeLimit = Time+DSSSDuration;
      //printf("    %f %f \n", *SupportTimeLimit, CurrentTimeLimit); 
    }
    
  /*
  if(s_FullDebug>0)
    {
      ofstream aof;
      aof.open("SupportState.dat");
      aof << *SupportPhase;// << " ";// << *SupportFoot << " " <<
      //	*SupportTimeLimit <<" ";
      //aof << endl;
      aof.close();
    }
  */
 
  
  //FSM
  if(Time+eps >= *SupportTimeLimit)
    {
      //SS->DS
      if(*SupportPhase == 1  && ReferenceGiven == -1 && *SupportStepsLeft==0)
	{
	  //printf("SSDS in c \n");
	  *SupportPhase = 0;	
	  *SupportTimeLimit = Time + DSDuration;
	  StateChanged = 1;
	}
      //DS->SS
      else if(*SupportPhase == 0 && ReferenceGiven == 1)
	{
	  //printf("DSSS in c \n");
	  *SupportPhase = 1;
	  *SupportFoot = StartSupportFoot;
	  *SupportTimeLimit = Time + SSDuration;
	  *SupportStepsLeft = NumberSteps;
	  StateChanged = 1;
	}
      //SS->SS
      else if(*SupportPhase == 1 && *SupportStepsLeft>0 || *SupportStepsLeft==0 && ReferenceGiven == 1)
	{
	  //printf("SSSS in c \n");
	  *SupportFoot = -1**SupportFoot;
	  StateChanged = 1;
	  *SupportTimeLimit = Time + SSDuration;
	  StepNumber = StepNumber+1;
	  if (ReferenceGiven == -1)
	    *SupportStepsLeft = *SupportStepsLeft-1;
	}
    }

  if(pi==0)
    initializePreviewedState();
  ODEBUG4( " " , "DebugSupportState.dat");
  
  printf("Leaving setSupportState \n");
  //printf("CurrentTimeLimit inside:    %f %f \n", *SupportTimeLimit, CurrentTimeLimit);
}

void SupportState::initializePreviewedState()
{ 
  printf("Inside initializePreviewedState \n");
  PrwPhase = CurrentPhase;
  PrwSupportFoot = CurrentSupportFoot;
  PrwStepsLeft = CurrentStepsLeft;
  PrwTimeLimit = CurrentTimeLimit;
  StepNumber = 0;
}

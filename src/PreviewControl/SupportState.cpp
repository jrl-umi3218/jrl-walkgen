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

SupportState::SupportState(const double & SimulationTime, const double & SimulationSamplingPeriod)
{

	SSDuration = 0.8; 		//Duration of one step
	DSDuration = 1e9;       //Duration of the DS phase
	DSSSDuration = 0.8;
	StartSupportFoot = 1;



double eps = 0.00000001;
	*StateChanged == -1;
	//Limit the DS phase when a non-zero reference is given
}


SupportState::~SupportState()
{
}

void setSupportState(const double &Time, const int pi, double *TimeLimit, double Ref[3],int *SupportPhase,
int* SupportFoot, int* StepNumber, int* StepsLeft, int* NumberSteps, int* StateChanged)

{




int i,ReferenceGiven;
ReferenceGiven = -1;
for(i = 0; i<3; i++)
{
if(fabs(Ref[i])>0)
ReferenceGiven = 1;
if(ReferenceGiven == 1 && *SupportPhase == 0 && (*TimeLimit-*Time-eps)>DSSSDuration)
*TimeLimit = *Time+DSSSDuration;
}
/*
 printf("double var: %f %f %f %f %f /n", *TimeLimit, *Time, Ref[0], Ref[1], Ref[2]);
 printf("int var: %d %d %d %d %d /n", *SupportPhase, *SupportFoot, *StepNumber, *StateChanged, *StepsLeft);
 */
//FSM
if(*Time+eps >= *TimeLimit)
{
//SS->DS
if(*SupportPhase == 1  && ReferenceGiven == -1 && *StepsLeft==0)
{
//printf("SSDS in c \n");
*SupportPhase = 0;	
*TimeLimit = *Time + DSDuration;
*StateChanged = 1;
}
//DS->SS
else if(*SupportPhase == 0 && ReferenceGiven == 1)
{
//printf("DSSS in c \n");
*SupportPhase = 1;
*SupportFoot = StartSupportFoot;
*TimeLimit = *Time + SSDuration;
*StepsLeft = *NumberSteps;
*StateChanged = 1;
}
//SS->SS
else if(*SupportPhase == 1 && *StepsLeft>0 || *StepsLeft==0 && ReferenceGiven == 1)
{
//printf("SSSS in c \n");
*SupportFoot = -*SupportFoot;
*StateChanged = 1;
*TimeLimit = *Time + SSDuration;
*StepNumber = *StepNumber+1;
if (ReferenceGiven == -1)
*StepsLeft = *StepsLeft-1;
}
}

void initializeFSM()
{ 
Phase = CurrentPhase;
Foot = CurrentFoot;
StepsLeft = CurrentStepsLeft;
TimeLimit = CurrentTimeLimit;
StepNumber = 0;
}
}

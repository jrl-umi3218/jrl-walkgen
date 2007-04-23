/* This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of steps.


   Copyright (c) 2005-2006, 
   Olivier Stasse,
   Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.h>
#include <Mathematics/ConvexHull.h>
#include <Mathematics/qld.h>

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile <<  x << endl; DebugFile.close();}
#else
#define RESETDEBUG4(y) 
#define ODEBUG4(x,y) 
#endif

#define RESETDEBUG6(y) 
#define ODEBUG6(x,y) 

#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << x << endl; DebugFile.close();}
#define ODEBUG5NOE(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << x ; DebugFile.close();}
#if 1
#define ODEBUG(x)
#else
#define ODEBUG(x)  std::cout << x << endl;
#endif

#define ODEBUG3(x)  std::cout << x << endl;

using namespace::PatternGeneratorJRL;

ZMPDiscretization::ZMPDiscretization(string DataFile, HumanoidSpecificities *aHS)
{
  m_HS = aHS;
  
  double lWidth,lHeight,lDepth;
  m_HS->GetFootSize(-1,lDepth,lWidth,lHeight);
  double AnklePosition[3];
  m_HS->GetAnklePosition(-1,AnklePosition);
  m_FootB = AnklePosition[0];
  m_FootH = AnklePosition[2];
  m_FootF = lDepth-AnklePosition[0];
  
  MAL_MATRIX_RESIZE(m_A,6,6);
  MAL_MATRIX_RESIZE(m_B,6,1);
  MAL_MATRIX_RESIZE(m_C,2,6);

  m_RelativeFootPositions.clear();

  //  m_Omega = 3.0;
  //m_Omega = 1.0;
  m_Omega =0.0;
  m_Tsingle = 0.78; 
  //m_Tsingle = 0.25; // For Bjorn  m_Tdble = 0.1;
  m_Tdble = 0.02;
  m_SamplingPeriod = 0.005;
  m_PreviewControlTime = 1.6;
  //m_StepHeight = 0.07;
  m_StepHeight = 0.12;

  

  m_ModulationSupportCoefficient=0.9;
    
  if (DataFile.length()!=0)
    {
      std::ifstream a_iof;
      a_iof.open(DataFile.c_str(),std::ifstream::in);
      if (a_iof.is_open())
	{
	  
	  a_iof.close();
	}
      
    }
#ifdef FULL_POLYNOME
  m_PolynomeX = new Polynome3(0,0);
  m_PolynomeY = new Polynome3(0,0);
  m_PolynomeZ = new Polynome6(0,0);
  m_PolynomeOmega = new Polynome3(0,0);
  m_PolynomeOmega2 = new Polynome3(0,0);
  m_PolynomeTheta = new Polynome3(0,0);
  m_PolynomeZMPTheta = new Polynome3(0,0);
#else 
  m_PolynomeX = new Polynome3(0,0);
  m_PolynomeY = new Polynome3(0,0);
  m_PolynomeZ = new Polynome4(0,0);
  m_PolynomeOmega = new Polynome3(0,0);
  m_PolynomeOmega2 = new Polynome3(0,0);
  m_PolynomeTheta = new Polynome3(0,0);
  m_PolynomeZMPTheta = new Polynome3(0,0);
#endif
  
  m_ZMPShift.resize(4);
  for (unsigned int i=0;i<4;i++)
    m_ZMPShift[i]=0.0;
  
  /*
    m_ZMPShift3Begin = 0.0;
    m_ZMPShift4Begin = 0.02;	
    m_ZMPShift3End  =  0.07;
    m_ZMPShift4End  =  0.02;

    ///Y direction does not work yet (bram said)!!

    m_ZMPShift3BeginY = 0.00;
    m_ZMPShift4BeginY = 0.00;	
    m_ZMPShift3EndY  =  0.00;
    m_ZMPShift4EndY  =  0.00;
  */
#if 0
#if 0
  m_ZMPNeutralPosition[0] = 0.00949035; // 0.015 according to Bjorn lol !
  m_ZMPNeutralPosition[1] = 0.0;
#else
  m_ZMPNeutralPosition[0] = 0.00709014; // 0.015 according to Bjorn lol !
  m_ZMPNeutralPosition[1] = 0.00381147;
#endif
#else
  m_ZMPNeutralPosition[0] = 0.0;
  m_ZMPNeutralPosition[1] = 0.0;

#endif

  MAL_MATRIX_RESIZE(m_CurrentSupportFootPosition,3,3);
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      if (i!=j)
	m_CurrentSupportFootPosition(i,j)= 0.0;
      else
	m_CurrentSupportFootPosition(i,j)= 1.0;
  
  // Create the window for the filter.
  double T=0.050; // Arbritrary from Kajita's San Matlab files.
  int n=0;
  double sum=0,tmp=0;

  n = (int)floor(T/m_SamplingPeriod);
  m_ZMPFilterWindow.resize(n+1);
  for(int i=0;i<n+1;i++)
    {
      tmp =sin((M_PI*i)/n);
      m_ZMPFilterWindow[i]=tmp*tmp;
    }

  for(int i=0;i<n+1;i++)
    sum+= m_ZMPFilterWindow[i];

  for(int i=0;i<n+1;i++)
    m_ZMPFilterWindow[i]/= sum;

  // Prepare size of the matrix used in on-line walking
  MAL_MATRIX_RESIZE(m_vdiffsupppre,2,1);
  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(m_vdiffsupppre);i++)
    m_vdiffsupppre(i,0) = 0.0;

  RESETDEBUG4("DebugDataRFPos.txt");
}

ZMPDiscretization::~ZMPDiscretization()
{

  if (m_PolynomeX!=0)
    delete m_PolynomeX;
  
  if (m_PolynomeY!=0)
    delete m_PolynomeY;

  if (m_PolynomeZ!=0)
    delete m_PolynomeZ;

  if (m_PolynomeOmega!=0)
    delete m_PolynomeOmega;

  if (m_PolynomeOmega2!=0)
    delete m_PolynomeOmega2;

  if (m_PolynomeTheta!=0)
    delete m_PolynomeTheta;

  if (m_PolynomeZMPTheta!=0)
    delete m_PolynomeZMPTheta;

 
  
}

float ZMPDiscretization::GetTSingleSupport()
{
  return m_Tsingle;
}

void  ZMPDiscretization::SetTSingleSupport(float af)
{
  m_Tsingle = af;
}

void ZMPDiscretization::SetOmega(float anOmega)
{
  m_Omega = anOmega;
}

float ZMPDiscretization::GetOmega()
{
  return m_Omega;
}

float ZMPDiscretization::GetTDoubleSupport()
{
  return m_Tdble;
}

double ZMPDiscretization::GetModulationSupportCoefficient()
{
  return m_ModulationSupportCoefficient;
}

void  ZMPDiscretization::SetModulationSupportCoefficient(double af)
{
  m_ModulationSupportCoefficient = af;
}

void  ZMPDiscretization::SetTDoubleSupport(float af)
{
  m_Tdble = af;
}



float ZMPDiscretization::GetSamplingPeriod()
{
  return m_SamplingPeriod;
}

void  ZMPDiscretization::SetSamplingPeriod(float af)
{
  m_SamplingPeriod = af;
}

void ZMPDiscretization::SetTimeWindowPreviewControl(float aTW)
{
  m_PreviewControlTime = aTW;
}

float ZMPDiscretization::GetTimeWindowPreviewControl()
{
  return m_PreviewControlTime;
}

float ZMPDiscretization::GetStepHeight()
{
  return m_StepHeight;
}

void ZMPDiscretization::SetStepHeight(float aStepHeight)
{
  m_StepHeight = aStepHeight;
}

void ZMPDiscretization::UpdateFootPosition(deque<FootAbsolutePosition> &SupportFootAbsolutePositions,
					   deque<FootAbsolutePosition> &NoneSupportFootAbsolutePositions,
					   int CurrentZMPindex, int k,
					   int indexinitial, double ModulatedSingleSupportTime,int StepType)
{
  // Local time
  double LocalTime = k*m_SamplingPeriod;
  double EndOfLiftOff = (m_Tsingle-ModulatedSingleSupportTime)*0.5;
  double StartLanding = EndOfLiftOff + ModulatedSingleSupportTime;

  /*
  cerr << "EndOfLiftOff:"<< EndOfLiftOff << " "
       << "StartLanding:"<< StartLanding << " " ;
  cerr << "LocalTime:" << LocalTime << endl;
  */
 
  // The foot support does not move.
  SupportFootAbsolutePositions[CurrentZMPindex] = 
    SupportFootAbsolutePositions[CurrentZMPindex-1];

  SupportFootAbsolutePositions[CurrentZMPindex].stepType = (-1)*StepType;


  NoneSupportFootAbsolutePositions[CurrentZMPindex].stepType = StepType;
  
  if (LocalTime < EndOfLiftOff)
    {
      // Do not modify x, y and theta while liftoff.
      NoneSupportFootAbsolutePositions[CurrentZMPindex].x = 
	NoneSupportFootAbsolutePositions[indexinitial].x;

      NoneSupportFootAbsolutePositions[CurrentZMPindex].y = 
	NoneSupportFootAbsolutePositions[indexinitial].y;
       
      NoneSupportFootAbsolutePositions[CurrentZMPindex].theta = 
	NoneSupportFootAbsolutePositions[indexinitial].theta;
    }
  else if (LocalTime < StartLanding)
    {
      // DO MODIFY x, y and theta the remaining time.
      NoneSupportFootAbsolutePositions[CurrentZMPindex].x = 
	m_PolynomeX->Compute(LocalTime - EndOfLiftOff) + 
	NoneSupportFootAbsolutePositions[indexinitial].x;

      NoneSupportFootAbsolutePositions[CurrentZMPindex].y = 
	m_PolynomeY->Compute(LocalTime - EndOfLiftOff) + 
	NoneSupportFootAbsolutePositions[indexinitial].y;

      NoneSupportFootAbsolutePositions[CurrentZMPindex].theta = 
	m_PolynomeTheta->Compute(LocalTime - EndOfLiftOff) + 
	NoneSupportFootAbsolutePositions[indexinitial].theta;
	  
    }
  else 
    {
      // Do not modify x, y and theta while landing.
      NoneSupportFootAbsolutePositions[CurrentZMPindex].x = 
	m_PolynomeX->Compute(ModulatedSingleSupportTime) + 
	NoneSupportFootAbsolutePositions[indexinitial].x;
       
      NoneSupportFootAbsolutePositions[CurrentZMPindex].y = 
	m_PolynomeY->Compute(ModulatedSingleSupportTime) + 
	NoneSupportFootAbsolutePositions[indexinitial].y;

      NoneSupportFootAbsolutePositions[CurrentZMPindex].theta = 
	m_PolynomeTheta->Compute(ModulatedSingleSupportTime) + 
	NoneSupportFootAbsolutePositions[indexinitial].theta;
    }

  

  NoneSupportFootAbsolutePositions[CurrentZMPindex].z = 
    m_PolynomeZ->Compute(LocalTime) + 
    NoneSupportFootAbsolutePositions[indexinitial].z;
  
/*if (StepType==3)
    {
       NoneSupportFootAbsolutePositions[CurrentZMPindex].x = 
       mPolynomeStepOverX->Compute(LocalTime) + 
       NoneSupportFootAbsolutePositions[indexinitial].x;
       NoneSupportFootAbsolutePositions[CurrentZMPindex].y = 
       mPolynomeStepOverY->Compute(LocalTime) + 
       NoneSupportFootAbsolutePositions[indexinitial].y;
       NoneSupportFootAbsolutePositions[CurrentZMPindex].z = 
       mPolynomeStepOverZ->Compute(LocalTime) + 
       NoneSupportFootAbsolutePositions[indexinitial].z;

       
    }
      else
    { 
      NoneSupportFootAbsolutePositions[CurrentZMPindex].z = 
      m_PolynomeZ->Compute(LocalTime) + 
      NoneSupportFootAbsolutePositions[indexinitial].z;
    }

*/

  bool ProtectionNeeded=false;

  // Treat Omega with the following strategy:
  // First treat the lift-off.
  if (LocalTime<EndOfLiftOff)
    {
      //cerr << " Case 1 " ;
      NoneSupportFootAbsolutePositions[CurrentZMPindex].omega = 
	m_PolynomeOmega->Compute(LocalTime) ;      
      ProtectionNeeded=true;
    }
  // Prepare for the landing.
  else if (LocalTime<StartLanding)
    {
      //      cerr << " Case 2 " ;
      NoneSupportFootAbsolutePositions[CurrentZMPindex].omega = 
	m_Omega - m_PolynomeOmega2->Compute(LocalTime-EndOfLiftOff);
    }
  // Realize the landing.
  else 
    {
      //      cerr << " Case 3 " ;
      NoneSupportFootAbsolutePositions[CurrentZMPindex].omega = 
	m_PolynomeOmega->Compute(LocalTime - StartLanding)  - m_Omega;
      ProtectionNeeded=true;
    }

  double dFX=0,dFY=0,dFZ=0;
  double lOmega = NoneSupportFootAbsolutePositions[CurrentZMPindex].omega*M_PI/180.0;
  double lTheta = NoneSupportFootAbsolutePositions[CurrentZMPindex].theta*M_PI/180.0;

  //  if (ProtectionNeeded)
  {
    // Make sure the foot is not going inside the floor.
    double dX=0,Z1=0,Z2=0,X1=0,X2=0;
    double B=m_FootB,H=m_FootH,F=m_FootF; 

    if (lOmega<0)
      {
	X1 = B*cos(-lOmega);
	X2 = H*sin(-lOmega);
	Z1 = H*cos(-lOmega);
	Z2 = B*sin(-lOmega);
	dX = -(B - X1 + X2);
	dFZ = Z1 + Z2 - H;
      }
    else
      {
	X1 = F*cos(lOmega);
	X2 = H*sin(lOmega);
	Z1 = H*cos(lOmega);
	Z2 = F*sin(lOmega);
	dX = (F - X1 + X2);
	dFZ = Z1 + Z2 - H;
      }
    double c = cos(lTheta);
    double s = sin(lTheta);
    dFX = c*dX;
    dFY = s*dX;
  }

#if _DEBUG_4_ACTIVATED_
  ofstream aoflocal;
  aoflocal.open("Corrections.dat",ofstream::app);
  aoflocal << dFX << " " << dFY << " " << dFZ << " " << lOmega << endl;
  aoflocal.close();
#endif

  // Modification of the foot position.
  NoneSupportFootAbsolutePositions[CurrentZMPindex].x += dFX;
  NoneSupportFootAbsolutePositions[CurrentZMPindex].y += dFY;
  NoneSupportFootAbsolutePositions[CurrentZMPindex].z += dFZ;

  //cerr<< NoneSupportFootAbsolutePositions[CurrentZMPindex].omega << endl;
}


void ZMPDiscretization::GetZMPDiscretization(deque<ZMPPosition> & FinalZMPPositions,
					     deque<FootAbsolutePosition> &SupportFootAbsolutePositions,
					     deque<RelativeFootPosition> &RelativeFootPositions,
					     deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					     deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					     deque<FootAbsolutePosition> &LeftHandAbsolutePositions,
					     deque<FootAbsolutePosition> &RightHandAbsolutePositions, 
					     double Xmax,
					     MAL_S3_VECTOR(& lStartingCOMPosition,double),
					     FootAbsolutePosition & InitLeftFootAbsolutePosition,
					     FootAbsolutePosition & InitRightFootAbsolutePosition)
{
  ODEBUG4("Step 1","DebugData.txt");
  deque<ZMPPosition> ZMPPositions;
  double CurrentTime=0;

  double TimeForThisFootPosition=0.0;
  ZMPPositions.clear();
  unsigned int AddArraySize=0;
  MAL_MATRIX_DIM(CurrentSupportFootPosition,double,3,3);
  FootAbsolutePosition CurrentLeftFootAbsPos, CurrentRightFootAbsPos;
  FootAbsolutePosition CurrentRightHandAbsPos, CurrentLeftHandAbsPos;

  MAL_MATRIX_DIM(MM,double,2,2);
  MAL_MATRIX_DIM(Orientation,double,2,2);
  MAL_MATRIX_DIM(v,double,2,1);
  MAL_MATRIX_DIM(v2,double,2,1);
  MAL_MATRIX_DIM(vrel,double,2,1);
  MAL_MATRIX_DIM(vdiffsupp,double,2,1);
  MAL_MATRIX_DIM(vdiffsupppre,double,2,1);

  int CurrentZMPindex=0;
  double CurrentAbsTheta=0.0;
  double NextTheta=0, NextZMPTheta=0, BufferNextZMPTheta=0,
    CurrentTheta=0,RelTheta=0;
  int WhoIsSupportFoot;
  double GainX=0.0;

  ofstream DebugFile;
  //DebugFile.open("DebugData.txt",ofstream::app);
  //DebugFile << " GetZMPDiscretization: Step 1 " << endl;
  //DebugFile.close();

  // Initialize position of the current support foot.
  CurrentSupportFootPosition(0,0) = 1;   CurrentSupportFootPosition(0,1) = 0;   CurrentSupportFootPosition(0,2) = 0;
  CurrentSupportFootPosition(1,0) = 0;   CurrentSupportFootPosition(1,1) = 1;   CurrentSupportFootPosition(1,2) = 0;
  CurrentSupportFootPosition(2,0) = 0;   CurrentSupportFootPosition(2,1) = 0;   CurrentSupportFootPosition(2,2) = 1;

  
  // Initialize position of the feet.  
  CurrentLeftFootAbsPos = InitLeftFootAbsolutePosition;
  CurrentLeftFootAbsPos.time = 0.0;
  
  ODEBUG("CurrentLeftFootAbsPos.y: " << CurrentLeftFootAbsPos.y);
  CurrentRightFootAbsPos = InitRightFootAbsolutePosition;
  CurrentRightFootAbsPos.time = 0.0;



  CurrentLeftHandAbsPos.x = 0.0;
  CurrentRightHandAbsPos.x = 0.0;

  // V pre is the difference between 
  // the current support position and the precedent.

  // Initialize who is support foot.
  if (RelativeFootPositions[0].sy < 0 )
    {
      WhoIsSupportFoot = -1;//Right
      vdiffsupppre(0,0) = 0;
      vdiffsupppre(1,0) = 2.0 * RelativeFootPositions[0].sy;
    }
  else 
    {
      WhoIsSupportFoot = 1;// Left
      vdiffsupppre(0,0) = 0;
      vdiffsupppre(1,0) = 2.0 * RelativeFootPositions[0].sy;
    }


  ODEBUG4("Step 2","DebugData.txt");

  // Initialization of the ZMP position (stable values during the Preview control time window).
  AddArraySize = (int)(m_PreviewControlTime/m_SamplingPeriod);
  ODEBUG4(AddArraySize,"DebugData.txt");
  ZMPPositions.resize(AddArraySize);
  LeftFootAbsolutePositions.resize(AddArraySize);
  RightFootAbsolutePositions.resize(AddArraySize);
  LeftHandAbsolutePositions.resize(AddArraySize);
  RightHandAbsolutePositions.resize(AddArraySize);

  // Also very important for the initialization: reshape the ZMP reference for a smooth starting.
  ODEBUG3("lStartingCOMPosition :" << lStartingCOMPosition);
#if 1
  double startingZMPREF[2] = { lStartingCOMPosition(0), lStartingCOMPosition(1)};
#else
  double startingZMPREF[2] = { 0.0, 0.0};
#endif
  double finalZMPREF[2] = {m_ZMPNeutralPosition[0],m_ZMPNeutralPosition[1]};

  ODEBUG4("Step 2.5","DebugData.txt");
  for(unsigned int i=0;i<ZMPPositions.size();i++)
    {
      double coef = (float)i/(float)ZMPPositions.size();
      // Set ZMP positions.

      // Smooth ramp
      ZMPPositions[CurrentZMPindex].px = startingZMPREF[0] + (finalZMPREF[0] - startingZMPREF[0])*coef;
      ZMPPositions[CurrentZMPindex].py = startingZMPREF[1] + (finalZMPREF[1] - startingZMPREF[1])*coef;

      ZMPPositions[CurrentZMPindex].theta = 0.0;
      ZMPPositions[CurrentZMPindex].time = CurrentTime;
      ZMPPositions[CurrentZMPindex].stepType = 0;

      // Set Left Foot positions.
      LeftFootAbsolutePositions[CurrentZMPindex] = CurrentLeftFootAbsPos;
      RightFootAbsolutePositions[CurrentZMPindex] = CurrentRightFootAbsPos;

      LeftFootAbsolutePositions[CurrentZMPindex].time = 
	RightFootAbsolutePositions[CurrentZMPindex].time = CurrentTime;

      LeftFootAbsolutePositions[CurrentZMPindex].stepType = 
	RightFootAbsolutePositions[CurrentZMPindex].stepType = 0;
	
      LeftHandAbsolutePositions[CurrentZMPindex] = CurrentLeftHandAbsPos;
      RightHandAbsolutePositions[CurrentZMPindex] = CurrentRightHandAbsPos;

      CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
      /*
	ofstream aoflocal;
	aoflocal.open("Corrections.dat",ofstream::app);
	aoflocal << "0.0 0.0 0.0 "<<endl;
	aoflocal.close();
      */	  
    }
  //  cout << ZMPPositions.size() << " " << CurrentZMPindex << endl;
  // Initialization of the foot position.
  ODEBUG4("Step 2.75 " << RelativeFootPositions.size() << " " 
	  << SupportFootAbsolutePositions.size(),"DebugData.txt");
  double c,s;
  double TimeFirstPhase=0;
  //  double TimeSndPhase=0;
  SupportFootAbsolutePositions.clear();
  ODEBUG4("Step 2.87.5 ","DebugData.txt");
  SupportFootAbsolutePositions.resize(RelativeFootPositions.size()-1);
  int currentsize;
  int SupportFootInsideTurn=0;

  ODEBUG4("Step 3 "<< RelativeFootPositions.size(),"DebugData.txt");

  for(unsigned int i=0;i<RelativeFootPositions.size()-1;i++)
    {
      if (RelativeFootPositions[i+1].DStime!=0.0)
	{	
	  m_Tdble =RelativeFootPositions[i+1].DStime; 
	  m_Tsingle =RelativeFootPositions[i+1].SStime; 
	}
      /*
	if (WhoIsSupportFoot==1)
	cout << "Left" << endl;
	else 
	cout << "Right" << endl;
      */
      // Compute on the direction of the support foot.
      double stheta = sin(RelativeFootPositions[i+1].theta*M_PI/180.0);     
      // Go to the left
      if (stheta>0)
	{
	  if (WhoIsSupportFoot==1)
	    // The support foot is inside the turn
	    SupportFootInsideTurn=1;
	  else 
	    // The support foot is outside the turn
	    SupportFootInsideTurn=0;
	}
      // Go to the right
      else if (stheta<0)
	{
	  if (WhoIsSupportFoot==1)
	    // The support foot is outside the turn.
	    SupportFootInsideTurn=0;
	  else 
	    // The support foot is inside the turn.
	    SupportFootInsideTurn=1;
	}
      else SupportFootInsideTurn=0;

      // Test if the motion in the X direction is nil.
      if (RelativeFootPositions[i].sx==0)
	{
	  // Make the difference between the first step and the last one.
	  if (i==0)
	    TimeFirstPhase =  m_Tdble/2.0;
	  else 
	    TimeFirstPhase = m_Tdble;
	  
	}
      else
	TimeFirstPhase = m_Tdble;

      TimeForThisFootPosition = TimeFirstPhase+ m_Tsingle;
      ODEBUG4("TimeFirstPhase: " << TimeFirstPhase << " m_Tsingle: " << m_Tsingle,"DebugData.txt");
      // Compute the size of cells to add inside the array.
      AddArraySize = (unsigned int)(TimeForThisFootPosition/m_SamplingPeriod);

      currentsize = ZMPPositions.size();
      ZMPPositions.resize(currentsize+AddArraySize);
      LeftFootAbsolutePositions.resize(currentsize+AddArraySize);
      RightFootAbsolutePositions.resize(currentsize+AddArraySize);
      LeftHandAbsolutePositions.resize(currentsize+AddArraySize);
      RightHandAbsolutePositions.resize(currentsize+AddArraySize);
  
      CurrentAbsTheta+= RelativeFootPositions[i].theta;
      CurrentAbsTheta = fmod(CurrentAbsTheta,180.0);
      // Computes the new ABSOLUTE position of the supporting foot .
      c = cos(RelativeFootPositions[i].theta*M_PI/180.0);
      s = sin(RelativeFootPositions[i].theta*M_PI/180.0);
      MM(0,0) = c;      MM(0,1) = -s;
      MM(1,0) = s;      MM(1,1) = c;
      for(int k=0;k<2;k++)
	for(int l=0;l<2;l++)
	  Orientation(k,l) = CurrentSupportFootPosition(k,l);

      v(0,0) = RelativeFootPositions[i].sx;
      v(1,0) = RelativeFootPositions[i].sy;
      
      Orientation = MAL_RET_A_by_B(MM , Orientation);
      v2 = MAL_RET_A_by_B(Orientation, v);

      for(int k=0;k<2;k++)
	for(int l=0;l<2;l++)
	  CurrentSupportFootPosition(k,l) = Orientation(k,l);

      for(int k=0;k<2;k++)
	CurrentSupportFootPosition(k,2) += v2(k,0);
      
      SupportFootAbsolutePositions[i].x = CurrentSupportFootPosition(0,2);
      SupportFootAbsolutePositions[i].y = CurrentSupportFootPosition(1,2);
      SupportFootAbsolutePositions[i].theta = CurrentAbsTheta;

      // First Phase of the step cycle.
      unsigned int SizeOf1stPhase = (unsigned int)(TimeFirstPhase/m_SamplingPeriod);
      double px0,py0,delta_x,delta_y;
      px0 = ZMPPositions[CurrentZMPindex-1].px;
      py0 = ZMPPositions[CurrentZMPindex-1].py;

      MAL_VECTOR_DIM(ZMPInFootCoordinates,double,3);
      
      ZMPInFootCoordinates[0] = m_ZMPNeutralPosition[0];
      ZMPInFootCoordinates[1] = m_ZMPNeutralPosition[1];
      ZMPInFootCoordinates[2] = 1.0;

      MAL_VECTOR_DIM(ZMPInWorldCoordinates,double,3);
      
      ZMPInWorldCoordinates = MAL_RET_A_by_B(CurrentSupportFootPosition,ZMPInFootCoordinates); 

      delta_x = (ZMPInWorldCoordinates(0) - px0)/SizeOf1stPhase;
      delta_y = (ZMPInWorldCoordinates(1) - py0)/SizeOf1stPhase;

      ODEBUG4("Step 4 TimeForThisFootPosition " << TimeForThisFootPosition,"DebugData.txt");  

      bool DoIt = 1; 
      if (DoIt)
	{
	  if (RelativeFootPositions[i+1].stepType == 3)
	    {
	      //delta_x = (CurrentSupportFootPosition(0,2)+m_ZMPShift3Begin - px0)/SizeOf1stPhase;
	      delta_x = (CurrentSupportFootPosition(0,2)+m_ZMPShift[0] - px0)/SizeOf1stPhase;
	      //delta_y = (CurrentSupportFootPosition(1,2)+(WhoIsSupportFoot)*m_ZMPShift3BeginY - py0)/SizeOf1stPhase;
	      delta_y = (CurrentSupportFootPosition(1,2) - py0)/SizeOf1stPhase;
	      
	    }
	  if (RelativeFootPositions[i+1].stepType == 4)
	    {
	      delta_x = (CurrentSupportFootPosition(0,2)+m_ZMPShift[2] - px0)/SizeOf1stPhase;
	      delta_y = (CurrentSupportFootPosition(1,2)- py0)/SizeOf1stPhase;
	      //delta_x = (CurrentSupportFootPosition(0,2)+m_ZMPShift4Begin - px0)/SizeOf1stPhase;
	      //delta_y = (CurrentSupportFootPosition(1,2)+(WhoIsSupportFoot)*m_ZMPShift3BeginY - py0)/SizeOf1stPhase;
	    }
	  
	  if (RelativeFootPositions[i+1].stepType == 5)
	    {
	      delta_x = (CurrentSupportFootPosition(0,2)-(m_ZMPShift[0] +m_ZMPShift[2]+
							  m_ZMPShift[1] +m_ZMPShift[3]) - px0)/SizeOf1stPhase;
	      delta_y = (CurrentSupportFootPosition(1,2)- py0)/SizeOf1stPhase;
	      //delta_x = (CurrentSupportFootPosition(0,2)-(m_ZMPShift3Begin +
	      // m_ZMPShift4Begin+m_ZMPShift3End + m_ZMPShift4End) - px0)/SizeOf1stPhase;
	      //delta_y = (CurrentSupportFootPosition(1,2)-(WhoIsSupportFoot)*
	      //(m_ZMPShift3BeginY + m_ZMPShift4BeginY+m_ZMPShift3EndY + m_ZMPShift4EndY) - py0)/SizeOf1stPhase;
	    }
	
	}   	

      ODEBUG4(" GetZMPDiscretization: Step 5 " << currentsize << " " << AddArraySize << " " ,"DebugData.txt");
      
      for(unsigned int k=0;k<SizeOf1stPhase;k++)
	{
	  ZMPPositions[CurrentZMPindex].px = 
	    ZMPPositions[CurrentZMPindex-1].px + delta_x;
	  ZMPPositions[CurrentZMPindex].py = 
	    ZMPPositions[CurrentZMPindex-1].py + delta_y;

	  ZMPPositions[CurrentZMPindex].theta =
	    ZMPPositions[CurrentZMPindex-1].theta;


	  ZMPPositions[CurrentZMPindex].time = CurrentTime;
	
	  ZMPPositions[CurrentZMPindex].stepType = RelativeFootPositions[i+1].stepType+10;
	  
	  // Right now the foot is not moving during the double support
	  // TO DO: whatever you need to do ramzi....
	  LeftFootAbsolutePositions[CurrentZMPindex] = 
	    LeftFootAbsolutePositions[CurrentZMPindex-1];

	  // WARNING : This assume that you are walking on a plane.
	  LeftFootAbsolutePositions[CurrentZMPindex].z = 0.0;
		  
	  RightFootAbsolutePositions[CurrentZMPindex] = 
	    RightFootAbsolutePositions[CurrentZMPindex-1];

	  // WARNING : This assume that you are walking on a plane.
	  RightFootAbsolutePositions[CurrentZMPindex].z = 0.0;

	  LeftHandAbsolutePositions[CurrentZMPindex] = 
	    LeftHandAbsolutePositions[CurrentZMPindex-1];
	  RightHandAbsolutePositions[CurrentZMPindex] = 
	    RightHandAbsolutePositions[CurrentZMPindex-1];
	    
	  LeftFootAbsolutePositions[CurrentZMPindex].time = 
	    RightFootAbsolutePositions[CurrentZMPindex].time = CurrentTime;

	  LeftFootAbsolutePositions[CurrentZMPindex].stepType = 
	    RightFootAbsolutePositions[CurrentZMPindex].stepType = RelativeFootPositions[i+1].stepType+10;
	  /*
	    ofstream aoflocal;
	    aoflocal.open("Corrections.dat",ofstream::app);
	    aoflocal << "0.0 0.0 0.0 "<<endl;
	    aoflocal.close();
	  */
	  CurrentTime += m_SamplingPeriod;
	  CurrentZMPindex++;
	}
      //-- End Of First phase.
      
      // Second Phase of the step cycle.
      
      // Compute relative feet position for the next phase.
      float lStepHeight=0;
      if (i<RelativeFootPositions.size()-1)
	{
	  NextTheta=RelativeFootPositions[i+1].theta;
	  if (SupportFootInsideTurn==1)
	    {
	      BufferNextZMPTheta=NextTheta;
	      NextZMPTheta = 0;
	    }
	  else
	    {
	      NextZMPTheta = NextTheta+BufferNextZMPTheta;
	      BufferNextZMPTheta=0;
	    }
	  RelTheta = NextTheta+CurrentTheta;
	  lStepHeight = m_StepHeight;

	  c = cos(NextTheta*M_PI/180.0);
	  s = sin(NextTheta*M_PI/180.0);
	  MM(0,0) = c;      MM(0,1) = -s;
	  MM(1,0) = s;      MM(1,1) = c;
	  
	  Orientation = MAL_RET_A_by_B(MM,Orientation);

	  v(0,0) = RelativeFootPositions[i+1].sx;
	  v(1,0) = RelativeFootPositions[i+1].sy;
	  vdiffsupp = MAL_RET_A_by_B(Orientation,v);
	  
	  vrel = vdiffsupp + vdiffsupppre;

	  // Compute relative feet orientation for the next step
	}
      else
	{
	  vrel(0,0)= 0.0;
	  vrel(1,0)= 0.0;
	  RelTheta= 0.0;
	  NextTheta=0.0;
	  lStepHeight = 0.0;
	}
#if 0
      cout << "vrel: " << vrel(0,0) << " " << vrel(1,0) << endl;
      cout << "vdiffsupp: " << vdiffsupp(0,0) << " " << vdiffsupp(1,0) << endl;
      cout << "vdiffsupppre: " << vdiffsupppre(0,0) << " " << vdiffsupppre(1,0) << endl;
#endif

      ODEBUG4(" GetZMPDiscretization: Step 6 " << ZMPPositions.size() << " " ,"DebugData.txt");


      vdiffsupppre = vdiffsupp;
      
      CurrentTheta = NextTheta;
      
      GainX = Xmax/0.15; // Should put Step max instead of 0.3
      // Create the polynomes for the none-support foot.
      // Change 08/12/2005: Speed up the modification of X and Y
      // for vertical landing of the foot (Kajita-San's trick n 1)
      //   double ModulationSupportCoefficient = 0.9;
      double ModulatedSingleSupportTime = m_Tsingle * m_ModulationSupportCoefficient;
      double EndOfLiftOff = (m_Tsingle-ModulatedSingleSupportTime)*0.5;

      m_PolynomeX->SetParameters(ModulatedSingleSupportTime,vrel(0,0));
      m_PolynomeY->SetParameters(ModulatedSingleSupportTime,vrel(1,0));
      m_PolynomeZ->SetParameters(m_Tsingle,lStepHeight);
      m_PolynomeTheta->SetParameters(ModulatedSingleSupportTime,RelTheta);
      m_PolynomeOmega->SetParameters(EndOfLiftOff,m_Omega);
      m_PolynomeOmega2->SetParameters(ModulatedSingleSupportTime,2*m_Omega);
      //m_PolynomeZMPTheta->SetParameters(m_Tsingle,NextZMPTheta);
      m_PolynomeZMPTheta->SetParameters(m_Tsingle,NextTheta);
      unsigned int SizeOfSndPhase = (unsigned int)(m_Tsingle/m_SamplingPeriod);
      int indexinitial = CurrentZMPindex-1;
      int SignRHAND=1, SignLHAND=1;

      /*//polynomial planning for the stepover 
     
	if (RelativeFootPositions[i].stepType==3)
	{
	StepOverPolyPlanner(RelativeFootPositions[i].stepType);
	};
*/	double px02,py02;
      px02 = ZMPPositions[CurrentZMPindex-1].px;
      py02 = ZMPPositions[CurrentZMPindex-1].py;

      for(unsigned int k=0;k<SizeOfSndPhase;k++)
	{

	  MAL_VECTOR_DIM(ZMPInFootCoordinates,double,3);

	  ZMPInFootCoordinates[0] = m_ZMPNeutralPosition[0];
	  ZMPInFootCoordinates[1] = m_ZMPNeutralPosition[1];
	  ZMPInFootCoordinates[2] = 1.0;
	  
	  MAL_VECTOR_DIM(ZMPInWorldCoordinates,double,3);

	  ZMPInWorldCoordinates = MAL_RET_A_by_B(CurrentSupportFootPosition , ZMPInFootCoordinates); 

	  ODEBUG4("CSFP: " << CurrentSupportFootPosition << endl <<
		   "ZMPiWC"  << ZMPInWorldCoordinates << endl, "DebugData.txt");	
		
	  ZMPPositions[CurrentZMPindex].px = ZMPInWorldCoordinates(0);
	  ZMPPositions[CurrentZMPindex].py = ZMPInWorldCoordinates(1);
	  ZMPPositions[CurrentZMPindex].time = CurrentTime;
	
	  if (DoIt)
	    {
	      if ((RelativeFootPositions[i+1].stepType == 3)||(RelativeFootPositions[i+1].stepType == 4))
		{
		
		
		  if (RelativeFootPositions[i+1].stepType == 3)
		    {	delta_x = (CurrentSupportFootPosition(0,2)+m_ZMPShift[1] - px02)/SizeOfSndPhase;
		    delta_y = (CurrentSupportFootPosition(1,2) - py02)/SizeOfSndPhase;
		    //delta_x = (CurrentSupportFootPosition(0,2)+m_ZMPShift3End - px02)/SizeOfSndPhase;
		    //delta_y = (CurrentSupportFootPosition(1,2)+
		    // (WhoIsSupportFoot)*m_ZMPShift3EndY - py02)/SizeOfSndPhase;
		    }
		  else
		    {
		      delta_x = (CurrentSupportFootPosition(0,2)+m_ZMPShift[3] - px02)/SizeOfSndPhase;
		      delta_y = (CurrentSupportFootPosition(1,2)- py02)/SizeOfSndPhase;
		      //delta_x = (CurrentSupportFootPosition(0,2)+m_ZMPShift4End - px02)/SizeOfSndPhase;
		      //delta_y = (CurrentSupportFootPosition(1,2)+
		      // (WhoIsSupportFoot)*m_ZMPShift4EndY - py02)/SizeOfSndPhase;
		    }
	
		  ZMPPositions[CurrentZMPindex].px = 
		    ZMPPositions[CurrentZMPindex-1].px + delta_x;
		  ZMPPositions[CurrentZMPindex].py = 
		    ZMPPositions[CurrentZMPindex-1].py + delta_y;
		}

	    }
	


	  /*	  

		  if (RelativeFootPositions[i+1].stepType == 3)
		  {
		  ZMPPositions[CurrentZMPindex].px = ZMPPositions[CurrentZMPindex].px + 0.03;
		  }
		  if (RelativeFootPositions[i+1].stepType == 4)
		  {
		  ZMPPositions[CurrentZMPindex].px = ZMPPositions[CurrentZMPindex].px + 0.02;
		  }
	  */	    
	  ZMPPositions[CurrentZMPindex].theta = m_PolynomeZMPTheta->Compute(k*m_SamplingPeriod) + 
	    ZMPPositions[indexinitial].theta;

	  ZMPPositions[CurrentZMPindex].stepType = WhoIsSupportFoot*RelativeFootPositions[i].stepType;
	  
	  if (WhoIsSupportFoot==1)
	    {
	      UpdateFootPosition(LeftFootAbsolutePositions,
				 RightFootAbsolutePositions,
				 CurrentZMPindex,k,indexinitial,
				 ModulatedSingleSupportTime,
				 RelativeFootPositions[i+1].stepType);
	      SignLHAND=1;
	      SignRHAND=-1;
	    }
	  else
	    {
	      UpdateFootPosition(RightFootAbsolutePositions,
				 LeftFootAbsolutePositions,
				 CurrentZMPindex,k,indexinitial,
				 ModulatedSingleSupportTime,
				 RelativeFootPositions[i+1].stepType);
	      SignLHAND=-1;
	      SignRHAND=1;
	      
	    }

  
	  // Move the hands with the linear relation with the foot.
	  // The direction are opposite.
	  RightHandAbsolutePositions[CurrentZMPindex].x =
	    SignRHAND * GainX * m_PolynomeX->Compute(k*m_SamplingPeriod) +
	    RightHandAbsolutePositions[indexinitial].x;
	  LeftHandAbsolutePositions[CurrentZMPindex].x =
	    SignLHAND * GainX * m_PolynomeX->Compute(k*m_SamplingPeriod) +
	    LeftHandAbsolutePositions[indexinitial].x;

	  LeftFootAbsolutePositions[CurrentZMPindex].time = 
	    RightFootAbsolutePositions[CurrentZMPindex].time = CurrentTime;
	  
	  CurrentTime += m_SamplingPeriod;
	  CurrentZMPindex++;
	}
      // cout << SignLHAND << " " << SignRHAND << endl;
      //-- End Of Second phase.
      //      cout << ZMPPositions.size() << " " << CurrentZMPindex << endl;

      if (WhoIsSupportFoot==1)
	WhoIsSupportFoot = -1;//Right
      else 
	WhoIsSupportFoot = 1;// Left
	  
    }
  

  // Deal with the end phase of the walking.
  TimeForThisFootPosition = m_Tdble/2.0;
  AddArraySize = (unsigned int)(TimeForThisFootPosition/m_SamplingPeriod);
  
  currentsize = ZMPPositions.size();
  ZMPPositions.resize(currentsize+AddArraySize);
  LeftFootAbsolutePositions.resize(currentsize+AddArraySize);
  RightFootAbsolutePositions.resize(currentsize+AddArraySize);
  LeftHandAbsolutePositions.resize(currentsize+AddArraySize);
  RightHandAbsolutePositions.resize(currentsize+AddArraySize);
  
  ODEBUG4(" GetZMPDiscretization: Step 7 " << currentsize << " " << AddArraySize,"DebugData.txt");


  unsigned int SizeOfEndPhase = (unsigned int)(m_Tdble/(2*m_SamplingPeriod));
  double px0,py0,delta_x,delta_y;
  double pxf=0,pyf=0;
  px0 = ZMPPositions[CurrentZMPindex-1].px;
  py0 = ZMPPositions[CurrentZMPindex-1].py;
  //  int lindex = SupportFootAbsolutePositions.size()-1;

  // We assume that the last positon of the ZMP
  // will the middle of the two last position
  // of the support foot.
  pxf = (LeftFootAbsolutePositions[CurrentZMPindex-1].x+
	 RightFootAbsolutePositions[CurrentZMPindex-1].x)/2.0;
  pyf = (LeftFootAbsolutePositions[CurrentZMPindex-1].y+
	 RightFootAbsolutePositions[CurrentZMPindex-1].y)/2.0;
  //  cout << "PX: " << pxf << " PY: " << pyf<< endl;
  
  delta_x = (pxf - px0)/(double)SizeOfEndPhase;
  delta_y = (pyf - py0)/(double)SizeOfEndPhase;
  
  for(unsigned int k=0;k<SizeOfEndPhase;k++)
    {
      ZMPPositions[CurrentZMPindex].px = 
	ZMPPositions[CurrentZMPindex-1].px + delta_x;
      ZMPPositions[CurrentZMPindex].py = 
	ZMPPositions[CurrentZMPindex-1].py + delta_y;
      ZMPPositions[CurrentZMPindex].time = CurrentTime;
      ZMPPositions[CurrentZMPindex].theta = 
	ZMPPositions[CurrentZMPindex-1].theta;

      ZMPPositions[CurrentZMPindex].stepType = 10; 

      LeftFootAbsolutePositions[CurrentZMPindex] = 
	LeftFootAbsolutePositions[CurrentZMPindex-1];
      RightFootAbsolutePositions[CurrentZMPindex] = 
	RightFootAbsolutePositions[CurrentZMPindex-1];

      LeftHandAbsolutePositions[CurrentZMPindex] = 
	LeftHandAbsolutePositions[CurrentZMPindex-1];
      RightHandAbsolutePositions[CurrentZMPindex] = 
	RightHandAbsolutePositions[CurrentZMPindex-1];
      
      LeftFootAbsolutePositions[CurrentZMPindex].time = 
	RightFootAbsolutePositions[CurrentZMPindex].time = CurrentTime;

      LeftFootAbsolutePositions[CurrentZMPindex].stepType = 
	RightFootAbsolutePositions[CurrentZMPindex].stepType = 10;

      CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
      /*
	cout << "ZMPX: "<<  ZMPPositions[CurrentZMPindex-1].px 
	<< " ZMPY: "<<  ZMPPositions[CurrentZMPindex-1].py  << endl; 
      */

    }
  //  cout << "ZMPX: "<<  ZMPPositions[CurrentZMPindex-1].px 
  //       << " ZMPY: "<<  ZMPPositions[CurrentZMPindex-1].py  << endl; 

  // Added a new phase for exhausting the preview control
  AddArraySize = (int)(3.0*m_PreviewControlTime/m_SamplingPeriod);
  currentsize = ZMPPositions.size();
  ZMPPositions.resize(currentsize+AddArraySize);
  LeftFootAbsolutePositions.resize(currentsize+AddArraySize);
  RightFootAbsolutePositions.resize(currentsize+AddArraySize);
  LeftHandAbsolutePositions.resize(currentsize+AddArraySize);
  RightHandAbsolutePositions.resize(currentsize+AddArraySize);

  ODEBUG4(" GetZMPDiscretization: Step 8 ","DebugData.txt");

  for(unsigned int i=0;i<AddArraySize;i++)
    {  
      ZMPPositions[CurrentZMPindex].px = ZMPPositions[CurrentZMPindex-1].px;
      ZMPPositions[CurrentZMPindex].py = ZMPPositions[CurrentZMPindex-1].py;
      ZMPPositions[CurrentZMPindex].theta = ZMPPositions[CurrentZMPindex-1].theta;
      ZMPPositions[CurrentZMPindex].time = CurrentTime;

      ZMPPositions[CurrentZMPindex].stepType = 10;

      LeftFootAbsolutePositions[CurrentZMPindex] = 
	LeftFootAbsolutePositions[CurrentZMPindex-1];
      RightFootAbsolutePositions[CurrentZMPindex] = 
	RightFootAbsolutePositions[CurrentZMPindex-1];

      LeftHandAbsolutePositions[CurrentZMPindex] = 
	LeftHandAbsolutePositions[CurrentZMPindex-1];
      RightHandAbsolutePositions[CurrentZMPindex] = 
	RightHandAbsolutePositions[CurrentZMPindex-1];
      
      LeftFootAbsolutePositions[CurrentZMPindex].time = 
	RightFootAbsolutePositions[CurrentZMPindex].time = CurrentTime;
	
      LeftFootAbsolutePositions[CurrentZMPindex].stepType = 
	RightFootAbsolutePositions[CurrentZMPindex].stepType = 10;
      
      CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
    }

  ODEBUG4(" GetZMPDiscretization: Step 9 " << ZMPPositions.size(),"DebugData.txt");


  FilterZMPRef(ZMPPositions,FinalZMPPositions);
}


void ZMPDiscretization::DumpFootAbsolutePosition(string aFileName,
						 deque<FootAbsolutePosition> &aFootAbsolutePositions)
{
  ofstream aof;
  aof.open(aFileName.c_str(),ofstream::out);
  if (aof.is_open())
    {
      for(unsigned int i=0;i<aFootAbsolutePositions.size();i++)
	{
	  aof << aFootAbsolutePositions[i].time << " " 
	      << aFootAbsolutePositions[i].x << " " 
	      << aFootAbsolutePositions[i].y << " " 
	      << aFootAbsolutePositions[i].z << " " 
	      << aFootAbsolutePositions[i].omega << " " 
	      << aFootAbsolutePositions[i].theta << " " 
	      << aFootAbsolutePositions[i].stepType << " " 
	      << endl;
	}
      aof.close();
    }
  
}

void ZMPDiscretization::DumpDataFiles(string ZMPFileName, string FootFileName,
				      deque<ZMPPosition> & ZMPPositions,
				      deque<FootAbsolutePosition> & SupportFootAbsolutePositions)
{
  ofstream aof;
  aof.open(ZMPFileName.c_str(),ofstream::out);
  if (aof.is_open())
    {
      for(unsigned int i=0;i<ZMPPositions.size();i++)
	{
	  aof << ZMPPositions[i].time << " " << ZMPPositions[i].px << " " << ZMPPositions[i].py << " " << ZMPPositions[i].stepType << " 0.0" <<   endl;
	}
      aof.close();
    }

  aof.open(FootFileName.c_str(),ofstream::out);
  if (aof.is_open())
    {
      for(unsigned int i=0;i<SupportFootAbsolutePositions.size();i++)
	{
	  aof << SupportFootAbsolutePositions[i].x << " " << SupportFootAbsolutePositions[i].y << " " 
	      << SupportFootAbsolutePositions[i].z << " " << 
	    SupportFootAbsolutePositions[i].stepType << " 0.0" <<  endl;
	}
      aof.close();
    }
}

void ZMPDiscretization::FilterZMPRef(deque<ZMPPosition> &ZMPPositionsX,
				     deque<ZMPPosition> &ZMPPositionsY)
{
  int n=0;
  double T=0.050; // Arbritraty fixed from Kajita's San matlab files.
  deque<double> window;

  ZMPPositionsY.resize(ZMPPositionsX.size());
  // Creates window.
  n = (int)floor(T/m_SamplingPeriod);

  // Filter ZMPref.

  // First part of the filter.
  for(int i=0;i<n+1;i++)
    {
      ZMPPositionsY[i] = ZMPPositionsX[i];
    }

  for(unsigned int i=n+1;i<ZMPPositionsX.size();i++)
    {
      double ltmp[2]={0,0};
      for(unsigned int j=0;j<m_ZMPFilterWindow.size();j++)
	{
	  ltmp[0] += m_ZMPFilterWindow[j]*ZMPPositionsX[i-j].px;
	  ltmp[1] += m_ZMPFilterWindow[j]*ZMPPositionsX[i-j].py;
	}

      ZMPPositionsY[i].px = ltmp[0];
      ZMPPositionsY[i].py = ltmp[1];
      ZMPPositionsY[i].theta = ZMPPositionsX[i].theta;
      ZMPPositionsY[i].time = ZMPPositionsX[i].time;
      ZMPPositionsY[i].stepType = ZMPPositionsX[i].stepType;
	
    }
    
}



void ZMPDiscretization::SetZMPShift(vector<double> &ZMPShift)
{
	
  for (unsigned int i=0;i<ZMPShift.size();i++)
    {
      m_ZMPShift[i] = ZMPShift[i];
    }


}

/* Start the online part of ZMP discretization. */

/* Initialiazation of the on-line stacks. */
int ZMPDiscretization::InitOnLine(deque<ZMPPosition> & FinalZMPPositions,					     
				  deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
				  deque<FootAbsolutePosition> &RightFootAbsolutePositions,
				  FootAbsolutePosition & InitLeftFootAbsolutePosition,
				  FootAbsolutePosition & InitRightFootAbsolutePosition,
				  deque<RelativeFootPosition> &RelativeFootPositions,
				  MAL_S3_VECTOR(& lStartingCOMPosition,double))
{
  m_RelativeFootPositions.clear();
  FootAbsolutePosition CurrentLeftFootAbsPos, CurrentRightFootAbsPos;
  RESETDEBUG4("ZMDInitOnLine.txt");
  ODEBUG4("ZMP::InitOnLine - Step 1 ","ZMDInitOnLine.txt");
//   m_RelativeFootPositions.resize(RelativeFootPositions.size());
//   for(unsigned int i=0;i<RelativeFootPositions.size();i++)
//     m_RelativeFootPositions.push_back(RelativeFootPositions[i]);

  // Initialize position of the current support foot.
  m_CurrentSupportFootPosition(0,0) = 1;   m_CurrentSupportFootPosition(0,1) = 0;   m_CurrentSupportFootPosition(0,2) = 0;
  m_CurrentSupportFootPosition(1,0) = 0;   m_CurrentSupportFootPosition(1,1) = 1;   m_CurrentSupportFootPosition(1,2) = 0;
  m_CurrentSupportFootPosition(2,0) = 0;   m_CurrentSupportFootPosition(2,1) = 0;   m_CurrentSupportFootPosition(2,2) = 1;

  ODEBUG4("ZMP::InitOnLine - Step 2 ","ZMDInitOnLine.txt");
  // Initialize position of the feet.  
  CurrentLeftFootAbsPos = InitLeftFootAbsolutePosition;
  CurrentLeftFootAbsPos.time = 0.0;
  
  ODEBUG4("CurrentLeftFootAbsPos.y: " << CurrentLeftFootAbsPos.y, "ZMDInitOnLine.txt");
  CurrentRightFootAbsPos = InitRightFootAbsolutePosition;
  CurrentRightFootAbsPos.time = 0.0;

  // V pre is the difference between 
  // the current support position and the precedent.
  ODEBUG4("ZMP::InitOnLine - Step 2.5 ","ZMDInitOnLine.txt");

  ODEBUG4("ZMP::InitOnLine - Step 3 ","ZMDInitOnLine.txt");
  // Initialization of the ZMP position (stable values during the Preview control time window).
  int AddArraySize = (int)(m_PreviewControlTime/m_SamplingPeriod);
  ODEBUG(AddArraySize);
  FinalZMPPositions.resize(AddArraySize);
  LeftFootAbsolutePositions.resize(AddArraySize);
  RightFootAbsolutePositions.resize(AddArraySize);
  int CurrentZMPindex=0;
  // Also very important for the initialization: reshape the ZMP reference for a smooth starting.
#if 1
  double startingZMPREF[2] = { lStartingCOMPosition(0), lStartingCOMPosition(1)};
#else
  double startingZMPREF[2] = { 0.0, 0.0};
#endif
  double finalZMPREF[2] = {m_ZMPNeutralPosition[0],m_ZMPNeutralPosition[1]};

  ODEBUG4("ZMP::InitOnLine - Step 4 ","ZMDInitOnLine.txt");
  for(unsigned int i=0;i<FinalZMPPositions.size();i++)
    {
      double coef = (float)i/(float)FinalZMPPositions.size();
      // Set ZMP positions.

      // Smooth ramp
      FinalZMPPositions[CurrentZMPindex].px = startingZMPREF[0] + (finalZMPREF[0] - startingZMPREF[0])*coef;
      FinalZMPPositions[CurrentZMPindex].py = startingZMPREF[1] + (finalZMPREF[1] - startingZMPREF[1])*coef;

      FinalZMPPositions[CurrentZMPindex].theta = 0.0;
      FinalZMPPositions[CurrentZMPindex].time = m_CurrentTime;
      FinalZMPPositions[CurrentZMPindex].stepType = 0;

      // Set Left Foot positions.
      LeftFootAbsolutePositions[CurrentZMPindex] = CurrentLeftFootAbsPos;
      RightFootAbsolutePositions[CurrentZMPindex] = CurrentRightFootAbsPos;

      LeftFootAbsolutePositions[CurrentZMPindex].time = 
	RightFootAbsolutePositions[CurrentZMPindex].time = m_CurrentTime;

      LeftFootAbsolutePositions[CurrentZMPindex].stepType = 
	RightFootAbsolutePositions[CurrentZMPindex].stepType = 0;
	
      m_CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
    }
  ODEBUG("InitOnline: FinalZMPPositions.size(): " << FinalZMPPositions.size());
  
  // The first foot when walking dynamically 
  // does not leave the soil, but needs to be treated for the first phase.
  m_RelativeFootPositions.push_back(RelativeFootPositions[0]);
  // Initialize who is support foot.
  if (m_RelativeFootPositions[0].sy < 0 )
    {
      m_vdiffsupppre(0,0) = 0;
      m_vdiffsupppre(1,0) = 2*m_RelativeFootPositions[0].sy; //*2.0
    }
  else 
    {
      m_vdiffsupppre(0,0) = 0;
      m_vdiffsupppre(1,0) =  2*m_RelativeFootPositions[0].sy;//*2.0
    }

  ODEBUG6("InitOnLine","DebugDataRFPos.txt" );
  for(unsigned int i=1;i<RelativeFootPositions.size();i++)
    {
      OnLine(RelativeFootPositions[i],
	     FinalZMPPositions,
	     LeftFootAbsolutePositions,
	     RightFootAbsolutePositions,
	     false);
    }
  m_CurrentTheta=0;
  ODEBUG4("ZMP::InitOnLine: End ","ZMDInitOnLine.txt");


  return RelativeFootPositions.size();
}

void ZMPDiscretization::UpdateCurrentSupportFootPosition(RelativeFootPosition aRFP)
{
  // First orientation
  double c = cos(aRFP.theta*M_PI/180.0);
  double s = sin(aRFP.theta*M_PI/180.0);
  MAL_MATRIX_DIM(MM,double,2,2);
  MAL_MATRIX_DIM(Orientation,double,2,2);

  MM(0,0) = c;      MM(0,1) = -s;
  MM(1,0) = s;      MM(1,1) = c;
  for(int k=0;k<2;k++)
    for(int l=0;l<2;l++)
      Orientation(k,l) = m_CurrentSupportFootPosition(k,l);
  
  // second position.
  MAL_MATRIX_DIM(v,double,2,1);
  MAL_MATRIX_DIM(v2,double,2,1);

  v(0,0) = aRFP.sx;
  v(1,0) = aRFP.sy;
  
  Orientation = MAL_RET_A_by_B(MM , Orientation);
  v2 = MAL_RET_A_by_B(Orientation, v);
  ODEBUG("v :" << v  << " "
	  "v2 : " << v2 << " "
	  "Orientation : " << Orientation << " "
	  "CurrentSupportFootPosition: " << m_CurrentSupportFootPosition );
	  
  
  for(int k=0;k<2;k++)
    for(int l=0;l<2;l++)
      m_CurrentSupportFootPosition(k,l) = Orientation(k,l);
  
  for(int k=0;k<2;k++)
    m_CurrentSupportFootPosition(k,2) += v2(k,0);
}

/* The interface method which returns an appropriate update of the
   appropriate stacks (ZMPRef, FootPosition) depending on the 
   state of the relative steps stack. */
void ZMPDiscretization::OnLine(RelativeFootPosition NewRelativeFootPosition,
			       deque<ZMPPosition> & FinalZMPPositions,					     
			       deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
			       deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
			       bool EndSequence)
{
  deque<ZMPPosition> ZMPPositions;
  deque<FootAbsolutePosition> LeftFootAbsolutePositions;
  deque<FootAbsolutePosition> RightFootAbsolutePositions;
  m_RelativeFootPositions.push_back(NewRelativeFootPosition);
  int WhoIsSupportFoot=1;
  double TimeFirstPhase=0.0;
  int CurrentZMPindex=0;
  MAL_MATRIX_DIM(vdiffsupp,double,2,1);
  MAL_MATRIX_DIM(vrel,double,2,1);

  double lTdble=0.02, lTsingle=0.78;

  ODEBUG6(m_RelativeFootPositions[0].sx << " "  << 
	  m_RelativeFootPositions[0].sy << " " << 
	  m_RelativeFootPositions[0].theta,"DebugDataRFPos.txt" );
  ODEBUG(" m_RelativeFootPositions.size: " <<  m_RelativeFootPositions.size());
  if (m_RelativeFootPositions[1].DStime!=0.0)
    {	
      lTdble =m_RelativeFootPositions[1].DStime; 
      lTsingle =m_RelativeFootPositions[1].SStime; 
    }
  // Compute on the direction of the support foot.
  //  double stheta = sin(m_RelativeFootPositions[1].theta*M_PI/180.0);     

#if 0  
  // Test if the motion in the X direction is nil.
  if (m_RelativeFootPositions[0].sx==0)
    {
	TimeFirstPhase =  lTdble/2.0;
      else 
	TimeFirstPhase = lTdble;
      
    }
  else

#endif
    TimeFirstPhase = lTdble;

  // Initialize who is support foot.
  if (m_RelativeFootPositions[0].sy < 0 )
    WhoIsSupportFoot = -1;//Right
  else 
    WhoIsSupportFoot = 1;// Left

  
  double TimeForThisFootPosition = TimeFirstPhase+ lTsingle;
  ODEBUG4("TimeFirstPhase: " << TimeFirstPhase << " lTsingle: " << lTsingle,"DebugData.txt");
  // Compute the size of cells to add inside the array.
  int AddArraySize = (unsigned int)(TimeForThisFootPosition/m_SamplingPeriod);

  ZMPPositions.resize(AddArraySize);
  LeftFootAbsolutePositions.resize(AddArraySize);
  RightFootAbsolutePositions.resize(AddArraySize);
    
  
  m_CurrentAbsTheta+= m_RelativeFootPositions[0].theta;
  m_CurrentAbsTheta = fmod(m_CurrentAbsTheta,180);

  // Computes the new ABSOLUTE position of the supporting foot .
  UpdateCurrentSupportFootPosition(m_RelativeFootPositions[0]);
  
  // First Phase of the step cycle.
  unsigned int SizeOf1stPhase = (unsigned int)(TimeFirstPhase/m_SamplingPeriod);
  ODEBUG("m_vdiffsupppre : " << m_vdiffsupppre); 
  double px0,py0,theta0, delta_x,delta_y;

  // Initial value to start the new ZMP profile.
  px0 = FinalZMPPositions.back().px;
  py0 = FinalZMPPositions.back().py;
  theta0 = FinalZMPPositions.back().theta;
  
  MAL_VECTOR_DIM(ZMPInFootCoordinates,double,3);
  
  ZMPInFootCoordinates[0] = m_ZMPNeutralPosition[0];
  ZMPInFootCoordinates[1] = m_ZMPNeutralPosition[1];
  ZMPInFootCoordinates[2] = 1.0;
  
  MAL_VECTOR_DIM(ZMPInWorldCoordinates,double,3);
  
  ZMPInWorldCoordinates = MAL_RET_A_by_B(m_CurrentSupportFootPosition, ZMPInFootCoordinates); 
  
  delta_x = (ZMPInWorldCoordinates(0) - px0)/SizeOf1stPhase;
  delta_y = (ZMPInWorldCoordinates(1) - py0)/SizeOf1stPhase;
  
  ODEBUG("delta_x :"<< delta_x << " delta_y : " << delta_y << " m_CurrentSFP: " <<
	  m_CurrentSupportFootPosition << " ZMPInFC : " << 
	  ZMPInFootCoordinates << " ZMPinWC : " << 
	  ZMPInWorldCoordinates
	  );
  ODEBUG4("Step 4 TimeForThisFootPosition " << TimeForThisFootPosition,"DebugData.txt");  
  
  // ZMP profile is changed if the stepping over is on, and then 
  // depends on the phase during stepping over.
  bool DoIt = 1; 
  if (DoIt)
    {
      if (m_RelativeFootPositions[1].stepType == 3)
	{
	  //delta_x = (m_CurrentSupportFootPosition(0,2)+m_ZMPShift3Begin - px0)/SizeOf1stPhase;
	  delta_x = (m_CurrentSupportFootPosition(0,2)+m_ZMPShift[0] - px0)/SizeOf1stPhase;
	  //delta_y = (m_CurrentSupportFootPosition(1,2)+(WhoIsSupportFoot)*m_ZMPShift3BeginY - py0)/SizeOf1stPhase;
	  delta_y = (m_CurrentSupportFootPosition(1,2) - py0)/SizeOf1stPhase;
	  
	}
      if (m_RelativeFootPositions[1].stepType == 4)
	{
	  delta_x = (m_CurrentSupportFootPosition(0,2)+m_ZMPShift[2] - px0)/SizeOf1stPhase;
	  delta_y = (m_CurrentSupportFootPosition(1,2)- py0)/SizeOf1stPhase;
	  //delta_x = (CurrentSupportFootPosition(0,2)+m_ZMPShift4Begin - px0)/SizeOf1stPhase;
	  //delta_y = (CurrentSupportFootPosition(1,2)+(WhoIsSupportFoot)*m_ZMPShift3BeginY - py0)/SizeOf1stPhase;
	}
      
      if (m_RelativeFootPositions[1].stepType == 5)
	{
	  delta_x = (m_CurrentSupportFootPosition(0,2)-(m_ZMPShift[0] +m_ZMPShift[2]+
							m_ZMPShift[1] +m_ZMPShift[3]) - px0)/SizeOf1stPhase;
	  delta_y = (m_CurrentSupportFootPosition(1,2)- py0)/SizeOf1stPhase;
	  //delta_x = (CurrentSupportFootPosition(0,2)-(m_ZMPShift3Begin + 
	  // m_ZMPShift4Begin+m_ZMPShift3End + m_ZMPShift4End) - px0)/SizeOf1stPhase;
	  //delta_y = (CurrentSupportFootPosition(1,2)-(WhoIsSupportFoot)*(m_ZMPShift3BeginY + 
	  // m_ZMPShift4BeginY+m_ZMPShift3EndY + m_ZMPShift4EndY) - py0)/SizeOf1stPhase;
	}
      
    }   	
  
  ODEBUG4(" GetZMPDiscretization: Step 5 " << AddArraySize << " " ,"DebugData.txt");
  ODEBUG("SizeOf1stPhase: " << SizeOf1stPhase << "dx: " << delta_x << " dy: " << delta_y);  
  for(unsigned int k=0;k<SizeOf1stPhase;k++)
    {
      
      ZMPPositions[CurrentZMPindex].px = px0 + k*delta_x;
      ZMPPositions[CurrentZMPindex].py = py0 + k*delta_y;
      
      ZMPPositions[CurrentZMPindex].theta =theta0;
      
      
      ZMPPositions[CurrentZMPindex].time = m_CurrentTime;
      
      ZMPPositions[CurrentZMPindex].stepType = m_RelativeFootPositions[1].stepType+10;
      
      // Right now the foot is not moving during the double support
      // TO DO: whatever you need to do ramzi....
      LeftFootAbsolutePositions[CurrentZMPindex] = FinalLeftFootAbsolutePositions.back();
      
      // WARNING : This assume that you are walking on a plane.
      LeftFootAbsolutePositions[CurrentZMPindex].z = 0.0;
		  
      RightFootAbsolutePositions[CurrentZMPindex] =FinalRightFootAbsolutePositions.back();

      // WARNING : This assume that you are walking on a plane.
      RightFootAbsolutePositions[CurrentZMPindex].z = 0.0;
	    
      LeftFootAbsolutePositions[CurrentZMPindex].time = 
	RightFootAbsolutePositions[CurrentZMPindex].time = m_CurrentTime;

      LeftFootAbsolutePositions[CurrentZMPindex].stepType = 
	RightFootAbsolutePositions[CurrentZMPindex].stepType = m_RelativeFootPositions[1].stepType+10;
      /*
	ofstream aoflocal;
	aoflocal.open("Corrections.dat",ofstream::app);
	aoflocal << "0.0 0.0 0.0 "<<endl;
	aoflocal.close();
      */
      m_CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
    }
  //-- End Of First phase.
      
  // Second Phase of the step cycle.
      
  // Compute relative feet position for the next step.
  float lStepHeight=0;
  double NextTheta=0, RelTheta=0;
  if (m_RelativeFootPositions.size()>1)
    {
      NextTheta=m_RelativeFootPositions[1].theta;
      RelTheta = NextTheta+m_CurrentTheta;
      lStepHeight = m_StepHeight;
      double c,s;
      c = cos(NextTheta*M_PI/180.0);
      s = sin(NextTheta*M_PI/180.0);
      MAL_MATRIX_DIM(Orientation,double,2,2);
      MAL_MATRIX_DIM(v,double,2,1);
      Orientation(0,0) = c;      Orientation(0,1) = -s;
      Orientation(1,0) = s;      Orientation(1,1) = c;
	  
      MAL_MATRIX_DIM(SubOrientation,double,2,2);
      MAL_MATRIX_C_eq_EXTRACT_A(SubOrientation,m_CurrentSupportFootPosition,double,0,0,2,2);
      Orientation = MAL_RET_A_by_B(Orientation,SubOrientation) ; 
      
      v(0,0) = m_RelativeFootPositions[1].sx;
      v(1,0) = m_RelativeFootPositions[1].sy;
      vdiffsupp = MAL_RET_A_by_B(Orientation,v);
	  
      vrel = vdiffsupp + m_vdiffsupppre;

      // Compute relative feet orientation for the next step
    }
  else
    {
      vrel(0,0)= 0.0;
      vrel(1,0)= 0.0;
      RelTheta= 0.0;
      NextTheta=0.0;
      lStepHeight = 0.0;
    }
#if 0
  cout << "vrel: " << vrel(0,0) << " " << vrel(1,0) << endl;
  cout << "vdiffsupp: " << vdiffsupp(0,0) << " " << vdiffsupp(1,0) << endl;
  cout << "vdiffsupppre: " << vdiffsupppre(0,0) << " " << vdiffsupppre(1,0) << endl;
#endif

  ODEBUG4(" GetZMPDiscretization: Step 6 " << ZMPPositions.size() << " " ,"DebugData.txt");


  m_vdiffsupppre = vdiffsupp;
      
  m_CurrentTheta = NextTheta;
      
  // Create the polynomes for the none-support foot.
  // Change 08/12/2005: Speed up the modification of X and Y
  // for vertical landing of the foot (Kajita-San's trick n 1)
  //   double ModulationSupportCoefficient = 0.9;
  double ModulatedSingleSupportTime = lTsingle * m_ModulationSupportCoefficient;
  double EndOfLiftOff = (lTsingle-ModulatedSingleSupportTime)*0.5;

  m_PolynomeX->SetParameters(ModulatedSingleSupportTime,vrel(0,0));
  m_PolynomeY->SetParameters(ModulatedSingleSupportTime,vrel(1,0));
  m_PolynomeZ->SetParameters(lTsingle,lStepHeight);
  m_PolynomeTheta->SetParameters(ModulatedSingleSupportTime,RelTheta);
  m_PolynomeOmega->SetParameters(EndOfLiftOff,m_Omega);
  m_PolynomeOmega2->SetParameters(ModulatedSingleSupportTime,2*m_Omega);
  //m_PolynomeZMPTheta->SetParameters(lTsingle,NextZMPTheta);
  m_PolynomeZMPTheta->SetParameters(lTsingle,NextTheta);
  unsigned int SizeOfSndPhase = (unsigned int)(lTsingle/m_SamplingPeriod);
  int indexinitial = CurrentZMPindex-1;
  int SignRHAND=1, SignLHAND=1;

  /*//polynomial planning for the stepover 
     
  if (m_RelativeFootPositions[0].stepType==3)
  {
  StepOverPolyPlanner(m_RelativeFootPositions[0].stepType);
  };
  */	
  double px02,py02;
  px02 = ZMPPositions[CurrentZMPindex-1].px;
  py02 = ZMPPositions[CurrentZMPindex-1].py;

  ODEBUG("SizeOfSndPhase: " << SizeOfSndPhase);
  for(unsigned int k=0;k<SizeOfSndPhase;k++)
    {

      MAL_VECTOR_DIM(ZMPInFootCoordinates,double,3);

      ZMPInFootCoordinates[0] = m_ZMPNeutralPosition[0];
      ZMPInFootCoordinates[1] = m_ZMPNeutralPosition[1];
      ZMPInFootCoordinates[2] = 1.0;
	  
      MAL_VECTOR_DIM(ZMPInWorldCoordinates,double,3);

      ZMPInWorldCoordinates = MAL_RET_A_by_B(m_CurrentSupportFootPosition, 
					     ZMPInFootCoordinates); 

      ODEBUG4("CSFP: " << m_CurrentSupportFootPosition << endl <<
	      "ZMPiWC"  << ZMPInWorldCoordinates << endl, "DebugData.txt");	
		
      ZMPPositions[CurrentZMPindex].px = ZMPInWorldCoordinates(0);
      ZMPPositions[CurrentZMPindex].py = ZMPInWorldCoordinates(1);
      ZMPPositions[CurrentZMPindex].time = m_CurrentTime;
	
      if (DoIt)
	{
	  if ((m_RelativeFootPositions[1].stepType == 3)||(m_RelativeFootPositions[1].stepType == 4))
	    {
		
		
	      if (m_RelativeFootPositions[1].stepType == 3)
		{	
		  delta_x = (m_CurrentSupportFootPosition(0,2)+m_ZMPShift[1] - px02)/SizeOfSndPhase;
		  delta_y = (m_CurrentSupportFootPosition(1,2) - py02)/SizeOfSndPhase;
		  //delta_x = (CurrentSupportFootPosition(0,2)+m_ZMPShift3End - px02)/SizeOfSndPhase;
		  //delta_y = (CurrentSupportFootPosition(1,2)+(WhoIsSupportFoot)*m_ZMPShift3EndY - py02)/SizeOfSndPhase;
		}
	      else
		{
		  delta_x = (m_CurrentSupportFootPosition(0,2)+m_ZMPShift[3] - px02)/SizeOfSndPhase;
		  delta_y = (m_CurrentSupportFootPosition(1,2)- py02)/SizeOfSndPhase;
		  //delta_x = (CurrentSupportFootPosition(0,2)+m_ZMPShift4End - px02)/SizeOfSndPhase;
		  //delta_y = (CurrentSupportFootPosition(1,2)+(WhoIsSupportFoot)*m_ZMPShift4EndY - py02)/SizeOfSndPhase;
		}
	
	      ZMPPositions[CurrentZMPindex].px = ZMPPositions[CurrentZMPindex-1].px + delta_x;
	      ZMPPositions[CurrentZMPindex].py = ZMPPositions[CurrentZMPindex-1].py + delta_y;
	    }

	}
	


      /*	  

      if (m_RelativeFootPositions[1].stepType == 3)
      {
      ZMPPositions[CurrentZMPindex].px = ZMPPositions[CurrentZMPindex].px + 0.03;
      }
      if (m_RelativeFootPositions[1].stepType == 4)
      {
      ZMPPositions[CurrentZMPindex].px = ZMPPositions[CurrentZMPindex].px + 0.02;
      }
      */	    
      ZMPPositions[CurrentZMPindex].theta = m_PolynomeZMPTheta->Compute(k*m_SamplingPeriod) + 
	ZMPPositions[indexinitial].theta;

      ZMPPositions[CurrentZMPindex].stepType = WhoIsSupportFoot*m_RelativeFootPositions[0].stepType;
	  
      if (WhoIsSupportFoot==1)
	{
	  UpdateFootPosition(LeftFootAbsolutePositions,
			     RightFootAbsolutePositions,
			     CurrentZMPindex,k,indexinitial,ModulatedSingleSupportTime,
			     m_RelativeFootPositions[1].stepType);
	  SignLHAND=1;
	  SignRHAND=-1;
	}
      else
	{
	  UpdateFootPosition(RightFootAbsolutePositions,
			     LeftFootAbsolutePositions,
			     CurrentZMPindex,k,indexinitial,ModulatedSingleSupportTime,
			     m_RelativeFootPositions[1].stepType);
	  SignLHAND=-1;
	  SignRHAND=1;
	      
	}

  

      LeftFootAbsolutePositions[CurrentZMPindex].time = 
	RightFootAbsolutePositions[CurrentZMPindex].time = m_CurrentTime;
	  
      m_CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
    }
  // cout << SignLHAND << " " << SignRHAND << endl;
  //-- End Of Second phase.
  //      cout << ZMPPositions.size() << " " << CurrentZMPindex << endl;

  if (WhoIsSupportFoot==1)
    WhoIsSupportFoot = -1;//Right
  else 
    WhoIsSupportFoot = 1;// Left

  if (EndSequence)
    EndPhaseOfTheWalking(ZMPPositions,
			 LeftFootAbsolutePositions,
			 RightFootAbsolutePositions);

  // Filter out the ZMP values.
  for(unsigned int i=0;i<ZMPPositions.size();i++)
    {
      double ltmp[2]={0,0};

      int o= FinalZMPPositions.size()-1;
      for(unsigned int j=0;j<m_ZMPFilterWindow.size();j++)
	{
	  int r;
	  r=i-j;
	  if (r<0)
	    {
	      if (-r<o)
		{
		  ltmp[0] += m_ZMPFilterWindow[j]*FinalZMPPositions[o+r].px;
		  ltmp[1] += m_ZMPFilterWindow[j]*FinalZMPPositions[o+r].py;
		}
	    }
	  else
	    {
	      ltmp[0] += m_ZMPFilterWindow[j]*ZMPPositions[r].px;
	      ltmp[1] += m_ZMPFilterWindow[j]*ZMPPositions[r].py;
	    }
	}
      
      ZMPPosition aZMPPos;
      aZMPPos.px = ltmp[0];
      aZMPPos.py = ltmp[1];
      aZMPPos.theta = ZMPPositions[i].theta;
      aZMPPos.time = ZMPPositions[i].time;
      aZMPPos.stepType = ZMPPositions[i].stepType;
      
      FinalZMPPositions.push_back(aZMPPos);
      FinalLeftFootAbsolutePositions.push_back(LeftFootAbsolutePositions[i]);
      FinalRightFootAbsolutePositions.push_back(RightFootAbsolutePositions[i]);
    }
  m_RelativeFootPositions.pop_front();
}

void ZMPDiscretization::EndPhaseOfTheWalking(  deque<ZMPPosition> &ZMPPositions,
					       deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					       deque<FootAbsolutePosition> &RightFootAbsolutePositions)

{
  double CurrentTime = 0;
  double TimeForThisFootPosition;
  // Deal with the end phase of the walking.
  TimeForThisFootPosition = m_Tdble/2.0;
  unsigned int AddArraySize = (unsigned int)(TimeForThisFootPosition/m_SamplingPeriod);
  
  unsigned int CurrentZMPindex;
  unsigned int currentsize = CurrentZMPindex = ZMPPositions.size();
  ZMPPositions.resize(currentsize+AddArraySize);
  LeftFootAbsolutePositions.resize(currentsize+AddArraySize);
  RightFootAbsolutePositions.resize(currentsize+AddArraySize);
  
  ODEBUG4(" GetZMPDiscretization: Step 7 " << currentsize << " " << AddArraySize,"DebugData.txt");


  unsigned int SizeOfEndPhase = (unsigned int)(m_Tdble/(2*m_SamplingPeriod));
  double px0,py0,delta_x,delta_y;
  double pxf=0,pyf=0;
  px0 = ZMPPositions[CurrentZMPindex-1].px;
  py0 = ZMPPositions[CurrentZMPindex-1].py;
  //  int lindex = SupportFootAbsolutePositions.size()-1;

  // We assume that the last positon of the ZMP
  // will the middle of the two last position
  // of the support foot.
  pxf = (LeftFootAbsolutePositions[CurrentZMPindex-1].x+
	 RightFootAbsolutePositions[CurrentZMPindex-1].x)/2.0;
  pyf = (LeftFootAbsolutePositions[CurrentZMPindex-1].y+
	 RightFootAbsolutePositions[CurrentZMPindex-1].y)/2.0;
  //  cout << "PX: " << pxf << " PY: " << pyf<< endl;
  
  delta_x = (pxf - px0)/(double)SizeOfEndPhase;
  delta_y = (pyf - py0)/(double)SizeOfEndPhase;
  
  for(unsigned int k=0;k<SizeOfEndPhase;k++)
    {
      ZMPPositions[CurrentZMPindex].px = 
	ZMPPositions[CurrentZMPindex-1].px + delta_x;
      ZMPPositions[CurrentZMPindex].py = 
	ZMPPositions[CurrentZMPindex-1].py + delta_y;
      ZMPPositions[CurrentZMPindex].time = CurrentTime;
      ZMPPositions[CurrentZMPindex].theta = 
	ZMPPositions[CurrentZMPindex-1].theta;

      ZMPPositions[CurrentZMPindex].stepType =0; 

      LeftFootAbsolutePositions[CurrentZMPindex] = 
	LeftFootAbsolutePositions[CurrentZMPindex-1];
      RightFootAbsolutePositions[CurrentZMPindex] = 
	RightFootAbsolutePositions[CurrentZMPindex-1];
      
      LeftFootAbsolutePositions[CurrentZMPindex].time = 
	RightFootAbsolutePositions[CurrentZMPindex].time = CurrentTime;

      LeftFootAbsolutePositions[CurrentZMPindex].stepType = 
	RightFootAbsolutePositions[CurrentZMPindex].stepType = 0;

      CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
      /*
	cout << "ZMPX: "<<  ZMPPositions[CurrentZMPindex-1].px 
	<< " ZMPY: "<<  ZMPPositions[CurrentZMPindex-1].py  << endl; 
      */

    }
  //  cout << "ZMPX: "<<  ZMPPositions[CurrentZMPindex-1].px 
  //       << " ZMPY: "<<  ZMPPositions[CurrentZMPindex-1].py  << endl; 

  // Added a new phase for exhausting the preview control
  AddArraySize = (int)(3.0*m_PreviewControlTime/m_SamplingPeriod);
  currentsize = ZMPPositions.size();
  ZMPPositions.resize(currentsize+AddArraySize);
  LeftFootAbsolutePositions.resize(currentsize+AddArraySize);
  RightFootAbsolutePositions.resize(currentsize+AddArraySize);

  ODEBUG4(" GetZMPDiscretization: Step 8 ","DebugData.txt");

  for(unsigned int i=0;i<AddArraySize;i++)
    {  
      ZMPPositions[CurrentZMPindex].px = ZMPPositions[CurrentZMPindex-1].px;
      ZMPPositions[CurrentZMPindex].py = ZMPPositions[CurrentZMPindex-1].py;
      ZMPPositions[CurrentZMPindex].theta = ZMPPositions[CurrentZMPindex-1].theta;
      ZMPPositions[CurrentZMPindex].time = CurrentTime;

      ZMPPositions[CurrentZMPindex].stepType = 0;

      LeftFootAbsolutePositions[CurrentZMPindex] = 
	LeftFootAbsolutePositions[CurrentZMPindex-1];
      RightFootAbsolutePositions[CurrentZMPindex] = 
	RightFootAbsolutePositions[CurrentZMPindex-1];

      
      LeftFootAbsolutePositions[CurrentZMPindex].time = 
	RightFootAbsolutePositions[CurrentZMPindex].time = CurrentTime;
	
      LeftFootAbsolutePositions[CurrentZMPindex].stepType = 
	RightFootAbsolutePositions[CurrentZMPindex].stepType = 0;
      
      CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
    }

  ODEBUG4(" GetZMPDiscretization: Step 9 " << ZMPPositions.size(),"DebugData.txt");


}

// Assuming that the points are going counter-clockwise
// and that the foot's interior is at the left of the points.
// The result is : A [ Zx(k), Zy(k)]' + B  >=0
int ZMPDiscretization::ComputeLinearSystem(vector<CH_Point> aVecOfPoints, 
					   MAL_MATRIX(&A,double),
					   MAL_MATRIX(&B,double))
{
  double a,b,c;
  unsigned int n = aVecOfPoints.size();
  MAL_MATRIX_RESIZE(A,aVecOfPoints.size(),2);
  MAL_MATRIX_RESIZE(B,aVecOfPoints.size(),1);

  // Dump a file to display on scilab .
  // This should be removed during real usage inside a robot.
  if (1)
    {
      ofstream aof;
      aof.open("Constraints.dat",ofstream::app);
      for(unsigned int i=0;i<n-1;i++)
	{
	  aof << aVecOfPoints[i].col << " " <<  aVecOfPoints[i].row << " "
	      << aVecOfPoints[i+1].col << " "  << aVecOfPoints[i+1].row << endl;
	}
      aof << aVecOfPoints[n-1].col << " " <<  aVecOfPoints[n-1].row << " "
	  << aVecOfPoints[0].col << " "  << aVecOfPoints[0].row << endl;
      aof.close();
    }
  
  for(unsigned int i=0;i<n-1;i++)
    {
      
      ODEBUG("(x["<< i << "],y["<<i << "]): " << aVecOfPoints[i].col << " " <<  aVecOfPoints[i].row << " "
	     << aVecOfPoints[i+1].col << " "  << aVecOfPoints[i+1].row );

      if (fabs(aVecOfPoints[i+1].col-aVecOfPoints[i].col)>1e-7)
	{
	  double y1,x1,y2,x2,lmul=-1.0;

	  if (aVecOfPoints[i+1].col < aVecOfPoints[i].col)
	    {
	      lmul=1.0;
	      y2 = aVecOfPoints[i].row;
	      y1 = aVecOfPoints[i+1].row;
	      x2 = aVecOfPoints[i].col;
	      x1 = aVecOfPoints[i+1].col;
	    }
	  else
	    {
	      y2 = aVecOfPoints[i+1].row;
	      y1 = aVecOfPoints[i].row;
	      x2 = aVecOfPoints[i+1].col;
	      x1 = aVecOfPoints[i].col;

	    }
	  
	  
	  a = (y2 - y1)/(x2-x1) ;
	  b = (aVecOfPoints[i].row - a * aVecOfPoints[i].col);

	  a = lmul*a;
	  b = lmul*b;
	  c= -lmul;
	  

	}
      else
	{
	  c = 0.0;
	  a = -1.0;	  
	  b = aVecOfPoints[i+1].col;
	  if (aVecOfPoints[i+1].row < aVecOfPoints[i].row)
	    {
	      a=-a;
	      b=-b;
	    }
	}
      
      
      A(i,0) = a; A(i,1)= c;
      B(i,0) = b;

    }
  
  ODEBUG("(x["<< n-1 << "],y["<< n-1 << "]): " << aVecOfPoints[n-1].col << " " <<  aVecOfPoints[n-1].row << " "
	 << aVecOfPoints[0].col << " "  << aVecOfPoints[0].row );
  
  if (fabs(aVecOfPoints[0].col-aVecOfPoints[n-1].col)>1e-7)
    {
      double y1,x1,y2,x2,lmul=-1.0;
      
      if (aVecOfPoints[0].col < aVecOfPoints[n-1].col)
	{
	  lmul=1.0;
	  y2 = aVecOfPoints[n-1].row;
	  y1 = aVecOfPoints[0].row;
	  x2 = aVecOfPoints[n-1].col;
	  x1 = aVecOfPoints[0].col;
	}
      else
	{
	  y2 = aVecOfPoints[0].row;
	  y1 = aVecOfPoints[n-1].row;
	  x2 = aVecOfPoints[0].col;
	  x1 = aVecOfPoints[n-1].col;
	  
	}
      
      
      a = (y2 - y1)/(x2-x1) ;
      b = (aVecOfPoints[0].row - a * aVecOfPoints[0].col);
      
      a = lmul*a;
      b = lmul*b;
      c= -lmul;
      
    }
  else
    {
      c = 0.0;
      a = -1.0;	  
      b = aVecOfPoints[0].col;
      if (aVecOfPoints[0].row < aVecOfPoints[n-1].row)
	{
	  a=-a;
	  b=-b;
	}
    }

  
  A(n-1,0) = a; A(n-1,1)= c;
  B(n-1,0) = b;
  
  
  ODEBUG("A: " << A );
  ODEBUG("B: " << B);
      
  return 0;
}

int ZMPDiscretization::BuildLinearConstraintInequalities2(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
							  deque<FootAbsolutePosition> &RightFootAbsolutePositions,
							  deque<LinearConstraintInequality_t *> &
							  QueueOfLConstraintInequalities,
							  double ConstraintOnX,
							  double ConstraintOnY)
{
  // Find the convex hull for each of the position,
  // in order to create the corresponding trajectory.
  ComputeConvexHull aCH;
  double lLeftFootHalfWidth,lLeftFootHalfHeight,
    lRightFootHalfWidth,lRightFootHalfHeight,lZ;
  
  // Read humanoid specificities.
  m_HS->GetFootSize(-1,lRightFootHalfWidth,lRightFootHalfHeight,lZ);
  m_HS->GetFootSize(1,lLeftFootHalfWidth,lLeftFootHalfHeight,lZ);
  
  lRightFootHalfWidth *= 0.5;
  lRightFootHalfHeight *= 0.5;
  lLeftFootHalfWidth *= 0.5;
  lLeftFootHalfHeight *= 0.5;

  lLeftFootHalfHeight -= ConstraintOnY;
  lRightFootHalfHeight -= ConstraintOnY;

  lLeftFootHalfWidth -= ConstraintOnX;
  lRightFootHalfWidth -= ConstraintOnX;
  
  if (LeftFootAbsolutePositions.size()!=
      RightFootAbsolutePositions.size())
    return -1;
  
  int State=0; // State for the system 0:start, 1: Right Support Foot, 2: Left Support Foot,
  // 3: Double Support.
  int ComputeCH=0;
  float lx=0.0, ly=0.0;
  float lxcoefs[4] = { 1.0, 1.0, -1.0, -1.0};
  float lycoefs[4] = {-1.0, 1.0,  1.0, -1.0};


  // Going through the set of generated data for each 5 ms.
  // from this extract a set of linear constraints.
  for(unsigned int i=0;i<LeftFootAbsolutePositions.size();i++)
    {
      
      ComputeCH=0;
      // First check if we have to compute a convex hull
      if (i==0)
	{
	  ComputeCH = 1;
	  State=3;
	}
      // Double support
      if (LeftFootAbsolutePositions[i].stepType>=10)
	{
	  if (State!=3)
	    ComputeCH=1;
	  State =3;
	}
      else
	{
	  if (LeftFootAbsolutePositions[i].z>0)
	    {
	      if (State!=2)
		ComputeCH=1;
	      State=2;
	    }
	  else if (RightFootAbsolutePositions[i].z>0)
	    {
	      if (State!=1)
		ComputeCH=1;
	      State=1;
	    }
	  
	}

      if (ComputeCH)
	{
	  vector<CH_Point> TheConvexHull;
	  // Check if we are in a single or double support phase,
	  // by testing the step type. In double support phase
	  // the value is greater than or equal to 10.
	  if (State==3)
	    {
	      // In this case the convex hull 
	      TheConvexHull.resize(4);

		      
	      if (LeftFootAbsolutePositions[i].x<RightFootAbsolutePositions[i].x)
		{
		  TheConvexHull[0].col= RightFootAbsolutePositions[i].x +  lRightFootHalfWidth;
		  TheConvexHull[1].col= RightFootAbsolutePositions[i].x +  lRightFootHalfWidth;
		  TheConvexHull[2].col= LeftFootAbsolutePositions[i].x -  lLeftFootHalfWidth;
		  TheConvexHull[3].col= LeftFootAbsolutePositions[i].x -  lLeftFootHalfWidth;
		  
		}
	      else 
		{
		  TheConvexHull[0].col= LeftFootAbsolutePositions[i].x +  lLeftFootHalfWidth;
		  TheConvexHull[1].col= LeftFootAbsolutePositions[i].x +  lLeftFootHalfWidth;
		  TheConvexHull[2].col= RightFootAbsolutePositions[i].x -  lRightFootHalfWidth;
		  TheConvexHull[3].col= RightFootAbsolutePositions[i].x -  lRightFootHalfWidth;

		}


	      if (LeftFootAbsolutePositions[i].y<RightFootAbsolutePositions[i].y)
		{
		  TheConvexHull[0].row= LeftFootAbsolutePositions[i].y - lLeftFootHalfHeight;
		  TheConvexHull[1].row= RightFootAbsolutePositions[i].y +  lRightFootHalfHeight;
		  TheConvexHull[2].row= RightFootAbsolutePositions[i].y +  lRightFootHalfHeight;
		  TheConvexHull[3].row= LeftFootAbsolutePositions[i].y -  lLeftFootHalfHeight;
		  
		}
	      else 
		{
		  TheConvexHull[0].row= RightFootAbsolutePositions[i].y -  lRightFootHalfHeight;
		  TheConvexHull[1].row= LeftFootAbsolutePositions[i].y +  lLeftFootHalfHeight;
		  TheConvexHull[2].row= LeftFootAbsolutePositions[i].y +  lLeftFootHalfHeight;
		  TheConvexHull[3].row= RightFootAbsolutePositions[i].y -  lRightFootHalfHeight;

		}

	    }
	  // In the second case, it is necessary to compute 
	  // the support foot.
	  else
	    {
	      
	      TheConvexHull.resize(4);
	      
	      // Who is support foot ?
	      if (LeftFootAbsolutePositions[i].z < RightFootAbsolutePositions[i].z)
		{
		  lx=LeftFootAbsolutePositions[i].x;
		  ly=LeftFootAbsolutePositions[i].y;
	      
		  for(unsigned j=0;j<4;j++)
		    {
		      TheConvexHull[j].col = lx + lxcoefs[j]*  lLeftFootHalfWidth;
		      TheConvexHull[j].row = ly + lycoefs[j]*  lLeftFootHalfHeight;
		    }
			      
		}
	      else
		{
		  lx=RightFootAbsolutePositions[i].x;
		  ly=RightFootAbsolutePositions[i].y;
		  
		  for(unsigned j=0;j<4;j++)
		    {
		      TheConvexHull[j].col = lx + lxcoefs[j]*  lRightFootHalfWidth;
		      TheConvexHull[j].row = ly + lycoefs[j]*  lRightFootHalfHeight;
		    }
		}
	      
	      
	    }

	  // Linear Constraint Inequality
	  LinearConstraintInequality_t * aLCI = new LinearConstraintInequality_t;
	  ComputeLinearSystem(TheConvexHull,aLCI->A, aLCI->B);
	  aLCI->StartingTime = LeftFootAbsolutePositions[i].time;
	  if (QueueOfLConstraintInequalities.size()>0)
	    {
	      QueueOfLConstraintInequalities.back()->EndingTime = LeftFootAbsolutePositions[i].time;

	    }

	  QueueOfLConstraintInequalities.push_back(aLCI);
	  
	  
	  
	}
      if (i==LeftFootAbsolutePositions.size()-1)
	{
	  if (QueueOfLConstraintInequalities.size()>0)
	    {
	      QueueOfLConstraintInequalities.back()->EndingTime = LeftFootAbsolutePositions[i].time;
	    }
	}
    }

  ODEBUG("Size of the 5 ms array: "<< LeftFootAbsolutePositions.size());
  ODEBUG("Size of the queue of Linear Constraint Inequalities " << QueueOfLConstraintInequalities.size());
  
  return 0;
}

int ZMPDiscretization::BuildLinearConstraintInequalities(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
							 deque<FootAbsolutePosition> &RightFootAbsolutePositions,
							 deque<LinearConstraintInequality_t *> &
							 QueueOfLConstraintInequalities,
							 double ConstraintOnX,
							 double ConstraintOnY)
{
  // Find the convex hull for each of the position,
  // in order to create the corresponding trajectory.
  ComputeConvexHull aCH;
  double lLeftFootHalfWidth,lLeftFootHalfHeight,
    lRightFootHalfWidth,lRightFootHalfHeight,lZ;
  
  // Read humanoid specificities.
  m_HS->GetFootSize(-1,lRightFootHalfWidth,lRightFootHalfHeight,lZ);
  m_HS->GetFootSize(1,lLeftFootHalfWidth,lLeftFootHalfHeight,lZ);

  lRightFootHalfWidth *= 0.5;
  lRightFootHalfHeight *= 0.5;
  lLeftFootHalfWidth *= 0.5;
  lLeftFootHalfHeight *= 0.5;
  
  lLeftFootHalfHeight -= ConstraintOnY;
  lRightFootHalfHeight -= ConstraintOnY;

  lLeftFootHalfWidth -= ConstraintOnX;
  lRightFootHalfWidth -= ConstraintOnX;
  
  if (LeftFootAbsolutePositions.size()!=
      RightFootAbsolutePositions.size())
    return -1;
  
  int State=0; // State for the system 0:start, 1: Right Support Foot, 2: Left Support Foot,
  // 3: Double Support.
  int ComputeCH=0;
  float lx=0.0, ly=0.0;
  float lxcoefs[4] = { 1.0, 1.0, -1.0, -1.0};
  float lycoefs[4] = {-1.0, 1.0,  1.0, -1.0};
  double prev_xmin=1e7, prev_xmax=-1e7, prev_ymin=1e7, prev_ymax=-1e7;
  RESETDEBUG4("ConstraintMax.dat");

  // Going through the set of generated data for each 5 ms.
  // from this extract a set of linear constraints.
  for(unsigned int i=0;i<LeftFootAbsolutePositions.size();i++)
    {
      
      ComputeCH=0;
      // First check if we have to compute a convex hull
      if (i==0)
	{
	  ComputeCH = 1;
	  State=3;
	}
      // Double support
      if (LeftFootAbsolutePositions[i].stepType>=10)
	{
	  if (State!=3)
	    ComputeCH=1;
	  State =3;
	}
      else
	{
	  if (LeftFootAbsolutePositions[i].z>0)
	    {
	      if (State!=2)
		ComputeCH=1;
	      State=2;
	    }
	  else if (RightFootAbsolutePositions[i].z>0)
	    {
	      if (State!=1)
		ComputeCH=1;
	      State=1;
	    }
	  
	}

      if (ComputeCH)
	{
	  double xmin=1e7, xmax=-1e7, ymin=1e7, ymax=-1e7;

	  vector<CH_Point> TheConvexHull;
	  // Check if we are in a single or double support phase,
	  // by testing the step type. In double support phase
	  // the value is greater than or equal to 10.
	  // In this case, we have to compute the convex hull
	  // of both feet.
	  if (State==3)
	    {
	      vector<CH_Point> aVecOfPoints;
	      
	      aVecOfPoints.resize(8);

	      lx=LeftFootAbsolutePositions[i].x;
	      ly=LeftFootAbsolutePositions[i].y;
	      
	      for(unsigned j=0;j<4;j++)
		{
		  aVecOfPoints[j].col = lx + lxcoefs[j]*  lLeftFootHalfWidth;
		  aVecOfPoints[j].row = ly + lycoefs[j]*  lLeftFootHalfHeight;

		  // Computes the maxima.
		  xmin = aVecOfPoints[j].col < xmin ? aVecOfPoints[j].col : xmin;
		  xmax = aVecOfPoints[j].col > xmax ? aVecOfPoints[j].col : xmax;
		  ymin = aVecOfPoints[j].row < ymin ? aVecOfPoints[j].row : ymin;
		  ymax = aVecOfPoints[j].row > ymax ? aVecOfPoints[j].row : ymax;
		  
		}
	      ODEBUG("State 3-1 " << xmin << " " << xmax << " " << ymin << " " << ymax);
	      lx=RightFootAbsolutePositions[i].x;
	      ly=RightFootAbsolutePositions[i].y;
	      ODEBUG("Right Foot: " << lx << " " << ly );
	      for(unsigned j=0;j<4;j++)
		{
		  aVecOfPoints[j+4].col = lx + lxcoefs[j]*  lRightFootHalfWidth;
		  aVecOfPoints[j+4].row = ly + lycoefs[j]*  lRightFootHalfHeight;
		  
		  // Computes the maxima.
		  xmin = aVecOfPoints[j+4].col < xmin ? aVecOfPoints[j+4].col : xmin;
		  xmax = aVecOfPoints[j+4].col > xmax ? aVecOfPoints[j+4].col : xmax;
		  ymin = aVecOfPoints[j+4].row < ymin ? aVecOfPoints[j+4].row : ymin;
		  ymax = aVecOfPoints[j+4].row > ymax ? aVecOfPoints[j+4].row : ymax;
		  
		}

	      ODEBUG("State 3-2" << xmin << " " << xmax << " " << ymin << " " << ymax);
	      aCH.DoComputeConvexHull(aVecOfPoints,TheConvexHull);
	    }
	  // In the second case, it is necessary to compute 
	  // the support foot.
	  else
	    {
	      
	      TheConvexHull.resize(4);
	      
	      // Who is support foot ?
	      if (LeftFootAbsolutePositions[i].z < RightFootAbsolutePositions[i].z)
		{
		  lx=LeftFootAbsolutePositions[i].x;
		  ly=LeftFootAbsolutePositions[i].y;
	      
		  for(unsigned j=0;j<4;j++)
		    {
		      TheConvexHull[j].col = lx + lxcoefs[j]*  lLeftFootHalfWidth;
		      TheConvexHull[j].row = ly + lycoefs[j]*  lLeftFootHalfHeight;

		      // Computes the maxima.
		      xmin = TheConvexHull[j].col < xmin ? TheConvexHull[j].col : xmin;
		      xmax = TheConvexHull[j].col > xmax ? TheConvexHull[j].col : xmax;
		      ymin = TheConvexHull[j].row < ymin ? TheConvexHull[j].row : ymin;
		      ymax = TheConvexHull[j].row > ymax ? TheConvexHull[j].row : ymax;

		    }
		  ODEBUG("Left support foot");
		}
	      else
		{
		  lx=RightFootAbsolutePositions[i].x;
		  ly=RightFootAbsolutePositions[i].y;
		  
		  for(unsigned j=0;j<4;j++)
		    {
		      TheConvexHull[j].col = lx + lxcoefs[j]*  lRightFootHalfWidth;
		      TheConvexHull[j].row = ly + lycoefs[j]*  lRightFootHalfHeight;
		      // Computes the maxima.
		      xmin = TheConvexHull[j].col < xmin ? TheConvexHull[j].col : xmin;
		      xmax = TheConvexHull[j].col > xmax ? TheConvexHull[j].col : xmax;
		      ymin = TheConvexHull[j].row < ymin ? TheConvexHull[j].row : ymin;
		      ymax = TheConvexHull[j].row > ymax ? TheConvexHull[j].row : ymax;

		    }
		  ODEBUG("Right support foot");
		}
	      ODEBUG("State !=3 " << xmin << " " << xmax << " " << ymin << " " << ymax);
	      
	    }

	  // Linear Constraint Inequality
	  LinearConstraintInequality_t * aLCI = new LinearConstraintInequality_t;
	  ComputeLinearSystem(TheConvexHull,aLCI->A, aLCI->B);
	  aLCI->StartingTime = LeftFootAbsolutePositions[i].time;
	  if (QueueOfLConstraintInequalities.size()>0)
	    {
	      QueueOfLConstraintInequalities.back()->EndingTime = LeftFootAbsolutePositions[i].time;
	      ODEBUG4( QueueOfLConstraintInequalities.back()->StartingTime << " " <<
		       QueueOfLConstraintInequalities.back()->EndingTime << " " <<
		       prev_xmin << " "  <<
		       prev_xmax << " "  <<
		       prev_ymin << " "  <<
		       prev_ymax
		       ,"ConstraintMax.dat");

	    }
	  ODEBUG("Final " << xmin << " " << xmax << " " << ymin << " " << ymax);
	  prev_xmin = xmin; prev_xmax = xmax;
	  prev_ymin = ymin; prev_ymax = ymax;

	  QueueOfLConstraintInequalities.push_back(aLCI);
	  
	  
	  
	}
      if (i==LeftFootAbsolutePositions.size()-1)
	{
	  if (QueueOfLConstraintInequalities.size()>0)
	    {
	      QueueOfLConstraintInequalities.back()->EndingTime = LeftFootAbsolutePositions[i].time;
	      ODEBUG4( QueueOfLConstraintInequalities.back()->StartingTime << " " <<
		       QueueOfLConstraintInequalities.back()->EndingTime << " " <<
		       prev_xmin << " "  <<
		       prev_xmax << " "  <<
		       prev_ymin << " "  <<
		       prev_ymax
		       ,"ConstraintMax.dat");

	    }
	}
    }

  ODEBUG("Size of the 5 ms array: "<< LeftFootAbsolutePositions.size());
  ODEBUG("Size of the queue of Linear Constraint Inequalities " << QueueOfLConstraintInequalities.size());
  
  return 0;
}

int ZMPDiscretization::BuildMatricesPxPu(double * & Px,double * &Pu, 
					 unsigned N, double T,
					 double StartingTime,
					 deque<LinearConstraintInequality_t *> & QueueOfLConstraintInequalities,
					 double Com_Height,
					 unsigned int &NbOfConstraints,
					 MAL_VECTOR(& xk,double))
{
  // Discretize the problem.
  ODEBUG(" N:" << N << " T: " << T);
  
  // Creates the matrices.
  // The memory will be bounded to 8 constraints per
  // support foot (double support case).
  // Will be probably all the time smaller.
  if (Px==0)
    Px = new double[8*N+1];

  if (Pu==0)
    Pu = new double[(8*N+1)*2*N];

  memset(Pu,0,(8*N+1)*2*N*sizeof(double));
  //memset(Px,0,(8*N+1)*sizeof(double));

  deque<LinearConstraintInequality_t *>::iterator LCI_it, store_it;
  LCI_it = QueueOfLConstraintInequalities.begin();
  while (LCI_it!=QueueOfLConstraintInequalities.end())
    {
      if ((StartingTime>=(*LCI_it)->StartingTime) &&
	  (StartingTime<=(*LCI_it)->EndingTime))
	{
	  break;
	}
      LCI_it++;
    }
  store_it = LCI_it;
  

  // Did not find the appropriate Linear Constraint.
  if (LCI_it==QueueOfLConstraintInequalities.end())
    {
      cout << "HERE 3" << endl;
      exit(0);

      return -1;
    }
      
  if (0)
    {
      char Buffer[1024];
      sprintf(Buffer,"PXD_%f.dat", StartingTime);
      RESETDEBUG4(Buffer);
      ODEBUG6("xk:" << xk << " Starting time: " <<StartingTime ,Buffer );
      char Buffer2[1024];
      sprintf(Buffer2,"PXxD_%f.dat", StartingTime);
      RESETDEBUG4(Buffer2);
      
      char Buffer3[1024];
      sprintf(Buffer3,"PXyD_%f.dat", StartingTime);
      RESETDEBUG4(Buffer3);
    }
  
  // Compute first the number of constraint.
  unsigned int IndexConstraint=0;
  for(unsigned int i=0;i<N;i++)
    {

      double ltime = StartingTime+ i* T;
      if (ltime > (*LCI_it)->EndingTime)
	LCI_it++;

      if (LCI_it==QueueOfLConstraintInequalities.end())
	{
	  break;
	}
      IndexConstraint += MAL_MATRIX_NB_ROWS((*LCI_it)->A);
    }  
  NbOfConstraints = IndexConstraint;

  LCI_it = store_it;
  IndexConstraint = 0;
  ODEBUG("Starting Matrix to build the constraints. ");
  ODEBUG((*LCI_it)->A );
  for(unsigned int i=0;i<N;i++)
    {

      double ltime = StartingTime+ i* T;
      if (ltime > (*LCI_it)->EndingTime)
	{
	  LCI_it++;
	}

      if (LCI_it==QueueOfLConstraintInequalities.end())
	{
	}

      // For each constraint.
      for(unsigned j=0;j<MAL_MATRIX_NB_ROWS((*LCI_it)->A);j++)
	{
	  Px[IndexConstraint] = 
	    // X Axis * A
	    (xk[0] +
	     xk[1] * T *(i+1) + 
	     xk[2]*((i+1)*(i+1)*T*T/2 - Com_Height/9.81))
	    * (*LCI_it)->A(j,0)
	     + 
	     // Y Axis * A
	    ( xk[3]+ xk[4]* T * (i+1) + 
	       xk[5]*((i+1)*(i+1)*T*T/2 - Com_Height/9.81))	  
	    * (*LCI_it)->A(j,1)
	     // Constante part of the constraint
	    + (*LCI_it)->B(j,0);

	  ODEBUG6(Px[IndexConstraint] << " " << (*LCI_it)->A(j,0)  << " "
		  << (*LCI_it)->A[j][1] << " " << (*LCI_it)->B(j,0) ,Buffer);
	  ODEBUG6(1 << " " <<    T *(i+1) << " " <<    (i+1)*(i+1)*T*T/2 - Com_Height/9.81,Buffer2);
	  ODEBUG6(1 << " " <<    T *(i+1) << " " <<    (i+1)*(i+1)*T*T/2 - Com_Height/9.81,Buffer3);
	  for(unsigned k=0;k<=i;k++)
	    {
	      // X axis
	      Pu[IndexConstraint+k*(NbOfConstraints+1)] = 
		(*LCI_it)->A(j,0)*
		((1+3*(i-k)+3*(i-k)*(i-k))*T*T*T/6.0 - T * Com_Height/9.81);
	      
	      // Y axis
	      Pu[IndexConstraint+(k+N)*(NbOfConstraints+1)] = 
		(*LCI_it)->A(j,1)*
		((1+3*(i-k)+3*(i-k)*(i-k))*T*T*T/6.0 - T * Com_Height/9.81);
		 
	      
	    }
	  ODEBUG("IC: " << IndexConstraint );
	  IndexConstraint++;
	}

    }
  ODEBUG6("Index Constraint :"<< IndexConstraint,Buffer);
  if (0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"Pu_%f.dat", StartingTime);
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<IndexConstraint;i++)
	{
	  for(unsigned int j=0;j<2*N;j++)
	    aof << Pu[i+j*(NbOfConstraints+1)] << " " ;
	  aof << endl;
	}
      aof.close();
      
      sprintf(Buffer,"PX_%f.dat", StartingTime);
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<IndexConstraint;i++)
	{
	  aof << Px[i] << endl ;
	}
      aof.close();
    }

  return 0;
}

int ZMPDiscretization::BuildZMPTrajectoryFromFootTrajectory(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
							    deque<FootAbsolutePosition> &RightFootAbsolutePositions,
							    deque<ZMPPosition> &ZMPRefPositions,
							    deque<ZMPPosition> &NewFinalZMPPositions,
							    deque<COMPosition> &COMPositions,
							    double ConstraintOnX,
							    double ConstraintOnY,
							    double T,
							    unsigned int N)
{
  //  double T=0.02; 
  //double T=0.02;
  //  unsigned int N=75; 
  //  unsigned int N = 100;
  //  double ComHeight=0.814;
  double ComHeight=0.80;
  double *Px=0,*Pu=0;
  unsigned int NbOfConstraints=8*N; // Nb of constraints to be taken into account
  // for each iteration
  MAL_VECTOR(xk,double);MAL_VECTOR(Buk,double);MAL_VECTOR(zk,double);
  MAL_MATRIX(vnlPx,double); MAL_MATRIX(vnlPu,double);
  MAL_MATRIX(vnlValConstraint,double);
  MAL_MATRIX(vnlX,double);MAL_MATRIX(vnlStorePx,double);
  MAL_MATRIX(vnlStoreX,double);
  MAL_VECTOR(ConstraintNb,int);
  MAL_MATRIX(PPu,double); MAL_MATRIX(VPu,double); 
  MAL_MATRIX(VPx,double); MAL_MATRIX(PPx,double);
  MAL_MATRIX(Id,double);MAL_MATRIX(OptA,double);
  MAL_MATRIX(OptB,double);MAL_MATRIX( OptC,double);
  MAL_VECTOR(ZMPRef,double);MAL_VECTOR(OptD,double);
  double alpha = 200.0, beta = 1000.0;
  int CriteriaToMaximize=1;


  if (1)
    RESETDEBUG4("DebugInterpol.dat");

  MAL_MATRIX_RESIZE(PPu,2*N,2*N);
  MAL_MATRIX_RESIZE(VPu,2*N,2*N);
  MAL_MATRIX_RESIZE(PPx,2*N,6);
  MAL_MATRIX_RESIZE(VPx,2*N,6);
  MAL_MATRIX_RESIZE(Id,2*N,2*N);
  MAL_VECTOR_RESIZE(ZMPRef,2*N);

  for(unsigned int i=0;i<N;i++)
    {
      // Compute VPx and PPx
      VPx(i,0)   = 0.0;   VPx(i,1) =     1.0; VPx(i,2)   = (i+1)*T;
      VPx(i,3)   = 0.0;   VPx(i,4) =     0.0; VPx(i,5)   = 0.0;
      VPx(i+N,0) = 0.0;   VPx(i+N,1) =   0.0; VPx(i+N,2) = 0.0;
      VPx(i+N,3) = 0.0;   VPx(i+N,4) =   1.0; VPx(i+N,5) = (i+1)*T;

      PPx(i,0) = 1.0; PPx(i,1)     = (i+1)*T; PPx(i,2) = (i+1)*(i+1)*T*T*0.5;
      PPx(i,3) = 0.0; PPx(i,4)     =       0; PPx(i,5) = 0.;
      PPx(i+N,0) = 0.0; PPx(i+N,1) =     0.0; PPx(i+N,2) = 0.0;
      PPx(i+N,3) = 1.0; PPx(i+N,4) = (i+1)*T; PPx(i+N,5) = (i+1)*(i+1)*T*T*0.5;
      
      
      for(unsigned int j=0;j<N;j++)
	{
	  PPu(i,j)=0;
	  
	  if (j<=i)
	    {

	      VPu(i,j)= (2*(i-j)+1)*T*T*0.5 ;
	      VPu(i+N,j+N)= (2*(i-j)+1)*T*T*0.5 ;
	      VPu(i,j+N)=0.0;
	      VPu(i+N,j)=0.0;

	      PPu(i,j)= (1 + 3*(i-j) + 3*(i-j)*(i-j)) * T*T*T/6.0;
	      PPu(i+N,j+N)= (1 + 3*(i-j) + 3*(i-j)*(i-j)) * T*T*T/6.0;
	      PPu(i,j+N)=0.0;
	      PPu(i+N,j)=0.0;

	    }
	  else
	    {

	      VPu(i,j) = 0.0;
	      VPu(i+N,j+N)=0.0;
	      VPu(i,j+N)=0.0;
	      VPu(i+N,j)=0.0;

	      PPu(i,j) = 0.0;
	      PPu(i+N,j+N)=0.0;
	      PPu(i,j+N)=0.0;
	      PPu(i+N,j)=0.0;

	    }

	  // Identity.
	  if (i==j)
	    {
	      Id(i,j)=1.0;Id(i+N,j+N)=1.0;
	      Id(i+N,j)=0.0;Id(i,j+N)=0.0;
	    }
	  else
	    {
	      Id(i,j)=0.0;Id(i+N,j+N)=0.0;
	      Id(i+N,j)=0.0;Id(i,j+N)=0.0;
	    }

	  //	  Zeros(i,j)=0.0;Zeros(i+N,j+N)=0.0;
	  //	  Zeros(i+N,j)=0.0;Zeros(i,j+N)=0.0;

	}
    }

  if (0)
    {
      ofstream aof;
      aof.open("VPx.dat");
      aof << VPx;
      aof.close();
      
      aof.open("PPx.dat");
      aof << PPx;
      aof.close();
      
      aof.open("VPu.dat");
      aof << VPu;
      aof.close();
      
      aof.open("PPu.dat");
      aof << PPu;
      aof.close();
    }
  
  MAL_MATRIX_RESIZE(vnlX,2*N,1);
  
  MAL_VECTOR_RESIZE(xk,6);
  for(unsigned int i=0;i<6;i++)
    xk[i] = 0.0;
  MAL_VECTOR_RESIZE(Buk,6);
  MAL_VECTOR_RESIZE(zk,2);

  int m = NbOfConstraints;
  int me= 0;
  int mmax = NbOfConstraints+1;
  int n = 2*N;
  int nmax = 2*N; // Size of the matrix to compute the cost function.
  int mnn = m+n+n;

  double *C=new double[4*N*N]; // Objective function matrix
  double *D=new double[2*N];   // Constant part of the objective function
  double *XL=new double[2*N];  // Lower bound of the jerk.
  double *XU=new double[2*N];  // Upper bound of the jerk.
  double *X=new double[2*N];   // Solution of the system.
  double Eps=1e-8 ;
  double *U = (double *)malloc( sizeof(double)*mnn); // Returns the Lagrange multipliers.;

  // Initialization of the matrices
  memset(C,0,4*N*N*sizeof(double));
  for(unsigned int i=0;i<2*N;i++)
    C[i*2*N+i] = 1.0;
  
  int iout=0;
  int ifail;
  int iprint=1;
  int lwar=3*nmax*nmax/2+ 10*nmax  + 2*mmax + 20000;;
  double *war= (double *)malloc(sizeof(double)*lwar);
  int liwar = n; //
  int *iwar = new int[liwar]; // The Cholesky decomposition is done internally.


  for(int i=0;i<6;i++)
    {
      m_B(i,0) = 0.0;
      m_C(0,i) = 0.0;
      m_C(1,i) = 0.0;
      for(int j=0;j<6;j++)
	m_A(i,j)=0.0;
    }

  m_A(0,0) = 1.0; m_A(0,1) =   T; m_A(0,2) = T*T/2.0;
  m_A(1,0) = 0.0; m_A(1,1) = 1.0; m_A(1,2) = T;
  m_A(2,0) = 0.0; m_A(2,1) = 0.0; m_A(2,2) = 1.0;
  m_A(3,3) = 1.0; m_A(3,4) =   T; m_A(3,5) = T*T/2.0;
  m_A(4,3) = 0.0; m_A(4,4) = 1.0; m_A(4,5) = T;
  m_A(5,3) = 0.0; m_A(5,4) = 0.0; m_A(5,5) = 1.0;
  

  m_B(0,0) = T*T*T/6.0;
  m_B(1,0) = T*T/2.0;
  m_B(2,0) = T;
  m_B(3,0) = T*T*T/6.0;
  m_B(4,0) = T*T/2.0;
  m_B(5,0) = T;

  
  m_C(0,0) = 1.0;
  m_C(0,1) = 0.0;
  m_C(0,2) = -ComHeight/9.81;

  m_C(1,3) = 1.0;
  m_C(1,4) = 0.0;
  m_C(1,5) = -ComHeight/9.81;

  deque<LinearConstraintInequality_t *> QueueOfLConstraintInequalities;
  
  if (1)
    {
      RESETDEBUG4("DebugPBW.dat");
      RESETDEBUG4("DebugPBW_Pb.dat");

      ODEBUG6("A:" << m_A << endl << "B:" << m_B, "DebugPBW_Pb.dat");
  

      ofstream aof;
      aof.open("Constraints.dat",ofstream::out);
      aof.close();
    }
      
  // Build a set of linear constraint inequalities.
  BuildLinearConstraintInequalities(LeftFootAbsolutePositions,
				    RightFootAbsolutePositions,
				    QueueOfLConstraintInequalities,
				    ConstraintOnX,
				    ConstraintOnY);

  deque<LinearConstraintInequality_t *>::iterator LCI_it;
  LCI_it = QueueOfLConstraintInequalities.begin();
  while(LCI_it!=QueueOfLConstraintInequalities.end())
    {
      //      cout << *LCI_it << endl; 
      //      cout << (*LCI_it)->StartingTime << " " << (*LCI_it)->EndingTime << endl;
      LCI_it++;
    }
  
  MAL_MATRIX_RESIZE(vnlStorePx,
		    6*N,
		    1+(unsigned int)(QueueOfLConstraintInequalities.back()->EndingTime/(T)));
  
  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(vnlStorePx);i++)
    {
      for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(vnlStorePx);j++)
	{
	  vnlStorePx(i,j) =0.0;
	}
    }
  MAL_MATRIX_RESIZE(vnlStoreX,
		    2*N,1+(unsigned int)(QueueOfLConstraintInequalities.back()->EndingTime/(T)));

  for(unsigned int i=0;i<2*N;i++)
    vnlStoreX(i,0) = 0.0;
  
  MAL_VECTOR_RESIZE(ConstraintNb,
		    1+(unsigned int)(QueueOfLConstraintInequalities.back()->EndingTime/(T)));

  // pre computes the matrices needed for the optimization.

  //  OptA = Id + alpha * VPu.Transpose() * VPu + beta * PPu.Transpose() * PPu;
  MAL_MATRIX(lterm1,double);
  lterm1 = MAL_RET_TRANSPOSE(PPu);
  lterm1 = MAL_RET_A_by_B(lterm1, PPu);
  lterm1 = beta * lterm1;

  MAL_MATRIX(lterm2,double);
  lterm2 = MAL_RET_TRANSPOSE(VPu);
  lterm2 = MAL_RET_A_by_B(lterm2, VPu);
  lterm2 = alpha * lterm2;

  MAL_MATRIX_RESIZE(OptA,
		    MAL_MATRIX_NB_ROWS(lterm1),
		    MAL_MATRIX_NB_COLS(lterm1));
  MAL_MATRIX_SET_IDENTITY(OptA);
  OptA = OptA + lterm1 + lterm2;



  if (CriteriaToMaximize==1)
    {
      for(unsigned int i=0;i<2*N;i++)
	for(unsigned int j=0;j<2*N;j++)
	  C[j*2*N+i] = OptA(i,j);

      if (0)
	{
	  ofstream aof;
	  char Buffer[1024];
	  sprintf(Buffer,"C.dat");
	  aof.open(Buffer,ofstream::out);
	  for(unsigned int i=0;i<2*N;i++)
	    {
	      for(unsigned int j=0;j<2*N-1;j++)
		aof << OptA(i,j) << " ";
	      aof << OptA(i,2*N-1);
	      aof << endl;
	    }
	  aof.close(); 
	  
	}
    }
  
  lterm1 = MAL_RET_TRANSPOSE(PPu);
  lterm1 = MAL_RET_A_by_B(lterm1,PPx);
  OptB = MAL_RET_TRANSPOSE(VPu);
  OptB = MAL_RET_A_by_B(OptB,VPx);
  OptB = alpha * OptB;
  OptB = OptB + beta * lterm1;

  if (0)
  {
    ofstream aof;
    char Buffer[1024];
    sprintf(Buffer,"OptB.dat");
    aof.open(Buffer,ofstream::out);
    for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(OptB);i++)
      {
	for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(OptB)-1;j++)
	  aof << OptB(i,j) << " ";
	aof << OptB(i,MAL_MATRIX_NB_COLS(OptB)-1);
	aof << endl;
      }
    aof.close(); 
    
  }
  
  OptC = MAL_RET_TRANSPOSE(PPu);
  OptC = beta * OptC;
  if (0)
  {
    ofstream aof;
    char Buffer[1024];
    sprintf(Buffer,"OptC.dat");
    aof.open(Buffer,ofstream::out);
    for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(OptC);i++)
      {
	for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(OptC)-1;j++)
	  aof << OptC(i,j) << " ";
	aof << OptC(i,MAL_MATRIX_NB_COLS(OptC)-1);
	aof << endl;
      }
    aof.close(); 
    
  }
  
  double TotalAmountOfCPUTime=0.0,CurrentCPUTime=0.0;
  struct timeval start,end;
  int li=0; int interval=(int)(T/m_SamplingPeriod);
  for(double StartingTime=0.0;
      StartingTime<QueueOfLConstraintInequalities.back()->EndingTime-
	N*T;
      StartingTime+=T,li++)
    {
      gettimeofday(&start,0);
      // Build the related matrices.
      BuildMatricesPxPu(Px,Pu,
			N,T,
			StartingTime,
			QueueOfLConstraintInequalities,
			ComHeight,
			NbOfConstraints,
			xk);
      

      m = NbOfConstraints;
      
      mmax = NbOfConstraints+1;
      lwar = 3*nmax*nmax/2+ 10*nmax  + 2*mmax + 20000;
      mnn = m+n+n;

      // Call to QLD (a linearly constrained quadratic problem solver)

      // Prepare D.
      for(unsigned int i=0;i<N;i++)
	{
	  ZMPRef[i] = ZMPRefPositions[li*interval+i*interval].px;
	  ZMPRef[i+N] = ZMPRefPositions[li*interval+i*interval].py;
	}
      
      if (0)
	{
	  ofstream aof;
	  char Buffer[1024];
	  sprintf(Buffer,"ZMPRef_%f.dat",StartingTime);
	  aof.open(Buffer,ofstream::out);
	  for(unsigned int i=0;i<2*N;i++)
	    {
	      aof << ZMPRef[i] << endl;
	    }
	  aof.close(); 
	}  

      if (CriteriaToMaximize==1)
	{
	  MAL_VECTOR(lterm1v,double);
	  MAL_C_eq_A_by_B(lterm1v,OptC,ZMPRef);
	  MAL_C_eq_A_by_B(OptD,OptB,xk);
	  OptD -= lterm1v;
	  for(unsigned int i=0;i<2*N;i++)
	    D[i] = OptD[i];

	  if (0)
	    {
	      ofstream aof;
	      char Buffer[1024];
	      sprintf(Buffer,"D_%f.dat",StartingTime);
	      aof.open(Buffer,ofstream::out);
	      for(unsigned int i=0;i<2*N;i++)
		{
		  aof << OptD[i] << endl;
		}
	      aof.close(); 
	    }

	}
      else
	{
	  // Default : set D to zero.
	  for(unsigned int i=0;i<2*N;i++)
	    D[i] = 0.0;
	}

      for(unsigned int i=0;i<2*N;i++)
	{
	  XL[i] = -1e8;
	  XU[i] = 1e8;
	}
      memset(X,0,2*N*sizeof(double));

      // Verification
      ConstraintNb[li] = m;
      MAL_MATRIX_RESIZE(vnlPu,m,2*N);
      MAL_MATRIX_RESIZE(vnlPx,m,1);
      
  
      for(int i=0; i<m;i++)
	{
	  vnlPx(i,0) =
	    vnlStorePx(i,li) = Px[i];
	}

      iwar[0]=1;
      ODEBUG("m: " << m);
      ql0001_(&m, &me, &mmax,&n, &nmax,&mnn,
	      C, D, Pu,Px,XL,XU,
	      X,U,&iout, &ifail, &iprint,
	      war, &lwar,
	      iwar, &liwar,&Eps);

      if (ifail!=0)
	{
	  cout << "IFAIL: " << ifail << endl;
	  //	  exit(0);
	}


      for(int i=0; i<m;i++)
	for(unsigned int j=0; j<2*N;j++)
	  vnlPu(i,j) = Pu[j*(m+1)+i];

      for(unsigned int i=0; i<2*N;i++)
	{
	  vnlStoreX(i,li) = X[i];
	  vnlX(i,0) = X[i];
	}

      vnlValConstraint = MAL_RET_A_by_B(vnlPu, vnlX)  + vnlPx;
      
      if (0)
      {
	ofstream aof;
	char Buffer[1024];
	sprintf(Buffer,"X_%f.dat",StartingTime);
	aof.open(Buffer,ofstream::out);
	for(unsigned int i=0;i<2*N;i++)
	  {
	    aof << X[i] << endl;
	  }
	aof.close(); 
      }

      if (MAL_MATRIX_NB_COLS(vnlValConstraint)!=1)
	{
	  cout << "Problem during validation of the constraints matrix: " << endl;
	  cout << "   size for the columns different from 1" << endl;
	  exit(0);
	}
      for(int i=0;i<m;i++)
	{
	  unsigned int pbOnCurrent=0;
	  if (vnlValConstraint(i,0)<-1e-8)
	    {
	      ODEBUG3("Problem during validation of the constraint: ");
	      ODEBUG3("  constraint " << i << " is not positive");
	      ODEBUG3(vnlValConstraint(i,0));
	      pbOnCurrent = 1;
	    }

	  if (pbOnCurrent)
	    {
	      ODEBUG3("PbonCurrent: " << pbOnCurrent << " " << li
		      << " Contrainte " << i 
		      << " StartingTime :" << StartingTime);
	      if (pbOnCurrent)
		{
		  exit(0);
		}
	    }
	    
	}

      ODEBUG("X[0] " << X[0] << " X[N] :" << X[N]);

      // Compute the command multiply 
      Buk[0] = X[0]*m_B(0,0);
      Buk[1] = X[0]*m_B(1,0);
      Buk[2] = X[0]*m_B(2,0);
      
      Buk[3] = X[N]*m_B(3,0);
      Buk[4] = X[N]*m_B(4,0);
      Buk[5] = X[N]*m_B(5,0);


      // Fill the queues with the interpolated CoM values.
      for(int lk=0;lk<interval;lk++)
	{
	  
	  COMPosition aCOMPos;
	  double lkSP;
	  lkSP = (lk+1) * m_SamplingPeriod;

	  aCOMPos.x[0] = 
	    xk[0] + // Position
	    lkSP * xk[1] +  // Speed
	    0.5 * lkSP*lkSP * xk[2] +// Acceleration 
	    lkSP * lkSP * lkSP * X[0] /6.0; // Jerk

	  aCOMPos.x[1] = 
	    xk[1] + // Speed
	    lkSP * xk[2] +  // Acceleration
	    0.5 * lkSP * lkSP * X[0]; // Jerk

	  aCOMPos.x[2] = 
	    xk[2] +  // Acceleration
	    lkSP * X[0]; // Jerk

	  aCOMPos.y[0] = 
	    xk[3] + // Position
	    lkSP * xk[4] +  // Speed
	    0.5 * lkSP*lkSP * xk[5] + // Acceleration 
	    lkSP * lkSP * lkSP * X[N] /6.0; // Jerk

	  aCOMPos.y[1] = 
	    xk[4] + // Speed
	    lkSP * xk[5] +  // Acceleration
	    0.5 * lkSP * lkSP * X[N]; // Jerk

	  aCOMPos.y[2] = 
	    xk[5] +  // Acceleration
	    lkSP * X[N]; // Jerk

	  aCOMPos.theta = ZMPRefPositions[li*interval+lk].theta;

	  COMPositions.push_back(aCOMPos);

	  // Compute ZMP position and orientation.
	  ZMPPosition aZMPPos;
	  aZMPPos.px = m_C(0,0) * aCOMPos.x[0] +
	    m_C(0,1) * aCOMPos.x[1] + m_C(0,2) * aCOMPos.x[2];
	  
	  aZMPPos.py = m_C(0,0) * aCOMPos.y[0] +
	    m_C(0,1) * aCOMPos.y[1] + m_C(0,2) * aCOMPos.y[2];

	  aZMPPos.theta = ZMPRefPositions[li*interval+lk].theta;
	  aZMPPos.stepType = ZMPRefPositions[li*interval+lk].stepType;

	  // Put it into the stack.
	  NewFinalZMPPositions.push_back(aZMPPos);
	  
	  ODEBUG4(aCOMPos.x[0] << " " << aCOMPos.x[1] << " " << aCOMPos.x[2] << " " <<
		  aCOMPos.y[0] << " " << aCOMPos.y[1] << " " << aCOMPos.y[2] << " " <<
		  aCOMPos.theta << " " <<
		  aZMPPos.px << " " << aZMPPos.py <<  " " << aZMPPos.theta << " " << 
		  X[0] << " " << X[N] << " " << 
		  lkSP << " " << T , "DebugInterpol.dat");
	}

      // Simulate the dynamical system
      MAL_C_eq_A_by_B(zk,m_C,xk);
      xk = MAL_RET_A_by_B(m_A,xk) + Buk ;

      ODEBUG4(xk[0] << " " << xk[1] << " " << xk[2] << " " <<
	      xk[3] << " " << xk[4] << " " << xk[5] << " " <<
	      X[0]  << " " << X[N]  << " " << 
	      zk[0] << " " << zk[1] << " " << 
	      StartingTime
	      ,"DebugPBW.dat");
      ODEBUG6("uk:" << uk,"DebugPBW.dat");
      ODEBUG6("xk:" << xk,"DebugPBW.dat");

      // Compute CPU consumption time.
      gettimeofday(&end,0);
      CurrentCPUTime = end.tv_sec - start.tv_sec + 
	0.000001 * (end.tv_usec - start.tv_usec);
      TotalAmountOfCPUTime += CurrentCPUTime;
      ODEBUG("Current Time : " << StartingTime << " " << 
	      " Virtual time to simulate: " << QueueOfLConstraintInequalities.back()->EndingTime - StartingTime << 
	      "Computation Time " << CurrentCPUTime << " " << TotalAmountOfCPUTime);

    }
  // Current heuristic to complete the ZMP buffer:
  // fill with the last correct value.
  ZMPPosition LastZMPPos = NewFinalZMPPositions.back();
  for (unsigned int i=0;i< N*T/m_SamplingPeriod;i++)
  {
    ZMPPosition aZMPPos;
    aZMPPos.px = LastZMPPos.px;
    aZMPPos.py = LastZMPPos.py;
    aZMPPos.theta = LastZMPPos.theta;
    aZMPPos.stepType = LastZMPPos.stepType;
    NewFinalZMPPositions.push_back(aZMPPos);
  }

  if (0)
    {
      ofstream aof;
      aof.open("StorePx.dat",ofstream::out);
      
      for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(vnlStorePx);i++)
	{
	  for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(vnlStorePx);j++)
	    {
	      aof << vnlStorePx(i,j) << " ";
	    }
	  aof << endl;
	}
      aof.close();
      
      
      char lBuffer[1024];
      sprintf(lBuffer,"StoreX.dat");
      aof.open(lBuffer,ofstream::out);
      
      for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(vnlStoreX);i++)
	{
	  for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(vnlStoreX);j++)
	    {
	      aof << vnlStoreX(i,j) << " ";
	    }
	  aof << endl;
	}
      aof.close();
      
      aof.open("Cnb.dat",ofstream::out);
      for(unsigned int i=0;i<MAL_VECTOR_SIZE(ConstraintNb);i++)
	{
	  aof << ConstraintNb[i]<<endl;
	}
      aof.close();
    }
  
  cout << "Size of PX: " << MAL_MATRIX_NB_ROWS(vnlStorePx) << " " 
       << MAL_MATRIX_NB_COLS(vnlStorePx) << " " << endl;
  delete C;
  delete D;
  delete XL;
  delete XU;
  delete X;
  free(war);
  free(U);
  delete iwar;
  // Clean the queue of Linear Constraint Inequalities.
  //  deque<LinearConstraintInequality_t *>::iterator LCI_it;
  LCI_it = QueueOfLConstraintInequalities.begin();
  while(LCI_it!=QueueOfLConstraintInequalities.end())
    {
      //      cout << *LCI_it << endl; 
      cout << (*LCI_it)->StartingTime << " " << (*LCI_it)->EndingTime << endl;
      delete *(LCI_it);
      LCI_it++;
    }
  QueueOfLConstraintInequalities.clear();
  
  return 0;
}



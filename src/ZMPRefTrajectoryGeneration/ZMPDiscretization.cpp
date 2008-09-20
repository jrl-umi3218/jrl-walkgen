 /* This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of steps.


   Copyright (c) 2005-2007, 
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

#include <walkGenJrl/ZMPRefTrajectoryGeneration/ZMPDiscretization.h>
#include <walkGenJrl/Mathematics/ConvexHull.h>

using namespace::PatternGeneratorJRL;

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


ZMPDiscretization::ZMPDiscretization(SimplePluginManager *lSPM,string DataFile, HumanoidSpecificities *aHS)
  : ZMPRefTrajectoryGeneration(lSPM)
{
  m_PC = 0;
  
  m_HS = aHS;
  m_FootTrajectoryGenerationStandard = new FootTrajectoryGenerationStandard(lSPM,m_HS);
  m_FootTrajectoryGenerationStandard->InitializeInternalDataStructures();

  m_PolynomeZMPTheta = new Polynome3(0,0);

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

  m_PrevCurrentSupportFootPosition = m_CurrentSupportFootPosition;
  
  // Create the window for the filter.
  double T=0.05; // Arbritrary from Kajita's San Matlab files.
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
  if (m_FootTrajectoryGenerationStandard!=0)
    delete m_FootTrajectoryGenerationStandard;
  
  if (m_PolynomeZMPTheta!=0)
    delete m_PolynomeZMPTheta;
}


void ZMPDiscretization::GetZMPDiscretization(deque<ZMPPosition> & FinalZMPPositions,
					     deque<COMPosition> & FinalCOMPositions,
					     deque<RelativeFootPosition> &RelativeFootPositions,
					     deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					     deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					     double Xmax,
					     COMPosition & lStartingCOMPosition,
					     MAL_S3_VECTOR(,double) & lStartingZMPPosition,
					     FootAbsolutePosition & InitLeftFootAbsolutePosition,
					     FootAbsolutePosition & InitRightFootAbsolutePosition)
{
  
  InitOnLine(FinalZMPPositions,
	     FinalCOMPositions,
	     LeftFootAbsolutePositions,
	     RightFootAbsolutePositions,
	     InitLeftFootAbsolutePosition,
	     InitRightFootAbsolutePosition,
	     RelativeFootPositions,
	     lStartingCOMPosition,
	     lStartingZMPPosition);

  EndPhaseOfTheWalking(FinalZMPPositions,
		       FinalCOMPositions,
		       LeftFootAbsolutePositions,
		       RightFootAbsolutePositions);

  FinalCOMPositions.resize(FinalZMPPositions.size());
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
	  aof << ZMPPositions[i].time << " " << ZMPPositions[i].px << " " 
	      << ZMPPositions[i].py << " " << ZMPPositions[i].stepType << " 0.0" <<   endl;
	}
      aof.close();
    }

  aof.open(FootFileName.c_str(),ofstream::out);
  if (aof.is_open())
    {
      for(unsigned int i=0;i<SupportFootAbsolutePositions.size();i++)
	{
	  aof << SupportFootAbsolutePositions[i].x << " " << SupportFootAbsolutePositions[i].y << " " 
	      << SupportFootAbsolutePositions[i].z << " " << SupportFootAbsolutePositions[i].stepType << " 0.0" <<  endl;
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
				  deque<COMPosition> & FinalCoMPositions,
				  deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
				  deque<FootAbsolutePosition> &RightFootAbsolutePositions,
				  FootAbsolutePosition & InitLeftFootAbsolutePosition,
				  FootAbsolutePosition & InitRightFootAbsolutePosition,
				  deque<RelativeFootPosition> &RelativeFootPositions,
				  COMPosition & lStartingCOMPosition,
				  MAL_S3_VECTOR(,double) & lStartingZMPPosition)
{
  m_RelativeFootPositions.clear();
  FootAbsolutePosition CurrentLeftFootAbsPos, CurrentRightFootAbsPos;
  double CurrentAbsTheta=0.0;
  RESETDEBUG4("ZMDInitOnLine.txt");
  ODEBUG4("ZMP::InitOnLine - Step 1 ","ZMDInitOnLine.txt");
//   m_RelativeFootPositions.resize(RelativeFootPositions.size());
//   for(unsigned int i=0;i<RelativeFootPositions.size();i++)
//     m_RelativeFootPositions.push_back(RelativeFootPositions[i]);

  // Initialize position of the current support foot.
  m_CurrentSupportFootPosition(0,0) = 1;   m_CurrentSupportFootPosition(0,1) = 0;   m_CurrentSupportFootPosition(0,2) = 0;
  m_CurrentSupportFootPosition(1,0) = 0;   m_CurrentSupportFootPosition(1,1) = 1;   m_CurrentSupportFootPosition(1,2) = 0;
  m_CurrentSupportFootPosition(2,0) = 0;   m_CurrentSupportFootPosition(2,1) = 0;   m_CurrentSupportFootPosition(2,2) = 1;

  m_PrevCurrentSupportFootPosition = m_CurrentSupportFootPosition;

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

    // Initialize who is support foot.
  double CurrentZMPTheta=0;
  CurrentZMPTheta = (CurrentRightFootAbsPos.theta + CurrentLeftFootAbsPos.theta)/2.0;
  ODEBUG("CurrentZMPTheta at start: " << CurrentZMPTheta << " " 
       << CurrentRightFootAbsPos.theta << " " 
	 << CurrentLeftFootAbsPos.theta);
  if (RelativeFootPositions[0].sy < 0 )
    {
      m_vdiffsupppre(0,0) = CurrentRightFootAbsPos.x - CurrentLeftFootAbsPos.x;
      m_vdiffsupppre(1,0) = CurrentRightFootAbsPos.y - CurrentLeftFootAbsPos.y;
      m_CurrentTheta = CurrentRightFootAbsPos.theta - CurrentLeftFootAbsPos.theta;
    }
  else 
    {
      m_vdiffsupppre(0,0) = -CurrentRightFootAbsPos.x + CurrentLeftFootAbsPos.x;
      m_vdiffsupppre(1,0) = -CurrentRightFootAbsPos.y + CurrentLeftFootAbsPos.y;
      m_CurrentTheta = CurrentLeftFootAbsPos.theta - CurrentRightFootAbsPos.theta;
    }

  ODEBUG4("ZMP::InitOnLine - Step 3 ","ZMDInitOnLine.txt");
  // Initialization of the ZMP position (stable values during the Preview control time window). 
  int AddArraySize;
  {
    double ldAddArraySize = m_PreviewControlTime/m_SamplingPeriod;
    AddArraySize = (int) ldAddArraySize;
  }

  ODEBUG(AddArraySize);
  FinalZMPPositions.resize(AddArraySize);
  FinalCoMPositions.resize(AddArraySize);
  LeftFootAbsolutePositions.resize(AddArraySize);
  RightFootAbsolutePositions.resize(AddArraySize);
  int CurrentZMPindex=0;
  // Also very important for the initialization: reshape the ZMP reference for a smooth starting.
  double startingZMPREF[2] = { lStartingZMPPosition(0), lStartingZMPPosition(1)};
  double finalZMPREF[2] = {m_ZMPNeutralPosition[0],m_ZMPNeutralPosition[1]};

  ODEBUG4("ZMP::InitOnLine - Step 4 ","ZMDInitOnLine.txt");
  for(unsigned int i=0;i<FinalZMPPositions.size();i++)
    {
      double coef = (float)i/(float)FinalZMPPositions.size();
      // Set ZMP positions.

      // Smooth ramp
      FinalZMPPositions[CurrentZMPindex].px = startingZMPREF[0] + (finalZMPREF[0] - startingZMPREF[0])*coef;
      FinalZMPPositions[CurrentZMPindex].py = startingZMPREF[1] + (finalZMPREF[1] - startingZMPREF[1])*coef;

      FinalZMPPositions[CurrentZMPindex].theta = CurrentAbsTheta;
      FinalZMPPositions[CurrentZMPindex].time = m_CurrentTime;
      FinalZMPPositions[CurrentZMPindex].stepType = 0;

      // Set CoM positions.
      FinalCoMPositions[CurrentZMPindex].z[0] = m_PC->GetHeightOfCoM();
      FinalCoMPositions[CurrentZMPindex].z[1] = 0.0;
      FinalCoMPositions[CurrentZMPindex].z[2] = 0.0;
      FinalCoMPositions[CurrentZMPindex].pitch = 0.0;
      FinalCoMPositions[CurrentZMPindex].roll = 0.0;
      
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


  if (0)
    {
      ofstream dbg_aof("DebugZMPRefPos.dat",ofstream::out);
      for(unsigned int i=0;i<FinalZMPPositions.size();i++)
	{
	  dbg_aof << FinalZMPPositions[i].px << " "
		  << FinalZMPPositions[i].py << endl;
	}
      dbg_aof.close();
    }
  ODEBUG6("InitOnLine","DebugDataRFPos.txt" );
  for(unsigned int i=1;i<RelativeFootPositions.size();i++)
    {
      OnLineAddFoot(RelativeFootPositions[i],
		    FinalZMPPositions,
		    FinalCoMPositions,
		    LeftFootAbsolutePositions,
		    RightFootAbsolutePositions,
		    false);
    }
  ODEBUG4("ZMP::InitOnLine: End ","ZMDInitOnLine.txt");


  return RelativeFootPositions.size();
}

void ZMPDiscretization::UpdateCurrentSupportFootPosition(RelativeFootPosition aRFP)
{
  m_PrevCurrentSupportFootPosition = m_CurrentSupportFootPosition;

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


void ZMPDiscretization::OnLine(double time,
				    deque<ZMPPosition> & FinalZMPPositions,				     
				    deque<COMPosition> & FinalCOMPositions,
				    deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
				    deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)
{
  /* Does nothing... */
}

/* The interface method which returns an appropriate update of the
   appropriate stacks (ZMPRef, FootPosition) depending on the 
   state of the relative steps stack. */
void ZMPDiscretization::OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
				      deque<ZMPPosition> & FinalZMPPositions,					     
				      deque<COMPosition> & FinalCOMPositions,
				      deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
				      deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
				      bool EndSequence)
{
  deque<ZMPPosition> ZMPPositions;
  deque<FootAbsolutePosition> LeftFootAbsolutePositions;
  deque<FootAbsolutePosition> RightFootAbsolutePositions;
  FootAbsolutePosition CurrentLeftFootAbsPos, CurrentRightFootAbsPos;

  CurrentLeftFootAbsPos = FinalLeftFootAbsolutePositions.back();
  CurrentRightFootAbsPos = FinalRightFootAbsolutePositions.back();

  m_RelativeFootPositions.push_back(NewRelativeFootPosition);
  int WhoIsSupportFoot=1;
  double TimeFirstPhase=0.0;
  int CurrentZMPindex=0;
  MAL_MATRIX_DIM(vdiffsupp,double,2,1);
  MAL_MATRIX_DIM(vrel,double,2,1);

  double lTdble=m_Tdble, lTsingle=m_Tsingle;

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
  ODEBUG("Time of double support phase in OnLineFootAdd: "<< lTdble);
  TimeFirstPhase = lTdble;

  // Initialize who is support foot.
  if (m_RelativeFootPositions[0].sy < 0 )
    {
      WhoIsSupportFoot = -1;//Right
      m_vdiffsupppre(0,0) = CurrentRightFootAbsPos.x - CurrentLeftFootAbsPos.x;
      m_vdiffsupppre(1,0) = CurrentRightFootAbsPos.y - CurrentLeftFootAbsPos.y;
      m_CurrentTheta = CurrentRightFootAbsPos.theta - CurrentLeftFootAbsPos.theta;
    }
  else 
    {
      WhoIsSupportFoot = 1;// Left
      m_vdiffsupppre(0,0) = -CurrentRightFootAbsPos.x + CurrentLeftFootAbsPos.x;
      m_vdiffsupppre(1,0) = -CurrentRightFootAbsPos.y + CurrentLeftFootAbsPos.y;
      m_CurrentTheta = CurrentLeftFootAbsPos.theta - CurrentRightFootAbsPos.theta;
    }
  
  double TimeForThisFootPosition = TimeFirstPhase+ lTsingle;
  ODEBUG4("TimeFirstPhase: " << TimeFirstPhase << " lTsingle: " << lTsingle,"DebugData.txt");
  // Compute the size of cells to add inside the array.
  double l2AddArraySize= TimeForThisFootPosition/m_SamplingPeriod;
  int AddArraySize = (unsigned int)round(l2AddArraySize);
  ODEBUG("Added part: "<<AddArraySize << " " << l2AddArraySize << 
	  " TimeForThisFootPosition " << TimeForThisFootPosition <<
	  " SamplingPeriod" << m_SamplingPeriod);
  ZMPPositions.resize(AddArraySize);
  LeftFootAbsolutePositions.resize(AddArraySize);
  RightFootAbsolutePositions.resize(AddArraySize);    
  
  m_CurrentAbsTheta+= m_RelativeFootPositions[0].theta;
  m_CurrentAbsTheta = fmod(m_CurrentAbsTheta,180);

  // Computes the new ABSOLUTE position of the supporting foot .
  UpdateCurrentSupportFootPosition(m_RelativeFootPositions[0]);
  
  // First Phase of the step cycle.
  double dSizeOf1stPhase =TimeFirstPhase/m_SamplingPeriod;
  unsigned int SizeOf1stPhase = (unsigned int)round(dSizeOf1stPhase);
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

  m_FootTrajectoryGenerationStandard->SetParameters(FootTrajectoryGenerationStandard::X_AXIS,
						    ModulatedSingleSupportTime,vrel(0,0));
  m_FootTrajectoryGenerationStandard->SetParameters(FootTrajectoryGenerationStandard::Y_AXIS,
						    ModulatedSingleSupportTime,vrel(1,0));
  m_FootTrajectoryGenerationStandard->SetParameters(FootTrajectoryGenerationStandard::Z_AXIS,
						    m_Tsingle,lStepHeight);
  m_FootTrajectoryGenerationStandard->SetParameters(FootTrajectoryGenerationStandard::THETA_AXIS,
						    ModulatedSingleSupportTime,RelTheta);
  m_FootTrajectoryGenerationStandard->SetParameters(FootTrajectoryGenerationStandard::OMEGA_AXIS,
						    EndOfLiftOff,m_Omega);
  m_FootTrajectoryGenerationStandard->SetParameters(FootTrajectoryGenerationStandard::OMEGA2_AXIS,
						    ModulatedSingleSupportTime,2*m_Omega);
  

  //m_PolynomeZMPTheta->SetParameters(lTsingle,NextZMPTheta);
  m_PolynomeZMPTheta->SetParameters(lTsingle,NextTheta);
  double dSizeOfSndPhase = lTsingle/m_SamplingPeriod;
  unsigned int SizeOfSndPhase = (unsigned int)round(dSizeOfSndPhase);
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

      ZMPPositions[CurrentZMPindex].theta = m_PolynomeZMPTheta->Compute(k*m_SamplingPeriod) + 
	ZMPPositions[indexinitial].theta;

      ZMPPositions[CurrentZMPindex].stepType = WhoIsSupportFoot*m_RelativeFootPositions[0].stepType;
	  
      if (WhoIsSupportFoot==1)
	{
	  m_FootTrajectoryGenerationStandard->UpdateFootPosition(LeftFootAbsolutePositions,
								 RightFootAbsolutePositions,
								 CurrentZMPindex,indexinitial,
								 ModulatedSingleSupportTime,
								 m_RelativeFootPositions[1].stepType,
								 -1);
	  SignLHAND=1;
	  SignRHAND=-1;
	}
      else
	{
	  m_FootTrajectoryGenerationStandard->UpdateFootPosition(RightFootAbsolutePositions,
								 LeftFootAbsolutePositions,
								 CurrentZMPindex,indexinitial,
								 ModulatedSingleSupportTime,
								 m_RelativeFootPositions[1].stepType,
								 1);
	  SignLHAND=-1;
	  SignRHAND=1;
	      
	}

  

      LeftFootAbsolutePositions[CurrentZMPindex].time = 
	RightFootAbsolutePositions[CurrentZMPindex].time = m_CurrentTime;
	  
      m_CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
    }

  if (WhoIsSupportFoot==1)
    WhoIsSupportFoot = -1;//Right
  else 
    WhoIsSupportFoot = 1;// Left

  if (EndSequence)
    EndPhaseOfTheWalking(ZMPPositions,
			 FinalCOMPositions,
			 LeftFootAbsolutePositions,
			 RightFootAbsolutePositions);

  if (0)
    {
      ofstream dbg_aof("DebugZMPRefPos.dat",ofstream::app);
      for(unsigned int i=0;i<ZMPPositions.size();i++)
	{
	  dbg_aof << ZMPPositions[i].px << " "
		  << ZMPPositions[i].py << endl;
	}
      dbg_aof.close();
    }
  // Filter out the ZMP values.
  ODEBUG("FinalZMPPositions.size()="<<FinalZMPPositions.size());
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
     
      COMPosition aCOMPosition;
      if (m_PC==0)
	aCOMPosition.z[0] = 0.0;
      else 
	aCOMPosition.z[0] = m_PC->GetHeightOfCoM();

      aCOMPosition.z[1] = aCOMPosition.z[2] = 0.0;
      aCOMPosition.yaw = aZMPPos.theta;
      
      FinalZMPPositions.push_back(aZMPPos);
      FinalCOMPositions.push_back(aCOMPosition);
      FinalLeftFootAbsolutePositions.push_back(LeftFootAbsolutePositions[i]);
      FinalRightFootAbsolutePositions.push_back(RightFootAbsolutePositions[i]);

     
    }
  ODEBUG("FinalZMPPositions.size()="<<FinalZMPPositions.size());
  m_RelativeFootPositions.pop_front();
}

int ZMPDiscretization::OnLineFootChange(double time,
					FootAbsolutePosition &aFootAbsolutePosition,
					deque<ZMPPosition> & FinalZMPPositions,			     
					deque<COMPosition> & CoMPositions,
					deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
					deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
					StepStackHandler *aStepStackHandler)
{
  return -1;
}

int ZMPDiscretization::ReturnOptimalTimeToRegenerateAStep()
{
  int r = (int)(m_PreviewControlTime/m_SamplingPeriod);
  return 2*r;
}

void ZMPDiscretization::EndPhaseOfTheWalking(  deque<ZMPPosition> &ZMPPositions,
					       deque<COMPosition> &FinalCOMPositions,
					       deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					       deque<FootAbsolutePosition> &RightFootAbsolutePositions)

{
  UpdateCurrentSupportFootPosition(m_RelativeFootPositions[0]);
  double CurrentTime = 0;
  double TimeForThisFootPosition;
  // Deal with the end phase of the walking.
  TimeForThisFootPosition = m_Tdble/2.0;
  double dlAddArraySize = TimeForThisFootPosition/m_SamplingPeriod;
  unsigned int AddArraySize = (unsigned int)round(dlAddArraySize);
  
  unsigned int CurrentZMPindex;
  unsigned int currentsize = CurrentZMPindex = ZMPPositions.size();
  ZMPPositions.resize(currentsize+AddArraySize);
  LeftFootAbsolutePositions.resize(currentsize+AddArraySize);
  RightFootAbsolutePositions.resize(currentsize+AddArraySize);
  
  ODEBUG4(" GetZMPDiscretization: Step 7 " << currentsize << " " << AddArraySize,"DebugData.txt");


  double dSizeOfEndPhase = m_Tdble/(2*m_SamplingPeriod);
  unsigned int SizeOfEndPhase = (unsigned int)round(dSizeOfEndPhase);
  double px0,py0,delta_x,delta_y;
  double pxf=0,pyf=0;
  px0 = ZMPPositions[CurrentZMPindex-1].px;
  py0 = ZMPPositions[CurrentZMPindex-1].py;
  //  int lindex = SupportFootAbsolutePositions.size()-1;

  // We assume that the last positon of the ZMP
  // will the middle of the two last position
  // of the support foot.
  pxf = 0.5*(m_CurrentSupportFootPosition(0,2) + m_PrevCurrentSupportFootPosition(0,2));
  pyf = 0.5*(m_CurrentSupportFootPosition(1,2) + m_PrevCurrentSupportFootPosition(1,2));
  
  delta_x = (pxf - px0)/(double)SizeOfEndPhase;
  delta_y = (pyf - py0)/(double)SizeOfEndPhase;
  
  for(unsigned int k=0;k<SizeOfEndPhase;k++)
    {

      // Set ZMP positions.
      ZMPPositions[CurrentZMPindex].px = 
	ZMPPositions[CurrentZMPindex-1].px + delta_x;
      ZMPPositions[CurrentZMPindex].py = 
	ZMPPositions[CurrentZMPindex-1].py + delta_y;
      ZMPPositions[CurrentZMPindex].time = CurrentTime;
      ZMPPositions[CurrentZMPindex].theta = 
	ZMPPositions[CurrentZMPindex-1].theta;

      ZMPPositions[CurrentZMPindex].stepType =0; 

      // Set CoM Positions.
      COMPosition aCOMPosition;
      if (m_PC==0)
	aCOMPosition.z[0] = 0.0;
      else 
	aCOMPosition.z[0] = m_PC->GetHeightOfCoM();

      aCOMPosition.z[1] = aCOMPosition.z[2] = 0.0;
      aCOMPosition.yaw = ZMPPositions[CurrentZMPindex].theta;
      FinalCOMPositions.push_back(aCOMPosition);

      // Set Feet positions.
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

  // Added a new phase for exhausting the preview control
  {
    double ldAddArraySize = 3.0*m_PreviewControlTime/m_SamplingPeriod;
    AddArraySize = (int)ldAddArraySize;
  }
  
  currentsize = ZMPPositions.size();
  ZMPPositions.resize(currentsize+AddArraySize);
  LeftFootAbsolutePositions.resize(currentsize+AddArraySize);
  RightFootAbsolutePositions.resize(currentsize+AddArraySize);

  ODEBUG4(" GetZMPDiscretization: Step 8 ","DebugData.txt");

  for(unsigned int i=0;i<AddArraySize;i++)
    {  

      // Set ZMP Positions
      ZMPPositions[CurrentZMPindex].px = ZMPPositions[CurrentZMPindex-1].px;
      ZMPPositions[CurrentZMPindex].py = ZMPPositions[CurrentZMPindex-1].py;
      ZMPPositions[CurrentZMPindex].theta = ZMPPositions[CurrentZMPindex-1].theta;
      ZMPPositions[CurrentZMPindex].time = CurrentTime;

      ZMPPositions[CurrentZMPindex].stepType = 0;


      // Set CoM Positions.
      COMPosition aCOMPosition;
      if (m_PC==0)
	aCOMPosition.z[0] = 0.0;
      else 
	aCOMPosition.z[0] = m_PC->GetHeightOfCoM();

      aCOMPosition.z[1] = aCOMPosition.z[2] = 0.0;
      aCOMPosition.yaw = ZMPPositions[CurrentZMPindex].theta;
      FinalCOMPositions.push_back(aCOMPosition);

      // Set Feet Positions
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

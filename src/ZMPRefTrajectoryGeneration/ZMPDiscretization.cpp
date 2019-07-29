/*
 * Copyright 2010,
 *
 * Andrei Herdt
 * Francois Keith
 * Florent Lamiraux
 * Evrard Paul
 * Nicolas Perrin
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
/* This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of steps. */

#include "portability/gettimeofday.hh"

#ifdef WIN32
# include <Windows.h>
#endif /* WIN32 */

#include <time.h>
#include <iostream>
#include <fstream>

#include <Debug.hh>

#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.hh>
#include <Mathematics/ConvexHull.hh>

#ifdef WIN32
inline double round( double d )
{
  return floor( d + 0.5 );
}
#endif /* WIN32 */

using namespace::PatternGeneratorJRL;



OnLineState::OnLineState()
{
  m_CurrentState = IDLE_STATE;
}

OnLineState::~OnLineState()
{
}

unsigned int OnLineState::operator()() const
{
  return m_CurrentState;
}

OnLineState & OnLineState::operator=(unsigned int NewState)
{
  if (NewState<DOUBLE_SUPPORT_PHASE)
    m_CurrentState = NewState;
  return *this;
}


ZMPDiscretization::ZMPDiscretization(SimplePluginManager *lSPM,
                                     string DataFile,
                                     PinocchioRobot *aPR)
  : ZMPRefTrajectoryGeneration(lSPM)
{

  m_InitializationProfile = PREV_ZMP_INIT_PROFIL;

  m_PR = aPR;
  PRFoot * lLeftFoot = m_PR->leftFoot();
  m_FootTrajectoryGenerationStandard =
    new FootTrajectoryGenerationStandard(lSPM,
                                         lLeftFoot);
  m_FootTrajectoryGenerationStandard->InitializeInternalDataStructures();

  m_PolynomeZMPTheta = new Polynome3(0,0);

  m_A.resize(6,6);
  m_B.resize(6,1);
  m_C.resize(2,6);

  m_RelativeFootPositions.clear();

  m_ModulationSupportCoefficient=0.9;

  ResetADataFile(DataFile);

  m_ZMPShift.resize(4);
  for (unsigned int i=0; i<4; i++)
    m_ZMPShift[i]=0.0;

  m_ZMPNeutralPosition[0] = 0.0;
  m_ZMPNeutralPosition[1] = 0.0;

  m_CurrentSupportFootPosition.resize(3,3);
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      if (i!=j)
        m_CurrentSupportFootPosition(i,j)= 0.0;
      else
        m_CurrentSupportFootPosition(i,j)= 1.0;

  m_PrevCurrentSupportFootPosition = m_CurrentSupportFootPosition;

  // Prepare size of the matrix used in on-line walking
  m_vdiffsupppre.resize(2,1);
  for(unsigned int i=0; i<m_vdiffsupppre.rows(); i++)
    m_vdiffsupppre(i,0) = 0.0;

  RESETDEBUG4("DebugDataRFPos.txt");
  RESETDEBUG5("DebugZMPRefPos.dat");
  RESETDEBUG4("DebugFinalZMPRefPos.dat");

  // Add internal methods specific to this class.
  RegisterMethodsForScripting();
}

ZMPDiscretization::~ZMPDiscretization()
{
  if (m_FootTrajectoryGenerationStandard!=0)
    delete m_FootTrajectoryGenerationStandard;

  if (m_PolynomeZMPTheta!=0)
    delete m_PolynomeZMPTheta;
}


void ZMPDiscretization::
GetZMPDiscretization
(deque<ZMPPosition> & FinalZMPPositions,
 deque<COMState> & FinalCOMStates,
 deque<RelativeFootPosition> &RelativeFootPositions,
 deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
 deque<FootAbsolutePosition> &RightFootAbsolutePositions,
 double,  //Xmax,
 COMState & lStartingCOMState,
 Eigen::Vector3d & lStartingZMPPosition,
 FootAbsolutePosition & InitLeftFootAbsolutePosition,
 FootAbsolutePosition & InitRightFootAbsolutePosition)
{

  InitOnLine(FinalZMPPositions,
             FinalCOMStates,
             LeftFootAbsolutePositions,
             RightFootAbsolutePositions,
             InitLeftFootAbsolutePosition,
             InitRightFootAbsolutePosition,
             RelativeFootPositions,
             lStartingCOMState,
             lStartingZMPPosition);


  EndPhaseOfTheWalking(FinalZMPPositions,
                       FinalCOMStates,
                       LeftFootAbsolutePositions,
                       RightFootAbsolutePositions);


  FinalCOMStates.resize(FinalZMPPositions.size());
}

void ZMPDiscretization::
DumpFootAbsolutePosition
(string aFileName,
 deque<FootAbsolutePosition> &aFootAbsolutePositions)
{
  ofstream aof;
  aof.open(aFileName.c_str(),ofstream::out);
  if (aof.is_open())
    {
      for(unsigned int i=0; i<aFootAbsolutePositions.size(); i++)
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

void ZMPDiscretization::ResetADataFile(string &DataFile)
{
  if (DataFile.length()!=0)
    {
      std::ifstream a_iof;
      a_iof.open(DataFile.c_str(),std::ifstream::in);
      if (a_iof.is_open())
        {

          a_iof.close();
        }

    }
}
void ZMPDiscretization::
DumpDataFiles
(string ZMPFileName, string FootFileName,
 deque<ZMPPosition> & ZMPPositions,
 deque<FootAbsolutePosition> & SupportFootAbsolutePositions)
{
  ofstream aof;
  aof.open(ZMPFileName.c_str(),ofstream::out);
  if (aof.is_open())
    {
      for(unsigned int i=0; i<ZMPPositions.size(); i++)
        {
          aof << ZMPPositions[i].time << " "
              << ZMPPositions[i].px << " "
              << ZMPPositions[i].py << " "
              << ZMPPositions[i].stepType << " 0.0" <<   endl;
        }
      aof.close();
    }

  aof.open(FootFileName.c_str(),ofstream::out);
  if (aof.is_open())
    {
      for(unsigned int i=0; i<SupportFootAbsolutePositions.size(); i++)
        {
          aof << SupportFootAbsolutePositions[i].x << " " <<
            SupportFootAbsolutePositions[i].y << " "
              << SupportFootAbsolutePositions[i].z << " " <<
            SupportFootAbsolutePositions[i].stepType << " 0.0" <<  endl;
        }
      aof.close();
    }
}

void ZMPDiscretization::InitializeFilter()
{
  // Create the window for the filter.
  double T=0.05; // Arbritrary from Kajita's San Matlab files.
  int n=0;
  double sum=0,tmp=0;

  assert(m_SamplingPeriod > 0);
  n = (int)floor(T/m_SamplingPeriod);
  m_ZMPFilterWindow.resize(n+1);
  for(int i=0; i<n+1; i++)
    {
      tmp =sin((M_PI*i)/n);
      m_ZMPFilterWindow[i]=tmp*tmp;
    }

  for(int i=0; i<n+1; i++)
    sum+= m_ZMPFilterWindow[i];

  for(int i=0; i<n+1; i++)
    m_ZMPFilterWindow[i]/= sum;

}

void ZMPDiscretization::FilterZMPRef(deque<ZMPPosition> &ZMPPositionsX,
                                     deque<ZMPPosition> &ZMPPositionsY)
{
  int n=0;
  double T=0.050; // Arbritraty fixed from Kajita's San matlab files.
  deque<double> window;

  ZMPPositionsY.resize(ZMPPositionsX.size());
  // Creates window.
  assert(m_SamplingPeriod > 0);
  n = (int)floor(T/m_SamplingPeriod);

  // Filter ZMPref.

  // First part of the filter.
  for(int i=0; i<n+1; i++)
    {
      ZMPPositionsY[i] = ZMPPositionsX[i];
    }

  for(unsigned int i=n+1; i<ZMPPositionsX.size(); i++)
    {
      double ltmp[2]= {0,0};
      for(unsigned int j=0; j<m_ZMPFilterWindow.size(); j++)
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

  for (unsigned int i=0; i<ZMPShift.size(); i++)
    {
      m_ZMPShift[i] = ZMPShift[i];
    }


}

/* Start the online part of ZMP discretization. */

/* Initialiazation of the on-line stacks. */
std::size_t ZMPDiscretization::
InitOnLine
(deque<ZMPPosition> & FinalZMPPositions,
 deque<COMState> & FinalCoMStates,
 deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
 deque<FootAbsolutePosition> &RightFootAbsolutePositions,
 FootAbsolutePosition & InitLeftFootAbsolutePosition,
 FootAbsolutePosition & InitRightFootAbsolutePosition,
 deque<RelativeFootPosition> &RelativeFootPositions,
 COMState & lStartingCOMState,
 Eigen::Vector3d & lStartingZMPPosition)
{
  m_RelativeFootPositions.clear();
  FootAbsolutePosition CurrentLeftFootAbsPos, CurrentRightFootAbsPos;

  double CurrentAbsTheta=0.0;
  RESETDEBUG4("ZMDInitOnLine.txt");
  ODEBUG4("ZMP::InitOnLine - Step 1 ","ZMDInitOnLine.txt");

  // Initialize position of the current support foot.
  for(unsigned int i=0; i<3; i++)
    for(unsigned int j=0; j<3; j++)
      if (i==j)
        m_CurrentSupportFootPosition(i,j) = 1.0;
      else
        m_CurrentSupportFootPosition(i,j) = 0.0;

  m_PrevCurrentSupportFootPosition = m_CurrentSupportFootPosition;

  ODEBUG4("ZMP::InitOnLine - Step 2 ","ZMDInitOnLine.txt");
  // Initialize position of the feet.
  CurrentLeftFootAbsPos = InitLeftFootAbsolutePosition;
  CurrentLeftFootAbsPos.z = 0.0;
  CurrentLeftFootAbsPos.time = 0.0;

  ODEBUG4("CurrentLeftFootAbsPos.y: " << CurrentLeftFootAbsPos.y,
          "ZMDInitOnLine.txt");
  CurrentRightFootAbsPos = InitRightFootAbsolutePosition;
  CurrentRightFootAbsPos.z = 0.0;
  CurrentRightFootAbsPos.time = 0.0;

  // V pre is the difference between
  // the current support position and the precedent.
  ODEBUG4("ZMP::InitOnLine - Step 2.5 ","ZMDInitOnLine.txt");

  // The current heading direction is the center of mass
  // between the direction of the left foot and the direction of the right foot.
  double CurrentAbsZMPTheta=0;
  CurrentAbsZMPTheta = (CurrentRightFootAbsPos.theta +
                        CurrentLeftFootAbsPos.theta)/2.0;
  ODEBUG("CurrentZMPTheta at start: " << " "
         << CurrentRightFootAbsPos.theta << " "
         << CurrentLeftFootAbsPos.theta);

  // Initialize who is support foot.
  if (RelativeFootPositions[0].sy < 0 )
    {
      m_vdiffsupppre(0,0) = CurrentRightFootAbsPos.x -
        CurrentLeftFootAbsPos.x;
      m_vdiffsupppre(1,0) = CurrentRightFootAbsPos.y -
        CurrentLeftFootAbsPos.y;
      m_AngleDiffToSupportFootTheta = CurrentRightFootAbsPos.theta -
        CurrentLeftFootAbsPos.theta;
      m_AngleDiffFromZMPThetaToSupportFootTheta =
        CurrentRightFootAbsPos.theta -
        CurrentAbsZMPTheta;
    }
  else
    {
      m_vdiffsupppre(0,0) = -CurrentRightFootAbsPos.x +
        CurrentLeftFootAbsPos.x;
      m_vdiffsupppre(1,0) = -CurrentRightFootAbsPos.y +
        CurrentLeftFootAbsPos.y;
      m_AngleDiffToSupportFootTheta = CurrentLeftFootAbsPos.theta -
        CurrentRightFootAbsPos.theta;
      m_AngleDiffFromZMPThetaToSupportFootTheta =
        CurrentLeftFootAbsPos.theta -
        CurrentAbsZMPTheta;
    }

  ODEBUG4("ZMP::InitOnLine - Step 3 ","ZMDInitOnLine.txt");
  // Initialization of the ZMP position
  // (stable values during the Preview control time window).
  int AddArraySize;
  {
    assert(m_SamplingPeriod > 0);
    double ldAddArraySize = 2*m_PreviewControlTime/m_SamplingPeriod;
    AddArraySize = (int) ldAddArraySize;
  }

  ODEBUG(AddArraySize);
  deque<ZMPPosition> ZMPPositions;
  ZMPPositions.resize(AddArraySize);
  FinalCoMStates.resize(AddArraySize);
  LeftFootAbsolutePositions.resize(AddArraySize);
  RightFootAbsolutePositions.resize(AddArraySize);
  int CurrentZMPindex=0;

  // Also very important for the initialization: reshape
  // the ZMP reference for a smooth starting.
  double startingZMPREF[3] = { lStartingCOMState.x[0],
                               lStartingCOMState.y[0],
                               lStartingZMPPosition(2)};

  // Reset the current ZMP because of the formulation of
  // Kajita implying that the CoM starts at (0,0).
  startingZMPREF[0] = 0.0;
  startingZMPREF[1] = 0.0;
  startingZMPREF[2] = lStartingZMPPosition(2);

  // Make sure that the robot thinks it is at the position it thinks it is.
  // double startingZMPREF[3] =  { 0.00949035, 0.00142561,
  // lStartingZMPPosition(2)};
  double finalZMPREF[2] = {m_ZMPNeutralPosition[0],m_ZMPNeutralPosition[1]};
  ODEBUG("ZMPNeutralPosition: " << m_ZMPNeutralPosition[0] << " "
         << m_ZMPNeutralPosition[1] << endl
         << "StartingZMPPosition(toto):"
         << lStartingZMPPosition(0) << " "
         << lStartingZMPPosition(1) << " "
         << lStartingZMPPosition(2) << endl
         << "lStartingCOMState: " << lStartingCOMState.x[0] << " "
         << lStartingCOMState.y[0] << " "
         << lStartingCOMState.z[0] << endl
         << "CurrentAbsTheta : " << CurrentAbsTheta << endl
         << "AddArraySize:"<< AddArraySize << " "
         << m_PreviewControlTime << " "
         << m_SamplingPeriod  << endl
         << "FinalZMPref :( " <<finalZMPREF[0]
         << " , " <<finalZMPREF[1] << " ) "
         << ZMPPositions.size() <<endl
         << "InitRightFootAbsPos.z "
         << InitRightFootAbsolutePosition.z);
  ODEBUG( "lStartingCOMState: " << lStartingCOMState.x[0] << " "
          << lStartingCOMState.y[0] << " "
          << lStartingCOMState.z[0] );

  ODEBUG4("ZMP::InitOnLine - Step 4 ","ZMDInitOnLine.txt");
  for(unsigned int i=0; i<ZMPPositions.size(); i++)
    {
      double coef = (double)i/(double)ZMPPositions.size();
      double icoef = (double)(ZMPPositions.size() -i)/
        (double)ZMPPositions.size();
      // Set ZMP positions.

      // Smooth ramp
      ZMPPositions[CurrentZMPindex].px = startingZMPREF[0] +
        (finalZMPREF[0] -  startingZMPREF[0])*coef;
      ZMPPositions[CurrentZMPindex].py =
        startingZMPREF[1] + (finalZMPREF[1] - startingZMPREF[1])*coef;
      ZMPPositions[CurrentZMPindex].pz =
        (-startingZMPREF[2] +
         InitRightFootAbsolutePosition.z) * icoef;
      ZMPPositions[CurrentZMPindex].theta = CurrentAbsTheta;
      ZMPPositions[CurrentZMPindex].time = m_CurrentTime;
      ZMPPositions[CurrentZMPindex].stepType = 0;

      // Set CoM positions.
      FinalCoMStates[CurrentZMPindex].z[0] = m_ComHeight;
      FinalCoMStates[CurrentZMPindex].z[1] = 0.0;
      FinalCoMStates[CurrentZMPindex].z[2] = 0.0;

      FinalCoMStates[CurrentZMPindex].pitch[0] =
        FinalCoMStates[CurrentZMPindex].pitch[1] =
        FinalCoMStates[CurrentZMPindex].pitch[2] = 0.0;

      FinalCoMStates[CurrentZMPindex].roll[0] =
        FinalCoMStates[CurrentZMPindex].roll[1] =
        FinalCoMStates[CurrentZMPindex].roll[2] = 0.0;

      FinalCoMStates[CurrentZMPindex].yaw[0] =
        ZMPPositions[CurrentZMPindex].theta;
      FinalCoMStates[CurrentZMPindex].yaw[1] =
        FinalCoMStates[CurrentZMPindex].yaw[2] = 0.0;

      // Set Left and Right Foot positions.
      LeftFootAbsolutePositions[CurrentZMPindex] =
        CurrentLeftFootAbsPos;
      RightFootAbsolutePositions[CurrentZMPindex] =
        CurrentRightFootAbsPos;

      LeftFootAbsolutePositions[CurrentZMPindex].time =
        RightFootAbsolutePositions[CurrentZMPindex].time =
        m_CurrentTime;

      LeftFootAbsolutePositions[CurrentZMPindex].stepType =
        RightFootAbsolutePositions[CurrentZMPindex].stepType = 10;

      m_CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
    }


  // The first foot when walking dynamically
  // does not leave the soil, but needs to be treated for the first phase.
  m_RelativeFootPositions.push_back(RelativeFootPositions[0]);


  if (1)
    {
      ofstream dbg_aof("DebugZMPRefPos.dat",ofstream::out);
      for(unsigned int i=0; i<ZMPPositions.size(); i++)
        {
          dbg_aof << ZMPPositions[i].px << " "
                  << ZMPPositions[i].py << endl;
        }
      dbg_aof.close();
    }
  FilterOutValues(ZMPPositions,
                  FinalZMPPositions,
                  true);

  ODEBUG5("InitOnLine","DebugDataRFPos.txt" );
  for(unsigned int i=1; i<RelativeFootPositions.size(); i++)
    {
      OnLineAddFoot(RelativeFootPositions[i],
                    FinalZMPPositions,
                    FinalCoMStates,
                    LeftFootAbsolutePositions,
                    RightFootAbsolutePositions,
                    false);
    }
  ODEBUG5("ZMP::InitOnLine: End ","ZMDInitOnLine.txt");

  return RelativeFootPositions.size();
}

void ZMPDiscretization::
UpdateCurrentSupportFootPosition
( RelativeFootPosition aRFP)
{
  m_PrevCurrentSupportFootPosition = m_CurrentSupportFootPosition;

  // First orientation
  double c = cos(aRFP.theta*M_PI/180.0);
  double s = sin(aRFP.theta*M_PI/180.0);
  Eigen::Matrix<double,2,2> MM;;
  Eigen::Matrix<double,2,2> Orientation;;

  MM(0,0) = c;
  MM(0,1) = -s;
  MM(1,0) = s;
  MM(1,1) = c;
  for(int k=0; k<2; k++)
    for(int l=0; l<2; l++)
      Orientation(k,l) = m_CurrentSupportFootPosition(k,l);

  // second position.
  Eigen::Matrix<double,2,1> v;;
  Eigen::Matrix<double,2,1> v2;;

  v(0,0) = aRFP.sx;
  v(1,0) = aRFP.sy;

  Orientation = MM*Orientation;
  v2=Orientation*v;
  ODEBUG("v :" << v  << " "
         "v2 : " << v2 << " "
         "Orientation : " << Orientation << " "
         "CurrentSupportFootPosition: " << m_CurrentSupportFootPosition );


  for(int k=0; k<2; k++)
    for(int l=0; l<2; l++)
      m_CurrentSupportFootPosition(k,l) = Orientation(k,l);

  for(int k=0; k<2; k++)
    m_CurrentSupportFootPosition(k,2) += v2(k,0);

  ODEBUG("v :" << v  << " "
         "v2 : " << v2 << " "
         "Orientation : " << Orientation << " "
         "CurrentSupportFootPosition: " << m_CurrentSupportFootPosition );

}


void ZMPDiscretization::OnLine
(double, // time,
 deque<ZMPPosition> &, // FinalZMPPositions,
 deque<COMState> &,  //FinalCOMStates,
 deque<FootAbsolutePosition> &,//FinalLeftFootAbsolutePositions,
 deque<FootAbsolutePosition> &)//FinalRightFootAbsolutePositions)
{
  /* Does nothing... */
}

/* The interface method which returns an appropriate update of the
   appropriate stacks (ZMPRef, FootPosition) depending on the
   state of the relative steps stack. */
void ZMPDiscretization::OnLineAddFoot
(RelativeFootPosition & NewRelativeFootPosition,
 deque<ZMPPosition> & FinalZMPPositions,
 deque<COMState> & FinalCOMStates,
 deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
 deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
 bool EndSequence)
{
  deque<ZMPPosition> ZMPPositions;
  deque<FootAbsolutePosition> LeftFootAbsolutePositions;
  deque<FootAbsolutePosition> RightFootAbsolutePositions;
  FootAbsolutePosition CurrentLeftFootAbsPos, CurrentRightFootAbsPos;
  double CurrentAbsZMPTheta=0;

  CurrentLeftFootAbsPos = FinalLeftFootAbsolutePositions.back();
  CurrentRightFootAbsPos = FinalRightFootAbsolutePositions.back();
  CurrentAbsZMPTheta = FinalZMPPositions.back().theta;

  m_RelativeFootPositions.push_back(NewRelativeFootPosition);
  int WhoIsSupportFoot=1;
  double TimeFirstPhase=0.0;
  int CurrentZMPindex=0;
  Eigen::Matrix<double,2,1> vdiffsupp;;
  Eigen::Matrix<double,2,1> vrel;;

  double lTdble=m_Tdble, lTsingle=m_Tsingle;

  ODEBUG5(m_RelativeFootPositions[0].sx << " "  <<
          m_RelativeFootPositions[0].sy << " " <<
          m_RelativeFootPositions[0].theta,"DebugDataRFPos.txt" );
  ODEBUG(" OnLineAddFoot: m_RelativeFootPositions.size: "
         <<  m_RelativeFootPositions.size());
  ODEBUG(" OnLineAddFoot: "<< endl <<
         " NewRelativeFootPositions.x: " <<  NewRelativeFootPosition.sx <<
         " NewRelativeFootPositions.y: " <<  NewRelativeFootPosition.sy <<
         " NewRelativeFootPositions.theta: " <<  NewRelativeFootPosition.theta
         );
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
      m_vdiffsupppre(0,0) = CurrentRightFootAbsPos.x -
        CurrentLeftFootAbsPos.x;
      m_vdiffsupppre(1,0) = CurrentRightFootAbsPos.y -
        CurrentLeftFootAbsPos.y;
      m_AngleDiffToSupportFootTheta = CurrentRightFootAbsPos.theta -
        CurrentLeftFootAbsPos.theta;
      m_AngleDiffFromZMPThetaToSupportFootTheta =
        CurrentRightFootAbsPos.theta -
        CurrentAbsZMPTheta;
    }
  else
    {
      WhoIsSupportFoot = 1;// Left
      m_vdiffsupppre(0,0) = -CurrentRightFootAbsPos.x +
        CurrentLeftFootAbsPos.x;
      m_vdiffsupppre(1,0) = -CurrentRightFootAbsPos.y +
        CurrentLeftFootAbsPos.y;
      m_AngleDiffToSupportFootTheta = CurrentLeftFootAbsPos.theta -
        CurrentRightFootAbsPos.theta;
      m_AngleDiffFromZMPThetaToSupportFootTheta = CurrentLeftFootAbsPos.theta -
        CurrentAbsZMPTheta;
    }


  double TimeForThisFootPosition = TimeFirstPhase+ lTsingle;
  ODEBUG5("TimeFirstPhase: " << TimeFirstPhase << " lTsingle: " << lTsingle,
          "DebugData.txt");
  // Compute the size of cells to add inside the array.
  assert(m_SamplingPeriod > 0);
  double l2AddArraySize= TimeForThisFootPosition/m_SamplingPeriod;
  int AddArraySize = (unsigned int)round(l2AddArraySize);
  ODEBUG("Added part: "<<AddArraySize << " " << l2AddArraySize <<
         " TimeForThisFootPosition " << TimeForThisFootPosition <<
         " SamplingPeriod" << m_SamplingPeriod);
  ZMPPositions.resize(AddArraySize);
  LeftFootAbsolutePositions.resize(AddArraySize);
  RightFootAbsolutePositions.resize(AddArraySize);

  m_CurrentAbsTheta+= m_RelativeFootPositions[0].theta;
  //  m_CurrentAbsTheta = fmod(m_CurrentAbsTheta,180);

  // Computes the new ABSOLUTE position of the supporting foot .
  UpdateCurrentSupportFootPosition(m_RelativeFootPositions[0]);

  // First Phase of the step cycle.
  assert(m_SamplingPeriod > 0);
  double dSizeOf1stPhase =TimeFirstPhase/m_SamplingPeriod;
  unsigned int SizeOf1stPhase = (unsigned int)round(dSizeOf1stPhase);
  ODEBUG("SizeOf1stPhase:" << SizeOf1stPhase);
  ODEBUG("m_vdiffsupppre : " << m_vdiffsupppre);
  double px0,py0,theta0, delta_x,delta_y;

  ODEBUG("FinalZMPPositions.size() = " <<FinalZMPPositions.size());
  // Initial value to start the new ZMP profile.
  px0 = FinalZMPPositions.back().px;
  py0 = FinalZMPPositions.back().py;
  theta0 = FinalZMPPositions.back().theta;

  Eigen::Matrix<double,3,1> ZMPInFootCoordinates;

  ZMPInFootCoordinates[0] = m_ZMPNeutralPosition[0];
  ZMPInFootCoordinates[1] = m_ZMPNeutralPosition[1];
  ZMPInFootCoordinates[2] = 1.0;

  Eigen::Matrix<double,3,1> ZMPInWorldCoordinates;

  ZMPInWorldCoordinates=m_CurrentSupportFootPosition*ZMPInFootCoordinates;

  delta_x = (ZMPInWorldCoordinates(0) - px0)/SizeOf1stPhase;
  delta_y = (ZMPInWorldCoordinates(1) - py0)/SizeOf1stPhase;

  ODEBUG("delta_x :"
         << delta_x << " delta_y : "
         << delta_y << " m_CurrentSFP: " <<
         m_CurrentSupportFootPosition << " ZMPInFC : " <<
         ZMPInFootCoordinates << " ZMPinWC : " <<
         ZMPInWorldCoordinates << " px0: " << px0 << " py0:" << py0
         );
  ODEBUG5("Step 4 TimeForThisFootPosition "
          << TimeForThisFootPosition,"DebugData.txt");

  // ZMP profile is changed if the stepping over is on, and then
  // depends on the phase during stepping over.
  bool DoIt = 1;
  if (DoIt)
    {
      if (m_RelativeFootPositions[1].stepType == 3)
        {
          //delta_x = (m_CurrentSupportFootPosition(0,2)+
          // m_ZMPShift3Begin - px0)/SizeOf1stPhase;
          delta_x = (m_CurrentSupportFootPosition
                     (0,2)+m_ZMPShift[0] - px0)/SizeOf1stPhase;
          // delta_y = (m_CurrentSupportFootPosition(1,2)+
          // (WhoIsSupportFoot)*m_ZMPShift3BeginY - py0)/SizeOf1stPhase;
          delta_y = (m_CurrentSupportFootPosition(1,2) - py0)/SizeOf1stPhase;

        }
      if (m_RelativeFootPositions[1].stepType == 4)
        {
          delta_x = (m_CurrentSupportFootPosition(0,2)+m_ZMPShift[2] -
             px0)/SizeOf1stPhase;
          delta_y = (m_CurrentSupportFootPosition(1,2)- py0)/SizeOf1stPhase;
          // delta_x = (CurrentSupportFootPosition(0,2)
          // +m_ZMPShift4Begin - px0)/SizeOf1stPhase;
          // delta_y = (CurrentSupportFootPosition(1,2)+(WhoIsSupportFoot)
          // *m_ZMPShift3BeginY - py0)/SizeOf1stPhase;
        }

      if (m_RelativeFootPositions[1].stepType == 5)
        {
          delta_x = (m_CurrentSupportFootPosition(0,2)-
                     (m_ZMPShift[0] +m_ZMPShift[2]+
                      m_ZMPShift[1] +m_ZMPShift[3]) - px0)/SizeOf1stPhase;
          delta_y = (m_CurrentSupportFootPosition(1,2)- py0)/SizeOf1stPhase;
          //delta_x = (CurrentSupportFootPosition(0,2)-(m_ZMPShift3Begin +
          // m_ZMPShift4Begin+m_ZMPShift3End + m_ZMPShift4End) -
          // px0)/SizeOf1stPhase;
          //delta_y = (CurrentSupportFootPosition(1,2)-
          // (WhoIsSupportFoot)*(m_ZMPShift3BeginY +
          // m_ZMPShift4BeginY+m_ZMPShift3EndY + m_ZMPShift4EndY) -
          // py0)/SizeOf1stPhase;
        }

    }

  ODEBUG5(" GetZMPDiscretization: Step 5 " << AddArraySize << " ",
          "DebugData.txt");
  ODEBUG("SizeOf1stPhase: " << SizeOf1stPhase << "dx: " << delta_x
         << " dy: " << delta_y);

  // First phase of the cycle aka Double support phase.
  for(unsigned int k=0; k<SizeOf1stPhase; k++)
    {

      ZMPPositions[CurrentZMPindex].px = px0 + k*delta_x;
      ZMPPositions[CurrentZMPindex].py = py0 + k*delta_y;
      ZMPPositions[CurrentZMPindex].pz = 0;
      ZMPPositions[CurrentZMPindex].theta =theta0;


      ZMPPositions[CurrentZMPindex].time = m_CurrentTime;

      ZMPPositions[CurrentZMPindex].stepType =
        m_RelativeFootPositions[1].stepType+10;

      // Right now the foot is not moving during the double support
      // TO DO: whatever you need to do ramzi....
      LeftFootAbsolutePositions[CurrentZMPindex] =
        FinalLeftFootAbsolutePositions.back();

      // WARNING : This assume that you are walking on a plane.
      LeftFootAbsolutePositions[CurrentZMPindex].z = 0.0;

      RightFootAbsolutePositions[CurrentZMPindex] =
        FinalRightFootAbsolutePositions.back();

      // WARNING : This assume that you are walking on a plane.
      RightFootAbsolutePositions[CurrentZMPindex].z = 0.0;

      LeftFootAbsolutePositions[CurrentZMPindex].time =
        RightFootAbsolutePositions[CurrentZMPindex].time = m_CurrentTime;

      LeftFootAbsolutePositions[CurrentZMPindex].stepType =
        RightFootAbsolutePositions[CurrentZMPindex].stepType =
        m_RelativeFootPositions[1].stepType+10;
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

  // Second Phase of the step cycle aka Single Support Phase.

  // Next Theta : next relative angle between the current support foot angle
  // and the next support foot angle.
  double NextTheta=0;

  // RelTheta : relative angle between the current angle of the
  // flying foot and the next angle of the flying foot.
  // RelZMPTheta : relative angle between the current angle of the
  // zmp and the next angle of the zmp.
  double RelTheta=0, RelZMPTheta=0;
  if (m_RelativeFootPositions.size()>1)
    {
      NextTheta=m_RelativeFootPositions[1].theta;
      RelTheta = NextTheta+m_AngleDiffToSupportFootTheta;
      RelZMPTheta = NextTheta+m_AngleDiffFromZMPThetaToSupportFootTheta;

      double c,s;
      c = cos(NextTheta*M_PI/180.0);
      s = sin(NextTheta*M_PI/180.0);
      Eigen::Matrix<double,2,2> Orientation;;
      Eigen::Matrix<double,2,1> v;;
      Orientation(0,0) = c;
      Orientation(0,1) = -s;
      Orientation(1,0) = s;
      Orientation(1,1) = c;

      Eigen::Matrix<double,2,2> SubOrientation;;
      SubOrientation = m_CurrentSupportFootPosition.block(0,0,2,2);
      Orientation = Orientation*SubOrientation ;

      v(0,0) = m_RelativeFootPositions[1].sx;
      v(1,0) = m_RelativeFootPositions[1].sy;
      vdiffsupp=Orientation*v;

      vrel = vdiffsupp + m_vdiffsupppre;

      // Compute relative feet orientation for the next step
    }
  else
    {
      vrel(0,0)= 0.0;
      vrel(1,0)= 0.0;
      RelTheta= 0.0;
      NextTheta=0.0;
    }

  ODEBUG(cout << "vrel: " << vrel(0,0) << " " << vrel(1,0));
  ODEBUG(cout << "vdiffsupp: " << vdiffsupp(0,0) << " " << vdiffsupp(1,0));
  ODEBUG( "vdiffsupppre: " << m_vdiffsupppre(0,0) << " "
          << m_vdiffsupppre(1,0));

  ODEBUG5(" GetZMPDiscretization: Step 6 "
          << ZMPPositions.size() << " ","DebugData.txt");


  m_vdiffsupppre = vdiffsupp;

  // m_AngleDiffToSupportFootTheta = NextTheta;


  // Create the polynomes for the none-support foot.
  // Change 08/12/2005: Speed up the modification of X and Y
  // for vertical landing of the foot (Kajita-San's trick n 1)
  //   double ModulationSupportCoefficient = 0.9;
  double ModulatedSingleSupportTime = lTsingle * m_ModulationSupportCoefficient;
  double EndOfLiftOff = (lTsingle-ModulatedSingleSupportTime)*0.5;
  ODEBUG("ModulatedSingleSupportTime:" << ModulatedSingleSupportTime << " "
         << vrel(0,0) << " "
         << vrel(1,0));
  m_FootTrajectoryGenerationStandard->
    SetParameters(FootTrajectoryGenerationStandard::X_AXIS,
                  ModulatedSingleSupportTime,vrel(0,0));
  m_FootTrajectoryGenerationStandard->
    SetParameters(FootTrajectoryGenerationStandard::Y_AXIS,
                  ModulatedSingleSupportTime,vrel(1,0));
  m_FootTrajectoryGenerationStandard->
    SetParameters(FootTrajectoryGenerationStandard::Z_AXIS,
                  m_Tsingle,0);
  m_FootTrajectoryGenerationStandard->
    SetParameters(FootTrajectoryGenerationStandard::THETA_AXIS,
                  ModulatedSingleSupportTime,RelTheta);
  m_FootTrajectoryGenerationStandard->
    SetParameters(FootTrajectoryGenerationStandard::OMEGA_AXIS,
                  EndOfLiftOff,m_Omega);
  m_FootTrajectoryGenerationStandard->
    SetParameters(FootTrajectoryGenerationStandard::OMEGA2_AXIS,
                  ModulatedSingleSupportTime,2*m_Omega);

  //  m_FootTrajectoryGenerationStandard->print();

  //m_PolynomeZMPTheta->SetParameters(lTsingle,NextZMPTheta);
  m_PolynomeZMPTheta->SetParameters(lTsingle,RelZMPTheta);
  assert(m_SamplingPeriod > 0);
  double dSizeOfSndPhase = lTsingle/m_SamplingPeriod;
  unsigned int SizeOfSndPhase = (unsigned int)round(dSizeOfSndPhase);
  int indexinitial = CurrentZMPindex-1;

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
  for(unsigned int k=0; k<SizeOfSndPhase; k++)
    {

      Eigen::Matrix<double,3,1> ZMPInFootCoordinates;

      ZMPInFootCoordinates[0] = m_ZMPNeutralPosition[0];
      ZMPInFootCoordinates[1] = m_ZMPNeutralPosition[1];
      ZMPInFootCoordinates[2] = 1.0;

      Eigen::Matrix<double,3,1> ZMPInWorldCoordinates;

      ZMPInWorldCoordinates=m_CurrentSupportFootPosition*ZMPInFootCoordinates;

      ODEBUG5("CSFP: " << m_CurrentSupportFootPosition << endl <<
              "ZMPiWC"  << ZMPInWorldCoordinates << endl, "DebugData.txt");

      ZMPPositions[CurrentZMPindex].px = ZMPInWorldCoordinates(0);
      ZMPPositions[CurrentZMPindex].py = ZMPInWorldCoordinates(1);
      ZMPPositions[CurrentZMPindex].pz = 0;
      ZMPPositions[CurrentZMPindex].time = m_CurrentTime;

      if (DoIt)
        {
          if ((m_RelativeFootPositions[1].stepType == 3)
              ||(m_RelativeFootPositions[1].stepType == 4))
            {


              if (m_RelativeFootPositions[1].stepType == 3)
                {
                  delta_x =
                    (m_CurrentSupportFootPosition(0, 2)
                     +m_ZMPShift[1] - px02)/SizeOfSndPhase;
                  delta_y = (m_CurrentSupportFootPosition(1,2) - py02)/
                    SizeOfSndPhase;
                  // delta_x = (CurrentSupportFootPosition(0,2)+
                  //  m_ZMPShift3End - px02)/SizeOfSndPhase;
                  // delta_y = (CurrentSupportFootPosition(1,2)+
                  //  (WhoIsSupportFoot)*m_ZMPShift3EndY - py02)/SizeOfSndPhase;
                }
              else
                {
                  delta_x =
                    (m_CurrentSupportFootPosition(0, 2)+
                     m_ZMPShift[3] - px02)/SizeOfSndPhase;
                  delta_y = (m_CurrentSupportFootPosition(1,2)
                             - py02)/SizeOfSndPhase;
                  // delta_x = (CurrentSupportFootPosition(0,2)+
                  //  m_ZMPShift4End - px02)/SizeOfSndPhase;
                  // delta_y = (CurrentSupportFootPosition(1,2)+
                  //  (WhoIsSupportFoot)*m_ZMPShift4EndY - py02)/SizeOfSndPhase;
                }

              ZMPPositions[CurrentZMPindex].px =
                ZMPPositions[CurrentZMPindex-1].px + delta_x;
              ZMPPositions[CurrentZMPindex].py =
                ZMPPositions[CurrentZMPindex-1].py + delta_y;
              ZMPPositions[CurrentZMPindex].pz = 0;
            }

        }

      ZMPPositions[CurrentZMPindex].theta =
        m_PolynomeZMPTheta->Compute(k*m_SamplingPeriod) +
        ZMPPositions[indexinitial].theta;

      ZMPPositions[CurrentZMPindex].stepType =
        WhoIsSupportFoot*m_RelativeFootPositions[0].stepType;
      if (WhoIsSupportFoot==1)
        {
          m_FootTrajectoryGenerationStandard->
            UpdateFootPosition
            (LeftFootAbsolutePositions,
             RightFootAbsolutePositions,
             CurrentZMPindex,indexinitial,
             ModulatedSingleSupportTime,
             m_RelativeFootPositions[1].stepType,
             -1);
        }
      else
        {
          m_FootTrajectoryGenerationStandard->
            UpdateFootPosition
            (RightFootAbsolutePositions,
             LeftFootAbsolutePositions,
             CurrentZMPindex,indexinitial,
             ModulatedSingleSupportTime,
             m_RelativeFootPositions[1].stepType,
             1);
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

  m_RelativeFootPositions.pop_front();

  for(unsigned int i=0; i<ZMPPositions.size(); i++)
    {
      COMState aCOMState;
      aCOMState.x[0] =
        aCOMState.x[1] =
        aCOMState.x[2] = 0.0;

      aCOMState.y[0] =
        aCOMState.y[1] =
        aCOMState.y[2] = 0.0;

      aCOMState.z[0] = m_ComHeight;

      aCOMState.pitch[0] =
        aCOMState.pitch[1] =
        aCOMState.pitch[2] =
        aCOMState.roll[0] =
        aCOMState.roll[1] =
        aCOMState.roll[2] =
        aCOMState.yaw[1] =
        aCOMState.yaw[2] = 0.0;

      aCOMState.z[1] = aCOMState.z[2] = 0.0;

      aCOMState.yaw[0] = ZMPPositions[i].theta;

      FinalCOMStates.push_back(aCOMState);
      FinalLeftFootAbsolutePositions.push_back
        (LeftFootAbsolutePositions[i]);
      FinalRightFootAbsolutePositions.push_back
        (RightFootAbsolutePositions[i]);

    }
  FilterOutValues(ZMPPositions,
                  FinalZMPPositions,false);

  ODEBUG_CODE(DumpReferences(FinalZMPPositions,ZMPPositions));

  if (EndSequence)
    {

      // End Phase of the walking includes the filtering.
      EndPhaseOfTheWalking(FinalZMPPositions,
                           FinalCOMStates,
                           FinalLeftFootAbsolutePositions,
                           FinalRightFootAbsolutePositions);
    }

}

void ZMPDiscretization::
DumpReferences
(deque<ZMPPosition> & FinalZMPPositions,
 deque<ZMPPosition> & ZMPPositions)
{


  ofstream dbg_aof("DebugZMPRefPos.dat",ofstream::app);
  for(unsigned int i=0; i<ZMPPositions.size(); i++)
    {
      dbg_aof << ZMPPositions[i].px << " "
              << ZMPPositions[i].py << endl;
    }
  dbg_aof.close();

  dbg_aof.open("DebugFinalZMPRefPos.dat",ofstream::app);
  for(unsigned int i=0; i<FinalZMPPositions.size(); i++)
    {
      dbg_aof << FinalZMPPositions[i].px << " "
              << FinalZMPPositions[i].py << endl;
    }
  dbg_aof.close();

}

void ZMPDiscretization::
FilterOutValues
(deque<ZMPPosition> &ZMPPositions,
 deque<ZMPPosition> &FinalZMPPositions,
 bool InitStep)
{
  unsigned int lshift=2;
  // Filter out the ZMP values.
  for(unsigned int i=0; i<ZMPPositions.size(); i++)
    {
      double ltmp[3]= {0,0,0};

      std::size_t o= FinalZMPPositions.size()-1-lshift;
      for(unsigned int j=0; j<m_ZMPFilterWindow.size(); j++)
        {
          int r;
          r=i-j+lshift;
          if (r<0)
            {

              if (InitStep)
                {
                  ltmp[0] += m_ZMPFilterWindow[j]*ZMPPositions[lshift].px;
                  ltmp[1] += m_ZMPFilterWindow[j]*ZMPPositions[lshift].py;
                  ltmp[2] += m_ZMPFilterWindow[j]*ZMPPositions[lshift].pz;
                }
              else
                {
                  if (-r<(int) o)
                    {
                      ltmp[0] += m_ZMPFilterWindow[j]*FinalZMPPositions[o+r].px;
                      ltmp[1] += m_ZMPFilterWindow[j]*FinalZMPPositions[o+r].py;
                      ltmp[2] += m_ZMPFilterWindow[j]*FinalZMPPositions[o+r].pz;
                    }
                  else
                    {
                      ltmp[0] += m_ZMPFilterWindow[j]*FinalZMPPositions[0].px;
                      ltmp[1] += m_ZMPFilterWindow[j]*FinalZMPPositions[0].py;
                      ltmp[2] += m_ZMPFilterWindow[j]*FinalZMPPositions[0].pz;
                    }
                }
            }
          else
            {
              if (r>=(int)ZMPPositions.size())
                r = (int)ZMPPositions.size()-1;

              ltmp[0] += m_ZMPFilterWindow[j]*ZMPPositions[r].px;
              ltmp[1] += m_ZMPFilterWindow[j]*ZMPPositions[r].py;
              ltmp[2] += m_ZMPFilterWindow[j]*ZMPPositions[r].pz;
            }
        }

      ZMPPosition aZMPPos;
      aZMPPos.px = ltmp[0];
      aZMPPos.py = ltmp[1];
      aZMPPos.pz = ltmp[2];
      aZMPPos.theta = ZMPPositions[i].theta;
      aZMPPos.time = ZMPPositions[i].time;
      aZMPPos.stepType = ZMPPositions[i].stepType;

      FinalZMPPositions.push_back(aZMPPos);
    }
  ODEBUG("ZMPPosition.back=( " <<ZMPPositions.back().px << " , " <<
         ZMPPositions.back().py << " )");
  ODEBUG("FinalZMPPosition.back=( " <<FinalZMPPositions.back().px << " , " <<
         FinalZMPPositions.back().py << " )");
  ODEBUG("FinalZMPPositions.size()="<<FinalZMPPositions.size());
}

int ZMPDiscretization::
OnLineFootChange
(double, //time,
 FootAbsolutePosition &, //aFootAbsolutePosition,
 deque<ZMPPosition> &, //FinalZMPPositions,
 deque<COMState> &, //CoMStates,
 deque<FootAbsolutePosition> &, //FinalLeftFootAbsolutePositions,
 deque<FootAbsolutePosition> &, //FinalRightFootAbsolutePositions,
 StepStackHandler * )//aStepStackHandler)
{
  return -1;
}

int ZMPDiscretization::ReturnOptimalTimeToRegenerateAStep()
{
  assert(m_SamplingPeriod > 0);
  int r = (int)(m_PreviewControlTime/m_SamplingPeriod);
  return 2*r;
}

void ZMPDiscretization::EndPhaseOfTheWalking
(  deque<ZMPPosition>
   &FinalZMPPositions,
   deque<COMState> &FinalCOMStates,
   deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
   deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)
  
{
  deque<ZMPPosition> ZMPPositions;
  FootAbsolutePosition LeftFootAbsolutePosition;
  FootAbsolutePosition RightFootAbsolutePosition;

  ODEBUG("m_RelativeFootPositions.size(): " << m_RelativeFootPositions.size());
  if (m_RelativeFootPositions.size()>0)
    UpdateCurrentSupportFootPosition(m_RelativeFootPositions[0]);

  // Deal with the end phase of the walking.
  assert(m_SamplingPeriod > 0);
  double dlAddArraySize = m_Tdble/(2*m_SamplingPeriod);
  unsigned int AddArraySize = (unsigned int)round(dlAddArraySize);

  std::size_t currentsize = 0;
  unsigned int CurrentZMPindex = 0;
  ZMPPositions.resize(currentsize+AddArraySize);
  ODEBUG5(" GetZMPDiscretization: Step 7 " << currentsize
          << " " << AddArraySize,
          "DebugData.txt");

  double dSizeOfEndPhase = m_Tdble/(2*m_SamplingPeriod);
  unsigned int SizeOfEndPhase = (unsigned int)round(dSizeOfEndPhase);
  double px0,py0,delta_x,delta_y;
  double pxf=0,pyf=0;
  px0 = FinalZMPPositions.back().px;
  py0 = FinalZMPPositions.back().py;
  //  int lindex = SupportFootAbsolutePositions.size()-1;

  // We assume that the last positon of the ZMP
  // will the middle of the two last position
  // of the support foot.
  ODEBUG("Cur/Prev SuppFootPos (X) : " <<m_CurrentSupportFootPosition(0,2)
         << " " << m_PrevCurrentSupportFootPosition(0,2));
  ODEBUG("Cur/Prev SuppFootPos (Y) : " <<m_CurrentSupportFootPosition(1,2)
         << " " << m_PrevCurrentSupportFootPosition(1,2));

  pxf = 0.5*(m_CurrentSupportFootPosition
             (0,2) + m_PrevCurrentSupportFootPosition(0,2));
  pyf = 0.5*(m_CurrentSupportFootPosition(1,2) +
             m_PrevCurrentSupportFootPosition(1,2));

  delta_x = (pxf - px0)/(double)SizeOfEndPhase;
  delta_y = (pyf - py0)/(double)SizeOfEndPhase;

  ZMPPositions[0].px = px0 + delta_x;
  ZMPPositions[0].py = py0 + delta_y;
  ZMPPositions[0].time = m_CurrentTime;

  ZMPPositions[0].theta = FinalZMPPositions.back().theta;
  ZMPPositions[0].stepType=0;
  CurrentZMPindex++;

  LeftFootAbsolutePosition =
    FinalLeftFootAbsolutePositions.back();
  RightFootAbsolutePosition =
    FinalRightFootAbsolutePositions.back();

  LeftFootAbsolutePosition.time =
    RightFootAbsolutePosition.time = m_CurrentTime;

  LeftFootAbsolutePosition.stepType =
    RightFootAbsolutePosition.stepType = 0;

  FinalLeftFootAbsolutePositions.push_back(LeftFootAbsolutePosition);
  FinalRightFootAbsolutePositions.push_back(RightFootAbsolutePosition);

  m_CurrentTime += m_SamplingPeriod;

  for(unsigned int k=1; k<SizeOfEndPhase; k++)
    {

      // Set ZMP positions.
      ZMPPositions[CurrentZMPindex].px =
        ZMPPositions[CurrentZMPindex-1].px + delta_x;
      ZMPPositions[CurrentZMPindex].py =
        ZMPPositions[CurrentZMPindex-1].py + delta_y;
      ZMPPositions[CurrentZMPindex].time = m_CurrentTime;
      ZMPPositions[CurrentZMPindex].theta =
        ZMPPositions[CurrentZMPindex-1].theta;

      ZMPPositions[CurrentZMPindex].stepType =0;

      // Set CoM Positions.
      COMState aCOMState;

      aCOMState.z[0] = m_ComHeight;
      aCOMState.z[1] = aCOMState.z[2] = 0.0;
      aCOMState.yaw[0] = ZMPPositions[CurrentZMPindex].theta;

      FinalCOMStates.push_back(aCOMState);

      // Set Feet positions.
      LeftFootAbsolutePosition =
        FinalLeftFootAbsolutePositions.back();
      RightFootAbsolutePosition =
        FinalRightFootAbsolutePositions.back();

      LeftFootAbsolutePosition.time =
        RightFootAbsolutePosition.time = m_CurrentTime;

      LeftFootAbsolutePosition.stepType =
        RightFootAbsolutePosition.stepType = 0;

      FinalLeftFootAbsolutePositions.push_back(LeftFootAbsolutePosition);
      FinalRightFootAbsolutePositions.push_back(RightFootAbsolutePosition);


      m_CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;

    }

  // Added a new phase for exhausting the preview control
  {
    assert(m_SamplingPeriod > 0);
    double ldAddArraySize = 3.0*m_PreviewControlTime/m_SamplingPeriod;
    AddArraySize = (int)ldAddArraySize;
  }

  currentsize = ZMPPositions.size();
  ZMPPositions.resize(currentsize+AddArraySize);

  ODEBUG5(" GetZMPDiscretization: Step 8 ","DebugData.txt");

  for(unsigned int i=0; i<AddArraySize; i++)
    {

      // Set ZMP Positions
      ZMPPositions[CurrentZMPindex].px = ZMPPositions[CurrentZMPindex-1].px;
      ZMPPositions[CurrentZMPindex].py = ZMPPositions[CurrentZMPindex-1].py;
      ZMPPositions[CurrentZMPindex].theta =
        ZMPPositions[CurrentZMPindex-1].theta;
      ZMPPositions[CurrentZMPindex].time = m_CurrentTime;

      ZMPPositions[CurrentZMPindex].stepType = 0;


      // Set CoM Positions.
      COMState aCOMState;
      aCOMState.z[0] = m_ComHeight;

      aCOMState.z[1] = aCOMState.z[2] = 0.0;
      aCOMState.yaw[0] = ZMPPositions[CurrentZMPindex].theta;
      FinalCOMStates.push_back(aCOMState);

      // Set Feet Positions
      LeftFootAbsolutePosition=
        FinalLeftFootAbsolutePositions.back();
      RightFootAbsolutePosition =
        FinalRightFootAbsolutePositions.back();


      LeftFootAbsolutePosition.time =
        RightFootAbsolutePosition.time = m_CurrentTime;

      LeftFootAbsolutePosition.stepType =
        RightFootAbsolutePosition.stepType = 0;

      FinalLeftFootAbsolutePositions.push_back(LeftFootAbsolutePosition);
      FinalRightFootAbsolutePositions.push_back(RightFootAbsolutePosition);

      m_CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
    }

  ODEBUG5(" GetZMPDiscretization: Step 9 " << ZMPPositions.size(),
          "DebugData.txt");
  FilterOutValues(ZMPPositions,FinalZMPPositions,false);

}

void ZMPDiscretization::RegisterMethodsForScripting()
{
  std::string aMethodName[3] =
    {
     ":prevzmpinitprofil",
     ":zeroinitprofil",
     ":previewcontroltime"
    };

  for(int i=0; i<3; i++)
    {
      if (!RegisterMethod(aMethodName[i]))
        {
          std::cerr << "Unable to register " << aMethodName << std::endl;
        }
      else
        {
          ODEBUG("Succeed in registering " << aMethodName[i]);
        }

    }

}
void ZMPDiscretization::CallMethod(std::string &Method,
                                   std::istringstream &strm)
{
  if (Method==":prevzmpinitprofil")
    {
      m_InitializationProfile = PREV_ZMP_INIT_PROFIL;
    }
  else if (Method==":zeroinitprofil")
    {
      m_InitializationProfile = ZERO_INIT_PROFIL;
    }
  else if (Method==":previewcontroltime")
    {
      strm >> m_PreviewControlTime;
    }
  else if (Method==":samplingperiod")
    {
      strm >> m_SamplingPeriod;
      InitializeFilter();
    }

  ZMPRefTrajectoryGeneration::CallMethod(Method,strm);

}

/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 *
 * Andrei  Herdt
 * Fumio   Kanehiro
 * Florent Lamiraux
 * Alireza Nakhaei
 * Nicolas Perrin
 * Mathieu Poirier
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
/*! \file ZMPPreviewControlWithMultiBodyZMP.cpp
  \brief This object generate all the values for the foot trajectories,
  and the desired ZMP based on a sequence of steps.
*/

#include <iostream>
#include <sstream>
#include <fstream>

#include <Debug.hh>

#include <PreviewControl/ZMPPreviewControlWithMultiBodyZMP.hh>

using namespace PatternGeneratorJRL;


ZMPPreviewControlWithMultiBodyZMP::
ZMPPreviewControlWithMultiBodyZMP(SimplePluginManager *lSPM)
  : SimplePlugin(lSPM)
{

  m_ComAndFootRealization = 0;
  m_PinocchioRobot = 0;

  m_StageStrategy = ZMPCOM_TRAJECTORY_FULL;

  RESETDEBUG4("DebugData.txt");
  RESETDEBUG4("DebugDataqrql.txt");
  RESETDEBUG4("DebugDataDiffZMP.txt");
  RESETDEBUG4("DebugDataCOMPC1.txt");
  RESETDEBUG4("DebugDataWaistZMP.txt");
  RESETDEBUG4("DebugDataZMPMB1.txt");
  RESETDEBUG4("DebugDataDeltaCOM.txt");
  RESETDEBUG4("DebugDataStartingCOM.dat");
  RESETDEBUG4("DebugDataUB.txt");
  RESETDEBUG4("DebugDatadUB.txt");
  RESETDEBUG4("DebugDataIKL.dat");
  RESETDEBUG4("DebugDataIKR.dat");
  RESETDEBUG4("DebugDataWP.txt");
  RESETDEBUG4("Dump.dat");
  RESETDEBUG4("DebugConfSO.dat");
  RESETDEBUG4("DebugConfSV.dat");
  RESETDEBUG4("DebugConfSA.dat");
  RESETDEBUG4("DebugDataCheckZMP1.txt");
  RESETDEBUG4("2ndStage.dat");
  RESETDEBUG4("ZMPPCWMZOGSOC.dat");
  // Sampling period.DMB
  m_SamplingPeriod = -1;

  RegisterMethods();

  // Initialization of the first and second preview controls.
  m_PC1x.resize(3,1);
  m_PC1y.resize(3,1);
  m_Deltax.resize(3,1);
  m_Deltay.resize(3,1);

  m_PC = new PreviewControl
    (lSPM,
     OptimalControllerSolver::MODE_WITHOUT_INITIALPOS,
     true);
  m_StartingNewSequence = true;

  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      m_FinalDesiredCOMPose(i,j) =0.0;


  m_NumberOfIterations = 0;
}


ZMPPreviewControlWithMultiBodyZMP::
~ZMPPreviewControlWithMultiBodyZMP()
{
}

void ZMPPreviewControlWithMultiBodyZMP::
SetPreviewControl(PreviewControl *aPC)
{
  m_PC = aPC;
  m_SamplingPeriod = m_PC->SamplingPeriod();
  m_PreviewControlTime = m_PC->PreviewControlTime();
  m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);
}

void ZMPPreviewControlWithMultiBodyZMP::
CallToComAndFootRealization
(COMState &acomp,
 FootAbsolutePosition &aLeftFAP,
 FootAbsolutePosition &aRightFAP,
 Eigen::VectorXd &CurrentConfiguration,
 Eigen::VectorXd &CurrentVelocity,
 Eigen::VectorXd &CurrentAcceleration,
 unsigned long int IterationNumber,
 int StageOfTheAlgorithm)
{

  // New scheme for WPG v3.0
  // We call the object in charge of generating the whole body
  // motion  ( for a given CoM and Feet points)
  // before applying the second filter.

  Eigen::VectorXd aCOMState(6);
  Eigen::VectorXd aCOMSpeed(6);
  Eigen::VectorXd aCOMAcc(6);

  aCOMState(0) = acomp.x[0];
  aCOMState(1) = acomp.y[0];
  aCOMState(2) = acomp.z[0];
  aCOMState(3) = acomp.roll[0];
  aCOMState(4) = acomp.pitch[0];
  aCOMState(5) = acomp.yaw[0];

  aCOMSpeed(0) = acomp.x[1];
  aCOMSpeed(1) = acomp.y[1];
  aCOMSpeed(2) = acomp.z[1];
  aCOMSpeed(3) = acomp.roll[1];
  aCOMSpeed(4) = acomp.roll[1];
  aCOMSpeed(5) = acomp.roll[1];

  aCOMAcc(0) = acomp.x[2];
  aCOMAcc(1) = acomp.y[2];
  aCOMAcc(2) = acomp.z[2];
  aCOMAcc(3) = acomp.roll[2];
  aCOMAcc(4) = acomp.roll[2];
  aCOMAcc(5) = acomp.roll[2];

  Eigen::VectorXd aLeftFootPosition(5);
  Eigen::VectorXd aRightFootPosition(5);

  aLeftFootPosition(0) = aLeftFAP.x;
  aLeftFootPosition(1) = aLeftFAP.y;
  aLeftFootPosition(2) = aLeftFAP.z;
  aLeftFootPosition(3) = aLeftFAP.theta;
  aLeftFootPosition(4) = aLeftFAP.omega;

  aRightFootPosition(0) = aRightFAP.x;
  aRightFootPosition(1) = aRightFAP.y;
  aRightFootPosition(2) = aRightFAP.z;
  aRightFootPosition(3) = aRightFAP.theta;
  aRightFootPosition(4) = aRightFAP.omega;

  /* Get the current configuration vector */
  CurrentConfiguration = m_PinocchioRobot->currentRPYConfiguration();

  /* Get the current velocity vector */
  CurrentVelocity = m_PinocchioRobot->currentRPYVelocity();

  /* Get the current acceleration vector */
  CurrentAcceleration = m_PinocchioRobot->currentRPYAcceleration();

  m_ComAndFootRealization->
    ComputePostureForGivenCoMAndFeetPosture
    (aCOMState, aCOMSpeed, aCOMAcc,
     aLeftFootPosition,
     aRightFootPosition,
     CurrentConfiguration,
     CurrentVelocity,
     CurrentAcceleration,
     IterationNumber,
     StageOfTheAlgorithm);

  if (StageOfTheAlgorithm==0)
    {
      /* Update the current configuration vector */
      m_PinocchioRobot->currentRPYConfiguration(CurrentConfiguration);

      /* Update the current velocity vector */
      m_PinocchioRobot->currentRPYVelocity(CurrentVelocity);

      /* Update the current acceleration vector */
      m_PinocchioRobot->currentRPYAcceleration(CurrentAcceleration);
    }
}

/* Removed lqr and lql, now they should be set automatically by
   m_ComAndFootRealization */
int ZMPPreviewControlWithMultiBodyZMP::
OneGlobalStepOfControl
(FootAbsolutePosition &LeftFootPosition,
 FootAbsolutePosition &RightFootPosition,
 ZMPPosition &,
 COMState &refandfinalCOMState,
 Eigen::VectorXd & CurrentConfiguration,
 Eigen::VectorXd & CurrentVelocity,
 Eigen::VectorXd & CurrentAcceleration)
{
  FirstStageOfControl(LeftFootPosition,RightFootPosition,refandfinalCOMState);
  // This call is suppose to initialize
  // correctly the current configuration, speed and acceleration.
  COMState acompos = m_FIFOCOMStates[m_NL];
  FootAbsolutePosition aLeftFAP = m_FIFOLeftFootPosition[m_NL];
  FootAbsolutePosition aRightFAP = m_FIFORightFootPosition[m_NL];

  ODEBUG4SIMPLE(m_FIFOZMPRefPositions[0].px << " " <<
                m_FIFOZMPRefPositions[0].py << " " <<
                m_FIFOZMPRefPositions[0].pz << " " <<
                acompos.x[0] << " " <<
                acompos.y[0] << " " <<
                acompos.z[0] << " " <<
                aLeftFAP.x << " " <<
                aLeftFAP.y << " " <<
                aLeftFAP.z << " " <<
                aRightFAP.x << " " <<
                aRightFAP.y << " " <<
                aRightFAP.z,
                "1ststage.dat");

  CallToComAndFootRealization
    (acompos,aLeftFAP,aRightFAP,
     CurrentConfiguration,
     CurrentVelocity,
     CurrentAcceleration,
     m_NumberOfIterations,
     0);

  if (m_StageStrategy!=ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY)
    EvaluateMultiBodyZMP(-1);

  aLeftFAP = m_FIFOLeftFootPosition[0];
  aRightFAP = m_FIFORightFootPosition[0];

  SecondStageOfControl(refandfinalCOMState);

  ODEBUG4SIMPLE(refandfinalCOMState.x[0] <<
                " " << refandfinalCOMState.y[0] <<
                " " << refandfinalCOMState.z[0] <<
                " " << aLeftFAP.x <<
                " " << aLeftFAP.y <<
                " " << aLeftFAP.z <<
                " " << aLeftFAP.stepType <<
                " " << aRightFAP.x <<
                " " << aRightFAP.y <<
                " " << aRightFAP.z <<
                " " << aRightFAP.stepType
                , "2ndStage.dat");

  if (m_StageStrategy!=ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY)
    {
      CallToComAndFootRealization
        (refandfinalCOMState,aLeftFAP,aRightFAP,
         CurrentConfiguration,
         CurrentVelocity,
         CurrentAcceleration,
         m_NumberOfIterations - m_NL,
         1);
    }

  // Here it is assumed that the 4x4 CoM matrix
  // is the orientation of the free flyer and
  // its position.
  double c,co,s,so;
  c = cos(CurrentConfiguration(5));
  s = sin(CurrentConfiguration(5));

  co = cos(CurrentConfiguration(4));
  so = sin(CurrentConfiguration(4));

  m_FinalDesiredCOMPose(0,0) = c*co;
  m_FinalDesiredCOMPose(0,1) = -s;
  m_FinalDesiredCOMPose(0,2) = c*so;

  m_FinalDesiredCOMPose(1,0) = s*co;
  m_FinalDesiredCOMPose(1,1) =  c;
  m_FinalDesiredCOMPose(1,2) = s*so;

  m_FinalDesiredCOMPose(2,0) =  -so;
  m_FinalDesiredCOMPose(2,1)=  0;
  m_FinalDesiredCOMPose(2,2) = co;

  m_FinalDesiredCOMPose(0,3) = refandfinalCOMState.x[0];
  m_FinalDesiredCOMPose(1,3) = refandfinalCOMState.y[0];
  m_FinalDesiredCOMPose(2,3) = refandfinalCOMState.z[0];
  m_FinalDesiredCOMPose(3,3) = 1.0;

  ODEBUG4SIMPLE(CurrentConfiguration[6]<< " " <<
                CurrentConfiguration[7]<< " " <<
                CurrentConfiguration[8]<< " " <<
                CurrentConfiguration[9]<< " " <<
                CurrentConfiguration[10]<< " " <<
                CurrentConfiguration[11]<< " " <<
                CurrentConfiguration[12]<< " " <<
                CurrentConfiguration[13]<< " " <<
                CurrentConfiguration[14]<< " " <<
                CurrentConfiguration[15]<< " " <<
                CurrentConfiguration[16]<< " " <<
                CurrentConfiguration[17]<< " " <<
                CurrentConfiguration[18]<< " ",
                "DebugDataqrql.txt");
  m_NumberOfIterations++;

  return 1;
}


COMState ZMPPreviewControlWithMultiBodyZMP::
GetLastCOMFromFirstStage()
{
  COMState aCOM;
  aCOM = m_FIFOCOMStates.back();
  return aCOM;
}

int ZMPPreviewControlWithMultiBodyZMP::
SecondStageOfControl(COMState &finalCOMState)
{
  double Deltazmpx2,Deltazmpy2;
  // Inverse Kinematics variables.

  COMState aCOMState = m_FIFOCOMStates[0];
  FootAbsolutePosition LeftFootPosition, RightFootPosition;

  LeftFootPosition = m_FIFOLeftFootPosition[0];
  RightFootPosition = m_FIFORightFootPosition[0];

  // Preview control on delta ZMP.
  if ((m_StageStrategy==ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY)||
      (m_StageStrategy==ZMPCOM_TRAJECTORY_FULL))
    {
      ODEBUG2(m_FIFODeltaZMPPositions[0].px << " " <<
              m_FIFODeltaZMPPositions[0].py);

      ODEBUG2("Second Stage Size of FIFODeltaZMPPositions: "
              << m_FIFODeltaZMPPositions.size()
              << " " << m_Deltax
              << " " << m_Deltay
              << " " << m_sxDeltazmp
              << " " << m_syDeltazmp
              << " " << Deltazmpx2
              << " " << Deltazmpy2);

      m_PC->OneIterationOfPreview
        (m_Deltax,m_Deltay,
         m_sxDeltazmp, m_syDeltazmp,
         m_FIFODeltaZMPPositions,0,
         Deltazmpx2,Deltazmpy2,
         true);


      // Correct COM position
      // but be carefull this is the COM for NL steps behind.
      for(int i=0; i<3; i++)
        {
          aCOMState.x[i] += m_Deltax(i,0);
          aCOMState.y[i] += m_Deltay(i,0);
        }
    }

  ODEBUG2("Delta :"
          << m_Deltax(0,0) << " " << m_Deltay(0,0) << " "
          << aCOMState.x[0] << " " << aCOMState.y[0]);
  // Update finalCOMState
  finalCOMState = aCOMState;

  if ((m_StageStrategy==ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY)||
      (m_StageStrategy==ZMPCOM_TRAJECTORY_FULL))
    {

      m_FIFODeltaZMPPositions.pop_front();
    }
  m_FIFOCOMStates.pop_front();
  m_FIFOLeftFootPosition.pop_front();
  m_FIFORightFootPosition.pop_front();

  ODEBUG2("End");
  return 1;
}

int ZMPPreviewControlWithMultiBodyZMP::
FirstStageOfControl
( FootAbsolutePosition &LeftFootPosition,
  FootAbsolutePosition &RightFootPosition,
  COMState& afCOMState )

{


  double zmpx2, zmpy2;
  COMState acomp;
  acomp.yaw[0] = 0.0;
  acomp.pitch[0] = 0.0;
  if ((m_StageStrategy==ZMPCOM_TRAJECTORY_FULL)
      || (m_StageStrategy==ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY))
    {

      m_PC->OneIterationOfPreview(m_PC1x,m_PC1y,
                                  m_sxzmp,m_syzmp,
                                  m_FIFOZMPRefPositions,0,
                                  zmpx2, zmpy2, true);
      for(unsigned j=0; j<3; j++)
        acomp.x[j] = m_PC1x(j,0);

      for(unsigned j=0; j<3; j++)
        acomp.y[j] = m_PC1y(j,0);

      for(unsigned j=0; j<3; j++)
        acomp.z[j] = afCOMState.z[j];

      for(unsigned j=0; j<3; j++)
        acomp.yaw[j] = afCOMState.yaw[j];

      for(unsigned j=0; j<3; j++)
        acomp.pitch[j] = afCOMState.pitch[j];

      for(unsigned j=0; j<3; j++)
        acomp.roll[j] = afCOMState.roll[j];

    }
  else if (m_StageStrategy==ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY)
    {
      for(unsigned j=0; j<3; j++)
        acomp.x[j] = m_PC1x(j,0)= afCOMState.x[j];

      for(unsigned j=0; j<3; j++)
        acomp.y[j] = m_PC1y(j,0)= afCOMState.y[j];

      for(unsigned j=0; j<3; j++)
        acomp.z[j] = afCOMState.z[j];

      for(unsigned j=0; j<3; j++)
        acomp.yaw[j] = afCOMState.yaw[j];

      for(unsigned j=0; j<3; j++)
        acomp.pitch[j] = afCOMState.pitch[j];
    }


  // Update of the FIFOs
  m_FIFOCOMStates.push_back(acomp);
  m_FIFORightFootPosition.push_back(RightFootPosition);
  m_FIFOLeftFootPosition.push_back(LeftFootPosition);

  ODEBUG("FIFOs COM:"<< m_FIFOCOMStates.size() <<
         " RF: "<< m_FIFORightFootPosition.size() <<
         " LF: "<< m_FIFOLeftFootPosition.size());
  m_FIFOZMPRefPositions.pop_front();
  return 1;
}

int ZMPPreviewControlWithMultiBodyZMP::
EvaluateMultiBodyZMP(int /* StartingIteration */)
{
  ODEBUG("Start EvaluateMultiBodyZMP");
  m_PinocchioRobot->computeInverseDynamics();

  // Call the Humanoid Dynamic Multi Body robot model to
  // compute the ZMP related to the motion found by CoMAndZMPRealization.
  Eigen::Vector3d ZMPmultibody;
  m_PinocchioRobot->zeroMomentumPoint(ZMPmultibody);
  ODEBUG4(ZMPmultibody[0] << " " << ZMPmultibody[1]
          << " " << m_FIFOZMPRefPositions[0].px
          << " " << m_FIFOZMPRefPositions[0].py,
          "DebugDataCheckZMP1.txt");

  Eigen::Vector3d CoMmultibody;
  m_PinocchioRobot->positionCenterOfMass(CoMmultibody);
  ODEBUG("Stage 2");
  // Fill the delta ZMP FIFO for the second stage of the control.
  ZMPPosition aZMPpos;
  aZMPpos.px = m_FIFOZMPRefPositions[0].px - ZMPmultibody[0];
  aZMPpos.py = m_FIFOZMPRefPositions[0].py - ZMPmultibody[1];
  aZMPpos.pz = 0.0;
  aZMPpos.theta = 0.0;
  aZMPpos.stepType = 1;
  aZMPpos.time = m_FIFOZMPRefPositions[0].time;
  ODEBUG("Stage 3");
  Eigen::VectorXd CurrentConfiguration;
  /* Get the current configuration vector */
  CurrentConfiguration = m_PinocchioRobot->currentRPYConfiguration();

  ODEBUG("Stage 4");
  m_FIFODeltaZMPPositions.push_back(aZMPpos);
  m_StartingNewSequence = false;
  ODEBUG("Final");
  return 1;
}



int ZMPPreviewControlWithMultiBodyZMP::
Setup
(deque<ZMPPosition> &ZMPRefPositions,
 deque<COMState> &COMStates,
 deque<FootAbsolutePosition> &LeftFootPositions,
 deque<FootAbsolutePosition> &RightFootPositions)
{
  m_NumberOfIterations = 0;
  Eigen::VectorXd CurrentConfiguration =
    m_PinocchioRobot->currentRPYConfiguration();
  Eigen::VectorXd CurrentVelocity =
    m_PinocchioRobot->currentRPYVelocity();
  Eigen::VectorXd CurrentAcceleration =
    m_PinocchioRobot->currentRPYAcceleration();

  m_PC->ComputeOptimalWeights(OptimalControllerSolver::MODE_WITHOUT_INITIALPOS);

  SetupFirstPhase(ZMPRefPositions,
                  COMStates,
                  LeftFootPositions,
                  RightFootPositions);
  for(unsigned int i=0; i<m_NL; i++)
    SetupIterativePhase(ZMPRefPositions,
                        COMStates,
                        LeftFootPositions,
                        RightFootPositions,
                        CurrentConfiguration,
                        CurrentVelocity,
                        CurrentAcceleration,
                        i);
  ODEBUG4("<========================================>","ZMPPCWMZOGSOC.dat");
  return 0;
}

int ZMPPreviewControlWithMultiBodyZMP::
SetupFirstPhase
(deque<ZMPPosition> &ZMPRefPositions,
 deque<COMState> &, //COMStates,
 deque<FootAbsolutePosition> &LeftFootPositions,
 deque<FootAbsolutePosition> &RightFootPositions)
{
  ODEBUG6("Beginning of Setup 0 ","DebugData.txt");
  ODEBUG("Setup");
  //double zmpx2, zmpy2;

  m_sxzmp =0.0;
  m_syzmp =0.0;
  m_sxDeltazmp = 0.0;
  m_syDeltazmp = 0.0;

  m_StartingNewSequence = true;

  // Fill the Fifo
  m_FIFOZMPRefPositions.resize(m_NL);
  m_FIFOLeftFootPosition.resize(m_NL);
  m_FIFORightFootPosition.resize(m_NL);
  for(unsigned int i=0; i<m_NL; i++)
    {
      m_FIFOZMPRefPositions[i] = ZMPRefPositions[i];
      m_FIFOLeftFootPosition[i] = LeftFootPositions[i];
      m_FIFORightFootPosition[i] = RightFootPositions[i];
    }
  ODEBUG6("After EvaluateCOM","DebugData.txt");

  ODEBUG6("Beginning of Setup 1 ","DebugData.txt");
  m_PC1x(0,0)= m_StartingCOMState[0];
  ODEBUG("COMPC1 init X: " <<  m_PC1x(0,0));
  //m_PC1x(0,0) = 0.0;
  m_PC1x(1,0)= 0.0;
  m_PC1x(2,0)= 0.0;

  m_PC1y(0,0)= m_StartingCOMState[1];
  ODEBUG("COMPC1 init Y: " <<  m_PC1y(0,0));
  //m_PC1y(0,0) = 0.0;
  m_PC1y(1,0)= 0;
  m_PC1y(2,0)= 0;

  m_Deltax(0,0)= 0.0; //-StartingCOMState[0];
  m_Deltax(1,0)= 0;
  m_Deltax(2,0)= 0;
  m_Deltay(0,0)= 0.0; //-StartingCOMState[1];
  m_Deltay(1,0)= 0;
  m_Deltay(2,0)= 0;

  //  m_sxzmp=-StartingCOMState[0];m_syzmp=-StartingCOMState[1];
  //  zmpx2=StartingCOMState[0];zmpy2=StartingCOMState[1];
  m_sxzmp = 0.0;
  m_syzmp =0.0;
  //zmpx2 = 0.0; zmpy2 = 0.0;

  m_FIFODeltaZMPPositions.clear();
  m_FIFOCOMStates.clear();


  Eigen::VectorXd CurrentConfiguration;
  Eigen::VectorXd CurrentVelocity;
  Eigen::VectorXd CurrentAcceleration;
  /* Get the current configuration vector */
  CurrentConfiguration = m_PinocchioRobot->currentRPYConfiguration();
  CurrentVelocity = m_PinocchioRobot->currentRPYVelocity();
  CurrentAcceleration = m_PinocchioRobot->currentRPYAcceleration();
  CurrentVelocity.setZero();
  CurrentAcceleration.setZero();

  m_PinocchioRobot->currentRPYVelocity(CurrentVelocity);
  m_PinocchioRobot->currentRPYAcceleration(CurrentAcceleration);

#ifdef _DEBUG_MODE_ON_
  m_FIFOTmpZMPPosition.clear();
#endif

  return 0;
}


int ZMPPreviewControlWithMultiBodyZMP::
SetupIterativePhase
(deque<ZMPPosition> &ZMPRefPositions,
 deque<COMState> &COMStates,
 deque<FootAbsolutePosition> &LeftFootPositions,
 deque<FootAbsolutePosition> &RightFootPositions,
 Eigen::VectorXd & CurrentConfiguration,
 Eigen::VectorXd & CurrentVelocity,
 Eigen::VectorXd & CurrentAcceleration,
 int localindex)
{

  ODEBUG("SetupIterativePhase " << localindex << " " << CurrentConfiguration );
  ODEBUG("m_FIFOZMPRefPositions.size():" << m_FIFOZMPRefPositions.size());
  ODEBUG("COMState["<<localindex<<"]=" << COMStates[localindex].x[0] << " " <<
         COMStates[localindex].y[0] << " " << COMStates[localindex].z[0] <<
         " COMStates.size()=" <<COMStates.size());
  FirstStageOfControl(LeftFootPositions[localindex],
                      RightFootPositions[localindex],COMStates[localindex]);
  ODEBUG("m_FIFOCOMStates["<<localindex<<"]="
         << m_FIFOCOMStates[localindex].x[0]
         << " " <<
         m_FIFOCOMStates[localindex].y[0] << " "
         << m_FIFOCOMStates[localindex].z[0] <<
         " m_FIFOCOMStates.size()=" <<m_FIFOCOMStates.size());
  //COMState acompos = m_FIFOCOMStates[localindex];
  //FootAbsolutePosition aLeftFAP = m_FIFOLeftFootPosition[localindex];
  //FootAbsolutePosition aRightFAP = m_FIFORightFootPosition[localindex];

  ODEBUG4SIMPLE(m_FIFOZMPRefPositions[0].px << " " <<
                m_FIFOZMPRefPositions[0].py << " " <<
                m_FIFOZMPRefPositions[0].pz << " " <<
                acompos.x[0] << " " <<
                acompos.y[0] << " " <<
                acompos.z[0] << " " <<
                aLeftFAP.x << " " <<
                aLeftFAP.y << " " <<
                aLeftFAP.z << " " <<
                aRightFAP.x << " " <<
                aRightFAP.y << " " <<
                aRightFAP.z,
                "ZMPPCWMZOGSOC.dat");

  CallToComAndFootRealization
    (m_FIFOCOMStates[localindex],
     m_FIFORightFootPosition[localindex],
     m_FIFOLeftFootPosition[localindex],
     CurrentConfiguration,
     CurrentVelocity,
     CurrentAcceleration,
     m_NumberOfIterations,
     0);


  EvaluateMultiBodyZMP(localindex);

  m_FIFOZMPRefPositions.push_back(ZMPRefPositions[localindex+1+m_NL]);

  m_NumberOfIterations++;
  return 0;
}
void ZMPPreviewControlWithMultiBodyZMP::
CreateExtraCOMBuffer
(deque<COMState> &m_ExtraCOMBuffer,
 deque<ZMPPosition> &m_ExtraZMPBuffer,
 deque<ZMPPosition> &m_ExtraZMPRefBuffer)

{
  deque<ZMPPosition> aFIFOZMPRefPositions;
  Eigen::MatrixXd aPC1x;
  Eigen::MatrixXd aPC1y;
  double aSxzmp, aSyzmp;
  double aZmpx2, aZmpy2;

  //initialize ZMP FIFO
  for (unsigned int i=0; i<m_NL; i++)
    aFIFOZMPRefPositions.push_back(m_ExtraZMPRefBuffer[i]);

  //use accumulated zmp error  of preview control so far
  aSxzmp = m_sxzmp;
  aSyzmp = m_syzmp;

  aPC1x = m_PC1x;
  aPC1y = m_PC1y;

  //create the extra COMbuffer

#ifdef _DEBUG_MODE_ON_
  ofstream aof_ExtraCOM;
  static unsigned char FirstCall=1;
  if (FirstCall)
    {
      aof_ExtraCOM.open("CartExtraCOM_1.dat",ofstream::out);
    }
  else
    {
      aof_ExtraCOM.open("CartExtraCOM_1.dat",ofstream::app);
    }

  if (FirstCall)
    FirstCall = 0;
#endif

  for (unsigned int i=0; i<m_ExtraCOMBuffer.size(); i++)
    {
      aFIFOZMPRefPositions.push_back(m_ExtraZMPRefBuffer[i]);
      m_PC->OneIterationOfPreview(aPC1x,aPC1y,
                                  aSxzmp,aSyzmp,
                                  aFIFOZMPRefPositions,0,
                                  aZmpx2, aZmpy2, true);

      for(unsigned j=0; j<3; j++)
        {
          m_ExtraCOMBuffer[i].x[j] = aPC1x(j,0);
          m_ExtraCOMBuffer[i].y[j] = aPC1y(j,0);
        }

      m_ExtraZMPBuffer[i].px=aZmpx2;
      m_ExtraZMPBuffer[i].py=aZmpy2;

      m_ExtraCOMBuffer[i].yaw[0] = m_ExtraZMPRefBuffer[i].theta;

      aFIFOZMPRefPositions.pop_front();


#ifdef _DEBUG_MODE_ON_
      if (aof_ExtraCOM.is_open())
        {
          aof_ExtraCOM << m_ExtraZMPRefBuffer[i].time << " "
                       << m_ExtraZMPRefBuffer[i].px << " "
                       << aPC1x(0,0)<< " "
                       << aPC1y(0,0) << endl;
        }
#endif

    }
  ODEBUG("ik ben hier b" << aFIFOZMPRefPositions.size() );
  ODEBUG("ik ben hier c" << m_ExtraZMPRefBuffer.size());

#ifdef _DEBUG_MODE_ON_
  if (aof_ExtraCOM.is_open())
    {
      aof_ExtraCOM.close();
    }
#endif

}

void ZMPPreviewControlWithMultiBodyZMP::
UpdateTheZMPRefQueue(ZMPPosition NewZMPRefPos)
{
  m_FIFOZMPRefPositions.push_back(NewZMPRefPos);
}

void ZMPPreviewControlWithMultiBodyZMP::
SetStrategyForStageActivation(int aZMPComTraj)
{
  switch(aZMPComTraj)
    {
    case ZMPCOM_TRAJECTORY_FULL:
      m_StageStrategy = ZMPCOM_TRAJECTORY_FULL;
      break;
    case ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY:
      m_StageStrategy = ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY;
      break;
    case ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY:
      m_StageStrategy = ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY;
      break;

    default:
      break;
    }
}

int ZMPPreviewControlWithMultiBodyZMP::
GetStrategyForStageActivation()
{
  return m_StageStrategy;
}

// TODO : Compute the position of the waist inside the COM Frame.
Eigen::Matrix4d ZMPPreviewControlWithMultiBodyZMP::
GetCurrentPositionofWaistInCOMFrame()
{
  Eigen::Matrix4d PosOfWaistInCoMFrame;
  PosOfWaistInCoMFrame =
    m_ComAndFootRealization->
    GetCurrentPositionofWaistInCOMFrame();

  //  cerr << " Should implement:
  // ZMPPreviewControlWithMultiBodyZMP::
  // GetCurrentPositionOfWaistInCOMFrame()" << endl;
  return PosOfWaistInCoMFrame;
}

Eigen::Matrix4d  ZMPPreviewControlWithMultiBodyZMP::
GetFinalDesiredCOMPose()
{
  return m_FinalDesiredCOMPose;
}


int ZMPPreviewControlWithMultiBodyZMP::
EvaluateStartingState
(Eigen::VectorXd & BodyAnglesInit,
 Eigen::Vector3d & aStartingCOMState,
 Eigen::Vector3d & aStartingZMPPosition,
 Eigen::Matrix<double, 6, 1> & aStartingWaistPosition,
 FootAbsolutePosition & InitLeftFootPosition,
 FootAbsolutePosition & InitRightFootPosition)
{
  int r = EvaluateStartingCoM(BodyAnglesInit,aStartingCOMState,
                              aStartingWaistPosition,
                              InitLeftFootPosition, InitRightFootPosition);
  aStartingZMPPosition= m_ComAndFootRealization->GetCOGInitialAnkles();
  return r;
}
int ZMPPreviewControlWithMultiBodyZMP::
EvaluateStartingCoM
(Eigen::VectorXd &BodyAnglesInit,
 Eigen::Vector3d &aStartingCOMState,
 Eigen::Matrix<double,6,1> & aStartingWaistPosition,
 FootAbsolutePosition & InitLeftFootPosition,
 FootAbsolutePosition & InitRightFootPosition)
{
  ODEBUG("EvaluateStartingCOM: BodyAnglesInit :" << BodyAnglesInit);

  m_ComAndFootRealization->
    InitializationCoM(BodyAnglesInit,m_StartingCOMState,
                      aStartingWaistPosition,
                      InitLeftFootPosition, InitRightFootPosition);

  ODEBUG("EvaluateStartingCOM: m_StartingCOMState: " << m_StartingCOMState);
  aStartingCOMState[0] = m_StartingCOMState[0];
  aStartingCOMState[1] = m_StartingCOMState[1];
  aStartingCOMState[2] = m_StartingCOMState[2];

  return 0;
}

void ZMPPreviewControlWithMultiBodyZMP::
SetStrategyForPCStages(int Strategy)
{
  m_StageStrategy = Strategy;
}

int ZMPPreviewControlWithMultiBodyZMP::
GetStrategyForPCStages()
{
  return m_StageStrategy;
}


void ZMPPreviewControlWithMultiBodyZMP::
RegisterMethods()
{
  std::string aMethodName[3] =
    {
     ":samplingperiod",
     ":previewcontroltime",
     ":comheight"
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

void ZMPPreviewControlWithMultiBodyZMP::
SetSamplingPeriod(double lSamplingPeriod)
{
  m_SamplingPeriod = lSamplingPeriod;

  m_NL=0;
  if (m_SamplingPeriod!=0.0)
    m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);

}

void ZMPPreviewControlWithMultiBodyZMP::
SetPreviewControlTime(double lPreviewControlTime)
{
  m_PreviewControlTime = lPreviewControlTime;

  m_NL=0;
  if (m_SamplingPeriod!=0.0)
    m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);

}

void ZMPPreviewControlWithMultiBodyZMP::
CallMethod(std::string &Method,
           std::istringstream &strm)
{
  if (Method==":samplingperiod")
    {
      std::string aws;
      if (strm.good())
        {
          double lSamplingPeriod;
          strm >> lSamplingPeriod;
          SetSamplingPeriod(lSamplingPeriod);
        }
    }
  else if (Method==":previewcontroltime")
    {
      std::string aws;
      if (strm.good())
        {
          double lpreviewcontroltime;
          strm >> lpreviewcontroltime;
          SetPreviewControlTime(lpreviewcontroltime);
        }
    }

}

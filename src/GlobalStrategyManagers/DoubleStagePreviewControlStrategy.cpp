/*
 * Copyright 2007, 2008, 2009, 2010,
 *
 * Francois Keith
 * Florent Lamiraux
 * Olivier  Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of jrl-walkgen.
 * jrl-walkgen is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * jrl-walkgen is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with jrl-walkgen.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/*! \file DoubleStagePreviewControlSrategy.h
  \brief This object defines a global strategy object to generate
  full body position every 5 ms over a preview window.
  It implements Kajita's algorithm presented in \ref Kajita2003 */
#include <iostream>
#include <fstream>
using namespace std;

#include <Debug.hh>
#include <GlobalStrategyManagers/DoubleStagePreviewControlStrategy.hh>
using namespace PatternGeneratorJRL;

DoubleStagePreviewControlStrategy::
DoubleStagePreviewControlStrategy
(SimplePluginManager * aSPM)
  :GlobalStrategyManager(aSPM)
{
  m_ZMPFrame = ZMPFRAME_WAIST;

  // The object to realize the global stage of preview control.
  m_ZMPpcwmbz = new ZMPPreviewControlWithMultiBodyZMP(aSPM);

  unsigned int NbOfMethods=4;
  std::string aMethodName[4] =
    {
     ":SetAlgoForZmpTrajectory",
     ":SetZMPFrame",
     ":samplingperiod",
     ":previewcontroltime"
    };

  for(unsigned int i=0; i<NbOfMethods; i++)
    {
      if (!RegisterMethod(aMethodName[i]))
        {
          std::cerr<< "Unable to register " << aMethodName[i]  << endl;
        }
    }

  RESETDEBUG4("ZMPRefAndWaist.dat");
  RESETDEBUG4("ZMPRef.dat");
}

DoubleStagePreviewControlStrategy::
~DoubleStagePreviewControlStrategy()
{
  if (m_ZMPpcwmbz!=0)
    delete m_ZMPpcwmbz;
}


int DoubleStagePreviewControlStrategy::
InitInterObjects
(PinocchioRobot *aPR,
 ComAndFootRealization * aCFR,
 StepStackHandler * aSSH)
{
  setHumanoidDynamicRobot(aPR);
  m_ZMPpcwmbz->setPinocchioRobot(m_PinocchioRobot);
  m_ZMPpcwmbz->setComAndFootRealization(aCFR);
  m_StepStackHandler = aSSH;
  return 1;
}

int DoubleStagePreviewControlStrategy::
OneGlobalStepOfControl
(FootAbsolutePosition &LeftFootPosition,
 FootAbsolutePosition &RightFootPosition,
 Eigen::VectorXd & ZMPRefPos,
 COMState & finalCOMState,
 Eigen::VectorXd & CurrentConfiguration,
 Eigen::VectorXd & CurrentVelocity,
 Eigen::VectorXd & CurrentAcceleration)
{
  // New scheme:
  // Update the queue of ZMP ref
  m_ZMPpcwmbz->UpdateTheZMPRefQueue((*m_ZMPPositions)[2*m_NL]);
  if ((m_StepStackHandler->GetWalkMode()==0) ||
      (m_StepStackHandler->GetWalkMode()==4))
    {
      (*m_COMBuffer)[m_NL].yaw[0] = (*m_ZMPPositions)[m_NL].theta;
    }

  //    COMStateFromPC1 = m_COMBuffer[m_NL];
  finalCOMState =  (*m_COMBuffer)[m_NL];

  ODEBUG("ZMP : " << (*m_ZMPPositions)[0].px
         << " " << (*m_ZMPPositions)[0].py
         << " " << (*m_ZMPPositions)[2*m_NL].px
         << " " << (*m_ZMPPositions)[2*m_NL].py );
  ODEBUG4( (*m_ZMPPositions)[0].px << " " <<
           (*m_ZMPPositions)[0].py << " " <<
           (*m_ZMPPositions)[0].pz,"ZMPRef.dat");

  ODEBUG("m_LeftFootPositions: "<<m_LeftFootPositions->size());
  ODEBUG("m_RightFootPositions: "<<m_RightFootPositions->size());
  ODEBUG("m_ZMPPositions: "<<m_ZMPPositions->size());

  m_ZMPpcwmbz->OneGlobalStepOfControl
    ((*m_LeftFootPositions)[2*m_NL],
     (*m_RightFootPositions)[2*m_NL],
     (*m_ZMPPositions)[2*m_NL],
     finalCOMState,
     CurrentConfiguration,
     CurrentVelocity,
     CurrentAcceleration);
  ODEBUG4("finalCOMState:" <<finalCOMState.x[0] << " "
          << finalCOMState.y[0],"DebugData.txt");

  (*m_COMBuffer)[0] = finalCOMState;

  LeftFootPosition = (*m_LeftFootPositions)[0];
  RightFootPosition = (*m_RightFootPositions)[0];

  // Compute the waist position in the current motion global reference frame.
  Eigen::Matrix4d PosOfWaistInCOMF;
  PosOfWaistInCOMF = m_ZMPpcwmbz->GetCurrentPositionofWaistInCOMFrame();

  COMState outWaistPosition;
  outWaistPosition = finalCOMState;
  outWaistPosition.x[0] =  CurrentConfiguration(0);
  outWaistPosition.y[0] =  CurrentConfiguration(1);
  outWaistPosition.z[0] =  CurrentConfiguration(2);

  // In case we are at the end of the motion
  //double CurrentZMPNeutralPosition[2];
  //CurrentZMPNeutralPosition[0] = (*m_ZMPPositions)[0].px;
  //CurrentZMPNeutralPosition[1] = (*m_ZMPPositions)[0].py;

  double temp1;
  double temp2;
  double temp3;

  if (m_ZMPFrame==ZMPFRAME_WAIST)
    {
      temp1 = (*m_ZMPPositions)[0].px - outWaistPosition.x[0];
      temp2 = (*m_ZMPPositions)[0].py - outWaistPosition.y[0];
      temp3 = finalCOMState.yaw[0]*M_PI/180.0;

      ZMPRefPos(0) = cos(temp3)*temp1+sin(temp3)*temp2 ;
      ZMPRefPos(1) = -sin(temp3)*temp1+cos(temp3)*temp2;
      ZMPRefPos(2) = -finalCOMState.z[0] - PosOfWaistInCOMF(2,3) -
        (*m_ZMPPositions)[0].pz;
    }
  else if (m_ZMPFrame==ZMPFRAME_WORLD)
    {
      temp1 = (*m_ZMPPositions)[0].px ;
      temp2 = (*m_ZMPPositions)[0].py ;
      temp3 = finalCOMState.yaw[0]*M_PI/180.0;

      ZMPRefPos(0) = cos(temp3)*temp1+sin(temp3)*temp2 ;
      ZMPRefPos(1) = -sin(temp3)*temp1+cos(temp3)*temp2;
      ZMPRefPos(2) = 0.0;

    }
  else
    {
      std::cerr << "Problem with the ZMP reference frame set to 0."
                << std::endl;
    }

  ODEBUG4SIMPLE((*m_ZMPPositions)[0].px <<  " " <<
                (*m_ZMPPositions)[0].py << " " <<
                outWaistPosition.x[0] << " " <<
                outWaistPosition.y[0] << " " <<
                ZMPRefPos(0) << " " <<
                ZMPRefPos(1) << " " <<
                ZMPRefPos(2) <<
                LeftFootPosition.stepType << " " <<
                RightFootPosition.stepType,
                "ZMPRefAndWaist.dat");

  m_ZMPPositions->pop_front();
  m_COMBuffer->pop_front();
  m_LeftFootPositions->pop_front();
  m_RightFootPositions->pop_front();

  m_CurrentWaistState = outWaistPosition;

  return 0;
}

int DoubleStagePreviewControlStrategy::
EvaluateStartingState(Eigen::VectorXd & BodyAngles,
                      COMState & aStartingCOMState,
                      Eigen::Vector3d & aStartingZMPPosition,
                      Eigen::Matrix<double, 6, 1> & aStartingWaistPose,
                      FootAbsolutePosition & InitLeftFootPosition,
                      FootAbsolutePosition & InitRightFootPosition)
{
  Eigen::Vector3d lStartingCOMState;
  lStartingCOMState(0) = aStartingCOMState.x[0];
  lStartingCOMState(1) = aStartingCOMState.y[0];
  lStartingCOMState(2) = aStartingCOMState.z[0];

  m_ZMPpcwmbz->EvaluateStartingState(BodyAngles,
                                     lStartingCOMState,
                                     aStartingZMPPosition,
                                     aStartingWaistPose,
                                     InitLeftFootPosition,
                                     InitRightFootPosition);

  aStartingCOMState.x[0] = lStartingCOMState(0);
  aStartingCOMState.y[0] = lStartingCOMState(1);
  aStartingCOMState.z[0] = lStartingCOMState(2);
  return 0;
}

int DoubleStagePreviewControlStrategy::EndOfMotion()
{
  ODEBUG("m_ZMPPositions->size()  2*m_NL+1 2*m_NL "
         << m_ZMPPositions->size() << " "
         << 2*m_NL+1 << " "
         << 2*m_NL << " " );
  if (m_ZMPPositions->size()== 2*m_NL)
    return 0;
  else if (m_ZMPPositions->size()< 2*m_NL)
    return -1;

  return 1;

}

void DoubleStagePreviewControlStrategy::
SetAlgoForZMPTraj(istringstream &strm)
{
  string ZMPTrajAlgo;
  strm >> ZMPTrajAlgo;
  if (ZMPTrajAlgo=="PBW")
    {
      m_ZMPpcwmbz->SetStrategyForStageActivation
        (ZMPPreviewControlWithMultiBodyZMP::
         ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY);
    }
  else if (ZMPTrajAlgo=="Kajita")
    {
      m_ZMPpcwmbz->SetStrategyForStageActivation
        (ZMPPreviewControlWithMultiBodyZMP::
         ZMPCOM_TRAJECTORY_FULL);
    }
  else if (ZMPTrajAlgo=="KajitaOneStage")
    {
      m_ZMPpcwmbz->SetStrategyForStageActivation
        (ZMPPreviewControlWithMultiBodyZMP::
         ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY);
    }
  else if (ZMPTrajAlgo=="Morisawa")
    {
      ODEBUG("Wrong Global Strategy");
    }

}

void DoubleStagePreviewControlStrategy::
SetZMPFrame
(std::istringstream &astrm)
{
  string aZMPFrame;
  astrm >> aZMPFrame;

  if (aZMPFrame=="waist")
    {
      m_ZMPFrame = ZMPFRAME_WAIST;
    }
  else if (aZMPFrame=="world")
    {
      m_ZMPFrame = ZMPFRAME_WORLD;
    }
  else
    {
      std::cerr << "Mistake wrong keyword" << aZMPFrame
                << std::endl;
    }
}

void DoubleStagePreviewControlStrategy::
SetSamplingPeriod
(double lSamplingPeriod)
{
  m_SamplingPeriod = lSamplingPeriod;

  m_NL=0;
  if (m_SamplingPeriod!=0.0)
    m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);

}

void DoubleStagePreviewControlStrategy::
SetPreviewControlTime
(double lPreviewControlTime)
{
  m_PreviewControlTime = lPreviewControlTime;

  m_NL=0;
  if (m_SamplingPeriod!=0.0)
    m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);

}

void DoubleStagePreviewControlStrategy::
CallMethod
(std::string &Method, std::istringstream &astrm)
{
  ODEBUG("Method: " << Method);

  if (Method==":SetAlgoForZmpTrajectory")
    {
      SetAlgoForZMPTraj(astrm);
    }
  else if (Method==":SetZMPFrame")
    {
      SetZMPFrame(astrm);
    }
  if (Method==":samplingperiod")
    {
      std::string aws;
      if (astrm.good())
        {
          double lSamplingPeriod;
          astrm >> lSamplingPeriod;
          SetSamplingPeriod(lSamplingPeriod);
        }
    }
  else if (Method==":previewcontroltime")
    {
      std::string aws;
      if (astrm.good())
        {
          double lpreviewcontroltime;
          astrm >> lpreviewcontroltime;
          SetPreviewControlTime(lpreviewcontroltime);
        }
    }
}

void DoubleStagePreviewControlStrategy::
Setup
(deque<ZMPPosition> & aZMPPositions,
 deque<COMState> & aCOMBuffer,
 deque<FootAbsolutePosition> & aLeftFootAbsolutePositions,
 deque<FootAbsolutePosition> & aRightFootAbsolutePositions)
{
  m_ZMPpcwmbz->Setup(aZMPPositions,
                     aCOMBuffer,
                     aLeftFootAbsolutePositions,
                     aRightFootAbsolutePositions);
}

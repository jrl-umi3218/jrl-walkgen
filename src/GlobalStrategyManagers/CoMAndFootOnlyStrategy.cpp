/*
 * Copyright 2007, 2008, 2009, 2010,
 *
 * Fumio    Kanehiro
 * Francois Keith
 * Florent  Lamiraux
 * Anthony  Mallet
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

/*! \file CoMAndFootOnlyStrategy.h
  \brief This object defines a global strategy object to generate
  only foot, ZMP reference and CoM trajectories position every 5 ms. */

#include <Debug.hh>
#include <GlobalStrategyManagers/CoMAndFootOnlyStrategy.hh>

using namespace PatternGeneratorJRL;

CoMAndFootOnlyStrategy::
CoMAndFootOnlyStrategy
(SimplePluginManager * aSimplePluginManager)
  : GlobalStrategyManager(aSimplePluginManager)
{
  m_BufferSizeLimit = 0;
}

CoMAndFootOnlyStrategy::~CoMAndFootOnlyStrategy()
{
}

int
CoMAndFootOnlyStrategy::
InitInterObjects
(PinocchioRobot * /* aPR */,
 std::vector<ComAndFootRealization *> aCFR,
 StepStackHandler * /* aSSH */)
{
  m_ComAndFootRealization = aCFR;
  return 0;
}

int
CoMAndFootOnlyStrategy::
OneGlobalStepOfControl
(FootAbsolutePosition &LeftFootPosition,
 FootAbsolutePosition &RightFootPosition,
 Eigen::VectorXd & ZMPRefPos,
 COMState & finalCOMPosition,
 Eigen::VectorXd &, //CurrentConfiguration,
 Eigen::VectorXd &, //CurrentVelocity,
 Eigen::VectorXd & )//CurrentAcceleration)
{
  ODEBUG("Begin OneGlobalStepOfControl "
         << m_LeftFootPositions->size() << " "
         << m_RightFootPositions->size() << " "
         << m_COMBuffer->size() << " "
         << m_ZMPPositions->size());

  /* The strategy of this class is simply to pull 
     off values from the buffers. */
  if (m_LeftFootPositions->size()>0)
    {
      LeftFootPosition = (*m_LeftFootPositions)[0];
      m_LeftFootPositions->pop_front();
    }
  else
    {
      std::cerr << "Problem on the left foot position queue: empty"
                << std::endl;
      return -2;
    }

  if (m_RightFootPositions->size()>0)
    {
      RightFootPosition = (*m_RightFootPositions)[0];
      m_RightFootPositions->pop_front();
    }
  else
    {
      std::cerr << "Problem on the right foot position queue: empty"
                << std::endl;
      return -3;
    }

  if (m_COMBuffer->size()>0)
    {
      finalCOMPosition = (*m_COMBuffer)[0];
      m_COMBuffer->pop_front();
    }
  else
    {
      std::cerr << "Problem on the COM queue: empty"
                << std::endl;
      return -4;
    }

  if(m_ZMPPositions->size()>0)
    {
      ZMPPosition aZMPPosition = (*m_ZMPPositions)[0];
      ZMPRefPos(0) = aZMPPosition.px;
      ZMPRefPos(1) = aZMPPosition.py;
      ZMPRefPos(2) = aZMPPosition.pz;
      m_ZMPPositions->pop_front();
    }
  else
    {
      ODEBUG("Problem on the ZMP size: empty");
      return -5;
    }

  ODEBUG("End of OneGlobalStepOfControl"
         << m_LeftFootPositions->size() << " "
         << m_RightFootPositions->size() << " "
         << m_COMBuffer->size() << " "
         << m_ZMPPositions->size());
  return 0;
}


int CoMAndFootOnlyStrategy::
EvaluateStartingState
(Eigen::VectorXd & BodyAngles,
 COMState & aStartingCOMState,
 Eigen::Vector3d & aStartingZMPPosition,
 Eigen::Matrix<double, 6,1> & lStartingWaistPose,
 FootAbsolutePosition & InitLeftFootPosition,
 FootAbsolutePosition & InitRightFootPosition)
{
  Eigen::Vector3d lStartingCOMState;

  lStartingCOMState(0) = aStartingCOMState.x[0];
  lStartingCOMState(1) = aStartingCOMState.y[0];
  lStartingCOMState(2) = aStartingCOMState.z[0];

  std::vector<ComAndFootRealization *>::iterator itCFR ;
  for (itCFR = m_ComAndFootRealization.begin() ;
       itCFR != m_ComAndFootRealization.end() ;
       ++itCFR )
    {
      // here we use the analytical forward kinematics
      // to initialise the position of the CoM of mass according to
      // the articular position of the robot.
      (*itCFR)->InitializationCoM
        (BodyAngles,lStartingCOMState,
         lStartingWaistPose,
         InitLeftFootPosition,
         InitRightFootPosition);

      ODEBUG("EvaluateStartingCOM: m_StartingCOMState: "
             << lStartingCOMState);
      aStartingCOMState.x[0] = lStartingCOMState(0);
      aStartingCOMState.y[0] = lStartingCOMState(1);
      aStartingCOMState.z[0] = lStartingCOMState(2);
      aStartingCOMState.yaw[0] = lStartingWaistPose(5);
      aStartingCOMState.pitch[0] = lStartingWaistPose(4);
      aStartingCOMState.roll[0] = lStartingWaistPose(3);
      aStartingZMPPosition= (*itCFR)->GetCOGInitialAnkles();
    }

  // We assume that the robot is not moving
  // at the beginning so the zmp is the projection of the com on the ground.
  aStartingZMPPosition(0) = aStartingCOMState.x[0] ;
  aStartingZMPPosition(1) = aStartingCOMState.y[0] ;
  // The  altitude of the zmp depend on the altitude of the support foot.
  aStartingZMPPosition(2) = 0.5 *
    (InitLeftFootPosition.z + InitRightFootPosition.z) ;

  ///  cerr << "YOU SHOULD INITIALIZE PROPERLY aStartingZMPosition in
  ///  CoMAndFootOnlyStrategy::EvaluateStartingState" <<endl;

  ODEBUG("com = " << aStartingCOMState);
  ODEBUG("zmp = " << aStartingZMPPosition);
  ODEBUG("lf = " << InitLeftFootPosition);
  ODEBUG("rf = " << InitRightFootPosition);
  return 0;
}

int CoMAndFootOnlyStrategy::EndOfMotion()
{
  // Just testing one buffer.
  // They suppose to have the same behavior.

  if (m_LeftFootPositions->size()>m_BufferSizeLimit)
    {
      if (m_LeftFootPositions->size()==m_BufferSizeLimit+1)
        {
          ODEBUG("LeftFootPositions position ( "<< (*m_LeftFootPositions)[0].x
                 << " , " << (*m_LeftFootPositions)[0].y << " ) " );
        }

      m_NbOfHitBottom=0;
      return 1;
    }
  else if ((m_LeftFootPositions->size()==m_BufferSizeLimit) &&
           (m_NbOfHitBottom==0))
    {
      ODEBUG("LeftFootPositions size : "<< m_LeftFootPositions->size()
             << "Buffer size limit: " << m_BufferSizeLimit);

      m_NbOfHitBottom++;
      return 0;
    }
  else if ((m_LeftFootPositions->size()==m_BufferSizeLimit) &&
           (m_NbOfHitBottom>0))
    {
      return -1;
    }
  return 0;
}

void CoMAndFootOnlyStrategy::
Setup
(deque<ZMPPosition>
 &,          // aZMPPositions,
 deque<COMState> &,             // aCOMBuffer,
 deque<FootAbsolutePosition> &, // aLeftFootAbsolutePositions,
 deque<FootAbsolutePosition> &) // aRightFootAbsolutePositions)
{
}

void CoMAndFootOnlyStrategy::CallMethod(std::string &,//Method,
                                        std::istringstream &)// astrm)
{
}

void CoMAndFootOnlyStrategy::SetTheLimitOfTheBuffer(unsigned int
                                                    lBufferSizeLimit)
{
  m_BufferSizeLimit = lBufferSizeLimit;
}

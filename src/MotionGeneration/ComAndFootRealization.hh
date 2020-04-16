/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Florent Lamiraux
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
/* Abstract class to generate Humanoid Body motion
   for given CoM and foot postures.
*/

#ifndef _COM_AND_FOOT_REALIZATION_H_
#define _COM_AND_FOOT_REALIZATION_H_

#include <SimplePlugin.hh>
#include <StepStackHandler.hh>
#include <jrl/walkgen/pinocchiorobot.hh>

namespace PatternGeneratorJRL {
class PatternGeneratorInterfacePrivate;

/** @ingroup motiongeneration
    This abstract specifies the different methods
    called by the Pattern Generator to generate a body posture.
    Taking as an input CoM and feet postures, it is in charge of
    finding a body posture, according to different strategies.

    The body posture, and the necessary information are store in an instance
    of \a CjrlHumanoidDynamicRobot.

    Mostly they have to provide a ZMP multibody after the first
    preview loop of the class \a ZMPPreviewControlWithMultiBodyZMP class.

*/
class ComAndFootRealization : public SimplePlugin {
private:
  /*! \brief Store the dynamic robot. */
  PinocchioRobot *m_PinocchioRobot;

  /*! Store the height of the CoM */
  double m_HeightOfCoM;

  /*! Sampling Period. */
  double m_SamplingPeriod;

  /*! Object which handles a Stack of steps */
  StepStackHandler *m_StepStackHandler;

public:
  /* \name Constructor and destructor.*/

  /*! \brief Constructor

   */
  inline ComAndFootRealization(
      PatternGeneratorInterfacePrivate *aPatternGeneratorInterface)
      : SimplePlugin((PatternGeneratorJRL::SimplePluginManager *)
                         aPatternGeneratorInterface),
        m_PinocchioRobot(0), m_HeightOfCoM(0), m_SamplingPeriod(0.005),
        m_StepStackHandler(0){};

  /*! \brief virtual destructor */
  inline virtual ~ComAndFootRealization(){};

  /*! \brief Initialization phase */
  virtual void Initialization() = 0;
  /** @} */

  /*! \name Methods related to the computation to be asked by
    \a  ZMPPreviewControlWithMultiBodyZMP.  */

  /*! Compute the robot state for a given CoM and feet posture.
    Each posture is given by a 3D position and two Euler angles
    \f$ (\theta, \omega) \f$.
    Very important: This method is assume to set correctly the body angles of
    its \a HumanoidDynamicRobot and a subsequent call to the ZMP position
    will return the associated ZMP vector.
    @param CoMPosition: a 5 dimensional vector with the first dimension
    for position,
    and the last two for the orientation (Euler angle).
    @param CoMSpeed: a 5 dimensional vector: 3 for the linear velocity in
    X,Y,Z,
    and two for the angular velocity.
    @param CoMAcc: a 5 dimensional vector: 3 for the linear acceleration in
    X,Y,Z,
    and two for the angular acceleration.
    @param LeftFoot: a 5 dimensional following the same convention than for
    \a CoMPosition.
    @param RightFoot: idem.
    @param CurrentConfiguration: The result is a state vector containing the
    position which are put inside this parameter.
    @param CurrentVelocity: The result is a state vector containing the speed
    which are put inside this parameter.x
    @param CurrentAcceleration: The result is a state vector containing the
    acceleration which are put inside this parameter.x
    @param IterationNumber: Number of iteration.
    @param Stage: indicates which stage is reach by the Pattern Generator.
  */
  virtual bool ComputePostureForGivenCoMAndFeetPosture(
      Eigen::VectorXd &COMPOSITION, Eigen::VectorXd &CoMSpeed,
      Eigen::VectorXd &CoMAcc, Eigen::VectorXd &LeftFoot,
      Eigen::VectorXd &RightFoot, Eigen::VectorXd &CurrentConfiguration,
      Eigen::VectorXd &CurrentVelocity, Eigen::VectorXd &CurrentAcceleration,
      unsigned long int IterationNumber, int Stage) = 0;

  /*! Returns the waist position associate to the current
    @} */

  /*! \name Setter and getter for the jrlHumanoidDynamicRobot object. */

  /*! @param aHumanoidDynamicRobot: an object able to
    compute dynamic parameters
    of the robot. */
  inline virtual bool setPinocchioRobot(PinocchioRobot *aPinocchioRobot) {
    m_PinocchioRobot = aPinocchioRobot;
    return true;
  }

  /*! Returns the object able to compute dynamic parametersof the robot. */
  inline PinocchioRobot *getPinocchioRobot() const { return m_PinocchioRobot; }

  /** @} */

  /*! This initialization phase does the following:
    1/ we take the current state of the robot
    to compute the current CoM value.
    2/ We deduce the difference between the CoM and the waist,
    which is suppose to be constant for the all duration of the motion.

    IMPORTANT: The jrlHumanoidDynamicRobot must have been properly set up.

  */
  virtual bool
  InitializationCoM(Eigen::VectorXd &BodyAnglesIni,
                    Eigen::Vector3d &lStartingCOMPosition,
                    Eigen::Matrix<double, 6, 1> &lStartingWaistPose,
                    FootAbsolutePosition &InitLeftFootAbsPos,
                    FootAbsolutePosition &InitRightFootAbsPos) = 0;

  /*! This initialization phase, make sure that the needed buffers
    for the upper body motion are correctly setup.
  */
  virtual bool InitializationUpperBody(
      deque<ZMPPosition> &inZMPPositions, deque<COMPosition> &inCOMBuffer,
      deque<RelativeFootPosition> lRelativeFootPositions) = 0;

  /* @} */

  /*! Set the algorithm used for ZMP and CoM trajectory. */
  void SetAlgorithmForZMPAndCoMTrajectoryGeneration(int anAlgo);

  /*! Get the algorithm used for ZMP and CoM trajectory. */
  int GetAlgorithmForZMPAndCoMTrajectoryGeneration();

  /*! \name Setter and getter for the height of the CoM.
    @{
  */

  void SetHeightOfTheCoM(double theHeightOfTheCoM) {
    m_HeightOfCoM = theHeightOfTheCoM;
  }

  const double &GetHeightOfTheCoM() const { return m_HeightOfCoM; }
  /*! @} */

  /*! \name Setter and getter for the sampling period.
    @{
  */
  /*! Setter for the sampling period. */
  inline void setSamplingPeriod(double aSamplingPeriod) {
    m_SamplingPeriod = aSamplingPeriod;
  }
  /*! Getter for the sampling period. */
  inline const double &getSamplingPeriod() const { return m_SamplingPeriod; }
  /* @} */

  /*! \name Getter and setter for the handler of step stack  @{ */
  void SetStepStackHandler(StepStackHandler *lStepStackHandler) {
    m_StepStackHandler = lStepStackHandler;
  }

  StepStackHandler *GetStepStackHandler() const { return m_StepStackHandler; }
  /*! @} */

  /*! Get the current position of the waist in the COM reference frame
    @return a 4x4 matrix which contains the pose and the position of the waist
    in the CoM reference frame.
  */
  virtual Eigen::Matrix4d GetCurrentPositionofWaistInCOMFrame() = 0;

  /*! \brief Get the COG of the ankles at the starting position. */
  virtual Eigen::Vector3d GetCOGInitialAnkles() = 0;
};

} // namespace PatternGeneratorJRL

#endif

/*
 * Copyright 2007, 2008, 2009, 2010,
 *
 * Andrei     Herdt
 * Fumio      Kanehiro
 * Francois   Keith
 * Alireza    Nakhaei
 * Olivier    Stasse
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
/*! \file PatternGeneratorInterface.h
  \brief  This object provides a unified interface to access
  the pattern generator.
  It allows to hide all the computation and hacking to the user.
*/

#ifndef _PATTERN_GENERATOR_INTERFACE_H_
#define _PATTERN_GENERATOR_INTERFACE_H_

#include <deque>
#include <jrl/walkgen/pgtypes.hh>
#include <jrl/walkgen/pinocchiorobot.hh>

namespace PatternGeneratorJRL {

/** @ingroup Interface
    This class is the interface between the Pattern Generator and the
    external world. In addition to the classical setter and getter for various
    parameters
    there is the possibility to pass commands a string of stream to the method
    \a ParseCmd().

    There is a set of functionnalities directly supported by the API:



*/
class WALK_GEN_JRL_EXPORT PatternGeneratorInterface {
public:
  // overload the new[] eigen operator
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /*! Constructor
    @param strm: Should provide the file to initialize the preview control,
    the path to the VRML model, and the name of the file containing the VRML
    model.
  */
  PatternGeneratorInterface(PinocchioRobot *){};

  /*! Destructor */
  virtual ~PatternGeneratorInterface(){};

  /*! \brief Function to specify steps in the stack of the walking pattern
    generator.
    This method is different AddOnLineStep which is the default step add
    when there is no policy,
    or no step available.
  */
  virtual void AddStepInStack(double dx, double dy, double theta) = 0;

  /*! Common Initialization of walking.
    @param[out] lStartingCOMPosition: For the starting position on the
    articular space, returns
    the COM position.
    @param[out] lStartingZMPPosition: For the starting position on the
    articular space, returns
    the ZMP position.
    @param[out] BodyAnglesIni: Basically it is a copy of CurrentJointValues
    but as a vector.
    @param[out] InitLeftFootAbsPos: Returns the current absolute position
    of the left foot for
    the given posture of the robot.
    @param[out] InitRightFootAbsPos: Returns the current absolute position
    of the right foot
    for the given posture of the robot.
    @param[out] lRelativeFootPositions: List of relative positions for the
    support foot still in the
    stack of steps.
    @param[in] lCurrentJointValues: The vector of articular values in
    classical C++ style.
    @param[in] ClearStepStackHandler: Clean the stack of steps after copy.
  */
  virtual void CommonInitializationOfWalking(
      COMState &lStartingCOMState, Eigen::Vector3d &lStartingZMPPosition,
      Eigen::VectorXd &BodyAnglesIni, FootAbsolutePosition &InitLeftFootAbsPos,
      FootAbsolutePosition &InitRightFootAbsPos,
      std::deque<RelativeFootPosition> &lRelativeFootPositions,
      std::vector<double> &lCurrentJointValues, bool ClearStepStackHandler) = 0;

  /*! \name Methods for the control part.
    @{
  */

  /*! \brief Run One Step of the global control loop aka
    The Main Method To Be Used.
    @param[out]  CurrentConfiguration
    The current configuration of the robot according to
    the implementation of dynamic-JRLJapan.
    This should be first position and orientation
    of the waist, and then all the DOFs of your robot.
    @param[out]  CurrentVelocity
    The current velocity of the robot according to the
    the implementation of dynamic-JRLJapan.
    @param[out]  CurrentAcceleration
    The current acceleration of the robot according to the
    the implementation of dynamic-JRLJapan.
    @param[out]  ZMPTarget
    The target ZMP in the waist reference frame.
    @return True is there is still some data to send, false otherwise.
  */
  virtual bool RunOneStepOfTheControlLoop(Eigen::VectorXd &CurrentConfiguration,
                                          Eigen::VectorXd &CurrentVelocity,
                                          Eigen::VectorXd &CurrentAcceleration,
                                          Eigen::VectorXd &ZMPTarget) = 0;

  /*! \brief Run One Step of the global control loop aka
    The Main Method To Be Used.
    @param[out]  CurrentConfiguration The current configuration of the robot
    according to
    the implementation of dynamic-JRLJapan. This should be first position
    and orientation
    of the waist, and then all the DOFs of your robot.
    @param[out]  CurrentVelocity  The current velocity of the robot
    according to the
    the implementation of dynamic-JRLJapan.
    @param[out]  CurrentAcceleration  The current acceleration of the
    robot according to the
    the implementation of dynamic-JRLJapan.
    @param[out]  ZMPTarget  The target ZMP in the waist reference frame.
    @param[out] COMPosition The CoM position for this motion.
    @param[out] LeftFootPosition: Absolute position of the left foot.
    @param[out] RightFootPosition: Absolute position of the right foot.
    @return True is there is still some data to send, false otherwise.
  */
  virtual bool RunOneStepOfTheControlLoop(
      Eigen::VectorXd &CurrentConfiguration, Eigen::VectorXd &CurrentVelocity,
      Eigen::VectorXd &CurrentAcceleration, Eigen::VectorXd &ZMPTarget,
      COMPosition &COMPosition, FootAbsolutePosition &LeftFootPosition,
      FootAbsolutePosition &RightFootPosition) = 0;

  /*! \brief Run One Step of the global control loop aka
    The Main Method To Be Used.
    @param[out]  CurrentConfiguration The current configuration of the robot
    according to
    the implementation of dynamic-JRLJapan. This should be first position and
    orientation
    of the waist, and then all the DOFs of your robot.
    @param[out]  CurrentVelocity  The current velocity of the robot according
    to the
    the implementation of dynamic-JRLJapan.
    @param[out]  CurrentAcceleration  The current acceleration of the robot
    according to the
    the implementation of dynamic-JRLJapan.
    @param[out]  ZMPTarget  The target ZMP in the waist reference frame.
    @param[out] COMState The CoM state (up to the acceleration) for this
    motion.
    @param[out] LeftFootPosition: Absolute position of the left foot.
    @param[out] RightFootPosition: Absolute position of the right foot.
    @return True is there is still some data to send, false otherwise.
  */
  virtual bool RunOneStepOfTheControlLoop(
      Eigen::VectorXd &CurrentConfiguration, Eigen::VectorXd &CurrentVelocity,
      Eigen::VectorXd &CurrentAcceleration, Eigen::VectorXd &ZMPTarget,
      COMState &COMState, FootAbsolutePosition &LeftFootPosition,
      FootAbsolutePosition &RightFootPosition) = 0;

  /*! \brief Run One Step of the global control loop aka
    The Main Method To Be Used.
    @param[out] LeftFootPosition: Absolute position of the left foot.
    @param[out] RightFootPosition: Absolute position of the right foot.
    @param[out] ZMPRefPos: ZMP position new reference
    @param[out] COMRefPos: COM position new reference.
    @return True is there is still some data to send, false otherwise.
  */
  virtual bool
  RunOneStepOfTheControlLoop(FootAbsolutePosition &LeftFootPosition,
                             FootAbsolutePosition &RightFootPosition,
                             ZMPPosition &ZMPRefPos,
                             COMPosition &COMRefPos) = 0;
  /*! @} */

  /*! Set the current joint values of the robot.
    This method is used to properly initialize the pattern generator.
    It also updates the state of the robot if other control mechanisms
    modifies the upper body part and if this should be taken into account
    into the pattern generator in the second loop of control. */
  virtual void SetCurrentJointValues(Eigen::VectorXd &lCurrentJointValues) = 0;

  /*! \brief Returns the walking mode. */
  virtual int GetWalkMode() const = 0;

  /*! \brief Get the leg joint velocity */
  virtual void GetLegJointVelocity(Eigen::VectorXd &dqr,
                                   Eigen::VectorXd &dql) const = 0;

  /*! \brief Read a sequence of steps. */
  virtual void ReadSequenceOfSteps(std::istringstream &strm) = 0;

  /*! \name On-line steps related methods
    @{
  */
  /*! \brief Start the creation of steps on line. */
  virtual void StartOnLineStepSequencing() = 0;

  /*! \brief Stop the creation of steps on line. */
  virtual void StopOnLineStepSequencing() = 0;

  /*! \brief Add an online step */
  virtual void AddOnLineStep(double X, double Y, double Theta) = 0;

  /*! \brief Change online step.
    The strategy is the following: the step in single support phase at time t
    has its landing position changed to \f$ (X,Y,\theta) \f$ in absolute
    coordinates (i.e. in the world reference frame of the free flyer of the
    robot).
    For stability reason there is no guarantee that this method can
    realized the operation. Please see the documentation of the walking
    pattern generator
    algorithm used.

    If the time falls during a double support phase, the next single support
    phase is chosen.

    @param[in] Time: Time information of the step.
    @param[in] aFootAbsolutePosition: Absolute position of the foot.
    @return If the operation failed the method returns a negative number
    related
    to an error, 0 otherwise.
  */
  virtual int ChangeOnLineStep(double Time,
                               FootAbsolutePosition &aFootAbsolutePosition,
                               double &newtime) = 0;

  /*! \brief Change online step.
    See the above method for the specifications.
    This method uses a different format with stream of strings.
    @param[in] Time: Time information of the step.
    @return nothing */
  virtual void ChangeOnLineStep(std::istringstream &strm, double &newtime) = 0;

  /*! @} */

  /*! \name For internal odometry.
    @{ */
  /*! \brief Update the current waist absolute position */
  virtual void UpdateAbsolutePosition(bool UpdateAbsMotionOrNot) = 0;

  /*! \brief Get the waist position and orientation as a quaternion,
    and the planar X-Y orientation in Orientation. */
  virtual void getWaistPositionAndOrientation(double TQ[7],
                                              double &Orientation) const = 0;

  /*! \brief Set Waist position and Orientation  */
  virtual void setWaistPositionAndOrientation(double TQ[7]) = 0;

  /*! \brief Get Waist velocity */
  virtual void getWaistVelocity(double &dx, double &dy,
                                double &omega) const = 0;

  /*! \brief An other method to get the waist position using a matrix. */
  virtual void getWaistPositionMatrix(Eigen::Matrix4d &lWaistAbsPos) const = 0;

  /*!@} */

  /*! \brief Set the initial ZMP reference point. */
  virtual void setZMPInitialPoint(Eigen::Vector3d &lZMPInitialPoint) = 0;

  /*! \brief Get the initial ZMP reference point. */
  virtual void getZMPInitialPoint(Eigen::Vector3d &lZMPInitialPoint) const = 0;

  /*! \name System to call a given method based on registration of a method.
    @{
  */

  /*! \brief Parse a command (to be used out of a plugin)
    and call all objects which registered the method. */
  virtual int ParseCmd(std::istringstream &strm) = 0;

  /*! @} */

  /*! \brief Returns the ZMP, CoM, left foot absolute position, and
    right foot absolute position
    for the initiale pose.*/
  virtual void
  EvaluateStartingState(COMState &lStartingCOMState,
                        Eigen::Vector3d &lStartingZMPPosition,
                        Eigen::Matrix<double, 6, 1> &lStartingWaistPose,
                        FootAbsolutePosition &InitLeftFootAbsPos,
                        FootAbsolutePosition &InitRightFootAbsPos) = 0;

  /*! @} */

  /*! \brief Set velocity reference
    This method is only supported by Herdt's algorithm.
    Currently only a 3D speed is supported:
    \param x: Velocity along the saggital plane.
    \param y: Velocity along the perpendicular plane.
    \param yaw: Angular velocity in the x-y plane.
  */
  virtual void setVelocityReference(double x, double y, double yaw) = 0;
  /*! \brief Set velocity reference
    \param x: Additive acceleration along the saggital plane.
    \param y: Additive acceleration along the lateral plane.
  */
  virtual void setCoMPerturbationForce(double x, double y) = 0;
};

/*! Factory of Pattern generator interface. */
WALK_GEN_JRL_EXPORT PatternGeneratorInterface *
patternGeneratorInterfaceFactory(PinocchioRobot *);
} // namespace PatternGeneratorJRL

#endif /* _PATTERN_GENERATOR_INTERFACE_H_ */

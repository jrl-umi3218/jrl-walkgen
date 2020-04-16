/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 *
 * Fumio   Kanehiro
 * Florent Lamiraux
 * Alireza Nakhaei
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
/*! \file ZMPPreviewControlWithMultiBodyZMP.h
  \brief This object generate all the values for the foot trajectories,
  and the desired ZMP based on a sequence of steps.
*/
#ifndef _ZMPREVIEWCONTROLWITHMULTIBODYZMP_H_
#define _ZMPREVIEWCONTROLWITHMULTIBODYZMP_H_

#include <deque>

#include <MotionGeneration/ComAndFootRealization.hh>
#include <PreviewControl/PreviewControl.hh>
#include <SimplePlugin.hh>
#include <jrl/walkgen/pgtypes.hh>

using namespace ::std;

namespace PatternGeneratorJRL {
/** @ingroup pgjrl

    Object to generate the angle positions
    every 5 ms from a set of absolute foot positions.
    This object handles one iteration.

    This algorithm use the preview control proposed by Kajita-San
    in \ref Kajita2003 with the two stages archictecture.

    You therefore have to use first the Setup method
    to fill all the queues. Then every 5 ms just use
    OneGlobalStepOfControl to compute the Waist
    Computation position, speed, acceleration and the
    angular values for the left and right legs.

    This class can also be used with Pierre-Brice Wieber algorithm's
    \ref Wieber2006 where only the second stage is used.

    Finally in the case that the strategy adopted do not involve
    to compute the second stage and the first stage you can use the
    dummy mode. The architecture is kept the same but no computation
    are performed.

*/
class ZMPPreviewControlWithMultiBodyZMP : public SimplePlugin {
private:
  /*! Vector from the Waist to the left and right hip. */
  //@{
  /*! Static part from the waist to the left hip.. */
  Eigen::Vector3d m_StaticToTheLeftHip;
  /*! Static part from the waist to the right hip. */
  Eigen::Vector3d m_StaticToTheRightHip;
  /*! Dynamic part from the waist to the left hip. */
  Eigen::Vector3d m_TranslationToTheLeftHip;
  /*! Dynamic part form the waist to the right hip. */
  Eigen::Vector3d m_TranslationToTheRightHip;

  /*! Displacement between the hip and the foot. */
  Eigen::Vector3d m_Dt;

  /*! Preview control time. */
  double m_PreviewControlTime;

  /*! Size of the preview control window. */
  unsigned int m_NL;

  /*! Final state of the leg joints. */
  //@{
  /*! The left leg */
  Eigen::MatrixXd Finalql;
  /*! The right leg */
  Eigen::MatrixXd Finalqr;
  //@}

  /*! Fifo for the ZMP ref. */
  deque<ZMPPosition> m_FIFOZMPRefPositions;

  /*! Fifo for the ZMP ref. */
  deque<ZMPPosition> m_FIFODeltaZMPPositions;

  /*! Fifo for the COM reference. */
  deque<COMState> m_FIFOCOMStates;

  /*! Fifo for the positionning of the left foot. */
  deque<FootAbsolutePosition> m_FIFOLeftFootPosition;

  /*! Fifo for the positionning of the right foot. */
  deque<FootAbsolutePosition> m_FIFORightFootPosition;

  /*! Error on preview control for the cart model. */
  double m_sxzmp, m_syzmp;

  /*! Error on preview control for the delta zmp. */
  double m_sxDeltazmp, m_syDeltazmp;

  /*! State of the Preview control. */
  Eigen::MatrixXd m_PC1x;
  Eigen::MatrixXd m_PC1y;

  /*! State of the Second Preview control. */
  Eigen::MatrixXd m_Deltax;
  Eigen::MatrixXd m_Deltay;

  /*! Starting a new step sequences. */
  bool m_StartingNewSequence;

  /*! Keep the ZMP reference. */
  deque<ZMPPosition> m_FIFOTmpZMPPosition;

  /*!extra COMState buffer calculated to give to the stepover planner  */
  std::vector<COMState> m_ExtraCOMBuffer;

  /*! Difference between the CoM and the Waist
    from the initialization phase,
    i.e. not reevaluated while walking. */
  Eigen::Vector3d m_DiffBetweenComAndWaist;

  /*! COM Starting position. */
  Eigen::Vector3d m_StartingCOMState;

  /*! Final COM pose. */
  Eigen::Matrix4d m_FinalDesiredCOMPose;

  /*! Store the distance between the ankle and the soil. */
  double m_AnkleSoilDistance;

  /*! Store a reference to the object to solve posture resolution. */
  ComAndFootRealization *m_ComAndFootRealization;

  /*! Store a reference to the object handling humanoid dynamics */
  PinocchioRobot *m_PinocchioRobot;

  /*! Number of iterations. */
  unsigned long int m_NumberOfIterations;

  /*! Pointer to the Preview Control object. */
  PreviewControl *m_PC;

  /*! Store the strategy to handle the preview control stages. */
  int m_StageStrategy;

  /*! Sampling period. */
  double m_SamplingPeriod;

  /*! Register method. */
  void RegisterMethods();

  /*! Set the sampling period and update NL.*/
  void SetSamplingPeriod(double lSamplingPeriod);

  /*! Set the preview control time and update NL. */
  void SetPreviewControlTime(double lPreviewControlTime);

public:
  /*! Constantes to define the strategy with the first and second stage.
    @{
  */

  /*! Constant to compute the first and second stage. */
  static const int ZMPCOM_TRAJECTORY_FULL = 1;

  /*! Constant to compute only the second stage */
  static const int ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY = 2;

  /*! Constant to compute only the first stage. */
  static const int ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY = 3;

  /*! @} */
  /*! Constructor. */
  ZMPPreviewControlWithMultiBodyZMP(SimplePluginManager *lSPM);

  /*! Destroctor. */
  ~ZMPPreviewControlWithMultiBodyZMP();

  /*! \name Implementation of the GlobalStrategyManager interface.
    @{ */
  /*! Set the algorithm used for ZMP and CoM trajectory.
    @param[in] anAlgo:
    The algorithm to be used for ZMP and CoM trajectory generation.
    They are 3 possible values:

    \li ZMPCOM_TRAJECTORY_FULL: Two preview control are computed. The first
    to generate a CoM trajectory based on the cart model.
    The second to correct
    this trajectory using the multibody ZMP.

    \li ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY: Only the second stage is used.
    The first CoM trajectory is used by a different process. This allow
    to mix different algorithms (notable the quadratic problem with
    constraints).

    \li ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY: Use only the first stage
    to generate the CoM trajectory.
    It is strongly adviced in this case, to not use
    the geometrical ZMP and CoM trajectory generation but an external
    CoM task.

    @return Returns false if this is not possible.
  */
  void SetStrategyForStageActivation(int anAlgo);

  /*! Get the strategy for the activation of the stage.
   */
  int GetStrategyForStageActivation();

  /*! @} */
  /*! Returns the difference between the Waist and the CoM
    for a starting position. */
  void GetDifferenceBetweenComAndWaist(double lComAndWaist[3]);

  /*! Perform a 5 ms step to generate the full set of angular positions.
    The main point of the preview control is to use the future to compute
    the current state needed for the robot. Therefore knowing that
    the future window needed is of size NL=SamplingPeriod *
    PreviewControlWindow,
    and that the algorithm is a two stages preview control,
    the foot position needs to be provided at k+NL, and the ZMP references
    at k+2*NL.

    @param[in] LeftFootPosition: The position of the k+NL Left Foot position.
    @param[in] RightFootPosition: The position of the k+NL
    Right Foot position.
    @param[in] NewZMPRefPos: The ZMP position at k + 2*NL.
    @param[out] finalCOMState: returns position, velocity and acceleration of
    the CoM
    after the second stage of control, i.e. the final value.
    @param[out] CurrentConfiguration: The results is a state vector
    containing the articular positions.
    @param[out] CurrentVelocity: The results is a state vector containing the
    speed.
    @param[out] CurrentAcceleration: The results is a state vector containing
    the acceleration.
  */
  int OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
                             FootAbsolutePosition &RightFootPosition,
                             ZMPPosition &NewZMPRefPos, COMState &finalCOMState,
                             Eigen::VectorXd &CurrentConfiguration,
                             Eigen::VectorXd &CurrentVelocity,
                             Eigen::VectorXd &CurrentAcceleration);

  /*! First stage of the control,
    i.e.preview control on the CART model with delayed step parameters,
    Inverse Kinematics, and ZMP calculated with the multi body model.
    aCOMState will be updated with the new value of the COM computed by
    the card model.
    @param[in] LeftFootPosition: The position of the k+NL Left Foot position.
    @param[in] RightFootPosition: The position of the k+NL
    Right Foot position.
    @param[in] afCOMState: A COM position of reference, in this context,
    this will be the height of the waist.

    @return If an error occurs returns a negative integer, 0 otherwise.
  */
  int FirstStageOfControl(FootAbsolutePosition &LeftFootPosition,
                          FootAbsolutePosition &RightFootPosition,
                          COMState &afCOMState);

  /*! This methods is used only to update the queue of ZMP difference
    for the second stage of control. Also it does not return
    anything this method is crucial for the overall process.

    @param[in] StartingIteration: -1 for the initialization, >=0 for
    a counter which gives the time.

    @return If an error occurs returns a negative integer, 0 otherwise.
  */
  int EvaluateMultiBodyZMP(int StartingIteration);

  /*! Second stage of the control, i.e. preview control on the Delta ZMP.
    COM correction, and computation of the final robot state
    (only the left and right legs).

    @param[out] refandfinal: The final position of the CoM.
    @return If an error occurs returns a negative integer, 0 otherwise.
  */
  int SecondStageOfControl(COMState &refandfinal);

  /*! Compute the COM of the robot with the Joint values given in BodyAngles,
    velocities set to zero, and returns the values of the COM in
    aStaringCOMState.
    Assuming that the waist is at (0,0,0)
    it returns the associate initial values for the left and right foot.
    @param[in] BodyAngles: 4x4 matrix of the robot's root (most of the time,
    the waist)
    pose (position + orientation).
    @param[out] aStartingCOMState: Returns the 3D position of the CoM for the
    current
    position of the robot.
    @param[out] aStartingZMPPosition: Returns the 3D position of the ZMP for
    the current
    position of the robot.
    @param[out] InitLeftFootPosition: Returns the position of the left foot in
    the waist coordinates frame.
    @param[out] InitRightFootPosition: Returns the position of the right foot
    in the waist coordinates frame.
  */
  int EvaluateStartingState(Eigen::VectorXd &BodyAngles,
                            Eigen::Vector3d &aStartingCOMState,
                            Eigen::Vector3d &aStartingZMPPosition,
                            Eigen::Matrix<double, 6, 1> &aStartingWaistPose,
                            FootAbsolutePosition &InitLeftFootPosition,
                            FootAbsolutePosition &InitRightFootPosition);

  /*! Compute the COM of the robot with the Joint values given in BodyAngles,
    velocities set to zero, and returns the values of the COM in
    aStaringCOMState.
    Assuming that the waist is at (0,0,0)
    it returns the associate initial values for the left and right foot.
    @param BodyAngles: Vector of the joint values for the robot.
    @param[out] aStartingCOMState: Position of the CoM.
    @param[out] aWaistPosition: Position of the Waist.
    @param[out] InitLeftFootPosition: Position of the left foot in the
    waist coordinates frame.
    @param[out] InitRightFootPosition: Position of the right foot in the
    waist coordinates frame.
  */
  int EvaluateStartingCoM(Eigen::MatrixXd BodyAngles,
                          Eigen::Vector3d &aStartingCOMState,
                          Eigen::VectorXd &aWaistPose,
                          FootAbsolutePosition &InitLeftFootPosition,
                          FootAbsolutePosition &InitRightFootPosition);

  /*! Methods related to the preparation of the ZMP preview control with
    Multibody ZMP compensation.
    @{
  */

  /*! Setup (Frontal Global), compute internally all the steps to get
    NL ZMP multibody values.

    @param[in] ZMPRefPositions: FIFO of the ZMP reference values.
    @param[out] COMStates: FIFO of the COM reference positions
    (here only the height position is taken into account).
    @param[in] LeftFootPositions: FIFO of the left foot positions computed by
    ZMPDiscretization (the object creating the ZMP and foot reference
    trajectories).
    @param[in] RightFootPositions: idem than the previous one but for the
    right foot.
  */
  int Setup(deque<ZMPPosition> &ZMPRefPositions, deque<COMState> &COMStates,
            deque<FootAbsolutePosition> &LeftFootPositions,
            deque<FootAbsolutePosition> &RightFootPositions);

  /*! Method to perform the First Phase. It initializes properly
    the internal fields of ZMPPreviewControlWithMultiBodyZMP
    for the setup phase.

    @param[in] ZMPRefPositions: FIFO of the ZMP reference values.
    @param[in] COMStates: FIFO of the COM reference positions
    (here only the height position is taken into account).
    @param[in] LeftFootPositions: FIFO of the left foot positions computed by
    ZMPDiscretization (the object creating the ZMP and foot reference
    trajectories).
    @param[in] RightFootPositions: idem than the previous one but for the
    right foot.
  */
  int SetupFirstPhase(deque<ZMPPosition> &ZMPRefPositions,
                      deque<COMState> &COMStates,
                      deque<FootAbsolutePosition> &LeftFootPositions,
                      deque<FootAbsolutePosition> &RightFootPositions);

  /*! Method to call while feeding the 2 preview windows.
    It updates the first values of the Preview control
    This structure is needed if it is needed to modify BodyAngles according
    to the value of the COM.

    @param[in] ZMPRefPositions: FIFO of the ZMP reference values.
    @param[out] COMStates: FIFO of the COM reference positions
    (here only the height position is taken into account).
    @param[in] LeftFootPositions: FIFO of the left foot positions computed by
    ZMPDiscretization (the object creating the ZMP and foot reference
    trajectories).
    @param[in] RightFootPositions: idem than the previous one but for the
    right foot.
    @param[out] CurrentConfiguration: The position part of the state vector
    realizing the current CoM and
    feet position instance.
    @param[out] CurrentVelocity: The velocity part of the state vector
    realizing the current CoM and
    feet position instance.
    @param[out] CurrentAcceleration: The acceleration part of the state
    vector realizing the current CoM and
    feet position instance.
    @param[in] localindex: Value of the index which goes from 0 to 2 * m_NL.
  */
  int SetupIterativePhase(deque<ZMPPosition> &ZMPRefPositions,
                          deque<COMState> &COMStates,
                          deque<FootAbsolutePosition> &LeftFootPositions,
                          deque<FootAbsolutePosition> &RightFootPositions,
                          Eigen::VectorXd &CurrentConfiguration,
                          Eigen::VectorXd &CurrentVelocity,
                          Eigen::VectorXd &CurrentAcceleration, int localindex);

  /*! Create an extra COM buffer with a first preview round to be
    used by the stepover planner.

    @param[out] ExtraCOMBuffer: Extra FIFO for the CoM positions.
    @param[out] ExtraZMPBuffer: Extra FIFO for the ZMP positions
    (for the stepping over
    first preview control).
    @param[out] ExtraZMPRefBuffer: Extra FIFO for the ZMP ref positions.
  */
  void CreateExtraCOMBuffer(deque<COMState> &ExtraCOMBuffer,
                            deque<ZMPPosition> &ExtraZMPBuffer,
                            deque<ZMPPosition> &ExtraZMPRefBuffer);

  /*! Evaluate Starting CoM for a given position.
    @param[in] BodyAnglesInit: The state vector used to compute the CoM.
    @param[out] aStartingCOMState: The CoM of the position specified.
    @param[out] InitLeftFootPosition: Position of the InitLeftFootPosition
    in the same reference frame than the waist.
    @param[out] InitRightFootPosition: Position of the InitRightFootPosition
    in the same reference frame than the waist
  */
  int EvaluateStartingCoM(Eigen::VectorXd &BodyAnglesInit,
                          Eigen::Vector3d &aStartingCOMState,
                          Eigen::Matrix<double, 6, 1> &aStartingWaistPosition,
                          FootAbsolutePosition &InitLeftFootPosition,
                          FootAbsolutePosition &InitRightFootPosition);

  /*! This method returns the final COM pose matrix after the second
    stage of control.
    @return A 4x4 matrix of double which includes the desired final
    CoM position and orientation.*/
  Eigen::Matrix4d GetFinalDesiredCOMPose();

  /*! This method returns the current waist position in the COM reference
    frame. This can be used with the previous method to get the final Waist
    position.
    @return A 4x4 matrix of double which includes the desired final
    Waist in the CoM
    phase position and orientation.*/
  Eigen::Matrix4d GetCurrentPositionofWaistInCOMFrame();

  /*! Returns the last element of the COM FIFO in the first stage of
    control */
  COMState GetLastCOMFromFirstStage();

  /*! Update the queue of ZMP reference value. */
  void UpdateTheZMPRefQueue(ZMPPosition NewZMPRefPos);

  /*! \name Setter and getter for the ComAndZMPTrajectoryGeneration. */
  inline bool setComAndFootRealization(ComAndFootRealization *aCFR) {
    m_ComAndFootRealization = aCFR;
    return true;
  };
  inline ComAndFootRealization *getComAndFootRealization() {
    return m_ComAndFootRealization;
  };

  /*! Call To CoM And Foot Realization object,
    the last values in the stack for the CoM
    and the feet positions will be used.
    @param[in] acomp : COM position,
    @param[in] aLeftFAP: Pose of the left foot (3D position + 2 euler angles)
    @param[in] aRightFAP: Pose of the right foot (3D position + 2 euler
    angles)
    @param[out] CurrentConfiguration: Returns the part of state vector
    corresponding
    to the position of the free floating, and the articular values.
    @param[out] CurrentVelocity: Returns the part of state vector
    corresponding to the velocity of the free floating and the
    articular values.
    @param[out] CurrentAcceleration: Returns the part of state vector
    corresponding to the acceleration of the free floating,
    and the articular values.
    @param[in] IterationNumber: Number of time slot realized so far.
    @param[in] StageOfTheAlgorithm: Indicates if this is the second stage of
    the preview control or the first one.
  */
  void CallToComAndFootRealization(
      COMState &acomp, FootAbsolutePosition &aLeftFAP,
      FootAbsolutePosition &aRightFAP, Eigen::VectorXd &CurrentConfiguration,
      Eigen::VectorXd &CurrentVelocity, Eigen::VectorXd &CurrentAcceleration,
      unsigned long int IterationNumber, int StageOfTheAlgorithm);

  /*! Set the link to the preview control. */
  void SetPreviewControl(PreviewControl *aPC);

  /*! \name Setter and getter for the jrlHumanoidDynamicRobot object. */
  /*! @param[in] aHumanoidDynamicRobot: an object able to compute dynamic
    parameters of the robot. */
  inline bool setPinocchioRobot(PinocchioRobot *aPinocchioRobot) {
    m_PinocchioRobot = aPinocchioRobot;
    return true;
  }

  /*! Returns the object able to compute the dynamic parameters
    of the robot. */
  inline PinocchioRobot *getPinocchioRobot() const { return m_PinocchioRobot; }

  /*! Set the strategy to handle the preview control stages. */
  void SetStrategyForPCStages(int Strategy);

  /*! Get the strategy to handle the preview control stages. */
  int GetStrategyForPCStages();

  /*! \brief Overloading method of SimplePlugin */
  void CallMethod(std::string &Method, std::istringstream &astrm);
};
} // namespace PatternGeneratorJRL
#endif /* _ZMPREVIEWCONTROLWITHMULTIBODYZMP_H_ */

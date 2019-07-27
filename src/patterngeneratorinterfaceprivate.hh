/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Andrei     Herdt
 * Benallegue Mehdi
 * Olivier    Stasse
 * Eiichi     Yoshida
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
  \brief This object provides a unified interface to access 
  the pattern generator.
  It allows to hide all the computation and hacking to the user.
*/


#ifndef _PATTERN_GENERATOR_INTERFACE_PRIVATE_H_
#define _PATTERN_GENERATOR_INTERFACE_PRIVATE_H_

#include <sstream>

#include <jrl/walkgen/patterngeneratorinterface.hh>

#include <jrl/walkgen/configJRLWPG.hh>
#include <PreviewControl/ZMPPreviewControlWithMultiBodyZMP.hh>
#include <PreviewControl/PreviewControl.hh>

#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.hh>
#include <ZMPRefTrajectoryGeneration/ZMPQPWithConstraint.hh>
#include <ZMPRefTrajectoryGeneration/ZMPConstrainedQPFastFormulation.hh>
#include <ZMPRefTrajectoryGeneration/ZMPVelocityReferencedQP.hh>
#include <ZMPRefTrajectoryGeneration/ZMPVelocityReferencedSQP.hh>
#include <ZMPRefTrajectoryGeneration/AnalyticalMorisawaCompact.hh>

#include <MotionGeneration/ComAndFootRealizationByGeometry.hh>
#include <MotionGeneration/GenerateMotionFromKineoWorks.hh>
#include <MotionGeneration/StepOverPlanner.hh>

#include <FootTrajectoryGeneration/LeftAndRightFootTrajectoryGenerationMultiple.hh>

#include <StepStackHandler.hh>

#include <SimplePluginManager.hh>
#include <SimplePlugin.hh>

#include <GlobalStrategyManagers/DoubleStagePreviewControlStrategy.hh>
#include <GlobalStrategyManagers/CoMAndFootOnlyStrategy.hh>

namespace PatternGeneratorJRL
{

  /** @ingroup Interface
      This class is the interface between the Pattern Generator and the
      external world. In addition to the classical setter and getter 
      for various parameters
      there is the possibility to pass commands a string of stream to the method
      \a ParseCmd().

      There is a set of functionalities directly supported by the API:



  */
  class PatternGeneratorInterfacePrivate : public virtual
  PatternGeneratorInterface,
                                           SimplePluginManager, SimplePlugin

  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*! Constructor
      @param strm: Should provide the file to initialize the preview control,
      the path to the VRML model, and the name of the file containing 
      the VRML model.
    */
    PatternGeneratorInterfacePrivate(PinocchioRobot *aPinocchioRobotRobot);

    /*! Destructor */
    ~PatternGeneratorInterfacePrivate();

    /*! \brief Function to specify steps in the stack of the walking pattern
      generator.
      This method is different AddOnLineStep which is the default step add when
      there is no policy, or no step available.
    */
    void AddStepInStack(double dx, double dy, double theta);

    /*! \name High levels function to create automatically stack of steps 
      following specific motions.
      @{
    */
    /*! \brief This methods generate a stack of steps which make the robot 
      follows an arc.
      The direction of the robot is tangential to the arc.

      @param[in] x: Position of the center of the circle along the X-axis.
      @param[in] y: Position of the center of the circle along the Y-axis.
      @param[in] R: Ray of the circle.
      @param[in] arc_deg: Arc in degrees along which the robot walks.
      @param[in] SupportFoot: Indicates which is the first support foot (1) 
      Left or (-1) Right.
    */
    void CreateArcInStepStack(  double x,
                                double y,
                                double R,
                                double arc_deg,
                                int SupportFoot);

    /*! \brief This methods generate a stack of steps which make the robot 
      follows an arc.
      The direction of the robot is towards the center of the arc.
      The robot is therefore expected to move sideways.

      @param[in] R: Ray of the circle.
      @param[in] arc_deg: Arc in degrees along which the robot walks.
      @param[in] SupportFoot: Indicates which is the first support foot (1) 
      Left or (-1) Right.
    */
    void CreateArcCenteredInStepStack( double R,
                                       double arc_deg,
                                       int SupportFoot);

    /*! \brief This specifies which foot will be used as the first support 
      of the motion. */
    void PrepareForSupportFoot(int SupportFoot);

    /*! \brief This method precomputes all the buffers necessary for walking
      according to the chosen strategy. */
    void FinishAndRealizeStepSequence();
    /*! @} */


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
      @param[out] lRelativeFootPositions: List of relative positions for 
      the support foot still in the
      stack of steps.
      @param[in] lCurrentJointValues: The vector of articular values in 
      classical C++ style.
      @param[in] ClearStepStackHandler: Clean the stack of steps after copy.
    */
    void CommonInitializationOfWalking
    (COMState & lStartingCOMState,
     Eigen::Vector3d & lStartingZMPPosition,
     Eigen::VectorXd & BodyAnglesIni,
     FootAbsolutePosition & InitLeftFootAbsPos,
     FootAbsolutePosition & InitRightFootAbsPos,
     deque<RelativeFootPosition> & lRelativeFootPositions,
     std::vector<double> & lCurrentJointValues,
     bool ClearStepStackHandler);


    /*! \name Methods for the control part.
      @{
    */

    /*! \brief Run One Step of the global control loop aka The Main Method 
      To Be Used.
      @param[out]  CurrentConfiguration The current configuration of the robot 
      according to the implementation of Pinocchio. 
      This should be first position and orientation
      of the waist, and then all the DOFs of your robot.
      @param[out]  CurrentVelocity  The current velocity of the robot 
      according to the
      the implementation of Pinocchio.
      @param[out]  CurrentAcceleration  The current acceleration of the robot 
      according to the
      the implementation of Pinocchio.
      @param[out]  ZMPTarget  The target ZMP in the waist reference frame.
      @return True is there is still some data to send, false otherwise.
    */
    bool RunOneStepOfTheControlLoop(Eigen::VectorXd & CurrentConfiguration,
                                    Eigen::VectorXd & CurrentVelocity,
                                    Eigen::VectorXd & CurrentAcceleration,
                                    Eigen::VectorXd & ZMPTarget);

    /*! \brief Run One Step of the global control loop aka The Main 
      Method To Be Used.
      @param[out]  CurrentConfiguration The current configuration of the robot 
      according to
      the implementation of Pinocchio. This should be first position and 
      orientation
      of the waist, and then all the DOFs of your robot.
      @param[out]  CurrentVelocity  The current velocity of the robot according 
      to the
      the implementation of Pinocchio.
      @param[out]  CurrentAcceleration  The current acceleration of the robot 
      according to the
      the implementation of Pinocchio.
      @param[out]  ZMPTarget  The target ZMP in the waist reference frame.
      @param[out] COMState The CoM state for this motion.
      @param[out] LeftFootPosition: Absolute position of the left foot.
      @param[out] RightFootPosition: Absolute position of the right foot.
      @return True is there is still some data to send, false otherwise.
    */
    bool RunOneStepOfTheControlLoop(Eigen::VectorXd & CurrentConfiguration,
                                    Eigen::VectorXd & CurrentVelocity,
                                    Eigen::VectorXd & CurrentAcceleration,
                                    Eigen::VectorXd &ZMPTarget,
                                    COMState &COMState,
                                    FootAbsolutePosition &LeftFootPosition,
                                    FootAbsolutePosition &RightFootPosition);

    /*! \brief Run One Step of the global control loop 
      aka The Main Method To Be Used.
      @param[out]  CurrentConfiguration The current configuration of the robot 
      according to
      the implementation of Pinocchio. This should be first position and 
      orientation
      of the waist, and then all the DOFs of your robot.
      @param[out]  CurrentVelocity  The current velocity of the robot according
      to the
      the implementation of Pinocchio.
      @param[out]  CurrentAcceleration  The current acceleration of the robot 
      according to the
      the implementation of Pinocchio.
      @param[out]  ZMPTarget  The target ZMP in the waist reference frame.
      @param[out] aCOMPosition The CoM position for this motion.
      @param[out] LeftFootPosition: Absolute position of the left foot.
      @param[out] RightFootPosition: Absolute position of the right foot.
      @return True is there is still some data to send, false otherwise.
    */
    bool RunOneStepOfTheControlLoop(Eigen::VectorXd & CurrentConfiguration,
                                    Eigen::VectorXd & CurrentVelocity,
                                    Eigen::VectorXd & CurrentAcceleration,
                                    Eigen::VectorXd &ZMPTarget,
                                    COMPosition &aCOMPosition,
                                    FootAbsolutePosition &LeftFootPosition,
                                    FootAbsolutePosition &RightFootPosition);


    /*! \brief Run One Step of the global control loop aka 
      The Main Method To Be Used.
      @param[out] LeftFootPosition: Absolute position of the left foot.
      @param[out] RightFootPosition: Absolute position of the right foot.
      @param[out] ZMPRefPos: ZMP position new reference
      @param[out] COMRefPos: COM position new reference.
      @return True is there is still some data to send, false otherwise.
    */
    bool RunOneStepOfTheControlLoop(FootAbsolutePosition &LeftFootPosition,
                                    FootAbsolutePosition &RightFootPosition,
                                    ZMPPosition &ZMPRefPos,
                                    COMPosition &COMRefPos);
    /*! @} */

    /*! Set the current joint values of the robot.
      This method is used to properly initialize the pattern generator.
      It also updates the state of the robot if other control mechanisms
      modifies the upper body part and if this should be taken into account
      into the pattern generator in the second loop of control. */
    void SetCurrentJointValues(Eigen::VectorXd &lCurrentJointValues);

    /*! \brief Returns the walking mode. */
    int GetWalkMode() const;

    /*! \brief Get the leg joint velocity */
    void GetLegJointVelocity(Eigen::VectorXd &dqr,
                             Eigen::VectorXd &dql) const;

    /*! \brief Read a velocity reference. */
    void setVelReference(istringstream &strm);

    /*! \brief Read a perturbation force on the com. */
    void setCoMPerturbationForce(istringstream &strm);

    /*! \brief Initialize online mode of Herdt. */
    void initOnlineHerdt();

    /*! \brief Initialize online mode of Naveau. */
    void initOnlineNaveau();

    /*! \brief Read a sequence of steps. */
    void ReadSequenceOfSteps(istringstream &strm);

    /*! \name On-line steps related methods
      @{
    */
    /*! \brief Start the creation of steps on line. */
    void StartOnLineStepSequencing();

    /*! \brief Stop the creation of steps on line. */
    void StopOnLineStepSequencing();

    /*! \brief Add an online step */
    void AddOnLineStep(double X, double Y, double Theta);

    /*! \brief Change online step.
      The strategy is the following: the step in single support phase at time t
      has its landing position changed to \f$ (X,Y,\theta) \f$ in absolute
      coordinates (i.e. in the world reference frame of the free flyer 
      of the robot).
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
    int ChangeOnLineStep(double Time,
                         FootAbsolutePosition &aFootAbsolutePosition,
                         double &newtime);

    /*! \brief Change online step.
      See the above method for the specifications.
      This method uses a different format with stream of strings.
      @param[in] Time: Time information of the step.
      @return nothing */
    void ChangeOnLineStep(istringstream &strm,double &newtime);

    /*! @} */

    /*! \name For SLAM
      @{ */
    /*! \brief Update the current waist absolute position */
    void UpdateAbsolutePosition(bool UpdateAbsMotionOrNot);

    /*! \brief Get the waist position and orientation as a quaternion,
      and the planar X-Y orientation in Orientation. */
    void getWaistPositionAndOrientation(double TQ[7],double &Orientation) const;

    /*! \brief Set Waist position and Orientation  */
    void setWaistPositionAndOrientation(double TQ[7]);

    /*! \brief Get Waist velocity */
    void getWaistVelocity(double &dx,
                          double &dy,
                          double &omega) const;

    /*! \brief An other method to get the waist position using a matrix. */
    void getWaistPositionMatrix(Eigen::Matrix4d &lWaistAbsPos) const;

    /*!@} */

    /*! \name Handling of the inter-objects relationships.
      @{
    */

    /*! \brief Instanciate the necessary objects. */
    void ObjectsInstanciation();


    /*! \brief Set the inter object relationship. */
    void InterObjectRelationInitialization();

    /*! @}*/

    /*! \brief Set the initial ZMP reference point. */
    void setZMPInitialPoint(Eigen::Vector3d & lZMPInitialPoint);

    /*! \brief Get the initial ZMP reference point. */
    void getZMPInitialPoint(Eigen::Vector3d & lZMPInitialPoint) const;

    /*! \name System to call a given method based on registration of a method.
      @{
    */

    /*! \brief Parse a command (to be used out of a plugin) and call all
      objects which registered the method. */
    int ParseCmd(std::istringstream &strm);

    /*! \brief This method register a method to a specific object which 
      derivates from SimplePlugin class. */
    bool RegisterMethod(string &MethodName, SimplePlugin *aSP);

    /*! @} */


    /*! \brief Returns the ZMP, CoM, left foot absolute position, and 
      right foot absolute position
      for the initiale pose.*/
    void EvaluateStartingState(COMState  & lStartingCOMState,
                               Eigen::Vector3d & lStartingZMPPosition,
                               Eigen::Matrix<double, 6, 1> & lStartingWaistPose,
                               FootAbsolutePosition & InitLeftFootAbsPos,
                               FootAbsolutePosition & InitRightFootAbsPos);

    /*! @} */

    /*! \brief Set velocity reference
      This method is only supported by Herdt's algorithm.
      Currently only a 3D speed is supported:
      \param x: Velocity along the saggital plane.
      \param y: Velocity along the perpendicular plane.
      \param yaw: Angular velocity in the x-y plane.
    */
    void setVelocityReference(double x,
                              double y,
                              double yaw);


    /*! @} */

    /*! \brief Set additive acceleration to the state
      \param x: Additive acceleration therm along the saggital plane.
      \param y: Additive acceleration therm along the perpendicular plane.
    */
    void setCoMPerturbationForce(double x,
                                 double y);

  protected:

    /*! \name Methods for interpreter.
      @{
    */

    /*! \brief Set the shift of the ZMP height for stepping over. */
    virtual void m_SetZMPShiftParameters(std::istringstream &strm);

    /*! \brief Set time distribution parameters. */
    virtual void m_SetTimeDistrParameters(std::istringstream &strm);

    /*! \brief Set upper body motion parameters. */
    virtual void m_SetUpperBodyMotionParameters(std::istringstream &strm);

    /*! \brief Set the limits of the feasibility (stepping over parameters) */
    virtual void m_SetLimitsFeasibility(std::istringstream &strm);

    /*! \brief Read file from Kineoworks. */
    virtual void m_ReadFileFromKineoWorks(std::istringstream &strm);

    /*! \brief Specify a sequence of step without asking for
      immediate execution and end sequence. */
    virtual void m_PartialStepSequence(istringstream &strm);

    /*! \brief This method set PBW's algorithm for ZMP trajectory planning. */
    virtual void m_SetAlgoForZMPTraj(istringstream &strm);

    /*! \brief Interface to hrpsys to start the realization of
      the stacked of step sequences. */
    virtual void m_FinishAndRealizeStepSequence(std::istringstream &strm);

    /*! \brief Realize a sequence of steps. */
    virtual void m_StepSequence(std::istringstream &strm);

    virtual void m_StepStairSequence(std::istringstream &strm);


  private:

    /*! Object to handle the stack of relative steps. */
    StepStackHandler *m_StepStackHandler;

    /*! Buffer needed to perform the stepping over
      obstacle. */
    std::vector<double> m_ZMPShift;

    /*! Gain factor for the default arm motion while walking. */
    double m_GainFactor;

    /*! Objects to generate a ZMP profile from
      the step of stacks. They provide a buffer for
      the ZMP position to be used every dt
      in the control loop. @{ */

    /*! Kajita's heuristic: the center of the convex hull. */
    ZMPDiscretization * m_ZMPD;

    /*! QP formulation with constraints. */
    ZMPQPWithConstraint * m_ZMPQP;

    /*! QP formulation with constraints. */
    ZMPConstrainedQPFastFormulation * m_ZMPCQPFF;

    /*! QP formulation with a velocity reference. */
    ZMPVelocityReferencedQP * m_ZMPVRQP;

#if USE_QUADPROG==1
    /*! SQP formulation with a velocity reference. */
    ZMPVelocityReferencedSQP * m_ZMPVRSQP;
#endif

    /*! ZMP and CoM trajectories generation from an analytical formulation */
    AnalyticalMorisawaCompact * m_ZMPM;

    /*! Specified ZMP starting point. */
    Eigen::Vector3d m_ZMPInitialPoint;

    /*! Boolean stating if the user has specified or not the ZMP initial point.
      This boolean is set to true when the user is setting the previous value.
      Otherwise it is set to false.*/
    bool m_ZMPInitialPointSet;

    /*@} */

    /*! The Preview Control object. */
    PreviewControl *m_PC;

    /*! The object to be used to perform one step of
      control, and generates the corrected CoM trajectory. */
    ZMPPreviewControlWithMultiBodyZMP *m_ZMPpcwmbz;

    /*! Object needed to perform a path provided by Kineo */
    GenerateMotionFromKineoWorks *m_GMFKW;


    /*! Conversion between the index of the plan and the robot DOFs. */
    std::vector<int> m_ConversionForUpperBodyFromLocalIndexToRobotDOFs;

    /*! Current Actuated Joint values of the robot. */
    std::vector<double> m_CurrentActuatedJointValues;

    /*! Position of the waist:
      Relative.*/
    Eigen::Matrix4d m_WaistRelativePos;

    /*! Absolute: */
    Eigen::Matrix4d m_WaistAbsPos;

    /*! Orientation: */
    double m_AbsTheta, m_AbsMotionTheta;

    /*! Position of the motion: */
    Eigen::Matrix4d m_MotionAbsPos;
    Eigen::Matrix4d m_MotionAbsOrientation;

    /*! Absolute speed:*/
    Eigen::Vector4d m_AbsLinearVelocity;
    Eigen::Vector4d m_AbsAngularVelocity;

    /*! Aboluste acceleration */
    Eigen::Vector4d m_AbsLinearAcc;

    /*! Keeps track of the last correct support foot. */
    int m_KeepLastCorrectSupportFoot;

    /*! Boolean to ensure a correct initialization of the
      step's stack. */
    bool m_IncorrectInitialization;

    /*! \name Global strategy handlers
      @{
    */
    /*! \brief Double stage preview control strategy */
    DoubleStagePreviewControlStrategy * m_DoubleStagePCStrategy;

    /*! \brief Simple strategy just output CoM and Foot position. */
    CoMAndFootOnlyStrategy * m_CoMAndFootOnlyStrategy;

    /*! \brief General handler. */
    GlobalStrategyManager *m_GlobalStrategyManager;

    /*! @} */

    /*! Store the debug mode. */
    int m_DebugMode;

    /*! Store the number of degree of freedoms */
    int m_DOF;

    /*! Store the height of the arm. */
    double m_ZARM;

    /**! \name Time related parameters.
       @{
    */
    /*! \brief Sampling period of the control loop. */
    double m_SamplingPeriod;

    /*! \brief Window of the preview control */
    double m_PreviewControlTime;

    /*! \brief Internal clock.
      This field is updated every call to RunOneStepOfControl.
      It is assumed that this is done every m_SamplingPeriod.
    */
    double m_InternalClock;

    /*! @} */

    /*! Store the local Single support time,
      and the Double support time. */
    double m_TSsupport, m_TDsupport;

    /*! Height of the CoM. */
    double m_Zc;

    /*! Discrete size of the preview control window */
    unsigned int m_NL;

    /*! Local time while performing the control loop. */
    unsigned long int m_count;


    /*! Maximal value for the arms in front of the robot */
    double m_Xmax;

    /*! Variables used to compute speed for other purposes. */
    Eigen::VectorXd m_prev_qr;
    Eigen::VectorXd m_prev_ql;
    Eigen::VectorXd m_prev_dqr;
    Eigen::VectorXd m_prev_dql;


    /* Debug variables. */
    Eigen::VectorXd m_Debug_prev_qr;
    Eigen::VectorXd m_Debug_prev_ql;
    Eigen::VectorXd m_Debug_prev_dqr;
    Eigen::VectorXd m_Debug_prev_dql;
    Eigen::VectorXd m_Debug_prev_UpperBodyAngles;
    Eigen::VectorXd m_Debug_prev_qr_RefState;
    Eigen::VectorXd m_Debug_prev_ql_RefState;

    double m_Debug_prev_qWaistYaw, m_Debug_prev_dqWaistYaw;
    Eigen::Vector3d m_Debug_prev_P, m_Debug_prev_L;
    bool m_FirstPrint, m_FirstRead;

    bool m_ShouldBeRunning;

    bool m_Running;

    bool m_feedBackControl ;



    /*! \name To handle a new step.
      @{
    */

    /*! \brief There is a new user specified step. */
    bool m_NewStep;

    /*! \brief X, Y, Theta coordinates of the new step.*/
    double m_NewStepX, m_NewStepY, m_NewStepZ, m_NewTheta;
    /*! @} */

    /*! \brief Add automatically the first new step */
    bool m_AutoFirstStep;

    /* ! Store the current relative state of the waist */
    COMState m_CurrentWaistState;

    /* ! current time period for the control */
    double m_dt;

    /*! \name Internals to deal with several ZMP CoM generation algorithms
      @{ */
    /*! Algorithm to compute ZMP and CoM trajectory */
    int m_AlgorithmforZMPCOM;

    /*! Constants
      @{ */
    /*! Using Preview Control with 2 stages proposed by Shuuji Kajita in 2003.
     */
    static const int ZMPCOM_KAJITA_2003=1;

    /*! Using the preview control with 2 stages proposed by Pierre-Brice 
      Wieber in 2006. */
    static const int ZMPCOM_WIEBER_2006=2;

    /*! Using the analytical solution proposed by Morisawa in 2007. */
    static const int ZMPCOM_MORISAWA_2007=3;

    /*! Using the QP constrained problem resolution proposed by Dimitrov 
      in 2008. */
    static const int ZMPCOM_DIMITROV_2008=4;

    /*! Using the velocity referenced QP proposed by Herdt in 2010. */
    static const int ZMPCOM_HERDT_2010=5;

    /*! Using the velocity referenced QP proposed by Herdt in 2010. */
    static const int ZMPCOM_NAVEAU_2015=6;
    /*! @} */
    /*! @} */

    /*! Humanoid Dynamic robot */
    PinocchioRobot * m_PinocchioRobot ;
    //CjrlHumanoidDynamicRobot * m_HumanoidDynamicRobot, *
    // m_2HumanoidDynamicRobot;

    /*! Speed of the leg. */
    Eigen::VectorXd m_dqr,m_dql;

    /*! Objet to realize the generate the posture for given CoM
      and feet positions. */
    std::vector<ComAndFootRealization *> m_ComAndFootRealization;

    /* \name Object related to stepping over.
       @{
    */

    /*! Planner for stepping over an obstacle. */
    StepOverPlanner *m_StOvPl;

    /*! Position and parameters related to the obstacle. */
    ObstaclePar m_ObstaclePars;

    /*! Boolean on the obstacle's detection */
    bool m_ObstacleDetected;

    /*! Time Distribution factor */
    std::vector<double> m_TimeDistrFactor;

    /*! Variable for delta feasibility limit */
    double m_DeltaFeasibilityLimit;

    /*! New time for step interval  using Changing On Line Step. */
    double m_NewNextStepInterval;

    /* @} */

    /*! \brief Foot Trajectory Generator */
    LeftAndRightFootTrajectoryGenerationMultiple * m_FeetTrajectoryGenerator;


    /*! \name Buffers of Positions.
      @{
    */

    /*! Buffer of ZMP positions */
    deque<ZMPPosition> m_ZMPPositions;

    /*! Buffer of Absolute foot position (World frame) */
    deque<FootAbsolutePosition> m_FootAbsolutePositions;

    /*! Buffer of absolute foot position. */
    deque<FootAbsolutePosition> m_LeftFootPositions, m_RightFootPositions;

    /*! Buffer for the COM position. */
    deque<COMState> m_COMBuffer;

    /*! @} */

    /*! \brief Reimplement the SimplePlugin interface. */
    virtual void CallMethod(string &MethodName,
                            istringstream &istrm);

    /*! \brief Register the methods handled by the SimplePlugin part of
      this object. */
    void RegisterPluginMethods();

    /*! \brief Start FPE trapping. */
    void AllowFPE();

  protected:

    /*! \name Internal methods which are not to be exposed.
      They are therefore subject to change.
      @{
    */

    /*! \brief Expansion of the buffers handling Center of Masse positions,
      as well as Upper Body Positions. */
    void ExpandCOMPositionsQueues(int aNumber);

    /*! \brief Compute the COM, left and right foot position for a 
      given BodyAngle position */
    void EvaluateStartingCOM(Eigen::VectorXd &Configuration,
                             Eigen::Vector3d &lStartingCOMPosition);


    /*! \brief Fill the internal buffer with the appropriate information 
      depending on the strategy.
      The behavior of this method depends on \a m_AlgorithmforZMPCOM.
    */
    int CreateZMPReferences(deque<RelativeFootPosition> &lRelativeFootPositions,
                            COMState &lStartingCOMState,
                            Eigen::Vector3d & lStartingZMPPosition,
                            FootAbsolutePosition & InitLeftFootAbsPos,
                            FootAbsolutePosition & InitRightFootAbsPos);

    /*! \brief Create automatically a new step for a ZMP based stability 
      criteria */
    void AutomaticallyAddFirstStep(deque<RelativeFootPosition> &
                                   lRelativeFootPositions,
                                   FootAbsolutePosition & InitLeftFootAbsPos,
                                   FootAbsolutePosition & InitRightFootAbsPos,
                                   COMState &lStartingCOMState);
    /* @} */
  };



}


#endif


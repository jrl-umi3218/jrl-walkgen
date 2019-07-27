/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Francois Keith
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
/* \file ComAndFootRealizationByGeometry.h
   \brief Realizes the CoM and Foot position by assuming that the robot
   has 6 DoFs legs. It is then a simple matter of inverse geometry,
   relying on the inverse kinematics object.
*/
#ifndef _COM_AND_FOOT_REALIZATION_BY_GEOMETRY_H_
#define _COM_AND_FOOT_REALIZATION_BY_GEOMETRY_H_


#include <jrl/walkgen/pgtypes.hh>
#include <MotionGeneration/ComAndFootRealization.hh>
#include <MotionGeneration/StepOverPlanner.hh>
#include <MotionGeneration/WaistHeightVariation.hh>
#include <MotionGeneration/UpperBodyMotion.hh>
#include <MotionGeneration/GenerateMotionFromKineoWorks.hh>

namespace PatternGeneratorJRL
{
  /* @ingroup motiongeneration

     This object realizes different kind of motion: stepping over,
     execution of planned trajectory, lowering the waist, but they all
     assume that the upper body position is separated from the legs.
     It also assumes that the robot has 6 DoFs legs.
     It is then a simple matter of inverse geometry,
     relying on the inverse kinematics object.

     The different strategies can by modified by changing the walking mode
     parameter.

  */
  class  ComAndFootRealizationByGeometry: public ComAndFootRealization
  {

  public:

    /*! \name Constructor and destructor */

    /*! Constructor */
    ComAndFootRealizationByGeometry(PatternGeneratorInterfacePrivate *
                                    aPatternGeneratorInterface);
    /*! Destructor */
    ~ComAndFootRealizationByGeometry();
    /** @} */

    /*! Initialization which should be done after setting the 
      HumanoidDynamicRobot member. */
    void Initialization();


    /*! Compute the robot state for a given CoM and feet posture.
      Each posture is given by a 3D position and two Euler angles 
      \f$ (\theta, \omega) \f$.
      Very important: This method is assume to set correctly the body angles of
      its \a HumanoidDynamicRobot and a subsequent call to the ZMP position
      will return the associated ZMP vector.
      @param[in] CoMPosition a 6 dimensional vector with the first 3 dimension 
      for position,
      and the last two for the orientation (Euler angle).
      @param[in] aCoMSpeed a 6 dimensional vector with the first 3 dimension 
      for linear velocity,
      and the last two for the angular velocity.
      @param[in] aCoMAcc a 6 dimensional vector with the first 3 dimension 
      for linear velocity,
      and the last two for the angular velocity.
      @param[in] LeftFoot a 6 dimensional following the same convention than 
      for \a CoMPosition.
      @param[in] RightFoot idem.
      @param[out] CurrentConfiguration The result is a state vector containing
      the position which are put inside this parameter.
      @param[out] CurrentVelocity The result is a state vector containing 
      the speed which are put inside this parameter.
      @param[out] CurrentAcceleration The result is a state vector containing 
      the acceleratio which are put inside this parameter.
      @param[in] IterationNumber Number of iteration.
      @param[in] Stage indicates which stage is reach by the Pattern Generator. 
      If this is the
      last stage, we store some information.

    */
    bool ComputePostureForGivenCoMAndFeetPosture
    (Eigen::VectorXd &CoMPosition,
     Eigen::VectorXd &aCoMSpeed,
     Eigen::VectorXd &aCoMAcc,
     Eigen::VectorXd &LeftFoot,
     Eigen::VectorXd &RightFoot,
     Eigen::VectorXd & CurrentConfiguration,
     Eigen::VectorXd & CurrentVelocity,
     Eigen::VectorXd & CurrentAcceleration,
     unsigned long int IterationNumber,
     int Stage);

    /*! \name Initialization of the walking.
      @{
    */


    /*! \brief Initialize the humanoid model considering the current
      configuration set by the user.
      \param[in] BodyAnglesIni: The configuration vector provided by the user.
      \param[out] lStartingWaistPose: The waist pose according to the user
      configuration vector.
    */
    bool InitializationHumanoid
    (Eigen::VectorXd &BodyAnglesIni,
     Eigen::Matrix<double, 6, 1> &lStartingWaistPose);

    /*! \brief Initialize the foot position.
      \param[in] aFoot: Pointer to the foot to be updated.
      \param[in] m_AnklePosition: Translation from the ankle to the soil.
      \param[out] InitFootPosition: The foot position according to the
      free flyer (set to 0.0 0.0 0.0)
    */
    bool InitializationFoot(PRFoot * aFoot,
                            Eigen::Vector3d &m_AnklePosition,
                            FootAbsolutePosition & InitFootPosition);

    /*! This initialization phase does the following:
      1/ we take the current state of the robot
      to compute the current CoM value.
      2/ We deduce the difference between the CoM and the waist,
      which is suppose to be constant for the all duration of the motion.

      IMPORTANT: The jrlHumanoidDynamicRobot must have been properly set up.

    */
    bool InitializationCoM(Eigen::VectorXd &BodyAnglesIni,
                           Eigen::Vector3d & lStartingCOMPosition,
                           Eigen::Matrix<double,6,1> & lStartingWaistPosition,
                           FootAbsolutePosition & InitLeftFootAbsPos,
                           FootAbsolutePosition & InitRightFootAbsPos);

    /*! This initialization phase, make sure that the needed buffers
      for the upper body motion are correctly setup.
    */
    bool InitializationUpperBody
    (deque<ZMPPosition> &inZMPPositions,
     deque<COMPosition> &inCOMBuffer,
     deque<RelativeFootPosition> lRelativeFootPositions);

    /* @} */

    /*! Evaluate CoM for a given position.
      Assuming that the waist is at (0,0,0)
      It returns the associate initial values for the left and right foot.
    */
    int EvaluateCOMForStartingPosition
    (Eigen::VectorXd &BodyAngles,
     double omega, double theta,
     Eigen::Vector3d &lCOMPosition,
     FootAbsolutePosition & LeftFootPosition,
     FootAbsolutePosition & RightFootPosition);

    /*! Evaluate CoM for a given position.
      Assuming that the waist is at (0,0,0)
      It returns the associate initial values for the left and right foot.*/

    int EvaluateStartingCoM(Eigen::VectorXd &BodyAngles,
                            Eigen::Vector3d & aStartingCOMPosition,
                            FootAbsolutePosition & InitLeftFootPosition,
                            FootAbsolutePosition & InitRightFootPosition);

    int EvaluateStartingCoM(Eigen::VectorXd &BodyAngles,
                            Eigen::Vector3d &aStartingCOMPosition,
                            Eigen::Matrix<double, 6, 1> &aWaistPose,
                            FootAbsolutePosition & InitLeftFootPosition,
                            FootAbsolutePosition & InitRightFootPosition);

    /*! Method to compute the heuristic for the arms. */
    void ComputeUpperBodyHeuristicForNormalWalking
    (Eigen::VectorXd & qArmr,
     Eigen::VectorXd & qArml,
     Eigen::VectorXd & aCOMPosition,
     Eigen::VectorXd & RFP,
     Eigen::VectorXd &  LFP);

    /*! This method returns the final COM pose matrix 
      after the second stage of control. */
    Eigen::MatrixXd GetFinalDesiredCOMPose();

    /*! Returns the position of the Waist in the the COM Frame . */
    void GetCurrentPositionofWaistInCOMFrame(Eigen::VectorXd &
                                             CurPosWICF_homogeneous);


    /*! Reimplementation of the setter of the HumanoidDynamicRobot. */
    bool setPinocchioRobot(PinocchioRobot * aHumanoidDynamicRobot);

    /*! Compute the angles values considering a 6DOF leg for a given 
      configuration
      of the waist and the foot of the leg:
      @param[in] Body_R : orientation of the Waist.
      @param[in] Body_P: position of the waist.
      @param[in] aFoot: A vector giving the foot configuration 
      (x,y,z, theta, omega).
      @param[in] lDt: Vector describing the hip configuration.
      @param[in] aCoMPosition: Position of the CoM.
      @param[in] ToTheHip: Vector to go from the Waist to the Hip.
      @param[in] LeftOrRight: -1 for the right leg, 1 for the left.
      @param[out] lq : Values of the leg which realize the position asked for.
    */
    bool KinematicsForOneLeg(Eigen::Matrix3d & Body_R,
                             Eigen::Vector3d & Body_P,
                             Eigen::VectorXd &aFoot,
                             Eigen::Vector3d &lDt,
                             Eigen::VectorXd &aCoMPosition,
                             Eigen::Vector3d &ToTheHip,
                             int LeftOrRight,
                             Eigen::VectorXd &lq,
                             int Stage);

    /*! Compute the angles values considering two 6DOF legs for a given 
      configuration
      of the waist and of the feet:
      @param aCoMPosition: Position of the CoM (x,y,z,theta, omega, phi).
      @param aLeftFoot: Position of the foot (x,y,z, theta, omega).
      @param aRightFoot: Position of the foot (x,y,z,theta,omega).
      @param Stage: Stage of the ZMP Preview algorithm.
      @param ql: Angles for the left leg to achieve the positions.
      @param qr: Angles for the right leg to achieve the positions.
      @param AbsoluteWaistPosition: The waist position.
    */
    bool KinematicsForTheLegs(Eigen::VectorXd & aCoMPosition,
                              Eigen::VectorXd & aLeftFoot,
                              Eigen::VectorXd & aRightFoot,
                              int Stage,
                              Eigen::VectorXd & ql,
                              Eigen::VectorXd & qr,
                              Eigen::Vector3d & AbsoluteWaistPosition);

    /*! \brief Implement the Plugin part to receive information from
      PatternGeneratorInterface.
    */
    void CallMethod(string &Method, istringstream &istrm);

    /*! Get the current position of the waist in the COM reference frame
      @return a 4x4 matrix which contains the pose and the position of the waist
      in the CoM reference frame.
    */
    Eigen::Matrix4d GetCurrentPositionofWaistInCOMFrame();

    /*! \brief Getter and setter for the previous 
      configurations and velocities */
    inline void SetPreviousConfigurationStage0
    (Eigen::VectorXd & prev_Configuration)
    {
      m_prev_Configuration = prev_Configuration ;
    }
    inline void SetPreviousVelocityStage0(Eigen::VectorXd & prev_Velocity)
    {
      m_prev_Velocity = prev_Velocity ;
    }

    inline void SetPreviousConfigurationStage1
    (Eigen::VectorXd & prev_Configuration)
    {
      m_prev_Configuration1 = prev_Configuration ;
    }
    inline void SetPreviousVelocityStage1(Eigen::VectorXd & prev_Velocity)
    {
      m_prev_Velocity1 = prev_Velocity ;
    }

    inline void SetPreviousConfigurationStage2
    (Eigen::VectorXd & prev_Configuration)
    {
      m_prev_Configuration2 = prev_Configuration ;
    }
    inline void SetPreviousVelocityStage2(Eigen::VectorXd & prev_Velocity)
    {
      m_prev_Velocity2 = prev_Velocity ;
    }

    /*! \brief Getter and setter for the 
      previous configurations and velocities */
    inline void SetPreviousConfigurationStage0(const Eigen::VectorXd &
                                               prev_Configuration)
    {
      m_prev_Configuration = prev_Configuration ;
    }
    inline void SetPreviousVelocityStage0(const Eigen::VectorXd & prev_Velocity)
    {
      m_prev_Velocity = prev_Velocity ;
    }

    inline void SetPreviousConfigurationStage1(const Eigen::VectorXd &
                                               prev_Configuration)
    {
      m_prev_Configuration1 = prev_Configuration ;
    }
    inline void SetPreviousVelocityStage1(const Eigen::VectorXd & prev_Velocity)
    {
      m_prev_Velocity1 = prev_Velocity ;
    }

    inline void SetPreviousConfigurationStage2(const Eigen::VectorXd &
                                               prev_Configuration)
    {
      m_prev_Configuration2 = prev_Configuration ;
    }
    inline void SetPreviousVelocityStage2(const Eigen::VectorXd & prev_Velocity)
    {
      m_prev_Velocity2 = prev_Velocity;
    }

    /*! \brief Getter and setter for the previous 
      configurations and velocities */
    inline Eigen::VectorXd & GetPreviousConfigurationStage0()
    {
      return m_prev_Configuration ;
    };

    inline Eigen::VectorXd & GetPreviousConfigurationStage1()
    {
      return m_prev_Configuration1 ;
    };

    inline Eigen::VectorXd & GetPreviousVelocityStage0()
    {
      return m_prev_Velocity ;
    };

    inline Eigen::VectorXd & GetPreviousVelocityStage1()
    {
      return m_prev_Velocity1 ;
    };

    inline void leftLegIndexinConfiguration
    (std::vector<int> & leftLegMaps) const
    {
      leftLegMaps = m_LeftLegIndexinConfiguration ;
    }
    inline void rightLegIndexinConfiguration
    (std::vector<int> & rightLegMaps) const
    {
      rightLegMaps = m_RightLegIndexinConfiguration ;
    }
    inline void leftArmIndexinConfiguration
    (std::vector<int> & leftArmMaps) const
    {
      leftArmMaps = m_LeftArmIndexinConfiguration ;
    }
    inline void rightArmIndexinConfiguration
    (std::vector<int> & rightArmMaps) const
    {
      rightArmMaps = m_RightArmIndexinConfiguration ;
    }
    inline void chestIndexinConfiguration(std::vector<int> & chestMaps) const
    {
      chestMaps = m_ChestIndexinConfiguration ;
    }

    inline void leftLegIndexinVelocity(std::vector<int> & leftLegMaps) const
    {
      leftLegMaps = m_LeftLegIndexinConfiguration ;
    }
    inline void rightLegIndexinVelocity(std::vector<int> & rightLegMaps) const
    {
      rightLegMaps = m_RightLegIndexinConfiguration ;
    }
    inline void leftArmIndexinVelocity(std::vector<int> & leftArmMaps) const
    {
      leftArmMaps = m_LeftArmIndexinConfiguration ;
    }
    inline void rightArmIndexinVelocity(std::vector<int> & rightArmMaps) const
    {
      rightArmMaps = m_RightArmIndexinConfiguration ;
    }
    inline void chestIndexinVelocity(std::vector<int> & chestMaps) const
    {
      chestMaps = m_ChestIndexinConfiguration ;
    }

    inline bool ShiftFoot()
    {
      return ShiftFoot_ ;
    }
    inline void ShiftFoot(bool ShiftFoot)
    {
      ShiftFoot_=ShiftFoot ;
    }

    /*! \brief Get the COG of the ankles at the starting position. */
    virtual Eigen::Vector3d GetCOGInitialAnkles();

    friend ostream& operator <<(ostream &os,const ComAndFootRealization &obj);

  protected:

    /*! \brief Initialization of internal maps of indexes */
    void InitializationMaps(std::vector<pinocchio::JointIndex> &FromRootToFoot,
                            pinocchio::JointModelVector & ActuatedJoints,
                            std::vector<int> &IndexinConfiguration,
                            std::vector<int> &IndexinVelocity);

    /*! Map shoulders and wrist
      \param[in] aHand: The hand to be used for extraction of data.
      \param[in] ActuatedJoints: The vector of actuated joints.
      \param[out] IndexesInVRML: The kinematic chain from the shoulder
      to the hand given with the VRML indexes.
      \param[out] IndexesInConfiguration: The kinematic chain
      from the shoulder given with the depth-first ordering.
      \param[out] associateShoulder: The shoulder extracted from
      the kinematic chain.
    */
    void InitializeMapsForAHand(pinocchio::JointIndex aWrist,
                                pinocchio::JointModelVector &  ActuatedJoints,
                                vector<int> & IndexesInConfiguration,
                                vector<int> & IndexesInVelocity,
                                pinocchio::JointIndex & associateShoulder);

    /*! Create the map of indexes for the shoulders and wrist */
    void InitializeMapForChest(pinocchio::JointModelVector & ActuatedJoints);

    /* Register methods. */
    void RegisterMethods();

  private:

    /*! \name Objects for stepping over.
      @{
    */

    /*! Planner for the waist variation for stepping
      over an obstacle. */
    WaistHeightVariation *m_WaistPlanner;

    /*! Planner for the upper body motion. */
    UpperBodyMotion * m_UpBody;

    /* @} */

    /*! Pointer related to Kineoworks planner. */
    GenerateMotionFromKineoWorks * m_GMFKW;


    /*! \brief Displacement between the hip and the foot. @{*/
    /*! \brief For the right foot. */
    Eigen::Vector3d m_DtRight;
    /*! \brief For the left foot. */
    Eigen::Vector3d m_DtLeft;
    /*! @} */

    /*! \name Vector from the Waist to the left and right hip. */

    /*! Dynamic part from the waist to the left hip. */
    Eigen::Vector3d m_TranslationToTheLeftHip;
    /*! Dynamic part form the waist to the right hip. */
    Eigen::Vector3d m_TranslationToTheRightHip;


    /*! @} */

    /*! \name Previous joint values. */
    //@{
    /*! \brief For the speed (stage 0). */
    Eigen::VectorXd m_prev_Configuration;

    /*! \brief For the speed (stage 1). */
    Eigen::VectorXd m_prev_Configuration1;

    /*! \brief For the speed (stage 1). */
    Eigen::VectorXd m_prev_Configuration2;

    /*! \brief For the speed (stage 0). */
    Eigen::VectorXd m_prev_Velocity;

    /*! \brief For the speed (stage 1). */
    Eigen::VectorXd m_prev_Velocity1;

    /*! \brief For the speed (stage 1). */
    Eigen::VectorXd m_prev_Velocity2;

    //@}

    /*! COM Starting position. */
    Eigen::Vector3d m_StartingCOMPosition;

    /*! Final COM pose. */
    Eigen::Matrix4d m_FinalDesiredCOMPose;

    /*! Store the position of the ankle in the right feet. */
    Eigen::Vector3d m_AnklePositionRight;

    /*! Store the position of the ankle in the left feet. */
    Eigen::Vector3d m_AnklePositionLeft;

    /*! Difference between the CoM and the Waist
      from the initialization phase,
      i.e. not reevaluated while walking. */
    Eigen::Vector3d m_DiffBetweenComAndWaist;

    /*! Difference between the CoM and the Waist
      in the CoM reference frame. */
    Eigen::Vector3d m_ComAndWaistInRefFrame;


    /*! Maximal distance along the X axis for the hand motion */
    double m_Xmax;

    /*! Maximal length of the arm */
    double m_ZARM;

    /*! Buffer of upper body position related to a plan */
    deque<PatternGeneratorJRL::KWNode > m_UpperBodyPositionsBuffer;

    /*! Conversion between the index of the plan and the robot DOFs. */
    std::vector<int> m_ConversionForUpperBodyFromLocalIndexToRobotDOFs;

    /*! \name Keep the indexes into the Configuration numbering system.
      @{
    */
    /*! \brief For the left leg, Specific for the Inverse Kinematics. */
    std::vector<int> m_LeftLegIndexinConfiguration ;
    /*! \brief For the right leg, Specific for the Inverse Kinematics. */
    std::vector<int> m_RightLegIndexinConfiguration ;
    /*! \brief For the left arm, Specific for the Inverse Kinematics. */
    std::vector<int> m_LeftArmIndexinConfiguration ;
    /*! \brief For the right leg, Specific for the Inverse Kinematics. */
    std::vector<int> m_RightArmIndexinConfiguration ;

    /*! \brief For the left leg, Specific for the Inverse Kinematics. */
    std::vector<int> m_LeftLegIndexinVelocity ;
    /*! \brief For the right leg, Specific for the Inverse Kinematics. */
    std::vector<int> m_RightLegIndexinVelocity ;
    /*! \brief For the left arm, Specific for the Inverse Kinematics. */
    std::vector<int> m_LeftArmIndexinVelocity ;
    /*! \brief For the right leg, Specific for the Inverse Kinematics. */
    std::vector<int> m_RightArmIndexinVelocity ;

    /*! \brief For the chest. */
    std::vector<int> m_ChestIndexinConfiguration;
    /*! \brief For the chest. */
    std::vector<int> m_ChestIndexinVelocity;

    /*! \brief For the entire system. */
    std::vector<int> m_GlobalVRMLIDtoConfiguration;

    /*! Gain factor for the arm motion heuristic. */
    double m_GainFactor;

    /*! Buffer of current Upper Body motion. */
    std::vector<double> m_UpperBodyMotion;

    /*! COG of the ankles in the waist reference frame
      when evaluating the initial position.
    */
    Eigen::Vector3d m_COGInitialAnkles;

    /*! Store the position of the left and right shoulders. */
    pinocchio::JointIndex m_LeftShoulder, m_RightShoulder;

    bool ShiftFoot_ ;
  };

  ostream & operator <<(ostream &os,const ComAndFootRealization &obj);

}
#endif

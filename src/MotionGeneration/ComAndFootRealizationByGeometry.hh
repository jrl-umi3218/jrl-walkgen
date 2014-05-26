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
    ComAndFootRealizationByGeometry(PatternGeneratorInterfacePrivate * aPatternGeneratorInterface);
    /*! Destructor */
    ~ComAndFootRealizationByGeometry();
    /** @} */

    /*! Initialization which should be done after setting the HumanoidDynamicRobot member. */
    void Initialization();

    
    /*! Compute the robot state for a given CoM and feet posture.
      Each posture is given by a 3D position and two Euler angles \f$ (\theta, \omega) \f$.
      Very important: This method is assume to set correctly the body angles of
      its \a HumanoidDynamicRobot and a subsequent call to the ZMP position 
      will return the associated ZMP vector.
      @param[in] CoMPosition a 6 dimensional vector with the first 3 dimension for position,
      and the last two for the orientation (Euler angle).
      @param[in] aCoMSpeed a 6 dimensional vector with the first 3 dimension for linear velocity,
      and the last two for the angular velocity.
      @param[in] aCoMAcc a 6 dimensional vector with the first 3 dimension for linear velocity,
      and the last two for the angular velocity.
      @param[in] LeftFoot a 6 dimensional following the same convention than for \a CoMPosition.
      @param[in] RightFoot idem.
      @param[out] CurrentConfiguration The result is a state vector containing 
      the position which are put inside this parameter.
      @param[out] CurrentVelocity The result is a state vector containing the speed which are put inside this parameter.
      @param[out] CurrentAcceleration The result is a state vector containing the acceleratio which are put inside this parameter.
      @param[in] IterationNumber Number of iteration.
      @param[in] Stage indicates which stage is reach by the Pattern Generator. If this is the 
      last stage, we store some information.
      
    */
    bool ComputePostureForGivenCoMAndFeetPosture(MAL_VECTOR_TYPE(double) &CoMPosition,
						 MAL_VECTOR_TYPE(double) & aCoMSpeed,
						 MAL_VECTOR_TYPE(double) & aCoMAcc,
						 MAL_VECTOR_TYPE(double) &LeftFoot,
						 MAL_VECTOR_TYPE(double) &RightFoot,
						 MAL_VECTOR_TYPE(double) & CurrentConfiguration,
						 MAL_VECTOR_TYPE(double) & CurrentVelocity,
						 MAL_VECTOR_TYPE(double) & CurrentAcceleration,
						 int IterationNumber,
						 int Stage);    

    /*! \name Initialization of the walking. 
      @{
     */
    
    
    /*! \brief Initialize the humanoid model considering the current
      configuration set by the user. 
      \param[in] BodyAnglesIni: The configuration vector provided by the user. 
      \param[out] lStartingWaistPose: The waist pose according to the user configuration vector.
    */
    bool InitializationHumanoid(MAL_VECTOR_TYPE(double) &BodyAnglesIni,
			    MAL_VECTOR_TYPE(double) &lStartingWaistPose);

    /*! \brief Initialize the foot position.
      \param[in] aFoot: Pointer to the foot to be updated.
      \param[in] m_AnklePosition: Translation from the ankle to the soil.
      \param[out] InitFootPosition: The foot position according to the 
      free flyer (set to 0.0 0.0 0.0)
    */
    bool InitializationFoot(CjrlFoot * aFoot,
			    MAL_S3_VECTOR(& m_AnklePosition,double),
			    FootAbsolutePosition & InitFootPosition);

    /*! This initialization phase does the following:
      1/ we take the current state of the robot
      to compute the current CoM value.
      2/ We deduce the difference between the CoM and the waist,
      which is suppose to be constant for the all duration of the motion. 

      IMPORTANT: The jrlHumanoidDynamicRobot must have been properly set up.
      
    */
    bool InitializationCoM(MAL_VECTOR_TYPE(double) &BodyAnglesIni,
			   MAL_S3_VECTOR_TYPE(double) & lStartingCOMPosition,
			   MAL_VECTOR_TYPE(double) & lStartingWaistPosition,
			   FootAbsolutePosition & InitLeftFootAbsPos, 
			   FootAbsolutePosition & InitRightFootAbsPos);

    /*! This initialization phase, make sure that the needed buffers
      for the upper body motion are correctly setup.
    */
    bool InitializationUpperBody(deque<ZMPPosition> &inZMPPositions,
				 deque<COMPosition> &inCOMBuffer,
				 deque<RelativeFootPosition> lRelativeFootPositions);

    /* @} */
    



    /*! Evaluate CoM for a given position.
      Assuming that the waist is at (0,0,0)
      It returns the associate initial values for the left and right foot.
    */
    int EvaluateCOMForStartingPosition(MAL_VECTOR( &BodyAngles,double),
				       double omega, double theta,
				       MAL_S3_VECTOR( &lCOMPosition,double),
				       FootAbsolutePosition & LeftFootPosition,
				       FootAbsolutePosition & RightFootPosition);
    
    /*! Evaluate CoM for a given position.
      Assuming that the waist is at (0,0,0)
      It returns the associate initial values for the left and right foot.*/

    int EvaluateStartingCoM(MAL_VECTOR_TYPE(double) &BodyAngles,
			    MAL_S3_VECTOR_TYPE(double) & aStartingCOMPosition,
			    FootAbsolutePosition & InitLeftFootPosition,
			    FootAbsolutePosition & InitRightFootPosition);

    int EvaluateStartingCoM(MAL_VECTOR(&BodyAngles,double),
			    MAL_S3_VECTOR(&aStartingCOMPosition,double),
			    MAL_VECTOR(&aWaistPose,double),
			    FootAbsolutePosition & InitLeftFootPosition,
			    FootAbsolutePosition & InitRightFootPosition);

    /*! Method to compute the heuristic for the arms. */
    void ComputeUpperBodyHeuristicForNormalWalking(MAL_VECTOR_TYPE(double) & qArmr, 
						   MAL_VECTOR_TYPE(double) & qArml,
						   MAL_VECTOR_TYPE(double) & aCOMPosition,
						   MAL_VECTOR_TYPE(double) & RFP,
						   MAL_VECTOR_TYPE(double) &  LFP);

    /*! This method returns the final COM pose matrix after the second stage of control. */
    MAL_MATRIX_TYPE(double) GetFinalDesiredCOMPose();

    /*! Returns the position of the Waist in the the COM Frame . */
    void GetCurrentPositionofWaistInCOMFrame(MAL_VECTOR_TYPE(double) & CurPosWICF_homogeneous);


    /*! Reimplementation of the setter of the HumanoidDynamicRobot. */
    bool setHumanoidDynamicRobot(CjrlHumanoidDynamicRobot * aHumanoidDynamicRobot);

    /*! Compute the angles values considering a 6DOF leg for a given configuration
      of the waist and the foot of the leg:
      @param[in] Body_R : orientation of the Waist.
      @param[in] Body_P: position of the waist.
      @param[in] aFoot: A vector giving the foot configuration (x,y,z, theta, omega).
      @param[in] lDt: Vector describing the hip configuration.
      @param[in] aCoMPosition: Position of the CoM.
      @param[in] ToTheHip: Vector to go from the Waist to the Hip.
      @param[in] LeftOrRight: -1 for the right leg, 1 for the left.
      @param[out] lq : Values of the leg which realize the position asked for.
     */
    bool KinematicsForOneLeg(MAL_S3x3_MATRIX_TYPE(double) & Body_R,
			     MAL_S3_VECTOR_TYPE(double) & Body_P,
			     MAL_VECTOR_TYPE(double) &aFoot,
			     MAL_S3_VECTOR_TYPE(double) &lDt,
			     MAL_VECTOR_TYPE(double) &aCoMPosition,
			     MAL_S3_VECTOR_TYPE(double) &ToTheHip,
			     int LeftOrRight,
			     MAL_VECTOR_TYPE(double) &lq,
			     int Stage);
    
    /*! Compute the angles values considering two 6DOF legs for a given configuration
      of the waist and of the feet:
      @param aCoMPosition: Position of the CoM (x,y,z,theta, omega, phi).
      @param aLeftFoot: Position of the foot (x,y,z, theta, omega).
      @param aRightFoot: Position of the foot (x,y,z,theta,omega).
      @param Stage: Stage of the ZMP Preview algorithm.
      @param ql: Angles for the left leg to achieve the positions.
      @param qr: Angles for the right leg to achieve the positions.
      @param AbsoluteWaistPosition: The waist position.
     */
    bool KinematicsForTheLegs(MAL_VECTOR_TYPE(double) & aCoMPosition,
			      MAL_VECTOR_TYPE(double) & aLeftFoot,
			      MAL_VECTOR_TYPE(double) & aRightFoot,
			      int Stage,
			      MAL_VECTOR_TYPE(double) & ql,
			      MAL_VECTOR_TYPE(double) & qr,
			      MAL_S3_VECTOR_TYPE(double) & AbsoluteWaistPosition);

    /*! \brief Implement the Plugin part to receive information from 
      PatternGeneratorInterface.
     */
    void CallMethod(string &Method, istringstream &istrm);

    /*! Get the current position of the waist in the COM reference frame 
      @return a 4x4 matrix which contains the pose and the position of the waist
      in the CoM reference frame.
    */
    MAL_S4x4_MATRIX_TYPE(double) GetCurrentPositionofWaistInCOMFrame();

    /*! \brief Get the COG of the ankles at the starting position. */
    virtual MAL_S3_VECTOR_TYPE(double) GetCOGInitialAnkles();

  protected:
    
    /*! \brief Initialization of internal maps of indexes */
    void InitializationMaps(std::vector<CjrlJoint *> &FromRootToFoot,
			    std::vector<CjrlJoint *> &ActuatedJoints,
			    std::vector<int> &IndexinConfiguration);

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
    void InitializeMapsForAHand(CjrlHand * aHand,
				std::vector<CjrlJoint *> &ActuatedJoints,
				vector<int> & IndexesInConfiguration,
				CjrlJoint * & associateShoulder);

    /*! Create the map of indexes for the shoulders and wrist */
    void InitializeMapForChest(std::vector<CjrlJoint *> &ActuatedJoints);

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
    MAL_S3_VECTOR(m_DtRight,double);
    /*! \brief For the left foot. */
    MAL_S3_VECTOR(m_DtLeft,double);
    /*! @} */
    
    /*! \name Vector from the Waist to the left and right hip. */

    /*! Static part from the waist to the left hip.. */
    MAL_S3_VECTOR(m_StaticToTheLeftHip,double);
    /*! Static part from the waist to the right hip. */
    MAL_S3_VECTOR(m_StaticToTheRightHip,double);
    /*! Dynamic part from the waist to the left hip. */
    MAL_S3_VECTOR(m_TranslationToTheLeftHip,double);
    /*! Dynamic part form the waist to the right hip. */
    MAL_S3_VECTOR( m_TranslationToTheRightHip,double);
    

    /*! @} */
    
    /*! \name Previous joint values. */
    //@{ 
    /*! \brief For the speed (stage 0). */
    MAL_VECTOR(m_prev_Configuration,double);

    /*! \brief For the speed (stage 1). */
    MAL_VECTOR( m_prev_Configuration1,double);

    /*! \brief For the speed (stage 0). */
    MAL_VECTOR(m_prev_Velocity,double);

    /*! \brief For the speed (stage 1). */
    MAL_VECTOR( m_prev_Velocity1,double);

    //@}
    
    /*! COM Starting position. */
    MAL_S3_VECTOR(m_StartingCOMPosition,double);
    
    /*! Final COM pose. */
    MAL_S4x4_MATRIX(m_FinalDesiredCOMPose,double);
      
    /*! Store the position of the ankle in the right feet. */
    MAL_S3_VECTOR(m_AnklePositionRight,double);

    /*! Store the position of the ankle in the left feet. */
    MAL_S3_VECTOR(m_AnklePositionLeft,double);
        
    /*! Difference between the CoM and the Waist 
      from the initialization phase,
      i.e. not reevaluated while walking. */
    MAL_S3_VECTOR(m_DiffBetweenComAndWaist,double);

    /*! Difference between the CoM and the Waist 
      in the CoM reference frame. */
    MAL_S3_VECTOR(m_ComAndWaistInRefFrame,double);

    
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
    std::vector<int> m_LeftLegIndexinConfiguration;
    /*! \brief For the right leg, Specific for the Inverse Kinematics. */
    std::vector<int> m_RightLegIndexinConfiguration;
    /*! \brief For the left arm, Specific for the Inverse Kinematics. */
    std::vector<int> m_LeftArmIndexinConfiguration;
    /*! \brief For the right leg, Specific for the Inverse Kinematics. */
    std::vector<int> m_RightArmIndexinConfiguration;
    
    /*! \brief For the chest. */
    std::vector<int> m_ChestIndexinConfiguration;

    /*! \brief For the entire system. */
    std::vector<int> m_GlobalVRMLIDtoConfiguration;

    /*! Gain factor for the arm motion heuristic. */
    double m_GainFactor;

    /*! Buffer of current Upper Body motion. */
    std::vector<double> m_UpperBodyMotion;

    /*! COG of the ankles in the waist reference frame 
      when evaluating the initial position.
     */
    MAL_S3_VECTOR_TYPE(double) m_COGInitialAnkles;

    /*! Store the position of the left and right shoulders. */
    CjrlJoint *m_LeftShoulder, *m_RightShoulder;
  };


}
#endif 

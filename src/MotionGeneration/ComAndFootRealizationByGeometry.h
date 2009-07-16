/* \file ComAndFootRealizationByGeometry.h
   \brief Realizes the CoM and Foot position by assuming that the robot
   has 6 DoFs legs. It is then a simple matter of inverse geometry,
   relying on the inverse kinematics object.
  
   Copyright (c) 2005-2006, 
   @author Francois Keith, Olivier Stasse.
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see License.txt for further information on license.   
*/
#ifndef _COM_AND_FOOT_REALIZATION_BY_GEOMETRY_H_
#define _COM_AND_FOOT_REALIZATION_BY_GEOMETRY_H_


#include <PGTypes.h>
#include <MotionGeneration/ComAndFootRealization.h> 
#include <MotionGeneration/StepOverPlanner.h>
#include <MotionGeneration/WaistHeightVariation.h>
#include <MotionGeneration/UpperBodyMotion.h>
#include <MotionGeneration/GenerateMotionFromKineoWorks.h>

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
    bool ComputePostureForGivenCoMAndFeetPosture(MAL_VECTOR(,double) &CoMPosition,
						 MAL_VECTOR(,double) & aCoMSpeed,
						 MAL_VECTOR(,double) & aCoMAcc,
						 MAL_VECTOR(,double) &LeftFoot,
						 MAL_VECTOR(,double) &RightFoot,
						 MAL_VECTOR(,double) & CurrentConfiguration,
						 MAL_VECTOR(,double) & CurrentVelocity,
						 MAL_VECTOR(,double) & CurrentAcceleration,
						 int IterationNumber,
						 int Stage);    

    /*! \name Initialization of the walking. 
      @{
     */
    

    /*! This initialization phase does the following:
      1/ we take the current state of the robot
      to compute the current CoM value.
      2/ We deduce the difference between the CoM and the waist,
      which is suppose to be constant for the all duration of the motion. 

      IMPORTANT: The jrlHumanoidDynamicRobot must have been properly set up.
      
    */
    bool InitializationCoM(MAL_VECTOR(,double) &BodyAnglesIni,
			   MAL_S3_VECTOR(,double) & lStartingCOMPosition,
			   MAL_VECTOR(,double) & lStartingWaistPosition,
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

    int EvaluateStartingCoM(MAL_VECTOR(,double) &BodyAngles,
			    MAL_S3_VECTOR(,double) & aStartingCOMPosition,
			    FootAbsolutePosition & InitLeftFootPosition,
			    FootAbsolutePosition & InitRightFootPosition);

    int EvaluateStartingCoM(MAL_VECTOR(&BodyAngles,double),
			    MAL_S3_VECTOR(&aStartingCOMPosition,double),
			    MAL_VECTOR(&aWaistPose,double),
			    FootAbsolutePosition & InitLeftFootPosition,
			    FootAbsolutePosition & InitRightFootPosition);

    /*! Method to compute the heuristic for the arms. */
    void ComputeUpperBodyHeuristicForNormalWalking(MAL_VECTOR(,double) & qArmr, 
						   MAL_VECTOR(,double) & qArml,
						   MAL_VECTOR(,double) & aCOMPosition,
						   MAL_VECTOR(,double) & RFP,
						   MAL_VECTOR(,double) &  LFP);

    /*! This method returns the final COM pose matrix after the second stage of control. */
    MAL_MATRIX(,double) GetFinalDesiredCOMPose();

    /*! Returns the position of the Waist in the the COM Frame . */
    void GetCurrentPositionofWaistInCOMFrame(MAL_VECTOR(,double) & CurPosWICF_homogeneous);


    /*! Reimplementation of the setter of the HumanoidDynamicRobot. */
    bool setHumanoidDynamicRobot(const CjrlHumanoidDynamicRobot * aHumanoidDynamicRobot);

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
    bool KinematicsForOneLeg(MAL_S3x3_MATRIX(,double) & Body_R,
			     MAL_S3_VECTOR(,double) & Body_P,
			     MAL_VECTOR(,double) &aFoot,
			     MAL_S3_VECTOR(,double) &lDt,
			     MAL_VECTOR(,double) &aCoMPosition,
			     MAL_S3_VECTOR(,double) &ToTheHip,
			     int LeftOrRight,
			     MAL_VECTOR(,double) &lq );
    
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
    bool KinematicsForTheLegs(MAL_VECTOR(,double) & aCoMPosition,
			      MAL_VECTOR(,double) & aLeftFoot,
			      MAL_VECTOR(,double) & aRightFoot,
			      int Stage,
			      MAL_VECTOR(,double) & ql,
			      MAL_VECTOR(,double) & qr,
			      MAL_S3_VECTOR(,double) & AbsoluteWaistPosition);

    /*! \brief Implement the Plugin part to receive information from 
      PatternGeneratorInterface.
     */
    void CallMethod(string &Method, istringstream &istrm);

    /*! Get the current position of the waist in the COM reference frame 
      @return a 4x4 matrix which contains the pose and the position of the waist
      in the CoM reference frame.
    */
    MAL_S4x4_MATRIX(,double) GetCurrentPositionofWaistInCOMFrame();

    /*! \brief Get the COG of the ankles at the starting position. */
    virtual MAL_S3_VECTOR(,double) GetCOGInitialAnkles();

  protected:
    
    /*! Initialization of internal maps */
    void InitializationMaps(std::vector<CjrlJoint *> &FromRootToFoot,
			    std::vector<CjrlJoint *> &ActuatedJoints,
			    std::vector<int> &IndexInVRML,
			    std::vector<int> &IndexinConfiguration);


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
        
    /*! Keep the indexes for the legs of the robot in the VRML numbering system. */
    std::vector<int> m_LeftLegIndexInVRML;
    std::vector<int> m_RightLegIndexInVRML;
    std::vector<int> m_LeftArmIndexInVRML;
    std::vector<int> m_RightArmIndexInVRML;
    std::vector<int> m_ChestIndexInVRML;

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
    MAL_S3_VECTOR(,double) m_COGInitialAnkles;
  };


};
#endif 

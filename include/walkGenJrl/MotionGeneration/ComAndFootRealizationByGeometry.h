/* Realizes the CoM and Foot position by assuming that the robot
   has 6 DoFs legs. It is then a simple matter of inverse geometry,
   relying on the inverse kinematics object.
  
   Copyright (c) 2005-2006, 
   @author Olivier Stasse.
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef _COM_AND_FOOT_REALIZATION_BY_GEOMETRY_H_
#define _COM_AND_FOOT_REALIZATION_BY_GEOMETRY_H_

#include <PGTypes.h>
#include <MotionGeneration/InverseKinematics.h>
#include <MotionGeneration/ComAndFootRealization.h> 
#include <MotionGeneration/StepOverPlanner.h>
#include <MotionGeneration/WaistHeightVariation.h>
#include <MotionGeneration/UpperBodyMotion.h>
#include <MotionGeneration/GenerateMotionFromKineoWorks.h>

namespace PatternGeneratorJRL
{
  /* @ingroup pgjrl
     This object realizes different kind of motion: stepping over, 
     execution of planned trajectory, lowering the waist, but they all
     assume that the upper body position is separated from the legs.
     It also assumes that the robot has 6 DoFs legs. 
     It is then a simple matter of inverse geometry,
     relying on the inverse kinematics object.
     
     The different strategies can by modified by changing the walking mode
     parameter.
     
   */
  class ComAndFootRealizationByGeometry: public ComAndFootRealization
  {
    
  public:

    /*! \name Constructor and destructor */

    /*! Constructor */
    ComAndFootRealizationByGeometry();
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
      @param CoMPosition: a 5 dimensional vector with the first dimension for position,
      and the last two for the orientation (Euler angle).
      @param LeftFoot: a 5 dimensional following the same convention than for \a CoMPosition.
      @param RightFoot: idem.
      @param Stage: indicates which stage is reach by the Pattern Generator. If this is the 
      last stage, we store some information.
      
    */
    bool ComputePostureForGivenCoMAndFeetPosture(MAL_VECTOR(,double) &CoMPosition,
						 MAL_VECTOR(,double) & aCoMSpeed,
						 MAL_VECTOR(,double) &LeftFoot,
						 MAL_VECTOR(,double) &RightFoot,
						 MAL_VECTOR(,double) & CurrentConfiguration,
						 MAL_VECTOR(,double) & CurrentVelocity,
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
			   FootAbsolutePosition & InitLeftFootAbsPos, 
			   FootAbsolutePosition & InitRightFootAbsPos);

    /*! This initialization phase, make sure that the needed buffers
      for the upper body motion are correctly setup.
    */
    bool InitializationUpperBody(deque<ZMPPosition> &inZMPPositions,
				 deque<COMPosition> &inCOMBuffer,
				 deque<RelativeFootPosition> lRelativeFootPositions);

    /* @} */
    

    /*! \name Setter and getter on Inverse Kinematics 
      @{
     */
    /*! Set the pointer to the inverse kinematics.*/
    inline bool setInverseKinematics ( InverseKinematics * anInverseKinematics)
      { m_InverseKinematics = anInverseKinematics;
	return true;};

    /*! Get the pointer to the inverse kinematics */
    inline InverseKinematics * getInverseKinematics () const
      { return m_InverseKinematics; };

    /*! */
    /*! @} */


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
			    MAL_S3_VECTOR(&aWaistPosition,double),
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
      @param : Body_R : orientation of the Waist.
      @param : Body_P: position of the waist.
      @param : aFoot: A vector giving the foot configuration (x,y,z, theta, omega).
      @param : lDt: Vector describing the hip configuration.
      @param : aCoMPosition: Position of the CoM.
      @return : lq : Values of the leg which realize the position asked for.
     */
    bool KinematicsForOneLeg(MAL_S3x3_MATRIX(,double) & Body_R,
			     MAL_S3_VECTOR(,double) & Body_P,
			     MAL_VECTOR(,double) &aFoot,
			     MAL_S3_VECTOR(,double) &lDt,
			     MAL_VECTOR(,double) &aCoMPosition,
			     MAL_S3_VECTOR(,double) &ToTheHip,
			     MAL_VECTOR(,double) &lq);
    
    /*! Compute the angles values considering two 6DOF legs for a given configuration
      of the waist and of the feet:
      @param aCoMPosition: Position of the CoM (x,y,z,theta, omega, phi).
      @param aLeftFoot: Position of the foot (x,y,z, theta, omega).
      @param aRightFoot: Position of the foot (x,y,z,theta,omega).
      @param Stage: Stage of the ZMP Preview algorithm.
      @return ql: Angles for the left leg.
      @return qr: Angles for the right leg.
      
     */
    bool KinematicsForTheLegs(MAL_VECTOR(,double) & aCoMPosition,
			      MAL_VECTOR(,double) & aLeftFoot,
			      MAL_VECTOR(,double) & aRightFoot,
			      int Stage,
			      MAL_VECTOR(,double) & ql,
			      MAL_VECTOR(,double) & qr,
			      MAL_S3_VECTOR(,double) & AbsoluteWaistPosition);


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
    
    /*! Pointer to Inverse Kinematics */
    InverseKinematics * m_InverseKinematics;
    
    /*! Displacement between the hip and the foot. */
    MAL_S3_VECTOR(m_Dt,double);
    
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
      
    /*! Store the distance between the ankle and the soil. */
    double m_AnkleSoilDistance;
        
    /*! Difference between the CoM and the Waist 
      from the initialization phase,
      i.e. not reevaluated while walking. */
    MAL_S3_VECTOR(m_DiffBetweenComAndWaist,double);

    /*! Link towards Humanoids specificities */
    HumanoidSpecificities * m_HS;

    /*! Maximal distance along the X axis for the hand motion */
    double m_Xmax;

    /*! Maximal length of the arm */
    double m_ZARM;

    /*! Buffer of upper body position related to a plan */
    deque<PatternGeneratorJRL::KWNode > m_UpperBodyPositionsBuffer;

    /*! Conversion between the index of the plan and the robot DOFs. */
    vector<int> m_ConversionForUpperBodyFromLocalIndexToRobotDOFs;
        
    /*! Keep the indexes for the legs of the robot in the VRML numbering system. */
    vector<int> m_LeftLegIndexInVRML;
    vector<int> m_RightLegIndexInVRML;
    vector<int> m_LeftArmIndexInVRML;
    vector<int> m_RightArmIndexInVRML;

    /*! \name Keep the indexes into the Configuration numbering system. 
     @{
    */
    /*! \brief For the left leg, Specific for the Inverse Kinematics. */
    vector<int> m_LeftLegIndexinConfiguration;
    /*! \brief For the right leg, Specific for the Inverse Kinematics. */
    vector<int> m_RightLegIndexinConfiguration;
    /*! \brief For the left arm, Specific for the Inverse Kinematics. */
    vector<int> m_LeftArmIndexinConfiguration;
    /*! \brief For the right leg, Specific for the Inverse Kinematics. */
    vector<int> m_RightArmIndexinConfiguration;

    /*! \brief For the entire system. */
    vector<int> m_GlobalVRMLIDtoConfiguration;

    /*! Gain factor for the arm motion heuristic. */
    double m_GainFactor;

  };


};
#endif 

/*! \file ZMPPreviewControlWithMultiBodyZMP.h
  \brief This object generate all the values for the foot trajectories,
  and the desired ZMP based on a sequence of steps.

  Copyright (c) 2007 
  @author Olivier Stasse  
  Copyright (c) 2005-2006, 
  @author Francois Keith, Olivier Stasse, Ramzi Sellouati
   
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
#ifndef _ZMPREVIEWCONTROLWITHMULTIBODYZMP_H_
#define _ZMPREVIEWCONTROLWITHMULTIBODYZMP_H_

#include <deque>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <dynamicsJRLJapan/HumanoidSpecificities.h>

#include <walkGenJrl/walkGenJrl_API.h>
#include <walkGenJrl/PGTypes.h>
#include <walkGenJrl/PreviewControl/PreviewControl.h>
#include <walkGenJrl/MotionGeneration/ComAndFootRealization.h>


using namespace::std;

namespace PatternGeneratorJRL
{
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
  class WALK_GEN_JRL_EXPORT ZMPPreviewControlWithMultiBodyZMP 
    {
    private:
      
      /*! Vector from the Waist to the left and right hip. */
      //@{
      /*! Static part from the waist to the left hip.. */
      MAL_S3_VECTOR(m_StaticToTheLeftHip,double);
      /*! Static part from the waist to the right hip. */
      MAL_S3_VECTOR(m_StaticToTheRightHip,double);
      /*! Dynamic part from the waist to the left hip. */
      MAL_S3_VECTOR(m_TranslationToTheLeftHip,double);
      /*! Dynamic part form the waist to the right hip. */
      MAL_S3_VECTOR( m_TranslationToTheRightHip,double);

      /*! Displacement between the hip and the foot. */
      MAL_S3_VECTOR(m_Dt,double);
      
      /*! Preview control time. */
      double m_PreviewControlTime;
      
      /*! Size of the preview control window. */
      unsigned int m_NL;

      /*! Final state of the leg joints. */
      //@{
      /*! The left leg */
      MAL_MATRIX( Finalql,double);
      /*! The right leg */
      MAL_MATRIX( Finalqr,double);
      //@}
      
      /*! Fifo for the ZMP ref. */
      deque<ZMPPosition> m_FIFOZMPRefPositions;
      
      /*! Fifo for the ZMP ref. */
      deque<ZMPPosition> m_FIFODeltaZMPPositions;

      /*! Fifo for the COM reference. */
      deque<COMPosition> m_FIFOCOMPositions;

      /*! Fifo for the positionning of the left foot. */
      deque<FootAbsolutePosition> m_FIFOLeftFootPosition;
      
      /*! Fifo for the positionning of the right foot. */
      deque<FootAbsolutePosition> m_FIFORightFootPosition;

      /*! Error on preview control for the cart model. */
      double m_sxzmp, m_syzmp;
      
      /*! Error on preview control for the delta zmp. */
      double m_sxDeltazmp, m_syDeltazmp;

      /*! State of the Preview control. */
      MAL_MATRIX( m_PC1x,double);
      MAL_MATRIX(m_PC1y,double);

      /*! State of the Second Preview control. */
      MAL_MATRIX( m_Deltax, double);
      MAL_MATRIX(m_Deltay,double);

      /*! Starting a new step sequences. */
      bool m_StartingNewSequence;

      /*! Keep the ZMP reference. */
      deque<ZMPPosition> m_FIFOTmpZMPPosition;
	
      /*!extra COMPosition buffer calculated to give to the stepover planner  */
      vector<COMPosition> m_ExtraCOMBuffer;

      /*! Difference between the CoM and the Waist 
      from the initialization phase,
      i.e. not reevaluated while walking. */
      MAL_S3_VECTOR(,double) m_DiffBetweenComAndWaist;

      /*! COM Starting position. */
      MAL_S3_VECTOR(,double) m_StartingCOMPosition;

      /*! Final COM pose. */
      MAL_S4x4_MATRIX(,double) m_FinalDesiredCOMPose;
      
      
      /*! Store the distance between the ankle and the soil. */
      double m_AnkleSoilDistance;
	
      /*! Store a reference to the object to solve posture resolution. */
      ComAndFootRealization *m_ComAndFootRealization;

      /*! Store a reference to the object handling humanoid dynamics */
      CjrlHumanoidDynamicRobot * m_HumanoidDynamicRobot;

      /*! Number of iterations. */
      unsigned long long int m_NumberOfIterations;

      /*! Pointer to the Preview Control object. */
      PreviewControl *m_PC;

      /*! Store the strategy to handle the preview control stages. */
      int m_StageStrategy;

      /*! Sampling period. */
      double m_SamplingPeriod;

    public:
	
      /*! Constantes to define the strategy with the first and second stage. 
	@{
       */

      /*! Constant to compute the first and second stage. */
      static const int ZMPCOM_TRAJECTORY_FULL=1;

      /*! Constant to compute only the second stage */
      static const int ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY=2;

      /*! Constant to compute only the first stage. */
      static const int ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY=3;
      
      /*! @} */
      /*! Constructor. */
      ZMPPreviewControlWithMultiBodyZMP();

      /*! Destroctor. */
      ~ZMPPreviewControlWithMultiBodyZMP();
      
      
      /*! \name Implementation of the GlobalStrategyManager interface. 
	@{ */
      /*! Set the algorithm used for ZMP and CoM trajectory. 
	@param[in] anAlgo: The algorithm to be used for ZMP and CoM trajectory generation.
	They are 3 possible values:

	\li ZMPCOM_TRAJECTORY_FULL: Two preview control are computed. The first
	to generate a CoM trajectory based on the cart model. The second to correct
	this trajectory using the multibody ZMP.

	\li ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY: Only the second stage is used.
	The first CoM trajectory is used by a different process. This allow
	to mix different algorithms (notable the quadratic problem with constraints).
	
	\li ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY: Use only the first stage to generate
	the CoM trajectory. It is strongly adviced in this case, to not use
	the geometrical ZMP and CoM trajectory generation but an external CoM task.

	@return Returns false if this is not possible.
      */
      void SetStrategyForStageActivation(int anAlgo);
      
      /*! Get the strategy for the activation of the stage.
       */
      int GetStrategyForStageActivation();
      
      /*! @} */
      /*! Returns the difference between the Waist and the CoM for a starting position. */
      void GetDifferenceBetweenComAndWaist(double lComAndWaist[3]);

      /*! Perform a 5 ms step to generate the full set of angular positions.
	  The main point of the preview control is to use the future to compute
	  the current state needed for the robot. Therefore knowing that
	  the future window needed is of size NL=SamplingPeriod * PreviewControlWindow,
	  and that the algorithm is a two stages preview control,
	  the foot position needs to be provided at k+NL, and the ZMP references
	  at k+2*NL.

	  @param[in] LeftFootPosition: The position of the k+NL Left Foot position.
	  @param[in] RightFootPosition: The position of the k+NL Right Foot position.
	  @param[in] NewZMPRefPos: The ZMP position at k + 2*NL.
	  @param[out] finalCOMPosition: returns position, velocity and acceleration of the CoM 
	  after the second stage of control, i.e. the final value.
	  @param[out] CurrentConfiguration: The results is a state vector containing the articular positions.
	  @param[out] CurrentVelocity: The results is a state vector containing the speed.
	  @param[out] CurrentAcceleration: The results is a state vector containing the acceleration.
       */
      int OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
				 FootAbsolutePosition &RightFootPosition,
				 ZMPPosition &NewZMPRefPos,
				 COMPosition & finalCOMPosition,
				 MAL_VECTOR(,double) & CurrentConfiguration,
				 MAL_VECTOR(,double) & CurrentVelocity,
				 MAL_VECTOR(,double) & CurrentAcceleration);

	
      /*! First stage of the control, 
	i.e.preview control on the CART model with delayed step parameters,
	Inverse Kinematics, and ZMP calculated with the multi body model.
	aCOMPosition will be updated with the new value of the COM computed by
	the card model.
	@param[in] LeftFootPosition: The position of the k+NL Left Foot position.
	@param[in] RightFootPosition: The position of the k+NL Right Foot position.
	@param[in] afCOMPosition: A COM position of reference, in this context,
	this will be the height of the waist.

	@return If an error occurs returns a negative integer, 0 otherwise.
      */
      int FirstStageOfControl(FootAbsolutePosition &LeftFootPosition,
			      FootAbsolutePosition &RightFootPosition,
			      COMPosition &afCOMPosition);
      
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
      int SecondStageOfControl(COMPosition &refandfinal);

      /*! Compute the COM of the robot with the Joint values given in BodyAngles,
	velocities set to zero, and returns the values of the COM in aStaringCOMPosition.
	Assuming that the waist is at (0,0,0)
	it returns the associate initial values for the left and right foot.
	@param[in] BodyAngles: 4x4 matrix of the robot's root (most of the time, the waist)
	pose (position + orientation).
	@param[out] aStartingCOMPosition: Returns the 3D position of the CoM for the current
	position of the robot.
	@param[out] aStartingZMPPosition: Returns the 3D position of the ZMP for the current
	position of the robot.
	@param[out] InitLeftFootPosition: Returns the position of the left foot in
	the waist coordinates frame.
	@param[out] InitRightFootPosition: Returns the position of the right foot
	in the waist coordinates frame.
      */
      int EvaluateStartingState(MAL_VECTOR( &,double) BodyAngles,
				MAL_S3_VECTOR( &,double) aStartingCOMPosition,
				MAL_S3_VECTOR( &,double) aStartingZMPPosition,
				MAL_VECTOR( &,double) aStartingWaistPose,
				FootAbsolutePosition & InitLeftFootPosition,
				FootAbsolutePosition & InitRightFootPosition);

      /*! Compute the COM of the robot with the Joint values given in BodyAngles,
	velocities set to zero, and returns the values of the COM in aStaringCOMPosition.
	Assuming that the waist is at (0,0,0)
	it returns the associate initial values for the left and right foot.
	@param BodyAngles: Vector of the joint values for the robot.
	@param[out] aStartingCOMPosition: Position of the CoM.
	@param[out] aWaistPosition: Position of the Waist.
	@param[out] InitLeftFootPosition: Position of the left foot in the waist coordinates frame.
	@param[out] InitRightFootPosition: Position of the right foot in the waist coordinates
	frame.
      */
      int EvaluateStartingCoM(MAL_MATRIX( &,double) BodyAngles,
			      MAL_S3_VECTOR( &,double) aStartingCOMPosition,
			      MAL_VECTOR( &,double) aWaistPose,
			      FootAbsolutePosition & InitLeftFootPosition,
			      FootAbsolutePosition & InitRightFootPosition);

      /*! Methods related to the preparation of the ZMP preview control with
	Multibody ZMP compensation. 
	@{
      */

      /*! Setup (Frontal Global), compute internally all the steps to get NL ZMP multibody values.
	
	@param[in] ZMPRefPositions: FIFO of the ZMP reference values.
	@param[out] COMPositions: FIFO of the COM reference positions 
	   (here only the height position is taken into account).
	@param[in] LeftFootPositions: FIFO of the left foot positions computed by
	ZMPDiscretization (the object creating the ZMP and foot reference 
	trajectories).
	@param[in] RightFootPositions: idem than the previous one but for the 
	right foot.
       */
      int Setup(deque<ZMPPosition> &ZMPRefPositions,
		deque<COMPosition> &COMPositions,
		deque<FootAbsolutePosition> &LeftFootPositions,
		deque<FootAbsolutePosition> &RightFootPositions);

      /*! Method to perform the First Phase. It initializes properly the internal fields
	of ZMPPreviewControlWithMultiBodyZMP for the setup phase.

       	@param[in] ZMPRefPositions: FIFO of the ZMP reference values.
	@param[in] COMPositions: FIFO of the COM reference positions 
	   (here only the height position is taken into account).
	@param[in] LeftFootPositions: FIFO of the left foot positions computed by
	ZMPDiscretization (the object creating the ZMP and foot reference 
	trajectories).
	@param[in] RightFootPositions: idem than the previous one but for the 
	right foot.
      */
      int SetupFirstPhase(deque<ZMPPosition> &ZMPRefPositions,
			  deque<COMPosition> &COMPositions,
			  deque<FootAbsolutePosition> &LeftFootPositions,
			  deque<FootAbsolutePosition> &RightFootPositions);


      /*! Method to call while feeding the 2 preview windows.
	It updates the first values of the Preview control
	This structure is needed if it is needed to modify BodyAngles according
	to the value of the COM.

	@param[in] ZMPRefPositions: FIFO of the ZMP reference values.
	@param[out] COMPositions: FIFO of the COM reference positions 
	   (here only the height position is taken into account).
	@param[in] LeftFootPositions: FIFO of the left foot positions computed by
	ZMPDiscretization (the object creating the ZMP and foot reference 
	trajectories).
	@param[in] RightFootPositions: idem than the previous one but for the 
	right foot.
	@param[out] CurrentConfiguration: The position part of the state vector realizing the current CoM and 
	feet position instance.
	@param[out] CurrentVelocity: The velocity part of the state vector realizing the current CoM and
	feet position instance.
	@param[out] CurrentAcceleration: The acceleration part of the state vector realizing the current CoM and
	feet position instance.
	@param[in] localindex: Value of the index which goes from 0 to 2 * m_NL.
      */
      int SetupIterativePhase(deque<ZMPPosition> &ZMPRefPositions,
			      deque<COMPosition> &COMPositions,
			      deque<FootAbsolutePosition> &LeftFootPositions,
			      deque<FootAbsolutePosition> &RightFootPositions,
			      MAL_VECTOR(,double) &CurrentConfiguration,
			      MAL_VECTOR(,double) & CurrentVelocity,
			      MAL_VECTOR(,double) & CurrentAcceleration,
			      int localindex);
      
      
      /*! Create an extra COM buffer with a first preview round to be 
	used by the stepover planner.

	@param[out] ExtraCOMBuffer: Extra FIFO for the CoM positions.
	@param[out] ExtraZMPBuffer: Extra FIFO for the ZMP positions (for the stepping over
	first preview control).
	@param[out] ExtraZMPRefBuffer: Extra FIFO for the ZMP ref positions.
      */
      void CreateExtraCOMBuffer(deque<COMPosition> &ExtraCOMBuffer,
				deque<ZMPPosition> &ExtraZMPBuffer,
				deque<ZMPPosition> &ExtraZMPRefBuffer);
      
      /*! Evaluate Starting CoM for a given position.
	@param[in] BodyAnglesInit: The state vector used to compute the CoM.
	@param[out] aStartingCOMPosition: The CoM of the position specified.
	@param[out] InitLeftFootPosition: Position of the InitLeftFootPosition in the same
	reference frame than the waist.
	@param[out] InitRightFootPosition: Position of the InitRightFootPosition
	in the same reference frame than the waist 
       */
      int EvaluateStartingCoM(MAL_VECTOR(&,double) BodyAnglesInit,
			      MAL_S3_VECTOR(&,double) aStartingCOMPosition,
			      MAL_VECTOR(&,double) aStartingWaistPosition,
			      FootAbsolutePosition & InitLeftFootPosition,
			      FootAbsolutePosition & InitRightFootPosition);

      /*! This method returns the final COM pose matrix after the second stage of control. 
       @return A 4x4 matrix of double which includes the desired final CoM position and orientation.*/
      MAL_S4x4_MATRIX(,double) GetFinalDesiredCOMPose();

      /*! This method returns the current waist position in the COM reference 
	frame. This can be used with the previous method to get the final Waist 
	position.
	@return A 4x4 matrix of double which includes the desired final Waist in the CoM 
	phase position and orientation.*/
      MAL_S4x4_MATRIX(,double) GetCurrentPositionofWaistInCOMFrame();

      /*! Returns the last element of the COM FIFO in the first stage of control */
      COMPosition GetLastCOMFromFirstStage();

      /*! Update the queue of ZMP reference value. */
      void UpdateTheZMPRefQueue(ZMPPosition NewZMPRefPos);

      /*! \name Setter and getter for the ComAndZMPTrajectoryGeneration. */
      inline bool setComAndFootRealization(ComAndFootRealization * aCFR)
      { m_ComAndFootRealization = aCFR; return true;};
      inline ComAndFootRealization * getComAndFootRealization()
	{ return m_ComAndFootRealization;};
      
      /*! Call To CoM And Foot Realization object,
	the last values in the stack for the CoM
	and the feet positions will be used. 
	@param[in] acomp : COM position,
	@param[in] aLeftFAP: Pose of the left foot (3D position + 2 euler angles)
	@param[in] aRightFAP: Pose of the right foot (3D position + 2 euler angles)
	@param[out] CurrentConfiguration: Returns the part of state vector corresponding 
	to the position of the free floating, and the articular values.
	@param[out] CurrentVelocity: Returns the part of state vector corresponding
	to the velocity of the free floating and the articular values.
	@param[out] CurrentAcceleration: Returns the part of state vector corresponding 
	to the acceleration of the free floating, and the articular values.
	@param[in] IterationNumber: Number of time slot realized so far.
	@param[in] StageOfTheAlgorithm: Indicates if this is the second stage of 
	the preview control or the first one.
      */
      void CallToComAndFootRealization(COMPosition &acomp,
				       FootAbsolutePosition &aLeftFAP,
				       FootAbsolutePosition &aRightFAP,
				       MAL_VECTOR(,double) & CurrentConfiguration,
				       MAL_VECTOR(,double) & CurrentVelocity,
				       MAL_VECTOR(,double) & CurrentAcceleration,
				       int IterationNumber,
				       int StageOfTheAlgorithm);


      
      /*! Set the link to the preview control. */
      void SetPreviewControl(PreviewControl *aPC);

      /*! \name Setter and getter for the jrlHumanoidDynamicRobot object. */
      /*! @param[in] aHumanoidDynamicRobot: an object able to compute dynamic parameters
	of the robot. */
      inline  bool setHumanoidDynamicRobot(const CjrlHumanoidDynamicRobot *aHumanoidDynamicRobot)
      { m_HumanoidDynamicRobot = (CjrlHumanoidDynamicRobot *)aHumanoidDynamicRobot;
	return true;}
      
      /*! Returns the object able to compute the dynamic parameters
	of the robot. */
      inline CjrlHumanoidDynamicRobot * getHumanoidDynamicRobot() const
	{ return m_HumanoidDynamicRobot;}
    
      /*! Set the strategy to handle the preview control stages. */
      void SetStrategyForPCStages(int Strategy);
      
      /*! Get the strategy to handle the preview control stages. */
      int GetStrategyForPCStages();

    };
};
#endif /* _ZMPREVIEWCONTROLWITHMULTIBODYZMP_H_ */

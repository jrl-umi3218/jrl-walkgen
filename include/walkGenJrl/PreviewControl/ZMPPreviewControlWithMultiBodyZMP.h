/* This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of steps.

   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its contributors 
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

#include <PreviewControl/PreviewControl.h>
#include <PGTypes.h>
#include <dynamicsJRLJapan/HumanoidSpecificities.h>
#include <MotionGeneration/ComAndFootRealization.h>

using namespace::std;

namespace PatternGeneratorJRL
{
  /** @ingroup pgjrl

      Object to generate the angle positions
      every 5 ms from a set of absolute foot positions.
      This algorithm use the preview control proposed by Kajita-San
      in ICRA 2003 Biped Walking Pattern Generation by using Preview
      Control of Zero-Moment Point pp.1620--1626.
      You therefore have to use first the Setup method
      to fill all the queues. Then every 5 ms just use
      OneGlobalStepOfControl to compute the Waist
      Computation position, speed, acceleration and the 
      angular values for the left and right legs.
      
   */
  class ZMPPreviewControlWithMultiBodyZMP
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
      
      /*! Pointer to the Preview Control object. */
      PreviewControl *m_PC;

      /*! Sampling Period. */
      double m_SamplingPeriod;
      
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
      
      /*! Store the index for the algorithm to use for ZMP and CoM trajectory. */
      int m_ZMPCoMTrajectoryAlgorithm;
      
      /*! Store the distance between the ankle and the soil. */
      double m_AnkleSoilDistance;
	
      /*! Store a reference to the object to solve posture resolution. */
      ComAndFootRealization *m_ComAndFootRealization;

      /*! Store a reference to the object handling humanoid dynamics */
      CjrlHumanoidDynamicRobot * m_HumanoidDynamicRobot;

      /*! Number of iterations. */
      unsigned long long int m_NumberOfIterations;

    public:
	
      static const int ZMPCOM_TRAJECTORY_KAJITA=1;
      static const int ZMPCOM_TRAJECTORY_WIEBER=2;
      
      /*! Constructor. */
      ZMPPreviewControlWithMultiBodyZMP();

      /*! Destroctor. */
      ~ZMPPreviewControlWithMultiBodyZMP();

      /*! Set the link to the preview control. */
      void SetPreviewControl(PreviewControl *aPC);
      
      
      /*! Returns the difference between the Waist and the CoM for a starting position. */
      void GetDifferenceBetweenComAndWaist(double lComAndWaist[3]);

      /*! Perform a 5 ms step to generate the full set of angular positions.
	  The main point of the preview control is to use the future to compute
	  the current state needed for the robot. Therefore knowing that
	  the future window needed is of size NL=SamplingPeriod * PreviewControlWindow,
	  and that the algorithm is a two stages preview control,
	  the foot position needs to be provided at k+NL, and the ZMP references
	  at k+2*NL.

	  @param LeftFootPosition: The position of the k+NL Left Foot position.
	  @param RightFootPosition: The position of the k+NL Right Foot position.
	  @param NewZMPRefPos: The ZMP position at k + 2*NL.
	  @return qr,ql: 1x6 vector for the right and left legs angular values at k.
	  @param refCOMPosition: reference values at k for the COM, right now only the height is 
	  taken into account. The output of the first stage of control is returned inside this structure.
	  @return finalCOMPosition: returns position, velocity and acceleration of the CoM 
	  after the second stage of control, i.e. the final value.
       */
      int OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
				 FootAbsolutePosition &RightFootPosition,
				 ZMPPosition &NewZMPRefPos,
				 COMPosition & finalCOMPosition,
				 MAL_VECTOR(,double) & CurrentConfiguration,
				 MAL_VECTOR(,double) & CurrentVelocity);

	
      /*! First stage of the control, 
	i.e.preview control on the CART model with delayed step parameters,
	Inverse Kinematics, and ZMP calculated with the multi body model.
	aCOMPosition will be updated with the new value of the COM computed by
	the card model.
	@param LeftFootPosition: The position of the k+NL Left Foot position.
	@param RightFootPosition: The position of the k+NL Right Foot position.
	@param refCOMPosition: A COM position of reference, in this context,
	this will be the height of the waist.
	@return ql,qr: The joint values for the left and right legs (1x6 vector for each).
	@return BodyAttitude: A 4x4 matrix which gives the position and the orientation
	of the waist.
      */
      int FirstStageOfControl(FootAbsolutePosition &LeftFootPosition,
			      FootAbsolutePosition &RightFootPosition,
			      COMPosition &afCOMPosition);
      
      /*! This methods is used only to update the queue of ZMP difference
	for the second stage of control. Also it does not return
	anything this method is crucial for the overall process.
	@param ql, qr: The joint values for the left and right legs.
	@param UpperBodyAngles: The joint values for the upper body.
	@param BodyAttitude: A 4x4 matrix which gives the position and the 
	orientation of the waist.
	@param StartingIteration: -1 for the initialization, >=0 for 
	a counter which gives the time.
      */
      int EvaluateMultiBodyZMP(int StartingIteration);

      /*! Second stage of the control, i.e. preview control on the Delta ZMP.
	COM correction, and computation of the final robot state
	(only the left and right legs).
	@return lqr, lql : The final value for the left and right leg joint values.
	@return aCOMPosition: The final position of the CoM.
      */
      int SecondStageOfControl(COMPosition &refandfinal);

      /*! Compute the COM of the robot with the Joint values given in BodyAngles,
	velocities set to zero, and returns the values of the COM in aStaringCOMPosition.
	Assuming that the waist is at (0,0,0)
	it returns the associate initial values for the left and right foot.
	@param BodyAngles: 4x4 matrix of the robot's root (most of the time, the waist)
	pose (position + orientation).
	@return aStartingCOMPosition: Returns the 3D position of the CoM for the current
	position of the robot.
	@return InitLeftFootPosition: Returns the position of the left foot in
	the waist coordinates frame.
	@return InitRightFootPosition: Returns the position of the right foot
	in the waist coordinates frame.
      */
      int EvaluateStartingCoM(MAL_MATRIX( &BodyAngles,double),
			      MAL_S3_VECTOR( &aStartingCOMPosition,double),
			      FootAbsolutePosition & InitLeftFootPosition,
			      FootAbsolutePosition & InitRightFootPosition);

      /*! Compute the COM of the robot with the Joint values given in BodyAngles,
	velocities set to zero, and returns the values of the COM in aStaringCOMPosition.
	Assuming that the waist is at (0,0,0)
	it returns the associate initial values for the left and right foot.
	@param BodyAngles: Vector of the joint values for the robot.
	@return aStartingCOMPosition: Position of the CoM.
	@return aWaistPosition: Position of the Waist.
	@return InitLeftFootPosition: Position of the left foot in the waist coordinates frame.
	@return InitRightFootPosition: Position of the right foot in the waist coordinates
	frame.
      */
      int EvaluateStartingCoM(MAL_MATRIX( &BodyAngles,double),
			      MAL_S3_VECTOR( &aStartingCOMPosition,double),
			      MAL_S3_VECTOR( &aWaistPosition,double),
			      FootAbsolutePosition & InitLeftFootPosition,
			      FootAbsolutePosition & InitRightFootPosition);

      /*! Setup, compute all the steps to get NL ZMP multibody values.
	@param ZMPRefPositions: FIFO of the ZMP reference values.
	@param COMPositions: FIFO of the COM reference positions 
	   (here only the height position is taken into account).
	@param LeftFootPositions: FIFO of the left foot positions computed by
	ZMPDiscretization (the object creating the ZMP and foot reference 
	trajectories).
	@param RightFootPositions: idem than the previous one but for the 
	right foot.
	@param BodyAngles: Value of the upper body joints.
       */
      int Setup(deque<ZMPPosition> &ZMPRefPositions,
		deque<COMPosition> &COMPositions,
		deque<FootAbsolutePosition> &LeftFootPositions,
		deque<FootAbsolutePosition> &RightFootPositions);

      /*! Setup, : First steps: Initialize properly the internal fields
	of ZMPPreviewControlWithMultiBodyZMP for the setup phase.
       	@param ZMPRefPositions: FIFO of the ZMP reference values.
	@param COMPositions: FIFO of the COM reference positions 
	   (here only the height position is taken into account).
	@param LeftFootPositions: FIFO of the left foot positions computed by
	ZMPDiscretization (the object creating the ZMP and foot reference 
	trajectories).
	@param RightFootPositions: idem than the previous one but for the 
	right foot.
	@param BodyAngles: Value of the upper body joints.
      */
      int SetupFirstPhase(deque<ZMPPosition> &ZMPRefPositions,
			  deque<COMPosition> &COMPositions,
			  deque<FootAbsolutePosition> &LeftFootPositions,
			  deque<FootAbsolutePosition> &RightFootPositions);


      /*! Setup, : Iterative step: Update the first values of the Preview control
	This structure is needed if it is needed to modify BodyAngles according
	to the value of the COM.
	@param ZMPRefPositions: FIFO of the ZMP reference values.
	@param COMPositions: FIFO of the COM reference positions 
	   (here only the height position is taken into account).
	@param LeftFootPositions: FIFO of the left foot positions computed by
	ZMPDiscretization (the object creating the ZMP and foot reference 
	trajectories).
	@param RightFootPositions: idem than the previous one but for the 
	right foot.
	@param BodyAngles: Value of the upper body joints.
	@param localindex: Value of the index which goes from 0 to 2 * m_NL.
      */
      int SetupIterativePhase(deque<ZMPPosition> &ZMPRefPositions,
			      deque<COMPosition> &COMPositions,
			      deque<FootAbsolutePosition> &LeftFootPositions,
			      deque<FootAbsolutePosition> &RightFootPositions,
			      MAL_VECTOR(,double) &CurrentConfiguration,
			      MAL_VECTOR(,double) & CurrentVelocity,
			      int localindex);
      
      
      /*! Create an extra COM buffer with first preview round to be 
	used by the stepover planner.
	@param ExtraCOMBuffer: Extra FIFO for the CoM positions.
	@param ExtraZMPBuffer: Extra FIFO for the ZMP positions (for the stepping over
	first preview control).
	@param ExtraZMPRefBuffer: Extra FIFO for the ZMP ref positions.
      */
      void CreateExtraCOMBuffer(deque<COMPosition> &ExtraCOMBuffer,
				deque<ZMPPosition> &ExtraZMPBuffer,
				deque<ZMPPosition> &ExtraZMPRefBuffer);
      

      /*! Evaluate CoM for a given position.
	Assuming that the waist is at (0,0,0)
	It returns the associate initial values for the left and right foot.
      */
      int EvaluateCOM(MAL_MATRIX( &BodyAngles,double),
		      double omega, double theta,
		      MAL_S3_VECTOR( &lCOMPosition,double),
		      FootAbsolutePosition & LeftFootPosition,
		      FootAbsolutePosition & RightFootPosition);
      
      /*! Evaluate CoM for a given position.
	Assuming that the waist is at (0,0,0)
	It returns the associate initial values for the left and right foot.*/
      int EvaluateCOM(MAL_MATRIX( &BodyAngles,double),
		      double omega, double theta,
		      MAL_S3_VECTOR( &lCOMPosition,double),
		      MAL_S3_VECTOR( &WaistPosition,double),
		      FootAbsolutePosition & LeftFootPosition,
		      FootAbsolutePosition & RightFootPosition);

      /*! Evaluate Starting CoM for a given position.
       */
      int EvaluateStartingCoM(MAL_VECTOR(&BodyAnglesInit,double),
			      MAL_S3_VECTOR(&aStartingCOMPosition,double),
			      FootAbsolutePosition & InitLeftFootPosition,
			      FootAbsolutePosition & InitRightFootPosition);

      /*! This method returns the final COM pose matrix after the second stage of control. */
      MAL_S4x4_MATRIX(,double) GetFinalDesiredCOMPose();

      /*! This method returns the current waist position in the COM reference 
	frame. This can be used with the previous method to get the final Waist 
	position.
      */
      MAL_S4x4_MATRIX(,double) GetCurrentPositionofWaistInCOMFrame();

      /*! Returns the last element of the COM FIFO in the first stage of control */
      COMPosition GetLastCOMFromFirstStage();

      /*! Update the queue of ZMP reference value. */
      void UpdateTheZMPRefQueue(ZMPPosition NewZMPRefPos);

      /*! \name Setter and getter for the ComAndZMPTrajectoryGeneration. */
      inline bool setComAndFootRealization(ComAndFootRealization * aCFR)
	{ m_ComAndFootRealization = aCFR;};
      inline ComAndFootRealization * getComAndFootRealization()
	{ return m_ComAndFootRealization;};
      
      /*! Call To CoM And Foot Realization object,
	the last values in the stack for the CoM
	and the feet positions will be used. 
	@param: acomp : COM position,
	@param: aLeftFAP: Pose of the left foot (3D position + 2 euler angles)
	@param: aRightFAP: Pose of the right foot (3D position + 2 euler angles)
      */
      void CallToComAndFootRealization(COMPosition &acomp,
				       FootAbsolutePosition &aLeftFAP,
				       FootAbsolutePosition &aRightFAP,
				       MAL_VECTOR(,double) & CurrentConfiguration,
				       MAL_VECTOR(,double) & CurrentVelocity,
				       int IterationNumber,
				       int StageOfTheAlgorithm);

      /*! \name Setter and getter for the jrlHumanoidDynamicRobot object. */
      
      /*! @param aHumanoidDynamicRobot: an object able to compute dynamic parameters
	of the robot. */
      inline  bool setHumanoidDynamicRobot(const CjrlHumanoidDynamicRobot *aHumanoidDynamicRobot)
	{ m_HumanoidDynamicRobot = (CjrlHumanoidDynamicRobot *)aHumanoidDynamicRobot;
	  return true;}
      
      /*! @param aHumanoidDynamicRobot: an object able to compute dynamic parameters
	of the robot. */
      inline CjrlHumanoidDynamicRobot * getHumanoidDynamicRobot() const
	{ return m_HumanoidDynamicRobot;}
      
      /** @} */

      /*! Set the algorithm used for ZMP and CoM trajectory. */
      void SetAlgorithmForZMPAndCoMTrajectoryGeneration(int anAlgo);
 
      /*! Get the algorithm used for ZMP and CoM trajectory. */
      int GetAlgorithmForZMPAndCoMTrajectoryGeneration();


    };
};
#endif /* _ZMPREVIEWCONTROLWITHMULTIBODYZMP_H_ */

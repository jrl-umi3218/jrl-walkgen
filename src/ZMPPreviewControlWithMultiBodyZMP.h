/** @doc This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of steps.

   $Id: ZMPPreviewControlWithMultiBodyZMP.h,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/ZMPPreviewControlWithMultiBodyZMP.h,v $
   $Log: ZMPPreviewControlWithMultiBodyZMP.h,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin


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
#include <VNL/matrix.h>
#include <PreviewControl.h>
#include <InverseKinematics.h>
#include <DynamicMultiBody.h>
#include <ZMPDiscretization.h>
#include <HumanoidSpecificities.h>

using namespace::std;

namespace PatternGeneratorJRL
{
  /** Object to generate the angle positions
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
    protected:

      /// Vector from the Waist to the left and right hip.
      VNL::Matrix<double> m_StaticToTheLeftHip, m_StaticToTheRightHip,
	m_TranslationToTheLeftHip, m_TranslationToTheRightHip;

      /// Displacement between the hip and the foot.
      VNL::Matrix<double> m_Dt;

      
	/// Pointer to the Preview Control object.
      PreviewControl *m_PC;
      /// Pointer to the Dynamic Multi body model.
      DynamicMultiBody *m_DMB;

      /// Pointer to the Inverse Kinematics model.
      InverseKinematics *m_IK;

      /// Previous joint values.
      VNL::Matrix<double> m_prev_ql,m_prev_qr;
      VNL::Matrix<double> m_prev_UpperBodyAngles;


      /// Sampling Period.
      double m_SamplingPeriod;
      
      /// Previous Linear momentum.
      vector3d m_prev_P;
      
      /// Previous Angular momentum.
      vector3d m_prev_L;

      /// Preview control time.
      double m_PreviewControlTime;
      
      /// Size of the preview control window.
      unsigned int m_NL;

      /// Final state of the leg joints.
      VNL::Matrix<double> Finalql, Finalqr;
      
      /// Fifo for the ZMP ref.
      deque<ZMPPosition> m_FIFOZMPRefPositions;
      
      /// Fifo for the ZMP ref.
      deque<ZMPPosition> m_FIFODeltaZMPPositions;

      /// Fifo for the COM reference.
      deque<COMPosition> m_FIFOCOMPositions;
      
      /// Fifo for the positionning of the left foot.
      deque<FootAbsolutePosition> m_FIFOLeftFootPosition;
      
      /// Fifo for the positionning of the right foot.
      deque<FootAbsolutePosition> m_FIFORightFootPosition;

      /// Error on preview control for the cart model.
      double m_sxzmp, m_syzmp;
      
      /// Error on preview control for the delta zmp.
      double m_sxDeltazmp, m_syDeltazmp;

      /// State of the Preview control.
      VNL::Matrix<double> m_PC1x,m_PC1y;

      /// State of the Second Preview control.
      VNL::Matrix<double> m_Deltax, m_Deltay;

      /// Starting a new step sequences.
      bool m_StartingNewSequence;

      /// Keep the ZMP reference.
      deque<ZMPPosition> m_FIFOTmpZMPPosition;
	
      ///extra COMPosition buffer calculated to give to the stepover planner 
      vector<COMPosition> m_ExtraCOMBuffer;

      /// Difference between the CoM and the Waist from the initialization phase,
      /// i.e. not reevaluated while walking.
      VNL::Vector<double> m_DiffBetweenComAndWaist;

      /// COM Starting position.
      vector3d m_StartingCOMPosition;

      /// Final COM pose.
      VNL::Matrix<double> m_FinalDesiredCOMPose;
      
      /// Store the index for the algorithm to use for ZMP and CoM trajectory.
      int m_ZMPCoMTrajectoryAlgorithm;

      /// Store the distance between the ankle and the soil.
	double m_AnkleSoilDistance;
       
      /// Store a reference to the object handling humanoid specific data.
      HumanoidSpecificities *m_HS;
    public:
	
	static const int ZMPCOM_TRAJECTORY_KAJITA=1;
	static const int ZMPCOM_TRAJECTORY_WIEBER=2;

      /// Constructor.
      ZMPPreviewControlWithMultiBodyZMP(HumanoidSpecificities *aHS);

      /// Destructor
      ~ZMPPreviewControlWithMultiBodyZMP();

      /// Set the link to the preview control.
      void SetPreviewControl(PreviewControl *aPC);
      
      /// Set the link with the Dynamic Multi Body model.
      void SetDynamicMultiBodyModel(DynamicMultiBody *aDMB);

      /// Set the link with the inverse kinematics model.
      void SetInverseKinematics(InverseKinematics *anIK);      
      
      /// Returns the difference between the Waist and the CoM for a starting position.
      void GetDifferenceBetweenComAndWaist(double lComAndWaist[3]);

      /** Perform a 5 ms step to generate the full set of angular positions.
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
				 VNL::Matrix<double> & lqr, 
				 VNL::Matrix<double> & lql, 
				 COMPosition & refCOMPosition, 
				 COMPosition & finalCOMPosition,
				 VNL::Matrix<double> &UpperBodyAngles);

      // For bakward compatibility this method complies with the previous
      // approach by merging the reference and final value.
      // Compare to the previous method the intermediate value of the COMPosition 
      // after the first stage is not provided.
      int OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
				 FootAbsolutePosition &RightFootPosition,
				 ZMPPosition &NewZMPRefPos,
				 VNL::Matrix<double> & lqr, 
				 VNL::Matrix<double> & lql, 
				 COMPosition & refandfinalCOMPosition,
				 VNL::Matrix<double> &UpperBodyAngles);
	
      // First stage of the control, 
      // i.e.preview control on the CART model with delayed step parameters,
      // Inverse Kinematics,
      // and ZMP calculated with the multi body model.
      // aCOMPosition will be updated with the new value of the COM computed by
      // the card model.
      int FirstStageOfControl(FootAbsolutePosition &LeftFootPosition,
			      FootAbsolutePosition &RightFootPosition,
			      COMPosition & refCOMPosition,
			      VNL::Matrix<double> &ql,
			      VNL::Matrix<double> &qr,
			      VNL::Matrix<double> &BodyAttitude);

      // This methods is used only to update the queue of ZMP difference
      // for the second stage of control. Also it does not return
      // anything this method is crucial for the overall process.
      int EvaluateMultiBodyZMP(VNL::Matrix<double> &ql,
				VNL::Matrix<double> &qr,
				VNL::Matrix<double> &UpperBodyAngles,
				VNL::Matrix<double> &BodyAttitude,
				int StartingIteration);

      // Second stage of the control,
      // i.e. preview control on the Delta ZMP.
      // COM correction,
      // and computation of the final robot state
      // (only the left and right legs).
      int SecondStageOfControl(VNL::Matrix<double> & lqr, VNL::Matrix<double> & lql, 
			       COMPosition & aCOMPosition);

      // Compute the COM of the robot with the Joint values given in BodyAngles,
      // velocities set to zero, and returns the values of the COM in aStaringCOMPosition.
      // Assuming that the waist is at (0,0,0)
      // it returns the associate initial values for the left and right foot.
      int EvaluateStartingCoM(VNL::Matrix<double> &BodyAngles,
			      VNL::Vector<double> &aStartingCOMPosition,
			      FootAbsolutePosition & InitLeftFootPosition,
			      FootAbsolutePosition & InitRightFootPosition);

      // Compute the COM of the robot with the Joint values given in BodyAngles,
      // velocities set to zero, and returns the values of the COM in aStaringCOMPosition.
      // Assuming that the waist is at (0,0,0)
      // it returns the associate initial values for the left and right foot.
      int EvaluateStartingCoM(VNL::Matrix<double> &BodyAngles,
			      VNL::Vector<double> &aStartingCOMPosition,
			      VNL::Vector<double> &aWaistPosition,
			      FootAbsolutePosition & InitLeftFootPosition,
			      FootAbsolutePosition & InitRightFootPosition);

      // Setup, compute all the steps to get NL ZMP multibody values.
      int Setup(deque<ZMPPosition> &ZMPRefPositions,
		deque<COMPosition> &COMPositions,
		deque<FootAbsolutePosition> &LeftFootPositions,
		deque<FootAbsolutePosition> &RightFootPositions,
		VNL::Matrix<double> &BodyAngles);

      // Setup, : First steps: Initialize properly the internal fields
      // of ZMPPreviewControlWithMultiBodyZMP for the setup phase.
      int SetupFirstPhase(deque<ZMPPosition> &ZMPRefPositions,
			  deque<COMPosition> &COMPositions,
			  deque<FootAbsolutePosition> &LeftFootPositions,
			  deque<FootAbsolutePosition> &RightFootPositions,
			  VNL::Matrix<double> &BodyAngles);


      // Setup, : Iterative step: Update the first values of the Preview control
      // This structure is needed if it is needed to modify BodyAngles according
      // to the value of the COM.
      int SetupIterativePhase(deque<ZMPPosition> &ZMPRefPositions,
			      deque<COMPosition> &COMPositions,
			      deque<FootAbsolutePosition> &LeftFootPositions,
			      deque<FootAbsolutePosition> &RightFootPositions,
			      VNL::Matrix<double> &BodyAngles, int localindex);
      
      
      //create an extra COM buffer with first preview round to be used by the stepover planner
      void CreateExtraCOMBuffer(deque<COMPosition> &m_ExtraCOMBuffer,
				deque<ZMPPosition> &m_ExtraZMPBuffer,
				deque<ZMPPosition> &m_ExtraZMPRefBuffer);
      

      // Evaluate CoM for a given position.
      // Assuming that the waist is at (0,0,0)
      // It returns the associate initial values for the left and right foot.
      int EvaluateCOM(VNL::Matrix<double> BodyAngles,
		      double omega, double theta,
		      vector3d &lCOMPosition,
		      FootAbsolutePosition & LeftFootPosition,
		      FootAbsolutePosition & RightFootPosition);

      // Evaluate CoM for a given position.
      // Assuming that the waist is at (0,0,0)
      // It returns the associate initial values for the left and right foot.
      int EvaluateCOM(VNL::Matrix<double> BodyAngles,
		      double omega, double theta,
		      vector3d &lCOMPosition,
		      vector3d &WaistPosition,
		      FootAbsolutePosition & LeftFootPosition,
		      FootAbsolutePosition & RightFootPosition);

      /// This method returns the final COM pose matrix after the second stage of control.
      VNL::Matrix<double> GetFinalDesiredCOMPose();

      /// This method returns the current waist position in the COM reference 
      /// frame. This can be used with the previous method to get the final Waist 
      /// position.
      VNL::Vector<double> GetCurrentPositionofWaistInCOMFrame();

      /// Returns the last element of the COM FIFO in the first stage of control
      COMPosition GetLastCOMFromFirstStage();

      /// Update the queue of ZMP reference value.
      void UpdateTheZMPRefQueue(ZMPPosition NewZMPRefPos);

      /// Set the algorithm used for ZMP and CoM trajectory.
      void SetAlgorithmForZMPAndCoMTrajectoryGeneration(int anAlgo);

      /// Get the algorithm used for ZMP and CoM trajectory.
      int GetAlgorithmForZMPAndCoMTrajectoryGeneration();
    };
};
#endif /* _ZMPREVIEWCONTROLWITHMULTIBODYZMP_H_ */

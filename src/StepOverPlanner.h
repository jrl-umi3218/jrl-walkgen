/** @doc This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of relative steps.
   If you want to change the reference trajectories, and the planning
   of the foot, this is the object to modify.

   CVS Information:
   $Id: ZMPDiscretization.h,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/ZMPDiscretization.h,v $
   $Log: ZMPDiscretization.h,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
   @author Olivier Stasse,Ramzi Sellouati
   
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

#ifndef _STEPOVER_PLANNER_H_
#define _STEPOVER_PLANNER_H_



#include <VNL/matrix.h>
#include <vector>
#include <string>
#include <ZMPDiscretization.h>
#include <PreviewControl.h>
#include <StepOverPolynome.h>
#include <InverseKinematics.h>
#include <DynamicMultiBody.h>
#include <deque>



//#include <PolynomeFoot.h>

namespace PatternGeneratorJRL
{

class CollisionDetector;
/// Structure to store the obstacle parameters and positions.
  struct ObstaclePar_t
  { 
    double x,y,z,theta; // x, y in meters, theta in DEGREES.
    double h,w,d;       // h,w,d in meters and represent the height, width and depth.
  };
  typedef struct ObstaclePar_t ObstaclePar;

  /// Object to compute new foot trajectories to step over obstacle dynamically
  class StepOverPlanner
    {
      public :
     
      /// Constructor
      StepOverPlanner(ObstaclePar &ObstacleParameters);

      /// Destructor
      ~StepOverPlanner();

	///function which calculates the different relative footholds to be set in function of an obstacle in front
      void CalculateFootHolds(deque<RelativeFootPosition> &FootHolds);
	
	///call for polynomial planning of both steps during the obstacle stepover
	void PolyPlanner(deque<COMPosition> &aCOMBuffer, 
			 deque<FootAbsolutePosition> & aLeftFootBuffer, 
			 deque<FootAbsolutePosition> & aRightFootBuffer,
			 deque<ZMPPosition> & aZMPPositions);
	
	///function which calculates the polynomial coeficients for the first step
        void PolyPlannerFirstStep(deque<FootAbsolutePosition> &aFirstStepOverFootBuffer);
	
	///function which calculates the polynomial coeficients for the first step
	void PolyPlannerSecondStep(deque<FootAbsolutePosition> &aSecondStepOverFootBuffer);

	///function which calculates the polynomial coeficients for the changing COM height
	void PolyPlannerHip();
	
	

	///this sets the extra COM buffer calculated in the ZMPMultybody class
	void SetExtraBuffer(deque<COMPosition> aExtraCOMBuffer, 
			    deque<FootAbsolutePosition> aExtraRightFootBuffer, 
			    deque<FootAbsolutePosition> aExtraLeftFootBuffer);

	///this gets the extra COM buPreviewControlffer calculated in the ZMPMultybody class
	void GetExtraBuffer(deque<COMPosition> &aExtraCOMBuffer, 
			    deque<FootAbsolutePosition> &aExtraRightFootBuffer, 
			    deque<FootAbsolutePosition> &aExtraLeftFootBuffer);


	///this sets the extra COM buffer calculated in the ZMPMultybody class
	void SetFootBuffers(deque<FootAbsolutePosition> aLeftFootBuffer, 
			    deque<FootAbsolutePosition> aRightFootBuffer);

	///this gets the extra COM buffer calculated in the ZMPMultybody class
	void GetFootBuffers(deque<FootAbsolutePosition> & aLeftFootBuffer, 
			    deque<FootAbsolutePosition> & aRightFootBuffer);

	/// Set obstacle information.
      void SetObstacleInformation(ObstaclePar ObstacleParameters);


	/// Set the link to the preview control.
      void SetPreviewControl(PreviewControl *aPC);

	/// Set the link to the ZMP discretization model.
      void SetZMPDiscretization(ZMPDiscretization *aZMPDiscr);      

      /// Set the link with the Dynamic Multi Body model.
      void SetDynamicMultiBodyModel(DynamicMultiBody *aDMB);

      /// Set the link with the inverse kinematics model.
      void SetInverseKinematics(InverseKinematics *anIK);

	///set parameters for the timedistribution of specific stepover points
	void TimeDistributeFactor(vector<double> &TimeDistrFactor);

	///set parameter which decreases the max stepover hipheight used during feasibility calculation
	void SetDeltaStepOverCOMHeightMax(double aDeltaStepOverCOMHeightMax);

	/// create the complete COM and ZMP buffer by the first preview round
	void CreateBufferFirstPreview(deque<COMPosition> &m_COMBuffer,
				      deque<ZMPPosition> &m_ZMPBuffer, 
				      deque<ZMPPosition> &m_ZMPRefBuffer);

	///Calculates the absolute coordinates (ref frame) of a point on the lower legs given in relative coordinates in the locale frame      (whichLeg positive for left leg and negative for right leg)
	void CalcCoordShankLowerLegPoint(VNL::Matrix<double> RelCoord, VNL::Matrix<double> &AbsCoord, VNL::Matrix<double> LegAngles, VNL::Matrix<double> WaistRot,VNL::Matrix<double> WaistPos,int WhichLeg);


	protected:
	
	///this function will calculate a feasible set for the stepleght and hip height during double support over the obstacle
	void DoubleSupportFeasibility();

	///
      ObstaclePar m_ObstacleParameters;

	VNL::Matrix<double> m_ObstaclePosition; //x, y, z position of obstacle in worldframe (point taken on the front plan of the obstacle on the floor and in the middel of the width
	
	VNL::Matrix<double> m_ObstacleRot; //this is the rotationmatrix from obstacle frame to world frame

	VNL::Matrix<double> m_ObstacleRotInv; //this is the rotationmatrix from world frame to obstacle frame

       
      float m_StepOverStepLenght,m_StepOverHipHeight;
	
	
      deque<RelativeFootPosition> m_FootHolds;

      VNL::Matrix<double> mBoundCond;   
      vector<double> mTimeDistr;

         
      StepOverPolynomeHip4 *m_PolynomeStepOverHipStep2;
      StepOverPolynomeHip4 *m_PolynomeStepOverHipRotation;
  

      StepOverClampedCubicSpline *m_ClampedCubicSplineStepOverFootX;
      StepOverClampedCubicSpline *m_ClampedCubicSplineStepOverFootY;	
      StepOverClampedCubicSpline *m_ClampedCubicSplineStepOverFootZ;
      StepOverClampedCubicSpline *m_ClampedCubicSplineStepOverFootOmega;
      StepOverClampedCubicSpline *m_ClampedCubicSplineStepOverFootOmegaImpact;
	
      ///extra COMPosition buffer calculated in ZMPMultibody class 
	deque<COMPosition> m_ExtraCOMBuffer;
	
	///extra foot buffers with the same lenght as extra COM buffer and representing the two stpes over the obstacle
	  deque<FootAbsolutePosition> m_ExtraRightFootBuffer, m_ExtraLeftFootBuffer;
	  
	  
	  
	  /// buffers for first preview
	    deque<COMPosition> m_COMBuffer;
	    deque<ZMPPosition> m_ZMPBuffer;

	///buffer of complete foot course to be changed 
	deque<FootAbsolutePosition> m_RightFootBuffer, m_LeftFootBuffer;

	///buffer of complete ZMP course to be changed 
	deque<ZMPPosition> m_ZMPPositions;

	unsigned int m_StartStepOver;
	unsigned int m_StartDoubleSupp;
	unsigned int m_StartSecondStep;
	unsigned int m_EndStepOver;
	bool m_WhileSpecialSteps;
	unsigned int m_StartPrevStepOver;
	unsigned int m_EndPrevStepOver;
	unsigned int m_StartAfterStepOver;
	unsigned int m_EndAfterStepOver;


	unsigned int m_ExtraBufferLength;
	
	double m_ModulationSupportCoefficient;
	float m_Tsingle,m_TsingleStepOver; 
        float m_Tdble,m_TdbleStepOver;
	float m_TdbleStepOverBeforeAfter,m_TsingleStepOverBeforeAfter;


	double m_tipToAnkle;
	double m_heelToAnkle;
	double m_soleToAnkle;	
	double m_heelDistAfter;
	double m_tipDistBefore;
	
	double m_nominalStepLenght;
	double m_nominalStepWidth;
	double m_NominalCOMStepHeight;
	double m_DeltaStepOverCOMHeightMax;

	double m_KneeAngleBound;

	double m_DiffBetweenComAndWaist;

	double m_WaistRotationStepOver;

	int m_WhoIsFirst;

	VNL::Matrix<double> m_LegLayoutPoint;

	/// Vector from the Waist to the left and right hip.
      VNL::Matrix<double> m_StaticToTheLeftHip, m_StaticToTheRightHip;

      /// Displacement between the hip and the foot.
      VNL::Matrix<double> m_Dt;

	/// Pointer to the Preview Control object.
      PreviewControl *m_PC;

	/// Pointer to the ZMPDiscretization object.
      ZMPDiscretization *m_ZMPDiscr;

      /// Pointer to the Dynamic Multi body model.
      DynamicMultiBody *m_DMB;

      /// Pointer to the Inverse Kinematics model.
      InverseKinematics *m_IK;

 	/// Pointer to the collision detector model.
      CollisionDetector *m_CollDet;	
	
      /// Previous joint values.
      VNL::Matrix<double> m_prev_ql,m_prev_qr;

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

	/// time distribution at which the specific intermediate points for the stepping over splines are to be exerted
	vector<double> m_TimeDistrFactor;
		
    };





};
#include<CollisionDetector.h>
#endif /* _STEPOVER_PLANNER_H_ */

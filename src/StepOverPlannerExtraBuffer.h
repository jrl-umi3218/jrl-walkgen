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
/// Structure to store the obstacle parameters and positions.
  struct ObstaclePar_t
  { 
    double x,y,theta; // x, y in meters, theta in DEGREES.
    double h,w;       // h,w in meters and represent the height and width.
  };
  typedef struct ObstaclePar_t ObstaclePar;

  /// Object to compute new foot trajectories to step over obstacle dynamically
  class StepOverPlanner
    {
      public :
    
      /// Constructor
      StepOverPlanner();

      /// Destructor
      ~StepOverPlanner();

	///function which calculates the different relative footholds to be set in function of an obstacle in front
      void CalculateFootHolds(vector<RelativeFootPosition> &FootHolds,ObstaclePar &ObstacleParameters);
	
	///call for polynomial planning of both steps during the obstacle stepover
	void PolyPlanner();
	
	///function which calculates the polynomial coeficients for the first step
     void PolyPlannerFirstStep(vector<FootAbsolutePosition> &aFirstStepOverFootBuffer);
	
	///function which calculates the polynomial coeficients for the first step
	void PolyPlannerSecondStep(vector<FootAbsolutePosition> &aSecondStepOverFootBuffer);
	
	///function which updates the footposition according to calculated polynomial
	void UpdatePosition(FootAbsolutePosition &aFootPositions,FootAbsolutePosition &iniFootPositions,double LocalTime);

	///this sets the extra COM buffer calculated in the ZMPMultybody class
	void SetExtraBuffer(vector<COMPosition> aExtraCOMBuffer, vector<FootAbsolutePosition> aExtraRightFootBuffer, vector<FootAbsolutePosition> aExtraLeftFootBuffer);

	///this gets the extra COM buffer calculated in the ZMPMultybody class
	void GetExtraBuffer(vector<COMPosition> &aExtraCOMBuffer, vector<FootAbsolutePosition> &aExtraRightFootBuffer, vector<FootAbsolutePosition> &aExtraLeftFootBuffer);


	///this sets the extra COM buffer calculated in the ZMPMultybody class
	void SetFootBuffers(vector<FootAbsolutePosition> aLeftFootBuffer, vector<FootAbsolutePosition> aRightFootBuffer);

	///this gets the extra COM buffer calculated in the ZMPMultybody class
	void GetFootBuffers(vector<FootAbsolutePosition> & aLeftFootBuffer, vector<FootAbsolutePosition> & aRightFootBuffer);

	/// Set the link to the preview control.
      void SetPreviewControl(PreviewControl *aPC);
      
      /// Set the link with the Dynamic Multi Body model.
      void SetDynamicMultiBodyModel(DynamicMultiBody *aDMB);

      /// Set the link with the inverse kinematics model.
      void SetInverseKinematics(InverseKinematics *anIK);

	/// create the complete COM and ZMP buffer by the first preview round
	void CreateBufferFirstPreview(vector<COMPosition> &m_COMBuffer,vector<ZMPPosition> &m_ZMPBuffer, vector<ZMPPosition> &m_ZMPRefBuffer);


	protected:
	
	///
      ObstaclePar m_ObstacleParameters;
       
      float m_StepOverStepLenght,m_StepOverHipHeight;
	
      vector<RelativeFootPosition> m_FootHolds;

      VNL::Matrix<double> mBoundCond;   
      vector<double> mTimeDistr;

      StepOverPolynomeFoot *m_PolynomeStepOverX;
      StepOverPolynomeFoot *m_PolynomeStepOverY;
      StepOverPolynomeFoot *m_PolynomeStepOverZ;
	
	///extra COMPosition buffer calculated in ZMPMultibody class 
	vector<COMPosition> m_ExtraCOMBuffer;

	///extra foot buffers with the same lenght as extra COM buffer and representing the two stpes over the obstacle
	vector<FootAbsolutePosition> m_ExtraRightFootBuffer, m_ExtraLeftFootBuffer;



	/// buffers for first preview
	vector<COMPosition> m_COMBuffer;
	vector<ZMPPosition> m_ZMPBuffer;

	///buffer of complete foot course to be changed 
	vector<FootAbsolutePosition> m_RightFootBuffer, m_LeftFootBuffer;

	unsigned int m_StartStepOver;
	unsigned int m_StartDoubleSupp;
	unsigned int m_StartSecondStep;
	unsigned int m_EndStepOver;
	unsigned int m_ExtraBufferLength;
	double m_SampleTime;

	double m_tipToAnkle;
	double m_heelToAnkle;
	double m_heelDistAfter;
	double m_nominalStepLenght;
	double m_nominalStepWidth;

	/// Pointer to the Preview Control object.
      PreviewControl *m_PC;

      /// Pointer to the Dynamic Multi body model.
      DynamicMultiBody *m_DMB;

      /// Pointer to the Inverse Kinematics model.
      InverseKinematics *m_IK;

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
		
    };





};
#endif _STEPOVER_PLANNER_H_

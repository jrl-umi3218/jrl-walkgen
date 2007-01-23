/** @doc This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of relative steps.
   If you want to change the reference trajectories, and the planning
   of the foot, thOn pageis is the object to modify.


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

#ifndef _FOOT_PRINT_H_
#define _FOOT_PRINT_H_

#ifdef max
#undef max
#undef min
#endif

#include <deque>
#include <string>
#include <MatrixAbstractLayer.h>

//#define FULL_POLYNOME

using namespace::std;

#include <PGTypes.h>

#include <PolynomeFoot.h>
#include <HumanoidSpecificities.h>
#include <ConvexHull.h>
#include <PreviewControl.h>

namespace PatternGeneratorJRL
{


  /// Object to compute the trajectories of the foot, the waist and the arms.
  class ZMPDiscretization
    {
      public :
    
      /// Constructor
	ZMPDiscretization(string DataFile="",
			  HumanoidSpecificities *aHS=0);

      /// Destructor
      ~ZMPDiscretization();

      /// Returns the single support time.
      float GetTSingleSupport();

      /// Set the single support time.
      void SetTSingleSupport(float);

      /// Returns the double support time.
      float GetTDoubleSupport();

      /// Set the double support time.
      void SetTDoubleSupport(float);


	/// Returns the ModulationSupportCoefficient.
      double GetModulationSupportCoefficient();

 	/// Set the ModulationSupportCoefficient.
      void SetModulationSupportCoefficient(double);

      /// Get the sampling period for the control, set to 0.005 by default.
      float GetSamplingPeriod();

      /// Set the sampling period for the control.
      void SetSamplingPeriod(float);

      /// Returns the step height.
      float GetStepHeight();

      /// Specify the step height.
      void SetStepHeight(float);

      /** Generate ZMP discreatization from a vector of foot position.
	  ASSUME A COMPLETE MOTION FROM END TO START, and GENERATE EVERY VALUE.

	  @param RelativeFootPositions: The only entry to this method: the set of 
	  relative steps to be performed by the robot.

	  
	  @return ZMPPositions: Returns the ZMP reference values for the overall motion.
	  Those are absolute position in the world reference frame. The origin is the initial
	  position of the robot. The relative foot position specified are added.

	  @return SupportFootAbsolutePositions: Returns the absolute position of the support
	  foot. Should be the same than the relative foot position but with a SamplingPeriod
	  time precision.

	  @return LeftFootAbsolutePositions: Returns the absolute position of the left foot.
	  According to the macro FULL_POLYNOME the trajectory will follow a third order
	  polynom or a fifth order. By experience it is wise to put a third order. 
	  A null acceleration might cause problem for the compensation of the Z-axis momentum.

	  @return RightFootAbsolutePositions: Returns the absolute position of the right foot.
	  
	  @return LeftHandAbsolutionPositions: Returns the hand absolute position.
	  Currently the output is not usable, see the OpenHRP's plugin instead.

	  @return RightHandAbsolutionPositions: Returns the hand absolute position.
	  Currently the output is not usable, see the OpenHRP's plugin instead.
	  
	  @return Xmax: Returns the maximal distance of a hand along the X axis in the waist coordinates.
	   */
      void GetZMPDiscretization(deque<ZMPPosition> & ZMPPositions,
				deque<FootAbsolutePosition> &SupportFootAbsolutePositions,
				deque<RelativeFootPosition> &RelativeFootPositions,
				deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
				deque<FootAbsolutePosition> &RightFootAbsolutePositions,
				deque<FootAbsolutePosition> &LeftHandAbsolutePositions,
				deque<FootAbsolutePosition> &RightHandAbsolutePositions, 
				double Xmax,
				MAL_S3_VECTOR(& lStartingCOMPosition,double),
				FootAbsolutePosition & InitLeftFootAbsolutePosition,
				FootAbsolutePosition & InitRightFootAbsolutePosition);

      /// Set the Preview control time window.
      void SetTimeWindowPreviewControl(float );
      
      /// Get the preview control time window.
      float GetTimeWindowPreviewControl( );

      /// Set for the foot angle on landing and taking off.
      void SetOmega(float anOmega);

      /// Get the foot angle on landing and taking off.
      float GetOmega(void);

      /// Dump data files.
      void DumpDataFiles(string ZMPFileName, string FootFileName,			       
			 deque<ZMPPosition> &ZMPPositions,
			 deque<FootAbsolutePosition> &FootAbsolutePositions);

      void DumpFootAbsolutePosition(string aFileName,
				    deque<FootAbsolutePosition> &aFootAbsolutePositions);

      /** Update the value of the foot configuration according to the 
	  current situation. */
      void UpdateFootPosition(deque<FootAbsolutePosition> &SupportFootAbsolutePositions,
			      deque<FootAbsolutePosition> &NoneSupportFootAbsolutePositions,
			      int index, int k, int indexinitial, double ModulationSupportTime,int StepType);

      /// IIR filtering of ZMP Position X put in ZMP Position Y.
      void FilterZMPRef(deque<ZMPPosition> &ZMPPositionsX,
			deque<ZMPPosition> &ZMPPositionsY);

      ///ZMP shift parameters to shift ZMP position during Single support with respect to the normal ankle position
      void SetZMPShift(vector<double> &ZMPShift);
	
      /*! Methods for on-line generation. (First version)
	The queues will be updated as follows:
	- The first values necessary to start walking will be inserted.
	- The initial positions of the feet will be taken into account
	according to InitLeftFootAbsolutePosition and InitRightFootAbsolutePosition.
	- The RelativeFootPositions stack will be taken into account,
	- The starting COM Position.
	Returns the number of steps which has been completely put inside 
	the queue of ZMP, and foot positions.
       */
      int InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
                     deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
		     deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
		     FootAbsolutePosition & InitLeftFootAbsolutePosition,
		     FootAbsolutePosition & InitRightFootAbsolutePosition,
		     deque<RelativeFootPosition> &RelativeFootPositions,
		     MAL_S3_VECTOR(& lStartingCOMPosition,double));
    
      /// Methods to update the stack on-line by inserting a new foot position.
      void OnLine(RelativeFootPosition NewRelativeFootPosition,
		  deque<ZMPPosition> & FinalZMPPositions,					     
		  deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
		  deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
		  bool EndSequence);

      /// Update the current support foot posture using the relative support foot postion RFP.
      void UpdateCurrentSupportFootPosition(RelativeFootPosition aRFP);

      /// End phase of the walking.
      void EndPhaseOfTheWalking(  deque<ZMPPosition> &ZMPPositions,
				  deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
				  deque<FootAbsolutePosition> &RightFootAbsolutePositions);
      
      /*! This method builds a set of linear constraint inequalities based
	on the foot trajectories given as an input.
	The result is a set Linear Constraint Inequalities. */
      int BuildLinearConstraintInequalities(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					    deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					    deque<LinearConstraintInequality_t *> & QueueOfLConstraintInequalities,
					    double ConstraintOnX,
					    double ConstraintOnY);
      int BuildLinearConstraintInequalities2(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					     deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					     deque<LinearConstraintInequality_t *> & QueueOfLConstraintInequalities,
					     double ConstraintOnX,
					     double ConstraintOnY);
					     

      /*! This method is a new way of computing the ZMP trajectory from
	foot trajectory. */
      int BuildZMPTrajectoryFromFootTrajectory(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					       deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					       deque<ZMPPosition> &ZMPRefPositions,
					       deque<ZMPPosition> &NewFinalZMPPositions,
					       deque<COMPosition> &COMPositions,
					       double ConstraintOnX,
					       double ConstraintOnY,
					       double T,
					       unsigned int N);

      /*! Build the necessary matrices for the QP problem under linear inequality constraints. */
      int BuildMatricesPxPu(double * &Px, double * &Pu,
			    unsigned N, double T,
			    double StartingTime,
			    deque<LinearConstraintInequality_t *> 
			    & QueueOfLConstraintInequalities,
			    double Com_Height,
			    unsigned int &NbOfConstraints,
			    MAL_VECTOR(&xk,double));

      /*! This method helps to build a linear system for constraining the ZMP. */
      int ComputeLinearSystem(vector<CH_Point> aVecOfPoints,
			      MAL_MATRIX(&A,double),
			      MAL_MATRIX(&B,double));

   protected:

      /// Time for single support.
      float m_Tsingle;
  
      /// Time for double support.
      float m_Tdble;

      /// Sampling period
      float m_SamplingPeriod;

	///ModulationSupportCoefficient coeeficient to wait a little before foot is of the ground
      double m_ModulationSupportCoefficient;


      /// The foot orientation for the lift off and the landing
      float m_Omega;

      /// Preview control window in second.
      float m_PreviewControlTime;

      /// Step height for the walking pattern.
      float m_StepHeight;

	


      /// Polynome to generate trajectories.
#ifdef FULL_POLYNOME
      Polynome3 *m_PolynomeX,*m_PolynomeY;
      Polynome3 *m_PolynomeTheta, *m_PolynomeOmega, *m_PolynomeOmega2;
      Polynome6 *m_PolynomeZ;
      Polynome3 *m_PolynomeZMPTheta;
#else

      Polynome3 *m_PolynomeX,*m_PolynomeY;
      Polynome3 *m_PolynomeTheta;
      Polynome3 *m_PolynomeOmega, *m_PolynomeOmega2;
      Polynome4 *m_PolynomeZ;
      Polynome3 *m_PolynomeZMPTheta;
#endif  

	

	///ZMP shift parameters to shift ZMP position during Single support with respect to the normal ankle position

	vector<double> m_ZMPShift;
 	//double m_ZMPShift3Begin, m_ZMPShift3End;
	//double m_ZMPShift4Begin, m_ZMPShift4End;

	//double m_ZMPShift3BeginY, m_ZMPShift3EndY;
	//double m_ZMPShift4BeginY, m_ZMPShift4EndY;
	
    // Neutral ZMP position.
    double m_ZMPNeutralPosition[2];

    // Current absolute orientation of the Waist.
    double m_CurrentAbsTheta;
    // Current orientation of the support foot.
    double m_CurrentTheta;

    // Current absolute support position in 2D (but 
    // with homogeneous coordinates).
    MAL_MATRIX(m_CurrentSupportFootPosition,double);

    // Window for the filtering of the ZMP positions..
    vector<double> m_ZMPFilterWindow;

    // Keep a stack of two steps as a reference before sending them to the 
    // external queues.
    deque<RelativeFootPosition> m_RelativeFootPositions;

    // Keep track of the time.
    double m_CurrentTime;

    // Keep track of the previous foot support position.
    MAL_MATRIX(m_vdiffsupppre,double);

    // Keep an object which relates the specificities
    // with an abstract layer.
    HumanoidSpecificities *m_HS;
    
    /// Matrices for the dynamical system.
    MAL_MATRIX(m_A,double);
    MAL_MATRIX(m_B,double);
    MAL_MATRIX(m_C,double);
      
      
    /// Values for the foot dimensions.
    double m_FootB, m_FootH, m_FootF;

   };
};
#include <PreviewControl.h>
#endif /* _FOOT_PRINT_H_*/

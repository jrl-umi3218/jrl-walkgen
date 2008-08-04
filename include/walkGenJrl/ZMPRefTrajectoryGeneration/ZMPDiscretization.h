/*!\file ZMPDiscretization.h
  \brief This class generate all the values for the foot trajectories,
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

#ifndef _ZMP_DISCRETIZATION_H_
#define _ZMP_DISCRETIZATION_H_

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <deque>
#include <string>
//#define FULL_POLYNOME

using namespace::std;



#include <dynamicsJRLJapan/HumanoidSpecificities.h>

#include <walkGenJrl/Mathematics/PolynomeFoot.h>
#include <walkGenJrl/Mathematics/ConvexHull.h>
#include <walkGenJrl/PGTypes.h>
#include <walkGenJrl/PreviewControl/PreviewControl.h>
#include <walkGenJrl/ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.h>
#include <walkGenJrl/FootTrajectoryGeneration/FootTrajectoryGenerationStandard.h>

using namespace dynamicsJRLJapan;
namespace PatternGeneratorJRL
{


  /*! \brief Class to compute the trajectories of the ZMP reference trajectory 
    following Kajita's heuristic. 
    Basically during single support phase, the ZMP is at the center of
    the support foot. During double support phase, the ZMP is 
    on the line linking the two centers of each foot. 
  */
  class ZMPDiscretization: public ZMPRefTrajectoryGeneration
    {
      public :
    
      /*!  Constructor */
      ZMPDiscretization(SimplePluginManager *lSPM,string DataFile="",HumanoidSpecificities *aHS=0);
      
      /*!  Destructor */
      ~ZMPDiscretization();
      

      /** Generate ZMP discreatization from a vector of foot position.
	  ASSUME A COMPLETE MOTION FROM END TO START, and GENERATE EVERY VALUE.
	  
	  @param[out] ZMPPositions: Returns the ZMP reference values for the overall motion.
	  Those are absolute position in the world reference frame. The origin is the initial
	  position of the robot. The relative foot position specified are added.

	  @param[out] CoMPositions: Returns the COM reference values for the overall motion.
	  Those are absolute position in the world reference frame. The origin is the initial
	  position of the robot. The relative foot position specified are added.

	  @param[in] RelativeFootPositions: The set of 
	  relative steps to be performed by the robot.

	  @param[out] LeftFootAbsolutePositions: Returns the absolute position of the left foot.
	  According to the macro FULL_POLYNOME the trajectory will follow a third order
	  polynom or a fifth order. By experience it is wise to put a third order. 
	  A null acceleration might cause problem for the compensation of the Z-axis momentum.

	  @param[out] RightFootAbsolutePositions: Returns the absolute position of the right foot.
	  	  
	  @param[in] Xmax: The maximal distance of a hand along the X axis in the waist coordinates.

	  @param[in] lStartingCOMPosition: The initial position of the CoM.
	  
	  @param[in] InitLeftFootAbsolutePosition: The initial position of the left foot.
	  
	  @param[in] InitRightFootAbsolutePosition: The initial position of the right foot.

	   */
      void GetZMPDiscretization(deque<ZMPPosition> & ZMPPositions,
				deque<COMPosition> & CoMPositions,
				deque<RelativeFootPosition> &RelativeFootPositions,
				deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
				deque<FootAbsolutePosition> &RightFootAbsolutePositions,
				double Xmax,
				COMPosition & lStartingCOMPosition,
				FootAbsolutePosition & InitLeftFootAbsolutePosition,
				FootAbsolutePosition & InitRightFootAbsolutePosition);
      /*! Dump data files. */
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

      /*! IIR filtering of ZMP Position X put in ZMP Position Y. */
      void FilterZMPRef(deque<ZMPPosition> &ZMPPositionsX,
			deque<ZMPPosition> &ZMPPositionsY);

      /*! ZMP shift parameters to shift ZMP position during Single support with respect to the normal ankle position */
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
		     deque<COMPosition> & CoMPositions,
                     deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
		     deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
		     FootAbsolutePosition & InitLeftFootAbsolutePosition,
		     FootAbsolutePosition & InitRightFootAbsolutePosition,
		     deque<RelativeFootPosition> &RelativeFootPositions,
		     COMPosition & lStartingCOMPosition);
      
      /*! \brief  Methods to update the stacks on-line. */
      void OnLine(double time,
		  deque<ZMPPosition> & FinalZMPPositions,					     
		  deque<COMPosition> & CoMPositions,
		  deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
		  deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions);

      /*! \brief  Methods to update the stack on-line by inserting a new foot position. */
      void OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
			 deque<ZMPPosition> & FinalZMPPositions,					     
			 deque<COMPosition> & CoMPositions,
			 deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
			 deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
			 bool EndSequence);

      /* ! \brief Method to change on line the landing position of a foot.
	 @return If the method failed it returns -1, 0 otherwise.
      */
      int OnLineFootChange(double time,
			   FootAbsolutePosition &aFootAbsolutePosition,
			   deque<ZMPPosition> & FinalZMPPositions,			     
			   deque<COMPosition> & CoMPositions,
			   deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
			   deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
			   StepStackHandler * aStepStackHandler=0);

      /*! \brief Return the time at which it is optimal to regenerate a step in online mode. 
       */
      int ReturnOptimalTimeToRegenerateAStep();
      
      /// Update the current support foot posture using the relative support foot postion RFP.
      void UpdateCurrentSupportFootPosition(RelativeFootPosition aRFP);

      /// End phase of the walking.
      void EndPhaseOfTheWalking(  deque<ZMPPosition> &ZMPPositions,
				  deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
				  deque<FootAbsolutePosition> &RightFootAbsolutePositions);
      
      
      /*! Set the ZMP neutral position in the global coordinates system */
      void setZMPNeutralPosition(const double aZMPNeutralPosition[2])
      { 
	m_ZMPNeutralPosition[0] = aZMPNeutralPosition[0];
	m_ZMPNeutralPosition[1] = aZMPNeutralPosition[1];
      }


   protected:


      /* ! ModulationSupportCoefficient coeeficient to wait a little before foot is of the ground */
      double m_ModulationSupportCoefficient;

      /* ! Polynome to generate trajectories. */
      Polynome3 *m_PolynomeZMPTheta;
      
      /* ! ZMP shift parameters to shift ZMP position during 
	 Single support with respect to the normal ankle position */
      vector<double> m_ZMPShift;
      
      /*! Neutral ZMP position. */
      double m_ZMPNeutralPosition[2];
      
      /* ! Current absolute orientation of the Waist. */
      double m_CurrentAbsTheta;
      
      /* ! Current orientation of the support foot. */
      double m_CurrentTheta;
      
      /* ! Current absolute support position in 2D (but 
	 with homogeneous coordinates). */
      MAL_MATRIX(m_CurrentSupportFootPosition,double);
      
      /* ! Window for the filtering of the ZMP positions.. */
      vector<double> m_ZMPFilterWindow;
      
      /* ! Keep a stack of two steps as a reference before sending them to the 
      external queues. */
      deque<RelativeFootPosition> m_RelativeFootPositions;
      
      /* ! Keep track of the time. */
      double m_CurrentTime;

      /* ! Keep track of the previous foot support position. */
      MAL_MATRIX(m_vdiffsupppre,double);
      
      /*!  Keep an object which relates the specificities
	with an abstract layer. */
      HumanoidSpecificities *m_HS;
      
      /* !  Matrices for the dynamical system. */
      MAL_MATRIX(m_A,double);
      MAL_MATRIX(m_B,double);
      MAL_MATRIX(m_C,double);
      
      /*! Object to handle foot trajectory generation */
      FootTrajectoryGenerationStandard * m_FootTrajectoryGenerationStandard;

   };
};
#include <walkGenJrl/PreviewControl/PreviewControl.h>
#endif /* _FOOT_PRINT_H_*/

/*! \file LeftAndRightFootTrajectoryGenerationMultiple.h
   \brief This class is a container for two analytical trajectories.

   @ingroup foottrajectorygeneration


   Copyright (c) 2007, 
   @author Olivier Stasse,

   $Id$
   
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

#ifndef _LEFT_AND_RIGHT_FOOT_TRAJECTORY_GENERATION_MULTIPLE_H_
#define _LEFT_AND_RIGHT_FOOT_TRAJECTORY_GENERATION_MULTIPLE_H_

/* Dynamics JRL Japan inclusion */
#include <dynamicsJRLJapan/HumanoidSpecificities.h>

/* Walking Pattern Generator inclusion */
#include <walkGenJrl/SimplePlugin.h>
#include <walkGenJrl/FootTrajectoryGeneration/FootTrajectoryGenerationMultiple.h>

namespace PatternGeneratorJRL
{

  /*! @ingroup foottrajectorygeneration
      The main goal of this class is to provide a simple interface to the
      objects derived from ZMPRefTrajectoryGeneration to generate the 
      foot trajectories.
      
      It acts as a container for two FootTrajectoryGenerationMultiple objects
      which handle several polynomials trajectory generation for the right and left
      feet.

      It provides an initialization of the underlying objects assuming that
      some basic informations have been provided: single support time, double support
      time, omega, step height.
      
      The information used follow the previously defined script like language,
      but it could be extended for a clear separation between the two feet.

   */
  class LeftAndRightFootTrajectoryGenerationMultiple : public SimplePlugin
    {

    public: 
      /*! \brief The constructor initialize the plugin part, and the data related to the humanoid. */
      LeftAndRightFootTrajectoryGenerationMultiple(SimplePluginManager * lSPM,
						   dynamicsJRLJapan::HumanoidSpecificities * lHS);

      /*! \brief Memory release. */
      ~LeftAndRightFootTrajectoryGenerationMultiple();

      /*! \brief Reimplementation of the call method for the plugin manager. 
	More explicitly this object will deal with the call which initialize
	the feet behaviors (\f$omega\f$, \f$ stepheight \f$) .
       */
      virtual void CallMethod(std::string &Method, std::istringstream &strm);

      /*! \brief Initialize the analytical feet trajectories from a set of relative step.
	It is based on the parameters given through the interface. All the settings are done internally.
	The only output is the set of absolute positions for the support foot.
	@param[in] RelativeFootPositions: The set of relative positions for the support foot.
	@param[in] LeftFootInitialPosition: The initial position of the left foot.
	@param[in] RightFootInitialPosition: the initial position of the right foot.
	@param[out] SupportFootAbsoluteFootPositions: The set of absolute foot positions
	corresponding to the set of relative foot positions (i.e given step by step 
	and not every sampled control time).
      */
      void InitializeFromRelativeSteps(deque<RelativeFootPosition> &RelativeFootPositions,
				       FootAbsolutePosition &LeftFootInitialPosition,
				       FootAbsolutePosition &RightFootInitialPosition,
				       deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions,
				       bool IgnoreFirst, bool Continuity);
      
      /*! \brief Method to compute the absolute position of the foot.
	@param[in] LeftOrRight: -1 indicates the right foot, 1 indicates the left foot.
	@param[in] time: The absolute time to be compared with the absolute reference defining the start
	of the trajectory.
	@param[out] aFootAbsolutePosition: The data structure to be filled with the information
	\f$ (x,y,z,\omega, \omega_2, \theta) \f$.
      */
      void ComputeAnAbsoluteFootPosition(int LeftOrRight, double time, FootAbsolutePosition & aFootAbsolutePosition);

      /*! \brief Method to compute the absolute position of the foot.
	@param[in] LeftOrRight: -1 indicates the right foot, 1 indicates the left foot.
	@param[in] time: The absolute time to be compared with the absolute reference defining the start
	of the trajectory.
	@param[out] aFootAbsolutePosition: The data structure to be filled with the information
	\f$ (x,y,z,\omega, \omega_2, \theta) \f$.	
	@param[in] IndexInterval: On which interval to compute the foot position.
	
      */
      void ComputeAnAbsoluteFootPosition(int LeftOrRight, double time, FootAbsolutePosition & aFootAbsolutePosition,
					 unsigned int IndexInterval);


      /*! \brief Method to compute absolute feet positions from a set of relative one.
	@param[in] RelativeFootPositions: The set of relative positions for the support foot.
	@param[in] LeftFootInitialPosition: The initial position of the left foot.
	@param[in] RightFootInitialPosition: the initial position of the right foot.
	@param[out] SupportFootAbsoluteFootPositions: The set of absolute foot positions
	corresponding to the set of relative foot positions (i.e given step by step 
	and not every sampled control time).
       */
      void ComputeAbsoluteStepsFromRelativeSteps(deque<RelativeFootPosition> &RelativeFootPositions,
						 FootAbsolutePosition &LeftFootInitialPosition,
						 FootAbsolutePosition &RightFootInitialPosition,
						 deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions);

      /*! \brief Method to compute absolute feet positions from a set of relative one.
	@param[in] RelativeFootPositions: The set of relative positions for the support foot.
	@param[in] SupportFootInitialPosition: The initial position of the support foot.
	@param[out] SupportFootAbsoluteFootPositions: The set of absolute foot positions
	corresponding to the set of relative foot positions (i.e given step by step 
	and not every sampled control time).
       */
      void ComputeAbsoluteStepsFromRelativeSteps(deque<RelativeFootPosition> &RelativeFootPositions,
						 FootAbsolutePosition &SupportFootInitialPosition,
						 deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions);

      /*! \brief Method to compute relative feet positions from a set of absolute one
	where one has changed.
	@param[in] RelativeFootPositions: The set of relative positions for the support foot.
	@param[in] ChangedInterval: The interval where the absolute foot position has been changed.
	@param[out] SupportFootAbsoluteFootPositions: The set of absolute foot positions
	corresponding to the set of relative foot positions (i.e given step by step 
	and not every sampled control time).
       */
      void ChangeRelStepsFromAbsSteps(deque<RelativeFootPosition> &RelativeFootPositions,
				      FootAbsolutePosition &SupportFootInitialPosition,
				      deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions,
				      unsigned int ChangedInterval);

     protected:

      /*! Internal method to modify the value of an interval. */
      void SetAnInterval(unsigned int IntervalIndex,
			 FootTrajectoryGenerationMultiple * aFTGM,
			 FootAbsolutePosition &FootInitialPosition,
			 FootAbsolutePosition &FootFinalPosition);

      /*! Left Foot Trajectory Generation object for several intervals. */
      FootTrajectoryGenerationMultiple * m_LeftFootTrajectory;
      
      /*! Right Foot Trajectory Generation object for several intervals. */
      FootTrajectoryGenerationMultiple * m_RightFootTrajectory;

      /*! Humanoid specificities object handler */
      dynamicsJRLJapan::HumanoidSpecificities * m_HS;
      
      /*! Set of time intervals */
      vector<double> m_DeltaTj;

      /*! Omega */
      double m_Omega;
      
      /*! Step height. */
      double m_StepHeight;

      /*! Single support time. */
      double m_SingleSupportTime;
      
      /*! Double support time. */
      double m_DoubleSupportTime;
      
    public:
      /*! Set the intervals time */
      void SetDeltaTj(vector<double> & aDeltaTj);

      /*! Get the intervals time */
      void GetDeltaTj(vector<double> & aDeltaTj);

      /*! Set the step height */
      void SetStepHeight(double aStepHeight);

      /*! Get the step height. */
      double GetStepHeight();      

      /*! \name Methods related to the time reference.
	@{ */

      /*! \brief This returns the absolute time reference. 
      As the object manipulates several trajectories generator,
      the coherency of the returned informations has to be checked.
      If this is not the case, i.e. the trajectory generators have different
      absolute time references, the method returns -1. */
      double GetAbsoluteTimeReference();

      /*! \brief Set the time reference for all the trajectory generators. */
      void SetAbsoluteTimeReference(double anAbsoluteTimeReference);

      /*! @} */

      
    };
}
#endif /* _LEFT_AND_RIGHT_FOOT_TRAJECTORY_GENERATION_MULTIPLE_H_ */

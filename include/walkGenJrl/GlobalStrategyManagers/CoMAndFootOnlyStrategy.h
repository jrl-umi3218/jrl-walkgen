/*! \file CoMAndFootOnlyStrategy.h
  \brief This object defines a global strategy object to generate 
  only foot, ZMP reference and CoM trajectories position every 5 ms.

  Copyright (c) 2007, 
  @author Francois Keith, Olivier Stasse
   
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

#include <walkGenJrl/walkGenJrl_API.h>
#include <walkGenJrl/SimplePlugin.h>
#include <walkGenJrl/PGTypes.h>

#include <walkGenJrl/MotionGeneration/ComAndFootRealization.h>
#include <walkGenJrl/GlobalStrategyManagers/GlobalStrategyManager.h>

#ifndef _COM_AND_FOOT_ONLY_STRATEGY_H_
#define _COM_AND_FOOT_ONLY_STRATEGY_H_
namespace PatternGeneratorJRL
{

  /** @ingroup pgjrl
      Implementation of the buffers handling without preview control
  */
  class WALK_GEN_JRL_EXPORT CoMAndFootOnlyStrategy: public GlobalStrategyManager
  {
    
  public:

    /*! Default constructor. */
    CoMAndFootOnlyStrategy(SimplePluginManager *aSimplePluginManager);

    /*! Default destructor. */
    ~CoMAndFootOnlyStrategy();

    /*! \name Reimplement the interface inherited from Global Strategy Manager 
     @{
    */

    /*! Perform a 5 ms step to generate the necessary information.
      \note{The meaning and the way to use this method depends on the child class}.

      @param[out] LeftFootPosition: The position of the Left Foot position.
      @param[out] RightFootPosition: The position of the Right Foot position.
      @param[out] ZMPRefPos: The ZMP position to be feed to the controller, in the waist 
      frame reference.
      @param[out] finalCOMPosition: returns position, velocity and acceleration of the CoM.
      @param[out] CurrentConfiguration: The results is a state vector containing the articular positions.
      @param[out] CurrentVelocity: The results is a state vector containing the speed.
      @param[out] CurrentAcceleration: The results is a state vector containing the acceleration.
    */
    int OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
			       FootAbsolutePosition &RightFootPosition,
			       MAL_VECTOR(,double) & ZMPRefPos,
			       COMPosition & finalCOMPosition,
			       MAL_VECTOR(,double) & CurrentConfiguration,
			       MAL_VECTOR(,double) & CurrentVelocity,
			       MAL_VECTOR(,double) & CurrentAcceleration);
    

    
    /*! Computes the COM of the robot with the Joint values given in BodyAngles,
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
			      COMPosition & aStartingCOMPosition,
			      MAL_S3_VECTOR(& ,double) aStartingZMPPosition,
			      FootAbsolutePosition & InitLeftFootPosition,
			      FootAbsolutePosition & InitRightFootPosition);




    /*! This method returns :
      \li -1 if there is no more motion to realize 
      \li 0 if a new step is needed,
      \li 1 if there is still enough steps inside the internal stack.
    */
    int EndOfMotion();

    /*! @} */

    /*! Methods related to the end of the motion.
     @{ 
    */
    /* Fix the end of the buffer to be tested. */
    void SetTheLimitOfTheBuffer(unsigned int lBufferSizeLimit);
    
    /*! @} */


    /*! Reimplement the Call method for SimplePlugin part */
    void CallMethod(std::string &Method, std::istringstream &astrm);
    
    /*! */
    void Setup(deque<ZMPPosition> & aZMPPositions,
	       deque<COMPosition> & aCOMBuffer,
	       deque<FootAbsolutePosition> & aLeftFootAbsolutePositions,
	       deque<FootAbsolutePosition> & aRightFootAbsolutePositions);

    /*! \brief Initialization of the inter objects relationship. */
    int InitInterObjects(PreviewControl * aPC,
			 CjrlHumanoidDynamicRobot * aHDR,
			 ComAndFootRealization * aCFR,
			 StepStackHandler * aSSH);

  protected:
    /*! Count the number of successives hits on the bottom of the buffers. */
    int m_NbOfHitBottom;

    /*! Keeps a link towards an object allowing to find a pose for a given CoM and
      foot position. */
    ComAndFootRealization *m_ComAndFootRealization;

    /*! Set the position of the buffer size limit. */
    unsigned m_BufferSizeLimit;
  };
};
#endif /* _COM_AND_FOOT_ONLY_STRATEGY_H_ */

/*! \file DoubleStagePreviewControlStrategy.h
  \brief This object defines a global strategy object to generate 
  full body position every 5 ms over a preview window. 
  It implements Kajita's algorithm presented in \ref Kajita2003
  
  
  Copyright (c) 2007, 
  @author Olivier Stasse
   
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

#include <walkGenJrl/SimplePlugin.h>
#include <walkGenJrl/GlobalStrategyManagers/GlobalStrategyManager.h>
#include <walkGenJrl/PreviewControl/ZMPPreviewControlWithMultiBodyZMP.h>

#ifndef _DOUBLE_STAGE_PREVIEW_CONTROL_STRATEGY_H_
#define _DOUBLE_STAGE_PREVIEW_CONTROL_STRATEGY_H_
namespace PatternGeneratorJRL
{

  /** @ingroup pgjrl
      Implementation of the buffers handling for Kajita's 
      algorithm.
  */
  class DoubleStagePreviewControlStrategy : public GlobalStrategyManager
  {
    
  public:
    /*! Default constructor. */
    DoubleStagePreviewControlStrategy(SimplePluginManager *aSimplePluginManager);

    /*! Default destructor. */
    ~DoubleStagePreviewControlStrategy();
    
    /*! Perform a 5 ms step to generate the necessary information.
      \note{The meaning and the way to use this method depends on the child class}.

      @param[out] LeftFootPosition: The position of the Left Foot position.
      @param[out] RightFootPosition: The position of the Right Foot position.
      @param[out] ZMPRefPos: The ZMP position to be feed to the controller, in the waist 
      frame reference.
      @param[out] COMPosition: returns position, velocity and acceleration of the CoM.
      @param[out] CurrentConfiguration: The results is a state vector containing the articular positions.
      @param[out] CurrentVelocity: The results is a state vector containing the speed.
    */
    int OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
			       FootAbsolutePosition &RightFootPosition,
			       MAL_VECTOR(,double) & ZMPRefPos,
			       COMPosition & COMPosition,
			       MAL_VECTOR(,double) & CurrentConfiguration,
			       MAL_VECTOR(,double) & CurrentVelocity);
    

    
    /*! Computes the COM of the robot with the Joint values given in BodyAngles,
      velocities set to zero, and returns the values of the COM in aStaringCOMPosition.
      Assuming that the waist is at (0,0,0)
      it returns the associate initial values for the left and right foot.
      @param[in] BodyAngles: 4x4 matrix of the robot's root (most of the time, the waist)
      pose (position + orientation).
      @param[out] aStartingCOMPosition: Returns the 3D position of the CoM for the current
      position of the robot.
      @param[out] InitLeftFootPosition: Returns the position of the left foot in
      the waist coordinates frame.
      @param[out] InitRightFootPosition: Returns the position of the right foot
      in the waist coordinates frame.
    */
    int EvaluateStartingState(MAL_VECTOR( &,double) BodyAngles,
			      COMPosition & aStartingCOMPosition,
			      FootAbsolutePosition & InitLeftFootPosition,
			      FootAbsolutePosition & InitRightFootPosition);

    /*! \brief Setter for the stack handler. */
    void SetStepStackHandler(StepStackHandler *aSSH)
    { m_StepStackHandler = aSSH; }

    /*! \brief Initialization of the inter objects relationship. */
    int InitInterObjects(PreviewControl * aPC,
			 CjrlHumanoidDynamicRobot * aHDR,
			 ComAndFootRealization * aCFR,
			 StepStackHandler * aSSH);
    
    /*! This method returns :
      \li -1 if there is no more motion to realize 
      (i.e. the stack of ZMP has a size \f$\le 2NL \f$),
      \li 0 if a new step is needed,
      (i.e. the stack of ZMP has a size $= 2NL$),
      \li 1 if there is still enough steps inside the internal stack.
            (i.e. the stack of ZMP has a size \f$\geq 2NL\f$).
    */
    int EndOfMotion();

    /*! \brief Reimplement the Call method for SimplePlugin part */
    void CallMethod(std::string &Method, std::istringstream &astrm);

    /*! \brief This method specify the algorithm to use for generating
      a ZMP and/or a CoM trajectory. This has some impact on our class.
     */
    void SetAlgoForZMPTraj(istringstream &strm);

    /*! \brief  Prepare the buffers at the beginning of the foot positions. 
      @param[out] aZMPositions: ZMP position reference
      @param[out] aCOMBuffer: COM trajectory 
      @param[out] aLeftFootAbsolutePositions: Trajectory of absolute positions for the left foot.
      @param[out] aRightFootAbsolutePositions: Trajectory of absolute positions for the right foot.
     */
    void Setup(deque<ZMPPosition> & aZMPositions,
	       deque<COMPosition> & aCOMBuffer,
	       deque<FootAbsolutePosition> & aLeftFootAbsolutePositions,
	       deque<FootAbsolutePosition> & aRightFootAbsolutePositions );

    /*! \brief Get Waist state. */
    bool getWaistState(WaistState & aWaistState);

  protected:
    
    /*! \brief The object to be used to perform one step of control, 
      and generates the corrected CoM trajectory. */
    ZMPPreviewControlWithMultiBodyZMP *m_ZMPpcwmbz;
    
    /*! \brief Pointer to the stack handler. */
    StepStackHandler *m_StepStackHandler;

    /*! \brief Keep the waist position using a CoM position data structure . */
    COMPosition m_CurrentWaistState;

    /*! \brief The size of the preview control window */
    unsigned int m_NL;

    /*! \brief The time of the preview control */
    double m_PreviewControlTime;
    
    /*! Pointer to the Preview Control object. */
    PreviewControl *m_PC;
    
  };
};
#endif /* _DOUBLE_STAGE_PREVIEW_CONTROL_STRATEGY_H_*/

/*! \file GlobalStrategyManager.h
  \brief This object defines a global strategy abstract object to generate an output
  handled by the PatternGeneratorInterface object.
  
  
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
#include <deque> 


/*! JRL inclusion */

// MAL 
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

// Dynamics
#include <robotDynamics/jrlHumanoidDynamicRobot.h>

// PG
#include <walkGenJrl/walkGenJrl_API.h>
#include <walkGenJrl/PGTypes.h>
#include <walkGenJrl/SimplePlugin.h>
#include <walkGenJrl/PreviewControl/PreviewControl.h>

/*! End of JRL inclusion */

#ifndef _GLOBAL_STRATEGY_MANAGER_H_
#define _GLOBAL_STRATEGY_MANAGER_H_

namespace PatternGeneratorJRL
{

  /** @ingroup pgjrl
      
  Abstract interface to the object handling the global strategy
  to generate the output of the pattern generator.
  */
  class WALK_GEN_JRL_EXPORT GlobalStrategyManager : public SimplePlugin
  {
    
  public:
    static const int MOTION_FINISHED = -1;
    static const int NEW_STEP_NEEDED =0;
    static const int DATA_PRESENT= 1;
    /*! Default constructor. */
    GlobalStrategyManager(SimplePluginManager *aSPM);

    /*! Default destructor. */
    virtual ~GlobalStrategyManager()
      {};
    
    /*! Set the link to the preview control. */
    void SetPreviewControl(PreviewControl *aPC);



    /*! Perform a 5 ms step to generate the necessary information.
      \note{The meaning and the way to use this method depends on the child class}.

      @param[out] LeftFootPosition: The position of the Left Foot position.
      @param[out] RightFootPosition: The position of the Right Foot position.
      @param[out] ZMPRefPos: The ZMP position to be feed to the controller.
      @param[out] COMPosition: returns position, velocity and acceleration of the CoM.
      @param[out] CurrentConfiguration: The results is a state vector containing the articular positions.
      @param[out] CurrentVelocity: The results is a state vector containing the speed.
      @param[out] CurrentAcceleration: The results is a state vector containing the acceleration.
    */
    virtual int OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
				       FootAbsolutePosition &RightFootPosition,
				       MAL_VECTOR(,double) & ZMPRefPos,
				       COMPosition & COMPosition,
				       MAL_VECTOR(,double) & CurrentConfiguration,
				       MAL_VECTOR(,double) & CurrentVelocity,
				       MAL_VECTOR(,double) & CurrentAcceleration)=0;


    
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
    virtual int EvaluateStartingState(MAL_VECTOR( &,double) BodyAngles,
				      COMPosition & aStartingCOMPosition,
				      MAL_S3_VECTOR(& ,double) aStartingZMPPosition,
				      MAL_VECTOR(& ,double) aStartingWaistPose,
				      FootAbsolutePosition & InitLeftFootPosition,
				      FootAbsolutePosition & InitRightFootPosition)=0;


    /*! \brief Method to detect the status regarding the end of the motion.
      This method returns :
      \li -1 if there is no more motion to realize,
      \li 0 if a new step is needed
      \li 1 if there is still enough steps inside the internal stack.
    */
    virtual int EndOfMotion()=0;

    /*! \brief Setting the pointers towards buffers of positions. 
      \param[in] aZMPositions: Absolute frame positions buffer of the Zero Momentum Point reference.
      \param[in] aCOMBuffer: Absolute frame positions buffer of the CoM trajectory related to the previous
      ZMP reference trajectory.
      \param[in] aLeftFootAbsolutePositions: Absolute frame positions buffer of the left foot.
      \param[in] aRightFootAbsolutePositions: Absolute frame positions buffer of the right foot.
    */
    void SetBufferPositions(deque<ZMPPosition> * aZMPositions,
			    deque<COMPosition> * aCOMBuffer,
			    deque<FootAbsolutePosition> *aLeftFootAbsolutePositions,
			    deque<FootAbsolutePosition> *aRightFootAbsolutePositions );

    /*! Prepare the buffers at the beginning of the foot positions. */
    virtual void Setup(deque<ZMPPosition> & aZMPositions,
		       deque<COMPosition> & aCOMBuffer,
		       deque<FootAbsolutePosition> & aLeftFootAbsolutePositions,
		       deque<FootAbsolutePosition> & aRightFootAbsolutePositions )=0;
      
  protected:

    /*! \name Positions buffers.
      @{
     */
    
    /*! Buffer of ZMP positions */
    deque<ZMPPosition> * m_ZMPPositions;

    /*! Buffer for the COM position. */
    deque<COMPosition> * m_COMBuffer;

    /*! Buffer of absolute foot position. */
    deque<FootAbsolutePosition> * m_LeftFootPositions, *m_RightFootPositions;
    
    /* @} */
    

    /*! Sampling Period. */
    double m_SamplingPeriod;

    /*! Store the index for the algorithm to use for ZMP and CoM trajectory. */
    int m_ZMPCoMTrajectoryAlgorithm;

    /*! Reference to the humanoid structure. */
    CjrlHumanoidDynamicRobot *m_HumanoidDynamicRobot ;

  public:

    /*! \name Setter and getter for the jrlHumanoidDynamicRobot object. */
    /*! @param[in] aHumanoidDynamicRobot: an object able to compute dynamic parameters
      of the robot. */
    inline  bool setHumanoidDynamicRobot(const CjrlHumanoidDynamicRobot *aHumanoidDynamicRobot)
      { m_HumanoidDynamicRobot = (CjrlHumanoidDynamicRobot *)aHumanoidDynamicRobot;
	return true;}
    
    /*! \brief Returns the object able to compute the dynamic parameters of the robot. */
    inline CjrlHumanoidDynamicRobot * getHumanoidDynamicRobot() const
      { return m_HumanoidDynamicRobot;}

    /** @} */

  };
};
#endif /* _GLOBAL_STRATEGY_MANAGER_H_ */

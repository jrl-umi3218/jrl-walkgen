/*! \file GlobalStrategyManager.h
  \brief This object defines a global strategy abstract object to generate an output
  handled by the PatternGeneratorInterface object.
  
  
  Copyright (c) 2007, 
  @author Francois Keith, Olivier Stasse
   
  JRL-Japan, CNRS/AIST
  
  All rights reserved.

   Please see License.txt for further information on license.  
*/
#include <deque> 


/*! JRL inclusion */

// MAL 
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

// Dynamics
#include <robotDynamics/jrlHumanoidDynamicRobot.h>

// PG

#include <PGTypes.h>
#include <SimplePlugin.h>
#include <PreviewControl/PreviewControl.h>

/*! End of JRL inclusion */

#ifndef _GLOBAL_STRATEGY_MANAGER_H_
#define _GLOBAL_STRATEGY_MANAGER_H_

namespace PatternGeneratorJRL
{

  /** @ingroup pgjrl
      
  Abstract interface to the object handling the global strategy
  to generate the output of the pattern generator.
  */
  class  GlobalStrategyManager : public SimplePlugin
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
      @param[out] aCOMState: returns position, velocity and acceleration of the CoM.
      @param[out] CurrentConfiguration: The results is a state vector containing the articular positions.
      @param[out] CurrentVelocity: The results is a state vector containing the speed.
      @param[out] CurrentAcceleration: The results is a state vector containing the acceleration.
    */
    virtual int OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
				       FootAbsolutePosition &RightFootPosition,
				       MAL_VECTOR(,double) & ZMPRefPos,
				       COMState & aCOMState,
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
				      COMState & aStartingCOMState,
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
			    deque<COMState> * aCOMBuffer,
			    deque<FootAbsolutePosition> *aLeftFootAbsolutePositions,
			    deque<FootAbsolutePosition> *aRightFootAbsolutePositions );

    /*! Prepare the buffers at the beginning of the foot positions. */
    virtual void Setup(deque<ZMPPosition> & aZMPositions,
		       deque<COMState> & aCOMBuffer,
		       deque<FootAbsolutePosition> & aLeftFootAbsolutePositions,
		       deque<FootAbsolutePosition> & aRightFootAbsolutePositions )=0;
      
  protected:

    /*! \name Positions buffers.
      @{
     */
    
    /*! Buffer of ZMP positions */
    deque<ZMPPosition> * m_ZMPPositions;

    /*! Buffer for the COM position. */
    deque<COMState> * m_COMBuffer;

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

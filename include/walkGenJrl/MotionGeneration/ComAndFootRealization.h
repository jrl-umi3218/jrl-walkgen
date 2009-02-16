/* Abstract class to generate Humanoid Body motion
   for given CoM and foot postures.

   Copyright (c) 2007, 
   @author Francois Keith, Olivier Stasse.
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or 
   other materials provided with the distribution.
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

#ifndef _COM_AND_FOOT_REALIZATION_H_
#define _COM_AND_FOOT_REALIZATION_H_

#include <robotDynamics/jrlHumanoidDynamicRobot.h>
#include <walkGenJrl/walkGenJrl_API.h>
#include <walkGenJrl/SimplePlugin.h>
#include <walkGenJrl/StepStackHandler.h>


namespace PatternGeneratorJRL
{
  class PatternGeneratorInterface;

  /** @ingroup motiongeneration
      This abstract specifies the different methods
      called by the Pattern Generator to generate a body posture.
      Taking as an input CoM and feet postures, it is in charge of
      finding a body posture, according to different strategies.

      The body posture, and the necessary information are store in an instance
      of \a CjrlHumanoidDynamicRobot.

      Mostly they have to provide a ZMP multibody after the first
      preview loop of the class \a ZMPPreviewControlWithMultiBodyZMP class.

   */
  class WALK_GEN_JRL_EXPORT ComAndFootRealization : public SimplePlugin
  {
  private:

    /*! \brief Store the dynamic robot. */
    CjrlHumanoidDynamicRobot * m_HumanoidDynamicRobot;
    
    /*! Store the height of the CoM */
    double m_HeightOfCoM;

    /*! Sampling Period. */
    double m_SamplingPeriod;    

    /*! Object which handles a Stack of steps */
    StepStackHandler * m_StepStackHandler;

  public:

    /* \name Constructor and destructor.*/

    /*! \brief Constructor 

     */
    inline ComAndFootRealization(PatternGeneratorInterface * aPatternGeneratorInterface):
      SimplePlugin((PatternGeneratorJRL::SimplePluginManager *)aPatternGeneratorInterface)
      ,m_HumanoidDynamicRobot(0)
      ,m_HeightOfCoM(0)
      ,m_SamplingPeriod(0.005)
      ,m_StepStackHandler(0)
      {
      };
      
    /*! \brief virtual destructor */
    inline virtual ~ComAndFootRealization() {};
      

    /*! \brief Initialization phase */
    virtual void Initialization() = 0;
    /** @} */

    /*! \name Methods related to the computation to be asked by 
      \a  ZMPPreviewControlWithMultiBodyZMP.  */

    /*! Compute the robot state for a given CoM and feet posture.
      Each posture is given by a 3D position and two Euler angles \f$ (\theta, \omega) \f$.
      Very important: This method is assume to set correctly the body angles of
      its \a HumanoidDynamicRobot and a subsequent call to the ZMP position 
      will return the associated ZMP vector.
      @param CoMPosition: a 5 dimensional vector with the first dimension for position,
      and the last two for the orientation (Euler angle).
      @param CoMSpeed: a 5 dimensional vector: 3 for the linear velocity in X,Y,Z,
      and two for the angular velocity.
      @param CoMAcc: a 5 dimensional vector: 3 for the linear acceleration in X,Y,Z,
      and two for the angular acceleration.
      @param LeftFoot: a 5 dimensional following the same convention than for \a CoMPosition.
      @param RightFoot: idem.
      @param CurrentConfiguration: The result is a state vector containing the position which are put inside this parameter.
      @param CurrentVelocity: The result is a state vector containing the speed which are put inside this parameter.x
      @param CurrentAcceleration: The result is a state vector containing the acceleration which are put inside this parameter.x
      @param IterationNumber: Number of iteration.
      @param Stage: indicates which stage is reach by the Pattern Generator.
    */
    virtual bool ComputePostureForGivenCoMAndFeetPosture(MAL_VECTOR(,double) &CoMPosition,
							 MAL_VECTOR(,double) &CoMSpeed,
							 MAL_VECTOR(,double) &CoMAcc,
							 MAL_VECTOR(,double) &LeftFoot,
							 MAL_VECTOR(,double) &RightFoot,
							 MAL_VECTOR(,double) &CurrentConfiguration,
							 MAL_VECTOR(,double) &CurrentVelocity,
							 MAL_VECTOR(,double) &CurrentAcceleration,
							 int IterationNumber,
							 int Stage) =0;

    /*! Returns the waist position associate to the current  
      @} */

    /*! \name Setter and getter for the jrlHumanoidDynamicRobot object. */
    
    /*! @param aHumanoidDynamicRobot: an object able to compute dynamic parameters
      of the robot. */
    inline  virtual bool setHumanoidDynamicRobot(const CjrlHumanoidDynamicRobot *aHumanoidDynamicRobot)
    { m_HumanoidDynamicRobot = (CjrlHumanoidDynamicRobot *)aHumanoidDynamicRobot;
      return true;}

    /*! Returns the object able to compute dynamic parametersof the robot. */
    inline CjrlHumanoidDynamicRobot * getHumanoidDynamicRobot() const
      { return m_HumanoidDynamicRobot;}
    
    /** @} */


    /*! This initialization phase does the following:
      1/ we take the current state of the robot
      to compute the current CoM value.
      2/ We deduce the difference between the CoM and the waist,
      which is suppose to be constant for the all duration of the motion. 

      IMPORTANT: The jrlHumanoidDynamicRobot must have been properly set up.
      
    */
    virtual bool InitializationCoM(MAL_VECTOR(,double) &BodyAnglesIni,
				   MAL_S3_VECTOR(,double) & lStartingCOMPosition,
				   MAL_VECTOR(,double) & lStartingWaistPose,
				   FootAbsolutePosition & InitLeftFootAbsPos, 
				   FootAbsolutePosition & InitRightFootAbsPos)=0;

    /*! This initialization phase, make sure that the needed buffers
      for the upper body motion are correctly setup.
    */
    virtual    bool InitializationUpperBody(deque<ZMPPosition> &inZMPPositions,
					    deque<COMPosition> &inCOMBuffer,
					    deque<RelativeFootPosition> lRelativeFootPositions)=0;

    /* @} */

    /*! Set the algorithm used for ZMP and CoM trajectory. */
    void SetAlgorithmForZMPAndCoMTrajectoryGeneration(int anAlgo);
    
    /*! Get the algorithm used for ZMP and CoM trajectory. */
    int GetAlgorithmForZMPAndCoMTrajectoryGeneration() ;

    /*! \name Setter and getter for the height of the CoM. 
      @{
     */
    
    void SetHeightOfTheCoM(double theHeightOfTheCoM) 
    { m_HeightOfCoM = theHeightOfTheCoM; }
    
    const double & GetHeightOfTheCoM() const
      {return m_HeightOfCoM;}
    /*! @} */

    /*! \name Setter and getter for the sampling period. 
      @{
     */
    /*! Setter for the sampling period. */
    inline void setSamplingPeriod(double  aSamplingPeriod)
      { m_SamplingPeriod = aSamplingPeriod; } 
    /*! Getter for the sampling period. */
    inline const double & getSamplingPeriod() const
      { return m_SamplingPeriod;}
    /* @} */


    /*! \name Getter and setter for the handler of step stack  @{ */
    
    void SetStepStackHandler(StepStackHandler * lStepStackHandler)
    { m_StepStackHandler = lStepStackHandler;}

    StepStackHandler *  GetStepStackHandler() const
      { return m_StepStackHandler; }

    /*! @} */


    /*! Get the current position of the waist in the COM reference frame 
      @return a 4x4 matrix which contains the pose and the position of the waist
      in the CoM reference frame.
    */
    virtual MAL_S4x4_MATRIX(,double) GetCurrentPositionofWaistInCOMFrame() = 0;

    /*! \brief Get the COG of the ankles at the starting position. */
    virtual MAL_S3_VECTOR(,double) GetCOGInitialAnkles() = 0;
    
  };

};


#endif

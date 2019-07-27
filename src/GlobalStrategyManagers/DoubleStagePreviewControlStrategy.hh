/*
 * Copyright 2007, 2008, 2009, 2010,
 *
 * Olivier  Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of jrl-walkgen.
 * jrl-walkgen is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * jrl-walkgen is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with jrl-walkgen.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/*! \file DoubleStagePreviewControlStrategy.h
  \brief This object defines a global strategy object to generate
  full body position every 5 ms over a preview window.
  It implements Kajita's algorithm presented in \ref Kajita2003 */


#include <SimplePlugin.hh>
#include <GlobalStrategyManagers/GlobalStrategyManager.hh>
#include <PreviewControl/ZMPPreviewControlWithMultiBodyZMP.hh>

#ifndef _DOUBLE_STAGE_PREVIEW_CONTROL_STRATEGY_H_
#define _DOUBLE_STAGE_PREVIEW_CONTROL_STRATEGY_H_
namespace PatternGeneratorJRL
{

  /** @ingroup pgjrl
      Implementation of the buffers handling for Kajita's
      algorithm.
  */
  class  DoubleStagePreviewControlStrategy : public GlobalStrategyManager
  {

  public:
    /*! Default constructor. */
    DoubleStagePreviewControlStrategy
    (SimplePluginManager *aSimplePluginManager);

    /*! Default destructor. */
    ~DoubleStagePreviewControlStrategy();

    /*! Perform a 5 ms step to generate the necessary information.
      \note{The meaning and the way to use this method depends on the child 
      class}.

      @param[out] LeftFootPosition: The position of the Left Foot position.
      @param[out] RightFootPosition: The position of the Right Foot position.
      @param[out] ZMPRefPos: The ZMP position to be feed to the controller, 
      in the waist
      frame reference.
      @param[out] COMState: returns position, velocity and acceleration of the 
      CoM.
      @param[out] CurrentConfiguration: The results is a state vector 
      containing the articular positions.
      @param[out] CurrentVelocity: The results is a state vector containing 
      the speed.
      @param[out] CurrentAcceleration: The results is a state vector containing 
      the acceleration.
    */
    int OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
                               FootAbsolutePosition &RightFootPosition,
                               Eigen::VectorXd & ZMPRefPos,
                               COMState & COMState,
                               Eigen::VectorXd & CurrentConfiguration,
                               Eigen::VectorXd & CurrentVelocity,
                               Eigen::VectorXd & CurrentAcceleration);



    /*! Computes the COM of the robot with the Joint values given in BodyAngles,
      velocities set to zero, and returns the values of the COM in 
      aStaringCOMState.
      Assuming that the waist is at (0,0,0)
      it returns the associate initial values for the left and right foot.
      @param[in] BodyAngles: 4x4 matrix of the robot's root (most of the time, 
      the waist)
      pose (position + orientation).
      @param[out] aStartingCOMState: Returns the 3D position of the CoM for 
      the current
      position of the robot.
      @param[out] aStartingZMPPosition: Returns the 3D position of the ZMP for 
      the current
      position of the robot.
      @param[out] InitLeftFootPosition: Returns the position of the left foot in
      the waist coordinates frame.
      @param[out] InitRightFootPosition: Returns the position of the right foot
      in the waist coordinates frame.
    */
    int EvaluateStartingState(Eigen::VectorXd & BodyAngles,
                              COMState & aStartingCOMState,
                              Eigen::Vector3d & aStartingZMPPosition,
                              Eigen::Matrix<double, 6, 1> & aStartingWaistPose,
                              FootAbsolutePosition & InitLeftFootPosition,
                              FootAbsolutePosition & InitRightFootPosition);

    /*! \brief Setter for the stack handler. */
    void SetStepStackHandler(StepStackHandler *aSSH)
    {
      m_StepStackHandler = aSSH;
    }

    /*! \brief Initialization of the inter objects relationship. */
    int InitInterObjects(PinocchioRobot * aPR,
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

    /*! \brief This method specifies the frame in which the ZMP is given.
     */
    void SetZMPFrame(istringstream &strm);

    /*! \brief  Prepare the buffers at the beginning of the foot positions.
      @param[out] aZMPositions: ZMP position reference
      @param[out] aCOMBuffer: COM trajectory
      @param[out] aLeftFootAbsolutePositions: Trajectory of absolute positions 
      for the left foot.
      @param[out] aRightFootAbsolutePositions: Trajectory of absolute positions 
      for the right foot.
    */
    void Setup(deque<ZMPPosition> & aZMPositions,
               deque<COMState> & aCOMBuffer,
               deque<FootAbsolutePosition> & aLeftFootAbsolutePositions,
               deque<FootAbsolutePosition> & aRightFootAbsolutePositions );

    /*! \brief Get Waist state. */
    bool getWaistState(WaistState & aWaistState);

    static const unsigned int ZMPFRAME_WAIST=0;
    static const unsigned int ZMPFRAME_WORLD=1;

  protected:

    /*! Set the sampling period and update NL.*/
    void SetSamplingPeriod(double lSamplingPeriod);

    /*! Set the preview control time and update NL. */
    void SetPreviewControlTime(double lPreviewControlTime);

    /*! \brief The object to be used to perform one step of control,
      and generates the corrected CoM trajectory. */
    ZMPPreviewControlWithMultiBodyZMP *m_ZMPpcwmbz;

    /*! \brief Pointer to the stack handler. */
    StepStackHandler *m_StepStackHandler;

    /*! \brief Keep the waist position using a CoM position data structure . */
    COMState m_CurrentWaistState;

    /*! \brief The size of the preview control window */
    unsigned int m_NL;

    /*! \brief The time of the preview control */
    double m_PreviewControlTime;

    /*! ZMP reference frame. */
    unsigned int m_ZMPFrame;
  };
}
#endif /* _DOUBLE_STAGE_PREVIEW_CONTROL_STRATEGY_H_*/

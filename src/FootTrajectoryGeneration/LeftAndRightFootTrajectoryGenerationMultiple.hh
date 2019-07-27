/*
 * Copyright 2008, 2009, 2010,
 *
 * Olivier  Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/*! \file LeftAndRightFootTrajectoryGenerationMultiple.h
  \brief This class is a container for two analytical trajectories.

  @ingroup foottrajectorygeneration
*/

#ifndef _LEFT_AND_RIGHT_FOOT_TRAJECTORY_GENERATION_MULTIPLE_H_
#define _LEFT_AND_RIGHT_FOOT_TRAJECTORY_GENERATION_MULTIPLE_H_

/* abstractRobotDynamics inclusion */
#include <jrl/walkgen/pinocchiorobot.hh>

/* Walking Pattern Generator inclusion */

#include <SimplePlugin.hh>
#include <FootTrajectoryGeneration/FootTrajectoryGenerationMultiple.hh>

namespace PatternGeneratorJRL
{

  /*! @ingroup foottrajectorygeneration
    The main goal of this class is to provide a simple interface to the
    objects derived from ZMPRefTrajectoryGeneration to generate the
    foot trajectories.

    It acts as a container for two FootTrajectoryGenerationMultiple objects
    which handle several polynomials trajectory generation for the right 
    and left feet.

    It provides an initialization of the underlying objects assuming that
    some basic informations have been provided: single support time, 
    double support time, omega, step height.

    The information used follow the previously defined script like language,
    but it could be extended for a clear separation between the two feet.

  */
  class  LeftAndRightFootTrajectoryGenerationMultiple : public SimplePlugin
  {

  public:
    /*! \brief The constructor initialize the plugin part, 
      and the data related to the humanoid. */
    LeftAndRightFootTrajectoryGenerationMultiple(SimplePluginManager * lSPM,
                                                 PRFoot * inFoot);

    /*! \brief Copy constructor. */
    LeftAndRightFootTrajectoryGenerationMultiple
    (const LeftAndRightFootTrajectoryGenerationMultiple &);

    /*! \brief Memory release. */
    ~LeftAndRightFootTrajectoryGenerationMultiple();

    /*! \brief Reimplementation of the call method for the plugin manager.
      More explicitly this object will deal with the call which initialize
      the feet behaviors (\f$omega\f$, \f$ stepheight \f$) .
    */
    virtual void CallMethod(std::string &Method, std::istringstream &strm);

    /*! \brief Initialize the analytical feet trajectories 
      from a set of relative step.
      It is based on the parameters given through the interface. 
      All the settings are done internally.
      The only output is the set of absolute positions for the support foot.
      @param[in] RelativeFootPositions: The set of relative positions 
      for the support foot.
      @param[in] LeftFootInitialPosition: The initial position of the left foot.
      @param[in] RightFootInitialPosition: the initial position of the right 
      foot.
      @param[out] SupportFootAbsoluteFootPositions: The set of absolute foot
      positions
      corresponding to the set of relative foot positions 
      (i.e given step by step and not every sampled control time).
      @param[in] IgnoreFirst: Ignore the first double support phase, 
      should be true at beginning of stepping.
      @param[in] Continuity: Should be true if more steps should be added, 
      false to stop stepping.
    */
    void InitializeFromRelativeSteps
    (deque<RelativeFootPosition>
     &RelativeFootPositions,
     FootAbsolutePosition &LeftFootInitialPosition,
     FootAbsolutePosition &RightFootInitialPosition,
     deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions,
     bool IgnoreFirst, bool Continuity);

    /*! \brief Method to compute the absolute position of the foot.
      @param[in] LeftOrRight: -1 indicates the right foot, 
      1 indicates the left foot.
      @param[in] time: The absolute time to be compared 
      with the absolute reference defining the start
      of the trajectory.
      @param[out] aFootAbsolutePosition: 
      The data structure to be filled with the information
      \f$ (x,y,z,\omega, \omega_2, \theta) \f$.
    */
    bool ComputeAnAbsoluteFootPosition
    (int LeftOrRight, double time,
     FootAbsolutePosition & aFootAbsolutePosition);

    /*! \brief Method to compute the absolute position of the foot.
      @param[in] LeftOrRight: -1 indicates the right foot, 
      1 indicates the left foot.
      @param[in] time: The absolute time to be compared 
      with the absolute reference defining the start
      of the trajectory.
      @param[out] aFootAbsolutePosition: The data structure to 
      be filled with the information
      \f$ (x,y,z,\omega, \omega_2, \theta) \f$.
      @param[in] IndexInterval: On which interval to compute the foot position.

    */
    bool ComputeAnAbsoluteFootPosition
    (int LeftOrRight, double time,
     FootAbsolutePosition & aFootAbsolutePosition,
     unsigned int IndexInterval);

    /*
      bool ComputeAnAbsoluteFootPosition(int LeftOrRight,
      double time,
      std::deque<FootAbsolutePosition> & adFAP,
      unsigned int IndexInterval);*/

    /*! \brief Method to compute absolute feet positions from a set of 
      relative one.
      @param[in] RelativeFootPositions: The set of relative positions for 
      the support foot.
      @param[in] LeftFootInitialPosition: The initial position of the left foot.
      @param[in] RightFootInitialPosition: the initial position of the 
      right foot.
      @param[out] SupportFootAbsoluteFootPositions: The set of absolute 
      foot positions
      corresponding to the set of relative foot positions 
      (i.e given step by step and not every sampled control time)
    */
    void ComputeAbsoluteStepsFromRelativeSteps
    (deque<RelativeFootPosition>
     &RelativeFootPositions,
     FootAbsolutePosition &LeftFootInitialPosition,
     FootAbsolutePosition &RightFootInitialPosition,
     deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions);
    
    /*! \brief Method to compute absolute feet positions from a set of 
      relative one.
      @param[in] RelativeFootPositions: The set of relative positions for the
      support foot.
      @param[in] SupportFootInitialPosition: The initial position of the 
      support foot.
      @param[out] SupportFootAbsoluteFootPositions: The set of absolute foot
      positions
      corresponding to the set of relative foot positions 
      (i.e given step by step and not every sampled control time).
    */
    void ComputeAbsoluteStepsFromRelativeSteps
    (deque<RelativeFootPosition>
     &RelativeFootPositions,
     FootAbsolutePosition &SupportFootInitialPosition,
     deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions);
    
    /*! \brief Method to compute relative feet positions from a set of absolute 
      one where one has changed.
      @param[in] RelativeFootPositions: The set of relative positions for the
      support foot.
      @param[in] ChangedInterval: The interval where the absolute foot position 
      has been changed.
      @param[in] SupportFootInitialPosition: The absolute foot position of the
      initial step in the stack of steps.
      @param[out] SupportFootAbsoluteFootPositions: The set of absolute foot
      positions corresponding to the set of relative foot positions 
      (i.e given step by step and not every sampled control time).
    */
    void ChangeRelStepsFromAbsSteps
    (deque<RelativeFootPosition>
     &RelativeFootPositions,
     FootAbsolutePosition &SupportFootInitialPosition,
     deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions,
     unsigned int ChangedInterval);
    
    /*! Returns foot */
    PRFoot *getFoot() const;

  protected:

    /*! Internal method to modify the value of an interval. */
    void SetAnInterval(unsigned int IntervalIndex,
                       FootTrajectoryGenerationMultiple * aFTGM,
                       FootAbsolutePosition &FootInitialPosition,
                       FootAbsolutePosition &FootFinalPosition,
                       vector<double> MiddlePos=vector<double>(3,-1));

    /*! Left Foot Trajectory Generation object for several intervals. */
    FootTrajectoryGenerationMultiple * m_LeftFootTrajectory;

    /*! Right Foot Trajectory Generation object for several intervals. */
    FootTrajectoryGenerationMultiple * m_RightFootTrajectory;

    /*! Humanoid specificities object handler */
    PRFoot * m_Foot;

    /*! Set of time intervals */
    std::vector<double> m_DeltaTj;

    /*! Omega */
    double m_Omega;

    /*! Omega2 */
    double m_Omega2;

    std::vector<double> wayPoint ;
    double m_WayPointThreshold ;

    /*! Step height. */
    double m_StepHeight;

    /*! Step Curving */
    double m_StepCurving;

    /*! Middle Way Point of foot trajectories */
    Eigen::VectorXd m_MiddleWayPoint ;

    /*! Single support time. */
    double m_SingleSupportTime;

    /*! Double support time. */
    double m_DoubleSupportTime;

  public:
    /*! Set the intervals time */
    void SetDeltaTj(std::vector<double> & aDeltaTj);

    /*! Get the intervals time */
    void GetDeltaTj(std::vector<double> & aDeltaTj) const;

    /*! Display intervals value for left and right feet. */
    void DisplayIntervals();

    /*! Set the step height */
    void SetStepHeight(double aStepHeight);

    /*! Get the step height. */
    double GetStepHeight() const;

    /*! Set the curving step */
    void SetStepCurving(double aStepCurving);

    /*! Get the curving step */
    double GetStepCurving() const;

    /*! \name Methods related to the time reference.
      @{ */

    /*! \brief This returns the absolute time reference.
      As the object manipulates several trajectories generator,
      the coherency of the returned informations has to be checked.
      If this is not the case, i.e. the trajectory generators have different
      absolute time references, the method returns -1. */
    double GetAbsoluteTimeReference() const;

    /*! \brief Set the time reference for all the trajectory generators. */
    void SetAbsoluteTimeReference(double anAbsoluteTimeReference);

    /*! @} */


    FootTrajectoryGenerationMultiple * getLeftFootTrajectory() const;
    FootTrajectoryGenerationMultiple * getRightFootTrajectory() const;

    LeftAndRightFootTrajectoryGenerationMultiple & operator=
    (const LeftAndRightFootTrajectoryGenerationMultiple & aLRFTGM);

  };
}
#endif /* _LEFT_AND_RIGHT_FOOT_TRAJECTORY_GENERATION_MULTIPLE_H_ */

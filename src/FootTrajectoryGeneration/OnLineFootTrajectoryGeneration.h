/*
 * Copyright 2011
 *
 * 
 * Andrei Herdt
 * Olivier Stasse
 *
 * JRL, CNRS/AIST, INRIA Grenoble
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
/*! \file FootTrajectoryGenerationStandard.h
  \brief This object generate all the values for the foot trajectories.
   @ingroup foottrajectorygeneration */


#ifndef _ONLINE_FOOT_TRAJECTORY_GENERATION_H_
#define _ONLINE_FOOT_TRAJECTORY_GENERATION_H_

#include <FootTrajectoryGeneration/FootTrajectoryGenerationStandard.hh>

namespace PatternGeneratorJRL
{

  /// @ingroup foottrajectorygeneration
  /// Generate online trajectories for the swinging and the stance foot some amount in the future.
  class  OnLineFootTrajectoryGeneration : public FootTrajectoryGenerationStandard
  {

    //
    // Public methods:
    //
  public:

    OnLineFootTrajectoryGeneration(SimplePluginManager *lSPM,CjrlFoot *aFoot);

    virtual ~OnLineFootTrajectoryGeneration();


    /// Interpolate piece of feet trajectories
    ///
    /// \param[in] Time Current time
    /// \param[in] CurrentSupport
    /// \param[in] FPx Previewed foot position to be interpolated
    /// \param[in] FPy Previewed foot position to be interpolated
    /// \param[in] PreviewedSupportAngles_deq
    /// \param[out] FinalLeftFootTraj_deq Left foot trajectory
    /// \param[out] FinalRightFootTraj_deq Right foot trajectory
    virtual void interpolate_feet_positions(double Time,
        const deque<support_state_t> & PrwSupportStates_deq,
        const solution_t & Solution,
        const deque<double> & PreviewedSupportAngles_deq,
        deque<FootAbsolutePosition> &FinalLeftFootTraj_deq,
        deque<FootAbsolutePosition> &FinalRightFootTraj_deq);

    /// \name Accessors
    /// \{
    inline double QPSamplingPeriod() const
    { return QP_T_; };
    inline void QPSamplingPeriod( double QP_T )
    { QP_T_ = QP_T; };
    inline unsigned int NbSamplingsPreviewed() const
    { return QP_N_; };
    inline void NbSamplingsPreviewed( unsigned int QP_N )
    { QP_N_ = QP_N; };
    inline double FeetDistance() const
    { return FeetDistanceDS_; };
    inline void FeetDistance( double FeetDistance )
    { FeetDistanceDS_ = FeetDistance; };
    inline void StepHeight( double StepHeight )
    { StepHeight_ = StepHeight; };
    inline double StepHeight()
    { return StepHeight_; };
    /// \}

    //
    // Protected methods:
    //
  protected:

    /// \brief Check if the solution should be used as is and
    /// propose alternative if not.
    ///
    /// \param[out] X Solution
    /// \param[out] Y Solution
    /// \param[in] CurrentSupport
    /// \param[in] CurrentTime
    virtual void interpret_solution( double CurrentTime, const solution_t & Solution,
        const support_state_t & CurrentSupport, unsigned int NbSteps, double & X, double & Y );


    /// \brief Compute the position of the swinging and the stance foot.
    /// Use polynomial of 3rd order for the X-axis, Y-axis,
    /// orientation in the X-Z axis, and orientation in the X-Y axis.
    /// Use a 4th order polynome for the Z-axis.
    ///
    /// \param SupportFootTraj_deq: Queue of positions for the support foot.
    /// Set the foot position at index CurrentAbsoluteIndex of the queue.
    /// \param StanceFootTraj_deq: Queue of absolute position for the swinging
    /// foot. Set the foot position at index NoneSupportFootAbsolutePositions of the queue.
    /// \param StartIndex: Index in the queues of the foot position to be set.
    /// \param k: Current sampling
    /// \param UnlockedSwingPeriod: Amount of time where the swinging foot can move horizontally.
    /// \param StepType: Type of steps (for book-keeping).
    /// \param LeftOrRight: Specify if it is left (1) or right (-1).
    virtual void UpdateFootPosition(deque<FootAbsolutePosition> &SupportFootTraj_deq,
        deque<FootAbsolutePosition> &StanceFootTraj_deq,
        int StartIndex, int k,
        double LocalInterpolationStartTime,
        double UnlockedSwingPeriod,
        int StepType, int LeftOrRight);

    //
    // Protected members
    //
  protected:

    /// \brief Sampling period of the QP
    double QP_T_;

    /// \brief Nb previewed samplings
    unsigned int QP_N_;

    /// \brief Distance between feet centers in ds phase
    double FeetDistanceDS_;

    /// \brief Height of the flying foot in the middle of the SS phase
    double StepHeight_;

    /// \brief Half of simple support passed trigger
    bool HalfTimePassed_;

  };

}
#endif /* _ONLINE_FOOT_TRAJECTORY_GENERATION_H_ */


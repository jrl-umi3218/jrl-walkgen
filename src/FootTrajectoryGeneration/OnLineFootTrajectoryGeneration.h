/*
 * Copyright 2011
 *
 * 
 * Andrei Herdt
 * Olivier Stasse
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
/*! \file FootTrajectoryGenerationStandard.h
  \brief This object generate all the values for the foot trajectories.
   @ingroup foottrajectorygeneration */


#ifndef _ONLINE_FOOT_TRAJECTORY_GENERATION_H_
#define _ONLINE_FOOT_TRAJECTORY_GENERATION_H_

#include <FootTrajectoryGeneration/FootTrajectoryGenerationStandard.h>

namespace PatternGeneratorJRL
{

  /// @ingroup foottrajectorygeneration
  /// Generate online trajectories for the swinging and the stance foot some amount in the future.
  /// Use 3rd order polynoms for horizontal directions and 4th order polynomes for vertical directions.
  /// Specify landing and take off phase using an angular value (\f$\omega\f$).
  class  OnLineFootTrajectoryGeneration : public FootTrajectoryGenerationStandard
  {

    //
    // Public methods:
    //
  public:

    /*! Constructor: In order to compute some appropriate strategies,
      this class needs to extract specific details from the humanoid model. */
    OnLineFootTrajectoryGeneration(SimplePluginManager *lSPM,CjrlFoot *aFoot);

    /*! Default destructor. */
    virtual ~OnLineFootTrajectoryGeneration();


   /// Interpolate piece of feet trajectories
   ///
   /// \param[in] time Current time
   /// \param[in] CurrentIndex Index in the queues of the foot position to be set.
   /// \param[in] CurrentSupport
   /// \param[in] FPx Previewed foot position to be interpolated
   /// \param[in] FPy Previewed foot position to be interpolated
   /// \param[ou] SupportFootTraj_deq Queue of positions for the support foot.
   /// Set the foot position at index CurrentAbsoluteIndex of the queue.
   /// \param[out] StanceFootTraj_deq Queue of absolute position for the swinging
   /// foot. Set the foot position at index NoneSupportFootAbsolutePositions of the queue.
   virtual void interpolate_feet_positions(double Time,
                                 const support_state_t & CurrentSupport,
                                 double FPx, double FPy,
                                 const deque<double> & PreviewedSupportAngles_deq,
                                 deque<FootAbsolutePosition> &FinalLeftFootTraj_deq,
                                 deque<FootAbsolutePosition> &FinalRightFootTraj_deq);

   /// \name Accessors
   /// \{
   inline double QPSamplingPeriod() const
   { return QP_T_; };
   inline void QPSamplingPeriod( double QP_T )
   { QP_T_ = QP_T; };
   inline double FeetDistance() const
   { return FeetDistanceDS_; };
   inline void FeetDistance( double FeetDistance )
   { FeetDistanceDS_ = FeetDistance; };
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
   /// \param[in] CurrentSupport Support state
   virtual void check_solution(double & X, double & Y,
       const support_state_t & CurrentSupport, double CurrentTime);


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

   /// \brief Distance between feet centers in ds phase
   double FeetDistanceDS_;
  };
  
}
#endif /* _ONLINE_FOOT_TRAJECTORY_GENERATION_H_ */


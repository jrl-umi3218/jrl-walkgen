/*
 * Copyright 2010,
 *
 * Mehdi    Benallegue
 * Andrei   Herdt
 * Francois Keith
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
/*
 * OrientationsPreview.h
 */

#ifndef ORIENTATIONSPREVIEW_H_
#define ORIENTATIONSPREVIEW_H_



#include <deque>

#include <privatepgtypes.hh>
#include <jrl/walkgen/pgtypes.hh>
#include <Mathematics/PolynomeFoot.hh>
#include <jrl/walkgen/pinocchiorobot.hh>

namespace PatternGeneratorJRL
{
  /// \brief The acceleration phase is fixed
  class OrientationsPreview
  {

    //
    // Public methods:
    //
  public:

    /// \name Accessors
    /// \{
    OrientationsPreview( PinocchioRobot *aPR );
    ~OrientationsPreview();
    /// \}

    /// \brief Preview feet and trunk orientations inside the preview window
    /// The orientations of the feet are adapted to the previewed orientation
    /// of the hip.
    /// The resulting velocities accelerations and orientations are verified
    /// against the limits.
    /// If the constraints can not be satisfied the rotational velocity
    /// of the trunk is reduced.
    /// The trunk is rotating with a constant speed after a constant
    /// acceleration phase of T_ length.
    /// During the initial double support phase the trunk is not rotating
    /// contrary to the following.
    ///
    /// \param[in] Time
    /// \param[in] Ref
    /// \param[in] StepDuration
    /// \param[in] LeftFootPositions_deq
    /// \param[in] RightFootPositions_deq
    /// \param[out] Solution Trunk and Foot orientations
    void preview_orientations
    (double Time,
     const reference_t & Ref,
     double StepDuration,
     const std::deque<FootAbsolutePosition> & LeftFootPositions_deq,
     const std::deque<FootAbsolutePosition> & RightFootPositions_deq,
     solution_t & Solution);

    /// \brief Interpolate previewed orientation of the trunk
    ///
    /// \param[in] Time
    /// \param[in] CurrentIndex
    /// \param[in] NewSamplingPeriod
    /// \param[in] PrwSupportStates_deq
    /// \param[out] FinalCOMTraj_deq
    void interpolate_trunk_orientation
    (double Time,
     int CurrentIndex,
     double NewSamplingPeriod,
     const std::deque<support_state_t> & PrwSupportStates_deq,
     std::deque<COMState> & FinalCOMTraj_deq);

    /// \brief Compute the current state for the preview of the orientation
    ///
    /// \param[in] Time
    /// \param[in] CurrentIndex
    /// \param[in] NewSamplingPeriod
    /// \param[in] PrwSupportStates_deq
    /// \param[out] FinalCOMTraj_deq
    void one_iteration
    (double Time,
     const std::deque<support_state_t> & PrwSupportStates_deq);
    

    /// \name Accessors
    /// \{
    inline COMState const & CurrentTrunkState() const
    {
      return TrunkState_;
    };
    inline void CurrentTrunkState(const COMState & TrunkState)
    {
      TrunkState_ = TrunkState;
    };
    inline COMState const & PreviewTrunkState() const
    {
      return TrunkStateT_;
    };
    inline void PreviewTrunkState(const COMState & TrunkState)
    {
      TrunkStateT_ = TrunkState;
    };
    inline double SSLength() const
    {
      return SSPeriod_;
    };
    inline void SSLength( double SSPeriod)
    {
      SSPeriod_ = SSPeriod;
    };
    inline double SamplingPeriod() const
    {
      return T_;
    };
    inline void SamplingPeriod( double SamplingPeriod)
    {
      T_ = SamplingPeriod;
    };
    inline double NbSamplingsPreviewed() const
    {
      return N_;
    };
    inline void NbSamplingsPreviewed( double SamplingsPreviewed)
    {
      N_ = SamplingsPreviewed;
    };
    /// \}

    //
    // Private methods:
    //
  private:

    /// \brief Verify and eventually reduce the maximal acceleration of
    /// the hip joint necessary to attain the velocity reference in one
    /// sampling T_.
    /// The verification is based on the supposition that the final joint
    /// trajectory is composed by
    /// a fourth-order polynomial acceleration phase inside T_ and
    /// a constant velocity phase for the rest of the preview horizon.
    ///
    /// \param[in] Ref
    /// \param[in] CurrentSupport
    void verify_acceleration_hip_joint(const reference_t & Ref,
                                       const support_state_t & CurrentSupport);

    /// \brief Verify velocity of hip joint
    /// The velocity is verified only between previewed supports.
    /// The verification is based on the supposition that the final joint
    /// trajectory is a third-order polynomial.
    ///
    /// \param[in] Time
    /// \param[in] PreviewedSupportFoot
    /// \param[in] PreviewedSupportAngle
    /// \param[in] StepNumber
    /// \param[in] CurrentSupport
    /// \param[in] CurrentRightFootAngle
    /// \param[in] CurrentLeftFootAngle
    /// \param[in] CurrentLeftFootVelocity
    /// \param[in] CurrentRightFootVelocity
    void verify_velocity_hip_joint(double Time,
                                   double PreviewedSupportFoot,
                                   double PreviewedSupportAngle,
                                   unsigned StepNumber,
                                   const support_state_t & CurrentSupport,
                                   double CurrentRightFootAngle,
                                   double CurrentLeftFootAngle,
                                   double CurrentLeftFootVelocity,
                                   double CurrentRightFootVelocity);

    /// \brief Verify angle of hip joint
    /// Reduce final velocity of the trunk if necessary
    ///
    /// \param[in] CurrentSupport
    /// \param[in] PreviewedTrunkAngleEnd
    /// \param[in] TrunkState
    /// \param[in] TrunkStateT
    /// \param[in] CurrentSupportAngle
    /// \param[in] StepNumber
    ///
    /// \return AngleOK
    bool verify_angle_hip_joint(const support_state_t & CurrentSupport,
                                double PreviewedTrunkAngleEnd,
                                const COMState & TrunkState,
                                COMState & TrunkStateT,
                                double CurrentSupportFootAngle,
                                unsigned StepNumber);

    /// \brief Fourth order polynomial trajectory
    /// \param[in] abcd Parameters
    /// \param[in] x
    ///
    /// \return Evaluation value
    double f(double a,double b,double c,double d,double x);

    /// \brief Fourth order polynomial trajectory derivative
    /// \param[in] abcd Parameters
    /// \param[in] x
    ///
    /// \return Evaluation value
    double df(double a,double b,double c,double d,double x);


    //
    // Private members:
    //
  private:

    /// \brief Angular limitations of the hip joints
    double lLimitLeftHipYaw_, uLimitLeftHipYaw_, lLimitRightHipYaw_,
      uLimitRightHipYaw_;

    /// \brief Maximal acceleration of a hip joint
    double uaLimitHipYaw_;

    /// \brief Upper crossing angle limit between the feet
    double uLimitFeet_;

    /// \brief Maximal velocity of a foot
    double uvLimitFoot_;

    /// \brief Single-support duration
    double SSPeriod_;

    /// \brief Number of sampling in a preview window
    double N_;

    /// \brief Time between two samplings
    double T_;

    /// \brief Rotation sense of the trunks angular velocity and acceleration
    double signRotVelTrunk_, signRotAccTrunk_;

    /// \brief
    double SupportTimePassed_;

    /// \brief
    double LastFirstPvwSol_ ;

    /// \brief Numerical precision
    const static double EPS_;

    /// \brief Current trunk state
    COMState TrunkState_;
    /// \brief State of the trunk at the first previewed sampling
    COMState TrunkStateT_;

    Polynome4 * TrunkStateYaw_ ;
  };
}
#endif /* ORIENTATIONSPREVIEW_H_ */

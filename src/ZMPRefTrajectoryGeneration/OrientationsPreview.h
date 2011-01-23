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

#include <PreviewControl/SupportFSM.h>
#include <jrl/walkgen/pgtypes.hh>
#include <abstract-robot-dynamics/joint.hh>

namespace PatternGeneratorJRL
{
  /// \brief The acceleration phase is fixed
  class OrientationsPreview {

    //
    // Public methods:
    //
  public:

    /// \name Accessors
    /// \{
    OrientationsPreview( CjrlJoint *aLeg );
    ~OrientationsPreview();
    /// \}

    /// \brief Preview feet orientations inside the preview window
    ///
    /// \param[in] Time
    /// \param[in] Ref
    /// \param[in] TrunkState
    /// \param[in] TrunkStateT
    /// \param[in] StepDuration
    /// \param[in] CurrentSupport
    /// \param[in] LeftFootAbsolutePositions
    /// \param[in] RightFootAbsolutePositions
    /// \param[out] PreviewedSupportAngles
    void preview_orientations(double Time, const reference_t & Ref,
			     double StepDuration, const support_state_t & CurrentSupport,
			     std::deque<FootAbsolutePosition> & LeftFootAbsolutePositions,
			     std::deque<FootAbsolutePosition> & RightFootAbsolutePositions,
                             std::deque<double> &PreviewedSupportAngles);

    /// \brief Interpolate orientation of the trunk
    ///
    /// \param[in] time
    /// \param[in] CurrentIndex
    /// \param[in] TrunkState
    /// \param[in] TrunkStateT
    /// \param[in] NewSamplingPeriod
    /// \param[in] CurrentSupport
    /// \param[out] FinalCOMTraj_deq
    void interpolate_trunk_orientation(double time, int CurrentIndex,
        double NewSamplingPeriod,
        const support_state_t & CurrentSupport,
        std::deque<COMState> & FinalCOMTraj_deq);

    /// \name Accessors
    /// \{
    inline COMState const & CurrentTrunkState() const
    { return TrunkState_; };
    inline void CurrentTrunkState(const COMState & TrunkState)
    { TrunkState_ = TrunkState; };
    inline double SSLength() const
    { return SSPeriod_; };
    inline void SSLength( double SSPeriod)
    { SSPeriod_ = SSPeriod; };
    inline double SamplingPeriod() const
    { return m_T; };
    inline void SamplingPeriod( double SamplingPeriod)
    { m_T = SamplingPeriod; };
    inline double NbSamplingsPreviewed() const
    { return m_N; };
    inline void NbSamplingsPreviewed( double SamplingsPreviewed)
    { m_N = SamplingsPreviewed; };
    /// \}

    //
    // Private methods:
    //
  private:

    /// \brief Verify and eventually reduce the acceleration of the hip joint
    ///
    /// \param[in] Ref Reference
    /// \param[in] TrunkState Current trunk state
    /// \param[out] TrunkStateT Trunk state at the next future sample
    /// \param [in] CurrentSupport Current support state
    void verify_acceleration_hip_joint(const reference_t & Ref,
                                      const support_state_t & CurrentSupport);

    /// \brief Verify velocity of hip joint
    ///
    /// \param[in] Time
    /// \param[in] TrunkStateT
    /// \param[in] PreviewedSupportFoot
    /// \param[in] PreviewedSupportAngle
    /// \param[in] StepNumber
    /// \param[in] CurrentSupport
    /// \param[in] CurrentRightFootAngle
    /// \param[in] CurrentLeftFootAngle
    /// \param[in] CurrentLeftFootVelocity
    /// \param[in] CurrentRightFootVelocity
    void verify_velocity_hip_joint(double Time, COMState &TrunkStateT,
                                  double PreviewedSupportFoot,
                                  double PreviewedSupportAngle, unsigned StepNumber,
                                  support_state_t CurrentSupport,
                                  double CurrentRightFootAngle, double CurrentLeftFootAngle,
                                  double CurrentLeftFootVelocity,
                                  double CurrentRightFootVelocity);

    /// \brief Verify angle of hip joint
    ///
    /// \param[in] CurrentSupport
    /// \param[in] PreviewedTrunkAngleEnd
    /// \param[in] TrunkState
    /// \param[in] TrunkStateT
    /// \param[in] CurrentSupportAngle
    /// \param[in] StepNumber
    /// \return AngleOK
    bool verify_angle_hip_joint(support_state_t CurrentSupport,
                               double PreviewedTrunkAngleEnd,
                               const COMState &TrunkState, COMState &TrunkStateT,
                               double CurrentSupportFootAngle,
                               unsigned StepNumber);

    /// \brief Fourth order polynomial trajectory
    double f(double a,double b,double c,double d,double x);

    /// \brief Fourth order polynomial trajectory derivative
    double df(double a,double b,double c,double d,double x);

    //
    // Private members:
    //
  private:

    /// \brief Angular limitations of the hip joints
    double m_lLimitLeftHipYaw, m_uLimitLeftHipYaw, m_lLimitRightHipYaw, m_uLimitRightHipYaw;

    /// \brief Maximal acceleration of a hip joint
    double m_uaLimitHipYaw;

    /// \brief Upper crossing angle limit between the feet
    double m_uLimitFeet;

    /// \brief Maximal velocity of a foot
    double m_uvLimitFoot;

    /// \brief Single-support duration
    double SSPeriod_;

    /// \brief Number of sampling in a preview window
    double m_N;

    /// \brief Time between two samplings
    double m_T;

    /// \brief Rotation sense of the trunks angular velocity and acceleration
    double m_signRotVelTrunk, m_signRotAccTrunk;

    /// \brief
    double m_SupportTimePassed;

    /// \brief
    const static double M_EPS;

    /// \brief Current trunk state
    COMState TrunkState_;
    /// \brief State of the trunk at the first previewed sampling
    COMState TrunkStateT_;



  };
}
#endif /* ORIENTATIONSPREVIEW_H_ */

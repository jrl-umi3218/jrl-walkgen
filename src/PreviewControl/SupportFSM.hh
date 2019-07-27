/*
 * Copyright 2010,
 *
 * Andrei  Herdt
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


#ifndef _SUPPORT_FSM_
#define _SUPPORT_FSM_

#include <jrl/walkgen/pgtypes.hh>
#include <privatepgtypes.hh>


namespace PatternGeneratorJRL
{

  /// \brief Finite state machine to determine the support parameters.
  class  SupportFSM
  {

    //
    // Public methods:
    //
  public:
    /// \brief Constructor
    SupportFSM();

    /// \brief Destructor
    ~SupportFSM();


    /// \brief Update the velocity reference after a pure rotation
    ///
    /// \param[in] Ref
    /// \param[in] CurrentSupport
    void update_vel_reference(reference_t & Ref,
                              const support_state_t & CurrentSupport);

    /// \brief Initialize the previewed state
    ///
    /// \param[in] time Current time
    /// \param[in] pi Number of (p)reviewed sampling (i)nstant
    /// inside the preview period
    /// \param[out] Support Support state to be actualized
    /// \param[in] Ref Trajectory reference
    void set_support_state( double time,
                            unsigned int pi,
                            support_state_t & Support,
                            const reference_t & Ref) const;

    /// \name Accessors
    /// \{
    inline double StepPeriod() const
    {
      return StepPeriod_;
    };
    inline void StepPeriod( const double StepPeriod )
    {
      StepPeriod_ = StepPeriod;
    };

    inline double DSPeriod() const
    {
      return DSPeriod_;
    };
    inline void DSPeriod( const double DSPeriod )
    {
      DSPeriod_ = DSPeriod;
    };

    inline double DSSSPeriod() const
    {
      return DSSSPeriod_;
    };
    inline void DSSSPeriod( const double DSSSPeriod )
    {
      DSSSPeriod_ = DSSSPeriod;
    };

    inline unsigned NBStepsSSDS() const
    {
      return NbStepsSSDS_;
    };
    inline void NbStepsSSDS( const unsigned NbStepsSSDS )
    {
      NbStepsSSDS_ = NbStepsSSDS;
    };

    inline double SamplingPeriod() const
    {
      return T_;
    };
    inline void SamplingPeriod( const double T )
    {
      T_ = T;
    };
    /// \}

    //
    // Private members:
    //
  private:

    /// \brief Number of steps to be done before DS
    unsigned NbStepsSSDS_;

    /// \brief Length of a double support phase
    double DSPeriod_;

    /// \brief Length of a step
    double StepPeriod_;

    /// \brief Duration of the transition ds -> ss
    double DSSSPeriod_;

    /// \Brief Sampling period
    double T_;

    /// \brief Precision constant
    const double EPS_;

    /// \name Rotation phase
    /// \{
    /// \brief True if the robot is in translation
    bool InTranslation_;

    /// \brief True if the robot is in rotation
    bool InRotation_;

    /// \brief Number of stabilize steps after the end of a rotation
    int NbStepsAfterRotation_;

    /// \brief Current support foot type (SS, DS)
    foot_type_e CurrentSupportFoot_;

    /// \brief True if the end phase of the rotation has begun
    bool PostRotationPhase_;
    /// \}


  };
}

#endif /* _SUPPORT_FSM_ */

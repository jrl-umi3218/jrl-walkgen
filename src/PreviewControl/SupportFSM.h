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
#include <privatepgtypes.h>

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

    /// \brief Initialize the previewed state.
    void setSupportState(const double &Time, const int &pi,
                support_state_t & Support, const reference_t & Ref) const;

    /// \name Accessors
    /// \{
    inline double const step_period() const
    { return StepPeriod_; };
    inline void step_period( const double StepPeriod )
    { StepPeriod_ = StepPeriod; };

    inline double const ds_period() const
    { return DSPeriod_; };
    inline void ds_period( const double DSPeriod )
    { DSPeriod_ = DSPeriod; };

    inline double const ds_ss_period() const
    { return DSSSPeriod_; };
    inline void ds_ss_period( const double DSSSPeriod )
    { DSSSPeriod_ = DSSSPeriod; };

    inline unsigned const nb_steps_ss_ds() const
    { return NbStepsSSDS_; };
    inline void nb_steps_ss_ds( const unsigned NbStepsSSDS )
    { NbStepsSSDS_ = NbStepsSSDS; };

    inline double const sampling_period() const
    { return T_; };
    inline void sampling_period( const double T )
    { T_ = T; };
    /// \}
 
    //
    // Private members:
    //
  private: 

    /// \brief Number of steps done before DS
    unsigned NbStepsSSDS_;
    /// \brief Length of a double support phase
    double DSPeriod_;
    /// \brief Length of a step
    double StepPeriod_;
    /// \brief Duration of the transition ds -> ss
    double DSSSPeriod_;

    /// \Brief Sampling period
    double T_;

  };
}

#endif /* _SUPPORT_FSM_ */

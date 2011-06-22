/*
 * Copyright 2011
 *
 * Andrei Herdt
 *
 * JRL, CNRS/AIST, INRIA Grenoble-Rhone-Alpes
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
 */

/// \doc Simulate a rigid body 


#ifndef _RIGID_BODY_
#define _RIGID_BODY_

#include <jrl/mal/matrixabstractlayer.hh>
#include <deque>
#include <jrl/walkgen/pgtypes.hh>
#include <privatepgtypes.h>

namespace PatternGeneratorJRL
{
  class RigidBody
  {

    //
    // Public types
    //
  public:

    /// \name Axes
    /// \{
    const static int X_AXIS = 0;
    const static int Y_AXIS = 1;
    const static int Z_AXIS = 2;
    const static int PITCH = 3;
    const static int ROLL = 4;
    const static int YAW = 5;
    /// \}

    /// \name Axes
    /// \{
    const static int POSITION = 10;
    const static int VELOCITY = 11;
    const static int ACCELERATION = 12;
    const static int JERK = 13;
    const static int COP = 14;
    /// \}

    /// \brief State vectors
    struct rigid_body_state_s
    {
      /// \name Translational degrees of freedom
      /// \{
      boost_ublas::vector<double> X;
      boost_ublas::vector<double> Y;
      boost_ublas::vector<double> Z;
      /// \}
      /// \name Rotational degrees of freedom
      /// \{
      boost_ublas::vector<double> Pitch;
      boost_ublas::vector<double> Roll;
      boost_ublas::vector<double> Yaw;
      /// \}

      struct rigid_body_state_s & operator=(const rigid_body_state_s &RB);

      void reset();

      rigid_body_state_s();
    };
    typedef struct rigid_body_state_s rigid_body_state_t;

    /// \name Dynamics matrices
    /// \{
    struct linear_dynamics_s
    {
      /// \brief Control matrix
      boost_ublas::matrix<double> U;
      /// \brief Transpose of control matrix
      boost_ublas::matrix<double> UT;

      /// \brief State matrix
      boost_ublas::matrix<double> S;

      int Type;
    };
    typedef linear_dynamics_s linear_dynamics_t;
    /// \}

    //
    // Public methods
    //
  public:

    RigidBody();

    ~RigidBody();

    /// \brief Initialize
    ///
    /// \return 0
    int initialize();

    /// \brief Interpolate
    int interpolate(std::deque<COMState> &COMStates,
		      std::deque<ZMPPosition> &ZMPRefPositions,
		      int CurrentPosition,
		      double CX, double CY);

    /// \brief Increment the state
    ///
    /// \param[in] Control Control vector
    rigid_body_state_t increment_state( double Control );

    /// \brief Decouple degree of freedom by injected trajectory
    ///
    /// \param[in] Axis The axis to be decoupled
    /// \param[in] Trajectory The injected trajectory
    ///
    /// \return 0
    int inject_trajectory( unsigned int Axis, boost_ublas::vector<double> Trajectory );

    /// \name Accessors
    /// \{
    linear_dynamics_t const & Dynamics( int Type ) const;
    linear_dynamics_t & Dynamics( int Type );

    inline double const & SamplingPeriodSim( ) const
    { return T_; }
    inline void SamplingPeriodSim( double T )
    { T_ = T; }

    inline double const & SamplingPeriodAct( ) const
    { return Ta_; }
    inline void SamplingPeriodAct( double Ta )
    { Ta_ = Ta; }

    inline double const & Mass( ) const
    { return Mass_; }
    inline void Mass( double Mass )
    { Mass_ = Mass; }
    /// \}

    
    //
    // Private members
    //
  private:

    /// \brief State
    rigid_body_state_t State_;

    /// \name Dynamics
    /// \{
    linear_dynamics_t
    PositionDynamics_,
    VelocityDynamics_,
    AccelerationDynamics_,
    JerkDynamics_;
    /// \}
    
    /// \brief Sampling period simulation
    double T_;

    /// \brief Recalculation period
    /// The state is incremented with respect to this parameter
    double Tr_;
    
    /// \brief Sampling period actuators
    double Ta_;

    /// \brief Nb previewed samples
    unsigned int N_;
    
    /// \brief Mass
    double Mass_;

  };
}
#endif /* _RIGID_BODY_ */

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


#include <deque>
#include <jrl/walkgen/pgtypes.hh>
#include <privatepgtypes.hh>

namespace PatternGeneratorJRL
{

  /// \brief State vectors
  struct rigid_body_state_s
  {
    /// \name Translational degrees of freedom
    /// \{
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    Eigen::VectorXd Z;
    /// \}
    /// \name Rotational degrees of freedom
    /// \{
    Eigen::VectorXd Pitch;
    Eigen::VectorXd Roll;
    Eigen::VectorXd Yaw;
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
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> U;

    /// \brief Inverse of control matrix
    Eigen::MatrixXd Um1;

    /// \brief Transpose of control matrix
    Eigen::MatrixXd UT;

    /// \brief State matrix
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> S;

    dynamics_e Type;

    void clear()
    {
      U.setZero();
      UT.setZero();
      S.setZero();
    }
  };
  typedef linear_dynamics_s linear_dynamics_t;
  /// \}


  class RigidBody
  {

    //
    // Public methods
    //
  public:

    RigidBody();

    ~RigidBody();

    /// \brief Interpolate
    int interpolate(std::deque<COMState> &COMStates,
                    std::deque<ZMPPosition> &ZMPRefPositions,
                    int CurrentPosition,
                    double CX, double CY);

    /// \brief Initialize
    ///
    /// \return 0
    int initialize();

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
    int inject_trajectory( unsigned int Axis, Eigen::VectorXd Trajectory );

    /// \name Accessors
    /// \{
    linear_dynamics_t const & Dynamics( dynamics_e ) const;
    linear_dynamics_t & Dynamics( dynamics_e );

    inline double const & SamplingPeriodSim( ) const
    {
      return T_;
    }
    inline void SamplingPeriodSim( double T )
    {
      T_ = T;
    }

    inline double const & SamplingPeriodAct( ) const
    {
      return Ta_;
    }
    inline void SamplingPeriodAct( double Ta )
    {
      Ta_ = Ta;
    }

    inline unsigned const & NbSamplingsPreviewed( ) const
    {
      return N_;
    }
    inline void NbSamplingsPreviewed( unsigned N )
    {
      N_ = N;
    }

    inline double const & Mass( ) const
    {
      return Mass_;
    }
    inline void Mass( double Mass )
    {
      Mass_ = Mass;
    }

    inline std::deque<rigid_body_state_t> & Trajectory()
    {
      return Trajectory_;
    }

    inline rigid_body_state_t & State()
    {
      return State_;
    }
    inline rigid_body_state_t const & State() const
    {
      return State_;
    }
    /// \}


    //
    // Private member functions
    //
  private:

    //
    // Private members
    //
  private:

    /// \brief State
    rigid_body_state_t State_;

    /// \brief Trajectory vector of decoupled DoF's
    std::deque<rigid_body_state_t> Trajectory_;

    /// \name Dynamics
    /// \{
    linear_dynamics_t
    PositionDynamics_,
      VelocityDynamics_,
      AccelerationDynamics_,
      JerkDynamics_,
      CoPDynamics_;
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

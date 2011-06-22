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


#ifndef _RIGID_BODY_SYSTEM_
#define _RIGID_BODY_SYSTEM_

#include <PreviewControl/rigid-body.hh>

namespace PatternGeneratorJRL
{
  class  RigidBodySystem
  {

    //
    // Public methods
    //
  public:

    RigidBodySystem();

    ~RigidBodySystem();

    /// \brief Initialize
    void initialize();

//    /// \brief Interpolate
//    int interpolate(std::deque<COMState> &COMStates,
//		      std::deque<ZMPPosition> &ZMPRefPositions,
//		      int CurrentPosition,
//		      double CX, double CY);
//
//    /// \brief Increment the state
//    ///
//    /// \param[in] Control Control vector
//    void increment_state( double Control );
//
//    /// \brief Decouple degree of freedom by injected trajectory
//    ///
//    /// \param[in] Axis The axis to be decoupled
//    /// \param[in] Trajectory The injected trajectory
//    ///
//    /// \return 0
//    int inject_trajectory( unsigned int Axis, boost_ublas::vector<double> Trajectory );

    /// \name Accessors and mutators
    /// \{
    inline const RigidBody operator ()() const
    {return CoM_;};
    inline void operator ()( RigidBody Body )
    {CoM_ = Body;};

    inline double const & SamplingPeriodSim( ) const
    { return T_; }
    inline void SamplingPeriodSim( double T )
    { T_ = T; }

    inline double const & SamplingPeriodAct( ) const
    { return Ta_; }
    inline void SamplingPeriodAct( double Ta )
    { Ta_ = Ta; }

    inline unsigned const & NbSamplingsPreviewed( ) const
    { return N_; }
    inline void NbSamplingsPreviewed( unsigned N )
    { N_ = N; }

    inline double const & Mass( ) const
    { return Mass_; }
    inline void Mass( double Mass )
    { Mass_ = Mass; }

    inline double const & CoMHeight( ) const
    { return CoMHeight_; }
    inline void CoMHeight( double Height )
    { CoMHeight_ = Height; }
    /// \}

    
    //
    // Private methods
    //
  private:

    /// \brief Initialize dynamics matrices
    ///
    /// \param[in] Dynamics Matrices to be filled
    void initialize_dynamics( RigidBody::linear_dynamics_t & Dynamics );

    //
    // Private members
    //
  private:

    /// \brief Body
    RigidBody CoM_;
    
    /// \brief Total robot mass
    double Mass_;

    /// \brief CoMHeight
    double CoMHeight_;

    /// \brief Sampling period simulation
    double T_;

    /// \brief Recalculation period
    /// The state is incremented with respect to this parameter
    double Tr_;
    
    /// \brief Sampling period actuators
    double Ta_;
    
    /// \brief Nb previewed samples
    unsigned int N_;

  };
}
#endif /* _RIGID_BODY_SYSTEM_ */

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
#include <privatepgtypes.h>
#include <PreviewControl/SupportFSM.h>
#include <FootTrajectoryGeneration/OnLineFootTrajectoryGeneration.h>
#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>

//#include <jrl/walkgen/pgtypes.hh>

namespace PatternGeneratorJRL
{
  class  RigidBodySystem
  {

    //
    // Public methods
    //
  public:

    RigidBodySystem( SimplePluginManager *SPM, CjrlHumanoidDynamicRobot *aHS, SupportFSM * FSM );

    ~RigidBodySystem();

    /// \brief Initialize
    void initialize( );

    /// \brief Interpolate
    ///
    /// \param[in] Result Optimization result
    /// \param[in] FinalZMPTraj_deq
    /// \param[in] FinalCOMTraj_deq
    /// \param[in] FinalLeftFootTraj_deq
    /// \param[in] FinalRightFootTraj_deq
    ///
    /// \return 0
    int interpolate( solution_t Result,
        std::deque<ZMPPosition> & FinalZMPTraj_deq,
        std::deque<COMState> & FinalCOMTraj_deq,
        std::deque<FootAbsolutePosition> &FinalLeftFootTraj_deq,
        std::deque<FootAbsolutePosition> &FinalRightFootTraj_deq );

    /// \brief Update feet matrices
    ///
    /// \param[in] FSM Finite state machine
    /// \param[in] CurrentSupport The current support state
    /// \param[in] Time Current time
    ///
    /// \return 0
    int update( std::deque<support_state_t> & SupportStates_deq,
        const std::deque<FootAbsolutePosition> & LeftFootTraj_deq,
        const std::deque<FootAbsolutePosition> & RightFootTraj_deq );

    /// \brief Initialize dynamics of the body center
    /// Suppose a piecewise constant jerk
    ///
    /// return 0
    int compute_dyn_cjerk();

    /// \brief Generate trajectories
    ///
    /// \param[in]
    /// \param[in]
    /// \param[in]
    /// \param[in]
    /// \param[in]
    ///
    /// return 0
    int generate_trajectories( double Time, const support_state_t & CurrentSupport, const solution_t & Result,
        const std::deque<support_state_t> & SupportStates_deq, const std::deque<double> & PreviewedSupportAngles_deq,
              std::deque<FootAbsolutePosition> & LeftFootTraj_deq, std::deque<FootAbsolutePosition> & RightFootTraj_deq);

    /// \name Accessors and mutators
    /// \{
    linear_dynamics_t const & DynamicsCoPJerk() const
    { return CoPDynamicsJerk_; }
    linear_dynamics_t & DynamicsCoPJerk()
    { return CoPDynamicsJerk_; }

    inline RigidBody const & CoM() const
    {return CoM_;};
    inline void CoM( const RigidBody & CoM )
    {CoM_ = CoM;};

    inline RigidBody const & LeftFoot() const
    {return LeftFoot_;};
    inline RigidBody & LeftFoot()
    {return LeftFoot_;};
    inline void LeftFoot( const RigidBody & LeftFoot )
    {LeftFoot_ = LeftFoot;};

    inline RigidBody const & RightFoot() const
    {return RightFoot_;};
    inline RigidBody & RightFoot()
    {return RightFoot_;};
    inline void RightFoot( const RigidBody & RightFoot )
    {RightFoot_ = RightFoot;};

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

    /// \brief Initialize dynamics of the CoP
    ///
    /// \param[in] Dynamics Matrices to be filled
    ///
    /// return 0
    int compute_dyn_cop( std::deque<support_state_t> & SupportStates_deq );

    /// \brief Initialize dynamics of the body center
    /// Suppose a piecewise constant jerk
    /// \param[in] Dynamics Matrices to be filled
    ///
    /// return 0
    int compute_dyn_cjerk( linear_dynamics_t & Dynamics );

    /// \brief Initialize dynamics of the body center
    /// Suppose a higher order polynomial
    /// \param[in] Dynamics Matrices to be filled
    ///
    /// return 0
    int compute_dyn_pol_feet( std::deque<support_state_t> & SupportStates_deq,
        linear_dynamics_t & LeftFootDynamics, linear_dynamics_t & RightFootDynamics);

    /// \brief Initialize trajectory constants
    int initialize_trajectories();

    /// \brief Compute predefined trajectories
    int precompute_trajectories( std::deque<support_state_t> & SupportStates_deq );

    /// \brief Compute a row of the dynamic matrix Sp
    int compute_sbar( double * Spbar, double * Sabar, double T, double Td );

    /// \brief Compute a row of the dynamic matrix Up
    int compute_ubar( double * Upbar, double * Uabar, double T, double Td );


    //
    // Private members
    //
  private:

    /// \brief Bodies
    RigidBody
    CoM_,
    LeftFoot_,
    RightFoot_,
    LeftWrist_,
    RightWrist_;

    /// \brief Center of Pressure dynamics
    /// Divided into two parts to reflect the two different
    /// parametrizations(piecewise jerk and single polynomials)
    linear_dynamics_t
    CoPDynamicsJerk_;

    /// \brief Fixed trajectories
    std::deque<rigid_body_state_t>
    FlyingFootTrajectory_deq_;

    /// \brief Ground reaction force for the whole preview window
    std::deque<double> GRF_deq_;

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

    /// \brief Standard polynomial trajectories for the feet.
    OnLineFootTrajectoryGeneration * OFTG_;

    /// \brief Finite State Machine
    SupportFSM * FSM_;

    CjrlHumanoidDynamicRobot * HDR_;

  };
}
#endif /* _RIGID_BODY_SYSTEM_ */

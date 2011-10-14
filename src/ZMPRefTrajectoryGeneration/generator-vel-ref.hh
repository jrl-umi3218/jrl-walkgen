/*
 * Copyright 2010,
 *
 * Olivier  Stasse
 * Andrei   Herdt
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
/*! This object constructs a QP as proposed by Herdt IROS 2010.
 */

#ifndef GENERATORVELREF_HH_
#define GENERATORVELREF_HH_


#include <ZMPRefTrajectoryGeneration/mpc-trajectory-generation.hh>

#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
#include <ZMPRefTrajectoryGeneration/qp-problem.hh>
#include <PreviewControl/SupportFSM.h>
#include <PreviewControl/LinearizedInvertedPendulum2D.h>
#include <PreviewControl/rigid-body-system.hh>
#include <PreviewControl/rigid-body.hh>
#include <Mathematics/intermediate-qp-matrices.hh>
#include <Mathematics/relative-feet-inequalities.hh>

#include <privatepgtypes.h>
#include <cmath>

namespace PatternGeneratorJRL
{
 
  /// \brief Generate optimization problem as proposed in Herdt2010IROS
  class  GeneratorVelRef : public MPCTrajectoryGeneration
  {

    //
    //Public methods
    //
  public:

    /// \name Constructors and destructors.
    /// \{
    GeneratorVelRef( SimplePluginManager *lSPM, IntermedQPMat * Data, RigidBodySystem * Robot );
    ~GeneratorVelRef();
    /// \}


    /// \brief Preview support state for the whole preview period
    ///
    /// \param[in] Time Current time
    /// \param[in] FSM Finite state machine
    /// \param[in] FinalLeftFootTraj_deq
    /// \param[in] FinalRightFootTraj_deq
    /// \param[out] SupportStates_deq
    void preview_support_states( double Time, const SupportFSM * FSM,
        const deque<FootAbsolutePosition> & FinalLeftFootTraj_deq, const deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
        deque<support_state_t> & SupportStates_deq );

    /// \brief Set the global reference from the local one and the orientation of the trunk frame
    /// for the whole preview window
    ///
    /// \param[in] Solution
    void compute_global_reference( const solution_t & Solution );

    /// \brief Initialize intermediate matrices
    ///
    void initialize_matrices();

    /// \brief Compute constraints
    ///
    /// \param[out] Pb
    /// \param[in] RFI Inequalities generator
    /// \param[in] LeftFootPositions_deq
    /// \param[in] RightFootPositions_deq
    /// \param[in] PrwSupportStates_deq
    /// \param[in] PrwSupportAngles_deq
    void build_constraints( QPProblem & Pb,
        RelativeFeetInequalities * RFI,
        const std::deque< FootAbsolutePosition> & LeftFootPositions_deq,
        const std::deque<FootAbsolutePosition> & RightFootPositions_deq,
        const std::deque<support_state_t> & PrwSupportStates_deq,
        const std::deque<double> & PrwSupportAngles_deq);

    /// \brief Build the constant part of the objective
    ///
    /// \param[in] Pb
    void build_invariant_part(QPProblem & Pb);

    /// \brief Compute the objective matrices
    ///
    /// \param[in] Pb
    /// \param[in] SupportStates_deq
    void update_problem(QPProblem & Pb, const std::deque<support_state_t> & SupportStates_deq);


    /// \brief Compute the initial solution vector for warm start
    ///
    /// \param[in] initial_solution initial solution vector
    /// \param[in] PrwSupportStates_deq   PrwSupportStates_deq
    /// \param[in] PrwSupportAngles_deq   PrwSupportAngles_deq
    void computeWarmStart(solution_t & Solution);

    /// \name Accessors
    /// \{
    /// \brief Set the weights on an objective term
    ///
    /// \param[in] Weight
    /// \param[in] Type Objective type
    void Ponderation(double Weight, int Type );

    inline void Reference(const reference_t & Ref)
    {  IntermedData_->Reference(Ref); };
    inline void SupportState(const support_state_t & SupportState)
    { IntermedData_->SupportState(SupportState); };
    inline void CoM(const com_t & CoM)
    { IntermedData_->CoM(CoM); };
    /// \}

    //
    // Protected methods
    //
  protected:

    /// \brief Compute the selection matrices
    ///
    /// \param[in] deqSupportStates
    void generate_selection_matrices( const std::deque<support_state_t> & deqSupportStates);

    /// \brief Generate a queue of inequalities with respect to the centers of the feet
    ///
    /// \param[out] Inequalities
    /// \param[in] FCALS
    /// \param[in] AbsoluteLeftFootPositions
    /// \param[in] AbsoluteRightFootPositions
    /// \param[in] deqSupportStates
    /// \param[in] PreviewedSupportAngles
    void build_inequalities_cop(linear_inequality_t & Inequalities,
        RelativeFeetInequalities * FCALS,
        const std::deque< FootAbsolutePosition> & LeftFootPositions_deq,
        const std::deque<FootAbsolutePosition> & RightFootPositions_deq,
        const std::deque<support_state_t> & SupportStates_deq,
        const std::deque<double> & PreviewedSupportAngles_deq) const;

    /// \brief Generate a queue of inequality constraints on
    /// the feet positions with respect to previous foot positions
    ///
    /// \param[out] Inequalities
    /// \param[in] FCALS
    /// \param[in] LeftFootPositions_deq
    /// \param[in] RightFootPositions_deq
    /// \param[in] SupportStates_deq
    /// \param[in] PreviewedSupportAngles_deq
    void build_inequalities_feet(linear_inequality_t & Inequalities,
        RelativeFeetInequalities * FCALS,
        const std::deque< FootAbsolutePosition> & LeftFootPositions_deq,
        const std::deque<FootAbsolutePosition> & RightFootPositions_deq,
        const std::deque<support_state_t> & SupportStates_deq,
        const std::deque<double> & PreviewedSupportAngles_deq) const;

    /// \brief Compute CoP constraints corresponding to the set of inequalities
    ///
    /// \param[in] IneqCop
    /// \param[in] NbStepsPreviewed
    /// \param[out] Pb
    void build_constraints_cop( const linear_inequality_t & IneqCoP, unsigned int NbStepsPreviewed,
         QPProblem & Pb );

    /// \brief Compute feet constraints corresponding to the set of inequalities
    ///
    /// \param[in] IneqFeet
    /// \param[in] State
    /// \param[in] NbStepsPreviewed
    /// \param[out] Pb
    void build_constraints_feet(const linear_inequality_t & IneqFeet,
        const IntermedQPMat::state_variant_t & State,
        int NbStepsPreviewed, QPProblem & Pb);

    /// \brief Compute feet equality constraints from a trajectory
    ///
    /// \param[in] SupportStates_deq
    /// \param[in] NbStepsPreviewed
    /// \param[out] Pb
    void build_eq_constraints_feet( const std::deque<support_state_t> & SupportStates_deq,
        unsigned int NbStepsPreviewed, QPProblem & Pb );

    /// \brief Initialize inequality matrices
    ///
    /// \param[out] Inequalities
    void initialize_matrices( linear_inequality_t & Inequalities);

    /// \brief Scaled product\f$ weight*M*M \f$
    void compute_term(MAL_MATRIX (&weightMM, double),
        double weight, const MAL_MATRIX (&M1, double), const MAL_MATRIX (&M2, double));

    /// \brief Scaled product \f$ weight*M*V \f$
    void compute_term(MAL_VECTOR (&weightMV, double),
        double weight, const MAL_MATRIX (&M, double), const MAL_VECTOR (&V, double));

    /// \brief Scaled product \f$ weight*M*V*scalar \f$
    void compute_term(MAL_VECTOR (&weightMV, double),
        double weight, const MAL_MATRIX (&M, double),
        const MAL_VECTOR (&V, double), const double scalar);

    /// \brief Scaled product \f$ weight*M*M*V \f$
    void compute_term(MAL_VECTOR (&weightMV, double),
        double weight, const MAL_MATRIX (&M1, double),
        const MAL_MATRIX (&M2, double), const MAL_VECTOR (&V2, double));


    //
    //Protected members
    //
  protected:

    IntermedQPMat * IntermedData_;
    RigidBodySystem * Robot_;


    //
    //Private members
    //
  private:

    /// \name Temporary vectors TODO: Move to intermediate data?
    /// \{
    boost_ublas::matrix<double> MM_;
    boost_ublas::vector<double> MV_;
    boost_ublas::vector<double> MV2_;
    /// \}


  };
}

#endif /* GENERATORVELREF_HH_ */

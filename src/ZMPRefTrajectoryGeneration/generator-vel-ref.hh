/*
 * Copyright 2010,
 *
 * Andrei   Herdt
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
/*! This object constructs a QP as proposed by Herdt IROS 2010.
 */

#ifndef GENERATORVELREF_HH_
#define GENERATORVELREF_HH_


#include <ZMPRefTrajectoryGeneration/mpc-trajectory-generation.hh>

#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
#include <ZMPRefTrajectoryGeneration/qp-problem.hh>
#include <PreviewControl/SupportFSM.h>
#include <PreviewControl/LinearizedInvertedPendulum2D.h>
#include <Mathematics/intermediate-qp-matrices.hh>
#include <Mathematics/relative-feet-inequalities.hh>

#include <privatepgtypes.h>


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
    GeneratorVelRef( SimplePluginManager *lSPM, IntermedQPMat * Data );
    ~GeneratorVelRef();
    /// \}


    /// \brief Preview support state for the whole preview period
    ///
    /// \param[in] FSM
    /// \param[out] deqSupportStates
    void preview_support_states( const SupportFSM * FSM, std::deque<support_state_t> & SupportStates_deq );

    /// \brief Set the global reference from the local one and the orientation of the trunk frame
    /// for the whole preview window
    ///
    /// \param[in] TrunkStateT State of the trunk at the end of the acceleration phase
    void compute_global_reference( const std::deque<COMState> & TrunkStates_deq );

    /// \brief Initialize intermediate matrices
    ///
    void initialize_matrices();

    /// \brief Compute constraints
    ///
    /// \param[in] Pb
    /// \param[in] FCALS
    /// \param[in] AbsoluteLeftFootPositions
    /// \param[in] AbsoluteRightFootPositions
    /// \param[in] deqSupportStates
    /// \param[in] PreviewedSupportAngles
    void build_constraints( QPProblem & Pb,
        RelativeFeetInequalities * RFI,
        const std::deque< FootAbsolutePosition> & LeftFootPositions_deq,
        const std::deque<FootAbsolutePosition> & RightFootPositions_deq,
        const std::deque<support_state_t> & SupportStates_deq,
        const std::deque<double> & PreviewedSupportAngles_deq);

    /// \brief Build the constant part of the objective
    ///
    /// \param[in] Pb
    void build_invariant_part(QPProblem & Pb);

    /// \brief Compute the objective matrices
    ///
    /// \param[in] Pb
    /// \param[in] deqSupportStates
    void update_problem(QPProblem & Pb, const std::deque<support_state_t> & SupportStates_deq);

    /// \name Accessors
    /// \{
    /// \brief Set the weights on an objective term
    ///
    /// \param[in] Weight
    /// \param[in] Type Objective type
    void Ponderation(double Weight, int Type );

    inline void Reference(const reference_t & Ref)
    {  Matrices_->Reference(Ref); };
    inline void SupportState(const support_state_t & SupportState)
    { Matrices_->SupportState(SupportState); };
    inline void CoM(const com_t & CoM)
    { Matrices_->CoM(CoM); };
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
    /// \param[in] AbsoluteLeftFootPositions
    /// \param[in] AbsoluteRightFootPositions
    /// \param[in] deqSupportStates
    /// \param[in] PreviewedSupportAngles
    void build_inequalities_feet(linear_inequality_t & Inequalities,
        RelativeFeetInequalities * FCALS,
        const std::deque< FootAbsolutePosition> & LeftFootPositions_deq,
        const std::deque<FootAbsolutePosition> & RightFootPositions_deq,
        const std::deque<support_state_t> & SupportStates_deq,
        const std::deque<double> & PreviewedSupportAngles_deq) const;

    /// \brief Compute CoP constraints corresponding to the set of inequalities
    ///
    /// \param[in] IneqCop
    /// \param[in] CoP
    /// \param[in] State
    /// \param[in] NbStepsPreviewed
    /// \param[in] Pb
    void build_constraints_cop(const linear_inequality_t & IneqCoP,
        const IntermedQPMat::dynamics_t & CoP,
        const IntermedQPMat::state_variant_t & State,
        int NbStepsPreviewed, QPProblem & Pb);

    /// \brief Compute feet constraints corresponding to the set of inequalities
    ///
    /// \param[in] IneqFeet
    /// \param[in] State
    /// \param[in] NbStepsPreviewed
    /// \param[in] Pb
    void build_constraints_feet(const linear_inequality_t & IneqFeet,
        const IntermedQPMat::state_variant_t & State,
        int NbStepsPreviewed, QPProblem & Pb);

    /// \brief Initialize intermediate matrices
    ///
    /// \param[in] Objective
    void initialize_matrices( IntermedQPMat::dynamics_t & Objective);

    /// \brief Initialize inequality matrices
    ///
    /// \param[in] Objective
    void initialize_matrices( linear_inequality_t & Inequalities);

    /// \brief Scaled product\f$ weight*M*M \f$
    void compute_term(MAL_MATRIX (&weightMM, double),
        double weight, const MAL_MATRIX (&M1, double), const MAL_MATRIX (&M2, double));

    /// \brief Scaled product\f$ M*M \f$
    void compute_term(MAL_MATRIX (&MM, double),
        const MAL_MATRIX (&M1, double), const MAL_MATRIX (&M2, double));

    /// \brief Scaled product \f$ weight*M*V \f$
    void compute_term(MAL_VECTOR (&weightMV, double),
        double weight, const MAL_MATRIX (&M, double), const MAL_VECTOR (&V, double));

    /// \brief Scaled product \f$ weight*M*V*scalar \f$
    void compute_term(MAL_VECTOR (&weightMV, double),
        double weight, const MAL_MATRIX (&M, double),
        const MAL_VECTOR (&V, double), const double scalar);

    /// \brief Scaled product \f$ weight*M*M*V \f$
    void compute_term(MAL_VECTOR (&weightMV, double),
        double weight, const MAL_MATRIX (&M1, double), MAL_VECTOR (&V1, double),
        const MAL_MATRIX (&M2, double), const MAL_VECTOR (&V2, double));


    //
    //Protected members
    //
  protected:

    IntermedQPMat * Matrices_;


  };
}

#endif /* GENERATORVELREF_HH_ */

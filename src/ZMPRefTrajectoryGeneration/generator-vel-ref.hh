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
#include <Mathematics/FootConstraintsAsLinearSystemForVelRef.h>

#include <privatepgtypes.h>


namespace PatternGeneratorJRL
{
 
  class  GeneratorVelRef : public MPCTrajectoryGeneration
  {

    //
    //Public methods
    //
  public:
    /// \name Constructors and destructors.
    /// \{
    GeneratorVelRef( SimplePluginManager *lSPM );
    ~GeneratorVelRef();
    /// \}

//    /// \brief Call method to handle the plugins (SimplePlugin interface).
//    void CallMethod(std::string &Method, std::istringstream &strm);

    /// \brief Set the weights on the objective terms
    ///
    /// \param[out] Matrices
    /// \param[in] weight
    /// \param[in] objective
    void setPonderation( IntermedQPMat & Matrices, double weight, int objective ) const;

        /// \brief get the weights on the objective terms
    ///
    /// \param[in] Matrices
    /// \param[in] objective
    double getPonderation(const IntermedQPMat & Matrices, int objective ) const;


    /// \brief Set the velocity reference from string
    ///
    /// \param[in] strm velocity reference string
    void setReference(std::istringstream &strm) const;

    /// \brief Preview support state for the whole preview period
    ///
    /// \param[in] Matrices
    /// \param[in] FSM
    /// \param[out] deqSupportStates
    void previewSupportStates(IntermedQPMat & Matrices,
			      const SupportFSM * FSM, std::deque<support_state_t> & deqSupportStates) const;

    /// \brief Compute the selection matrices
    ///
    /// \param[out] Matrices
    /// \param[in] deqSupportStates
    void generateSelectionMatrices( IntermedQPMat & Matrices,
				   const std::deque<support_state_t> & deqSupportStates) const;

    /// \brief Set the global reference from the local one and the orientation of the trunk frame
    /// for the whole preview window
    ///
    /// \param[out] Matrices
    /// \param[in] TrunkStateT State of the trunk at the end of the acceleration phase
    void computeGlobalReference(IntermedQPMat & Matrices, const COMState & TrunkStateT) const;

    /// \brief Initialize the optimization programm
    ///
    /// \param[out] Matrices
    void initialize(IntermedQPMat & Matrices) const;

//    /// \brief Add one equality constraint to the queue of constraints
//    ///
//    /// \param[in] DU
//    /// \param[in] DS
//    void addEqConstraint(std::deque<linear_inequality_ff_t> ConstraintsDeque,
//			 MAL_MATRIX (&DU, double), MAL_MATRIX (&DS, double));
//
//    /// \brief Add one inequality constraint to the queue of constraints
//    ///
//    /// \param[in] DU
//    /// \param[in] DS
//    void addIneqConstraint(std::deque<linear_inequality_ff_t> ConstraintsDeque,
//			   MAL_MATRIX (&DU, double), MAL_MATRIX (&DS, double));

    /// \brief Generate a queue of inequalities with respect to the centers of the feet
    ///
    /// \param[out] Inequalities
    /// \param[in] FCALS
    /// \param[in] AbsoluteLeftFootPositions
    /// \param[in] AbsoluteRightFootPositions
    /// \param[in] deqSupportStates
    /// \param[in] PreviewedSupportAngles
    void buildInequalitiesCoP(linear_inequality_t & Inequalities,
			      FootConstraintsAsLinearSystemForVelRef * FCALS,
			      const std::deque< FootAbsolutePosition> & AbsoluteLeftFootPositions,
			      const std::deque<FootAbsolutePosition> & AbsoluteRightFootPositions,
			      const std::deque<support_state_t> & deqSupportStates,
			      const std::deque<double> & PreviewedSupportAngles) const;

    /// \brief Generate a queue of inequality constraints on
    /// the feet positions with respect to previous footpositions
    ///
    /// \param[out] Inequalities
    /// \param[in] FCALS
    /// \param[in] AbsoluteLeftFootPositions
    /// \param[in] AbsoluteRightFootPositions
    /// \param[in] deqSupportStates
    /// \param[in] PreviewedSupportAngles
      void buildInequalitiesFeet(linear_inequality_t & Inequalities,
			       FootConstraintsAsLinearSystemForVelRef * FCALS,
			       const std::deque< FootAbsolutePosition> & AbsoluteLeftFootPositions,
			       const std::deque<FootAbsolutePosition> & AbsoluteRightFootPositions,
			       const std::deque<support_state_t> & deqSupportStates,
			       const std::deque<double> & PreviewedSupportAngles) const;

      /// \brief Compute CoP constraints corresponding to the set of inequalities
      ///
      /// \param[in] IneqCop
      /// \param[in] CoP
      /// \param[in] State
      /// \param[in] NbStepsPreviewed
      /// \param[in] Pb
    void buildConstraintsCoP(const linear_inequality_t & IneqCoP,
			     const IntermedQPMat::dynamics_t & CoP,
			     const IntermedQPMat::state_variant_t & State,
			     int NbStepsPreviewed, QPProblem & Pb);


    /// \brief Compute feet constraints corresponding to the set of inequalities
    ///
    /// \param[in] IneqFeet
    /// \param[in] State
    /// \param[in] NbStepsPreviewed
    /// \param[in] Pb
    void buildConstraintsFeet(const linear_inequality_t & IneqFeet,
			      const IntermedQPMat::state_variant_t & State,
			      int NbStepsPreviewed, QPProblem & Pb);

    /// \brief Compute constraints
    ///
    /// \param[in] Matrices
    /// \param[in] Pb
    /// \param[in] FCALS
    /// \param[in] AbsoluteLeftFootPositions
    /// \param[in] AbsoluteRightFootPositions
    /// \param[in] deqSupportStates
    /// \param[in] PreviewedSupportAngles
    void buildConstraints(IntermedQPMat & Matrices, QPProblem & Pb,
			  FootConstraintsAsLinearSystemForVelRef * FCALS,
			  const std::deque< FootAbsolutePosition> & AbsoluteLeftFootPositions,
			  const std::deque<FootAbsolutePosition> & AbsoluteRightFootPositions,
			  const std::deque<support_state_t> & deqSupportStates,
			  const std::deque<double> & PreviewedSupportAngles);

    /// \brief Build the constant part of the objective
    ///
    /// \param[in] Pb
    /// \param[in] Matrices
    void buildInvariantPart(QPProblem & Pb, const IntermedQPMat & Matrices);

    /// \brief Compute the objective matrices
    ///
    /// \param[in] Pb
    /// \param[in] Matrices
    /// \param[in] deqSupportStates
    void updateProblem(QPProblem & Pb, const IntermedQPMat & Matrices,
		       const std::deque<support_state_t> & deqSupportStates);
	  

    //
    //Private methods
    //
  private:

    /// \brief Initialize objective matrices
    ///
    /// \param[in] Objective
    void initializeMatrices( IntermedQPMat::dynamics_t & Objective) const;

    /// \brief Initialize inequality matrices
    ///
    /// \param[in] Objective
    void initializeMatrices( linear_inequality_t & Inequalities) const;

    /// \brief Scaled product\f$ weight*M*M \f$
    void computeTerm(MAL_MATRIX (&weightMM, double),
		     double weight, const MAL_MATRIX (&M1, double), const MAL_MATRIX (&M2, double));

    /// \brief Scaled product\f$ M*M \f$
    void computeTerm(MAL_MATRIX (&MM, double),
		     const MAL_MATRIX (&M1, double), const MAL_MATRIX (&M2, double));

    /// \brief Scaled product \f$ weight*M*V \f$
    void computeTerm(MAL_VECTOR (&weightMV, double),
		     double weight, const MAL_MATRIX (&M, double), const MAL_VECTOR (&V, double));

    /// \brief Scaled product \f$ weight*M*V*scalar \f$
    void computeTerm(MAL_VECTOR (&weightMV, double),
		     double weight, const MAL_MATRIX (&M, double),
		     const MAL_VECTOR (&V, double), const double scalar);

    /// \brief Scaled product \f$ weight*M*M*V \f$
    void computeTerm(MAL_VECTOR (&weightMV, double),
		     double weight, const MAL_MATRIX (&M1, double), MAL_VECTOR (&V1, double),
		     const MAL_MATRIX (&M2, double), const MAL_VECTOR (&V2, double));

	  
    //
    //Private members
    //
  private:


  };
}

#endif /* GENERATORVELREF_HH_ */

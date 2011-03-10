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

#include <ZMPRefTrajectoryGeneration/qp-problem.hh>
#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
#include <PreviewControl/SupportFSM.h>
#include <privatepgtypes.h>
#include <Mathematics/intermediate-qp-matrices.hh>
#include <PreviewControl/LinearizedInvertedPendulum2D.h>

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
    GeneratorVelRef(SimplePluginManager *lSPM, std::string DataFile,
		    CjrlHumanoidDynamicRobot *aHS=0);
    ~GeneratorVelRef();
    /// \}

    /// \brief Call method to handle the plugins (SimplePlugin interface).
    void CallMethod(std::string &Method, std::istringstream &strm);

    /// \brief Set the weights on the objective terms
    ///
    /// \param alpha
    /// \param beta
    /// \param gamma
    /// \param delta
    void setPonderation( IntermedQPMat Matrices, double weight, int objective );

    /// \brief Set the velocity reference from string
    ///
    /// \param strm velocity reference string
    void setReference(std::istringstream &strm);

    /// \brief Set the velocity reference from external reference
    ///
    /// \param dx
    /// \param dy
    /// \param dyaw
    void setReference(double dx, double dy, double dyaw);

    /// \brief Initialize the optimization programm
    ///
    /// \param Pb
    /// \param Matrices
    void initializeProblem(QPProblem & Pb, IntermedQPMat Matrices);

    /// \brief Initialize objective matrices
    ///
    /// \param Objective
    void initializeMatrices( IntermedQPMat::objective_variant_t & Objective);

    /// \brief Add one equality constraint to the queue of constraints
    ///
    /// \param DU
    /// \param DS
    void addEqConstraint(std::deque<LinearConstraintInequalityFreeFeet_t> ConstraintsDeque,
			 MAL_MATRIX (&DU, double), MAL_MATRIX (&DS, double));
	  
    /// \brief Add one inequality constraint to the queue of constraints
    ///
    /// \param DU
    /// \param DS
    void addIneqConstraint(std::deque<LinearConstraintInequalityFreeFeet_t> ConstraintsDeque,
			   MAL_MATRIX (&DU, double), MAL_MATRIX (&DS, double));
	  
    /// \brief Generate a queue of inequality constraints on the ZMP for the whole preview window
    ///
    /// \param CjrlFoot
    /// \param PreviewedSupportAngles
    /// \param SupportFSM
    /// \param CurrentSupportState
    void generateZMPConstraints (CjrlFoot & Foot,
				 std::deque < double > & PreviewedSupportAngles,
				 SupportFSM & FSM,
				 SupportState_t & CurrentSupportState,
				 QPProblem & Pb);

    /// \brief Generate a queue of inequality constraints on the feet positions for the whole preview window
    ///
    /// \param Foot
    /// \param PreviewedSupportAngles
    /// \param SupportFSM
    /// \param CurrentSupportState
    void generateFeetPosConstraints (CjrlFoot & Foot,
        std::deque < double > & PreviewedSupportAngles,
				     SupportFSM & ,
				     SupportState_t &,
				     QPProblem & Pb);

    /// \brief Build the constant part of the objective
    ///
    /// \param Pb
    void buildInvariantProblemPart(QPProblem & Pb, IntermedQPMat & Matrices);

    /// \brief Compute the objective matrices
    ///
    /// \param Pb
    void updateProblem(QPProblem & Pb, IntermedQPMat & Matrices);

    /// \brief Infitialize constant parts of the objective
    void initMatrices( IntermedQPMat::objective_variant_t & Matrix, int objective);
	  

    //
    //Private methods
    //
  private:

    /// \brief Compute a Least Squares objective term and add it to the optimization problem
    void updateProblem(double * Q, double *p,
        const IntermedQPMat::objective_variant_t & Objective,
        const IntermedQPMat::state_variant_t & State);

    /// \brief Compute a quadratic Least Squares objective term and add it to the optimization problem
    void updateProblem(double * Q, const IntermedQPMat::objective_variant_t & Objective);

    /// \brief Scaled product\f$ weight*M*M \f$
    void computeTerm(MAL_MATRIX (&weightMM, double),
        const double & weight, const MAL_MATRIX (&M1, double), const MAL_MATRIX (&M2, double));

    /// \brief Scaled product \f$ weight*M*V \f$
    void computeTerm(MAL_VECTOR (&weightMV, double),
        const double weight, const MAL_MATRIX (&M, double), const MAL_VECTOR (&V, double));

    /// \brief Scaled product \f$ weight*M*V*scalar \f$
    void computeTerm(MAL_VECTOR (&weightMV, double),
        const double weight, const MAL_MATRIX (&M, double),
        const MAL_VECTOR (&V, double), const double scalar);

    /// \brief Scaled product \f$ weight*M*M*V \f$
    void computeTerm(MAL_VECTOR (&weightMV, double),
        const double weight, const MAL_MATRIX (&M1, double), MAL_VECTOR (&V1, double),
        const MAL_MATRIX (&M2, double), const MAL_VECTOR (&V2, double));

    /// \brief Add the computed matrix to the final optimization problem in array form
    void addTerm(MAL_MATRIX (&Mat, double), double * QPMat, int row, int col, int nrows, int ncols);

    /// \brief Add the computed vector to the final optimization problem in array form
    void addTerm(MAL_VECTOR (&Vec, double), double * QPVec, int index, int nelem);

    /// \brief Build the constant part of the constraint matrices.
    int buildConstantPartOfConstraintMatrices();

    /// \brief This method does the same once the previous method has been called
    ///  to compute the static part of the optimization function.
    ///  Assuming that the optimization function is of the form
    ///  \f$ min_{u_k} \frac{1}{2} u^{\top}_k Q u_k + p^{\top}_k u_k \f$
    ///  this method computes \f$Q\f$, the constant part of $p^{\top}_k$.
    int buildConstantPartOfTheObjectiveFunction();


    /// \brief Compute constant matrices over all the instances of the problem, i.e.
    ///  \f$P_{pu}, P_{px}, P_{vs}, P_{vu}\f$.
    ///  The necessary parameters to build those matrices are extracted from the
    ///  PreviewControl link.
    int initializeMatrixPbConstants();


    /// \name Debugging related methods
    /// @{
    /// \brief Dump the instance of the quadratic problem for one iteration. */
    int dumpProblem(MAL_VECTOR(& xk,double),
		    double Time);
    /// @}
	  
	  
    //
    //Private members
    //
  private:
	  
    /// \brief The current and the previewed support state.
    SupportState_t m_CurrentSupport, m_PrwSupport;

    /// \brief Future support states
    std::deque<SupportState_t> m_PrwSupStates;
	  
    /// \brief Current state of the trunk and the trunk state after m_QP_T
    COMState m_TrunkState, m_TrunkStateT;
	  
    /// \brief Velocity reference (constant)
    ReferenceAbsoluteVelocity m_RefVel;
	  
    /// \brief Inequality constraints
    std::deque<LinearConstraintInequalityFreeFeet_t> m_LinearInequalities;
	  
    /// \brief History of support states
    std::deque<SupportFeet_t> m_SupportFeetDeque;
	  
    /// \brief Previewed trunk states 
    std::deque<COMState> m_TrunkStatesDeque;
	  
    /// \brief Additional term on the acceleration of the CoM
    MAL_VECTOR(m_PerturbationAcceleration,double);  
	  
    /// \brief Mass of the robot  
    double m_RobotMass;
	  
    /// \brief Perturbation
    bool m_PerturbationOccured;
	  
    /// \brief Distance between the feet
    double m_FeetDistanceDS;
	  
    /// \brief
    bool m_EndingPhase;
	  
    /// \brief 
    double m_TimeToStopOnLineMode;
	
    /// \brief Buffer to handle computational delay
    double m_TimeBuffer;


    /// \name Variables related to the QP
    /// @{
	  
    /// \brief Sampling of the QP. 
    double m_QP_T;
	  
    /// \brief Number of previewed samples 
    int m_QP_N;

    /// \brief Constant part of the constraint matrices. 
    double * m_Pu;
    /// @} 
	  

    /// \brief Debugging variable
    int m_FullDebug;

    /// \brief Fast formulations mode. 
    unsigned int m_FastFormulationMode;

  };
}

#endif /* GENERATORVELREF_HH_ */

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
    void setPonderation(double alpha, double beta, double gamma, double delta);

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

    /// \brief Build constant parts of the objective
    void buildConstantPartOfObjective(QPProblem & Pb);

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

    /// \brief Compute the objective matrices of a quadratic optimization problem
    ///
    /// \param Pb
    void computeObjective(QPProblem & Pb, IntermedQPMat & QPMat, std::deque<SupportState_t> PrwSupStates);

    /// \brief Infitialize constant parts of the objective
    /// \return A negative value in case of a problem 0 otherwise.
    int initConstants(QPProblem & Pb);
	  
    /// \brief Set the perturbation force on the CoM from external reference
    ///
    /// \param Fx
    /// \param Fy
    /// \param 2DLIPM
    void setCoMPerturbationForce(double Fx, double Fy, LinearizedInvertedPendulum2D & CoM);
	  
    /// \brief Set the perturbation force on the CoM from string
    ///
    /// \param strm
    /// \param 2DLIPM
    void setCoMPerturbationForce(std::istringstream & strm,
				 LinearizedInvertedPendulum2D & CoM);
	  

    //
    //Private methods
    //
  private:

    /// \brief Compute the Least Squares objective term and add it to the optimization problem
    void addLeastSquaresTerm(ObjectiveTerm_t & type, QPProblem & Pb, int NbSt);
	  
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
	  
    /// \brief Current and previewed support states. 
    SupportState_t m_CurrentSupport, m_PrwSupport;
	  
    /// \brief Current state of the trunk and the trunk state after m_QP_T
    COMState m_TrunkState, m_TrunkStateT;
	  
    /// Velocity reference (constant)
    ReferenceAbsoluteVelocity m_RefVel;
	  
    std::deque<LinearConstraintInequalityFreeFeet_t> m_ConstraintInequalitiesDeque;
    std::deque<LinearConstraintInequalityFreeFeet_t> m_FeetPosInequalitiesDeque;
	  
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
	  
    /// \name Weights on objective functions
    /// @{ 
    /// \brief Weight on the instant-velocity term in the objective
    double m_Beta;
	  
    /// \brief Weight on the jerk term in the objective.
    double m_Alpha;
	  
    /// \brief Weight on the ZMP term in the objective.
    double m_Gamma;
	  
    /// \brief Weight on the average-velocity term in the objective
    double m_Delta;
    /// @}
	  
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

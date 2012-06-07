/*
 * Copyright 2010,
 *
 * Medhi    Benallegue
 * Andrei   Herdt
 * Francois Keith
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
/*! This object provides the generation of ZMP and CoM trajectory
  using a new formulation of the stability problem.
*/

#ifndef _ZMPVELOCITYREFERENCEDQP_WITH_CONSTRAINT_H_
#define _ZMPVELOCITYREFERENCEDQP_WITH_CONSTRAINT_H_



#include <PreviewControl/LinearizedInvertedPendulum2D.hh>
#include <Mathematics/FootConstraintsAsLinearSystemForVelRef.hh>
#include <Mathematics/OptCholesky.hh>
#include <ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.hh>
#include <Mathematics/PLDPSolverHerdt.hh>
#include <PreviewControl/SupportFSM.hh>
#include <FootTrajectoryGeneration/FootTrajectoryGenerationStandard.hh>
#include <ZMPRefTrajectoryGeneration/OrientationsPreview.hh>
#include <ZMPRefTrajectoryGeneration/problem-vel-ref.hh>

namespace PatternGeneratorJRL
{


  class ZMPDiscretization;
  class  ZMPVelocityReferencedQP : public ZMPRefTrajectoryGeneration
  {

  public:

    /* Default constructor. */
    ZMPVelocityReferencedQP(SimplePluginManager *lSPM, string DataFile,
			    CjrlHumanoidDynamicRobot *aHS=0);

    /* Default destructor. */
    ~ZMPVelocityReferencedQP();


    /*! \name Methods to build the optimization problem
      @{
    */

    /*! \brief Compute the constant matrices over all the instances of the problem.
      This means \f$P_{pu}, P_{px}, P_{vs}, P_{vu}\f$.
      The necessary parameters to build those matrices are extracted from the
      PreviewControl link.
    */
    int InitializeMatrixPbConstants();

    /*! \brief This method does the same once the previous method has been called
      to compute the static part of the optimization function.
      Assuming that the optimization function is of the form
      \f$ min_{u_k} \frac{1}{2} u^{\top}_k Q u_k + p^{\top}_k u_k \f$
      this method computes \f$Q\f$, the constant part of $p^{\top}_k$.

    */
    int BuildingConstantPartOfTheObjectiveFunction();

    /*! \brief Call the two previous methods
      \return A negative value in case of a problem 0 otherwise.
    */
    int InitConstants();

    void initFeet();


    int buildConstraintMatrices(double * &DS, double * &DU,
				double StartingTime,
				deque<LinearConstraintInequalityFreeFeet_t>    & QueueOfLConstraintInequalitiesFreeFeet,
				deque<LinearConstraintInequalityFreeFeet_t>    & QueueOfFeetPosInequalities,
				deque<SupportFeet_t>    & QueueOfSupportFeet,
				double Com_Height,
				int NbOfConstraints,
				MAL_VECTOR(&xk,double));



    /*! \brief Build the constant part of the constraint matrices. */
    int BuildingConstantPartOfConstraintMatrices();

    int buildConstraintMatricesPLDPHerdt();

    /*! This method helps to build a linear system for constraining the ZMP. */
    int ComputeLinearSystem(vector<CH_Point> aVecOfPoints,
			    MAL_MATRIX(&A,double),
			    MAL_MATRIX(&B,double));

    /*! @} */


    /*! Call method to handle the plugins (SimplePlugin interface) . */
    void CallMethod(std::string &Method, std::istringstream &strm);

    /*! \name Call method to handle on-line generation of ZMP reference trajectory.
      @{*/

    /*! Methods for on-line generation. (First version!)
      The queues will be updated as follows:
      - The first values necessary to start walking will be inserted.
      - The initial positions of the feet will be taken into account
      according to InitLeftFootAbsolutePosition and InitRightFootAbsolutePosition.
      - The RelativeFootPositions stack will NOT be taken into account,
      - The starting COM Position will NOT be taken into account.
      Returns the number of steps which has been completely put inside
      the queue of ZMP, and foot positions.
    */
    int InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
		   deque<COMState> & CoMStates,
		   deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
		   deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
		   FootAbsolutePosition & InitLeftFootAbsolutePosition,
		   FootAbsolutePosition & InitRightFootAbsolutePosition,
		   deque<RelativeFootPosition> &RelativeFootPositions,
		   COMState & lStartingCOMState,
		   MAL_S3_VECTOR_TYPE(double) & lStartingZMPPosition);


    /* ! \brief Method to update the stacks on-line */
    void OnLine(double time,
		deque<ZMPPosition> & FinalZMPPositions,
		deque<COMState> & CoMStates,
		deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
		deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions);


    int validateConstraints(double * & DS,double * &DU,
			    int NbOfConstraints,  int li,
			    double *X, double time);


    /*! \name Setter and getter for the objective function parameters
      @{
    */

    /*! Set the velocity reference */
    void setVelReference(istringstream &strm);

    /*! Set the velocity reference from external reference */
    void setVelReference(double x,double y, double yaw);

    /*! Set the velocity reference from external reference */
    void setCoMPerturbationForce(double x,double y);

    void setCoMPerturbationForce(istringstream &strm);

    void interpolateFeet(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			 deque<FootAbsolutePosition> &RightFootAbsolutePositions);

    /*! Return \f$\alpha\f$ */
    const double & GetAlpha() const;

    /*! Set \f$\alpha\f$ */
    void SetAlpha(const double &);

    /*! Return \f$\beta\f$ */
    const double & GetBeta() const;

    /*! Set \f$\beta\f$ */
    void SetBeta(const double &);

    /*! @}*/
    /* @} */
    ReferenceAbsoluteVelocity RefVel;

   
    static const unsigned int QLD=0;
    static const unsigned int QLDANDLQ=1;
    static const unsigned int PLDP=2;
    static const unsigned int PLDPHerdt = 3;

  private:

    double m_RobotMass;
    bool m_PerturbationOccured;
    double m_FeetDistanceDS;
    
    bool m_EndingPhase;
    double m_TimeToStopOnLineMode;

    double m_FPx, m_FPy, m_FPtheta;
    double m_StartTime;

    double m_UpperTimeLimitToUpdate;

    deque<SupportFeet_t> QueueOfSupportFeet;

    double m_TimeBuffer;

    /*! Uses a 2D LIPM to simulate the evolution of the robot. */
    LinearizedInvertedPendulum2D * m_2DLIPM;

    /*! Uses a Finite State Machine to simulate the evolution of the support states. */
    SupportFSM * m_SupportFSM;

    /*! Deecoupled optimization problem to compute the evolution of feet angles. */
    OrientationsPreview * m_OP;

    /*! \brief Object creating Linear inequalities constraints
      based on the foot position. Those constraints are *NOT* the
      one put in the QP, but they are a necessary intermediate step. */
    FootConstraintsAsLinearSystemForVelRef * m_fCALS;

    /*! \brief Standard polynomial trajectories for the feet. */
    FootTrajectoryGenerationStandard * m_FTGS;

    /*! Constraint on X and Y */
    double m_ConstraintOnX, m_ConstraintOnY;

    /*! Com height */
    double m_ComHeight;

    /*! Current state of the trunk and the trunk state after m_QP_T*/
    COMState m_TrunkState, m_TrunkStateT;

    deque<COMState> m_QueueOfTrunkStates;

    double m_a, m_TrunkPolCoeffB, m_c, m_d, m_TrunkPolCoeffE;

    //Additional term on the acceleration of the CoM
    MAL_VECTOR(m_PerturbationAcceleration,double);

    /*! Sampling of the QP. */
    double m_QP_T;

    /*! Preview window */
    int m_QP_N;

    /*! Orientations of the previewed support feet */
    deque<double> m_PreviewedSupportAngles;

    //Final optimization problem
    Problem m_Pb;

    // Landing foot position.
    double m_LandingXPosition, m_LandingYPosition;

    SupportState_t m_CurrentSupport, m_PrwSupport;

    /*! \name Variables related to the QP
      @{ */
    /*! \brief Matrix relating the command and the CoM position. */
    MAL_MATRIX(m_PPu,double);

    /*! \brief Matrix relating the command and the ZMP position. */
    MAL_MATRIX(m_PZu,double);

    /*! \brief Matrix relating the command and the CoM speed. */
    MAL_MATRIX(m_VPu,double);

    /*! \brief Matrix relating the CoM state and the CoM position. */
    MAL_MATRIX(m_PPx,double);

    /*! \brief Matrix relating the CoM state and the ZMP position. */
    MAL_MATRIX(m_PZx,double);

    /*! \brief Matrix relating the CoM state and the CoM speed. */
    MAL_MATRIX(m_VPx,double);

    /*! \brief Selection matrix for the previewed feet positions. */
    MAL_MATRIX(m_U,double);

    /*! \brief Selection matrix for the support feet. */
    MAL_VECTOR(m_Uc,double);

    /*! \brief Matrix of the objective function $Q$ */
    //    double *m_Q;

    /*! \brief Cholesky decomposition of the initial objective function $Q$ */
    MAL_MATRIX(m_LQ,double);

    /*! \brief Cholesky decomposition of the initial objective function $Q$ */
    MAL_MATRIX(m_iLQ,double);

    /*! \brief Optimized cholesky decomposition */
    OptCholesky * m_OptCholesky;

    /*! \brief Sub matrix to compute the linear part of the objective function $p^{\top}_k$. */
    MAL_MATRIX(m_OptA,double);
    MAL_MATRIX(m_OptB,double);
    MAL_MATRIX(m_OptC,double);
    MAL_MATRIX(m_OptD,double);

    enum MatrixType  { M_PPU, M_PZU, M_VPU, M_PPX,
			M_PZX, M_VPX, M_U, 
			M_LQ, M_ILQ};
    enum VectorType  {m_UC,M_OPTA, M_OPTB,	M_OPTC, M_OPTD };
      
    /*! \name Parameters of the objective function
      @{ */
    /*! Putting weight on the velocity */
    double m_Beta;

    /*! Putting weight on the jerk minimization. */
    double m_Alpha;
    /*! @} */

    /*! Putting weight on the ZMP */
    double m_Gamma;

    /* Constant parts of the linear constraints. */
    double * m_Pu;

    /* Constant parts of the linear constraints. */
    MAL_MATRIX(m_iPu,double);

    /* Constant parts of the dynamical system. */
    MAL_MATRIX(m_Px,double);

    /*! \brief Debugging variable: dump everything is set to 1 */
    int m_FullDebug;

    /*! \brief Fast formulations mode. */
    unsigned int m_FastFormulationMode;

    //! Primal Least square Distance Problem solver *\/ */
    Optimization::Solver::PLDPSolverHerdt * m_PLDPSolverHerdt;

    void initializeProblem();

    void computeCholeskyOfQ(double * OptA);

    void computeObjective(deque<LinearConstraintInequalityFreeFeet_t> & QueueOfLConstraintInequalitiesFreeFeet,
			  deque<SupportFeet_t> & QueueOfSupportFeet,
			  int NbOfConstraints, int NbOfEqConstraints,
			  int & CriteriaToMaximize, MAL_VECTOR(& xk,double), double time);


    void interpolateTrunkState(double time, int CurrentIndex,
			       deque<COMState> & FinalCOMStates);

    void interpolateFeetPositions(double time, int CurrentIndex,
				  deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
				  deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions);

    /*! \name Debugging related methods 
     @{*/
    /*! \brief Dump the instance of the quadratic problem for one iteration. */
    int dumpProblem(MAL_VECTOR(& xk,double),
		    double Time);

    /*! \brief Debugging point for the constructor. */
    void debugConstructor();

    /*! \brief Dumping matrix specified by MatrixID in aos. */
    void debugMatrix(ostream & aos, enum MatrixType MatrixID);

    /*! \brief Dumping matrix specified by MatrixID in file named filename. */
    void debugMatrix(const char *filename, enum MatrixType MatrixID);
    /*! @} */
    
  public:

    /*! Methods to comply with the initial interface of ZMPRefTrajectoryGeneration.
      TODO: Change the internal structure to make those methods not mandatory
      for compiling.
    */

    void GetZMPDiscretization(std::deque<ZMPPosition> & ZMPPositions,
			      std::deque<COMState> & COMStates,
			      std::deque<RelativeFootPosition> &RelativeFootPositions,
			      std::deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			      std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
			      double Xmax,
			      COMState & lStartingCOMState,
			      MAL_S3_VECTOR_TYPE(double) & lStartingZMPPosition,
			      FootAbsolutePosition & InitLeftFootAbsolutePosition,
			      FootAbsolutePosition & InitRightFootAbsolutePosition);

    void OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
		       std::deque<ZMPPosition> & FinalZMPPositions,
		       std::deque<COMState> & COMStates,
		       std::deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
		       std::deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
		       bool EndSequence);

    int OnLineFootChange(double time,
			 FootAbsolutePosition &aFootAbsolutePosition,
			 deque<ZMPPosition> & FinalZMPPositions,
			 deque<COMState> & CoMPositions,
			 deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
			 deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
			 StepStackHandler  *aStepStackHandler);

    void EndPhaseOfTheWalking(deque<ZMPPosition> &ZMPPositions,
			      deque<COMState> &FinalCOMStates,
			      deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			      deque<FootAbsolutePosition> &RightFootAbsolutePositions);

    int ReturnOptimalTimeToRegenerateAStep();

    /* Get foot position  */
    void getLandingFootPosition(double &x, double &y, double &yaw)
    { x = m_FPx; y = m_FPy; yaw = m_PreviewedSupportAngles[0];}
    

  };
}

#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.hh>
#endif /* _ZMPQP_WITH_CONSTRAINT_H_ */

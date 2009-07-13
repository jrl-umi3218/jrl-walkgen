/* This object provides the generation of ZMP and CoM trajectory
   using a new formulation of the stability problem.

   Copyright (c) 2005-2009, 
   Olivier Stasse,Francois Keith
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see Licence.txt for details on the license.
*/

#ifndef _ZMPCONSTRAINEDQPFASTFORMULATION_WITH_CONSTRAINT_H_
#define _ZMPCONSTRAINEDQPFASTFORMULATION_WITH_CONSTRAINT_H_


#include <walkGenJrl_API.h>
#include <PreviewControl/LinearizedInvertedPendulum2D.h>
#include <Mathematics/FootConstraintsAsLinearSystem.h>
#include <Mathematics/OptCholesky.h>
#include <Mathematics/PLDPSolver.h>
#include <ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.h>

namespace PatternGeneratorJRL
{
  class ZMPDiscretization;
  class WALK_GEN_JRL_EXPORT ZMPConstrainedQPFastFormulation : public ZMPRefTrajectoryGeneration
  {
    
  public:

    /* Default constructor. */
    ZMPConstrainedQPFastFormulation(SimplePluginManager *lSPM, string DataFile, 
				    CjrlHumanoidDynamicRobot *aHS=0);

    /* Default destructor. */
    ~ZMPConstrainedQPFastFormulation();
    

      /** Generate ZMP discreatization from a vector of foot position.
	  ASSUME A COMPLETE MOTION FROM END TO START, and GENERATE EVERY VALUE.
	  
	  @param[out] ZMPPositions: Returns the ZMP reference values for the overall motion.
	  Those are absolute position in the world reference frame. The origin is the initial
	  position of the robot. The relative foot position specified are added.

	  @param[out] CoMPositions: Returns the CoM reference values for the overall motion.
	  Those are absolute position in the world reference frame. The origin is the initial
	  position of the robot. The relative foot position specified are added.

	  @param[in] RelativeFootPositions: The set of 
	  relative steps to be performed by the robot.

	  @param[out] LeftFootAbsolutePositions: Returns the absolute position of the left foot.
	  According to the macro FULL_POLYNOME the trajectory will follow a third order
	  polynom or a fifth order. By experience it is wise to put a third order. 
	  A null acceleration might cause problem for the compensation of the Z-axis momentum.

	  @param[out] RightFootAbsolutePositions: Returns the absolute position of the right foot.
	  
	  @param[in] Xmax: The maximal distance of a hand along the X axis in the waist coordinates.

	  @param[in] lStartingCOMPosition: The initial position of the CoM.
	  
	  @param[in] lStartingZMPPosition: The initial position of the ZMP.
	  
	  @param[in] InitLeftFootAbsolutePosition: The initial position of the left foot.
	  
	  @param[in] InitRightFootAbsolutePosition: The initial position of the right foot.

	  
	   */
    void GetZMPDiscretization(deque<ZMPPosition> & ZMPPositions,
			      deque<COMPosition> & CoMPositions,
			      deque<RelativeFootPosition> &RelativeFootPositions,
			      deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			      deque<FootAbsolutePosition> &RightFootAbsolutePositions,
			      double Xmax,
			      COMPosition & lStartingCOMPosition,
			      MAL_S3_VECTOR(,double) & lStartingZMPPosition,
			      FootAbsolutePosition & InitLeftFootAbsolutePosition,
			      FootAbsolutePosition & InitRightFootAbsolutePosition);

    /*! This method is a new way of computing the ZMP trajectory from
      foot trajectory. */
    int BuildZMPTrajectoryFromFootTrajectory(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					     deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					     deque<ZMPPosition> &ZMPRefPositions,		       
					     deque<COMPosition> &COMPositions,
					     double ConstraintOnX,
					     double ConstraintOnY,
					     double T,
					     unsigned int N);

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

    /*! Transform the matrices with LQ. */
    int BuildingConstantPartOfTheObjectiveFunctionQLDANDLQ(MAL_MATRIX(,double) &OptA);

    /*! Compute the quadratic form of the objective function. */
    int BuildingConstantPartOfTheObjectiveFunctionQLD(MAL_MATRIX(,double) &OptA);


    /*! \brief Call the two previous methods 
      \return A negative value in case of a problem 0 otherwise.
     */
    int InitConstants();
      
    
    /*! Build the necessary matrices for the QP problem under linear inequality constraints. */
    int BuildConstraintMatrices(double * &Px, double * &DPu,
				unsigned N, double T,
				double StartingTime,
				deque<LinearConstraintInequality_t *> 
				& QueueOfLConstraintInequalities,
				double Com_Height,
				unsigned int &NbOfConstraints,
				MAL_VECTOR(&xk,double),
				MAL_VECTOR(&ZMPRef,double),
				unsigned int &NextNumberOfRemovedConstraints);

    /*! \brief Build the constant part of the constraint matrices. */
    int BuildingConstantPartOfConstraintMatrices();

    /*! This method helps to build a linear system for constraining the ZMP. */
    int ComputeLinearSystem(vector<CH_Point> aVecOfPoints,
			    MAL_MATRIX(&A,double),
			    MAL_MATRIX(&B,double));

    /*! @} */


    /*! Call method to handle the plugins (SimplePlugin interface) . */
    void CallMethod(std::string &Method, std::istringstream &strm);

    /*! \name Call method to handle on-line generation of ZMP reference trajectory. 
      @{*/
        
    /*! Methods for on-line generation. (First version)
      The queues will be updated as follows:
      - The first values necessary to start walking will be inserted.
      - The initial positions of the feet will be taken into account
      according to InitLeftFootAbsolutePosition and InitRightFootAbsolutePosition.
      - The RelativeFootPositions stack will be taken into account,
      - The starting COM Position.
      Returns the number of steps which has been completely put inside 
      the queue of ZMP, and foot positions.
    */
    int InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
		   deque<COMPosition> & CoMPositions,		   
		   deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
		   deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
		   FootAbsolutePosition & InitLeftFootAbsolutePosition,
		   FootAbsolutePosition & InitRightFootAbsolutePosition,
		   deque<RelativeFootPosition> &RelativeFootPositions,
		   COMPosition & lStartingCOMPosition,
		   MAL_S3_VECTOR(,double) & lStartingZMPPosition);
    
    /* ! Methods to update the stack on-line by inserting a new foot position. */
    void OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
		       deque<ZMPPosition> & FinalZMPPositions,		
		       deque<COMPosition> & CoMPositions,			     
		       deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
		       deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
		       bool EndSequence);

    /* ! \brief Method to update the stacks on-line */
    void OnLine(double time,
		deque<ZMPPosition> & FinalZMPPositions,		
		deque<COMPosition> & CoMPositions,			     
		deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
		deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions);

    /* ! \brief Method to change on line the landing position of a foot.
       @return If the method failed it returns -1, 0 otherwise.
    */
    int OnLineFootChange(double time,
			 FootAbsolutePosition &aFootAbsolutePosition,
			 deque<ZMPPosition> & FinalZMPPositions,			     
			 deque<COMPosition> & CoMPositions,
			 deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
			 deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
			 StepStackHandler * aStepStackHandler=0);

    /*! \brief Method to stop walking.
      @param[out] ZMPPositions: The queue of ZMP reference positions.
      @param[out] FinalCOMPositions: The queue of COM reference positions.
      @param[out] LeftFootAbsolutePositions: The queue of left foot absolute positions.
      @param[out] RightFootAbsolutePositions: The queue of right foot absolute positions.
    */
    void EndPhaseOfTheWalking(deque<ZMPPosition> &ZMPPositions,
			      deque<COMPosition> &FinalCOMPositions,
			      deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			      deque<FootAbsolutePosition> &RightFootAbsolutePositions);

    
    int ValidationConstraints(double * & DPx,double * &DPu,
			      int NbOfConstraints,
			      deque<LinearConstraintInequality_t *> &
			      QueueOfLConstraintInequalities,
			      unsigned int li,
			      double *X,
			      double StartingTime);
    /*! \brief Return the time at which it is optimal to regenerate a step in online mode. 
     */
    int ReturnOptimalTimeToRegenerateAStep();

    /*! \name Setter and getter for the objective function parameters
      @{
    */

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
    
    /*! Set the preview control object. */
    void SetPreviewControl(PreviewControl *aPC);

    static const unsigned int QLD=0;
    static const unsigned int QLDANDLQ=1;
    static const unsigned int PLDP=2;

  private:

    /*! Uses a ZMPDiscretization scheme to get the usual Kajita heuristic. */
    ZMPDiscretization * m_ZMPD;

    /*! Uses a 2D LIPM to simulate the evolution of the robot. */
    LinearizedInvertedPendulum2D * m_2DLIPM;

    /*! \brief Object creating Linear inequalities constraints 
      based on the foot position. Those constraints are *NOT* the
      one put in the QP, but they are a necessary intermediate step. */
    FootConstraintsAsLinearSystem * m_FCALS;
      

    /*! Constraint on X and Y */
    double m_ConstraintOnX, m_ConstraintOnY;
    
    /*! Com height */
    double m_ComHeight;

    /*! Sampling of the QP. */
    double m_QP_T;
    
    /*! Preview window */
    unsigned int m_QP_N;

    /*! \name Variables related to the QP
      @{ */
    /*! \brief Matrix relating the command and the CoM position. */
    MAL_MATRIX(m_PPu,double);

    /*! \brief Matrix relating the command and the CoM speed. */
    MAL_MATRIX(m_VPu,double); 
 
    /*! \brief Matrix relating the CoM state and the CoM position. */
    MAL_MATRIX(m_PPx,double);

    /*! \brief Matrix relating the CoM state and the CoM speed. */
    MAL_MATRIX(m_VPx,double);

    /*! \brief Matrix of the objective function $Q$ */
    double *m_Q;

    /*! \brief Cholesky decomposition of the initial objective function $Q$ */
    MAL_MATRIX(m_LQ,double);

    /*! \brief Cholesky decomposition of the initial objective function $Q$ */
    MAL_MATRIX(m_iLQ,double);

    /*! \brief Optimized cholesky decomposition */
    OptCholesky * m_OptCholesky;

    /*! \brief Sub matrix to compute the linear part of the objective function $p^{\top}_k$. */
    MAL_MATRIX(m_OptB,double);
    MAL_MATRIX(m_OptC,double);

    /*! \name Parameters of the objective function 
    @{ */
    /*! Putting weight on the ZMP ref trajectory */
    double m_Beta;

    /*! Putting weight on the jerk minimization. */
    double m_Alpha;
    /*! @} */

    /* Constant parts of the linear constraints. */
    double * m_Pu;

    /* Constant parts of the linear constraints. */
    MAL_MATRIX(m_iPu,double);

    /* Constant parts of the dynamical system. */
    MAL_MATRIX(m_Px,double);

    /*! \brief Debugging variable: dump everything is set to 1 */
    unsigned int m_FullDebug;

    /*! \brief Fast formulations mode. */
    unsigned int m_FastFormulationMode;
    
    /*! Primal Least square Distance Problem solver */
    Optimization::Solver::PLDPSolver * m_PLDPSolver;

    /*! @} */
    
    int DumpProblem(double * Q,
		    double * D, 
		    double * Pu,
		    unsigned int NbOfConstraints,
		    double * Px,
		    double * XL,
		    double * XU,
		    double Time);
      
    /*! Vector of similar constraints. */
    vector<int> m_SimilarConstraints;
  };
};

#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.h>
#endif /* _ZMPQP_WITH_CONSTRAINT_H_ */

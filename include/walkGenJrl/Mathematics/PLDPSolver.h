/*! \file PLDPSolver.cpp
  \brief This file implements the optimized QP solver proposed by Dimitar 2009.
    On the Application of Linear Model Predictive Control 
    for Walking Pattern Generation in the Presence of Strong Disturbances
    D. Dimitrov and P.-B. Wieber and H. Diedam and O. Stasse

   Copyright (c) 2009, 
   Olivier Stasse,
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _PLDP_SOLVER_H_

#include <vector>
#include <walkGenJrl/Mathematics/OptCholesky.h>

namespace Optimization
{
  namespace Solver
    {
      /*! This class implements a two stage strategy to solve the following optimal problem:
       */
      class PLDPSolver
	{
	public:
	  /*! \brief Constructor */
	  PLDPSolver(unsigned int CardU,
		     double * iPu,
		     double * Px,
		     double * Pu,
		     double * iLQ);

	  /*! \brief Destructor */
	  ~PLDPSolver();

	  /*! \brief Solve the optimization problem 
	   */
	  int SolveProblem(double *CstPartOfTheCostFunction,
			   unsigned int NbOfConstraints,
			   double *LinearPartOfConstraints,
			   double *CstPartOfConstraints,
			   double *ZMPRef,
			   double *XkYk,
			   double *X,
			   std::vector<int> &SimilarConstraint,
			   unsigned int NumberOfRemovedConstraints,
			   bool StartingSequence);
	protected:
	  
	  /*! \name Initial solution methods related 
	    @{
	   */
	  /*! Compute the initial solution */
	  int ComputeInitialSolution(double *ZMPRef,
				     double *XkYk,
				     bool StartingSequence);

	  /*! Precompite iPuPx */
	  int PrecomputeiPuPx(); 
	  /*! @} */


	  /*! Initialize the internal variables of
	    the class. */
	  void InitializeSolver();

	  /*! Allocate memory for solver. */
	  void AllocateMemoryForSolver();

	  /*! \name Projected descent direction methods related 
	    @{
	   */
	  /*! \brief Compute Projected descent direction. */
	  int ComputeProjectedDescentDirection();

	  /*! \brief Forward substitution. 
	      First Phase
	      EE^t v2 = v1 <-> LL^t v2 = v1
	      Now solving L y = v1

	   */
	  int ForwardSubstitution();

	  /*! \brief Compute v2 q (14b) in Dimitrov 2009.
	  Second phase a
	   Now solving
	   LL^t v2 = v1 <-> L y = v1 with L^t v2 = y
	   y solved with first phase.
	   So now we are looking for v2.
	  */
	  int BackwardSubstitution();

	  /*! @} */

	  /*! Detecting violated constraints */
	  double ComputeAlpha(vector<unsigned int> &NewActivatedConstraints,
			      vector<int> &SimilarConstraint);

	  /*! Store the current ZMP solution for hot start purposes. */
	  void StoreCurrentZMPSolution(double *XkYk);

	  /*! Write current ZMP ref trajectory associated with 
	    current value of m_Vk. */
	  void WriteCurrentZMPSolution(string filename,
				       double *XkYk);

	  /*! \name Methods related to a limited amount of computational time 
	    @{ */
	    
	private:
	  
	  /*! \brief Store Pu */
	  double *m_Pu;

	  /*! Store the inverse of Pu. */
	  double * m_iPu;

	  /*! Store Px. */
	  double * m_Px;
	  
	  /*! Store the inverse of Pu. */
	  double * m_iPuPx;
	  
	  /*! Store Vk */
	  double *m_Vk;

	  /*! Store the constart part of the cost function. */
	  double *m_CstPartOfCostFunction;

	  /*! Store the unconstrained descent direction. */
	  double *m_UnconstrainedDescentDirection;
	  
	  /*! Store the cholesky decomposition of EE^t. */
	  double * m_L;
	  
	  /*! Store the inverse of the cholesky decomposition. */
	  double *m_iL;

	  /*! Store iLQ for debuggin purposes. */
	  double *m_iLQ;

	  /*! Store the projector of descent  */
	  double *m_d;

	  /*! Store some temporary variables  */
	  double *m_v1, *m_v2, *m_y;

	  /*! Store some temporary variables  */
	  double *m_tmp1, *m_tmp2;
	  
	  /*! Store the linear part of the constraints. */
	  double * m_A;

	  /*! Store the cst part of the constraints. */
	  double * m_b;


	  /*! Store if the A*Vk values has been computed  */
	  bool * m_ConstraintsValueComputed;


	  /*! Store the maximum number of Constraints.
	    It is also the dimension of L in its maximal storage
	    form. */
	  unsigned int m_NbMaxOfConstraints;

	  /*! Store the current number of Constraints of matrix A. */
	  unsigned int m_NbOfConstraints;

	  /*! Store the size of the control vector. */
	  unsigned int m_CardV;

	  
	  /*! \name Debugging fields 
	    @{
	   */
	  /*! Level of verbosity */
	  unsigned int m_DebugMode;

	  /*! Number of iterations */
	  int m_ItNb;
	  /*! @} */

	  /*! Cholesky decomposition optimized for QP solving 
	   ( specifically this one). */
	  PatternGeneratorJRL::OptCholesky * m_OptCholesky;

	  /*! List of activated constraints. */
	  vector<unsigned int> m_ActivatedConstraints;

	  /*! List of previously activated constraints. */
	  vector<unsigned int> m_PreviouslyActivatedConstraints;

	  /*! Boolean to perform a hotstart */
	  bool m_HotStart;

	  /*! Store the current ZMP solution. */
	  double * m_PreviousZMPSolution;

	  /*! Double internal time. */
	  double m_InternalTime;

	  /*! Tolerance for zero value */
	  double m_tol;

	  /*! \name Data related to a limited amount of computational time 
	    @{
	   */
	  /*! Is the algorithm limited in time. */
	  bool m_LimitedComputationTime;

	  /*! Amount of limited */
	  double m_AmountOfLimitedComputationTime;
	  
	  /*! @} */
	};
    };
};
#endif /* _PLDPSOLVER_H_*/

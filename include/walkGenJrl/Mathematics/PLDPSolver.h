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
		     double * Px);

	  /*! \brief Destructor */
	  ~PLDPSolver();

	  /*! \brief Solve the optimization problem */
	  int SolveProblem(double *CstPartOfTheCostFunction,
			    unsigned int NbOfConstraints,
			    double *LinearPartOfConstraints,
			    double *CstPartOfConstraints,
			    double *ZMPRef,
			    double *XkYk);
	protected:
	  
	  /*! Compute the initial solution */
	  void ComputeInitialSolution(double *ZMPRef,
				      double *XkYk);

	  /*! Initialize the internal variables of
	    the class. */
	  void InitializeSolver();

	  /*! Allocate memory for solver. */
	  void AllocateMemoryForSolver();

	private:
	  
	  /*! Store the inverse of Pu. */
	  double * m_iPu;

	  /*! Store Px. */
	  double * m_Px;
	  

	  /*! Store the inverse of Pu. */
	  double * m_iPuPx;

	  
	  /*! Store Uk */
	  double *m_Uk;

	  /*! Store the constart part of the cost function. */
	  double *m_CstPartOfCostFunction;

	  /*! Store the unconstrained descent direction. */
	  double *m_UnconstrainedDescentDirection;
	  
	  /*! Store the cholesky decomposition of EE^t. */
	  double * m_L;
	  
	  /*! Store the inverse of the cholesky decomposition. */
	  double *m_iL;

	  /*! Store the projector of descent  */
	  double *m_d;

	  /*! Store some temporary variables  */
	  double *m_v1;
	  
	  /*! Store the linear part of the constraints. */
	  double * m_A;

	  /*! Cholesky decomposition optimized for QP solving 
	   ( specifically this one). */
	  PatternGeneratorJRL::OptCholesky * m_OptCholesky;

	  /*! List of activated constraints. */
	  vector<unsigned int> m_ActivatedConstraints;

	};
    };
};
#endif _PLDPSOLVER_H_

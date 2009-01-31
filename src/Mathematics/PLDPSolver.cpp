/*! \file PLDPSolver.cpp
  \brief This file implements the optimized QP solver proposed by Dimitar 2009.

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
#include <walkGenJrl/Mathematics/PLDPSolver.h>

using namespace PatternGeneratorJRL;
using namespace Optimization::Solver;

PLDPSolver::PLDPSolver(unsigned int CardU,
		       double *iPu,
		       double *Px)
{
  m_CardU = CardU;
  m_iPu = iPu;
  m_Px = Px;
  m_iPuPx = 0;
}

PDLPSolver::InitializeSolver()
{
  PrecomputeiPuPx();
  m_Uk = new double[2*m_CardU];
  memset(m_Uk,0,2*m_CardU*sizeof(double));

  m_UnconstrainedDescentDirection = new double[2*m_CardU];
  memset(m_UnconstrainedDescentDirection,0,2*m_CardU*sizeof(double));
}

PLDPSolver::~PLDPSolver()
{
  if (m_iPuPx!=0)
    delete [] m_iPuPx;

  if (m_Uk!=0)
    delete [] m_Uk;

  if (m_UnconstrainedDescentDirection!=0)
    delete [] m_UnconstrainedDescentDirection;
}

PLDLPSolver::PrecomputeiPuPx()
{
  m_iPuPx = new double[2*m_CardU*6];

  // This should be reimplemented with a proper matrix library.
  // will be done in a second time.
  for(unsigned int i=0;i<2*m_CardU;i++)
    {
      for(unsigned int j=0;j<6;j++)
	{
	  m_iPuPx[i*2*m_CardU+j]= 0.0;
	  for(unsigned int k=0;k<2*N;k+++)
	    {
	      m_iPuPx[i*2*m_CardU+j]+= 
		m_iPu[i*2*m_CardU+k] * 
		m_iPu[k*2*m_CardU+j];
	    }
	}
    }
}

PLDPSolver::ComputeInitialSolution(double *ZMPRef,
				   double *XkYk)
{
  /*! The initial solution of the problem is given by
    eq(14) Dimitar ICRA 2008
    U0 = iPu * Px [Xkt Ykt]t + iPu * ZMPRef 
    The only part which can not be precomputed is ZMPRef.
   */
  for(unsigned int i=0;i<2*m_CardU;i++)
    {
      m_Uk[i] = 0.0;
      for(unsigned int j=0;j<6;j++)
	m_Uk[i]+= m_iPuPx[i*2*m_CardU+j] *  XkYk[j];

      for(unsigned int j=0;j<2*m_CardU;j++)
	m_Uk[i]+= m_iPu[i*2*m_CardU+j] * ZMPRef[j];  
      
    }
}

int PLDPSolver::SolveProblem(double *CstPartOfTheCostFunction,
			      unsigned int NbOfConstraints,
			      double *LinearPartOfConstraints,
			      double *CstPartOfConstraints,
			      double *ZMPRef,
			      double *XkYk)
{
  /* Step one : Algorithm initialization. */
  m_CstPartOfCostFunction = CstPartOfTheCostFunction;
  ComputeInitialSolution(ZMPRef,XkYk);

  /* Step two : Compute descent direction. */
  for(unsigned int i=0;i<2*m_CardU;i++)
    m_UnconstrainedDescentDirection[i] = 
      m_CstPartOftheCostFunction[i] -  m_Uk[i];
  
  /*! Step three: Compute the projected descent direction. */
}

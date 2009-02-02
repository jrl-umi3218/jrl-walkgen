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
  m_NbMaxOfConstraints = 8*m_CardU;
  m_OptCholesky = new PatternGeneratorJRL::OptCholesky(m_NbMaxOfConstraints,2*m_CardU);

  AllocateMemoryForSolver();
}

void PLDPSolver::AllocateMemoryForSolver()
{
  PrecomputeiPuPx();
  m_Uk = new double[2*m_CardU];
  memset(m_Uk,0,2*m_CardU*sizeof(double));

  m_UnconstrainedDescentDirection = new double[2*m_CardU];
  memset(m_UnconstrainedDescentDirection,0,2*m_CardU*sizeof(double));

  m_L = new double[m_NbMaxOfConstraints*m_NbMaxOfConstraints];
  m_iL = new double[m_NbMaxOfConstraints*m_NbMaxOfConstraints];

  m_v1 = new double[m_NbMaxOfConstraints];
  m_v2 = new double[m_NbMaxOfConstraints];
  m_y = new double[m_NbMaxOfConstraints];


  m_d = new double[2*m_CardU];  
  m_OptCholesky->SetL(m_L);
  m_OptCholesky->SetL(m_iL);
}

void PLDPSolver::InitializeSolver()
{
  // Allocation max:
  // We assume that we might at max. 8 actives constraintes 
  // per control time.
  memset(m_v1,0,8*m_CardU*sizeof(double));
  // Same than v1.
  memset(m_v2,0,8*m_CardU*sizeof(double));
  // Same than y.
  memset(m_y,0,8*m_CardU*sizeof(double));

  // Same than L.
  memset(m_L,0,8*m_CardU*sizeof(double));
  // Same than iL.
  memset(m_iL,0,8*m_CardU*sizeof(double));

  // Same for the descent.
  memset(m_d,0,2*m_CardU*sizeof(double));
}

PLDPSolver::~PLDPSolver()
{
  if (m_iPuPx!=0)
    delete [] m_iPuPx;

  if (m_Uk!=0)
    delete [] m_Uk;

  if (m_UnconstrainedDescentDirection!=0)
    delete [] m_UnconstrainedDescentDirection;

  if (m_OptCholesky!=0)
    delete m_OptCholesky;

  if (m_L!=0)
    delete [] m_L;

  if (m_iL!=0)
    delete [] m_iL;

  if (m_v1!=0)
    delete [] m_v1;

  if (m_v2!=0)
    delete [] m_v2;

  if (m_d!=0)
    delete [] m_d;

}

int PLDPSolver::PrecomputeiPuPx()
{
  m_iPuPx = new double[2*m_CardU*6];

  // This should be reimplemented with a proper matrix library.
  // will be done in a second time.
  for(unsigned int i=0;i<2*m_CardU;i++)
    {
      for(unsigned int j=0;j<6;j++)
	{
	  m_iPuPx[i*2*m_CardU+j]= 0.0;
	  for(unsigned int k=0;k<2*m_CardU;k++)
	    {
	      m_iPuPx[i*2*m_CardU+j]+= 
		m_iPu[i*2*m_CardU+k] * 
		m_iPu[k*2*m_CardU+j];
	    }
	}
    }
  return 0;
}

int PLDPSolver::ComputeInitialSolution(double *ZMPRef,
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
  return 0;
}

int PLDPSolver::ForwardSubstitution()
{
  // Compute v2 q (14b) in Dimitrov 2009.
  // First Phase
  // EE^t v2 = v1 <-> LL^t v2 = v1
  // Now solving
  // L y = v1 
  for(unsigned int i=0;i<m_ActivatedConstraints.size();i++)
    {
      m_y[i] = 0.0;
      for(unsigned int k;k<2*m_CardU;k++)
	m_y[i] += (m_v1[i] - m_L[i*m_NbMaxOfConstraints+k]*m_v1[k])/m_L[k*m_NbMaxOfConstraints+k];
    }
  return 0;
}

int PLDPSolver::BackwardSubstitution()
{
  // Compute v2 q (14b) in Dimitrov 2009.
  // Second phase 
  // Now solving
  // LL^t v2 = v1 <-> L y = v1 with L^t v2 = y
  // y solved with first phase.
  // So now we are looking for v2.
  unsigned SizeOfL = m_ActivatedConstraints.size();
  for(unsigned int i=SizeOfL-1;
      i>=0; i--)
    {
      m_v2[i] = 0.0;
      for(unsigned int k=i+1;k<SizeOfL;k++)
	m_v2[i] += (m_y[i] - m_L[i*m_NbMaxOfConstraints+k]*m_y[k]);
      m_v2[i] = m_v2[i]/m_L[i*m_NbMaxOfConstraints+i];
    }
  return 0;
}

int PLDPSolver::ComputeProjectedDescentDirection()
{

  // Compute v1 eq (14a) in Dimitrov 2009
  for(unsigned int li=0;li<m_ActivatedConstraints.size();li++)
    {
      m_v1[li] = 0.0;
      unsigned int RowCstMatrix = m_ActivatedConstraints[li];
      for(unsigned int lj=li;lj<2*m_CardU;lj++)
	{
	  m_v1[li]+= m_A[RowCstMatrix*2*m_CardU+lj]*
	    m_UnconstrainedDescentDirection[lj];
	}
    }

  // Compute v2 by Forward and backward substitution.
  // v2 = iLt iL E c 
  ForwardSubstitution();
  BackwardSubstitution();

  // Compute d
  // d = c - Et v2
  for(unsigned int li=0;li<2*m_CardU;li++)
    {
      m_d[li] = m_UnconstrainedDescentDirection[li];
      for(unsigned int lj=0;lj<m_ActivatedConstraints.size();lj++)
	{
	  unsigned int RowCstMatrix = m_ActivatedConstraints[lj];
	  m_d[li]-= m_A[RowCstMatrix+li*m_NbOfConstraints]*
	    m_v2[lj];
	}
    }
  return 0;
}

double PLDPSolver::ComputeAlpha(vector<unsigned int> & NewActivatedConstraints)
{
  double Alpha=10000000.0;
  double *ptA = m_A;	
  

  for(unsigned li=0;li<m_NbOfConstraints;li++)
    {

      bool ConstraintFound=false;

      for(unsigned int ConstraintIndex=0;
	  ConstraintIndex<m_ActivatedConstraints.size();
	  ConstraintIndex++)
	{
	  if (m_ActivatedConstraints[ConstraintIndex]==li)
	    {
	      ConstraintFound=true;
	      break;
	    }
	}
      if (ConstraintFound)
	continue;

      m_v1[li]=0.0;
      for(unsigned lj=0;lj<2*m_CardU;lj++)
	{
	  m_v1[li]+= *ptA++ * m_d[lj];
	}

      if (m_v1[li]<0.0)
	{
	  double *pt2A = m_A + 2*m_CardU *li;	
	  m_v2[li]= m_b[li];
	  for(unsigned lj=0;lj<2*m_CardU;lj++)
	    {
	      m_v2[li]-= *pt2A++ * m_Uk[lj];
	    }
	  m_v2[li] /= m_v1[li];
	  if (Alpha>m_v2[li])
	    {
	      Alpha = m_v2[li];
	      if (Alpha<1)
		NewActivatedConstraints.push_back(li);
	    }
	}
    }

  return Alpha;
}
int PLDPSolver::SolveProblem(double *CstPartOfTheCostFunction,
			      unsigned int NbOfConstraints,
			      double *LinearPartOfConstraints,
			      double *CstPartOfConstraints,
			      double *ZMPRef,
			      double *XkYk)
{
  vector<unsigned int> NewActivatedConstraints;

  m_A = LinearPartOfConstraints;
  m_NbOfConstraints = NbOfConstraints;

  /* Step zero : Algorithm initialization. */
  m_CstPartOfCostFunction = CstPartOfTheCostFunction;
  ComputeInitialSolution(ZMPRef,XkYk);

  bool ContinueAlgo=true;

  /*! Initialization de cholesky. */
  m_OptCholesky->SetA(LinearPartOfConstraints);
  //  m_OptCholesky->SetLToZero();

  double alpha=0.0;
  while(ContinueAlgo)
    {
      /* Step one : Compute descent direction. */
      for(unsigned int i=0;i<2*m_CardU;i++)
	 m_UnconstrainedDescentDirection[i] = 
	  m_CstPartOfCostFunction[i] -  m_Uk[i];
      
      /*! Step two: Compute the projected descent direction. */
      ComputeProjectedDescentDirection();

      /*! Step three : Compute alpha */
      alpha = ComputeAlpha(NewActivatedConstraints);

      if (alpha>=1.0)
	{
	  alpha=1.0;
	  ContinueAlgo=false;
	}

      /*! Compute new solution. */
      for(unsigned int i=0;i<2*m_CardU;i++)
	{
	  m_Uk[i] = m_Uk[i] + alpha * m_d[i];
	}
      if (ContinueAlgo)
	{
	  m_OptCholesky->AddActiveConstraints(NewActivatedConstraints);
	  for(unsigned int i=0;i<NewActivatedConstraints.size();i++)
	    m_ActivatedConstraints.push_back(NewActivatedConstraints[i]);
	}
    }

  
}

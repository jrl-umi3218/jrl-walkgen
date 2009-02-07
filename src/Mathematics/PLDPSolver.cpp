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
#include <iostream>
#include <fstream>
#include <walkGenJrl/Mathematics/PLDPSolver.h>

#ifdef WIN32
#include <float.h>
#define isnan _isnan

//definition of isinf for win32
//src:  http://www.gnu.org/software/libtool/manual/autoconf/Function-Portability.html
inline int isinf (double x){return isnan (x - x);}
#endif

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; \
                         DebugFile.open(y,ofstream::out); \
                         DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; \
                       DebugFile.open(y,ofstream::app); \
                       DebugFile <<  x << endl; DebugFile.close();}
#else
#define RESETDEBUG4(y) 
#define ODEBUG4(x,y) 
#endif

#define RESETDEBUG6(y) 
#define ODEBUG6(x,y) 

#define RESETDEBUG5(y) { ofstream DebugFile; \
                         DebugFile.open(y,ofstream::out);\
                         DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; \
                       DebugFile.open(y,ofstream::app); \
                       DebugFile << x << endl; \
                       DebugFile.close();}
#define ODEBUG5NOE(x,y) { ofstream DebugFile; \
                          DebugFile.open(y,ofstream::app); \
                          DebugFile << x ; DebugFile.close();}
#if 1
#define ODEBUG(x)
#else
#define ODEBUG(x)  std::cout << "PLDPSolver " << x << endl;
#endif

#define ODEBUG3(x)  std::cout << "PLDPSolver " << x << endl;

using namespace PatternGeneratorJRL;
using namespace Optimization::Solver;
using namespace std;

PLDPSolver::PLDPSolver(unsigned int CardU,
		       double *iPu,
		       double *Px,
		       double *Pu,
		       double *iLQ)
{
  m_DebugMode = 0;

  /* Initialize pointers */
  m_CardV = CardU;

  m_Pu = Pu;
  m_iPu = iPu;
  m_Px = Px;
  m_iPuPx = 0;
  m_Vk = 0;
  m_CstPartOfCostFunction = 0;
  m_UnconstrainedDescentDirection =0;
  m_L =0 ;
  m_iLQ = iLQ;
  m_d = 0;
  m_v1 = m_v2 = m_y = 0;
  m_A = m_b= 0;

  m_NbMaxOfConstraints = 8*m_CardV;
  m_NbOfConstraints = 0;

  m_OptCholesky = new PatternGeneratorJRL::OptCholesky(m_NbMaxOfConstraints,2*m_CardV,
						       OptCholesky::MODE_FORTRAN);
  RESETDEBUG5("Infos.dat");
  AllocateMemoryForSolver();

}

void PLDPSolver::AllocateMemoryForSolver()
{
  PrecomputeiPuPx();
  m_Vk = new double[2*m_CardV];
  memset(m_Vk,0,2*m_CardV*sizeof(double));

  m_UnconstrainedDescentDirection = new double[2*m_CardV];
  memset(m_UnconstrainedDescentDirection,0,2*m_CardV*sizeof(double));

  m_L = new double[m_NbMaxOfConstraints*m_NbMaxOfConstraints];
  m_iL = new double[m_NbMaxOfConstraints*m_NbMaxOfConstraints];

  m_v1 = new double[m_NbMaxOfConstraints];
  m_v2 = new double[m_NbMaxOfConstraints];
  m_y = new double[m_NbMaxOfConstraints];

  m_ConstraintsValueComputed = new bool [m_NbMaxOfConstraints*2];

  m_d = new double[2*m_CardV];  
  m_OptCholesky->SetL(m_L);
  m_OptCholesky->SetiL(m_iL);
}

void PLDPSolver::InitializeSolver()
{
#if 0
  // Allocation max:
  // We assume that we might at max. 8 actives constraintes 
  // per control time.
  memset(m_v1,0,m_NbMaxOfConstraints*sizeof(double));
  // Same than v1.
  memset(m_v2,0,m_NbMaxOfConstraints*sizeof(double));
  // Same than y.
  memset(m_y,0,m_NbMaxOfConstraints*sizeof(double));

  // Same than L.
  memset(m_L,0,8*m_CardV*sizeof(double));
  // Same than iL.
  memset(m_iL,0,8*m_CardV*sizeof(double));

  // Same for the descent.
  memset(m_d,0,2*m_CardV*sizeof(double));
#endif
  m_ActivatedConstraints.clear();
}

PLDPSolver::~PLDPSolver()
{
  if (m_iPuPx!=0)
    delete [] m_iPuPx;

  if (m_Vk!=0)
    delete [] m_Vk;

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
  

  if ( m_ConstraintsValueComputed!=0)
    delete [] m_ConstraintsValueComputed;

}

int PLDPSolver::PrecomputeiPuPx()
{
  m_iPuPx = new double[2*m_CardV*6];

  memset(m_iPuPx,0,2*m_CardV*6);

  if (m_DebugMode>1)
    {
      ofstream aof;
      aof.open("iPu.dat",ofstream::out);
      for(unsigned int i=0;i<m_CardV;i++)
	{
	  for(unsigned int j=0;j<m_CardV;j++)
	    {
	      aof << m_iPu[i*m_CardV+j] << " " ;
	    }
	  aof << endl;
	}
      aof.close();

      aof.open("Pu.dat",ofstream::out);
      for(unsigned int i=0;i<m_CardV;i++)
	{
	  for(unsigned int j=0;j<m_CardV;j++)
	    {
	      aof << m_Pu[i*m_CardV+j] << " " ;
	    }
	  aof << endl;
	}
      aof.close();


      aof.open("Px.dat",ofstream::out);
      for(unsigned int i=0;i<m_CardV;i++)
	{
	  for(unsigned int j=0;j<3;j++)
	    {
	      aof << m_Px[i*3+j] << " " ;
	    }
	  aof << endl;
	}
      aof.close();

      aof.open("isLQ.dat",ofstream::out);
      for(unsigned int i=0;i<m_CardV;i++)
	{
	  for(unsigned int j=0;j<m_CardV;j++)
	    {
	      aof << m_iLQ[i*2*m_CardV+j] << " " ;
	    }
	  aof << endl;
	}
      aof.close();

    }
  
  // This should be reimplemented with a proper matrix library.
  // will be done in a second time.
  for(unsigned int i=0;i<m_CardV;i++)
    {
      for(unsigned int j=0;j<3;j++)
	{
	  m_iPuPx[i*6+j]= 0.0;
	  m_iPuPx[i*6+j+3]= 0.0;
	  m_iPuPx[(i+m_CardV)*6+j]= 0.0;
	  m_iPuPx[(i+m_CardV)*6+j+3]= 0.0;
	  for(unsigned int k=0;k<m_CardV;k++)
	    {
	      double tmp = m_iPu[k*m_CardV+i] * 
		m_Px[k*3+j];

	      m_iPuPx[i*6+j]+= tmp;
	      //	      m_iPuPx[i*6+j+3]+= tmp;
	      //	      m_iPuPx[(i+m_CardV)*6+j]+= tmp; 
	      m_iPuPx[(i+m_CardV)*6+j+3]+= tmp;

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
  for(unsigned int i=0;i<m_CardV;i++)
    {
      m_Vk[i] = 0.0;
      m_Vk[i+m_CardV] = 0.0;
      for(unsigned int j=0;j<3;j++)
	m_Vk[i]-= m_iPuPx[i*6+j] *  XkYk[j];

      for(unsigned int j=3;j<6;j++)
	m_Vk[i+m_CardV]-= m_iPuPx[(i+m_CardV)*6+j] *  XkYk[j];

      for(unsigned int j=0;j<m_CardV;j++)
	m_Vk[i]+= m_iPu[j*m_CardV+i] * ZMPRef[j];  

      for(unsigned int j=0;j<m_CardV;j++)
	m_Vk[i+m_CardV]+= m_iPu[j*m_CardV+i] * ZMPRef[j+m_CardV];  
      
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
      m_y[i] = m_v1[i] ;
      for(unsigned int k=0;k<i;k++)
	m_y[i] += - m_L[i*m_NbMaxOfConstraints+k]*m_y[k];

      if (m_L[i*m_NbMaxOfConstraints+i]!=0.0)
	m_y[i] /= m_L[i*m_NbMaxOfConstraints+i];
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
  unsigned int SizeOfL = m_ActivatedConstraints.size();
  if (SizeOfL==0)
    return 0;
  
  for(int i=SizeOfL-1;
      i>=0; i--)
    {
      m_v2[i]= m_y[i];
      for(int k=i+1;k<(int)SizeOfL;k++)
	m_v2[i] -=  m_L[k*m_NbMaxOfConstraints+i]*m_v2[k];
      m_v2[i] = m_v2[i]/m_L[i*m_NbMaxOfConstraints+i];
      ODEBUG("BS: m_L[i*m_NbMaxOfConstraints+i]:"<<
	      m_L[i*m_NbMaxOfConstraints+i] << " " << m_y[i]);
      ODEBUG("m_v2[i] = "<<m_v2[i]);
    }
  return 0;
}

int PLDPSolver::ComputeProjectedDescentDirection()
{

  if(m_DebugMode>1)
    {
      ofstream aof;

      char Buffer[1024];
      sprintf(Buffer,"AC_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int li=0;li<m_ActivatedConstraints.size();li++)
	aof<< m_ActivatedConstraints[li] << " ";
      aof <<endl;
      aof.close();

      sprintf(Buffer,"E_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int li=0;li<m_ActivatedConstraints.size();li++)
	{
	  unsigned int RowCstMatrix = m_ActivatedConstraints[li];
	  for(unsigned int lj=0;lj<2*m_CardV;lj++)
	    {
	      aof << m_A[RowCstMatrix+lj*(m_NbOfConstraints+1)] << " ";
	    }
	  aof <<endl;
	}
      aof.close();

      sprintf(Buffer,"c_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int li=0;li<2*m_CardV;li++)
	{
	  aof << m_UnconstrainedDescentDirection[li] << " " ;
	}
      aof << endl;
      aof.close();
    }
  // Compute v1 eq (14a) in Dimitrov 2009
  for(unsigned int li=0;li<m_ActivatedConstraints.size();li++)
    {
      m_v1[li] = 0.0;
      unsigned int RowCstMatrix = m_ActivatedConstraints[li];
      ODEBUG("RowCstMatrix:"<<RowCstMatrix);
      for(unsigned int lj=0;lj<2*m_CardV;lj++)
	{
	  m_v1[li]+= m_A[RowCstMatrix+lj*(m_NbOfConstraints+1)]*
	    m_UnconstrainedDescentDirection[lj];
	}
      ODEBUG("m_v1["<<li<<" ]="<< m_v1[li]);
    }

  if (m_DebugMode>1)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"v1_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int lj=0;lj<m_ActivatedConstraints.size();lj++)
	aof << m_v1[lj] << endl;
      aof.close();
    }

  // Compute v2 by Forward and backward substitution.
  // v2 = iLt iL E c 
  // LtL v2 = c
  // Lt y =c
  ForwardSubstitution();
  if (m_DebugMode>1)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"y_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int lj=0;lj<m_ActivatedConstraints.size();lj++)
	aof << m_y[lj] << endl;
      aof.close();

    }
  
  // Solve L v2 = y
  BackwardSubstitution();

  if (m_DebugMode>1)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"v2_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int lj=0;lj<m_ActivatedConstraints.size();lj++)
	aof << m_v2[lj] << endl;
      aof.close();
    }

  // Compute d
  // d = c - Et v2
  ODEBUG("Size of ActivatedConstraints: "<<
	  m_ActivatedConstraints.size());
  for(unsigned int li=0;li<2*m_CardV;li++)
    {
      m_d[li] = m_UnconstrainedDescentDirection[li];
      for(unsigned int lj=0;lj<m_ActivatedConstraints.size();lj++)
	{
	  unsigned int RowCstMatrix = m_ActivatedConstraints[lj];
	  m_d[li]-= m_A[RowCstMatrix+li*(m_NbOfConstraints+1)]*
	    m_v2[lj];
	}
    }

  if (m_DebugMode>1)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"UDD_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int lj=0;lj<2*m_CardV;lj++)
	aof  << m_d[lj] << " ";
      aof << endl;
      aof.close();
    }
  return 0;
}

double PLDPSolver::ComputeAlpha(vector<unsigned int> & NewActivatedConstraints,
				vector<int> &SimilarConstraint)
{
  double Alpha=10000000.0;
  double *ptA = 0;
  bool TheConstraintIsToBeAdded=false;
  unsigned int TheConstraintToActivate=0;

  for(unsigned li=0;li<m_NbOfConstraints;li++)
    {

      bool ConstraintFound=false;

      m_ConstraintsValueComputed[li] = false;
      m_ConstraintsValueComputed[li+m_NbOfConstraints] = false;

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

      ptA = m_A + li;

      m_v1[li]=0.0;

      // Check if we can not reuse an already computed result   
      {
	bool ToBeComputed=true;
	if (SimilarConstraint[li]!=0) 
	  {
	    int lindex = li+SimilarConstraint[li];
	    if (m_ConstraintsValueComputed[lindex])
	      {
		m_v1[li] = -m_v1[lindex];
		ToBeComputed=false;
	      }
	  }
	
	if(ToBeComputed)
	  for(unsigned lj=0;lj<2*m_CardV;lj++)
	    {
	      m_v1[li]+= *ptA * m_d[lj];
	      ptA+=(m_NbOfConstraints+1);
	    }
      }
      
      m_ConstraintsValueComputed[li] = true;

      if (m_v1[li]<0.0)
	{
	  double lalpha=0.0;
	  double *pt2A = m_A + li;	
	  m_v2[li]= -m_b[li];

	  // Check if we can not reuse an already computed result
	  {
	    bool ToBeComputed=true;
	    if (SimilarConstraint[li]!=0) 
	      {
		int lindex = li+SimilarConstraint[li];
		if (m_ConstraintsValueComputed[lindex+m_NbOfConstraints])
		  {
		    m_v2[li] += -m_v2[lindex]-m_b[lindex];
		    ToBeComputed=false;
		  }
	      }
	    
	    if(ToBeComputed)
	      for(unsigned lj=0;lj<2*m_CardV;lj++)
		{
		  m_v2[li]-= *pt2A * m_Vk[lj];
		  pt2A+=(m_NbOfConstraints+1);
		}
	  }

	  if (m_v2[li]>0.0)
	    {
	      cout << "PB ON constraint "<<li<< endl;
	      cout << " Check current V k="<<m_ItNb<< endl;
	      cout << " should be faisable : " << -m_v2[li] << endl;
	    }
	  lalpha = m_v2[li]/m_v1[li];

	  if (Alpha>lalpha)
	    {
	      ODEBUG("m_v2[li] : "<< m_v2[li] <<
		      " m_v1[li] : " << m_v1[li] << " " 
		      " lalpha: "<< lalpha << " "
		      " Constrainte " << li << " on "
		      << m_NbOfConstraints << " constraints.");
		  
	      Alpha = lalpha;
	      if (Alpha<1)
		{
		  TheConstraintIsToBeAdded = true;
		  TheConstraintToActivate=li;
		}
	    }
	}
    }

  if (TheConstraintIsToBeAdded)
    NewActivatedConstraints.push_back(TheConstraintToActivate);

  return Alpha;
}
int PLDPSolver::SolveProblem(double *CstPartOfTheCostFunction,
			     unsigned int NbOfConstraints,
			     double *LinearPartOfConstraints,
			     double *CstPartOfConstraints,
			     double *ZMPRef,
			     double *XkYk,
			     double *X,
			     vector<int> &SimilarConstraints)
{
  static double lTime=-0.02;
  vector<unsigned int> NewActivatedConstraints;

  lTime+=0.02;
  InitializeSolver();

  m_A = LinearPartOfConstraints;
  m_b = CstPartOfConstraints;
  m_NbOfConstraints = NbOfConstraints;

  /* Step zero : Algorithm initialization. */
  m_CstPartOfCostFunction = CstPartOfTheCostFunction;

  ODEBUG("State: " << XkYk[0] << " " << XkYk[3] << " " <<
	  XkYk[1] << " " << XkYk[4] << " " <<
	  XkYk[2] << " " << XkYk[5] << " ");
  
  ComputeInitialSolution(ZMPRef,XkYk);
 

  if (m_DebugMode>1)
    {
      string DebugVkFileName("VkInit.dat");
      WriteCurrentZMPSolution("VkInit.dat",XkYk);

      ofstream aof;
      aof.open("InitialSolution.dat",ofstream::out);
      for(unsigned int i=0;i<2*m_CardV;i++)
	aof <<m_Vk[i] << " ";
      aof << endl;
      aof.close();

      aof.open("iPuPx.dat",ofstream::out);
      for(unsigned int i=0;i<2*m_CardV;i++)
	{
	  for(unsigned int j=0;j<6;j++)
	    aof << m_iPuPx[i*6+j] << " ";
	  aof <<endl;
	}
      aof << endl;
      aof.close();

      aof.open("A.dat",ofstream::out);
      for(unsigned int i=0;i<m_NbOfConstraints;i++)
	{
	  for(unsigned int j=0;j<2*m_CardV;j++)
	    aof << m_A[j*(m_NbOfConstraints+1)+i] << " ";
	  aof <<endl;
	}
      aof << endl;
      aof.close();

      aof.open("b.dat",ofstream::out);
      for(unsigned int i=0;i<m_NbOfConstraints;i++)
	{
	  aof << m_b[i] << " ";
	}
      aof << endl;
      aof.close();

      aof.open("ZMPRef.dat",ofstream::out);
      for(unsigned int i=0;i<2*m_CardV;i++)
	{
	    aof << ZMPRef[i] << " ";
	}
      aof << endl;
      aof.close();

      aof.open("XkYk.dat",ofstream::out);
      for(unsigned int i=0;i<6;i++)
	{
	    aof << XkYk[i] << " ";
	}
      aof << endl;
      aof.close();
    }
  if (m_DebugMode>1)
    {
      ofstream aof;
      aof.open("A.dat",ofstream::out);
      for(unsigned int i=0;i<m_NbOfConstraints;i++)
	{
	  for(unsigned int j=0;j<2*m_CardV;j++)
	    {
	      aof << m_A[i+j*(NbOfConstraints+1)] << " ";
	    }
	  aof << endl;
	}
      aof << endl;
      aof.close();

      aof.open("b.dat",ofstream::out);
      
      for(unsigned int j=0;j<m_NbOfConstraints;j++)
	{
	  aof << m_b[j] << " ";
	}
      aof << endl;

      aof.close();

    }


  bool ContinueAlgo=true;

  /*! Initialization de cholesky. */
  m_OptCholesky->SetA(LinearPartOfConstraints,m_NbOfConstraints);
  m_OptCholesky->SetToZero();

  double alpha=0.0;
  m_ItNb=0;
  while(ContinueAlgo)
    {
      ODEBUG("Iteration Number:" << m_ItNb);
      /* Step one : Compute descent direction. */
      for(unsigned int i=0;i<2*m_CardV;i++)
	 m_UnconstrainedDescentDirection[i] = 
	  -m_CstPartOfCostFunction[i] -  m_Vk[i];
      
      /*! Step two: Compute the projected descent direction. */
      ComputeProjectedDescentDirection();

      /*! Step three : Compute alpha */
      alpha = ComputeAlpha(NewActivatedConstraints,
			   SimilarConstraints);

      if (alpha>=1.0)
	{
	  alpha=1.0;
	  ContinueAlgo=false;
	}
      if (alpha<0.0)
	{
	  ODEBUG3("Problem with alpha: should be positive");
	  ODEBUG3("The initial solution is incorrect: "<< m_ItNb << " " << lTime);

	  exit(0);
	}

      /*! Compute new solution. */
      for(unsigned int i=0;i<2*m_CardV;i++)
	{
	  m_Vk[i] = m_Vk[i] + alpha * m_d[i];
	}

      if (m_DebugMode>1)
	{
	  
	  ODEBUG("Alpha:" <<alpha);
	  
	  ofstream aof;
	  char Buffer[1024];
	  sprintf(Buffer,"U_%02d.dat",m_ItNb);
	  aof.open(Buffer,ofstream::out);
	  for(unsigned int i=0;i<2*m_CardV;i++)
	    {
	      aof << m_Vk[i] << " " ;
	    }
	  aof.close();;
	}

      if (ContinueAlgo)
	{
	  ODEBUG("Nb of activated constraints: " << NewActivatedConstraints.size() 
		  << " " << lTime);
	  m_OptCholesky->AddActiveConstraints(NewActivatedConstraints);
	  for(unsigned int i=0;i<NewActivatedConstraints.size();i++)
	    m_ActivatedConstraints.push_back(NewActivatedConstraints[i]);

	  NewActivatedConstraints.clear();

	  if (m_DebugMode>1)
	    {
	      ofstream aof;
	      char Buffer[1024];
	      sprintf(Buffer,"LE_%02d.dat",m_ItNb);
	      aof.open(Buffer,ofstream::out);
	      for(unsigned int i=0;i<m_ActivatedConstraints.size();i++)
		{
		  for(unsigned int j=0;j<m_ActivatedConstraints.size();j++)
		    {
		      aof << m_L[i*m_NbMaxOfConstraints+j] << " ";
		    }
		  aof<<endl;
		}
	      aof.close();
	      ODEBUG("m_L(0,0)= " << m_L[0]);
	      sprintf(Buffer,"alpha_%02d.dat",m_ItNb);
	      aof.open(Buffer,ofstream::out);
	      aof << alpha << endl;
	      aof.close();

	    }

	}

      m_ItNb++;
    }

  for(unsigned int i=0;i<2*m_CardV;i++)
    X[i] = m_Vk[i];

  if ((isnan(X[0])) ||
      (isnan(X[m_CardV])) ||
      (isinf(X[0])) ||
      (isinf(X[m_CardV])) 
      )
    {
      cout << "Nan or inf value " << X[0]<< " " << X[m_CardV] 
	   << " at iteration " << m_ItNb -1 <<endl;
      return -1;
    }

  if (m_DebugMode>1)
    {
      ofstream aof;
      aof.open("UFinal.dat",ofstream::out);
      for(unsigned int i=0;i<2*m_CardV;i++)
	{
	  aof << X[i] << " " ;
	}
      aof<<endl;
      aof.close();

      aof.open("Pb.dat",ofstream::out);
      aof<<" Number of iterations " << m_ItNb << endl;
      aof.close();

    }
  

  ODEBUG5(m_ActivatedConstraints.size() << " " << NbOfConstraints,"Infos.dat");
  return 0;
}

/* Write the current solution for debugging. */
void PLDPSolver::WriteCurrentZMPSolution(string filename,
					 double *XkYk)
{
  // The current solution is Vk,
  // but its graphical representation is better understood
  // when transformed in ZMP ref trajectory.
  int lZMP_size=2*m_CardV;
  double* lZMP = new double [lZMP_size];

  ofstream aof;
  aof.open((char *)filename.c_str(),ofstream::out);

	  
  for(unsigned int i=0;i<m_CardV;i++)
    {
      lZMP[i] = 0.0; // X axis
      lZMP[i+m_CardV] =0.0; // Y axis
      for(unsigned int j=0;j<m_CardV;j++)
	{
	  lZMP[i] += m_Pu[j*m_CardV+i] * m_Vk[j];
	  lZMP[i+m_CardV] += m_Pu[j*m_CardV+i] * m_Vk[j+m_CardV];
	}
      for(unsigned int j=0;j<3;j++)
	{
	  lZMP[i] += m_Px[i*3+j] * XkYk[j];
	  //	  lZMP[i] += m_Px[i*3+j] * XkYk[j+3];
	  //	  lZMP[i+m_CardV] += m_Px[i*3+j] * XkYk[j];
	  lZMP[i+m_CardV] += m_Px[i*3+j] * XkYk[j+3];
	}
      aof << lZMP[i] << " " << lZMP[i+m_CardV] << endl;
    }

  aof.close();
  delete lZMP;
}

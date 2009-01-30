/* This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of steps following a QP
   formulation and a new QP solver as proposed by Dimitrov ICRA 2009.

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
   * Neither the name of the CNRS/AIST nor the names of its contributors 
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

#ifdef UNIX
#include <sys/time.h>
#endif /* UNIX */

#ifdef WIN32
#include <Windows.h>
#include <walkGenJrl/TimeUtilsWindows.h>
#endif

#include <time.h>

#include <iostream>
#include <fstream>

#include "jrlMathTools/jrlConstants.h"
#include <walkGenJrl/Mathematics/qld.h>
#include <walkGenJrl/ZMPRefTrajectoryGeneration/ZMPConstrainedQPFastFormulation.h>

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
#define ODEBUG(x)  std::cout << "ZMPConstrainedQPFastFormulation: " << x << endl;
#endif

#define ODEBUG3(x)  std::cout << "ZMPConstrainedQPFastFormulation: " << x << endl;

using namespace std;
using namespace PatternGeneratorJRL;

ZMPConstrainedQPFastFormulation::ZMPConstrainedQPFastFormulation(SimplePluginManager *lSPM, 
								 string DataFile,
								 HumanoidSpecificities *aHS) :
  ZMPRefTrajectoryGeneration(lSPM)
{
  m_Q = 0;
  m_Pu = 0;
  m_FullDebug = 0;
  m_FastFormulationMode = QLDANDLQ;

  /*! Getting the ZMP reference from Kajita's heuristic. */
  m_ZMPD = new ZMPDiscretization(lSPM,DataFile,aHS);

  /*! For simulating the linearized inverted pendulum in 2D. */
  m_2DLIPM = new LinearizedInvertedPendulum2D();

  /*! For computing the stability constraints from the feet positions. */
  m_FCALS = new FootConstraintsAsLinearSystem(lSPM,aHS);
  
  // Register method to handle
  string aMethodName[1] = 
    {":setdimitrovconstraint"};
  
  for(int i=0;i<1;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
    }

  m_ConstraintOnX = 0.04;
  m_ConstraintOnY = 0.04;
  
  m_QP_T = 0.02;
  m_QP_N = 75;

  m_SamplingPeriod = 0.005;

  m_ComHeight = 0.80;

  /* Initialize  the 2D LIPM */
  m_2DLIPM->SetSimulationControlPeriod(m_QP_T);
  m_2DLIPM->SetRobotControlPeriod(m_SamplingPeriod);
  m_2DLIPM->SetComHeight(m_ComHeight);
  m_2DLIPM->InitializeSystem();

  m_Alpha = 200.0;
  m_Beta = 1000.0;
  
  InitConstants();

  RESETDEBUG4("Check2DLIPM.dat");
}

ZMPConstrainedQPFastFormulation::~ZMPConstrainedQPFastFormulation()
{

  if (m_ZMPD!=0)
    delete m_ZMPD;

  if (m_2DLIPM!=0)
    delete m_2DLIPM;

  if (m_FCALS!=0)
    delete m_FCALS;

  if (m_Q!=0)
    delete m_Q;

}

void ZMPConstrainedQPFastFormulation::SetPreviewControl(PreviewControl *aPC)
{
  m_ZMPD->SetPreviewControl(aPC);
}

int ZMPConstrainedQPFastFormulation::InitializeMatrixPbConstants()
{
  MAL_MATRIX_RESIZE(m_PPu,2*m_QP_N,2*m_QP_N);
  MAL_MATRIX_RESIZE(m_VPu,2*m_QP_N,2*m_QP_N);
  MAL_MATRIX_RESIZE(m_PPx,2*m_QP_N,6);
  MAL_MATRIX_RESIZE(m_VPx,2*m_QP_N,6);

  for(unsigned int i=0;i<m_QP_N;i++)
    {
      // Compute VPx and PPx
      m_VPx(i,0)   = 0.0;   m_VPx(i,1) =     1.0; m_VPx(i,2)   = (i+1)*m_QP_T;
      m_VPx(i,3)   = 0.0;   m_VPx(i,4) =     0.0; m_VPx(i,5)   = 0.0;
      m_VPx(i+m_QP_N,0) = 0.0;   m_VPx(i+m_QP_N,1) =   0.0; m_VPx(i+m_QP_N,2) = 0.0;
      m_VPx(i+m_QP_N,3) = 0.0;   m_VPx(i+m_QP_N,4) =   1.0; m_VPx(i+m_QP_N,5) = (i+1)*m_QP_T;

      m_PPx(i,0) = 1.0; m_PPx(i,1)     = (i+1)*m_QP_T; m_PPx(i,2) = (i+1)*(i+1)*m_QP_T*m_QP_T*0.5;
      m_PPx(i,3) = 0.0; m_PPx(i,4)     =       0; m_PPx(i,5) = 0.;
      m_PPx(i+m_QP_N,0) = 0.0; m_PPx(i+m_QP_N,1) =     0.0; m_PPx(i+m_QP_N,2) = 0.0;
      m_PPx(i+m_QP_N,3) = 1.0; m_PPx(i+m_QP_N,4) = (i+1)*m_QP_T; m_PPx(i+m_QP_N,5) = (i+1)*(i+1)*m_QP_T*m_QP_T*0.5;
      
      
      for(unsigned int j=0;j<m_QP_N;j++)
	{
	  m_PPu(i,j)=0;
	  
	  if (j<=i)
	    {

	      m_VPu(i,j)= (2*(i-j)+1)*m_QP_T*m_QP_T*0.5 ;
	      m_VPu(i+m_QP_N,j+m_QP_N)= (2*(i-j)+1)*m_QP_T*m_QP_T*0.5 ;
	      m_VPu(i,j+m_QP_N)=0.0;
	      m_VPu(i+m_QP_N,j)=0.0;


	      m_PPu(i,j)= (1 + 3*(i-j) + 3*(i-j)*(i-j)) * m_QP_T*m_QP_T*m_QP_T/6.0;
	      m_PPu(i+m_QP_N,j+m_QP_N)= (1 + 3*(i-j) + 3*(i-j)*(i-j)) * m_QP_T*m_QP_T*m_QP_T/6.0;
	      m_PPu(i,j+m_QP_N)=0.0;
	      m_PPu(i+m_QP_N,j)=0.0;

	    }
	  else
	    {

	      m_VPu(i,j) = 0.0;
	      m_VPu(i+m_QP_N,j+m_QP_N)=0.0;
	      m_VPu(i,j+m_QP_N)=0.0;
	      m_VPu(i+m_QP_N,j)=0.0;

	      m_PPu(i,j) = 0.0;
	      m_PPu(i+m_QP_N,j+m_QP_N)=0.0;
	      m_PPu(i,j+m_QP_N)=0.0;
	      m_PPu(i+m_QP_N,j)=0.0;

	    }

	}
    }

  if (m_FullDebug>2)
    {
      ofstream aof;
      aof.open("VPx.dat");
      aof << m_VPx;
      aof.close();
      
      aof.open("m_PPx.dat");
      aof << m_PPx;
      aof.close();
      
      aof.open("VPu.dat");
      aof << m_VPu;
      aof.close();
      
      aof.open("PPu.dat");
      aof << m_PPu;
      aof.close();
    }

  return 0;
}

int ZMPConstrainedQPFastFormulation::BuildingConstantPartOfTheObjectiveFunctionQLD(MAL_MATRIX(,double) &OptA)
{
  for(unsigned int i=0;i<2*m_QP_N;i++)
    for(unsigned int j=0;j<2*m_QP_N;j++)
      m_Q[i*m_QP_N*2+j] = OptA(j,i);
  
  return 0;
}
int ZMPConstrainedQPFastFormulation::BuildingConstantPartOfTheObjectiveFunctionQLDANDLQ(MAL_MATRIX(,double) &OptA)
{
  ODEBUG3("Go through -BuildingConstantPartOfTheObjectiveFunctionQLDANDLQ");

  /*! Build cholesky matrix of the optimum 
    We copy only the upper corner of the OptA matrix
    because we know its specific structure.
   */
  double *localQ=new double[m_QP_N*m_QP_N]; 
  for(unsigned int i=0;i<m_QP_N;i++)
    for(unsigned int j=0;j<m_QP_N;j++)
      localQ[i*m_QP_N+j] = OptA(i,j);
  
  double *localLQ=new double[m_QP_N*m_QP_N]; 
  double *localiLQ=new double[m_QP_N*m_QP_N]; 
  
  memset(localLQ,0,m_QP_N*m_QP_N*sizeof(double));
  memset(localiLQ,0,m_QP_N*m_QP_N*sizeof(double));

  OptCholesky anOCD(m_QP_N,m_QP_N);
  anOCD.SetA(localQ);
  anOCD.SetL(localLQ);
  anOCD.SetiL(localiLQ);
  
  anOCD.ComputeNormalCholeskyOnA();
  anOCD.ComputeInverseCholesky(1);
  
  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"localQ.dat");
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<m_QP_N;i++)
	{
	  for(unsigned int j=0;j<m_QP_N;j++)
	    aof << localQ[i*m_QP_N+j] << " ";
	  aof<<endl;
	}
      aof.close(); 
      
      sprintf(Buffer,"localLQ.dat");
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<m_QP_N;i++)
	{
	  for(unsigned int j=0;j<m_QP_N;j++)
	    aof << localLQ[i*m_QP_N+j] << " ";
	  aof << endl;
	}
      aof.close(); 

      sprintf(Buffer,"localiLQ.dat");
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<m_QP_N;i++)
	{
	  for(unsigned int j=0;j<m_QP_N;j++)
	    aof << localiLQ[i*m_QP_N+j] << " ";
	  aof << endl;
	}
      aof.close(); 

    }  

  
  MAL_MATRIX_RESIZE(m_LQ,2*m_QP_N,2*m_QP_N);
  MAL_MATRIX_RESIZE(m_iLQ,2*m_QP_N,2*m_QP_N);
  
  
  for(unsigned int i=0;i<m_QP_N;i++)
    {
      for(unsigned int j=0;j<m_QP_N;j++)
	{
	  m_LQ(i,j) = localLQ[i*m_QP_N+j];
	  m_LQ(i+m_QP_N,j+m_QP_N) = localLQ[i*m_QP_N+j];
	  m_LQ(i,j+m_QP_N) = 0.0;
	  m_LQ(i+m_QP_N,j) = 0.0;
	  
	  m_iLQ(i,j) = localiLQ[i*m_QP_N+j];
	  m_iLQ(i+m_QP_N,j+m_QP_N) = localiLQ[i*m_QP_N+j];
	  m_iLQ(i,j+m_QP_N) = 0.0;
	  m_iLQ(i+m_QP_N,j) = 0.0;
	}
    }


  // New formulation (Dimitar08)
  m_OptB = MAL_RET_A_by_B(m_iLQ,m_OptB);

  // New formulation (Dimitar08)
  m_OptC = MAL_RET_A_by_B(m_iLQ,m_OptC);

  if (m_FullDebug>0)
    {  
      ofstream aof;
      char Buffer[1024];

      sprintf(Buffer,"LQ.dat");
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<2*m_QP_N;i++)
	{
	  for(unsigned int j=0;j<2*m_QP_N;j++)
	    aof << m_LQ(i,j) << " ";
	  aof << endl;
	}
      aof.close(); 
      
      sprintf(Buffer,"iLQ.dat");
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<2*m_QP_N;i++)
	{
	  for(unsigned int j=0;j<2*m_QP_N;j++)
	    aof << m_iLQ(i,j) << " ";
	  aof << endl;
	}
      aof.close(); 
    }
  delete [] localLQ;
  delete [] localiLQ;
 
  return 0;
}

int ZMPConstrainedQPFastFormulation::BuildingConstantPartOfTheObjectiveFunction()
{

  MAL_MATRIX(OptA,double);

  //  OptA = Id + alpha * VPu.Transpose() * VPu + beta * PPu.Transpose() * PPu;
  MAL_MATRIX(lterm1,double);
  lterm1 = MAL_RET_TRANSPOSE(m_PPu);
  lterm1 = MAL_RET_A_by_B(lterm1, m_PPu);
  lterm1 = m_Beta * lterm1;

  MAL_MATRIX(lterm2,double);
  lterm2 = MAL_RET_TRANSPOSE(m_VPu);
  lterm2 = MAL_RET_A_by_B(lterm2,m_VPu);
  lterm2 = m_Alpha * lterm2;

  MAL_MATRIX_RESIZE(OptA,
		    MAL_MATRIX_NB_ROWS(lterm1),
		    MAL_MATRIX_NB_COLS(lterm1));
  MAL_MATRIX_SET_IDENTITY(OptA);
  OptA = OptA + lterm1 + lterm2;

  // Initialization of the matrice regarding the quadratic
  // part of the objective function.
  m_Q=new double[4*m_QP_N*m_QP_N]; 
  memset(m_Q,0,4*m_QP_N*m_QP_N*sizeof(double));
  for(unsigned int i=0;i<2*m_QP_N;i++)
    m_Q[i*2*m_QP_N+i] = 1.0;


  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"Q.dat");
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<2*m_QP_N;i++)
	{
	  for(unsigned int j=0;j<2*m_QP_N;j++)
	    aof << m_Q[i*m_QP_N*2+j] << " ";
	  aof << endl;
	}
      aof.close(); 
    }

  /*! Compute constants of the linear part of the objective function. */
  lterm1 = MAL_RET_TRANSPOSE(m_PPu);
  lterm1 = MAL_RET_A_by_B(lterm1,m_PPx);
  m_OptB = MAL_RET_TRANSPOSE(m_VPu);
  m_OptB = MAL_RET_A_by_B(m_OptB,m_VPx);
  m_OptB = m_Alpha * m_OptB;
  m_OptB = m_OptB + m_Beta * lterm1;
  
  m_OptC = MAL_RET_TRANSPOSE(m_PPu);
  m_OptC = m_Beta * m_OptC;

  if (m_FastFormulationMode==QLDANDLQ)
    {
      BuildingConstantPartOfTheObjectiveFunctionQLDANDLQ(OptA);
    }  
  else 
    {
      BuildingConstantPartOfTheObjectiveFunctionQLD(OptA);
    }

  if (m_FullDebug>0)
  {
    ofstream aof;
    char Buffer[1024];
    sprintf(Buffer,"OptB.dat");
    aof.open(Buffer,ofstream::out);
    for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(m_OptB);i++)
      {
	for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(m_OptB)-1;j++)
	  aof << m_OptB(i,j) << " ";
	aof << m_OptB(i,MAL_MATRIX_NB_COLS(m_OptB)-1);
	aof << endl;
      }
    aof.close(); 
    
  }
  
  if (m_FullDebug>0)
  {
    ofstream aof;
    char Buffer[1024];
    sprintf(Buffer,"OptC.dat");
    aof.open(Buffer,ofstream::out);
    for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(m_OptC);i++)
      {
	for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(m_OptC)-1;j++)
	  aof << m_OptC(i,j) << " ";
	aof << m_OptC(i,MAL_MATRIX_NB_COLS(m_OptC)-1);
	aof << endl;
      }
    aof.close(); 
    
  }

  return 0;
}

int ZMPConstrainedQPFastFormulation::BuildingConstantPartOfConstraintMatrices()
{
  if (m_Pu==0)
    m_Pu = new double[m_QP_N*m_QP_N];

  double * lInterPu=0;
  double * ptPu=0;
  
  if (m_FastFormulationMode==QLDANDLQ)
    {
      lInterPu = new double[m_QP_N*m_QP_N];
      memset(lInterPu,0,m_QP_N*m_QP_N*sizeof(double));
      ptPu = lInterPu;
    }
  else 
    ptPu = m_Pu;

  memset(m_Pu,0,m_QP_N*m_QP_N*sizeof(double));

  // Recursive multiplication of the system is applied.
  // we keep the transpose form, i.e. Pu'.
  for(unsigned i=0;i<m_QP_N;i++)
    {

      for(unsigned k=0;k<=i;k++)
	{
	  ptPu[k*m_QP_N+i] = 
	    ((1+3*(i-k)+3*(i-k)*(i-k))*m_QP_T*m_QP_T*m_QP_T/6.0 - m_QP_T * m_ComHeight/9.81);
	  ODEBUG("IC: " << IndexConstraint );
	}
    }
  
  // Consider QLDANDLQ formulation.
  if (m_FastFormulationMode==QLDANDLQ)
    {
      // Premultiplication by LQ-1
      // Indeed we have to provide qld transpose matrix,
      // therefore instead of providing D*Pu*iLQ (IROS 2008 p 28)
      // we provide its transpose:
      // (D*Pu*iLQ')' = iLQ*Pu'*D'
      // Be careful with the two stages resolution.
      for(unsigned i=0;i<m_QP_N;i++)
	{
	
	  for(unsigned j=0;j<m_QP_N;j++)
	    {
	      m_Pu[i*m_QP_N+j] = 0;
	      for(unsigned k=0;k<m_QP_N;k++)
		{
		  m_Pu[i*m_QP_N+j] += m_iLQ(i,k) * ptPu[k*m_QP_N+j];
		  ODEBUG("IC: " << IndexConstraint );
		}
	    }
	}
    }

  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"PuCst.dat");
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<m_QP_N;i++)
	{
	  for(unsigned int j=0;j<m_QP_N;j++)
	    aof << m_Pu[j+i*m_QP_N] << " " ;
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"tmpPuCst.dat");
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<m_QP_N;i++)
	{
	  for(unsigned int j=0;j<m_QP_N;j++)
	    aof << ptPu[j+i*m_QP_N] << " " ;
	  aof << endl;
	}
      aof.close();

      ODEBUG3("m_iLQ:" << MAL_MATRIX_NB_ROWS(m_iLQ) << " " <<
	      MAL_MATRIX_NB_COLS(m_iLQ) );
      sprintf(Buffer,"tmpiLQ.dat");
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<m_QP_N;i++)
	{
	  for(unsigned int j=0;j<m_QP_N;j++)
	    aof << m_iLQ(i,j) << " " ;
	  aof << endl;
	}
      aof.close();
      
    }
    
  delete [] lInterPu;
  return 0;
}


int ZMPConstrainedQPFastFormulation::InitConstants()
{
  int r;
  if ((r=InitializeMatrixPbConstants())<0)
    return r;

  if ((r=BuildingConstantPartOfTheObjectiveFunction())<0)
    return r;
  
  if ((r=BuildingConstantPartOfConstraintMatrices())<0)
    return r;

  return 0;
}

void ZMPConstrainedQPFastFormulation::SetAlpha(const double &anAlpha)
{
  m_Alpha = anAlpha;
}

const double & ZMPConstrainedQPFastFormulation::GetAlpha() const
{
  return m_Alpha;
}

void ZMPConstrainedQPFastFormulation::SetBeta(const double &anAlpha)
{
  m_Beta = anAlpha;
}

const double & ZMPConstrainedQPFastFormulation::GetBeta() const
{
  return m_Beta;
}


int ZMPConstrainedQPFastFormulation::BuildConstraintMatrices(double * & Px,double * &Pu, 
							     unsigned N, double T,
							     double StartingTime,
							     deque<LinearConstraintInequality_t *> & 
							     QueueOfLConstraintInequalities,
							     double Com_Height,
							     unsigned int &NbOfConstraints,
							     MAL_VECTOR(& xk,double))
{
  // Discretize the problem.
  ODEBUG(" N:" << N << " T: " << T);
  
  // Creates the matrices.
  // The memory will be bounded to 8 constraints per
  // support foot (double support case).
  // Will be probably all the time smaller.
  if (Px==0)
    Px = new double[8*N+1];

  if (Pu==0)
    Pu = new double[(8*N+1)*2*N];

  memset(Pu,0,(8*N+1)*2*N*sizeof(double));

  deque<LinearConstraintInequality_t *>::iterator LCI_it, store_it;
  LCI_it = QueueOfLConstraintInequalities.begin();
  while (LCI_it!=QueueOfLConstraintInequalities.end())
    {
      if ((StartingTime>=(*LCI_it)->StartingTime) &&
	  (StartingTime<=(*LCI_it)->EndingTime))
	{
	  break;
	}
      LCI_it++;
    }
  store_it = LCI_it;
  

  // Did not find the appropriate Linear Constraint.
  if (LCI_it==QueueOfLConstraintInequalities.end())
    {
      cout << "HERE 3" << endl;
      return -1;
    }
      
  if (m_FullDebug>2)
    {
      char Buffer[1024];
      sprintf(Buffer,"PXD_%f.dat", StartingTime);
      RESETDEBUG4(Buffer);
      ODEBUG6("xk:" << xk << " Starting time: " <<StartingTime ,Buffer );
      char Buffer2[1024];
      sprintf(Buffer2,"PXxD_%f.dat", StartingTime);
      RESETDEBUG4(Buffer2);
      
      char Buffer3[1024];
      sprintf(Buffer3,"PXyD_%f.dat", StartingTime);
      RESETDEBUG4(Buffer3);
    }
  
  // Compute first the number of constraint.
  unsigned int IndexConstraint=0;
  for(unsigned int i=0;i<N;i++)
    {

      double ltime = StartingTime+ i* T;
      if (ltime > (*LCI_it)->EndingTime)
	LCI_it++;

      if (LCI_it==QueueOfLConstraintInequalities.end())
	{
	  break;
	}
      IndexConstraint += MAL_MATRIX_NB_ROWS((*LCI_it)->A);
    }  
  NbOfConstraints = IndexConstraint;
  
  MAL_MATRIX(lD,double);
  MAL_MATRIX_RESIZE(lD,2*N,NbOfConstraints);
  
  LCI_it = store_it;
  IndexConstraint = 0;
  ODEBUG("Starting Matrix to build the constraints. ");
  ODEBUG((*LCI_it)->A );
  for(unsigned int i=0;i<N;i++)
    {

      double ltime = StartingTime+ i* T;
      if (ltime > (*LCI_it)->EndingTime)
	{
	  LCI_it++;
	}

      // For each constraint.
      for(unsigned j=0;j<MAL_MATRIX_NB_ROWS((*LCI_it)->A);j++)
	{
	  Px[IndexConstraint] = 
	    // X Axis * A
	    (xk[0] +
	     xk[1] * T *(i+1) + 
	     xk[2]*((i+1)*(i+1)*T*T/2 - Com_Height/9.81))
	    * (*LCI_it)->A(j,0)
	     + 
	     // Y Axis * A
	    ( xk[3]+ xk[4]* T * (i+1) + 
	       xk[5]*((i+1)*(i+1)*T*T/2 - Com_Height/9.81))	  
	    * (*LCI_it)->A(j,1)
	     // Constante part of the constraint
	    + (*LCI_it)->B(j,0);

	  ODEBUG6(Px[IndexConstraint] << " " << (*LCI_it)->A(j,0)  << " "
		  << (*LCI_it)->A[j][1] << " " << (*LCI_it)->B(j,0) ,Buffer);
	  ODEBUG6(1 << " " <<    T *(i+1) << " " <<    (i+1)*(i+1)*T*T/2 - Com_Height/9.81,Buffer2);
	  ODEBUG6(1 << " " <<    T *(i+1) << " " <<    (i+1)*(i+1)*T*T/2 - Com_Height/9.81,Buffer3);

	  lD(i,IndexConstraint) = (*LCI_it)->A(j,0);
	  lD(i+N,IndexConstraint) = (*LCI_it)->A(j,1);
	  if (m_FastFormulationMode==QLD)
	    {
	      // In this case, Pu is triangular.
	      // so we can speed up the computation.
	      for(unsigned k=0;k<=i;k++)
		{
		  // X axis
		  Pu[IndexConstraint+k*(NbOfConstraints+1)] = 
		    (*LCI_it)->A(j,0)*m_Pu[k*N+i];
		  // Y axis
		  Pu[IndexConstraint+(k+N)*(NbOfConstraints+1)] = 
		    (*LCI_it)->A(j,1)*m_Pu[k*N+i];	      
		}
	    }
	  else if (m_FastFormulationMode==QLDANDLQ)
	    {
	      // In this case, Pu is *NOT* triangular.
	      for(unsigned k=0;k<N;k++)
		{
		  // X axis
		  Pu[IndexConstraint+k*(NbOfConstraints+1)] = 
		    (*LCI_it)->A(j,0)*m_Pu[k*N+i];
		  // Y axis
		  Pu[IndexConstraint+(k+N)*(NbOfConstraints+1)] = 
		    (*LCI_it)->A(j,1)*m_Pu[k*N+i];	      
		}
	    }
	  ODEBUG("IC: " << IndexConstraint );
	  IndexConstraint++;
	}

    }

  
  ODEBUG6("Index Constraint :"<< IndexConstraint,Buffer);

  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"Pu_%f.dat", StartingTime);
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<IndexConstraint;i++)
	{
	  for(unsigned int j=0;j<2*N;j++)
	    aof << Pu[i+j*(NbOfConstraints+1)] << " " ;
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"PuCst_%f.dat",StartingTime);
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<m_QP_N;i++)
	{
	  for(unsigned int j=0;j<m_QP_N;j++)
	    aof << m_Pu[j+i*m_QP_N] << " " ;
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"D_%f.dat",StartingTime);
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<2*m_QP_N;i++)
	{
	  for(unsigned int j=0;j<NbOfConstraints;j++)
	    aof << lD(i,j) << " " ;
	  aof << endl;
	}
      aof.close();

      if (0)
	{
	  sprintf(Buffer,"PX_%f.dat", StartingTime);
	  aof.open(Buffer,ofstream::out);
	  for(unsigned int i=0;i<IndexConstraint;i++)
	    {
	      aof << Px[i] << endl ;
	    }
	  aof.close();
	}
    }

  return 0;
}

int ZMPConstrainedQPFastFormulation::DumpProblem(double * Q,
						 double * D, 
						 double * Pu,
						 unsigned int NbOfConstraints,
						 double * Px,
						 double * XL,
						 double * XU,
						 double Time)
{
  ofstream aof;

  char Buffer[1024];
  sprintf(Buffer,"Problem_%f.dat",Time);
  aof.open(Buffer,ofstream::out);

  // Dumping Q.
  aof << "Q:"<< endl;
  for(unsigned int i=0;i<2*m_QP_N;i++)
    {
      for(unsigned int j=0;j<2*m_QP_N;j++)
	{
	  aof <<Q[j*m_QP_N*2+i]<< " ";
	}
      aof <<endl;
    }

  // Dumping D.
  aof << "D:"<< endl;
  for(unsigned int i=0;i<2*m_QP_N;i++)
    {
      aof <<D[i]<< " ";
    }
  aof <<endl;

  // Dumping Pu.
  aof << "Pu:"<< endl;
  for(unsigned int i=0;i<NbOfConstraints;i++)
    {
      for(unsigned int j=0;j<2*m_QP_N;j++)
	{
	  aof << Pu[j*(NbOfConstraints+1)+i] << " ";
	}
      aof <<endl;
    }

  // Dumping Px.
  aof << "Px:"<< endl;
  for(unsigned int i=0;i<NbOfConstraints;i++)
    {
      aof << Px[i] << " ";
    }
  aof << endl;
  
  // Dumping XL.
  aof << "XL:"<< endl;
  for(unsigned int i=0;i<2*m_QP_N;i++)
    {
      aof << XL[i] << " ";
    }
  aof << endl;
  
  // Dumping XU.
  aof << "XU:"<< endl;
  for(unsigned int i=0;i<2*m_QP_N;i++)
    {
      aof << XU[i] << " ";
    }
  aof << endl;
      
  aof.close();
  return 0;
}

int ZMPConstrainedQPFastFormulation::BuildZMPTrajectoryFromFootTrajectory(deque<FootAbsolutePosition> 
									  &LeftFootAbsolutePositions,
									  deque<FootAbsolutePosition> 
									  &RightFootAbsolutePositions,
									  deque<ZMPPosition> &ZMPRefPositions,
									  deque<COMPosition> &COMPositions,
									  double ConstraintOnX,
									  double ConstraintOnY,
									  double T,
									  unsigned int N)
{

  double *Px=0,*Pu=0;
  unsigned int NbOfConstraints=8*N; // Nb of constraints to be taken into account
  // for each iteration

  MAL_MATRIX(vnlPx,double); MAL_MATRIX(vnlPu,double);
  MAL_MATRIX(vnlValConstraint,double);
  MAL_MATRIX(vnlX,double);MAL_MATRIX(vnlStorePx,double);
  MAL_MATRIX(vnlStoreX,double);
  MAL_VECTOR(ConstraintNb,int);
  MAL_VECTOR(ZMPRef,double);
  MAL_VECTOR_DIM(OptD,double,2*N);

  int CriteriaToMaximize=1;


  RESETDEBUG4("DebugInterpol.dat");
  MAL_VECTOR_RESIZE(ZMPRef,2*N);
  
  MAL_MATRIX_RESIZE(vnlX,2*N,1);

  int m = NbOfConstraints;
  int me= 0;
  int mmax = NbOfConstraints+1;
  int n = 2*N;
  int nmax = 2*N; // Size of the matrix to compute the cost function.
  int mnn = m+n+n;


  double *D=new double[2*N];   // Constant part of the objective function
  double *XL=new double[2*N];  // Lower bound of the jerk.
  double *XU=new double[2*N];  // Upper bound of the jerk.
  double *X=new double[2*N];   // Solution of the system.
  double *NewX=new double[2*N];   // Solution of the system.
  double Eps=1e-8 ;
  double *U = (double *)malloc( sizeof(double)*mnn); // Returns the Lagrange multipliers.;

  
  int iout=0;
  int ifail;
  int iprint=1;
  int lwar=3*nmax*nmax/2+ 10*nmax  + 2*mmax + 20000;;
  double *war= (double *)malloc(sizeof(double)*lwar);
  int liwar = n; //
  int *iwar = new int[liwar]; // The Cholesky decomposition is done internally.


  deque<LinearConstraintInequality_t *> QueueOfLConstraintInequalities;
  
  if (m_FullDebug>0)
    {
      RESETDEBUG4("DebugPBW.dat");
      RESETDEBUG4("DebugPBW_Pb.dat");

      ODEBUG6("A:" << m_A << endl << "B:" << m_B, "DebugPBW_Pb.dat");

    }
      
  // Build a set of linear constraint inequalities.
  m_FCALS->BuildLinearConstraintInequalities(LeftFootAbsolutePositions,
					     RightFootAbsolutePositions,
					     QueueOfLConstraintInequalities,
					     ConstraintOnX,
					     ConstraintOnY);
  
  deque<LinearConstraintInequality_t *>::iterator LCI_it;
  LCI_it = QueueOfLConstraintInequalities.begin();
  while(LCI_it!=QueueOfLConstraintInequalities.end())
    {
      //      cout << *LCI_it << endl; 
      //      cout << (*LCI_it)->StartingTime << " " << (*LCI_it)->EndingTime << endl;
      LCI_it++;
    }
  
  double lSizeMat = QueueOfLConstraintInequalities.back()->EndingTime/T;
  MAL_MATRIX_RESIZE(vnlStorePx,
		    NbOfConstraints,
		    //6*N,
		    1+(unsigned int)lSizeMat);
  
  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(vnlStorePx);i++)
    {
      for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(vnlStorePx);j++)
	{
	  vnlStorePx(i,j) =0.0;
	}
    }
  MAL_MATRIX_RESIZE(vnlStoreX,
		    2*N,1+(unsigned int)lSizeMat);

  for(unsigned int i=0;i<2*N;i++)
    vnlStoreX(i,0) = 0.0;
  
  MAL_VECTOR_RESIZE(ConstraintNb,
		    1+(unsigned int)lSizeMat);

  // pre computes the matrices needed for the optimization.
  
  double TotalAmountOfCPUTime=0.0,CurrentCPUTime=0.0;
  struct timeval start,end;
  int li=0; 
  double dinterval = T /  m_SamplingPeriod;
  int interval=(int)dinterval;

  MAL_VECTOR_DIM(xk,double,6);

  ODEBUG("0.0 " << QueueOfLConstraintInequalities.back()->EndingTime-	N*T << " " 
	  << " T: " << T << " N: " << N << " interval " << interval);
  for(double StartingTime=0.0;
      StartingTime<QueueOfLConstraintInequalities.back()->EndingTime-
	N*T;
      StartingTime+=T,li++)
    {
      gettimeofday(&start,0);
      
      // Read the current state of the 2D Linearized Inverted Pendulum.
      m_2DLIPM->GetState(xk);

      ODEBUG4(xk[0] << " " << xk[3] << " " <<
	      xk[1] << " " << xk[4] << " " <<
	      xk[2] << " " << xk[5] << " ", "Check2DLIPM.dat");
	      
      // Build the related matrices.
      BuildConstraintMatrices(Px,Pu,
			      N,T,
			      StartingTime,
			      QueueOfLConstraintInequalities,
			      m_ComHeight,
			      NbOfConstraints,
			      xk);
      

      m = NbOfConstraints;
      
      mmax = NbOfConstraints+1;
      lwar = 3*nmax*nmax/2+ 10*nmax  + 2*mmax + 20000;
      mnn = m+n+n;

      // Call to QLD (a linearly constrained quadratic problem solver)

      // Prepare D.
      for(unsigned int i=0;i<N;i++)
	{
	  ZMPRef[i] = ZMPRefPositions[li*interval+i*interval].px;
	  ZMPRef[i+N] = ZMPRefPositions[li*interval+i*interval].py;
	}
      
      if (m_FullDebug>2)
	{
	  ofstream aof;
	  char Buffer[1024];
	  sprintf(Buffer,"ZMPRef_%f.dat",StartingTime);
	  aof.open(Buffer,ofstream::out);
	  for(unsigned int i=0;i<2*N;i++)
	    {
	      aof << ZMPRef[i] << endl;
	    }
	  aof.close(); 
	}  

      if (CriteriaToMaximize==1)
	{
	  MAL_VECTOR(lterm1v,double);
	  MAL_C_eq_A_by_B(lterm1v,m_OptC,ZMPRef);
	  MAL_VECTOR_RESIZE(OptD,2*N);
	  MAL_C_eq_A_by_B(OptD,m_OptB,xk);
	  OptD -= lterm1v;
	  for(unsigned int i=0;i<2*N;i++)
	    D[i] = OptD[i];

	  if (m_FullDebug>0)
	    {
	      ofstream aof;
	      char Buffer[1024];
	      sprintf(Buffer,"D_%f.dat",StartingTime);
	      aof.open(Buffer,ofstream::out);
	      for(unsigned int i=0;i<2*N;i++)
		{
		  aof << OptD[i] << endl;
		}
	      aof.close(); 
	    }

	}
      else
	{
	  // Default : set D to zero.
	  for(unsigned int i=0;i<2*N;i++)
	    D[i] = 0.0;
	}

      for(unsigned int i=0;i<2*N;i++)
	{
	  XL[i] = -1e8;
	  XU[i] = 1e8;
	}
      memset(X,0,2*N*sizeof(double));

      // Verification
      ConstraintNb[li] = m;
      MAL_MATRIX_RESIZE(vnlPu,m,2*N);
      MAL_MATRIX_RESIZE(vnlPx,m,1);
      
  
      for(int i=0; i<m;i++)
	{
	  vnlPx(i,0) =
	    vnlStorePx(i,li) = Px[i];
	}

      if (m_FastFormulationMode==QLDANDLQ)
	iwar[0]=0;
      else
	iwar[0]=1;

      ODEBUG("m: " << m);
      //      DumpProblem(m_Q, D, Pu, m, Px,XL,XU,StartingTime);
		  
		  
      ql0001_(&m, &me, &mmax,&n, &nmax,&mnn,
	      m_Q, D, Pu,Px,XL,XU,
	      X,U,&iout, &ifail, &iprint,
	      war, &lwar,
	      iwar, &liwar,&Eps);

      if (ifail!=0)
	{
	  cout << "IFAIL: " << ifail << endl;
	  return -1;
	}

      /* Constraint validation */

      for(int i=0; i<m;i++)
	for(unsigned int j=0; j<2*N;j++)
	  vnlPu(i,j) = Pu[j*(m+1)+i];

      for(unsigned int i=0; i<2*N;i++)
	{
	  vnlStoreX(i,li) = X[i];
	  vnlX(i,0) = X[i];
	}

      vnlValConstraint = MAL_RET_A_by_B(vnlPu, vnlX)  + vnlPx;
      
      if (m_FullDebug>2)
      {
	ofstream aof;
	char Buffer[1024];
	sprintf(Buffer,"X_%f.dat",StartingTime);
	aof.open(Buffer,ofstream::out);
	for(unsigned int i=0;i<2*N;i++)
	  {
	    aof << X[i] << endl;
	  }
	aof.close(); 
      }

      if (MAL_MATRIX_NB_COLS(vnlValConstraint)!=1)
	{
	  cout << "Problem during validation of the constraints matrix: " << endl;
	  cout << "   size for the columns different from 1" << endl;
	  return -1;
	}


      for(int i=0;i<m;i++)
	{
	  unsigned int pbOnCurrent=0;
	  if (vnlValConstraint(i,0)<-1e-8)
	    {
	      ODEBUG3("Problem during validation of the constraint: ");
	      ODEBUG3("  constraint " << i << " is not positive");
	      ODEBUG3(vnlValConstraint(i,0));
	      pbOnCurrent = 1;
	    }

	  if (pbOnCurrent)
	    {
	      ODEBUG3("PbonCurrent: " << pbOnCurrent << " " << li
		      << " Contrainte " << i 
		      << " StartingTime :" << StartingTime);
	      if (pbOnCurrent)
		{
		  return -1;
		}
	    }
	    
	}

      double *ptX=0;
      if (m_FastFormulationMode==QLDANDLQ)
	{
	  /* Multiply the solution by the transpose of iLQ 
	     because it is a triangular matrix we do a specific 
	     multiplication.
	  */
	  memset(NewX,0,2*N*sizeof(double));
	  
	  double *pm_iLQ = MAL_RET_MATRIX_DATABLOCK(m_iLQ);
	  double *pNewX = NewX;
	  
	  for(unsigned int i=0;i<2*N;i++)
	    {
	      double *pX= X+i;
	      double *piLQ = pm_iLQ+i*2*N+i;
	      *pNewX = 0.0;
	      for(unsigned int j=i;j<2*N;j++)
		{
		  *pNewX+= (*piLQ) * (*pX++);
		  piLQ+=2*N;
		}
	      pNewX++;
	    }
	  ptX=NewX;
	} 
      else
	ptX=X;
	  
      /* Simulation of the Single Point Mass model 
	 with the new command.
      */
      ODEBUG("X[0] " << X[0] << " X[N] :" << X[N]);
      
      // Calling this method will automatically 
      // update the ZMPRefPositions.
      m_2DLIPM->Interpolation(COMPositions,
			      ZMPRefPositions,
			      li*interval,
			      ptX[0],ptX[N]);
      
      m_2DLIPM->OneIteration(ptX[0],ptX[N]);

      ODEBUG6("uk:" << uk,"DebugPBW.dat");
      ODEBUG6("xk:" << xk,"DebugPBW.dat");

      // Compute CPU consumption time.
      gettimeofday(&end,0);
      CurrentCPUTime = end.tv_sec - start.tv_sec + 
	0.000001 * (end.tv_usec - start.tv_usec);
      TotalAmountOfCPUTime += CurrentCPUTime;
      ODEBUG("Current Time : " << StartingTime << " " << 
	     " Virtual time to simulate: " << QueueOfLConstraintInequalities.back()->EndingTime - StartingTime << 
	     "Computation Time " << CurrentCPUTime << " " << TotalAmountOfCPUTime);

    }
  ODEBUG("NewZMPsize: " << NewFinalZMPPositions.size());
  if (m_FullDebug>2)
    {
      ofstream aof;
      aof.open("StorePx.dat",ofstream::out);
      
      for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(vnlStorePx);i++)
	{
	  for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(vnlStorePx);j++)
	    {
	      aof << vnlStorePx(i,j) << " ";
	    }
	  aof << endl;
	}
      aof.close();
      
      
      char lBuffer[1024];
      sprintf(lBuffer,"StoreX.dat");
      aof.open(lBuffer,ofstream::out);
      
      for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(vnlStoreX);i++)
	{
	  for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(vnlStoreX);j++)
	    {
	      aof << vnlStoreX(i,j) << " ";
	    }
	  aof << endl;
	}
      aof.close();
      
      aof.open("Cnb.dat",ofstream::out);
      for(unsigned int i=0;i<MAL_VECTOR_SIZE(ConstraintNb);i++)
	{
	  aof << ConstraintNb[i]<<endl;
	}
      aof.close();
    }
  
  /*  cout << "Size of PX: " << MAL_MATRIX_NB_ROWS(vnlStorePx) << " " 
      << MAL_MATRIX_NB_COLS(vnlStorePx) << " " << endl; */
  delete [] D;
  delete [] XL;
  delete [] XU;
  delete [] X;
  free(war);
  free(U);
  delete [] iwar;
  // Clean the queue of Linear Constraint Inequalities.
  //  deque<LinearConstraintInequality_t *>::iterator LCI_it;
  LCI_it = QueueOfLConstraintInequalities.begin();
  while(LCI_it!=QueueOfLConstraintInequalities.end())
    {
      //      cout << *LCI_it << endl; 
      //      cout << (*LCI_it)->StartingTime << " " << (*LCI_it)->EndingTime << endl;
      delete *(LCI_it);
      LCI_it++;
    }
  QueueOfLConstraintInequalities.clear();
  
  return 0;
}


void ZMPConstrainedQPFastFormulation::GetZMPDiscretization(deque<ZMPPosition> & ZMPPositions,
							   deque<COMPosition> & COMPositions,
							   deque<RelativeFootPosition> &RelativeFootPositions,
							   deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
							   deque<FootAbsolutePosition> &RightFootAbsolutePositions,
							   double Xmax,
							   COMPosition & lStartingCOMPosition,
							   MAL_S3_VECTOR(&,double) lStartingZMPPosition,
							   FootAbsolutePosition & InitLeftFootAbsolutePosition,
							   FootAbsolutePosition & InitRightFootAbsolutePosition)
{
  if (m_ZMPD==0)
    return;
  
  m_ZMPD->GetZMPDiscretization(ZMPPositions,
			       COMPositions,
			       RelativeFootPositions,
			       LeftFootAbsolutePositions,
			       RightFootAbsolutePositions,
			       Xmax,
			       lStartingCOMPosition,
			       lStartingZMPPosition,
			       InitLeftFootAbsolutePosition,
			       InitRightFootAbsolutePosition);

  ODEBUG3("Dimitrov algo set on");

  BuildZMPTrajectoryFromFootTrajectory(LeftFootAbsolutePositions,
				       RightFootAbsolutePositions,
				       ZMPPositions,
				       COMPositions,
				       m_ConstraintOnX,
				       m_ConstraintOnY,
				       m_QP_T,
				       m_QP_N);
  if (m_FullDebug>0)
    {
      ofstream aof;
      aof.open("DebugDimitrovZMP.dat",ofstream::out);
      for(unsigned int i=0;i<ZMPPositions.size();i++)
	{
	  aof << ZMPPositions[i].px << " " << ZMPPositions[i].py << endl;
	}
      aof.close();
      
    }
  
}

void ZMPConstrainedQPFastFormulation::CallMethod(std::string & Method, std::istringstream &strm)
{
  if (Method==":setdimitrovconstraint")
    {
      string PBWCmd;
      strm >> PBWCmd;
      if (PBWCmd=="XY")
	{
	  strm >> m_ConstraintOnX;
	  strm >> m_ConstraintOnY;
	  cout << "Constraint On X: " << m_ConstraintOnX
	       << " Constraint On Y: " << m_ConstraintOnY << endl;
	}
      else if (PBWCmd=="T")
	{
	  strm >> m_QP_T;
	  cout << "Sampling for the QP " << m_QP_T <<endl;
	}
      else if (PBWCmd=="N")
	{
	  strm >> m_QP_N;
	  cout << "Preview window for the QP " << m_QP_N << endl;
	}
    }

}


int ZMPConstrainedQPFastFormulation::InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
				    deque<COMPosition> & FinalCOMPositions,
				    deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
				    deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
				    FootAbsolutePosition & InitLeftFootAbsolutePosition,
				    FootAbsolutePosition & InitRightFootAbsolutePosition,
				    deque<RelativeFootPosition> &RelativeFootPositions,
				    COMPosition & lStartingCOMPosition,
				    MAL_S3_VECTOR(&,double) lStartingZMPPosition)
{
  cout << "To be implemented" << endl;
  return 0;
}

void ZMPConstrainedQPFastFormulation::OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
					deque<ZMPPosition> & FinalZMPPositions,	
					deque<COMPosition> & FinalCOMPositions,
					deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
					deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
					bool EndSequence)
{
  cout << "To be implemented" << endl;
}

void ZMPConstrainedQPFastFormulation::OnLine(double time,
				 deque<ZMPPosition> & FinalZMPPositions,				     
				 deque<COMPosition> & FinalCOMPositions,
				 deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
				 deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)
{
  cout << "To be implemented" << endl;
}

int ZMPConstrainedQPFastFormulation::OnLineFootChange(double time,
					  FootAbsolutePosition &aFootAbsolutePosition,
					  deque<ZMPPosition> & FinalZMPPositions,			     
					  deque<COMPosition> & CoMPositions,
					  deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
					  deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
					  StepStackHandler  *aStepStackHandler)
{
  cout << "To be implemented" << endl;
  return -1;
}

void ZMPConstrainedQPFastFormulation::EndPhaseOfTheWalking(deque<ZMPPosition> &ZMPPositions,
					       deque<COMPosition> &FinalCOMPositions,
					       deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					       deque<FootAbsolutePosition> &RightFootAbsolutePositions)
{
  
}

int ZMPConstrainedQPFastFormulation::ReturnOptimalTimeToRegenerateAStep()
{
  int r = (int)(m_PreviewControlTime/m_SamplingPeriod);
  return 2*r;
}

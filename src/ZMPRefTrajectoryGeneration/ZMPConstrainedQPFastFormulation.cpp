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
#define ODEBUG(x)  std::cout << "ZMPQPWithConstraint: " << x << endl;
#endif

#define ODEBUG3(x)  std::cout << "ZMPQPWithConstraint: " << x << endl;

using namespace std;
using namespace PatternGeneratorJRL;

ZMPConstrainedQPFastFormulation::ZMPConstrainedQPFastFormulation(SimplePluginManager *lSPM, 
								 string DataFile,
								 HumanoidSpecificities *aHS) :
  ZMPRefTrajectoryGeneration(lSPM)
{
  m_Q = 0;

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

  m_Alpha = 200.0;
  m_Beta = 1000.0;
  
  InitConstants();

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

  if (0)
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

int ZMPConstrainedQPFastFormulation::BuildingConstantPartOfTheObjectiveFunction()
{
  int CriteriaToMaximize=1;

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
  
  m_Q=new double[4*m_QP_N*m_QP_N]; 

  // Initialization of the matrice regarding the quadratic
  // part of the objective function.
  memset(m_Q,0,4*m_QP_N*m_QP_N*sizeof(double));
  for(unsigned int i=0;i<2*m_QP_N;i++)
    m_Q[i*2*m_QP_N+i] = 1.0;

  if (CriteriaToMaximize==1)
    {
      for(unsigned int i=0;i<2*m_QP_N;i++)
	for(unsigned int j=0;j<2*m_QP_N;j++)
	  m_Q[j*2*m_QP_N+i] = OptA(i,j);


      if (0)
	{
	  ofstream aof;
	  char Buffer[1024];
	  sprintf(Buffer,"C.dat");
	  aof.open(Buffer,ofstream::out);
	  for(unsigned int i=0;i<2*m_QP_N;i++)
	    {
	      for(unsigned int j=0;j<2*m_QP_N-1;j++)
		aof << OptA(i,j) << " ";
	      aof << OptA(i,2*m_QP_N-1);
	      aof << endl;
	    }
	  aof.close(); 
	  
	}  

    }

  /*! Compute constants of the linear part of the objective function. */
  lterm1 = MAL_RET_TRANSPOSE(m_PPu);
  lterm1 = MAL_RET_A_by_B(lterm1,m_PPx);
  m_OptB = MAL_RET_TRANSPOSE(m_VPu);
  m_OptB = MAL_RET_A_by_B(m_OptB,m_VPx);
  m_OptB = m_Alpha * m_OptB;
  m_OptB = m_OptB + m_Beta * lterm1;

  if (0)
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
  
  m_OptC = MAL_RET_TRANSPOSE(m_PPu);
  m_OptC = m_Beta * m_OptC;
  if (0)
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


int ZMPConstrainedQPFastFormulation::InitConstants()
{
  int r;
  if ((r=InitializeMatrixPbConstants())<0)
    return r;

  if ((r=BuildingConstantPartOfTheObjectiveFunction())<0)
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


int ZMPConstrainedQPFastFormulation::BuildMatricesPxPu(double * & Px,double * &Pu, 
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
  //memset(Px,0,(8*N+1)*sizeof(double));

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
      
  if (0)
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

      if (LCI_it==QueueOfLConstraintInequalities.end())
	{
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
	  for(unsigned k=0;k<=i;k++)
	    {
	      // X axis
	      Pu[IndexConstraint+k*(NbOfConstraints+1)] = 
		(*LCI_it)->A(j,0)*
		((1+3*(i-k)+3*(i-k)*(i-k))*T*T*T/6.0 - T * Com_Height/9.81);
	      
	      // Y axis
	      Pu[IndexConstraint+(k+N)*(NbOfConstraints+1)] = 
		(*LCI_it)->A(j,1)*
		((1+3*(i-k)+3*(i-k)*(i-k))*T*T*T/6.0 - T * Com_Height/9.81);
		 
	      
	    }
	  ODEBUG("IC: " << IndexConstraint );
	  IndexConstraint++;
	}

    }
  ODEBUG6("Index Constraint :"<< IndexConstraint,Buffer);
  if (0)
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
      
      sprintf(Buffer,"PX_%f.dat", StartingTime);
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0;i<IndexConstraint;i++)
	{
	  aof << Px[i] << endl ;
	}
      aof.close();
    }

  return 0;
}

int ZMPConstrainedQPFastFormulation::BuildZMPTrajectoryFromFootTrajectory(deque<FootAbsolutePosition> 
									  &LeftFootAbsolutePositions,
									  deque<FootAbsolutePosition> 
									  &RightFootAbsolutePositions,
									  deque<ZMPPosition> &ZMPRefPositions,
									  deque<ZMPPosition> &NewFinalZMPPositions,
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
  
  if (1)
    {
      RESETDEBUG4("DebugPBW.dat");
      RESETDEBUG4("DebugPBW_Pb.dat");

      ODEBUG6("A:" << m_A << endl << "B:" << m_B, "DebugPBW_Pb.dat");
  

      RESETDEBUG5("Constraints.dat");

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

  MAL_VECTOR(,double) xk;

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

      // Build the related matrices.
      BuildMatricesPxPu(Px,Pu,
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
      
      if (0)
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

	  if (0)
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

      iwar[0]=1;
      ODEBUG("m: " << m);
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


      for(int i=0; i<m;i++)
	for(unsigned int j=0; j<2*N;j++)
	  vnlPu(i,j) = Pu[j*(m+1)+i];

      for(unsigned int i=0; i<2*N;i++)
	{
	  vnlStoreX(i,li) = X[i];
	  vnlX(i,0) = X[i];
	}

      vnlValConstraint = MAL_RET_A_by_B(vnlPu, vnlX)  + vnlPx;
      
      if (0)
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
      ODEBUG("X[0] " << X[0] << " X[N] :" << X[N]);
      
      // Calling this method will automatically 
      // update the NewFinalZMPPositions.
      m_2DLIPM->Interpolation(NewFinalZMPPositions,
			      COMPositions,
			      ZMPRefPositions,
			      li*interval,
			      X[0],X[N]);

      m_2DLIPM->OneIteration(X[0],X[N]);
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
  // Current heuristic to complete the ZMP buffer:
  // fill with the last correct value.
  ZMPPosition LastZMPPos = NewFinalZMPPositions.back();

  ODEBUG("N*T/m_SamplingPeriod :" << N*T/m_SamplingPeriod << " " << (int)N*T/m_SamplingPeriod);
  double lLimit = N*T/m_SamplingPeriod;
  int Limit = (int)lLimit;
  
  ODEBUG("Limit: " << Limit);

  // This test is here to compensate for the discrency 
  // between ZMPRef Position and the new one.
  // As the parameters N and T can be different you may have one
  // or two iterations of difference. They are compensate for here.
  // It is assumed that NewFinalZMPPositions.size() < ZMPRefPositions.size()
  if ((Limit + NewFinalZMPPositions.size()) != ZMPRefPositions.size())
    Limit = ZMPRefPositions.size() - NewFinalZMPPositions.size();

  ODEBUG("Limit: " << Limit);
  for (int i=0;i< Limit;i++)
  {
    ZMPPosition aZMPPos;
    aZMPPos.px = LastZMPPos.px;
    aZMPPos.py = LastZMPPos.py;
    aZMPPos.theta = LastZMPPos.theta;
    aZMPPos.stepType = LastZMPPos.stepType;
    NewFinalZMPPositions.push_back(aZMPPos);
  }
  ODEBUG("NewZMPsize: " << NewFinalZMPPositions.size());
  if (0)
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
  delete D;
  delete XL;
  delete XU;
  delete X;
  free(war);
  free(U);
  delete iwar;
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

  deque<ZMPPosition> NewZMPPositions;
  ODEBUG3("Dimitrov algo set on");
  ODEBUG3("Size of COMBuffer: " << m_COMBuffer.size());
  m_COMBuffer.clear();

  BuildZMPTrajectoryFromFootTrajectory(LeftFootAbsolutePositions,
				       RightFootAbsolutePositions,
				       ZMPPositions,
				       NewZMPPositions,
				       m_COMBuffer,
				       m_ConstraintOnX,
				       m_ConstraintOnY,
				       m_QP_T,
				       m_QP_N);
  if (ZMPPositions.size()!=NewZMPPositions.size())
    {
      cout << "Problem here between m_ZMPPositions and new zmp positions" << endl;
      cout << ZMPPositions.size() << " " << NewZMPPositions.size() << endl;
    }
  
  for(unsigned int i=0;i<ZMPPositions.size();i++)
    ZMPPositions[i] = NewZMPPositions[i];

  if (1)
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

void ZMPConstrainedQPFastFormulation::GetComBuffer(deque<COMPosition> &aCOMBuffer)
{
  for(unsigned int i=0;i<m_COMBuffer.size();i++)
    {
      aCOMBuffer.push_back(m_COMBuffer[i]);
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

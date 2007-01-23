/** @doc Object to compute the weights of the preview control
    @ingroup previewcontrol
    @endgroup

   Copyright (c) 2005-2006, 
   @author Olivier Stasse
   
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

#include <iostream>
#include <OptimalControllerSolver.h>
#ifdef _VNL_MATRIX_
#include <f2c.h>
#endif

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "OptimalControllerServer :" << x << endl
#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#if 0
#define ODEBUG(x) cerr << "OptimalControllerServer :" <<  x << endl
#else
#define ODEBUG(x) 
#endif

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1 
#else
#define RESETDEBUG4(y) 
#define ODEBUG4(x,y)
#endif

#define ODEBUG6(x,y)

using namespace PatternGeneratorJRL;
using namespace std;

  logical sb02ox (double *alphar, double * alphai, double *beta)
  {
    logical r;
    double w =dlapy2_(alphar,alphai);
    double w2 = fabs(*beta);
    r = w < w2;
    return r;
  }

/* 
   Inspired from scilab 4.0...


   To select the stable generalized eigenvalues for continuous-time.

   alphar: double precision,
   the real part of the numerator of the current eigenvalue considered.
   
   alphai: double precision,
   The imaginary part of the numerator of the current
   eigenvalue considered.
   
   beta: double precision
   The (real) denominator of the current eigenvalue
   considered. It is assumed that beta <> 0 (regular case).

   description:

   The function value SB02OW is set to true for a stable eigenvalue
     and to false, otherwise.
*/
logical sb02ow (double *alphar, double * alphai, double *beta)
{
  logical r = ((*alphar <0.0) && (*beta>0.0)) ||
    ( (*alphar>0.0) && (*beta<0.0)) &&
      ( fabs(*beta) > fabs(*alphar)*dlamch_("p"));
      ;
    return r;
  }


OptimalControllerSolver::OptimalControllerSolver(MAL_MATRIX( &A, double), 
						 MAL_MATRIX( &b, double),
						 MAL_MATRIX( &c, double),
						 double Q, double R,
						 unsigned int Nl)
{
  m_A = A;
  m_b = b;
  m_c = c;
  
  m_Q = Q;
  m_R = R;
  m_Nl = Nl;
}

OptimalControllerSolver::~OptimalControllerSolver()
{
  
}

bool OptimalControllerSolver::GeneralizedSchur(MAL_MATRIX( &A,double),
					       MAL_MATRIX( &B,double),
					       MAL_VECTOR( &alphar,double),
					       MAL_VECTOR( &alphai,double),
					       MAL_VECTOR( &beta,double),
					       MAL_MATRIX( &L,double),
					       MAL_MATRIX( &R,double))
{
  int n = MAL_MATRIX_NB_ROWS(A);
  MAL_VECTOR_RESIZE(alphar,n); MAL_VECTOR_FILL(alphar,0);
  MAL_VECTOR_RESIZE(alphai,n); MAL_VECTOR_FILL(alphai,0);
  MAL_VECTOR_RESIZE(beta,n); MAL_VECTOR_FILL(beta,0);
  MAL_MATRIX_RESIZE(L,n, n); MAL_MATRIX_FILL(L,0);
  MAL_MATRIX_RESIZE(R,n, n); MAL_MATRIX_FILL(R,0);
		    
  int sdim = 0;
  int lwork = 1000+ (8*n + 16);
  double *work = new double[lwork]; //std::vector<double> work(lwork);
  int info = 0;
  logical *bwork=new logical[2*n];
  for(int i=0;i<2*n;i++)
    bwork[i] =0;
  A = MAL_RET_TRANSPOSE(A);
  B = MAL_RET_TRANSPOSE(B);
  dgges_ ("V", "V",
          "S",
         (logical (*)(...))sb02ox,
          &n,
          MAL_RET_MATRIX_DATABLOCK(A), &n,
          MAL_RET_MATRIX_DATABLOCK(B), &n,
          &sdim,
          MAL_RET_VECTOR_DATABLOCK(alphar),
          MAL_RET_VECTOR_DATABLOCK(alphai),
          MAL_RET_VECTOR_DATABLOCK(beta),
          MAL_RET_MATRIX_DATABLOCK(L), &n,
          MAL_RET_MATRIX_DATABLOCK(R), &n,
          &work[0], &lwork,
          bwork,
          &info);

  A = MAL_RET_TRANSPOSE(A);
  B = MAL_RET_TRANSPOSE(B);
  L = MAL_RET_TRANSPOSE(L);
  R = MAL_RET_TRANSPOSE(R);
  
  delete [] work;
  delete [] bwork;
  if (info != 0) {
    std::cout << ": info = " << info << std::endl;
    return false;
  }
  else
    return true;

}

void OptimalControllerSolver::ComputeWeights()
{
  // Compute the symplectic matrix related 
  // to the discrete dynamical system given in parameters.

  // H is the 2n x 2n matrix
  // H = [[ A , 0 ]
  //      [-c'Qc E]]
  
  // And each sub-matrix is n x n matrix.
  MAL_MATRIX(H,double);
  MAL_MATRIX(tm_b,double);

  // Store the transpose of m_b;
  tm_b = MAL_RET_TRANSPOSE(m_b);

  MAL_MATRIX_RESIZE(H,
		    2*MAL_MATRIX_NB_ROWS(m_A),
		    2*MAL_MATRIX_NB_COLS(m_A));
  
  MAL_MATRIX_SET_IDENTITY(H);
  
  // Build the upper left sub-block of H
  int n = MAL_MATRIX_NB_ROWS(m_A);
  for(int i=0;i< n;i++)
    for(int j=0;j<n;j++)
      H(i,j) = m_A(i,j);

  MAL_MATRIX(H21,double);
  H21 = MAL_RET_TRANSPOSE(m_c);
  H21 = H21 * m_Q;
  H21 = MAL_RET_A_by_B(H21, m_c);
  H21 = -H21;

  for(int i=0;i< n;i++)
    for(int j=0;j<n;j++)
      H(i+n,j) = H21(i,j);

  ODEBUG("H:" << endl << H);
  MAL_MATRIX_DIM(E,double,2*n,2*n);
  MAL_MATRIX_SET_IDENTITY(E);

  MAL_MATRIX(G,double);
  G = MAL_RET_A_by_B(m_b * (1/m_R) , tm_b);

  MAL_MATRIX(At,double);
  At= MAL_RET_TRANSPOSE(m_A);
  for(int i=0;i< n;i++)
    for(int j=0;j<n;j++)
      {
	E(i,j+n) = G(i,j);
	E(i+n,j+n) = At(i,j);
      }

  ODEBUG("E:" << endl << E);
  // Computes S the Schur form of Laub_Z
  MAL_MATRIX_DIM( ZH,double,2*n,2*n); // The matrix of schur vectors
  MAL_MATRIX_DIM( ZE,double,2*n,2*n); // The matrix of Schur vectors.
  MAL_VECTOR_DIM( WR,double,2*n);
  MAL_VECTOR_DIM( WI,double,2*n); // The eigenvalues ( a matrix to handle complex eigenvalues).
  MAL_VECTOR_DIM( GS,double,2*n);

  GeneralizedSchur(H,E,WR,WI,GS,ZH,ZE);

  ODEBUG("Hx:"<<H);
  ODEBUG("Ex:"<<E);
  ODEBUG("ZH:"<<ZH);
  ODEBUG("ZE:"<<ZE);


  // Computes P the solution of the Riccati equation.
  MAL_MATRIX(P,double);
  MAL_MATRIX_DIM(Z11,double,n,n);
  MAL_MATRIX_DIM(Z21,double,n,n);


  for(int i=0;i< n;i++)
    {
      for(int j=0;j<n;j++)
	{
	  Z11(i,j) = ZE(i,j);
	  Z21(i,j) = ZE(i+n,j);
	}
    }
  ODEBUG( "Z11:" << endl << Z11 << endl
	   << "Z21:" << endl << Z21 );
  
  MAL_MATRIX(iZ11,double);
  MAL_INVERSE(Z11,iZ11,double);
  P = MAL_RET_A_by_B(Z21, iZ11);
  
  ODEBUG( "P: " << endl << P);

  // Compute the weights.
  MAL_MATRIX(r,double);

  double la;
  r = tm_b;
  r = MAL_RET_A_by_B(r,P);
  r = MAL_RET_A_by_B(r,m_b);
  la = m_R + r(0,0);
  la = 1/la;
  
  // Computes the weights for the accumulated difference.
  // and the speed.
  m_K = MAL_RET_A_by_B(P,m_A);
  m_K = MAL_RET_A_by_B(tm_b,m_K);
  m_K = m_K * la;
  
  ODEBUG("K: "<< endl << m_K);
  
  // Computes the weights for the future.
  MAL_MATRIX(PreMatrix,double);
  MAL_MATRIX(Recursive,double);
  MAL_MATRIX(BaseOfRecursion,double);
  MAL_MATRIX(PostMatrix,double);
  MAL_MATRIX(Intermediate,double);	

  PreMatrix = la * tm_b;
  BaseOfRecursion = m_A - MAL_RET_A_by_B(m_b , m_K);
  BaseOfRecursion = MAL_RET_TRANSPOSE(BaseOfRecursion);
  Recursive = BaseOfRecursion;
  PostMatrix = MAL_RET_TRANSPOSE(m_c);
  PostMatrix = PostMatrix * m_Q;
  PostMatrix = MAL_RET_A_by_B(P ,PostMatrix);

  ODEBUG("BaseOfRecursion:" << endl << BaseOfRecursion << endl << 
	 "Recursive:" << endl << Recursive << endl << 
	 "PreMatrix:" << endl << PreMatrix << endl <<
	 "PostMatrix:" << endl <<PostMatrix<< endl);
  
  MAL_MATRIX_RESIZE(m_F,m_Nl,1);
  for(int k=0;k<m_Nl;k++)
    {
      Intermediate = MAL_RET_A_by_B(Recursive, PostMatrix);
      Intermediate = MAL_RET_A_by_B(PreMatrix, Intermediate);
      m_F(k,0) = Intermediate(0,0);
      Recursive = MAL_RET_A_by_B(Recursive,BaseOfRecursion);
    }
  
}

void OptimalControllerSolver::DisplayWeights()
{
  std::cout << "K:" << m_K << std::endl;
  std::cout << "F:" << m_F << std::endl;
}

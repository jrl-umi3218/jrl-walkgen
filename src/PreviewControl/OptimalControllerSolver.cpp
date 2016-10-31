/*
 * Copyright 2006, 2007, 2008, 2009, 2010, 
 *
 * Florent Lamiraux
 * Olivier Stasse
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
/** @doc Object to compute the weights of the preview control
    @ingroup previewcontrol
    @endgroup
*/

#include <iostream>
#include <PreviewControl/OptimalControllerSolver.hh>

//#define _DEBUG_MODE_ON_
#include <Debug.hh>

using namespace PatternGeneratorJRL;
using namespace std;


typedef long int logical;
typedef double doublereal;
typedef logical(* L_fp)(...);
typedef int integer ;
extern "C" {
extern doublereal dlapy2_(doublereal *, doublereal *); 
extern double dlamch_ (char *);
extern /* Subroutine */ int dgges_(char *, char *, char *, L_fp, integer *
				   , doublereal *, integer *, doublereal *, integer *, integer *, 
				   doublereal *, doublereal *, doublereal *, doublereal *, integer *,
				   doublereal *, integer *, doublereal *, integer *, logical *, 
				   integer *);

}

logical sb02ox (double *_alphar, double * _alphai, double *_beta)
{
  logical r;
  double w =dlapy2_(_alphar,_alphai);
  double w2 = fabs(*_beta);
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
logical sb02ow (double *_alphar, double * /* _alphai */, double *_beta)
{
  char lp[2]="p";
  logical r = ((*_alphar <0.0) && (*_beta>0.0)) ||
    
    (( (*_alphar>0.0) && (*_beta<0.0)) &&
      (( fabs(*_beta) > fabs(*_alphar)*dlamch_(lp))));
      ;
    return r;
  }


OptimalControllerSolver::OptimalControllerSolver(MAL_MATRIX( &A, double), 
						 MAL_MATRIX( &b, double),
						 MAL_MATRIX( &c, double),
						 double Q, double R,
						 unsigned int Nl)
{
  MAL_MATRIX_RESIZE(m_A, MAL_MATRIX_NB_ROWS(A),
		    MAL_MATRIX_NB_COLS(A));
  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(A);i++)
    for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(A);j++)
      m_A(i,j) = A(i,j);

  MAL_MATRIX_RESIZE(m_b, MAL_MATRIX_NB_ROWS(b),
		    MAL_MATRIX_NB_COLS(b));
  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(b);i++)
    for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(b);j++)
      m_b(i,j) = b(i,j);


  MAL_MATRIX_RESIZE(m_c, MAL_MATRIX_NB_ROWS(c),
		    MAL_MATRIX_NB_COLS(c));
  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(c);i++)
    for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(c);j++)
      m_c(i,j) = c(i,j);
  
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
  ODEBUG("A:" << A);
  ODEBUG("B:" << A);
  int n = MAL_MATRIX_NB_ROWS(A);
  MAL_VECTOR_RESIZE(alphar,n); MAL_VECTOR_FILL(alphar,0);
  MAL_VECTOR_RESIZE(alphai,n); MAL_VECTOR_FILL(alphai,0);
  MAL_VECTOR_RESIZE(beta,n); MAL_VECTOR_FILL(beta,0);
  MAL_MATRIX_RESIZE(L,n, n); MAL_MATRIX_FILL(L,0);
  MAL_MATRIX_RESIZE(R,n, n); MAL_MATRIX_FILL(R,0);
		    
  int sdim = 0;
  int lwork = 1000+ (8*n + 16);
  double *work = new double[lwork]; //std::vector<double> work(lwork);
  for(int i=0;i<lwork;i++)
    work[i] = 0.0;
  int info = 0;
  logical *bwork=new logical[2*n];
  for(int i=0;i<2*n;i++)
    bwork[i] = 0;

  char lV[2]="V";
  char lS[2]="S";
  for(int i=0;i<2*n;i++)
    bwork[i] =0;
  A = MAL_RET_TRANSPOSE(A);
  B = MAL_RET_TRANSPOSE(B);
  dgges_ (lV, lV,
          lS,
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
    std::cout << ": info = " << info << " n = " << n << std::endl;
    return false;
  }
  else
    {
      return true;
    }

}

void OptimalControllerSolver::ComputeWeights(unsigned int Mode)
{
  // Compute the symplectic matrix related 
  // to the discrete dynamical system given in parameters.

  // H is the 2n x 2n matrix
  // H = [[ A , 0 ]
  //      [-c'Qc E]]
  //  ODEBUG("m_A:" << m_A);
  // And each sub-matrix is n x n matrix.
  MAL_MATRIX(H,double);
  MAL_MATRIX(tm_b,double);

  // Store the transpose of m_b;
  tm_b = MAL_RET_TRANSPOSE(m_b);

  ODEBUG(" ROWS(A): " << MAL_MATRIX_NB_ROWS(m_A)
	 << " COLS(A): " << MAL_MATRIX_NB_COLS(m_A) );
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
  ODEBUG("H21 (1):" << H21);
  H21 = H21 * m_Q;
  ODEBUG("H21 (2):" << H21);
  H21 = MAL_RET_A_by_B(H21, m_c);
  ODEBUG("H21 (3):" << H21);
  H21 = -H21;
  
  ODEBUG("H21:" << H21);
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

  if (!GeneralizedSchur(H,E,WR,WI,GS,ZH,ZE))
    {
      ODEBUG3("Something is wrong with the weights for the preview control !");
    }

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
  r = tm_b;  // b^T
  r = MAL_RET_A_by_B(r,P); // b^T P
  r = MAL_RET_A_by_B(r,m_b); // b^T P b
  la = m_R + r(0,0); // R + b^T P b
  la = 1/la; 
  
  // Computes the weights for the accumulated difference
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

  PostMatrix = MAL_RET_TRANSPOSE(m_c);
  PostMatrix = PostMatrix * m_Q;
  if (Mode==MODE_WITHOUT_INITIALPOS)
    PostMatrix = MAL_RET_A_by_B(P ,PostMatrix);

  Recursive = PostMatrix;

  ODEBUG("BaseOfRecursion:" << endl << BaseOfRecursion << endl << 
	 "Recursive:" << endl << Recursive << endl << 
	 "PreMatrix:" << endl << PreMatrix << endl <<
	 "PostMatrix:" << endl <<PostMatrix<< endl);
  
  MAL_MATRIX_RESIZE(m_F,m_Nl,1);
  for(int k=0;k<m_Nl;k++)
    {
      Intermediate = MAL_RET_A_by_B(PreMatrix, Recursive);
      m_F(k,0) = Intermediate(0,0);
      Recursive = MAL_RET_A_by_B(BaseOfRecursion,Recursive);
    }

  
}

void OptimalControllerSolver::DisplayWeights()
{
  std::cout << "K:" << m_K << std::endl;
  std::cout << "F:" << m_F << std::endl;
}

void OptimalControllerSolver::GetF(MAL_MATRIX(& lF, double))
{
  lF = m_F;
}

void OptimalControllerSolver::GetK(MAL_MATRIX(& lK,double))
{
  lK = m_K;
}

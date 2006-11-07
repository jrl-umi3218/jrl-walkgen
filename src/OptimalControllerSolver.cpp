/** @doc Object to compute the weights of the preview control
    @ingroup previewcontrol
    @endgroup
   $Id:$
   $Author: stasse $
   $Date:$
   $Revision:$
   $Source:  $
   $Log: $
   Revision 1.2  2006-01-18 06:34:58  stasse



   Copyright (c) 2005-2006, 
   @author Olivier Stass
   
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


#include <VNL/Algo/matrixinverse.h>
#include <VNL/NetLib/netlib.h>

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

  logical sb02ow (double *alphar, double * alphai, double *beta)
  {
    logical r = ((*alphar <0.0) && (*beta>0.0)) ||
      ( (*alphar>0.0) && (*beta<0.0)) &&
      ( fabs(*beta) > fabs(*alphar)*dlamch_("p"));
      ;
    return r;
  }


OptimalControllerSolver::OptimalControllerSolver(VNL::Matrix<double> &A, 
						 VNL::Matrix<double> &b, 
						 VNL::Matrix<double> &c,
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

bool OptimalControllerSolver::GeneralizedSchur(VNL::Matrix<double> *A,
					       VNL::Matrix<double> *B,
					       VNL::Vector<double> *alphar,
					       VNL::Vector<double> *alphai,
					       VNL::Vector<double> *beta,
					       VNL::Matrix<double> *L,
					       VNL::Matrix<double> *R)
{
  int n = A->Rows();
  assert(alphar!=0); alphar->Resize(n);    alphar->Fill(0);
  assert(alphai!=0); alphai->Resize(n);    alphai->Fill(0);
  assert(beta!=0);   beta  ->Resize(n);    beta  ->Fill(0);
  assert(L!=0);      L     ->Resize(n, n); L     ->Fill(0);
  assert(R!=0);      R     ->Resize(n, n); R     ->Fill(0);

  int sdim = 0;
  int lwork = 1000+ (8*n + 16);
  double *work = new double[lwork]; //std::vector<double> work(lwork);
  int info = 0;
  logical *bwork=new logical[2*n];
  for(int i=0;i<2*n;i++)
    bwork[i] =0;
  A->InplaceTranspose();
  B->InplaceTranspose();
  dgges_ ("V", "V",
          "S",
         (logical (*)(...))sb02ox,
          &n,
          A->DataBlock(), &n,
          B->DataBlock(), &n,
          &sdim,
          alphar->DataBlock(),
          alphai->DataBlock(),
          beta->DataBlock(),
          L->DataBlock(), &n,
          R->DataBlock(), &n,
          &work[0], &lwork,
          bwork,
          &info);
  A->InplaceTranspose();
  B->InplaceTranspose();
  L->InplaceTranspose();
  R->InplaceTranspose();


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
  VNL::Matrix<double> H;

  H.Resize(2*m_A.Rows(),2*m_A.Cols());
  
  H.SetIdentity();
  
  // Build the upper left sub-block of H
  int n = m_A.Rows();
  for(int i=0;i< n;i++)
    for(int j=0;j<n;j++)
      H(i,j) = m_A(i,j);

  VNL::Matrix<double> H21;
  H21 = -m_c.Transpose() * m_Q * m_c;
  for(int i=0;i< n;i++)
    for(int j=0;j<n;j++)
      H(i+n,j) = H21(i,j);

  ODEBUG("H:" << endl << H);
  VNL::Matrix<double> E(2*n,2*n);
  E.SetIdentity();

  VNL::Matrix<double> G;
  G = (m_b * (1/m_R)) * m_b.Transpose();

  VNL::Matrix<double> At = m_A.Transpose();
  for(int i=0;i< n;i++)
    for(int j=0;j<n;j++)
      {
	E(i,j+n) = G(i,j);
	E(i+n,j+n) = At(i,j);
      }

  ODEBUG("E:" << endl << E);
  // Computes S the Schur form of Laub_Z
  VNL::Matrix<double> ZH(2*n,2*n); // The matrix of schur vectors
  VNL::Matrix<double> ZE(2*n,2*n); // The matrix of Schur vectors.
  VNL::Vector<double> WR(2*n,0.0),WI(2*n,0.0); // The eigenvalues ( a matrix to handle complex eigenvalues).
  VNL::Vector<double> GS(2*n,0.0);

  GeneralizedSchur(&H,&E,&WR,&WI,&GS,&ZH,&ZE);

  ODEBUG("Hx:"<<H);
  ODEBUG("Ex:"<<E);
  ODEBUG("ZH:"<<ZH);
  ODEBUG("ZE:"<<ZE);


  // Computes P the solution of the Riccati equation.
  VNL::Matrix<double> P;
  VNL::Matrix<double> Z11(n,n), Z21(n,n);


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
  
  VNL::MatrixInverse<double> iZ11 (Z11);
  
  P = Z21 * iZ11;
  
  ODEBUG( "P: " << endl << P);

  // Compute the weights.
  VNL::Matrix<double> r;

  double la;
  r = m_b.Transpose() * P * m_b;
  la = m_R + r(0,0);
  la = 1/la;
  
  // Computes the weights for the accumulated difference.
  // and the speed.
  m_K = (m_b.Transpose() * P * m_A) * la;
  
  ODEBUG("K: "<< endl << m_K);
  
  // Computes the weights for the future.
  VNL::Matrix<double> PreMatrix, Recursive, BaseOfRecursion, PostMatrix, Intermediate;	

  PreMatrix = la * m_b.Transpose();
  BaseOfRecursion = m_A - m_b * m_K;
  BaseOfRecursion = BaseOfRecursion.Transpose();
  Recursive = BaseOfRecursion;
  PostMatrix = P * m_c.Transpose() * m_Q;

  ODEBUG("BaseOfRecursion:" << endl << BaseOfRecursion << endl << 
	  "Recursive:" << endl << Recursive << endl << 
	  "PreMatrix:" << endl << PreMatrix << endl <<
	  "PostMatrix:" << endl <<PostMatrix<< endl);

  m_F.Resize(m_Nl,1);
  for(int k=0;k<m_Nl;k++)
    {
      Intermediate = PreMatrix * Recursive * PostMatrix;
      m_F(k,0) = Intermediate(0,0);
      Recursive *= BaseOfRecursion;
    }
  
}

void OptimalControllerSolver::DisplayWeights()
{
  std::cout << "K:" << m_K << std::endl;
  std::cout << "F:" << m_F << std::endl;
}

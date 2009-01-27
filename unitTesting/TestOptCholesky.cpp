/*! \file TestOptCholesky.cpp
  \brief Example to compute a cholesky decomposition using
  an optimized implementation for QP solving.

   Copyright (c) 2009, 
   @author Olivier Stasse,

   $Id$
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation 
   and/or other materials provided with the distribution.
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

#include <stdlib.h>

#include <iostream>
#include <fstream>

#include <walkGenJrl/Mathematics/OptCholesky.h>

using namespace std;

void MatrixMatrixT(double *A,double *AAT, int lNbOfConstraints, int lCardU)
{
  for( int i=0;i<lNbOfConstraints;i++)
    {
      for(int j=0;j<lNbOfConstraints;j++)
	{
	  AAT[i*lNbOfConstraints+j] = 0.0;
	  for(int k=0;k<lCardU;k++)
	    AAT[i*lNbOfConstraints+j] += A[i*lCardU+k]*A[j*lCardU+k];

	}
    }
}

void DisplayMatrix(double *A,int rows, int columns, string name, int format)
{
  cout.precision(5);
  cout.setf(ios::fixed,ios::floatfield);
  cout << name << "=[";
  
  for(int i=0;i<rows;i++)
    {
      cout << "[";
      for(int j=0;j<columns;j++)
	{
	  cout << A[i*rows+j] << " ";
	}
      cout << "];";
      if (format==0) // Human readable.
	cout << endl;
    }
  cout << "]"<<endl;
  
}
double CheckCholeskyDecomposition(double *A, double *L, int n)
{
  double * LLT = new double[n*n];
  MatrixMatrixT(L,LLT,n,n);

  double distance=0.0;
  for(int i=0;i<n;i++)
    {
      for(int j=0;j<n;j++)
	{
	  double r=A[i*n+j] - LLT[i*n+j];
	  distance += r*r;
	}
    }

  delete LLT;
  distance = sqrt(distance);
  
  return distance;
}

int main()
{
  PatternGeneratorJRL::OptCholesky *anOptCholesky;

  unsigned int lNbOfConstraints = 12;
  unsigned int lCardU = 15;
  unsigned int verbose = 0;

  /* Declare the linear system */
  double * A = new double[lNbOfConstraints*lCardU];
  double * L = new double[lNbOfConstraints*lNbOfConstraints];
  
  /* Create the object for optimized Cholesky computation */
  anOptCholesky = new PatternGeneratorJRL::OptCholesky(lNbOfConstraints,lCardU);

  /* Build the test matrix */
  srand(0);
  for(unsigned int i=0;i<lNbOfConstraints;i++)
    {
      for(unsigned int j=0;j<lCardU;j++)
	{
	  A[i*lCardU+j] = (double)rand()/(double)RAND_MAX;
	}
    }
  if (verbose>1)
    DisplayMatrix(A,lNbOfConstraints,lCardU,string("A"),0);

  anOptCholesky->SetA(A);
  anOptCholesky->SetL(L);


  for(unsigned int i=0;i<lNbOfConstraints;i++)
    anOptCholesky->AddActiveConstraint(i);

  if (verbose>1)
    DisplayMatrix(L,lNbOfConstraints,lNbOfConstraints,string("L"),0);
       

  double * AAT = new double[lNbOfConstraints*lNbOfConstraints];

  MatrixMatrixT(A,AAT,lNbOfConstraints,lCardU);

  if (verbose>1)
    DisplayMatrix(AAT,lNbOfConstraints,lNbOfConstraints,string("AAT"),0);

  int return_value = 0;

  double r = CheckCholeskyDecomposition(AAT,L,lNbOfConstraints);
  if (r>1e-6)
    {
      cout << "Optimized Cholesky decomposition for QP pb:" 
	   << r
	   << endl;
      return_value = -1;
    }
  
  delete anOptCholesky;

  /* Create the object for normal Cholesky computation */
  anOptCholesky = new PatternGeneratorJRL::OptCholesky(lNbOfConstraints,lNbOfConstraints);

  anOptCholesky->SetA(AAT);

  anOptCholesky->ComputeNormalCholeskyOnA();

  if (verbose>1)
    DisplayMatrix(L,lNbOfConstraints,lNbOfConstraints,string("L"),0);

  if (r>1e-6)
    {
      cout << "Normal Cholesky decomposition pb:" 
	   << r
	   << endl;
      return_value = -1;
    }
  
  delete anOptCholesky;
  delete [] L;
  delete [] A;
  return return_value;
}

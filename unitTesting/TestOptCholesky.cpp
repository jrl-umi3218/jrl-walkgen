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

int main()
{
  PatternGeneratorJRL::OptCholesky *anOptCholesky;

  unsigned int lNbOfConstraints = 12;
  unsigned int lCardU = 15;
  /* Declare the linear system */
  double * A = new double[lNbOfConstraints*lCardU];
  
  /* Create the object for optimized Cholesky computation */
  anOptCholesky = new PatternGeneratorJRL::OptCholesky(lNbOfConstraints,lCardU);

  /* Build the test matrix */
  srand(0);
  cout.precision(5);
  cout.setf(ios::fixed,ios::floatfield);
  cout << "A=[";
  for(unsigned int i=0;i<lNbOfConstraints;i++)
    {
      cout << "[";
      for(unsigned int j=0;j<lCardU;j++)
	{
	  A[i*lCardU+j] = (double)rand()/(double)RAND_MAX;
	  cout << A[i*lCardU+j] << " ";
	}
      cout << "];";
    }
  cout << "]"<<endl;
  cout << "***********************************************" << endl;
  anOptCholesky->SetA(A);


  cout << "M=[";
  for(unsigned int i=0;i<lNbOfConstraints;i++)
    anOptCholesky->AddActiveConstraint(i);
  cout << "]"<<endl;
  cout << "***********************************************" << endl;
  double *L=0;
  L = anOptCholesky->GetL();
  
  cout << "Matrix L:" << endl;
  for(unsigned int li=0;li<lNbOfConstraints;li++)
    {
      for(unsigned int lj=0;lj<lNbOfConstraints;lj++)
	cout << L[li*lNbOfConstraints+lj] << " ";
      cout << endl;
    }
  cout << "***********************************************" << endl;
	
  delete anOptCholesky;

}

/*! \file TestRiccatiEquation.cpp
  \brief Example to solve the Riccati Equation for the preview control.

   Copyright (c) 2007, 
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

#include <iostream>
#include <fstream>
#include <walkGenJrl/PreviewControl/OptimalControllerSolver.h>

using namespace std;

int main()
{
  PatternGeneratorJRL::OptimalControllerSolver *anOCS;

  /* Declare the linear system */
  MAL_MATRIX_DIM(A,double,3,3);
  MAL_MATRIX_DIM(b,double,3,1);
  MAL_MATRIX_DIM(c,double,1,3);
  MAL_MATRIX(, double) lF;

  /* Declare the weights for the function */
  ofstream aof;
  double Q, R;
  int Nl;
  double T = 0.005;

  /* Build the initial discrete system
     regarding the CoM and the ZMP. */
  A(0,0) = 1.0; A(0,1) =   T; A(0,2) = T*T/2.0;
  A(1,0) = 0.0; A(1,1) = 1.0; A(1,2) = T;
  A(2,0) = 0.0; A(2,1) = 0.0; A(2,2) = 1;

  b(0,0) = T*T*T/6.0;
  b(1,0) = T*T/2.0;
  b(2,0) = T;

  c(0,0) = 1.0;
  c(0,1) = 0.0;
  c(0,2) = -0.814/9.8;
  
  Q = 1.0;
  R = 1e-6;

  Nl = (int)(1.6/T);

  // Build the derivated system
  MAL_MATRIX_DIM(Ax,double,4,4);
  MAL_MATRIX(tmpA,double);
  MAL_MATRIX_DIM(bx,double,4,1);
  MAL_MATRIX(tmpb,double);
  MAL_MATRIX_DIM(cx,double,1,4);
  
  tmpA = MAL_RET_A_by_B(c,A);
  cout << "tmpA :" << tmpA<<endl;

  Ax(0,0)= 1.0;
  for(int i=0;i<3;i++)
    {
      Ax(0,i+1) = tmpA(0,i);
      for(int j=0;j<3;j++)
	Ax(i+1,j+1) = A(i,j);
    }
  cout << "Ax: " << endl << Ax<< endl;

  tmpb = MAL_RET_A_by_B(c,b);
  bx(0,0) = tmpb(0,0);
  for(int i=0;i<3;i++)
    {
      bx(i+1,0) = b(i,0);
    }

  cout << "bx: " << endl << bx << endl;

  cx(0,0) =1.0;
  cout << "cx: " << endl << cx << endl;
  

  anOCS = new PatternGeneratorJRL::OptimalControllerSolver(Ax,bx,cx,Q,R,Nl);

  anOCS->ComputeWeights(PatternGeneratorJRL::OptimalControllerSolver::MODE_WITHOUT_INITIALPOS);

  anOCS->DisplayWeights();

  anOCS->GetF(lF);

  aof.open("WeightsOutput.dat",ofstream::out);

  for(unsigned int li=0;li<MAL_MATRIX_NB_ROWS(lF);li++)
    {
      aof << lF(li,0) << endl;
    }
  aof.close();

  delete anOCS;


  // Build the initial discrete system
  // regarding the CoM and the ZMP.
  T=0.01;
  A(0,0) = 1.0; A(0,1) =   T; A(0,2) = T*T/2.0;
  A(1,0) = 0.0; A(1,1) = 1.0; A(1,2) = T;
  A(2,0) = 0.0; A(2,1) = 0.0; A(2,2) = 1;

  b(0,0) = T*T*T/6.0;
  b(1,0) = T*T/2.0;
  b(2,0) = T;

  c(0,0) = 1.0;
  c(0,1) = 0.0;
  c(0,2) = -0.814/9.8;

  //Q = 10000000.000000;
  //  R = 1.000000 ;
  Q = 1.0;
  R = 1e-5;

  Nl = (int)(1.6/T);

  cout << "A: " << endl << A << endl;
  cout << "b: " << endl << b << endl;
  cout << "c: " << endl << c << endl;
  cout << "Nl: " << Nl << endl;
  anOCS = new PatternGeneratorJRL::OptimalControllerSolver(A,b,c,Q,R,Nl);

  anOCS->ComputeWeights(PatternGeneratorJRL::OptimalControllerSolver::MODE_WITH_INITIALPOS);

  anOCS->DisplayWeights();

  anOCS->GetF(lF);

  aof.open("WeightsOutput2.dat",ofstream::out);

  for(unsigned int li=0;li<MAL_MATRIX_NB_ROWS(lF);li++)
    {
      aof << lF(li,0) << endl;
    }
  aof.close();

  delete anOCS;

}

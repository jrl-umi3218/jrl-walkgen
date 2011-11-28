/*
 * Copyright 2009, 2010, 
 *
 * Francois Keith
 * Olivier  Stasse
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
/*! \file TestRiccatiEquation.cpp
  \brief Example to solve the Riccati Equation for the preview control. */

#include <iostream>
#include <fstream>
#include "PreviewControl/OptimalControllerSolver.hh"

using namespace std;

int main()
{
  PatternGeneratorJRL::OptimalControllerSolver *anOCS;

  /* Declare the linear system */
  MAL_MATRIX_DIM(A,double,3,3);
  MAL_MATRIX_DIM(b,double,3,1);
  MAL_MATRIX_DIM(c,double,1,3);
  MAL_MATRIX_TYPE( double) lF;

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
	  Ax(i+1,0) = 0.;
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
  cx(0,1) =0.0;
  cx(0,2) =0.0;
  cx(0,3) =0.0;
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

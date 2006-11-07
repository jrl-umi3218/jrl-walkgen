#include <iostream>
#include "OptimalControllerSolver.h"

using namespace std;

int main()
{
  PatternGeneratorJRL::OptimalControllerSolver *anOCS;

  VNL::Matrix<double> A(3,3,0.0);
  VNL::Matrix<double> b(3,1);
  VNL::Matrix<double> c(1,3);
  double Q, R;
  int Nl;
  double T = 0.005;

  // Build the initial discrete system
  // regarding the CoM and the ZMP.
  A(0,0) = 1.0; A(0,1) =   T; A(0,2) = T*T/2.0;
  A(1,0) = 0.0; A(1,1) = 1.0; A(1,2) = T;
  A(2,0) = 0.0; A(2,1) = 0.0; A(2,2) = 1;

  b(0,0) = T*T*T/6.0;
  b(1,0) = T*T/2.0;
  b(2,0) = T;

  c(0,0) = 1.0;
  c(0,1) = 0.0;
  c(0,2) = -0.814/9.81;

  Q = 1.0;
  R = 1e-6;

  Nl = (int)(0.8/T);

  // Build the derivated system
  VNL::Matrix<double> Ax(4,4,0.0),tmpA;
  VNL::Matrix<double> bx(4,1,0.0),tmpb;
  VNL::Matrix<double> cx(1,4,0.0);
  
  tmpA = c*A;
  cout << "tmpA :" << tmpA<<endl;

  Ax(0,0)= 1.0;
  for(int i=0;i<3;i++)
    {
      Ax(0,i+1) = tmpA(0,i);
      for(int j=0;j<3;j++)
	Ax(i+1,j+1) = A(i,j);
    }
  cout << "Ax: " << endl << Ax<< endl;

  tmpb = c*b;
  bx(0,0) = tmpb(0,0);
  for(int i=0;i<3;i++)
    {
      bx(i+1,0) = b(i,0);
    }

  cout << "bx: " << endl << bx << endl;

  cx(0,0) =1.0;
  cout << "cx: " << endl << cx << endl;


  
  anOCS = new PatternGeneratorJRL::OptimalControllerSolver(Ax,bx,cx,Q,R,Nl);

  anOCS->ComputeWeights();

  anOCS->DisplayWeights();
  delete anOCS;
}

/*
 * Copyright 2010,
 *
 * Andrei Herdt
 * Francois Keith
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

#include "portability/gettimeofday.hh"

#ifdef WIN32
# include <Windows.h>
#endif /* WIN32 */

#include <time.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <fstream>

#include <ZMPRefTrajectoryGeneration/problem-vel-ref.hh>
using namespace PatternGeneratorJRL;

ProblemVelRef_s::ProblemVelRef_s():
  m(0),me(0),mmax(0), n(0), nmax(0), mnn(0),
  Q(0),D(0),DU(0),DS(0),XL(0),XU(0),X(0), NewX(0),
  U(0),war(0), iwar(0),
  iout(0),ifail(0), iprint(0),
  lwar(0), liwar(0),
  Eps(0)
{

}
ProblemVelRef_s::~ProblemVelRef_s()
{
  ReleaseMemory();
}

void ProblemVelRef_s::ReleaseMemory()
{
  if (Q!=0)
    delete [] Q;

  if (D!=0)
    delete [] D;

  if (DS!=0)
    delete [] DS;

  if (DU!=0)
    delete [] DU;

  if (XL!=0)
    delete [] XL;

  if (XU!=0)
    delete [] XU;

  if (X!=0)
    delete [] X;

  if (NewX!=0)
    delete [] NewX;

  if (iwar!=0)
    delete [] iwar;

  if (war!=0)
    delete [] war;

  if (U!=0)
    free(U);
}

void ProblemVelRef_s::AllocateMemory()
{
  war= new double[lwar];
  iwar = new int[liwar]; // The Cholesky decomposition is done internally.

  U = (double *)malloc( sizeof(double)*(unsigned int)
                        mnn); // Returns the Lagrange multipliers.;

  DS = new double[(8*m_QP_N+1)*2*(m_QP_N+m_stepNumber)];

  DU = new double[(8*m_QP_N+1)*2*(m_QP_N+m_stepNumber)];


  Q=new double[4*(m_QP_N+m_stepNumber)*(m_QP_N
                                        +m_stepNumber)];
  //Quadratic part of the objective function
  D=new double[2*(m_QP_N
                  +m_stepNumber)];   // Linear part of the objective function
  XL=new double[2*(m_QP_N+m_stepNumber)];  // Lower bound of the jerk.
  XU=new double[2*(m_QP_N+m_stepNumber)];  // Upper bound of the jerk.
  X=new double[2*(m_QP_N+m_stepNumber)];   // Solution of the system.
  NewX=new double[2*(m_QP_N+m_stepNumber)];   // Solution of the system.


}

void ProblemVelRef_s::setDimensions(int NbOfConstraints,
                                    int NbOfEqConstraints,
                                    int QP_N,
                                    int StepNumber)
{
  bool reallocationNeeded = true;


  // If all the dimensions are less than
  // the current ones no need to reallocate.
  if ((NbOfConstraints <= m) &&
      (StepNumber <= m_stepNumber) &&
      (QP_N <= m_QP_N))
    reallocationNeeded = false;
  m_stepNumber = StepNumber;
  m_QP_N = QP_N;
  m=NbOfConstraints;
  me=NbOfEqConstraints;
  mmax=m+1;
  n=2*(m_QP_N+StepNumber);
  nmax=n;
  mnn=m+2*n;

  iout=0;
  iprint=1;
  lwar=3*nmax*nmax/2+ 10*nmax  + 2*mmax + 20000;
  liwar=n;
  Eps=1e-8;

  if(reallocationNeeded)
    {
      ReleaseMemory();
      AllocateMemory();
    }
}

void ProblemVelRef_s::initializeProblem()
{

  memset(DU,0,(8*m_QP_N+1)*2*(m_QP_N+m_stepNumber)*sizeof(double));
}

void ProblemVelRef_s::dumpMatrix(std::ostream & aos,
                                 int type)
{

  int lnbrows=0, lnbcols=0;
  double *aMatrix=0;
  switch(type)
    {
    case MATRIX_Q:
      lnbrows = lnbcols = (m_QP_N+m_stepNumber)*2 ;
      aMatrix = Q;
      break;

    case MATRIX_DU:
      lnbrows = m; // NbOfConstraints.
      lnbcols = (m_QP_N+m_stepNumber)*2;
      aMatrix = DU;
      break;
    }

  aos << "["<<lnbcols << ","<< lnbrows << "]" << std::endl;

  for(int i=0; i<lnbrows; i++)
    {
      for(int j=0; j<lnbcols; j++)
        aos << aMatrix[j*lnbrows+i] << " ";
      aos << std::endl;
    }
}

void ProblemVelRef_s::dumpVector(std::ostream & aos,
                                 int type)
{

  int lsize=0;
  double *aVector=0;
  switch(type)
    {

    case VECTOR_D:
      lsize=2*(m_QP_N+m_stepNumber) ;
      aVector = D;
      break;

    case VECTOR_XL:
      lsize=2*(m_QP_N+m_stepNumber) ;
      aVector = XL;
      break;

    case VECTOR_XU:
      lsize=2*(m_QP_N+m_stepNumber) ;
      aVector = XU;
      break;

    case VECTOR_DS:
      lsize= m;
      aVector = DS;
      break;
    }

  for(int i=0; i<lsize; i++)
    {
      aos << aVector[i] << " ";
    }
  aos << std::endl;

}

void ProblemVelRef_s::dumpVector(const char * filename,
                                 int type)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  dumpVector(aof,type);
  aof.close();
}


void ProblemVelRef_s::dumpMatrix(const char * filename,
                                 int type)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  dumpMatrix(aof,type);
  aof.close();
}

void ProblemVelRef_s::dumpProblem(std::ostream &aos)
{
  dumpMatrix(aos,MATRIX_Q);
  dumpMatrix(aos,MATRIX_DU);

  dumpVector(aos,VECTOR_D);
  dumpVector(aos,VECTOR_DL);
  dumpVector(aos,VECTOR_XL);
  dumpVector(aos,VECTOR_XU);
  dumpVector(aos,VECTOR_DS);

}
void ProblemVelRef_s::dumpProblem(const char * filename)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  dumpProblem(aof);
  aof.close();
}

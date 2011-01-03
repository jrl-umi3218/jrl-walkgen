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

#include <exception>

#include <ZMPRefTrajectoryGeneration/qp-problem.hh>
using namespace PatternGeneratorJRL;

QPProblem_s::QPProblem_s():
  m(0),me(0),mmax(0), n(0), nmax(0), mnn(0),
  Q(0),D(0),DU(0),DS(0),XL(0),XU(0),X(0),NewX(0),
  U(0),war(0), iwar(0),
  iout(0),ifail(0), iprint(0),
  lwar(0), liwar(0),
  Eps(0),
  m_NbVariables(0), m_NbConstraints(0),
  m_ReallocMarginVar(0), m_ReallocMarginConstr(0)
{

}
QPProblem_s::~QPProblem_s()
{
  ReleaseMemory();
}

void QPProblem_s::ReleaseMemory()
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
    delete [] U;
}


void
QPProblem_s::resizeAll(const int & NbVariables, const int & NbConstraints)
{

  resize(war,2*lwar,2*(3*NbVariables*NbVariables/2+ 10*NbVariables  + 2*(NbConstraints+1) + 20000));
  initialize(war,2*(3*NbVariables*NbVariables/2+ 10*NbVariables  + 2*(NbConstraints+1) + 20000));
  resize(iwar,2*liwar,2*NbVariables); // The Cholesky decomposition is done internally.
  initialize(iwar,2*NbVariables);
  resize(U,2*mnn,2*(NbConstraints+2*NbVariables));
  initialize(U,2*(NbConstraints+2*NbVariables));

  resize(DS,2*m,2*NbConstraints);
  initialize(DS,2*NbConstraints);
  resize(DU,2*m*n,2*NbVariables*NbConstraints);
  initialize(DU,2*NbVariables*NbConstraints);

  resize(Q,2*n*n,2*NbVariables*NbVariables);  //Quadratic part of the objective function
  initialize(Q,2*NbVariables*NbVariables);
  resize(D,2*n,2*NbVariables);   // Linear part of the objective function
  initialize(D,2*NbVariables);

  resize(XL,2*n,2*NbVariables);  // Lower bound on the solution.
  initialize(XL,2*NbVariables);
  resize(XU,2*n,2*NbVariables);  // Upper bound on the solution.
  initialize(XU,2*NbVariables);
  resize(X,2*n,2*NbVariables);  // Solution of the problem.
  initialize(X,2*NbVariables);
  resize(NewX,2*n,2*NbVariables);  // Solution of the problem.
  initialize(NewX,2*NbVariables);

}


int
QPProblem_s::resize(double *& array, const int & old_size, const int & new_size)
{

  try
    {
      double * NewArray = new double[new_size];
      for(int i = 0; i < old_size; i++)
	NewArray[i] = array[i];

      if (array!=0)
	delete [] array;
      array = NewArray;
    }
  catch (std::bad_alloc& ba)
    {
      std::cerr << "bad_alloc caught: " << ba.what() << std::endl;
    }

  return 0;

}


int
QPProblem_s::resize(int *& array, const int & old_size, const int & new_size)
{

  try
    {
      int * NewArray = new int[new_size];
      for(int i = 0; i < old_size; i++)
	NewArray[i] = array[i];

      if (array!=0)
	delete [] array;
      array = NewArray;
    }
  catch (std::bad_alloc& ba)
    {
      std::cerr << "bad_alloc caught: " << ba.what() << std::endl;
    }

  return 0;

}


void
QPProblem_s::setDimensions(const int & NbVariables,
			   const int & NbConstraints,
			   const int & NbEqConstraints)
{

  // If all the dimensions are less than
  // the current ones no need to reallocate.
  if (NbVariables > m_ReallocMarginVar)
    {
      m_ReallocMarginVar = 2*NbVariables;
      resizeAll(NbVariables, NbConstraints);
    }
  if (NbConstraints > m_ReallocMarginConstr)
    {
      m_ReallocMarginConstr = 2*NbConstraints;
      resize(DS,2*m,2*NbConstraints);
      initialize(DS,2*NbConstraints);
      resize(DU,2*m*n,2*NbVariables*NbConstraints);
      initialize(DU,2*NbVariables*NbConstraints);
    }

  m = m_NbConstraints = NbConstraints;
  me = NbEqConstraints;
  mmax = m+1;
  n = m_NbVariables = NbVariables;
  nmax = n;
  mnn = m+2*n;

  iout = 0;
  iprint = 1;
  lwar = 3*nmax*nmax/2+ 10*nmax  + 2*mmax + 20000;
  liwar = n;
  Eps = 1e-8;
  
}


void
QPProblem_s::initialize(double * array, const int & size)
{
  memset(array,0,size*sizeof(double));
}


void
QPProblem_s::initialize(int * array, const int & size)
{
  memset(array,0,size*sizeof(int));
}


void
QPProblem_s::solve(const int solver)
{
  switch(solver)
    {
    case QLD:
      ql0001_(&m, &me, &mmax, &n, &nmax, &mnn,
              Q, D, DU, DS, XL, XU,
              X, U, &iout, &ifail, &iprint,
              war, &lwar, iwar, &liwar, &Eps);
    }
}


void
QPProblem_s::addTerm(const MAL_MATRIX (&Mat, double), const int target,
		     const int row, const int col)
{

  double * aArray;
  int max_rows, max_cols;

  switch(target)
    {
    case MATRIX_Q:
      aArray = Q;
      max_rows = n;
      max_cols = n;
      break;

    case MATRIX_DU:
      aArray = DU;
      max_rows = m;
      max_cols = n;
      break;
    }

  if(row >= max_rows || col >= max_cols)
    {
      //throw sth.
    }

  for(unsigned int i = 0;i < MAL_MATRIX_NB_ROWS(Mat); i++)
    for(unsigned int j = 0;j < MAL_MATRIX_NB_COLS(Mat); j++)
      aArray[row+i+(col+j)*n] += Mat(i,j);

}


void QPProblem_s::addTerm(const MAL_VECTOR (&Vec, double), const int target,
			  const int row)
{

  double * aArray;
  int max_rows;

  switch(target)
    {
    case VECTOR_D:
      aArray = D;
      max_rows = n;
      break;

    case VECTOR_XL:
      aArray = XL;
      max_rows = n;
      break;

    case VECTOR_XU:
      aArray = XU;
      max_rows = n;
      break;

    case VECTOR_DS:
      aArray = DS;
      max_rows = n;
      break;
    }

  if(row >= max_rows)
    {
      //throw sth.
    }

  for(unsigned int i = 0;i < MAL_VECTOR_SIZE(Vec); i++)
    aArray[row+i] += Vec(i);

}


void
QPProblem_s::dumpSolverParameters(std::ostream & aos)
{
  aos << "m: " << m << std::endl
      << "me: " << me << std::endl
      << "mmax: " << mmax << std::endl
      << "n: " << n << std::endl
      << "nmax: " << nmax << std::endl
      << "mnn: " << mnn << std::endl
      << "iout: " << iout << std::endl
      << "iprint: " << iprint << std::endl
      << "lwar: " << lwar << std::endl
      << "liwar: " << liwar << std::endl
      << "Eps: " << Eps << std::endl;
}


void
QPProblem_s::dump(const int array, std::ostream & aos)
{

  int lnbrows=0, lnbcols=0;
  double * aArray=0;
  std::string Name;
  switch(array)
    {
    case MATRIX_Q:
      lnbrows = lnbcols = n ;
      aArray = Q;
      Name = "Q";
      break;

    case MATRIX_DU:
      lnbrows = mmax;
      lnbcols = n;
      aArray = DU;
      Name = "DU";
      break;

    case VECTOR_D:
      lnbrows = n;
      lnbcols = 1 ;
      aArray = D;
      Name = "D";
      break;

    case VECTOR_XL:
      lnbrows = n;
      lnbcols = 1;
      aArray = XL;
      Name = "XL";
      break;

    case VECTOR_XU:
      lnbrows = n;
      lnbcols=1;
      aArray = XU;
      Name = "XU";
      break;

    case VECTOR_DS:
      lnbrows = m;
      lnbcols= 1;
      aArray = DS;
      Name = "DS";
      break;
    }

  aos << Name <<"["<<lnbrows<< ","<< lnbcols << "]" << std::endl;
  
  for(int i=0;i<lnbrows;i++)
    {
      for(int j=0;j<lnbcols;j++)
	aos << aArray[i+j*lnbrows] << " ";
      aos << std::endl;
    }
  aos << std::endl;
}


void
QPProblem_s::dump(const int array, const char * filename)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  dump(array,aof);
  aof.close();
}


void
QPProblem_s::dumpProblem(std::ostream &aos)
{
  dump(MATRIX_Q,aos);
  dump(MATRIX_DU,aos);
  
  dump(VECTOR_D,aos);
  dump(VECTOR_XL,aos);
  dump(VECTOR_XU,aos);
  dump(VECTOR_DS,aos);

  dumpSolverParameters(aos);
}


void
QPProblem_s::dumpProblem(const char * filename)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  dumpProblem(aof);
  aof.close();
}

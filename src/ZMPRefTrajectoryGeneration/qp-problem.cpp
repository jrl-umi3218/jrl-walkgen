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
  Q(0),D(0),DU(0),DS(0),XL(0),XU(0),X(0),
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

  if (iwar!=0)
    delete [] iwar;

  if (war!=0)
    delete [] war;

  if (U!=0)
    delete [] U;
}


void
QPProblem_s::resizeAll( int NbVariables, int NbConstraints)
{

  resize(Q,2*n*n,2*NbVariables*NbVariables);  //Quadratic part of the objective function
  resize(D,2*n,2*NbVariables);   // Linear part of the objective function

  resize(DS,2*m,2*NbConstraints);
  resize(DU,2*m*n,2*NbVariables*NbConstraints);

  resize(XL,2*n,2*NbVariables);  // Lower bound on the solution.
  initialize(XL,2*NbVariables,-1e8);
  resize(XU,2*n,2*NbVariables);  // Upper bound on the solution.
  initialize(XU,2*NbVariables,1e8);

  resize(X,2*n,2*NbVariables);  // Solution of the problem.
  resize(U,2*mnn,2*(NbConstraints+2*NbVariables));

  resize(war,2*lwar,2*(3*NbVariables*NbVariables/2+ 10*NbVariables  + 2*(NbConstraints+1) + 20000));
  resize(iwar,2*liwar,2*NbVariables); // The Cholesky decomposition is done internally.


}


void QPProblem_s::clear( int type )
{

  double * array;
  int array_size;
  switch(type)
    {
    case MATRIX_Q:
      array = Q;
      array_size = n*n;
      break;
    case MATRIX_DU:
      array = DU;
      array_size = mmax*n;
      break;
    case VECTOR_D:
      array = D;
      array_size = n;
      break;
    case VECTOR_DS:
      array = DS;
      array_size = mmax;
      break;
    case VECTOR_XL:
      array = XL;
      array_size = n;
      break;
    case VECTOR_XU:
      array = XU;
      array_size = n;
      break;
    }

  std::fill_n(array,array_size,0);

}


void QPProblem_s::clear( int type,
			 int row, int col,
			 int nb_rows, int nb_cols )
{

  double * array;
  int
    max_rows, max_cols,
    n_rows,n_cols;

  switch(type)
    {
    case MATRIX_Q:
      array = Q;
      max_rows = n_rows = n;
      max_cols = n;
      break;

    case MATRIX_DU:
      array = DU;
      max_rows = m;
      max_cols = n;
      n_rows = mmax;
      n_cols = n;
      break;
    }

  if(row + nb_rows > max_rows || col + nb_cols > max_cols)
    {
      //throw sth. like:
      std::cout<<"Matrix "<<type<<" bounds violated in clear(): "<<std::endl<<
        "max_rows: "<<max_rows<<std::endl<<
        "max_cols: "<<max_cols<<std::endl<<
        "req. cols: "<<row + nb_rows<<
        "req. rows: "<<col + nb_cols<<std::endl;
    }

  for( int i = row;i < row+nb_rows; i++)
    for( int j = col;j < nb_cols; j++)
      array[row+i+(col+j)*n_rows] = 0.0;

}


void
QPProblem_s::setDimensions( int nb_variables,
			    int nb_constraints,
			    int nb_eq_constraints )
{

  // If all the dimensions are less than
  // the current ones no need to reallocate.
  if (nb_variables > m_ReallocMarginVar)
    {
      m_ReallocMarginVar = 2*nb_variables;
      resizeAll(nb_variables, nb_constraints);
    }
  if (nb_constraints > m_ReallocMarginConstr)
    {
      m_ReallocMarginConstr = 2*nb_constraints;
      resize(DS,2*m,2*nb_constraints);
      resize(DU,2*m*n,2*nb_variables*nb_constraints);
    }

  m = m_NbConstraints = nb_constraints;
  me = nb_eq_constraints;
  mmax = m+1;
  n = m_NbVariables = nb_variables;
  nmax = n;
  mnn = m+2*n;

  iout = 0;
  iprint = 1;
  lwar = 3*nmax*nmax/2+ 10*nmax  + 2*mmax + 20000;
  liwar = n;
  Eps = 1e-8;
  
  //      if (m_FastFormulationMode==QLDANDLQ)
   //        m_Pb.iwar[0]=0;
   //      else
  iwar[0]=1;

}


void
QPProblem_s::solve( int solver, solution_t & result )
{
  switch(solver)
    {
    case QLD:
      ql0001_(&m, &me, &mmax, &n, &nmax, &mnn,
              Q, D, DU, DS, XL, XU,
              X, U, &iout, &ifail, &iprint,
              war, &lwar, iwar, &liwar, &Eps);

      result.resize(n,m);


      for(int i = 0; i < n; i++)
        {
          result.vecSolution(i) = X[i];
          result.vecLBoundsLagr(i) = U[m+i];
          result.vecUBoundsLagr(i) = U[m+n+i];
        }
      for(int i = 0; i < m; i++)
        {
          result.vecConstrLagr(i) = U[i];
        }

      result.Fail = ifail;
      result.Print = iprint;

    }
}


void
QPProblem_s::addTerm( const MAL_MATRIX (&Mat, double), int type,
		      unsigned int row, unsigned int col )
{

  double * array;
  unsigned int
    max_rows, max_cols,
    n_rows,n_cols;

  switch(type)
    {
    case MATRIX_Q:
      array = Q;
      max_rows = n_rows = n;
      max_cols = n;
      break;

    case MATRIX_DU:
      array = DU;
      max_rows = m;
      max_cols = n;
      n_rows = mmax;
      n_cols = n;
      break;
    }

  if(row + Mat.size1() > max_rows || col + Mat.size2() > max_cols)
    {
      //throw sth. like:
      std::cout<<"Matrix "<<type<<" bounds violated in addTerm: "<<std::endl<<
        " max_rows: "<<max_rows<<std::endl<<
        " max_cols: "<<max_cols<<std::endl<<
        " req. cols: "<<row + Mat.size1()<<std::endl<<
        " req. rows: "<<col + Mat.size2()<<std::endl;
    }

  for(unsigned int i = 0;i < MAL_MATRIX_NB_ROWS(Mat); i++)
    for(unsigned int j = 0;j < MAL_MATRIX_NB_COLS(Mat); j++)
      array[row+i+(col+j)*n_rows] += Mat(i,j);

}


void QPProblem_s::addTerm( const MAL_VECTOR (&Vec, double), int type,
			   unsigned int row )
{

  double * aArray;
  unsigned int max_rows;

  switch(type)
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
      max_rows = mmax;
      break;
    }

  if(row + Vec.size() > max_rows)
    {
      //throw sth. like:
      std::cout<<"Vector "<<type<<" bounds violated in addTerm: "<<std::endl<<
        "max_rows: "<<max_rows<<std::endl<<
        "required: "<<row + Vec.size()<<std::endl;
    }

  for(unsigned int i = 0;i < Vec.size(); i++)
    aArray[row+i] += Vec(i);

}

void
QPProblem_s::solution_t::resize( int size_sol, int size_constr )
{
  NbVariables = size_sol;
  NbConstraints = size_constr;

  vecSolution.resize(size_sol, false);
  vecConstrLagr.resize(size_constr, false);
  vecLBoundsLagr.resize(size_sol, false);
  vecUBoundsLagr.resize(size_sol, false);
}


void
QPProblem_s::solution_t::dump(const char * filename)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  print(aof);
  aof.close();
}


void
QPProblem_s::solution_t::print(std::ostream & aos)
{
  aos << "Arrays:" << std::endl
      << "Solution: ";
   for(int i = 0; i < NbVariables; i++)
     {aos<<vecSolution[i]<<" ";}; aos<<std::endl;
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
QPProblem_s::dump( int type, std::ostream & aos)
{

  int lnbrows=0, lnbcols=0;
  double * aArray=0;
  std::string Name;
  switch(type)
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
QPProblem_s::dump( int type, const char * filename)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  dump(type,aof);
  aof.close();
}


void
QPProblem_s::dumpProblem(std::ostream &aos)
{
  dump(MATRIX_Q,aos);
  dump(VECTOR_D,aos);

  dump(MATRIX_DU,aos);
  dump(VECTOR_DS,aos);

  dump(VECTOR_XL,aos);
  dump(VECTOR_XU,aos);

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

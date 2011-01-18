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
  iout(0),ifail(0), iprint(0),
  lwar(0), liwar(0),
  Eps(0),
  m_NbVariables(0), m_NbConstraints(0),
  m_ReallocMarginVar(0), m_ReallocMarginConstr(0),
  scale_factor_(2)
{

}


QPProblem_s::~QPProblem_s()
{

  releaseMemory();
}


void
QPProblem_s::releaseMemory()
{
}


//void setNbVariables( int nb_variables )
//{ if(nb_variables*m_NbConstraints > Q_.size_)
//  {
//    bool preserve = true;
//    Q_.resize(2*nb_variables*nb_variables, nb_variables, nb_variables, preserve);
//    D_.resize(2*nb_variables, nb_variables, preserve);
//
//    DS_.resize(2*m_NbConstraints,2*nb_constraints);
//    resize(DU_.array_,2*m*n,2*nb_variables*nb_constraints);
//
//    resize(XL_.array_,2*n,2*nb_variables);  // Lower bound on the solution.
//    initialize(XL_.array_,2*nb_variables,-1e8);
//    resize(XU_.array_,2*n,2*nb_variables);  // Upper bound on the solution.
//    initialize(XU_.array_,2*nb_variables,1e8);
//
//    resize(X_.array_,2*n,2*nb_variables);  // Solution of the problem.
//    resize(U_.array_,2*mnn,2*(nb_constraints+2*nb_variables));
//
//    resize(war_.array_,2*lwar,2*(3*nb_variables*nb_variables/2+ 10*nb_variables  + 2*(nb_constraints+1) + 20000));
//    resize(iwar_.array_,2*liwar,2*nb_variables); // The Cholesky decomposition is done internally.
//
//    m_NbVariables = n = nb_variables;
//  }
//}


void
QPProblem_s::resizeAll()
{
  Q_.resize(2*m_NbVariables, 2*m_NbVariables,true);
  D_.resize(2*m_NbVariables, 1,true);
  DU_.resize(2*m_NbConstraints, 2*m_NbVariables,true);
  DS_.resize(2*m_NbConstraints, 1,true);
  XL_.resize(2*m_NbVariables, 1,true);
  XL_.fill(-1e8);
  XU_.resize(2*m_NbVariables, 1,true);
  XU_.fill(1e8);
  U_.resize(2*(m_NbConstraints+2*m_NbVariables), 1,true);
  X_.resize(2*m_NbVariables, 1,true);
  war_.resize(2*(3*m_NbVariables*m_NbVariables/2+10*m_NbVariables+2*(m_NbConstraints+1)+20000), 1,true);
  iwar_.resize(2*m_NbVariables, 1,true);
}


void
QPProblem_s::clear( int type )
{

  double * array;
  int array_size;
  switch(type)
    {
    case MATRIX_Q:
      Q_.fill(0.0);
      break;
    case MATRIX_DU:
      DU_.fill(0.0);
      break;
    case VECTOR_D:
      D_.fill(0.0);
      break;
    case VECTOR_DS:
      DS_.fill(0.0);
      break;
    case VECTOR_XL:
      XL_.fill(-1e8);
      break;
    case VECTOR_XU:
      XU_.fill(1e8);
      break;
    }

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
      array = Q_.array_;
      max_rows = n_rows = n;
      max_cols = n;
      break;

    case MATRIX_DU:
      array = DU_.array_;
      max_rows = m_NbConstraints;
      max_cols = m_NbVariables;
      n_rows = m_NbConstraints+1;
      n_cols = m_NbVariables;
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
//
//  // If all the dimensions are less than
//  // the current ones no need to reallocate.
//  if (nb_variables > m_ReallocMarginVar)
//    {
//      m_ReallocMarginVar = 2*nb_variables;
//      resizeAll(nb_variables, nb_constraints);
//    }
//  if (nb_constraints > m_ReallocMarginConstr)
//    {
//      m_ReallocMarginConstr = 2*nb_constraints;
//      resize(DS_.array_,2*m,2*nb_constraints);
//      resize(DU_.array_,2*m*n,2*nb_variables*nb_constraints);
//    }
//
//  m = m_NbConstraints = nb_constraints;
//  me = nb_eq_constraints;
//  mmax = m+1;
//  n = m_NbVariables = nb_variables;
//  nmax = n;
//  mnn = m+2*n;
//
//  iout = 0;
//  iprint = 1;
//  lwar = 3*nmax*nmax/2+ 10*nmax  + 2*mmax + 20000;
//  liwar = n;
//  Eps = 1e-8;
//
//  //      if (m_FastFormulationMode==QLDANDLQ)
//   //        m_Pb.iwar_.array_[0]=0;
//   //      else
//  iwar_.array_[0]=1;

}


void
QPProblem_s::solve( int solver, solution_t & result )
{
  switch(solver)
    {
    case QLD:

       m = m_NbConstraints;
       me = m_NbEqConstraints;
       mmax = m+1;
       n = m_NbVariables;
       nmax = n;
       mnn = m+2*n;

       iout = 0;
       iprint = 1;
       lwar = 3*nmax*nmax/2+ 10*nmax  + 2*mmax + 20000;
       liwar = n;
       Eps = 1e-8;

       //      if (m_FastFormulationMode==QLDANDLQ)
        //        m_Pb.iwar_.array_[0]=0;
        //      else
       iwar_.array_[0]=1;

       Q_.stick_together(Q_dense_.array_,n,n);
       DU_.stick_together(DU_dense_.array_,mmax,n);

      ql0001_(&m, &me, &mmax, &n, &nmax, &mnn,
              Q_dense_.array_, D_.array_, DU_dense_.array_, DS_.array_, XL_.array_, XU_.array_,
              X_.array_, U_.array_, &iout, &ifail, &iprint,
              war_.array_, &lwar, iwar_.array_, &liwar, &Eps);

      result.resize(n,m);


      for(int i = 0; i < n; i++)
        {
          result.vecSolution(i) = X_.array_[i];
          result.vecLBoundsLagr(i) = U_.array_[m+i];
          result.vecUBoundsLagr(i) = U_.array_[m+n+i];
        }
      for(int i = 0; i < m; i++)
        {
          result.vecConstrLagr(i) = U_.array_[i];
        }

      result.Fail = ifail;
      result.Print = iprint;

    }

  Q_.fill(0.0);
  DU_.fill(0.0);
  D_.fill(0.0);
  DS_.fill(0.0);

}


void
QPProblem_s::addTerm( const MAL_MATRIX (&Mat, double), int type,
		       int row,  int col )
{

  array_s<double> * pArray_s = 0;

  switch(type)
    {
    case MATRIX_Q:
      pArray_s = &Q_;
      m_NbVariables = (col+Mat.size2()>m_NbVariables) ? col+Mat.size2() : m_NbVariables;
      break;

    case MATRIX_DU:
      pArray_s = &DU_;
      m_NbConstraints = (row+Mat.size1()>m_NbConstraints) ? row+Mat.size1() : m_NbConstraints;
      m_NbVariables = (col+Mat.size2()>m_NbVariables) ? col+Mat.size2() : m_NbVariables;
      break;
    }

  if(row + Mat.size1() > pArray_s->nrows_ || col + Mat.size2() > pArray_s->ncols_ )
    {
      //throw sth. like:
      std::cout<<"Matrix "<<pArray_s->id_<<" bounds violated in addTerm: "<<std::endl<<
        " max_rows: "<<pArray_s->nrows_<<std::endl<<
        " max_cols: "<<pArray_s->ncols_<<std::endl<<
        " req. cols: "<<row + Mat.size1()<<std::endl<<
        " req. rows: "<<col + Mat.size2()<<std::endl;
    }

  if(m_NbVariables > pArray_s->ncols_ )
    {
      resizeAll();
    }

  if( m_NbConstraints > DU_.nrows_-1 )
    {
      DU_.resize(2*m_NbConstraints, 2*m_NbVariables,true);
      DS_.resize(2*m_NbConstraints,1,true);

      U_.resize(2*(m_NbConstraints+2*m_NbVariables), 1,true);
      war_.resize(2*(3*m_NbVariables*m_NbVariables/2+10*m_NbVariables+2*(m_NbConstraints+1)+20000), 1,true);
    }

  for( int i = 0;i < MAL_MATRIX_NB_ROWS(Mat); i++)
    for( int j = 0;j < MAL_MATRIX_NB_COLS(Mat); j++)
      {
        pArray_s->array_[row+i+(col+j)*pArray_s->nrows_] += Mat(i,j);
      }
}


void QPProblem_s::addTerm( const MAL_VECTOR (&Vec, double), int type,
			    int row )
{

  array_s<double> * pArray_s = 0;


  switch(type)
    {
    case VECTOR_D:
      pArray_s = &D_;
      m_NbVariables = (row+Vec.size()>m_NbVariables) ? row+Vec.size() : m_NbVariables;
      break;

    case VECTOR_XL:
      pArray_s = &XL_;
      m_NbVariables = (row+Vec.size()>m_NbVariables) ? row+Vec.size() : m_NbVariables;
      break;

    case VECTOR_XU:
      pArray_s = &XU_;
      m_NbVariables = (row+Vec.size()>m_NbVariables) ? row+Vec.size() : m_NbVariables;
      break;

    case VECTOR_DS:
      pArray_s = &DS_;
      m_NbConstraints = (row+Vec.size()>m_NbConstraints) ? row+Vec.size() : m_NbConstraints;
      break;
    }

  if(row + Vec.size() > pArray_s->nrows_)
    {
      //throw sth. like:
      std::cout<<"Vector "<<pArray_s->id_<<" bounds violated in addTerm: "<<std::endl<<
        "max_rows: "<<pArray_s->nrows_<<std::endl<<
        "required: "<<row + Vec.size()<<std::endl;
    }

  if(m_NbVariables > D_.ncols_ )
    {
      resizeAll();
    }


  for( int i = 0; i < Vec.size(); i++ )
    pArray_s->array_[row+i] += Vec(i);

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
  double * array=0;
  std::string Name;
  switch(type)
    {
    case MATRIX_Q:
      lnbrows = lnbcols = m_NbVariables ;
      array = Q_dense_.array_;
      Name = "Q";
      break;

    case MATRIX_DU:
      lnbrows = m_NbConstraints+1;
      lnbcols = m_NbVariables;
      array = DU_dense_.array_;
      Name = "DU";
      break;

    case VECTOR_D:
      lnbrows = m_NbVariables;
      lnbcols = 1 ;
      array = D_.array_;
      Name = "D";
      break;

    case VECTOR_XL:
      lnbrows = m_NbVariables;
      lnbcols = 1;
      array = XL_.array_;
      Name = "XL";
      break;

    case VECTOR_XU:
      lnbrows = m_NbVariables;
      lnbcols=1;
      array = XU_.array_;
      Name = "XU";
      break;

    case VECTOR_DS:
      lnbrows = m_NbConstraints+1;
      lnbcols= 1;
      array = DS_.array_;
      Name = "DS";
      break;
    }

  aos << Name <<"["<<lnbrows<< ","<< lnbcols << "]" << std::endl;
  
  for(int i=0;i<lnbrows;i++)
    {
      for(int j=0;j<lnbcols;j++)
	aos << array[i+j*lnbrows] << " ";
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

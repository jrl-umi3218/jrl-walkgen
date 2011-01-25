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
  nbvariables_(0), nbconstraints_(0),nbeqconstraints_(0),
  m_ReallocMarginVar(0), m_ReallocMarginConstr(0),
  scale_factor_(2)
{

}


QPProblem_s::~QPProblem_s()
{

  release_memory();
}


void
QPProblem_s::release_memory()
{
}


void
QPProblem_s::resize_all()
{
  Q_.resize(2*nbvariables_, 2*nbvariables_,true);
  D_.resize(2*nbvariables_, 1,true);
  DU_.resize(2*nbconstraints_, 2*nbvariables_,true);
  DS_.resize(2*nbconstraints_, 1,true);
  XL_.resize(2*nbvariables_, 1,true);
  XL_.fill(-1e8);
  XU_.resize(2*nbvariables_, 1,true);
  XU_.fill(1e8);
  U_.resize(2*(nbconstraints_+2*nbvariables_), 1,true);
  X_.resize(2*nbvariables_, 1,true);
  war_.resize(2*(3*nbvariables_*nbvariables_/2+10*nbvariables_+2*(nbconstraints_+1)+20000), 1,true);
  iwar_.resize(2*nbvariables_, 1,true);
}


void
QPProblem_s::clear( int type )
{

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


void
QPProblem_s::solve( int solver, solution_t & result )
{
  switch(solver)
    {
    case QLD:

       m = nbconstraints_;
       me = nbeqconstraints_;
       mmax = m+1;
       n = nbvariables_;
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
          result.Solution_vec(i) = X_.array_[i];
          result.LBoundsLagr_vec(i) = U_.array_[m+i];
          result.UBoundsLagr_vec(i) = U_.array_[m+n+i];
        }
      for(int i = 0; i < m; i++)
        {
          result.ConstrLagr_vec(i) = U_.array_[i];
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
QPProblem_s::add_term( const MAL_MATRIX (&Mat, double), int type,
		       int row,  int col )
{

  array_s<double> * pArray_s = 0;

  switch(type)
    {
    case MATRIX_Q:
      pArray_s = &Q_;
      nbvariables_ = (col+(int)Mat.size2()>nbvariables_) ? col+(int)Mat.size2() : nbvariables_;
      break;

    case MATRIX_DU:
      pArray_s = &DU_;
      nbconstraints_ = (row+(int)Mat.size1()>nbconstraints_) ? row+(int)Mat.size1() : nbconstraints_;
      nbvariables_ = (col+(int)Mat.size2()>nbvariables_) ? col+(int)Mat.size2() : nbvariables_;
      break;
    }

  if(nbvariables_ > pArray_s->ncols_ )
    {
      resize_all();
    }

  if( nbconstraints_ > DU_.nrows_-1 )
    {
      DU_.resize(2*nbconstraints_, 2*nbvariables_,true);
      DS_.resize(2*nbconstraints_,1,true);

      U_.resize(2*(nbconstraints_+2*nbvariables_), 1,true);
      war_.resize(2*(3*nbvariables_*nbvariables_/2+10*nbvariables_+2*(nbconstraints_+1)+20000), 1,true);
    }

  for( int i = 0;i < (int)MAL_MATRIX_NB_ROWS(Mat); i++)
    for( int j = 0;j < (int)MAL_MATRIX_NB_COLS(Mat); j++)
      {
        pArray_s->array_[row+i+(col+j)*pArray_s->nrows_] += Mat(i,j);
      }

}


void QPProblem_s::add_term( const MAL_VECTOR (&Vec, double), int type,
			    int row )
{

  array_s<double> * pArray_s = 0;


  switch(type)
    {
    case VECTOR_D:
      pArray_s = &D_;
      nbvariables_ = (row+(int)Vec.size()>nbvariables_) ? row+(int)Vec.size() : nbvariables_;
      break;

    case VECTOR_XL:
      pArray_s = &XL_;
      nbvariables_ = (row+(int)Vec.size()>nbvariables_) ? row+(int)Vec.size() : nbvariables_;
      break;

    case VECTOR_XU:
      pArray_s = &XU_;
      nbvariables_ = (row+(int)Vec.size()>nbvariables_) ? row+(int)Vec.size() : nbvariables_;
      break;

    case VECTOR_DS:
      pArray_s = &DS_;
      nbconstraints_ = (row+(int)Vec.size()>nbconstraints_) ? row+(int)Vec.size() : nbconstraints_;
      break;
    }

  if(nbvariables_ > D_.nrows_ )
    {
      resize_all();
    }

  boost_ublas::vector<double>::const_iterator VecIt = Vec.begin();
  for( int i = 0; i < (int)Vec.size(); i++ )
  {
    pArray_s->array_[row+i] += *VecIt;
    VecIt++;
  }

}


void
QPProblem_s::solution_t::resize( int size_sol, int size_constr )
{
  NbVariables = size_sol;
  NbConstraints = size_constr;

  Solution_vec.resize(size_sol, false);
  ConstrLagr_vec.resize(size_constr, false);
  LBoundsLagr_vec.resize(size_sol, false);
  UBoundsLagr_vec.resize(size_sol, false);
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
     {aos<<Solution_vec[i]<<" ";}; aos<<std::endl;
}


void
QPProblem_s::dump_solver_parameters(std::ostream & aos)
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
      lnbrows = lnbcols = nbvariables_ ;
      array = Q_dense_.array_;
      Name = "Q";
      break;

    case MATRIX_DU:
      lnbrows = nbconstraints_+1;
      lnbcols = nbvariables_;
      array = DU_dense_.array_;
      Name = "DU";
      break;

    case VECTOR_D:
      lnbrows = nbvariables_;
      lnbcols = 1 ;
      array = D_.array_;
      Name = "D";
      break;

    case VECTOR_XL:
      lnbrows = nbvariables_;
      lnbcols = 1;
      array = XL_.array_;
      Name = "XL";
      break;

    case VECTOR_XU:
      lnbrows = nbvariables_;
      lnbcols=1;
      array = XU_.array_;
      Name = "XU";
      break;

    case VECTOR_DS:
      lnbrows = nbconstraints_+1;
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
QPProblem_s::dump_problem(std::ostream &aos)
{
  dump(MATRIX_Q,aos);
  dump(VECTOR_D,aos);

  dump(MATRIX_DU,aos);
  dump(VECTOR_DS,aos);

  dump(VECTOR_XL,aos);
  dump(VECTOR_XU,aos);

  dump_solver_parameters(aos);
}


void
QPProblem_s::dumpProblem(const char * filename)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  dump_problem(aof);
  aof.close();
}

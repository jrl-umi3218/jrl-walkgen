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
  m_(0),me_(0),mmax_(0), n_(0), nmax_(0), mnn_(0),
  iout_(0),ifail_(0), iprint_(0),
  lwar_(0), liwar_(0),
  Eps_(0),
  NbVariables_(0), NbConstraints_(0),NbEqConstraints_(0)
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
  bool ok=false;

  if ((NbConstraints_>0) && (NbVariables_>0)) 
    {
      DU_.resize(2*NbConstraints_, 2*NbVariables_,true);
      ok=true;
    }

  if (NbVariables_>0)
    {
      Q_.resize(2*NbVariables_, 2*NbVariables_,true);
      D_.resize(2*NbVariables_, 1,true);
      XL_.resize(2*NbVariables_, 1,true);
      XL_.fill(-1e8);
      XU_.resize(2*NbVariables_, 1,true);
      XU_.fill(1e8); 
      X_.resize(2*NbVariables_, 1,true);
      iwar_.resize(2*NbVariables_, 1,true);
      ok=true;
    }

  if (ok)
    {
      U_.resize(2*(NbConstraints_+2*NbVariables_), 1,true);
      war_.resize(2*(3*NbVariables_*NbVariables_/2+10*NbVariables_+2*(NbConstraints_+1)+20000), 1,true);
    }
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

      m_ = NbConstraints_;
      me_ = NbEqConstraints_;
      mmax_ = m_+1;
      n_ = NbVariables_;
      nmax_ = n_;
      mnn_ = m_+2*n_;

      iout_ = 0;
      iprint_ = 1;
      lwar_ = 3*nmax_*nmax_/2+ 10*nmax_  + 2*mmax_ + 20000;
      liwar_ = n_;
      Eps_ = 1e-8;

      //      if (m_FastFormulationMode==QLDANDLQ)
      //        m_Pb.iwar_.array_[0]=0;
      //      else
      iwar_.array_[0]=1;

      Q_.stick_together(Q_dense_.array_,n_,n_);
      DU_.stick_together(DU_dense_.array_,mmax_,n_);

      
      ql0001_(&m_, &me_, &mmax_, &n_, &nmax_, &mnn_,
              Q_dense_.array_, D_.array_, DU_dense_.array_, DS_.array_, XL_.array_, XU_.array_,
              X_.array_, U_.array_, &iout_, &ifail_, &iprint_,
              war_.array_, &lwar_, iwar_.array_, &liwar_, &Eps_);

      result.resize(n_,m_);


      for(int i = 0; i < n_; i++)
        {
          result.Solution_vec(i) = X_.array_[i];
          result.LBoundsLagr_vec(i) = U_.array_[m_+i];
          result.UBoundsLagr_vec(i) = U_.array_[m_+n_+i];
        }
      for(int i = 0; i < m_; i++)
        {
          result.ConstrLagr_vec(i) = U_.array_[i];
        }

      result.Fail = ifail_;
      result.Print = iprint_;

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
      NbVariables_ = (col+(int)Mat.size2()>NbVariables_) ? col+(int)Mat.size2() : NbVariables_;
      break;

    case MATRIX_DU:
      pArray_s = &DU_;
      NbConstraints_ = (row+(int)Mat.size1()>NbConstraints_) ? row+(int)Mat.size1() : NbConstraints_;
      NbVariables_ = (col+(int)Mat.size2()>NbVariables_) ? col+(int)Mat.size2() : NbVariables_;
      break;
    }

  std::cout << "NbVariables_:" << NbVariables_
	    << " NbConstraints_:" << NbConstraints_ 
	    << std::endl;
  if (NbVariables_ > pArray_s->ncols_ )
    {
      resize_all();
    }
  if (( NbConstraints_ > DU_.nrows_) &&
      (NbConstraints_>0))
    {
      if (NbVariables_>0)
	{
	  DU_.resize(2*NbConstraints_, 2*NbVariables_,true);
	}
    }
  
  if ((NbConstraints_ > DS_.nrows_ ) &&
      (NbConstraints_>0))
    {
      DS_.resize(2*NbConstraints_,1,true);
    }
  
  unsigned long int Usize = 2*(NbConstraints_+2*NbVariables_);
  if (Usize>U_.nrows_)
    {
      U_.resize(Usize, 1,true);
    }
  
  unsigned long int warsize = 2*(3*NbVariables_*NbVariables_/2+10*NbVariables_+2*(NbConstraints_+1)+20000);
  
  if (warsize> war_.nrows_)
    {
      war_.resize(warsize, 1,true);
    }

  double * p = pArray_s->array_;
  for( int i = 0;i < (int)MAL_MATRIX_NB_ROWS(Mat); i++)
    for( int j = 0;j < (int)MAL_MATRIX_NB_COLS(Mat); j++)
      {
        p[row+i+(col+j)*pArray_s->nrows_] += Mat(i,j);
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
      NbVariables_ = (row+(int)Vec.size()>NbVariables_) ? row+(int)Vec.size() : NbVariables_;
      break;

    case VECTOR_XL:
      pArray_s = &XL_;
      NbVariables_ = (row+(int)Vec.size()>NbVariables_) ? row+(int)Vec.size() : NbVariables_;
      break;

    case VECTOR_XU:
      pArray_s = &XU_;
      NbVariables_ = (row+(int)Vec.size()>NbVariables_) ? row+(int)Vec.size() : NbVariables_;
      break;

    case VECTOR_DS:
      pArray_s = &DS_;
      NbConstraints_ = (row+(int)Vec.size()>NbConstraints_) ? row+(int)Vec.size() : NbConstraints_;
      break;
    }

  if((NbVariables_ > D_.nrows_ ) && (NbVariables_>0))
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
  aos << "m: " << m_ << std::endl
      << "me: " << me_ << std::endl
      << "mmax: " << mmax_ << std::endl
      << "n: " << n_ << std::endl
      << "nmax: " << nmax_ << std::endl
      << "mnn: " << mnn_ << std::endl
      << "iout: " << iout_ << std::endl
      << "iprint: " << iprint_ << std::endl
      << "lwar: " << lwar_ << std::endl
      << "liwar: " << liwar_ << std::endl
      << "Eps: " << Eps_ << std::endl;
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
      lnbrows = lnbcols = NbVariables_ ;
      array = Q_dense_.array_;
      Name = "Q";
      break;

    case MATRIX_DU:
      lnbrows = NbConstraints_+1;
      lnbcols = NbVariables_;
      array = DU_dense_.array_;
      Name = "DU";
      break;

    case VECTOR_D:
      lnbrows = NbVariables_;
      lnbcols = 1 ;
      array = D_.array_;
      Name = "D";
      break;

    case VECTOR_XL:
      lnbrows = NbVariables_;
      lnbcols = 1;
      array = XL_.array_;
      Name = "XL";
      break;

    case VECTOR_XU:
      lnbrows = NbVariables_;
      lnbcols=1;
      array = XU_.array_;
      Name = "XU";
      break;

    case VECTOR_DS:
      lnbrows = NbConstraints_+1;
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

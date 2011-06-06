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
  eps_(0),
  NbVariables_(36), NbConstraints_(74),NbEqConstraints_(0)
{
  NbVariables_ = 36;
  NbConstraints_ = 74;

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
  eps_ = 1e-8;

  resize_all();
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
QPProblem_s::clear( int Type )
{

  switch(Type)
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


void QPProblem_s::reset()
{

  Q_.fill(0.0);
  Q_dense_.fill(0.0);
  DU_.fill(0.0);
  DU_dense_.fill(0.0);
  D_.fill(0.0);
  DS_.fill(0.0);
  NbConstraints_ = 0;
  NbEqConstraints_ = 0;
  NbVariables_ = 0;

}


void
QPProblem_s::solve( int Solver, solution_t & Result )
{
  switch(Solver)
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
      eps_ = 1e-8;

      //      if (m_FastFormulationMode==QLDANDLQ)
      //        m_Pb.iwar_.Array_[0]=0;
      //      else
      iwar_.Array_[0]=1;

      Q_.stick_together(Q_dense_,n_,n_);
      DU_.stick_together(DU_dense_,mmax_,n_);
      
      ql0001_(&m_, &me_, &mmax_, &n_, &nmax_, &mnn_,
              Q_dense_.Array_, D_.Array_, DU_dense_.Array_, DS_.Array_, XL_.Array_, XU_.Array_,
              X_.Array_, U_.Array_, &iout_, &ifail_, &iprint_,
              war_.Array_, &lwar_, iwar_.Array_, &liwar_, &eps_);

      Result.resize(n_,m_);

      for(int i = 0; i < n_; i++)
        {
          Result.Solution_vec(i) = X_.Array_[i];
          Result.LBoundsLagr_vec(i) = U_.Array_[m_+i];
          Result.UBoundsLagr_vec(i) = U_.Array_[m_+n_+i];
        }
      for(int i = 0; i < m_; i++)
        {
          Result.ConstrLagr_vec(i) = U_.Array_[i];
        }

      Result.Fail_ = ifail_;
      Result.Print_ = iprint_;
    }

}


void
QPProblem_s::add_term( const MAL_MATRIX (&Mat, double), int Type,
    unsigned int Row,  unsigned int Col )
{

  array_s<double> * Array_p = 0;

  switch(Type)
    {
    case MATRIX_Q:
      Array_p = &Q_;
      NbVariables_ = (Col+Mat.size2()>NbVariables_) ? Col+Mat.size2() : NbVariables_;
      break;

    case MATRIX_DU:
      Array_p = &DU_;
      NbConstraints_ = (Row+Mat.size1()>NbConstraints_) ? Row+Mat.size1() : NbConstraints_;
      NbVariables_ = (Col+Mat.size2()>NbVariables_) ? Col+Mat.size2() : NbVariables_;
      break;
    }


  if (NbVariables_ > Array_p->NbCols_ )
    {
      resize_all();
    }
  if ((NbConstraints_ > DU_.NbRows_) &&
      (NbConstraints_>0))
    {
      if (NbVariables_>0)
	{
	  DU_.resize(2*NbConstraints_, 2*NbVariables_,true);
	}
    }
  
  if ((NbConstraints_ > DS_.NbRows_ ) &&
      (NbConstraints_>0))
    {
      DS_.resize(2*NbConstraints_,1,true);
    }
  
  unsigned int USize = 2*(NbConstraints_+2*NbVariables_);
  if (USize>U_.NbRows_)
    {
      U_.resize(USize, 1,true);
    }
  
  unsigned int warsize = 2*(3*NbVariables_*NbVariables_/2+10*NbVariables_+2*(NbConstraints_+1)+20000);
  
  if (warsize> war_.NbRows_)
    {
      war_.resize(warsize, 1,true);
    }

  double * p = Array_p->Array_;
  for( unsigned int i = 0;i < Mat.size1(); i++)
    for( unsigned int j = 0;j < Mat.size2(); j++)
      {
        p[Row+i+(Col+j)*Array_p->NbRows_] += Mat(i,j);
      }


}


void QPProblem_s::add_term( const MAL_VECTOR (&Vec, double), int Type,
    unsigned int Row )
{

  array_s<double> * Array_p = 0;

  switch(Type)
    {
    case VECTOR_D:
      Array_p = &D_;
      NbVariables_ = (Row+Vec.size()>NbVariables_) ? Row+Vec.size() : NbVariables_;
      break;

    case VECTOR_XL:
      Array_p = &XL_;
      NbVariables_ = (Row+Vec.size()>NbVariables_) ? Row+Vec.size() : NbVariables_;
      break;

    case VECTOR_XU:
      Array_p = &XU_;
      NbVariables_ = (Row+Vec.size()>NbVariables_) ? Row+Vec.size() : NbVariables_;
      break;

    case VECTOR_DS:
      Array_p = &DS_;
      NbConstraints_ = (Row+Vec.size()>NbConstraints_) ? Row+Vec.size() : NbConstraints_;
      break;
    }

  if((NbVariables_ > D_.NbRows_ ) && (NbVariables_>0))
    {
      resize_all();
    }

  boost_ublas::vector<double>::const_iterator VecIt = Vec.begin();
  for( unsigned int i = 0; i < Vec.size(); i++ )
    {
      Array_p->Array_[Row+i] += *VecIt;
      VecIt++;
    }

}


void
QPProblem_s::solution_t::resize( unsigned int SizeSolution, unsigned int SizeConstraints )
{
  NbVariables_ = SizeSolution;
  NbConstraints_ = SizeConstraints;

  Solution_vec.resize(SizeSolution, false);
  ConstrLagr_vec.resize(SizeConstraints, false);
  LBoundsLagr_vec.resize(SizeSolution, false);
  UBoundsLagr_vec.resize(SizeSolution, false);
}


void
QPProblem_s::solution_t::dump(const char * FileName)
{
  std::ofstream aof;
  aof.open(FileName,std::ofstream::out);
  print(aof);
  aof.close();
}


void
QPProblem_s::solution_t::print(std::ostream & aos)
{
  aos << "Arrays:" << std::endl
      << "Solution: ";
  for( unsigned int i = 0; i < NbVariables_; i++ )
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
      << "Eps: " << eps_ << std::endl;
}


void
QPProblem_s::dump( int Type, std::ostream & aos)
{

  unsigned int NbRows=0, NbCols=0;
  double * Array=0;
  std::string Name;
  switch(Type)
    {
    case MATRIX_Q:
      NbRows = NbCols = NbVariables_ ;
      Array = Q_dense_.Array_;
      Name = "Q";
      break;

    case MATRIX_DU:
      NbRows = NbConstraints_+1;
      NbCols = NbVariables_;
      Array = DU_dense_.Array_;
      Name = "DU";
      break;

    case VECTOR_D:
      NbRows = NbVariables_;
      NbCols = 1 ;
      Array = D_.Array_;
      Name = "D";
      break;

    case VECTOR_XL:
      NbRows = NbVariables_;
      NbCols = 1;
      Array = XL_.Array_;
      Name = "XL";
      break;

    case VECTOR_XU:
      NbRows = NbVariables_;
      NbCols=1;
      Array = XU_.Array_;
      Name = "XU";
      break;

    case VECTOR_DS:
      NbRows = NbConstraints_+1;
      NbCols= 1;
      Array = DS_.Array_;
      Name = "DS";
      break;
    }

  aos << Name <<"["<<NbRows<< ","<< NbCols << "]" << std::endl;
  
  for(unsigned int i=0;i<NbRows;i++)
    {
      for(unsigned int j=0;j<NbCols;j++)
	aos << Array[i+j*NbRows] << " ";
      aos << std::endl;
    }
  aos << std::endl;
}


void
QPProblem_s::dump( int Type, const char * FileName )
{
  std::ofstream aof;
  aof.open(FileName,std::ofstream::out);
  dump(Type,aof);
  aof.close();
}


void
QPProblem_s::dump_problem( std::ostream &aos )
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
QPProblem_s::dump_problem( const char * FileName )
{
  std::ofstream aof;
  aof.open(FileName,std::ofstream::out);
  dump_problem(aof);
  aof.close();
}

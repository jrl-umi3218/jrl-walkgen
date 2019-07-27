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

#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>

#include <exception>

#include <ZMPRefTrajectoryGeneration/qp-problem.hh>

#ifdef LSSOL_FOUND
# include <lssol/lssol.h>
#endif //LSSOL_FOUND


using namespace PatternGeneratorJRL;




QPProblem::QPProblem():
  m_(0),me_(0),mmax_(0), n_(0), nmax_(0), mnn_(0),
  iout_(0),ifail_(0), iprint_(0),
  lwar_(0), liwar_(0), eps_(0),
  NbVariables_(0), NbConstraints_(0),NbEqConstraints_(0),
  nbInvariantRows_(0),nbInvariantCols_(0)

{
  NbVariables_ = 0;
  NbConstraints_ = 0;

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

  lastSolution_.resize(1,1);

  istate_       = 0x0;
  kx_          = 0x0;
  b_            = 0x0;
  clamda_       = 0x0;

  resize_all();

}


QPProblem::~QPProblem()
{
  release_memory();
}


void
QPProblem::release_memory()
{
}


void
QPProblem::resize_all()
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
      iwar_.resize(2*NbVariables_+1000, 1,true);
      ok=true;
    }

  if (ok)
    {
      U_.resize(2*(NbConstraints_+2*NbVariables_), 1,true);
      war_.resize(2*(3*NbVariables_*NbVariables_/2+10*NbVariables_+2*
                     (NbConstraints_+1)+20000), 1,true);
    }

  if (istate_!=0x0)
    {
      delete [] istate_;
      delete [] kx_;
      delete [] b_;
      delete [] clamda_;
    }

  istate_     = new int [(NbVariables_+NbConstraints_+2)*10];
  kx_          = new int [(NbVariables_+1)*10];
  b_            = new double [(NbVariables_+1)*10];
  clamda_       = new double [(NbVariables_+NbConstraints_+2)*10];

}


void
QPProblem::clear( qp_element_e Type )
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


void
QPProblem::reset()
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


int
QPProblem::reset_variant()
{

  unsigned firstRow = (nbInvariantRows_ < Q_.NbRows_) ? nbInvariantRows_ :
    Q_.NbRows_;
  unsigned firstCol = 0;
  unsigned lastCol = (nbInvariantCols_ < Q_.NbCols_) ? nbInvariantCols_ :
    Q_.NbCols_;
  if( (firstRow >= Q_.NbRows_) || (firstCol >= Q_.NbCols_) )
    return 0;

  double * p_it = 0;
  for( unsigned col = firstCol; col < lastCol; col++ )
    {
      p_it = &Q_.Array_[firstRow + col*Q_.NbRows_];
      for( unsigned i = firstRow; i < Q_.NbRows_; i++ )
        {
          *p_it = 0.0;
          ++p_it;
        }
    }

  firstRow = 0;
  firstCol = (nbInvariantCols_ < Q_.NbCols_) ? nbInvariantCols_ : Q_.NbCols_;
  lastCol = Q_.NbCols_;
  if( (firstRow >= Q_.NbRows_) || (firstCol >= Q_.NbCols_) )
    return 0;

  for( unsigned col = firstCol; col < lastCol; col++ )
    {
      p_it = &Q_.Array_[firstRow + col*Q_.NbRows_];
      for( unsigned i = firstRow; i < Q_.NbRows_; i++ )
        {
          *p_it = 0.0;
          ++p_it;
        }
    }

  Q_dense_.fill(0.0);
  DU_.fill(0.0);
  DU_dense_.fill(0.0);
  D_.fill(0.0);
  DS_.fill(0.0);
  NbConstraints_ = 0;
  NbEqConstraints_ = 0;
  NbVariables_ = 0;

  return 0;

}


void
QPProblem::solve( solver_e Solver, solution_t & Result, const tests_e & tests )
{

  m_ = NbConstraints_+1;
  me_ = NbEqConstraints_;
  mmax_ = m_+1;
  n_ = NbVariables_;
  nmax_ = n_;
  mnn_ = m_+2*n_;

  iout_ = 0;
  iprint_ = 1;
  lwar_ = 2*(3*NbVariables_*NbVariables_/2+10*NbVariables_+2*
             (NbConstraints_+1)+20000);
  liwar_ = 2*NbVariables_+1000;
  eps_ = 1e-8;


  iwar_.Array_[0]=1;

  Q_.stick_together(Q_dense_,n_,n_);
  DU_.stick_together(DU_dense_,mmax_,n_);


  Result.resize(n_,m_);

  switch(Solver)
    {
    case QLD:

      ql0001_(&m_, &me_, &mmax_, &n_, &nmax_, &mnn_,
              Q_dense_.Array_, D_.Array_, DU_dense_.Array_,
              DS_.Array_, XL_.Array_,
              XU_.Array_,
              X_.Array_, U_.Array_, &iout_, &ifail_, &iprint_,
              war_.Array_, &lwar_, iwar_.Array_, &liwar_, &eps_);


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

      Result.Fail = ifail_;
      Result.Print = iprint_;



      if (tests==ITT || tests==ALL)
        {
          int nb_itt_approx=0;
          for(int i = 0; i < m_; i++)
            {
              if (Result.ConstrLagr_vec(i)!=0)
                {
                  nb_itt_approx++;
                }
            }
          std::cout << "nb iterations : " << nb_itt_approx << std::endl;
        }

      break;
    case LSSOL:
#ifdef LSSOL_FOUND

      sendOption("Print Level = 0");

      sendOption("Problem Type = QP2");

      double *bl=new double[n_+m_];
      double *bu=new double[n_+m_];
      int size1=n_;
      for(int i=0; i<size1; ++i)
        {
          bl[i]=XL_.Array_[i];
          bu[i]=XU_.Array_[i];
        }
      int size2=size1+me_;
      for(int i=size1; i<size2; ++i)
        {
          bl[i]=-DS_.Array_[i-size1];
          bu[i]=bl[i];
        }
      int size3=size1+m_;
      for(int i=size2; i<size3; ++i)
        {
          bl[i]=-DS_.Array_[i-size1];
          bu[i]=10e10;
        }

      if (Result.useWarmStart)
        {
          for(unsigned i=0; i<NbVariables_; ++i)
            {
              X_.Array_[i]=Result.initialSolution(i);
            }
        }

      if (tests==CTR || tests==ALL)
        {
          // Check if initial solution respect all the constraints
          Eigen::MatrixXd DU(NbConstraints_, NbVariables_);
          Eigen::VectorXd XX(NbVariables_);
          Eigen::VectorXd DS(NbConstraints_);
          Eigen::VectorXd tmp;
          for(unsigned i=0; i<NbConstraints_; ++i)
            {
              for(unsigned j=0; j<NbVariables_; ++j)
                {
                  DU(i,j)=DU_.Array_[i+DU_.NbRows_*j];
                }
              DS(i)=DS_.Array_[i];
            }

          for(unsigned i=0; i<NbVariables_; ++i)
            {
              XX(i)=X_.Array_[i];
            }

          tmp=prod(DU,XX);
          int nb_ctr=0;
          for(unsigned i=0; i<NbConstraints_; ++i)
            {
              if (tmp(i)+DS(i)<-1e-6)
                {
                  std::cout << "Unrespected constraint " << i
                            << " : " << tmp(i) << " < "
                            << -DS(i)  << std::endl;
                  ++nb_ctr;
                }
            }
          std::cout << std::endl << "Nb unrespected constraints : " << nb_ctr <<
            std::endl;
        }


      lssol_(&n_, &n_,
             &m_, &mmax_, &n_,
             DU_dense_.Array_, bl, bu, D_.Array_,
             istate_, kx_, X_.Array_, Q_dense_.Array_, b_,
             &inform_, &iter_, &obj_, clamda_,
             iwar_.Array_, &liwar_, war_.Array_, &lwar_);

      lastSolution_.resize(n_);
      for(int i = 0; i < n_; i++)
        {
          Result.Solution_vec(i) = X_.Array_[i];
          lastSolution_(i)=Result.Solution_vec(i);
          Result.LBoundsLagr_vec(i) = 0;
          Result.UBoundsLagr_vec(i) = 0;
        }
      Result.Fail=0;
      Result.Print = 0;
      for(int i = 0; i < m_; i++)
        {
          Result.ConstrLagr_vec(i) = 0;
        }



      if (tests==ITT || tests==ALL)
        {
          std::cout << "nb iterations : " << iter_ << std::endl;
        }

#else
      std::cerr << " LSSOL_FOUND not available" << std::endl;
#endif //LSSOL_FOUND

      break;

    }

}


void
QPProblem::add_term_to( qp_element_e Type, const Eigen::MatrixXd &Mat,
                        unsigned int row,  unsigned int col )
{

  array_s<double> * Array_p = 0;

  switch(Type)
    {
    case MATRIX_Q:
      Array_p = &Q_;
      NbVariables_ =
        (unsigned int)((col+Mat.cols()>NbVariables_) ?
                       col+Mat.cols() :
                       NbVariables_);
      break;

    case MATRIX_DU:
      Array_p = &DU_;
      NbConstraints_ = (unsigned int)(( row+Mat.rows() > NbConstraints_ ) ? row
                                      +Mat.rows() : NbConstraints_);
      NbVariables_ = (unsigned int)(( col+Mat.cols() > NbVariables_ ) ?
                                    col+Mat.cols()
                                    : NbVariables_);
      row++;//The first rows of DU,DS are empty
      break;

    case VECTOR_D:
      break;
    case VECTOR_XL:
      break;
    case VECTOR_XU:
      break;
    case VECTOR_DS:
      Array_p = &DS_;
      NbConstraints_ = (unsigned int)( (row+Mat.rows()>NbConstraints_) ? row
                                       +Mat.rows() : NbConstraints_);
      row++;//The first rows of DU,DS are empty
      break;
    }


  if( NbVariables_ > Array_p->NbCols_ )
    resize_all();

  if( (NbConstraints_ > DU_.NbRows_) && (NbConstraints_>0) )
    {
      if( NbVariables_>0 )
        DU_.resize( 2*NbConstraints_, 2*NbVariables_, true );
    }

  if( (NbConstraints_ > DS_.NbRows_ ) && (NbConstraints_>0) )
    DS_.resize(2*NbConstraints_,1,true);

  if( U_.NbRows_ < NbConstraints_+2*NbVariables_ )
    U_.resize( NbConstraints_+2*NbVariables_, 1, true );

  unsigned warsize = 3*NbVariables_*NbVariables_/2+10*NbVariables_+2*
    (NbConstraints_+1)+20000;
  if( warsize> war_.NbRows_ )
    war_.resize( warsize, 1, true );

  double * p = Array_p->Array_;
  double * p_it = &p[row+(col)*Array_p->NbRows_];
  for( unsigned j = 0; j < Mat.cols(); j++ )
    {
      p_it = &p[row+(col+j)*Array_p->NbRows_];
      for( unsigned i = 0; i < Mat.rows(); i++ )
        {
          *p_it += Mat(i,j);
          ++p_it;
        }
    }

}


void
QPProblem::add_term_to( qp_element_e Type, const Eigen::VectorXd &Vec,
                        unsigned row, unsigned col )
{

  array_s<double> * Array_p = 0;

  switch(Type)
    {
    case VECTOR_D:
      Array_p = &D_;
      NbVariables_ = (unsigned int)((row+Vec.size()>NbVariables_) ?
                                    row+Vec.size() :
                                    NbVariables_);
      break;

    case VECTOR_XL:
      Array_p = &XL_;
      NbVariables_ = (unsigned int)((row+Vec.size()>NbVariables_) ?
                                    row+Vec.size() :
                                    NbVariables_);
      break;

    case VECTOR_XU:
      Array_p = &XU_;
      NbVariables_ = (unsigned int)((row+Vec.size()>NbVariables_) ?
                                    row+Vec.size() :
                                    NbVariables_);
      break;

    case VECTOR_DS:
      Array_p = &DS_;
      NbConstraints_ = (unsigned int)((row+Vec.size()>NbConstraints_) ?
                                      row+Vec.size()
                                      : NbConstraints_);
      row++;//The first rows of DU,DS are empty
      break;

    case MATRIX_DU:
      Array_p = &DU_;
      NbConstraints_ = (row+1>NbConstraints_) ? row+1 : NbConstraints_;
      break;
    case MATRIX_Q:
      break;
    }

  if( (NbVariables_ > D_.NbRows_ ) && (NbVariables_>0) )
    resize_all();

  double * p_it = &Array_p->Array_[row+col*Array_p->NbRows_];
  if(Type == MATRIX_DU)
    {
      for( unsigned i = 0; i < Vec.size(); i++ )
        {
          *p_it += Vec(i);
          ++p_it;
        }
    }
  else
    {
      for( unsigned i = 0; i < Vec.size(); i++ )
        {
          *p_it += Vec(i);
          ++p_it;
        }
    }

}


void
QPProblem::dump_solver_parameters(std::ostream & aos)
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
QPProblem::dump( qp_element_e Type, std::ostream & aos)
{

  unsigned int NbRows=0, NbCols=0;
  double * Array=0;
  std::string Name;
  switch(Type)
    {
    case MATRIX_Q:
      NbRows = NbCols = Q_dense_.NbCols_ ;
      Array = Q_dense_.Array_;
      Name = "Q";
      break;

    case MATRIX_DU:
      NbRows = DU_dense_.NbRows_;
      NbCols = DU_dense_.NbCols_;
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
      NbRows = DS_.NbRows_;
      NbCols= 1;
      Array = DS_.Array_;
      Name = "DS";
      break;
    }
  aos << Name <<"["<<NbRows<< ","<< NbCols << "]" << std::endl;

  for(unsigned int i=0; i<NbRows; i++)
    {
      for(unsigned int j=0; j<NbCols; j++)
        aos << std::scientific << Array[i+j*NbRows] << " ";
      aos << std::endl;
    }
  aos << std::endl;
}


void
QPProblem::dump( qp_element_e Type, const char * FileName )
{
  std::ofstream aof;
  aof.open(FileName,std::ofstream::out);
  dump(Type,aof);
  aof.close();
}


void
QPProblem::dump_problem( std::ostream &aos )
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
QPProblem::dump( const char * FileName )
{
  std::ofstream aof;
  aof.open(FileName,std::ofstream::out);
  dump_problem(aof);
  aof.close();
}


void
QPProblem::dump( double Time )
{
  char Buffer[1024];
  sprintf(Buffer, "/tmp/Problem_%f.dat", Time);
  std::ofstream aof;
  aof.open(Buffer, std::ofstream::out);
  dump_problem(aof);
  aof.close();
}

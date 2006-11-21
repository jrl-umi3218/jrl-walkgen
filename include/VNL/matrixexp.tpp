#ifndef vnl_matrix_exp_t_
#define vnl_matrix_exp_t_
// This is vxl/VNL/matrix_exp.t

/*
  fsm@robots.ox.ac.uk
*/
#include "matrixexp.h"
#include <assert.h>
#include <cmath>
//#include <iostream>

template <class T>
bool VNL::MatrixExp(VNL::Matrix<T> const &X, VNL::Matrix<T> &expX, double max_err)
{
  unsigned N = X.Rows();
  X.assert_size(N, N);
  expX.assert_size(N, N);

  double norm_X = X.operatorInfNorm();
  //std::cerr << "norm_X = " << norm_X << std::endl;

  // exponential series
  expX.SetIdentity();
  VNL::Matrix<T> acc(X);
  double norm_acc_bound = norm_X;
  for (unsigned n=1; true; ++n) {
    expX += acc;
#ifdef DEBUG
    std::cerr << "n=" << n << std::endl;
#endif

    if (norm_X < n) {
      double err_bound = norm_acc_bound / (1 - norm_X/n);
#ifdef DEBUG
      std::cerr << "err_bound = " << err_bound << std::endl;
#endif
      if (err_bound < max_err)
        break;
    }

    acc = acc * X;
    acc /= n+1;

    norm_acc_bound *= norm_X/(n+1);
  }

  return true;
}

template <class T>
VNL::Matrix<T> VNL::MatrixExp(VNL::Matrix<T> const &X)
{
  VNL::Matrix<T> expX(X.Rows(), X.Cols());
#ifndef NDEBUG
  bool retval = 
#endif
  VNL::MatrixExp(X, expX, 1e-10);

  assert(retval);
  return expX;
}

//--------------------------------------------------------------------------------

#undef VNL_MATRIX_EXP_INSTANTIATE
#define VNL_MATRIX_EXP_INSTANTIATE(T) \
namespace VNL { \
template bool MatrixExp(VNL::Matrix<T > const &, VNL::Matrix<T > &, double); \
template VNL::Matrix<T > MatrixExp(VNL::Matrix<T > const &); \
};

#endif

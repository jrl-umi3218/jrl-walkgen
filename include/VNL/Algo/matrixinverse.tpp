// This is vxl/VNL/algo/matrix_inverse.t
#ifndef vnl_matrix_inverse_t_
#define vnl_matrix_inverse_t_
// Author: Andrew W. Fitzgibbon, Oxford RRG
// Created: 22 Nov 96
//
//-----------------------------------------------------------------------------

#include "matrixinverse.h"

#undef VNL_MATRIX_INVERSE_INSTANTIATE
#define VNL_MATRIX_INVERSE_INSTANTIATE(T) \
namespace VNL {template struct MatrixInverse<T >;};\
template VNL::Vector<T > operator*(VNL::MatrixInverse<T > const &, VNL::Vector<T > const &); \
template VNL::Matrix<T > operator*(VNL::MatrixInverse<T > const &, VNL::Matrix<T > const &)

#endif // vnl_matrix_inverse_t_

// This is vxl/VNL/matrix_fixed.t
#ifndef vnl_matrix_fixed_t_
#define vnl_matrix_fixed_t_

#include "matrixfixed.h"

#define VNL_MATRIX_FIXED_INSTANTIATE(M, N, T) \
namespace VNL { \
template class MatrixFixed<M ,N, T >; \
};




# undef VNL_MATRIX_FIXED_PAIR_INSTANTIATE
#if !defined(VCL_SUNPRO_CC) && !defined(_WIN32)
# define VNL_MATRIX_FIXED_PAIR_INSTANTIATE(M, N, O, T ) \
  template VNL::MatrixFixed<M, O, T > operator*(const VNL::MatrixFixed<M, N, T >& a, const VNL::MatrixFixed<N, O, T >& b)
#else
# define VNL_MATRIX_FIXED_PAIR_INSTANTIATE(M, N, O, T ) /* */
#endif

#endif // vnl_matrix_fixed_t_

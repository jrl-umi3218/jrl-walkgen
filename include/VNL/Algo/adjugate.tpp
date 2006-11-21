#ifndef vnl_adjugate_t_
#define vnl_adjugate_t_
/*
  fsm@robots.ox.ac.uk
*/
#include "adjugate.h"
#include <VNL/matrix.h>
#include <VNL/Algo/determinant.h>

// This is a rudimentary implementation. It could be improved by noting
// that adj(A B) = adj(B) adj(A) for all matrices A, B (invertible or
// not) and then using a matrix decomposition for larger matrices.
//
// E.g. using a singular value decomposition A = U D V^* gives
// adj(A) = V adj(D) U^*.

template <class T>
void VNL::Adjugate(VNL::Matrix<T> const &A, VNL::Matrix<T> *out)
{
  int n = A.Rows();
  A.assert_size(n, n);
  out->assert_size(n, n);

  VNL::Matrix<T> sub(n-1, n-1);
  for (int i=0; i<n; ++i) {
    for (int j=0; j<n; ++j) {
      for (int u=1; u<n; ++u)
        for (int v=1; v<n; ++v)
          sub[u-1][v-1] = A[(i+u)%n][(j+v)%n];
      (*out)[j][i] = VNL::Determinant(sub);
    }
  }
}

template <class T>
VNL::Matrix<T> VNL::Adjugate(VNL::Matrix<T> const &A)
{
  VNL::Matrix<T> adj(A.Rows(), A.Columns());
  VNL::Adjugate(A, &adj);
  return adj;
}

//--------------------------------------------------------------------------------

#undef VNL_ADJUGATE_INSTANTIATE
#define VNL_ADJUGATE_INSTANTIATE(T) \
namespace VNL { \
  template void       Adjugate(Matrix<T > const &, Matrix<T > *); \
  template Matrix<T > Adjugate(Matrix<T > const &); \
};
#endif

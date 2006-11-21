#ifndef vnl_algo_determinant_t_
#define vnl_algo_determinant_t_
/*
  fsm@robots.ox.ac.uk
*/
#include "determinant.h"

#include <assert.h>
#include <VNL/Algo/qr.h>


template <class T>
T VNL::Determinant(T const *row0, T const *row1) {
  return row0[0]*row1[1] - row0[1]*row1[0];
}

template <class T>
T VNL::Determinant(T const *row0, T const *row1, T const *row2) {
  return // the extra '+' makes it work nicely with emacs indentation.
    + row0[0]*row1[1]*row2[2]
    - row0[0]*row2[1]*row1[2]
    - row1[0]*row0[1]*row2[2]
    + row1[0]*row2[1]*row0[2]
    + row2[0]*row0[1]*row1[2]
    - row2[0]*row1[1]*row0[2];
}

template <class T>
T VNL::Determinant(T const *row0, T const *row1, T const *row2, T const *row3) {
  return
    + row0[0]*row1[1]*row2[2]*row3[3]
    - row0[0]*row1[1]*row3[2]*row2[3]
    - row0[0]*row2[1]*row1[2]*row3[3]
    + row0[0]*row2[1]*row3[2]*row1[3]
    + row0[0]*row3[1]*row1[2]*row2[3]
    - row0[0]*row3[1]*row2[2]*row1[3]
    - row1[0]*row0[1]*row2[2]*row3[3]
    + row1[0]*row0[1]*row3[2]*row2[3]
    + row1[0]*row2[1]*row0[2]*row3[3]
    - row1[0]*row2[1]*row3[2]*row0[3]
    - row1[0]*row3[1]*row0[2]*row2[3]
    + row1[0]*row3[1]*row2[2]*row0[3]
    + row2[0]*row0[1]*row1[2]*row3[3]
    - row2[0]*row0[1]*row3[2]*row1[3]
    - row2[0]*row1[1]*row0[2]*row3[3]
    + row2[0]*row1[1]*row3[2]*row0[3]
    + row2[0]*row3[1]*row0[2]*row1[3]
    - row2[0]*row3[1]*row1[2]*row0[3]
    - row3[0]*row0[1]*row1[2]*row2[3]
    + row3[0]*row0[1]*row2[2]*row1[3]
    + row3[0]*row1[1]*row0[2]*row2[3]
    - row3[0]*row1[1]*row2[2]*row0[3]
    - row3[0]*row2[1]*row0[2]*row1[3]
    + row3[0]*row2[1]*row1[2]*row0[3];
}

//--------------------------------------------------------------------------------

template <class T>
T VNL::Determinant(VNL::Matrix<T> const &M, bool balance)
{
  unsigned n = M.Rows();
  assert(M.Columns() == n);

  switch (n) {
  case 1: return M[0][0];
  case 2: return VNL::Determinant(M[0], M[1]);
  case 3: return VNL::Determinant(M[0], M[1], M[2]);
  case 4: return VNL::Determinant(M[0], M[1], M[2], M[3]);
  default:
    if (balance) {
      VNL::Matrix<T> tmp(M);
      typedef typename VNL::NumericTraits<T>::abs_t abs_t;
      abs_t scalings(1);
      for (int t=0; t<5; ++t) {
#if 1
        // normalize rows.
        for (unsigned int i=0; i<n; ++i) {
          abs_t rn = tmp.GetRow(i).RMS();
          if (rn > 0) {
            scalings *= rn;
            tmp.ScaleRow(i, abs_t(1)/rn);
          }
        }
#endif
#if 1
        // normalize columns.
        {for (unsigned int i=0; i<n; ++i) {
          abs_t rn = tmp.GetColumn(i).RMS();
          if (rn > 0) {
            scalings *= rn;
            tmp.ScaleColumn(i, abs_t(1)/rn);
          }
        }}
#endif
#if 0
        // pivot
        for (int k=0; k<n-1; ++k) {
          // find largest element after (k, k):
          int i0 = k, j0 = k;
          abs_t v0(0);
          for (int i=k; i<n; ++i) {
            for (int j=k; j<n; ++j) {
              abs_t v = std::abs(tmp[i][j]);
              if (v > v0) {
                i0 = i;
                j0 = j;
                v0 = v;
              }
            }
          }
          // largest element is in position (i0, j0).
          if (i0 != k) {
            for (int j=0; j<n; ++j)
              std::swap(tmp[k][j], tmp[i0][j]);
            scalings = -scalings;
          }
          if (j0 != k) {
            for (int i=0; i<n; ++i)
              std::swap(tmp[i][k], tmp[i][j0]);
            scalings = -scalings;
          }
        }
#endif
      }
      T balanced_det = VNL::QR<T>(tmp).Determinant();
      //std::clog << __FILE__ ": scalings, balanced_det = " << scalings << ", " << balanced_det << std::endl;
      return T(scalings) * balanced_det;
    }
    else
      return VNL::QR<T>(M).Determinant();
  }
}

//--------------------------------------------------------------------------------

#define VNL_DETERMINANT_INSTANTIATE_1(T) \
namespace VNL {\
template T Determinant(T const *, T const *); \
template T Determinant(T const *, T const *, T const *); \
template T Determinant(T const *, T const *, T const *, T const *); \
};

#define VNL_DETERMINANT_INSTANTIATE_2(T) \
namespace VNL {template T Determinant(VNL::Matrix<T > const &, bool);};

#undef VNL_DETERMINANT_INSTANTIATE
#define VNL_DETERMINANT_INSTANTIATE(T) \
VNL_DETERMINANT_INSTANTIATE_1(T); \
VNL_DETERMINANT_INSTANTIATE_2(T)

#endif // vnl_algo_determinant_t_

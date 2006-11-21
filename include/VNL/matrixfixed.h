// This is vxl/VNL/matrix_fixed.h
#ifndef vnl_matrix_fixed_h_
#define vnl_matrix_fixed_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief fixed size matrix

*
* A subclass of vnl_matrix_fixed_ref,
* all storage is local and all vnl_matrix operations are valid.
*
* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   04 Aug 96
*
   \verbatim
   Modifications:
   Peter Vanroose, 23 Nov 1996:  added explicit copy constructor
   LSB (Manchester) 15/03/2001:  added Binary I/O and tidied up the documentation
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*/

#include <assert.h>
#include <VNL/matrixfixedref.h>
#include <VNL/vectorfixed.h>


namespace VNL
{
/** Fixed size matrix.
*  A subclass of vnl_matrix_fixed_ref,
*  all storage is local and all vnl_matrix operations are valid.
*/

template <int m, int n, class T>
class MatrixFixed : public MatrixFixedRef<m,n,T>
{
  T space[m*n]; // Local storage
 public:

/** Construct an empty m*n matrix.
*/
  MatrixFixed() : MatrixFixedRef<m,n,T>(space) {}

/** Construct an m*n matrix and fill with value.
*/
  MatrixFixed(const T& value):MatrixFixedRef<m,n,T>(space) {
    int i = m*n;
    while (i--)
      space[i] = value;
  }

/** Construct an m*n Matrix and copy data into it row-wise.
*/
  MatrixFixed(const T* datablck) : MatrixFixedRef<m,n,T>(space) {
    memcpy(space, datablck, m*n*sizeof(T));
  }

/** Construct an m*n Matrix and copy rhs into it.
*  Abort if rhs is not the same size.
*/
  MatrixFixed(const Matrix<T>& rhs) : MatrixFixedRef<m,n,T>(space) {
    assert(rhs.Rows() == m && rhs.Columns() == n);
    memcpy(space, rhs.DataBlock(), m*n*sizeof(T));
  }

  //  Destruct the m*n matrix.
  // An explicit destructor seems to be necessary, at least for gcc 3.0.0,
  // to avoid the compiler generating multiple versions of it.
  // (This way, a weak symbol is generated; otherwise not.  A bug of gcc 3.0.)
  ~MatrixFixed() {}

/** Copy a vnl_matrix into this.
*  Abort if rhs is not the same size.
*/
  MatrixFixed<m,n,T>& operator=(const Matrix<T>& rhs) {
    assert(rhs.Rows() == m && rhs.Columns() == n);
    memcpy(space, rhs.DataBlock(), m*n*sizeof(T));
    return *this;
  }

/** Copy another MatrixFixed<m,n,T> into this.
*/
  MatrixFixed<m,n,T>& operator=(const MatrixFixed<m, n, T>& rhs) {
    memcpy(space, rhs.DataBlock(), m*n*sizeof(T));
    return *this;
  }

  MatrixFixed(const MatrixFixed<m,n,T>& rhs) : MatrixFixedRef<m,n,T>(space) {
    memcpy(space, rhs.DataBlock(), m*n*sizeof(T));
  }
};

}; // End namespace VNL

/** Multiply two conformant MatrixFixed (M x N) times (N x O).
*/
template <int M, int N, int O, class T>
VNL::MatrixFixed<M, O, T> operator*(const VNL::MatrixFixed<M, N, T>& a, const VNL::MatrixFixed<N, O, T>& b)
{
  VNL::MatrixFixed<M, O, T> out;
  for (int i = 0; i < M; ++i)
    for (int j = 0; j < O; ++j) {
      T accum = a(i,0) * b(0,j);
      for (int k = 1; k < N; ++k)
        accum += a(i,k) * b(k,j);
      out(i,j) = accum;
    }
  return out;
}


// BJT: For some reason, including VNL/vectorfixed.h here crashes compiler on Win32


/** Multiply MatrixFixed (M x N) by a scalar.
*/
template <int M, int N, class T>
VNL::MatrixFixed<M, N, T> operator*(const VNL::MatrixFixed<M, N, T>& a, 
				    const double b)
{
  VNL::MatrixFixed<M, N, T> out;
  for (int i = 0; i < M; ++i)
    for (int j = 0; j < N; ++j) {
      out(i,j) = a(i,j) * b;
    }
  return out;
}

/** Multiply  conformant MatrixFixed (M x N) and vector_fixed (N).
*/
template <int M, int N, class T>
VNL::VectorFixed<M, T> operator*(const VNL::MatrixFixed<M, N, T>& a, const VNL::VectorFixed<N, T>& b)
{
  VNL::VectorFixed<M, T> out;
  for (int i = 0; i < M; ++i) {
    T accum = a(i,0) * b(0);
    for (int k = 1; k < N; ++k)
      accum += a(i,k) * b(k);
    out(i) = accum;
  }
  return out;
}


// BJT: Partially specialise the 2x2, 3x3 and 4x4 cases (doing this in the
// tpp or inst file doesn't seem to work :-(
// This gives roughly 25% speed increase for g++, yet to be tested on windows.
template <class T>
VNL::MatrixFixed<2, 2, T> operator*(const VNL::MatrixFixed<2, 2, T>& a, const VNL::MatrixFixed<2, 2, T>& b)
{
  VNL::MatrixFixed<2, 2, T> out;
  out[0][0] = a[0][0]*b[0][0] + a[0][1]*b[1][0];
  out[0][1] = a[0][0]*b[0][1] + a[0][1]*b[1][1];

  out[1][0] = a[1][0]*b[0][0] + a[1][1]*b[1][0];
  out[1][1] = a[1][0]*b[0][1] + a[1][1]*b[1][1];
  return out;
}
template <class T>
VNL::MatrixFixed<3, 3, T> operator*(const VNL::MatrixFixed<3, 3, T>& a, const VNL::MatrixFixed<3, 3, T>& b)
{
  VNL::MatrixFixed<3, 3, T> out;
  out[0][0] = a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0];
  out[0][1] = a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1];
  out[0][2] = a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2];

  out[1][0] = a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0];
  out[1][1] = a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1];
  out[1][2] = a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2];

  out[2][0] = a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0];
  out[2][1] = a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1];
  out[2][2] = a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2];
  return out;
}
template <class T>
VNL::MatrixFixed<4, 4, T> operator*(const VNL::MatrixFixed<4, 4, T>& a, const VNL::MatrixFixed<4, 4, T>& b)
{
  VNL::MatrixFixed<4, 4, T> c;
  c[0][0] = a[0][0]*b[0][0]+a[0][1]*b[1][0]+a[0][2]*b[2][0]+a[0][3]*b[3][0];
  c[0][1] = a[0][0]*b[0][1]+a[0][1]*b[1][1]+a[0][2]*b[2][1]+a[0][3]*b[3][1];
  c[0][2] = a[0][0]*b[0][2]+a[0][1]*b[1][2]+a[0][2]*b[2][2]+a[0][3]*b[3][2];
  c[0][3] = a[0][0]*b[0][3]+a[0][1]*b[1][3]+a[0][2]*b[2][3]+a[0][3]*b[3][3];

  c[1][0] = a[1][0]*b[0][0]+a[1][1]*b[1][0]+a[1][2]*b[2][0]+a[1][3]*b[3][0];
  c[1][1] = a[1][0]*b[0][1]+a[1][1]*b[1][1]+a[1][2]*b[2][1]+a[1][3]*b[3][1];
  c[1][2] = a[1][0]*b[0][2]+a[1][1]*b[1][2]+a[1][2]*b[2][2]+a[1][3]*b[3][2];
  c[1][3] = a[1][0]*b[0][3]+a[1][1]*b[1][3]+a[1][2]*b[2][3]+a[1][3]*b[3][3];

  c[2][0] = a[2][0]*b[0][0]+a[2][1]*b[1][0]+a[2][2]*b[2][0]+a[2][3]*b[3][0];
  c[2][1] = a[2][0]*b[0][1]+a[2][1]*b[1][1]+a[2][2]*b[2][1]+a[2][3]*b[3][1];
  c[2][2] = a[2][0]*b[0][2]+a[2][1]*b[1][2]+a[2][2]*b[2][2]+a[2][3]*b[3][2];
  c[2][3] = a[2][0]*b[0][3]+a[2][1]*b[1][3]+a[2][2]*b[2][3]+a[2][3]*b[3][3];

  c[3][0] = a[3][0]*b[0][0]+a[3][1]*b[1][0]+a[3][2]*b[2][0]+a[3][3]*b[3][0];
  c[3][1] = a[3][0]*b[0][1]+a[3][1]*b[1][1]+a[3][2]*b[2][1]+a[3][3]*b[3][1];
  c[3][2] = a[3][0]*b[0][2]+a[3][1]*b[1][2]+a[3][2]*b[2][2]+a[3][3]*b[3][2];
  c[3][3] = a[3][0]*b[0][3]+a[3][1]*b[1][3]+a[3][2]*b[2][3]+a[3][3]*b[3][3];

  return c;
}
template <class T>
VNL::VectorFixed<2, T> operator*(const VNL::MatrixFixed<2, 2, T>& a, const VNL::VectorFixed<2, T>& b)
{
  VNL::VectorFixed<2, T> out;
  out[0] = a[0][0]*b[0] + a[0][1]*b[1];
  out[1] = a[1][0]*b[0] + a[1][1]*b[1];
  return out;
}
template <class T>
VNL::VectorFixed<3, T> operator*(const VNL::MatrixFixed<3, 3, T>& a, const VNL::VectorFixed<3, T>& b)
{
  VNL::VectorFixed<3, T> out;
  out[0] = a[0][0]*b[0] + a[0][1]*b[1] + a[0][2]*b[2];
  out[1] = a[1][0]*b[0] + a[1][1]*b[1] + a[1][2]*b[2];
  out[2] = a[2][0]*b[0] + a[2][1]*b[1] + a[2][2]*b[2];
  return out;
}
template <class T>
VNL::VectorFixed<4, T> operator*(const VNL::MatrixFixed<4, 4, T>& a, const VNL::VectorFixed<4, T>& b)
{
  VNL::VectorFixed<4, T> out;
  out[0] = a[0][0]*b[0] + a[0][1]*b[1] + a[0][2]*b[2] + a[0][3]*b[3];
  out[1] = a[1][0]*b[0] + a[1][1]*b[1] + a[1][2]*b[2] + a[1][3]*b[3];
  out[2] = a[2][0]*b[0] + a[2][1]*b[1] + a[2][2]*b[2] + a[2][3]*b[3];
  out[3] = a[3][0]*b[0] + a[3][1]*b[1] + a[3][2]*b[2] + a[3][3]*b[3];
  return out;
}



#define VNL_MATRIX_FIXED_PAIR_INSTANTIATE(M, N, O, T) \
extern "please include VNL/matrix_fixed.t instead"



#endif // vnl_matrix_fixed_h_

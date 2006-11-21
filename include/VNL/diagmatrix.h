// This is vxl/VNL/diag_matrix.h
#ifndef _vnl_diag_matrix_h_
#define _vnl_diag_matrix_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Contains class for diagonal matrices

*  \author Andrew W. Fitzgibbon (Oxford RRG)
*  \date   5/8/96
*
   \verbatim
    Modifications
    IMS (Manchester) 16/03/2001: Tidied up the documentation + added binary_io
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*/

#include <assert.h>
#include <VNL/vector.h>
#include <VNL/matrix.h>

namespace VNL {

  // Export not yet supported - uncomment this when it is
  //export template <class T> class DiagMatrix;

/** stores a diagonal matrix as a single vector.
*  DiagMatrix stores a diagonal matrix for time and space efficiency.
*  Specifically, only the diagonal elements are stored, and some matrix
*  operations (currently *, + and -) are overloaded to use more efficient
*  algorithms.
*/
template <class T>
class DiagMatrix
{
 public:
  DiagMatrix() {}

/** Construct an empty diagonal matrix.
*/
  DiagMatrix(unsigned nn) : diagonal_(nn) {}

/** Construct a diagonal matrix with diagonal elements equal to value.
*/
  DiagMatrix(unsigned nn, T const& value) : diagonal_(nn, value) {}

/** Construct a diagonal matrix from a vnl_vector.
*  The vector elements become the diagonal elements.
*/
  DiagMatrix(Vector<T> const& that): diagonal_(that) {}
 ~DiagMatrix() {}

  DiagMatrix& operator=(DiagMatrix<T> const& that) {
  this->diagonal_ = that.diagonal_;
  return *this;
    }

  // Operations----------------------------------------------------------------

/** In-place arithmetic operations.
*/
  DiagMatrix<T>& operator*=(T v) { diagonal_ *= v; return *this; }
/** In-place arithmetic operations.
*/
  DiagMatrix<T>& operator/=(T v) { diagonal_ /= v; return *this; }

  // Computations--------------------------------------------------------------

  void InvertInPlace();
  T Determinant() const;
  Vector<T> Solve(Vector<T> const& b);
  void Solve(Vector<T> const& b, Vector<T>* out);

  // Data Access---------------------------------------------------------------

  T operator () (unsigned i, unsigned j) const {
    return (i != j) ? T(0) : diagonal_[i];
  }

  T& operator () (unsigned i, unsigned j) {
    assert(i == j);
    return diagonal_[i];
  }
  T& operator() (unsigned i) { return diagonal_[i]; }
  T const& operator() (unsigned i) const { return diagonal_[i]; }

  T& operator[] (unsigned i) { return diagonal_[i]; }
  T const& operator[] (unsigned i) const { return diagonal_[i]; }

  // STL style iterators
  typedef typename Vector<T>::iterator iterator;
  inline iterator begin() { return diagonal_.begin(); }
  inline iterator end() { return diagonal_.end(); }
  typedef typename Vector<T>::const_iterator const_iterator;
  inline const_iterator begin() const { return diagonal_.begin(); }
  inline const_iterator end() const { return diagonal_.end(); }
  unsigned size() const { return diagonal_.size(); }

  unsigned Size() const { return diagonal_.size(); }
  unsigned Rows() const { return diagonal_.size(); }
  unsigned Cols() const { return diagonal_.size(); }
  unsigned Columns() const { return diagonal_.size(); }

  // Need this until we add a DiagMatrix ctor to Matrix;
  inline Matrix<T> AsMatrix() const;

  void Resize(int n) { diagonal_.Resize(n); }
  void Clear() { diagonal_.Clear(); }
  void Fill(T const &x) { diagonal_.Fill(x); }

/** Return pointer to the diagonal elements as a contiguous 1D C array;.
*/
  T*       DataBlock()       { return diagonal_.DataBlock(); }
  T const* DataBlock() const { return diagonal_.DataBlock(); }

/** Return diagonal elements as a vector.
*/
  Vector<T> const& Diagonal() const { return diagonal_; }

/** Set diagonal elements using vector.
*/
  void Set(Vector<T> const& v)  { diagonal_=v; }

 protected:
  Vector<T> diagonal_;

 private:

  #if VCL_NEED_FRIEND_FOR_TEMPLATE_OVERLOAD
  friend Vector<T> operator*(DiagMatrix<T> const&,Vector<T> const&);
  #endif
};



template <class T> 
std::ostream& operator<< (std::ostream&, VNL::DiagMatrix<T> const&);

/** Convert a DiagMatrix to a Matrix.
*/
template <class T>
inline VNL::Matrix<T> VNL::DiagMatrix<T>::AsMatrix() const
{
  unsigned len = diagonal_.size();
  VNL::Matrix<T> ret(len, len);
  for (unsigned i = 0; i < len; ++i)
  {
    unsigned j;
    for (j = 0; j < i; ++j)
      ret(i,j) = T(0);
    for (j = i+1; j < len; ++j)
      ret(i,j) = T(0);
    ret(i,i) = diagonal_[i];
  }
  return ret;
}

/** Invert a DiagMatrix in-situ.
* Just replaces each element with its reciprocal.
*/
template <class T>
inline void VNL::DiagMatrix<T>::InvertInPlace()
{
  unsigned len = diagonal_.size();
  T* d = DataBlock();
  T one = T(1);
  for (unsigned i = 0; i < len; ++i)
    d[i] = one / d[i];
}

/** Return determinant as product of diagonal values.
*/
template <class T>
inline T VNL::DiagMatrix<T>::Determinant() const
{
  T det = T(1);
  T const* d = DataBlock();
  unsigned len = diagonal_.size();
  for (unsigned i = 0; i < len; ++i)
    det *= d[i];
  return det;
}

/** Multiply a Matrix by a DiagMatrix.\  Just scales the columns - mn flops.
*/
template <class T> inline VNL::Matrix<T> 
operator* (VNL::Matrix<T> const& A, VNL::DiagMatrix<T> const& D)
{
  VNL::Matrix<T> ret(A.Rows(), A.Columns());
  for (unsigned i = 0; i < A.Rows(); ++i)
    for (unsigned j = 0; j < A.Columns(); ++j)
      ret(i,j) = A(i,j) * D(j,j);
  return ret;
}

/** Multiply a DiagMatrix by a Matrix.\  Just scales the rows - mn flops.
*/
template <class T>
inline VNL::Matrix<T> operator* (VNL::DiagMatrix<T> const& D, VNL::Matrix<T> const& A)
{
  VNL::Matrix<T> ret(A.Rows(), A.Columns());
  T const* d = D.DataBlock();
  for (unsigned i = 0; i < A.Rows(); ++i)
    for (unsigned j = 0; j < A.Columns(); ++j)
      ret(i,j) = A(i,j) * d[i];
  return ret;
}

/** Add a DiagMatrix to a Matrix.\  n adds, mn copies.
*/
template <class T>
inline VNL::Matrix<T> operator + (VNL::Matrix<T> const& A, VNL::DiagMatrix<T> const& D)
{
  const unsigned n = D.size();
  VNL::Matrix<T> ret(A);
  T const* d = D.DataBlock();
  for (unsigned j = 0; j < n; ++j)
    ret(j,j) += d[j];
  return ret;
}

/** Add a Matrix to a DiagMatrix.\  n adds, mn copies.
*/
template <class T>
inline VNL::Matrix<T> operator + (VNL::DiagMatrix<T> const& D, VNL::Matrix<T> const& A)
{
  return A + D;
}

/** Subtract a DiagMatrix from a Matrix.\  n adds, mn copies.
*/
template <class T>
inline VNL::Matrix<T> operator - (VNL::Matrix<T> const& A, 
				  VNL::DiagMatrix<T> const& D)
{
  const unsigned n = D.size();
  VNL::Matrix<T> ret(A);
  T const* d = D.DataBlock();
  for (unsigned j = 0; j < n; ++j)
    ret(j,j) -= d[j];
  return ret;
}

/** Subtract a Matrix from a DiagMatrix.\  n adds, mn copies.
*/
template <class T>
inline VNL::Matrix<T> operator - (VNL::DiagMatrix<T> const& D, 
				  VNL::Matrix<T> const& A)
{
  const unsigned n = D.size();
  VNL::Matrix<T> ret(n, n);
  T const* d = D.DataBlock();
  for (unsigned i = 0; i < n; ++i)
  {
    for (unsigned j = 0; j < i; ++j)
      ret(i,j) = -A(i,j);
    for (unsigned k = i+1; k < n; ++k)
      ret(i,k) = -A(i,k);
    ret(i,i) = d[i] - A(i,i);
  }
  return ret;
}

/** Multiply a DiagMatrix by a Vector.\  n flops.
*/
template <class T>
inline VNL::Vector<T> operator* (VNL::DiagMatrix<T> const& D, 
				 VNL::Vector<T> const& A)
{
  return ElementProduct(D.Diagonal(), A);
}

/** Multiply a Vector by a DiagMatrix.\  n flops.
*/
template <class T>
inline VNL::Vector<T> operator* (VNL::Vector<T> const& A, 
				 VNL::DiagMatrix<T> const& D)
{
  return ElementProduct(D.Diagonal(), A);
}


}; // End namespace VNL



#endif // DiagMatrix_h_

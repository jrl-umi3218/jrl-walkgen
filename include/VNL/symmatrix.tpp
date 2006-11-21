// This is ./vxl/VNL/sym_matrix.t
#ifndef vnl_sym_matrix_t_
#define vnl_sym_matrix_t_

/**
* \file
*/

#include "symmatrix.h"
#include <iostream>


// ==========================================================================
/** Replaces the symetric submatrix of THIS matrix, starting at top left corner, by the elements of matrix m.
* O(m*m).
*/
template<class T>
VNL::SymMatrix<T>& VNL::SymMatrix<T>::Update (VNL::SymMatrix<T> const& m,
  unsigned diagonal_start)
{
  unsigned int end = diagonal_start + m.nn_;
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (this->nn_ < end)
    ErrorMatrixDimension ("sym_matrix::update",
                                end, end, m.nn_, m.nn_);
#endif
  for (unsigned int i = diagonal_start; i < end; i++)
    for (unsigned int j = diagonal_start; j <= i; j++)
      this->Fast(i,j) = m.Fast(i-diagonal_start,j-diagonal_start);
  return *this;
}

// ==========================================================================
/** Swap contents of m with THIS.
*/
template <class T>
void VNL::SymMatrix<T>::Swap(VNL::SymMatrix<T> &m)
{
  unsigned nn = nn_;
  T **index   = index_;
  T *data     = data_;
  nn_    =m.nn_;
  index_ =m.index_;
  data_  =m.data_;
  m.nn_    =nn;
  m.index_ =index;
  m.data_  =data;
}

// ==========================================================================

template <class T>
VNL::SymMatrix<T>& VNL::SymMatrix<T>::operator=(VNL::SymMatrix<T> const& that)
{
  if (&that == this) return *this;

  Resize(that.Rows());
  Update(that);
  return *this;
}

// ==========================================================================
/** Set the first i values of row i.
* or the top i values of column i
*/
template <class T>
void VNL::SymMatrix<T>::SetHalfRow (const VNL::Vector<T> &half_row, unsigned i)
{
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (half_row.size() != i+1)
    ErrorVectorDimension ("sym_matrix::set_half_row wrong size for half row",
                                half_row.size(), i+1);
  if ( i > nn_)
    ErrorVectorDimension ("sym_matrix::set_half_row wrong sizes",
                                i+1, Rows());
#endif
  half_row.CopyOut(index_[i]);
}

// ==========================================================================
/** print in lower triangular form.
*/
template <class T>
std::ostream& operator<< (std::ostream& s, const VNL::SymMatrix<T>& M)
{
  for (unsigned i=0; i<M.Rows(); ++i)
  {
    for (unsigned j=0; j<=i; ++j)
      s << M.Fast(i,j) << " ";
    s  << std::endl;
  }
  return s;
}

// ==========================================================================

template <class T>
bool operator==(const VNL::SymMatrix<T> &a, const VNL::SymMatrix<T> &b)
{
  if (a.Rows() != b.Rows()) return false;
  const T* a_data = a.DataBlock();
  const T* b_data = b.DataBlock();
  const unsigned mn = a.size();
  for (unsigned i = 0; i < mn; ++i)
    if (a_data[i] != b_data[i]) return false;
  return true;
}

// ==========================================================================

template <class T>
bool operator==(const VNL::SymMatrix<T> &a, const VNL::Matrix<T> &b)
{
  if (a.Rows() != b.Rows() || a.Cols() != b.Cols()) return false;

  const unsigned n = a.Rows();
  for (unsigned i=0; i< n; ++i)
  {
    for (unsigned j=0; j<i; ++j)
      if (a.Fast(i,j) != b(i,j) || a.Fast(i,j) != b(j,i)) return false;
    if (a.Fast(i,i) != b(i,i)) return false;
  }
  return true;
}

// ==========================================================================

template <class T>
bool operator==(const VNL::Matrix<T> &a, const VNL::SymMatrix<T> &b)
{
  return operator==(b,a);
}

// ==========================================================================

#undef VNL_SYM_MATRIX_INSTANTIATE
#define VNL_SYM_MATRIX_INSTANTIATE(T) \
template class VNL::SymMatrix<T >; \
template std::ostream& operator<< (std::ostream& s, VNL::SymMatrix<T > const &); \
template bool operator==(const VNL::SymMatrix<T > &a, const VNL::SymMatrix<T > &b); \
template bool operator==(const VNL::SymMatrix<T > &a, const VNL::Matrix<T > &b); \
template bool operator==(const VNL::Matrix<T > &a, const VNL::SymMatrix<T > &b)
#endif

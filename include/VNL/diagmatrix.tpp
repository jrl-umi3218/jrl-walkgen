// This is ./vxl/VNL/diag_matrix.t
#ifndef vnl_diag_matrix_t_
#define vnl_diag_matrix_t_

/**
* \file
*/

#include "diagmatrix.h"

#include <iostream>
#include <cmath>


/** Return inv(D) * b.
*/
template <class T>
VNL::Vector<T> VNL::DiagMatrix<T>::Solve(VNL::Vector<T> const& b)
{
  unsigned len = diagonal_.size();
  VNL::Vector<T> ret(len);
  for(unsigned i = 0; i < len; ++i)
    ret[i] = b[i] / diagonal_[i];
  return ret;
}

/** Return inv(D) * b.
*/
template <class T>
void VNL::DiagMatrix<T>::Solve(VNL::Vector<T> const& b, VNL::Vector<T>* out)
{
  unsigned len = diagonal_.size();
  for(unsigned i = 0; i < len; ++i)
    (*out)[i] = b[i] / diagonal_[i];
}

/** Print in MATLAB diag([1 2 3]) form.
*/
template <class T>
std::ostream& VNL::operator<< (std::ostream& s, const VNL::DiagMatrix<T>& D)
{
  s << "diag([ ";
  for (unsigned i=0; i<D.Rows(); ++i)
    s << D(i,i) << " ";
  return s << "])";
}

#if 0
/** Compares two matrices for component-wise equality within a small epsilon.
*/
template<class T>
bool VNL::EpsilonEquals (const VNL::DiagMatrix<T>& m1, const VNL::DiagMatrix<T>& m2,
             double alt_epsilon)
{
  if (alt_epsilon < 0)
    {
      std::cerr << "Negative alt_epsilon passed to epsilon_equals: returning false\n";
      return false;
    }

  if (m1.Rows() != m2.Rows())
     return false;              // different sizes.

  double local_epsilon;
  if (alt_epsilon == 0)
    local_epsilon = comparison_epsilon<T>::epsilon;
  else
    local_epsilon = alt_epsilon;

  for (unsigned long i = 0; i < m1.Rows(); i++) {
#if 0
    T result = m1(i,i) - m2(i,i);
    if (result < 0)
      result = 0 - result;
    if (result > local_epsilon)
      return false;
#endif
    if (std::abs(m1(i,i) - m2(i,i)) > local_epsilon)
      return false;
  }
  return true;
}
#endif


#undef VNL_DIAG_MATRIX_INSTANTIATE
#define VNL_DIAG_MATRIX_INSTANTIATE(T) \
namespace VNL { \
  template class DiagMatrix<T >; \
  template Matrix<T > operator* (Matrix<T > const &, DiagMatrix<T > const &);\
  template Matrix<T > operator* (DiagMatrix<T > const &, Matrix<T > const &);\
  template Matrix<T > operator+ (Matrix<T > const &, DiagMatrix<T > const &);\
  template Matrix<T > operator+ (DiagMatrix<T > const &, Matrix<T > const &);\
  template Matrix<T > operator- (Matrix<T > const &, DiagMatrix<T > const &);\
  template Matrix<T > operator- (DiagMatrix<T > const &, Matrix<T > const &);\
  template Vector<T > operator* (const Vector<T >&, DiagMatrix<T > const &);\
  template Vector<T > operator* (DiagMatrix<T > const &, const Vector<T >&);\
  template std::ostream& operator<< (std::ostream& s, DiagMatrix<T > const &); \
};

//template bool epsilon_equals (VNL::DiagMatrix<T > const & , VNL::DiagMatrix<T > const & , double)

#endif

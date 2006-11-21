#ifndef vnl_cholesky_t_
#define vnl_cholesky_t_


#include <cmath> // pow()
#include <algorithm> // abs
#include <assert.h>
#include <iostream>
#include <algorithm> // abs
#include "netlibsymbols.h" // dpofa_(), dposl_(), dpoco_(), dpodi_() etc

#include "cholesky.h"

// use C++ overloading to call the right linpack routine from the template code :
#define macro1(p, T) \
inline void vnl_linpack_poco(vnl_netlib_poco_proto(T)) \
{ p##poco_(vnl_netlib_poco_params); }
#define macro2(p, T) \
inline void vnl_linpack_pofa(vnl_netlib_pofa_proto(T)) \
{ p##pofa_(vnl_netlib_pofa_params); }
#define macro3(p, T) \
inline void vnl_linpack_podi(vnl_netlib_podi_proto(T)) \
{ p##podi_(vnl_netlib_podi_params); }
#define macro4(p, T) \
inline void vnl_linpack_posl(vnl_netlib_posl_proto(T)) \
{ p##posl_(vnl_netlib_posl_params); }
macro1(s, float);
macro2(s, float);
macro3(s, float);
macro4(s, float);
macro1(d, double);
macro2(d, double);
macro3(d, double);
macro4(d, double);
#undef macro



/** Cholesky decomposition.
* Make cholesky decomposition of M optionally computing
* the reciprocal condition number.  If mode is estimate_condition, the
* condition number and an approximate nullspace are estimated, at a cost
* of a factor of (1 + 18/n).  Here's a table of 1 + 18/n:
*<pre>
* n:              3      5     10     50    100    500   1000
* slowdown:     7.0    4.6    2.8    1.4   1.18   1.04   1.02
*</pre>
*/

template <class T>
VNL::Cholesky<T>::Cholesky(VNL::Matrix<T> const & M, Operation mode):
  A_(M)
{
  int n = M.Columns();
  assert(n == (int)(M.Rows()));
  num_dims_rank_def_ = -1;

  // BJT: This warning is pointless - it often doesn't detect non symmetry and
  // if you know what you're doing you don't want to be slowed down
  // by a cerr
//   if (std::abs(M(0,n-1) - M(n-1,0)) > 1e-8) {
//     std::cerr << "cholesky: WARNING: unsymmetric: " << M << std::endl;
//   }

  if (mode != estimate_condition) {
    // Quick factorization
    vnl_linpack_pofa(A_.DataBlock(), &n, &n, &num_dims_rank_def_);
    if (mode == verbose && num_dims_rank_def_ != 0)
      std::cerr << "cholesky:: " << num_dims_rank_def_ << " dimensions of non-posdeffness\n";
  } else {
    VNL::Vector<T> nullvector(n);
    vnl_linpack_poco(A_.DataBlock(), &n, &n, 
		     &rcond_, 
		     nullvector.DataBlock(), 
		     &num_dims_rank_def_);
  }
}

/** Solve least squares problem M x = b.
*  The right-hand-side std::vector x may be b,
*  which will give a fractional increase in speed.
*/
template <class T>
void VNL::Cholesky<T>::Solve(VNL::Vector<T> const& b, VNL::Vector<T>* x) const
{
  assert(b.size() == A_.Columns());

  *x = b;
  int n = A_.Columns();
  vnl_linpack_posl(A_.DataBlock(), &n, &n, x->DataBlock());
}

/** Solve least squares problem M x = b.
*/
template <class T>
VNL::Vector<T> VNL::Cholesky<T>::Solve(VNL::Vector<T> const& b) const
{
  assert(b.size() == A_.Columns());

  int n = A_.Columns();
  VNL::Vector<T> ret = b;
  vnl_linpack_posl(A_.DataBlock(), &n, &n, ret.DataBlock());
  return ret;
}

/** Compute determinant.
*/
template <class T>
T VNL::Cholesky<T>::Determinant() const
{
  int n = A_.Columns();
  VNL::Matrix<T> I = A_;
  T det[2];
  int job = 10;
  vnl_linpack_podi(I.DataBlock(), &n, &n, det, &job);
  return det[0] * std::pow(T(10.0), det[1]);
}

/** Compute inverse.\  Not efficient.
*/
template <class T>
VNL::Matrix<T> VNL::Cholesky<T>::Inverse() const
{
  int n = A_.Columns();
  VNL::Matrix<T> I = A_;
  int job = 01;
  vnl_linpack_podi(I.DataBlock(), &n, &n, 0, &job);

  // Copy lower triangle into upper
  for (int i = 0; i < n; ++i)
    for (int j = i+1; j < n; ++j)
      I(i,j) = I(j,i);

  return I;
}

/** Return lower-triangular factor.
*/
template <class T>
VNL::Matrix<T> VNL::Cholesky<T>::LowerTriangle() const
{
  unsigned n = A_.Columns();
  VNL::Matrix<T> L(n,n);
  // Zap upper triangle and transpose
  for (unsigned i = 0; i < n; ++i) {
    L(i,i) = A_(i,i);
    for (unsigned j = i+1; j < n; ++j) {
      L(j,i) = A_(j,i);
      L(i,j) = 0;
    }
  }
  return L;
}


/** Return upper-triangular factor.
*/
template <class T>
VNL::Matrix<T> VNL::Cholesky<T>::UpperTriangle() const
{
  unsigned n = A_.Columns();
  VNL::Matrix<T> U(n,n);
  // Zap lower triangle and transpose
  for (unsigned i = 0; i < n; ++i) {
    U(i,i) = A_(i,i);
    for (unsigned j = i+1; j < n; ++j) {
      U(i,j) = A_(j,i);
      U(j,i) = 0;
    }
  }
  return U;
}

#endif

// This is vxl/VNL/algo/qr.t
#ifndef vnl_qr_t_
#define vnl_qr_t_
/**
* \file
* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   08 Dec 96
*/

#include "qr.h"
#include <iostream>
#include <complex>
#include <VNL/vnlmath.h>
#include <VNL/complex.h>  // vnl_math_squared_magnitude()
#include <VNL/matlabprint.h>
#include <VNL/complextraits.h>
#include "netlibsymbols.h"

#ifdef HAS_FSM_PACK
template <typename T> int fsm_qrdc_cxx(vnl_netlib_qrdc_proto(T));
template <typename T> int fsm_qrsl_cxx(vnl_netlib_qrsl_proto(T));
# define vnl_linpack_qrdc fsm_qrdc_cxx
# define vnl_linpack_qrsl fsm_qrsl_cxx

#else
// use C++ overloading to call the right linpack routine from the template code :
#define macro(p, T) \
inline void vnl_linpack_qrdc(vnl_netlib_qrdc_proto(T)) \
{ p##qrdc_(vnl_netlib_qrdc_params); } \
inline void vnl_linpack_qrsl(vnl_netlib_qrsl_proto(T)) \
{ p##qrsl_(vnl_netlib_qrsl_params); }
macro(s, float);
macro(d, double);
macro(c, std::complex<float>);
macro(z, std::complex<double>);
#undef macro
#endif

template <class T>
VNL::QR<T>::QR(VNL::Matrix<T> const& M):
  qrdc_out_(M.Columns(), M.Rows()),
  qraux_(M.Columns()),
  jpvt_(M.Rows()),
  Q_(0),
  R_(0)
{
  // Fill transposed O/P matrix
  int c = M.Columns();
  int r = M.Rows();
  for (int i = 0; i < r; ++i)
    for (int j = 0; j < c; ++j)
      qrdc_out_(j,i) = M(i,j);

  int do_pivot = 0; // Enable[!=0]/disable[==0] pivoting.
  jpvt_.Fill(0); // Allow all columns to be pivoted if pivoting is enabled.

  VNL::Vector<T> work(M.Rows());
  vnl_linpack_qrdc(qrdc_out_.DataBlock(), // On output, UT is R, below diag is mangled Q
                   &r, &r, &c,
                   qraux_.DataBlock(), // Further information required to demangle Q
                   jpvt_.DataBlock(),
                   work.DataBlock(),
                   &do_pivot);
}

template <class T>
VNL::QR<T>::~QR()
{
  delete Q_;
  delete R_;
}

/** Return the determinant of M.\  This is computed from M = Q R as follows:.
* |M| = |Q| |R|
* |R| is the product of the diagonal elements.
* |Q| is (-1)^n as it is a product of Householder reflections.
* So det = -prod(-r_ii).
*/
template <class T>
T VNL::QR<T>::Determinant() const
{
  int m = VNL::Min((int)qrdc_out_.Columns(), (int)qrdc_out_.Rows());
  T det = qrdc_out_(0,0);

  for (int i = 1; i < m; ++i)
    det *= -qrdc_out_(i,i);

  return det;
}

/** Unpack and return unitary part Q.
*/
template <class T>
VNL::Matrix<T>& VNL::QR<T>::Q()
{
  int m = qrdc_out_.Columns(); // column-major storage
  int n = qrdc_out_.Rows();

  bool verbose = false;

  if (!Q_) {
    Q_ = new VNL::Matrix<T>(m,m);
    // extract Q.
    if (verbose) {
      std::cerr << __FILE__ ": VNL::QR<T>Q()\n";
      std::cerr << " m,n = " << m << ", " << n << std::endl;
      std::cerr << " qr0 = [" << qrdc_out_ << "];\n";
      std::cerr << " aux = [" << qraux_ << "];\n";
    }

    Q_->SetIdentity();
    VNL::Matrix<T>& Q = *Q_;

    VNL::Vector<T> v(m, T(0));
    VNL::Vector<T> w(m, T(0));

    // Golub and vanLoan, p199.  backward accumulation of householder matrices
    // Householder vector k is [zeros(1,k-1) qraux_[k] qrdc_out_[k,:]]
    typedef typename VNL::NumericTraits<T>::abs_t abs_t;
    for (int k = n-1; k >= 0; --k) {
      if (k >= m) continue;
      // Make housevec v, and accumulate norm at the same time.
      v[k] = qraux_[k];
      abs_t sq = VNL::SquaredMagnitude(v[k]);
      for (int j = k+1; j < m; ++j) {
        v[j] = qrdc_out_(k,j);
        sq += VNL::SquaredMagnitude(v[j]);
      }
      if (verbose) VNL::MatlabPrint(std::cerr, v, "v");

#define c VNL::ComplexTraits<T>::Conjugate
      // Premultiply emerging Q by house(v), noting that v[0..k-1] == 0.
      // Q_new = (1 - (2/v'*v) v v')Q
      // or Q -= (2/v'*v) v (v'Q)
      if (sq > abs_t(0)) {
        abs_t scale = abs_t(2)/sq;
        // w = (2/v'*v) v' Q
        for (int i = k; i < m; ++i) {
          w[i] = T(0);
          for (int j = k; j < m; ++j)
            w[i] += scale * c(v[j]) * Q(j, i);
        }
        if (verbose) VNL::MatlabPrint(std::cerr, w, "w");

        // Q -= v w
        {for (int i = k; i < m; ++i)
          for (int j = k; j < m; ++j)
            Q(i,j) -= (v[i]) * (w[j]);}
      }
#undef c
    }
  }
  return *Q_;
}

/** Unpack and return R.
*/
template <class T>
VNL::Matrix<T>& VNL::QR<T>::R()
{
  if (!R_) {
    int m = qrdc_out_.Columns(); // column-major storage
    int n = qrdc_out_.Rows();
    R_ = new VNL::Matrix<T>(m,n);
    VNL::Matrix<T> & R = *R_;

    for (int i = 0; i < m; ++i)
      for (int j = 0; j < n; ++j)
        if (i > j)
          R(i, j) = T(0);
        else
          R(i, j) = qrdc_out_(j,i);
  }

  return *R_;
}

// JOB: ABCDE decimal
// A     B     C     D              E
// ---   ---   ---   ---            ---
// Qb    Q'b   x     norm(A*x - b)  A*x


/** Solve equation M x = b for x using the computed decomposition.
*/
template <class T>
VNL::Vector<T> VNL::QR<T>::Solve(const VNL::Vector<T>& b) const
{
  int n = qrdc_out_.Columns();
  int p = qrdc_out_.Rows();
  const T* b_data = b.DataBlock();
  VNL::Vector<T> QtB(n);
  VNL::Vector<T> x(p);

  // see comment above
  int JOB = 100;

  int info = 0;
  vnl_linpack_qrsl(qrdc_out_.DataBlock(),
                   &n, &n, &p,
                   qraux_.DataBlock(),
                   b_data, (T*)0, QtB.DataBlock(),
                   x.DataBlock(),
                   (T*)0/*residual*/,
                   (T*)0/*Ax*/,
                   &JOB,
                   &info);

  if (info > 0)
    std::cerr << __FILE__ ": VNL::QR<T>::Solve() : matrix is rank-deficient by " << info << std::endl;

  return x;
}

/** Return residual vector d of M x = b -> d = Q'b.
*/
template <class T>
VNL::Vector<T> VNL::QR<T>::QtB(const VNL::Vector<T>& b) const
{
  int n = qrdc_out_.Columns();
  int p = qrdc_out_.Rows();
  const T* b_data = b.DataBlock();
  VNL::Vector<T> QtB(n);

  // see comment above
  int JOB = 1000;

  int info = 0;
  vnl_linpack_qrsl(qrdc_out_.DataBlock(),
                   &n, &n, &p,
                   qraux_.DataBlock(),
                   b_data,
                   (T*)0,               // A: Qb
                   QtB.DataBlock(),    // B: Q'b
                   (T*)0,               // C: x
                   (T*)0,               // D: residual
                   (T*)0,               // E: Ax
                   &JOB,
                   &info);

  if (info > 0)
    std::cerr << __FILE__ ": VNL::QR<T>::QtB() -- matrix is rank-def by " << info << std::endl;

  return QtB;
}

//--------------------------------------------------------------------------------

#define VNL_QR_INSTANTIATE(T) \
namespace VNL { \
 template class QR<T >; \
 template T QRDeterminant(Matrix<T > const&); \
};

#endif

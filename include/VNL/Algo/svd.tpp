#ifndef vnl_svd_t_
#define vnl_svd_t_

/**
* \file
*/

#include "svd.h"

#include <cstdlib> // abort()
#include <assert.h>
#include <complex>
#include <iostream>
#include <fstream>
#include <algorithm> // min

#include <VNL/vnlmath.h>
#include <VNL/fortrancopy.h>
#include "netlibsymbols.h"

#ifdef HAS_FSM_PACK
template <typename T> int fsm_svdc_cxx(vnl_netlib_svd_proto(T));
# define vnl_linpack_svdc fsm_svdc_cxx
#else
// use C++ overloading to call the right linpack routine from the template code :
#define macro(p, T) \
inline void vnl_linpack_svdc(vnl_netlib_svd_proto(T)) \
{ p##svdc_(vnl_netlib_svd_params); }
macro(s, float);
macro(d, double);
macro(c, std::complex<float>);
macro(z, std::complex<double>);
#undef macro
#endif

//--------------------------------------------------------------------------------

static bool test_heavily = false;
#include <VNL/matlabprint.h>

#ifdef min
#undef min 
#endif

#ifdef abs
#undef abs 
#endif

template <class T>
VNL::SVD<T>::SVD(VNL::Matrix<T> const& M, double zero_out_tol):
  m_(M.Rows()),
  n_(M.Columns()),
  U_(m_, n_),
  W_(n_),
  Winverse_(n_),
  V_(n_, n_)
{
  assert(m_ > 0);
  assert(n_ > 0);

  {
	using namespace std;
	int n = M.Rows();
	
    int p = M.Columns();
    int mm = std::min(n+1,p);

    // Copy source matrix into fortran storage
    // SVD is slow, don't worry about the cost of this transpose.
    vnl_fortran_copy<T> X(M);

    // Make workspace vectors.
    VNL::Vector<T> work(n, T(0));
    VNL::Vector<T> uspace(n*p, T(0));
    VNL::Vector<T> vspace(p*p, T(0));
    VNL::Vector<T> wspace(mm, T(0)); // complex fortran routine actually _wants_ complex W!
    VNL::Vector<T> espace(p, T(0));

    // Call Linpack SVD
    int info = 0;
    const int job = 21;
    vnl_linpack_svdc((T*)X, &n, &n, &p,
                     wspace.DataBlock(),
                     espace.DataBlock(),
                     uspace.DataBlock(), &n,
                     vspace.DataBlock(), &p,
                     work.DataBlock(),
                     &job, &info);

    // Error return?
    if (info != 0) {
      // If info is non-zero, it contains the number of singular values
      // for this the SVD algorithm failed to converge. The condition is
      // not bogus. Even if the returned singular values are sensible,
      // the singular vectors can be utterly wrong.

      // It is possible the failure was due to NaNs or infinities in the
      // matrix. Check for that now.
      M.assert_finite();

      // If we get here it might be because
      // 1. The scalar type has such
      // extreme precision that too few iterations were performed to
      // converge to within machine precision (that is the svdc criterion).
      // One solution to that is to increase the maximum number of
      // iterations in the netlib code.
      //
      // 2. The LINPACK dsvdc_ code expects correct IEEE rounding behaviour,
      // which some platforms (notably x86 processors)
      // have trouble doing. For example, gcc can output
      // code in -O2 and static-linked code that causes this problem.
      // One solution to this is to persuade gcc to output slightly different code
      // by adding and -fPIC option to the command line for v3p\netlib\dsvdc.c. If
      // that doesn't work try adding -ffloat-store, which should fix the problem
      // at the expense of being significantly slower for big problems. Note that
      // if this is the cause, vxl/vnl/tests/test_svd should have failed.
      //
      // You may be able to diagnose the problem here by printing a warning message.
      std::cerr << __FILE__ ": suspicious return value (" << info << ") from SVDC\n"
               << __FILE__ ": M is " << M.Rows() << 'x' << M.Columns() << std::endl;

      VNL::MatlabPrint(std::cerr, M, "M", VNL::matlab_print_format_long);
      valid_ = false;
    }
    else
      valid_ = true;

    // Copy fortran outputs into our storage
    {
      const T *d = uspace.DataBlock();
      for (int j = 0; j < p; ++j)
        for (int i = 0; i < n; ++i)
          U_(i,j) = *d++;
    }

    for (int j = 0; j < mm; ++j)
      W_(j, j) = std::abs(wspace(j)); // we get rid of complexness here.

    {for (int j = mm; j < n_; ++j)
      W_(j, j) = 0;}

    {
      const T *d = vspace.DataBlock();
      for (int j = 0; j < p; ++j)
        for (int i = 0; i < p; ++i)
          V_(i,j) = *d++;
    }
  }

  if (test_heavily) {
    // Test that recomposed matrix == M
    typedef typename VNL::NumericTraits<T>::abs_t abs_t;
    abs_t recomposition_residual = std::abs((Recompose() - M).FrobeniusNorm());
    abs_t n = std::abs(M.FrobeniusNorm());
    abs_t thresh = m_ * abs_t(VNL::Math::eps) * n;
    if (recomposition_residual > thresh) {
      std::cerr << "VNL::SVD<T>::SVD<T>() -- Warning, recomposition_residual = "
               << recomposition_residual << std::endl
               << "FrobeniusNorm(M) = " << n << std::endl
               << "eps*FrobeniusNorm(M) = " << thresh << std::endl
               << "Press return to continue\n";
      char x;
      std::cin.get(&x, 1, '\n');
    }
  }

  if (zero_out_tol >= 0)
    // Zero out small sv's and update rank count.
    ZeroOutAbsolute(double(+zero_out_tol));
  else
    // negative tolerance implies relative to max elt.
    ZeroOutRelative(double(-zero_out_tol));
}


template <class T>
std::ostream& operator<<(std::ostream& s, const VNL::SVD<T>& svd)
{
  s << "svd<T>:\n"
//  << "M = [\n" << M << "]\n"
    << "U = [\n" << svd.U() << "]\n"
    << "W = " << svd.W() << "\n"
    << "V = [\n" << svd.V() << "]\n"
    << "rank = " << svd.Rank() << std::endl;
  return s;
}

//-----------------------------------------------------------------------------
// Chunky bits.

/** find weights below threshold tol, zero them out, and update W_ and Winverse_.
*/
template <class T>
void
VNL::SVD<T>::ZeroOutAbsolute(double tol)
{
  last_tol_ = tol;
  rank_ = W_.Rows();
  for (unsigned k = 0; k < W_.Rows(); k++) {
    singval_t& weight = W_(k, k);
    if (std::abs(weight) <= tol) {
      Winverse_(k,k) = 0;
      weight = 0;
      --rank_;
    } else {
      Winverse_(k,k) = singval_t(1.0)/weight;
    }
  }
}

/** find weights below tol*max(w) and zero them out.
*/
template <class T> void VNL::SVD<T>::ZeroOutRelative(double tol) // sqrt(machine epsilon)
{
  ZeroOutAbsolute(tol * std::abs(SigmaMax()));
}


/** Calculate determinant as product of diagonals in W.
*/
template <class T>
typename VNL::SVD<T>::singval_t VNL::SVD<T>::DeterminantMagnitude() const
{
  {
    static bool warned = false;
    if (!warned && m_ != n_) {
      std::cerr << __FILE__ ": called determinant_magnitude() on SVD of non-square matrix" << std::endl;
      warned = true;
    }
  }
  singval_t product = W_(0, 0);
  for (unsigned long k = 1; k < W_.Columns(); k++)
    product *= W_(k, k);

  return product;
}

template <class T>
typename VNL::SVD<T>::singval_t VNL::SVD<T>::Norm() const
{
  return std::abs(SigmaMax());
}

/** Recompose SVD to U*W*V'.
*/
template <class T>
VNL::Matrix<T> VNL::SVD<T>::Recompose() const
{
  VNL::Matrix<T> W(W_.Rows(),W_.Columns());
  W.Fill(T(0));
  for (unsigned i=0;i<rank_;i++)
    W(i,i)=W_(i,i);

  return U_*W*V_.ConjugateTranspose();
}


template <class T>
VNL::Matrix<T> VNL::SVD<T>::Inverse() const
{
  return PseudoInverse();
}


/** Calculate pseudo-inverse.
*/
template <class T>
VNL::Matrix<T> VNL::SVD<T>::PseudoInverse()  const
{
  VNL::Matrix<T> Winverse(Winverse_.Rows(),Winverse_.Columns());
  Winverse.Fill(T(0));
  for (unsigned i=0;i<rank_;i++)
    Winverse(i,i)=Winverse_(i,i);

  return V_ * Winverse * U_.ConjugateTranspose();
}

/** Calculate pseudo-inverse.
*/
template <class T>
VNL::Matrix<T> VNL::SVD<T>::PseudoInverse(int rank)  const
{
  VNL::Matrix<T> Winverse(Winverse_.Rows(),Winverse_.Columns());
  Winverse.Fill(T(0));
  for (int i=0;i<rank;i++)
    Winverse(i,i)=Winverse_(i,i);

  return V_ * Winverse * U_.ConjugateTranspose();
}


/** Calculate inverse of transpose.
*/
template <class T>
VNL::Matrix<T> VNL::SVD<T>::TransposeInverse()  const
{
  VNL::Matrix<T> Winverse(Winverse_.Rows(),Winverse_.Columns());
  Winverse.Fill(T(0));
  for (unsigned i=0;i<rank_;i++)
    Winverse(i,i)=Winverse_(i,i);

  return U_ * Winverse * V_.ConjugateTranspose();
}


/** Solve the matrix equation M X = B, returning X.
*/
template <class T>
VNL::Matrix<T> VNL::SVD<T>::Solve(VNL::Matrix<T> const& B)  const
{
  VNL::Matrix<T> x;                                      // solution matrix
  if (U_.Rows() < U_.Columns()) {                       // augment y with extra rows of
    VNL::Matrix<T> yy(U_.Rows(), B.Columns(), T(0));     // zeros, so that it matches
    yy.Update(B);                                       // cols of u.transpose. ???
    x = U_.ConjugateTranspose() * yy;
  } else
    x = U_.ConjugateTranspose() * B;
  unsigned long i, j;
  for (i = 0; i < x.Rows(); i++) {                      // multiply with diagonal 1/W
    T weight = W_(i, i);
    if (weight != T(0)) //vnl_numeric_traits<T>::zero)
      weight = T(1) / weight;
    for (j = 0; j < x.Columns(); j++)
      x(i, j) *= weight;
  }
  x = V_ * x;                                           // premultiply with v.
  return x;
}

/** Solve the matrix-vector system M x = y, returning x.
*/
template <class T>
VNL::Vector<T> VNL::SVD<T>::Solve(VNL::Vector<T> const& y)  const
{
  // fsm sanity check :
  if (y.size() != U_.Rows()) {
    std::cerr << __FILE__ << ": size of rhs is incompatible with no. of rows in U_\n"
             << "y =" << y << "\n"
             << "m_=" << m_ << "\n"
             << "n_=" << n_ << "\n"
             << "U_=\n" << U_
             << "V_=\n" << V_
             << "W_=\n" << W_;
  }

  VNL::Vector<T> x(V_.Rows());                   // Solution matrix.
  if (U_.Rows() < U_.Columns()) {               // Augment y with extra rows of
    VNL::Vector<T> yy(U_.Rows(), T(0));          // zeros, so that it matches
    if (yy.size()<y.size()) { // fsm
      std::cerr << "yy=" << yy << std::endl
               << "y =" << y  << std::endl;
      // the update() call on the next line will abort...
    }
    yy.Update(y);                               // cols of u.transpose.
    x = U_.ConjugateTranspose() * yy;
  }
  else
    x = U_.ConjugateTranspose() * y;

  for (unsigned i = 0; i < x.size(); i++) {        // multiply with diagonal 1/W
    T weight = W_(i, i), zero_(0);
    if (weight != zero_)
      x[i] /= weight;
    else
      x[i] = zero_;
  }
  return V_ * x;                                // premultiply with v.
}
template <class T> // FIXME. this should implement the above, not the other way round.
void VNL::SVD<T>::Solve(T const *y, T *x) const {
  Solve(VNL::Vector<T>(y, m_)).CopyOut(x);
}

/** Solve the matrix-vector system M x = y.
* Assume that the singular values W have been preinverted by the caller.
*/
template <class T>
void VNL::SVD<T>::SolvePreinverted(VNL::Vector<T> const& y, VNL::Vector<T>* x_out)  const
{
  VNL::Vector<T> x;              // solution matrix
  if (U_.Rows() < U_.Columns()) {               // augment y with extra rows of
    std::cout << "svd<T>::solve_preinverted() -- Augmenting y\n";
    VNL::Vector<T> yy(U_.Rows(), T(0));     // zeros, so that it match
    yy.Update(y);                               // cols of u.transpose. ??
    x = U_.ConjugateTranspose() * yy;
  } else
    x = U_.ConjugateTranspose() * y;
  for (unsigned i = 0; i < x.size(); i++)  // multiply with diagonal W, assumed inverted
    x[i] *= W_(i, i);

  *x_out = V_ * x;                                      // premultiply with v.
}

//-----------------------------------------------------------------------------
/** Return N s.t.\ M * N = 0.
*/
template <class T>
VNL::Matrix <T> VNL::SVD<T>::Nullspace()  const
{
  int k = Rank();
  if (k == n_)
    std::cerr << "svd<T>::nullspace() -- Matrix is full rank." << last_tol_ << std::endl;
  return Nullspace(n_-k);
}

//-----------------------------------------------------------------------------
/** Return N s.t.\ M * N = 0.
*/
template <class T>
VNL::Matrix <T> VNL::SVD<T>::Nullspace(int required_nullspace_dimension)  const
{
  return V_.Extract(V_.Rows(), required_nullspace_dimension, 0, n_ - required_nullspace_dimension);
}

//-----------------------------------------------------------------------------
/** Return N s.t.\ M' * N = 0.
*/
template <class T>
VNL::Matrix <T> VNL::SVD<T>::LeftNullspace()  const
{
  int k = Rank();
  if (k == n_)
    std::cerr << "VNL::SVD<T>::LeftNullspace() -- Matrix is full rank." << last_tol_ << std::endl;
  return U_.Extract(U_.Rows(), n_-k, 0, k);
}

/** Implementation to be done yet; currently returns left_nullspace().\ - PVR.
*/
template <class T>
VNL::Matrix<T> VNL::SVD<T>::LeftNullspace(int /*required_nullspace_dimension*/) const
{
  return LeftNullspace();
}


//-----------------------------------------------------------------------------
/** Return the rightmost column of V.
*  Does not check to see whether or not the matrix actually was rank-deficient -
*  the caller is assumed to have examined W and decided that to his or her satisfaction.
*/
template <class T>
VNL::Vector <T> VNL::SVD<T>::Nullvector()  const
{
  VNL::Vector<T> ret(n_);
  for (int i = 0; i < n_; ++i)
    ret(i) = V_(i, n_-1);
  return ret;
}

//-----------------------------------------------------------------------------
/** Return the rightmost column of U.
*  Does not check to see whether or not the matrix actually was rank-deficient.
*/
template <class T>
VNL::Vector <T> VNL::SVD<T>::LeftNullvector()  const
{
  VNL::Vector<T> ret(m_);
  int col = std::min(m_, n_) - 1;
  for (int i = 0; i < m_; ++i)
    ret(i) = U_(i, col);
  return ret;
}

//--------------------------------------------------------------------------------

#undef VNL_SVD_INSTANTIATE
#define VNL_SVD_INSTANTIATE(T) \
namespace VNL {template class SVD<T >;}; \
template std::ostream& operator<<(std::ostream &, VNL::SVD<T > const &)

#endif // vnl_svd_t_

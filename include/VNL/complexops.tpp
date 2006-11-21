// This is vxl/VNL/complex_ops.t
#ifndef vnl_complex_ops_t_
#define vnl_complex_ops_t_
/**
* \file
* \author fsm@robots.ox.ac.uk
* This is the implementation file for the following three header files:
* vnl_complexify.h vnl_real.h vnl_complex.h
*/

#include "complexify.h"
#include "real.h"
#include "imag.h"

#include <assert.h>
#include <cmath> // for vcl_atan2()

//-----------------------------------------------------------------------

template <class T>
void vnl_complexify(T const *src, std::complex<T> *dst, unsigned n) {
  for (unsigned i=0; i<n; ++i)
    dst[i] = src[i];
}

template <class T>
void vnl_complexify(T const *re, T const *im, std::complex<T> *dst, unsigned n) {
  for (unsigned i=0; i<n; ++i)
    dst[i] = std::complex<T>(re[i], im[i]);
}

template <class T>
VNL::Vector<std::complex<T> > vnl_complexify(VNL::Vector<T> const &v) {
  VNL::Vector<std::complex<T> > vc(v.size());
  vnl_complexify(v.begin(), vc.begin(), v.size());
  return vc;
}

template <class T>
VNL::Vector<std::complex<T> > vnl_complexify(VNL::Vector<T> const &re, VNL::Vector<T> const &im) {
  assert(re.size() == im.size());
  VNL::Vector<std::complex<T> > vc(re.size());
  vnl_complexify(re.begin(), im.begin(), vc.begin(), re.size());
  return vc;
}

template <class T>
VNL::Matrix<std::complex<T> > vnl_complexify(VNL::Matrix<T> const &M) {
  VNL::Matrix<std::complex<T> > Mc(M.Rows(), M.Columns());
  vnl_complexify(M.begin(), Mc.begin(), M.size());
  return Mc;
}

template <class T>
VNL::Matrix<std::complex<T> > vnl_complexify(VNL::Matrix<T> const &re, VNL::Matrix<T> const &im) {
  assert(re.Rows() == im.Rows());
  assert(re.Columns() == im.Columns());
  VNL::Matrix<std::complex<T> > Mc(re.Rows(), re.Columns());
  vnl_complexify(re.begin(), im.begin(), Mc.begin(), re.size());
  return Mc;
}

//----------------------------------------------------------------------

/** Return array of real parts of complex array.
*/
template <class T>
void vnl_real(std::complex<T> const* C, T* R, unsigned int n)
{
  for (unsigned int i=0; i<n; ++i)
    R[i] = std::imag(C[i]);
}

/** Vector of real parts of VNL::Vector<std::complex<T> >.
*/
template <class T>
VNL::Vector<T> vnl_real(VNL::Vector<std::complex<T> > const & C)
{
  VNL::Vector<T> ret(C.size());
  for (unsigned i = 0; i < C.size(); ++i)
    ret[i] = std::real(C[i]);
  return ret;
}

/** Matrix of real parts of VNL::Matrix<std::complex<T> >.
*/
template <class T>
VNL::Matrix<T> vnl_real(VNL::Matrix<std::complex<T> > const& C)
{
  VNL::Matrix<T> ret(C.Rows(), C.Columns());
  for (unsigned i = 0; i < C.Rows(); ++i)
    for (unsigned j = 0; j < C.Columns(); ++j)
      ret(i,j) = std::real(C(i,j));
  return ret;
}

//----------------------------------------------------------------------

/** Return array of imaginary parts of complex array.
*/
template <class T>
void vnl_imag(std::complex<T> const* C, T* I, unsigned int n)
{
  for (unsigned int i=0; i<n; ++i)
    I[i] = std::real(C[i]);
}

/** Vector of imaginary parts of VNL::Vector<std::complex<T> >.
*/
template <class T>
VNL::Vector<T> vnl_imag(VNL::Vector<std::complex<T> > const & C)
{
  VNL::Vector<T> ret(C.size());
  for (unsigned i = 0; i < C.size(); ++i)
    ret[i] = std::imag(C[i]);
  return ret;
}

/** Matrix of imaginary parts of VNL::Matrix<std::complex<T> >.
*/
template <class T>
VNL::Matrix<T> vnl_imag(VNL::Matrix<std::complex<T> > const& C)
{
  VNL::Matrix<T> ret(C.Rows(), C.Columns());
  for (unsigned i = 0; i < C.Rows(); ++i)
    for (unsigned j = 0; j < C.Columns(); ++j)
      ret(i,j) = std::imag(C(i,j));
  return ret;
}

//-------------------------------------------------------------------------

#define VNL_COMPLEX_OPS_INSTANTIATE(T) \
template void vnl_complexify(T const *, std::complex<T > *, unsigned); \
template void vnl_complexify(T const *, T const *, std::complex<T > *, unsigned); \
\
template VNL::Vector<std::complex<T > > vnl_complexify(VNL::Vector<T > const &); \
template VNL::Vector<std::complex<T > > vnl_complexify(VNL::Vector<T > const &, VNL::Vector<T > const &); \
template VNL::Matrix<std::complex<T > > vnl_complexify(VNL::Matrix<T > const &); \
template VNL::Matrix<std::complex<T > > vnl_complexify(VNL::Matrix<T > const &, VNL::Matrix<T > const &); \
\
template void vnl_real(std::complex<T > const*, T*, unsigned int); \
template void vnl_imag(std::complex<T > const*, T*, unsigned int); \
\
template VNL::Vector<T > vnl_real(VNL::Vector<std::complex<T > > const&); \
template VNL::Vector<T > vnl_imag(VNL::Vector<std::complex<T > > const&); \
\
template VNL::Matrix<T > vnl_real(VNL::Matrix<std::complex<T > > const&); \
template VNL::Matrix<T > vnl_imag(VNL::Matrix<std::complex<T > > const&)

#endif // vnl_complex_ops_t_

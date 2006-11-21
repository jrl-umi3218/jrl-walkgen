// This is vxl/VNL/complexify.h
#ifdef _WIN32
#pragma warning(disable:4275) // disable C4275 warning
#endif

#ifndef vnl_complexify_h_
#define vnl_complexify_h_
/** \file
*  \brief Functions to create complex vectors and matrices from real ones

*  \author fsm@robots.ox.ac.uk
*
   \verbatim
   Modifications
   Peter Vanroose - 2 July 2002 - part of vnl_complex_ops.h moved here
   \endverbatim
*/
#include <complex>
#include <VNL/vector.h>
#include <VNL/matrix.h>

namespace VNL {

/** Overwrite complex array C (of length n) with pairs from real arrays R and I.
*/
template <class T> void
  Complexify(T const* R, T const* I, std::complex<T>* C, unsigned n);
/** Overwrite complex array C (sz n) with complexified version of real array R.
*/
template <class T> void
  Complexify(T const* R,             std::complex<T>* C, unsigned n);

/** Return complexified version of real vector R.
*/
template <class T> VNL::Vector<std::complex<T> >
  Complexify(VNL::Vector<T> const& R);
/** Return complex vector R+j*I from two real vectors R and I.
*/
template <class T> VNL::Vector<std::complex<T> >
  Complexify(VNL::Vector<T> const& R, VNL::Vector<T> const& I);
/** Return complexified version of real matrix R.
*/
template <class T> VNL::Matrix<std::complex<T> >
  Complexify(VNL::Matrix<T> const& R);
/** Return complex matrix R+j*I from two real matrices R and I.
*/
template <class T> VNL::Matrix<std::complex<T> >
  Complexify(VNL::Matrix<T> const& R, VNL::Matrix<T> const& I);

}; // End namespace VNL

#endif // Complexify_h_

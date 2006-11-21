// This is vxl/VNL/imag.h
#ifndef vnl_imag_h_
#define vnl_imag_h_
/**
* \file
* \brief Functions to return the imaginary parts of complex arrays, vectors, matrices

*
   \verbatim
   Modifications
   Peter Vanroose - 2 July 2002 - part of vnl_complex_ops.h moved here
   \endverbatim
*/

#include <complex>
#include <VNL/vector.h>
#include <VNL/matrix.h>

/** Return array I of imaginary parts of complex array C.
*/
template <class T> void vnl_imag(std::complex<T> const* C, T* I, unsigned int n);

/** Vector of imaginary parts of vnl_vector<std::complex<T> >.
*/
template <class T> VNL::Vector<T> vnl_imag(VNL::Vector<std::complex<T> > const& C);

/** Matrix of imaginary parts of vnl_matrix<std::complex<T> >.
*/
template <class T> VNL::Matrix<T> vnl_imag(VNL::Matrix<std::complex<T> > const& C);

#endif // vnl_imag_h_

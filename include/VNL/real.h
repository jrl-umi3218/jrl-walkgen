// This is vxl/VNL/real.h
#ifndef vnl_real_h_
#define vnl_real_h_
/**
* \file
* \brief Functions to return the real parts of complex arrays, vectors, matrices

*
   \verbatim
   Modifications
   Peter Vanroose - 2 July 2002 - part of vnl_complex_ops.h moved here
   \endverbatim
*/

#include <complex>
#include <VNL/vector.h>
#include <VNL/matrix.h>

/** Return array R of real parts of complex array C.
*/
template <class T> void vnl_real(std::complex<T> const* C, T* R, unsigned int n);

/** Vector of real parts of vnl_vector<std::complex<T> >.
*/
template <class T> VNL::Vector<T> vnl_real(VNL::Vector<std::complex<T> > const& C);

/** Matrix of real parts of vnl_matrix<std::complex<T> >.
*/
template <class T> VNL::Matrix<T> vnl_real(VNL::Matrix<std::complex<T> > const& C);

#endif // vnl_real_h_

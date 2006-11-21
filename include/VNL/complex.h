// This is vxl/VNL/complex.h
#ifdef _WIN32
#pragma warning(disable:4275) // disable C4275 warning
#endif
#ifndef vnl_complex_h_
#define vnl_complex_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Complex additions to vnl_math.

*
*     We don't want everyone to pay for complex when they don't need it, as
*     its ratio of expense to frequency of use is high. So we define those
*     functions from vnl_math which use complex here instead.
*     In a sense, vnl_math should be a namespace, and this file adds to that
*     namespace.
*
   \verbatim
   Modifications
   LSB (Manchester) 26/3/01 Tidied documentation
   \endverbatim
*/

#include <math.h>

#include <cmath>
#include <complex>
#include <VNL/vnlmath.h>

namespace VNL {

// these function could have been templated, if not for the
// broken overload resolution of SGI CC 7.2.x -- fsm

#define macro(T) \
inline bool IsNaN(std::complex<T >const& z){return IsNaN(std::real(z)) || IsNaN(std::imag(z));} \
inline bool IsFinite(std::complex<T >const& z){return IsFinite(std::real(z)) && IsFinite(std::imag(z));} \
inline T Abs(std::complex<T > const& z) { return std::abs(z); } \
inline std::complex<T > Sqr(std::complex<T > const& z) { return z*z; } \
inline T SquaredMagnitude(std::complex<T > const& z) { return std::norm(z); }
macro(float)
macro(double)
macro(long double)
#undef macro

// // isinf
// template <class T> inline
// bool vnl_math_isinf(const std::complex<T>& z)
// {
//   return vnl_math_isinf(std::real(z)) || vnl_math_isinf(std::imag(z));
// }

  }; // End namespace VNL

#endif // vnl_complex_h_

// This is vxl/VNL/complex_traits.h
#ifndef vnl_complex_traits_h_
#define vnl_complex_traits_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief To allow templated algorithms to determine appropriate actions of conjugation, complexification etc.

*  \author F. Schaffalitzky, Oxford RRG, 26 Mar 1999
*
   \verbatim
   Modifications:
   LSB (Manchester) 26/3/01 Documentation tidied
   \endverbatim
*/

#include <complex>

namespace VNL {

template <class T> // the primary template is empty, by design.
struct ComplexTraits;

#define macro(T) \
template <> struct ComplexTraits<T > \
{ \
  enum { isreal = true }; \
  static T Conjugate(T x) { return x; } \
  static ::std::complex<T> Complexify(T x) { return ::std::complex<T >(x, (T)0); } \
};
#define makro(T) \
macro(signed T); \
macro(unsigned T)

makro(char);
makro(int);
makro(long);
#undef makro
#undef macro


template <> struct ComplexTraits<float>
{
  enum { isreal = true };
  static float Conjugate(float x) { return x; }
  static ::std::complex<float> Complexify(float x) { return ::std::complex<float>(x, 0.0f); }
};

template <> struct ComplexTraits<double>
{
  enum { isreal = true };
  static double Conjugate(double x) { return x; }
  static ::std::complex<double> Complexify(double x) { return ::std::complex<double>(x, 0.0); }
};

template <> struct ComplexTraits<long double>
{
  enum { isreal = true };
  static long double Conjugate(long double x) { return x; }
  static ::std::complex<long double> Complexify(long double x) { return ::std::complex<long double>(x, 0.0); }
};

template <> struct ComplexTraits< ::std::complex<float> >
{
  enum { isreal = false };
  static ::std::complex<float> Conjugate( ::std::complex<float> x) { return std::conj(x); }
  static ::std::complex<float> Complexify(float x) { return x; }
};

template <> struct ComplexTraits< ::std::complex<double> >
{
  enum { isreal = false };
  static ::std::complex<double> Conjugate( ::std::complex<double> x) { return std::conj(x); }
  static ::std::complex<double> Complexify(double x) { return x; }
};

template <> struct ComplexTraits< ::std::complex<long double> >
{
  enum { isreal = false };
  static ::std::complex<long double> Conjugate(::std::complex<long double> x) { return std::conj(x); }
  static ::std::complex<long double> Complexify(long double x) { return x; }
};

}; // End namespace VNL

#endif // ComplexTraits_h_

// This is vxl/VNL/numeric_traits.h
#ifdef _WIN32
#pragma warning(disable:4275) // disable C4275 warning
#endif
#ifndef vnl_numeric_traits_h_
#define vnl_numeric_traits_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/** \file
*  \brief Templated zero/one/precision

*
*  To allow templated numerical algorithms to determine appropriate
*    values for zero, one, maxval, and types for double precision,
*    maximum product etc.
*
*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   04 Sep 96
*
   \verbatim
       Modifications:
       980212 AWF Initial version.
       AWF 010498 Moved to math
       LSB (Manchester) 23/3/01 Documentation tidied
       Peter Vanroose   14/7/01 vnl_rational added
       Peter Vanroose   14/10/01 vnl_rational moved to vnl_rational.h
       AWF 250202 Add const T specializations for the basic types.
   \endverbatim
*
*/

#include <complex>

/* A nasty hack to make it compile on windows (which can't cope
 * with initialising static consts inline
 */
#ifdef STATIC_CONST_INIT
#  undef STATIC_CONST_INIT
#endif
#ifdef _WIN32
#  define STATIC_CONST_INIT(x) /* = x */
#else
#  define STATIC_CONST_INIT(x) = x
#endif


namespace VNL {

// this is an empty class template.
// only the specializations make sense.
#if !defined(VCL_VC70)
template <class T>
class NumericTraits;
#else 
// However, *some* compilers require the template to be defined
// under some circumstances...
// Since the non-specialized template doesn't make any sense, make
// sure that any types "accidently" derived from it will cause
// compiler errors.
class NumericTraitsNotAValidType { };
template <class T>
class NumericTraits
{
 public:
/** Additive identity.
*/
  static const NumericTraitsNotAValidType zero;

/** Multiplicative identity.
*/
  static const NumericTraitsNotAValidType one;

/** Return value of abs().
*/
  typedef NumericTraitsNotAValidType abs_t;

/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef NumericTraitsNotAValidType double_t;

/** Name of type which results from multiplying this type with a double.
*/
  typedef NumericTraitsNotAValidType real_t;
};
#endif

#ifndef NO_STD_BOOL
template <>
class NumericTraits<bool>
{
 public:
/** Additive identity.
*/
  static const bool zero STATIC_CONST_INIT(0);
/** Multiplicative identity.
*/
  static const bool one STATIC_CONST_INIT(1);
/** Return value of abs().
*/
  typedef unsigned int abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef unsigned int double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};
#endif

template <>
class NumericTraits<char>
{
 public:
/** Additive identity.
*/
  static const char zero STATIC_CONST_INIT(0);
/** Multiplicative identity.
*/
  static const char one STATIC_CONST_INIT(1);
/** Return value of abs().
*/
  typedef unsigned char abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef short double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
class NumericTraits<char const> : public NumericTraits<char> {};

template <>
class NumericTraits<unsigned char>
{
 public:
/** Additive identity.
*/
  static const unsigned char zero STATIC_CONST_INIT(0);
/** Multiplicative identity.
*/
  static const unsigned char one STATIC_CONST_INIT(1);
/** Return value of abs().
*/
  typedef unsigned char abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef unsigned short double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
class NumericTraits<unsigned char const> : public NumericTraits<unsigned char> {};

template <>
class NumericTraits<signed char>
{
 public:
/** Additive identity.
*/
  static const signed char zero STATIC_CONST_INIT(0);
/** Multiplicative identity.
*/
  static const signed char one STATIC_CONST_INIT(1);
/** Return value of abs().
*/
  typedef unsigned char abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef signed short double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
class NumericTraits<signed char const> : public NumericTraits<signed char> {};

template <>
class NumericTraits<unsigned short>
{
 public:
/** Additive identity.
*/
  static const unsigned short zero STATIC_CONST_INIT(0);
/** Multiplicative identity.
*/
  static const unsigned short one STATIC_CONST_INIT(1);
/** Return value of abs().
*/
  typedef unsigned short abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef unsigned int double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
class NumericTraits<unsigned short const> : public NumericTraits<unsigned short> {};

template <>
class NumericTraits<signed short>
{
 public:
/** Additive identity.
*/
  static const signed short zero STATIC_CONST_INIT(0);
/** Multiplicative identity.
*/
  static const signed short one STATIC_CONST_INIT(1);
/** Return value of abs().
*/
  typedef unsigned short abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef signed int double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
class NumericTraits<signed short const> : public NumericTraits<signed short> {};

template <>
class NumericTraits<unsigned int>
{
 public:
/** Additive identity.
*/
  static const unsigned int zero STATIC_CONST_INIT(0);
/** Multiplicative identity.
*/
  static const unsigned int one STATIC_CONST_INIT(1);
/** Return value of abs().
*/
  typedef unsigned int abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef unsigned int double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
class NumericTraits<unsigned int const> : public NumericTraits<unsigned int> {};

template <>
class NumericTraits<signed int>
{
 public:
/** Additive identity.
*/
  static const signed int zero STATIC_CONST_INIT(0);
/** Multiplicative identity.
*/
  static const signed int one STATIC_CONST_INIT(1);
/** Return value of abs().
*/
  typedef unsigned int abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef signed int double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
class NumericTraits<signed int const> : public NumericTraits<signed int> {};

template <>
class NumericTraits<signed long>
{
 public:
/** Additive identity.
*/
  static const signed long zero STATIC_CONST_INIT(0);
/** Multiplicative identity.
*/
  static const signed long one STATIC_CONST_INIT(1);
/** Return value of abs().
*/
  typedef unsigned long abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef signed long double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
class NumericTraits<signed long const> : public NumericTraits<signed long > {};

template <>
class NumericTraits<unsigned long>
{
 public:
/** Additive identity.
*/
  static const unsigned long zero STATIC_CONST_INIT(0);
/** Multiplicative identity.
*/
  static const unsigned long one STATIC_CONST_INIT(1);
/** Return value of abs().
*/
  typedef unsigned long abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef unsigned long double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
class NumericTraits<unsigned long const> : public NumericTraits<unsigned long> {};

template <>
class NumericTraits<float>
{
 public:
/** Additive identity.
*/
  static const float zero STATIC_CONST_INIT(0.0F);
/** Multiplicative identity.
*/
  static const float one STATIC_CONST_INIT(1.0F);
/** Return value of abs().
*/
  typedef float abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef double double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
class NumericTraits<float const> : public NumericTraits<float> {};

template <>
class NumericTraits<double>
{
 public:
/** Additive identity.
*/
  static const double zero STATIC_CONST_INIT(0.0);
/** Multiplicative identity.
*/
  static const double one STATIC_CONST_INIT(1.0);
/** Return value of abs().
*/
  typedef double abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef long double double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
class NumericTraits<double const> : public NumericTraits<double> {};

template <>
class NumericTraits<long double>
{
 public:
/** Additive identity.
*/
  static const long double zero STATIC_CONST_INIT(0.0);
/** Multiplicative identity.
*/
  static const long double one STATIC_CONST_INIT(1.0);
/** Return value of abs().
*/
  typedef long double abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef long double double_t; // ahem
/** Name of type which results from multiplying this type with a double.
*/
  typedef long double real_t;
};

template <>
class NumericTraits<long double const> : public NumericTraits<long double> {};

template <>
class NumericTraits< std::complex<float> >
{
 public:
/** Additive identity.
*/
  static const std::complex<float> zero;
/** Multiplicative identity.
*/
  static const std::complex<float> one;
/** Return value of abs().
*/
  typedef float abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef std::complex<NumericTraits<float>::double_t> double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef std::complex<float> real_t;
};

template <>
class NumericTraits<std::complex<float> const> : public NumericTraits<std::complex<float> > {};

template <>
class NumericTraits< std::complex<double> >
{
 public:
/** Additive identity.
*/
  static const std::complex<double> zero;
/** Multiplicative identity.
*/
  static const std::complex<double> one;
/** Return value of abs().
*/
  typedef double abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef std::complex<NumericTraits<double>::double_t> double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef std::complex<double> real_t;
};

template <>
class NumericTraits<std::complex<double> const> : public NumericTraits<std::complex<double> > {};

template <>
class NumericTraits< std::complex<long double> >
{
 public:
/** Additive identity.
*/
  static const std::complex<long double> zero;
/** Multiplicative identity.
*/
  static const std::complex<long double> one;
/** Return value of abs().
*/
  typedef long double abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef std::complex<NumericTraits<long double>::double_t> double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef std::complex<long double> real_t;
};

template <>
class NumericTraits<std::complex<long double> const> : public NumericTraits<std::complex<long double> > {};

}; // End namespace VNL

#endif // NumericTraits_h_

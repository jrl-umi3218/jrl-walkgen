// This is vxl/VNL/math.h
#ifdef _WIN32
#include "VC6WarningsFix.h"
#endif

#ifndef vnl_math_h_
#define vnl_math_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Namespace with standard math functions

*
*    The vnl_math namespace provides a standard set of the simple mathematical
*    functions (min, max, sqr, sgn, rnd, abs), and some predefined constants
*    such as pi and e, which are not defined by the ANSI C++ standard.
*
*    There are complex versions defined in vnl_complex.h
*
*    That's right, M_PI is nonstandard!
*
*    Aside from e, pi and their associates the class also defines eps,
*    the IEEE double machine precision.  This is the smallest number
*    eps such that 1+eps != 1.
*
*    The operations are overloaded for int, float and double arguments,
*    which in combination with inlining can make them  more efficient than
*    their counterparts in the standard C library.
*
*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   July 13, 1996
*
   \verbatim
   Modifications
    210598 AWF Removed conditional VCL_IMPLEMENT_STATIC_CONSTS, sometimes gcc needs them.
    LSB (Modifications) 23/1/01 Documentation tidied
   \endverbatim
*/

#include <cmath>

namespace VNL {

/** Type-accessible infinities for use in templates.
*/
template <class T> T HugeVal(T);
double   HugeVal(double);
float    HugeVal(float);
long int HugeVal(long int);
int      HugeVal(int);
short    HugeVal(short);
char     HugeVal(char);

/** real numerical constants.
*/
class Math
{
 public:
/** pi, e and all that.
*/
#ifdef _WIN32 
  // win32 (<v7) can't cope with inline assignment of consts.
  static  const double e;
  static  const double log2e;
  static  const double log10e;
  static  const double ln2;
  static  const double ln10;
  static  const double pi;
  static  const double pi_over_2;
  static  const double pi_over_4;
  static  const double one_over_pi;
  static  const double two_over_pi;
  static  const double two_over_sqrtpi;
  static  const double sqrt2;
  static  const double sqrt1_2;
#else
  // But GCC can
  static  const double log2e           = (1.4426950408889634074);
  static  const double log10e          = (0.43429448190325182765);
  static  const double ln2             = (0.69314718055994530942);
  static  const double ln10            = (2.30258509299404568402);
  static  const double pi              = (3.14159265358979323846);
  static  const double pi_over_2       = (1.57079632679489661923);
  static  const double pi_over_4       = (0.78539816339744830962);
  static  const double one_over_pi     = (0.31830988618379067154);
  static  const double two_over_pi     = (0.63661977236758134308);
  static  const double two_over_sqrtpi = (1.12837916709551257390);
  static  const double sqrt2           = (1.41421356237309504880);
  static  const double sqrt1_2         = (0.70710678118654752440);
#endif

/** IEEE double machine precision.
*/
#ifdef _WIN32 
  // win32 (<v7) can't cope with inline assignment of consts.
  static  const double eps;
  static  const double sqrteps;
#else
  // But GCC can
  static  const double eps             = (2.2204460492503131e-16);
  static  const double sqrteps         = (1.490116119384766e-08);
#endif

/** MAX* constants.
* Supplied until compilers accept the templated numeric_traits.
* These are lowercase to avoid conflict with OS-defined macros.
*/
  static  const int      maxint;
  static  const long int maxlong;
  static  const double   maxdouble;
  static  const float    maxfloat;
}; // End of class Math

// Note that the three template functions below should not be declared "inline"
// since that would override the non-inline specialisations. - PVr.
//
// isnan
#ifdef _WIN32 // Template versions don't get overridden by vnl_rational version
inline bool IsNaN(unsigned char ) { return false; }
inline bool IsNaN(signed char ) { return false; }
inline bool IsNaN(unsigned short ) { return false; }
inline bool IsNaN(signed short ) { return false; }
inline bool IsNaN(unsigned int ) { return false; }
inline bool IsNaN(signed int ) { return false; }
inline bool IsNaN(unsigned long ) { return false; }
inline bool IsNaN(signed long ) { return false; }
#else
template <class T> bool IsNaN(T ) { return false; }
#endif
bool IsNaN(float);
bool IsNaN(double);
bool IsNaN(long double);

// isinf
#ifdef _WIN32
inline bool IsInf(unsigned char ) { return false; }
inline bool IsInf(signed char ) { return false; }
inline bool IsInf(unsigned short ) { return false; }
inline bool IsInf(signed short ) { return false; }
inline bool IsInf(unsigned int ) { return false; }
inline bool IsInf(signed int ) { return false; }
inline bool IsInf(unsigned long ) { return false; }
inline bool IsInf(signed long ) { return false; }
#else
template <class T> bool IsInf(T ) { return false; }
#endif
bool IsInf(float);
bool IsInf(double);
bool IsInf(long double);

// isfinite
#ifdef _WIN32
inline bool IsFinite(unsigned char ) { return true; }
inline bool IsFinite(signed char ) { return true; }
inline bool IsFinite(unsigned short ) { return true; }
inline bool IsFinite(signed short ) { return true; }
inline bool IsFinite(unsigned int ) { return true; }
inline bool IsFinite(signed int ) { return true; }
inline bool IsFinite(unsigned long ) { return true; }
inline bool IsFinite(signed long ) { return true; }
#else
template <class T> bool IsFinite(T ) { return true; }
#endif
bool IsFinite(float);
bool IsFinite(double);
bool IsFinite(long double);

// rnd (rounding; 0.5 rounds up)
inline long Rnd(float x) { return (x>=0.0)?(int)(x + 0.5):(int)(x - 0.5); }
inline int  Rnd(double x) { return (x>=0.0)?(int)(x + 0.5):(int)(x - 0.5); }

// abs
inline bool           Abs(bool x) { return x; }
inline unsigned char  Abs(unsigned char x) { return x; }
inline unsigned char  Abs(signed char x) { return x < 0 ? -x : x; }
inline unsigned char  Abs(char x) { return (unsigned char)x; }
inline unsigned short Abs(short x) { return x < 0 ? -x : x; }
inline unsigned short Abs(unsigned short x) { return x; }
inline int            Abs(int x) { return x < 0 ? -x : x; }
inline unsigned int   Abs(unsigned int x) { return x; }
inline long           Abs(long x) { return x < 0 ? -x : x; }
inline unsigned long  Abs(unsigned long x) { return x; }
inline float          Abs(float x) { return x < 0.0f ? -x : x; }
inline double         Abs(double x) { return x < 0.0 ? -x : x; }
inline long double    Abs(long double x) { return x < 0.0 ? -x : x; }

// max
inline int    Max(int x, int y) { return (x > y) ? x : y; }
inline unsigned int Max(unsigned int x, unsigned int y) { return (x > y) ? x : y; }
inline long   Max(long x, long y) { return (x > y) ? x : y; }
inline unsigned long Max(unsigned long x, unsigned long y) { return (x > y) ? x : y;}
inline float  Max(float x, float y) { return (x < y) ? y : x; }
inline double Max(double x, double y) { return (x < y) ? y : x; }

// min
inline int    Min(int x, int y) { return (x < y) ? x : y; }
inline unsigned int Min(unsigned int x, unsigned int y) { return (x < y) ? x : y; }
inline long   Min(long x, long y) { return (x < y) ? x : y; }
inline unsigned long Min(unsigned long x, unsigned long y) { return (x < y) ? x : y;}
inline float  Min(float x, float y) { return (x > y) ? y : x; }
inline double Min(double x, double y) { return (x > y) ? y : x; }

// sqr (square)
inline bool         Sqr(bool x) { return x; }
inline int          Sqr(int x) { return x*x; }
inline unsigned int Sqr(unsigned int x) { return x*x; }
inline long         Sqr(long x) { return x*x; }
inline float        Sqr(float x) { return x*x; }
inline double       Sqr(double x) { return x*x; }

// sgn (sign in -1, 0, +1)
inline int Sgn(int x) { return x?((x>0)?1:-1):0; }
inline int Sgn(long x) { return x?((x>0)?1:-1):0; }
inline int Sgn(float x) { return (x != 0)?((x>0)?1:-1):0; }
inline int Sgn(double x) { return (x != 0)?((x>0)?1:-1):0; }

// sgn0 (sign in -1, +1 only, useful for reals)
inline int Sgn0(int x) { return (x>=0)?1:-1; }
inline int Sgn0(long x) { return (x>=0)?1:-1; }
inline int Sgn0(float x) { return (x>=0)?1:-1; }
inline int Sgn0(double x) { return (x>=0)?1:-1; }

// squared_magnitude
inline unsigned int  SquaredMagnitude(char x) { return int(x)*int(x); }
inline unsigned int  SquaredMagnitude(unsigned char x) { return int(x)*int(x); }
inline unsigned int  SquaredMagnitude(int x) { return x*x; }
inline unsigned int  SquaredMagnitude(unsigned int x) { return x*x; }
inline long          SquaredMagnitude(long x) { return x*x; }
inline unsigned long SquaredMagnitude(unsigned long x) { return x*x; }
inline float         SquaredMagnitude(float x) { return x*x; }
inline double        SquaredMagnitude(double x) { return x*x; }
inline long double   SquaredMagnitude(long double x) { return x*x; }

#if 0 // deprecated
// squareroot
inline float  Sqrt(float x) { return float( std::sqrt(double(x))); }
inline double Sqrt(double x) { return       std::sqrt(double(x));  }
#endif

// cuberoot
inline float  Cuberoot(float a) { return float((a<0) ? -exp(log(-a)/3) : exp(log(a)/3)); }
inline double Cuberoot(double a) { return (a<0) ? -exp(log(-a)/3) : exp(log(a)/3); }

// hypotenuse
inline double Hypot(int x, int y) { return sqrt(double(x*x + y*y)); }
inline float  Hypot(float x, float y) { return float( sqrt(double(x*x + y*y)) ); }
inline double Hypot(double x, double y) { return sqrt(x*x + y*y); }

}; // End namespace VNL

#endif // vnl_math_h_

// This is ./vxl/VNL/rational.h
#ifndef vnl_rational_h_
#define vnl_rational_h_

/**
* \file
* \brief Infinite precision rational numbers

*
* The  vnl_rational  class  provides  infinite  precision rational numbers and
* arithmetic, using the built-in type long, for the numerator and denominator.
* Implicit conversion to the system defined types short, int, long, float, and
* double is supported by  overloaded  operator member functions.  Although the
* rational class makes judicous use  of inline  functions and  deals only with
* integral values, the user  is warned that  the rational  integer  arithmetic
* class is still considerably slower than the built-in  integer data types. If
* the range  of values  anticipated will  fit into a  built-in  type, use that
* instead.
*
* In  addition  to  the  original  COOL Rational class, vnl_rational is able to
* represent plus and minus infinity.  An  other  interesting  addition  is  the
* possibility  to construct a rational from a double.  This allows for lossless
* conversion from e.g. double 1.0/3.0 to the rational number 1/3, hence no more
* rounding errors.  This is implemented with continued fraction approximations.
*
* \author
* Copyright (C) 1991 Texas Instruments Incorporated.
*
* Permission is granted to any individual or institution to use, copy, modify,
* and distribute this software, provided that this complete copyright and
* permission notice is maintained, intact, in all copies and supporting
* documentation.
*
* Texas Instruments Incorporated provides this software "as is" without
* express or implied warranty.
*
   \verbatim
   Modifications
    Peter Vanroose, 13 July 2001: Added continued fraction cnstrctr from double
    Peter Vanroose, 10 July 2001: corrected operator%=()
    Peter Vanroose, 10 July 2001: corrected ceil() and floor() for negative args
    Peter Vanroose, 10 July 2001: extended operability range of += by using gcd
    Peter Vanroose, 10 July 2001: added abs().
    Peter Vanroose, 10 July 2001: removed state data member and added Inf repres
    Peter Vanroose,  9 July 2001: ported to vnl from COOL
   \endverbatim
*/

#include <iostream>
#include <assert.h>
#include <cmath> // for sqrt

namespace VNL {

/** Infinite precision rational numbers.
*
* The  vnl_rational  class  provides  infinite  precision rational numbers and
* arithmetic, using the built-in type long, for the numerator and denominator.
* Implicit conversion to the system defined types short, int, long, float, and
* double is supported by  overloaded  operator member functions.  Although the
* rational class makes judicous use  of inline  functions and  deals only with
* integral values, the user  is warned that  the rational  integer  arithmetic
* class is still considerably slower than the built-in  integer data types. If
* the range  of values  anticipated will  fit into a  built-in  type, use that
* instead.
*
* In  addition  to  the  original  COOL Rational class, vnl_rational is able to
* represent plus and minus infinity.  An  other  interesting  addition  is  the
* possibility  to construct a rational from a double.  This allows for lossless
* conversion from e.g. double 1.0/3.0 to the rational number 1/3, hence no more
* rounding errors.  This is implemented with continued fraction approximations.
*
*/
class Rational {
public:
/** Creates a rational with given numerator and denominator.
*  Default constructor gives 0.
*  Also serves as automatic cast from long to Rational.
*  The only input which is not allowed is (0,0);
*  the denominator is allowed to be 0, to represent +Inf or -Inf.
*/
  inline Rational (long num = 0L, long den = 1L)
    : num_(num), den_(den) { assert(num!=0||den!=0); Normalize(); }
  inline Rational (int num, int den = 1)
    : num_(num), den_(den) { assert(num!=0||den!=0); Normalize(); }
  inline Rational (unsigned int num, unsigned int den = 1)
    : num_((long)num), den_((long)den) { assert(num!=0||den!=0); Normalize(); }
/** Creates a rational from a double.
*  This is done by computing the continued fraction approximation for d.
*  Note that this is explicitly *not* an automatic type conversion.
*/
  explicit Rational (double d);
  //  Copy constructor
  inline Rational (Rational const& from)
    : num_(from.Numerator()), den_(from.Denominator()) {}
  //  Destructor
  inline ~Rational() {}
  //  Assignment: overwrite an existing Rational
  inline void Set(long num, long den) { assert(num!=0||den!=0); num_=num; den_=den; Normalize(); }

/** Return the numerator of the (simplified) rational number representation.
*/
  inline long Numerator () const { return num_; }
/** Return the denominator of the (simplified) rational number representation.
*/
  inline long Denominator () const { return den_; }

/** Copies the contents and state of rhs rational over to the lhs.
*/
  inline Rational& operator= (Rational const& rhs) {
    num_ = rhs.Numerator(); den_ = rhs.Denominator(); return *this; }

/** Returns true if the two rationals have the same representation.
*/
  inline bool operator== (Rational const& rhs) const {
    return num_ == rhs.Numerator() && den_ == rhs.Denominator(); }
  inline bool operator!= (Rational const& rhs) const { return !operator==(rhs); }
  inline bool operator== (long rhs) const { return num_ == rhs && den_ == 1; }
  inline bool operator!= (long rhs) const { return !operator==(rhs); }
  inline bool operator== (int rhs) const { return num_ == rhs && den_ == 1; }
  inline bool operator!= (int rhs) const { return !operator==(rhs); }

/** Unary minus - returns the negation of the current rational.
*/
  inline Rational operator-() const { return Rational(-num_, den_); }
/** Unary plus - returns the current rational.
*/
  inline Rational operator+() const { return *this; }
/** Unary not - returns true if rational is equal to zero.
*/
  inline bool operator!() const { return num_ == 0L; }
/** Returns the absolute value of the current rational.
*/
  inline Rational Abs() const { return Rational(num_<0?-num_:num_, den_); }
/** Replaces rational with 1/rational and returns it.
*  Inverting 0 gives +Inf, inverting +-Inf gives 0.
*/
  Rational& Invert () {
    long t = num_; num_ = den_; den_ = t; Normalize(); return *this; }

/** Plus/assign: replace lhs by lhs + rhs.
*  Note that +Inf + -Inf and -Inf + +Inf are undefined.
*/
  inline Rational& operator+= (Rational const& r) {
    if (den_ == r.Denominator()) num_ += r.Numerator();
    else { long c = Rational::GCD(den_,r.Denominator()); if (c==0) c=1;
           num_ = num_*(r.Denominator()/c) + (den_/c)*r.Numerator();
           den_ *= r.Denominator()/c; }
    assert(num_!=0 || den_ != 0); // +Inf + -Inf is undefined
    Normalize (); return *this;
  }
  inline Rational& operator+= (long r) { num_ += den_*r; return *this; }
/** Minus/assign: replace lhs by lhs - rhs.
*  Note that +Inf - +Inf and -Inf - -Inf are undefined.
*/
  inline Rational& operator-= (Rational const& r) {
    if (den_ == r.Denominator()) num_ -= r.num_;
    else { long c = Rational::GCD(den_,r.Denominator()); if (c==0) c=1;
           num_ = num_*(r.Denominator()/c) - (den_/c)*r.Numerator();
           den_ *= r.Denominator()/c; }
    assert(num_!=0 || den_ != 0); // +Inf - +Inf is undefined
    Normalize (); return *this;
  }
  inline Rational& operator-= (long r) { num_ -= den_*r; return *this; }
/** Multiply/assign: replace lhs by lhs * rhs.
*  Note that 0 * Inf and Inf * 0 are undefined.
*/
  inline Rational& operator*= (Rational const& r) {
    num_ *= r.Numerator(); den_ *= r.Denominator();
    assert(num_!=0 || den_ != 0); // 0 * Inf is undefined
    Normalize (); return *this;
  }
  inline Rational& operator*= (long r) {num_*=r;Normalize();return *this;}
/** Divide/assign: replace lhs by lhs / rhs.
*  Note that 0 / 0 and Inf / Inf are undefined.
*/
  inline Rational& operator/= (Rational const& r) {
    num_ *= r.Denominator(); den_ *= r.Numerator();
    assert(num_!=0 || den_ != 0); // 0/0, Inf/Inf undefined
    Normalize (); return *this;
  }
  inline Rational& operator/= (long r) {
    den_ *= r; assert(num_!=0 || den_ != 0); // 0/0 undefined
    Normalize (); return *this;
  }
/** Modulus/assign: replace lhs by lhs % rhs.
*  Note that r % Inf is r, and that r % 0 and Inf % r are undefined.
*/
  inline Rational& operator%= (Rational const& r) {
    assert(r.Numerator() != 0);
    if (den_ == r.Denominator()) num_ %= r.Numerator();
    else { long c = Rational::GCD(den_,r.Denominator()); if (c==0) c=1;
           num_ *= r.Denominator()/c;
           num_ %= (den_/c)*r.Numerator();
           den_ *= r.Denominator()/c; }
    Normalize (); return *this;
  }
  inline Rational& operator%=(long r){assert(r);num_%=den_*r;Normalize();return *this;}

/** Pre-increment (++r).\  No-op when +-Inf.
*/
  inline Rational& operator++ () { num_ += den_; return *this; }
/** Pre-decrement (--r).\  No-op when +-Inf.
*/
  inline Rational& operator-- () { num_ -= den_; return *this; }
/** Post-increment (r++).\  No-op when +-Inf.
*/
  inline Rational operator++(int){Rational b=*this;num_+=den_;return b;}
/** Post-decrement (r--).\  No-op when +-Inf.
*/
  inline Rational operator--(int){Rational b=*this;num_-=den_;return b;}

  inline bool operator< (Rational const& rhs) const {
    if (den_ == rhs.Denominator())   // If same denominator
      return num_ < rhs.Numerator(); // includes the case -Inf < +Inf
    // note that denominator is always >= 0:
    else
      return num_ * rhs.Denominator() < den_ * rhs.Numerator();
  }
  inline bool operator> (Rational const& r) const { return r < *this; }
  inline bool operator<= (Rational const& r) const { return !operator>(r); }
  inline bool operator>= (Rational const& r) const { return !operator<(r); }
  inline bool operator< (long r) const { return num_ < den_ * r; }
  inline bool operator> (long r) const { return num_ > den_ * r; }
  inline bool operator<= (long r) const { return !operator>(r); }
  inline bool operator>= (long r) const { return !operator<(r); }
  inline bool operator< (int r) const { return num_ < den_ * r; }
  inline bool operator> (int r) const { return num_ > den_ * r; }
  inline bool operator<= (int r) const { return !operator>(r); }
  inline bool operator>= (int r) const { return !operator<(r); }
  inline bool operator< (double r) const { return num_ < den_ * r; }
  inline bool operator> (double r) const { return num_ > den_ * r; }
  inline bool operator<= (double r) const { return !operator>(r); }
  inline bool operator>= (double r) const { return !operator<(r); }

/** Converts rational value to integer by truncating towards zero.
*/
  inline long Truncate () const { assert(den_ != 0);  return num_/den_; }
/** Converts rational value to integer by truncating towards negative infinity.
*/
  inline long Floor () const { long t = Truncate();
    return num_<0L && (num_%den_) != 0 ? t-1 : t; }
/** Converts rational value to integer by truncating towards positive infinity.
*/
  inline long Ceil () const { long t = Truncate();
    return num_>0L && (num_%den_) != 0 ? t+1 : t; }
/** Rounds rational to nearest integer.
*/
  inline long Round () const { long t = Truncate();
    if (num_ < 0) return ((-num_)%den_) >= 0.5*den_ ? t-1 : t;
    else          return   (num_ %den_) >= 0.5*den_ ? t+1 : t;
  }

  // Implicit conversions
  inline operator short () {
    long t = Truncate (); short r = (short)t;
    assert(r == t); // abort on underflow or overflow
    return r;
  }
  inline operator int () {
    long t = Truncate (); int r = (int)t;
    assert(r == t); // abort on underflow or overflow
    return r;
  }
  inline operator long () const { return Truncate(); }
  inline operator long () { return Truncate(); }
  inline operator float () const { return ((float)num_)/((float)den_); }
  inline operator float () { return ((float)num_)/((float)den_); }
  inline operator double () const { return ((double)num_)/((double)den_); }
  inline operator double () { return ((double)num_)/((double)den_); }

/** Calculate greatest common divisor of two integers.
*  Used to simplify rational number.
*/
  static inline long GCD (long l1, long l2) {
    while (l2!=0) { long t = l2; l2 = l1 % l2; l1 = t; }
    return l1<0 ? (-l1) : l1;
  }

private:
  long num_; //!< Numerator portion
  long den_; //!< Denominator portion

/** Private function to Normalize numerator/denominator of rational number.
*  If num_ and den_ are both nonzero, their gcd is made 1 and den_ made positive.
*  Otherwise, the nonzero den_ is set to 1 or the nonzero num_ to +1 or -1.
*/
  inline void Normalize () {
    if (num_ == 0) { den_ = 1; return; } // zero
    if (den_ == 0) { num_ = (num_>0) ? 1 : -1; return; } // +-Inf
    if (num_ != 1 && num_ != -1 && den_ != 1) {
      long common = Rational::GCD (num_, den_);
      if (common != 1) { num_ /= common; den_ /= common; }
    }
    // if negative, put sign in numerator:
    if (den_ < 0) { num_ *= -1; den_ *= -1; }
  }
};

/** formatted output.
*/
inline std::ostream& operator<< (std::ostream& s, Rational const& r) {
  return s << r.Numerator() << "/" << r.Denominator();
}

/** simple input.
*/
inline std::istream& operator>> (std::istream& s, Rational& r) {
  long n, d; s >> n >> d;
  r.Set(n,d); return s;
}

/** Returns the addition of two rational numbers.
*/
inline Rational operator+ (Rational const& r1, Rational const& r2) {
  Rational result(r1); return result += r2;
}
inline Rational operator+ (Rational const& r1, long r2) {
  Rational result(r1); return result += r2;
}
inline Rational operator+ (Rational const& r1, int r2) {
  Rational result(r1); return result += (long)r2;
}
inline Rational operator+ (long r2, Rational const& r1) {
  Rational result(r1); return result += r2;
}
inline Rational operator+ (int r2, Rational const& r1) {
  Rational result(r1); return result += (long)r2;
}

/** Returns the difference of two rational numbers.
*/
inline Rational operator- (Rational const& r1, Rational const& r2) {
  Rational result(r1); return result -= r2;
}
inline Rational operator- (Rational const& r1, long r2) {
  Rational result(r1); return result -= r2;
}
inline Rational operator- (Rational const& r1, int r2) {
  Rational result(r1); return result -= (long)r2;
}
inline Rational operator- (long r2, Rational const& r1) {
  Rational result(-r1); return result += r2;
}
inline Rational operator- (int r2, Rational const& r1) {
  Rational result(-r1); return result += (long)r2;
}

/** Returns the multiplication of two rational numbers.
*/
inline Rational operator* (Rational const& r1, Rational const& r2) {
  Rational result(r1); return result *= r2;
}
inline Rational operator* (Rational const& r1, long r2) {
  Rational result(r1); return result *= r2;
}
inline Rational operator* (Rational const& r1, int r2) {
  Rational result(r1); return result *= (long)r2;
}
inline Rational operator* (long r2, Rational const& r1) {
  Rational result(r1); return result *= r2;
}
inline Rational operator* (int r2, Rational const& r1) {
  Rational result(r1); return result *= (long)r2;
}

/** Returns the division of two rational numbers.
*/
inline Rational operator/ (Rational const& r1, Rational const& r2) {
  Rational result(r1); return result /= r2;
}
inline Rational operator/ (Rational const& r1, long r2) {
  Rational result(r1); return result /= r2;
}
inline Rational operator/ (Rational const& r1, int r2) {
  Rational result(r1); return result /= (long)r2;
}
inline Rational operator/ (long r1, Rational const& r2) {
  Rational result(r1); return result /= r2;
}
inline Rational operator/ (int r1, Rational const& r2) {
  Rational result((long)r1); return result /= r2;
}

/** Returns the remainder of r1 divided by r2.
*/
inline Rational operator% (Rational const& r1, Rational const& r2) {
  Rational result(r1); return result %= r2;
}
inline Rational operator% (Rational const& r1, long r2) {
  Rational result(r1); return result %= r2;
}
inline Rational operator% (Rational const& r1, int r2) {
  Rational result(r1); return result %= (long)r2;
}
inline Rational operator% (long r1, Rational const& r2) {
  Rational result(r1); return result %= r2;
}
inline Rational operator% (int r1, Rational const& r2) {
  Rational result((long)r1); return result %= r2;
}

inline bool operator== (int  r1, Rational const& r2) { return r2==r1; }
inline bool operator== (long r1, Rational const& r2) { return r2==r1; }
inline bool operator!= (int  r1, Rational const& r2) { return r2!=r1; }
inline bool operator!= (long r1, Rational const& r2) { return r2!=r1; }
inline bool operator<  (int  r1, Rational const& r2) { return r2> r1; }
inline bool operator<  (long r1, Rational const& r2) { return r2> r1; }
inline bool operator>  (int  r1, Rational const& r2) { return r2< r1; }
inline bool operator>  (long r1, Rational const& r2) { return r2< r1; }
inline bool operator<= (int  r1, Rational const& r2) { return r2>=r1; }
inline bool operator<= (long r1, Rational const& r2) { return r2>=r1; }
inline bool operator>= (int  r1, Rational const& r2) { return r2<=r1; }
inline bool operator>= (long r1, Rational const& r2) { return r2<=r1; }

inline long Truncate (Rational const& r) { return r.Truncate(); }
inline long Floor (Rational const& r) { return r.Floor(); }
inline long Ceil (Rational const& r) { return r.Ceil(); }
inline long Round (Rational const& r) { return r.Round(); }

inline Rational Abs(Rational const& x) { return x<0L ? -x : x; }
inline Rational SquaredMagnitude(Rational const& x) { return x*x; }
inline Rational Sqr(Rational const& x) { return x*x; }
inline bool IsNaN(Rational const& ){return false;}
inline bool IsFinite(Rational const& x){return x.Denominator() != 0L;} 

}; // End namespace VNL

// Standard stuff must be outside VNL namespace

namespace std {
  inline VNL::Rational abs (VNL::Rational const& x) { return x.Abs(); }
  inline VNL::Rational sqrt(VNL::Rational const& x) 
	{ 
	  // On Win32, sqrt isn't in the standard namespace, aaargh, muppets!
#ifdef _WIN32
	  return VNL::Rational( ::sqrt(double(x))); 
#else
	  return VNL::Rational(std::sqrt(double(x))); 
#endif
	}
}

// Includes must be outsided namespace
#include <complex>
#include <VNL/complextraits.h>
#include <VNL/numerictraits.h>



namespace VNL {

template <>
struct ComplexTraits<Rational>
{
  enum { isreal = true };
  static Rational Conjugate(Rational x) { return x; }
  static std::complex<Rational> Complexify(Rational x) { return std::complex<Rational>(x, Rational(0,1)); }
};

template <>
class NumericTraits<Rational> {
public:
/** Additive identity.
*/
  static const Rational zero; // = 0L
/** Multiplicative identity.
*/
  static const Rational one; // = 1L
/** Return value of abs().
*/
  typedef Rational abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef Rational double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
class NumericTraits<Rational const> : public NumericTraits<Rational> {
};


inline bool vnl_math_isnan(std::complex<Rational> const& z)
  { return vnl_math_isnan(std::real(z)) || vnl_math_isnan(std::imag(z)); }
inline bool vnl_math_isfinite(std::complex<Rational> const& z)
  { return vnl_math_isfinite(std::real(z)) && vnl_math_isfinite(std::imag(z)); }
inline Rational vnl_math_squared_magnitude(std::complex<Rational> const& z) { return std::norm(z); }
inline Rational vnl_math_abs(std::complex<Rational> const& z) { return std::sqrt(std::norm(z)); }
inline std::complex<Rational> vnl_math_sqr(std::complex<Rational> const& z) { return z*z; }
inline std::ostream& operator<< (std::ostream& s, std::complex<Rational> const& z) {
  return s << '(' << z.real() << "," << z.imag() << ')'; }
inline std::istream& operator>> (std::istream& s, std::complex<Rational>& z) {
  Rational r, i; s >> r >> i; z=std::complex<Rational>(r,i); return s; }


template <>
struct ComplexTraits<std::complex<Rational> >
{
  enum { isreal = false };
  static std::complex<Rational> conjugate(std::complex<Rational> x) {return std::complex<Rational>(x.real(),-x.imag());}
  static std::complex<Rational> complexify(std::complex<Rational> x) { return x; }
};

template <>
class NumericTraits<std::complex<Rational> > {
public:
/** Additive identity.
*/
  static const std::complex<Rational> zero; // = std::complex<Rational>(0L,0L)
/** Multiplicative identity.
*/
  static const std::complex<Rational> one; // = std::complex<Rational>(1L,0L)
/** Return value of abs().
*/
  typedef Rational abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef std::complex<Rational> double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef std::complex<Rational> real_t; // should be std::complex<double>, but that gives casting problems
};

template <>
class NumericTraits<std::complex<Rational> const> 
  : public NumericTraits<std::complex<Rational> > 
{
};


}; // End namespace VNL

#endif // Rational_h_

// This is ./vxl/VNL/bignum.h
#ifndef vnl_bignum_h_
#define vnl_bignum_h_

/**
* \file
* \brief Infinite precision integers

*
* The vnl_bignum class implements near-infinite precision integers
* and arithmetic by using a dynamic bit vector. A
* vnl_bignum object will grow in size as necessary to hold its
* integer value.  Implicit conversion to the system defined
* types: short, int, long, float, double and long double
* is supported by overloaded operator member functions.
* Addition and subtraction operators are performed by
* simple bitwise addition and subtraction on
* unsigned short boundaries with checks for carry flag propagation.
* The multiplication, division, and remainder operations
* utilize the algorithms from Knuth's Volume 2 of "The
* Art of Computer Programming". However, despite the use of
* these algorithms and inline member functions, arithmetic
* operations on vnl_bignum objects are considerably slower than
* the built-in integer types that use hardware integer arithmetic
* capabilities.
*
* The vnl_bignum class supports the parsing of character string
* representations of all the literal number formats. The following
* table shows an example of a character string
* representation on the left and a brief description of the
* interpreted meaning on the right:
*
* Character String  Interpreted Meaning
* 1234              1234
* 1234l             1234
* 1234L             1234
* 1234u             1234
* 1234U             1234
* 1234ul            1234
* 1234UL            1234
* 01234             1234 in octal (leading 0)
* 0x1234            1234 in hexadecimal (leading 0x)
* 0X1234            1234 in hexadecimal (leading 0X)
* 123.4             123 (value truncated)
* 1.234e2           123 (exponent expanded/truncated)
* 1.234e-5          0 (truncated value less than 1)
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
    Peter Vanroose, 24 January 2002: ported to vnl from COOL
   \endverbatim
*/

#include <iostream>
#include <cmath> // for sqrt(double)
#include <string>

namespace VNL {

class BigNum;

int MagnitudeCmp(const BigNum&, const BigNum&);
void Add(const BigNum&, const BigNum&, BigNum&);
void Subtract(const BigNum&, const BigNum&, BigNum&);
void MultiplyAux(const BigNum&, unsigned short d, BigNum&, unsigned short i);
/** Normalize for division.
*/
unsigned short Normalize(const BigNum&, const BigNum&, BigNum&, BigNum&);
/** Divide digit.
*/
void DivideAux(const BigNum&, unsigned short, BigNum&, unsigned short&);
unsigned short EstimateQHat(const BigNum&, const BigNum&, unsigned short);
/** Multiply quotient and subtract.
*/
unsigned short MultiplySubtract(BigNum&, const BigNum&, unsigned short, unsigned short);
void Divide(const BigNum&, const BigNum&, BigNum&, BigNum&);
BigNum LeftShift(const BigNum& b1, int l);
BigNum RightShift(const BigNum& b1, int l);

/** formatted output.
*/
std::ostream& operator<<(std::ostream& s, BigNum const& r);

/** simple input.
*/
std::istream& operator>>(std::istream& s, BigNum& r);

/** Infinite precision integers.
*
* The BigNum class implements near-infinite precision integers
* and arithmetic by using a dynamic bit vector. A
* BigNum object will grow in size as necessary to hold its
* integer value.  Implicit conversion to the system defined
* types: short, int, long, float, double and long double 
* is supported by overloaded operator member functions.
* Addition and subtraction operators are performed by
* simple bitwise addition and subtraction on
* unsigned short boundaries with checks for carry flag propagation.
* The multiplication, division, and remainder operations
* utilize the algorithms from Knuth's Volume 2 of "The
* Art of Computer Programming". However, despite the use of
* these algorithms and inline member functions, arithmetic
* operations on BigNum objects are considerably slower than
* the built-in integer types that use hardware integer arithmetic
* capabilities.
*
* The BigNum class supports the parsing of character string
* representations of all the literal number formats. The following
* table shows an example of a character string
* representation on the left and a brief description of the
* interpreted meaning on the right:
*
* Character String  Interpreted Meaning
* 1234              1234
* 1234l             1234
* 1234L             1234
* 1234u             1234
* 1234U             1234
* 1234ul            1234
* 1234UL            1234
* 01234             1234 in octal (leading 0)
* 0x1234            1234 in hexadecimal (leading 0x)
* 0X1234            1234 in hexadecimal (leading 0X)
* 123.4             123 (value truncated)
* 1.234e2           123 (exponent expanded/truncated)
* 1.234e-5          0 (truncated value less than 1)
*
*/
class BigNum {
public:
  BigNum();                        // Void constructor
  BigNum(long);                    // Long constructor
  BigNum(unsigned long);           // Unsigned Long constructor
  BigNum(int);                     // Int constructor
  BigNum(unsigned int);            // Unsigned Int constructor
  BigNum(double);                  // Double constructor
  BigNum(long double);             // Long Double constructor
  BigNum(BigNum const&);       // Copy constructor
  BigNum(const char*);             // String constructor
  ~BigNum();                       // Destructor

  operator short() const;              // Implicit type conversion
  operator int() const;                // Implicit type conversion
  operator long() const;               // Implicit type conversion
  operator float() const;              // Implicit type conversion
  operator double() const;             // Implicit type conversion
  operator long double() const;        // Implicit type conversion
  inline operator short() { return ((const BigNum*)this)->operator short(); }
  inline operator int() { return ((const BigNum*)this)->operator int(); }
  inline operator long() { return ((const BigNum*)this)->operator long(); }
  inline operator float() { return ((const BigNum*)this)->operator float(); }
  inline operator double() { return ((const BigNum*)this)->operator double(); }
  inline operator long double() { return ((const BigNum*)this)->operator long double(); }

  BigNum operator-() const;        // Unary minus operator
  inline BigNum operator+() { return *this; } // Unary plus operator
  inline BigNum operator+() const { return *this; } // Unary plus operator

  BigNum& operator=(const BigNum&); // Assignment operator

  BigNum operator<<(int l) const;  // Bit shift
  BigNum operator>>(int l) const;  // Bit shift
  BigNum operator+(BigNum const& r) const;
  inline BigNum& operator+=(BigNum const& r) { return *this = operator+(r); }
  inline BigNum& operator-=(BigNum const& r) { return *this = operator+(-r); }
  BigNum& operator*=(BigNum const& r);
  BigNum& operator/=(BigNum const& r);
  BigNum& operator%=(BigNum const& r);
  inline BigNum& operator<<=(int l) { return *this = *this << l; }
  inline BigNum& operator>>=(int l) { return *this = *this >> l; }

/** prefix increment (++b).
*/
  BigNum& operator++();
/** decrement.
*/
  BigNum& operator--();
/** postfix increment (b++).
*/
  inline BigNum operator++(int) { BigNum b=(*this); operator++(); return b; }
/** decrement.
*/
  inline BigNum operator--(int) { BigNum b=(*this); operator--(); return b; }

  bool operator==(BigNum const&) const; // equality
  bool operator< (BigNum const&) const; // less than
  inline bool operator!=(BigNum const& r) const { return !operator==(r); }
  inline bool operator> (BigNum const& r) const { return r<(*this); }
  inline bool operator<=(BigNum const& r) const { return !operator>(r); }
  inline bool operator>=(BigNum const& r) const { return !operator<(r); }
  inline bool operator==(long r) const { return operator==(BigNum(r)); }
  inline bool operator!=(long r) const { return !operator==(BigNum(r)); }
  inline bool operator< (long r) const { return operator<(BigNum(r)); }
  inline bool operator> (long r) const { return BigNum(r) < (*this); }
  inline bool operator<=(long r) const { return !operator>(BigNum(r)); }
  inline bool operator>=(long r) const { return !operator<(BigNum(r)); }
  inline bool operator==(int r) const { return operator==(long(r)); }
  inline bool operator!=(int r) const { return !operator==(long(r)); }
  inline bool operator< (int r) const { return operator<(long(r)); }
  inline bool operator> (int r) const { return BigNum(long(r)) < (*this); }
  inline bool operator<=(int r) const { return !operator>(long(r)); }
  inline bool operator>=(int r) const { return !operator<(long(r)); }
  inline bool operator==(double r) const { return r == operator double(); }
  inline bool operator!=(double r) const { return r != operator double(); }
  inline bool operator< (double r) const { return r > operator double(); }
  inline bool operator> (double r) const { return r < operator double(); }
  inline bool operator<=(double r) const { return r >= operator double(); }
  inline bool operator>=(double r) const { return r <= operator double(); }
  inline bool operator==(long double r) const { return r == operator long double(); }
  inline bool operator!=(long double r) const { return r != operator long double(); }
  inline bool operator< (long double r) const { return r > operator long double(); }
  inline bool operator> (long double r) const { return r < operator long double(); }
  inline bool operator<=(long double r) const { return r >= operator long double(); }
  inline bool operator>=(long double r) const { return r <= operator long double(); }

  inline BigNum Abs() const { return operator<(0L) ? operator-() : *this; }

  void Dump(std::ostream& = std::cout) const;     // Dump contents of BigNum

  friend int MagnitudeCmp(const BigNum&, const BigNum&);
  friend void Add(const BigNum&, const BigNum&, BigNum&);
  friend void Subtract(const BigNum&, const BigNum&, BigNum&);
  friend void MultiplyAux(const BigNum&, unsigned short, BigNum&, unsigned short);
  friend unsigned short Normalize(const BigNum&, const BigNum&, BigNum&, BigNum&);
  friend void DivideAux(const BigNum&, unsigned short, BigNum&, unsigned short&);
  friend unsigned short EstimateQHat(const BigNum&, const BigNum&, unsigned short);
  friend unsigned short MultiplySubtract(BigNum&, const BigNum&, unsigned short, unsigned short);
  friend void Divide(const BigNum&, const BigNum&, BigNum&, BigNum&);
  friend BigNum LeftShift(const BigNum& b1, int l);
  friend BigNum RightShift(const BigNum& b1, int l);
  friend std::ostream& operator<< (std::ostream&, const BigNum&);
  friend std::istream& operator>> (std::istream&, BigNum&);
  friend std::string& BigNumToString (std::string& s, const BigNum& b);
  friend BigNum& BigNumFromString (BigNum& b, const std::string& s);

private:
  unsigned short count; // Number of data elements
  int sign;    // Sign of BigNum (+,-,or 0)
  unsigned short* data;     // Pointer to data value

  void XToBigNum(const char *s);       // convert hex to BigNum
  int  DToBigNum(const char *s);       // convert decimal to BigNum
  void OToBigNum(const char *s);       // convert octal to BigNum
  void ExpToBigNum(const char *s);     // convert exponential to BigNum

  void Resize(short);                  // Resize BigNum data
  BigNum& Trim();                  // Trim BigNum data
};


/** Convert the number to a decimal representation in a string.
*/
std::string& BigNumToString (std::string& s, const BigNum& b);

/** Convert the number from a decimal representation in a string.
*/
BigNum& BigNumFromString (BigNum& b, const std::string& s);

/** Returns the addition of two bignum numbers.
*/
inline BigNum operator+(BigNum const& r1, long r2) { return r1+BigNum(r2); }
inline BigNum operator+(BigNum const& r1, int r2) { return r1+long(r2); }
inline BigNum operator+(BigNum const& r1, double r2) { return r1+BigNum(r2); }
inline BigNum operator+(BigNum const& r1, long double r2) { return r1+BigNum(r2); }
inline BigNum operator+(long r2, BigNum const& r1) { return r1 + r2; }
inline BigNum operator+(int r2, BigNum const& r1) { return r1 + r2; }
inline BigNum operator+(double r2, BigNum const& r1) { return r1 + r2; }
inline BigNum operator+(long double r2, BigNum const& r1) { return r1 + r2; }

/** Returns the difference of two bignum numbers.
*/
inline BigNum operator-(BigNum const& r1, BigNum const& r2) { return r1 + (-r2); }
inline BigNum operator-(BigNum const& r1, long r2) { return r1 + (-r2); }
inline BigNum operator-(BigNum const& r1, int r2) { return r1 + (-r2); }
inline BigNum operator-(BigNum const& r1, double r2) { return r1 + (-r2); }
inline BigNum operator-(BigNum const& r1, long double r2) { return r1 + (-r2); }
inline BigNum operator-(long r2, BigNum const& r1) { return -(r1 + (-r2)); }
inline BigNum operator-(int r2, BigNum const& r1) { return -(r1 + (-r2)); }
inline BigNum operator-(double r2, BigNum const& r1) { return -(r1 + (-r2)); }
inline BigNum operator-(long double r2, BigNum const& r1) { return -(r1 + (-r2)); }

/** Returns the multiplication of two bignum numbers.
*/
inline BigNum operator*(BigNum const& r1, BigNum const& r2) {
  BigNum result(r1); return result *= r2;
}
inline BigNum operator*(BigNum const& r1, long r2) {
  BigNum result(r1); return result *= BigNum(r2);
}
inline BigNum operator*(BigNum const& r1, int r2) {
  BigNum result(r1); return result *= (long)r2;
}
inline BigNum operator*(BigNum const& r1, double r2) {
  BigNum result(r1); return result *= (BigNum)r2;
}
inline BigNum operator*(BigNum const& r1, long double r2) {
  BigNum result(r1); return result *= (BigNum)r2;
}
inline BigNum operator*(long r2, BigNum const& r1) {
  BigNum result(r1); return result *= r2;
}
inline BigNum operator*(int r2, BigNum const& r1) {
  BigNum result(r1); return result *= (long)r2;
}
inline BigNum operator*(double r2, BigNum const& r1) {
  BigNum result(r1); return result *= (BigNum)r2;
}
inline BigNum operator*(long double r2, BigNum const& r1) {
  BigNum result(r1); return result *= (BigNum)r2;
}

/** Returns the division of two bignum numbers.
*/
inline BigNum operator/(BigNum const& r1, BigNum const& r2) {
  BigNum result(r1); return result /= r2;
}
inline BigNum operator/(BigNum const& r1, long r2) {
  BigNum result(r1); return result /= r2;
}
inline BigNum operator/(BigNum const& r1, int r2) {
  BigNum result(r1); return result /= (long)r2;
}
inline BigNum operator/(BigNum const& r1, double r2) {
  BigNum result(r1); return result /= (BigNum)r2;
}
inline BigNum operator/(BigNum const& r1, long double r2) {
  BigNum result(r1); return result /= (BigNum)r2;
}
inline BigNum operator/(long r1, BigNum const& r2) {
  BigNum result(r1); return result /= r2;
}
inline BigNum operator/(int r1, BigNum const& r2) {
  BigNum result((long)r1); return result /= r2;
}
inline BigNum operator/(double r1, BigNum const& r2) {
  BigNum result(r1); return result /= r2;
}
inline BigNum operator/(long double r1, BigNum const& r2) {
  BigNum result(r1); return result /= r2;
}

/** Returns the remainder of r1 divided by r2.
*/
inline BigNum operator%(BigNum const& r1, BigNum const& r2) {
  BigNum result(r1); return result %= r2;
}
inline BigNum operator%(BigNum const& r1, long r2) {
  BigNum result(r1); return result %= r2;
}
inline BigNum operator%(BigNum const& r1, int r2) {
  BigNum result(r1); return result %= (long)r2;
}
inline BigNum operator%(long r1, BigNum const& r2) {
  BigNum result(r1); return result %= r2;
}
inline BigNum operator%(int r1, BigNum const& r2) {
  BigNum result((long)r1); return result %= r2;
}

// Miscellaneous operators and functions

inline bool operator==(long r1, BigNum const& r2) { return r2==r1; }
inline bool operator!=(long r1, BigNum const& r2) { return r2!=r1; }
inline bool operator< (long r1, BigNum const& r2) { return r2> r1; }
inline bool operator> (long r1, BigNum const& r2) { return r2< r1; }
inline bool operator<=(long r1, BigNum const& r2) { return r2>=r1; }
inline bool operator>=(long r1, BigNum const& r2) { return r2<=r1; }

#if defined(VCL_VC)
//inline BigNum sqrt(BigNum const& x) { return BigNum(sqrt(double(x))); }
//inline BigNum abs(BigNum const& x) { return x.Abs(); }
#else
}; // End namespace VNL
namespace std {
  inline VNL::BigNum sqrt(VNL::BigNum const& x) { return VNL::BigNum(std::sqrt(double(x))); }
  inline VNL::BigNum abs(VNL::BigNum const& x) { return x.Abs(); }
};
namespace VNL {
#endif

inline BigNum Abs(BigNum const& x) { return x.Abs(); }
inline BigNum SquaredMagnitude(BigNum const& x) { return x*x; }
inline BigNum Sqr(BigNum const& x) { return x*x; }
inline bool IsNaN(BigNum const& ) { return false; }
inline bool IsFinite(BigNum const& ) { return true; } 

}; // End namespace VNL

// Inlcude can't be in namespace
#include <complex>
#include <VNL/complextraits.h>
#include <VNL/numerictraits.h>


namespace VNL {

template <>
class NumericTraits<BigNum> {
public:
/** Additive identity.
*/
  static const BigNum zero; // = 0L
/** Multiplicative identity.
*/
  static const BigNum one; // = 1L
/** Return value of abs().
*/
  typedef BigNum abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef BigNum double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef double real_t;
};

template <>
struct ComplexTraits<BigNum>
{
  enum { isreal = true };
  static BigNum Conjugate(BigNum x) { return x; }
  static std::complex<BigNum> Complexify(BigNum x)
  { return std::complex<BigNum>(x,BigNum(0L)); }
};

inline bool IsNaN(std::complex<BigNum> const& ) { return false; }
inline bool IsFinite(std::complex<BigNum> const&) { return true; }
inline BigNum SquaredMagnitude(std::complex<BigNum> const& z) { return std::norm(z); }
inline BigNum Abs(std::complex<BigNum> const& z) { return std::sqrt(std::norm(z)); }
inline std::complex<BigNum> Sqr(std::complex<BigNum> const& z) { return z*z; }
inline std::ostream& operator<<(std::ostream& s, std::complex<BigNum> const& z) {
  return s << '(' << z.real() << "," << z.imag() << ')'; }
inline std::istream& operator>>(std::istream& s, std::complex<BigNum>& z) {
  BigNum r, i; s >> r >> i; z=std::complex<BigNum>(r,i); return s; }


template <>
struct ComplexTraits<std::complex<BigNum> >
{
  enum { isreal = false };
  static std::complex<BigNum> Conjugate(std::complex<BigNum> x)
  { return std::complex<BigNum>(x.real(),-x.imag()); }
  static std::complex<BigNum> Complexify(std::complex<BigNum> x) { return x; }
};

template <>
class NumericTraits<std::complex<BigNum> > {
public:
/** Additive identity.
*/
  static const std::complex<BigNum> zero; // = 0L
/** Multiplicative identity.
*/
  static const std::complex<BigNum> one; // = 1L
/** Return value of abs().
*/
  typedef BigNum abs_t;
/** Name of a type twice as long as this one for accumulators and products.
*/
  typedef std::complex<BigNum> double_t;
/** Name of type which results from multiplying this type with a double.
*/
  typedef std::complex<BigNum> real_t;
};


}; // End namespace VNL


#endif // BigNum_h_

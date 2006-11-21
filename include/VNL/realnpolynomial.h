// This is vxl/VNL/real_npolynomial.h
#ifndef vnl_real_npolynomial_h_
#define vnl_real_npolynomial_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief contains class for polynomials with N variables

*
*  Implements a polynomial with N variables
*
*  \author Marc Pollefeys, ESAT-VISICS, K.U.Leuven
*  \date   12-08-97
*
    \verbatim
    Modifications
    Peter Vanroose 10 Oct 1999 - added simplify();
                                 determine nterms_ nvar_ ideg_ automatically
    Peter Vanroose 20 Oct 1999 - Added operator+(), - * and std::ostream <<
    dac (Manchester) 15/03/2001: Tidied up the documentation + added binary_io
    \endverbatim
*/


//-----------------------------------------------------------------------------

#include <VNL/vector.h>
#include <VNL/matrix.h>

namespace VNL {

/** real polynomial in N variables.
*    vnl_real_npolynomial represents a polynomial in multiple variables.
*    Used by vnl_rnpoly_solve which solves systems of polynomial equations.
*    Representation:  an N-omial (N terms) is represented by (1) a vector
*    with the N coefficients (vnl_vector<double>), and (2) a matrix with
*    N rows, the i-th row representing the exponents of term i, as follows:
*    (VNL::Matrix<int>) column k contains the (integer) exponent of variable
*    k.  Example: the polynomial \f$A X^3 + B XY + C Y^2 + D XY^2\f$ is
*    represented by the coefficients vector [A B C D] and the exponents
*    matrix
    \verbatim
      [3 0]
      [1 1]
      [0 2]
      [1 2].
    \endverbatim
*/

class RealNPolynomial
{
  friend class RNPolySolve;

 public:

  // Constructor-----------------------------------------------------------------
  RealNPolynomial() { } // don't use this. only here for the STL vector class.
  RealNPolynomial(const VNL::Vector<double>& c, const VNL::Matrix<int>& p);

  // Computations--------------------------------------------------------------

  double Eval(const VNL::Vector<double>& x);
  int Degree();
  RealNPolynomial operator-() const; // unary minus
  RealNPolynomial operator+(RealNPolynomial const& ) const;
  RealNPolynomial operator-(RealNPolynomial const& ) const;
  RealNPolynomial operator*(RealNPolynomial const& ) const;
  RealNPolynomial operator+(double ) const;
  RealNPolynomial operator-(double P) const { return operator+(-P); }
  RealNPolynomial operator*(double ) const;
  RealNPolynomial& operator*=(double P) { coeffs_ *= P; return *this; }
  RealNPolynomial operator/(double P) const { return operator*(1.0/P); }
  RealNPolynomial& operator/=(double P) { return operator*=(1.0/P); }
  friend std::ostream& operator<<(std::ostream& , RealNPolynomial const& );

  // nb also added functions to access the coeffs_ member variable

  //--- Data Access------------------------------------------------------------

/** Return the degree (highest power of x) of the polynomial.
*/
  int Degree() const { return ((int)coeffs_.size()) - 1; }

/** Access to the polynomial coefficients.
*/
  double& operator [] (int i)       { return coeffs_[i]; }
/** Access to the polynomial coefficients.
*/
  double  operator [] (int i) const { return coeffs_[i]; }

/** Return the vector of coefficients.
*/
  const VNL::Vector<double>& Coefficients() const { return coeffs_; }
/** Return the vector of coefficients.
*/
  VNL::Vector<double>& Coefficients()       { return coeffs_; }

/** Set vector of coefficients of each product.
*/
  void Set(const VNL::Vector<double> & c, const VNL::Matrix<int> & p);

/** Return the polynomial matrix.
* (ie specifying the variables in each product)
*/
  const VNL::Matrix<int>& PolyN() const { return polyn_; }

/** Return the vector of coefficients.
*/
  VNL::Matrix<int>& PolyN()       { return polyn_; }

 private:
  void _Simplify();
  double _Eval(const VNL::Matrix<double>& xn);

  // Data Members--------------------------------------------------------------

/** coefficients.
*/
  VNL::Vector<double> coeffs_;
/** degrees of every term for every variable.
*/
  VNL::Matrix<int>    polyn_;
/** number of variables = # columns of polyn_.
*/
  int                nvar_;
/** number of terms of polynomial.
*/
  int                nterms_;
/** max.\ degree of polynomial.
*/
  int                ideg_;
};

}; // End namespace VNL

#endif // RealNPolynomial_h_

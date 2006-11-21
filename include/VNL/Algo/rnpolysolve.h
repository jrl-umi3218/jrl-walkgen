// This is vxl/VNL/algo/rnpoly_solve.h
#ifndef vnl_rnpoly_solve_h_
#define vnl_rnpoly_solve_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Solves for roots of system of real polynomials

* \author Marc Pollefeys, ESAT-VISICS, K.U.Leuven
* \date   12-08-97
*
   \verbatim
   Modifications
    Peter Vanroose, 20 Oct 1999: implementation simplified through "cmplx" class for doing complex arithmetic.
    dac (Manchester) 28/03/2001: tidied up documentation
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
     May.2002 - Peter Vanroose - added operator*=(cmplx) and operator/=(cmplx)
   \endverbatim
*/

#include <VNL/vector.h>
#include <VNL/realnpolynomial.h>
#include <vector>

#ifdef static
# error "grr!!"
#endif

namespace VNL {

/** Solves for roots of system of real polynomials.
*  Calculates all the roots of a system of N polynomials in N variables
*  through continuation.
*  Adapted from the  PARALLEL CONTINUATION algorithm , written by Darrell
*  Stam, 1991, and further improved by  Kriegman and Ponce, 1992.
*/

class RNPolySolve
{
 public:
#ifndef _WIN32
  static const unsigned int M = 11;   // Maximum dimension of problem
  static const unsigned int T = 2500; // Max. number of terms in a polynomial
#else
  enum { M = 11 };   // Maximum dimension of problem
  enum { T = 2500 }; // Maximum number of terms in a polynomial
#endif

  // Constructor---------------------------------------------------------------

/** The constructor already does all the calculations.
*/
  inline RNPolySolve(std::vector<RealNPolynomial*> const& ps)
    : ps_(ps) { _Compute(); }

  // Operations----------------------------------------------------------------

/** Array of real parts of roots.
*/
  inline std::vector<VNL::Vector<double>*> Real() { return r_; }

/** Array of imaginary parts of roots.
*/
  inline std::vector<VNL::Vector<double>*> Imag() { return i_; }

/** Return real roots only.
*  Roots are real if the absolute value of their imaginary part is less than
*  the optional argument tol, which defaults to 1e-12 [untested]
*/
  std::vector<VNL::Vector<double>*> RealRoots(double tol = 1e-12);

  // Computations--------------------------------------------------------------

 private:
/** Compute roots using continuation algorithm.
*/
  bool _Compute();

  int _ReadInput(int ideg[M], int terms[M],
                 int polyn[M][T][M], double coeff[M][T]);

  // Data Members--------------------------------------------------------------
  std::vector<RealNPolynomial*> ps_;   // the input
  std::vector<VNL::Vector<double>*> r_; // the output (real part)
  std::vector<VNL::Vector<double>*> i_; // the output (imaginary part)
};

}; // End namespace VNL

#endif // vnl_rnpoly_solve_h_

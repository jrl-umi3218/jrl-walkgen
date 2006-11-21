#ifndef vnl_cpoly_roots_h_
#define vnl_cpoly_roots_h_

/**
*  \file
*  \brief  finds roots of a univariate polynomial with complex coefficients

*  \author fsm@robots.ox.ac.uk
*
   \verbatim
   Modifications
    dac (Manchester) March 28th 2001: Tidied documentation
   \endverbatim
*/

#include <complex>
#include <VNL/vector.h>


namespace VNL {

/** Find all the roots of a univariate polynomial with complex coefficients.
*  Class to find all the roots of a univariate polynomial f
*  with complex coefficients. Currently works by computing the
*  eigenvalues of the companion matrix of f.
*
*  The input vector a of coefficients are given to the constructor.
*  The polynomial is f = t^N + a[0] t^{N-1} + ... + a[N-1]
*  The roots can then be found in the 'solns' member.
*/

class CPolyRoots
{
public:
  CPolyRoots(VNL::Vector<std::complex<double> > const & a);
  CPolyRoots(VNL::Vector<double> const & a_real,
	     VNL::Vector<double> const & a_imag);

  // the roots can be found in here :
  VNL::Vector<std::complex<double> > solns;

private:
  unsigned N; //degree
/** does the actual work.
*/
  void _Compute(VNL::Vector<std::complex<double> > const & a);
};

}; // End namespace VNL

#endif // CPolyRoots_h_

// This is vxl/VNL/algo/brent.h
#ifndef vnl_brent_h_
#define vnl_brent_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \author awf@robots.ox.ac.uk
* \date   07 Dec 00
*
   \verbatim
   Modifications
   31 May 2001 Ian Scott (Manchester). Added some documentation
   31 May 2001 Ian Scott (Manchester). Added minimize_given_bounds_and_1st_f
   \endverbatim
*/

#include <VNL/costfunction.h>
#include <VNL/nonlinearminimizer.h>


namespace VNL {


struct BrentData;

/** Brent 1D minimizer.
* This minimised uses both golden section search and parbolic interpolation
* for a fast and robust function minimiser.
*/
class Brent : public NonlinearMinimizer
{
 public:
  Brent(CostFunction* functor);
 ~Brent();

/** Find a minimum of f(x) near to ax.
*/
  double Minimize(double ax);

/** Find the minimum value of f(x) within a<= x <= c.
* The minimum value is the return value, and *xmin the relevent value of x.
* You need to provide a bracket for the minimum
* Also returns fa = f(a), etc.
*/
  double MinimizeGivenBounds(double ax, double bx, double cx,
                               double tol,
                               double *xmin);

/** Save time over minimize_given_bounds() if you know f(b).
* This function avoids a single computation of f, if you already know
* it.
*/
  double MinimizeGivenBoundsAnd1stF(double ax, double bx, double fb,
                                         double cx,  double tol, double *xmin);

/** Given distinct points ax, and bx, find a bracket for the minimum.
* Return a bracket ax < bx < cx, f(b) < f(a), f(b) < f(c) for minimum.
* Also returns fa = f(a), etc.
*/
  void BracketMinimum(double *ax, double *bx, double *cx,
                       double *fa, double *fb, double *fc);

/** Given distinct points ax, and bx, find a bracket for the minimum.
* Return a bracket ax < bx < cx, f(b) < f(a), f(b) < f(c) for minimum.
*/
  void BracketMinimum(double *ax, double *bx, double *cx);

 protected:
  BrentData *p;
};

};  // End namespace VNL

#endif // Brent_h_

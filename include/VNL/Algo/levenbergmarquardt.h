// This is vxl/VNL/algo/levenberg_marquardt.h
#ifndef vnl_levenberg_marquardt_h_
#define vnl_levenberg_marquardt_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Levenberg Marquardt nonlinear least squares

* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   31 Aug 96
*
   \verbatim
   Modifications
    AGAP 160701 Some comments. Changed minimize to call the correct minimization
                routine.
    RWMC 001097 Added verbose flag to get rid of all that blathering.
    AWF  151197 Added trace flag to increase blather.
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*
*/

#include <iosfwd>
#include <VNL/vector.h>
#include <VNL/matrix.h>
#include <VNL/nonlinearminimizer.h>

namespace VNL{

  class LeastSquaresFunction;

/** Levenberg Marquardt nonlinear least squares.
*  vnl_levenberg_marquardt is an interface to the MINPACK routine lmdif,
*  and implements Levenberg Marquardt nonlinear fitting.  The function
*  to be minimized is passed as a vnl_least_squares_function object, which
*  may or may not wish to provide derivatives.  If derivatives are not
*  supplied, they are calculated by forward differencing, which costs
*  one function evaluation per dimension, but is perfectly accurate.
*  (See Hartley in ``Applications of Invariance in Computer Vision''
*  for example).
*/

class LevenbergMarquardt : public NonlinearMinimizer
{
 public:

/** Initialize with the function object that is to be minimized.
*/
  LevenbergMarquardt(LeastSquaresFunction& f) { _Init(&f); }

/** Initialize as above, and then run minimization.
*
* obsolete, as virtuals in base class vnl_nonlinear_minimizer not valid...
* i.e. if minimize() calls base::get_covariance(), it will call the
* base version rather than any overridden here or in classes derived
* from this.  This is an argument against computation in constructors.
* You should replace code like
*    LevenbergMarquardt lm(f, x);
* with
*    LevenbergMarquardt lm(f);
*    lm.minimize(x);
* Or
*    x = LevenbergMarquardt_minimize(f, x);
*/

#if 0
  LevenbergMarquardt(LeastSquaresFunction& f,
                          Vector<double>& x)
  {
    _Init(&f);
    Minimize(x);
  }
#endif

  ~LevenbergMarquardt();

/** Minimize the function supplied in the constructor until convergence or failure.
*  On return, x is such that f(x) is the lowest value achieved.
*  Returns true for convergence, false for failure.
*  Does not use the gradient even if the cost function provides one.
*/
  bool MinimizeWithoutGradient(Vector<double>& x);

/** Minimize the function supplied in the constructor until convergence or failure.
*  On return, x is such that f(x) is the lowest value achieved.
*  Returns true for convergence, false for failure.
*  The cost function must provide a gradient.
*/
  bool MinimizeUsingGradient  (Vector<double>& x);

/** Calls minimize_using_gradient() or minimize_without_gradient().
* , depending on whether the cost function provides a gradient.
*/
  bool Minimize(Vector<double>& x);

  // Coping with failure-------------------------------------------------------

/** Provide an ASCII diagnosis of the last minimization on std::ostream.
*/
  void DiagnoseOutcome(/*std::cerr*/) const;
  void DiagnoseOutcome(std::ostream&) const;

/** Return J'*J computed at last minimum.
*/
  Matrix<double> const& GetJtJ();

 protected:

  LeastSquaresFunction* f_;
  Matrix<double>* fdjac_; // Computed during lmdif/lmder
  Vector<int>*    ipvt_;  // Also computed, both needed to get J'*J at end.

  Matrix<double>* covariance_;
  bool set_covariance_; // Set if covariance_ holds J'*J

  void _Init(LeastSquaresFunction* f);

  // Communication with callback
  friend class LevenbergMarquardt_Activate;
  static void lmdif_lsqfun(int* m, int* n, double* x,
                           double* fx, int* iflag);
  static void lmder_lsqfun(int* m, int* n, double* x,
                           double* fx, double* fJ, int*, int* iflag);
};

/** Find minimum of "f", starting at "initial_estimate", and return.
*/
Vector<double> LevenbergMarquardtMinimize(LeastSquaresFunction& f,
					  Vector<double> const& initial_estimate);


}; // End namespace VNL

#endif // LevenbergMarquardt_h_

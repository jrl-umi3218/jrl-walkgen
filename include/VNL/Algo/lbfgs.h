// This is vxl/VNL/algo/lbfgs.h
#ifndef vnl_lbfgs_h_
#define vnl_lbfgs_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Limited memory Broyden Fletcher Goldfarb Shannon minimization

* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   22 Aug 99
*
   \verbatim
   Modifications
    990822 AWF Initial version.
    dac (Manchester) 28/03/2001: tidied up documentation
    scottim 4/02/2002: Added a better description
   \endverbatim
*
*/

#include <VNL/costfunction.h>
#include <VNL/nonlinearminimizer.h>

namespace VNL {


/** Limited memory Broyden Fletcher Goldfarb Shannon minimization.
* Considered to be the best optimisation algorithm for functions 
* which are well behaved (i.e. locally smooth
* without too many local minima,) have 1st derivatives available,
* and do not have a structure that makes them suitable for alternative
* methods (e.g. if f(x) is a sum of squared terms you should use
* vnl_levenberg_marquardt.)
*
* This limited-memory rank-2 quasi-newton method
* maintains an estimate of (the inverse of) the Hessian matrix of f at x.
* Unlike Newton's method, it doesn't need 2nd derivatives of f(x),
* has superlinear rather than quadratic convergence and is
* better behaved away from minima. 2 ranks of this matrix are updated at each
* step. In order to reduce memory and time requirements, this limited memory
* version of BFGS only maintains a certain number of vector corrections
* to a diagonal estimate of the inverse hessian estimate.
*/

class LBFGS : public NonlinearMinimizer
{
 public:
  LBFGS();
  LBFGS(CostFunction& f);

  bool Minimize(Vector<double>& x);

/** Step accuracy/speed tradeoff.
* Effectively the number of correction vectors to the diagonal approximation
* of the inverse hessian estimate that are kept.
*
* Large values of M will result in excessive computing time.
* 3<= memory <=7 is recommended.
* Memory requirements will be roughly Const+sizeof(element)*dim(X)*memory.
* Default is 5.
*/
  int memory;

/** Accuracy of line search.
* If function evaluations are cheap wrt the actual minimization steps,
* change to 0.1, from default of 0.9;
*/
  double line_search_accuracy;

/** Default step length in line search.
* If, on tracing, the STP is always 1, then you could try setting this to a
* higher value to see how far along the gradient the minimum typically is.
* Then set this to a number just below that to get maximally far with the
* single evaluation.
*/
  double default_step_length;

 private:
  void _InitParameters();
  CostFunction* f_;
  //  LBFGS() {} // default constructor makes no sense
  // does too.  Can set values for parameters.
};

}; // End namespace VNL

#endif // LBFGS_h_

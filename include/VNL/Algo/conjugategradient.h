// This is vxl/VNL/algo/conjugate_gradient.h
#ifndef vnl_conjugate_gradient_h_
#define vnl_conjugate_gradient_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief  real function minimization

*  \author Geoffrey Cross, Oxford RRG
*  \date   15 Feb 99
*
   \verbatim
   Modifications
   990215 Geoff Initial version.
   000628 David Capel - Major rewrite. Now derived from vnl_nonlinear_minimizer and operates on a vnl_cost_function.
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*
*/

#include <iosfwd>
#include <VNL/vector.h>
#include <VNL/matrix.h>
#include <VNL/nonlinearminimizer.h>

namespace VNL {

  class CostFunction;

/** real function minimization.
*/

class ConjugateGradient : public NonlinearMinimizer
{
 public:
  // Constructors/Destructors--------------------------------------------------

/** Initialize with the function object that is to be minimized.
*/
  ConjugateGradient(CostFunction& f) { Init( f); }

/** Initialize as above, and then run minimization.
*/
  ConjugateGradient(CostFunction& f, Vector<double>& x) {
    Init(f);
    Minimize(x);
  }

/** Initialize all variables.
*/
  void Init(CostFunction &f);

/** Destructor.
*/
  ~ConjugateGradient();

  // Operations----------------------------------------------------------------

  void DiagnoseOutcome(std::ostream&) const;
  void DiagnoseOutcome(/*std::ostream& = std::cout*/) const;

  // Computations--------------------------------------------------------------

/** Minimize the function supplied in the constructor until convergence or failure.
* On return, x is such that f(x) is the lowest value achieved.
* Returns true for convergence, false for failure.
*/
  bool Minimize(Vector<double>& x);

 protected:
  // Data Members--------------------------------------------------------------

  CostFunction *f_;
  double final_step_size_;

  // Helpers-------------------------------------------------------------------

  friend class ConjugateGradientActivate;

#ifdef VCL_SUNPRO_CC
 public:
#endif
  static double ValueComputer_( double *x);
  static int GradientComputer_( double *g, double *x);
  static int ValueAndGradientComputer_( double *v, double *g, double *x);
  static int Preconditioner_( double *out, double *in);

#if 0
 protected:
  void _ApproximateGradient( const Vector<double> &x,
                             Vector<double> &g, const double step);
  void _ApproximateHessian( const Vector<double> &x,
                            Matrix<double> &h, const double step);
#endif
};

}; // End namespace VNL

#endif // ConjugateGradient_h_

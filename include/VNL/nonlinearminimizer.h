// This is vxl/VNL/nonlinear_minimizer.h
#ifndef vnl_nonlinear_minimizer_h_
#define vnl_nonlinear_minimizer_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Base class for nonlinear optimization

*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   22 Aug 99
*
   \verbatim
   Modifications
    22/03/2001  dac - added binary io and tidied documentation
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*/

#include <string>
#include <VNL/matrix.h>

namespace VNL {


/** vnl_nonlinear_minimizer is a base class for nonlinear optimization.
* It defines a few common abilities such as get_num_evaluations.
* Known derived classes are:
* -  vnl_levenberg_marquardt
* -  vnl_lbfgs
* -  vnl_conjugate_gradient
* -  vnl_brent
* -  vnl_powell
*/
class NonlinearMinimizer
{
 public:
  NonlinearMinimizer();

  virtual ~NonlinearMinimizer();


/** Set the convergence tolerance on F (sum of squared residuals).
* When the differences in successive RMS errors is less than this, the
* routine terminates.  So this is effectively the desired precision of your
* minimization.  Setting it too low wastes time, too high might cause early
* convergence.  The default of 1e-9 is on the safe side, but if speed is an
* issue, you can try raising it.
*/
  void SetFTolerance(double v) { ftol = v; }
  double GetFTolerance() const { return ftol; }

/** Set the convergence tolerance on X.
*  When the length of the steps taken in X are about this long, the routine
* terminates.  The default is 1e-8, which should work for many problems,
* but if you can get away with 1e-4, say, minimizations will be much quicker.
*/
  void SetXTolerance(double v) {
    xtol = v;
    epsfcn = xtol * 0.001;
  }
  double GetXTolerance() const { return xtol; }

/** Set the convergence tolerance on Grad(F)' * F.
*/
  void SetGTolerance(double v) { gtol = v; }
  double GetGTolerance() const { return gtol; }

/** Set the termination maximum number of iterations.
*/
  void SetMaxFunctionEvals(int v) { maxfev = v; }
  int GetMaxFunctionEvals() const { return maxfev; }

/** Set the step length for FD Jacobian.
* Be aware that SetXTolerance will reset this to xtol * 0.001.
* The default is 1e-11.
*/
  void SetEpsilonFunction(double v) { epsfcn = v; }
  double GetEpsilonFunction() const { return epsfcn; }

/** Turn on per-iteration printouts.
*/
  void SetTrace(bool on) { trace = on; }
  bool GetTrace() const { return trace; }

/** Set verbose flag.
*/
  void SetVerbose(bool verb) { verbose_ = verb; }
  bool GetVerbose() const { return verbose_; }

/** Set check_derivatives flag.\  Negative values may mean fewer checks.
*/
  void SetCheckDerivatives(int cd) { check_derivatives_ = cd; }
  int SetCheckDerivatives() const { return check_derivatives_; }

/** Return the error of the function when it was evaluated at the start point of the last minimization.
* For minimizers driven by a vnl_least_squares_function (Levenberg-Marquardt)
* this is usually the RMS error.
* For those driven by a vnl_cost_function (CG, LBFGS, Amoeba) it is simply the
* value of the vnl_cost_function at the start (usually the sum of squared residuals).
*/
  double GetStartError() const { return start_error_; }

/**Return the best error that was achieved by the last minimization, corresponding to the returned x.
*/
  double GetEndError() const { return end_error_; }

/**Return the total number of times the function was evaluated by the last minimization.
*/
  int GetNumEvaluations() const { return num_evaluations_; }

/**Return the number of {\em iterations} in the last minimization.
* Each iteration may have comprised several function evaluations.
*/
  int GetNumIterations() const { return num_iterations_; }

/**Return the covariance of the estimate at the end.
*/
  virtual Matrix<double> const& GetCovariance();

/** Return the name of the class.
*  Used by polymorphic IO
*/
  virtual std::string IsA() const;

/** Return true if the name of the class matches the argument.
*  Used by polymorphic IO
*/
  virtual bool IsClass(std::string const& s) const;

/**Some generic return codes that apply to all minimizers.
*/
  enum ReturnCodes {
    ERROR_FAILURE               =-1,
    ERROR_DODGY_INPUT           = 0,
    CONVERGED_FTOL              = 1,
    CONVERGED_XTOL              = 2,
    CONVERGED_XFTOL             = 3,
    CONVERGED_GTOL              = 4,
    FAILED_TOO_MANY_ITERATIONS  = 5,
    FAILED_FTOL_TOO_SMALL       = 6,
    FAILED_XTOL_TOO_SMALL       = 7,
    FAILED_GTOL_TOO_SMALL       = 8
  };

/**Return the failure code of the last minimization.
*/
  ReturnCodes GetFailureCode() const { return failure_code_; }

 protected:
  // Data Members--------------------------------------------------------------
  // Input variables
  double xtol;    //: Termination tolerance on X (solution vector)
  int    maxfev;  //: Termination maximum number of iterations
  double ftol;    //: Termination tolerance on F (sum of squared residuals)
  double gtol;    //: Termination tolerance on Grad(F)' * F = 0
  double epsfcn;  //: Step length for FD Jacobian

  // Output variables
  unsigned num_iterations_;
  int    num_evaluations_;
  double start_error_;
  double end_error_;

  bool trace;

  // Verbose flag.
  bool verbose_;
  int check_derivatives_;
  ReturnCodes failure_code_;

  void _Reset();
  void _ReportEval(double f);
  void _ReportIter();
};


}; // End namespace VNL

#endif // vnl_nonlinear_minimizer_h_

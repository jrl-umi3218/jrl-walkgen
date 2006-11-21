// This is vxl/VNL/cost_function.h
#ifndef vnl_cost_function_h_
#define vnl_cost_function_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Vector->Real function

*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   23 Oct 97
*
   \verbatim
   Modifications
    971023 AWF Initial version.
    LSB (Manchester) 26/3/01 Tidied documentation
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*
*/

#include <VNL/unaryfunction.h>
#include <VNL/vector.h>

namespace VNL {


/**   An object that represents a function from R^n -> R.
*    It is commonly used to express the
*    interface of a minimizer.
*/
class CostFunction : public UnaryFunction<double, Vector<double> >
{
 public:
  CostFunction(int number_of_unknowns)
    : dim(number_of_unknowns) {}

  virtual ~CostFunction() {}

/**  The main function.\  Given the parameter vector x, compute the value of f(x).
*/
  virtual double F(Vector<double> const& x);

/**  Calculate the gradient of f at parameter vector x.
*/
  virtual void GradF(Vector<double> const& x, Vector<double>& gradient);

/**  Compute one or both of f and g.
*  Normally implemented in terms of the above two, but may be faster if specialized. f != 0 => compute f
*/
  virtual void Compute(Vector<double> const& x, double *f, Vector<double>* g);

/**  Return the number of unknowns.
*/
  int GetNumberOfUnknowns() const { return dim; }

/**  Compute finite-difference gradient.
*/
  void FDGradF(Vector<double> const& x, Vector<double>& gradient, double stepsize = 1e-5);

/**  Called when error is printed for user.
*/
  virtual double ReportedError(double f_value) { return f_value; }

/**  Conveniences for printing grad, fdgrad.
*/
  Vector<double> GradF(Vector<double> const& x);
  Vector<double> FDGradF(Vector<double> const& x);

 public:
  int dim;
};



}; // End namespace VNL

#endif // vnl_cost_function_h_

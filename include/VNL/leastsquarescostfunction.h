// This is vxl/VNL/least_squares_cost_function.h
#ifndef vnl_least_squares_cost_function_h_
#define vnl_least_squares_cost_function_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/** \file
*  \brief vnl_least_squares_function -> vnl_cost_function adaptor

*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   20 Aug 99
*
   \verbatim
   Modifications:
       990820 AWF Initial version.
       LSB (Manchester) 23/3/01 Tidied documentation
   \endverbatim
*
*/

#include <VNL/costfunction.h>
#include <VNL/leastsquaresfunction.h>

namespace VNL {


/** An adaptor that converts a vnl_least_squares_function to a vnl_cost_function.
*/
class LeastSquaresCostFunction : public CostFunction
{
 public:
  LeastSquaresCostFunction(LeastSquaresFunction* f);

  double F(const Vector<double>& x);

  virtual void GradF(const Vector<double>& x, Vector<double>& gradient);

 protected:
  Vector<double> storage_;
  Matrix<double> jacobian_;
  LeastSquaresFunction* f_;
};


}; // End namespace VNL

#endif // vnl_least_squares_cost_function_h_

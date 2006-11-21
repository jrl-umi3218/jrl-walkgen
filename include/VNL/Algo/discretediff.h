#ifndef vnl_discrete_diff_h_
#define vnl_discrete_diff_h_

/**
*  \file
*  \brief Functions to compute jacobians of vnl_least_squares_functions

*
* Functions to compute jacobians of vnl_least_squares_functions
* by discrete differences.  They return false on failure and
* true on success.
*
   \verbatim
   name size    description
  
   lsf  ---     the function.
   h    1 or n  step size (scalar or a vector).
   x    n       point at which to evaluate the derivative of the function.
   y    m       value of the function at x.
   J    mxn     jacobian of the function at x.
   \endverbatim
*
*  \author fsm@robots.ox.ac.uk
*
   \verbatim
   Modifications
    dac (Manchester) 28/03/2001: tidied up documentation
    Peter Vanroose   27/05/2001: Corrected documentation
   \endverbatim
*/

#include <VNL/vector.h>
#include <VNL/matrix.h>

// Forward declare the least squares stuff
namespace VNL {class LeastSquaresFunction;};

/** forward differences.
*/
bool vnl_discrete_diff_fwd(VNL::LeastSquaresFunction *lsf,
                           double h,
                           VNL::Vector<double> const &x,
                           VNL::Matrix<double>       &J);

/** forward differences.
*/
bool vnl_discrete_diff_fwd(VNL::LeastSquaresFunction *lsf,
                           VNL::Vector<double> const &h,
                           VNL::Vector<double> const &x,
                           VNL::Matrix<double>       &J);

/** forward differences.
*/
bool vnl_discrete_diff_fwd(VNL::LeastSquaresFunction *lsf,
                           VNL::Vector<double> const &h,
                           VNL::Vector<double> const &x,
                           VNL::Vector<double> const &y,
                           VNL::Matrix<double>       &J);

/** symmetric differences.
*/
bool vnl_discrete_diff_sym(VNL::LeastSquaresFunction *lsf,
                           double h,
                           VNL::Vector<double> const &x,
                           VNL::Matrix<double>       &J);

/** symmetric differences.
*/
bool vnl_discrete_diff_sym(VNL::LeastSquaresFunction *lsf,
                           VNL::Vector<double> const &h,
                           VNL::Vector<double> const &x,
                           VNL::Matrix<double>       &J);

void vnl_discrete_diff_test_lsf(VNL::LeastSquaresFunction *lsf, VNL::Vector<double> const &x);

#endif // vnl_discrete_diff_h_

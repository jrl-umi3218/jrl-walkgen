// This is vxl/VNL/matrix_exp.h
#ifndef vnl_matrix_exp_h_
#define vnl_matrix_exp_h_
/**
* \file
* \brief Compute the exponential of a square matrix

*
* Compute the exponential of a square matrix, by summing its
* exponential series \f$\exp(X) = \displaystyle\sum_{n \ge 0} X^n/n!\f$
* till a convergence requirement is met.
*
* Many improvements are possible.
*
*  \author fsm@robots.ox.ac.uk
*/

#include <VNL/matrix.h>

namespace VNL {

/** Compute the exponential of a square matrix - fiddly form.
*/
template <class T>
bool MatrixExp(Matrix<T> const &X, Matrix<T> &expX, double max_err);

/** Compute the exponential of a square matrix - easy form.
*/
template <class T>
Matrix<T> MatrixExp(Matrix<T> const &X);

}; // End namespace VNL

#endif // vnl_matrix_exp_h_

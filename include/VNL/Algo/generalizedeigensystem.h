// This is vxl/VNL/algo/generalized_eigensystem.h
#ifndef vnl_generalized_eigensystem_h_
#define vnl_generalized_eigensystem_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief  Solves the generalized eigenproblem Ax=La

* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   29 Aug 96
*
   \verbatim
   Modifications
    dac (Manchester) 28/03/2001: tidied up documentation
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*/

#include <VNL/diagmatrix.h>

namespace VNL {

/** Solves the generalized eigenproblem Ax=Bx.
*  Solves the generalized eigenproblem of \f$A x = \lambda B x\f$,
*  with \f$A\f$ symmetric and \f$B\f$ positive definite. \n
*  See Golub and van Loan, Section 8.7.
*/

class GeneralizedEigensystem
{
 public:
// Public data members because they're unique.
  int n;

/** Solves the generalized eigenproblem Ax=Bx.
*  Solve real generalized eigensystem \f$A x = \lambda B x\f$ for
*  \f$\lambda\f$ and \f$x\f$, where \f$A\f$ symmetric, \f$B\f$ positive definite.
*  Initializes storage for the matrix \f$V = [ x_0 x_1 .. x_n ]\f$ and
*  the VNL::DiagMatrix \f$D = [ \lambda_0 \lambda_1 ... \lambda_n ]\f$.
*  The eigenvalues are sorted into increasing order (of value, not
*  absolute value).
*
*  Uses vnl_cholesky decomposition \f$C^\top C = B\f$, to convert to
*  \f$C^{-\top} A C^{-1} x = \lambda x\f$ and then uses the
*  symmetric eigensystem code.   It will print a verbose warning
*  if \f$B\f$ is not positive definite.
*/

  GeneralizedEigensystem(const VNL::Matrix<double>& A,
			 const VNL::Matrix<double>& B);

/** Public eigenvectors.
*  After construction, this contains the matrix of eigenvectors.
*/
  VNL::Matrix<double> V;

/** Public eigenvalues.
*  After construction, this contains the diagonal matrix of eigenvalues, stored as a vector.
*/
  VNL::DiagMatrix<double> D;
};

}; // End namespace VNL

#endif // GeneralizedEigensystem_h_

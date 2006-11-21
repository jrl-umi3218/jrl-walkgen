// This is vxl/VNL/algo/symmetric_eigensystem.h
#ifndef vnl_symmetric_eigensystem_h_
#define vnl_symmetric_eigensystem_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Find eigenvalues of a symmetric matrix

*
*    vnl_symmetric_eigensystem_compute()
*    solves the eigenproblem \f$A x = \lambda x\f$, with \f$A\f$ symmetric.
*    The resulting eigenvectors and values are sorted in increasing order
*    so <CODE> V.column(0) </CODE> is the eigenvector corresponding to the smallest
*    the smallest eigenvalue.
*
*    As a matrix decomposition, this is \f$A = V D V^t\f$
*
*    Uses the EISPACK routine RS, which in turn calls TRED2 to reduce A
*    to tridiagonal form, followed by TQL2, to find the eigensystem.
*    This is summarized in Golub and van Loan, \S8.2.  The following are
*    the original subroutine headers:
*
* \remark TRED2 is a translation of the Algol procedure tred2,
*     Num. Math. 11, 181-195(1968) by Martin, Reinsch, and Wilkinson.
*     Handbook for Auto. Comp., Vol.ii-Linear Algebra, 212-226(1971).
*
* \remark This subroutine reduces a real symmetric matrix to a
*     symmetric tridiagonal matrix using and accumulating
*     orthogonal similarity transformations.
*
* \remark TQL2 is a translation of the Algol procedure tql2,
*     Num. Math. 11, 293-306(1968) by Bowdler, Martin, Reinsch, and Wilkinson.
*     Handbook for Auto. Comp., Vol.ii-Linear Algebra, 227-240(1971).
*
* \remark This subroutine finds the eigenvalues and eigenvectors
*     of a symmetric tridiagonal matrix by the QL method.
*     the eigenvectors of a full symmetric matrix can also
*     be found if  tred2  has been used to reduce this
*     full matrix to tridiagonal form.
*
* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   29 Aug 96
*
   \verbatim
   Modifications
    fsm@robots, 5 March 2000: templated
    dac (Manchester) 28/03/2001: tidied up documentation
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*/

#include <VNL/matrix.h>
#include <VNL/diagmatrix.h>


namespace VNL {


/** Find eigenvalues of a symmetric matrix.
*/

bool SymmetricEigensystemCompute(Matrix<float> const & A,
				 Matrix<float> & V,
				 Vector<float> & D);

/** Find eigenvalues of a symmetric matrix.
*/

bool SymmetricEigensystemCompute(Matrix<double> const & A,
				 Matrix<double> & V,
				 Vector<double> & D);

/** Computes and stores the eigensystem decomposition of a symmetric matrix.
*/

template <class T>
class SymmetricEigensystem
{
 public:
/** Solve real symmetric eigensystem $A x = \lambda x$.
*/
  SymmetricEigensystem(Matrix<T> const & M);

 protected:
  // need this here to get inits in correct order, but still keep gentex
  // in the right order.
  int n_;

 public:
/** Public eigenvectors.
*  After construction, the columns of V are the eigenvectors, sorted by
* increasing eigenvalue, from most negative to most positive.
*/
  Matrix<T> V;

/** Public eigenvalues.
*  After construction,  D contains the eigenvalues, sorted as described above.
*  Note that D is a DiagMatrix, and is therefore stored as a std::vector while behaving as a matrix.
*/
  DiagMatrix<T> D;

/** Recover specified eigenvector after computation.
*/
  Vector<T> GetEigenvector(int i) const;

/** Recover specified eigenvalue after computation.
*/
  T             GetEigenvalue(int i) const;

/** Convenience method to get least-squares nullvector.
* It is deliberate that the signature is the same as on vnl_svd<T>.
*/
  Vector<T> Nullvector() const { return GetEigenvector(0); }

/** Return the matrix $V  D  V^\top$.
*  This can be useful if you've modified \f$D\f$.  So an inverse is obtained using
* \code
*   vnl_symmetric_eigensystem} eig(A);
*   eig.D.invert_in_place}();
*   VNL::Matrix<double> Ainverse = eig.recompose();
* \endcode
*/

  Matrix<T> Recompose() const { return V * D * V.Transpose(); }

/** return the pseudoinverse.
*/
  Matrix<T> PsuedoInverse() const;

/** return the square root, if positive semi-definite.
*/
  Matrix<T> SquareRoot() const;

/** return the inverse of the square root, if positive semi-definite.
*/
  Matrix<T> InverseSquareRoot() const;

/** Solve LS problem M x = b.
*/
  Vector<T> Solve(Vector<T> const & b);

/** Solve LS problem M x = b.
*/
  void Solve(Vector<T> const & b, Vector<T> * x);
};


}; // End namespace VNL

#endif // vnl_symmetric_eigensystem_h_

// This is vxl/VNL/linear_system.h
#ifndef vnl_linear_system_h_
#define vnl_linear_system_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Abstraction for a linear system of equations.

*  \author David Capel, capes@robots
*  \date   July 2000
*
   \verbatim
   Modifications:
   LSB (Manchester) 23/1/01 Documentation tidied
   \endverbatim
*/

#include <VNL/vector.h>

namespace VNL {


/** Abstraction for a linear system of equations.
*    vnl_linear_system provides an abstraction for a linear system
*    of equations, Ax = b, to be solved by one of the iterative linear
*    solvers. Access to the systems is via the pure virtual methods
*    multiply() and transpose_multiply(). This procedural access scheme
*    makes it possible to solve very large, sparse systems which it would
*    be inefficient to store in matrix form.
*
*    To solve the system, use an algorithm like vnl_lsqr.
*/
class LinearSystem
{
 public:

  LinearSystem(int number_of_unknowns, int number_of_residuals) :
    p_(number_of_unknowns), n_(number_of_residuals) {}

  virtual ~LinearSystem();

/** Compute A*x,  putting result in y.
*/
  virtual void Multiply(Vector<double> const& x, 
			Vector<double>& y) const = 0;

/** Compute A_transpose * y, putting result in x.
*/
  virtual void TransposeMultiply(Vector<double> const& y, 
				 Vector<double>& x) const = 0;

  //; Put the right-hand side of the system Ax = b into b
  virtual void GetRHS(Vector<double>& b) const = 0;

  //; (Optional) Apply a suitable preconditioner to x.
  // A preconditioner is an approximation of the inverse of A.
  // Common choices are Jacobi (1/diag(A'A)), Gauss-Seidel,
  // and incomplete LU or Cholesky decompositions.
  // The default implementation applies the identity.
  virtual void ApplyPreconditioner(Vector<double> const& x, 
				   Vector<double>& px) const;

/** Return the number of unknowns.
*/
  int GetNumberOfUnknowns() const { return p_; }

/** Return the number of residuals.
*/
  int GetNumberOfResiduals() const { return n_; }

/** Compute rms error for parameter vector x.
*/
  double GetRMSError(Vector<double> const& x) const;

/** Compute relative residual (|Ax - b| / |b| )for parameter vector x.
*/
  double GetRelativeResidual(Vector<double> const& x) const;

 protected:
  int p_;
  int n_;
};


}; // End namespace VNL

#endif // vnl_linear_system_h_

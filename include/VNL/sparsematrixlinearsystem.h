// This is vxl/VNL/sparse_matrix_linear_system.h
#ifndef vnl_sparse_matrix_linear_system_h_
#define vnl_sparse_matrix_linear_system_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief vnl_sparse_matrix -> vnl_linear_system adaptor

*
*  An adaptor that converts a vnl_sparse_matrix<T> to a vnl_linear_system
*
*  \author David Capel, capes@robots
*  \date   July 2000
*
   \verbatim
    Modifications
    LSB (Manchester) 19/3/01 Documentation tidied
   \endverbatim
*
*/

#include <VNL/linearsystem.h>
#include <VNL/sparsematrix.h>

namespace VNL {

/** SparseMatrix -> LinearSystem adaptor.
*  An adaptor that converts a SparseMatrix<T> to a LinearSystem
*/
template <class T>
class SparseMatrixLinearSystem : public LinearSystem
{
 public:
/**:Constructor from VNL::SparseMatrix<double> for system Ax = b.
* Keeps a reference to the original sparse matrix A and vector b so DO NOT DELETE THEM!!
*/
  SparseMatrixLinearSystem(VNL::SparseMatrix<T> const& A, VNL::Vector<T> const& b) :
    LinearSystem(A.columns(), A.rows()), A_(A), b_(b) {}

/**  Implementations of the vnl_linear_system virtuals.
*/
  void Multiply(VNL::Vector<double> const& x, VNL::Vector<double> & b) const;
/**  Implementations of the vnl_linear_system virtuals.
*/
  void TransposeMultiply(VNL::Vector<double> const& b, VNL::Vector<double> & x) const;
/**  Implementations of the vnl_linear_system virtuals.
*/
  void GetRHS(VNL::Vector<double>& b) const;
/**  Implementations of the vnl_linear_system virtuals.
*/
  void ApplyPreconditioner(VNL::Vector<double> const& x, VNL::Vector<double> & px) const;

 protected:
  VNL::SparseMatrix<T> const& A_;
  VNL::Vector<T> const& b_;
  VNL::Vector<double> jacobi_precond_;
};

}; // End namespace VNL

#endif // vnl_sparse_matrix_linear_system_h_

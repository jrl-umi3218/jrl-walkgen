// This is vxl/VNL/algo/sparse_symmetric_eigensystem.h
#ifndef vnl_sparse_symmetric_eigensystem_h_
#define vnl_sparse_symmetric_eigensystem_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Find the eigenvalues of a sparse symmetric matrix

* \author Rupert W. Curwen, GE CR&D
* \date   20 Oct 98
*
   \verbatim
   Modifications
    dac (Manchester) 28/03/2001: tidied up documentation
   \endverbatim
*/

#include <VNL/sparsematrix.h>
#include <vector>


namespace VNL {

/** Find the eigenvalues of a sparse symmetric matrix.
*  Solve the eigenproblem \f$A x = \lambda x\f$, with \f$A\f$ symmetric and
*  sparse.  The block Lanczos algorithm is used to allow the
*  recovery of a number of eigenvale/eigenvector pairs from either
*  end of the spectrum, to a required accuracy.
*
*  Uses the dnlaso routine from the LASO package of netlib.
*/

class SparseSymmetricEigensystem
{
 public:
  SparseSymmetricEigensystem();

  // Find n eigenvalue/eigenvectors.  If smallest is true, will
  // calculate the n smallest eigenpairs, else the n largest.
  int CalculateNPairs(VNL::SparseMatrix<double>& M, int n,
                      bool smallest = true, int nfigures = 10);

  // Recover specified eigenvector after computation.  The argument
  // must be less than the requested number of eigenvectors.
  VNL::Vector<double> GetEigenvector(int i) const;
  double GetEigenvalue(int i) const;

  // Used as a callback in solving.
  int CalculateProduct(int n, int m, const double* p, double* q);
  int SaveVectors(int n, int m, const double* q, int base);
  int RestoreVectors(int n, int m, double* q, int base);

 protected:
  int nvalues;  // this is the size of the next two arrays.
  VNL::Vector<double> * vectors; // eigenvectors
  double * values;              // eigenvalues

  VNL::SparseMatrix<double> * mat;

  std::vector<double*> temp_store;
};

}; // End namespace VNL

#endif // SparseSymmetricEigensystem_h_

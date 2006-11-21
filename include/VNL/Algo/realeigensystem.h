// This is vxl/VNL/algo/real_eigensystem.h
#ifndef vnl_real_eigensystem_h_
#define vnl_real_eigensystem_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Extract eigensystem of unsymmetric matrix M, using EISPACK

* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   23 Jan 97
*
   \verbatim
   Modifications
    dac (Manchester) 28/03/2001: tidied up documentation
   \endverbatim
*
*/

#include <complex>
#include <VNL/matrix.h>
#include <VNL/diagmatrix.h>

namespace VNL {

/** Extract eigensystem of unsymmetric matrix M, using the EISPACK routine.
*  vnl_eigensystem is a full-bore real eigensystem.  If your matrix
*  is symmetric, it is *much* better to use vnl_symmetric_eigensystem.
*/

class RealEigensystem
{
 public:
  RealEigensystem(Matrix<double> const& M);

 public:
  Matrix<double> Vreal;

/** Output matrix of eigenvectors, which will in general be complex.
*/
  Matrix<std::complex<double> > V;

/** Output diagonal matrix of eigenvalues.
*/
  DiagMatrix<std::complex<double> > D;
};

}; // End namespace VNL

#endif // vnl_real_eigensystem_h_

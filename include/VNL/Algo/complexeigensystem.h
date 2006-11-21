#ifndef vnl_complex_eigensystem_h_
#define vnl_complex_eigensystem_h_

/**
*  \file
*  \brief  Calculates eigenvalues and eigenvectors of a square complex matrix

*  \author fsm@robots.ox.ac.uk
*
    \verbatim
    Modifications
    dac (Manchester) 26/03/2001: tidied up documentation
    \endverbatim
*/

#include <complex>
#include <VNL/vector.h>
#include <VNL/matrix.h>

namespace VNL {

/** Calculates eigenvalues and eigenvectors of a square complex matrix.
*
*  Class to compute and hold the eigenvalues and (optionally) eigenvectors
*  of a square complex matrix, using the LAPACK routine zgeev.
*
*  Default behaviour is to compute the eigenvalues and the right
*  eigenvectors.
*
*  The input NxN matrix A is passed into the constructor. The flags
*  right,left request the calculation of right and left eigenvectors
*  respectively. The compute eigenvalues are stored in the member 'W'.
*
*  Computed right eigenvectors are stored in the **ROWS** of the
*  member 'R' and computed left eigenvectors are stored in the **ROWS**
*  of the member 'L'. When eigenvectors are not requested, the corre-
*  sponding matrices L and R will be empty.
*
*  The ith right eigenvector v satisfies A*v = W[i]*v   \n
*  The ith left  eigenvector u satisfies u*A = W[i]*u (no conjugation)
*/

class ComplexEigensystem {
public:

  ComplexEigensystem(VNL::Matrix<double> const& A_real,
                          VNL::Matrix<double> const& A_imag,
                          bool right=true, bool left=false);

  ComplexEigensystem(VNL::Matrix<std::complex<double> > const& A,
                          bool right=true, bool left=false);

  // please do not add underscores to my members.
  int const N;
  VNL::Matrix<std::complex<double> > L; // left evecs
  VNL::Matrix<std::complex<double> > R; // right evecs
  VNL::Vector<std::complex<double> > W; // evals

  // convenience methods
  std::complex<double> EigenValue(unsigned i) const { return W[i]; }
  VNL::Vector<std::complex<double> > LeftEigenVector(unsigned i)
      const { return L.GetRow(i); }
  VNL::Vector<std::complex<double> > RightEigenVector(unsigned i)
      const { return R.GetRow(i); }

private:
  void _Compute(VNL::Matrix<std::complex<double> > const&,bool,bool);
};


}; // End namespace VNL

#endif // ComplexEigensystem_h_

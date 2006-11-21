// This is vxl/VNL/algo/matrix_inverse.h
#ifndef vnl_matrix_inverse_h_
#define vnl_matrix_inverse_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Calculates inverse of a matrix (wrapper around vnl_svd<double>)

* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   22 Nov 96
*
   \verbatim
   Modifications
    dac (Manchester) 28/03/2001: tidied up documentation
   \endverbatim
*/

#include <VNL/Algo/svd.h>


namespace VNL {

/** Calculates inverse of a matrix (wrapper around vnl_svd<double>).
*  vnl_matrix_inverse is a wrapper around vnl_svd<double> that allows
*  you to write
*
*  x = vnl_matrix_inverse(A) * b;
*
*  This is exactly equivalent to x = vnl_svd<double>(A).solve(b);
*  but is arguably clearer, and also allows for the vnl_matrix_inverse
*  class to be changed  to use vnl_qr, say.
*/

  template <class T = double>
  struct MatrixInverse : public VNL::SVD<T>
  {
    MatrixInverse(VNL::Matrix<T> const & M): VNL::SVD<T>(M) { }
    ~MatrixInverse() {};
    
	operator VNL::Matrix<T> () const { return VNL::SVD<T>::Inverse(); }
  };
  
  
}; // End namespace VNL

template <class T>
inline
VNL::Vector<T> operator*(VNL::MatrixInverse<T> const & i,
			 VNL::Vector<T> const & B)
{
  return i.Solve(B);
}

template <class T>
inline
VNL::Matrix<T> operator*(VNL::MatrixInverse<T> const & i,
			 VNL::Matrix<T> const & B)
{
  return i.Solve(B);
}

#endif // vnl_matrix_inverse_h_

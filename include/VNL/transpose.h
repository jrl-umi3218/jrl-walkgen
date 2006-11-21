// This is vxl/VNL/transpose.h
#ifndef vnl_transpose_h_
#define vnl_transpose_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Efficient matrix transpose

*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   23 Dec 96
*
   \verbatim
    Modifications
    LSB (Manchester) 19/3/01 Tidied documentation
   \endverbatim
*/

#include <iostream>
#include <VNL/fastops.h>

namespace VNL {

/** Efficient matrix transpose.
*    vnl_transpose is an efficient way to write C = vnl_transpose(A) * B.
*    The vnl_transpose class holds a reference to the original matrix
*    and when involved in an operation for which it has been specialized,
*    performs the operation without copying.
*
*    If the operation has not been specialized, the vnl_transpose performs
*    a copying conversion to a matrix, printing a message to stdout.
*    At that stage, the user may choose to implement the particular operation
*    or use vnl_transpose::asMatrix() to clear the warning.
*
*    NOTE: This is a reference class, so should be shorter-lived than the
*    matrix to which it refers.
*/

class Transpose
{
  const Matrix<double>& M_;
 public:

/** Make a vnl_transpose object referring to matrix M.
*/
  Transpose(const Matrix<double>& M): M_(M) {}

/** Noisily convert a vnl_transpose to a matrix.
*/
  operator Matrix<double> () const {
    std::cerr << "transpose being converted to matrix -- help! I don't wanna go!\n";
    return M_.Transpose();
  }

/** Quietly convert a vnl_transpose to a matrix.
*/
  Matrix<double> AsMatrix () const {
    return M_.Transpose();
  }

/** Return M' * O.
*/
  Matrix<double> operator* (const Matrix<double>& O) {
    Matrix<double> ret(M_.Columns(), O.Columns());
    vnl_fastops::AtB(M_, O, &ret);
    return ret;
  }

/** Return M' * O.
*/
  Vector<double> operator* (const Vector<double>& O) {
    Vector<double> ret(M_.Columns());
    vnl_fastops::AtB(M_, O, &ret);
    return ret;
  }

/** Return A * B'.
*/
  friend Matrix<double> operator* (const Matrix<double>& A, 
					const Transpose& B) {
    VNL::Matrix<double> ret(A.Rows(), B.M_.Rows());
    vnl_fastops::ABt(A, B.M_, &ret);
    return ret;
  }
};

}; // End namespace VNL

#endif // vnl_transpose_h_

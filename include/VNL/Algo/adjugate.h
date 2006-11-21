// This is vxl/VNL/algo/adjugate.h
#ifndef vnl_adjugate_h_
#define vnl_adjugate_h_
/**
* \file
* \author fsm@robots.ox.ac.uk
*/

namespace VNL {
  // Forward declare the matrix class rather than including it
  template <class T> class Matrix;

  // By definition, the product of A and its adjugate
  // is det(A) [times an identity matrix].

  template <class T>
  void Adjugate(Matrix<T> const &A, Matrix<T> *out);
  
  template <class T>
  Matrix<T> Adjugate(Matrix<T> const &A);

}; // End namespace VNL

#endif // vnl_adjugate_h_

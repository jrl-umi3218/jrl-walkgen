/** 
 *  @class   VNL::CrossProductMatrix
 *  @ingroup MathsStuff
 *  @author  Andrew W. Fitzgibbon, Oxford RRG
 *  @date    19 Sep 96
 *  @brief   3x3 cross-product matrix of vector
 *
 * \verbatim
 * Modifications:
 * 4/4/01 LSB (Manchester) Tidied Documentation
 * 15/2/03 BJT (Ox) added templating
 * \endverbatim
 *
 *  Calculates the 3x3 skew symmetric cross product matrix from a vector.
 *
 * CrossProductMatrix(e) is the matrix [e]_ x:
 * \verbatim
 *     0    -e_3   e_2
 *     e_3   0    -e_1
 *    -e_2   e_1   0
 * \endverbatim
 */


#ifndef _cross_product_matrix_h_
#define _cross_product_matrix_h_

// Local includes
#include "vector.h"
#include "matrixfixed.h"




namespace VNL {

template <class T>
class CrossProductMatrix : public MatrixFixed<3,3,T>
{
 public:
  typedef MatrixFixed<3,3,T> base;

  CrossProductMatrix(const Vector<T>& v) { Set(v.DataBlock()); }
  CrossProductMatrix(const T* v) { Set(v); }
  CrossProductMatrix(const CrossProductMatrix& that): base(that) {}
 ~CrossProductMatrix() {}

  CrossProductMatrix& operator=(const CrossProductMatrix& that) {
    base::operator= (that);
    return *this;
  }

  void Set(const T* v) // override method in VNL::Vector
{
  T e1 = v[0];
  T e2 = v[1];
  T e3 = v[2];

  MatrixFixed<3,3,T> & E = *this; //win32 doesn't like E(*this);

  E(0,0) =   0; E(0,1) = -e3; E(0,2) =  e2;
  E(1,0) =  e3; E(1,1) =   0; E(1,2) = -e1;
  E(2,0) = -e2; E(2,1) =  e1; E(2,2) =   0;
}
};


}; // End namespace VNL











#endif // _cross_product_matrix_h_

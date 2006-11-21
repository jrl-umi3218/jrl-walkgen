// This is vxl/VNL/algo/scatter_3x3.h
#ifndef vnl_scatter_3x3_h_
#define vnl_scatter_3x3_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief

* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   02 Oct 96
*
   \verbatim
   Modifications:
    18 Feb 2000. fsm: templated.
    4/4/01 LSB (Manchester) documentation tidied
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*/

#include <VNL/matrixfixed.h>
#include <VNL/vectorfixed.h>

namespace VNL {

template <class T>
class Scatter3x3 : public MatrixFixed<3,3,T>
{
 public:
  typedef MatrixFixed<3,3,T> base;
  typedef VectorFixed<3,T> vect;

/** Constructor.\  Fills with zeros.
*/
  Scatter3x3();

/** Add v*v' to scatter.
*/
  void add_outer_product(const VectorFixed<3,T> & v);

/** Add v*u' to scatter.
*/
  void add_outer_product(const VectorFixed<3,T> & u,
        const VectorFixed<3,T> & v);

/** Subtract v*v' from scatter.
*/
  void sub_outer_product(const VectorFixed<3,T> & v);

/** Subtract v*u' from scatter.
*/
  void sub_outer_product(const VectorFixed<3,T> & u,
        const VectorFixed<3,T> & v);

/** Replace S with $(S+S^\top)/2$.
*/
  void force_symmetric();

/** Compute the eigensystem of S.
*/
  void compute_eigensystem();

/** Return the eigenvector corresponding to the smallest eigenvalue.
*/
  VectorFixed<3,T> minimum_eigenvector() {
    if (!eigenvectors_currentp) compute_eigensystem();
    return VectorFixed<3,T>(V_(0,0), V_(1,0), V_(2,0));
  }

/** Return the column matrix of eigenvectors, sorted in increasing order of eigenvalue.
*/
  MatrixFixed<3,3,T>& V() {
    if (!eigenvectors_currentp) compute_eigensystem();
    return V_;
  }

 protected:
  bool symmetricp;
  bool eigenvectors_currentp;
  MatrixFixed<3,3,T> V_;
  VectorFixed<3,T> D;
};


}; // End namespace VNL

#endif // vnl_scatter_3x3_h_

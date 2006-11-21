// This is vxl/VNL/algo/orthogonal_complement.h
#ifndef vnl_orthogonal_complement_h_
#define vnl_orthogonal_complement_h_
/**
* \file
* \brief For computing the orthogonal complement to a linear subspace.

* \author fsm@robots.ox.ac.uk
*
   \verbatim
   Modifications
   4/4/01 LSB(Manchester) Tidied documentation
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*/

#include <VNL/vector.h>
#include <VNL/matrix.h>

/** Return a matrix whose columns span is the orthogonal complement of v.
*/
template <class T>
VNL::Matrix<T> vnl_orthogonal_complement(VNL::Vector<T> const &v);

#if 0
/** Return a matrix whose column span is the orthogonal complement of the column span of M.
*/
template <typename T>
VNL::Matrix<T> vnl_orthogonal_complement(VNL::Matrix<T> const &M);
#endif

#endif // vnl_orthogonal_complement_h_

// This is vxl/VNL/identity.h
#ifndef vnl_identity_h_
#define vnl_identity_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/** \file
*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   07 Dec 98
*
   \verbatim
   Modification
   LSB (Manchester) 23/1/01 Tidied documentation
   \endverbatim
*/

#include <VNL/unaryfunction.h>

namespace VNL {

template <class T>
class Identity : public UnaryFunction<T,T>
{
 public:
  UnaryFunction<T,T>* Copy() const {
    Identity<T>* copy = new Identity<T>;
    *copy = *this;
    return copy;
  }

  T F(T const& x) {
    return x;
  }
};


}; // End namespace VNL

#endif // vnl_identity_h_

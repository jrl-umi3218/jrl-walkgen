// This is vxl/VNL/copy.h
#ifndef vnl_copy_h_
#define vnl_copy_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*   \file
*   \brief Easy conversion between vectors and matrices templated over different types.

*   \author fsm@robots.ox.ac.uk
*
   \verbatim
   Modifications
     LSB (Manchester) 26/3/01 Tidied documentation
   \endverbatim
*/

namespace VNL {

/** Easy conversion between vectors and matrices templated over different types.
*/
template <class S, class T>
void Copy(S const *src, T *dst, unsigned n);


/** Easy conversion between vectors and matrices templated over different types.
*/
template <class S, class T>
void Copy(S const &, T &);

}; // End namespace VNL

#endif // vnl_copy_h_

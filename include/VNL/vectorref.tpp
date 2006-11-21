// This is vxl/VNL/vector_ref.t
#ifndef vnl_vector_ref_t_
#define vnl_vector_ref_t_

/*
  fsm@robots.ox.ac.uk
*/
#include "vectorref.h"

//--------------------------------------------------------------------------------

#undef VNL_VECTOR_REF_INSTANTIATE
#define VNL_VECTOR_REF_INSTANTIATE(T) \
namespace VNL {template class VectorRef<T >;};

#endif // vnl_vector_ref_t_

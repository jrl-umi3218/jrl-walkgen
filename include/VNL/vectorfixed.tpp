// This is vxl/VNL/vector_fixed.t
#ifndef vnl_vector_fixed_t_
#define vnl_vector_fixed_t_
// Author: Andrew W. Fitzgibbon, Oxford RRG
// Created: 05 Aug 96

#include "vectorfixed.h"


// instantiation macros for vnl_vector_fixed<T,int> :

#define VNL_VECTOR_FIXED_INSTANTIATE(n,T) \
namespace VNL { template class VectorFixed<n, T >; \
template VNL::VectorFixed<n, T > operator+(T const, VNL::VectorFixedRef<n,T > const&);\
template VNL::VectorFixed<n, T > operator-(T const, VNL::VectorFixedRef<n,T > const&);\
template VNL::VectorFixed<n, T > operator*(T const, VNL::VectorFixedRef<n,T > const&);\
template VNL::VectorFixed<n, T > \
    ElementProduct (VNL::VectorFixedRef<n,T > const&, VNL::VectorFixedRef<n,T > const&);\
template VNL::VectorFixed<n, T > \
    ElementQuotient(VNL::VectorFixedRef<n,T > const&, VNL::VectorFixedRef<n,T > const&);\
}; 

//------------------------------------------------------------------------------

#define VNL_NON_TEMPLATE_FIXED_CROSS_3D_INSTANTIATE(T) \
VNL::VectorFixed<3,T > Cross3D(VNL::VectorFixed<3,T > const& v1, \
                               VNL::VectorFixed<3,T > const& v2) \
{ \
  VNL::VectorFixed<3,T > result; \
  result.x() = v1.y() * v2.z() - v1.z() * v2.y(); \
  result.y() = v1.z() * v2.x() - v1.x() * v2.z(); \
  result.z() = v1.x() * v2.y() - v1.y() * v2.x(); \
  return result; \
}

#endif // VNL::VectorFixed_t_

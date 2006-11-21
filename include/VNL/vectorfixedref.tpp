//-*- c++ -*-
// This is vxl/VNL/vector_fixed_ref.t
#ifndef vnl_vector_fixed_ref_t_
#define vnl_vector_fixed_ref_t_
// Author: Paul P. Smyth, Vicon Motion Systems Ltd. 
// Created: 02 May 2001
//
#include "vectorfixedref.h"
#include <assert.h>

template<int n, class T> VNL::Vector<T>& 
VNL::VectorFixedRef<n,T>::PreMultiply (VNL::Matrix<T> const& ) 
{
    assert(!"cannot use pre_multiply on VNL::VectorFixed<n,T>, since it deallocates the data storage");
    return *this;
}

template<int n, class T> VNL::Vector<T>& 
VNL::VectorFixedRef<n,T>::PostMultiply (VNL::Matrix<T> const& ) 
{
    assert(!"cannot use post_multiply on VNL::VectorFixed<n,T>, since it deallocates the data storage");
    return *this;
}

template<int n, class T> VNL::Vector<T>& 
VNL::VectorFixedRef<n,T>::operator*= (VNL::Matrix<T> const& ) 
{
    assert(!"cannot use pre_multiply on VNL::VectorFixedRef<n,T>, since it deallocates the data storage");
    return *this;
}

//------------------------------------------------------------

// instantiation macros for vnl_vector_fixed_ref<T,int> :

#define VNL_VECTOR_FIXED_REF_INSTANTIATE(n,T) \
namespace VNL {template class VectorFixedRef<n, T >;};

#endif // vnl_vector_fixed_ref_t_

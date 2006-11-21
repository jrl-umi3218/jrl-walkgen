// This is vxl/VNL/algo/orthogonal_complement.t
#ifndef vnl_orthogonal_complement_t_
#define vnl_orthogonal_complement_t_
/*
  fsm@robots.ox.ac.uk
*/
#include "orthogonalcomplement.h"
#include <VNL/Algo/svd.h>

template <class T>
VNL::Matrix<T> vnl_orthogonal_complement(VNL::Vector<T> const &v)
{
  unsigned n = v.size();
  VNL::Matrix<T> tmp(1, n);
  tmp.SetRow(0, v);
  return VNL::SVD<T>(tmp).V().Extract(n, n-1, 0, 1);
}

#if 0
template <class T>
VNL::Matrix<T> vnl_orthogonal_complement(VNL::Matrix<T> const &M)
{
  // \todo
}
#endif

//--------------------------------------------------------------------------------

#undef VNL_ORTHOGONAL_COMPLEMENT_INSTANTIATE
#define VNL_ORTHOGONAL_COMPLEMENT_INSTANTIATE(T) \
/* template VNL::Matrix<T > vnl_orthogonal_complement(VNL::Matrix<T > const &); */ \
template VNL::Matrix<T > vnl_orthogonal_complement(VNL::Vector<T > const &)

#endif // vnl_orthogonal_complement_t_

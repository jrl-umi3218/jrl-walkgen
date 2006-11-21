// This is vxl/VNL/fortran_copy.t
#ifndef vnl_fortran_copy_t_
#define vnl_fortran_copy_t_
/**
* \file
* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   29 Aug 96
*/

#include "fortrancopy.h"

/** Generate a fortran column-storage matrix from the given matrix.
*/
template <class T>
vnl_fortran_copy<T>::vnl_fortran_copy(VNL::Matrix<T> const & M)
{
  unsigned n = M.Rows();
  unsigned p = M.Columns();

  data = VNL::CVector<T>::AllocateT(sz = n*p);
  T *d = data;
  for (unsigned j = 0; j < p; ++j)
    for (unsigned i = 0; i < n; ++i)
      *d++ = M(i,j);
}

/** Destructor.
*/
template <class T>
vnl_fortran_copy<T>::~vnl_fortran_copy()
{
  VNL::CVector<T>::Deallocate(data, sz);
}

//--------------------------------------------------------------------------------

#undef VNL_FORTRAN_COPY_INSTANTIATE
#define VNL_FORTRAN_COPY_INSTANTIATE(T) template class vnl_fortran_copy<T >

#endif // vnl_fortran_copy_t_

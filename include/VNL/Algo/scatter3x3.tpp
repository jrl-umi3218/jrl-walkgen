// This is vxl/VNL/algo/scatter_3x3.t
#ifndef vnl_scatter_3x3_t_
#define vnl_scatter_3x3_t_

// Author: Andrew W. Fitzgibbon, Oxford RRG
// Created: 02 Oct 96
//-----------------------------------------------------------------------------

#include "scatter3x3.h"
#include <iostream>
#include <VNL/Algo/symmetriceigensystem.h>

template <class T>
VNL::Scatter3x3<T>::Scatter3x3()
  : base(T(0))
  , symmetricp(true)
  , eigenvectors_currentp(false)
{
}

template <class T>
void VNL::Scatter3x3<T>::add_outer_product(const VNL::VectorFixed<3,T> & v)
  // awf/gcc-2.7.2: VNL::Scatter3x3<T>::vect doesn't work in the arglist
{
  VNL::Scatter3x3<T> & S = *this;
  for(int i = 0; i < 3; ++i) {
    S(i,i) +=  v[i]*v[i];
    for(int j = i+1; j < 3; ++j) {
      T value = v[i]*v[j];
      S(i,j) += value;
      S(j,i) = S(i,j);
    }
  }
}

template <class T>
void VNL::Scatter3x3<T>::add_outer_product(const VNL::VectorFixed<3,T> & u,
                                           const VNL::VectorFixed<3,T> & v)
{
  VNL::Scatter3x3<T> & S = *this;
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      S(i,j) += v[i]*u[j];
  symmetricp = false; // conservative assumption -- use add_outer_product(v) to maintain symmetry
}

template <class T>
void VNL::Scatter3x3<T>::force_symmetric()
{
  if (symmetricp)
    return;
  VNL::Scatter3x3<T> & S = *this;
  for(int i = 0; i < 3; ++i)
    for(int j = i+1; j < 3; ++j) {
      T vbar = (S(i,j) + S(j,i)) * 0.5;
      S(i,j) = S(j,i) = vbar;
    }
  symmetricp = true;
}

template <class T>
void VNL::Scatter3x3<T>::compute_eigensystem()
{
  VNL::Scatter3x3<T> &S = *this;
  if (symmetricp)
    VNL::SymmetricEigensystemCompute(S, V_, D);
  else
    std::cerr << "Unsymmetric scatter not handled now\n";

  eigenvectors_currentp = true;
}

//--------------------------------------------------------------------------------

#define VNL_SCATTER_3X3_INSTANTIATE(T) \
template class VNL::Scatter3x3<T >

#endif // VNL::Scatter3x3_t_

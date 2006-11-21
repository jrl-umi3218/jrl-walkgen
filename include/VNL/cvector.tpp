// This is ./vxl/VNL/c_vector.t
#ifndef vnl_c_vector_t_
#define vnl_c_vector_t_

/**
* \file
* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   12 Feb 98
*
*/

#include "cvector.h"
#include <cmath>     // std::sqrt()
#include <VNL/vnlmath.h>
#include <VNL/complextraits.h>
#include <VNL/numerictraits.h>

template <class T>
T VNL::CVector<T>::Sum(T const* v, unsigned n)
{
  T tot(0);
  for(unsigned i = 0; i < n; ++i)
    tot += *v++;
  return tot;
}


template <class T>
void VNL::CVector<T>::Normalize(T* v, unsigned n)
{
using namespace std;
  typedef typename VNL::NumericTraits<T>::abs_t abs_t;
  typedef typename VNL::NumericTraits<abs_t>::real_t real_t;
  abs_t tmp(0);
  for(unsigned i = 0; i < n; ++i)
    tmp += VNL::SquaredMagnitude(v[i]);
  tmp = abs_t(real_t(1) / sqrt(real_t(tmp)));
  {
  for(unsigned i = 0; i < n; ++i)
    v[i] = T(tmp*v[i]);
	}
}

template <class T>
void VNL::CVector<T>::Apply(T const* v, unsigned n, T (*f)(T const&), T* v_out)
{
  for(unsigned i = 0; i < n; ++i)
    v_out[i] = f(v[i]);
}

template <class T>
void VNL::CVector<T>::Apply(T const* v, unsigned n, T (*f)(T), T* v_out) {
  for(unsigned i = 0; i < n; ++i)
    v_out[i] = f(v[i]);
}

template <class T>
void VNL::CVector<T>::Copy(T const *src, T *dst, unsigned n)
{
  for (unsigned i=0; i<n; ++i)
    dst[i] = src[i];
}

template <class T>
void VNL::CVector<T>::Scale(T const *x, T *y, unsigned n, T const &a_) {
  T a = a_;
  if (x == y)
    for (unsigned i=0; i<n; ++i)
      y[i] *= a;
  else
    for (unsigned i=0; i<n; ++i)
      y[i] = a*x[i];
}

//----------------------------------------------------------------------------
#define impl_elmt_wise_commutative(op) \
  if (z == x) \
    for (unsigned i=0; i<n; ++i) \
      z[i] op##= y[i]; \
\
  else if (z == y) \
    for (unsigned i=0; i<n; ++i) \
      z[i] op##= x[i]; \
\
  else \
    for (unsigned i=0; i<n; ++i) \
      z[i] = x[i] op y[i];

#define impl_elmt_wise_non_commutative(op) \
  if (z == x) \
    for (unsigned i=0; i<n; ++i) \
      z[i] op##= y[i]; \
\
  else \
    for (unsigned i=0; i<n; ++i) \
      z[i] = x[i] op y[i];

#define impl_elmt_wise_commutative_a(op) \
  if (z == x) \
    for (unsigned i=0; i<n; ++i) \
      z[i] op##= y; \
\
  else \
    for (unsigned i=0; i<n; ++i) \
      z[i] = x[i] op y;

#define impl_elmt_wise_non_commutative_a(op) \
  if (z == x) \
    for (unsigned i=0; i<n; ++i) \
      z[i] op##= y; \
\
  else \
    for (unsigned i=0; i<n; ++i) \
      z[i] = x[i] op y;

template <class T>
void VNL::CVector<T>::Add(T const *x, T const *y, T *z, unsigned n) {
  impl_elmt_wise_commutative(+);
}

template <class T>
void VNL::CVector<T>::Add(T const *x, T const& y, T *z, unsigned n) {
  impl_elmt_wise_commutative_a(+);
}

template <class T>
void VNL::CVector<T>::Subtract(T const *x, T const *y, T *z, unsigned n) {
  impl_elmt_wise_non_commutative(-);
}

template <class T>
void VNL::CVector<T>::Subtract(T const *x, T const& y, T *z, unsigned n) {
  impl_elmt_wise_commutative_a(-);
}

template <class T>
void VNL::CVector<T>::Multiply(T const *x, T const *y, T *z, unsigned n) {
  impl_elmt_wise_commutative(*);
}

template <class T>
void VNL::CVector<T>::Multiply(T const *x, T const& y, T *z, unsigned n) {
  impl_elmt_wise_commutative_a(*);
}

template <class T>
void VNL::CVector<T>::Divide(T const *x, T const *y, T *z, unsigned n) {
  impl_elmt_wise_non_commutative(/);
}

template <class T>
void VNL::CVector<T>::Divide(T const *x, T const& y, T *z, unsigned n) {
  impl_elmt_wise_commutative_a(/);
}

#undef impl_elmt_wise_commutative
#undef impl_elmt_wise_noncommutative
//--------------------------------------------------------------------------

template <class T>
void VNL::CVector<T>::Negate(T const *x, T *y, unsigned n) {
  if (x == y)
    for (unsigned i=0; i<n; ++i)
      y[i] = -y[i];
  else
    for (unsigned i=0; i<n; ++i)
      y[i] = -x[i];
}

template <class T>
void VNL::CVector<T>::Invert(T const *x, T *y, unsigned n) {
  if (x == y)
    for (unsigned i=0; i<n; ++i)
      y[i] = T(1)/y[i];
  else
    for (unsigned i=0; i<n; ++i)
      y[i] = T(1)/x[i];
}

template <class T>
void VNL::CVector<T>::SelfPlusAx(T const &a_, T const *x, T *y, unsigned n) {
  T a = a_;
  for (unsigned i=0; i<n; ++i)
    y[i] += a*x[i];
}

template <class T>
void VNL::CVector<T>::Fill(T *x, unsigned n, T const &v_) {
  T v = v_;
  for (unsigned i=0; i<n; ++i)
    x[i] = v;
}

template <class T>
void VNL::CVector<T>::Reverse (T *x, unsigned n) {
  for (unsigned i=0; 2*i<n; ++i) {
    T tmp = x[i];
    x[i] = x[n-1-i];
    x[n-1-i] = tmp;
  }
}

// non-conjugating "dot" product.
template<class T>
T VNL::CVector<T>::DotProduct(T const *a, T const *b, unsigned n) {
  T ip(0);
  for (unsigned i=0; i<n; ++i)
    ip += a[i] * b[i];
  return ip;
}

// conjugating "dot" product.
template<class T>
T VNL::CVector<T>::InnerProduct(T const *a, T const *b, unsigned n) {
  T ip(0);
  for (unsigned i=0; i<n; ++i)
    ip += a[i] * VNL::ComplexTraits<T>::Conjugate(b[i]);
  return ip;
}

// conjugates one block of data into another block.
template<class T>
void VNL::CVector<T>::Conjugate(T const *src, T *dst, unsigned n) {
  for (unsigned i=0; i<n; ++i)
    dst[i] = VNL::ComplexTraits<T>::Conjugate( src[i] );
}

//------------------------------------------------------------------------------

/** Returns max value of the vector.
*/
template<class T>
T VNL::CVector<T>::MaxValue(T const *src, unsigned n) {
  T tmp = src[0];

  for (unsigned i=1; i<n; ++i)
    if (src[i] > tmp)
      tmp = src[i];

  return tmp;
}

/** Returns min value of the vector.
*/
template<class T>
T VNL::CVector<T>::MinValue(T const *src, unsigned n) {
  T tmp = src[0];

  for (unsigned i=1; i<n; ++i)
    if (src[i] < tmp)
      tmp = src[i];

  return tmp;
}

// Sum of Differences squared.
template<class T>
T VNL::CVector<T>::EuclidDist2(T const *a, T const *b, unsigned n)
{
//IMS: Unable to optimise this any further for MSVC compiler
  T sum(0);
  T diff;
  for (unsigned i=0; i<n; ++i)
  {
    diff = a[i] - b[i];
    sum += diff*diff;
  }
  return sum;
}


//------------------------------------------------------------

template <class T, class S>
void VNL::CVectorTwoNormSquared(T const *p, unsigned n, S *out)
{
#if 1
// IMS: MSVC's optimiser does much better with this
// consistently about 30% better over vectors from 4 to 20000 dimensions.
// PVr: with gcc 3.0 on alpha this is even a factor 3 faster!
  S val =0;
  T const* end = p+n;
  while (p != end)
    val += VNL::SquaredMagnitude(*p++);
  *out = val;
#else
  *out = 0;
  for(unsigned i=0; i<n; ++i)
    *out += VNL::SquaredMagnitude(p[i]);
#endif
}

template <class T, class S>
void VNL::CVectorRMSNorm(T const *p, unsigned n, S *out)
{
	using namespace std;
  VNL::CVectorTwoNormSquared(p, n, out);
  *out /= n;
  typedef typename VNL::NumericTraits<S>::real_t real_t;
  *out = S(sqrt(real_t(*out)));
}

template <class T, class S>
void VNL::CVectorOneNorm(T const *p, unsigned n, S *out)
{
  *out = 0;
  for (unsigned i=0; i<n; ++i)
    *out += VNL::Abs(p[i]);
}

template <class T, class S>
void VNL::CVectorTwoNorm(T const *p, unsigned n, S *out)
{
	using namespace std;
  VNL::CVectorTwoNormSquared(p, n, out);
  typedef typename VNL::NumericTraits<S>::real_t real_t;
  *out = S(sqrt(real_t(*out)));
}

template <class T, class S>
void VNL::CVectorInfNorm(T const *p, unsigned n, S *out)
{
  *out = 0;
  for (unsigned i=0; i<n; ++i) {
    S v = VNL::Abs(p[i]);
    if (v > *out)
      *out = v;
  }
}


//---------------------------------------------------------------------------

#ifdef VNL_C_VECTOR_USE_VNL_ALLOC
// if set in build environment, go with that.
#else
// else, see what vnl_config.h has to say about it.
# include "config.h"
# if VNL_CONFIG_THREAD_SAFE
#  define VNL_C_VECTOR_USE_VNL_ALLOC 0
# else
#  define VNL_C_VECTOR_USE_VNL_ALLOC 1
# endif
#endif

#if VNL_C_VECTOR_USE_VNL_ALLOC
# include <VNL/alloc.h>
#endif

//#include <iostream>

inline void* vnl_c_vector_alloc(int n, int size)
{
  //std::cerr << "\ncall to vnl_c_vector_alloc(" << n << ", " << size << ")\n";
  //#if vnl_c_vector_use_win32_native_alloc
  // native was:  return (T**)std::allocator<T*>().allocate(n, 0);
  // on windows, it just calls malloc, so is useless....
#if VNL_C_VECTOR_USE_VNL_ALLOC
  return vnl_alloc::allocate((n == 0) ? 8 : (n * size));
#else
  return new char[n * size];
#endif
}

inline void vnl_c_vector_dealloc(void* v, int n, int size)
{
  //std::cerr << "\ncall to vnl_c_vector_dealloc(" << v << ", " << n
  //         << ", " << size << ")\n";
#if VNL_C_VECTOR_USE_VNL_ALLOC
  if (v)
    vnl_alloc::deallocate(v, (n == 0) ? 8 : (n * size));
#else
  delete [] static_cast<char*>(v);
#endif
}


template<class T>
T** VNL::CVector<T>::AllocateTptr(int n)
{
  return (T**)vnl_c_vector_alloc(n, sizeof (T*));
}

template<class T>
void VNL::CVector<T>::Deallocate(T** v, int n)
{
  vnl_c_vector_dealloc(v, n, sizeof (T*));
}

// "T *" is POD, but "T" might not be.
#include <new>
template <class T> inline void vnl_c_vector_construct(T *p, int n)
{
  for (int i=0; i<n; ++i)
    new (p+i) T();
}
#if 1
inline void vnl_c_vector_construct(float *, int) { }
inline void vnl_c_vector_construct(double *, int) { }
inline void vnl_c_vector_construct(long double *, int) { }
inline void vnl_c_vector_construct(std::complex<float> *, int) { }
inline void vnl_c_vector_construct(std::complex<double> *, int) { }
inline void vnl_c_vector_construct(std::complex<long double> *, int) { }
#endif
template <class T> inline void vnl_c_vector_destruct(T *p, int n)
{
  for (int i=0; i<n; ++i)
    (p+i)->~T();
}
#if 1
inline void vnl_c_vector_destruct(float *, int) { }
inline void vnl_c_vector_destruct(double *, int) { }
inline void vnl_c_vector_destruct(long double *, int) { }
inline void vnl_c_vector_destruct(std::complex<float> *, int) { }
inline void vnl_c_vector_destruct(std::complex<double> *, int) { }
inline void vnl_c_vector_destruct(std::complex<long double> *, int) { }
#endif

template<class T>
T* VNL::CVector<T>::AllocateT(int n)
{
  T *p = (T*)vnl_c_vector_alloc(n, sizeof (T));
  vnl_c_vector_construct(p, n);
  return p;
}

template<class T>
void VNL::CVector<T>::Deallocate(T* p, int n)
{
  vnl_c_vector_destruct(p, n);
  vnl_c_vector_dealloc(p, n, sizeof (T));
}

template<class T>
std::ostream& VNL::CVector<T>::PrintVector(std::ostream& s, T const* v, unsigned size)
{
  for (unsigned i = 0; i+1 < size; ++i)   // For each index in vector
    s << v[i] << " ";                              // Output data element
  if (size > 0)  s << v[size-1];
  return s;
}

//---------------------------------------------------------------------------

#define VNL_C_VECTOR_INSTANTIATE_norm(T, S) \
namespace VNL {\
template void CVectorTwoNormSquared(T const *, unsigned, S *); \
template void CVectorRMSNorm(T const *, unsigned, S *); \
template void CVectorOneNorm(T const *, unsigned, S *); \
template void CVectorTwoNorm(T const *, unsigned, S *); \
template void CVectorInfNorm(T const *, unsigned, S *); \
}

#undef VNL_C_VECTOR_INSTANTIATE_ordered
#define VNL_C_VECTOR_INSTANTIATE_ordered(T) \
VNL_C_VECTOR_INSTANTIATE_norm(T, VNL::CVector<T >::abs_t); \
namespace VNL {template class CVector<T >;};


#undef VNL_C_VECTOR_INSTANTIATE_unordered
#define VNL_C_VECTOR_INSTANTIATE_unordered(T) \
T VNL::CVector<T >::MaxValue(T const *, unsigned) {return T(0);}; \
T VNL::CVector<T >::MinValue(T const *, unsigned) {return T(0);}; \
VNL_C_VECTOR_INSTANTIATE_norm(T, VNL::CVector<T >::abs_t); \
namespace VNL {template class CVector<T >;};

#undef VNL_C_VECTOR_INSTANTIATE
#define VNL_C_VECTOR_INSTANTIATE(T) extern "no such macro"

#endif // vnl_c_vector_t_

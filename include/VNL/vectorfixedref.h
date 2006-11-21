// This is vxl/VNL/vector_fixed_ref.h
#ifndef vnl_vector_fixed_ref_h_
#define vnl_vector_fixed_ref_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Fixed size vnl_vector using user-supplied storage

*
* See also vnl_vector_ref, vnl_vector_fixed
*
* \author Paul P. Smyth, Vicon Motion Systems Ltd.
* \date 02 May 2001
*/

#include <assert.h>
#include <VNL/vector.h>
#include <VNL/vectorref.h>

namespace VNL {

template <int n, class T>
class VectorFixedRef : public VectorRef<T>
{
 public:
  VectorFixedRef(Vector<T>& rhs, unsigned int offset)
    : VectorRef<T>(n, rhs.DataBlock() + offset)
  {
    assert(& rhs != this);
    assert(rhs.size() >= offset + n);
  }

  VectorFixedRef(T *space) : VectorRef<T>(n, space)
  {
  }

  VectorFixedRef<n,T>& operator= (T const& t)
    { Vector<T>::operator=  (t); return *this; }
  VectorFixedRef<n,T>& operator+= (T const t)
    { Vector<T>::operator+= (t); return *this; }
  VectorFixedRef<n,T>& operator-= (T const t)
    { Vector<T>::operator-= (t); return *this; }
  VectorFixedRef<n,T>& operator*= (T const t)
    { Vector<T>::operator*= (t); return *this; }
  VectorFixedRef<n,T>& operator/= (T const t)
    { Vector<T>::operator/= (t); return *this; }

  VectorFixedRef<n,T>& operator+= (Vector<T> const& rhs)
    { Vector<T>::operator+= (rhs); return *this; }
  VectorFixedRef<n,T>& operator-= (Vector<T> const& rhs)
    { Vector<T>::operator-= (rhs); return *this; }


  VectorFixedRef<n,T>& Update (Vector<T> const& v, unsigned int start=0)
    { return (VectorFixedRef<n,T>&) Vector<T>::Update (v, start); }

  VectorFixedRef<n,T>& Normalize()  // v /= sqrt(dot(v,v))
    { return (VectorFixedRef<n,T>&) Vector<T>::Normalize(); }


 public:
  // void these methods on vnl_vector_fixed, since they deallocate the underlying
  // storage
  Vector<T>& PreMultiply (Matrix<T> const&); // v = m * v
  Vector<T>& PostMultiply (Matrix<T> const&); // v = v * m
  Vector<T>& operator*= (Matrix<T> const&);
};

}; // End namespace VNL

#endif // VectorFixedRef_h_

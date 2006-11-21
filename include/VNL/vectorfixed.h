// This is vxl/VNL/vector_fixed.h
#ifndef _vnl_vector_fixed_h_
#define _vnl_vector_fixed_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Fixed length stack-stored vnl_vector

 *
 *    vnl_vector_fixed is a fixed-length, stack storage vnl_vector.
 *  vnl_vector_fixed allocates storage space,
 *  and passes reference to this space to vnl_vector_ref
 *  See the docs for vnl_matrix_ref
 *
 * \author Andrew W. Fitzgibbon, Oxford RRG
 * \date   04 Aug 96
 *
 \verbatim
 Modifications
 LSB Manchester 16/3/01 Binary I/O added
 Paul Smyth     02/5/01 Inserted vnl_vector_fixed_ref as immediate base clase
 Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
 \endverbatim
*/



#include <cstring> // memcpy()
#include <VNL/vectorfixedref.h>
#include <VNL/vectorref.h>
#include <VNL/cvector.h>

namespace VNL {
	
	// forward-declare friends
	
#ifndef _WIN32
	// VC++ can't cope with forward declaring templated namespace members
	template <int n, class T> class VectorFixed;
	
	template <int n, class T> VectorFixed<n,T> 
		ElementProduct(VectorFixedRef<n,T> const&, VectorFixedRef<n,T> const&);
	
	template <int n, class T> VectorFixed<n,T> 
		ElementQuotient(VectorFixedRef<n,T>const&, VectorFixedRef<n,T> const&);
	
#endif
	
	
	// Export not yet supported - uncomment this when it is
	//export template <int n, class T = double> class VectorFixed
	
	/** fixed length  stack-stored vnl_vector.
	*
	*  VectorFixed is a fixed-length, stack storage vnl_vector.
	*  VectorFixed allocates storage space,
	*  and passes reference to this space to vnl_vector_ref
	* \see vnl_matrix_ref
	*
	*/
	template <int n, class T >
		class VectorFixed : public VectorFixedRef<n,T>
	{
		typedef VectorFixedRef<n,T> Base;
 public:
 /** Construct an uninitialized n-vector.
	 */
	 VectorFixed():Base(space) {}
	 
	 /** Construct an n-vector copy of rhs.
	 *  Does not check that rhs is the right size.
	 */
	 VectorFixed(Vector<T> const& rhs):Base(space) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
		 if (rhs.size() != n)
			 ErrorVectorDimension ("VNL::VectorFixed(const VNL::Vector&) ", n, rhs.size());
#endif
		 memcpy(space, rhs.DataBlock(), sizeof space);
	 }
	 
	 /**
	 * GCC generates (and calls) this even though above should do...
	 */
	 VectorFixed(VectorFixed<n,T> const& rhs):Base(space) {
		 memcpy(space, rhs.space, sizeof space);
	 }
	 
	 /** Constructs n-vector with elements initialised to v.
	 */
	 VectorFixed (T const& v): Base(space) {
		 for (int i = 0; i < n; ++i)
			 Base::data[i] = v;
	 }
	 
	 /** Constructs 3D vector(px, py, pz ).
	 */
	 VectorFixed (T const& px, T const& py, T const& pz): Base(space) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
		 if (n != 3) ErrorVectorDimension ("constructor (x,y,z): n != 3", n, 3);
#endif
		 Base::data[0] = px;
		 Base::data[1] = py;
		 Base::data[2] = pz;
	 }
	 
	 /** Constructs 2D vector  (px, py).
	 */
	 VectorFixed (T const& px, T const& py): Base(space) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
		 if (n != 2) ErrorVectorDimension ("constructor (x,y): n != 2", n, 2);
#endif
		 Base::data[0] = px;
		 Base::data[1] = py;
	 }
	 
	 VectorFixed<n,T>& operator=(VectorFixed<n,T> const& rhs) {
		 memcpy(space, rhs.space, sizeof space);
		 return *this;
	 }
	 
	 VectorFixed<n,T>& operator=(Vector<T> const& rhs) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
		 if (rhs.size() != n)
			 ErrorVectorDimension ("operator=", n, rhs.size());
#endif
		 memcpy(space, rhs.DataBlock(), sizeof space);
		 return *this;
	 }
	 
	 VectorFixed<n,T> Apply(T (*f)(T)) {
		 VectorFixed<n,T> ret;
		 VNL::CVector<T>::Apply(this->data, Base::num_elmts, f, ret.data);
		 return ret;
	 }
	 
	 VectorFixed<n,T> Apply(T (*f)(T const&)) {
		 VectorFixed<n,T> ret;
		 VNL::CVector<T>::Apply(this->data, Base::num_elmts, f, ret.data);
		 return ret;
	 }
	 
	 VectorFixed<n,T> operator- () const
	 { return  VectorFixed<n,T> (*this) *= -1; }
	 VectorFixed<n,T> operator+ (T const t) const
	 { return  VectorFixed<n,T> (*this) += t; }
	 VectorFixed<n,T> operator- (T const t) const
	 { return  VectorFixed<n,T> (*this) -= t; }
	 VectorFixed<n,T> operator* (T const t) const
	 { return  VectorFixed<n,T> (*this) *= t; }
	 VectorFixed<n,T> operator/ (T const t) const
	 { return  VectorFixed<n,T> (*this) /= t; }
	 
	 VectorFixed<n,T> operator+ (Vector<T> const& rhs) const
	 { return  VectorFixed<n,T> (*this) += rhs; }
	 VectorFixed<n,T> operator- (Vector<T> const& rhs) const
	 { return  VectorFixed<n,T> (*this) -= rhs; }
	 
#ifdef _WIN32
	 friend VectorFixed<n,T> ElementProduct  (VectorFixedRef<n,T> const&,
		 VectorFixedRef<n,T> const&);
	 friend VectorFixed<n,T> ElementQuotient  (VectorFixedRef<n,T> const&,
		 VectorFixedRef<n,T> const&);
#else
	 friend VectorFixed<n,T> ElementProduct <> (VectorFixedRef<n,T> const&,
		 VectorFixedRef<n,T> const&);
	 friend VectorFixed<n,T> ElementQuotient <> (VectorFixedRef<n,T> const&,
		 VectorFixedRef<n,T> const&);
	 
	 
#endif
	 
 private:
	 T space[n];
};



// Non-member inlines

// define inline friends.

template <int n, class T>
inline VectorFixed<n,T> operator+(T const t, VectorFixedRef<n,T> const & rhs)
{ return  VectorFixed<n,T> (rhs) += t; }

template <int n, class T>
inline VectorFixed<n,T> operator-(T const t, VectorFixedRef<n,T> const & rhs)
{ return  ( - VectorFixed<n,T> (rhs)) += t; }

template <int n, class T>
inline VectorFixed<n,T> operator*(T const t, VectorFixedRef<n,T> const& rhs)
{ return  VectorFixed<n,T> (rhs) *= t; }


template <int n, class T>
inline VectorFixed<n,T> ElementProduct (VectorFixedRef<n,T> const& a,
										VectorFixedRef<n,T> const& b)
{
	VectorFixed<n,T> ret (a);
	for (int i=0; i<n; i++) ret[i] *= b[i];
	return ret;
}

template <int n, class T>
inline VectorFixed<n,T> ElementQuotient (VectorFixedRef<n,T> const& a,
										 VectorFixedRef<n,T> const& b)
{
	VectorFixed<n,T> ret (a);
	for (int i=0; i<n; i++) ret[i] /= b[i];
	return ret;
}


/* VectorFixed<3,double> Cross3D (VectorFixed<3,double> const& vect1, */
/* 							   VectorFixed<3,double> const& vect2);							    */
/* VectorFixed<3,double> Cross3D (VectorFixed<3,double> const& vect1, */
/* VectorFixed<3,double> const& vect2); */
/* VectorFixed<3,float> Cross3D (VectorFixed<3,float> const& vect1, */
/* VectorFixed<3,float> const& vect2); */
/* VectorFixed<3,int> Cross3D (VectorFixed<3,int> const& vect1, */
/* VectorFixed<3,int> const& vect2); */


}; // End namespace VNL

#endif // VectorFixed_h_

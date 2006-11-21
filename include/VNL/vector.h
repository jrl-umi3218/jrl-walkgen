// This is vxl/VNL/vector.h

#include <VNL/VC6WarningsFix.h>

#ifndef _vnl_vector_h_
#define _vnl_vector_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \author Andrew W. Fitzgibbon
*
   \verbatim
   Modifications
   Comments re-written by Tim Cootes, for his sins.
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*/

#include <iosfwd>
#include <VNL/tag.h>
#include <VNL/cvector.h>
#ifndef NDEBUG
# include <VNL/error.h>
# include "config.h"
# if VNL_CONFIG_CHECK_BOUNDS
#  include <assert.h>
# endif
#else
# define VNL_CONFIG_CHECK_BOUNDS 0
#endif

namespace VNL {

  // The export command isn't supported in GCC and generates oodles of warnings
  //export template <class T> class Vector;
  //export template <class T> class Matrix;
  template <class T> class Vector;
  template <class T> class Matrix;


  typedef Vector<double> VecD;
  typedef Vector<float> VecF;
  //----------------------------------------------------------------------
  
#define v Vector<T>
#define m Matrix<T>
  template <class T> T      DotProduct (v const&, v const&);
  template <class T> T      InnerProduct (v const&, v const&);
  template <class T> T      Bracket (v const &, m const &, v const &);
  template <class T> T      CosAngle(v const&, v const& );
  template <class T> double Angle (v const&, v const&);
  template <class T> m      OuterProduct (v const&, v const&);
  template <class T> v      operator+(T, v const&);
  template <class T> v      operator-(T, v const&);
  template <class T> v      operator*(T, v const&);
  // also exists as method: template <class T> v      operator*(m const&, v const&);
  template <class T> v      operator*(v const&, m const&);
  template <class T> v      ElementProduct(v const&,v const&);
  template <class T> v      ElementQuotient(v const&,v const&);
  template <class T> T      Cross2D (v const&, v const&);
  template <class T> v      Cross3D (v const&, v const&);
  template <class T> T      VectorSSD(v const&, v const&);
  template <class T> void   Swap(v &, v &);
#undef v
#undef m
  
  //----------------------------------------------------------------------
  
  
  
  /** Mathematical vector class, templated by type of element.
   * The Vector<T> class implements one-dimensional arithmetic
   * vectors to be used with the Matrix<T> class. Vector<T>
   * has size fixed by constructor time or changed by assignment
   * operator.
   * For faster, non-mallocing vectors with size known at compile
   * time, use VectorFixed*.
   *
   * NOTE: Vectors are indexed from zero!  Thus valid elements are [0,size()-1].
   */
  template<class T>
  class Vector
  {
  public:
    friend class Matrix<T>;
    
    /** Creates an empty vector.\ O(1).
     */
    Vector () : num_elmts(0) , data(0) {}
    
    /** Creates vector containing n elements.
     * Elements are not initialized.
     */
    explicit Vector (unsigned len);
    
    /** Creates vector of len elements, all set to v0.
     */
    Vector (unsigned len, T const& v0);
    
    /** Creates a vector of specified length and initialize first n elements with values.\ O(n).
     */
  Vector (unsigned len, int n, T const values[]);

/** Creates a vector of length 3 and initializes with the arguments, x,y,z.
*/
  Vector (T const&, T const&, T const&);

/** Create n element vector and copy data from data_block.
*/
  Vector (T const* data_block,unsigned int n);

/** Copy constructor.
*/
  Vector (Vector<T> const&);

#ifndef VXL_DOXYGEN_SHOULD_SKIP_THIS 
// <internal>
  // These constructors are here so that operator* etc can take
  // advantage of the C++ return value optimization.
  Vector (Vector<T> const &, Vector<T> const &, VNL::TagAdd); // v + v
  Vector (Vector<T> const &, Vector<T> const &, VNL::TagSub); // v - v
  Vector (Vector<T> const &, T,                 VNL::TagMul); // v * s
  Vector (Vector<T> const &, T,                 VNL::TagDiv); // v / s
  Vector (Vector<T> const &, T,                 VNL::TagAdd); // v + s
  Vector (Vector<T> const &, T,                 VNL::TagSub); // v - s
  Vector (Matrix<T> const &, Vector<T> const &, VNL::TagMul); // M * v
  Vector (Vector<T> const &, Matrix<T> const &, VNL::TagMul); // v * M
  Vector (Vector<T> &that, VNL::TagGrab)
    : num_elmts(that.num_elmts), data(that.data)
  { that.num_elmts=0; that.data=0; } // "*this" now uses "that"'s data.
// </internal>
#endif

/** Destructor.
*/
  ~Vector() { if (data) destroy(); }

/** Return the length, number of elements, dimension of this vector.
*/
  unsigned size() const { return num_elmts; }

/** Return the length, number of elements, dimension of this vector.
*/
  unsigned Size() const { return num_elmts; }

/** Put value at given position in vector.
*/
  inline void Put (unsigned int i, T const&);

/** Get value at element i.
*/
  inline T Get (unsigned int i) const;

/** Set all values to v.
*/
  void Fill (T const& v);

/** Sets elements to ptr[i].
*  Note: ptr[i] must be valid for i=0..size()-1
*/
  void CopyIn(T const * ptr);

/** Copy elements to ptr[i].
*  Note: ptr[i] must be valid for i=0..size()-1
*/
  void CopyOut(T *) const; // from vector to array[].


/** Sets elements to ptr[i].
*  Note: ptr[i] must be valid for i=0..size()-1
*/
  void Set (T const *ptr) { CopyIn(ptr); }

/** Return reference to the element at specified index.
* There are assert style boundary checks - #define NDEBUG to turn them off.
*/
  T       & operator() (unsigned int i)
  {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
    assert(i<size());   // Check the index is valid.
#endif
    return data[i];
  }
/** Return reference to the element at specified index.\ No range checking.
* There are assert style boundary checks - #define NDEBUG to turn them off.
*/
  T const & operator() (unsigned int i) const
  {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
    assert(i<size());   // Check the index is valid
#endif
    return data[i];
  }

/** Return reference to the element at specified index.\ No range checking.
*/
  T       & operator[] (unsigned int i) { return data[i]; }
/** Return reference to the element at specified index.\ No range checking.
*/
  T const & operator[] (unsigned int i) const { return data[i]; }

/** Set all elements to value v.
*/
  Vector<T>& operator= (T const&v) { Fill(v); return *this; }

/** Copy operator.
*/
  Vector<T>& operator= (Vector<T> const& rhs);

/** Add scalar value to all elements.
*/
  Vector<T>& operator+= (T );

/** Subtract scalar value from all elements.
*/
  Vector<T>& operator-= (T );

/** Multiply all elements by scalar.
*/
  Vector<T>& operator*= (T );

/** Divide all elements by scalar.
*/
  Vector<T>& operator/= (T );

/** Add rhs to this and return *this.
*/
  Vector<T>& operator+= (Vector<T> const& rhs);

/** Subtract rhs from this and return *this.
*/
  Vector<T>& operator-= (Vector<T> const& rhs);

/** *this = M*(*this) where M is a suitable matrix.
*  this is treated as a column vector
*/
  Vector<T>& PreMultiply (Matrix<T> const& M);

/** *this = (*this)*M where M is a suitable matrix.
*  this is treated as a row vector
*/
  Vector<T>& PostMultiply (Matrix<T> const& M);

/** *this = (*this)*M where M is a suitable matrix.
*  this is treated as a row vector
*/
  Vector<T>& operator*= (Matrix<T> const& m) { return this->PostMultiply(m); }

/** Unary plus operator.
* Return new vector = (*this)
*/
  Vector<T> operator+ () const { return *this; }

/** Unary minus operator.
* Return new vector = -1*(*this)
*/
  Vector<T> operator- () const;

  Vector<T> operator+ (T v) const { return Vector<T>(*this, v, VNL::TagAdd()); }
  Vector<T> operator- (T v) const { return Vector<T>(*this, v, VNL::TagSub()); }
  Vector<T> operator* (T v) const { return Vector<T>(*this, v, VNL::TagMul()); }
  Vector<T> operator/ (T v) const { return Vector<T>(*this, v, VNL::TagDiv()); }

  Vector<T> operator+ (Vector<T> const& v) const { return Vector<T>(*this, v, VNL::TagAdd()); }
  Vector<T> operator- (Vector<T> const& v) const { return Vector<T>(*this, v, VNL::TagSub()); }
  Vector<T> operator* (Matrix<T> const& M) const { return Vector<T>(*this, M, VNL::TagMul()); }

  //--------------------------------------------------------------------------------

/** Access the contiguous block storing the elements in the vector.\ O(1).
*  data_block()[0] is the first element of the vector
*/
  T const* DataBlock () const { return data; }

/** Access the contiguous block storing the elements in the vector.\ O(1).
*  data_block()[0] is the first element of the vector
*/
  T      * DataBlock () { return data; }

/** Type defs for iterators.
*/
  typedef T element_type;
/** Type defs for iterators.
*/
  typedef T       *iterator;
/** Iterator pointing to start of data.
*/
  iterator begin() { return data; }

/** Iterator pointing to element beyond end of data.
*/
  iterator end() { return data+num_elmts; }

/** Const iterator type.
*/
  typedef T const *const_iterator;
/** Iterator pointing to start of data.
*/
  const_iterator begin() const { return data; }
/** Iterator pointing to element beyond end of data.
*/
  const_iterator end() const { return data+num_elmts; }

/** Applies function to elements.
*/
  Vector<T> Apply(T (*f)(T)) const;
/** Applies function to elements.
*/
  Vector<T> Apply(T (*f)(T const&)) const;

/** Returns a subvector specified by the start index and length.\ O(n).
*/
  Vector<T> Extract (unsigned int len, unsigned int start=0) const;

/** Replaces elements with index begining at start, by values of v.\ O(n).
*/
  Vector<T>& Update (Vector<T> const&, unsigned int start=0);

  // norms etc
  typedef typename CVector<T>::abs_t abs_t;

/** Return sum of squares of elements.
*/
  abs_t SquaredMagnitude() const { return CVector<T>::TwoNorm2(begin(), size()); }

/** Return magnitude (length) of vector.
*/
  abs_t Magnitude() const { return TwoNorm(); }

/** Return sum of absolute values of the elements.
*/
  abs_t OneNorm() const { return CVector<T>::OneNorm(begin(), size()); }

/** Return sqrt of sum of squares of values of elements.
*/
  abs_t TwoNorm() const { return CVector<T>::TwoNorm(begin(), size()); }

/** Return largest absolute element value.
*/
  abs_t InfNorm() const { return CVector<T>::InfNorm(begin(), size()); }

/** Normalise by dividing through by the magnitude.
*/
  Vector<T>& Normalize() { CVector<T>::Normalize(begin(), size()); return *this; }

  // These next 6 functions are should really be helper functions since they aren't
  // really proper functions on a vector in a philosophial sense.

/** Root Mean Squares of values.
*/
  abs_t RMS     () const { return CVector<T>::RMSNorm(begin(), size()); }

/** Smallest value.
*/
  T MinValue () const { return CVector<T>::MinValue(begin(), size()); }

/** Largest value.
*/
  T MaxValue () const { return CVector<T>::MaxValue(begin(), size()); }

/** Mean of values in vector.
*/
  T Mean() const { return CVector<T>::Mean(begin(), size()); }

/** Sum of values in a vector.
*/
  T Sum() const { return CVector<T>::Sum(begin(), size()); }

/** Reverse the order of the elements.
*  Element i swaps with element size()-1-i
*/
  void Flip();

/** Set this to that and that to this.
*/
  void Swap(Vector<T> & that);

/** Return first element of vector.
*/
  T& x() const { return data[0]; }
/** Return second element of vector.
*/
  T& y() const { return data[1]; }
/** Return third element of vector.
*/
  T& z() const { return data[2]; }
/** Return fourth element of vector.
*/
  T& t() const { return data[3]; }

#if VNL_CONFIG_LEGACY_METHODS
/** Set the first element (with bound checking).
*/
  void set_x(T const&xx) { if (size() >= 1) data[0] = xx; }
/** Set the second element (with bound checking).
*/
  void set_y(T const&yy) { if (size() >= 2) data[1] = yy; }
/** Set the third element (with bound checking).
*/
  void set_z(T const&zz) { if (size() >= 3) data[2] = zz; }
/** Set the fourth element (with bound checking).
*/
  void set_t(T const&tt) { if (size() >= 4) data[3] = tt; }
#endif

/** Check that size()==sz if not, abort();.
* This function does or tests nothing if NDEBUG is defined
*/
  void assert_size(unsigned sz) const {
#ifndef NDEBUG
    assert_size_internal(sz);
#endif
  }

/** Check that this is finite if not, abort();.
* This function does or tests nothing if NDEBUG is defined
*/
  void assert_finite() const {
#ifndef NDEBUG
    assert_finite_internal();
#endif
  }

/** Return true if its finite.
*/
  bool IsFinite() const;

/** Return true iff all the entries are zero.
*/
  bool IsZero() const;

/** Return true iff the size is zero.
*/
  bool IsEmpty() const { return !data || !num_elmts; }

/** Return true if *this == v.
*/
  bool operator_eq (Vector<T> const& v) const;

/** Equality test.
*/
  bool operator==(Vector<T> const &that) const { return  this->operator_eq(that); }

/** Inequality test.
*/
  bool operator!=(Vector<T> const &that) const { return !this->operator_eq(that); }

/** Resize to n elements.
* Checks early and does nothing if already size n, otherwise
* old data is discarded.  Returns true if size change successful.
*/
  bool Resize (unsigned n);

/** Make the vector as if it had been default-constructed.
*/
  void Clear();


/** Read from text stream.
*/
  bool ReadASCII(std::istream& s);

/** Read from text stream.
*/
  static Vector<T> Read(std::istream& s);


 protected:
  unsigned num_elmts;           // Number of elements
  T* data;                      // Pointer to the Vector

  void assert_size_internal(unsigned sz) const;
  void assert_finite_internal() const;

  void destroy();

#if VCL_NEED_FRIEND_FOR_TEMPLATE_OVERLOAD
# define v Vector<T>
# define m Matrix<T>
  friend T      dot_product      <> (v const&, v const&);
  friend T      inner_product    <> (v const&, v const&);
  friend T      bracket          <> (v const&, m const&, v const&);
  friend T      cos_angle        <> (v const&, v const&);
  friend double angle            <> (v const&, v const&);
  friend m      outer_product    <> (v const&, v const&);
  friend v      operator+        <> (T const,  v const&);
  friend v      operator-        <> (T const,  v const&);
  friend v      operator*        <> (T const,  v const&);
  friend v      operator*        <> (m const&, v const&);
  friend v      element_product  <> (v const&, v const&);
  friend v      element_quotient <> (v const&, v const&);
  friend T      cross_2d         <> (v const&, v const&);
  friend v      cross_3d         <> (v const&, v const&);
# undef v
# undef m
#endif

  // inline function template instantiation hack for gcc 2.97 -- fsm
  static void inline_function_tickler();
};


// Definitions of inline functions


/** Gets the element at specified index and return its value.\ O(1).
* Range check is performed.
*/

template <class T>
inline T Vector<T>::Get (unsigned int index) const {
#if ERROR_CHECKING
  if (index >= this->num_elmts)     // If invalid index specified
    vnl_error_vector_index ("get", index);  // Raise exception
#endif
  return this->data[index];
}

/** Puts the value at specified index.\ O(1).
* Range check is performed.
*/

template <class T>
inline void Vector<T>::Put (unsigned int index, T const& value) {
#if ERROR_CHECKING
  if (index >= this->num_elmts)     // If invalid index specified
    vnl_error_vector_index ("put", index); // Raise exception
#endif
  this->data[index] = value;    // Assign data value
}






// Non member stuff (still in VNL namespace - see Stroustrup S11.2.4)

/** multiply matrix and (column) vector.\ O(m*n).
*/
template<class T>
inline Vector<T> operator* (Matrix<T> const& m, Vector<T> const& v) {
  return Vector<T>(m, v, VNL::TagMul());
}

/** add scalar and vector.\ O(n).
*/
template<class T>
inline Vector<T> operator+ (T s, Vector<T> const& v) {
  return Vector<T>(v, s, VNL::TagAdd());
}

/** subtract vector from scalar.\ O(n).
*/
template<class T>
inline Vector<T> operator- (T s, Vector<T> const& v) {
  return Vector<T>(-v, s, VNL::TagAdd());
}

/** multiply scalar and vector.\ O(n).
*/
template<class T>
inline Vector<T> operator* (T s, Vector<T> const& v) {
  return Vector<T>(v, s, VNL::TagMul());
}

template<class T>
inline void Swap(Vector<T> &a, Vector<T> &b) { a.swap(b); }

/** Euclidean Distance between two vectors.
* Sum of Differences squared.
*/
template<class T>
inline T VectorSSD (Vector<T> const& v1, Vector<T> const& v2)
{
#ifndef NDEBUG
  if (v1.size() != v2.size())
    vnl_error_vector_dimension ("vector_ssd", v1.size(), v2.size());
#endif
  return VNL::CVector<T>::EuclidDist2(v1.begin(), v2.begin(), v1.size());
}


// Non-vector Functions which are nevertheless very useful.


/** Read/write vector from/to a std::istream.
*/
template <class T> std::ostream& operator<< (std::ostream &, Vector<T> const&);
template <class T> std::istream& operator>> (std::istream &, Vector<T>      &);
//export template <class T> std::ostream& operator<< (std::ostream &, Vector<T> const&);
//export template <class T> std::istream& operator>> (std::istream &, Vector<T>      &);

}; // End namespace VNL


#endif // Vector_h_

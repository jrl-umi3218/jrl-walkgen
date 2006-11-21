#ifndef _vnl_vector_t_
#define _vnl_vector_t_

/**
* \file
*
* Created: VDN 02/21/92 new lite version adapted from Matrix.h
*
* The parameterized vnl_vector<T> class implements 1D arithmetic vectors of a
* user specified type. The only constraint placed on the type is that
* it must overload the following operators: +, -, *, and /. Thus, it will
* be possible to have a vnl_vector over std::complex<T>.  The vnl_vector<T>
* class is static in size, that is once a vnl_vector<T> of a particular
* size has been declared, elements cannot be added or removed. Using the
* resize() method causes the vector to resize, but the contents will be
* lost.
*
* Each vector contains  a protected  data section  that has a  T* slot that
* points to the  physical memory allocated  for the one  dimensional array. In
* addition, an integer  specifies   the number  of  elements  for the
* vector.  These values  are provided in the  constructors.
*
* Several constructors are provided. See .h file for descriptions.
*
* Methods   are  provided   for destructive   scalar   and vector  addition,
* multiplication, check for equality  and inequality, fill, reduce, and access
* and set individual elements.  Finally, both  the  input and output operators
* are overloaded to allow for fomatted input and output of vector elements.
*
* vnl_vector is a special type of matrix, and is implemented for space and time
* efficiency. When vnl_vector is pre_multiplied by/with matrix, m*v, vnl_vector is
* implicitly a column matrix. When vnl_vector is post_multiplied by/with matrix
* v*m, VNL::Vector is implicitly a row matrix.
*
*/

#include "vector.h"

#include <cstdlib> // abort()
#include <vector>
#include <iostream>
#include <algorithm>

#include <VNL/vnlmath.h>
#include <VNL/matrix.h>
#include <VNL/cvector.h>
#include <VNL/numerictraits.h>

//--------------------------------------------------------------------------------

// This macro allocates the dynamic storage used by a VNL::Vector.
#define vnl_vector_alloc_blah(size) \
do { \
  this->num_elmts = (size); \
  this->data = VNL::CVector<T>::AllocateT(this->num_elmts); \
} while(false)

// This macro deallocates the dynamic storage used by a VNL::Vector.
#define vnl_vector_free_blah \
do { \
  VNL::CVector<T>::Deallocate(this->data, this->num_elmts); \
} while (false)

/** Creates a vector with specified length.\ O(n).
* Elements are not initialized.
*/

template<class T>
VNL::Vector<T>::Vector (unsigned len)
{
  vnl_vector_alloc_blah(len);
}


/** Creates a vector of specified length, and initialize all elements with value.\ O(n).
*/

template<class T>
VNL::Vector<T>::Vector (unsigned len, T const& value)
{
  vnl_vector_alloc_blah(len);
  for (unsigned i = 0; i < len; i ++)           // For each elmt
    this->data[i] = value;                      // Assign initial value
}

/** Creates a vector of specified length and initialize first n elements with values.\ O(n).
*/

template<class T>
VNL::Vector<T>::Vector (unsigned len, int n, T const values[])
{
  vnl_vector_alloc_blah(len);
  if (n > 0) {                                  // If user specified values
    for (unsigned i = 0; i < len && n; i++, n--)        // Initialize first n elements
      this->data[i] = values[i];                // with values
  }
}

/** Creates a vector of length 3 and initializes with the arguments, x,y,z.
*/

template<class T>
VNL::Vector<T>::Vector (T const& px, T const& py, T const& pz)
{
  vnl_vector_alloc_blah(3);
  this->data[0] = px;
  this->data[1] = py;
  this->data[2] = pz;
}

#if 0 // commented out
/** Creates a vector of specified length and initialize first n elements with values in ...\ O(n).
* Arguments in ... can only be pointers, primitive types like int,
* and NOT OBJECTS passed by value, like vectors, matrices,
* because constructors must be known and called at compile time!!!
*/

template<class T>
VNL::Vector<T>::Vector (unsigned len, int n, T v00, ...)
: num_elmts(len), data(VNL::CVector<T>::allocate_T(len))
{
  std::cerr << "Please use automatic arrays instead variable arguments\n";
  if (n > 0) {                               // If user specified values
    va_list argp;                            // Declare argument list
    va_start (argp, v00);                    // Initialize macro
    for (int i = 0; i < len && n; i++, n--)  // For remaining values given
      if (i == 0)
     this->data[0] = v00;                    // Hack for v00 ...
      else
     this->data[i] = va_arg(argp, T);        // Extract and assign
    va_end(argp);
  }
}

template<class T>
VNL::Vector<T>::Vector (unsigned len, int n, T v00, ...)
: num_elmts(len), data(VNL::CVector<T>::allocate_T(len))
{
  std::cerr << "Please use automatic arrays instead variable arguments\n";
  if (n > 0) {                                  // If user specified values
    va_list argp;                               // Declare argument list
    va_start (argp, v00);                       // Initialize macro
    for (int i = 0; i < len && n; i++, n--)     // For remaining values given
      if (i == 0)
        this->data[0] = v00;                    // Hack for v00 ...
      else
        this->data[i] = va_arg(argp, T);        // Extract and assign
    va_end(argp);
  }
}
#endif

/** Creates a new copy of vector v.\ O(n).
*/
template<class T>
VNL::Vector<T>::Vector (VNL::Vector<T> const& v)
{
  vnl_vector_alloc_blah(v.num_elmts);
  for (unsigned i = 0; i < v.num_elmts; i ++)   // For each element in v
    this->data[i] = v.data[i];                  // Copy value
}

/** Creates a vector from a block array of data, stored row-wise.
* Values in datablck are copied. O(n).
*/

template<class T>
VNL::Vector<T>::Vector (T const* datablck, unsigned len)
{
  vnl_vector_alloc_blah(len);
  for (unsigned i = 0; i < len; ++i)    // Copy data from datablck
    this->data[i] = datablck[i];
}

//------------------------------------------------------------

template<class T>
VNL::Vector<T>::Vector (VNL::Vector<T> const &u, VNL::Vector<T> const &v, VNL::TagAdd)
{
  vnl_vector_alloc_blah(u.num_elmts);
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (u.size() != v.size())
    ErrorVectorDimension ("vector<>::vnl_vector(v, v, vnl_vector_add_tag)", u.size(), v.size());
#endif
  for (unsigned int i=0; i<num_elmts; ++i)
    data[i] = u[i] + v[i];
}

template<class T>
VNL::Vector<T>::Vector (VNL::Vector<T> const &u, VNL::Vector<T> const &v, VNL::TagSub)
{
  vnl_vector_alloc_blah(u.num_elmts);
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (u.size() != v.size())
    ErrorVectorDimension ("vector<>::vnl_vector(v, v, vnl_vector_sub_tag)", u.size(), v.size());
#endif
  for (unsigned int i=0; i<num_elmts; ++i)
    data[i] = u[i] - v[i];
}

template<class T>
VNL::Vector<T>::Vector (VNL::Vector<T> const &u, T s, VNL::TagMul)
{
  vnl_vector_alloc_blah(u.num_elmts);
  for (unsigned int i=0; i<num_elmts; ++i)
    data[i] = u[i] * s;
}

template<class T>
VNL::Vector<T>::Vector (VNL::Vector<T> const &u, T s, VNL::TagDiv)
{
  vnl_vector_alloc_blah(u.num_elmts);
  for (unsigned int i=0; i<num_elmts; ++i)
    data[i] = u[i] / s;
}

template<class T>
VNL::Vector<T>::Vector (VNL::Vector<T> const &u, T s, VNL::TagAdd)
{
  vnl_vector_alloc_blah(u.num_elmts);
  for (unsigned int i=0; i<num_elmts; ++i)
    data[i] = u[i] + s;
}

template<class T>
VNL::Vector<T>::Vector (VNL::Vector<T> const &u, T s, VNL::TagSub)
{
  vnl_vector_alloc_blah(u.num_elmts);
  for (unsigned int i=0; i<num_elmts; ++i)
    data[i] = u[i] - s;
}

template<class T>
VNL::Vector<T>::Vector (VNL::Matrix<T> const &M, VNL::Vector<T> const &v, VNL::TagMul)
{
  vnl_vector_alloc_blah(M.Rows());

#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (M.Columns() != v.size())
    ErrorVectorDimension ("vector<>::vnl_vector(M, v, vnl_vector_mul_tag)", M.Columns(), v.size());
#endif
  for (unsigned int i=0; i<num_elmts; ++i) {
    T sum(0);
    for (unsigned int j=0; j<M.Columns(); ++j)
      sum += M[i][j] * v[j];
    data[i] = sum;
  }
}

template<class T>
VNL::Vector<T>::Vector (VNL::Vector<T> const &v, VNL::Matrix<T> const &M, VNL::TagMul)
{
  vnl_vector_alloc_blah(M.Columns());
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (v.size() != M.Rows())
    ErrorVectorDimension ("vector<>::vnl_vector(v, M, vnl_vector_mul_tag)", v.size(), M.Rows());
#endif
  for (unsigned int j=0; j<num_elmts; ++j) {
    T sum(0);
    for (unsigned int i=0; i<M.Rows(); ++i)
      sum += v[i] * M[i][j];
    data[j] = sum;
  }
}

/** Frees up the array inside vector.\ O(1).
*/

template<class T>
void VNL::Vector<T>::destroy()
{
  vnl_vector_free_blah;
}

template<class T>
void VNL::Vector<T>::Clear()
{
  if (data) {
    destroy();
    num_elmts = 0;
    data = 0;
  }
}

template<class T>
bool VNL::Vector<T>::Resize(unsigned n)
{
  if (this->data) {
    // if no change in size, do not reallocate.
    if (this->num_elmts == n)
      return false;

    vnl_vector_free_blah;
    vnl_vector_alloc_blah(n);
  }
  else {
    // this happens if the vector is default constructed.
    vnl_vector_alloc_blah(n);
  }
  return true;
}

#undef vnl_vector_alloc_blah
#undef vnl_vector_free_blah

//------------------------------------------------------------

/** Read a VNL::Vector from an ascii std::istream.
* If the vector has nonzero size on input, read that many values.
* Otherwise, read to EOF.
*/
template <class T>
bool VNL::Vector<T>::ReadASCII(std::istream& s)
{
  bool size_known = (this->size() != 0);
  if (size_known) {
    for(unsigned i = 0; i < this->size(); ++i)
      s >> (*this)(i);
    return s.good() || s.eof();
  }

  // Just read until EOF
  std::vector<T> allvals;
  unsigned n = 0;
  while (!s.eof()) {
    T value;
    s >> value;

    if (!s.good())
      break;
    allvals.push_back(value);
    ++n;
  }
  this->Resize(n); //*this = VNL::Vector<T>(n);
  for(unsigned i = 0; i < n; ++i)
    (*this)[i] = allvals[i];
  return true;
}

template <class T>
VNL::Vector<T> VNL::Vector<T>::Read(std::istream& s)
{
  VNL::Vector<T> V;
  V.ReadASCII(s);
  return V;
}

/** Sets all elements of a vector to a specified fill value.\ O(n).
*/

template<class T>
void VNL::Vector<T>::Fill (T const& value) {
  for (unsigned i = 0; i < this->num_elmts; i++)
    this->data[i] = value;
}

/** Sets elements of a vector to those in an array.\ O(n).
*/

template<class T>
void VNL::Vector<T>::CopyIn (T const *ptr) {
  for (unsigned i = 0; i < num_elmts; ++i)
    data[i] = ptr[i];
}

/** Sets elements of an array to those in vector.\ O(n).
*/

template<class T>
void VNL::Vector<T>::CopyOut (T *ptr) const {
  for (unsigned i = 0; i < num_elmts; ++i)
    ptr[i] = data[i];
}

/** Copies rhs vector into lhs vector.\ O(n).
* Changes the dimension of lhs vector if necessary.
*/

template<class T>
VNL::Vector<T>& VNL::Vector<T>::operator= (VNL::Vector<T> const& rhs) {
  if (this != &rhs) { // make sure *this != m
    if (rhs.data) {
      if (this->num_elmts != rhs.num_elmts)
        this->Resize(rhs.size());
      for (unsigned i = 0; i < this->num_elmts; i++)
        this->data[i] = rhs.data[i];
    }
    else {
      // rhs is default-constructed.
      Clear();
    }
  }
  return *this;
}

/** Increments all elements of vector with value.\ O(n).
*/

template<class T>
VNL::Vector<T>& VNL::Vector<T>::operator+= (T value) {
  for (unsigned i = 0; i < this->num_elmts; i++)
    this->data[i] += value;
  return *this;
}

template<class T>
VNL::Vector<T>& VNL::Vector<T>::operator-= (T value) {
  for (unsigned i = 0; i < this->num_elmts; i++)
    this->data[i] -= value;
  return *this;
}

/** Multiplies all elements of vector with value.\ O(n).
*/

template<class T>
VNL::Vector<T>& VNL::Vector<T>::operator*= (T value) {
  for (unsigned i = 0; i < this->num_elmts; i++)
    this->data[i] *= value;
  return *this;
}

/** Divides all elements of vector by value.\ O(n).
*/

template<class T>
VNL::Vector<T>& VNL::Vector<T>::operator/= (T value) {
  for (unsigned i = 0; i < this->num_elmts; i++)
    this->data[i] /= value;
  return *this;
}


/** Mutates lhs vector with its addition with rhs vector.\ O(n).
*/

template<class T>
VNL::Vector<T>& VNL::Vector<T>::operator+= (VNL::Vector<T> const& rhs) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (this->num_elmts != rhs.num_elmts)
    ErrorVectorDimension ("operator+=",
                           this->num_elmts, rhs.num_elmts);
#endif
  for (unsigned i = 0; i < this->num_elmts; i++)
    this->data[i] += rhs.data[i];
  return *this;
}


/**  Mutates lhs vector with its substraction with rhs vector.\ O(n).
*/

template<class T>
VNL::Vector<T>& VNL::Vector<T>::operator-= (VNL::Vector<T> const& rhs) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (this->num_elmts != rhs.num_elmts)
    ErrorVectorDimension ("operator-=",
                           this->num_elmts, rhs.num_elmts);
#endif
  for (unsigned i = 0; i < this->num_elmts; i++)
    this->data[i] -= rhs.data[i];
  return *this;
}

/** Pre-multiplies vector with matrix and stores result back in vector.
* v = m * v. O(m*n). Vector is assumed a column matrix.
*/

template<class T>
VNL::Vector<T>& VNL::Vector<T>::PreMultiply (VNL::Matrix<T> const& m) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (m.Columns() != this->num_elmts)           // dimensions do not match?
    ErrorVectorDimension ("operator*=",
                           this->num_elmts, m.Columns());
#endif
  T* temp= VNL::CVector<T>::AllocateT(m.Rows()); // Temporary
  VNL::Matrix<T>& mm = (VNL::Matrix<T>&) m;       // Drop const for get()
  for (unsigned i = 0; i < m.Rows(); i++) {     // For each index
    temp[i] = (T)0;                             // Initialize element value
    for (unsigned k = 0; k < this->num_elmts; k++)      // Loop over column values
      temp[i] += (mm.Get(i,k) * this->data[k]); // Multiply
  }
  VNL::CVector<T>::Deallocate(this->data, this->num_elmts); // Free up the data space
  num_elmts = m.Rows();                         // Set new num_elmts
  this->data = temp;                            // Pointer to new storage
  return *this;                                 // Return vector reference
}

/** Post-multiplies vector with matrix and stores result back in vector.
* v = v * m. O(m*n). Vector is assumed a row matrix.
*/

template<class T>
VNL::Vector<T>& VNL::Vector<T>::PostMultiply (VNL::Matrix<T> const& m) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (this->num_elmts != m.Rows())              // dimensions do not match?
    ErrorVectorDimension ("operator*=", this->num_elmts, m.Rows());
#endif
  T* temp= VNL::CVector<T>::AllocateT(m.Columns()); // Temporary
  VNL::Matrix<T>& mm = (VNL::Matrix<T>&) m;       // Drop const for get()
  for (unsigned i = 0; i < m.Columns(); i++) {  // For each index
    temp[i] = (T)0;                             // Initialize element value
    for (unsigned k = 0; k < this->num_elmts; k++) // Loop over column values
      temp[i] += (this->data[k] * mm.Get(k,i)); // Multiply
  }
  VNL::CVector<T>::Deallocate(this->data, num_elmts); // Free up the data space
  num_elmts = m.Columns();                      // Set new num_elmts
  this->data = temp;                            // Pointer to new storage
  return *this;                                 // Return vector reference
}


/** Creates new vector containing the negation of THIS vector.\ O(n).
*/

template<class T>
VNL::Vector<T> VNL::Vector<T>::operator- () const {
  VNL::Vector<T> result(this->num_elmts);
  for (unsigned i = 0; i < this->num_elmts; i++)
    result.data[i] = - this->data[i];           // negate element
  return result;
}

#if 0 // commented out
/** Returns new vector which is the multiplication of matrix m with column vector v.\ O(m*n).
*/

template<class T>
VNL::Vector<T> operator* (VNL::Matrix<T> const& m, VNL::Vector<T> const& v) {

#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (m.Columns() != v.size())                  // dimensions do not match?
    ErrorVectorDimension ("operator*",
                                m.Columns(), v.size());
#endif
  VNL::Vector<T> result(m.Rows());               // Temporary
  VNL::Matrix<T>& mm = (VNL::Matrix<T>&) m;       // Drop const for get()
  for (unsigned i = 0; i < m.Rows(); i++) {     // For each index
    result[i] = (T)0;                           // Initialize element value
    for (unsigned k = 0; k < v.size(); k++)     // Loop over column values
      result[i] += (mm.get(i,k) * v[k]);        // Multiply
  }
  return result;
}


/** Returns new vector which is the multiplication of row vector v with matrix m.\ O(m*n).
*/

template<class T>
VNL::Vector<T> VNL::Vector<T>::operator* (VNL::Matrix<T> const&m) const {

  // rick@aai: casting away const avoids the following error (using gcc272)
  // at m.rows during instantiation of 'template class VNL::Vector<double >;'
  // "cannot lookup method in incomplete type `const VNL::Matrix<double>`"
  // For some reason, instantiating the following function prior to VNL::Vector
  // also avoids the error.
  // template VNL::Matrix<double > outer_product (const VNL::Vector<double >&,const VNL::Vector<dou

#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (num_elmts != m.Rows())                    // dimensions do not match?
    ErrorVectorDimension ("operator*", num_elmts, m.Rows());
#endif
  VNL::Vector<T> result(m.Columns());            // Temporary
  VNL::Matrix<T>& mm = (VNL::Matrix<T>&) m;       // Drop const for get()
  for (unsigned i = 0; i < m.Columns(); i++) {  // For each index
    result.data[i] = (T)0;                      // Initialize element value
    for (unsigned k = 0; k < num_elmts; k++)    // Loop over column values
      result.data[i] += (data[k] * mm.get(k,i)); // Multiply
  }
  return result;
}
#endif


/** Replaces elements with index begining at start, by values of v.\ O(n).
*/

template<class T>
VNL::Vector<T>& VNL::Vector<T>::Update (VNL::Vector<T> const& v, unsigned start) {
  unsigned end = start + v.size();
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if ( end> this->num_elmts)
    ErrorVectorDimension ("update", end-start, v.size());
#endif
  for (unsigned i = start; i < end; i++)
    this->data[i] = v.data[i-start];
  return *this;
}


/** Returns a subvector specified by the start index and length.\ O(n).
*/

template<class T>
VNL::Vector<T> VNL::Vector<T>::Extract (unsigned len, unsigned start) const {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  unsigned end = start + len;
  if (this->num_elmts < end)
    ErrorVectorDimension ("extract", end-start, len);
#endif
  VNL::Vector<T> result(len);
  for (unsigned i = 0; i < len; i++)
    result.data[i] = data[start+i];
  return result;
}

/** Returns new vector whose elements are the products v1[i]*v2[i].\ O(n).
*/

template<class T>
VNL::Vector<T> VNL::ElementProduct (VNL::Vector<T> const& v1, VNL::Vector<T> const& v2) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (v1.size() != v2.size())
    ErrorVectorDimension ("element_product", v1.size(), v2.size());
#endif
  VNL::Vector<T> result(v1.size());
  for (unsigned i = 0; i < v1.size(); i++)
    result[i] = v1[i] * v2[i];
  return result;
}

/** Returns new vector whose elements are the quotients v1[i]/v2[i].\ O(n).
*/

template<class T>
VNL::Vector<T> VNL::ElementQuotient (VNL::Vector<T> const& v1, VNL::Vector<T> const& v2) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (v1.size() != v2.size())
    ErrorVectorDimension ("element_quotient",
                        v1.size(), v2.size());
#endif
  VNL::Vector<T> result(v1.size());
  for (unsigned i = 0; i < v1.size(); i++)
    result[i] = v1[i] / v2[i];
  return result;
}

/**
*/
template <class T>
VNL::Vector<T> VNL::Vector<T>::Apply(T (*f)(T const&)) const {
  VNL::Vector<T> ret(size());
  VNL::CVector<T>::Apply(this->data, num_elmts, f, ret.data);
  return ret;
}

/** Return the vector made by applying "f" to each element.
*/
template <class T>
VNL::Vector<T> VNL::Vector<T>::Apply(T (*f)(T)) const {
  VNL::Vector<T> ret(num_elmts);
  VNL::CVector<T>::Apply(this->data, num_elmts, f, ret.data);
  return ret;
}

/** Returns the dot product of two nd-vectors, or [v1]*[v2]^T.\ O(n).
*/

template<class T>
T VNL::DotProduct (VNL::Vector<T> const& v1, VNL::Vector<T> const& v2) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (v1.size() != v2.size())
    ErrorVectorDimension ("dot_product",
                                v1.size(), v2.size());
#endif
  return VNL::CVector<T>::DotProduct(v1.begin(),
                                      v2.begin(),
                                      v1.size());
}

/** Hermitian inner product.\ O(n).
*/

template<class T>
T VNL::InnerProduct (VNL::Vector<T> const& v1, VNL::Vector<T> const& v2) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (v1.size() != v2.size())
    ErrorVectorDimension ("inner_product",
                                v1.size(), v2.size());
#endif
  return VNL::CVector<T>::InnerProduct(v1.begin(),
                                	        v2.begin(),
                                        	v1.size());
}

/** Returns the 'matrix element' <u|A|v> = u^t * A * v.\ O(mn).
*/

template<class T>
T VNL::Bracket(VNL::Vector<T> const &u, VNL::Matrix<T> const &A, VNL::Vector<T> const &v) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (u.size() != A.Rows())
    ErrorVectorDimension("bracket",u.size(),A.Rows());
  if (A.Columns() != v.size())
    ErrorVectorDimension("bracket",A.Columns(),v.size());
#endif
  T brak(0);
  for (unsigned i=0; i<u.size(); ++i)
    for (unsigned j=0; j<v.size(); ++j)
      brak += u[i]*A(i,j)*v[j];
  return brak;
}

/** Returns the nxn outer product of two nd-vectors, or [v1]^T*[v2].\ O(n).
*/

template<class T>
VNL::Matrix<T> VNL::OuterProduct (VNL::Vector<T> const& v1,
                             VNL::Vector<T> const& v2) {
  VNL::Matrix<T> out(v1.size(), v2.size());
  for (unsigned i = 0; i < out.Rows(); i++)             // v1.column() * v2.row()
    for (unsigned j = 0; j < out.Columns(); j++)
      out[i][j] = v1[i] * v2[j];
  return out;
}


/** Returns the cross-product of two 2d-vectors.
*/

template<class T>
T VNL::Cross2D (VNL::Vector<T> const& v1, VNL::Vector<T> const& v2) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (v1.size() < 2 || v2.size() < 2)
    ErrorVectorDimension ("cross_2d", v1.size(), v2.size());
#endif
  return v1[0] * v2[1] - v1[1] * v2[0];
}

/** Returns the 3X1 cross-product of two 3d-vectors.
*/

template<class T>
VNL::Vector<T> VNL::Cross3D (VNL::Vector<T> const& v1, VNL::Vector<T> const& v2) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (v1.size() != 3 || v2.size() != 3)
    ErrorVectorDimension ("cross_3d", v1.size(), v2.size());
#endif
  VNL::Vector<T> result(v1.size());

  result.x() = v1.y() * v2.z() - v1.z() * v2.y(); // work for both col/row
  result.y() = v1.z() * v2.x() - v1.x() * v2.z(); // representation
  result.z() = v1.x() * v2.y() - v1.y() * v2.x();
  return result;
}

//--------------------------------------------------------------------------------

template <class T>
void VNL::Vector<T>::Flip() {
  for (unsigned i=0;i<num_elmts/2;i++) {
    T tmp=data[i];
    data[i]=data[num_elmts-1-i];
    data[num_elmts-1-i]=tmp;
  }
}

template <class T>
void VNL::Vector<T>::Swap(VNL::Vector<T> &that)
{
  std::swap(this->num_elmts, that.num_elmts);
  std::swap(this->data, that.data);
}

//--------------------------------------------------------------------------------

// fsm@robots : cos_angle should return a T, or a double-precision extension
// of T. "double" is wrong since it won't work if T is complex.
template <class T>
T VNL::CosAngle(VNL::Vector<T> const& a, VNL::Vector<T> const& b) {
  typedef typename VNL::NumericTraits<T>::real_t real_t;
  typedef typename VNL::NumericTraits<T>::abs_t abs_t;
  typedef typename VNL::NumericTraits<abs_t>::real_t abs_r;

  real_t ab = VNL::InnerProduct(a,b);
  abs_r a_b = std::sqrt( abs_r(a.SquaredMagnitude() * b.SquaredMagnitude()) );
  return T( ab / (real_t)a_b);
}

/** Returns smallest angle between two non-zero n-dimensional vectors.\ O(n).
*/

template<class T>
double VNL::Angle (VNL::Vector<T> const& a, VNL::Vector<T> const& b) {
  typedef typename VNL::NumericTraits<T>::abs_t abs_t;
  typedef typename VNL::NumericTraits<abs_t>::real_t abs_r;
  const abs_r c = abs_r( VNL::CosAngle(a, b) );
  // IMS: sometimes cos_angle returns 1+eps, which can mess up acos.
  if (c >= 1.0) return 0;
  if (c <= -1.0) return VNL::Math::pi;
  return acos( c );
}

template <class T>
bool VNL::Vector<T>::IsFinite() const {
  for(unsigned i = 0; i < this->size();++i)
    if (!VNL::IsFinite( (*this)[i] ))
      return false;

  return true;
}

template <class T>
bool VNL::Vector<T>::IsZero() const
{
  T const zero(0);
  for(unsigned i = 0; i < this->size();++i)
    if ( !( (*this)[i] == zero) )
      return false;

  return true;
}

template <class T>
void VNL::Vector<T>::assert_finite_internal() const {
  if (this->IsFinite())
    return;

  std::cerr << __FILE__ ": *** NAN FEVER **\n";
  std::cerr << *this;
  abort();
}

template <class T>
void VNL::Vector<T>::assert_size_internal(unsigned sz) const {
  if (this->size() != sz) {
    std::cerr << __FILE__ ": Size is " << this->size() << ". Should be " << sz << std::endl;
    abort();
  }
}

template<class T>
bool VNL::Vector<T>::operator_eq (VNL::Vector<T> const& rhs) const {
  if (this == &rhs)                               // same object => equal.
    return true;

  if (this->size() != rhs.size())                 // Size different ?
    return false;                                 // Then not equal.
  for (unsigned i = 0; i < size(); i++)           // For each index
    if (!(this->data[i] == rhs.data[i]))          // Element different ?
      return false;                               // Then not equal.

  return true;                                    // Else same; return true.
}

//--------------------------------------------------------------------------------

/** Overloads the output operator to print a vector.\ O(n).
*/

template<class T>
std::ostream& VNL::operator<< (std::ostream& s, VNL::Vector<T> const& v) {
  for (unsigned i = 0; i+1 < v.size(); ++i)   // For each index in vector
    s << v[i] << " ";                              // Output data element
  if (v.size() > 0)  s << v[v.size()-1];
  return s;
}

/** Read a VNL::Vector from an ascii std::istream.
* If the vector has nonzero size on input, read that many values.
* Otherwise, read to EOF.
*/
template <class T>
std::istream& VNL::operator>>(std::istream& s, VNL::Vector<T>& M) {
  M.ReadASCII(s); return s;
}

template <class T>
void VNL::Vector<T>::inline_function_tickler()
{
  VNL::Vector<T> v;
  // fsm: hacks to get 2.96/2.97/3.0 to instantiate the inline functions.
  v = T(3) + v;
  v = T(3) - v;
  v = T(3) * v;
}


//--------------------------------------------------------------------------------

// The instantiation macros are split because some functions
// (vnl_angle) shouldn't be instantiated for complex types.

#define VNL_VECTOR_INSTANTIATE_COMMON(T) \
namespace VNL { \
  template class Vector<T >; \
/* arithmetic, comparison etc */ \
  template VNL::Vector<T > operator+(T const, Vector<T > const &); \
  template Vector<T > operator-(T const, Vector<T > const &); \
  template Vector<T > operator*(T const, Vector<T > const &); \
  template Vector<T > operator*(Matrix<T > const &, Vector<T > const &); \
/* element-wise */ \
  template Vector<T > ElementProduct(Vector<T > const &, Vector<T > const &); \
  template Vector<T > ElementQuotient(Vector<T > const &, Vector<T > const &); \
/* dot products, angles etc */ \
  template T InnerProduct(Vector<T > const &, Vector<T > const &); \
  template T DotProduct(Vector<T > const &, Vector<T > const &); \
  template T CosAngle(Vector<T > const & , Vector<T > const &); \
  template T Bracket(Vector<T > const &, Matrix<T > const &, Vector<T > const &); \
  template Matrix<T > OuterProduct(Vector<T > const &,Vector<T > const &); \
/* cross products */ \
  template T Cross2D(Vector<T > const &, Vector<T > const &); \
  template Vector<T > Cross3D(Vector<T > const &, Vector<T > const &); \
/* I/O */ \
  template std::ostream & operator<<(std::ostream &, Vector<T > const &); \
  template std::istream & operator>>(std::istream &, Vector<T >       &); \
};

#define VNL_VECTOR_INSTANTIATE(T) \
VNL_VECTOR_INSTANTIATE_COMMON(T); \
namespace VNL { template double Angle(Vector<T > const & , Vector<T > const &); };

#define VNL_VECTOR_INSTANTIATE_COMPLEX(T) \
VNL_VECTOR_INSTANTIATE_COMMON(T)

#endif // VNL::Vector_t_

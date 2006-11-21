// This is vxl/VNL/matrix.t
#ifndef vnl_matrix_t_
#define vnl_matrix_t_

/**
* \file
*
* Copyright (C) 1991 Texas Instruments Incorporated.
* Copyright (C) 1992 General Electric Company.
*
* Permission is granted to any individual or institution to use, copy, modify,
* and distribute this software, provided that this complete copyright and
* permission notice is maintained, intact, in all copies and supporting
* documentation.
*
* Texas Instruments Incorporated, General Electric Company,
* provides this software "as is" without express or implied warranty.
*
* Created: MBN 04/21/89  Initial design and implementation
* Updated: MBN 06/22/89  Removed non-destructive methods
* Updated: LGO 08/09/89  Inherit from Generic
* Updated: MBN 08/20/89  Changed template usage to reflect new syntax
* Updated: MBN 09/11/89  Added conditional exception handling and base class
* Updated: LGO 10/05/89  Don't re-allocate data in operator= when same size
* Updated: LGO 10/19/89  Add extra parameter to varargs constructor
* Updated: MBN 10/19/89  Added optional argument to set_compare method
* Updated: LGO 12/08/89  Allocate column data in one chunk
* Updated: LGO 12/08/89  Clean-up get and put, add const everywhere.
* Updated: LGO 12/19/89  Remove the map and reduce methods
* Updated: MBN 02/22/90  Changed size arguments from int to unsigned int
* Updated: MJF 06/30/90  Added base class name to constructor initializer
* Updated: VDN 02/21/92  New lite version
* Updated: VDN 05/05/92  Use envelope to avoid unecessary copying
* Updated: VDN 09/30/92  Matrix inversion with singular value decomposition
* Updated: AWF 08/21/96  set_identity, normalize_rows, scale_row.
* Updated: AWF 09/30/96  set_row/set_column methods.  Const-correct DataBlock().
* Updated: AWF 14/02/97  get_n_rows, get_n_columns.
* Updated: PVR 20/03/97  get_row, get_column.
*
* The parameterized vnl_matrix<T> class implements two dimensional arithmetic
* matrices of a user specified type. The only constraint placed on the type is
* that it must overload the following operators: +, -,  *,  and /. Thus, it
* will be possible to have a vnl_matrix over std::complex<T>. The vnl_matrix<T>
* class is static in size, that is once a vnl_matrix<T> of a particular size
* has been created, there is no dynamic growth or resize method available.
*
* Each matrix contains  a protected  data section  that has a T** slot that
* points to the  physical memory allocated  for the two  dimensional array. In
* addition, two integers  specify   the number  of  rows  and columns  for the
* matrix.  These values  are provided in the  constructors. A single protected
* slot  contains a pointer  to a compare  function  to   be used  in  equality
* operations. The default function used is the built-in == operator.
*
* Four  different constructors are provided.  The  first constructor takes two
* integer arguments  specifying the  row  and column  size.   Enough memory is
* allocated to hold row*column elements  of type Type.  The second constructor
* takes the  same two  first arguments, but  also accepts  an additional third
* argument that is  a reference to  an  object of  the appropriate  type whose
* value is used as an initial fill value.  The third constructor is similar to
* the third, except that it accpets a variable number of initialization values
* for the Matrix.  If there are  fewer values than elements,  the rest are set
* to zero. Finally, the last constructor takes a single argument consisting of
* a reference to a Matrix and duplicates its size and element values.
*
* Methods   are  provided   for destructive   scalar   and Matrix    addition,
* multiplication, check for equality  and inequality, fill, reduce, and access
* and set individual elements.  Finally, both  the  input and output operators
* are overloaded to allow for fomatted input and output of matrix elements.
*
* Good matrix inversion is needed. We choose singular value decomposition,
* since it is general and works great for nearly singular cases. Singular
* value decomposition is preferred to LU decomposition, since the accuracy
* of the pivots is independent from the left->right top->down elimination.
* LU decomposition also does not give eigenvectors and eigenvalues when
* the matrix is symmetric.
*
* Several different constructors are provided. See .h file for brief descriptions.
*/

//--------------------------------------------------------------------------------

#include "matrix.h"

#include <assert.h>
#include <cstdlib>  // abort()
#include <cstdio>   // sprintf()
#include <cctype>   // isspace()
#include <cstring>  // strcpy()
#include <iostream>
#include <vector>
#include <algorithm>

#include <VNL/vnlmath.h>
#include <VNL/vector.h>
#include <VNL/cvector.h>
#include <VNL/numerictraits.h>
//--------------------------------------------------------------------------------

// This macro allocates and initializes the dynamic storage used by a vnl_matrix.
#define vnl_matrix_alloc_blah(rowz_, colz_) \
do { \
  this->num_rows = (rowz_); \
  this->num_cols = (colz_); \
  if (this->num_rows && this->num_cols) { \
    /* Allocate memory to hold the row pointers */ \
    this->data = VNL::CVector<T>::AllocateTptr(this->num_rows); \
    /* Allocate memory to hold the elements of the matrix */ \
    T* elmns = VNL::CVector<T>::AllocateT(this->num_rows * this->num_cols); \
    /* Fill in the array of row pointers */ \
    for (unsigned int i = 0; i < this->num_rows; ++ i) \
      this->data[i] = elmns + i*this->num_cols; \
  } \
  else { \
   /* This is to make sure .begin() and .end() work for 0xN matrices: */ \
   (this->data = VNL::CVector<T>::AllocateTptr(1))[0] = 0; \
  } \
} while (false)

// This macro releases the dynamic storage used by a vnl_matrix.
#define vnl_matrix_free_blah \
do { \
  if (this->data) { \
    if (this->num_cols && this->num_rows) { \
      VNL::CVector<T>::Deallocate(this->data[0], this->num_cols * this->num_rows); \
      VNL::CVector<T>::Deallocate(this->data, this->num_rows); \
    } else { \
      VNL::CVector<T>::Deallocate(this->data, 1); \
    } \
  } \
} while(false)

/** Creates a matrix with given number of rows and columns.
* Elements are not initialized. O(m*n).
*/

template<class T>
VNL::Matrix<T>::Matrix (unsigned rowz, unsigned colz)
{
  vnl_matrix_alloc_blah(rowz, colz);
}

/** Creates a matrix with given number of rows and columns, and initialize all elements to value.\ O(m*n).
*/

template<class T>
VNL::Matrix<T>::Matrix (unsigned rowz, unsigned colz, T const& value)
{
  vnl_matrix_alloc_blah(rowz, colz);
  for (unsigned int i = 0; i < rowz; ++ i)
    for (unsigned int j = 0; j < colz; ++ j)
      this->data[i][j] = value;
}

/** r rows, c cols, special type.\  Currently implements "identity".
*/
template <class T>
VNL::Matrix<T>::Matrix(unsigned r, unsigned c, VNL::MatrixType t)
{
  vnl_matrix_alloc_blah(r, c);
  if (t == MatrixIdentity) {
    assert(r == c);
    for (unsigned int i = 0; i < r; ++ i)
      for (unsigned int j = 0; j < c; ++ j)
        this->data[i][j] = (i==j) ? T(1) : T(0);
  }
}

#if 1 // fsm: who uses this?
/** Creates a matrix with given dimension (rows, cols) and initialize first n elements, row-wise, to values.\ O(m*n).
*/

template<class T>
VNL::Matrix<T>::Matrix (unsigned rowz, unsigned colz, unsigned n, T const values[])
{
  vnl_matrix_alloc_blah(rowz, colz);
  if (n > rowz*colz)
    n = rowz*colz;
  T *dst = this->data[0];
  for (unsigned k=0; k<n; ++k)
    dst[k] = values[k];
}
#endif

/** Creates a matrix from a block array of data, stored row-wise.
* O(m*n).
*/

template<class T>
VNL::Matrix<T>::Matrix (T const* datablck, unsigned rowz, unsigned colz)
{
  vnl_matrix_alloc_blah(rowz, colz);
  unsigned int n = rowz*colz;
  T *dst = this->data[0];
  for (unsigned int k=0; k<n; ++k)
    dst[k] = datablck[k];
}


/** Creates a new matrix and copies all the elements.
* O(m*n).
*/

template<class T>
VNL::Matrix<T>::Matrix (Matrix<T> const& from)
{
  if (from.data) {
    vnl_matrix_alloc_blah(from.num_rows, from.num_cols);
    unsigned int n = this->num_rows * this->num_cols;
    T *dst = this->data[0];
    T const *src = from.data[0];
    for (unsigned int k=0; k<n; ++k)
      dst[k] = src[k];
  }
  else {
    num_rows = 0;
    num_cols = 0;
    data = 0;
  }
}

//------------------------------------------------------------

template<class T>
VNL::Matrix<T>::Matrix (Matrix<T> const &A, Matrix<T> const &B, VNL::TagAdd)
{
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (A.num_rows != B.num_rows || A.num_cols != B.num_cols)
    ErrorMatrixDimension ("tag_add", A.num_rows, A.num_cols, B.num_rows, B.num_cols);
#endif

  vnl_matrix_alloc_blah(A.num_rows, A.num_cols);

  unsigned int n = A.num_rows * A.num_cols;
  T const *a = A.data[0];
  T const *b = B.data[0];
  T *dst = this->data[0];

  for (unsigned int i=0; i<n; ++i)
    dst[i] = a[i] + b[i];
}

template<class T>
VNL::Matrix<T>::Matrix (Matrix<T> const &A, Matrix<T> const &B, VNL::TagSub)
{
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (A.num_rows != B.num_rows || A.num_cols != B.num_cols)
    ErrorMatrixDimension ("tag_sub", A.num_rows, A.num_cols, B.num_rows, B.num_cols);
#endif

  vnl_matrix_alloc_blah(A.num_rows, A.num_cols);

  unsigned int n = A.num_rows * A.num_cols;
  T const *a = A.data[0];
  T const *b = B.data[0];
  T *dst = this->data[0];

  for (unsigned int i=0; i<n; ++i)
    dst[i] = a[i] - b[i];
}

template<class T>
VNL::Matrix<T>::Matrix (Matrix<T> const &M, T s, VNL::TagMul)
{
  vnl_matrix_alloc_blah(M.num_rows, M.num_cols);

  unsigned int n = M.num_rows * M.num_cols;
  T const *m = M.data[0];
  T *dst = this->data[0];

  for (unsigned int i=0; i<n; ++i)
    dst[i] = m[i] * s;
}

template<class T>
VNL::Matrix<T>::Matrix (Matrix<T> const &M, T s, VNL::TagDiv)
{
  vnl_matrix_alloc_blah(M.num_rows, M.num_cols);

  unsigned int n = M.num_rows * M.num_cols;
  T const *m = M.data[0];
  T *dst = this->data[0];

  for (unsigned int i=0; i<n; ++i)
    dst[i] = m[i] / s;
}

template<class T>
VNL::Matrix<T>::Matrix (Matrix<T> const &M, T s, VNL::TagAdd)
{
  vnl_matrix_alloc_blah(M.num_rows, M.num_cols);

  unsigned int n = M.num_rows * M.num_cols;
  T const *m = M.data[0];
  T *dst = this->data[0];

  for (unsigned int i=0; i<n; ++i)
    dst[i] = m[i] + s;
}

template<class T>
VNL::Matrix<T>::Matrix (Matrix<T> const &M, T s, VNL::TagSub)
{
  vnl_matrix_alloc_blah(M.num_rows, M.num_cols);

  unsigned int n = M.num_rows * M.num_cols;
  T const *m = M.data[0];
  T *dst = this->data[0];

  for (unsigned int i=0; i<n; ++i)
    dst[i] = m[i] - s;
}

template<class T>
VNL::Matrix<T>::Matrix (Matrix<T> const &A, Matrix<T> const &B, VNL::TagMul)
{
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (A.num_cols != B.num_rows)
    ErrorMatrixDimension("tag_mul", A.num_rows, A.num_cols, B.num_rows, B.num_cols);
#endif

  unsigned int l = A.num_rows;
  unsigned int m = A.num_cols; // == B.num_rows
  unsigned int n = B.num_cols;

  vnl_matrix_alloc_blah(l, n);

  for (unsigned int i=0; i<l; ++i) {
    for (unsigned int k=0; k<n; ++k) {
      T sum(0);
      for (unsigned int j=0; j<m; ++j)
        sum += A.data[i][j] * B.data[j][k];
      this->data[i][k] = sum;
    }
  }
}

//------------------------------------------------------------

/** Frees up the dynamic storage used by matrix.
* O(m*n).
*/

template<class T>
void VNL::Matrix<T>::destroy()
{
  vnl_matrix_free_blah;
}

template<class T>
void VNL::Matrix<T>::Clear()
{
  if (data) {
    destroy();
    num_rows = 0;
    num_cols = 0;
    data = 0;
  }
}

// resize // Resizes the data arrays of THIS matrix to (rows x cols). O(m*n).
// Elements are not initialized. Returns true if size is changed.

template<class T>
bool VNL::Matrix<T>::Resize (unsigned rowz, unsigned colz)
{
  if (this->data) {
    // if no change in size, do not reallocate.
    if (this->num_rows == rowz && this->num_cols == colz)
      return false;

    // else, simply release old storage and allocate new.
    vnl_matrix_free_blah;
    vnl_matrix_alloc_blah(rowz, colz);
  }
  else {
    // This happens if the matrix is default constructed.
    vnl_matrix_alloc_blah(rowz, colz);
  }

  return true;
}

#undef vnl_matrix_alloc_blah
#undef vnl_matrix_free_blah

//------------------------------------------------------------

/** Sets all elements of matrix to specified value.\ O(m*n).
*/

template<class T>
void VNL::Matrix<T>::Fill (T const& value) {
  for (unsigned int i = 0; i < this->num_rows; i++)
    for (unsigned int j = 0; j < this->num_cols; j++)
      this->data[i][j] = value;
}

/** Sets all diagonal elements of matrix to specified value.\ O(n).
*/

template<class T>
void VNL::Matrix<T>::FillDiagonal (T const& value) {
  for (unsigned int i = 0; i < this->num_rows && i < this->num_cols; i++)
    this->data[i][i] = value;
}

/** Assigns value to all elements of a matrix.\ O(m*n).
*/

//template<class T>
//Matrix<T>& Matrix<T>::operator= (T const& value) {
//  for (unsigned i = 0; i < this->num_rows; i++)    // For each row in Matrix
//    for (unsigned j = 0; j < this->num_cols; j++)  // For each column in Matrix
//      this->data[i][j] = value;                 // Assign value
//  return *this;                                 // Return Matrix reference
//}


/** Copies all elements of rhs matrix into lhs matrix.\ O(m*n).
* If needed, the arrays in lhs matrix are freed up, and new arrays are
* allocated to match the dimensions of the rhs matrix.
*/

template<class T>
VNL::Matrix<T>& VNL::Matrix<T>::operator= (Matrix<T> const& rhs) {
  if (this != &rhs) { // make sure *this != m
    if (rhs.data) {
      this->Resize(rhs.num_rows, rhs.num_cols);
      for (unsigned int i = 0; i < this->num_rows; i++)
        for (unsigned int j = 0; j < this->num_cols; j++)
          this->data[i][j] = rhs.data[i][j];
    }
    else {
      // rhs is default-constructed.
      Clear();
    }
  }
  return *this;
}

template<class T>
void VNL::Matrix<T>::print(std::ostream& os) const {
  for (unsigned int i = 0; i < this->Rows(); i++) {
    for (unsigned int j = 0; j < this->Columns(); j++)
      os << this->data[i][j] << " ";
    os << "\n";
  }
}

/** Prints the 2D array of elements of a matrix out to a stream.
* O(m*n).
*/
template<class T>
std::ostream& VNL::operator<< (std::ostream& os, VNL::Matrix<T> const& m) {
  for (unsigned int i = 0; i < m.Rows(); ++i) {
    for (unsigned int j = 0; j < m.Columns(); ++j)
      os << m(i, j) << " ";
    os << std::endl;
  }
  return os;
}

/** Read an Matrix from an ascii std::istream.
* Automatically determines file size if the input matrix has zero size.
*/
template<class T>
std::istream& VNL::operator>> (std::istream& s, VNL::Matrix<T>& M) {
  M.ReadASCII(s);
  return s;
}


template<class T>
void VNL::Matrix<T>::inline_function_tickler()
{
  Matrix<T> M;
  // fsm: hack to get 2.96 to instantiate the inline function.
  M = T(1) + T(3) * M;
}

template<class T>
VNL::Matrix<T>& VNL::Matrix<T>::operator+= (T value) {
  for (unsigned int i = 0; i < this->num_rows; i++)
    for (unsigned int j = 0; j < this->num_cols; j++)
      this->data[i][j] += value;
  return *this;
}

template<class T>
VNL::Matrix<T>& VNL::Matrix<T>::operator-= (T value) {
  for (unsigned int i = 0; i < this->num_rows; i++)
    for (unsigned int j = 0; j < this->num_cols; j++)
      this->data[i][j] -= value;
  return *this;
}

template<class T>
VNL::Matrix<T>& VNL::Matrix<T>::operator*= (T value) {
  for (unsigned int i = 0; i < this->num_rows; i++)
    for (unsigned int j = 0; j < this->num_cols; j++)
      this->data[i][j] *= value;
  return *this;
}

template<class T>
VNL::Matrix<T>& VNL::Matrix<T>::operator/= (T value) {
  for (unsigned int i = 0; i < this->num_rows; i++)
    for (unsigned int j = 0; j < this->num_cols; j++)
      this->data[i][j] /= value;
  return *this;
}


/** Adds lhs matrix with rhs matrix, and stores in place in lhs matrix.
* O(m*n). The dimensions of the two matrices must be identical.
*/

template<class T>
VNL::Matrix<T>& VNL::Matrix<T>::operator+= (Matrix<T> const& rhs) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (this->num_rows != rhs.num_rows ||
      this->num_cols != rhs.num_cols)           // Size match?
    ErrorMatrixDimension ("operator+=",
                                this->num_rows, this->num_cols,
                                rhs.num_rows, rhs.num_cols);
#endif
  for (unsigned int i = 0; i < this->num_rows; i++)    // For each row
    for (unsigned int j = 0; j < this->num_cols; j++)  // For each element in column
      this->data[i][j] += rhs.data[i][j];       // Add elements
  return *this;
}


/** Substract lhs matrix with rhs matrix and store in place in lhs matrix.
* O(m*n).
* The dimensions of the two matrices must be identical.
*/

template<class T>
VNL::Matrix<T>& VNL::Matrix<T>::operator-= (Matrix<T> const& rhs) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (this->num_rows != rhs.num_rows ||
      this->num_cols != rhs.num_cols) // Size?
    ErrorMatrixDimension ("operator-=",
                                this->num_rows, this->num_cols,
                                rhs.num_rows, rhs.num_cols);
#endif
  for (unsigned int i = 0; i < this->num_rows; i++)
    for (unsigned int j = 0; j < this->num_cols; j++)
      this->data[i][j] -= rhs.data[i][j];
  return *this;
}


template<class T>
VNL::Matrix<T> VNL::operator- (T const& value, VNL::Matrix<T> const& m) {
  VNL::Matrix<T> result(m.Rows(),m.Columns());
  for (unsigned int i = 0; i < m.Rows(); i++)  // For each row
    for (unsigned int j = 0; j < m.Columns(); j++) // For each elmt. in column
      result.Put(i,j, value - m.Get(i,j) );     // subtract from value elmt.
  return result;
}


#if 0 // commented out
/** Returns new matrix which is the product of m1 with m2, m1 * m2.
* O(n^3). Number of columns of first matrix must match number of rows
* of second matrix.
*/

template<class T>
VNL::Matrix<T> VNL::Matrix<T>::operator* (VNL::Matrix<T> const& rhs) const {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (this->num_cols != rhs.num_rows)           // dimensions do not match?
    ErrorMatrixDimension("operator*",
                               this->num_rows, this->num_cols,
                               rhs.num_rows, rhs.num_cols);
#endif
  Matrix<T> result(this->num_rows, rhs.num_cols); // Temp to store product
  for (unsigned i = 0; i < this->num_rows; i++) {  // For each row
    for (unsigned j = 0; j < rhs.num_cols; j++) {  // For each element in column
      T sum = 0;
      for (unsigned k = 0; k < this->num_cols; k++) // Loop over column values
        sum += (this->data[i][k] * rhs.data[k][j]);     // Multiply
      result(i,j) = sum;
    }
  }
  return result;
}
#endif

/** Returns new matrix which is the negation of THIS matrix.
* O(m*n).
*/

template<class T>
VNL::Matrix<T> VNL::Matrix<T>::operator- () const {
  Matrix<T> result(this->num_rows, this->num_cols);
  for (unsigned int i = 0; i < this->num_rows; i++)
    for (unsigned int j = 0; j < this->num_cols; j++)
      result.data[i][j] = - this->data[i][j];
  return result;
}

#if 0 // commented out
/** Returns new matrix with elements of lhs matrix added with value.
* O(m*n).
*/

template<class T>
VNL::Matrix<T> VNL::Matrix<T>::operator+ (T const& value) const {
  Matrix<T> result(this->num_rows, this->num_cols);
  for (unsigned i = 0; i < this->num_rows; i++)    // For each row
    for (unsigned j = 0; j < this->num_cols; j++)  // For each element in column
      result.data[i][j] = (this->data[i][j] + value);   // Add scalar
  return result;
}


/** Returns new matrix with elements of lhs matrix multiplied with value.
* O(m*n).
*/

template<class T>
VNL::Matrix<T> VNL::Matrix<T>::operator* (T const& value) const {
  Matrix<T> result(this->num_rows, this->num_cols);
  for (unsigned i = 0; i < this->num_rows; i++)    // For each row
    for (unsigned j = 0; j < this->num_cols; j++)  // For each element in column
      result.data[i][j] = (this->data[i][j] * value);   // Multiply
  return result;
}


/** Returns new matrix with elements of lhs matrix divided by value.\ O(m*n).
*/
template<class T>
VNL::Matrix<T> VNL::Matrix<T>::operator/ (T const& value) const {
  Matrix<T> result(this->num_rows, this->num_cols);
  for (unsigned i = 0; i < this->num_rows; i++)    // For each row
    for (unsigned j = 0; j < this->num_cols; j++)  // For each element in column
      result.data[i][j] = (this->data[i][j] / value);   // Divide
  return result;
}
#endif

/** Return the matrix made by applying "f" to each element.
*/
template <class T>
VNL::Matrix<T> VNL::Matrix<T>::Apply(T (*f)(T const&)) const {
  Matrix<T> ret(num_rows, num_cols);
  VNL::CVector<T>::Apply(this->data[0], num_rows * num_cols, f, ret.DataBlock());
  return ret;
}

/** Return the matrix made by applying "f" to each element.
*/
template <class T>
VNL::Matrix<T> VNL::Matrix<T>::Apply(T (*f)(T)) const {
  Matrix<T> ret(num_rows, num_cols);
  VNL::CVector<T>::Apply(this->data[0], num_rows * num_cols, f, ret.DataBlock());
  return ret;
}

////--------------------------- Additions------------------------------------

/** Returns new matrix with rows and columns transposed.
* O(m*n).
*/

template<class T>
VNL::Matrix<T> VNL::Matrix<T>::Transpose() const {
  Matrix<T> result(this->num_cols, this->num_rows);
  for (unsigned int i = 0; i < this->num_cols; i++)
    for (unsigned int j = 0; j < this->num_rows; j++)
      result.data[i][j] = this->data[j][i];
  return result;
}

// adjoint/hermitian transpose

template<class T>
VNL::Matrix<T> VNL::Matrix<T>::ConjugateTranspose() const  {
  Matrix<T> result(Transpose());
  VNL::CVector<T>::Conjugate(result.begin(),  // src
                             result.begin(),  // dst
                             result.size());  // size of block
  return result;
}

/** Replaces the submatrix of THIS matrix, starting at top left corner, by the elements of matrix m.\ O(m*n).
* This is the reverse of extract().
*/

template<class T>
VNL::Matrix<T>& VNL::Matrix<T>::Update (Matrix<T> const& m,
                                      unsigned top, unsigned left) {
  unsigned int bottom = top + m.num_rows;
  unsigned int right = left + m.num_cols;
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (this->num_rows < bottom || this->num_cols < right)
    ErrorMatrixDimension ("update",
                                bottom, right, m.num_rows, m.num_cols);
#endif
  for (unsigned int i = top; i < bottom; i++)
    for (unsigned int j = left; j < right; j++)
      this->data[i][j] = m.data[i-top][j-left];
  return *this;
}


/** Returns a copy of submatrix of THIS matrix, specified by the top-left corner and size in rows, cols.\ O(m*n).
* Use update() to copy new values of this submatrix back into THIS matrix.
*/

template<class T>
VNL::Matrix<T> VNL::Matrix<T>::Extract (unsigned rowz, unsigned colz,
                                      unsigned top, unsigned left) const{
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  unsigned int bottom = top + rowz;
  unsigned int right = left + colz;
  if ((this->num_rows < bottom) || (this->num_cols < right))
    ErrorMatrixDimension ("extract",
                                this->num_rows, this->num_cols, bottom, right);
#endif
  Matrix<T> result(rowz, colz);
  for (unsigned int i = 0; i < rowz; i++)      // actual copy of all elements
    for (unsigned int j = 0; j < colz; j++)    // in submatrix
      result.data[i][j] = data[top+i][left+j];
  return result;
}

/** Returns the dot product of the two matrices.\ O(m*n).
* This is the sum of all pairwise products of the elements m1[i,j]*m2[i,j].
*/

template<class T>
T VNL::DotProduct (VNL::Matrix<T> const& m1, VNL::Matrix<T> const& m2) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (m1.Rows() != m2.Rows() || m1.Columns() != m2.Columns()) // Size?
    ErrorMatrixDimension ("dot_product",
                                m1.Rows(), m1.Columns(),
                                m2.Rows(), m2.Columns());
#endif
  return VNL::CVector<T>::DotProduct(m1.begin(), m2.begin(), m1.Rows()*m1.Columns());
}

/** Hermitian inner product.
* O(mn).
*/

template<class T>
T VNL::InnerProduct (VNL::Matrix<T> const& m1, VNL::Matrix<T> const& m2) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (m1.Rows() != m2.Rows() || m1.Columns() != m2.Columns()) // Size?
    ErrorMatrixDimension ("inner_product",
                                m1.Rows(), m1.Columns(),
                                m2.Rows(), m2.Columns());
#endif
  return VNL::CVector<T>::InnerProduct(m1.begin(), m2.begin(), m1.Rows()*m1.Columns());
}

// cos_angle. O(mn).

template<class T>
T VNL::CosAngle (VNL::Matrix<T> const& a, VNL::Matrix<T> const& b) {

using namespace std;
  typedef typename VNL::NumericTraits<T>::real_t real_t;
  typedef typename VNL::NumericTraits<T>::abs_t abs_t;
  typedef typename VNL::NumericTraits<abs_t>::real_t abs_r;

  T ab = InnerProduct(a,b);
  abs_t a_b = (abs_t)sqrt( (abs_r)VNL::Abs(InnerProduct(a,a) * InnerProduct(b,b)) );

  return T( ab / a_b);
}

/** Returns new matrix whose elements are the products m1[ij]*m2[ij].
* O(m*n).
*/

template<class T>
VNL::Matrix<T> VNL::ElementProduct (VNL::Matrix<T> const& m1,
                               VNL::Matrix<T> const& m2) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (m1.Rows() != m2.Rows() || m1.Columns() != m2.Columns()) // Size?
    ErrorMatrixDimension ("element_product",
                                m1.Rows(), m1.Columns(), m2.Rows(), m2.Columns());
#endif
  VNL::Matrix<T> result(m1.Rows(), m1.Columns());
  for (unsigned int i = 0; i < m1.Rows(); i++)
    for (unsigned int j = 0; j < m1.Columns(); j++)
      result.Put(i,j, m1.Get(i,j) * m2.Get(i,j) );
  return result;
}

/** Returns new matrix whose elements are the quotients m1[ij]/m2[ij].
* O(m*n).
*/

template<class T>
VNL::Matrix<T> VNL::ElementQuotient (VNL::Matrix<T> const& m1,
                                VNL::Matrix<T> const& m2) {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (m1.Rows() != m2.Rows() || m1.Columns() != m2.Columns()) // Size?
    ErrorMatrixDimension("element_quotient",
                               m1.Rows(), m1.Columns(), m2.Rows(), m2.Columns());
#endif
  VNL::Matrix<T> result(m1.Rows(), m1.Columns());
  for (unsigned int i = 0; i < m1.Rows(); i++)
    for (unsigned int j = 0; j < m1.Columns(); j++)
      result.Put(i,j, m1.Get(i,j) / m2.Get(i,j) );
  return result;
}

/** Fill this matrix with the given data.
*  We assume that p points to a contiguous rows*cols array, stored rowwise.
*/
template<class T>
void VNL::Matrix<T>::CopyIn(T const *p)
{
  T* dp = this->data[0];
  unsigned int n = this->num_rows * this->num_cols;
  while (n--)
    *dp++ = *p++;
}

/** Fill the given array with this matrix.
*  We assume that p points to a contiguous rows*cols array, stored rowwise.
*/
template<class T>
void VNL::Matrix<T>::CopyOut(T *p) const
{
  T* dp = this->data[0];
  unsigned int n = this->num_rows * this->num_cols;
  while (n--)
    *p++ = *dp++;
}

/** Fill this matrix with a row*row identity matrix.
*/
template<class T>
void VNL::Matrix<T>::SetIdentity()
{
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (this->num_rows != this->num_cols) // Size?
    ErrorMatrixNonSquare ("set_identity");
#endif
  for (unsigned int i = 0; i < this->num_rows; i++)    // For each row in the Matrix
    for (unsigned int j = 0; j < this->num_cols; j++)  // For each element in column
      if (i == j)
        this->data[i][j] = T(1);
      else
        this->data[i][j] = T(0);
}

/** Make each row of the matrix have unit norm.
* All-zero rows are ignored.
*/
template<class T>
void VNL::Matrix<T>::NormalizeRows()
{
  using namespace std;

  typedef typename VNL::NumericTraits<T>::abs_t abs_t;
  for (unsigned int i = 0; i < this->num_rows; i++) {  // For each row in the Matrix
    abs_t norm(0); // double will not do for all types.
    for (unsigned int j = 0; j < this->num_cols; j++)  // For each element in row
      norm += VNL::SquaredMagnitude(this->data[i][j]);

    if (norm != 0) {
      typedef typename VNL::NumericTraits<abs_t>::real_t real_t;
      real_t scale = real_t(1)/sqrt((real_t)norm);
      for (unsigned int j = 0; j < this->num_cols; j++) {
        // FIXME need correct rounding here
        // There is e.g. no *standard* operator*=(complex<float>, double), hence the T() cast.
        this->data[i][j] *= T(scale);
      }
    }
  }
}

/** Make each column of the matrix have unit norm.
* All-zero columns are ignored.
*/
template<class T>
void VNL::Matrix<T>::NormalizeColumns()
{
	using namespace std;

  typedef typename VNL::NumericTraits<T>::abs_t abs_t;
  for (unsigned int j = 0; j < this->num_cols; j++) {  // For each column in the Matrix
    abs_t norm(0); // double will not do for all types.
    for (unsigned int i = 0; i < this->num_rows; i++)
      norm += VNL::SquaredMagnitude(this->data[i][j]);

    if (norm != 0) {
      typedef typename VNL::NumericTraits<abs_t>::real_t real_t;
      real_t scale = real_t(1)/sqrt((real_t)norm);
      for (unsigned int i = 0; i < this->num_rows; i++) {
        // FIXME need correct rounding here
        // There is e.g. no *standard* operator*=(complex<float>, double), hence the T() cast.
        this->data[i][j] *= T(scale);
      }
    }
  }
}

/** Multiply row[row_index] by value.
*/
template<class T>
void VNL::Matrix<T>::ScaleRow(unsigned row_index, T value)
{
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (row_index >= this->num_rows)
    ErrorMatrixRowIndex("scale_row", row_index);
#endif
  for (unsigned int j = 0; j < this->num_cols; j++)    // For each element in row
    this->data[row_index][j] *= value;
}

/** Multiply column[column_index] by value.
*/
template<class T>
void VNL::Matrix<T>::ScaleColumn(unsigned column_index, T value)
{
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (column_index >= this->num_cols)
    ErrorMatrixColIndex("scale_column", column_index);
#endif
  for (unsigned int j = 0; j < this->num_rows; j++)    // For each element in column
    this->data[j][column_index] *= value;
}

/** Returns a copy of n rows, starting from "row".
*/
template<class T>
VNL::Matrix<T> VNL::Matrix<T>::GetNRows (unsigned row, unsigned n) const {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (row + n > this->num_rows)
    ErrorMatrixRowIndex ("get_n_rows", row);
#endif

  // Extract data rowwise.
  return Matrix<T>(data[row], n, this->num_cols);
}

/** Returns a copy of n columns, starting from "column".
*/
template<class T>
VNL::Matrix<T> VNL::Matrix<T>::GetNColumns (unsigned column, unsigned n) const {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (column + n > this->num_cols)
    ErrorMatrixColIndex ("get_n_columns", column);
#endif

  Matrix<T> result(this->num_rows, n);
  for (unsigned int c = 0; c < n; ++c)
    for (unsigned int r = 0; r < this->num_rows; ++r)
      result(r, c) = data[r][column + c];
  return result;
}

/** Create a vector out of row[row_index].
*/
template<class T>
VNL::Vector<T> VNL::Matrix<T>::GetRow(unsigned row_index) const
{
#if ERROR_CHECKING
  if (row_index >= this->num_rows)
    ErrorMatrixRowIndex ("get_row", row_index);
#endif

  VNL::Vector<T> v(this->num_cols);
  for (unsigned int j = 0; j < this->num_cols; j++)    // For each element in row
    v[j] = this->data[row_index][j];
  return v;
}

/** Create a vector out of column[column_index].
*/
template<class T>
VNL::Vector<T> VNL::Matrix<T>::GetColumn(unsigned column_index) const
{
#if ERROR_CHECKING
  if (column_index >= this->num_cols)
    ErrorMatrixColIndex ("get_column", column_index);
#endif

  VNL::Vector<T> v(this->num_rows);
  for (unsigned int j = 0; j < this->num_rows; j++)    // For each element in row
    v[j] = this->data[j][column_index];
  return v;
}

//--------------------------------------------------------------------------------

/** Set row[row_index] to data at given address.\ No bounds check.
*/
template<class T>
void VNL::Matrix<T>::SetRow(unsigned row_index, T const *v)
{
  for (unsigned int j = 0; j < this->num_cols; j++)    // For each element in row
    this->data[row_index][j] = v[j];
}

/** Set row[row_index] to given vector.\ No bounds check.
*/
template<class T>
void VNL::Matrix<T>::SetRow(unsigned row_index, VNL::Vector<T> const &v)
{
  SetRow(row_index,v.DataBlock());
}

/** Set row[row_index] to given value.
*/
template<class T>
void VNL::Matrix<T>::SetRow(unsigned row_index, T v)
{
  for (unsigned int j = 0; j < this->num_cols; j++)    // For each element in row
    this->data[row_index][j] = v;
}

//--------------------------------------------------------------------------------

/** Set column[column_index] to data at given address.
*/
template<class T>
void VNL::Matrix<T>::SetColumn(unsigned column_index, T const *v)
{
  for (unsigned int i = 0; i < this->num_rows; i++)    // For each element in row
    this->data[i][column_index] = v[i];
}

/** Set column[column_index] to given vector.\ No bounds check.
*/
template<class T>
void VNL::Matrix<T>::SetColumn(unsigned column_index, VNL::Vector<T> const &v)
{
  SetColumn(column_index,v.DataBlock());
}

/** Set column[column_index] to given value.
*/
template<class T>
void VNL::Matrix<T>::SetColumn(unsigned column_index, T v)
{
  for (unsigned int j = 0; j < this->num_rows; j++)    // For each element in row
    this->data[j][column_index] = v;
}


/** Set columns starting at starting_column to given matrix.
*/
template<class T>
void VNL::Matrix<T>::SetColumns(unsigned starting_column, Matrix<T> const& m)
{
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
  if (this->num_rows != m.num_rows ||
      this->num_cols < m.num_cols + starting_column)           // Size match?
    ErrorMatrixDimension ("set_columns",
                                this->num_rows, this->num_cols,
                                m.num_rows, m.num_cols);
#endif

  for (unsigned int j = 0; j < m.num_cols; ++j)
    for (unsigned int i = 0; i < this->num_rows; i++)    // For each element in row
      this->data[i][starting_column + j] = m.data[i][j];
}

//--------------------------------------------------------------------------------

/** Two matrices are equal if and only if they have the same dimensions and the same values.
* O(m*n).
* Elements are compared with operator== as default.
* Change this default with set_compare() at run time or by specializing
* Matrix_compare at compile time.
*/

template<class T>
bool VNL::Matrix<T>::operator_eq(VNL::Matrix<T> const& rhs) const {
  if (this == &rhs)                                      // same object => equal.
    return true;

  if (this->num_rows != rhs.num_rows || this->num_cols != rhs.num_cols)
    return false;                                        // different sizes => not equal.

  for (unsigned int i = 0; i < this->num_rows; i++)     // For each row
    for (unsigned int j = 0; j < this->num_cols; j++)   // For each columne
      if (!(this->data[i][j] == rhs.data[i][j]))            // different element ?
        return false;                                    // Then not equal.

  return true;                                           // Else same; return true
}

template <class T>
bool VNL::Matrix<T>::IsIdentity() const
{
  T const zero(0);
  T const one(1);
  for (unsigned int i = 0; i < this->Rows(); ++i)
    for (unsigned int j = 0; j < this->Columns(); ++j) {
      T xm = (*this)(i,j);
      if ( !((i == j) ? (xm == one) : (xm == zero)) )
        return false;
    }
  return true;
}

/** Return true if maximum absolute deviation of M from identity is <= tol.
*/
template <class T>
bool VNL::Matrix<T>::IsIdentity(double tol) const
{
  T one(1);
  for (unsigned int i = 0; i < this->Rows(); ++i)
    for (unsigned int j = 0; j < this->Columns(); ++j) {
      T xm = (*this)(i,j);
      abs_t absdev = (i == j) ? VNL::Abs(xm - one) : VNL::Abs(xm);
      if (absdev > tol)
        return false;
    }
  return true;
}

template <class T>
bool VNL::Matrix<T>::IsZero() const
{
  T const zero(0);
  for (unsigned int i = 0; i < this->Rows(); ++i)
    for (unsigned int j = 0; j < this->Columns(); ++j)
      if ( !( (*this)(i, j) == zero) )
        return false;

  return true;
}

/** Return true if max(abs((*this))) <= tol.
*/
template <class T>
bool VNL::Matrix<T>::IsZero(double tol) const
{
  for (unsigned int i = 0; i < this->Rows(); ++i)
    for (unsigned int j = 0; j < this->Columns(); ++j)
      if (VNL::Abs((*this)(i,j)) > tol)
        return false;

  return true;
}

/** Return true if any element of (*this) is nan.
*/
template <class T>
bool VNL::Matrix<T>::HasNaNs() const
{
  for (unsigned int i = 0; i < this->Rows(); ++i)
    for (unsigned int j = 0; j < this->Columns(); ++j)
      if (VNL::IsNaN((*this)(i,j)))
        return true;

  return false;
}

/** Return false if any element of (*this) is inf or nan.
*/
template <class T>
bool VNL::Matrix<T>::IsFinite() const
{
  for (unsigned int i = 0; i < this->Rows(); ++i)
    for (unsigned int j = 0; j < this->Columns(); ++j)
      if (!VNL::IsFinite((*this)(i,j)))
        return false;

  return true;
}

/** Abort if any element of M is inf or nan.
*/
template <class T>
void VNL::Matrix<T>::assert_finite_internal() const
{
  if (IsFinite())
    return;

  std::cerr << std::endl << std::endl;
  std::cerr << __FILE__ ":" << __LINE__ << ": matrix has non-finite elements" << std::endl;

  if (Rows() <= 20 && Columns() <= 20) {
    std::cerr << __FILE__ ": here it is:\n" << *this;
  }
  else {
    std::cerr << __FILE__ ": it is quite big (" << Rows() << 'x' << Columns() << ")" << std::endl;
    std::cerr << __FILE__ ": in the following picture '-' means finite and '*' means non-finite:" << std::endl;

    for (unsigned int i=0; i<Rows(); ++i) {
      for (unsigned int j=0; j<Columns(); ++j)
        std::cerr << char(VNL::IsFinite((*this)(i, j)) ? '-' : '*');
      std::cerr << std::endl;
    }
  }
  std::cerr << __FILE__ ": calling abort()\n";
  abort();
}

/** Abort unless M has the given size.
*/
template <class T>
void VNL::Matrix<T>::assert_size_internal(unsigned rs,unsigned cs) const
{
  if (this->Rows()!=rs || this->Columns()!=cs) {
    std::cerr << __FILE__ ": size is " << this->Rows() << 'x' << this->Columns()
             << ". should be " << rs << 'x' << cs << std::endl;
    abort();
  }
}

/** Read a Matrix from an ascii std::istream.
* Automatically determines file size if the input matrix has zero size.
*/
template <class T>
bool VNL::Matrix<T>::ReadASCII(std::istream& s)
{
  if (!s.good()) {
    std::cerr << __FILE__ ": Matrix<T>::read_ascii: Called with bad stream\n";
    return false;
  }

  bool size_known = (this->Rows() != 0);

  if (size_known) {
    for (unsigned int i = 0; i < this->Rows(); ++i)
      for (unsigned int j = 0; j < this->Columns(); ++j)
        s >> this->data[i][j];

    return s.good() || s.eof();
  }

  bool debug = false;

  std::vector<T> first_row_vals;
  if (debug)
    std::cerr << __FILE__ ": Matrix<T>::read_ascii: Determining file dimensions: ";

  int c = ' ';
  for (;;) {
    // Clear whitespace, looking for a newline
    while (1) {
      c = s.get();
      if (c == EOF)
        goto loademup;
      if (!isspace(c)) {
        if (!s.putback(c).good())
          std::cerr << "matrix<T>::read_ascii: Could not push back '" << c << "'\n";

        goto readfloat;
      }
      // First newline after first number tells us the column dimension
      if (c == '\n' && first_row_vals.size() > 0) {
        goto loademup;
      }
    }
  readfloat:
    T val;
    s >> val;
    if (!s.fail())
      first_row_vals.push_back(val);
    if (s.eof())
      goto loademup;
  }
 loademup:
  unsigned int colz = first_row_vals.size();

  if (debug) std::cerr << colz << " cols, ";

  if (colz == 0)
    return false;

  // need to be careful with resizing here as will often be reading humungous files
  // So let's just build an array of row pointers
  std::vector<T*> row_vals;
  row_vals.reserve(1000);
  {
    // Copy first row.  Can't use first_row_vals, as may be a vector of bool...
    T* row = VNL::CVector<T>::AllocateT(colz);
    for (unsigned int k = 0; k < colz; ++k)
      row[k] = first_row_vals[k];
    row_vals.push_back(row);
  }

  while (1) {
    T* row = VNL::CVector<T>::AllocateT(colz);
    if (row == 0) {
      std::cerr << "matrix<T>::read_ascii: Error, Out of memory on row " << row_vals.size() << std::endl;
      return false;
    }
    s >> row[0];
    if (!s.good())
      break;
    for (unsigned int k = 1; k < colz; ++k) {
      if (s.eof()) {
        std::cerr << "matrix<T>::read_ascii: Error, EOF on row " << row_vals.size() << ", column " << k << std::endl;
        return false;
      }
      s >> row[k];
      if (s.fail()) {
        std::cerr << "matrix<T>::read_ascii: Error, row " << row_vals.size() << " failed on column " << k << std::endl;
        return false;
      }
    }
    row_vals.push_back(row);
  }

  unsigned int rowz = row_vals.size();

  if (debug)
    std::cerr << rowz << " rows.\n";

  Resize(rowz, colz);

  T* p = this->data[0];
  for (unsigned int i = 0; i < rowz; ++i) {
    for (unsigned int j = 0; j < colz; ++j)
      *p++ = row_vals[i][j];
    if (i > 0) VNL::CVector<T>::Deallocate(row_vals[i], colz);
  }

  return true;
}

/** Read a Matrix from an ascii std::istream.
* Automatically determines file size if the input matrix has zero size.
* This is a static method so you can type
* <verb>
* Matrix<float> M = Matrix<float>::read(cin);
* </verb>
* which many people prefer to the ">>" alternative.
*/
template <class T>
VNL::Matrix<T> VNL::Matrix<T>::Read(std::istream& s)
{
  Matrix<T> M;
  s >> M;
  return M;
}

template <class T>
void VNL::Matrix<T>::Swap(Matrix<T> &that)
{
  std::swap(this->num_rows, that.num_rows);
  std::swap(this->num_cols, that.num_cols);
  std::swap(this->data, that.data);
}

/** Reverse order of rows.\  Name is from Matlab, meaning "flip upside down".
*/
template <class T>
void VNL::Matrix<T>::FlipUD()
{
  unsigned int n = this->Rows();
  unsigned int colz = this->Columns();

  unsigned int m = n / 2;
  for (unsigned int r = 0; r < m; ++r) {
    unsigned int r1 = r;
    unsigned int r2 = n - 1 - r;
    for (unsigned int c = 0; c < colz; ++c) {
      T tmp = (*this)(r1, c);
      (*this)(r1, c) = (*this)(r2, c);
      (*this)(r2, c) = tmp;
    }
  }
}
/** Reverse order of columns.
*/
template <class T>
void VNL::Matrix<T>::FlipLR()
{
  unsigned int n = this->Columns();
  unsigned int rowz = this->Rows();

  unsigned int m = n / 2;
  for (unsigned int c = 0; c < m; ++c) {
    unsigned int c1 = c;
    unsigned int c2 = n - 1 - c;
    for (unsigned int r = 0; r < rowz; ++r) {
      T tmp = (*this)(r, c1);
      (*this)(r, c1) = (*this)(r, c2);
      (*this)(r, c2) = tmp;
    }
  }
}

// || M ||  = \max \sum | M   |
//        1     j    i     ij
template <class T>
typename VNL::Matrix<T>::abs_t VNL::Matrix<T>::operatorOneNorm() const
{
  //typedef VNL::NumericTraits<T>::abs_t abs_t;
  abs_t max = 0;
  for (unsigned int j=0; j<this->num_cols; ++j) {
    abs_t tmp = 0;
    for (unsigned int i=0; i<this->num_rows; ++i)
      tmp += VNL::Abs(this->data[i][j]);
    if (tmp > max)
      max = tmp;
  }
  return max;
}

// || M ||   = \max \sum | M   |
//        oo     i    j     ij
template <class T>
typename VNL::Matrix<T>::abs_t VNL::Matrix<T>::operatorInfNorm() const
{
  //typedef VNL::NumericTraits<T>::abs_t abs_t;
  abs_t max = 0;
  for (unsigned int i=0; i<this->num_rows; ++i) {
    abs_t tmp = 0;
    for (unsigned int j=0; j<this->num_cols; ++j)
      tmp += VNL::Abs(this->data[i][j]);
    if (tmp > max)
      max = tmp;
  }
  return max;
}

namespace VNL {
template <class doublereal>                        // ideally, char* should be bool* - PVr
int InplaceTranspose(doublereal *a, unsigned m, unsigned n, char* move, unsigned iwrk)
{
  static doublereal b, c;
  int k = m * n - 1;
  static int iter, i1, i2, im, i1c, i2c, ncount, max_;

// *****
//  ALGORITHM 380 - REVISED
// *****
//  A IS A ONE-DIMENSIONAL ARRAY OF LENGTH MN=M*N, WHICH
//  CONTAINS THE MXN MATRIX TO BE TRANSPOSED (STORED
//  COLUMWISE). MOVE IS A ONE-DIMENSIONAL ARRAY OF LENGTH IWRK
//  USED TO STORE INFORMATION TO SPEED UP THE PROCESS.  THE
//  VALUE IWRK=(M+N)/2 IS RECOMMENDED. IOK INDICATES THE
//  SUCCESS OR FAILURE OF THE ROUTINE.
//  NORMAL RETURN  IOK=0
//  ERRORS         IOK=-2 ,IWRK NEGATIVE OR ZERO
//                 IOK.GT.0, (SHOULD NEVER OCCUR),IN THIS CASE
//  WE SET IOK EQUAL TO THE FINAL VALUE OF ITER WHEN THE SEARCH
//  IS COMPLETED BUT SOME LOOPS HAVE NOT BEEN MOVED
//  NOTE * MOVE(I) WILL STAY ZERO FOR FIXED POINTS

  if (m < 2 || n < 2)
    return 0; // JUST RETURN IF MATRIX IS SINGLE ROW OR COLUMN
  if (iwrk < 1)
    return -2; // ERROR RETURN
  if (m == n) {
    // IF MATRIX IS SQUARE, EXCHANGE ELEMENTS A(I,J) AND A(J,I).
    for (unsigned i = 0; i < n; ++i)
    for (unsigned j = i+1; j < n; ++j) {
      i1 = i + j * n;
      i2 = j + i * m;
      b = a[i1];
      a[i1] = a[i2];
      a[i2] = b;
    }
    return 0; // NORMAL RETURN
  }
  ncount = 2;
  for (unsigned i = 0; i < iwrk; ++i)
    move[i] = char(0); // false;
  if (m > 2 && n > 2) {
// CALCULATE THE NUMBER OF FIXED POINTS, EUCLIDS ALGORITHM FOR GCD(M-1,N-1).
    int ir2 = m - 1;
    int ir1 = n - 1;
    int ir0 = ir2 % ir1;
    while (ir0 != 0) {
      ir2 = ir1;
      ir1 = ir0;
      ir0 = ir2 % ir1;
    }
    ncount += ir1 - 1;
  }
// SET INITIAL VALUES FOR SEARCH
  iter = 1;
  im = m;
// AT LEAST ONE LOOP MUST BE RE-ARRANGED
  goto L80;
// SEARCH FOR LOOPS TO REARRANGE
L40:
  max_ = k - iter;
  ++iter;
  if (iter > max_)
    return iter; // error return
  im += m;
  if (im > k)
    im -= k;
  i2 = im;
  if (iter == i2)
    goto L40;
  if (iter <= (int)iwrk) {
    if (move[iter-1])
      goto L40;
    else
      goto L80;
  }
  while (i2 > iter && i2 < max_) {
    i1 = i2;
    i2 = m * i1 - k * (i1 / n);
  }
  if (i2 != iter)
    goto L40;
// REARRANGE THE ELEMENTS OF A LOOP AND ITS COMPANION LOOP
L80:
  i1 = iter;
  b = a[i1];
  i1c = k - iter;
  c = a[i1c];
  while (true) {
    i2 = m * i1 - k * (i1 / n);
    i2c = k - i2;
    if (i1 <= (int)iwrk)
      move[i1-1] = '1'; // true;
    if (i1c <= (int)iwrk)
      move[i1c-1] = '1'; // true;
    ncount += 2;
    if (i2 == iter)
      break;
    if (i2+iter == k) {
      doublereal d = b; b = c; c = d; // interchange b and c
      break;
    }
    a[i1] = a[i2];
    a[i1c] = a[i2c];
    i1 = i2;
    i1c = i2c;
  }
// FINAL STORE AND TEST FOR FINISHED
  a[i1] = b;
  a[i1c] = c;
  if (ncount > k)
    return 0; // NORMAL RETURN
  goto L40;
} /* dtrans_ */
}; // End namespace VNL

/** Transpose matrix M in place.
*  Works for rectangular matrices using an enormously clever algorithm from ACM TOMS.
*/
template <class T>
void VNL::Matrix<T>::InplaceTranspose()
{
  unsigned m = Rows();
  unsigned n = Columns();
  unsigned iwrk = (m+n)/2;
  std::vector<char> move(iwrk);

  int iok = VNL::InplaceTranspose(DataBlock(), n, m, &move[0], iwrk);
  if (iok != 0)
    std::cerr << __FILE__ " : inplace_transpose() -- iok = " << iok << std::endl;

  this->num_rows = n;
  this->num_cols = m;

  // row pointers. we have to reallocate even when n<=m because
  // VNL::CVector<T>::Deallocate needs to know n_when_allocatod.
  {
    T *tmp = data[0];
    VNL::CVector<T>::Deallocate(data, m);
    data = VNL::CVector<T>::AllocateTptr(n);
    for (unsigned i=0; i<n; ++i)
      data[i] = tmp + i * m;
  }
}

//--------------------------------------------------------------------------------

#define VNL_MATRIX_INSTANTIATE(T) \
namespace VNL { \
  template class Matrix<T >; \
  template std::ostream & operator<<(std::ostream &, Matrix<T > const &); \
  template std::istream & operator>>(std::istream &, Matrix<T >       &); \
  template Matrix<T > operator-(T const &, Matrix<T > const &); \
  template Matrix<T > operator+(T const &, Matrix<T > const &); \
  template Matrix<T > operator*(T const &, Matrix<T > const &); \
  template T DotProduct(Matrix<T > const &, Matrix<T > const &); \
  template T InnerProduct(Matrix<T > const &, Matrix<T > const &); \
  template T CosAngle(Matrix<T > const &, Matrix<T > const &); \
  template Matrix<T > ElementProduct(Matrix<T > const &, Matrix<T > const &); \
  template Matrix<T > ElementQuotient(Matrix<T > const &, Matrix<T > const &); \
  template int InplaceTranspose(T*, unsigned, unsigned, char*, unsigned); \
};

#endif // Matrix_t_

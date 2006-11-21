// This is vxl/VNL/matrix.h
#ifndef vnl_matrix_h_
#define vnl_matrix_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief An ordinary mathematical matrix

*/

#include <iosfwd>
#include <VNL/tag.h>
#include <VNL/cvector.h>
#ifndef NDEBUG
# include "config.h"
# if VNL_CONFIG_CHECK_BOUNDS
#  include <VNL/error.h>
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
//--------------------------------------------------------------------------------

// Non member funcs (not in VNL namespace)
#define v VNL::Vector<T>
#define m VNL::Matrix<T>
template <class T> m operator+ (T const&, m const&);
template <class T> m operator- (T const&, m const&);
template <class T> m operator* (T const&, m const&);
template <class T> m ElementProduct(m const&, m const&);
template <class T> m ElementQuotient(m const&, m const&);
template <class T> T DotProduct (m const&, m const&);
template <class T> T InnerProduct (m const&, m const&);
template <class T> T CosAngle(m const&, m const& );
template <class T> std::ostream& operator<< (std::ostream&, m const&);
template <class T> std::istream& operator>> (std::istream&, m&);
#undef v
#undef m

//--------------------------------------------------------------------------------


enum MatrixType {
  MatrixNull,
  MatrixIdentity
};

/**  An ordinary mathematical matrix.
* The Matrix<T> class implements two-dimensional arithmetic
* matrices  for  a user-specified numeric data type. Using the
* parameterized types facility of C++,  it  is  possible,  for
* example, for the user to create a matrix of rational numbers
* by parameterizing the Matrix class over the Rational  class.
* The  only  requirement  for the type is that it supports the
* basic arithmetic operators.
*
* Note: Unlike   the   other   sequence   classes,   the
* Matrix<T>  class is fixed-size. It will not grow once the
* size has been specified to the constructor or changed by the
* assignment  or  multiplication  operators.  The Matrix<T>
* class is row-based with addresses of rows being cached,  and
* elements accessed as m[row][col].
*
* Note: The matrix can, however, be resized using the resize(nr,nc) function.
*
* Note: Indexing of the matrix is zero-based, so the top-left element is M(0,0).
*
* Note: Inversion of matrix M, and other operations such as solving systems of linear
* equations are handled by the matrix decomposition classes in vnl/algo, such
* as matrix_inverse, svd, qr etc.
*
* Note: Use a Vector<T> with these matrices.
*/

template<class T>
class Matrix
{
 public:
/** Default constructor creates an empty matrix of size 0,0.
*/
  Matrix () :
    num_rows(0),
    num_cols(0),
    data(0)
  {
  }

/** Construct a matrix of size r rows by c columns.
* Contents are unspecified.
* Complexity \f$O(1)\f$
*/
  Matrix(unsigned r, unsigned c);                           // r rows, c cols.

/** Construct a matrix of size r rows by c columns, and all emelemnts equal to v0.
* Complexity \f$O(r.c)\f$
*/
  Matrix(unsigned r, unsigned c, T const& v0);              // r rows, c cols, value v0.

/** Construct a matrix of size r rows by c columns, with a special type.
* Contents are specified by t
* Complexity \f$O(r.c)\f$
*/
 Matrix(unsigned r, unsigned c, MatrixType t);        // r rows, c cols, special type

/** Construct a matrix of size r rows by c columns, initialised by an automatic array.
* The first n elements, are initialised row-wise, to values.
* Complexity \f$O(n)\f$
*/
  Matrix(unsigned r, unsigned c, unsigned n, T const values[]);  // use automatic arrays.

/** Construct a matrix of size r rows by c columns, initialised by a memory block.
* The values are initialise row wise from the data.
* Complexity \f$O(r.c)\f$
*/
  Matrix(T const* data_block, unsigned r, unsigned c);      // fill row-wise.

/** Copy construct a matrix.
* Complexity \f$O(r.c)\f$
*/
  Matrix(Matrix<T> const&);                             // from another matrix.

  // These constructors are here so that operator* etc can take
  // advantage of the C++ return value optimization.
  Matrix (Matrix<T> const &, Matrix<T> const &, VNL::TagAdd); // M + M
  Matrix (Matrix<T> const &, Matrix<T> const &, VNL::TagSub); // M - M
  Matrix (Matrix<T> const &, T,                     VNL::TagMul); // M * s
  Matrix (Matrix<T> const &, T,                     VNL::TagDiv); // M / s
  Matrix (Matrix<T> const &, T,                     VNL::TagAdd); // M + s
  Matrix (Matrix<T> const &, T,                     VNL::TagSub); // M - s
  Matrix (Matrix<T> const &, Matrix<T> const &, VNL::TagMul); // M * M
  Matrix (Matrix<T> &that, VNL::TagGrab)
    : num_rows(that.num_rows), num_cols(that.num_cols), data(that.data)
  { that.num_cols=that.num_rows=0; that.data=0; } // "*this" now uses "that"'s data.

/** Matrix destructor.
*/
  ~Matrix() {
    // save some fcalls if data is 0 (i.e. in matrix_fixed)
    if (data) destroy();
  }

// Basic 2D-Array functionality-------------------------------------------

/** Return number of rows.
*/
  unsigned Rows ()    const { return num_rows; }

/** Return number of columns.
* A synonym for cols()
*/
 unsigned Columns () const { return num_cols; }

/** Return number of columns.
* A synonym for columns()
*/

  unsigned Cols ()    const { return num_cols; }

/** Return number of elements.
* This equals rows() * cols()
*/
  unsigned Size ()    const { return Rows()*Cols(); }

/** Return number of elements.
* This equals rows() * cols()
*/
  unsigned size ()    const { return Rows()*Cols(); }

/** set element with boundary checks if error checking is on.
*/
  void Put (unsigned r, unsigned c, T const&);

/** get element with boundary checks if error checking is on.
*/
  T    Get (unsigned r, unsigned c) const;

/** return pointer to given row.
* No boundary checking here.
*/
  T       * operator[] (unsigned r) { return data[r]; }

/** return pointer to given row.
* No boundary checking here.
*/
  T const * operator[] (unsigned r) const { return data[r]; }

/** Access an element for reading or writing.
* There are assert style boundary checks - #define NDEBUG to turn them off.
*/
  T       & operator() (unsigned r, unsigned c)
  {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
    assert(r<Rows());   // Check the row index is valid
    assert(c<Cols());   // Check the column index is valid
#endif
    return this->data[r][c];
  }

/** Access an element for reading.
* There are assert style boundary checks - #define NDEBUG to turn them off.
*/
  T const & operator() (unsigned r, unsigned c) const
  {
#if VNL_CONFIG_CHECK_BOUNDS  && (!defined NDEBUG)
    assert(r<Rows());   // Check the row index is valid
    assert(c<Cols());   // Check the column index is valid
#endif
    return this->data[r][c];
  }


// Filling and copying------------------------------------------------

/** Set all elements of matrix to specified value.
* Complexity \f$O(r.c)\f$
*/
  void Fill (T const&);

/** Set all diagonal elements of matrix to specified value.
* Complexity \f$O(\min(r,c))\f$
*/
  void FillDiagonal (T const&);

/** Fill (laminate) this matrix with the given data.
* We assume that p points to a contiguous rows*cols array, stored rowwise.
*/
  void CopyIn(T const *);

/** Fill (laminate) this matrix with the given data.
* A synonym for copy_in()
*/
  void Set(T const *d) { CopyIn(d); }

/** Fill the given array with this matrix.
* We assume that p points to
* a contiguous rows*cols array, stored rowwise.
* No bounds checking on the array
*/
  void CopyOut(T *) const;


/** Set all elements to value v.
* Complexity \f$O(r.c)\f$
*/
  Matrix<T>& operator= (T const&v) { Fill(v); return *this; }

/** Copies all elements of rhs matrix into lhs matrix.
* Complexity \f$O(\min(r,c))\f$
*/
  Matrix<T>& operator= (Matrix<T> const&);

// Arithmetic ----------------------------------------------------
  // note that these functions should not pass scalar as a const&.
  // Look what would happen to A /= A(0,0).

/** Add rhs to each element of lhs matrix in situ.
*/
  Matrix<T>& operator+= (T value);

/** Subtract rhs from each element of lhs matrix in situ.
*/
  Matrix<T>& operator-= (T value);

/** Scalar multiplication in situ of lhs matrix  by rhs.
*/
  Matrix<T>& operator*= (T value);

/** Scalar division of lhs matrix  in situ by rhs.
*/
  Matrix<T>& operator/= (T value);

/** Add rhs to lhs  matrix in situ.
*/
  Matrix<T>& operator+= (Matrix<T> const&);
/** Subtract rhs from lhs matrix in situ.
*/
  Matrix<T>& operator-= (Matrix<T> const&);
/** Multiply lhs matrix in situ by rhs.
*/
  Matrix<T>& operator*= (Matrix<T> const&rhs) { *this = (*this) * rhs; return *this; }

/** Negate all elements of matrix.
*/
  Matrix<T> operator- () const;


/** Add rhs to each element of lhs matrix and return result in new matrix.
*/
  Matrix<T> operator+ (T const& v) const { return Matrix<T>(*this, v, VNL::TagAdd()); }

/** Subtract rhs from each element of lhs matrix and return result in new matrix.
*/
  Matrix<T> operator- (T const& v) const { return Matrix<T>(*this, v, VNL::TagSub()); }

/** Scalar multiplication of lhs matrix by rhs  and return result in new matrix.
*/
  Matrix<T> operator* (T const& v) const { return Matrix<T>(*this, v, VNL::TagMul()); }

/** Scalar division of lhs matrix by rhs and return result in new matrix.
*/
  Matrix<T> operator/ (T const& v) const { return Matrix<T>(*this, v, VNL::TagDiv()); }

/** Matrix add rhs to lhs matrix and return result in new matrix.
*/
  Matrix<T> operator+ (Matrix<T> const& rhs) const { return Matrix<T>(*this, rhs, VNL::TagAdd()); }
/** Matrix subtract rhs from lhs and return result in new matrix.
*/
  Matrix<T> operator- (Matrix<T> const& rhs) const { return Matrix<T>(*this, rhs, VNL::TagSub()); }
/** Matrix multiply lhs by rhs matrix and return result in new matrix.
*/
  Matrix<T> operator* (Matrix<T> const& rhs) const { return Matrix<T>(*this, rhs, VNL::TagMul()); }

  ////--------------------------- Additions ----------------------------

/** Make a new matrix by applying function to each element.
*/
  Matrix<T> Apply(T (*f)(T)) const;

/** Make a new matrix by applying function to each element.
*/
  Matrix<T> Apply(T (*f)(T const&)) const;

/** Return transpose.
*/
  Matrix<T> Transpose () const;

/** Return conjugate transpose.
*/
  Matrix<T> ConjugateTranspose () const;

/** Set values of this matrix to those of M, starting at [top,left].
*/
  Matrix<T>& Update (Matrix<T> const&, unsigned top=0, unsigned left=0);

/** Set the elements of the i'th column to v[j]  (No bounds checking).
*/
  void SetColumn(unsigned i, T const * v);

/** Set the elements of the i'th column to value.
*/
  void SetColumn(unsigned i, T value );

/** Set j-th colum to v.
*/
  void SetColumn(unsigned j, Vector<T> const& v);

/** Set columns to those in M, starting at starting_column.
*/
  void SetColumns(unsigned starting_column, Matrix<T> const& M);

/** Set the elements of the i'th row to v[j]  (No bounds checking).
*/
  void SetRow   (unsigned i, T const * v);

/** Set the elements of the i'th row to value.
*/
  void SetRow   (unsigned i, T value );

/** Set the i-th row.
*/
  void SetRow   (unsigned i, Vector<T> const&);

/** Extract a sub-matrix of size rows x cols, starting at (top,left).
*  Thus it contains elements  [top,top+rows-1][left,left+cols-1]
*/
  Matrix<T> Extract (unsigned rows,  unsigned cols,
             unsigned top=0, unsigned left=0) const;

/** Get a vector equal to the given row.
*/
  Vector<T> GetRow   (unsigned row) const;

/** Get a vector equal to the given column.
*/
  Vector<T> GetColumn(unsigned col) const;

/** Get n rows beginning at rowstart.
*/
  Matrix<T> GetNRows   (unsigned rowstart, unsigned n) const;

/** Get n columns beginning at colstart.
*/
  Matrix<T> GetNColumns(unsigned colstart, unsigned n) const;


  // mutators

/** Set this matrix to an identity matrix.
*  Abort if the matrix is not square
*/
  void SetIdentity();

/** Transpose this matrix efficiently.
*/
  void InplaceTranspose();

/** Reverse order of rows.
*/
  void FlipUD();
/** Reverse order of columns.
*/
  void FlipLR();

/** Normalize each row so it is a unit vector.
*  Zero rows are ignored
*/
  void NormalizeRows();

/** Normalize each column so it is a unit vector.
*  Zero columns are ignored
*/
  void NormalizeColumns();

/** Scale elements in given row by a factor of T.
*/
  void ScaleRow   (unsigned row, T value);

/** Scale elements in given column by a factor of T.
*/
  void ScaleColumn(unsigned col, T value);

/** Swap this matrix with that matrix.
*/
  void Swap(Matrix<T> & that);

/** Type def for norms.
*/
  typedef typename CVector<T>::abs_t abs_t;

/** Return sum of absolute values of elements.
*/
  abs_t ArrayOneNorm() const { return CVector<T>::OneNorm(begin(), size()); }

/** Return square root of sum of squared absolute element values.
*/
  abs_t ArrayTwoNorm() const { return CVector<T>::TwoNorm(begin(), size()); }

/** Return largest absolute element value.
*/
  abs_t ArrayInfNorm() const { return CVector<T>::InfNorm(begin(), size()); }

/** Return sum of absolute values of elements.
*/
  abs_t AbsoluteValueSum() const { return ArrayOneNorm(); }

/** Return largest absolute value.
*/
  abs_t AbsoluteValueMax() const { return ArrayInfNorm(); }

  abs_t operatorOneNorm() const;

  //abs_t operator_two_norm() const;

  abs_t operatorInfNorm() const;

/** Return frobenius norm of matrix (sqrt of sum of squares of its elements).
*/
  abs_t FrobeniusNorm() const { return CVector<T>::TwoNorm(begin(), size()); }

/** Return frobenius norm of matrix (sqrt of sum of squares of its elements).
*/
  abs_t FroNorm() const { return FrobeniusNorm(); }

/** Return RMS of all elements.
*/
  abs_t RMS() const { return CVector<T>::RMSNorm(begin(), size()); }

/** Return minimum value of elements.
*/
  T MinValue() const { return CVector<T>::MinValue(begin(), size()); }

/** Return maximum value of elements.
*/
  T MaxValue() const { return CVector<T>::MaxValue(begin(), size()); }

/** Return mean of all matrix elements.
*/
  T Mean() const { return CVector<T>::Mean(begin(), size()); }

#if 0 // deprecated
  // <deprecated>
  // These two methods have been intentionally poisoned. The new equivalents are:
  //   array_one_norm() / array_inf_norm()
  // or
  //   absolute_value_sum() / absolute_value_max()
  abs_t OneNorm(void *) const { return CVector<T>::OneNorm(begin(), size()); }
  abs_t InfNorm(void *) const { return CVector<T>::InfNorm(begin(), size()); }
  // </deprecated>
#endif

  // predicates

/** Return true iff the size is zero.
*/
  bool IsEmpty() const { return !data || !num_rows || !num_cols; }

/**  Return true if all elements equal to identity.
*/
  bool IsIdentity() const;

/**  Return true if all elements equal to identity, within given tolerance.
*/
  bool IsIdentity(double tol) const;

/** Return true if all elements equal to zero.
*/
  bool IsZero() const;

/** Return true if all elements equal to zero, within given tolerance.
*/
  bool IsZero(double tol) const;

/** Return true if finite.
*/
  bool IsFinite() const;

/** Return true if matrix contains NaNs.
*/
  bool HasNaNs() const;

/** abort if size is not as expected.
* This function does or tests nothing if NDEBUG is defined
*/
  void assert_size(unsigned rows, unsigned cols) const {
#ifndef NDEBUG
    assert_size_internal(rows, cols);
#endif
  }
/** abort if matrix containins any INFs or NANs.
* This function does or tests nothing if NDEBUG is defined
*/
  void assert_finite() const {
#ifndef NDEBUG
    assert_finite_internal();
#endif 
  }

  ////----------------------- Input/Output ----------------------------

  // : Read a Matrix from an ascii std::istream, automatically
  // determining file size if the input matrix has zero size.
  static Matrix<T> Read(std::istream& s);

  // : Read a Matrix from an ascii std::istream, automatically
  // determining file size if the input matrix has zero size.
  bool ReadASCII(std::istream& s);

  //--------------------------------------------------------------------------------

/** Access the contiguous block storing the elements in the matrix row-wise.\ O(1).
* 1d array, row-major order.
*/
  T const* DataBlock () const { return data[0]; }

/** Access the contiguous block storing the elements in the matrix row-wise.\ O(1).
* 1d array, row-major order.
*/
  T      * DataBlock () { return data[0]; }

/** Access the 2D array, so that elements can be accessed with array[row][col] directly.
*  2d array, [row][column].
*/
  T const* const* DataArray () const { return data; }

/** Access the 2D array, so that elements can be accessed with array[row][col] directly.
*  2d array, [row][column].
*/
  T      *      * DataArray () { return data; }

  typedef T element_type;

/** Iterators.
*/
  typedef T       *iterator;
/** Iterator pointing to start of data.
*/
  iterator       begin() { return data?data[0]:0; }
/** Iterator pointing to element beyond end of data.
*/
  iterator       end() { return data?data[0]+num_rows*num_cols:0; }

/** Const iterators.
*/
  typedef T const *const_iterator;
/** Iterator pointing to start of data.
*/
  const_iterator begin() const { return data?data[0]:0; }
/** Iterator pointing to element beyond end of data.
*/
  const_iterator end() const { return data?data[0]+num_rows*num_cols:0; }

  //--------------------------------------------------------------------------------

/** Return true if *this == rhs.
*/
  bool operator_eq (Matrix<T> const & rhs) const;

/** Equality operator.
*/
  bool operator==(Matrix<T> const &that) const { return  this->operator_eq(that); }

/** Inequality operator.
*/
  bool operator!=(Matrix<T> const &that) const { return !this->operator_eq(that); }

/** Print matrix to os in some hopefully sensible format.
*/
  void print(std::ostream& os) const;

/** Make the matrix as if it had been default-constructed.
*/
  void Clear();

/** Resize to r rows by c columns.\ Old data lost.
* returns true if size changed.
*/
  bool Resize (unsigned r, unsigned c);
//--------------------------------------------------------------------------------


 protected:
  unsigned num_rows;   // Number of rows
  unsigned num_cols;   // Number of columns
  T** data;            // Pointer to the Matrix

  void assert_size_internal(unsigned rows, unsigned cols) const;
  void assert_finite_internal() const;

/** Delete data.
*/
  void destroy();

#if VCL_NEED_FRIEND_FOR_TEMPLATE_OVERLOAD
# define v Vector<T>
# define m Matrix<T>
  friend m operator+         <> (T const&, m const&);
  friend m operator-         <> (T const&, m const&);
  friend m operator*         <> (T const&, m const&);
  friend m ElementProduct   <> (m const&, m const&);
  friend m ElementQuotient  <> (m const&, m const&);
  friend T DotProduct       <> (m const&, m const&);
  friend T InnerProduct     <> (m const&, m const&);
  friend T CosAngle         <> (m const&, m const&);
  friend std::ostream& operator<< <> (std::ostream&, m const&);
  friend std::istream& operator>> <> (std::istream&, m&);
# undef v
# undef m
#endif

  // inline function template instantiation hack for gcc 2.97 -- fsm
  static void inline_function_tickler();
};


// Definitions of inline functions.


/** get -- Returns the value of the element at specified row and column.\ O(1).
* Checks for valid range of indices.
*/

template<class T>
inline T Matrix<T>::Get (unsigned row, unsigned column) const {
#if ERROR_CHECKING
  if (row >= this->num_rows)                    // If invalid size specified
    vnl_error_matrix_row_index ("get", row);    // Raise exception
  if (column >= this->num_cols)                 // If invalid size specified
    vnl_error_matrix_col_index ("get", column); // Raise exception
#endif
  return this->data[row][column];
}

/** put -- Puts value into element at specified row and column.\ O(1).
* Checks for valid range of indices.
*/

template<class T>
inline void Matrix<T>::Put (unsigned row, unsigned column, T const& value) {
#if ERROR_CHECKING
  if (row >= this->num_rows)                    // If invalid size specified
    vnl_error_matrix_row_index ("put", row);  // Raise exception
  if (column >= this->num_cols)                 // If invalid size specified
    vnl_error_matrix_col_index ("put", column); // Raise exception
#endif
  this->data[row][column] = value;              // Assign data value
}





// non-member arithmetical operators (not in VNL namespace).

template<class T>
inline Matrix<T> operator* (T const& value, Matrix<T> const& m) {
  return Matrix<T>(m, value, VNL::TagMul());
}

template<class T>
inline Matrix<T> operator+ (T const& value, Matrix<T> const& m) {
  return Matrix<T>(m, value, VNL::TagAdd());
}

template<class T>
inline void Swap(Matrix<T> &A, Matrix<T> &B) { A.Swap(B); }

}; // End namespace VNL


#endif // Matrix_h_

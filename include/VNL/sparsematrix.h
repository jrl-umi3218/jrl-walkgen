// This is vxl/VNL/sparse_matrix.h
#ifndef vnl_sparse_matrix_h_
#define vnl_sparse_matrix_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/** \file
*  \brief Simple sparse matrix

*
*    Only those values which
*    are non-zero are stored. The sparse matrix currently supports
*    only getting/putting elements, and multiply by vector or another
*    sparse matrix.
*
*    Each row is stored as a vector of vcl_pair<unsigned int,T>, where the first
*    of the pair indicates the column index, and the second the
*    value.  All rows are stored, as std::vector< row >;
*
*  \author Rupert W. Curwen, GE CR&D
*  \date   20 Oct 98
*
   \verbatim
       Modifications
  
       Robin Flatland 5/31/99 Added pre_mult(lhs,result), where
                              lhs is a vector.
  
       Robin Flatland 6/08/99 Added iterator that allows sequential
                              access to non-zero values in matrix.
                              Iterator is controlled using reset, next,
                              getrow, getcolumn, and value.
  
       David Capel May 2000   Added set_row, scale_row, mult, vcat and const
                              methods where appropriate.
   \endverbatim
*/

#include <vector>
#include <VNL/vector.h>
#include <functional>

namespace VNL {


/** Stores elements of sparse matrix.
*  Only those values which
*  are non-zero are stored. The sparse matrix currently supports
*  only getting/putting elements, and multiply by vector or another
*  sparse matrix.
*
*  Each row is stored as a vector of vcl_pair<unsigned int,T>, where the first
*  of the pair indicates the column index, and the second the
*  value.  All rows are stored, as std::vector< row >;
*
*/
template <class T>
class SparseMatrixPair
{
 public:
  unsigned int first;
  T second;

/** Constructs a pair with null values.
*/
  SparseMatrixPair() : first(0), second(T(0)) {}

/** Constructs a pair with position a and value b.
*/
  SparseMatrixPair(unsigned int const& a, T const& b) : first(a), second(b) {}

  SparseMatrixPair(const SparseMatrixPair<T>& o) : first(o.first), second(o.second) {}

  SparseMatrixPair<T>& operator=(SparseMatrixPair const &o) {
    if (&o != this) {
      first = o.first;
      second = o.second;
    }
    return *this;
  }

  struct less : public std::binary_function<SparseMatrixPair, SparseMatrixPair, bool>
  {
    bool operator() (SparseMatrixPair const& p1, SparseMatrixPair const& p2) {
      return p1.first < p2.first;
    }
  };
};


/** Simple sparse matrix.
*  Stores non-zero elements as a sparse_matrix_pair
*/
template <class T>
class SparseMatrix
{
 public:
  typedef SparseMatrixPair<T> pair_t;
#if defined(VCL_SUNPRO_CC)
  // SunPro is the broken one.
  typedef std::vector < typename pair_t > row;
  typedef std::vector < typename row > SparseMatrixElements;
#else
  typedef std::vector < pair_t > row;
  typedef std::vector < row > SparseMatrixElements;
#endif

  // typedef std::vector<typename pair_t> row;

/** Construct an empty matrix.
*/
  SparseMatrix();

/** Construct an empty m*n matrix.
*/
  SparseMatrix(unsigned int m, unsigned int n);

/** Construct an m*n Matrix and copy rhs into it.
*/
  SparseMatrix(const SparseMatrix<T>& rhs);

/** Copy another SparseMatrix<T> into this.
*/
  SparseMatrix<T>& operator=(const SparseMatrix<T>& rhs);

/** Multiply this*rhs, another sparse matrix.
*/
  void mult(SparseMatrix<T> const& rhs, SparseMatrix<T>& result) const;

/** Multiply this*rhs, where rhs is a vector.
*/
  void mult(Vector<T> const& rhs, Vector<T>& result) const;

/** Multiply this*p, a fortran order matrix.
*/
  void mult(unsigned int n, unsigned int m, T const* p, T* q) const;

/** Multiplies lhs*this, where lhs is a vector.
*/
  void pre_mult(const Vector<T>& lhs, Vector<T>& result) const;

/** Add rhs to this.
*/
  void add(const SparseMatrix<T>& rhs, SparseMatrix<T>& result) const;

/** Subtract rhs from this.
*/
  void subtract(const SparseMatrix<T>& rhs, SparseMatrix<T>& result) const;

/** Get a reference to an entry in the matrix.
*/
  T& operator()(unsigned int row, unsigned int column);

/** Get diag(A_tranpose * A).
* Useful for forming Jacobi preconditioners for linear solvers.
*/
  void diag_AtA(Vector<T>& result) const;

/** Set a whole row at once.\ Much faster.
*/
  void set_row(unsigned int r,
               std::vector<int> const& cols,
               std::vector<T> const& vals);

/** Return row as vector of pairs.
*  Added to aid binary I/O
*/
  row& get_row(unsigned int r) {return elements[r];}

/** Laminate matrix A onto the bottom of this one.
*/
  SparseMatrix<T>& vcat(SparseMatrix<T> const& A);

/** Get the number of rows in the matrix.
*/
  unsigned int rows() const { return rs_; }

/** Get the number of columns in the matrix.
*/
  unsigned int columns() const { return cs_; }

/** Get the number of columns in the matrix.
*/
  unsigned int cols() const { return cs_; }

/** Return whether a given row is empty.
*/
  bool empty_row(unsigned int r) const { return elements[r].empty(); }

/** This is occasionally useful.
*/
  T sum_row(unsigned int r);

/** Useful for normalizing row sums in convolution operators.
*/
  void scale_row(unsigned int r, T scale);

/** Resizes the array to have r rows and c cols.
*    Currently not implemented.
*/
  void resize( int r, int c );

/** Resets the internal iterator.
*/
  void reset();

/** Moves the internal iterator to next non-zero entry in matrix.
* Returns true if there is another value, false otherwise. Use
* in combination with methods reset, getrow, getcolumn, and value.
*/
  bool next();

/** Returns the row of the entry pointed to by internal iterator.
*/
  int getrow();

/** Returns the column of the entry pointed to by internal iterator.
*/
  int getcolumn();

/** Returns the value pointed to by the internal iterator.
*/
  T value();


 protected:
  SparseMatrixElements elements;
  unsigned int rs_, cs_;

  // internal iterator
  unsigned int itr_row;
  typename row::iterator itr_cur;
  bool itr_isreset;
};


}; // End namespace VNL


#endif // SparseMatrix_h_

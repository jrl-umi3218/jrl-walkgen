// This is vxl/VNL/sym_matrix.h
#ifndef vnl_sym_matrix_h_
#define vnl_sym_matrix_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Contains class for symmetric matrices

*  \author Ian Scott (Manchester ISBE)
*  \date   6/12/2001
*
*/
#include <assert.h>
#include <VNL/vector.h>
#include <VNL/matrix.h>
#include <VNL/cvector.h>

namespace VNL {

/** stores a symmetric matrix as just the diagonal and lower triangular part.
*  vnl_sym_matrix stores a symmetric matrix for time and space efficiency.
*  Specifically, only the diagonal and lower triangular elements are stored.
*/

template <class T>
class SymMatrix
{
 public:
/** Construct an empty symmetic matrix.
*/
  SymMatrix(): data_(0), index_(0), nn_(0) {}

/** Construct an symmetic matrix of size nn by nn.
*/
  explicit SymMatrix(unsigned nn)
    : data_(VNL::CVector<T>::AllocateT(nn * (nn + 1) / 2)),
      index_(VNL::CVector<T>::AllocateTptr(nn)),
      nn_(nn) { _SetupIndex(); }

/** Construct a symmetric matrix with elements equal to data.
* value should be stored row-wise, and contrain the
* n*(n+1)/2 diagonal and lower triangular elements
*/
  inline SymMatrix(T const * data, unsigned nn);

/** Construct a symmetric matrix with all elements equal to value.
*/
  inline SymMatrix(unsigned nn, const T & value);

/** Construct a symmetric matrix from a full matrix.
* If NDEBUG is set, the symmetry of the matrix will be asserted.
*/
  inline explicit SymMatrix(VNL::Matrix<T> const& that);
  ~SymMatrix()
  { VNL::CVector<T>::Deallocate(data_, size());
    VNL::CVector<T>::Deallocate(index_, nn_);}

  SymMatrix<T>& operator=(SymMatrix<T> const& that);

  // Operations----------------------------------------------------------------

/** In-place arithmetic operations.
*/
  SymMatrix<T>& operator*=(T v) { VNL::CVector<T>::Scale(data_, data_, size(), v); return *this; }
/** In-place arithmetic operations.
*/
  SymMatrix<T>& operator/=(T v) { VNL::CVector<T>::Scale(data_, data_, size(), ((T)1)/v); return *this; }


  // Data Access---------------------------------------------------------------

  T operator () (unsigned i, unsigned j) const {
    return (i > j) ? index_[i][j] : index_[j][i];
  }

  T& operator () (unsigned i, unsigned j) {
    return (i > j) ? index_[i][j] : index_[j][i];
  }

/** Access a half-row of data.
* Only the first i+1 values from this pointer are valid.
*/
  const T* operator [] (unsigned i) const {
    assert (i < nn_);
    return index_[i];
  }

/** fast access, however i >= j.
*/
  T Fast (unsigned i, unsigned j) const {
    assert (i >= j);
    return index_[i][j];
  }

/** fast access, however i >= j.
*/
  T& Fast (unsigned i, unsigned j) {
    assert (i >= j);
    return index_[i][j];
  }

  // STL style iterators

  typedef T* iterator;
  inline iterator begin() { return data_; }
  inline iterator end() { return data_ + size(); }
  typedef const T* const_iterator;
  inline const_iterator begin() const { return data_; }
  inline const_iterator end() const { return data_ + size(); }
  unsigned long size() const { return nn_ * (nn_ + 1) / 2; }


  unsigned long Size() const { return nn_ * (nn_ + 1) / 2; }
  unsigned Rows() const { return nn_; }
  unsigned Cols() const { return nn_; }
  unsigned Columns() const { return nn_; }

  // Need this until we add a SymMatrix ctor to vnl_matrix;
  inline Matrix<T> AsMatrix() const;

/** Resize matrix to n by n.
*/
  inline void Resize(int n);

/** Return pointer to the lower triangular elements as a contiguous 1D C array;.
*/
  T*       DataBlock()       { return data_; }
/** Return pointer to the lower triangular elements as a contiguous 1D C array;.
*/
  T const* DataBlock() const { return data_; }

/** Set the first i values of row i.
* or the top i values of column i
*/
  void SetHalfRow (const VNL::Vector<T> &half_row, unsigned i);
 
/** Replaces the symmetric submatrix of THIS matrix with the elements of matrix m.
* Starting at top left corner. Complexity is \f$O(m^2)\f$.
*/
  SymMatrix<T>& Update (SymMatrix<T> const& m, unsigned diag_start=0);

/** Swap contents of m with THIS.
*/
  void Swap(SymMatrix &m);

 protected:
/** Set up the index array.
*/
  inline void _SetupIndex() {
    T * data = data_;
    for (unsigned i=0; i< nn_; ++i) { index_[i] = data; data += i+1; }
  }

  T* data_;
  T** index_;
  unsigned nn_;
};

}; // End namespace VNL

template <class T> std::ostream& operator<< (std::ostream&, VNL::SymMatrix<T> const&);


template <class T>
inline VNL::SymMatrix<T>::SymMatrix(T const * data, unsigned nn):
  data_(VNL::CVector<T>::AllocateT(nn * (nn + 1) / 2)),
  index_(VNL::CVector<T>::AllocateTptr(nn)),
  nn_(nn)
{
  _SetupIndex();
  for (unsigned i = 0; i < nn_; ++i)
    for (unsigned j = 0; j <= i; ++j)
      Fast(i,j) = *(data++);
}

template <class T>
inline VNL::SymMatrix<T>::SymMatrix(unsigned nn, const T & value):
  data_(VNL::CVector<T>::AllocateT(nn * (nn + 1) / 2)),
  index_(VNL::CVector<T>::AllocateTptr(nn)),
  nn_(nn)
{
  _SetupIndex();
  VNL::CVector<T>::Fill(data_, size(), value);
}


template <class T>
inline VNL::SymMatrix<T>::SymMatrix(VNL::Matrix<T> const& that):
  data_(VNL::CVector<T>::AllocateT(that.Rows() * (that.Rows() + 1) / 2)),
  index_(VNL::CVector<T>::AllocateTptr(that.Rows())),
  nn_(that.Rows())
{
  _SetupIndex();
  assert (nn_ == that.Cols());
  for (unsigned i = 0; i < nn_; ++i)
    for (unsigned j = 0; j <= i; ++j)
    {
      assert( that(i,j) == that(j,i) );
      Fast(i,j) = that(i,j);
    }
}

/** Convert a SymMatrix to a Matrix.
*/
template <class T>
inline VNL::Matrix<T> VNL::SymMatrix<T>::AsMatrix() const
{
  VNL::Matrix<T> ret(nn_, nn_);
  for (unsigned i = 0; i < nn_; ++i)
    for (unsigned j = 0; j <= i; ++j)
      ret(i,j) = ret(j,i) = Fast(i,j);
  return ret;
}


template <class T>
inline void VNL::SymMatrix<T>::Resize(int n)
{
  if (n == (int)nn_) return;

  VNL::CVector<T>::Deallocate(data_, size());
  VNL::CVector<T>::Deallocate(index_, nn_);

  nn_ = n;
  data_ = VNL::CVector<T>::AllocateT(size());
  index_ = VNL::CVector<T>::AllocateTptr(n);

  _SetupIndex();
}

template <class T>
bool operator==(const VNL::SymMatrix<T> &a, const VNL::SymMatrix<T> &b);

template <class T>
bool operator==(const VNL::SymMatrix<T> &a, const VNL::Matrix<T> &b);

template <class T>
bool operator==(const VNL::Matrix<T> &a, const VNL::SymMatrix<T> &b);

/** Swap the contents of a and b.
*/
template <class T>
void Swap(VNL::SymMatrix<T> &a, VNL::SymMatrix<T> &b)
{ a.Swap(b); }


#endif // SymMatrix_h_

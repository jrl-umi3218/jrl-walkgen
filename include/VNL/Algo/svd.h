// This is vxl/VNL/algo/svd.h
#ifndef vnl_svd_h_
#define vnl_svd_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Holds the singular value decomposition of a VNL::Matrix.

*  \author Andrew W. Fitzgibbon, Oxford IERG
*  \date   15 Jul 96
*
   \verbatim
    Modifications
   F. Schaffalitzky, Oxford IESRG, 26 Mar 1999
       1. The singular values are now stored as reals (not complexes) when T is complex.
       2. Fixed bug : for complex T, matrices have to be conjugated as well as transposed.
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*/

#include <VNL/numerictraits.h>
#include <VNL/vector.h>
#include <VNL/matrix.h>
#include <VNL/diagmatrix.h>
#include <iosfwd>


namespace VNL {

  // The export command isn't supported in GCC and generates oodles of warnings
  //export template <class T> class SVD;


/** Holds the singular value decomposition of a VNL::Matrix.
*
*  The class holds three matrices U, W, V such that the original matrix
*  \f$M = U W V^\top\f$.  The DiagMatrix W stores the singular values in decreasing
*  order.  The columns of U which correspond to the nonzero singular values
*  form a basis for range of M, while the columns of V corresponding to the
*  zero singular values are the nullspace.
*
*  The SVD is computed at construction time, and enquiries may then be made
*  of the SVD.  In particular, this allows easy access to multiple
*  right-hand-side solves without the bother of putting all the RHS's into a
*  Matrix.
*
*  This class is supplied even though there is an existing VNL::Matrix method
*  for several reasons:
*
*  It is more convenient to use as it manages all the storage for
*  the U,S,V matrices, allowing repeated queries of the same SVD
*  results.
*
*  It avoids namespace clutter in the Matrix class.   While svd()
*  is a perfectly reasonable method for a Matrix, there are many other
*  decompositions that might be of interest, and adding them all would
*  make for a very large Matrix class.
*
*  It demonstrates the holder model of compute class, implementing an
*  algorithm on an object without adding a member that may not be of
*  general interest.  A similar pattern can be used for other
*  decompositions which are not defined as members of the library Matrix
*  class.
*
*  It extends readily to n-ary operations, such as generalized
*  eigensystems, which cannot be members of just one matrix.
*/

template <class T>
class SVD
{
 public:
/** The singular values of a matrix of complex<T> are of type T, not complex<T>.
*/
  typedef typename VNL::NumericTraits<T>::abs_t singval_t;

/**
* Construct an vnl_svd<T> object from \f$m \times n\f$ matrix \f$M\f$.  The
* vnl_svd<T> object contains matrices \f$U\f$, \f$W\f$, \f$V\f$ such that
* \f$U W V^\top = M\f$.
*
* Uses linpack routine DSVDC to calculate an ``economy-size'' SVD
* where the returned \f$U\f$ is the same size as \f$M\f$, while \f$W\f$
* and \f$V\f$ are both \f$n \times n\f$.  This is efficient for
* large rectangular solves where \f$m > n\f$, typical in least squares.
*
* The optional argument zero_out_tol is used to mark the zero singular
* values: If nonnegative, any s.v. smaller than zero_out_tol in
* absolute value is set to zero.  If zero_out_tol is negative, the
* zeroing is relative to |zero_out_tol| * sigma_max();
*/

  SVD(Matrix<T> const &M, double zero_out_tol = 0.0);
 ~SVD() {}

  // Data Access---------------------------------------------------------------

/** find weights below threshold tol, zero them out, and update W_ and Winverse_.
*/
  void            ZeroOutAbsolute(double tol = 1e-8); //sqrt(machine epsilon)

/** find weights below tol*max(w) and zero them out.
*/
  void            ZeroOutRelative(double tol = 1e-8); //sqrt(machine epsilon)
  int             Singularities () const { return W_.Rows() - Rank(); }
  int             Rank () const { return rank_; }
  singval_t       WellCondition () const { return SigmaMin()/SigmaMax(); }

/** Calculate determinant as product of diagonals in W.
*/
  singval_t       DeterminantMagnitude () const;
  singval_t       Norm() const;

/** Return the matrix U.
*/
  Matrix<T>      & U()       { return U_; }

/** Return the matrix U.
*/
  Matrix<T> const& U() const { return U_; }

/** Return the matrix U's (i,j)th entry (to avoid svd.U()(i,j); ).
*/
  T U(int i, int j) { return U_(i,j); }

/** Get at DiagMatrix (q.v.) of singular values, sorted from largest to smallest.
*/
  DiagMatrix<singval_t>       & W()             { return W_; }

/** Get at DiagMatrix (q.v.) of singular values, sorted from largest to smallest.
*/
  DiagMatrix<singval_t> const & W() const       { return W_; }
  DiagMatrix<singval_t>       & WInverse()      { return Winverse_; }
  VNL::DiagMatrix<singval_t> const & Winverse() const { return Winverse_; }
  singval_t                   & W(int i, int j) { return W_(i,j); }
  singval_t                   & W(int i)        { return W_(i,i); }
  singval_t     SigmaMax() const { return W_(0,0); }       // largest
  singval_t     SigmaMin() const { return W_(n_-1,n_-1); } // smallest

/** Return the matrix V.
*/
  Matrix<T>      & V()       { return V_; }

/** Return the matrix V.
*/
  Matrix<T> const& V() const { return V_; }

/** Return the matrix V's (i,j)th entry (to avoid svd.V()(i,j); ).
*/
  T V(int i, int j) { return V_(i,j); }

/**
*/
  Matrix<T> Inverse () const;

/** pseudo-inverse (for non-square matrix).
*/
  Matrix<T> PseudoInverse () const;

/** pseudo-inverse (for non-square matrix) of desired rank.
*/
  Matrix<T> PseudoInverse (int rank) const;

/** Calculate inverse of transpose.
*/
  Matrix<T> TransposeInverse () const;

/** Recompose SVD to U*W*V'.
*/
  Matrix<T> Recompose () const;

/** Solve the matrix equation M X = B, returning X.
*/
  Matrix<T> Solve (Matrix<T> const& B) const;

/** Solve the matrix-vector system M x = y, returning x.
*/
  Vector<T> Solve (Vector<T> const& y) const;
  void          Solve (T const *rhs, T *lhs) const; // min ||A*lhs - rhs||

/** Solve the matrix-vector system M x = y.
* Assuming that the singular values W have been preinverted by the caller.
*/
  void SolvePreinverted(Vector<T> const& rhs, Vector<T>* out) const;

/** Return N such that M * N = 0.
*/
  VNL::Matrix<T> Nullspace() const;

/** Return N such that M' * N = 0.
*/
  VNL::Matrix<T> LeftNullspace() const;

/** Return N such that M * N = 0.
*/
  VNL::Matrix<T> Nullspace(int required_nullspace_dimension) const;

/** Implementation to be done yet; currently returns left_nullspace().\ - PVR.
*/
  VNL::Matrix<T> LeftNullspace(int required_nullspace_dimension) const;

/** Return the rightmost column of V.
*  Does not check to see whether or not the matrix actually was rank-deficient -
* the caller is assumed to have examined W and decided that to his or her satisfaction.
*/
  VNL::Vector<T> Nullvector() const;

/** Return the rightmost column of U.
*  Does not check to see whether or not the matrix actually was rank-deficient.
*/
  VNL::Vector<T> LeftNullvector() const;

  bool Valid() const { return valid_; }

 private:

  int m_, n_;              // Size of M, local cache.
  VNL::Matrix<T> U_;        // Columns Ui are basis for range of M for Wi != 0
  VNL::DiagMatrix<singval_t> W_;// Singular values, sorted in decreasing order
  VNL::DiagMatrix<singval_t> Winverse_;
  VNL::Matrix<T> V_;       // Columns Vi are basis for nullspace of M for Wi = 0
  unsigned rank_;
  bool have_max_;
  singval_t max_;
  bool have_min_;
  singval_t min_;
  double last_tol_;
  bool valid_;        // false if the NETLIB call failed.

  // Disallow assignment.
  SVD(SVD<T> const &) { }
  SVD<T>& operator=(SVD<T> const &) { return *this; }
};

template <class T>
inline
VNL::Matrix<T> SVDInverse(VNL::Matrix<T> const& m)
{
  return VNL::SVD<T>(m).Inverse();
}


}; // End namespace VNL



// this aint no friend.
//export readd when export is supported
template <class T>
std::ostream& operator<<(std::ostream&, VNL::SVD<T> const& svd);

#endif // vnl_svd_h_

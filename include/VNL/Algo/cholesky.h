// This is vxl/VNL/algo/cholesky.h
#ifndef vnl_cholesky_h_
#define vnl_cholesky_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Decomposition of symmetric matrix

*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   08 Dec 96
*
   \verbatim
    Modifications
    Peter Vanroose, Leuven, Apr 1998: added L() (return decomposition matrix)
    dac (Manchester) 26/03/2001: tidied up documentation
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line

     Ben Tordoff, Oxford 2003: changed to tamplate form to match all the other
     decompositions.
   \endverbatim
*/

#include <VNL/vector.h>
#include <VNL/matrix.h>


namespace VNL {

/** Decomposition of symmetric matrix.
 *  A class to hold the Cholesky decomposition of a symmetric matrix and
 *  use that to solve linear systems, compute determinants and inverses.
 *  The cholesky decomposition decomposes symmetric A = L*L.transpose()
 *  where L is lower triangular, and it is therefore possible to save a
 *  little time and pass it a matrix with only the lower triangle filled
 *  in (the upper is assumed symmetric). No test for symmetry is performed.
 *
 *  To check that the decomposition can be used safely for solving a linear
 *  equation it is wise to construct with mode==estimate_condition and
 *  check that rcond()>sqrt(machine precision).  If this is not the case
 *  it might be a good idea to use VNL::SVD instead.
 */
template <class T>
class Cholesky
{
 public:
/** Modes of computation.\  See constructor for details.
*/
  enum Operation {
    quiet,
    verbose,
    estimate_condition
  };

/** Make cholesky decomposition of M optionally computing the reciprocal condition number.
*/
  Cholesky(Matrix<T> const& M, Operation mode = verbose);
 ~Cholesky() {}

/** Solve LS problem M x = b.
*/
  Vector<T> Solve(Vector<T> const& b) const;

/** Solve LS problem M x = b.
*/
  void Solve(Vector<T> const& b, Vector<T>* x) const;

/** Compute determinant.
*/
  T Determinant() const;

/** Compute inverse.\  Not efficient.
*/
  Matrix<T> Inverse() const;

/** Return lower-triangular factor.
*/
  Matrix<T> LowerTriangle() const;

/** Return upper-triangular factor.
*/
  Matrix<T> UpperTriangle() const;

/** Return the decomposition matrix.
*/
  Matrix<T> const& LBadlyNamedMethod() { return A_; }

/** A Success/failure flag.
*/
  int RankDeficiency() const { return num_dims_rank_def_; }

/** Return reciprocal condition number (smallest/largest singular values).
* As long as rcond()>sqrt(precision) the decomposition can be used for
* solving equations safely.
* Not calculated unless Operaton mode at construction was estimate_condition.
*/
  T RCond() const { return rcond_; }

/** Return computed nullvector.
* Not calculated unless Operaton mode at construction was estimate_condition.
*/
  Vector<T>      & Nullvector()       { return nullvector_; }
  Vector<T> const& Nullvector() const { return nullvector_; }

 protected:
  // Data Members--------------------------------------------------------------
  Matrix<T> A_;
  T rcond_;
  int num_dims_rank_def_;
  Vector<T> nullvector_;

 private:
/** Copy constructor - privatised to avoid it being used.
*/
  Cholesky(Cholesky const & that);
/** Assignment operator - privatised to avoid it being used.
*/
  Cholesky& operator=(Cholesky const & that);
};

}; // End namespace VNL

#endif // Cholesky_h_

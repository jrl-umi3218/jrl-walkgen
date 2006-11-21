#ifndef _schur_h_
#define _schur_h_

#include <VNL/matrix.h>

/** 
 * \file
 * \brief This object is a wrapper around dgees to compute the schur form
 * of a matrix A.
 *
 * 2006 - Olivier Stasse
 *
 */
/*! This function computes the Schur form of A,
  and put it into T. A is supposed to be a square matrix.
  W returns the real and imaginary parts of the 
  eigenvalues as they appear on the diagonal of the output Schur form T.
  The matrix Z of Schur vectors gives the Schur factorization:
  A = Z * T * (Z**T)
  Z will have the same size than A
  @return r
  \li 0 : successful exit
  \li < 0 : if r =-i the i-th argument had an illegal value.
  \li > 0 : if r = i and i is
    <= N: The QR algorithm failed to compute all the eigenvalues; elements 1:i and i+1:N
    of W contain those eigenvalues which have converged; Z contains the matrix
    which reduces A to its partially converged Schur form;
    = N+1: the eigvenvalues could not be reordered because some eigenvalues were too close
    to separate (the problem is very ill-conditioned);    
    
    2006 Olivier Stasse.
*/


namespace VNL
{
  
  int Schur(VNL::Matrix<double> &A,
	    VNL::Matrix<double> &T,
	    VNL::Matrix<double> &W,
	    VNL::Matrix<double> &Z);
};
#endif 

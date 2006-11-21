// This is vxl/VNL/algo/chi_squared.h
#ifndef vnl_chi_squared_h_
#define vnl_chi_squared_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Name space for various chi-squared distribution functions.

* \author Rupert Curwen, GE CRD
* \date   August 18th, 1998
*
   \verbatim
   Modifications
    dac (Manchester) 26/03/2001: tidied up documentation
   \endverbatim
*/

namespace VNL {


/** Compute cumulative distribution function value for chi-squared distribution.
*/
extern double ChiSquaredCumulative(double chisq, int dof);

//------------------------------------------------------------

/** Name space for various chi-squared distribution functions.
*
*  A[] and B[] are (pointers to) arrays containing histograms.
*  If the 'normalize' parameter is true, each histogram will
*  be implicitly normalized (so as to sum to 1) before the
*  statistic is calculated :
*
*  \f$a[i] = A[i] / \sum_j A[j]\f$
*
*  \f$b[i] = B[i] / \sum_j B[j]\f$
*
*  *DO NOT* add scale factors to these functions or you will break
*  the code written by those who read the documentation. fsm.
*
* \f$\displaystyle   \sum_i \frac{ (a[i] - b[i])^2 }{ a[i] } \f$
*
*/

template <class T>
double ChiSquaredStatistic1 (T const *A, T const *B,
                                    int n, bool normalize);

/**
* \f$\displaystyle   \sum_i \frac{ (a[i] - b[i])^2 }{ b[i] } \f$
*/
template <class T>
double ChiSquaredStatistic2 (T const *A, T const *B,
                                    int n, bool normalize);

/**
* \f$\displaystyle   \sum_i \frac{ (a[i] - b[i])^2 }{ a[i] + b[i] } \f$
*/
template <class T>
double ChiSquaredStatistic12(T const *A, T const *B,
                                    int n, bool normalize);

}; // End namespace VNL

#endif // vnl_chi_squared_h_

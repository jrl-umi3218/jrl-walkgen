// This is vxl/VNL/c_vector.h
#ifndef vnl_c_vector_h_
#define vnl_c_vector_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/** \file
*  \brief Math on blocks of memory

*
*    vnl_c_vector interfaces to lowlevel memory-block operations.
*
* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   12 Feb 98
*
   \verbatim
   Modifications
       980212 AWF Initial version.
       LSB (Manchester) 26/3/01 Tidied documentation
   \endverbatim
*
*/

#include <iosfwd>
#include <VNL/numerictraits.h>

namespace VNL {

  // Keyword export not yet supported, but when it is, put this pack in
  //export template <class T> class CVector

// avoid messing about with aux_* functions for gcc 2.7 -- fsm
template <class T, class S> void CVectorOneNorm(T const *p, unsigned n, S *out);
template <class T, class S> void CVectorTwoNorm(T const *p, unsigned n, S *out);
template <class T, class S> void CVectorInfNorm(T const *p, unsigned n, S *out);
template <class T, class S> void CVectorTwoNormSquared(T const *p, unsigned n, S *out);
template <class T, class S> void CVectorRMSNorm(T const *p, unsigned n, S *out);

/** vnl_c_vector interfaces to lowlevel memory-block operations.
*/
template <class T>
class CVector
{
 public:
	 typedef typename NumericTraits<T>::abs_t abs_t;

  static T Sum(T const* v, unsigned n);
  static inline abs_t SquaredMagnitude(T const *p, unsigned n)
    { abs_t val; CVectorTwoNormSquared(p, n, &val); return val; }
  static void Normalize(T *, unsigned n);
  static void Apply(T const *, unsigned, T (*f)(T), T* v_out);
  static void Apply(T const *, unsigned, T (*f)(T const&), T* v_out);


/** y[i]  = x[i].
*/
  static void Copy    (T const *x, T       *y, unsigned);

/**  y[i]  = a*x[i].
*/
  static void Scale   (T const *x, T       *y, unsigned, T const &);

/** z[i]  = x[i] + y[i];.
*/
  static void Add     (T const *x, T const *y, T *z, unsigned);

/** z[i]  = x[i] + y;.
*/
  static void Add     (T const *x, T const& y, T *z, unsigned);

/** z[i]  = x[i] - y[i].
*/
  static void Subtract(T const *x, T const *y, T *z, unsigned);

/** z[i]  = x[i] - y[i].
*/
  static void Subtract(T const *x, T const& y, T *z, unsigned);

/** z[i]  = x[i] * y[i].
*/
  static void Multiply(T const *x, T const *y, T *z, unsigned);

/** z[i]  = x[i] * y[i].
*/
  static void Multiply(T const *x, T const& y, T *z, unsigned);

/** z[i]  = x[i] / y[i].
*/
  static void Divide  (T const *x, T const *y, T *z, unsigned);

/** z[i]  = x[i] / y[i].
*/
  static void Divide  (T const *x, T const& y, T *z, unsigned);

/** y[i]  = -x[i].
* Note that this is a no-op when T is an unsigned type.
*/
  static void Negate  (T const *x, T       *y, unsigned);

/** y[i]  = 1/x[i].
*/
  static void Invert  (T const *x, T       *y, unsigned);

/**  y[i] += a*x[i].
*/
  static void SelfPlusAx   (T const &a, T const *x, T *y, unsigned);

/** x[i]  = v.
*/
  static void Fill    (T *x, unsigned, T const &v);


  static void Reverse (T *x, unsigned);
  static T DotProduct  (T const *, T const *, unsigned);

/** conjugate second.
*/
  static T InnerProduct(T const *, T const *, unsigned);
  static void Conjugate(T const *, T *, unsigned);

  static T MaxValue(T const *, unsigned);
  static T MinValue(T const *, unsigned);
  static T Mean(T const *p, unsigned n) { return Sum(p,n)/T(n); }

/**  one_norm : sum of abs values.
*/
  static inline abs_t OneNorm(T const *p, unsigned n)
    { abs_t val; CVectorOneNorm(p, n, &val); return val; }

/** two_norm : sqrt of sum of squared abs values.
*/
  static inline abs_t TwoNorm(T const *p, unsigned n)
    { abs_t val; CVectorTwoNorm(p, n, &val); return val; }

/** inf_norm : max of abs values.
*/
  static inline abs_t InfNorm(T const *p, unsigned n)
    { abs_t val; CVectorInfNorm(p, n, &val); return val; }

/** two_nrm2 : sum of squared abs values.
*/
  static inline abs_t TwoNorm2(T const *p, unsigned n)
    { abs_t val; CVectorTwoNormSquared(p, n, &val); return val; }

/** rms_norm : sqrt of mean sum of squared abs values.
*/
  static inline abs_t RMSNorm(T const *p, unsigned n)
    { abs_t val; CVectorRMSNorm(p, n, &val); return val; }

/** Euclidean Distance between two vectors.
* Sum of Differences squared.
*/
  static T EuclidDist2(T const *, T const *, unsigned);

/** Memory allocation.
*/
  static T** AllocateTptr(int n);
  static T*  AllocateT(int n);
  static void Deallocate(T**, int n_when_allocated);
  static void Deallocate(T*, int n_when_allocated);

/** Input & output.
*/
  static std::ostream& PrintVector(std::ostream&, T const*, unsigned);
};

}; // End namespace VNL

#endif // vnl_c_vector_h_

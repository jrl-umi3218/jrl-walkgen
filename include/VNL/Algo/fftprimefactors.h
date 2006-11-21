// This is vxl/VNL/algo/fft_prime_factors.h
#ifndef vnl_fft_prime_factors_h_
#define vnl_fft_prime_factors_h_
/**
* \file
* \brief Holds prime factor information

* \author Veit U.B. Schenk, Oxford RRG
* \date   19 Mar 98
*
   \verbatim
   Modifications
   10/4/2001 Ian Scott (Manchester) Converted perceps header to doxygen
   \endverbatim
*/

 // for "export" keyword

namespace VNL {

/** Holds prime factor information.
* Helper class used by the vnl_fft_xd<> FFT routines
*
* Given an integer N of the form
*   \f$N = 2^P 3^Q 5^R\f$
* split N into its primefactors (2, 3, 5)
*/
template <class T>
struct FFTPrimeFactors
{
  FFTPrimeFactors();

/** constructor takes the size of the signal.
*/
  FFTPrimeFactors(int N) { _Construct(N); }

  ~FFTPrimeFactors () { _Destruct(); }

/** array of twiddle factors.
*/
  T const *Trigs () const { return trigs_; }

/** number which was factorized.
*/
  int Number () const { return number_; }

/** exponents P, Q, R.
*/
  int const *PQR () const { return pqr_; }

  operator bool () const { return trigs_ && info_ >= 0; }

  void Resize(int N) {
    _Destruct();
    _Construct(N);
  }

private:
  T *trigs_;
  int number_;   // the number that is being split into prime-facs
  int pqr_[3];   // store P, Q and R
  int info_;

  void _Construct(int N);
  void _Destruct();

  // disallow copying
  FFTPrimeFactors (FFTPrimeFactors<T> const &) { }
  void operator= (FFTPrimeFactors<T> const &) { }
};

}; // End namespace VNL

#endif // FFTPrimeFactors_h_

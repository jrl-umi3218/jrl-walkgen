#ifndef vnl_fft_prime_factors_t_
#define vnl_fft_prime_factors_t_
/*
  fsm@robots.ox.ac.uk
*/
#include "fftprimefactors.h"
#include <VNL/Algo/fft.h>
#include <assert.h>

template <class T>
VNL::FFTPrimeFactors<T>::FFTPrimeFactors()
  : trigs_(0)
  , number_(0)
{
}

template <class T>
void VNL::FFTPrimeFactors<T>::_Construct(int N)
{
  trigs_ = new T[2*N];
  number_ = N;
  VNL::FFTSetgpfa (trigs_, number_, pqr_, &info_);
  // info_ == -1 if cannot split into primes
  if (info_ == -1)
    assert(!"you probably gave a signal size not of the form 2^p 3^q 5^r");
}

template <class T>
void VNL::FFTPrimeFactors<T>::_Destruct()
{
  if (trigs_)
    delete [] trigs_;
}

#undef VNL_FFT_PRIME_FACTORS_INSTANTIATE
#define VNL_FFT_PRIME_FACTORS_INSTANTIATE(T) \
namespace VNL { \
  template struct FFTPrimeFactors<T >; \
};

#endif

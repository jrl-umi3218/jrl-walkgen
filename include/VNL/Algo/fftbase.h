// This is vxl/VNL/algo/fft_base.h
#ifndef vnl_fft_base_h_
#define vnl_fft_base_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief In-place n-D fast fourier transform

* \author fsm@robots.ox.ac.uk
*/

#include <complex>
#include <VNL/Algo/fftprimefactors.h>

namespace VNL {

/** Base class for in-place ND fast fourier transform.
*/

template <int D, class T>
struct FFTBase
{
  FFTBase() { }

/** dir = +1/-1 according to direction of transform.
*/
  void Transform(std::complex<T> *signal, int dir);

 protected:
/** prime factorizations of signal dimensions.
*/
  FFTPrimeFactors<T> factors_[D];
};

}; // End namespace VNL

#endif // vnl_fft_base_h_

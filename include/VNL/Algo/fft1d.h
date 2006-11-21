// This is vxl/VNL/algo/fft_1d.h
#ifndef vnl_fft_1d_h_
#define vnl_fft_1d_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief In-place 1D fast fourier transform

* \author fsm@robots.ox.ac.uk
*/

#include <VNL/vector.h>
#include <VNL/Algo/fftbase.h>

namespace VNL {

/** In-place 1D fast fourier transform.
*/

template <class T>
struct FFT1D : public FFTBase<1, T>
{
  typedef FFTBase<1, T> base;

/** constructor takes length of signal.
*/
  FFT1D(int N) {
    base::factors_[0].Resize(N);
  }

/** dir = +1/-1 according to direction of transform.
*/
  void Transform(VNL::Vector<std::complex<T> > &signal, int dir)
  { base::Transform(signal.DataBlock(), dir); }

/** forward FFT.
*/
  void FwdTransform(VNL::Vector<std::complex<T> > &signal)
  { Transform(signal, +1); }

/** backward (inverse) FFT.
*/
  void BwdTransform(VNL::Vector<std::complex<T> > &signal)
  { Transform(signal, -1); }

/** return length of signal.
*/
  unsigned size() const { return base::factors_[0].Number(); }
};

}; // End namespace VNL

#endif // vnl_fft_1d_h_

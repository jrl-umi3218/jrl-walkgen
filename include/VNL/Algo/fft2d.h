// This is vxl/VNL/algo/fft_2d.h
#ifndef vnl_fft_2d_h_
#define vnl_fft_2d_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief In-place 2D fast fourier transform

* \author fsm@robots.ox.ac.uk
*/

#include <VNL/matrix.h>
#include <VNL/Algo/fftbase.h>



namespace VNL {

/** In-place 2D fast fourier transform.
*/

template <class T>
struct FFT2D : public FFTBase<2, T>
{
  typedef FFTBase<2, T> base;

/** constructor takes size of signal.
*/
  FFT2D(int M, int N) {
    base::factors_[0].Resize(M);
    base::factors_[1].Resize(N);
  }

/** dir = +1/-1 according to direction of transform.
*/
  void Transform(VNL::Matrix<std::complex<T> > &signal, int dir)
  { base::Transform(signal.DataBlock(), dir); }

/** forward FFT.
*/
  void FwdTransform(VNL::Matrix<std::complex<T> > &signal)
  { Transform(signal, +1); }

/** backward (inverse) FFT.
*/
  void BwdTransform(VNL::Matrix<std::complex<T> > &signal)
  { Transform(signal, -1); }

/** return size of signal.
*/
  unsigned Rows() const { return base::factors_[0].Number(); }
  unsigned Cols() const { return base::factors_[1].Number(); }
};

}; // End namespace VNL

#endif // vnl_fft_2d_h_

#ifndef vnl_fft_1d_t_
#define vnl_fft_1d_t_
// -*- c++ -*-

#include "fft1d.h"

#undef VNL_FFT_1D_INSTANTIATE
#define VNL_FFT_1D_INSTANTIATE(T) \
namespace VNL { \
  template struct FFT1D<T >; \
};

#endif

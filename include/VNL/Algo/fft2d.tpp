#ifndef vnl_fft_2d_t_
#define vnl_fft_2d_t_
// -*- c++ -*-

#include "fft2d.h"

#undef VNL_FFT_2D_INSTANTIATE
#define VNL_FFT_2D_INSTANTIATE(T) \
namespace VNL { \
  template struct FFT2D<T >; \
};

#endif

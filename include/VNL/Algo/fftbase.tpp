#ifndef vnl_fft_base_t_
#define vnl_fft_base_t_
/*
  fsm@robots.ox.ac.uk
*/
#include "fftbase.h"
#include <VNL/Algo/fft.h>
#include <assert.h>

template <int D, class T>
void VNL::FFTBase<D, T>::Transform(std::complex<T> *signal, int dir)
{
  assert((dir == +1) || (dir == -1));

  // transform along each dimension, i, in turn.
  for (int i=0; i<D; ++i) {
    int N1 = 1; // n[0] n[1] ... n[i-1]
    int N2 = 1; // n[i]
    int N3 = 1; // n[i+1] n[i+2] ... n[D-1]
    for (int j=0; j<D; ++j) {
      int d = factors_[j].Number();
      if (j <  i) N1 *= d;
      if (j == i) N2 *= d;
      if (j >  i) N3 *= d;
    }

    // pretend the signal is N1xN2xN3. we want to transform
    // along the second dimension.
    for (int n1=0; n1<N1; ++n1) {
      // FIXME: we could avoid one loop by using the LOT parameter
      // but it's not entirely clear that would save us anything.

      for (int n3=0; n3<N3; ++n3) {
        // This relies on the assumption that std::complex<T> is layout
        // compatible with "struct { T real; T imag; }". It is probably
        // a valid assumption for all sane C++ libraries.
        T *data = (T *) (signal + n1*N2*N3 + n3);

        int info = 0;
        VNL::FFTgpfa (/* A */     data,
                      /* B */     data + 1,
                      /* TRIGS */ factors_[i].Trigs (),
                      /* INC */   2*N3,
                      /* JUMP */  0,
                      /* N */     N2,
                      /* LOT */   1,
                      /* ISIGN */ dir,
                      /* NIPQ */  factors_[i].PQR (),
                      /* INFO */  &info);
        assert(info != -1);
      }
    }
  }
}

#undef VNL_FFT_BASE_INSTANTIATE
#define VNL_FFT_BASE_INSTANTIATE(D, T) \
namespace VNL { \
  template struct FFTBase<D, T >; \
};

#endif

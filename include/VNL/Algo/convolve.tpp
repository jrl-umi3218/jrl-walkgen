#ifndef vnl_convolve_t_
#define vnl_convolve_t_
// This is vxl/VNL/convolve.t

#include <VNL/Algo/convolve.h>
#include <VNL/Algo/fft1d.h> // this #includes <complex>
#include <assert.h>
#include <iostream> // for warning messages

namespace VNL {
template <class T1, class T2, class U>
inline
VNL::Vector<U> ConvolveCyclicUsingFFT(VNL::Vector<T1> const& v1, VNL::Vector<T2> const& v2, U*)
{
  assert (v1.size() == v2.size());
  unsigned int n = v1.size();

  typedef std::complex<double> C;
  VNL::Vector<C> w1(n, C(0)); for (unsigned i=0; i<n; ++i) w1[i]=v1[i];
  VNL::Vector<C> w2(n, C(0)); {for (unsigned i=0; i<n; ++i) w2[i]=v2[i];}

  VNL::FFT1D<double> fft(n); fft.FwdTransform(w1); fft.FwdTransform(w2);
  {for (unsigned int i=0; i<n; ++i) w1[i] *= w2[i];}
  fft.BwdTransform(w1);
#ifdef DEBUG
  std::cout << w1 << std::endl;
#endif

  VNL::Vector<U> r(n);
  {for (unsigned int i = 0; i<n; ++i)
    r[i] = U(std::real(w1[i]) / n);} // the imaginary part is certainly zero
#ifdef DEBUG
  for (unsigned int i = 0; i<n; ++i)
    assert(std::imag(w1[i]) == 0);
#endif
  return r;
}
}; // End namespace VNL

template <class T1, class T2, class U>
VNL::Vector<U> VNL::ConvolveCyclic(VNL::Vector<T1> const& v1, VNL::Vector<T2> const& v2, U*, bool use_fft)
{
  assert (v1.size() == v2.size());
  unsigned int n = v1.size();

  // Quick return if possible:
  if (n == 0) return VNL::Vector<U>(0, U(0));
  if (n == 1) return VNL::Vector<U>(1, U(v1[0]*v2[0]));

  if (use_fft)
    return VNL::ConvolveCyclicUsingFFT(v1, v2, (U*)0);

  VNL::Vector<U> ret(n, (U)0); // all elements already initialized to zero
  for (unsigned int k=0; k<n; ++k)
  {
    for (unsigned int i=0; i<=k; ++i)
      ret[k] += U(v1[k-i]) * U(v2[i]);
    {for (unsigned int i=k+1; i<n; ++i)
      ret[k] += U(v1[n+k-i]) * U(v2[i]);
	  }
  }

  return ret;
}

static bool has_only_primefactors_2_3_5(unsigned int n)
{
  if (n <= 1) return true;
  while (n%2 == 0) n /= 2;
  while (n%3 == 0) n /= 3;
  while (n%5 == 0) n /= 5;
  return n==1;
}

namespace VNL {
template <class T1, class T2, class U>
inline
VNL::Vector<U> ConvolveUsingFFT(VNL::Vector<T1> const& v1, VNL::Vector<T2> const& v2, U*, int n)
{
  if (n+1 < int(v1.size() + v2.size())) n = v1.size() + v2.size() - 1;

  // Make sure n has only prime factors 2, 3 and 5; if necessary, increase n.
  while (!has_only_primefactors_2_3_5(n)) ++n;

  // pad with zeros, so the cyclic convolution is a convolution:
  VNL::Vector<U> w1(n, U(0)); for (unsigned i=0; i<v1.size(); ++i) w1[i]=v1[i];
  VNL::Vector<U> w2(n, U(0)); {for (unsigned i=0; i<v2.size(); ++i) w2[i]=v2[i];}
  // convolve, using n-points FFT:
  w1 = VNL::ConvolveCyclicUsingFFT(w1, w2, (U*)0);
  // return w1, but possibly drop the last few (zero) entries:
  return VNL::Vector<U>(v1.size()+v2.size()-1, v1.size()+v2.size()-1, w1.DataBlock());
}
}; // End namespace VNL

template <class T>
VNL::Vector<T> VNL::Convolve(VNL::Vector<T> const& v1, VNL::Vector<T> const& v2, int use_fft)
{
  // Quick return if possible:
  if (v1.size() == 0 || v2.size() == 0)
    return VNL::Vector<T>(0);
  if (v1.size() == 1) return v2*v1[0];
  if (v2.size() == 1) return v1*v2[0];

  if (use_fft != 0)
    return VNL::ConvolveUsingFFT(v1, v2, (T*)0, use_fft);

  unsigned int n = v1.size() + v2.size() - 1;
  VNL::Vector<T> ret(n, (T)0); // all elements already initialized to zero
  for (unsigned int k=0; k<v1.size(); ++k)
    for (unsigned int i=0; i<=k && i<v2.size(); ++i)
      ret[k] += v1[k-i] * v2[i];
  {for (unsigned int k=v1.size(); k<n; ++k)
    for (unsigned int i=k+1-v1.size(); i<=k && i<v2.size(); ++i)
      ret[k] += v1[k-i] * v2[i];}

  return ret;
}

template <class T1, class T2, class U>
VNL::Vector<U> VNL::Convolve(VNL::Vector<T1> const& v1, VNL::Vector<T2> const& v2, U*, int use_fft)
{
  // Quick return if possible:
  if (v1.size() == 0 || v2.size() == 0)
    return VNL::Vector<U>(0);

  if (use_fft != 0)
    return VNL::ConvolveUsingFFT(v1, v2, (U*)0, use_fft);

  unsigned int n = v1.size() + v2.size() - 1;
  VNL::Vector<U> ret(n, (U)0); // all elements already initialized to zero
  for (unsigned int k=0; k<v1.size(); ++k)
    for (unsigned int i=0; i<=k && i<v2.size(); ++i)
      ret[k] += U(v1[k-i]) * U(v2[i]);
	  
  {for (unsigned int k=v1.size(); k<n; ++k)
    for (unsigned int i=k+1-v1.size(); i<=k && i<v2.size(); ++i)
      ret[k] += U(v1[k-i]) * U(v2[i]);}
	  

  return ret;
}

#undef VNL_CONVOLVE_INSTANTIATE
#define VNL_CONVOLVE_INSTANTIATE_2(T,U) \
namespace VNL { \
template VNL::Vector<U > Convolve      (Vector<T > const&, Vector<U > const&, U*, int); \
template VNL::Vector<U > ConvolveCyclic(Vector<T > const&, Vector<U > const&, U*, bool); \
};

#define VNL_CONVOLVE_INSTANTIATE(T,U) \
VNL_CONVOLVE_INSTANTIATE_2(T,U); \
namespace VNL { \
template VNL::Vector<T > Convolve(Vector<T > const&, Vector<T > const&, int); \
};

#endif // vnl_convolve_t_

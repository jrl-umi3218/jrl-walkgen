// This is vxl/VNL/sample.h
#ifndef vnl_sample_h_
#define vnl_sample_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief easy ways to sample from various probability distributions

*/

namespace VNL {


/** re-seed the random number generator.
*/
void SampleReseed();

/** re-seed the random number generator given a seed.
*/
void SampleReseed(int seed);

/** uniform on [a, b).
*/
double SampleUniform(double a, double b);

/** two independent samples from a standard normal distribution.
*/
void SampleNormal2(double *x, double *y);

/** Normal distribution with given mean and standard deviation.
*/
double SampleNormal(double mean, double sigma);

// P(X = k) = [kth term in binomial expansion of (p + (1-p))^n]
//int vnl_sample_binomial(int n, int k, double p);

// ----------------------------------------



/** handy function to fill a range of values.
*/
template <class I>
inline void SampleUniform(I begin, I end, double a, double b)
{
  for (I p=begin; p!=end; ++p)
    (*p) = VNL::SampleUniform(a, b);
}

/** handy function to fill a range of values.
*/
template <class I>
inline void SampleNormal(I begin, I end, double mean, double sigma)
{
  for (I p=begin; p!=end; ++p)
    (*p) = VNL::SampleNormal(mean, sigma);
}

}; // End namespace VNL

#endif // VNL::Sample_h_

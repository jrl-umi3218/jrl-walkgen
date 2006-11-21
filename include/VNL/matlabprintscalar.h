#ifndef vnl_matlab_print_scalar_h_
#define vnl_matlab_print_scalar_h_
/*
  fsm@robots.ox.ac.uk
*/

/**
* \file
*/

#include <iosfwd>
#include <complex>
#include <VNL/matlabprintformat.h>


namespace VNL {

/** print real or complex scalar into character buffer.
*/
#define MATLAB_PRINT_SCALAR_DECLARE(T) \
  void MatlabPrintScalar(T v, \
                         char *buf, \
                         VNL::MatlabPrintFormat =VNL::matlab_print_format_default)

// Even with a function template we would have to
// forward declare all the specializations anyway.
MATLAB_PRINT_SCALAR_DECLARE(int);
MATLAB_PRINT_SCALAR_DECLARE(unsigned int);
MATLAB_PRINT_SCALAR_DECLARE(float);
MATLAB_PRINT_SCALAR_DECLARE(double);
MATLAB_PRINT_SCALAR_DECLARE(long double);
MATLAB_PRINT_SCALAR_DECLARE(std::complex<float>);
MATLAB_PRINT_SCALAR_DECLARE(std::complex<double>);
MATLAB_PRINT_SCALAR_DECLARE(std::complex<long double>);

/** print scalar to std::ostream.
*/
template <class T>
std::ostream &MatlabPrintScalar(std::ostream &,
				T value,
				VNL::MatlabPrintFormat =VNL::matlab_print_format_default);


}; // End namespace VNL

#endif

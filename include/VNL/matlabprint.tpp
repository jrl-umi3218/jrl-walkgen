#ifndef vnl_matlab_print_t_
#define vnl_matlab_print_t_

// This is vxl/VNL/matlabprint.tcc
// It is different from vxl/VNL/matlab_print.cxx

/*
  fsm@robots.ox.ac.uk
*/
// Adapted from awf's MatOps class.

#include "VC6WarningsFix.h"

#include "matlabprint.h"

#include <iostream>

#include <VNL/vector.h>
#include <VNL/matrix.h>
#include <VNL/diagmatrix.h>
#include <VNL/matlabprintscalar.h>

//--------------------------------------------------------------------------------

namespace VNL {

template <class T>
std::ostream &MatlabPrint(std::ostream& s,
                              T const* array,
                              unsigned length,
                              VNL::MatlabPrintFormat format)
{
  char buf[1024];
  for (unsigned j=0; j<length; j++ ) {
    // Format according to selected style
    // In both cases an exact 0 goes out as such
    VNL::MatlabPrintScalar(array[j], buf, format);
    s << buf;
  }

  return s;
}

template <class T>
std::ostream &MatlabPrint(std::ostream &s,
                              T const * const *array,
                              unsigned rows, unsigned cols,
                              VNL::MatlabPrintFormat format)
{
  for (unsigned i=0; i<rows; ++i)
    VNL::MatlabPrint(s, array[i], cols, format) << std::endl;
  return s;
}

template <class T>
std::ostream& MatlabPrint(std::ostream& s,
                              VNL::DiagMatrix<T> const& D,
                              char const* variable_name,
                              VNL::MatlabPrintFormat format)
{
  if (variable_name)
    s << variable_name << " = diag([ ";

  VNL::MatlabPrint(s, D.begin(), D.size(), format);

  if (variable_name) {
    s << " ])";
    s << std::endl;
  }

  return s;
}

template <class T>
std::ostream& MatlabPrint(std::ostream& s,
                              VNL::Matrix<T> const& M,
                              char const* variable_name,
                              VNL::MatlabPrintFormat format)
{
  if (variable_name)
    s << variable_name << " = [ ...\n";

  if (M.Rows() == 0)
    return s << "];\n";

  for (unsigned int i=0; i<M.Rows(); i++ ) {
    VNL::MatlabPrint(s, M[i], M.Columns(), format);

    if (variable_name && (i == M.Rows()-1))
      s << " ]";

    s << std::endl;
  }

  return s;
}

template <class T>
std::ostream& MatlabPrint(std::ostream& s,
                              VNL::Vector<T> const & v,
                              char const* variable_name,
                              VNL::MatlabPrintFormat format)
{
  if (variable_name)
    s << variable_name << " = [ ";

  VNL::MatlabPrint(s, v.begin(), v.size(), format);

  if (variable_name) {
    s << " ]";
    s << std::endl;
  }

  return s;
}

}; // End namespace VNL

//--------------------------------------------------------------------------------

#undef  VNL_MATLAB_PRINT_INSTANTIATE
#define VNL_MATLAB_PRINT_INSTANTIATE(T) \
namespace VNL { \
template std::ostream &MatlabPrint(std::ostream &, T const *, unsigned, VNL::MatlabPrintFormat); \
template std::ostream &MatlabPrint(std::ostream &, T const * const *, unsigned, unsigned, VNL::MatlabPrintFormat); \
template std::ostream &MatlabPrint(std::ostream &, VNL::DiagMatrix<T > const &, char const *, VNL::MatlabPrintFormat); \
template std::ostream &MatlabPrint(std::ostream &, VNL::Matrix<T > const &, char const *, VNL::MatlabPrintFormat); \
template std::ostream &MatlabPrint(std::ostream &, VNL::Vector<T > const &, char const *, VNL::MatlabPrintFormat); \
}

#endif

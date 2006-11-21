// This is vxl/VNL/matlab_print.h
#ifndef vnl_matlab_print_h_
#define vnl_matlab_print_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/** \file
*  \brief Print matrices and vectors in nice MATLAB format.

*  \author fsm@robots.ox.ac.uk, from awf's MatOps code.
*/

#include <iosfwd>

#include <VNL/matlabprintformat.h>

namespace VNL {
template <class T> class Vector;
template <class T> class Matrix;
template <class T> class DiagMatrix;


// If a variable name (e.g. "foo") is given, the raw data will be preceded by
//   "foo = diag([ " for a vnl_diag_matrix
//   "foo = [ ...\n" for a vnl_matrix and
//   "foo = [ "      for a vnl_vector
// and followed by "])\n", "]\n" and "]\n" respectively. If the variable name
// is a null pointer, the data is printed as is.

//-------------------- "unnamed" forms.

/** print a 1D array.
*/
template <class T>
std::ostream &MatlabPrint(std::ostream &,
			  T const *array,
			  unsigned length,
			  VNL::MatlabPrintFormat =VNL::matlab_print_format_default);

/** print a 2D array.
*/
template <class T>
std::ostream &MatlabPrint(std::ostream &,
                              T const * const *array,
                              unsigned rows, unsigned cols,
                              VNL::MatlabPrintFormat =VNL::matlab_print_format_default);

//-------------------- "named" forms.

/** print a vnl_diagonal_matrix<>.
*/
template <class T>
std::ostream &MatlabPrint(std::ostream &,
			  VNL::DiagMatrix<T> const &,
			  char const *variable_name =0,
			  VNL::MatlabPrintFormat =VNL::matlab_print_format_default);

/** print a vnl_matrix<>.
*/
template <class T>
std::ostream &MatlabPrint(std::ostream &,
			  VNL::Matrix<T> const &,
			  char const *variable_name =0,
			  VNL::MatlabPrintFormat =VNL::matlab_print_format_default);

/** print a vnl_vector<>.
*/
template <class T>
std::ostream &MatlabPrint(std::ostream &,
			  VNL::Vector<T> const &,
			  char const *variable_name =0,
			  VNL::MatlabPrintFormat =VNL::matlab_print_format_default);


}; // End namespace VNL

/** naughty naming-convention-defying-but-handy macro.
*/
#define MATLABPRINT(X) (VNL::MatlabPrint(std::cerr, X, #X))

#endif // vnl_matlab_print_h_

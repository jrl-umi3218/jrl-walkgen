// This is vxl/VNL/file_matrix.h
#ifndef vnl_file_matrix_h_
#define vnl_file_matrix_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/** \file
* \brief Load vnl_matrix<double> from file

*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   23 Dec 96
*
   \verbatim
   Modifications:
   LSB (Manchester) 23/3/01 Documenation tidied
   \endverbatim
*
*/

#include <VNL/matrix.h>

namespace VNL {

/** Class to load a matrix from a file.
*/
template <class T>
class FileMatrix : public Matrix<T>
{
 public:
  FileMatrix(char const* filename);

  operator bool() const { return ok_; }

 private:
  bool ok_;
};

}; // End namespace VNL

#endif // FileMatrix_h_

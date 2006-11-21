// This is ./vxl/VNL/file_matrix.t
#ifndef vnl_file_matrix_t_
#define vnl_file_matrix_t_

/**
* \file
*
* \author Andrew W. Fitzgibbon, Oxford RRG
* \date   23 Dec 96
*
*/

#include "filematrix.h"
#include <iostream>
#include <fstream>
#include <cstring> // for strcmp()

/** Load matrix from filename.
*/
template <class T>
VNL::FileMatrix<T>::FileMatrix(char const* filename)
{
  if (filename && strcmp(filename, "-")) {
    std::ifstream o(filename);
    ok_=Matrix<T>::ReadASCII(o);
    if (!ok_)
      std::cerr << "FileMatrix: ERROR loading " << filename << std::endl;
  }
  else {
    ok_=Matrix<T>::ReadASCII(std::cin);
    if (!ok_)
      std::cerr << "FileMatrix: ERROR loading from stdin " << std::endl;
  }
}

//--------------------------------------------------------------------------------

#undef VNL_FILE_MATRIX_INSTANTIATE
#define VNL_FILE_MATRIX_INSTANTIATE(T) namespace VNL { template class FileMatrix<T >; };

#endif // vnl_file_matrix_t_

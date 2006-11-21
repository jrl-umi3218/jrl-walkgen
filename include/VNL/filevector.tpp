// This is ./vxl/VNL/file_vector.t
#ifndef vnl_file_vector_t_
#define vnl_file_vector_t_

/**
* \file
*/

#include "filevector.h"

#include <iostream>
#include <fstream>
#include <cstring> // for strcmp()

/** Load vector from filename.
*/
template <class T>
VNL::FileVector<T>::FileVector(char const* filename)
  : VNL::Vector<T>() // makes an empty vector.
{
  //std::cerr << "filename=" << filename << std::endl;
  //std::cerr << "length=" << this->length() << std::endl;
  if (filename && strcmp(filename, "-")) {
    std::ifstream o(filename);
    ok_=Vector<T>::ReadASCII(o);
  }
  else
    ok_=Vector<T>::ReadASCII(std::cin);
  //std::cerr << "length=" << this->length() << std::endl;
}

//--------------------------------------------------------------------------------

#undef VNL_FILE_VECTOR_INSTANTIATE
#define VNL_FILE_VECTOR_INSTANTIATE(T) namespace VNL {template class FileVector<T >; };

#endif // vnl_file_vector_t_

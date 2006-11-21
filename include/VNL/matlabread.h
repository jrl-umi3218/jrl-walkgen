// This is vxl/VNL/matlab_read.h
#ifndef vnl_matlab_read_h_
#define vnl_matlab_read_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Read from MATLAB files

* \author fsm@robots.ox.ac.uk
*
   \verbatim
   Modifications
   LSB (Manchester) 23/3/01 documentation tidied
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*/

#include <iosfwd>
#include <complex>
#include <VNL/matlabheader.h>

// ------------------------------ easy ------------------------------

namespace VNL {
template <class T> class Vector;
template <class T> class Matrix;


/** Attempt to read vector or matrix.
* If the MATLAB header cannot be read, return false.
* Else, if a name is given, and it doesn't match what's in the file, abort().
* If the data in the file cannot reasonably be read into the destination, abort().
*
* The vector/matrix will be resized if necessary.
*/
template <class T> bool MatlabReadOrDie(std::istream &, Vector<T> &, char const *name =0);
template <class T> bool MatlabReadOrDie(std::istream &, Matrix<T> &, char const *name =0);

// ------------------------------ less easy ------------------------------

/** MATLAB stores its data as a real block followed by an imaginary block.
* This function will read both blocks and interleave them into the area
* pointed to by ptr. For real T, it is equivalent to s.read(ptr, sizeof(T)*n);
*/
template <class T> void MatlabReadData(std::istream &s, T *ptr, unsigned n);

class MatlabReadHdr
{
 public:
  MatlabReadHdr(std::istream &);
  ~MatlabReadHdr();

  operator bool () const;
  void ReadNext(); // skip to next header in file

  bool IsSingle() const;
  bool IsRowwise() const;
  bool IsBigendian() const; // don't use this
  long Rows() const { return hdr.rows; }
  long Cols() const { return hdr.cols; }
  bool IsComplex() const { return hdr.imag != 0; }
  char const *Name() const { return varname; }

  // bah! no member templates
  //template <class T> bool read_data(T &); // scalar
  //template <class T> bool read_data(T *); // vector
  //template <class T> bool read_data(T * const *); // 2D array
#define fsm_declare_methods(T) \
 private: \
  bool TypeChck(T &); \
 public: \
  bool ReadData(T &); \
  bool ReadData(T *); \
  bool ReadData(T * const *) // no ; here, please. SunPro 5.0 barfs.
fsm_declare_methods(float);
fsm_declare_methods(double);
fsm_declare_methods(std::complex<float>);
fsm_declare_methods(std::complex<double>);
#undef fsm_declare_methods

 private:
  std::istream &s;
  MatlabHeader hdr;
  char *varname;
  bool data_read;

  void _ReadHdr(); // internal work routine
};

}; // End namespace VNL

#endif // vnl_matlab_read_h_

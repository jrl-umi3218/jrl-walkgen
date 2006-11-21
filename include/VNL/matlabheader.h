// This is vxl/VNL/matlab_header.h
#ifndef vnl_matlab_header_h_
#define vnl_matlab_header_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/** \file
*  \brief MATLAB header structure

*  \author fsm@robots.ox.ac.uk
*/

namespace VNL {

struct MatlabHeader
{
  long type; // sum of one byte order, one storage specifier and one precision specifier
  long rows;
  long cols;
  long imag;
  long namlen;

  enum type_t {
    // precision specifier
    VNL_DOUBLE_PRECISION = 0,
    VNL_SINGLE_PRECISION = 10,
    // storage specifier
    VNL_COLUMN_WISE = 0,
    VNL_ROW_WISE    = 100,
    // byte order
    VNL_LITTLE_ENDIAN = 0,
    VNL_BIG_ENDIAN    = 1000,
    //
    VNL_NONE = 0
  };
};

}; // End namespace VNL

#endif // _vnl_matlab_header_h_

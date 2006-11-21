// This is vxl/VNL/error.h
#ifndef vnl_error_h_
#define vnl_error_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/** \file
*  \brief

*  \author fsm@robots.ox.ac.uk
*/

namespace VNL {


//
extern void ErrorVectorIndex (const char* fcn, int index);
extern void ErrorVectorDimension (const char* fcn, int l1, int l2);
extern void ErrorVectorVaArg (int n);

//
extern void ErrorMatrixRowIndex (char const* fcn, int r);
extern void ErrorMatrixColIndex (char const* fcn, int c);
extern void ErrorMatrixDimension (char const* fcn, int r1, int c1, int r2, int c2);
extern void ErrorMatrixNonSquare (char const* fcn);
extern void ErrorMatrixVaArg (int n);

};

#endif // vnl_error_h_

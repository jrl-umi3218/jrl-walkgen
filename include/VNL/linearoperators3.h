// This is vxl/VNL/linear_operators_3.h
#ifndef vnl_linear_operators_3_h_
#define vnl_linear_operators_3_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief 3D linear algebra operations

*
*    Specialized linear operators for 3D vectors and matrices.
*    Include this file if you're inlining or compiling linear algebra
*    code for speed.
*
*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   04 Aug 96
*
   \verbatim
   Modifications:
   LSB (Manchester) 23/3/01 Tidied documentation
   \endverbatim
*/

#include <VNL/matrixfixed.h>
//#include <VNL/vectorfixed.h>

/** The binary multiplication operator.
*/
/*
inline
VNL::VectorFixed<3,double> operator* (const VNL::MatrixFixed<3,3,double>& A, 
				      const VNL::VectorFixed<3,double>& x)
{
  const double* a = A.DataBlock();
  double r0 = a[0] * x[0] + a[1] * x[1] + a[2] * x[2];
  double r1 = a[3] * x[0] + a[4] * x[1] + a[5] * x[2];
  double r2 = a[6] * x[0] + a[7] * x[1] + a[8] * x[2];
  return VNL::VectorFixed<3,double>(r0, r1, r2);
}
*/

/** The binary addition operator.
*/
/*
inline
VNL::VectorFixed<3,double> operator+ (const VNL::VectorFixed<3,double>& a, 
				      const VNL::VectorFixed<3,double>& b)
{
  double r0 = a[0] + b[0];
  double r1 = a[1] + b[1];
  double r2 = a[2] + b[2];
  return VNL::VectorFixed<3,double>(r0, r1, r2);
}
*/

#endif // vnl_linear_operators_3_h_

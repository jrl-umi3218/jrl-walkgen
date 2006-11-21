// This is vxl/VNL/fastops.h
#ifndef vnl_fastops_h_
#define vnl_fastops_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Collection of C-style matrix functions

*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   09 Dec 96
*
   \verbatim
     Modifications
     LSB (Manchester) 23/3/01 Tidied documentation
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*/

#include <VNL/vector.h>
#include <VNL/matrix.h>

/** Collection of C-style matrix functions for the most time-critical applications.
* In general, however one should consider using the vnl_transpose envelope-letter
* class to achieve the same results with about a 10% speed penalty.
*/
class vnl_fastops
{
 public:
  static void AtA(const VNL::Matrix<double>& A, VNL::Matrix<double>* out);

  static void AB(const VNL::Matrix<double>& A, const VNL::Matrix<double>& B, VNL::Matrix<double>* out);
  static void AtB(const VNL::Matrix<double>& A, const VNL::Matrix<double>& B, VNL::Matrix<double>* out);
  static void AtB(const VNL::Matrix<double>& A, const VNL::Vector<double>& B, VNL::Vector<double>* out);
  static void ABt(const VNL::Matrix<double>& A, const VNL::Matrix<double>& B, VNL::Matrix<double>* out);

  static void inc_X_by_AtB(VNL::Matrix<double>& X, const VNL::Matrix<double>& A, const VNL::Matrix<double>& B);
  static void inc_X_by_AtB(VNL::Vector<double>& X, const VNL::Matrix<double>& A, const VNL::Vector<double>& B);
  static void inc_X_by_AtA(VNL::Matrix<double>& X, const VNL::Matrix<double>& A);

  static void dec_X_by_AtB(VNL::Matrix<double>& X, const VNL::Matrix<double>& A, const VNL::Matrix<double>& B);
  static void dec_X_by_AtA(VNL::Matrix<double>& X, const VNL::Matrix<double>& A);

  static void dec_X_by_ABt(VNL::Matrix<double>& X, const VNL::Matrix<double>& A, const VNL::Matrix<double>& B);

  // BLAS-like operations
  static double dot(const double* a, const double* b, int n);
};

#endif // vnl_fastops_h_

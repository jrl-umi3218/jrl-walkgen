// This is vxl/VNL/matops.h
#ifndef vnl_matops_h_
#define vnl_matops_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/** \file
*  \brief A collection of Matrix operations

*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   05 Aug 96
*
   \verbatim
    Modifications:
     23 may 97, Peter Vanroose - "NO_COMPLEX" option added
     LSB (Manchester) 23/3/01 Documentation tidied
   \endverbatim
*
*/

#include <VNL/vector.h>
#include <VNL/matrix.h>
/**   A collection of Matrix operations.
*    mostly declared as static methods.
*    Highlights include the in-place transpose, and type conversions.
*    matlab_print has been moved to vnl_matlab_print.h.
*/
class vnl_matops
{
 public:
  static double homg_diff(VNL::Matrix<double> const& A, VNL::Matrix<double> const& B);

/** Laminating.
*/
  static VNL::Matrix<double> cat(VNL::Matrix<double> const& A, VNL::Matrix<double> const& B);
  static VNL::Matrix<double> cat(VNL::Matrix<double> const& A, VNL::Vector<double> const& B);
  static VNL::Matrix<double> cat(VNL::Vector<double> const& A, VNL::Matrix<double> const& B);

  static VNL::Matrix<double> vcat(VNL::Matrix<double> const& A, VNL::Matrix<double> const& B);

/** Conversions.
*/
  static VNL::Matrix<double> f2d(VNL::Matrix<float> const&);
  static VNL::Vector<double> f2d(VNL::Vector<float> const&);
  static VNL::Matrix<float>  d2f(VNL::Matrix<double> const&);
  static VNL::Vector<float>  d2f(VNL::Vector<double> const&);
};

#endif // vnl_matops_h_

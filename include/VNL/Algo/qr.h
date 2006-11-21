// This is vxl/VNL/algo/qr.h
#ifndef vnl_qr_h_
#define vnl_qr_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Calculate inverse of a matrix using QR

*  \author  Andrew W. Fitzgibbon, Oxford RRG
*  \date   08 Dec 96
*
   \verbatim
    Modifications:
    081296 AWF Temporarily abandoned as I realized my problem was symmetric...
    080697 AWF Recovered, implemented solve().
    200897 AWF Added determinant().
    071097 AWF Added Q(), R().
    Christian Stoecklin, ETH Zurich, added QtB(v)
    31-mar-2000 fsm@robots.ox.ac.uk: templated
    dac (Manchester) 28/03/2001: tidied up documentation
   \endverbatim
*/

#include <VNL/vector.h>
#include <VNL/matrix.h>

namespace VNL {


// Uncomment this when export is working
//export template <class T> class QR;


/** Extract the Q*R decomposition of matrix M.
*  The decomposition is stored in a compact and time-efficient
* packed form, which is most easily used via the "solve" and
* "determinant" methods.
*/


template <class T>
class QR
{
 public:
  QR(Matrix<T> const & M);
 ~QR();

  //not implemented? PMN //Matrix<T> Inverse () const;   // inverse
  //not implemented? PMN //Matrix<T> TInverse () const;  // transpose-inverse
  //not implemented? PMN //Matrix<T> Recompose () const;

   //not implemented? PMN //Matrix<T> Solve (const Matrix<T>& rhs) const;
  Vector<T> Solve (const Vector<T>& rhs) const;

  T Determinant() const;
  Matrix<T>& Q();
  Matrix<T>& R();
  Vector<T> QtB(const Vector<T>& b) const;

   //not implemented? PMN // void ExtractQAndR(Matrix<T>* Q, Matrix<T>* R);

 private:
  Matrix<T> qrdc_out_;
  Vector<T> qraux_;
  Vector<int> jpvt_;
  Matrix<T>* Q_;
  Matrix<T>* R_;

  // Disallow assignment.
  QR(const QR<T> &) { }
  void operator=(const QR<T> &) { }
};

/** Compute determinant of matrix "M" using QR.
*/
template <class T>
inline T QRDeterminant(Matrix<T> const& m)
{
  return QR<T>(m).Determinant();
}

// uncomment the export command when it works
//export 
template <class T>
std::ostream& operator<<(std::ostream&, QR<T> const & qr);

}; // End namespace VNL

#endif // QR_h_

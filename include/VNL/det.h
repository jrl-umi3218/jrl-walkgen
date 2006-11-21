// This is vxl/VNL/det.h
#ifndef vnl_det_h_
#define vnl_det_h_
/** \file
*  \brief Direct evaluation of 2x2, 3x3 and 4x4 determinants.

*  \author fsm@robots.ox.ac.uk
*
   \verbatim
    Modifications
    Peter Vanroose - 15 Oct. 2001 - Renamed from vnl_determinant to vnl_det
    Peter Vanroose - 15 Oct. 2001 - Added vnl_matrix_fixed interface
   \endverbatim
*/

#include <VNL/matrixfixed.h>

namespace VNL {

/** 2x2 matrix.
*/
template <class T> T Det(T const *row0,
                             T const *row1);

/** 3x3 matrix.
*/
template <class T> T Det(T const *row0,
                             T const *row1,
                             T const *row2);

/** 4x4 matrix.
*/
template <class T> T Det(T const *row0,
                             T const *row1,
                             T const *row2,
                             T const *row3);

template <class T>
inline T Det(MatrixFixed<1,1,T> const& m) { return m[0][0]; }

template <class T>
inline T Det(MatrixFixed<2,2,T> const& m) { return Det(m[0],m[1]); }

template <class T>
inline T Det(MatrixFixed<3,3,T> const& m) { return Det(m[0],m[1],m[2]); }

template <class T>
inline T Det(MatrixFixed<4,4,T> const& m) { return Det(m[0],m[1],m[2],m[3]); }

}; // End namespace VNL

#endif // vnl_det_h_

// This is vxl/VNL/trace.h
#ifndef vnl_trace_h_
#define vnl_trace_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Calculate trace of a matrix

*  \author fsm@robots.ox.ac.uk
*
   \verbatim
   Modifications
   LSB (Manchester) 19/3/01 Documentation tidied
   \endverbatim
*/

namespace VNL {

template <class T> class Matrix;

/** Calculate trace of a matrix.
*/
template <class T>
T Trace(VNL::Matrix<T> const &);

}; // End namespace VNL

#endif // vnl_trace_h_

// This is vxl/VNL/algo/powell.h
#ifndef vnl_powell_h_
#define vnl_powell_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief Powell minimizer.

* \author awf@robots.ox.ac.uk
* \date   05 Dec 00
*/

#include <VNL/costfunction.h>
#include <VNL/nonlinearminimizer.h>

namespace VNL {

/** The ever-popular Powell minimizer.
* Derivative-free method which may be faster if your
* function is expensive to compute and many-dimensional.
* Implemented from scratch from NR.
*/
class Powell : public NonlinearMinimizer
{
 public:

/** Initialize a powell with the given cost function.
*/
  Powell(CostFunction* functor);

/** Run minimization, place result in x.
*/
  ReturnCodes Minimize(Vector<double>& x);

 protected:
  CostFunction* functor_;

  friend class Powell1DFun;
  void _PubReportEval(double e) {
    _ReportEval(e);
  }
};

}; // End namespace VNL

#endif // vnl_powell_h_

// This is vxl/VNL/matlab_print2.h
#ifndef vnl_matlab_print2_h_
#define vnl_matlab_print2_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
*
* After including this header file, the client should be able to say :
* \code
*   vnl_matrix<double> foo(3, 14);
*  ....
*   std::cerr << "and the blasted matrix is :" << endl
*            << vnl_matlab_print(foo)
*            << vnl_matlab_print(foo, "foo")
*            << vnl_matlab_print(foo, 0, vnl_matlab_fmt_long);
* \endcode
* instead of
* \code
*  ....
*   std::cerr << "and the blasted matrix is :" << endl;
*   vnl_matlab_print(std::cerr, foo);
*   vnl_matlab_print(std::cerr, foo, "foo");
*   vnl_matlab_print(std::cerr, foo, 0, vnl_matlab_fmt_long);
* \endcode
*
*  \author fsm@robots.ox.ac.uk
*/

#include <VNL/matlabprint.h>


namespace VNL {

// The proxy classes.
template <class T>
struct MatlabPrintProxy
{
  T const &obj;
  char const *name;
  MatlabPrintFormat format;
  MatlabPrintProxy(T const &obj_,
		   char const *name_,
		   MatlabPrintFormat format_)
    : obj(obj_), name(name_), format(format_) { }
  ~MatlabPrintProxy() { }
};

}; // End namespace VNL

// Output operator for the proxies.
template <class T>
inline
std::ostream &operator<<(std::ostream &os, 
			 VNL::MatlabPrintProxy<T> const &mp)
{
  return VNL::MatlabPrint(os, mp.obj, mp.name, mp.format);
}

namespace VNL {

  // Functions to make proxies. This should work for objects of types
  // derived from vnl_vector, vnl_matrix etc because the overload
  // resolution is done in the operator<< above.
  template <class T>
inline
  MatlabPrintProxy<T>
  MatlabPrint(T const &obj,
	      char const *name = 0,
		  VNL::MatlabPrintFormat format = VNL::matlab_print_format_default)
  {
    return VNL::MatlabPrintProxy<T>(obj, name, format);
  }
  
}; // End namespace VNL

#define VNL_MATLAB_PRINT2_INSTANTIATE(T) \
namespace VNL { template struct MatlabPrintProxy<T >; }; \
template std::ostream & \
    operator<<(std::ostream &, VNL::MatlabPrintProxy<T > const &); \
namespace VNL { template MatlabPrintProxy<T > \
    MatlabPrint(T const &, char const *, MatlabPrintFormat); };

#endif // vnl_matlab_print2_h_

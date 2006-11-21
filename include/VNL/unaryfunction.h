// This is vxl/VNL/unary_function.h
#ifndef vnl_unary_function_h_
#define vnl_unary_function_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief Abstract 1D map

*
*  vnl_unary_function is an abstract map between two types (read spaces).
*
*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   28 Nov 98
*
   \verbatim
   Modifications
    981128 AWF Initial version.
    LSB Manchester 19/3/01 Documentation tidied
     Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
   \endverbatim
*
*/

#include <VNL/numericlimits.h>


namespace VNL {

/** Abstract 1D map between two types (read spaces).
*/
template <class Return, class Argument>
class UnaryFunction
{
 public:
  typedef NumericLimits<Return> limits;

/** Apply the function.
* The name is "f" rather than operator(), as the function will generally be
* called through a pointer.  Note that the function is NOT const when you subclass.
*/
  virtual Return F(Argument const& i) = 0;
#if 0
/** Return bounding cube of domain (inputs).
* Removed by awf simply to avoid implementing vnl_numeric_limits<vnl_vector<T> >.
*/
  virtual Argument GetDomainMin() const { return NumericLimits<Argument>::min(); }
  virtual Argument GetDomainMax() const { return NumericLimits<Argument>::max(); }
#endif
/** Return bounding cube of range (outputs).
*/
  virtual Return GetRangeMin() const { return NumericLimits<Return >::min(); }
  virtual Return GetRangeMax() const { return limits::max(); }

/** Copy should allocate a copy of this on the heap and return it.
* If Subclasses do not implement this function, it will return null, but many
* applications will never call it, so this may not be a problem for you.
*/
  virtual UnaryFunction<Return,Argument>* Copy() const { return 0; }

  virtual ~UnaryFunction() {}
};


}; // End namespace VNL

#endif // vnl_unary_function_h_

// This is vxl/VNL/vector_ref.h
#ifndef _vnl_vector_ref_h_
#define _vnl_vector_ref_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
*  \file
*  \brief vnl_vector using user-supplied storage

*  \author Andrew W. Fitzgibbon, Oxford RRG
*  \date   04 Aug 96
*
   \verbatim
   Modifications
   LSB (Manchester) 19/03/2001: Tidied up the documentation
   \endverbatim
*/

#include <VNL/vector.h>


namespace VNL {
  // Export not yet supported - reintroduce this when it is
  //  export template <class T> class VectorRef;


/** vnl_vector using user-supplied storage.
*   vnl_vector for which the data space has
*   been supplied externally.
*/
template <class T>
class VectorRef : public Vector<T>
{
 public:
  typedef Vector<T> Base;

/** Constructor.
* Do *not* call anything else than the default constructor of VNL::Vector<T>
*/
  VectorRef(int n, T *space) : Vector<T>() {
    Base::data = space;
    Base::num_elmts = n;
  }

/** Copy constructor.
* Do *not* call anything else than the default constructor of VNL::Vector<T>
* (That is why the default copy constructor is *not* good.)
*/
  VectorRef(VectorRef<T> const& v) : VNL::Vector<T>() {
    Base::data = (T*)(v.DataBlock()); // const incorrect!
    Base::num_elmts = v.size();
  }

/** Destructor.
* Prevents base destructor from releasing memory we don't own
*/
  ~VectorRef() {
    Base::data = 0;
  }

 private:

/** Copy constructor from VNL::Vector<T> is disallowed:.
*/
  VectorRef(VNL::Vector<T> const&) {}
 
#if 0 // NOW COMMENTED OUT - PVR, may 97
  // Private operator new because deleting a pointer to
  // one of these through a baseclass pointer will attempt
  // to free the referenced memory.
  // Therefore disallow newing of these -- if you're paying for
  // one malloc, you can afford two.
  void* operator new(size_t) { return 0; }

 public:
  // Privatizing other new means we must offer placement new for STL
  void* operator new(size_t, void* space) { return space; }
#endif
};


}; // End namespace VNL

#endif // VNL::Vector_ref_h_

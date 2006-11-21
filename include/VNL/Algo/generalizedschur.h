// This is vxl/VNL/algo/generalized_schur.h
#ifndef vnl_generalized_schur_h_
#define vnl_generalized_schur_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
/**
* \file
* \brief  Solves the generalized eigenproblem det(t A - s B) = 0.

* \author fsm, Oxford RRG
* \date   2 Oct 2001
*
   \verbatim
   Modifications:
   \endverbatim
*/

#include <algorithm> // for copy
#include <VNL/matrix.h>
#include <VNL/vector.h>


namespace VNL {

// For a *real* scalar type T, this function computes orthogonal L, R such that
//
  template <class T>
    bool GeneralizedSchur(VNL::Matrix<T> *A,
			  VNL::Matrix<T> *B,
			  VNL::Vector<T> *alphar,
			  VNL::Vector<T> *alphai,
			  VNL::Vector<T> *beta,
			  VNL::Matrix<T> *L,
			  VNL::Matrix<T> *R);
  
  template <>
    bool GeneralizedSchur(VNL::Matrix<double> *A,
			  VNL::Matrix<double> *B,
			  VNL::Vector<double> *alphar,
			  VNL::Vector<double> *alphai,
			  VNL::Vector<double> *beta,
			  VNL::Matrix<double> *L,
			  VNL::Matrix<double> *R);
  
  template <class T>
    inline bool GeneralizedSchur(VNL::Matrix<T> *A,
				 VNL::Matrix<T> *B,
				 VNL::Vector<T> *alphar,
				 VNL::Vector<T> *alphai,
				 VNL::Vector<T> *beta,
				 VNL::Matrix<T> *L,
				 VNL::Matrix<T> *R)
    {
      VNL::Matrix<double> A_(A->Rows(), A->Cols());
      VNL::Matrix<double> B_(B->Rows(), B->Cols());
      std::copy(A->begin(), A->end(), A_.begin());
      std::copy(B->begin(), B->end(), B_.begin());
      
      VNL::Vector<double> alphar_;
      VNL::Vector<double> alphai_;
      VNL::Vector<double> beta_;
      VNL::Matrix<double> L_;
      VNL::Matrix<double> R_;
      
      if (! GeneralizedSchur/*<double>*/(&A_, &B_, 
					 &alphar_, &alphai_, 
					 &beta_, &L_, &R_))
	return false;

      std::copy(A_.begin(), A_.end(), A->begin());
      std::copy(B_.begin(), B_.end(), B->begin());

      alphar->resize(alphar_.size()); std::copy(alphar_.begin(), alphar_.end(), alphar->begin());
      alphai->resize(alphai_.size()); std::copy(alphai_.begin(), alphai_.end(), alphai->begin());
      beta  ->resize(beta_  .size()); std::copy(beta_  .begin(), beta_  .end(), beta  ->begin());
      L->resize(L_.Rows(), L_.Cols()); std::copy(L_.begin(), L_.end(), L->begin());
      R->resize(R_.Rows(), R_.Cols()); std::copy(R_.begin(), R_.end(), R->begin());
      
      return true;
    }

} // End namespace VNL

#endif // vnl_generalized_schur_h_

/**
 * This header file intend to define an abstraction layer
 * to allow the use of diverse matrix libraries.
 * This is supposed to solve the dependency to a particular
 * matrix library.
 *
 * This specific file implements a wrapping around the
 * Boost matrices.
 *
 * (c) 2006 , Olivier Stasse JRL-Japan, CNRS-AIST, ISRI.
 */


#ifdef _BOOST_MATRIX_

/*!
 * @defgroup Boost
 * @ingroup linearalgebra
 * 
 */

#include <cstring>
//#include <boost/numeric/bindings/atlas/clapack.hpp>
#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/traits/std_vector.hpp>
#include <boost/numeric/bindings/traits/std_vector.hpp>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/bindings/lapack/gesvd.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

#include <f2c.h>
//#include <cblas.h>
//#include <clapack.h>

namespace ublas = boost::numeric::ublas;
//namespace atlas = boost::numeric::bindings::atlas;
namespace traits = boost::numeric::bindings::traits;
namespace lapack = boost::numeric::bindings::lapack;

#define MAL_VECTOR(name, type)	\
  ublas::vector<type> name

#define MAL_VECTOR_DIM(name, type, nb_rows)	\
  ublas::vector<type> name(nb_rows)

#define MAL_VECTOR_SIZE(name) \
  name.size()

#define MAL_VECTOR_RESIZE(name, nb_rows) \
  name.resize(nb_rows)

#define MAL_VECTOR_FILL(name, value) \
  { for(unsigned int i=0;i<name.size();name[i++]=value);}

#define MAL_VECTOR_NORM(name) \
  ublas::norm_2(name)

#define MAL_VECTOR_3D_CROSS_PRODUCT(res,v1,v2)	\
  if ((v1.size()==3) && (v2.size()==3))		\
    { \
      res[0] = v1[1] * v2[2] - v2[1] * v1[2]; \
      res[1] = v1[2] * v2[0] - v2[2] * v1[0]; \
      res[2] = v1[0] * v2[1] - v2[0] * v1[1]; \
    }

#define MAL_RET_VECTOR_DATABLOCK(name)\
  traits::vector_storage(name)

#define MAL_MATRIX(name, type)			\
  ublas::matrix<type> name

#define MAL_MATRIX_DIM(name, type, nb_rows, nb_cols) \
  ublas::matrix<type> name(nb_rows,nb_cols)

#define MAL_MATRIX_RESIZE(name,nb_rows,nb_cols) \
  name.resize(nb_rows,nb_cols)

#define MAL_MATRIX_NB_ROWS(name)  \
  name.size1()

#define MAL_MATRIX_NB_COLS(name)  \
  name.size2()

#define MAL_MATRIX_CLEAR(name) \
  name.clear()

#define MAL_INVERSE(name, inv_matrix, type)		\
  {							\
    ublas::matrix<type> U(name.size1(),name.size2());	\
    ublas::matrix<type> VT(name.size1(),name.size2());	\
    ublas::vector<type> s(name.size1());		\
    char Jobu='A'; /* Compute complete U Matrix */	\
    char Jobvt='A'; /* Compute complete VT Matrix */	\
    char Lw='O'; /* Compute the optimal size for the working vector */ \
    ublas::matrix<type> nametranspose;                  \
    nametranspose = trans(name);                        \
    int lw = lapack::gesvd_work(Lw,Jobu,Jobvt,nametranspose);    \
    ublas::vector<double> w(lw);		        \
    lapack::gesvd(Jobu, Jobvt,name,s,U,VT,w);		\
    ublas::matrix<type> S(name.size1(),name.size2());	\
    for(unsigned int i=0;i<s.size();i++)		\
      for(unsigned int j=0;j<s.size();j++)              \
	if (i==j) S(i,i)=1/s(i); else S(i,j)=0;		\
    ublas::matrix<type> tmp1;				\
    /* It appears that getsvd return Ut and V instead   \
       of U and VT as specified by the documentation*/	\
    tmp1 = prod(S,U);					\
    tmp1 = prod(VT,tmp1);				\
    /* This transpose issue is probablue due to the     \
       FORTRAN format of the matrices */                \
    inv_matrix = trans(tmp1);                           \
  }

#define MAL_PSEUDOINVERSE(matrix, pinv_matrix, type)

#define MAL_RET_TRANSPOSE(matrix) \
  trans(matrix)

#define MAL_TRANSPOSE_A_in_At(A,At)			\
  At=trans(A)

#define MAL_RET_A_by_B(A,B) \
  prod(A,B)

#define MAL_C_eq_A_by_B(C,A,B) \
  { \
    C = prod(A,B); \
  }

#define MAL_MATRIX_SET_IDENTITY(matrix) \
  { \
    for(unsigned int i=0;i<matrix.size1();i++) \
      for(unsigned  int j=0;j<matrix.size2();j++)\
        if (i==j) \
           matrix(i,j) = 1; \
        else  \
	  matrix(i,j) = 0;\
   } 

#define MAL_MATRIX_FILL(matrix, value) \
  {\
    for(unsigned int i=0;i<matrix.size1();i++) \
      for(unsigned int j=0;j<matrix.size2();j++)\
  	  matrix(i,j) = value;\
  }

#define MAL_RET_MATRIX_DATABLOCK(matrix)\
  traits::matrix_storage(matrix)

#define MAL_MATRIX_C_eq_EXTRACT_A(C,A,type, top,left, nbrows, nbcols) \
  { \
    ublas::matrix_slice< ublas::matrix<type> > amatrix(A,ublas::slice(top,1,nbrows),ublas::slice(left,1,nbcols)); \
    C = amatrix;\
  }


template<class type> inline double __ret_mal_matrix_ret_determinant(ublas::matrix<type> const & m)
{
  if (m.size1()!=m.size2())
    return -1;
  
  ublas::matrix<type> mLu(m);
  ublas::permutation_matrix<std::size_t> pivots(m.size1());

  lu_factorize(mLu,pivots);
  double det=1.0;
  for(std::size_t i=0;
      i< pivots.size();
      i++)
    {
      if (pivots(i)!=i)
	det*=-1.0;
      
      det *= mLu(i,i);
    }
  return det;
} 

#define MAL_MATRIX_RET_DETERMINANT(name,type)	\
  __ret_mal_matrix_ret_determinant<type>(name)

/* This specific externalisation is for a direct access to
   lapack : should be later on integrated inside boost */
  extern "C"
  {
    double dlapy2_(double *, double *);
    double dlamch_(char *);
    void dgges_(char *jobvsl, char *jobvsr, char *sort,/* logical (*delctg)(double*,double*,double*),*/
#if 1 
   logical (*delctg)(...),
		int *n, double *a, int *lda, double *b, int *ldb, int *sdim, double *alphar, double *alphai, double *beta,
		double *vsl, int *ldvsl, double *vsr, int *ldvsr, double *work, int *lwork, logical *bwork, int *info);
    
#endif
  };

#define _MAL_VERSION_ 1
#endif




/**
 * This header file intend to define an abstraction layer
 * to allow the use of diverse matrix libraries.
 * This is supposed to solve the dependency to a particular
 * matrix library.
 * If you do not like the current two solutions:
 *  1/ Oxford VNL
 *  2/ uBLAS + LAPACK 
 *  3/ Implement your own.
 * 
 * We however assume that the library used 
 * is a C++ class and has a sensible implementation
 * of matrix operators.
 *
 * (c) 2006 , Olivier Stasse JRL-Japan, CNRS-AIST, ISRI.
 */

#ifdef _VNL_MATRIX_

#include <VNL/matrix.h>
#include <VNL/vector.h>
#include <VNL/Algo/matrixinverse.h>
#include <VNL/Algo/determinant.h>
#include <VNL/Algo/svd.h>
#include <VNL/NetLib/netlib.h>

#define _MAL_VERSION_ 0

#define MAL_VECTOR(name, type)	\
  VNL::Vector<type> name

#define MAL_VECTOR_DIM(name, type, nb_rows)	\
  VNL::Vector<type> name(nb_rows, 0.0)

#define MAL_VECTOR_SIZE(name) \
  name.Size()

#define MAL_VECTOR_RESIZE(name, nb_rows) \
  name.Resize(nb_rows)

#define MAL_VECTOR_FILL(name, value) \
  name.Fill(value) 

#define MAL_RET_VECTOR_DATABLOCK(vector)\
  vector.DataBlock()

#define MAL_MATRIX(name, type)	\
  VNL::Matrix<type> name

#define MAL_MATRIX_DIM(name, type, nb_rows, nb_cols)	\
  VNL::Matrix<type> name(nb_rows,nb_cols,0.0)

#define MAL_MATRIX_RESIZE(name,nb_rows,nb_cols) \
  name.Resize(nb_rows,nb_cols)

#define MAL_MATRIX_NB_ROWS(name) \
  name.Rows()

#define MAL_MATRIX_NB_COLS(name) \
  name.Columns()

#define MAL_MATRIX_CLEAR(name) \
  name.Clear()

#define MAL_INVERSE(matrix, inv_matrix, type)	\
  { \
    VNL::MatrixInverse<type> tmp_InvMatrix(matrix); \
    inv_matrix = tmp_InvMatrix;		    \
  } 
#define MAL_PSEUDOINVERSE(matrix, pinv_matrix,type) \
  { \
    VNL::SVD<double> svd(matrix);\
    pinv_matrix = svd.PseudoInverse();\
  }

#define MAL_RET_TRANSPOSE(matrix) \
  matrix.Transpose()
  

#define MAL_RET_A_by_B(A,B) \
  A * B

#define MAL_C_eq_A_by_B(C,A,B) \
  C = A * B;

#define MAL_MATRIX_SET_IDENTITY(matrix) \
  matrix.SetIdentity()

#define MAL_MATRIX_FILL(matrix, value) \
  matrix.Fill(value) 

#define MAL_RET_MATRIX_DATABLOCK(matrix)\
  matrix.DataBlock()

#define MAL_MATRIX_C_eq_EXTRACT_A(C,A,type, top,left, nbrows, nbcols) \
  C = A.Extract(nbrows,nbcols,top,left)

#define MAL_MATRIX_RET_DETERMINANT(name)	\
  Determinant(name)

#endif

#ifdef _BOOST_MATRIX_

//#include <boost/numeric/bindings/atlas/clapack.hpp>
#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/traits/std_vector.hpp>
#include <boost/numeric/bindings/traits/std_vector.hpp>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/bindings/lapack/gesvd.hpp>

//#include <f2c.h>
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

#define MAL_RET_VECTOR_DATABLOCK(name)\
  traits::vector_storage(name)

#define MAL_MATRIX(name, type)			\
  ublas::matrix<type> name

#define MAL_MATRIX_DIM(name, type, nb_rows, nb_cols)	\
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
    ublas::matrix<type> U,VT;				\
    ublas::vector<type> s;				\
    ublas::vector<type> w; 				\
    char Jobu='A'; /* Compute complete U Matrix */	\
    char Jobvt='A'; /* Compute complte VT Matrix */	\
    lapack::gesvd(Jobu, Jobvt,name,s,U,VT,w);		\
    ublas::matrix<type> S(name.size1(),name.size2());	\
    for(unsigned int i=0;i<s.size();i++)		\
      S(i,i)=1/s(i);					\
    inv_matrix = prod(S,trans(U));			\
    inv_matrix = prod(trans(VT),inv_matrix);		\
  }

#define MAL_PSEUDOINVERSE(matrix, pinv_matrix, type)

#define MAL_RET_TRANSPOSE(matrix) \
  trans(matrix)

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
    C = A;\
  }
  
#define MAL_MATRIX_RET_DETERMINANT(name) \
  1.0

/* This specific externalisation is for a direct access to
   lapack : should be later on integrated inside boost */
  extern "C"
  {
    double dlapy2_(double *, double *);
    double dlamch_(char *);
    void dgges_(char *jobvsl, char *jobvsr, char *sort,/* logical (*delctg)(double*,double*,double*),*/
		logical (*delctg)(...),
		int *n, double *a, int *lda, double *b, int *ldb, int *sdim, double *alphar, double *alphai, double *beta,
		double *vsl, int *ldvsl, double *vsr, int *ldvsr, double *work, int *lwork, logical *bwork, int *info);
    
  };

#define _MAL_VERSION_ 1
#endif



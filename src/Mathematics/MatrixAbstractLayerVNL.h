/**  
 * This header file intend to define an abstraction layer
 * to allow the use of diverse matrix libraries.
 * This is supposed to solve the dependency to a particular
 * matrix library.
 *
 * This header file implements the wrapping around the VNL
 * matrix solution.
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

/*!
 * @defgroup VNL
 * @ingroup linearalgebra
 * 
 */
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

#define MAL_VECTOR_NORM(name) \
  name.TwoNorm()

#define MAL_VECTOR_3D_CROSS_PRODUCT(res,v1,v2)	\
  if ((v1.Size()==3) && (v2.Size()==3))		\
    res = Cross3D(v1,v2); 
  
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
  
#define MAL_TRANSPOSE_A_in_At(A,At)		\
  At = A.Transpose()

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


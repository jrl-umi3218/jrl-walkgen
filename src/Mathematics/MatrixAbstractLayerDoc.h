/*! This part of the documentation specifies what is the functionnality
 * of each macro, it is the responsability of the implemtor to comply with this
 * specification.
 * @defgroup vector Vector package 
 * @ingroup linearalgebra 
 */

/*!
 * @{
 */
/*!
 * \brief declare a vector named \a name with type \a type 
 * @ingroup vector
 */
#define MAL_VECTOR(name,type)
#undef MAL_VECTOR

/*!
 * \brief declare a vector named \a name with type \a type 
 * and of size \a nb_rows
 * @ingroup vector
 */
#define MAL_VECTOR_DIM(name, type, nb_rows)
#undef MAL_VECTOR_DIM

/*! \brief returns the size of the vector named \a name
 * @ingroup vector
 */
#define MAL_VECTOR_SIZE(name)
#undef MAL_VECTOR_SIZE

/*! \brief Resizes of the vector named \a name .
 * It acts as a function.
 * @ingroup vector
 */
#define MAL_VECTOR_RESIZE(name, nb_rows)
#undef MAL_VECTOR_RESIZE


/*! \brief Fill the vector named \a name with the value \a value.
 *  The user is responsible to make sure that the type are compatible.
 *  It acts as a function.
 * @ingroup vector
 */
#define MAL_VECTOR_FILL(name, value)
#undef MAL_VECTOR_FILL

/*! \brief Returns the two norms of the vector named \a name.
 *  More precisely if the vector \a name is of size \f$ n \f$, the
 *  norm is defined as:
 *  \f$ \sqrt{\sum_{i=0}^{i=n-1}x_i^2}  \f$
 * @ingroup vector
 */
#define MAL_VECTOR_NORM(name) 
#undef MAL_VECTOR_NORM

/*! \brief Returns in \a res the cross product of two vectors \f$ v_1,v_2 \in \mathbb{R}^3 \f$.
 * The cross product \f$ res = v_1 \times v_2 \f$ is defined as:
 * \f{eqnarray*}
 * res^x &=  v_1^y v_2^z  - v_2^y v_1^x \\
 * res^y &=  v_1^z v_2^x  - v_2^z v_1^x \\
 * res^z &=  v_1^x v_2^y  - v_2^x v_1^y \\
 * \f}
 * @ingroup vector
 */
#define MAL_VECTOR_3D_CROSS_PRODUCT(res,v1,v2)
#undef MAL_VECTOR_3D_CROSS_PRODUCT

/*! \brief Returns a direct access to the data information. It is implementation
 * dependent. However most of the time, it is assume to be an array of data.
 * @ingroup vector
 */
#define MAL_RET_VECTOR_DATABLOCK(name)
#undef MAL_RET_VECTOR_DATABLOCK

/*!
 *@}
 */

/*!
 * @defgroup matrix Matrix package
 * @ingroup linearalgebra
 */

/*! \brief Defines a matrix named \a name and of type \a type.
 * 
 * @ingroup matrix
 */
#define MAL_MATRIX(name,type)
#undef MAL_MATRIX


/*! \brief Defines a matrix named \a name of type \a type,
  with size (\a nb_rows, \a nb_cols ).
  It can be use for declaration.
 * @ingroup matrix
 */
#define MAL_MATRIX_DIM(name,type, nb_rows, nb_cols)
#undef MAL_MATRIX_DIM

/*! \brief Resize the matrix named \a name to  (\a nb_rows, \a nb_cols).
 *  This acts as a function.
 * @ingroup matrix
 */
#define MAL_MATRIX_RESIZE(name,type, nb_rows, nb_cols)
#undef MAL_MATRIX_RESIZE

/*! \brief Returns the number of rows of matrix \a name. 
 * @ingroup matrix
 */
#define MAL_MATRIX_NB_ROWS(name)
#undef MAL_MATRIX_NB_ROWS

/*! \brief Returns the number of cols of matrix \a name. 
 * @ingroup matrix
 */
#define MAL_MATRIX_NB_COLS(name)
#undef MAL_MATRIX_NB_COLS

/*! \brief Clear the matrix \a name , i.e. put all the value
 * to 0. In some case, it also mean that the data is deallocated.
 *  It is wise to resize the matrix after the call to this macro.
 * This macro acts as a function.
 * @ingroup matrix
 */
#define MAL_MATRIX_CLEAR(name)
#undef MAL_MATRIX_CLEAR

/*! \brief Returns in \a inv_matrix, the inverse of matrix \a name.
 * \a name should be a square matrix.
 * \f$ inv\_matrix = name^{-1} \f$
 * For implementation issue, the type of the data must be specified in \a type.
 * You should read the documentation of the specific implementation
 * to handle properly the numerical issues.
 * This macro acts as a function.

 Example:
 \code
  MAL_MATRIX(A,5,5,double);
  MAL_MATRIX(pA,5,5,double);
  ...
  MAL_INVERSE(A,iA,double);
  \endcode

 * @ingroup matrix
 */
#define MAL_INVERSE(name,inv_matrix,type)
#undef MAL_INVERSE

/*! \brief Returns in \a pinv_matrix the pseudo-inverse of matrix \a name.
 * \a name can be of arbitrary size.
 * \f$ inv\_matrix = matrix^{+} \f$
 * For implementation issue, the type of the data must be specified in \a type.
 * This macro acts as a function.

 * Example:
 \code
  MAL_MATRIX(A,double);
  MAL_MATRIX(pA,double);
  ...
  MAL_PSEUDOINVERSE(A,pA,double);
  \endcode
 
 * @ingroup matrix
 */
#define MAL_PSEUDOINVERSE(matrix, pinv_matrix, type)
#undef MAL_PSEUDOINVERSE

/*! \brief Returns the transpose of matrix, i.e. go through a temporary matrix.

 * Example: 
 \code
  MAL_MATRIX(A,double);
  MAL_MATRIX(At,double);
  ...
  At = MAL_TRANSPOSE(A);
  \endcode
 * @ingroup matrix
 */
#define MAL_RET_TRANSPOSE(matrix)
#undef MAL_RET_TRANSPOSE


/*! \brief Puts the transpose of matrix directly inside the argument.

 * Example: 
 \code
  MAL_MATRIX(A,double);
  MAL_MATRIX(At,double);
  ...
  MAL_TRANSPOSE_A_in_At(A,At);
  \endcode
 * @ingroup matrix
 */
#define MAL_TRANSPOSE_A_in_At(matrix)
#undef MAL_TRANSPOSE_A_in_At

/*! \brief Returns the product of \a A by \a B, i.e. \f$ {\bf A B} \f$
  This forces the use of a temporary matrix structure.
  
 * Example:
 \code
 MAT_MATRIX(A,double);
 MAT_MATRIX(B,double);
 MAL_MATRIX(C,double);
 ...
 C = MAL_RET_A_by_B(A,B);
 \endcode
 * @ingroup matrix
 */
#define MAL_RET_A_by_B(A,B)
#undef MAL_RET_A_by_B

/*! \brief Puts in \a C the result of the product of \a A by \a B,
   i.e. \f$ {\bf C} = {\bf AB} \f$

   
 * Example
   \code
   MAL_MATRIX(A,double);
   MAL_MATRIX(B,double);
   MAL_MATRIX(C,double);
   ...
   MAL_C_eq_A_by_B(C,A,B);
   \endcode
   This notation should be used to avoid temporary matrix,
   and should be avoided when writting something like:    
   \code
   MAL_MATRIX(A,double);
   MAL_MATRIX(B,double);
   ...
   MAL_C_eq_A_by_B(A,A,B);
   \endcode
   instead you should write:
   \code
   A = MAL_RET_A_by_B(A,B);
   \endcode

 * @ingroup matrix
*/
#define MAL_C_eq_A_by_B(C,A,B)
#undef MAL_C_eq_A_by_B

/*! \brief Set the matrix named \a matrix to the identity matrix. When \a matrix
  is not square, the identity matrix is set to the upper left greatest square matrix
  included into \a matrix. The other values are set to zero.

  if \n
  \f$ A = \left( \begin{matrix}
  1 & 2 & 3 & 4 \\
  5 & 6 & 7 & 8 \\
  9 & 10& 11& 12 \\
  \end{matrix}
  \right) \f$
  \n
  then with 
  \code
  MAL_MATRIX_SET_IDENTITY(A)
  \endcode
  \f$ A\f$ becomes \n
  \f$ A = \left( \begin{matrix}
  1 & 0 & 0 & 0 \\
  0 & 1 & 0 & 0 \\
  0 & 0 & 1 & 0 \\
  \end{matrix}
  \right) \f$


  * @ingroup matrix
*/
#define MAL_MATRIX_SET_IDENTITY(matrix)
#undef MAL_MATRIX_SET_IDENTITY

/*! \brief Fill the matrix named \a matrix with the value \a value.
  
 * Example:
 \code
 MAL_MATRIX(A,3,3);
 ...
 MAL_MATRIX_FILL(A,0);
 \endcode
 * @ingroup matrix
 */
#define MAL_MATRIX_FILL(matrix, value)
#undef MAL_MATRIX_FILL
  

/*! \brief Returns a pointer on the data stored for the matrix.
  
  This is specially useful for mapping with fortran routines,
  for instance LAPACK.
 * @ingroup matrix
 */
#define MAL_RET_MATRIX_DATABLOCK(matrix)
#undef MAL_RET_MATRIX_DATABLOCK

/*! \brief This is used to extract a submatrix \a C from \a A.
  
  For implementation purposes the data type is specified by \a type.
  The submatrix extracted is specified by its \a top - \a left coordinates,
  and its size: \a nbrows and \a nbcols for the number of rows and columns
  respectivly.
  
 * @ingroup matrix
 */
#define MAL_MATRIX_C_eq_EXTRACT_A(C,A, type, top,left, nbrows, nbcols)
#undef MAL_MATRIX_C_eq_EXTRACT_A

/*! \brief This macro returns the determinant of matrix \a name.
  
 * For implementation purposes the type has to be specified by \a type.
 * @ingroup matrix
 */
#define MAL_MATRIX_RET_DETERMINANT(name,type)
#undef MAL_MATRIX_RET_DETERMINANT



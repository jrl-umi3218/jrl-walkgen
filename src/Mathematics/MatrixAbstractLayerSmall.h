/*! This part of the documentation specifies what is the functionnality
 * of each macro, it is the responsability of the implementor to comply with this
 * specification.
 * @defgroup vector
 * @ingroup linearalgebra
 * 
 * (c) Olivier Stasse, JRL, CNRS-AIST, ISRI, 2007
 * 
 */

/*!
 * @{
 */
/*!
 * \brief declare a vector named \a name with type \a type 
 * @ingroup vector
 */
#define MAL_S3_VECTOR(name,type)
#undef MAL_S3_VECTOR

/* Specifying the dimension do not make sens...*/

/*! \brief returns the size of the vector named \a name
 * @ingroup vector
 */
#define MAL_S3_VECTOR_SIZE(name)
#undef MAL_S3_VECTOR_SIZE


/* Resizing does not make sense... */

/*! \brief Fill the vector named \a name with the value \a value.
 *  The user is responsible to make sure that the type are compatible.
 *  It acts as a function.
 * @ingroup vector
 */
#define MAL_S3_VECTOR_FILL(name, value)
#undef MAL_S3_VECTOR_FILL

/*! \brief Returns the two norms of the vector named \a name.
 *  More precisely if the vector \a name is of size \f$ n \f$, the
 *  norm is defined as:
 *  \f$ \sqrt{\sum_{i=0}^{i=n-1}x_i^2}  \f$
 * @ingroup vector
 */
#define MAL_S3_VECTOR_NORM(name) 
#undef MAL_S3_VECTOR_NORM

/*! \brief Returns in \a res the cross product of two vectors \f$ v_1,v_2 \in \mathbb{R}^3 \f$.
 * The cross product \f$ res = v_1 \times v_2 \f$ is defined as:
 * \f{eqnarray*}
 * res^x &=  v_1^y v_2^z  - v_2^y v_1^x \\
 * res^y &=  v_1^z v_2^x  - v_2^z v_1^x \\
 * res^z &=  v_1^x v_2^y  - v_2^x v_1^y \\
 * \f}
 * @ingroup vector
 */
#define MAL_S3_VECTOR_CROSS_PRODUCT(res,v1,v2)
#undef MAL_S3_VECTOR_CROSS_PRODUCT

/*! \brief Returns a direct access to the data information. It is implementation
 * dependent. However most of the time, it is assume to be an array of data.
 * @ingroup vector
 */
#define MAL_S3_RET_VECTOR_DATABLOCK(name)
#undef MAL_S3_RET_VECTOR_DATABLOCK

/*!
 *@}
 */

/*!
 * @defgroup matrix
 * @ingroup linearalgebra
 */

/*! \brief Defines a matrix named \a name and of type \a type.
 * 
 * @ingroup matrix
 */
#define MAL_S3x3_MATRIX(name,type)
#undef MAL_S3x3_MATRIX

/*! \brief Clear the matrix \a name , i.e. put all the value
 * to 0. In some case, it also mean that the data is deallocated.
 *  It is wise to resize the matrix after the call to this macro.
 * This macro acts as a function.
 * @ingroup matrix
 */
#define MAL_S3x3_MATRIX_CLEAR(name)
#undef MAL_S3x3_MATRIX_CLEAR

/*! \brief Returns in \a inv_matrix, the inverse of matrix \a name.
 * \a name should be a square matrix.
 * \f$ inv\_matrix = name^{-1} \f$
 * For implementation issue, the type of the data must be specified in \a type.
 * You should read the documentation of the specific implementation
 * to handle properly the numerical issues.
 * This macro acts as a function.

 Example:
 \code
  MAL_S3x3_MATRIX(A,5,5,double);
  MAL_S3x3_MATRIX(pA,5,5,double);
  ...
  MAL_S3x3_INVERSE(A,iA,double);
  \endcode

 * @ingroup matrix
 */
#define MAL_S3x3_INVERSE(name,inv_matrix,type)
#undef MAL_S3x3_INVERSE

/*! \brief Returns the transpose of matrix.

 * Example: 
 \code
  MAL_S3x3_MATRIX(A,double);
  MAL_S3x3_MATRIX(At,double);
  ...
  At = MAL_S3x3_TRANSPOSE(A);
  \endcode
 * @ingroup matrix
 */
#define MAL_S3x3_RET_TRANSPOSE(matrix)
#undef MAL_S3x3_RET_TRANSPOSE


/*! \brief Returns the product of \a A by \a B, i.e. \f$ {\bf A B} \f$
  This forces the use of a temporary matrix structure.
  
 * Example:
 \code
 MAL_S3x3_MATRIX(A,double);
 MAL_S3x3_MATRIX(B,double);
 MAL_S3x3_MATRIX(C,double);
 ...
 C = MAL_S3x3_RET_A_by_B(A,B);
 \endcode
 * @ingroup matrix
 */
#define MAL_S3x3_RET_A_by_B(A,B)
#undef MAL_S3x3_RET_A_by_B

/*! \brief Puts in \a C the result of the product of \a A by \a B,
   i.e. \f$ {\bf C} = {\bf AB} \f$

   
 * Example
   \code
   MAL_S3x3_MATRIX(A,double);
   MAL_S3x3_MATRIX(B,double);
   MAL_S3x3_MATRIX(C,double);
   ...
   MAL_S3x3_C_eq_A_by_B(C,A,B);
   \endcode
   This notation should be used to avoid temporary matrix,
   and should be avoided when writting something like:    
   \code
   MAL_S3x3_MATRIX(A,double);
   MAL_S3x3_MATRIX(B,double);
   ...
   MAL_S3x3_C_eq_A_by_B(A,A,B);
   \endcode
   instead you should write:
   \code
   A = MAL_S3x3_RET_A_by_B(A,B);
   \endcode

 * @ingroup matrix
*/
#define MAL_S3x3_C_eq_A_by_B(C,A,B)
#undef MAL_S3x3_C_eq_A_by_B

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
  MAL_S3x3_MATRIX_SET_IDENTITY(A)
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
#define MAL_S3x3_MATRIX_SET_IDENTITY(matrix)
#undef MAL_S3x3_MATRIX_SET_IDENTITY

/*! \brief Fill the matrix named \a matrix with the value \a value.
  
 * Example:
 \code
 MAL_S3x3_MATRIX(A,3,3);
 ...
 MAL_S3x3_MATRIX_FILL(A,0);
 \endcode
 * @ingroup matrix
 */
#define MAL_S3x3_MATRIX_FILL(matrix, value)
#undef MAL_S3x3_MATRIX_FILL
  

/*! \brief Returns a pointer on the data stored for the matrix.
  
  This is specially useful for mapping with fortran routines,
  for instance LAPACK.
 * @ingroup matrix
 */
#define MAL_S3x3_RET_MATRIX_DATABLOCK(matrix)
#undef MAL_S3x3_RET_MATRIX_DATABLOCK

/*! \brief This is used to extract a submatrix \a C from \a A.
  
  For implementation purposes the data type is specified by \a type.
  The submatrix extracted is specified by its \a top - \a left coordinates,
  and its size: \a nbrows and \a nbcols for the number of rows and columns
  respectivly.
  
 * @ingroup matrix
 */
#define MAL_S3x3_MATRIX_C_eq_EXTRACT_A(C,A, type, top,left, nbrows, nbcols)
#undef MAL_S3x3_MATRIX_C_eq_EXTRACT_A

/*! \brief This macro returns the determinant of matrix \a name.
  
 * For implementation purposes the type has to be specified by \a type.
 * @ingroup matrix
 */
#define MAL_S3x3_MATRIX_RET_DETERMINANT(name,type)
#undef MAL_S3x3_MATRIX_RET_DETERMINANT


/*!
 * @defgroup matrix
 * @ingroup linearalgebra
 */

/*! \brief Defines a matrix named \a name and of type \a type.
 * 
 * @ingroup matrix
 */
#define MAL_S4x4_MATRIX(name,type)
#undef MAL_S4x4_MATRIX

/*! \brief Clear the matrix \a name , i.e. put all the value
 * to 0. In some case, it also mean that the data is deallocated.
 *  It is wise to resize the matrix after the call to this macro.
 * This macro acts as a function.
 * @ingroup matrix
 */
#define MAL_S4x4_MATRIX_CLEAR(name)
#undef MAL_S4x4_MATRIX_CLEAR

/*! \brief Returns in \a inv_matrix, the inverse of matrix \a name.
 * \a name should be a square matrix.
 * \f$ inv\_matrix = name^{-1} \f$
 * For implementation issue, the type of the data must be specified in \a type.
 * You should read the documentation of the specific implementation
 * to handle properly the numerical issues.
 * This macro acts as a function.

 Example:
 \code
  MAL_S4x4_MATRIX(A,5,5,double);
  MAL_S4x4_MATRIX(pA,5,5,double);
  ...
  MAL_S4x4_INVERSE(A,iA,double);
  \endcode

 * @ingroup matrix
 */
#define MAL_S4x4_INVERSE(name,inv_matrix,type)
#undef MAL_S4x4_INVERSE

/*! \brief Returns the transpose of matrix.

 * Example: 
 \code
  MAL_S4x4_MATRIX(A,double);
  MAL_S4x4_MATRIX(At,double);
  ...
  At = MAL_S4x4_TRANSPOSE(A);
  \endcode
 * @ingroup matrix
 */
#define MAL_S4x4_RET_TRANSPOSE(matrix)
#undef MAL_S4x4_RET_TRANSPOSE


/*! \brief Returns the product of \a A by \a B, i.e. \f$ {\bf A B} \f$
  This forces the use of a temporary matrix structure.
  
 * Example:
 \code
 MAL_S4x4_MATRIX(A,double);
 MAL_S4x4_MATRIX(B,double);
 MAL_S4x4_MATRIX(C,double);
 ...
 C = MAL_S4x4_RET_A_by_B(A,B);
 \endcode
 * @ingroup matrix
 */
#define MAL_S4x4_RET_A_by_B(A,B)
#undef MAL_S4x4_RET_A_by_B

/*! \brief Puts in \a C the result of the product of \a A by \a B,
   i.e. \f$ {\bf C} = {\bf AB} \f$

   
 * Example
   \code
   MAL_S4x4_MATRIX(A,double);
   MAL_S4x4_MATRIX(B,double);
   MAL_S4x4_MATRIX(C,double);
   ...
   MAL_S4x4_C_eq_A_by_B(C,A,B);
   \endcode
   This notation should be used to avoid temporary matrix,
   and should be avoided when writting something like:    
   \code
   MAL_S4x4_MATRIX(A,double);
   MAL_S4x4_MATRIX(B,double);
   ...
   MAL_S4x4_C_eq_A_by_B(A,A,B);
   \endcode
   instead you should write:
   \code
   A = MAL_S4x4_RET_A_by_B(A,B);
   \endcode

 * @ingroup matrix
*/
#define MAL_S4x4_C_eq_A_by_B(C,A,B)
#undef MAL_S4x4_C_eq_A_by_B

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
  MAL_S4x4_MATRIX_SET_IDENTITY(A)
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
#define MAL_S4x4_MATRIX_SET_IDENTITY(matrix)
#undef MAL_S4x4_MATRIX_SET_IDENTITY

/*! \brief Fill the matrix named \a matrix with the value \a value.
  
 * Example:
 \code
 MAL_S4x4_MATRIX(A,3,3);
 ...
 MAL_S4x4_MATRIX_FILL(A,0);
 \endcode
 * @ingroup matrix
 */
#define MAL_S4x4_MATRIX_FILL(matrix, value)
#undef MAL_S4x4_MATRIX_FILL
  

/*! \brief Returns a pointer on the data stored for the matrix.
  
  This is specially useful for mapping with fortran routines,
  for instance LAPACK.
 * @ingroup matrix
 */
#define MAL_S4x4_RET_MATRIX_DATABLOCK(matrix)
#undef MAL_S4x4_RET_MATRIX_DATABLOCK

/*! \brief This is used to extract a submatrix \a C from \a A.
  
  For implementation purposes the data type is specified by \a type.
  The submatrix extracted is specified by its \a top - \a left coordinates,
  and its size: \a nbrows and \a nbcols for the number of rows and columns
  respectivly.
  
 * @ingroup matrix
 */
#define MAL_S4x4_MATRIX_C_eq_EXTRACT_A(C,A, type, top,left, nbrows, nbcols)
#undef MAL_S4x4_MATRIX_C_eq_EXTRACT_A

/*! \brief This macro returns the determinant of matrix \a name.
  
 * For implementation purposes the type has to be specified by \a type.
 * @ingroup matrix
 */
#define MAL_S4x4_MATRIX_RET_DETERMINANT(name,type)
#undef MAL_S4x4_MATRIX_RET_DETERMINANT



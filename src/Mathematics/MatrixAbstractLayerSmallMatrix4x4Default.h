/*! This is a very fast and simple implementation
 * of a 3D matrix class of double.
 * 
 * (c) Olivier Stasse, JRL, CNRS-AIST, ISRI, 2007
 */

#ifndef _MATRIX4D_PATTERNGENERATOR_JRL_
#define _MATRIX4D_PATTERNGENERATOR_JRL_

#ifdef _DEFAULT_MATRIX4x4_

namespace PatternGeneratorJRL
{
  /*! Template to handle a 4x4 matrix*/
  template <typename T> struct Matrix4x4
    {
      /*! The data array. */
      T m[16];

      /*! Defaut constructor. */
      Matrix4x4<T>() 
      {  m[0]  = 0.0;  m[1]  = 0.0; m[2]  = 0.0; m[3]  = 0.0; 
	 m[4]  = 0.0;  m[5]  = 0.0; m[6]  = 0.0; m[7]  = 0.0; 
	 m[8]  = 0.0;  m[9]  = 0.0; m[10] = 0.0; m[11] = 0.0;
         m[12] = 0.0;  m[13] = 0.0; m[14] = 0.0; m[15] = 0.0;
      }

      /*! Constructor form a scalar */
      Matrix4x4<T> (const T x)
      {
	for(int i=0;i<16;m[i++]=x);
      }
	
      /*! Copy constructor */
      Matrix4x4<T> (const struct Matrix4x4<T> &v)
      {
	for(int i=0;i<16;i++)
	  m[i] = v.m[i];
      }
      
      /*! Hybrid copy constructor */
      template <typename T2>
      Matrix4x4<T> (const struct Matrix4x4<T2> &v)
      {
	for(int i=0;i<16;i++)
	  m[i] = v.m[i];
      }
      
      /*! ith element considering the matrix as an array. */
      inline T& operator[](unsigned int i) 
      { return m[i];}
      
      /*! Access by giving the (i,j) element. */
      inline T& operator()(unsigned int i, unsigned int j)
      { return m[4*i+j]; }
      
      /*! Set to zero matrix */
      inline void setZero(void) 
      {
	for(int i=0;i<16;i++)
	  m[i] = 0.0;
      }
      
      /*! Set to identity */
      void setIdentity(void)
      {
	setZero();
	m[0] = m[5] = m[10] = m[15] = 1.0;
      }
      
      /*! Addition operator */
      Matrix4x4<T>  operator+(const Matrix4x4<T> & B)
      {
	Matrix4x4<T> A;
	A.m[0] = m[0] + B.m[0]; A.m[1] = m[1] + B.m[1]; A.m[2] = m[2] + B.m[2];
	A.m[3] = m[3] + B.m[3]; A.m[4] = m[4] + B.m[4]; A.m[5] = m[5] + B.m[5];
	A.m[6] = m[6] + B.m[6]; A.m[7] = m[7] + B.m[7]; A.m[8] = m[8] + B.m[8];
	A.m[9] = m[9] + B.m[9]; 
	A.m[10] = m[10] + B.m[10]; A.m[11] = m[11] + B.m[11];A.m[12] = m[12] + B.m[12]; 
	A.m[13] = m[13] + B.m[13]; A.m[14] = m[14] + B.m[14];A.m[15] = m[15] + B.m[15]; 
	
	return A;
      }
      
      /*! Substraction operator */
      struct Matrix4x4<T>  operator-(const struct Matrix4x4<T> &B)
      {
	Matrix4x4<T> A;
	A.m[0] = m[0] - B.m[0]; A.m[1] = m[1] - B.m[1]; A.m[2] = m[2] - B.m[2];
	A.m[3] = m[3] - B.m[3]; A.m[4] = m[4] - B.m[4]; A.m[5] = m[5] - B.m[5];
	A.m[6] = m[6] - B.m[6]; A.m[7] = m[7] - B.m[7]; A.m[8] = m[8] - B.m[8];
	A.m[9] = m[9] - B.m[9]; 
	A.m[10] = m[10] - B.m[10]; A.m[11] = m[11] - B.m[11];A.m[12] = m[12] - B.m[12]; 
	A.m[13] = m[13] - B.m[13]; A.m[14] = m[14] - B.m[14];A.m[15] = m[15] - B.m[15]; 
	return A;
      }

      /*! Multiplication operator with another matrix */
      Matrix4x4<T>  operator* (const Matrix4x4<T> &B) const
      {
	Matrix4x4<T> A;
	A.m[0] = m[0] * B.m[0] + m[1] * B.m[3] + m[2] * B.m[6];
	A.m[0] = m[0] * B.m[0] + m[1] * B.m[4] + m[2] * B.m[8] + m[3] * B.m[12];
	A.m[1] = m[0] * B.m[1] + m[1] * B.m[5] + m[2] * B.m[9] + m[3] * B.m[13];
	A.m[2] = m[0] * B.m[2] + m[1] * B.m[6] + m[2] * B.m[10] + m[3] * B.m[14];
	A.m[3] = m[0] * B.m[3] + m[1] * B.m[7] + m[2] * B.m[11] + m[3] * B.m[15];
	A.m[4] = m[4] * B.m[0] + m[5] * B.m[4] + m[6] * B.m[8] + m[7] * B.m[12];
	A.m[5] = m[4] * B.m[1] + m[5] * B.m[5] + m[6] * B.m[9] + m[7] * B.m[13];
	A.m[6] = m[4] * B.m[2] + m[5] * B.m[6] + m[6] * B.m[10] + m[7] * B.m[14];
	A.m[7] = m[4] * B.m[3] + m[5] * B.m[7] + m[6] * B.m[11] + m[7] * B.m[15];
	A.m[8] = m[8] * B.m[0] + m[9] * B.m[4] + m[10] * B.m[8] + m[11] * B.m[12];
	A.m[9] = m[8] * B.m[1] + m[9] * B.m[5] + m[10] * B.m[9] + m[11] * B.m[13];
	A.m[10] = m[8] * B.m[2] + m[9] * B.m[6] + m[10] * B.m[10] + m[11] * B.m[14];
	A.m[11] = m[8] * B.m[3] + m[9] * B.m[7] + m[10] * B.m[11] + m[11] * B.m[15];
	A.m[12] = m[12] * B.m[0] + m[13] * B.m[4] + m[14] * B.m[8] + m[15] * B.m[12];
	A.m[13] = m[12] * B.m[1] + m[13] * B.m[5] + m[14] * B.m[9] + m[15] * B.m[13];
	A.m[14] = m[12] * B.m[2] + m[13] * B.m[6] + m[14] * B.m[10] + m[15] * B.m[14];
	A.m[15] = m[12] * B.m[3] + m[13] * B.m[7] + m[14] * B.m[11] + m[15] * B.m[15];
	return A;
	
      }

      /*! Multiplication operator with another matrix */
      void  CeqthismulB (const Matrix4x4<T> &B, Matrix4x4<T> &C) const
      {
	C.m[0] = m[0] * B.m[0] + m[1] * B.m[4] + m[2] * B.m[8] + m[3] * B.m[12];
	C.m[1] = m[0] * B.m[1] + m[1] * B.m[5] + m[2] * B.m[9] + m[3] * B.m[13];
	C.m[2] = m[0] * B.m[2] + m[1] * B.m[6] + m[2] * B.m[10] + m[3] * B.m[14];
	C.m[3] = m[0] * B.m[3] + m[1] * B.m[7] + m[2] * B.m[11] + m[3] * B.m[15];
	C.m[4] = m[4] * B.m[0] + m[5] * B.m[4] + m[6] * B.m[8] + m[7] * B.m[12];
	C.m[5] = m[4] * B.m[1] + m[5] * B.m[5] + m[6] * B.m[9] + m[7] * B.m[13];
	C.m[6] = m[4] * B.m[2] + m[5] * B.m[6] + m[6] * B.m[10] + m[7] * B.m[14];
	C.m[7] = m[4] * B.m[3] + m[5] * B.m[7] + m[6] * B.m[11] + m[7] * B.m[15];
	C.m[8] = m[8] * B.m[0] + m[9] * B.m[4] + m[10] * B.m[8] + m[11] * B.m[12];
	C.m[9] = m[8] * B.m[1] + m[9] * B.m[5] + m[10] * B.m[9] + m[11] * B.m[13];
	C.m[10] = m[8] * B.m[2] + m[9] * B.m[6] + m[10] * B.m[10] + m[11] * B.m[14];
	C.m[11] = m[8] * B.m[3] + m[9] * B.m[7] + m[10] * B.m[11] + m[11] * B.m[15];
	C.m[12] = m[12] * B.m[0] + m[13] * B.m[4] + m[14] * B.m[8] + m[15] * B.m[12];
	C.m[13] = m[12] * B.m[1] + m[13] * B.m[5] + m[14] * B.m[9] + m[15] * B.m[13];
	C.m[14] = m[12] * B.m[2] + m[13] * B.m[6] + m[14] * B.m[10] + m[15] * B.m[14];
	C.m[15] = m[12] * B.m[3] + m[13] * B.m[7] + m[14] * B.m[11] + m[15] * B.m[15];	
      }

      /*! Multiplication operator with another matrix */
      void  CeqthismulB (const Vector4D<T> &B, Vector4D<T> &C) const
      {
	C.m_x = m[0] * B.m_x + m[1] * B.m_y + m[2] * B.m_z + m[3] * B.m_w;
	C.m_y = m[4] * B.m_x + m[5] * B.m_y + m[6] * B.m_z + m[7] * B.m_w;
	C.m_z = m[8] * B.m_x + m[9] * B.m_y + m[10] * B.m_z + m[11] * B.m_w;
	C.m_w = m[12] * B.m_x + m[13] * B.m_y + m[14] * B.m_z + m[15] * B.m_w;

      }


      /*! Multiplication operator with a constant */
      Matrix4x4<T> operator * (const double & r) 	
      {	
	struct Matrix4x4<T> result;
	result.m[0] = m[0] * r;
	result.m[1] = m[1] * r;
	result.m[2] = m[2] * r;
	result.m[3] = m[3] * r;
	result.m[4] = m[4] * r;
	result.m[5] = m[5] * r;
	result.m[6] = m[6] * r;
	result.m[7] = m[7] * r;
	result.m[8] = m[8] * r;
	result.m[9] = m[3] * r;
	result.m[10] = m[10] * r;
	result.m[11] = m[11] * r;
	result.m[12] = m[12] * r;
	result.m[13] = m[13] * r;
	result.m[14] = m[14] * r;
	result.m[15] = m[15] * r;
	return result;
      }


      
      
      /*! Transposition */
      Matrix4x4<T> Transpose()
      {
	struct Matrix4x4 A;
	A.m[0]  = m[0];  A.m[1]  = m[4]; A.m[2]  = m[8];  A.m[3]  = m[12]; 
	A.m[4]  = m[1];  A.m[5]  = m[5]; A.m[6]  = m[9];  A.m[7]  = m[13]; 
	A.m[8]  = m[2];  A.m[9]  = m[6]; A.m[10] = m[10]; A.m[11] = m[14];
	A.m[12] = m[3];  A.m[13] = m[7]; A.m[14] = m[11]; A.m[15] = m[15];
	return A;
      }
 
      /*! Inversion */
      void Inversion(Matrix4x4 A)
      {
	T det = 1/determinant();
	A.m[0] = m[0];
	A.m[1] = m[4];
	A.m[2] = m[8];
	A.m[3] = m[0]*m[3] + m[1]*m[7] + m[2]*m[11];
	A.m[4] = m[1];
	A.m[5] = m[5];
	A.m[6] = m[9];
	A.m[7] = m[4]*m[3] + m[5]*m[7] + m[6]*m[11];
	A.m[8] = m[2];
	A.m[9] = m[6];
	A.m[10] = m[10];
	A.m[11] = m[8]*m[3] + m[9]*m[7] + m[10]*m[11];
	A.m[12] = (0);
	A.m[13] = (0);
	A.m[14] = (0);
	A.m[15] = (1);
      }
     
      /*! Inversion */
      Matrix4x4<T> Inversion()
      {
	struct Matrix4x4 A; T det = 1/determinant();
	A.m[0] = m[0];
	A.m[1] = m[4];
	A.m[2] = m[8];
	A.m[3] = m[0]*m[3] + m[1]*m[7] + m[2]*m[11];
	A.m[4] = m[1];
	A.m[5] = m[5];
	A.m[6] = m[9];
	A.m[7] = m[4]*m[3] + m[5]*m[7] + m[6]*m[11];
	A.m[8] = m[2];
	A.m[9] = m[6];
	A.m[10] = m[10];
	A.m[11] = m[8]*m[3] + m[9]*m[7] + m[10]*m[11];
	A.m[12] = (0);
	A.m[13] = (0);
	A.m[14] = (0);
	A.m[15] = (1);
	return A;
      }
      
      /*! Determinant */
      T determinant() const	
      { 
	return  m[0]*m[5]*m[10]
	  + m[1]*m[6]*m[8]
	  + m[2]*m[4]*m[9]
	  - m[2]*m[5]*m[8]
	  - m[0]*m[6]*m[9]
	  - m[1]*m[4]*m[10];
      }

      T trace() const	{ return m[0]+m[5]+m[10]+m[15]; }

      /*! Self matrix addition */
      void operator += (const Matrix3x3<T>& B)	
      {	
	m[0] += B.m[0];
	m[1] += B.m[1];
	m[2] += B.m[2];
	m[3] += B.m[3];
	m[4] += B.m[4];
	m[5] += B.m[5];
	m[6] += B.m[6];
	m[7] += B.m[7];
	m[8] += B.m[8]; 
	m[9] += B.m[0];
	m[10] += B.m[10];
	m[11] += B.m[11];
	m[12] += B.m[12];
	m[13] += B.m[13];
	m[14] += B.m[14];
	m[15] += B.m[15];

      }


      /*! Local matrix substraction */
      void operator -= (const Matrix4x4<T>& B)	
      {	
	m[0] -= B.m[0];
	m[1] -= B.m[1];
	m[2] -= B.m[2];
	m[3] -= B.m[3];
	m[4] -= B.m[4];
	m[5] -= B.m[5];
	m[6] -= B.m[6];
	m[7] -= B.m[7];
	m[8] -= B.m[8]; 
	m[9] -= B.m[9];
	m[10] -= B.m[10];
	m[11] -= B.m[11];
	m[12] -= B.m[12];
	m[13] -= B.m[13];
	m[14] -= B.m[14];
	m[15] -= B.m[15];

      }
      
      ///Local matrix multiplication
      void operator *= (const Matrix4x4<T>& B)	
      {	Matrix4x4<T> temp(*this);
	m[0] = temp.m[0] * B.m[0] + temp.m[1] * B.m[4] + temp.m[2] * B.m[8] + temp.m[3] * B.m[12];
	m[1] = temp.m[0] * B.m[1] + temp.m[1] * B.m[5] + temp.m[2] * B.m[9] + temp.m[3] * B.m[13];
	m[2] = temp.m[0] * B.m[2] + temp.m[1] * B.m[6] + temp.m[2] * B.m[10] + temp.m[3] * B.m[14];
	m[3] = temp.m[0] * B.m[3] + temp.m[1] * B.m[7] + temp.m[2] * B.m[11] + temp.m[3] * B.m[15];
	m[4] = temp.m[4] * B.m[0] + temp.m[5] * B.m[4] + temp.m[6] * B.m[8] + temp.m[7] * B.m[12];
	m[5] = temp.m[4] * B.m[1] + temp.m[5] * B.m[5] + temp.m[6] * B.m[9] + temp.m[7] * B.m[13];
	m[6] = temp.m[4] * B.m[2] + temp.m[5] * B.m[6] + temp.m[6] * B.m[10] + temp.m[7] * B.m[14];
	m[7] = temp.m[4] * B.m[3] + temp.m[5] * B.m[7] + temp.m[6] * B.m[11] + temp.m[7] * B.m[15];
	m[8] = temp.m[8] * B.m[0] + temp.m[9] * B.m[4] + temp.m[10] * B.m[8] + temp.m[11] * B.m[12];
	m[9] = temp.m[8] * B.m[1] + temp.m[9] * B.m[5] + temp.m[10] * B.m[9] + temp.m[11] * B.m[13];
	m[10] = temp.m[8] * B.m[2] + temp.m[9] * B.m[6] + temp.m[10] * B.m[10] + temp.m[11] * B.m[14];
	m[11] = temp.m[8] * B.m[3] + temp.m[9] * B.m[7] + temp.m[10] * B.m[11] + temp.m[11] * B.m[15];
	m[12] = temp.m[12] * B.m[0] + temp.m[13] * B.m[4] + temp.m[14] * B.m[8] + temp.m[15] * B.m[12];
	m[13] = temp.m[12] * B.m[1] + temp.m[13] * B.m[5] + temp.m[14] * B.m[9] + temp.m[15] * B.m[13];
	m[14] = temp.m[12] * B.m[2] + temp.m[13] * B.m[6] + temp.m[14] * B.m[10] + temp.m[15] * B.m[14];
	m[15] = temp.m[12] * B.m[3] + temp.m[13] * B.m[7] + temp.m[14] * B.m[11] + temp.m[15] * B.m[15];
      }
      
      inline friend std::ostream& operator <<(std::ostream &os,Matrix4x4<T> const &A)
      {
	for(int i=0;i<4;i++)
	  {
	    for(int j=0;j<4;j++)
	      os << A.m[i*4+j] << " ";
	    os << endl;
	  }
	return os;
      }
      


    };

  
#define MAL_S4x4_MATRIX(name,type) \
  PatternGeneratorJRL::Matrix4x4<type> name

#define MAL_S4x4_MATRIX_CLEAR(name) \
  name.setZero()

#define MAL_S4x4_MATRIX_SET_IDENTITY(name) \
  name.setIdentity()

#define MAL_S4x4_INVERSE(name,inv_matrix,type)	\
  inv_matrix = name.Inversion();

#define MAL_S4x4_RET_TRANSPOSE(matrix) \
  matrix.Transpose();

#define MAL_S4x4_TRANSPOSE_A_in_At(A,At)	\
  A.Transpose(At);

#define MAL_S4x4_RET_A_by_B(A,B) \
  A*B

#define MAL_S4x4_C_eq_A_by_B(C,A,B) \
  A.CeqthismulB(B,C)
  
  
};
#endif

#endif

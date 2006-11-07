/************************************************
*				Linear Algebra					*
*				  4x4 mat4f						*
*												*
*				by Mathieu Brunel				*
************************************************/

/************************************************************
* mat4f organization:										*
*	m[0]  m[1]  m[2]  m[3]		m(0,0) m(0,1) m(0,2) m(0,3)	*
*	m[4]  m[5]  m[6]  m[7]		m(1,0) m(1,1) m(1,2) m(1,3)	*
*	m[8]  m[9]  m[10] m[11]		m(2,0) m(2,1) m(2,2) m(2,3)	*
*	m[12] m[13] m[14] m[15]		m(3,0) m(3,1) m(3,2) m(3,3)	*
************************************************************/

#ifndef _MATRIX_4_H_
#define _MATRIX_4_H_
template <typename T>
class matrix4
{
template <typename T2> friend class matrix4;
public:
	T m[16];

	//Constructors & destructor
	///Default constructor
	matrix4() {}

	///Constructor from a scalar
	matrix4(const T x) {	m[0]=x;
							m[1]=x;
							m[2]=x;
							m[3]=x;
							m[4]=x;
							m[5]=x;
							m[6]=x;
							m[7]=x;
							m[8]=x;
							m[9]=x;
							m[10]=x;
							m[11]=x;
							m[12]=x;
							m[13]=x;
							m[14]=x;
							m[15]=x;}

	///Constructor from 9 scalars
	matrix4( const T m0, const T m1, const T m2, const T m3,
			 const T m4, const T m5, const T m6, const T m7,
			 const T m8, const T m9, const T m10, const T m11,
			 const T m12, const T m13, const T m14, const T m15) {	m[0]=m0;
																	m[1]=m1;
																	m[2]=m2;
																	m[3]=m3;
																	m[4]=m4;
																	m[5]=m5;
																	m[6]=m6;
																	m[7]=m7;
																	m[8]=m8;
																	m[9]=m9;
																	m[10]=m10;
																	m[11]=m11;
																	m[12]=m12;
																	m[13]=m13;
																	m[14]=m14;
																	m[15]=m15;}

	///Constructor from an array
	matrix4(const T* array) {	m[0]=array[0];
								m[1]=array[1];
								m[2]=array[2];
								m[3]=array[3];
								m[4]=array[4];
								m[5]=array[5];
								m[6]=array[6];
								m[7]=array[7];
								m[8]=array[8];
								m[9]=array[9];
								m[10]=array[10];
								m[11]=array[11];
								m[12]=array[12];
								m[13]=array[13];
								m[14]=array[14];
								m[15]=array[15];}
	
	///Copy constructor
	matrix4(const matrix4<T>& mv) {	m[0]=mv.m[0];
								m[1]=mv.m[1];
								m[2]=mv.m[2];
								m[3]=mv.m[3];
								m[4]=mv.m[4];
								m[5]=mv.m[5];
								m[6]=mv.m[6];
								m[7]=mv.m[7];
								m[8]=mv.m[8];
								m[9]=mv.m[9];
								m[10]=mv.m[10];
								m[11]=mv.m[11];
								m[12]=mv.m[12];
								m[13]=mv.m[13];
								m[14]=mv.m[14];
								m[15]=mv.m[15]; }

	matrix4(const matrix3<T>& mv) {	m[0]=mv.m[0];
								m[1]=mv.m[1];
								m[2]=mv.m[2];
								m[4]=mv.m[3];
								m[5]=mv.m[4];
								m[6]=mv.m[5];
								m[8]=mv.m[6];
								m[9]=mv.m[7];
								m[10]=mv.m[8];

								m[3]=
								m[7]=
								m[11]=
								m[12]=
								m[13]=
								m[14]=(0);
								m[15]=(1); }

	///Hybrid copy constructor
	template <typename T2>
	matrix4(const matrix4<T2>& mv) {	m[0]=mv.m[0];
									m[1]=mv.m[1];
									m[2]=mv.m[2];
									m[3]=mv.m[3];
									m[4]=mv.m[4];
									m[5]=mv.m[5];
									m[6]=mv.m[6];
									m[7]=mv.m[7];
									m[8]=mv.m[8];
									m[9]=mv.m[9];
									m[10]=mv.m[0];
									m[11]=mv.m[11];
									m[12]=mv.m[12];
									m[13]=mv.m[13];
									m[14]=mv.m[14];
									m[15]=mv.m[15];}

	///Destructor
	~matrix4(){}
	
	//Accessors
	///ith element i considering the matrix as an array
	T& operator [] (unsigned int i)	{ return m[i]; }

	///(i,j) element
	T& operator () (unsigned int i, unsigned int j) { return m[4*i+j]; }

	///Set to zero matrix
	matrix4& setZero(void) {	m[0]=(0);
								m[1]=(0);
								m[2]=(0);
								m[3]=(0);
								m[4]=(0);
								m[5]=(0);
								m[6]=(0);
								m[7]=(0);
								m[8]=(0);
								m[9]=(0);
								m[10]=(0);
								m[11]=(0);
								m[12]=(0);
								m[13]=(0);
								m[14]=(0);
								m[15]=(0);
								return *this; }

	///Set to identity
	matrix4& setIdentity(void)	{	m[0]=(1);
									m[1]=(0);
									m[2]=(0);
									m[3]=(0);
									m[4]=(0);
									m[5]=(1);
									m[6]=(0);
									m[7]=(0);
									m[8]=(0);
									m[9]=(0);
									m[10]=(1);
									m[11]=(0);
									m[12]=(0);
									m[13]=(0);
									m[14]=(0);
									m[15]=(1);
									return *this; }

	///Local matrix addition
	void operator += (const matrix4<T>& mv)	{	m[0] += mv.m[0];
												m[1] += mv.m[1];
												m[2] += mv.m[2];
												m[3] += mv.m[3];
												m[4] += mv.m[4];
												m[5] += mv.m[5];
												m[6] += mv.m[6];
												m[7] += mv.m[7];
												m[8] += mv.m[8];
												m[9] += mv.m[9];
												m[10] += mv.m[10];
												m[11] += mv.m[11];
												m[12] += mv.m[12];
												m[13] += mv.m[13];
												m[14] += mv.m[14];
												m[15] += mv.m[15]; }

	///Local matrix substraction
	void operator -= (const matrix4<T>& mv)	{	m[0] -= mv.m[0];
												m[1] -= mv.m[1];
												m[2] -= mv.m[2];
												m[3] -= mv.m[3];
												m[4] -= mv.m[4];
												m[5] -= mv.m[5];
												m[6] -= mv.m[6];
												m[7] -= mv.m[7];
												m[8] -= mv.m[8];
												m[9] -= mv.m[9];
												m[10] -= mv.m[10];
												m[11] -= mv.m[11];
												m[12] -= mv.m[12];
												m[13] -= mv.m[13];
												m[14] -= mv.m[14];
												m[15] -= mv.m[15]; }

	///Local matrix multiplication
	void operator *= (const matrix4<T>& mv)	{	matrix4<T> temp(*this);
												m[0] = temp.m[0] * mv.m[0] + temp.m[1] * mv.m[4] + temp.m[2] * mv.m[8] + temp.m[3] * mv.m[12];
												m[1] = temp.m[0] * mv.m[1] + temp.m[1] * mv.m[5] + temp.m[2] * mv.m[9] + temp.m[3] * mv.m[13];
												m[2] = temp.m[0] * mv.m[2] + temp.m[1] * mv.m[6] + temp.m[2] * mv.m[10] + temp.m[3] * mv.m[14];
												m[3] = temp.m[0] * mv.m[3] + temp.m[1] * mv.m[7] + temp.m[2] * mv.m[11] + temp.m[3] * mv.m[15];
												m[4] = temp.m[4] * mv.m[0] + temp.m[5] * mv.m[4] + temp.m[6] * mv.m[8] + temp.m[7] * mv.m[12];
												m[5] = temp.m[4] * mv.m[1] + temp.m[5] * mv.m[5] + temp.m[6] * mv.m[9] + temp.m[7] * mv.m[13];
												m[6] = temp.m[4] * mv.m[2] + temp.m[5] * mv.m[6] + temp.m[6] * mv.m[10] + temp.m[7] * mv.m[14];
												m[7] = temp.m[4] * mv.m[3] + temp.m[5] * mv.m[7] + temp.m[6] * mv.m[11] + temp.m[7] * mv.m[15];
												m[8] = temp.m[8] * mv.m[0] + temp.m[9] * mv.m[4] + temp.m[10] * mv.m[8] + temp.m[11] * mv.m[12];
												m[9] = temp.m[8] * mv.m[1] + temp.m[9] * mv.m[5] + temp.m[10] * mv.m[9] + temp.m[11] * mv.m[13];
												m[10] = temp.m[8] * mv.m[2] + temp.m[9] * mv.m[6] + temp.m[10] * mv.m[10] + temp.m[11] * mv.m[14];
												m[11] = temp.m[8] * mv.m[3] + temp.m[9] * mv.m[7] + temp.m[10] * mv.m[11] + temp.m[11] * mv.m[15];
												m[12] = temp.m[12] * mv.m[0] + temp.m[13] * mv.m[4] + temp.m[14] * mv.m[8] + temp.m[15] * mv.m[12];
												m[13] = temp.m[12] * mv.m[1] + temp.m[13] * mv.m[5] + temp.m[14] * mv.m[9] + temp.m[15] * mv.m[13];
												m[14] = temp.m[12] * mv.m[2] + temp.m[13] * mv.m[6] + temp.m[14] * mv.m[10] + temp.m[15] * mv.m[14];
												m[15] = temp.m[12] * mv.m[3] + temp.m[13] * mv.m[7] + temp.m[14] * mv.m[11] + temp.m[15] * mv.m[15]; }


	///mat4f addition
	matrix4 operator + (const matrix4<T>& mv) const	{	matrix4<T> result;
														result.m[0] = m[0] + mv.m[0];
														result.m[1] = m[1] + mv.m[1];
														result.m[2] = m[2] + mv.m[2];
														result.m[3] = m[3] + mv.m[3];
														result.m[4] = m[4] + mv.m[4];
														result.m[5] = m[5] + mv.m[5];
														result.m[6] = m[6] + mv.m[6];
														result.m[7] = m[7] + mv.m[7];
														result.m[8] = m[8] + mv.m[8];
														result.m[9] = m[9] + mv.m[9];
														result.m[10] = m[10] + mv.m[10];
														result.m[11] = m[11] + mv.m[11];
														result.m[12] = m[12] + mv.m[12];
														result.m[13] = m[13] + mv.m[13];
														result.m[14] = m[14] + mv.m[14];
														result.m[15] = m[15] + mv.m[15];
														return result; }

	///mat4f substraction
	matrix4 operator - (const matrix4<T>& mv) const	{	matrix4<T> result(*this);
														result.m[0] = m[0] - mv.m[0];
														result.m[1] = m[1] - mv.m[1];
														result.m[2] = m[2] - mv.m[2];
														result.m[3] = m[3] - mv.m[3];
														result.m[4] = m[4] - mv.m[4];
														result.m[5] = m[5] - mv.m[5];
														result.m[6] = m[6] - mv.m[6];
														result.m[7] = m[7] - mv.m[7];
														result.m[8] = m[8] - mv.m[8];
														result.m[9] = m[9] - mv.m[9];
														result.m[10] = m[10] - mv.m[10];
														result.m[11] = m[11] - mv.m[11];
														result.m[12] = m[12] - mv.m[12];
														result.m[13] = m[13] - mv.m[13];
														result.m[14] = m[14] - mv.m[14];
														result.m[15] = m[15] - mv.m[15];
														return result; }

	///mat4f multiplication
	matrix4 operator * (const matrix4<T>& mv) const	{	matrix4<T> result;
														result.m[0] = m[0] * mv.m[0] + m[1] * mv.m[3] + m[2] * mv.m[6];
														result.m[0] = m[0] * mv.m[0] + m[1] * mv.m[4] + m[2] * mv.m[8] + m[3] * mv.m[12];
														result.m[1] = m[0] * mv.m[1] + m[1] * mv.m[5] + m[2] * mv.m[9] + m[3] * mv.m[13];
														result.m[2] = m[0] * mv.m[2] + m[1] * mv.m[6] + m[2] * mv.m[10] + m[3] * mv.m[14];
														result.m[3] = m[0] * mv.m[3] + m[1] * mv.m[7] + m[2] * mv.m[11] + m[3] * mv.m[15];
														result.m[4] = m[4] * mv.m[0] + m[5] * mv.m[4] + m[6] * mv.m[8] + m[7] * mv.m[12];
														result.m[5] = m[4] * mv.m[1] + m[5] * mv.m[5] + m[6] * mv.m[9] + m[7] * mv.m[13];
														result.m[6] = m[4] * mv.m[2] + m[5] * mv.m[6] + m[6] * mv.m[10] + m[7] * mv.m[14];
														result.m[7] = m[4] * mv.m[3] + m[5] * mv.m[7] + m[6] * mv.m[11] + m[7] * mv.m[15];
														result.m[8] = m[8] * mv.m[0] + m[9] * mv.m[4] + m[10] * mv.m[8] + m[11] * mv.m[12];
														result.m[9] = m[8] * mv.m[1] + m[9] * mv.m[5] + m[10] * mv.m[9] + m[11] * mv.m[13];
														result.m[10] = m[8] * mv.m[2] + m[9] * mv.m[6] + m[10] * mv.m[10] + m[11] * mv.m[14];
														result.m[11] = m[8] * mv.m[3] + m[9] * mv.m[7] + m[10] * mv.m[11] + m[11] * mv.m[15];
														result.m[12] = m[12] * mv.m[0] + m[13] * mv.m[4] + m[14] * mv.m[8] + m[15] * mv.m[12];
														result.m[13] = m[12] * mv.m[1] + m[13] * mv.m[5] + m[14] * mv.m[9] + m[15] * mv.m[13];
														result.m[14] = m[12] * mv.m[2] + m[13] * mv.m[6] + m[14] * mv.m[10] + m[15] * mv.m[14];
														result.m[15] = m[12] * mv.m[3] + m[13] * mv.m[7] + m[14] * mv.m[11] + m[15] * mv.m[15];
														return result; }

	///Transposition
	void transposeTo(matrix4& mv) const	{	mv.m[0] = m[0];
											mv.m[1] = m[4];
											mv.m[2] = m[8];
											mv.m[3] = m[12];
											mv.m[4] = m[1];
											mv.m[5] = m[5];
											mv.m[6] = m[9];
											mv.m[7] = m[13];
											mv.m[8] = m[2];
											mv.m[9] = m[6];
											mv.m[10] = m[10];
											mv.m[11] = m[14];
											mv.m[12] = m[3];
											mv.m[13] = m[7];
											mv.m[14] = m[11];
											mv.m[15] = m[15];};

	///Inversion assuming a transformation matrix
	void inverseTo(matrix4& mv) const	{	mv.m[0] = m[0];
											mv.m[1] = m[4];
											mv.m[2] = m[8];
											mv.m[3] = m[0]*m[3] + m[1]*m[7] + m[2]*m[11];
											mv.m[4] = m[1];
											mv.m[5] = m[5];
											mv.m[6] = m[9];
											mv.m[7] = m[4]*m[3] + m[5]*m[7] + m[6]*m[11];
											mv.m[8] = m[2];
											mv.m[9] = m[6];
											mv.m[10] = m[10];
											mv.m[11] = m[8]*m[3] + m[9]*m[7] + m[10]*m[11];
											mv.m[12] = (0);
											mv.m[13] = (0);
											mv.m[14] = (0);
											mv.m[15] = (1);

											/*
											T det = 1 / determinant();
											mv.m[0] = ( m[5]*m[10] - m[6]*m[9] ) * det;
											mv.m[1] = ( m[2]*m[9] - m[1]*m[10] ) * det;
											mv.m[2] = ( m[1]*m[6] - m[2]*m[5] ) * det;
											mv.m[3] = ( m[1]*( m[7]*m[10] - m[6]*m[11] )
														+ m[2]*( m[5]*m[11] - m[7]*m[9] )
														+ m[3]*( m[6]*m[9] - m[5]*m[10] ) ) * det;
											mv.m[4] = ( m[6]*m[8] - m[4]*m[10] ) * det;
											mv.m[5] = ( m[0]*m[10] - m[2]*m[8] ) * det;
											mv.m[6] = ( m[2]*m[4] - m[0]*m[6] ) * det;
											mv.m[7] = ( m[0]*( m[6]*m[11] - m[7]*m[10] )
														+ m[2]*( m[7]*m[8] - m[4]*m[11] )
														+ m[3]*( m[4]*m[10] - m[6]*m[8] ) ) * det;
											mv.m[8] = ( m[4]*m[9] - m[5]*m[8] ) * det;
											mv.m[9] = ( m[1]*m[8] - m[0]*m[9] ) * det;
											mv.m[10] = ( m[0]*m[5] - m[1]*m[4] ) * det;
											mv.m[11] = ( m[0]*( m[7]*m[9] - m[5]*m[11] )
														+ m[1]*( m[4]*m[11] - m[7]*m[8] )
														+ m[3]*( m[5]*m[8] - m[4]*m[9] ) ) * det;
											mv.m[12] = m[12];
											mv.m[13] = m[13];
											mv.m[14] = m[14];
											mv.m[15] = m[15]; */}


	///Local transposition
	matrix4& transpose()	{	T temp = m[1]; m[1] = m[4]; m[4] = temp;
								temp = m[2]; m[2] = m[8]; m[8] = temp;
								temp = m[3]; m[3] = m[12]; m[12] = temp;
								temp = m[6]; m[6] = m[9]; m[9] = temp;
								temp = m[7]; m[7] = m[13]; m[13] = temp;
								temp = m[11]; m[11] = m[14]; m[14] = temp;
								return *this; }
	
	///Local inversion assuming a transformation matrix
	matrix4& inverse_tranformation() {	T temp = m[1]; m[1] = m[4]; m[4] = temp;
							temp = m[2]; m[2] = m[8]; m[8] = temp;
							temp = m[6]; m[6] = m[9]; m[9] = temp;

							temp = m[3];
							T temp2 = m[7];
							m[3] = m[0]*temp + m[1]*temp2 + m[2]*m[11];
							m[7] = m[4]*temp + m[5]*temp2 + m[6]*m[11];
							m[11] = m[8]*temp + m[9]*temp2 + m[10]*m[11];
							/*matrix4<T> temp(*this);
							T det = 1 / determinant();
							m[0] = ( temp.m[5]*temp.m[10] - temp.m[6]*temp.m[9] ) * det;
							m[1] = ( temp.m[2]*temp.m[9] - temp.m[1]*temp.m[10] ) * det;
							m[2] = ( temp.m[1]*temp.m[6] - temp.m[2]*temp.m[5] ) * det;
							m[3] = ( temp.m[1]*( temp.m[7]*temp.m[10] - temp.m[6]*temp.m[11] )
										  + temp.m[2]*( temp.m[5]*temp.m[11] - temp.m[7]*temp.m[9] )
										  + temp.m[3]*( temp.m[6]*temp.m[9] - temp.m[5]*temp.m[10] ) ) * det;
							m[4] = ( temp.m[6]*temp.m[8] - temp.m[4]*temp.m[10] ) * det;
							m[5] = ( temp.m[0]*temp.m[10] - temp.m[2]*temp.m[8] ) * det;
							m[6] = ( temp.m[2]*temp.m[4] - temp.m[0]*temp.m[6] ) * det;
							m[7] = ( temp.m[0]*( temp.m[6]*temp.m[11] - temp.m[7]*temp.m[10] )
										  + temp.m[2]*( temp.m[7]*temp.m[8] - temp.m[4]*temp.m[11] )
										  + temp.m[3]*( temp.m[4]*temp.m[10] - temp.m[6]*temp.m[8] ) ) * det;
							m[8] = ( temp.m[4]*temp.m[9] - temp.m[5]*temp.m[8] ) * det;
							m[9] = ( temp.m[1]*temp.m[8] - temp.m[0]*temp.m[9] ) * det;
							m[10] = ( temp.m[0]*temp.m[5] - temp.m[1]*temp.m[4] ) * det;
							m[11] = ( temp.m[0]*( temp.m[7]*temp.m[9] - temp.m[5]*temp.m[11] )
										   + temp.m[1]*( temp.m[4]*temp.m[11] - temp.m[7]*temp.m[8] )
										   + temp.m[3]*( temp.m[5]*temp.m[8] - temp.m[4]*temp.m[9] ) ) * det;*/
							return *this; }

	matrix4& inverse() {	matrix4<T> temp(*this);
							T det = 1 / determinant();
							m[0] = ( temp.m[5]*temp.m[10] - temp.m[6]*temp.m[9] ) * det;
							m[1] = ( temp.m[2]*temp.m[9] - temp.m[1]*temp.m[10] ) * det;
							m[2] = ( temp.m[1]*temp.m[6] - temp.m[2]*temp.m[5] ) * det;
							m[3] = ( temp.m[1]*( temp.m[7]*temp.m[10] - temp.m[6]*temp.m[11] )
										  + temp.m[2]*( temp.m[5]*temp.m[11] - temp.m[7]*temp.m[9] )
										  + temp.m[3]*( temp.m[6]*temp.m[9] - temp.m[5]*temp.m[10] ) ) * det;
							m[4] = ( temp.m[6]*temp.m[8] - temp.m[4]*temp.m[10] ) * det;
							m[5] = ( temp.m[0]*temp.m[10] - temp.m[2]*temp.m[8] ) * det;
							m[6] = ( temp.m[2]*temp.m[4] - temp.m[0]*temp.m[6] ) * det;
							m[7] = ( temp.m[0]*( temp.m[6]*temp.m[11] - temp.m[7]*temp.m[10] )
										  + temp.m[2]*( temp.m[7]*temp.m[8] - temp.m[4]*temp.m[11] )
										  + temp.m[3]*( temp.m[4]*temp.m[10] - temp.m[6]*temp.m[8] ) ) * det;
							m[8] = ( temp.m[4]*temp.m[9] - temp.m[5]*temp.m[8] ) * det;
							m[9] = ( temp.m[1]*temp.m[8] - temp.m[0]*temp.m[9] ) * det;
							m[10] = ( temp.m[0]*temp.m[5] - temp.m[1]*temp.m[4] ) * det;
							m[11] = ( temp.m[0]*( temp.m[7]*temp.m[9] - temp.m[5]*temp.m[11] )
										   + temp.m[1]*( temp.m[4]*temp.m[11] - temp.m[7]*temp.m[8] )
										   + temp.m[3]*( temp.m[5]*temp.m[8] - temp.m[4]*temp.m[9] ) ) * det;
							return *this; }



	//Characteristical values
	///Determinant considering a transformation matrix
	T determinant() const	{ return  m[0]*m[5]*m[10]
									+ m[1]*m[6]*m[8]
									+ m[2]*m[4]*m[9]
									- m[2]*m[5]*m[8]
									- m[0]*m[6]*m[9]
									- m[1]*m[4]*m[10]; }

	///Trace
	T trace() const	{ return m[0]+m[5]+m[10]+m[15]; }

	///Eigen values
	void eigenValue(T& vp1, T& vp2, T& vp3, T& vp4) const;
	void eigenValue(T* vp) const;

	///Cast
	  //operator matrix4<T>&(){return *this;}

	//Static members
	///Identity matrix
	static const matrix4<T> identity;

	///Zero matrix
	static const matrix4<T> zero;
};

template <typename T>
const matrix4<T> matrix4<T>::identity((1), (0), (0), (0),
									  (0), (1), (0), (0),
									  (0), (0), (1), (0),
									  (0), (0), (0), (1));

template <typename T>
const matrix4<T> matrix4<T>::zero((0));
#endif

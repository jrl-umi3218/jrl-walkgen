/************************************************
*				Linear Algebra					*
*				  4x4 mat4f					*
*												*
*				by Mathieu Brunel				*
************************************************/

/************************************************
* mat4f organization:							*
*	m[0] m[1] m[2]		m(0,0) m(0,1) m(0,2)	*
*	m[3] m[4] m[5]		m(1,0) m(1,1) m(1,2)	*
*	m[6] m[7] m[8]		m(2,0) m(2,1) m(2,2)	*
************************************************/
#ifndef _MATRIX_3_H_
#define  _MATRIX_3_H_

#include "vector3.h"
#include <iostream>

using namespace std;
template <typename T>
class matrix3
{
template <typename T2> friend class matrix3;
public:
	T m[9];

	//Constructors & destructor
	///Default constructor
	matrix3<T> () {}

	///Constructor from a scalar
	matrix3<T> (const T x) {	m[0]=x;
							m[1]=x;
							m[2]=x;
							m[3]=x;
							m[4]=x;
							m[5]=x;
							m[6]=x;
							m[7]=x;
							m[8]=x; }

	///Constructor from 9 scalars
	matrix3<T> (const T m0, const T m1, const T m2,
			 const T m3, const T m4, const T m5,
			 const T m6, const T m7, const T m8) {	m[0]=m0;
													m[1]=m1;
													m[2]=m2;
													m[3]=m3;
													m[4]=m4;
													m[5]=m5;
													m[6]=m6;
													m[7]=m7;
													m[8]=m8; }

	///Constructor from an array
	matrix3<T> (const T* array) {	m[0]=array[0];
								m[1]=array[1];
								m[2]=array[2];
								m[3]=array[3];
								m[4]=array[4];
								m[5]=array[5];
								m[6]=array[6];
								m[7]=array[7];
								m[8]=array[8]; }
	
	///Copy constructor
	matrix3<T> (const matrix3<T>& mv) {	m[0]=mv.m[0];
								m[1]=mv.m[1];
								m[2]=mv.m[2];
								m[3]=mv.m[3];
								m[4]=mv.m[4];
								m[5]=mv.m[5];
								m[6]=mv.m[6];
								m[7]=mv.m[7];
								m[8]=mv.m[8]; }

	///Hybrid copy constructor
	template <typename T2>
	matrix3<T> (const matrix3<T2>& mv) {	
	  m[0]=  mv.m[0];
									m[1]=mv.m[1];
									m[2]=mv.m[2];
									m[3]=mv.m[3];
									m[4]=mv.m[4];
									m[5]=mv.m[5];
									m[6]=mv.m[6];
									m[7]=mv.m[7];
									m[8]=mv.m[8]; }

	///Destructor
	~matrix3(){}
	
	//Accessors
	///ith element i considering the matrix as an array
	T& operator [] (unsigned int i)	{ return m[i]; }

	///(i,j) element
	T& operator () (unsigned int i, unsigned int j) { return m[3*i+j]; }

	///Set to zero matrix
	matrix3& setZero(void) {	m[0]=(0);
								m[1]=(0);
								m[2]=(0);
								m[3]=(0);
								m[4]=(0);
								m[5]=(0);
								m[6]=(0);
								m[7]=(0);
								m[8]=(0);
								return *this; }

	///Set to identity
	matrix3& setIdentity(void)	{	m[0]=(1);
									m[1]=(0);
									m[2]=(0);
									m[3]=(0);
									m[4]=(1);
									m[5]=(0);
									m[6]=(0);
									m[7]=(0);
									m[8]=(1);
									return *this; }

	///Local matrix addition
	void operator += (const matrix3<T>& mv)	{	m[0] += mv.m[0];
												m[1] += mv.m[1];
												m[2] += mv.m[2];
												m[3] += mv.m[3];
												m[4] += mv.m[4];
												m[5] += mv.m[5];
												m[6] += mv.m[6];
												m[7] += mv.m[7];
												m[8] += mv.m[8]; }

	///Local matrix substraction
	void operator -= (const matrix3<T>& mv)	{	m[0] -= mv.m[0];
												m[1] -= mv.m[1];
												m[2] -= mv.m[2];
												m[3] -= mv.m[3];
												m[4] -= mv.m[4];
												m[5] -= mv.m[5];
												m[6] -= mv.m[6];
												m[7] -= mv.m[7];
												m[8] -= mv.m[8]; }

	///Local matrix multiplication
	void operator *= (const matrix3<T>& mv)	{	matrix3<T> temp(*this);
												m[0] = temp.m[0] * mv.m[0] + temp.m[1] * mv.m[3] + temp.m[2] * mv.m[6];
												m[1] = temp.m[0] * mv.m[1] + temp.m[1] * mv.m[4] + temp.m[2] * mv.m[7];
												m[2] = temp.m[0] * mv.m[2] + temp.m[1] * mv.m[5] + temp.m[2] * mv.m[8];
												m[3] = temp.m[3] * mv.m[0] + temp.m[4] * mv.m[3] + temp.m[5] * mv.m[6];
												m[4] = temp.m[3] * mv.m[1] + temp.m[4] * mv.m[4] + temp.m[5] * mv.m[7];
												m[5] = temp.m[3] * mv.m[2] + temp.m[4] * mv.m[5] + temp.m[5] * mv.m[8];
												m[6] = temp.m[6] * mv.m[0] + temp.m[7] * mv.m[3] + temp.m[8] * mv.m[6];
												m[7] = temp.m[6] * mv.m[1] + temp.m[7] * mv.m[4] + temp.m[8] * mv.m[7];
												m[8] = temp.m[6] * mv.m[2] + temp.m[7] * mv.m[5] + temp.m[8] * mv.m[8]; }


	///mat4f addition
	matrix3<T> operator + (const matrix3<T>& mv) const	{	matrix3<T> result;
														result.m[0] = m[0] + mv.m[0];
														result.m[1] = m[1] + mv.m[1];
														result.m[2] = m[2] + mv.m[2];
														result.m[3] = m[3] + mv.m[3];
														result.m[4] = m[4] + mv.m[4];
														result.m[5] = m[5] + mv.m[5];
														result.m[6] = m[6] + mv.m[6];
														result.m[7] = m[7] + mv.m[7];
														result.m[8] = m[8] + mv.m[8];
														return result; }

	///mat4f substraction
	matrix3<T> operator - (const matrix3<T>& mv) const	{	matrix3<T> result(*this);
														result.m[0] = m[0] - mv.m[0];
														result.m[1] = m[1] - mv.m[1];
														result.m[2] = m[2] - mv.m[2];
														result.m[3] = m[3] - mv.m[3];
														result.m[4] = m[4] - mv.m[4];
														result.m[5] = m[5] - mv.m[5];
														result.m[6] = m[6] - mv.m[6];
														result.m[7] = m[7] - mv.m[7];
														result.m[8] = m[8] - mv.m[8];
														return result;	}

	///mat4f multiplication
	matrix3<T> operator * (const matrix3<T>& mv) const	
	{	
		matrix3<T> result;
		result.m[0] = m[0] * mv.m[0] + m[1] * mv.m[3] + m[2] * mv.m[6];
		result.m[1] = m[0] * mv.m[1] + m[1] * mv.m[4] + m[2] * mv.m[7];
		result.m[2] = m[0] * mv.m[2] + m[1] * mv.m[5] + m[2] * mv.m[8];
		result.m[3] = m[3] * mv.m[0] + m[4] * mv.m[3] + m[5] * mv.m[6];
		result.m[4] = m[3] * mv.m[1] + m[4] * mv.m[4] + m[5] * mv.m[7];
		result.m[5] = m[3] * mv.m[2] + m[4] * mv.m[5] + m[5] * mv.m[8];
		result.m[6] = m[6] * mv.m[0] + m[7] * mv.m[3] + m[8] * mv.m[6];
		result.m[7] = m[6] * mv.m[1] + m[7] * mv.m[4] + m[8] * mv.m[7];
		result.m[8] = m[6] * mv.m[2] + m[7] * mv.m[5] + m[8] * mv.m[8];
		return result;
	}

	matrix3<T> operator * (const double & r) const	
	{	
		matrix3<T> result;
		result.m[0] = m[0] * r;
		result.m[1] = m[1] * r;
		result.m[2] = m[2] * r;
		result.m[3] = m[3] * r;
		result.m[4] = m[4] * r;
		result.m[5] = m[5] * r;
		result.m[6] = m[6] * r;
		result.m[7] = m[7] * r;
		result.m[8] = m[8] * r;
		return result;
	}


	vector3<T>	operator * (const vector3<T>& v) const {
															vector3<T>	vr;
															vr.x = m[0]*v.x+m[1]*v.y+m[2]*v.z;
															vr.y = m[3]*v.x+m[4]*v.y+m[5]*v.z;
															vr.z = m[6]*v.x+m[7]*v.y+m[8]*v.z;
															return vr;
														}

	///Transposition
	void transposeTo(matrix3& mv) const	{	mv.m[0] = m[0];
											mv.m[1] = m[3];
											mv.m[2] = m[6];
											mv.m[3] = m[1];
											mv.m[4] = m[4];
											mv.m[5] = m[7];
											mv.m[6] = m[2];
											mv.m[7] = m[5];
											mv.m[8] = m[8]; };
	///Inversion
	void inverseTo(matrix3& mv) const	{	T det = 1 / determinant();
											mv.m[0] = ( m[4]*m[8] - m[5]*m[7] ) * det;
											mv.m[1] = ( m[2]*m[7] - m[1]*m[8] ) * det;
											mv.m[2] = ( m[1]*m[5] - m[2]*m[4] ) * det;
											mv.m[3] = ( m[5]*m[6] - m[3]*m[8] ) * det;
											mv.m[4] = ( m[0]*m[8] - m[2]*m[6] ) * det;
											mv.m[5] = ( m[2]*m[3] - m[0]*m[5] ) * det;
											mv.m[6] = ( m[3]*m[7] - m[4]*m[6] ) * det;
											mv.m[7] = ( m[1]*m[6] - m[0]*m[7] ) * det;
											mv.m[8] = ( m[0]*m[4] - m[1]*m[3] ) * det; }

	///Local transposition
	matrix3<T>& transpose()	{	T temp = m[1];
								m[1] = m[3];
								m[3] = temp;
								temp = m[2];
								m[2] = m[6];
								m[6] = temp;
								temp = m[5];
								m[5] = m[7];
								m[7] = temp;
								return *this; }
	
	///Local inversion
	matrix3<T>& inverse() {  	matrix3<T> temp(*this);
							T det = 1 / determinant();
							m[0] = ( temp.m[4]*temp.m[8] - temp.m[5]*temp.m[7] ) * det;
							m[1] = ( temp.m[2]*temp.m[7] - temp.m[1]*temp.m[8] ) * det;
							m[2] = ( temp.m[1]*temp.m[5] - temp.m[2]*temp.m[4] ) * det;
							m[3] = ( temp.m[5]*temp.m[6] - temp.m[3]*temp.m[8] ) * det;
							m[4] = ( temp.m[0]*temp.m[8] - temp.m[2]*temp.m[6] ) * det;
							m[5] = ( temp.m[2]*temp.m[3] - temp.m[0]*temp.m[5] ) * det;
							m[6] = ( temp.m[3]*temp.m[7] - temp.m[4]*temp.m[6] ) * det;
							m[7] = ( temp.m[1]*temp.m[6] - temp.m[0]*temp.m[7] ) * det;
							m[8] = ( temp.m[0]*temp.m[4] - temp.m[1]*temp.m[3] ) * det;
							return *this; }

	//Characteristical values
	///Determinant
	T determinant() const	{ return  m[0]*m[4]*m[8]
									+ m[1]*m[5]*m[6]
									+ m[2]*m[3]*m[7]
									- m[2]*m[4]*m[6]
									- m[0]*m[5]*m[7]
									- m[1]*m[3]*m[8]; }

	///Trace
	T trace() const	{ return m[0]+m[4]+m[8]; }

	///Eigen values
	void eigenValue(T& vp1, T& vp2, T& vp3) const;
	void eigenValue(T* vp) const;

	///Cast
	  //operator matrix3<T>&(){return *this;}

	//Static members
	///Identity matrix
	static const matrix3<T> identity;

	///Zero matrix
	static const matrix3<T> zero;

	void display( void)
	  {
	    for(int i=0;i<9;i++)
	      cout << m[i] << " ";
	  }
};

template <typename T>
const matrix3<T> matrix3<T>::identity((1), (0), (0),
									  (0), (1), (0),
									  (0), (0), (1));

template <typename T>
const matrix3<T> matrix3<T>::zero((0));
#endif

/**
 *	mathout
 *
 *	\author Aurélien Pocheville
 *
 *	(C) 2004 Aurélien Pocheville
 */


//#if !defined(MATH_OUPUT_DEFINED)

#ifndef __PG_MATH_OUTPUT_DEFINED_
#define __PH_MATH_OUTPUT_DEFINED_

#include <iostream>
#include <iomanip>

#include "matrix3.h"
#include "matrix4.h"
#include "vector3.h"
#include "quaternion.h"
#include "transformation.h"

const unsigned int c_iMatPrecision = 4;
const unsigned int c_iQuatVectPrecision = 6;

template <typename T>
std::ostream& operator << (std::ostream& o, quaternion<T> q)
{
	o << "quat : x: " << std::setw(c_iQuatVectPrecision) << std::setprecision(c_iQuatVectPrecision) << q.x << "; y: " << std::setw(c_iQuatVectPrecision) << std::setprecision(c_iQuatVectPrecision) << q.y << "; z: " << std::setw(c_iQuatVectPrecision) << std::setprecision(c_iQuatVectPrecision) << q.z << ";  s: " << std::setw(c_iQuatVectPrecision) << std::setprecision(c_iQuatVectPrecision) << q.s;
	return o;
}

template <typename T>
std::ostream& operator << (std::ostream& o, vector3<T> v)
{
	o << "vect : x: " << std::setw(c_iQuatVectPrecision) << std::setprecision(c_iQuatVectPrecision) << v.x << "; y: " << std::setw(c_iQuatVectPrecision) << std::setprecision(c_iQuatVectPrecision) << v.y << "; z: " << std::setw(c_iQuatVectPrecision) << std::setprecision(c_iQuatVectPrecision) << v .z;
	return o;
}

template <typename T>
std::ostream& operator << (std::ostream& o, transformation<T> t)
{
	o << "transformation : " << endl << "\t";
	o << t.q << endl << "\t";
	o << t.t;
	return o;
}

template <typename T>
std::ostream& operator << (std::ostream& o, matrix3<T> m)
{
	o << "mat3 : " << endl;
	o << "( "	<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[0] << " , " 
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[1] << " , "
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[2] << endl;
	o << "  "	<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[3] << " , " 
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[4] << " , "
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[5] << endl;
	o << "  "	<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[6] << " , " 
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[7] << " , "
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[8] << " ) ";
	return o;
}

template <typename T>
std::ostream& operator << (std::ostream& o, matrix4<T> m)
{
	o << "mat4 : " << endl;
	o << "( "	<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[0] << " , " 
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[1] << " , "
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[2] << " , "
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[3] << endl;
	o << "  "	<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[4] << " , " 
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[5] << " , "
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[6] << " , "
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[7] << endl;
	o << "  "	<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[8] << " , " 
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[9] << " , "
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[10] << " , "
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[11] << endl;
	o << "  "	<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[12] << " , " 
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[13] << " , "
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[14] << " , "
				<< std::setw(c_iMatPrecision) << std::setprecision(c_iMatPrecision) << m.m[15] << " ) ";
	return o;
}

#endif //MATH_OUPUT_DEFINED

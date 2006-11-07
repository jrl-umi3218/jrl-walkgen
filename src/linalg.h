/**
 *	Linear algebra inclusion header
 *
 */

#if !defined(LINALG_H_DEFINED_2004_07_27)
#define LINALG_H_DEFINED_2004_07_27

#include "vector3.h"
#include "matrix3.h"
#include "matrix4.h"
#include "quaternion.h"
#include "transformation.h"
#include "mathout.h"
//#include "mathparse.h"

typedef matrix3<float>			matrix3f;
typedef matrix3<double>			matrix3d;
typedef matrix4<float>			matrix4f;
typedef matrix4<double>			matrix4d;
typedef vector3<float>			vector3f;
typedef vector3<double>			vector3d;
typedef quaternion<float>		quatf;
typedef transformation<float>	tranf;

//Hack !!
typedef vector3<float>			Vector3f;
typedef vector3<double>			Vector3d;
typedef vector3<int>			Vector3i;
//typedef	vector3<float>			Vector;
typedef vector3<unsigned char>	Vector3uc;

#endif //LINALG_H_DEFINED_2004_07_27

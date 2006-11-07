/**
 *	transformation
 *
 *	\author Aurélien Pocheville
 *
 *	(C) 2004 Aurélien Pocheville
 */

#if !defined(TRANSFORMATION_H_INCLUDE)
#define TRANSFORMATION_H_INCLUDE

#include "quaternion.h"
#include "vector3.h"
#include "matrix3.h"
#include "matrix4.h"

template <typename T>
class transformation
{
public:
	quaternion<T>	q;
	vector3<T>		t;

	/// Operator *
	inline void operator * (const transformation<T>& tr)
	{
		vector3<T> nt = tr.t;
		nt.rotateBy(q);
		t += nt;
		q *= tr.q;
	} 

	/// Transform a vector
	inline vector3<T>	operator *(const vector3<T>& v) const
	{
		vector3<T> vr(v);
		q.rotate(vr);
		return vr+t;
	}


	/// Tranform a vector by a transformation
	inline void tranform(vector3<T>& v) const 
		{
			vector3<T> vr;
			vector3<T> vq(q.x,q.y,q.z);
			vr = v*(q.s*q.s) + vq*(v % vq) - ((v ^ vq)*2*q.s) - ( ( vq  ^  (v) ) ^ vq );
			v = vr + t;
		}
	
	/// Get the matrix associated
	inline matrix4<T> getTranformationMatrix() const
		{
			// 9 muls, 15 adds
			T x2 = q.x + q.x; // *2
			T y2 = q.y + q.y; // *2
			T z2 = q.z + q.z; // *2
			T xx = q.x * x2;  T xy = q.x * y2;  T xz = q.x * z2;
			T yy = q.y * y2;  T yz = q.y * z2;  T zz = q.y * z2;
			T wx = q.s * x2;  T wy = q.s * y2;  T wz = q.s * z2;

			matrix4<T>	m;
			
			m.m[3] = t.x;
			m.m[7] = t.y;
			m.m[11] = t.z;
			m.m[12] = m[13] = m[14] = 0; 
			m.m[15] = 1;
			m.m[0] = (1 -(yy+zz)); m.m[4] = (xy+wz);      m.m[8] = (xz-wy);
			m.m[1] = (xy-wz);      m.m[5] = (1-(xx+zz));  m.m[9] = (yz+wx);
			m.m[2] = (xz+wy);      m.m[6] = (yz-wx);      m.m[10] = (1-(xx+yy));

			return m;
		}

	/// Get the matrix associated
	inline void getTranformationMatrixTo(matrix4<T>& m) const
		{
			// 9 muls, 15 adds
			T x2 = q.x + q.x; // *2
			T y2 = q.y + q.y; // *2
			T z2 = q.z + q.z; // *2
			T xx = q.x * x2;  T xy = q.x * y2;  T xz = q.x * z2;
			T yy = q.y * y2;  T yz = q.y * z2;  T zz = q.y * z2;
			T wx = q.s * x2;  T wy = q.s * y2;  T wz = q.s * z2;
	
			m.m[3] = t.x;
			m.m[7] = t.y;
			m.m[11] = t.z;
			m.m[12] = m[13] = m[14] = 0; 
			m.m[15] = 1;
			m.m[0] = (1 -(yy+zz)); m.m[4] = (xy+wz);      m.m[8] = (xz-wy);
			m.m[1] = (xy-wz);      m.m[5] = (1-(xx+zz));  m.m[9] = (yz+wx);
			m.m[2] = (xz+wy);      m.m[6] = (yz-wx);      m.m[10] = (1-(xx+yy));
		} 
		
	/// Get the matrix associated
	inline matrix3<T> getRotationMatrix() const
		{
			// 9 muls, 15 adds
			T x2 = q.x + q.x; // *2
			T y2 = q.y + q.y; // *2
			T z2 = q.z + q.z; // *2
			T xx = q.x * x2;  T xy = q.x * y2;  T xz = q.x * z2;
			T yy = q.y * y2;  T yz = q.y * z2;  T zz = q.y * z2;
			T wx = q.s * x2;  T wy = q.s * y2;  T wz = q.s * z2;

			matrix3<T>	m;
			
			m.m[0] = (1 -(yy+zz)); m.m[3] = (xy+wz);      m.m[6] = (xz-wy);
			m.m[1] = (xy-wz);      m.m[4] = (1-(xx+zz));  m.m[7] = (yz+wx);
			m.m[2] = (xz+wy);      m.m[5] = (yz-wx);      m.m[8] = (1-(xx+yy));

			return m;
		}
	
	/// Get the matrix associated
	inline void getRotationMatrixTo(matrix3<T>& m) const
		{
			// 9 muls, 15 adds
			T x2 = q.x + q.x; // *2
			T y2 = q.y + q.y; // *2
			T z2 = q.z + q.z; // *2
			T xx = q.x * x2;  T xy = q.x * y2;  T xz = q.x * z2;
			T yy = q.y * y2;  T yz = q.y * z2;  T zz = q.y * z2;
			T wx = q.s * x2;  T wy = q.s * y2;  T wz = q.s * z2;

			m.m[0] = (1 -(yy+zz)); m.m[3] = (xy+wz);      m.m[6] = (xz-wy);
			m.m[1] = (xy-wz);      m.m[4] = (1-(xx+zz));  m.m[7] = (yz+wx);
			m.m[2] = (xz+wy);      m.m[5] = (yz-wx);      m.m[8] = (1-(xx+yy));
		}

};


#endif

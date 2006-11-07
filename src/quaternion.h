/**
 *	quaternion
 *
 *	\author Aurélien Pocheville
 *
 *	(C) 2004 Aurélien Pocheville
 */
#if !defined(QUATERNION_H_INCLUDE)
#define QUATERNION_H_INCLUDE
#include "vector3.h"
#include "matrix3.h"
template <typename T>
class quaternion
{
public:
	T x,y,z, s;
	/// Default constructor
	quaternion<T>()
		{}
	/// Copy constructor
	quaternion<T>(const quaternion<T>& q):x(q.x),y(q.y),z(q.z),s(q.s)
		{}
	/// Constructor with values
	quaternion<T>(T _x, T _y, T _z, T _s):x(_x),y(_y),z(_z),s(_s)
		{}
	/// Fill in constructor
	inline quaternion<T>(const T& t):x(t),y(t),z(t),s(t)
		{}
	/// Assignement operator
	inline quaternion<T> operator = ( const quaternion<T>& q)
		{x = q.x ; y = q.y ; z = q.z ; s = q.s ; return *this;}

	/// Operator +
	inline quaternion<T> operator + ( const quaternion<T>& q) const
		{
			quaternion<T> qr;
			qr.x = x + q.x;
			qr.y = y + q.y;
			qr.z = z + q.z;
			qr.s = s + q.s;
			return qr;
		}
	/// Operator -
	inline quaternion<T> operator - ( const quaternion<T>& q) const
		{
			quaternion<T> qr;
			qr.x = x - q.x;
			qr.y = y - q.y;
			qr.z = z - q.z;
			qr.s = s - q.s;
			return qr;
		}
	/// Operator *
	inline quaternion<T> operator * ( const quaternion<T>&q) const
		{
			quaternion qr;
			// 12 muls, 30 adds
			T e = (z + x)*(q.x + q.y);
			T f = (z - x)*(q.x - q.y);
			T g = (s + y)*(q.s - q.z);
			T h = (s - y)*(q.s + q.z);
			T a = f - e;
			T b = f + e;
			qr.x = (s + x)*(q.s + q.x) + (a - g - h) * static_cast<T>(0.5);
			qr.y = (s - x)*(q.y + q.z) + (b + g - h) * static_cast<T>(0.5);
			qr.z = (z + y)*(q.s - q.x) + (b - g + h) * static_cast<T>(0.5);
			qr.s = (z - y)*(q.y - q.z) + (a + g + h) * static_cast<T>(0.5);
			return qr;
		}
	/// Operator +=
	inline void operator += ( const quaternion<T>& q)
		{
			x += q.x;
			y += q.y;
			z += q.z;
			s += q.s;
		}
	/// Operator -=
	inline void operator -= ( const quaternion<T>& q)
		{
			x -= q.x;
			y -= q.y;
			z -= q.z;
			s -= q.s;
		}

	/// Operator *
	inline  void operator *= ( const quaternion<T>&q)
		{
			quaternion qr;
			// 12 muls, 30 adds
			double e = (z + x)*(q.x + q.y);
			double f = (z - x)*(q.x - q.y);
			double g = (s + y)*(q.s - q.z);
			double h = (s - y)*(q.s + q.z);
			double a = f - e;
			double b = f + e;
			qr.x = (s + x)*(q.s + q.x) + (a - g - h) * 0.5;
			qr.y = (s - x)*(q.y + q.z) + (b + g - h) * 0.5;
			qr.z = (z + y)*(q.s - q.x) + (b - g + h) * 0.5;
			qr.s = (z - y)*(q.y - q.z) + (a + g + h) * 0.5;
			*this = qr;
		}
	/// Operator *
	inline quaternion<T> operator * ( const T& t) const
		{
			quaternion<T> qr;
			qr.x = x * t;
			qr.y = y * t;
			qr.z = z * t;
			qr.s = s * t;
			return qr;
		}
	/// Operator /
	inline quaternion<T> operator / ( const T& t) const
		{
			vector3<T> qr;
			qr.x = x / t;
			qr.y = y / t;
			qr.z = z / t;
			qr.s = s / t;
			return qr;
		}
	/// Operator *=
	inline void operator *= ( const T& t)
		{
			x *= t;
			y *= t;
			z *= t;
			s *= t;
		}
	/// Operator /=
	inline void operator /= ( const T& t)
		{
			x /= t;
			y /= t;
			z /= t;
			s /= t;
		}
	/// Normalization
	inline void	normalize ()
		{
			T in = static_cast<T>(static_cast<T>(1)/sqrt(x*x+y*y+z*z+s*s));
			x *= in;
			y *= in;
			z *= in;
			s *= in;
		}
	/// Inverse
	inline void	inverse ()
		{
			//s = -s;
			//This is less dangerous
			x = -x;
			y = -y;
			z = -z;
		}
	/// Inverse
	inline quaternion<T>	getInverse()
		{
			return quaternion<T>(-x,-y,-z,s);
		}
	/// Rotation by a quaternion
	/// todo Optimize !!
	inline void	rotate(vector3<T>& v) const
		{
			vector3<T> vr;
			vector3<T> vq(x,y,z);
			vr = v*(s*s) + vq*(v % vq) - ((v ^ vq)*2*s) - ( ( vq  ^  (v) ) ^ vq );
			v = vr;
		}
	///Rotation by a quaternion
	inline vector3<T> operator * (vector3<T>& v) const
		{
			vector3<T> vq(x,y,z);
			return v*(s*s) + vq*(v % vq) - ((v ^ vq)*2*s) - ( ( vq  ^  (v) ) ^ vq );	
		}
	
	/// Get the matrix associated
	inline matrix3<T> getRotationMatrix() const
		{
			// 9 muls, 15 adds
			T x2 = x + x; // *2
			T y2 = y + y; // *2
			T z2 = z + z; // *2
			T xx = x * x2;  T xy = x * y2;  T xz = x * z2;
			T yy = y * y2;  T yz = y * z2;  T zz = z * z2;
			T wx = s * x2;  T wy = s * y2;  T wz = s * z2;
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
			T x2 = x + x; // *2
			T y2 = y + y; // *2
			T z2 = z + z; // *2
			T xx = x * x2;  T xy = x * y2;  T xz = x * z2;
			T yy = y * y2;  T yz = y * z2;  T zz = z * z2;
			T wx = s * x2;  T wy = s * y2;  T wz = s * z2;
			m.m[0] = (1 -(yy+zz)); m.m[3] = (xy+wz);      m.m[6] = (xz-wy);
			m.m[1] = (xy-wz);      m.m[4] = (1-(xx+zz));  m.m[7] = (yz+wx);
			m.m[2] = (xz+wy);      m.m[5] = (yz-wx);      m.m[8] = (1-(xx+yy));
		}
};

#endif //VECTOR3_H_INCLUDE

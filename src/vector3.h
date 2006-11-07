
/**
 *	vector3
 *
 *	\author Aurélien Pocheville
 *
 *	(C) 2004 Aurélien Pocheville
 */

#if !defined(VECTOR3_H_INCLUDE)
#define VECTOR3_H_INCLUDE
#define VECT_TO_ARG(v) (v.x, v.y, v.z)

#include <cmath>

const double	zeroConst = 1.0E-6;

template <typename T>
class vector3
{
 
public: 
	T x,y,z;

	/// Default constructor
	inline vector3<T>()
		{}
	/// Copy constructor
	inline vector3<T>(const vector3<T>& v):x(v.x),y(v.y),z(v.z)
		{}
	/// Constructor with values
	inline vector3<T>(const T& _x, const T& _y, const T& _z):x(_x),y(_y),z(_z)
		{}
	/// Fill in constructor
	inline vector3<T>(const T& t):x(t),y(t),z(t)
		{}
	/// Constructor with to points
	inline vector3<T>(const vector3<T>& a, const vector3<T>& b):x(b.x-a.x),y(b.y-a.y),z(b.z-a.z)
		{}

	/// Assignement operator
	inline vector3<T> operator = ( const vector3<T>& v)
		{x = v.x ; y = v.y ; z = v.z ; return *this;}

	///Hack operator
	inline T	operator[](int i)
	{
		return ((i==0)?x:(i==1)?y:z);
	}

	/// Unary - operator
	inline vector3<T> operator - ()
	{
		return vector3<T>(-x,-y,-z);
	}

	/// Equality operator
	inline bool operator == ( const vector3<T>& v)
	{ return ( ((v.x-x)<=zeroConst) && ((v.x-x)>=-zeroConst) && ((v.y-y)<=zeroConst) && ((v.y-y)>=-zeroConst) && ((v.z-z)<=zeroConst) && ((v.z-z)>=-zeroConst)); }

	/// Operator +
	inline vector3<T> operator + ( const vector3<T>& v) const
		{
			vector3<T> vr;
			vr.x = x + v.x;
			vr.y = y + v.y;
			vr.z = z + v.z;
			return vr;
		}

	/// Operator -
	inline vector3<T> operator - ( const vector3<T>& v) const
		{
			vector3<T> vr;
			vr.x = x - v.x;
			vr.y = y - v.y;
			vr.z = z - v.z;
			return vr;
		}

	/// Operator +=
	inline void operator += ( const vector3<T>& v)
		{
			x += v.x;
			y += v.y;
			z += v.z;
		}

	/// Operator -=
	inline void operator -= ( const vector3<T>& v)
		{
			x -= v.x;
			y -= v.y;
			z -= v.z;
		}


	/// Operator *
	inline vector3<T> operator * ( const T& t) const
		{
			vector3<T> vr;
			vr.x = x * t;
			vr.y = y * t;
			vr.z = z * t;
			return vr;
		}

	/// Operator /
	inline vector3<T> operator / ( const T& t) const
		{
			vector3<T> vr;
			vr.x = x / t;
			vr.y = y / t;
			vr.z = z / t;
			return vr;
		}

	/// Operator *=
	inline void operator *= ( const T& t)
		{
			x *= t;
			y *= t;
			z *= t;
		}

	/// Operator /=
	inline void operator /= ( const T& t)
		{
			x /= t;
			y /= t;
			z /= t;
		}

	/// Normalization
	inline void	normalize ()
		{
			T in = static_cast<T>(1/sqrt(x*x+y*y+z*z));
			x *= in;
			y *= in;
			z *= in;
		}


	/// Get the norm
	inline T norm() const
		{
			return static_cast<T>(sqrt(x*x+y*y+z*z));
		}

	/// Get the norm squared
	inline T normsquared() const
		{
			return (x*x+y*y+z*z);
		}

	/// Cross product
	inline vector3<T> operator ^ (const vector3<T>& v2) const
		{
			vector3<T> vr;
			vr.x = y*v2.z - v2.y*z;
			vr.y = z*v2.x - v2.z*x;
			vr.z = x*v2.y - v2.x*y;
			return vr;
		}

	/// Dot product
	inline T	operator % (const vector3<T>& v2) const
		{
			return x*v2.x + y*v2.y + z*v2.z;
		}
	
	/// Colinear ?
	inline bool	operator | (const vector3<T> v) const
		{
			return ( ((y*v.z - v.y*z) <= zeroConst ) && ((y*v.z - v.y*z) >= -zeroConst ) &&
					 ((z*v.x - v.z*x) <= zeroConst ) && ((z*v.x - v.z*x) >= -zeroConst ) &&
					 ((x*v.y - v.x*y) <= zeroConst ) && ((x*v.y - v.x*y) >= -zeroConst ) );
		}

	/// IsIn ?
	inline bool isInBetween(const vector3<T>& v1, const vector3<T>& v2) const
		{
			vector3<T> va = (*this - v1);
			vector3<T> vb = (v2 - v1);
			
			return ( (va | vb) && ( ( va.x >= 0 && va.x <= vb.x ) || ( va.x <= 0 && va.x >= vb.x ) ) );
		}

};

#endif //VECTOR3_H_INCLUDE

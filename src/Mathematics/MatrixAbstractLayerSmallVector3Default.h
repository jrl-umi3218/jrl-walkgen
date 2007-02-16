/*! This is a very fast and simple implementation
 * of a 3D vector class of double.
 * 
 * (c) Olivier Stasse, JRL, CNRS-AIST, ISRI, 2007
 */

#ifndef _VECTOR3D_PATTERNGENERATOR_JRL_
#define _VECTOR3D_PATTERNGENERATOR_JRL_

#include <iostream>

using namespace std;
#ifdef _DEFAULT_VECTOR3D_

namespace PatternGeneratorJRL
{
  
  /*! Template to handle 3 dimensional vector */
  template <typename T>
    struct Vector3D
    {
      public:
      
      T m_x,m_y,m_z;
      
      /*! Basic constructor: all the field 
	are set to zero. */
      inline Vector3D()
      { m_x= 0.0; m_y=0.0; m_z=0.0;}
      
      /*! Assignement operator */
      inline Vector3D<T> operator= (const struct Vector3D<T> &v)
      {
	m_x = v.m_x;
	m_y = v.m_y;
	m_z = v.m_z;
	return *this;
      }
	
	
      /*! Unary operator - */
      inline Vector3D<T> operator-()
      {
	return Vector3D<T>(-m_x, -m_y, -m_z);
      }
    
      /*! Array operator */
      inline T& operator[](unsigned i) 
      {
	return ((i==0) ? m_x: (i==1)? m_y:m_z);
      }

      /*! Array operator */
      inline T& operator()(unsigned i) 
      {
	return ((i==0) ? m_x: (i==1)? m_y:m_z);
      }

      /*! Binary operator == */
      inline bool operator==(const struct Vector3D<T> &v)
      {
	return ((v.m_x==m_x) &&
		(v.m_y==m_y) && 
		(v.m_z==m_z));
      }
    
      /*! Binary operator + */
      inline Vector3D<T> operator+ (const struct Vector3D<T> &v) const
      {
	Vector3D<T> vr;
	vr.m_x = m_x + v.m_x;
	vr.m_y = m_y + v.m_y;
	vr.m_z = m_z + v.m_z;
	return vr;
      }

      /*! Binary operator - */
      inline Vector3D<T> operator- (const struct Vector3D<T> &v) const
      {
	Vector3D<T> vr;
	vr.m_x = m_x - v.m_x;
	vr.m_y = m_y - v.m_y;
	vr.m_z = m_z - v.m_z;
	return vr;
      }
    
      /*! Binary operator += */
      inline void operator+= (const struct Vector3D<T> &v)
      {
	m_x += v.m_x;
	m_y += v.m_y;
	m_z += v.m_z;
      }

      /*! Binary operator -= */
      inline void operator-= (const struct Vector3D<T> &v) const
      {
	m_x -= v.m_x;
	m_y -= v.m_y;
	m_z -= v.m_z;
      }

      /*! Binary operator * */
      inline Vector3D<T> operator* (const T& t) const
      {
	Vector3D<T> vr;
	vr.m_x = m_x * t;
	vr.m_y = m_y * t;
	vr.m_z = m_z * t;
	return vr;
      }
    

      /*! Binary operator / */
      inline Vector3D<T> operator/ (const T &t) const
      {
	Vector3D<T> vr;
	vr.m_x = m_x/t;
	vr.m_y = m_y/t;
	vr.m_z = m_z/t;
	return vr;
      }

      /*! Binary operator *= */
      inline void operator*= (const T &t) 
      {
	m_x = m_x/t;
	m_y = m_y/t;
	m_z = m_z/t;
      }
    
      /*! Binary operator /= */
      inline void operator/= (const T &t) 
      {
	m_x = m_x/t;
	m_y = m_y/t;
	m_z = m_z/t;
      }
    
    
      /*! Normalize . */
      inline void normalize()
      {
	T in = static_cast<T> (1/sqrt(m_x*m_x+m_y*m_y+m_z*m_z));
	m_x *= in;
	m_y *= in;
	m_z *= in;
      }

      /*! Get the norm. */
      inline T norm() const
      {
	return static_cast<T>(sqrt(m_x*m_x+m_y*m_y+m_z*m_z));
      }

      /*! Get the norm squared */
      inline T normsquared() const
      {
	return (m_x*m_x+m_y*m_y+m_z*m_z);
      }

      /*! Cross product */
      inline struct Vector3D<T> operator ^ (const struct Vector3D<T>& v2) const
      {
	struct Vector3D<T> vr;
	vr.m_x = m_y*v2.m_z - v2.m_y*m_z;
	vr.m_y = m_z*v2.m_x - v2.m_z*m_x;
	vr.m_z = m_x*v2.m_y - v2.m_x*m_y;
	return vr;
      }
    
      inline friend std::ostream& operator<<(std::ostream &os,Vector3D<T> const &v) 
      {
	os << v.m_x << " " << v.m_y << " " << v.m_z << " ";
	return os;
      }
    };
};


#define MAL_S3_VECTOR(name,type)		\
  PatternGeneratorJRL::Vector3D<type> name

#define MAL_S3_VECTOR_SIZE(name)		\
  3 

#define MAL_S3_VECTOR_FILL(name,value)		\
  { name.m_x = value;				\
    name.m_y = value;				\
    name.m_z = value; }

#define MAL_S3_VECTOR_NORM(name)		\
  name.norm()

#define MAL_S3_VECTOR_CROSS_PRODUCT(res,v1,v2)	\
  res = v1 ^ v2;

#define MAL_S3_RET_VECTOR_DATABLOCK(name)	\
  &name

#endif

#endif /* _VECTOR3D_PATTERNGENERATOR_JRL_ */

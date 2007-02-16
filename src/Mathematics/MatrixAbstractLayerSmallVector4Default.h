/*! This is a very fast and simple implementation
 * of a 4D vector class of double.
 * 
 * 01/02/2007
 *
 * (c) Olivier Stasse, JRL, CNRS-AIST, ISRI, 2007 
 */

#ifndef _VECTOR4D_PATTERNGENERATOR_JRL_
#define _VECTOR4D_PATTERNGENERATOR_JRL_

#include <iostream>

using namespace std;
#ifdef _DEFAULT_VECTOR4D_

namespace PatternGeneratorJRL
{
  
  /*! Template to handle 3 dimensional vector */
  template <typename T>
    struct Vector4D
    {
      public:
      
      T m_x,m_y,m_z, m_w;
      
      /*! Basic constructor: all the field 
	are set to zero. */
      inline Vector4D()
      { m_x= 0.0; m_y=0.0; m_z=0.0; m_w=0.0;}
      
      /*! Assignement operator */
      inline Vector4D<T> operator= (const struct Vector4D<T> &v)
      {
	m_x = v.m_x;
	m_y = v.m_y;
	m_z = v.m_z;
	m_w = v.m_w;
	return *this;
      }
	
	
      /*! Unary operator - */
      inline Vector4D<T> operator-()
      {
	return Vector4D<T>(-m_x, -m_y, -m_z, -m_w);
      }
    
      /*! Array operator */
      inline T& operator[](unsigned i) 
      {
	return ((i==0) ? m_x: (i==1)? m_y: (i==2) ? m_z : m_w);
      }

      /*! Array operator */
      inline T& operator()(unsigned i) 
      {
	return ((i==0) ? m_x: (i==1)? m_y: (i==2) ? m_z : m_w);
      }

      /*! Binary operator == */
      inline bool operator==(const struct Vector4D<T> &v)
      {
	return ((v.m_x==m_x) &&
		(v.m_y==m_y) && 
		(v.m_z==m_z) &&
		(v.m_w==m_w));
      }
    
      /*! Binary operator + */
      inline Vector4D<T> operator+ (const struct Vector4D<T> &v) const
      {
	Vector4D<T> vr;
	vr.m_x = m_x + v.m_x;
	vr.m_y = m_y + v.m_y;
	vr.m_z = m_z + v.m_z;
	vr.m_w = m_w + v.m_w;
	return vr;
      }

      /*! Binary operator - */
      inline Vector4D<T> operator- (const struct Vector4D<T> &v) const
      {
	Vector4D<T> vr;
	vr.m_x = m_x - v.m_x;
	vr.m_y = m_y - v.m_y;
	vr.m_z = m_z - v.m_z;
	vr.m_w = m_w - v.m_w;
	return vr;
      }
    
      /*! Binary operator += */
      inline void operator+= (const struct Vector4D<T> &v)
      {
	m_x += v.m_x;
	m_y += v.m_y;
	m_z += v.m_z;
	m_w += v.m_w;
      }

      /*! Binary operator -= */
      inline void operator-= (const struct Vector4D<T> &v) const
      {
	m_x -= v.m_x;
	m_y -= v.m_y;
	m_z -= v.m_z;
	m_w -= v.m_w;
      }

      /*! Binary operator * */
      inline Vector4D<T> operator* (const T& t) const
      {
	Vector4D<T> vr;
	vr.m_x = m_x * t;
	vr.m_y = m_y * t;
	vr.m_z = m_z * t;
	vr.m_w = m_w * t;
	return vr;
      }
    

      /*! Binary operator / */
      inline Vector4D<T> operator/ (const T &t) const
      {
	Vector4D<T> vr;
	vr.m_x = m_x/t;
	vr.m_y = m_y/t;
	vr.m_z = m_z/t;
	vr.m_w = m_w/t;
	return vr;
      }

      /*! Binary operator *= */
      inline void operator*= (const T &t) 
      {
	m_x = m_x/t;
	m_y = m_y/t;
	m_z = m_z/t;
	m_w = m_w/t;
      }
    
      /*! Binary operator /= */
      inline void operator/= (const T &t) 
      {
	m_x = m_x/t;
	m_y = m_y/t;
	m_z = m_z/t;
	m_w = m_w/t;
      }
    
    
      /*! Normalize . */
      inline void normalize()
      {
	T in = static_cast<T> (1/sqrt(m_x*m_x+m_y*m_y+m_z*m_z+ m_w*m_w));
	m_x *= in;
	m_y *= in;
	m_z *= in;
	m_w *= in;
      }

      /*! Get the norm. */
      inline T norm() const
      {
	return static_cast<T>(sqrt(m_x*m_x+m_y*m_y+m_z*m_z+ m_w*m_w));
      }

      /*! Get the norm squared */
      inline T normsquared() const
      {
	return (m_x*m_x+m_y*m_y+m_z*m_z+m_w*m_w);
      }

    
      inline friend std::ostream& operator<<(std::ostream &os,Vector4D<T> const &v) 
      {
	os << v.m_x << " " << v.m_y << " " << v.m_z << " " << v.m_w;
	return os;
      }
    };
};


#define MAL_S4_VECTOR(name,type)		\
  PatternGeneratorJRL::Vector4D<type> name

#define MAL_S4_VECTOR_SIZE(name)		\
  3 

#define MAL_S4_VECTOR_FILL(name,value)		\
  { name.m_x = value;				\
    name.m_y = value;				\
    name.m_z = value;                           \
    name.m_w = value; }

#define MAL_S4_VECTOR_NORM(name)		\
  name.norm()

#define MAL_S4_RET_VECTOR_DATABLOCK(name)	\
  &name

#endif /* _DEFAULT_VECTOR4D_ */

#endif /* _VECTOR4D_PATTERNGENERATOR_JRL_ */

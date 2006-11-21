
#ifdef _WIN32

#include <cmath>
#include <algorithm>
//#include <xcomplex>
namespace std
{

	
	#if _MSC_VER < 1300 // Only needed for Dev studio 6 or below

	template <typename T>
inline T max(T const& a, T const& b)
{
  return (a > b) ? a : b;
}


template <typename T>
inline T min(T const& a, T const& b)
{
  return (a < b) ? a : b;
}

template <typename T>
inline T sqrt(T const & a)
{
	return ::sqrt(a);
}



inline  long double abs(long double  x) { return x >= 0 ? x : -x; }
inline double abs(double  x) { return x >= 0 ? x : -x; }
inline float  abs(float x) { return x >= 0 ? x : -x; }
#endif

//template <> inline double  abs< complex<double> >(complex<double> const & x) { return sqrt(norm(x)); }


//template <> inline std::complex<T> abs(std::complex const & a)
//{
//	return ::abs
//}



}
#endif




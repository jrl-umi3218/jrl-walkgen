dnl autoconf script for walkGenJRL libs
dnl
dnl Copyright (c) 2007 CNRS/LAAS
dnl
dnl
dnl Modified by Olivier Stasse 2007 CNRS/AIST
dnl
AC_INIT([walkGenJRL],[2.2],[openrobots@laas.fr])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE

AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_PROG_CXX

dnl AC_SUBST(BOOST_SANDBOX_CFLAGS)
dnl AC_SUBST(BOOST_CFLAGS)
dnl AC_SUBST(VNL_CFLAGS)
dnl AC_SUBST(VNL_LIB)
dnl
dnl AC_ARG_WITH(boost,
dnl	AS_HELP_STRING([--with-boost], [boost installation prefix]),
dnl	BOOST_CFLAGS=-I${withval}/include
dnl	CXXFLAGS="${CXXFLAGS} -I${withval}/include",
dnl	)
dnl
dnl AC_ARG_WITH(boost-sandbox, 
dnl	AS_HELP_STRING([--with-boost-sandbox], [boost-sandbox prefix (lapack bindings)]),
dnl	BOOST_SANDBOX_CFLAGS="-I${withval} -D_BOOST_MATRIX_"
dnl	CXXFLAGS="${CXXFLAGS} -I${withval}" 
dnl	CPPFLAGS="${CPPFLAGS} -D_BOOST_MATRIX_",
dnl	)
dnl
dnl AC_ARG_WITH(VNL,
dnl	AS_HELP_STRING([--with-VNL], [VNL linear algebra package prefix]),
dnl	VNL_CFLAGS="-I${withval}/include -D_VNL_MATRIX_"
dnl	VNL_LIB="-L${withval}/lib -lVNL"
dnl	CXXFLAGS="${CXXFLAGS} -D_VNL_MATRIX_ -I${withval}/include"
dnl	LDFLAGS="${LDFLAGS} -L${withval}/lib -lVNL",
dnl	)
dnl
dnl AC_ARG_WITH(lapack,
dnl	AS_HELP_STRING([--with-lapack], [lapack prefix]),
dnl	LAPACK_CFLAGS="-I${withval}/include"
dnl	LAPACK_LIBS="-L${withval}/lib",
dnl	)


PKG_CHECK_MODULES(DYNAMICJRLJAP, dynamicsJRLJapan >= 1.1 )
AC_SUBST(DYNAMICJRLJAP_CFLAGS)
AC_SUBST(DYNAMICJRLJAP_LIBS)


CXXFLAGS="${CXXFLAGS} -D_DEFAULT_MATRIX3x3_ -D_DEFAULT_VECTOR3D_"
dnl LDFLAGS="${LDFLAGS} -llapack"

AC_OUTPUT(Makefile
	src/Makefile
	include/walkGenJrl/Makefile
	include/walkGenJrl/Mathematics/Makefile
	include/walkGenJrl/MotionGeneration/Makefile
	include/walkGenJrl/PreviewControl/Makefile
	include/walkGenJrl/ZMPRefTrajectoryGeneration/Makefile
	doc/Makefile
	unitTesting/Makefile
	doc/pgjrl.dox
	walkGenJrl.pc
)

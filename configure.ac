dnl autoconf script for walkGenJRL libs
dnl
dnl Copyright (c) 2007 CNRS/LAAS
dnl
dnl
AC_INIT(src/PreviewControl/ZMPPreviewControlWithMultiBodyZMP.cpp)

AM_INIT_AUTOMAKE

AC_PROG_CXX

PACKAGE=walkGenJRL
VERSION=2.2
AC_SUBST(PACKAGE)
AC_SUBST(VERSION)

AC_SUBST(BOOST_SANDBOX_CFLAGS)
AC_SUBST(BOOST_CFLAGS)
AC_SUBST(VNL_CFLAGS)
AC_SUBST(VNL_LIB)

AC_PROG_LIBTOOL
AC_PROG_INSTALL

AC_ARG_WITH(boost,
	for ublas,
	BOOST_CFLAGS=-I${withval}/include
	CXXFLAGS="${CXXFLAGS} -I${withval}/include",
	)

AC_ARG_WITH(boost-sandbox, 
	for lapack bindings,
	BOOST_SANDBOX_CFLAGS="-I${withval} -D_BOOST_MATRIX_"
	CXXFLAGS="${CXXFLAGS} -I${withval}" 
	CPPFLAGS="${CPPFLAGS} -D_BOOST_MATRIX_",
	)


AC_ARG_WITH(VNL,
	for using VNL linear algebra package,
	VNL_CFLAGS="-I${withval}/include -D_VNL_MATRIX_"
	VNL_LIB="-L${withval}/lib -lVNL"
	CXXFLAGS="${CXXFLAGS} -D_VNL_MATRIX_ -I${withval}/include"
	LDFLAGS="${LDFLAGS} -L${withval}/lib -lVNL",
	)

AC_ARG_WITH(lapack,
	for lapack,
	LAPACK_CFLAGS="-I${withval}/include"
	LAPACK_LIBS="-L${withval}/lib",
	)

CXXFLAGS="${CXXFLAGS} -D_DEFAULT_MATRIX3x3_ -D_DEFAULT_VECTOR3D_"
LDFLAGS="${LDFLAGS} -llapack"

AC_OUTPUT(Makefile
	src/Makefile
	doc/Makefile
	doc/pgjrl.dox
	walkGenJrl.pc
)


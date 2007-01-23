dnl autoconf script for walkGenJRL libs
dnl
dnl Copyright (c) 2007 CNRS/LAAS
dnl
dnl
AC_INIT(src/ZMPPreviewControlWithMultiBodyZMP.cpp)

AM_INIT_AUTOMAKE

AC_PROG_CXX

PACKAGE=walkGenJRL
VERSION=2.0
AC_SUBST(PACKAGE)
AC_SUBST(VERSION)

AC_PROG_LIBTOOL
AC_PROG_INSTALL

AC_ARG_WITH(boost,
	for ublas,
	CXXFLAGS="${CXXFLAGS} -I${withval}/include",
	)

AC_ARG_WITH(boost-sandbox, 
	for lapack bindings,
	CXXFLAGS="${CXXFLAGS} -I${withval}" 
	CPPFLAGS="${CPPFLAGS=} -D_BOOST_MATRIX_",
	)

AC_ARG_WITH(VNL,
	for using VNL linear algebra package,
	CXXFLAGS="${CXXFLAGS} -D_VNL_MATRIX_ -I${withval}/include"
	LDFLAGS="${LDFLAGS} -L${withval}/lib -lVNL -llapack",
	)

AC_ARG_WITH(atlas,
	for lapack cblas and atlas,
	CXXFLAGS="${CXXFLAGS} -D_DEFAULT_MATRIX3x3_ -D_DEFAULT_VECTOR3D_ -I${withval}/include"
	LDFLAGS="${LDFLAGS} -L${withval}/lib -llapack -lcblas -latlas",
	CXXFLAGS="${CXXFLAGS} -D_DEFAULT_MATRIX3x3_ -D_DEFAULT_VECTOR3D_"
	LDFLAGS="${LDFLAGS} -L/usr/lib/atlas -llapack -lcblas -latlas")

AC_OUTPUT(Makefile
	src/Makefile
	doc/Makefile
	doc/pgjrl.dox
	walkGenJrl.pc
)


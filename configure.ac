dnl autoconf script for walkGenJRL libs
dnl
dnl Copyright (c) 2007 CNRS/LAAS
dnl
dnl
AC_INIT([walkGenJRL],[2.1],[openrobots@laas.fr])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_PROG_CXX

AC_SUBST(BOOST_SANDBOX_CFLAGS)
AC_SUBST(BOOST_CFLAGS)
AC_SUBST(VNL_CFLAGS)
AC_SUBST(VNL_LIB)

AC_ARG_WITH(boost,
	AS_HELP_STRING([--with-boost], [boost installation prefix]),
	BOOST_CFLAGS=-I${withval}/include
	CXXFLAGS="${CXXFLAGS} -I${withval}/include",
	)

AC_ARG_WITH(boost-sandbox, 
	AS_HELP_STRING([--with-boost-sandbox], [boost-sandbox prefix (lapack bindings)]),
	BOOST_SANDBOX_CFLAGS="-I${withval} -D_BOOST_MATRIX_"
	CXXFLAGS="${CXXFLAGS} -I${withval}" 
	CPPFLAGS="${CPPFLAGS} -D_BOOST_MATRIX_",
	)


AC_ARG_WITH(VNL,
	AS_HELP_STRING([--with-VNL], [VNL linear algebra package prefix]),
	VNL_CFLAGS="-I${withval}/include -D_VNL_MATRIX_"
	VNL_LIB="-L${withval}/lib -lVNL"
	CXXFLAGS="${CXXFLAGS} -D_VNL_MATRIX_ -I${withval}/include"
	LDFLAGS="${LDFLAGS} -L${withval}/lib -lVNL",
	)

AC_ARG_WITH(lapack,
	AS_HELP_STRING([--with-lapack], [lapack prefix]),
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


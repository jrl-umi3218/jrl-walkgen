dnl autoconf script for walkGenJRL libs
dnl
dnl Copyright (c) 2007 CNRS/LAAS
dnl
dnl
dnl Modified by Olivier Stasse 2007 CNRS/AIST
dnl

AC_INIT([walkGenJRL],[2.3],[openrobots@laas.fr])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE

AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_PROG_CXX


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

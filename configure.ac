dnl autoconf script for walkGenJRL libs
dnl
dnl Copyright (c) 2007 CNRS/LAAS
dnl
dnl
dnl Modified by Olivier Stasse 2007 CNRS/AIST
dnl

dnl define([walkGenJRL_version], [esyscmd([/usr/bin/git log -n 1 | /bin/grep commit | /usr/bin/awk '{print $2}' | /usr/bin/git describe | /usr/bin/tr -d '\n'])])
define([walkGenJRL_version], [2.5.0])

AC_INIT([walkGenJRL],[walkGenJRL_version],[olivier.stasse@aist.go.jp])
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
	include/walkGenJrl/FootTrajectoryGeneration/Makefile
	include/walkGenJrl/GlobalStrategyManagers/Makefile
	doc/Makefile
	unitTesting/Makefile
	doc/pgjrl.dox
	walkGenJrl.pc
)

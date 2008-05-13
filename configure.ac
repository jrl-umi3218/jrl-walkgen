dnl autoconf script for walkGenJRL libs
dnl
dnl Copyright (c) 2007 CNRS/LAAS
dnl
dnl
dnl Modified by Olivier Stasse 2007 CNRS/AIST
dnl

AC_INIT([walkGenJrl],[2.4.99],[openrobots@laas.fr])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE([foreign no-define])

AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_PROG_CXX

define(dynamicsjrljapan_reqd, [dynamicsJRLJapan >= 1.8])
AC_SUBST(DYNAMICSJRLJAPAN_REQD, "dynamicsjrljapan_reqd")
PKG_CHECK_MODULES(DYNAMICJRLJAP, dynamicsjrljapan_reqd)

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

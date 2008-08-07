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

define(dynamicjrljap_reqd, [dynamicsJRLJapan >= 1.1])
AC_SUBST(DYNAMICJRLJAP_REQD, "dynamicjrljap_reqd")
PKG_CHECK_MODULES(DYNAMICJRLJAP, dynamicjrljap_reqd)
AC_SUBST(DYNAMICJRLJAP_CFLAGS)
AC_SUBST(DYNAMICJRLJAP_LIBS)


CXXFLAGS="${CXXFLAGS} -D_DEFAULT_MATRIX3x3_ -D_DEFAULT_VECTOR3D_"
dnl LDFLAGS="${LDFLAGS} -llapack"

AC_ARG_WITH(boost,
	[AS_HELP_STRING([--with-boost],
	[Specify the inclusion directory for boost])],
	    AC_SUBST([BOOST_CXXFLAGS],[-I$withval]),
	    )

AC_ARG_WITH(boost-sandbox,  
	[AS_HELP_STRING([--with-boost-sandbox],
	[Specify the inclusion directory for boost-sandbox])],
	    AC_SUBST([BOOST_SANDBOX_CXXFLAGS],[-I$withval]),
	    )

CXXFLAGS=" ${CXXFLAGS} ${BOOST_CXXFLAGS} ${BOOST_SANDBOX_CXXFLAGS}"
	   
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
	plugin/Makefile.user
	plugin/Makefile
	doc/pgjrl.dox
	walkGenJrl.pc
)

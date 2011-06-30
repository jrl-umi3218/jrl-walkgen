Source: jrl-walkgen
Priority: extra
Maintainer: Thomas Moulard <thomas.moulard@gmail.com>
Build-Depends: debhelper (>= 7.0.50~), cmake (>= 2.6),
	       doxygen (>= 1.6.3),
	       pkg-config (>= 0.22),
	       perl (>= 5.10.1),
	       libboost-dev (>= 1.33.1),
	       libboost-thread-dev (>= 1.40.0),
	       liblapack-dev (>= 1.2),
	       libjrl-dynamics-dev (>= 1.9.3), @PACKAGE_HRP2_14_DATA@
	       libabstract-robot-dynamics-dev (>= 1.16.2)
Standards-Version: 3.8.4
Vcs-Git:git://github.com/jrl-umi3218/jrl-walkgen.git
Vcs-browser:http://github.com/jrl-umi3218/jrl-walkgen
Section: libs
Homepage:http://github.com/jrl-umi3218/jrl-walkgen

Package: libjrl-walkgen-dev
Section: libdevel
Architecture: any
Depends: libboost-dev (>= 1.33.1), libboost-thread-dev (>= 1.33.1), libjrl-dynamics-dev (>= 1.9.3), @PACKAGE_HRP2_14_DATA@ libabstract-robot-dynamics-dev (>= 1.16.2), liblapack-dev (>= 1.2), libjrl-walkgen3.1.4 (= ${binary:Version}), ${misc:Depends}
Suggests: libjrl-walkgen-doc
Description: the JRL walking pattern generator development package
 The JRL walking pattern generator generates a whole-body movement
 allowing biped robot to walk safely.
 It relies on abstract-robot-dynamics to realize dynamics computation.
 .
 This package contains development files (headers and pkg-config file).


Package: libjrl-walkgen3.1.4
Architecture: any
Depends: ${shlibs:Depends}, ${misc:Depends}
Description: the JRL walking pattern generator C++ library
 The JRL walking pattern generator generates a whole-body movement
 allowing biped robot to walk safely.
 It relies on abstract-robot-dynamics to realize dynamics computation.
 .
 This package contains the unversioned shared libraries.

Package: libjrl-walkgen-doc
Section: doc
Architecture: all
Depends: ${misc:Depends}
Suggests: libabstract-robot-dynamics-doc
Description: documentation for the JRL walking pattern generator
 The JRL walking pattern generator generates a whole-body movement
 allowing biped robot to walk safely.
 It relies on abstract-robot-dynamics to realize dynamics computation.
 .
 This package contains the Doxygen documentation.

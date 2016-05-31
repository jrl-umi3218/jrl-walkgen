jrl-walkgen
===========
jrl-umi travis results :
[![Build Status](https://travis-ci.org/jrl-umi3218/jrl-walkgen.png)](https://travis-ci.org/jrl-umi3218/jrl-walkgen)
[![Coverage Status](https://coveralls.io/repos/jrl-umi3218/jrl-walkgen/badge.png?branch=master)](https://coveralls.io/r/jrl-umi3218/jrl-walkgen?branch=master)

This software provides a pattern generator for biped robots.

It relies on the [StackOfTasks/Pinocchio][StackOfTasks/Pinocchio]
specification to realize dynamics computation.


Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The matrix abstract layer depends on several packages which
have to be available on your machine.

 - Libraries:
   - [pinocchio][https://github.com/stack-of-tasks/pinocchio] (>= 1.1.0)
     The pattern generator uses dynamics provided by the pinocchio template
   programming interface.
   - [jrl-mal][https://github.com/jrl-umi3218/jrl-mal] (>= 1.9.0)
   - [qpOASES][https://projects.coin-or.org/qpOASES] (normally compatible with
   the last version)
   - [simple-humanoid-description][https://github.com/MaximilienNaveau/simple_humanoid_description.git] ( >= 1.0.1)
   - Boost
   - Eigen

 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)



Additional notes regarding compilation on Microsoft Windows
-----------------------------------------------------------

This package has been tested using Microsoft Visual Studio 8.

 - Install MinGW
 - Make sure all environment variables are set up properly.
 - Generate the Makefile using the NMake Makefile generator.


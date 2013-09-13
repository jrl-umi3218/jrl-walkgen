jrl-walkgen
===========

[![Build Status](https://travis-ci.org/jrl-umi3218/jrl-walkgen.png)](https://travis-ci.org/jrl-umi3218/jrl-walkgen)
[![Coverage Status](https://coveralls.io/repos/jrl-umi3218/jrl-walkgen/badge.png?branch=master)](https://coveralls.io/r/jrl-umi3218/jrl-walkgen?branch=master)


This software provides a pattern generator for biped robots.

It relies on the [abstract-robot-dynamics][abstract-robot-dynamics]
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
   - [abstract-robot-dynamics][abstract-robot-dynamics] (>= 1.16)
     The pattern generator uses dynamics provided by the abstract interface.
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


[abstract-robot-dynamics]: http://github.com/laas/abstract-robot-dynamics

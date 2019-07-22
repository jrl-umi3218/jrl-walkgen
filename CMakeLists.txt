# Copyright 2010, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of jrl-walkgen.
# jrl-walkgen is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# jrl-walkgen is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# jrl-walkgen. If not, see <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Includes from jrl-cmaketools
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(CMakeDependentOption)

# Define properties of the project
SET(PROJECT_NAME jrl-walkgen)
SET(PROJECT_DESCRIPTION "Walking pattern generator for Humanoids")
SET(PROJECT_URL "")

SET(CXX_DISABLE_WERROR True)
SET(DOXYGEN_USE_MATHJAX YES)

# Initialize generic properties of the project
SETUP_PROJECT()

# Handle OS specificities
INCLUDE(CheckIncludeFiles)
CHECK_INCLUDE_FILES("sys/time.h" SYS_TIME_H)
IF(SYS_TIME_H)
ADD_DEFINITIONS("-DHAVE_SYS_TIME_H")
ENDIF(SYS_TIME_H)

# Required dependencies
ADD_REQUIRED_DEPENDENCY("pinocchio >=  2.1.0")

# Search for Boost.
# Boost.Test is used by the test suite.
# Boost program_options is used by the embedfile utility, which is used
# by metapod_robotbuilder
# Boost filesystem and regex are used by metapod_robotbuilder.
# Boost filesystem depends on Boost system.
SET(BOOST_COMPONENTS
  filesystem system unit_test_framework program_options regex)
SEARCH_FOR_BOOST()
# If Boost is recent enough, we look for Boost timer which can be used by
# by metapod_timer, which is in turn used by the benchmark.
# Boost timer depends on Boost chrono and system.
IF((BUILD_MULTI_MODEL_BENCHMARK OR BUILD_SINGLE_MODEL_BENCHMARKS)
   AND NOT Boost_VERSION LESS 104800)
  SET(BOOST_COMPONENTS ${BOOST_COMPONENTS} timer chrono system)
  SEARCH_FOR_BOOST()
ENDIF()

SEARCH_FOR_LAPACK()

# Search for Eigen.
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.5")
# Eigen (at least version 3.0.5) makes gcc report conversion warnings
# when one assigns to a block. This floods the compilation results.
# So we include Eigen headers as if they were system headers: the compiler
# won't report warning coming from them.
INCLUDE_DIRECTORIES(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# TODO kinda dirty patch to find lssol for now
#  using ADD_OPTIONAL_DEPENDENCY prevents the creation
#  of classic variables such as ${PKG}_FOUND
# hence we use ADD_REQUIRED_DEPENDENCY to get the data
OPTION(USE_LSSOL "Do you want to use the solver lssol?" OFF)
IF(USE_LSSOL)
  ADD_REQUIRED_DEPENDENCY("lssol >= 0.1.0")
ENDIF(USE_LSSOL)

OPTION(USE_QUADPROG "Do you want to use the solver eigen-quadprog?" ON)
OPTION(BUILD_TESTING OFF)

IF(USE_QUADPROG)
  ADD_REQUIRED_DEPENDENCY("eigen-quadprog >= 1.0.0")
ENDIF(USE_QUADPROG)

# Add aggressive optimization flags in release mode.
IF(CMAKE_COMPILER_IS_GNUCXX)
  SET (CMAKE_CXX_FLAGS_RELEASE
    "-O3 -DNDEBUG ")
#  SET (CMAKE_CXX_FLAGS
#    "-std=c++0x")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

#IF(CMAKE_COMPILER_IS_GNUCXX)
#  SET (CMAKE_CXX_FLAGS_RELEASE
#    "-O3 -funroll-loops -frerun-loop-opt -fschedule-insns2")
#  SET (CMAKE_CXX_FLAGS_RELEASE
#    "${CMAKE_CXX_FLAGS_RELEASE} -frerun-cse-after-loop -falign-functions")
#  SET (CMAKE_CXX_FLAGS_RELEASE
#    "${CMAKE_CXX_FLAGS_RELEASE} -falign-labels -falign-loops -falign-jumps")
#  SET (CMAKE_CXX_FLAGS_RELEASE
#    "${CMAKE_CXX_FLAGS_RELEASE} -fexpensive-optimizations")
#ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# Define the set of headers to be exported
SET(${PROJECT_NAME}_HEADERS
  include/jrl/walkgen/patterngeneratorinterface.hh
  include/jrl/walkgen/pgtypes.hh
  include/jrl/walkgen/pinocchiorobot.hh
)

# Define subdirectories to explore for cmake
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

# Generate dependency to jrl-walkgen in pc file
PKG_CONFIG_APPEND_LIBS("jrl-walkgen")

# Ask to generate the final steps.
SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

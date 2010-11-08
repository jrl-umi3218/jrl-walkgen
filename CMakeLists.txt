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

INCLUDE(cmake/base.cmake)

SET(PROJECT_NAME jrl-walkgen)
SET(PROJECT_VERSION 3.1.1.99)
SET(PROJECT_DESCRIPTION "Walking pattern generator for Humanoids")
SET(PROJECT_URL "")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

SET(CXX_DISABLE_WERROR True)

SETUP_PROJECT()

INCLUDE(CheckIncludeFiles)
CHECK_INCLUDE_FILES("sys/time.h" SYS_TIME_H)
ADD_DEFINITIONS("-DHAVE_SYS_TIME_H=${SYS_TIME_H}")

ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.9.0")
ADD_REQUIRED_DEPENDENCY("abstract-robot-dynamics >= 1.15")
ADD_REQUIRED_DEPENDENCY("jrl-dynamics >= 1.19.1")


ADD_OPTIONAL_DEPENDENCY("hrp2-dynamics >= 1.1.0")

# Add aggressive optimization flags in release mode.
IF(CMAKE_COMPILER_IS_GNUCXX)
  SET (CMAKE_CXX_FLAGS_RELEASE
    "-O3 -funroll-loops -frerun-loop-opt -fschedule-insns2")
  SET (CMAKE_CXX_FLAGS_RELEASE
    "${CMAKE_CXX_FLAGS_RELEASE} -frerun-cse-after-loop -falign-functions")
  SET (CMAKE_CXX_FLAGS_RELEASE
    "${CMAKE_CXX_FLAGS_RELEASE} -falign-labels -falign-loops -falign-jumps")
  SET (CMAKE_CXX_FLAGS_RELEASE
    "${CMAKE_CXX_FLAGS_RELEASE} -fexpensive-optimizations")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()

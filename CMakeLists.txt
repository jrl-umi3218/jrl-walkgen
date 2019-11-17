# Copyright 2010, Olivier Stasse, JRL, CNRS/AIST
# See LICENSE file

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME jrl-walkgen)
SET(PROJECT_DESCRIPTION "Walking pattern generator for Humanoids")
SET(PROJECT_URL "")

# Export CMake Target
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

SET(CXX_DISABLE_WERROR True)
SET(DOXYGEN_USE_MATHJAX YES)

# Includes from jrl-cmaketools
INCLUDE(cmake/base.cmake)

# Specify the project.
cmake_policy(SET CMP0048 NEW)
PROJECT(${PROJECT_NAME}
  LANGUAGES
  CXX C
  VERSION
  ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}
  )

find_package (Boost REQUIRED
  filesystem system unit_test_framework program_options regex)
find_package (Eigen3 REQUIRED NO_MODULE)

#ADD_PROJECT_DEPENDENCY(lapack REQUIRED)
#Include(LAPACK)
find_package(LAPACK)
#INCLUDE(cmake/cpack.cmake)
#INCLUDE(cmake/boost.cmake)
INCLUDE(CMakeDependentOption)

# Define properties of the project

# Handle OS specificities
INCLUDE(CheckIncludeFiles)
CHECK_INCLUDE_FILES("sys/time.h" SYS_TIME_H)
#IF(SYS_TIME_H)
ADD_DEFINITIONS("-DHAVE_SYS_TIME_H")
#ENDIF(SYS_TIME_H)

# Required dependencies
ADD_REQUIRED_DEPENDENCY("pinocchio >=  2.1.0")

#SEARCH_FOR_LAPACK()

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

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
  list(APPEND LOGGING_WATCHED_VARIABLES ${_variableName})
endforeach()

# Ask to generate the final steps.
SETUP_PROJECT_PACKAGE_FINALIZE()



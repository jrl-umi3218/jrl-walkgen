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

INCLUDE(CTest)

ADD_LIBRARY(${PROJECT_NAME}-test SHARED
  TestObject.cpp
  CommonTools.cpp
  ClockCPUTime.cpp
  DumpReferencesObjects.cpp
  ../src/portability/gettimeofday.cc )

PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME}-test pinocchio)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}-test ${PROJECT_NAME})

# Make sure private headers can be used.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

# Make the config_private.hh file available
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/private_cmake/config_private.hh.cmake
               ${CMAKE_BINARY_DIR}/include/${HEADER_DIR}/config_private.hh)

ADD_REQUIRED_DEPENDENCY("simple_humanoid_description >= 1.0.1")
#ADD_OPTIONAL_DEPENDENCY("hrp2_14_description >= 1.0.2")

#################
## Generic test #
#################
#SET(GENERIC_TEST)
IF(GENERIC_TEST)
  ADD_EXECUTABLE(TestFootPrintPGInterface
    TestFootPrintPGInterfaceData.cpp
    TestFootPrintPGInterface.cpp
    )

  TARGET_LINK_LIBRARIES(TestFootPrintPGInterface ${PROJECT_NAME})
  ADD_DEPENDENCIES(TestFootPrintPGInterface ${PROJECT_NAME})
ENDIF(GENERIC_TEST)

##########################
## Test Optimal Cholesky #
##########################
ADD_EXECUTABLE(TestOptCholesky
  TestOptCholesky.cpp
  ../src/Mathematics/OptCholesky.cpp
)
# Add test on the ricatti equation
ADD_TEST(TestOptCholesky TestOptCholesky)

##########################
## Test Bspline #
##########################
#TODO create a proper test for polynomes and bsplines
ADD_EXECUTABLE(TestBsplines
  TestBsplines.cpp
  ../src/Mathematics/Bsplines.cpp
  )

##########################
## Test Ricatti Equation #
##########################
CONFIG_FILES_CMAKE(TestRiccatiEquationWeightsWithoutInitialPose.datref)
CONFIG_FILES_CMAKE(TestRiccatiEquationWeightsWithInitialPose.datref)
ADD_EXECUTABLE(TestRiccatiEquation
  TestRiccatiEquation.cpp
  ../src/PreviewControl/OptimalControllerSolver.cpp
  )

# Add test on the ricatti equation
ADD_TEST(TestRiccatiEquation TestRiccatiEquation)
TARGET_LINK_LIBRARIES(TestRiccatiEquation ${LAPACK_LIBRARIES} ${PROJECT_NAME})

################################################
## Generic Macro That Create a Boost Test Case #
################################################
MATH(EXPR BITS "8*${CMAKE_SIZEOF_VOID_P}")
MACRO(ADD_JRL_WALKGEN_TEST test_arg test_file_name)
  SET(test_name "${test_arg}${BITS}")
  CONFIG_FILES_CMAKE(${test_name}TestFGPI.datref)
  MESSAGE(STATUS "Prepare test : " ${test_name})
  SET(urdfpath
    ${SIMPLE_HUMANOID_DESCRIPTION_PKGDATAROOTDIR}/simple_humanoid_description/urdf/simple_humanoid.urdf
  )
  SET(srdfpath
    ${SIMPLE_HUMANOID_DESCRIPTION_PKGDATAROOTDIR}/simple_humanoid_description/srdf/simple_humanoid.srdf
  )
  LIST(APPEND LOGGING_WATCHED_VARIABLES urdfpath srdfpath)
  ADD_TEST(${test_name} ${test_name} ${urdfpath} ${srdfpath})
ENDMACRO(ADD_JRL_WALKGEN_TEST)
#################################################
MACRO(ADD_JRL_WALKGEN_EXE test_arg test_file_name)

  SET(test_name "${test_arg}${BITS}")

  ADD_EXECUTABLE(${test_name}
    ${test_file_name}
  )

  TARGET_LINK_LIBRARIES(${test_name} ${PROJECT_NAME})
  TARGET_LINK_LIBRARIES(${test_name} ${PROJECT_NAME}-test)
  PKG_CONFIG_USE_DEPENDENCY(${test_name} pinocchio)

ENDMACRO(ADD_JRL_WALKGEN_EXE)

#######################
## Test Morisawa 2007 #
#######################

#disabled as it fail : fix the code
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007OnLine TestMorisawa2007.cpp)

# the following disabled test case are just useful code for offline
# motion generation using the specialized inverse kinmatics of a robot
#ADD_JRL_WALKGEN_EXE(TestMorisawa2007OnLine TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_EXE(TestMorisawa2007ShortWalk TestMorisawa2007.cpp)
##ADD_JRL_WALKGEN_EXE(TestMorisawa2007Climbing10 TestMorisawa2007.cpp)
##ADD_JRL_WALKGEN_EXE(TestMorisawa2007Climbing15 TestMorisawa2007.cpp)
##ADD_JRL_WALKGEN_EXE(TestMorisawa2007GoingDown10 TestMorisawa2007.cpp)
##ADD_JRL_WALKGEN_EXE(TestMorisawa2007GoingDown15 TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_EXE(TestMorisawa2007SteppingStones TestMorisawa2007.cpp)
##ADD_JRL_WALKGEN_EXE(TestMorisawa2007WalkingOnBeam TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_EXE(TestMorisawa2007GoThroughWall TestMorisawa2007.cpp)

#ADD_JRL_WALKGEN_TEST(TestMorisawa2007OnLine TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007ShortWalk TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007Climbing TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007GoingDown10 TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007GoingDown15 TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007SteppingStones TestMorisawa2007.cpp)
##ADD_JRL_WALKGEN_TEST(TestMorisawa2007WalkingOnBeam TestMorisawa2007.cpp)
#ADD_JRL_WALKGEN_TEST(TestMorisawa2007GoThroughWall TestMorisawa2007.cpp)

####################
## Test Herdt 2010 #
####################

#ADD_JRL_WALKGEN_EXE(TestHerdt2010OnLine TestHerdt2010.cpp)
#ADD_JRL_WALKGEN_EXE(TestHerdt2010EmergencyStop TestHerdt2010.cpp)
#ADD_JRL_WALKGEN_TEST(TestHerdt2010OnLine TestHerdt2010.cpp)
#ADD_JRL_WALKGEN_TEST(TestHerdt2010EmergencyStop TestHerdt2010.cpp)

############################
## Test Inverse Kinematics #
############################

#Disabled : fix the code to handl multiple inverse kinematics
# Todo compare halfsiting pose after forward and inverse kinematics.
#ADD_JRL_WALKGEN_EXE(TestInverseKinematics TestInverseKinematics.cpp)
#ADD_JRL_WALKGEN_TEST(TestInverseKinematics TestInverseKinematics.cpp)

###############################
## Test Dynamic Filter #
###############################

#TO BE FIXED with a proper inverse kinematics computed for the simple humanoid
#ADD_JRL_WALKGEN_EXE(TestDynamicFilter TestKajitaDynamicFilter.cpp)
#ADD_JRL_WALKGEN_TEST(TestDynamicFilter TestKajitaDynamicFilter.cpp)

#####################
## Test Kajita 2003 #
#####################

## These test is disabled for now as it fails.
## FIXME: fix the test and/or the implementation
#CONFIG_FILES_CMAKE(TestKajita2003StraightWalking${BITS}TestFGPI.datref)
#CONFIG_FILES_CMAKE(TestKajita2003Circle${BITS}TestFGPI.datref)
#CONFIG_FILES_CMAKE(TestKajita2003PbFlorentSeq1${BITS}TestFGPI.datref)
#CONFIG_FILES_CMAKE(TestKajita2003PbFlorentSeq2${BITS}TestFGPI.datref)
#CONFIG_FILES_CMAKE(TestKajita2003WalkingOnSpot${BITS}TestFGPI.datref)
ADD_JRL_WALKGEN_EXE(TestKajita2003StraightWalking TestKajita2003.cpp)
ADD_JRL_WALKGEN_EXE(TestKajita2003Circle          TestKajita2003.cpp)
ADD_JRL_WALKGEN_EXE(TestKajita2003PbFlorentSeq1   TestKajita2003.cpp)
ADD_JRL_WALKGEN_EXE(TestKajita2003PbFlorentSeq2   TestKajita2003.cpp)
ADD_JRL_WALKGEN_EXE(TestKajita2003WalkingOnSpot   TestKajita2003.cpp)

#IF(BUILD_TESTING)
#  ADD_JRL_WALKGEN_TEST(TestKajita2003StraightWalking TestKajita2003.cpp)
#  ADD_JRL_WALKGEN_TEST(TestKajita2003Circle          TestKajita2003.cpp)
#  ADD_JRL_WALKGEN_TEST(TestKajita2003PbFlorentSeq1   TestKajita2003.cpp)
#  ADD_JRL_WALKGEN_TEST(TestKajita2003PbFlorentSeq2   TestKajita2003.cpp)
#  ADD_JRL_WALKGEN_TEST(TestKajita2003WalkingOnSpot   TestKajita2003.cpp)
#ENDIF(BUILD_TESTING)

##########################
## Read Novela Data 2011 #
##########################

#ADD_JRL_WALKGEN_EXE(ReadDataForNovela ReadDataForNovela.cpp)
#ADD_JRL_WALKGEN_TEST(ReadDataForNovela ReadDataForNovela.cpp)

################
# NMPC test #
################

# Disabled as the test fail : random results oscillating around mean behaviour
ADD_JRL_WALKGEN_EXE(TestNaveau2015Online TestNaveau2015.cpp)
ADD_JRL_WALKGEN_EXE(TestNaveau2015OnlineSimple TestNaveau2015.cpp)

#####################
# Add user examples #
#####################
SET(EXAMPLES_SOURCES
  TestFootPrintPGInterfaceData.cpp
  TestFootPrintPGInterfaceData.h
  TestFootPrintPGInterface.cpp
  TestHerdt2010.cpp
  CommonTools.cpp
  CommonTools.hh
  examples.CMakeList
  )

INSTALL(FILES ${EXAMPLES_SOURCES}
  DESTINATION share/jrl/dynamics/examples/
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )

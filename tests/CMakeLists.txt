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

SET(urdfpath ${TALOS_DATA_PREFIX}/share/talos_data/urdf/talos_reduced_wpg.urdf)
SET(srdfpath ${TALOS_DATA_PREFIX}/share/talos_data/srdf/talos_wpg.srdf)


ADD_LIBRARY(${PROJECT_NAME}-test SHARED
  TestObject.cpp
  CommonTools.cpp
  ClockCPUTime.cpp
  DumpReferencesObjects.cpp
  ../src/portability/gettimeofday.cc )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}-test ${PROJECT_NAME} pinocchio::pinocchio)
TARGET_COMPILE_DEFINITIONS(${PROJECT_NAME}-test PUBLIC
  URDF_FULL_PATH="${urdfpath}" SRDF_FULL_PATH="${srdfpath}")

# Make sure private headers can be used.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

#################
## Generic test #
#################
#SET(GENERIC_TEST)
IF(GENERIC_TEST)
  ADD_UNIT_TEST(TestFootPrintPGInterface
    TestFootPrintPGInterfaceData.cpp
    TestFootPrintPGInterface.cpp
    )

  TARGET_LINK_LIBRARIES(TestFootPrintPGInterface ${PROJECT_NAME})
ENDIF(GENERIC_TEST)

##########################
## Test Optimal Cholesky #
##########################
ADD_UNIT_TEST(TestOptCholesky
  TestOptCholesky.cpp
  ../src/Mathematics/OptCholesky.cpp
  )
TARGET_LINK_LIBRARIES(TestOptCholesky ${PROJECT_NAME})

##########################
## Test Bspline #
##########################
#TODO create a proper test for polynomes and bsplines
#ADD_UNIT_TEST(TestBsplines
#TestBsplines.cpp
#../src/Mathematics/Bsplines.cpp
#)
#TARGET_LINK_LIBRARIES(TestBsplines ${PROJECT_NAME})

##########################
## Test Ricatti Equation #
##########################
CONFIG_FILES_CMAKE(TestRiccatiEquationWeightsWithoutInitialPose.datref)
CONFIG_FILES_CMAKE(TestRiccatiEquationWeightsWithInitialPose.datref)
ADD_UNIT_TEST(TestRiccatiEquation
  TestRiccatiEquation.cpp
  ../src/PreviewControl/OptimalControllerSolver.cpp
  )

# Add test on the ricatti equation
TARGET_LINK_LIBRARIES(TestRiccatiEquation ${LAPACK_LIBRARIES} ${PROJECT_NAME})

################################################
## Generic Macro That Create a Boost Test Case #
################################################
MATH(EXPR BITS "8*${CMAKE_SIZEOF_VOID_P}")
MACRO(ADD_JRL_WALKGEN_TEST test_arg test_file_name)
  # Create file name
  SET(test_name "${test_arg}${BITS}")
  # Create configuration file name
  CONFIG_FILES_CMAKE(${test_name}TestFGPI.datref)
  # Add unit test
  ADD_UNIT_TEST(${test_name} ${test_file_name})
  # Add library to the executable
  TARGET_LINK_LIBRARIES(${test_name} ${PROJECT_NAME} ${PROJECT_NAME}-test
    pinocchio::pinocchio)
ENDMACRO(ADD_JRL_WALKGEN_TEST)
#################################################
MACRO(ADD_JRL_WALKGEN_EXE test_arg test_file_name)

  SET(test_name "${test_arg}${BITS}")

  ADD_UNIT_TEST(${test_name} ${test_file_name})

  TARGET_LINK_LIBRARIES(${test_name} ${PROJECT_NAME} ${PROJECT_NAME}-test
    pinocchio::pinocchio)

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
IF(BUILD_TESTING)
  ADD_JRL_WALKGEN_TEST(TestKajita2003StraightWalking TestKajita2003.cpp)
  ADD_JRL_WALKGEN_TEST(TestKajita2003Circle          TestKajita2003.cpp)
  ADD_JRL_WALKGEN_TEST(TestKajita2003PbFlorentSeq1   TestKajita2003.cpp)
  ADD_JRL_WALKGEN_TEST(TestKajita2003WalkingOnSpot   TestKajita2003.cpp)

  IF(FULL_BUILD_TESTING)
    ADD_JRL_WALKGEN_TEST(TestKajita2003PbFlorentSeq2   TestKajita2003.cpp)
  ENDIF(FULL_BUILD_TESTING)
ENDIF(BUILD_TESTING)

##########################
## Read Novela Data 2011 #
##########################

#ADD_JRL_WALKGEN_EXE(ReadDataForNovela ReadDataForNovela.cpp)
#ADD_JRL_WALKGEN_TEST(ReadDataForNovela ReadDataForNovela.cpp)

################
# NMPC test #
################

# Disabled as the test fail : random results oscillating around mean behaviour
IF(BUILD_TESTING)
  ADD_JRL_WALKGEN_TEST(TestNaveau2015OnlineSimple TestNaveau2015.cpp)
  IF (FULL_BUILD_TESTING)
    ADD_JRL_WALKGEN_TEST(TestNaveau2015Online TestNaveau2015.cpp)
    SET_TESTS_PROPERTIES("TestNaveau2015Online${BITS}" PROPERTIES TIMEOUT 7200)
  ENDIF(FULL_BUILD_TESTING)
ENDIF(BUILD_TESTING)


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

INSTALL(FILES ${EXAMPLES_SOURCES} DESTINATION share/jrl/dynamics/examples)

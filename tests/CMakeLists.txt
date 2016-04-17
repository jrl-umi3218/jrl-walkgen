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

# Import jrl-mal flags.
ADD_DEFINITIONS(${JRL_MAL_CFLAGS})
LINK_DIRECTORIES(${JRL_DYNAMICS_LIBRARY_DIRS})

# Make sure private headers can be used.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

#################
## Generic test #
#################
#ADD_EXECUTABLE(TestFootPrintPGInterface
#  ../src/portability/gettimeofday.cc
#  TestFootPrintPGInterfaceData.cpp
#  CommonTools.cpp
#  TestFootPrintPGInterface.cpp
#  )

#TARGET_LINK_LIBRARIES(TestFootPrintPGInterface ${PROJECT_NAME})
##PKG_CONFIG_USE_DEPENDENCY(TestFootPrintPGInterface jrl-dynamics)
#ADD_DEPENDENCIES(TestFootPrintPGInterface ${PROJECT_NAME})

##########################
## Test Optimal Cholesky #
##########################
#ADD_EXECUTABLE(TestOptCholesky
#  TestOptCholesky.cpp
#  ../src/Mathematics/OptCholesky.cpp
#)

##########################
## Test Bspline #
##########################
#ADD_EXECUTABLE(TestBsplines
#  TestBsplines.cpp
#  ../src/Mathematics/Bsplines.cpp
#  )

##########################
## Test Ricatti Equation #
##########################
CONFIG_FILES_CMAKE(TestRiccatiEquationWeightsWithoutInitialPose.datref)
CONFIG_FILES_CMAKE(TestRiccatiEquationWeightsWithInitialPose.datref)
ADD_EXECUTABLE(TestRiccatiEquation
  TestRiccatiEquation.cpp
  ../src/PreviewControl/OptimalControllerSolver.cpp
  )
# Need this dependency for the use of the fortran functions (it seems)
PKG_CONFIG_USE_DEPENDENCY(TestRiccatiEquation jrl-mal)
# Add test on the ricatti equation
ADD_TEST(TestRiccatiEquation TestRiccatiEquation)

#######################
## Test Morisawa 2007 #
#######################
#IF(HRP2_DYNAMICS_FOUND)

#  IF(HRP2_14_FOUND)

#    MATH(EXPR BITS "8*${CMAKE_SIZEOF_VOID_P}")
##    CONFIG_FILES_CMAKE(TestMorisawa2007OnLine${BITS}TestFGPI.datref)
#    CONFIG_FILES_CMAKE(TestMorisawa2007ShortWalk${BITS}TestFGPI.datref)
##    CONFIG_FILES_CMAKE(TestMorisawa2007Climbing${BITS}TestFGPI.datref)
##    CONFIG_FILES_CMAKE(TestMorisawa2007GoingDown${BITS}TestFGPI.datref)
#    CONFIG_FILES_CMAKE(TestMorisawa2007SteppingStones${BITS}TestFGPI.datref)
##    CONFIG_FILES_CMAKE(TestMorisawa2007WalkingOnBeam${BITS}TestFGPI.datref)
##    CONFIG_FILES_CMAKE(TestMorisawa2007GoThroughWall${BITS}TestFGPI.datref)

#    CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/private_cmake/config_private.hh.cmake ${CMAKE_BINARY_DIR}/include/${HEADER_DIR}/config_private.hh)


#    MACRO(ADD_MORISAWA_2007 test_morisawa_arg)
#      SET(testmorisawa2007 "${test_morisawa_arg}${BITS}")

#      ADD_EXECUTABLE(${testmorisawa2007}
#      ../src/portability/gettimeofday.cc
#      TestMorisawa2007.cpp
#      CommonTools.cpp
#      TestObject.cpp
#      ClockCPUTime.cpp
#      )
#      TARGET_LINK_LIBRARIES(${testmorisawa2007} ${PROJECT_NAME})
#      #PKG_CONFIG_USE_DEPENDENCY(${testmorisawa2007} jrl-dynamics)
#      PKG_CONFIG_USE_DEPENDENCY(${testmorisawa2007} hrp2-dynamics)

#      ADD_DEPENDENCIES(${testmorisawa2007} ${PROJECT_NAME})
#      MESSAGE(STATUS "hrp2-14 data dir: " ${HRP2_14_PKGDATAROOTDIR})
#      SET(hrp2dynamicsmodelpath ${HRP2_14_PKGDATAROOTDIR}/hrp2-14/)
#      SET(hrp2dynamicsspec
#        ${HRP2_14_PKGDATAROOTDIR}/hrp2-14/HRP2SpecificitiesSmall.xml
#      )
#      SET(hrp2dynamicsljr
#        ${HRP2_14_PKGDATAROOTDIR}/hrp2-14/HRP2LinkJointRankSmall.xml
#      )
#      SET(hrp2dynamicsinitconfig
#        ${HRP2_14_PKGDATAROOTDIR}/hrp2-14/HRP2JRLInitConfigSmall.dat)

#      LIST(APPEND LOGGING_WATCHED_VARIABLES samplespec sampleljr)

#      # FIXME : use pinochio inside the WPG to handle different robot models.
#      # remove HRP2-14 dependencies
#      ADD_TEST(${testmorisawa2007} ${testmorisawa2007}
#        ${hrp2dynamicsmodelpath} HRP2JRLmainsmall.wrl ${hrp2dynamicsspec}
#        ${hrp2dynamicsljr} ${hrp2dynamicsinitconfig})

#    ENDMACRO(ADD_MORISAWA_2007)

#    #disabled as it fail : fix the code
#    #ADD_MORISAWA_2007(TestMorisawa2007OnLine)

#    ADD_MORISAWA_2007(TestMorisawa2007ShortWalk)
##    ADD_MORISAWA_2007(TestMorisawa2007Climbing)
##    ADD_MORISAWA_2007(TestMorisawa2007GoingDown)
#    ADD_MORISAWA_2007(TestMorisawa2007SteppingStones)
##    ADD_MORISAWA_2007(TestMorisawa2007WalkingOnBeam)
##    ADD_MORISAWA_2007(TestMorisawa2007GoThroughWall)

#  ENDIF(HRP2_14_FOUND)
#ENDIF(HRP2_DYNAMICS_FOUND)

####################
## Test Herdt 2010 #
####################
#IF(HRP2_DYNAMICS_FOUND)

#  IF(HRP2_14_FOUND)

#CONFIG_FILES_CMAKE(TestHerdt2010OnLineTestFGPI.datref)
#CONFIG_FILES_CMAKE(TestHerdt2010EmergencyStopTestFGPI.datref)

#MACRO(ADD_HERDT_2010 test_herdt2010_arg)
#  ADD_EXECUTABLE(${test_herdt2010_arg}
#  ../src/portability/gettimeofday.cc
#  ${test_herdt2010_arg}.cpp
#  CommonTools.cpp
#  TestObject.cpp
#  ClockCPUTime.cpp
#  )

#  TARGET_LINK_LIBRARIES(${test_herdt2010_arg} ${PROJECT_NAME})
#  ADD_DEPENDENCIES(${test_herdt2010_arg} ${PROJECT_NAME})
#  #PKG_CONFIG_USE_DEPENDENCY(${test_herdt2010_arg} jrl-dynamics)
#  IF(HRP2_DYNAMICS_FOUND)
#    PKG_CONFIG_USE_DEPENDENCY(${test_herdt2010_arg} hrp2-dynamics)
#  ENDIF(HRP2_DYNAMICS_FOUND)

#  SET(hrp2dynamicsmodelpath ${HRP2_14_PKGDATAROOTDIR}/hrp2-14/)
#  SET(hrp2dynamicsspec
#    ${HRP2_14_PKGDATAROOTDIR}/hrp2-14/HRP2SpecificitiesSmall.xml
#  )
#  SET(hrp2dynamicsljr
#    ${HRP2_14_PKGDATAROOTDIR}/hrp2-14/HRP2LinkJointRankSmall.xml
#  )
#  SET(hrp2dynamicsinitconfig
#    ${HRP2_14_PKGDATAROOTDIR}/hrp2-14/HRP2JRLInitConfigSmall.dat)

#  LIST(APPEND LOGGING_WATCHED_VARIABLES samplespec sampleljr)

#  ADD_TEST(${test_herdt2010_arg}
#    ${test_herdt2010_arg}
#    ${hrp2dynamicsmodelpath} HRP2JRLmainsmall.wrl ${hrp2dynamicsspec}
#    ${hrp2dynamicsljr} ${hrp2dynamicsinitconfig})
#ENDMACRO(ADD_HERDT_2010)

#ADD_HERDT_2010(TestHerdt2010)

#  ENDIF(HRP2_14_FOUND)
#ENDIF(HRP2_DYNAMICS_FOUND)
############################
## Test Inverse Kinematics #
############################
##SET(INVERSE_KINEMATICS)
#IF(INVERSE_KINEMATICS)

#CONFIG_FILES_CMAKE(TestHerdt2010OnLineTestFGPI.datref)

#ADD_EXECUTABLE(TestInverseKinematics
#    ../src/portability/gettimeofday.cc
#    TestInverseKinematics.cpp
#    CommonTools.cpp
#    TestObject.cpp
#    ClockCPUTime.cpp
#)

#TARGET_LINK_LIBRARIES(TestInverseKinematics ${PROJECT_NAME})
##PKG_CONFIG_USE_DEPENDENCY(TestInverseKinematics jrl-dynamics)
#PKG_CONFIG_USE_DEPENDENCY(TestInverseKinematics hrp2-dynamics)
#ADD_DEPENDENCIES(TestInverseKinematics ${PROJECT_NAME})

#SET(samplemodelpath ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/)
#SET(samplespec
#    ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleSpecificities.xml
#)
#SET(sampleljr
#    ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleLinkJointRank.xml
#)
#SET(sampleinitconfig
#    ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleInitConfig.dat
#)

#LIST(APPEND LOGGING_WATCHED_VARIABLES samplespec sampleljr)

## This test is disabled for now as it fails.
## FIXME: fix the test and/or the implementation
#ADD_TEST(TestInverseKinematics
#TestInverseKinematics
#${samplemodelpath} sample.wrl ${samplespec} ${sampleljr} ${sampleinitconfig})

#ENDIF(INVERSE_KINEMATICS)

###############################
## Test Kajita Dynamic Filter #
###############################

##SET(DYNAMIC_FILTER)
#IF(DYNAMIC_FILTER)

#ADD_EXECUTABLE(TestKajitaDynamicFilter
#    ../src/portability/gettimeofday.cc
#    TestKajitaDynamicFilter.cpp
#    CommonTools.cpp
#    TestObject.cpp
#    ClockCPUTime.cpp
#)

#TARGET_LINK_LIBRARIES(TestKajitaDynamicFilter ${PROJECT_NAME})
##PKG_CONFIG_USE_DEPENDENCY(TestKajitaDynamicFilter jrl-dynamics)
#IF(HRP2_DYNAMICS_FOUND)
#  PKG_CONFIG_USE_DEPENDENCY(TestKajitaDynamicFilter hrp2-dynamics)
#ENDIF(HRP2_DYNAMICS_FOUND)

#ADD_DEPENDENCIES(TestKajitaDynamicFilter ${PROJECT_NAME})

#SET(samplemodelpath ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/)
#SET(samplespec
#    ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleSpecificities.xml
#)
#SET(sampleljr
#    ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleLinkJointRank.xml
#)
#SET(sampleinitconfig
#    ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleInitConfig.dat
#)

#LIST(APPEND LOGGING_WATCHED_VARIABLES samplespec sampleljr)

## This test is disabled for now as it fails.
## FIXME: fix the test and/or the implementation
#ADD_TEST(TestKajitaDynamicFilter
#TestKajitaDynamicFilter
#${hrp2dynamicsmodelpath} sample.wrl ${hrp2dynamicsspec} ${hrp2dynamicsljr} ${hrp2dynamicsinitconfig})

#ENDIF(DYNAMIC_FILTER)

#####################
## Test Kajita 2003 #
#####################
#IF(HRP2_DYNAMICS_FOUND)
#  IF(HRP2_14_FOUND)

#  CONFIG_FILES_CMAKE(TestKajita2003StraightWalkingTestFGPI.datref)
#  CONFIG_FILES_CMAKE(TestKajita2003CircleTestFGPI.datref)
#  CONFIG_FILES_CMAKE(TestKajita2003PbFlorentSeq1TestFGPI.datref)
#  CONFIG_FILES_CMAKE(TestKajita2003PbFlorentSeq2TestFGPI.datref)

#  ADD_EXECUTABLE(TestKajita2003
#    ../src/portability/gettimeofday.cc
#    TestKajita2003.cpp
#    CommonTools.cpp
#    TestObject.cpp
#    ClockCPUTime.cpp
#  )

#  SET(samplemodelpath ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/)
#  SET(samplespec
#      ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleSpecificities.xml
#  )
#  SET(sampleljr
#      ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleLinkJointRank.xml
#  )
#  SET(sampleinitconfig
#      ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleInitConfig.dat
#  )

#  TARGET_LINK_LIBRARIES(TestKajita2003 ${PROJECT_NAME})

#  PKG_CONFIG_USE_DEPENDENCY(TestKajita2003 hrp2-dynamics)

#  ADD_DEPENDENCIES(TestKajita2003 ${PROJECT_NAME})

## This test is disabled for now as it fails.
## FIXME: fix the test and/or the implementation
#  #ADD_TEST(TestKajita2003 TestKajita2003
#    #${samplemodelpath} sample.wrl ${samplespec} ${sampleljr} ${sampleinitconfig})
#  ENDIF(HRP2_14_FOUND)
#ENDIF(HRP2_DYNAMICS_FOUND)

##########################
## Read Novela Data 2011 #
##########################

#IF(HRP2_DYNAMICS_FOUND)
#  IF(HRP2_14_FOUND)

#  ADD_EXECUTABLE(ReadDataForNovela
#    ../src/portability/gettimeofday.cc
#    ReadDataForNovela.cpp
#    CommonTools.cpp
#    TestObject.cpp
#    ClockCPUTime.cpp
#  )

#  SET(hrp2dynamicsmodelpath ${HRP2_14_PKGDATAROOTDIR}/)
#  SET(hrp2dynamicsspec
#    ${HRP2_14_PKGDATAROOTDIR}/HRP2SpecificitiesSmall.xml
#  )
#  SET(hrp2dynamicsljr
#    ${HRP2_14_PKGDATAROOTDIR}/HRP2LinkJointRankSmall.xml
#  )
#  SET(hrp2dynamicsinitconfig
#    ${HRP2_14_PKGDATAROOTDIR}/HRP2JRLInitConfigSmall.dat)

#  TARGET_LINK_LIBRARIES(ReadDataForNovela ${PROJECT_NAME})
#  PKG_CONFIG_USE_DEPENDENCY(ReadDataForNovela hrp2-dynamics)


#  ADD_DEPENDENCIES(ReadDataForNovela ${PROJECT_NAME})

##  ADD_TEST(ReadDataForNovela
##    ${hrp2dynamicsmodelpath} HRP2JRLmainsmall.wrl ${hrp2dynamicsspec} ${hrp2dynamicsljr} ${hrp2dynamicsinitconfig})
#  ENDIF(HRP2_14_FOUND)
#ENDIF(HRP2_DYNAMICS_FOUND)

################
# NMPC test #
################
#IF(HRP2_DYNAMICS_FOUND)
#  IF(HRP2_14_FOUND)

  CONFIG_FILES_CMAKE(TestNaveau2015TestFGPI.datref)
  ADD_EXECUTABLE(TestNaveau2015
     ../src/portability/gettimeofday.cc
    TestNaveau2015.cpp
    CommonTools.cpp
    TestObject.cpp
    ClockCPUTime.cpp
  )

  SET(urdfpath
    ${HRP2_14_PKGDATAROOTDIR}/hrp2_14_description/srdf/hrp2_14.urdf
  )
  SET(srdfpath
    ${HRP2_14_PKGDATAROOTDIR}/hrp2_14_description/srdf/hrp2_14.srdf
  )

  TARGET_LINK_LIBRARIES(TestNaveau2015 ${PROJECT_NAME})
  ADD_DEPENDENCIES(TestNaveau2015 ${PROJECT_NAME})

  ADD_TEST(TestNaveau2015 TestNaveau2015 ${urdfpath} ${srdfpath})

#  ENDIF(HRP2_14_FOUND)
#ENDIF(HRP2_DYNAMICS_FOUND)

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

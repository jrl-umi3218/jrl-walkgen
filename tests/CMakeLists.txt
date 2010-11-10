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

################
# Generic test #
################
ADD_EXECUTABLE(TestFootPrintPGInterface
  ../src/portability/gettimeofday.cc
  TestFootPrintPGInterfaceData.cpp
  CommonTools.cpp
  TestFootPrintPGInterface.cpp
  )

TARGET_LINK_LIBRARIES(TestFootPrintPGInterface ${PROJECT_NAME})
PKG_CONFIG_USE_DEPENDENCY(TestFootPrintPGInterface jrl-dynamics)
ADD_DEPENDENCIES(TestFootPrintPGInterface ${PROJECT_NAME})

#########################
# Test Optimal Cholesky #
#########################
ADD_EXECUTABLE(TestOptCholesky
  TestOptCholesky.cpp
  ../src/Mathematics/OptCholesky.cpp
)

#########################
# Test Ricatti Equation #
#########################
ADD_EXECUTABLE(TestRiccatiEquation
  TestRiccatiEquation.cpp
  ../src/PreviewControl/OptimalControllerSolver.cpp
  )

PKG_CONFIG_USE_DEPENDENCY(TestRiccatiEquation jrl-mal)

######################
# Test Morisawa 2007 #
######################
CONFIG_FILES_CMAKE(TestMorisawa2007OnLineTestFGPI.datref)
CONFIG_FILES_CMAKE(TestMorisawa2007ShortWalkTestFGPI.datref)

ADD_EXECUTABLE(TestMorisawa2007
  ../src/portability/gettimeofday.cc
  TestMorisawa2007.cpp
  CommonTools.cpp
  TestObject.cpp
  ClockCPUTime.cpp
  )
TARGET_LINK_LIBRARIES(TestMorisawa2007 ${PROJECT_NAME})
PKG_CONFIG_USE_DEPENDENCY(TestMorisawa2007 jrl-dynamics)
ADD_DEPENDENCIES(TestMorisawa2007 ${PROJECT_NAME})

MESSAGE(STATUS "jrl data dir: " ${JRL_DYNAMICS_DATAROOTDIR})
SET(samplemodelpath ${JRL_DYNAMICS_DATAROOTDIR}/examples/data/)
SET(samplespec
  ${JRL_DYNAMICS_DATAROOTDIR}/examples/data/sampleSpecificities.xml
  )
SET(sampleljr
  ${JRL_DYNAMICS_DATAROOTDIR}/examples/data/sampleLinkJointRank.xml
  )
SET(sampleinitconfig
  ${JRL_DYNAMICS_DATAROOTDIR}/examples/data/sampleInitConfig.dat)

LIST(APPEND LOGGING_WATCHED_VARIABLES samplespec sampleljr)

ADD_TEST(TestMorisawa2007
  TestMorisawa2007
  ${samplemodelpath} sample.wrl ${samplespec} ${sampleljr} ${sampleinitconfig})

###################
# Test Herdt 2010 #
###################
CONFIG_FILES_CMAKE(TestHerdt2010OnLineTestFGPI.datref)

ADD_EXECUTABLE(TestHerdt2010
  ../src/portability/gettimeofday.cc
  TestHerdt2010.cpp
  CommonTools.cpp
  TestObject.cpp
  ClockCPUTime.cpp
  )

TARGET_LINK_LIBRARIES(TestHerdt2010 ${PROJECT_NAME})
PKG_CONFIG_USE_DEPENDENCY(TestHerdt2010 jrl-dynamics)
ADD_DEPENDENCIES(TestHerdt2010 ${PROJECT_NAME})

ADD_TEST(TestHerdt2010 TestHerdt2010
  ${samplemodelpath} sample.wrl ${samplespec} ${sampleljr} ${sampleinitconfig})

#####################
# Add user examples #
#####################
SET(EXAMPLES_SOURCES
  TestFootPrintPGInterfaceData.cpp
  TestFootPrintPGInterfaceData.h
  TestFootPrintPGInterface.cpp
  TestHerdt2010.cpp
  CommonTools.cpp
  CommonTools.h
  examples.CMakeList
  )

INSTALL(FILES ${EXAMPLES_SOURCES}
  DESTINATION share/jrl/dynamics/examples/
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )

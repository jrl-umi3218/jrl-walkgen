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
INCLUDE_DIRECTORIES(${JRL_MAL_INCLUDEDIR})

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
ADD_DEPENDENCIES(TestMorisawa2007 ${PROJECT_NAME})

SET(samplemodelpath ${DYNAMICSJRLJAPAN_DATADIR}/examples/jrl/dynamics/data/)
SET(samplespec
  ${DYNAMICSJRLJAPAN_DATADIR}/examples/jrl/dynamics/data/sampleSpecificities.xml
  )
SET(sampleljr
  ${DYNAMICSJRLJAPAN_DATADIR}/examples/jrl/dynamics/data/sampleLinkJointRank.xml
  )
SET(sampleinitconfig
  ${DYNAMICSJRLJAPAN_DATADIR}/examples/jrl/dynamics/data/sampleInitConfig.dat)

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
  DESTINATION share/examples/jrl/walkgen/
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )

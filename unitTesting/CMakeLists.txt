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

include(CTest)

#############
# Set Flags #
#############
SET (${PROJECT_NAME}_test_CXXFLAGS "-I${${PROJECT_NAME}_SOURCE_DIR}/include -I${${PROJECT_NAME}_SOURCE_DIR}/src ${${PROJECT_NAME}_src_CXXFLAGS} ")

IF(UNIX)
  SET(${PROJECT_NAME}_test_CXXFLAGS "${${PROJECT_NAME}_test_CXXFLAGS} -DUNIX")
  SET(${PROJECT_NAME}_test_LDFLAGS "${LIBDIR_KW}${${PROJECT_NAME}_BINARY_DIR}/src ${LIBDIR_RP}${${PROJECT_NAME}_BINARY_DIR}/src ${LIBINCLUSION_KW}${PROJECT_NAME} ${${PROJECT_NAME}_src_LDFLAGS}")
ENDIF(UNIX)

IF(WIN32)
   SET(${PROJECT_NAME}_test_LDFLAGS " ")
ENDIF(WIN32)

##########################
# Deal with hrp2Dynamics #
##########################
SET(_hrp2Dynamics_FOUND 0)
SET(_hrp2Dynamics_OPTIONAL "hrp2Dynamics >= 1.1.0")
PKG_CHECK_MODULES(_hrp2Dynamics ${_hrp2Dynamics_OPTIONAL})

IF(NOT _hrp2Dynamics_FOUND)
  MESSAGE(WARNING "Check that package hrp2Dynamics is installed in a directory pointed out by PKG_CONFIG_PATH.")
ELSE(NOT _hrp2Dynamics_FOUND)
  MESSAGE(WARNING "HRP2 Dynamics FOUND !:" ${_hrp2Dynamics_FOUND})
  FOREACH(_ITEM ${_hrp2Dynamics_CFLAGS})
    SET(${PROJECT_NAME}_test_CXXFLAGS  "${${PROJECT_NAME}_test_CXXFLAGS} ${_ITEM}")
  ENDFOREACH(_ITEM)

  SET(${PROJECT_NAME}_test_CXXFLAGS "${${PROJECT_NAME}_test_CXXFLAGS} -DWITH_HRP2DYNAMICS")

  FOREACH(_ITEM ${_hrp2Dynamics_LDFLAGS})
    SET(${PROJECT_NAME}_test_LDFLAGS  "${${PROJECT_NAME}_test_LDFLAGS} ${_ITEM}")
  ENDFOREACH(_ITEM)

ENDIF(NOT _hrp2Dynamics_FOUND)

MESSAGE(STATUS "UL: ${${PROJECT_NAME}_test_LDFLAGS}  ${${PROJECT_NAME}_test_CXXFLAGS}")

##############################
# Deal with dynamicsJRLJapan #
##############################
SET(_dynamicsJRLJapan_FOUND 0)
SET(_dynamicsJRLJapan_OPTIONAL "dynamicsJRLJapan >= 1.19.1")
PKG_CHECK_MODULES(_dynamicsJRLJapan ${_dynamicsJRLJapan_OPTIONAL})

IF(NOT _dynamicsJRLJapan_FOUND)
  MESSAGE(WARNING "Check that package dynamicsJRLJapan is installed in a directory pointed out by PKG_CONFIG_PATH.")
ELSE(NOT _dynamicsJRLJapan_FOUND)
  EXEC_PROGRAM(${PKG_CONFIG_EXECUTABLE} ARGS  --variable=datarootdir dynamicsJRLJapan
    OUTPUT_VARIABLE _dynamicsJRLJapan_DATADIR)
  
  IF(NOT _hrp2Dynamics_FOUND)
    FOREACH(_ITEM ${_dynamicsJRLJapan_CFLAGS})
      SET(${PROJECT_NAME}_test_CXXFLAGS  "${${PROJECT_NAME}_test_CXXFLAGS} ${_ITEM}")
    ENDFOREACH(_ITEM)

    FOREACH(_ITEM ${_dynamicsJRLJapan_LDFLAGS})
      SET(${PROJECT_NAME}_test_LDFLAGS  "${${PROJECT_NAME}_test_LDFLAGS} ${_ITEM}")
    ENDFOREACH(_ITEM)

  ENDIF(NOT _hrp2Dynamics_FOUND)
  
ENDIF(NOT _dynamicsJRLJapan_FOUND)

MESSAGE(STATUS "dynamicsJRLJapan_DATADIR:" ${_dynamicsJRLJapan_DATADIR})
################
# Generic test #
################

ADD_EXECUTABLE(	TestFootPrintPGInterface 
  ../src/TimeUtilsWindows.cpp
  TestFootPrintPGInterfaceData.cpp
  CommonTools.cpp
  TestFootPrintPGInterface.cpp)
SET_TARGET_PROPERTIES(TestFootPrintPGInterface
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)
target_link_libraries (TestFootPrintPGInterface ${PROJECT_NAME})
ADD_DEPENDENCIES (TestFootPrintPGInterface ${PROJECT_NAME})

#########################
# Test Optimal Cholesky #
#########################
ADD_EXECUTABLE(	TestOptCholesky 
	TestOptCholesky.cpp 
	../src/Mathematics/OptCholesky.cpp
)
SET_TARGET_PROPERTIES(TestOptCholesky
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)

#########################
# Test Ricatti Equation #
#########################

ADD_EXECUTABLE(	TestRiccatiEquation 
	TestRiccatiEquation.cpp 
	../src/PreviewControl/OptimalControllerSolver.cpp 
)
SET_TARGET_PROPERTIES(TestRiccatiEquation
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)
######################
# Test Morisawa 2007 #
######################
CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/unitTesting/TestMorisawa2007OnLineTestFGPI.datref
		 ${${PROJECT_NAME}_BINARY_DIR}/unitTesting/TestMorisawa2007OnLineTestFGPI.datref)

CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/unitTesting/TestMorisawa2007ShortWalkTestFGPI.datref
		 ${${PROJECT_NAME}_BINARY_DIR}/unitTesting/TestMorisawa2007ShortWalkTestFGPI.datref)


ADD_EXECUTABLE( TestMorisawa2007
  ../src/TimeUtilsWindows.cpp
  TestMorisawa2007.cpp
  CommonTools.cpp
  TestObject.cpp
  ClockCPUTime.cpp
)
SET_TARGET_PROPERTIES( TestMorisawa2007
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}  
)
target_link_libraries (TestMorisawa2007 ${PROJECT_NAME})
ADD_DEPENDENCIES (TestMorisawa2007 ${PROJECT_NAME})

set (samplemodelpath ${_dynamicsJRLJapan_DATADIR}/examples/dynamicsJRLJapan/data/ )
set (samplespec ${_dynamicsJRLJapan_DATADIR}/examples/dynamicsJRLJapan/data/sampleSpecificities.xml)
set (sampleljr ${_dynamicsJRLJapan_DATADIR}/examples/dynamicsJRLJapan/data/sampleLinkJointRank.xml)
set (sampleinitconfig ${_dynamicsJRLJapan_DATADIR}/examples/dynamicsJRLJapan/data/sampleInitConfig.dat)

MESSAGE(STATUS "Sample specificities:" ${samplespec})
MESSAGE(STATUS "Sample link joint rank:" ${sampleljr})

ADD_TEST(TestMorisawa2007 TestMorisawa2007 ${samplemodelpath} sample.wrl ${samplespec} ${sampleljr} ${sampleinitconfig})

###################
# Test Herdt 2010 #
###################

ADD_EXECUTABLE( TestHerdt2010
  ../src/TimeUtilsWindows.cpp
  TestHerdt2010.cpp
  CommonTools.cpp
  TestObject.cpp
  ClockCPUTime.cpp
)
SET_TARGET_PROPERTIES( TestHerdt2010
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}  
)
target_link_libraries (TestHerdt2010 ${PROJECT_NAME})
ADD_DEPENDENCIES (TestHerdt2010 ${PROJECT_NAME})

ADD_TEST(TestHerdt2010 TestHerdt2010 ${samplemodelpath} sample.wrl ${samplespec} ${sampleljr} ${sampleinitconfig})

#####################
# Add user examples #
#####################
SET(${PROJECT_NAME}_EXAMPLES 
  TestFootPrintPGInterfaceData.cpp
  TestFootPrintPGInterfaceData.h
  TestFootPrintPGInterface.cpp
  TestHerdt2010.cpp
  CommonTools.cpp
  CommonTools.h
  examples.CMakeList
  )

INSTALL(FILES ${${PROJECT_NAME}_EXAMPLES}
  DESTINATION share/examples/${PROJECT_NAME}/
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )


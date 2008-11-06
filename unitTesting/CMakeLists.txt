# Olivier Stasse, JRL, CNRS/AIST
# Creation: 4/11/2008
# History:
#
# Copyright CNRS/AIST
SET (UNITTESTING_CXXFLAGS "${${PROJECT_NAME}_CXXFLAGS} -L${${PROJECT_NAME}_BINARY_DIR}/include")
SET (UNITTESTING_LDFLAGS "${${PROJECT_NAME}_LDFLAGS} -L${${PROJECT_NAME}_BINARY_DIR}/src")

MESSAGE(STATUS "UL: ${UNITTESTING_LDFLAGS}  ${UNITTESTING_CXXFLAGS}")

ADD_EXECUTABLE(	TestFootPrintPGInterface TestFootPrintPGInterface.cpp)
SET_TARGET_PROPERTIES(TestFootPrintPGInterface
	              PROPERTIES
	              COMPILE_FLAGS ${UNITTESTING_CXXFLAGS}
		      LINK_FLAGS ${UNITTESTING_LDFLAGS}
)


ADD_EXECUTABLE(	TestRiccatiEquation TestRiccatiEquation.cpp)
SET_TARGET_PROPERTIES( TestRiccatiEquation
	              PROPERTIES
	              COMPILE_FLAGS ${UNITTESTING_CXXFLAGS}
		      LINK_FLAGS ${UNITTESTING_LDFLAGS}
)

ADD_EXECUTABLE(	TestMinimal TestMinimal.cpp)
SET_TARGET_PROPERTIES( TestMinimal
	              PROPERTIES
	              COMPILE_FLAGS ${UNITTESTING_CXXFLAGS}
		      LINK_FLAGS ${UNITTESTING_LDFLAGS}
)

ADD_EXECUTABLE(	TestAnalyticalMorisawaCompact TestAnalyticalMorisawaCompact.cpp)
SET_TARGET_PROPERTIES(TestAnalyticalMorisawaCompact
	              PROPERTIES
	              COMPILE_FLAGS ${UNITTESTING_CXXFLAGS}
		      LINK_FLAGS ${UNITTESTING_LDFLAGS}
)

ADD_EXECUTABLE(	TestAnalyticalMorisawaCompact2 TestAnalyticalMorisawaCompact2.cpp)
SET_TARGET_PROPERTIES(TestAnalyticalMorisawaCompact2
	              PROPERTIES
	              COMPILE_FLAGS ${UNITTESTING_CXXFLAGS}
		      LINK_FLAGS ${UNITTESTING_LDFLAGS}
)

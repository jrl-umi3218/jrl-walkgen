# Olivier Stasse,  Francois Keith, JRL, CNRS/AIST
# Creation: 04/11/2008
# History:
#
# Copyright CNRS/AIST



# Create the library "walkGenJRL" 

# Create list of source files
# ---------------------------
SET(libwpg_src
	./FootTrajectoryGeneration/FootTrajectoryGenerationAbstract.cpp 
	./FootTrajectoryGeneration/FootTrajectoryGenerationStandard.cpp 
	./FootTrajectoryGeneration/FootTrajectoryGenerationMultiple.cpp 
	./FootTrajectoryGeneration/LeftAndRightFootTrajectoryGenerationMultiple.cpp 
	./GlobalStrategyManagers/CoMAndFootOnlyStrategy.cpp 
	./GlobalStrategyManagers/GlobalStrategyManager.cpp 
	./GlobalStrategyManagers/DoubleStagePreviewControlStrategy.cpp 
	./Mathematics/AnalyticalZMPCOGTrajectory.cpp 
	./Mathematics/ConvexHull.cpp 
	./Mathematics/FootConstraintsAsLinearSystem.cpp
	./Mathematics/OptCholesky.cpp
	./Mathematics/Polynome.cpp 
	./Mathematics/PolynomeFoot.cpp 
	./Mathematics/PLDPSolver.cpp 
	./Mathematics/qld.cpp 
	./Mathematics/StepOverPolynome.cpp
	./PreviewControl/PreviewControl.cpp 
	./PreviewControl/OptimalControllerSolver.cpp 
	./PreviewControl/ZMPPreviewControlWithMultiBodyZMP.cpp 
	./PreviewControl/LinearizedInvertedPendulum2D.cpp
	./ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.cpp 
	./ZMPRefTrajectoryGeneration/ZMPDiscretization.cpp 
	./ZMPRefTrajectoryGeneration/ZMPQPWithConstraint.cpp 
	./ZMPRefTrajectoryGeneration/ZMPConstrainedQPFastFormulation.cpp 
	./ZMPRefTrajectoryGeneration/AnalyticalMorisawaAbstract.cpp 
	./ZMPRefTrajectoryGeneration/AnalyticalMorisawaCompact.cpp 
	./ZMPRefTrajectoryGeneration/FilteringAnalyticalTrajectoryByPreviewControl.cpp 
	./MotionGeneration/StepOverPlanner.cpp
	./MotionGeneration/CollisionDetector.cpp
	./MotionGeneration/WaistHeightVariation.cpp
	./MotionGeneration/UpperBodyMotion.cpp 
	./MotionGeneration/GenerateMotionFromKineoWorks.cpp 
	./MotionGeneration/ComAndFootRealizationByGeometry.cpp 
	StepStackHandler.cpp 
	PatternGeneratorInterfacePrivate.cpp 
	SimplePlugin.cpp 
	SimplePluginManager.cpp 
	Clock.cpp
	TimeUtilsWindows.cpp
 )

 SET(libwpg_headers  )

# Create list of headers files
#-----------------------------

FOREACH(srcfile ${libwpg_src})
  SET(newsrcfile "")
  STRING(REGEX REPLACE "cpp$" "h" newsrcfile ${srcfile})
#  SET(libwpg_headers ../include/${PROJECT_NAME}/${newsrcfile} ${libwpg_headers})
#   MESSAGE(STATUS "newsrcfile: " ${newsrcfile})
ENDFOREACH(srcfile)

# Create name of the library.
#---------------------------
SET(PROJECT_NAME_LIB ${PROJECT_NAME})

# Add main target: walkGenJRL
# ---------------------------------
IF(UNIX)
ADD_LIBRARY(${PROJECT_NAME_LIB}
  ${libwpg_src} 
  ${libwpg_headers} 
)
ENDIF(UNIX)
IF(WIN32)
ADD_LIBRARY(${PROJECT_NAME_LIB}
  ${libwpg_src} 
  ${libwpg_headers} 
)
ENDIF(WIN32)


# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build walkGenJRL with shared libraries." ON)

# Specify soname, version and rpath
# ---------------------------------
SET_TARGET_PROPERTIES(${PROJECT_NAME_LIB}
      		      PROPERTIES 
		      VERSION ${PROJECT_VERSION}
		      SOVERSION ${PROJECT_VERSION}
		      INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib
		      INSTALL_RPATH_USE_LINK_PATH TRUE
		      )

# Specify compiling options
# -------------------------
SET(${PROJECT_NAME}_src_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/ ${${PROJECT_NAME}_src_CXXFLAGS}")
SET(${PROJECT_NAME}_src_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/src/ ${${PROJECT_NAME}_src_CXXFLAGS}")
SET(${PROJECT_NAME}_src_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME} ${${PROJECT_NAME}_src_CXXFLAGS}")
SET(${PROJECT_NAME}_src_CXXFLAGS "${${PROJECT_NAME}_src_CXXFLAGS} -I${CMAKE_INSTALL_PREFIX}/include")
SET(${PROJECT_NAME}_src_CXXFLAGS "${${PROJECT_NAME}_src_CXXFLAGS} ${${PROJECT_NAME}_CXXFLAGS}")
SET_TARGET_PROPERTIES(${PROJECT_NAME_LIB}
      		      PROPERTIES 
	              COMPILE_FLAGS  ${${PROJECT_NAME}_src_CXXFLAGS}
				  LINK_FLAGS ${${PROJECT_NAME}_src_LDFLAGS}
	             )
			 
				 
# Specify header files
SET(${PROJECT_NAME}_HEADERS ${libwpg_headers})

# Installation 
# -------------
MESSAGE(STATUS "Source directory: ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}")
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME} 
	DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
	FILES_MATCHING PATTERN "*.h" )

IF(UNIX)

INSTALL(FILES 
        ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}.so.${PROJECT_VERSION}
        ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}.so
	DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)	

ENDIF(UNIX)

IF(WIN32)

	SET (LIBRARY_EXTENSION "dll" "lib")
	SET( CONFIGURATION_EXTENSION "")

	FOREACH (lib_extension ${LIBRARY_EXTENSION} ) 
		ADD_CUSTOM_COMMAND( TARGET ${PROJECT_NAME_LIB}
			POST_BUILD
			COMMAND 
			${CMAKE_COMMAND} -E copy 
				${${PROJECT_NAME}_BINARY_DIR}/src/${PROJECT_NAME}.${lib_extension}
				${${PROJECT_NAME}_BINARY_DIR}/src/${PROJECT_NAME}-${PROJECT_VERSION}${CONFIGURATION_EXTENSION}.${lib_extension}
		)
			
		INSTALL(FILES
			${${PROJECT_NAME}_BINARY_DIR}/src/${PROJECT_NAME}-${PROJECT_VERSION}.${lib_extension}
			${${PROJECT_NAME}_BINARY_DIR}/src/${PROJECT_NAME}.${lib_extension}
		
			DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
				PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
		)	

	ENDFOREACH (lib_extension)



ENDIF(WIN32)


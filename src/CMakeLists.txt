# Olivier Stasse, JRL, CNRS/AIST
# Creation: 04/11/2008
# History:
#
# Copyright CNRS/AIST



# Create the library "walkGenJRL" 

# Create list of source files
# ---------------------------
SET(libdyn_src
	./FootTrajectoryGeneration/FootTrajectoryGenerationAbstract.cpp 
	./FootTrajectoryGeneration/FootTrajectoryGenerationStandard.cpp 
	./FootTrajectoryGeneration/FootTrajectoryGenerationMultiple.cpp 
	./FootTrajectoryGeneration/LeftAndRightFootTrajectoryGenerationMultiple.cpp 
	./GlobalStrategyManagers/CoMAndFootOnlyStrategy.cpp 
	./GlobalStrategyManagers/GlobalStrategyManager.cpp 
	./GlobalStrategyManagers/DoubleStagePreviewControlStrategy.cpp 
	./Mathematics/Polynome.cpp 
	./Mathematics/PolynomeFoot.cpp 
	./Mathematics/StepOverPolynome.cpp
	./Mathematics/ConvexHull.cpp 
	./Mathematics/qld.cpp 
	./Mathematics/AnalyticalZMPCOGTrajectory.cpp 
	./PreviewControl/PreviewControl.cpp 
	./PreviewControl/OptimalControllerSolver.cpp 
	./PreviewControl/ZMPPreviewControlWithMultiBodyZMP.cpp 
	./ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.cpp 
	./ZMPRefTrajectoryGeneration/ZMPDiscretization.cpp 
	./ZMPRefTrajectoryGeneration/ZMPQPWithConstraint.cpp 
	./ZMPRefTrajectoryGeneration/AnalyticalMorisawaAbstract.cpp 
	./ZMPRefTrajectoryGeneration/AnalyticalMorisawaCompact.cpp 
	./ZMPRefTrajectoryGeneration/AnalyticalMorisawaFull.cpp 
	./MotionGeneration/InverseKinematics.cpp 
	./MotionGeneration/StepOverPlanner.cpp
	./MotionGeneration/CollisionDetector.cpp
	./MotionGeneration/WaistHeightVariation.cpp
	./MotionGeneration/UpperBodyMotion.cpp 
	./MotionGeneration/GenerateMotionFromKineoWorks.cpp 
	./MotionGeneration/ComAndFootRealizationByGeometry.cpp 
	StepStackHandler.cpp 
	PatternGeneratorInterface.cpp 
	SimplePlugin.cpp 
	SimplePluginManager.cpp 
	Clock.cpp
 )


# Create list of headers files
#-----------------------------

FOREACH(srcfile ${libdyn_src})
  SET(newsrcfile "")
  STRING(REGEX REPLACE "cpp$" "h" newsrcfile ${srcfile})
  SET(libdyn_headers ../include/${PROJECT_NAME}/${newsrcfile} ${libdyn_headers})
#   MESSAGE(STATUS "newsrcfile: " ${newsrcfile})
ENDFOREACH(srcfile)

# Create name of the library.
#---------------------------
SET(PROJECT_NAME_LIB ${PROJECT_NAME})

# Add main target: walkGenJRL
# ---------------------------------
ADD_LIBRARY(${PROJECT_NAME_LIB}
  ${libdyn_src} ./Mathematics/jrlgesvd.f 
  ${libdyn_headers} 
)

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build walkGenJRL with shared libraries." ON)


# Specify compiling options
# -------------------------
SET_TARGET_PROPERTIES(${PROJECT_NAME_LIB}
      		      PROPERTIES 
	              COMPILE_FLAGS  "-I${PROJECT_SOURCE_DIR}/include/ -I${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME} ${${PROJECT_NAME}_CXXFLAGS}"
	             )

# Specify header files
SET(${PROJECT_NAME}_HEADERS ${libdyn_headers})

# Installation 
# -------------
INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
	DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)


ADD_CUSTOM_COMMAND( TARGET ${PROJECT_NAME_LIB}
            POST_BUILD
	    COMMAND ${CMAKE_COMMAND} -E copy ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}.so ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}-${PROJECT_VERSION}.so 
            COMMAND ${CMAKE_COMMAND} -E remove -f ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}.so 
            COMMAND ${CMAKE_COMMAND} -E create_symlink lib${PROJECT_NAME}-${PROJECT_VERSION}.so  lib${PROJECT_NAME}.so 
	    )

INSTALL(FILES 
        ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}-${PROJECT_VERSION}.so 
        ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}.so
	DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)	

IF(OPENHRP_FOUND)
   MESSAGE(STATUS " They will be installed")
   INSTALL(FILES 
        ${${PROJECT_NAME}_SOURCE_DIR}/src/data/HRP2Specificities.xml
        ${${PROJECT_NAME}_SOURCE_DIR}/src/data/HRP2LinkJointRank.xml
	DESTINATION ${OPENHRP_HRP2_CONFIG_DIRECTORY}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)	
   

ENDIF(OPENHRP_FOUND)
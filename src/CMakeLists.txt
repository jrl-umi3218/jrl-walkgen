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
	./Mathematics/FootConstraintsAsLinearSystemForVelRef.cpp
	./Mathematics/FootHalfSize.cpp
	./Mathematics/OptCholesky.cpp
	./Mathematics/Polynome.cpp 
	./Mathematics/PolynomeFoot.cpp 
	./Mathematics/PLDPSolver.cpp
	./Mathematics/PLDPHerdt.cpp 
	./Mathematics/qld.cpp 
	./Mathematics/StepOverPolynome.cpp
	./PreviewControl/PreviewControl.cpp 
	./PreviewControl/OptimalControllerSolver.cpp 
	./PreviewControl/ZMPPreviewControlWithMultiBodyZMP.cpp 
	./PreviewControl/LinearizedInvertedPendulum2D.cpp
	./PreviewControl/SupportFSM.cpp
	./ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.cpp 
	./ZMPRefTrajectoryGeneration/ZMPDiscretization.cpp 
	./ZMPRefTrajectoryGeneration/ZMPQPWithConstraint.cpp 
	./ZMPRefTrajectoryGeneration/ZMPConstrainedQPFastFormulation.cpp 
	./ZMPRefTrajectoryGeneration/ZMPVelocityReferencedQP.cpp 
	./ZMPRefTrajectoryGeneration/OrientationsPreview.cpp 
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
	PGTypes.cpp
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
ADD_LIBRARY(${PROJECT_NAME_LIB}
  ${libwpg_src} 
  ${libwpg_headers} 
)

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build walkGenJRL with shared libraries." ON)

# Specify soname, version and rpath
# ---------------------------------
SET_TARGET_PROPERTIES(${PROJECT_NAME_LIB}
      		      PROPERTIES 
		      VERSION ${PROJECT_VERSION}
		      SOVERSION ${PROJECT_VERSION}
		      INSTALL_RPATH lib
		      INSTALL_RPATH_USE_LINK_PATH TRUE
		      )

# Specify compiling options
# -------------------------
SET(${PROJECT_NAME}_src_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/ ${${PROJECT_NAME}_src_CXXFLAGS}")
SET(${PROJECT_NAME}_src_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/src/ ${${PROJECT_NAME}_src_CXXFLAGS}")
SET(${PROJECT_NAME}_src_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/jrl/walkgen/ ${${PROJECT_NAME}_src_CXXFLAGS}")
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
#INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
#	DESTINATION include/jrl/walkgen
#	PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)	

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/jrl/walkgen 
	DESTINATION include/jrl/walkgen
	FILES_MATCHING PATTERN "*.hh" )

IF(UNIX)

INSTALL(FILES 
        ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}.so.${PROJECT_VERSION}
        ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}.so
	DESTINATION lib
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)	

ENDIF(UNIX)

IF(WIN32)
	INSTALL(TARGETS ${PROJECT_NAME}
			DESTINATION lib
			PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
	)
ENDIF(WIN32)


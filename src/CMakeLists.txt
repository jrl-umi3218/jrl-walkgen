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

# Make sure to find Debug.h
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(BEFORE ${HOME}/devel/metapod/_bin/include/)
INCLUDE_DIRECTORIES(BEFORE ${HOME}/devel/metapod/_bin/include/metapod/)

# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
message (STATUS "-- Boost Dir : " ${Boost_INCLUDE_DIRS} )

# add flag to compile qld.cc
IF(WIN32)
ADD_DEFINITIONS("/D __STDC__")
ENDIF(WIN32)

IF(USE_LSSOL)
ADD_DEFINITIONS("/DLSSOL_FOUND")
ENDIF(USE_LSSOL)

SET(SOURCES
  FootTrajectoryGeneration/FootTrajectoryGenerationAbstract.cpp
  FootTrajectoryGeneration/FootTrajectoryGenerationStandard.cpp
  FootTrajectoryGeneration/FootTrajectoryGenerationMultiple.cpp
  FootTrajectoryGeneration/LeftAndRightFootTrajectoryGenerationMultiple.cpp
  FootTrajectoryGeneration/OnLineFootTrajectoryGeneration.cpp
  GlobalStrategyManagers/CoMAndFootOnlyStrategy.cpp
  GlobalStrategyManagers/GlobalStrategyManager.cpp
  GlobalStrategyManagers/DoubleStagePreviewControlStrategy.cpp
  Mathematics/AnalyticalZMPCOGTrajectory.cpp
  Mathematics/ConvexHull.cpp
  Mathematics/FootConstraintsAsLinearSystem.cpp
#  Mathematics/FootConstraintsAsLinearSystemForVelRef.cpp
  Mathematics/FootHalfSize.cpp
  Mathematics/OptCholesky.cpp
  Mathematics/Polynome.cpp
  Mathematics/PolynomeFoot.cpp
  Mathematics/PLDPSolver.cpp
  Mathematics/qld.cpp
  Mathematics/StepOverPolynome.cpp
  Mathematics/relative-feet-inequalities.cpp
  Mathematics/intermediate-qp-matrices.cpp
  PreviewControl/PreviewControl.cpp
  PreviewControl/OptimalControllerSolver.cpp
  PreviewControl/ZMPPreviewControlWithMultiBodyZMP.cpp
  PreviewControl/LinearizedInvertedPendulum2D.cpp
  PreviewControl/rigid-body.cpp
  PreviewControl/rigid-body-system.cpp
  PreviewControl/SupportFSM.cpp
  ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.cpp
  ZMPRefTrajectoryGeneration/ZMPDiscretization.cpp
  ZMPRefTrajectoryGeneration/ZMPQPWithConstraint.cpp
  ZMPRefTrajectoryGeneration/ZMPConstrainedQPFastFormulation.cpp
  ZMPRefTrajectoryGeneration/ZMPVelocityReferencedQP.cpp
  ZMPRefTrajectoryGeneration/OrientationsPreview.cpp
  ZMPRefTrajectoryGeneration/AnalyticalMorisawaAbstract.cpp
  ZMPRefTrajectoryGeneration/AnalyticalMorisawaCompact.cpp
  ZMPRefTrajectoryGeneration/FilteringAnalyticalTrajectoryByPreviewControl.cpp
  ZMPRefTrajectoryGeneration/problem-vel-ref.cpp
  ZMPRefTrajectoryGeneration/qp-problem.cpp
  ZMPRefTrajectoryGeneration/generator-vel-ref.cpp
  ZMPRefTrajectoryGeneration/mpc-trajectory-generation.cpp
  ZMPRefTrajectoryGeneration/DynamicFilter.cpp
  MotionGeneration/StepOverPlanner.cpp
  MotionGeneration/CollisionDetector.cpp
  MotionGeneration/WaistHeightVariation.cpp
  MotionGeneration/UpperBodyMotion.cpp
  MotionGeneration/GenerateMotionFromKineoWorks.cpp
  MotionGeneration/ComAndFootRealizationByGeometry.cpp
  StepStackHandler.cpp
  PatternGeneratorInterfacePrivate.cpp
  SimplePlugin.cpp
  SimplePluginManager.cpp
  Clock.cpp
  PGTypes.cpp
  portability/gettimeofday.cc
  privatepgtypes.cpp
  )

# prefix and suffix each element of list by ${prefix}elemnt${suffix}
macro(ADDPREFIX newlist prefix list_name)
    # create empty list - necessary?
    SET(${newlist})

     # prefix and suffix elements
    foreach(l ${${list_name}})
      list(APPEND ${newlist} ${prefix}${l} )
    endforeach()

endmacro(ADDPREFIX)

IF ( '${CMAKE_EXTRA_GENERATOR}' STREQUAL 'CodeBlocks' )
    ADDPREFIX(${PROJECT_NAME}_ABSOLUTE_HEADERS  "${CMAKE_SOURCE_DIR}/"  ${PROJECT_NAME}_HEADERS)
ENDIF ( '${CMAKE_EXTRA_GENERATOR}' STREQUAL 'CodeBlocks' )

ADD_LIBRARY(jrl-walkgen SHARED ${SOURCES} ${${PROJECT_NAME}_ABSOLUTE_HEADERS})
SET_TARGET_PROPERTIES(jrl-walkgen PROPERTIES SOVERSION ${PROJECT_VERSION})
# Define dependencies

# define and search for metapod library
SET(WITH_METAPODFROMURDF TRUE)
SET(model_name "hrp2_14")
# Define the simple humanoid model for the metapod algorithm
ADD_SAMPLEURDFMODEL(${model_name})

TARGET_LINK_LIBRARIES(jrl-walkgen metapod_${model_name} ${Boost_LIBRARIES})
message (STATUS "--Boost : " ${Boost_LIBRARIES} )

INSTALL(TARGETS jrl-walkgen DESTINATION ${CMAKE_INSTALL_LIBDIR})

PKG_CONFIG_USE_DEPENDENCY(jrl-walkgen abstract-robot-dynamics)
PKG_CONFIG_USE_DEPENDENCY(jrl-walkgen jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(jrl-walkgen jrl-dynamics)
IF(USE_LSSOL)
 PKG_CONFIG_USE_DEPENDENCY(jrl-walkgen lssol)
ENDIF(USE_LSSOL)

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

IF(USE_QUADPROG)
  SET(QUADPROG_COMPILE_FLAG 1)
ELSE()
  SET(QUADPROG_COMPILE_FLAG 0)
ENDIF()

CONFIGURE_FILE(configJRLWPG.hh.in ${CMAKE_BINARY_DIR}/include/jrl/walkgen/configJRLWPG.hh)
# Make sure to find Debug.h
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src/ZMPRefTrajectoryGeneration)
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src/FootTrajectoryGeneration)

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

SET(INCLUDES
  PreviewControl/rigid-body.hh
  PreviewControl/OptimalControllerSolver.hh
  PreviewControl/rigid-body-system.hh
  PreviewControl/ZMPPreviewControlWithMultiBodyZMP.hh
  PreviewControl/SupportFSM.hh
  PreviewControl/LinearizedInvertedPendulum2D.hh
  PreviewControl/PreviewControl.hh
  PreviewControl/SupportFSM_backup.hh
  FootTrajectoryGeneration/FootTrajectoryGenerationMultiple.hh
  FootTrajectoryGeneration/FootTrajectoryGenerationAbstract.hh
  FootTrajectoryGeneration/FootTrajectoryGenerationStandard.hh
  FootTrajectoryGeneration/LeftAndRightFootTrajectoryGenerationMultiple.hh
  Debug.hh
  SimplePluginManager.hh
  privatepgtypes.hh
#  MultiContactRefTrajectoryGeneration/MultiContactHirukawa.hh
  patterngeneratorinterfaceprivate.hh
  Mathematics/FootConstraintsAsLinearSystem.hh
  Mathematics/Polynome.hh
  Mathematics/ConvexHull.hh
  Mathematics/Bsplines.hh
  Mathematics/StepOverPolynome.hh
  Mathematics/AnalyticalZMPCOGTrajectory.hh
  Mathematics/qld.hh
  Mathematics/PLDPSolver.hh
  Mathematics/FootHalfSize.hh
  Mathematics/relative-feet-inequalities.hh
  Mathematics/intermediate-qp-matrices.hh
  Mathematics/PolynomeFoot.hh
  Mathematics/PLDPSolverHerdt.hh
  Mathematics/OptCholesky.hh
  StepStackHandler.hh
  configJRLWPG.hh
  Clock.hh
  GlobalStrategyManagers/CoMAndFootOnlyStrategy.hh
  GlobalStrategyManagers/GlobalStrategyManager.hh
  GlobalStrategyManagers/DoubleStagePreviewControlStrategy.hh
  ZMPRefTrajectoryGeneration/AnalyticalMorisawaAbstract.hh
  ZMPRefTrajectoryGeneration/DynamicFilter.hh
  ZMPRefTrajectoryGeneration/ZMPConstrainedQPFastFormulation.hh
  ZMPRefTrajectoryGeneration/qp-problem.hh
  ZMPRefTrajectoryGeneration/ZMPDiscretization.hh
  ZMPRefTrajectoryGeneration/OrientationsPreview.hh
  ZMPRefTrajectoryGeneration/mpc-trajectory-generation.hh
  ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.hh
  ZMPRefTrajectoryGeneration/ZMPVelocityReferencedQP.hh
  ZMPRefTrajectoryGeneration/AnalyticalMorisawaCompact.hh
  ZMPRefTrajectoryGeneration/generator-vel-ref.hh
  ZMPRefTrajectoryGeneration/nmpc_generator.hh
  ZMPRefTrajectoryGeneration/FilteringAnalyticalTrajectoryByPreviewControl.hh
  ZMPRefTrajectoryGeneration/problem-vel-ref.hh
  ZMPRefTrajectoryGeneration/ZMPQPWithConstraint.hh
  SimplePlugin.hh
  portability/gettimeofday.hh
  portability/bzero.hh
  MotionGeneration/ComAndFootRealizationByGeometry.hh
  MotionGeneration/StepOverPlanner.hh
  MotionGeneration/WaistHeightVariation.hh
  MotionGeneration/ComAndFootRealization.hh
  MotionGeneration/GenerateMotionFromKineoWorks.hh
  MotionGeneration/UpperBodyMotion.hh
  MotionGeneration/CollisionDetector.hh
  ../tests/CommonTools.hh
  ../tests/ClockCPUTime.hh
  ../tests/TestObject.hh
  ../doc/additionalHeader/modules.hh
  ../include/jrl/walkgen/pgtypes.hh
  ../include/jrl/walkgen/patterngeneratorinterface.hh
  ../include/jrl/walkgen/pinocchiorobot.hh
)

IF(USE_QUADPROG)
SET(INCLUDES
  ${INCLUDES}
  ZMPRefTrajectoryGeneration/ZMPVelocityReferencedSQP.hh
  ZMPRefTrajectoryGeneration/nmpc_generator.hh
)
ENDIF(USE_QUADPROG)

SET(SOURCES
  ${INCLUDES}
  RobotDynamics/pinocchiorobot.cpp
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
  Mathematics/Bsplines.cpp
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
#  MultiContactRefTrajectoryGeneration/MultiContactHirukawa.cc
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
  pgtypes.cpp
  Clock.cpp
  portability/gettimeofday.cc
  privatepgtypes.cpp
)

IF(USE_QUADPROG)
SET(SOURCES
  ${SOURCES}
  ZMPRefTrajectoryGeneration/ZMPVelocityReferencedSQP.cpp
  ZMPRefTrajectoryGeneration/nmpc_generator.cpp
)
ENDIF(USE_QUADPROG)


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

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${${PROJECT_NAME}_ABSOLUTE_HEADERS})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LAPACK_LIBRARIES})

# Define dependencies
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-msse -msse2 -msse3 -march=core2 -mfpmath=sse -fivopts -ftree-loop-im -fipa-pta ")
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} pinocchio)

IF(USE_LSSOL)
 PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} lssol)
ENDIF(USE_LSSOL)

IF(USE_QUADPROG)
 PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} eigen-quadprog)
ENDIF(USE_QUADPROG)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})


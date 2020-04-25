# Copyright 2010, 2020, Olivier Stasse, Guilhem Saurel, JRL, CNRS/AIST, LAAS-CNRS
# See LICENSE file.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

# Project properties
SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME jrl-walkgen)
SET(PROJECT_DESCRIPTION "Walking pattern generator for Humanoids")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
OPTION(USE_LSSOL "Do you want to use the solver lssol?" OFF)
OPTION(USE_QUADPROG "Do you want to use the solver eigen-quadprog?" ON)
OPTION(SUFFIX_SO_VERSION "Suffix library name with its version" ON)
OPTION(FULL_BUILD_TESTING "Complete and long testing" OFF)

# Project configuration
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(CXX_DISABLE_WERROR TRUE)
SET(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX C)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
ADD_PROJECT_DEPENDENCY(LAPACK REQUIRED)
ADD_PROJECT_DEPENDENCY(pinocchio REQUIRED PKG_CONFIG_REQUIRES pinocchio)

IF(BUILD_TESTING)
  ADD_PROJECT_DEPENDENCY("talos_data" REQUIRED)
ENDIF(BUILD_TESTING)

# Handle OS specificities
INCLUDE(CheckIncludeFile)
CHECK_INCLUDE_FILE("sys/time.h" SYS_TIME_H)
#IF(SYS_TIME_H)
ADD_DEFINITIONS("-DHAVE_SYS_TIME_H")
#ENDIF(SYS_TIME_H)

# TODO kinda dirty patch to find lssol for now
#  using ADD_OPTIONAL_DEPENDENCY prevents the creation
#  of classic variables such as ${PKG}_FOUND
# hence we use ADD_REQUIRED_DEPENDENCY to get the data
IF(USE_LSSOL)
  ADD_REQUIRED_DEPENDENCY("lssol >= 0.1.0")
ENDIF(USE_LSSOL)

IF(USE_QUADPROG)
  ADD_REQUIRED_DEPENDENCY("eigen-quadprog >= 1.0.0")
ENDIF(USE_QUADPROG)

# Add aggressive optimization flags in release mode.
IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# Main Library

SET(${PROJECT_NAME}_HEADERS
  include/jrl/walkgen/patterngeneratorinterface.hh
  include/jrl/walkgen/pgtypes.hh
  include/jrl/walkgen/pinocchiorobot.hh
  )

SET(${PROJECT_NAME}_SOURCES
  src/RobotDynamics/pinocchiorobot.cpp
  src/FootTrajectoryGeneration/FootTrajectoryGenerationAbstract.cpp
  src/FootTrajectoryGeneration/FootTrajectoryGenerationStandard.cpp
  src/FootTrajectoryGeneration/FootTrajectoryGenerationMultiple.cpp
  src/FootTrajectoryGeneration/LeftAndRightFootTrajectoryGenerationMultiple.cpp
  src/FootTrajectoryGeneration/OnLineFootTrajectoryGeneration.cpp
  src/GlobalStrategyManagers/CoMAndFootOnlyStrategy.cpp
  src/GlobalStrategyManagers/GlobalStrategyManager.cpp
  src/GlobalStrategyManagers/DoubleStagePreviewControlStrategy.cpp
  src/Mathematics/AnalyticalZMPCOGTrajectory.cpp
  src/Mathematics/ConvexHull.cpp
  src/Mathematics/FootConstraintsAsLinearSystem.cpp
  #src/Mathematics/FootConstraintsAsLinearSystemForVelRef.cpp
  src/Mathematics/FootHalfSize.cpp
  src/Mathematics/OptCholesky.cpp
  src/Mathematics/Bsplines.cpp
  src/Mathematics/Polynome.cpp
  src/Mathematics/PolynomeFoot.cpp
  src/Mathematics/PLDPSolver.cpp
  src/Mathematics/qld.cpp
  src/Mathematics/StepOverPolynome.cpp
  src/Mathematics/relative-feet-inequalities.cpp
  src/Mathematics/intermediate-qp-matrices.cpp
  src/PreviewControl/PreviewControl.cpp
  src/PreviewControl/OptimalControllerSolver.cpp
  src/PreviewControl/ZMPPreviewControlWithMultiBodyZMP.cpp
  src/PreviewControl/LinearizedInvertedPendulum2D.cpp
  src/PreviewControl/rigid-body.cpp
  src/PreviewControl/rigid-body-system.cpp
  src/PreviewControl/SupportFSM.cpp
  src/ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.cpp
  src/ZMPRefTrajectoryGeneration/ZMPDiscretization.cpp
  src/ZMPRefTrajectoryGeneration/ZMPQPWithConstraint.cpp
  src/ZMPRefTrajectoryGeneration/ZMPConstrainedQPFastFormulation.cpp
  src/ZMPRefTrajectoryGeneration/ZMPVelocityReferencedQP.cpp
  src/ZMPRefTrajectoryGeneration/OrientationsPreview.cpp
  src/ZMPRefTrajectoryGeneration/AnalyticalMorisawaAbstract.cpp
  src/ZMPRefTrajectoryGeneration/AnalyticalMorisawaCompact.cpp
  src/ZMPRefTrajectoryGeneration/FilteringAnalyticalTrajectoryByPreviewControl.cpp
  src/ZMPRefTrajectoryGeneration/problem-vel-ref.cpp
  src/ZMPRefTrajectoryGeneration/qp-problem.cpp
  src/ZMPRefTrajectoryGeneration/generator-vel-ref.cpp
  src/ZMPRefTrajectoryGeneration/mpc-trajectory-generation.cpp
  src/ZMPRefTrajectoryGeneration/DynamicFilter.cpp
  #src/MultiContactRefTrajectoryGeneration/MultiContactHirukawa.cc
  src/MotionGeneration/StepOverPlanner.cpp
  src/MotionGeneration/CollisionDetector.cpp
  src/MotionGeneration/WaistHeightVariation.cpp
  src/MotionGeneration/UpperBodyMotion.cpp
  src/MotionGeneration/GenerateMotionFromKineoWorks.cpp
  src/MotionGeneration/ComAndFootRealizationByGeometry.cpp
  src/StepStackHandler.cpp
  src/PatternGeneratorInterfacePrivate.cpp
  src/SimplePlugin.cpp
  src/SimplePluginManager.cpp
  src/pgtypes.cpp
  src/Clock.cpp
  src/portability/gettimeofday.cc
  src/privatepgtypes.cpp
  )

IF(USE_QUADPROG)
  SET(${PROJECT_NAME}_SOURCES
    ${${PROJECT_NAME}_SOURCES}
    src/ZMPRefTrajectoryGeneration/ZMPVelocityReferencedSQP.cpp
    src/ZMPRefTrajectoryGeneration/nmpc_generator.cpp
    )
ENDIF(USE_QUADPROG)

ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/FootTrajectoryGeneration>
  PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LAPACK_LIBRARIES} pinocchio::pinocchio)
IF(USE_QUADPROG)
  TARGET_COMPILE_DEFINITIONS(${PROJECT_NAME} PUBLIC
    USE_QUADPROG=1)
ENDIF(USE_QUADPROG)

IF(SUFFIX_SO_VERSION)
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
ENDIF(SUFFIX_SO_VERSION)

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS "-msse -msse2 -msse3 -march=core2 -mfpmath=sse -fivopts -ftree-loop-im -fipa-pta ")

IF(USE_LSSOL)
  PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} lssol)
ENDIF(USE_LSSOL)

IF(USE_QUADPROG)
  PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} eigen-quadprog)
ENDIF(USE_QUADPROG)

INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

ADD_SUBDIRECTORY(tests)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})

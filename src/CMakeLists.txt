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

# add flag to compile qld.cc
IF(WIN32)
ADD_DEFINITIONS("/D __STDC__")
ENDIF(WIN32)

SET(SOURCES
  FootTrajectoryGeneration/FootTrajectoryGenerationAbstract.cpp
  FootTrajectoryGeneration/FootTrajectoryGenerationStandard.cpp
  FootTrajectoryGeneration/FootTrajectoryGenerationMultiple.cpp
  FootTrajectoryGeneration/LeftAndRightFootTrajectoryGenerationMultiple.cpp
  GlobalStrategyManagers/CoMAndFootOnlyStrategy.cpp
  GlobalStrategyManagers/GlobalStrategyManager.cpp
  GlobalStrategyManagers/DoubleStagePreviewControlStrategy.cpp
  Mathematics/AnalyticalZMPCOGTrajectory.cpp
  Mathematics/ConvexHull.cpp
  Mathematics/FootConstraintsAsLinearSystem.cpp
  Mathematics/FootConstraintsAsLinearSystemForVelRef.cpp
  Mathematics/FootHalfSize.cpp
  Mathematics/OptCholesky.cpp
  Mathematics/Polynome.cpp
  Mathematics/PolynomeFoot.cpp
  Mathematics/PLDPSolver.cpp
  Mathematics/PLDPHerdt.cpp
  Mathematics/qld.cpp
  Mathematics/StepOverPolynome.cpp
  Mathematics/intermediate-qp-matrices.cpp
  PreviewControl/PreviewControl.cpp
  PreviewControl/OptimalControllerSolver.cpp
  PreviewControl/ZMPPreviewControlWithMultiBodyZMP.cpp
  PreviewControl/LinearizedInvertedPendulum2D.cpp
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
  ZMPRefTrajectoryGeneration/qp-problem.cpp
  ZMPRefTrajectoryGeneration/generator-vel-ref.cpp
  ZMPRefTrajectoryGeneration/mpc-trajectory-generation.cpp
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
  privatepgtypes.cpp
  portability/gettimeofday.cc
  )

ADD_LIBRARY(jrl-walkgen SHARED ${SOURCES})
SET_TARGET_PROPERTIES(jrl-walkgen PROPERTIES SOVERSION ${PROJECT_VERSION})
INSTALL(TARGETS jrl-walkgen DESTINATION lib)

PKG_CONFIG_USE_DEPENDENCY(jrl-walkgen jrl-mal)

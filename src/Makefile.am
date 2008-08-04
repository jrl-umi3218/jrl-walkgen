#
# Copyright (c) 2007 LAAS/CNRS                        --  Wed Jan 31 2007
# All rights reserved.
#
# Redistribution  and  use in source   and binary forms,  with or without
# modification, are permitted provided that  the following conditions are
# met:
#
#   1. Redistributions  of  source code must  retain  the above copyright
#      notice, this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must  reproduce the above copyright
#      notice,  this list of  conditions and  the following disclaimer in
#      the  documentation   and/or  other  materials   provided with  the
#      distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE  AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY  EXPRESS OR IMPLIED WARRANTIES, INCLUDING,  BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES   OF MERCHANTABILITY AND  FITNESS  FOR  A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO  EVENT SHALL THE AUTHOR OR  CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT,  INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING,  BUT  NOT LIMITED TO, PROCUREMENT  OF
# SUBSTITUTE  GOODS OR SERVICES;  LOSS   OF  USE,  DATA, OR PROFITS;   OR
# BUSINESS  INTERRUPTION) HOWEVER CAUSED AND  ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE  USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

configdir=	$(datadir)/$(PACKAGE)
dist_config_DATA=\
	./data/HRP2LinkJointRank.xml		\
	./data/HRP2Specificities.xml		\
	./data/PreviewControlParameters.ini

lib_LTLIBRARIES = libwalkGenJRL.la
libwalkGenJRL_la_SOURCES = \
	./FootTrajectoryGeneration/FootTrajectoryGenerationAbstract.cpp \
	./FootTrajectoryGeneration/FootTrajectoryGenerationStandard.cpp \
	./FootTrajectoryGeneration/FootTrajectoryGenerationMultiple.cpp \
	./FootTrajectoryGeneration/LeftAndRightFootTrajectoryGenerationMultiple.cpp \
	./GlobalStrategyManagers/CoMAndFootOnlyStrategy.cpp \
	./GlobalStrategyManagers/GlobalStrategyManager.cpp \
	./GlobalStrategyManagers/DoubleStagePreviewControlStrategy.cpp \
	./Mathematics/Polynome.cpp \
	./Mathematics/PolynomeFoot.cpp \
	./Mathematics/StepOverPolynome.cpp\
	./Mathematics/ConvexHull.cpp \
	./Mathematics/qld.cpp \
	./Mathematics/AnalyticalZMPCOGTrajectory.cpp \
	./PreviewControl/PreviewControl.cpp \
	./PreviewControl/OptimalControllerSolver.cpp \
	./PreviewControl/ZMPPreviewControlWithMultiBodyZMP.cpp \
	./ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.cpp \
	./ZMPRefTrajectoryGeneration/ZMPDiscretization.cpp \
	./ZMPRefTrajectoryGeneration/ZMPQPWithConstraint.cpp \
	./ZMPRefTrajectoryGeneration/AnalyticalMorisawaAbstract.cpp \
	./ZMPRefTrajectoryGeneration/AnalyticalMorisawaCompact.cpp \
	./ZMPRefTrajectoryGeneration/AnalyticalMorisawaFull.cpp \
	./MotionGeneration/InverseKinematics.cpp \
	./MotionGeneration/StepOverPlanner.cpp\
	./MotionGeneration/CollisionDetector.cpp\
	./MotionGeneration/WaistHeightVariation.cpp\
	./MotionGeneration/UpperBodyMotion.cpp \
	./MotionGeneration/GenerateMotionFromKineoWorks.cpp \
	./MotionGeneration/ComAndFootRealizationByGeometry.cpp \
	StepStackHandler.cpp \
	PatternGeneratorInterface.cpp \
	SimplePlugin.cpp \
	SimplePluginManager.cpp \
	Clock.cpp

libwalkGenJRL_la_CPPFLAGS = \
	-I$(srcdir)/../include/ \
	@DYNAMICJRLJAP_CFLAGS@ 


libwalkGenJRL_la_LDFLAGS = \
	@DYNAMICJRLJAP_LIBS@		\
	-release ${PACKAGE_VERSION}

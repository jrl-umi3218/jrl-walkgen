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
	HRP2Specificities.xml		\
	PreviewControlParameters.ini

lib_LTLIBRARIES = libwalkGenJRL.la
libwalkGenJRL_la_SOURCES = \
	Polynome.cpp \
	PolynomeFoot.cpp \
	ZMPDiscretization.cpp \
	PreviewControl.cpp \
	InverseKinematics.cpp \
	MultiBody.cpp \
	DynamicMultiBody.cpp \
	fileReader.cpp \
	Body.cpp \
	DynamicBody.cpp \
	OptimalControllerSolver.cpp \
	ZMPPreviewControlWithMultiBodyZMP.cpp \
	StepOverPlanner.cpp\
	StepOverPolynome.cpp\
	CollisionDetector.cpp\
        WaistHeightVariation.cpp\
	UpperBodyMotion.cpp \
	GenerateMotionFromKineoWorks.cpp \
	StepStackHandler.cpp \
	PatternGeneratorInterface.cpp \
	ConvexHull.cpp \
	HumanoidSpecificities.cpp \
	qld.cpp

libwalkGenJRL_la_CPPFLAGS = -I$(srcdir) -I$(top_srcdir)/include/VNL/NetLib

bin_PROGRAMS = TestFootPrintPGInterface
TestFootPrintPGInterface_SOURCES = TestFootPrintPGInterface.cpp
TestFootPrintPGInterface_CPPFLAGS = -I$(srcdir) -I$(top_srcdir)/include/VNL/NetLib
TestFootPrintPGInterface_LDADD = $(top_builddir)/src/libwalkGenJRL.la

# XXX Please select only public headers, i.e. those that are to be installed.
walkGenJRLdir = @includedir@/walkGenJRL
walkGenJRL_HEADERS = \
	Body.h \
	CollisionDetector.h \
	ConvexHull.h \
	DynamicBody.h \
	DynamicMultiBody.h \
	GenerateMotionFromKineoWorks.h \
	HumanoidDynamicMultiBody.h \
	HumanoidSpecificities.h \
	InverseKinematics.h \
	Joint.h \
	MatrixAbstractLayer.h \
	MatrixAbstractLayerBoost.h \
	MatrixAbstractLayerDoc.h \
	MatrixAbstractLayerSmall.h \
	MatrixAbstractLayerSmallMatrix3x3Default.h \
	MatrixAbstractLayerSmallVector3Default.h \
	MatrixAbstractLayerVNL.h \
	MultiBody.h \
	OptimalControllerSolver.h \
	PGTypes.h \
	PatternGeneratorInterface.h \
	Polynome.h \
	PolynomeFoot.h \
	PreviewControl.h \
	StepOverPlanner.h \
	StepOverPolynome.h \
	StepStackHandler.h \
	UpperBodyMotion.h \
	WaistHeightVariation.h \
	ZMPDiscretization.h \
	ZMPPreviewControlWithMultiBodyZMP.h \
	configJRLWPG.h \
	fileReader.h \
	linalg.h \
	mathout.h \
	matrix.h \
	matrix3.h \
	matrix4.h \
	qld.h \
	quaternion.h \
	vector3.h 

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

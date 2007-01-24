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
TestFootPrintPGInterface_CPPFLAGS = -I$(srcdir) 
TestFootPrintPGInterface_LDADD = $(top_builddir)/src/libwalkGenJRL.la

walkGenJRLdir = @includedir@/walkGenJRL
walkGenJRL_HEADERS = \
	Polynome.h \
	PolynomeFoot.h \
	ZMPDiscretization.h \
	PreviewControl.h \
	InverseKinematics.h \
	MultiBody.h \
	DynamicMultiBody.h \
	fileReader.h \
	Body.h \
	DynamicBody.h \
	ZMPPreviewControlWithMultiBodyZMP.h \
	StepOverPlanner.h\
	StepOverPolynome.h\
	CollisionDetector.h\
	WaistHeightVariation.h\
	OptimalControllerSolver.h \
	UpperBodyMotion.h \
	GenerateMotionFromKineoWorks.h \
	StepStackHandler.h \
	PatternGeneratorInterface.h \
	ConvexHull.h \
	HumanoidSpecificities.h \
	qld.h 

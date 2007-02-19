lib_LTLIBRARIES = libwalkGenJRL.la
libwalkGenJRL_la_SOURCES = \
	./Mathematics/Polynome.cpp \
	./Mathematics/PolynomeFoot.cpp \
	./Mathematics/StepOverPolynome.cpp\
	./Mathematics/ConvexHull.cpp \
	./Mathematics/qld.cpp \
	./PreviewControl/PreviewControl.cpp \
	./PreviewControl/OptimalControllerSolver.cpp \
	./PreviewControl/ZMPPreviewControlWithMultiBodyZMP.cpp \
	./ZMPRefTrajectoryGeneration/ZMPDiscretization.cpp \
	./RobotModel/fileReader.cpp \
	./RobotModel/Joint.cpp \
	./RobotModel/Body.cpp \
	./RobotModel/MultiBody.cpp \
	./RobotModel/DynamicBody.cpp \
	./RobotModel/DynamicMultiBody.cpp \
	./RobotModel/HumanoidDynamicMultiBody.cpp \
	./RobotModel/HumanoidSpecificities.cpp \
	./MotionGeneration/InverseKinematics.cpp \
	./MotionGeneration/StepOverPlanner.cpp\
	./MotionGeneration/CollisionDetector.cpp\
	./MotionGeneration/WaistHeightVariation.cpp\
	./MotionGeneration/UpperBodyMotion.cpp \
	./MotionGeneration/GenerateMotionFromKineoWorks.cpp \
	./MotionGeneration/ComAndFootRealizationByGeometry.cpp \
	StepStackHandler.cpp \
	PatternGeneratorInterface.cpp 

libwalkGenJRL_la_CPPFLAGS = -I$(srcdir)/Mathematics -I$(srcdir)/PreviewControl -I$(srcdir)/ZMPRefTrajectoryGeneration -I$(srcdir)/RobotModel -I$(srcdir)/MotionGeneration -I$(srcdir) -I$(top_srcdir)/include/VNL/NetLib -I$(top_srcdir)/../jrlCIR-dynamics/interfaces

bin_PROGRAMS = TestFootPrintPGInterface TestHumanoidDynamicRobot

TestFootPrintPGInterface_SOURCES = TestFootPrintPGInterface.cpp
TestFootPrintPGInterface_CPPFLAGS = -I$(srcdir)/Mathematics -I$(srcdir)/PreviewControl -I$(srcdir)/ZMPRefTrajectoryGeneration -I$(srcdir)/RobotModel -I$(srcdir)/MotionGeneration -I$(srcdir) -I$(top_srcdir)/include/VNL/NetLib -I$(top_srcdir)/../jrlCIR-dynamics/interfaces
TestFootPrintPGInterface_LDADD = $(top_builddir)/src/libwalkGenJRL.la

TestHumanoidDynamicRobot_SOURCES = TestHumanoidDynamicRobot.cpp
TestHumanoidDynamicRobot_CPPFLAGS = -I$(srcdir)/Mathematics -I$(srcdir)/PreviewControl -I$(srcdir)/ZMPRefTrajectoryGeneration -I$(srcdir)/RobotModel -I$(srcdir)/MotionGeneration -I$(srcdir) -I$(top_srcdir)/include/VNL/NetLib -I$(top_srcdir)/../jrlCIR-dynamics/interfaces
TestHumanoidDynamicRobot_LDADD = $(top_builddir)/src/libwalkGenJRL.la

walkGenJRLdir = @includedir@/walkGenJRL
walkGenJRL_HEADERS = \
	./Mathematics/Polynome.h \
	./Mathematics/PolynomeFoot.h \
	./Mathematics/StepOverPolynome.h\
	./Mathematics/ConvexHull.h \
	./Mathematics/qld.h \
	./PreviewControl/PreviewControl.h \
	./PreviewControl/OptimalControllerSolver.h \
	./PreviewControl/ZMPPreviewControlWithMultiBodyZMP.h \
	./ZMPRefTrajectoryGeneration/ZMPDiscretization.h \
	./RobotModel/fileReader.h \
	./RobotModel/Body.h \
	./RobotModel/MultiBody.h \
	./RobotModel/DynamicBody.h \
	./RobotModel/DynamicMultiBody.h \
	./RobotModel/HumanoidDynamicMultiBody.h \
	./RobotModel/HumanoidSpecificities.h \
	./MotionGeneration/InverseKinematics.h \
	./MotionGeneration/StepOverPlanner.h\
	./MotionGeneration/CollisionDetector.h\
	./MotionGeneration/WaistHeightVariation.h\
	./MotionGeneration/UpperBodyMotion.h \
	./MotionGeneration/GenerateMotionFromKineoWorks.h \
	./MotionGeneration/ComAndFootRealization.h \
	./MotionGeneration/ComAndFootRealizationByGeometry.h \
	StepStackHandler.h \
	PatternGeneratorInterface.h 

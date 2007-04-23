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
	./MotionGeneration/InverseKinematics.cpp \
	./MotionGeneration/StepOverPlanner.cpp\
	./MotionGeneration/CollisionDetector.cpp\
	./MotionGeneration/WaistHeightVariation.cpp\
	./MotionGeneration/UpperBodyMotion.cpp \
	./MotionGeneration/GenerateMotionFromKineoWorks.cpp \
	./MotionGeneration/ComAndFootRealizationByGeometry.cpp \
	StepStackHandler.cpp \
	PatternGeneratorInterface.cpp 

libwalkGenJRL_la_CPPFLAGS = -I$(srcdir)/../include/walkGenJrl/	  
libwalkGenJRL_la_LIBS = -llapack
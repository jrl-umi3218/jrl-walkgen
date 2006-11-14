#   Copyright (c) 2005-2006, 
#   Olivier Stasse, Ramzi Sellouati
#   
#    CVS Information:
#    $Id: Makefile,v 1.3 2006-01-18 06:34:58 stasse Exp $
#    $Author: stasse $
#    $Date: 2006-01-18 06:34:58 $
#    $Revision: 1.3 $
#    $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/Makefile,v $
#    $Log: Makefile,v $
#    Revision 1.3  2006-01-18 06:34:58  stasse
#    OS: Updated the names of the contributors, the documentation
#    and added a sample file for WalkPlugin
#
#
#   JRL-Japan, CNRS/AIST
#
#   All rights reserved.
#   
#   Redistribution and use in source and binary forms, with or without modification, 
#   are permitted provided that the following conditions are met:
#   
#   * Redistributions of source code must retain the above copyright notice, 
#   this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright notice, 
#   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#   * Neither the name of the <ORGANIZATION> nor the names of its contributors 
#   may be used to endorse or promote products derived from this software without specific prior written permission.
#   
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
#   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
#   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
#   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
#   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
#   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
#   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
#   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
#   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

LIBRARY=libWalkGenJRL.a
SHLIBRARY=libWalkGenJRL.so

all: $(LIBRARY) $(SHLIBRARY)  TestFootPrintPGInterface \
#	TestFootPrint1 TestGMFKW TestFootPrint2	\
#	TestRiccatiEquation

# TestLapack

AR= ar cru
CRTSHLIB= gcc-3.3 -shared -Wl


CXX=g++-3.3

HDRS= Polynome.h \
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
	UpperBodyMotion.h \
	GenerateMotionFromKineoWorks.h \
	StepStackHandler.h \
	PatternGeneratorInterface.h \
	OptimalControllerSolver.h \
	ConvexHull.h \
	HumanoidSpecificities.h \
	qld.h 

SRCS= Polynome.cpp \
	PolynomeFoot.cpp \
	ZMPDiscretization.cpp \
	PreviewControl.cpp \
	InverseKinematics.cpp \
	MultiBody.cpp \
	DynamicMultiBody.cpp \
	fileReader.cpp \
	Body.cpp \
	DynamicBody.cpp \
	ZMPPreviewControlWithMultiBodyZMP.cpp \
	StepOverPlanner.cpp\
	StepOverPolynome.cpp\
	CollisionDetector.cpp\
        WaistHeightVariation.cpp\
	UpperBodyMotion.cpp \
	GenerateMotionFromKineoWorks.cpp \
	StepStackHandler.cpp \
	PatternGeneratorInterface.cpp \
	OptimalControllerSolver.cpp \
	ConvexHull.cpp \
	HumanoidSpecificities.cpp \
	qld.cpp


OBJS=${SRCS:.cpp=.o}

CXX_FLAGS = -Wall 
CXX_OPT = -g

#CXX_OPT = -O3
CXX_FLAGS +=  -I. -I../include  -Wall


LIBS_FLAGS = -L. -L../lib -lVNL -lg2c 

ROBOT=HRP2JRL
PICFLAGS = -fpic

$(OBJS): %.o : %.cpp 
	$(CXX) $(CXX_OPT) $(CXX_FLAGS) -c $<
	$(CXX) $(CXX_OPT) $(CXX_FLAGS) $(PICFLAGS) -c -o ./shared/$@ $<

$(OBJS_C): %.o : %.c
	$(CC) $(CC_OPT) $(CC_FLAGS) -c $<
	$(CC) $(CC_OPT) $(CC_FLAGS) $(PICFLAGS) -c -o ./shared/$@ $<

TestFootPrint1.o: TestFootPrint1.cpp $(HDRS) 
	$(CXX) $(CXX_OPT) $(CXX_FLAGS) -c $<

TestFootPrint1:TestFootPrint1.o $(OBJS)
	$(CXX) -o $@ TestFootPrint1.o $(OBJS)  $(LIBS_FLAGS) 

TestFootPrint2.o: TestFootPrint2.cpp $(HDRS) 
	$(CXX) $(CXX_OPT) $(CXX_FLAGS) -c $<

TestFootPrint2:TestFootPrint2.o $(OBJS)
	$(CXX) -o $@ TestFootPrint2.o $(OBJS)  $(LIBS_FLAGS) 

TestGMFKW.o: TestGMFKW.cpp $(HDRS) 
	$(CXX) $(CXX_OPT) $(CXX_FLAGS) -c $<

TestGMFKW: TestGMFKW.o $(OBJS)
	$(CXX) -g -o $@ TestGMFKW.o  -L.  -lWalkGenJRL  $(LIBS_FLAGS)

TestFootPrintPGInterface.o: TestFootPrintPGInterface.cpp $(HDRS) 
	$(CXX) $(CXX_OPT) $(CXX_FLAGS) -c $<

TestFootPrintPGInterface: TestFootPrintPGInterface.o $(OBJS)
	$(CXX) -g -o $@ TestFootPrintPGInterface.o  -L.  -lWalkGenJRL $(LIBS_FLAGS)

TestRiccatiEquation.o: TestRiccatiEquation.cpp $(HDRS) 
	$(CXX) $(CXX_OPT) $(CXX_FLAGS) -c $<

TestRiccatiEquation: TestRiccatiEquation.o $(OBJS)
	$(CXX) -g -o $@ TestRiccatiEquation.o  -L.  -lWalkGenJRL  $(LIBS_FLAGS)

TestLapack.o: TestLapack.cpp $(HDRS) 
	$(CXX) $(CXX_OPT) $(CXX_FLAGS) -c $<

#TestLapack: TestLapack.o 
#	$(CXX) -g -o $@ $< ./libdgges.a -lf2c ./blas.a

$(LIBRARY): $(OBJS) $(OBJS_C)
	$(AR) $(LIBRARY) $(OBJS) $(OBJS_C)

$(SHLIBRARY): $(OBJS) $(OBJS_C)
	(cd shared; $(CRTSHLIB) -o $@ $(OBJS))
	mv shared/$@ $@

$(SHLIBRARY): $(OBJS)

doc:	
	doc++ --dir html $(HDRS)
clean:
	rm -f *.o *~ *\# *.so *.a
	(cd shared; rm -f *.o)

clean-all:	
	rm -f *.o
	(cd shared; rm -f *.o)
	rm -f *.a *.so *~ *.dat
	rm -f TestFootPrint1 TestFootPrint2


install:
	cp -f *.h ../include
	cp -f $(LIBRARY) ../lib
	cp -f $(SHLIBRARY) ../lib
	cp -f $(SHLIBRARY) ../../../../Controller/IOserver/robot/HRP2JRL/bin/

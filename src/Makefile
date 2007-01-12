#   Copyright (c) 2005-2006, 
#   Olivier Stasse, Ramzi Sellouati
#   
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
	OptimalControllerSolver.h \
	UpperBodyMotion.h \
	GenerateMotionFromKineoWorks.h \
	StepStackHandler.h \
	PatternGeneratorInterface.h \
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


#LIB_MATRIX=_VNL_MATRIX_
LIB_MATRIX=_BOOST_MATRIX_

OBJS=${SRCS:.cpp=.o}

CXX_FLAGS = -Wall 
# Choice for the matrix library

ifeq ($(LIB_MATRIX),_BOOST_MATRIX_)
  CXX_FLAGS += -I/usr/include/boost-sandbox/ 
  CXX_FLAGS += -I/usr/local/atlas/include
endif


#CXX_OPT = -g
CXX_OPT = -O3

CXX_FLAGS += -D_DEFAULT_MATRIX3x3_ -D_DEFAULT_VECTOR3D_


CXX_FLAGS +=   -I../include -I. -Wall
CXX_FLAGS += -D$(LIB_MATRIX)

LIBS_FLAGS = -L. 

ifeq ($(LIB_MATRIX),_VNL_MATRIX_)
 LIBS_FLAGS+=-L../lib -lVNL -llapack
endif

ifeq ($(LIB_MATRIX),_BOOST_MATRIX_)
# LIBS_FLAGS+= -L/usr/local/atlas/lib -llapack -lcblas -latlas
  LIBS_FLAGS+= -llapack 
endif

LIBS_FLAGS+=

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

TestDynamicMultiBody.o: TestDynamicMultiBody.cpp $(HDRS) 
	$(CXX) $(CXX_OPT) $(CXX_FLAGS) -c $<

TestDynamicMultiBody: TestDynamicMultiBody.o $(OBJS)
	$(CXX) -g -o $@ TestDynamicMultiBody.o  -L.  -lWalkGenJRL $(LIBS_FLAGS)

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
	rm -f ../include/*.h
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

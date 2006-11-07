CXX=g++-3.3
CC=g++-3.3
CONFIG += qt opengl
HEADERS += HRP2Display.h HRP2DisplayGL.h \
	MotorValuesEdit.h MainView.h \
	HRP2Model/Dynamique.h \	
	HRP2Model/fileReader.h \
	HRP2Model/Link.h \	
	HRP2Model/mathparse.h \	
	joystick.h

SOURCES += HRP2Display.cpp HRP2DisplayGL.cpp MotorValuesEdit.cpp main.cpp MainView.cpp \
	HRP2Model/ASE.cpp joystick.cpp

INCLUDEPATH += ./HRP2Model /usr/include/GL ../src ../include
LIBS += -lglut -L../src/ -lWalkGenJRL -L../lib -lVNL -lg2c

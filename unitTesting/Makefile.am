# 
#  Copyright 
# 


noinst_PROGRAMS = TestFootPrintPGInterface TestRiccatiEquation

TestFootPrintPGInterface_SOURCES = TestFootPrintPGInterface.cpp 
TestFootPrintPGInterface_CPPFLAGS = -I$(srcdir)/../include/walkGenJrl/ @DYNAMICJRLJAP_CFLAGS@
TestFootPrintPGInterface_LDADD = -L../src -ldynamicsJRLJapan -lwalkGenJRL @DYNAMICJRLJAP_LIBS@

TestRiccatiEquation_SOURCES = TestRiccatiEquation.cpp 
TestRiccatiEquation_CPPFLAGS = -I$(srcdir)/../include/walkGenJrl/ @DYNAMICJRLJAP_CFLAGS@
TestRiccatiEquation_LDADD = -L../src -ldynamicsJRLJapan -lwalkGenJRL @DYNAMICJRLJAP_LIBS@

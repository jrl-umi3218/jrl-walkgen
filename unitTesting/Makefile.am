# 
#  Copyright 
# 


noinst_PROGRAMS = TestFootPrintPGInterface TestRiccatiEquation TestMinimal

TestFootPrintPGInterface_SOURCES = TestFootPrintPGInterface.cpp 
TestFootPrintPGInterface_CPPFLAGS = -I$(srcdir)/../include/walkGenJrl/ @DYNAMICJRLJAP_CFLAGS@
TestFootPrintPGInterface_LDADD = -L../src -ldynamicsJRLJapan -lwalkGenJRL @DYNAMICJRLJAP_LIBS@

TestRiccatiEquation_SOURCES = TestRiccatiEquation.cpp 
TestRiccatiEquation_CPPFLAGS = -I$(srcdir)/../include/walkGenJrl/ @DYNAMICJRLJAP_CFLAGS@
TestRiccatiEquation_LDADD = -L../src -ldynamicsJRLJapan -lwalkGenJRL @DYNAMICJRLJAP_LIBS@

TestMinimal_SOURCES = TestMinimal.cpp 
TestMinimal_CPPFLAGS = -I$(srcdir)/../include/walkGenJrl/ @DYNAMICJRLJAP_CFLAGS@
TestMinimal_LDADD = -L../src -ldynamicsJRLJapan -lwalkGenJRL @DYNAMICJRLJAP_LIBS@

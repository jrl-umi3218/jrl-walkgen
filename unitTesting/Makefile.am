# 
#  Copyright 
# 
noinst_PROGRAMS = TestFootPrintPGInterface \
	TestRiccatiEquation \
	TestMinimal \
	TestAnalyticalMorisawa \
	TestAnalyticalMorisawaCompact \
	TestAnalyticalMorisawaCompact2

TestFootPrintPGInterface_SOURCES = TestFootPrintPGInterface.cpp 
TestFootPrintPGInterface_CPPFLAGS = -I$(srcdir)/../include/ @DYNAMICJRLJAP_CFLAGS@
TestFootPrintPGInterface_LDADD = -L../src -ldynamicsJRLJapan -lwalkGenJRL @DYNAMICJRLJAP_LIBS@ 

TestRiccatiEquation_SOURCES = TestRiccatiEquation.cpp 
TestRiccatiEquation_CPPFLAGS = -I$(srcdir)/../include/ @DYNAMICJRLJAP_CFLAGS@
TestRiccatiEquation_LDADD = -L../src -ldynamicsJRLJapan -lwalkGenJRL @DYNAMICJRLJAP_LIBS@

TestMinimal_SOURCES = TestMinimal.cpp 
TestMinimal_CPPFLAGS = -I$(srcdir)/../include/ @DYNAMICJRLJAP_CFLAGS@
TestMinimal_LDADD = -L../src -ldynamicsJRLJapan -lwalkGenJRL @DYNAMICJRLJAP_LIBS@


TestAnalyticalMorisawa_SOURCES = TestAnalyticalMorisawa.cpp 
TestAnalyticalMorisawa_CPPFLAGS = -I$(srcdir)/../include/ @DYNAMICJRLJAP_CFLAGS@
TestAnalyticalMorisawa_LDADD = -L../src -ldynamicsJRLJapan -lwalkGenJRL @DYNAMICJRLJAP_LIBS@

TestAnalyticalMorisawaCompact_SOURCES = TestAnalyticalMorisawaCompact.cpp 
TestAnalyticalMorisawaCompact_CPPFLAGS = -I$(srcdir)/../include/ @DYNAMICJRLJAP_CFLAGS@
TestAnalyticalMorisawaCompact_LDADD = -L../src -ldynamicsJRLJapan -lwalkGenJRL @DYNAMICJRLJAP_LIBS@

TestAnalyticalMorisawaCompact2_SOURCES = TestAnalyticalMorisawaCompact2.cpp 
TestAnalyticalMorisawaCompact2_CPPFLAGS = -I$(srcdir)/../include/ @DYNAMICJRLJAP_CFLAGS@
TestAnalyticalMorisawaCompact2_LDADD = -L../src -ldynamicsJRLJapan -lwalkGenJRL @DYNAMICJRLJAP_LIBS@

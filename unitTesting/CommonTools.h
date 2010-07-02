/* Olivier Stasse
 * (c) 2010
 * 
 */
#ifdef UNIX
#include <sys/time.h>
#include <stdlib.h>
#endif /*UNIX*/

#ifdef WIN32
#include <Windows.h>
#include "TimeUtilsWindows.h"
#endif /*WIN32*/

#include <time.h>
#include <sstream>
#include <fstream>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

#ifdef WITH_HRP2DYNAMICS
#include <hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h>
#endif

#include <walkGenJrl/PatternGeneratorInterface.h>

#include "TestFootPrintPGInterfaceData.h"

#ifndef _COMMON_TOOLS_PATTERN_GENERATOR_UTESTING_H_
#define _COMMON_TOOLS_PATTERN_GENERATOR_UTESTING_H_

void getOptions(int , char *[], 
		std::string &VRMLPath,
		std::string &VRMLFileName,
		std::string &SpecificitiesFileName,
		std::string &LinkJointRank,
		unsigned int &TestProfil);

void CommonInitialization(PatternGeneratorJRL::PatternGeneratorInterface &aPGI);

void CreateAndInitializeHumanoidRobot(std::string &RobotFileName,
				      std::string &LinkJointRank,
				      std::string &SpecificitiesFileName,
				      CjrlHumanoidDynamicRobot *& aHDR,
				      CjrlHumanoidDynamicRobot *& aDebugHDR,
				      PatternGeneratorJRL::PatternGeneratorInterface *&aPGI);

#endif /* _COMMON_TOOLS_PATTERN_GENERATOR_UTESTING_H_*/

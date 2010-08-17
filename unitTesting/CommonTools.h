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

#include <string.h>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

#ifdef WITH_HRP2DYNAMICS
#include <hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h>
#endif

#include <walkGenJrl/PatternGeneratorInterface.h>

#include "TestFootPrintPGInterfaceData.h"
#include "ClockCPUTime.h"

#ifndef _COMMON_TOOLS_PATTERN_GENERATOR_UTESTING_H_
#define _COMMON_TOOLS_PATTERN_GENERATOR_UTESTING_H_

namespace PatternGeneratorJRL
{
  namespace TestSuite
  {
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
    
    /*! \brief Structure to handle information related to one step of each algorithm */
    struct OneStep
    {
      COMPosition finalCOMPosition;
      FootAbsolutePosition LeftFootPosition;
      FootAbsolutePosition RightFootPosition;
      MAL_VECTOR(ZMPTarget,double);
      unsigned long int NbOfIt;
      
      OneStep()
      {
	MAL_VECTOR_RESIZE(ZMPTarget,3);
	NbOfIt = 0;
	memset(&LeftFootPosition,0,sizeof(LeftFootPosition));
	memset(&RightFootPosition,0,sizeof(RightFootPosition));
	memset(&finalCOMPosition,0,sizeof(finalCOMPosition));
      }
    };    

  }; /* end of TestSuite namespace */
}; /* end of PatternGeneratorJRL namespace */
#endif /* _COMMON_TOOLS_PATTERN_GENERATOR_UTESTING_H_*/

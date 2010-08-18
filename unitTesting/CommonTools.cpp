
/* Olivier Stasse
 * (c) 2005-2009
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


#include <sstream>
#include <fstream>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

#ifdef WITH_HRP2DYNAMICS
#include <hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h>
#endif

#include <walkGenJrl/PatternGeneratorInterface.h>

#include "TestFootPrintPGInterfaceData.h"

using namespace std;

namespace PatternGeneratorJRL {
namespace TestSuite {

double InitialPoses[7][40] = {

  // 1- With previous half-sitting value
  { 
    0.0, 0.0, -20.0, 40.0, -20.0, 0.0, 0.0, 0.0, -20.0, 40.0, -20.0, 0.0,  // legs

    0.0, 0.0, -23.7, 6.6, // chest and head

    27.0, -5.0, -4.0, -87.0, -4.0, -16.0, 20.0, // right arm
    15.0,  10.0, 0.0, -20.0, 0.0, 0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  },
  // 2- Nicolas position + New half sitting for the legs
  { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, -23.7, 6.6, // chest and head

    27.0, -5.0, -4.0, -87.0, -4.0, -16.0, 20.0, // right arm
    15.0,  10.0, 0.0, -20.0, 0.0, 0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  },
  // 3- Test for comparison with PG v1.x
  { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // right arm
    0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // left arm 

    0.0, 0.0, 0.0, 0.0, 0.0, // right hand
    0.0, 0.0, 0.0, 0.0, 0.0  // left hand
  },
  // 4- New Half sitting
  { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // left arm 

    -10.0, 10.0, -10.0, 10.0, -10.0,  // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  },
  // 5- Position for interaction
  {
    0.0 ,  0.0 , -26.0 ,  50.0 , -24 ,   0.0,  
    0.0 ,  0.0 , -26.0 ,  50.0 , -24 ,   0.0 , // legs
    0.0 ,  0.0 ,  // chest
    0.0 ,  0.0 , // head 

    10.0, -18.0, 0.0, -100.0, -18.0, 0.0, 10.0, // right arm  
    10.0,  18.0, 0.0, -100.0,  18.0, 0.0, 10.0, 
    -10.000004 ,  10.000004 , -10.000004 ,  10.000004 , -10.000004 , // right hand 
    -10.000004 ,  10.000004 , -10.000004 ,  10.000004 , -10.000004 // left hand
  },
  // 6- Initial position for model building 1,
  {
    14.323945,  -6.0363396,  -13.459409,    44.02602,  -30.566611,    6.0363396,
    0.0000001,   7.4859801,  -27.663319,    44.65489,  -16.991579,   -7.4859801,
    0.,    0.,    0.,    0.,    
    12.397718,  -10.000004,    0.,  -29.618538,    0.,    0.,    10.0,
    16.536364,   10.000004,    0.,  -29.828011,    0.,    0.,    10.0,
    
    -10.0,  10.0, -10.0,  10,   -10.0, 
    -10.0,  10.0, -10.0,  10.0, -10.0 
  },
  // 7- Initial position for model buiding 2
  {
    -7.16197, -7.69299, -16.1787, 44.5201, -28.3415,  7.69299, 
    7.16197,   5.74946, -31.3668, 44.1057, -12.7389, -5.74946,
    
    0., 0., 0., 0., 
    
    12.622 , -10, 0, -29.678 , 0, 0, 10, 
    16.7091,  10, 0, -29.7841, 0, 0, 10, 
    
    -10.0,  10.0, -10.0,  10,   -10.0, 
    -10.0,  10.0, -10.0,  10.0, -10.0 
  }

};

//      ":comheight 0.807727",
void CommonInitialization(PatternGeneratorInterface &aPGI)
{
  const char lBuffer[12][256] =
    {":comheight 0.8078",
     ":samplingperiod 0.005",
     ":previewcontroltime 1.6",
     ":omega 0.0",
     ":stepheight 0.07",
     ":singlesupporttime 0.78",
     ":doublesupporttime 0.02",
     ":armparameters 0.5",
     ":LimitsFeasibility 0.0",
     ":ZMPShiftParameters 0.015 0.015 0.015 0.015",
     ":TimeDistributionParameters 2.0 3.7 1.7 3.0",
     ":UpperBodyMotionParameters -0.1 -1.0 0.0"
    };
  
  for(int i=0;i<9;i++)
    {
      std::istringstream strm(lBuffer[i]);
      aPGI.ParseCmd(strm);
    }
  // Evaluate current state of the robot in the PG.
  COMState   lStartingCOMPosition;
  MAL_S3_VECTOR(,double)  lStartingZMPPosition;
  MAL_VECTOR(,double)  lStartingWaistPose;
  FootAbsolutePosition  InitLeftFootAbsPos;
  FootAbsolutePosition  InitRightFootAbsPos;

  aPGI.EvaluateStartingState(lStartingCOMPosition,
			     lStartingZMPPosition,
			     lStartingWaistPose,
			     InitLeftFootAbsPos,
			     InitRightFootAbsPos);
      

}

void CreateAndInitializeHumanoidRobot(string &RobotFileName,
				      string &SpecificitiesFileName,
				      string &LinkJointRank,
				      CjrlHumanoidDynamicRobot * & aHDR,
				      CjrlHumanoidDynamicRobot * & aDebugHDR,
				      PatternGeneratorInterface * & aPGI)
{
  // Creating the humanoid robot.
  //CjrlHumanoidDynamicRobot * aHDR = 0, * aDebugHDR = 0;
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;

#ifndef WITH_HRP2DYNAMICS
  aHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
  aDebugHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
#else
  Chrp2OptHumanoidDynamicRobot *aHRP2HDR= new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
  aHDR = aHRP2HDR;
  aDebugHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
#endif

  // Parsing the file.
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,
					 LinkJointRank,
					 SpecificitiesFileName);

  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aDebugHDR,RobotFileName,
					 LinkJointRank,
					 SpecificitiesFileName);

  
  // Create Pattern Generator Interface
  aPGI = patternGeneratorInterfaceFactory(aHDR);

  bool conversiontoradneeded=true;
  
  //  double * dInitPos = InitialPoses[INTERACTION_2008];
  double * dInitPos = InitialPoses[HALF_SITTING_2008];

  bool DebugConfiguration = true;
  ofstream aofq;
  if (DebugConfiguration)
    {
      aofq.open("TestConfiguration.dat",ofstream::out);
      if (aofq.is_open())
	{
	  for(unsigned int k=0;k<30;k++)
	    {
	      aofq << dInitPos[k] << " ";
	    }
	  aofq << endl;
	}

    }
  
  // This is a vector corresponding to the DOFs actuated of the robot.
  MAL_VECTOR_DIM(InitialPosition,double,40);
  //MAL_VECTOR_DIM(CurrentPosition,double,40);
  if (conversiontoradneeded)
    for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
      InitialPosition(i) = dInitPos[i]*M_PI/180.0;
  else
    for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
      InitialPosition(i) = dInitPos[i];
  aPGI->SetCurrentJointValues(InitialPosition);

  // Specify the walking mode: here the default one.
  istringstream strm2(":walkmode 0");
  aPGI->ParseCmd(strm2);

  // This is a vector corresponding to ALL the DOFS of the robot:
  // free flyer + actuated DOFS.
  MAL_VECTOR_DIM(CurrentConfiguration,double,46);
  MAL_VECTOR_DIM(CurrentVelocity,double,46);
  MAL_VECTOR_DIM(CurrentAcceleration,double,46);
  MAL_VECTOR_DIM(PreviousConfiguration,double,46) ;
  MAL_VECTOR_DIM(PreviousVelocity,double,46);
  MAL_VECTOR_DIM(PreviousAcceleration,double,46);
  for(int i=0;i<6;i++)
    {
      PreviousConfiguration[i] = 
	PreviousVelocity[i] = 
	PreviousAcceleration[i] = 0.0;
    }

  for(int i=6;i<46;i++)
    {
      PreviousConfiguration[i] = InitialPosition[i-6];
      PreviousVelocity[i] = 
	PreviousAcceleration[i] = 0.0;
    }

  MAL_VECTOR_DIM(ZMPTarget,double,3);
  
  
  string inProperty[5]={"TimeStep","ComputeAcceleration",
			"ComputeBackwardDynamics", "ComputeZMP",
			"ResetIteration"};
  string inValue[5]={"0.005","false","false","true","true"};
  
  for(unsigned int i=0;i<5;i++)
    aDebugHDR->setProperty(inProperty[i],
			   inValue[i]);

  
}

void getOptions(int argc,
		char *argv[],
		string &VRMLPath,
		string &VRMLFileName,
		string &SpecificitiesFileName,
		string &LinkJointRank,
		unsigned int &TestProfil)
{
  if (argc!=5)
    {
      const char *openhrphome="OPENHRPHOME";
      char *value = 0;
      value = getenv(openhrphome);
      if (value==0)
	{
	  cerr << " This program takes 4 arguments: " << endl;
	  cerr << "./TestFootPrintPGInterface \
                         PATH_TO_VRML_FILE	   \
                         VRML_FILE_NAME		   \
                         PATH_TO_SPECIFICITIES_XML \
                         LINK_JOINT_RANK" << endl;
	  exit(-1);
	}
      else
	{
	  VRMLPath=value;
	  //VRMLPath+="/Controller/IOserver/robot/HRP2JRL/model/";
	  VRMLFileName="/HRP2JRLmain.wrl";
	  SpecificitiesFileName = value;
	  //SpecificitiesFileName +="/Controller/IOserver/robot/HRP2JRL/etc/";
	  SpecificitiesFileName += "/HRP2Specificities.xml";
	  LinkJointRank = value;
	  //LinkJointRank += "/Controller/IOserver/robot/HRP2JRL/etc/";
	  LinkJointRank += "/HRP2LinkJointRank.xml";
	  	      
	}
    }	
  else 
    {
      VRMLPath=argv[1];
      VRMLFileName=argv[2];
      SpecificitiesFileName = argv[3];
      LinkJointRank = argv[4];
    }
}



}; /* End of TestSuite namespace */
}; /* End of PatternGeneratorJRL namespace */

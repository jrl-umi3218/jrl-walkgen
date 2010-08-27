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
				      string &InitConfig,
				      CjrlHumanoidDynamicRobot * & aHDR,
				      CjrlHumanoidDynamicRobot * & aDebugHDR,
				      PatternGeneratorInterface * & aPGI)
{
  // Creating the humanoid robot.
  //CjrlHumanoidDynamicRobot * aHDR = 0, * aDebugHDR = 0;
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;

  aHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
  aDebugHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();

#if 0
#ifdef WITH_HRP2DYNAMICS
  delete aHDR;
  delete aDebugHDR;
  Chrp2OptHumanoidDynamicRobot *aHRP2HDR= new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
  aHDR = aHRP2HDR;
  aDebugHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
#endif
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
  unsigned int lNbDofs = aHDR->numberDof() ;
  cout << "Nb of DOFs: " <<  lNbDofs << endl;

  vector<CjrlJoint *> actuatedJoints = aHDR->getActuatedJoints();
  unsigned int lNbActuatedJoints = actuatedJoints.size();
  
  double * dInitPos = new double[lNbActuatedJoints];

  ifstream aif;
  aif.open((char *)InitConfig.c_str(),ifstream::in);
  if (aif.is_open())
    {
      for(unsigned int i=0;i<lNbActuatedJoints;i++)
	aif >> dInitPos[i];
    }
  aif.close();
  
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
  MAL_VECTOR_DIM(InitialPosition,double,lNbActuatedJoints);
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
  MAL_VECTOR_DIM(CurrentConfiguration,double,lNbDofs);
  MAL_VECTOR_DIM(CurrentVelocity,double,lNbDofs);
  MAL_VECTOR_DIM(CurrentAcceleration,double,lNbDofs);
  MAL_VECTOR_DIM(PreviousConfiguration,double,lNbDofs) ;
  MAL_VECTOR_DIM(PreviousVelocity,double,lNbDofs);
  MAL_VECTOR_DIM(PreviousAcceleration,double,lNbDofs);
  for(int i=0;i<6;i++)
    {
      PreviousConfiguration[i] = 
	PreviousVelocity[i] = 
	PreviousAcceleration[i] = 0.0;
    }

  for(unsigned int i=6;i<lNbDofs;i++)
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
		string &InitConfig,
		unsigned int &TestProfil)
{
  std::cout << "argc:" << argc << std::endl;
  if (argc!=6)
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
      VRMLPath=argv[1];
      VRMLFileName=argv[2];
      SpecificitiesFileName = argv[3];
      LinkJointRank = argv[4];
      InitConfig = argv[5];
    }
}



}; /* End of TestSuite namespace */
}; /* End of PatternGeneratorJRL namespace */

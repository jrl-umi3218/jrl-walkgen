/*
 * Copyright 2010, 
 *
 * Andrei  Herdt
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the 
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */

#ifdef UNIX
#include <sys/time.h>
#include <stdlib.h>
#endif /*UNIX*/

#ifdef WIN32
#include <Windows.h>
#include "portability/gettimeofday.hh"
#endif /*WIN32*/


#include <sstream>
#include <fstream>

#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl/dynamics/dynamicsfactory.hh>

#ifdef WITH_HRP2DYNAMICS
#include <hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h>
#endif

#include <jrl/walkgen/patterngeneratorinterface.hh>

#include "TestFootPrintPGInterfaceData.h"

using namespace std;

namespace PatternGeneratorJRL {
  namespace TestSuite {


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
      MAL_S3_VECTOR_TYPE(double)  lStartingZMPPosition;
      MAL_VECTOR_TYPE(double)  lStartingWaistPose;
      FootAbsolutePosition  InitLeftFootAbsPos;
      FootAbsolutePosition  InitRightFootAbsPos;

      aPGI.EvaluateStartingState(lStartingCOMPosition,
				 lStartingZMPPosition,
				 lStartingWaistPose,
				 InitLeftFootAbsPos,
				 InitRightFootAbsPos);
      
      cout << "Starting COM Position: " 
	   << lStartingCOMPosition.x[0] << " "
	   << lStartingCOMPosition.y[0] << " "
	   << lStartingCOMPosition.z[0] << endl;

      cout << "Starting Left Foot Pos: " 
	   << InitLeftFootAbsPos.x << " "
	   << InitLeftFootAbsPos.y << " "
	   << InitLeftFootAbsPos.z << " " 
	   << InitLeftFootAbsPos.theta<< " "
	   << InitLeftFootAbsPos.omega << " "
	   << InitLeftFootAbsPos.omega2 << " " 
	   << endl;

      cout << "Starting Right Foot Pos: " 
	   << InitRightFootAbsPos.x << " "
	   << InitRightFootAbsPos.y << " "
	   << InitRightFootAbsPos.z << " " 
	   << InitRightFootAbsPos.theta<< " "
	   << InitRightFootAbsPos.omega << " "
	   << InitRightFootAbsPos.omega2 << " " 
	   << endl;

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
      for(unsigned int i=0;i<lNbActuatedJoints;i++)
        dInitPos[i] = 0.0;

      ifstream aif;
      aif.open(InitConfig.c_str(),ifstream::in);
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

      delete [] dInitPos;
  
    }

    void getOptions(int argc,
		    char *argv[],
		    string &VRMLPath,
		    string &VRMLFileName,
		    string &SpecificitiesFileName,
		    string &LinkJointRank,
		    string &InitConfig,
		    unsigned int &) // TestProfil)
    {
      std::cout << "argc:" << argc << std::endl;
      if (argc!=6)
	{
	  cerr << " This program takes 5 arguments: " << endl;
	  cerr << "./TestFootPrintPGInterface \
                         PATH_TO_VRML_FILE	   \
                         VRML_FILE_NAME		   \
                         SPECIFICITIES_XML \
                         LINK_JOINT_RANK	\
                         INITIAL_CONFIGURATION" << endl;
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
  } /* End of TestSuite namespace */
} /* End of PatternGeneratorJRL namespace */

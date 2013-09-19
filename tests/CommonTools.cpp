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

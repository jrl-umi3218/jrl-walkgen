/*
 * Copyright 2010,
 *
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
/* \file Abstract Object test aim at testing various walking algorithms
 * Olivier Stasse
 */

#include <fstream>
#include "Debug.hh"
#include "TestObject.hh"

using namespace std;

#define NB_OF_FIELDS 26

#ifdef WIN32
double trunc (double x)
{
  return x < 0 ? ceil (x) : 0 < x ? floor (x) : x;
}
#endif /* WIN32 */

namespace PatternGeneratorJRL
{
  namespace TestSuite
  {

    double filterprecision(double adb)
    {
      if (fabs(adb)<1e-7)
	return 0.0;
	    
      double ladb2 = adb * 1e7;
      double lintadb2 = trunc(ladb2);
      return lintadb2/1e7;
    }


    TestObject::TestObject(int argc, char *argv[],
			   string &aTestName,
			   int lPGIInterface)
    {
      m_TestName = aTestName;
      m_PGIInterface = lPGIInterface;

      m_OuterLoopNbItMax = 1;

      /*! default debug output */
      m_DebugFGPI = true;
      m_DebugZMP2 = false;

      /*! Extract options and fill in members. */
      getOptions(argc,argv,
		 m_VRMLPath,
		 m_VRMLFileName,
		 m_SpecificitiesFileName,
		 m_LinkJointRank,
		 m_InitConfig,
		 m_TestProfile);
    }

    void TestObject::init()
    {


      // Instanciate and initialize.
      string RobotFileName = m_VRMLPath + m_VRMLFileName;

      bool fileExist = false;
      {
	std::ifstream file (RobotFileName.c_str ());
	fileExist = !file.fail ();
      }
      if (!fileExist)
	throw std::string ("failed to open robot model");

      CreateAndInitializeHumanoidRobot(RobotFileName,
				       m_SpecificitiesFileName,
				       m_LinkJointRank,
				       m_InitConfig,
				       m_HDR, m_DebugHDR, m_PGI);

      // Specify the walking mode: here the default one.
      istringstream strm2(":walkmode 0");
      m_PGI->ParseCmd(strm2);

      MAL_VECTOR_RESIZE(m_CurrentConfiguration, m_HDR->numberDof());
      MAL_VECTOR_RESIZE(m_CurrentVelocity, m_HDR->numberDof());
      MAL_VECTOR_RESIZE(m_CurrentAcceleration, m_HDR->numberDof());

      MAL_VECTOR_RESIZE(m_PreviousConfiguration, m_HDR->numberDof());
      MAL_VECTOR_RESIZE(m_PreviousVelocity, m_HDR->numberDof());
      MAL_VECTOR_RESIZE(m_PreviousAcceleration, m_HDR->numberDof());

    }

    TestObject::~TestObject()
    {

      if (m_HDR!=0)
	delete m_HDR;

      if (m_DebugHDR!=0)
	delete m_DebugHDR;

      if (m_PGI!=0)
	delete m_PGI;

    }

    void TestObject::SpecializedRobotConstructor(   CjrlHumanoidDynamicRobot *& aHDR,
					CjrlHumanoidDynamicRobot *& aDebugHDR)
    {
      aHDR = 0;
      aDebugHDR = 0;
    }

    void TestObject::CreateAndInitializeHumanoidRobot(string &RobotFileName,
						      string &SpecificitiesFileName,
						      string &LinkJointRank,
						      string &InitConfig,
						      CjrlHumanoidDynamicRobot * & aHDR,
						      CjrlHumanoidDynamicRobot * & aDebugHDR,
						      PatternGeneratorInterface * & aPGI)
    {
      // Creating the humanoid robot.
      SpecializedRobotConstructor(aHDR,aDebugHDR);
      
      if ((aHDR==0) || (aDebugHDR==0))
	{
	  if (aHDR!=0) delete aHDR;
	  if (aDebugHDR!=0) delete aDebugHDR;
	  
	  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
	  aHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
	  aDebugHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
	}


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
	      for(unsigned int k=0;k<lNbActuatedJoints;k++)
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

    void TestObject::prepareDebugFiles()
    {

      if (m_DebugZMP2)
	{
	  ofstream aofzmpmb2;
	  string aFileName = m_TestName;
	  aFileName += "ZMPMBSTAGE2.dat";
	  aofzmpmb2.open(aFileName.c_str(),ofstream::out);
	}


      if (m_DebugFGPI)
	{
	  ofstream aof;
	  string aFileName = m_TestName;
	  aFileName += "TestFGPI_description.dat";

	  aof.open(aFileName.c_str(),ofstream::out);
	  string Titles[NB_OF_FIELDS] =
	    { "Time",
	      "Com X",
	      "Com Y" ,
	      "Com Z" ,
	      "Com Yaw",
	      "Com dX" ,
	      "Com dY" ,
	      "Com dZ" ,
	      "ZMP X (waist ref.)" ,
	      "ZMP Y (waist ref.)" ,
	      "Left Foot X" ,
	      "Left Foot Y" ,
	      "Left Foot Z" ,
	      "Left Foot Theta" ,
	      "Left Foot Omega" ,
	      "Left Foot Omega2" ,
	      "Right Foot X" ,
	      "Right Foot Y" ,
	      "Right Foot Z" ,
	      "Right Foot Theta" ,
	      "Right Foot Omega" ,
	      "Right Foot Omega2" ,
	      "ZMP X (world ref.)" ,
	      "ZMP Y (world ref.)" ,
	      "Waist X (world ref.)" ,
	      "Waist Y (world ref.)" };
	  for(unsigned int i=0;i<NB_OF_FIELDS;i++)
	    aof << i+1 << ". " <<Titles[i] <<std::endl;

	  aof.close();

	  aFileName = m_TestName;
	  aFileName += "TestFGPI.dat";
	  aof.open(aFileName.c_str(),ofstream::out);
	  aof.close();
	}
    }

    void TestObject::fillInDebugFiles( )
    {
      if (m_DebugFGPI)
	{
	  ofstream aof;
	  string aFileName;
	  aFileName = m_TestName;
	  aFileName += "TestFGPI.dat";
	  aof.open(aFileName.c_str(),ofstream::app);
	  aof.precision(8);
	  aof.setf(ios::scientific, ios::floatfield);
	  aof << filterprecision(m_OneStep.NbOfIt*0.005 ) << " "
	      << filterprecision(m_OneStep.finalCOMPosition.x[0] ) << " "
	      << filterprecision(m_OneStep.finalCOMPosition.y[0] ) << " "
	      << filterprecision(m_OneStep.finalCOMPosition.z[0] ) << " "
	      << filterprecision(m_OneStep.finalCOMPosition.yaw ) << " "
	      << filterprecision(m_OneStep.finalCOMPosition.x[1] ) << " "
	      << filterprecision(m_OneStep.finalCOMPosition.y[1] ) << " "
	      << filterprecision(m_OneStep.finalCOMPosition.z[1] ) << " "
	      << filterprecision(m_OneStep.ZMPTarget(0) ) << " "
	      << filterprecision(m_OneStep.ZMPTarget(1) ) << " "
	      << filterprecision(m_OneStep.LeftFootPosition.x  ) << " "
	      << filterprecision(m_OneStep.LeftFootPosition.y  ) << " "
	      << filterprecision(m_OneStep.LeftFootPosition.z  ) << " "
	      << filterprecision(m_OneStep.LeftFootPosition.theta  ) << " "
	      << filterprecision(m_OneStep.LeftFootPosition.omega  ) << " "
	      << filterprecision(m_OneStep.LeftFootPosition.omega2  ) << " "
	      << filterprecision(m_OneStep.RightFootPosition.x ) << " "
	      << filterprecision(m_OneStep.RightFootPosition.y ) << " "
	      << filterprecision(m_OneStep.RightFootPosition.z ) << " "
	      << filterprecision(m_OneStep.RightFootPosition.theta ) << " "
	      << filterprecision(m_OneStep.RightFootPosition.omega  ) << " "
	      << filterprecision(m_OneStep.RightFootPosition.omega2  ) << " "
	      << filterprecision(m_OneStep.ZMPTarget(0)*cos(m_CurrentConfiguration(5)) -
	    m_OneStep.ZMPTarget(1)*sin(m_CurrentConfiguration(5))
	    +m_CurrentConfiguration(0) ) << " "
	      << filterprecision(m_OneStep.ZMPTarget(0)*sin(m_CurrentConfiguration(5)) +
	    m_OneStep.ZMPTarget(1)*cos(m_CurrentConfiguration(5))
	    +m_CurrentConfiguration(1) ) << " "
	      << filterprecision(m_CurrentConfiguration(0) ) << " "
	      << filterprecision(m_CurrentConfiguration(1) ) << " "
	      << endl;
	  aof.close();
	}
    }


    bool TestObject::compareDebugFiles( )
    {
      bool SameFile= false;
      if (m_DebugFGPI)
	{
	  SameFile = true;
	  ifstream alif;
	  string aFileName;
	  aFileName = m_TestName;
	  aFileName += "TestFGPI.dat";
	  alif.open(aFileName.c_str(),ifstream::in);

	  ifstream arif;
	  aFileName = m_TestName;
	  aFileName += "TestFGPI.datref";
	  arif.open(aFileName.c_str(),ifstream::in);
	 
	  ofstream areportof;
	  aFileName = m_TestName;
	  aFileName += "TestFGPI_report.dat";
	  areportof.open(aFileName.c_str(),ofstream::out);
	    
	  // Time
	  double LocalInput[NB_OF_FIELDS], ReferenceInput[NB_OF_FIELDS];
	  bool finalreport = true;
	  unsigned long int nblines = 0;
	  bool endofinspection=false;

	  while ((!alif.eof()) ||
		 (!arif.eof()) ||
		 (endofinspection))
	    {
	      for (unsigned int i=0;i<NB_OF_FIELDS;i++)
		{
		  alif >> LocalInput[i];
		  if (alif.eof())
		    {
		      endofinspection =true;
		      break;
		    }
		}
	      if (endofinspection)
		break;

	      for (unsigned int i=0;i<NB_OF_FIELDS;i++)
		{
		  arif >> ReferenceInput[i];
		  if (alif.eof())
		    {
		      endofinspection =true;
		      break;
		    }
		}
	      if (endofinspection)
		break;

	      
	      for (unsigned int i=0;i<NB_OF_FIELDS;i++)
		{
		  if  (fabs(LocalInput[i]-
			    ReferenceInput[i])>=1e-6)
		    {
		      finalreport = false;
		      areportof << "l: " << nblines 
				<< " col:" << i 
				<< " ref: " << ReferenceInput[i] 
				<< " now: " << LocalInput[i] 
				<<std::endl;
		    }
		}
	      nblines++;
	    }

	  alif.close();
	  arif.close();
	  areportof.close();
	  return finalreport;
	}
      return SameFile;
    }

    bool TestObject::doTest(ostream &os)
    {

      // Set time reference.
      m_clock.startingDate();

      /*! Open and reset appropriatly the debug files. */
      prepareDebugFiles();

      for (unsigned int lNbIt=0;lNbIt<m_OuterLoopNbItMax;lNbIt++)
	{
	  os << "<===============================================================>"<<endl;
	  os << "Iteration nb: " << lNbIt << endl;

	  m_clock.startPlanning();

	  /*! According to test profile initialize the current profile. */
	  chooseTestProfile();

	  m_clock.endPlanning();

	  if (m_DebugHDR!=0)
	    {
	      m_DebugHDR->currentConfiguration(m_PreviousConfiguration);
	      m_DebugHDR->currentVelocity(m_PreviousVelocity);
	      m_DebugHDR->currentAcceleration(m_PreviousAcceleration);
	      m_DebugHDR->computeForwardKinematics();
	    }

	  bool ok = true;
	  while(ok)
	    {
	      m_clock.startOneIteration();

	      if (m_PGIInterface==0)
		{
		  ok = m_PGI->RunOneStepOfTheControlLoop(m_CurrentConfiguration,
							 m_CurrentVelocity,
							 m_CurrentAcceleration,
							 m_OneStep.ZMPTarget,
							 m_OneStep.finalCOMPosition,
							 m_OneStep.LeftFootPosition,
							 m_OneStep.RightFootPosition);
		}
	      else if (m_PGIInterface==1)
		{
		  ok = m_PGI->RunOneStepOfTheControlLoop(m_CurrentConfiguration,
							 m_CurrentVelocity,
							 m_CurrentAcceleration,
							 m_OneStep.ZMPTarget);
		}

	      m_OneStep.NbOfIt++;

	      m_clock.stopOneIteration();

	      m_PreviousConfiguration = m_CurrentConfiguration;
	      m_PreviousVelocity = m_CurrentVelocity;
	      m_PreviousAcceleration = m_CurrentAcceleration;

	      /*! Call the reimplemented method to generate events. */
	      if (ok)
		{
		  m_clock.startModification();
		  generateEvent();
		  m_clock.stopModification();

		  m_clock.fillInStatistics();

		  /*! Fill the debug files with appropriate information. */
		  fillInDebugFiles();
		}
	      else 
		{
		  cerr << "Nothing to dump after " << m_OneStep.NbOfIt << endl;
		}

	    }

	  os << endl << "End of iteration " << lNbIt << endl;
	  os << "<===============================================================>"<<endl;
	}

      string lProfileOutput= m_TestName;
      lProfileOutput +="TimeProfile.dat";
      m_clock.writeBuffer(lProfileOutput);
      m_clock.displayStatistics(os,m_OneStep);

      // Compare debugging files
      return compareDebugFiles();
    }

  }
}

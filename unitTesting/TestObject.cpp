/* \file Abstract Object test aim at testing various walking algorithms
 * Olivier Stasse
 * (c) 2010, JRL, UMI 3218/CRT, JRL, CNRS/AIST
 * 
 */

#include "TestObject.h"

using namespace std;

namespace PatternGeneratorJRL
{
  namespace TestSuite
  {
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

      /*! \brief Path to the VRML. */
      string VRMLPath;
      /*! \brief Name of the VRML. */
      string VRMLFileName;
      /*! \brief File describing the specificities of the robot. */
      string SpecificitiesFileName;
      /*! \brief File describing the relationship between the Joints
	and their rank in the robot's state vector */
      string LinkJointRank;

      string InitConfig;

      /*! Extract options and fill in members. */
      getOptions(argc,argv,
		 VRMLPath, 
		 VRMLFileName,
		 SpecificitiesFileName, 
		 LinkJointRank,
		 InitConfig,
		 m_TestProfile);

      // Instanciate and initialize.
      string RobotFileName = VRMLPath + VRMLFileName;
      CreateAndInitializeHumanoidRobot(RobotFileName,
				       SpecificitiesFileName,
				       LinkJointRank,
				       InitConfig,
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
	  string Titles[26] =
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
	  for(unsigned int i=0;i<26;i++)
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

	  aof << m_OneStep.NbOfIt*0.005 << " " 
	      << m_OneStep.finalCOMPosition.x[0] << " "
	      << m_OneStep.finalCOMPosition.y[0] << " " 
	      << m_OneStep.finalCOMPosition.z[0] << " "
	      << m_OneStep.finalCOMPosition.yaw << " "
	      << m_OneStep.finalCOMPosition.x[1] << " "
	      << m_OneStep.finalCOMPosition.y[1] << " " 
	      << m_OneStep.finalCOMPosition.z[1] << " "
	      << m_OneStep.ZMPTarget(0) << " " 
	      << m_OneStep.ZMPTarget(1) << " " 
	      << m_OneStep.LeftFootPosition.x  << " " 
	      << m_OneStep.LeftFootPosition.y  << " " 
	      << m_OneStep.LeftFootPosition.z  << " " 
	      << m_OneStep.LeftFootPosition.theta  << " "  
	      << m_OneStep.LeftFootPosition.omega  << " " 
	      << m_OneStep.LeftFootPosition.omega2  << " "
	      << m_OneStep.RightFootPosition.x << " " 
	      << m_OneStep.RightFootPosition.y << " " 
	      << m_OneStep.RightFootPosition.z << " " 
	      << m_OneStep.RightFootPosition.theta << " " 
	      << m_OneStep.RightFootPosition.omega  << " " 
	      << m_OneStep.RightFootPosition.omega2  << " "
	      << m_OneStep.ZMPTarget(0)*cos(m_CurrentConfiguration(5)) - 
	    m_OneStep.ZMPTarget(1)*sin(m_CurrentConfiguration(5))
	    +m_CurrentConfiguration(0) << " " 
	      << m_OneStep.ZMPTarget(0)*sin(m_CurrentConfiguration(5)) + 
	    m_OneStep.ZMPTarget(1)*cos(m_CurrentConfiguration(5))
	    +m_CurrentConfiguration(1) << " "
	      << m_CurrentConfiguration(0) << " " 
	      << m_CurrentConfiguration(1) << " " 
	      << m_clock.getStartOneIteration()
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
	  
	  // Time
	  double LocalInput[70], ReferenceInput[70];

	  while ((!alif.eof()) ||
		 (!arif.eof()))
	    {
	      for (unsigned int i=0;i<70;i++)
		alif >> LocalInput[i];

	      for (unsigned int i=0;i<70;i++)
		arif >> ReferenceInput[i];

	      for (unsigned int i=0;i<70;i++)
		{
		  if  (LocalInput[i]!=
		       ReferenceInput[i])
		    return false;
		}
	    }

	  alif.close();
	  arif.close();
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

  };
};

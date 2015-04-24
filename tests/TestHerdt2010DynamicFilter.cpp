/*
 * Copyright 2010,
 *
 * Andrei Herdt
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
/* \file This file tests A. Herdt's walking algorithm for
 * automatic foot placement giving an instantaneous CoM velocity reference.
 */
#include "Debug.hh"
#include "CommonTools.hh"
#include "TestObject.hh"
#include <jrl/walkgen/pgtypes.hh>
#include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>
#include <MotionGeneration/ComAndFootRealizationByGeometry.hh>

#include <metapod/models/hrp2_14/hrp2_14.hh>
#ifndef METAPOD_INCLUDES
#define METAPOD_INCLUDES
// metapod includes
#include <metapod/tools/print.hh>
#include <metapod/tools/initconf.hh>
#include <metapod/algos/rnea.hh>
#include <Eigen/StdVector>
#endif

#ifndef METAPOD_TYPEDEF2
#define METAPOD_TYPEDEF2
typedef double LocalFloatType2;
typedef metapod::Spatial::ForceTpl<LocalFloatType2> Force2;
typedef metapod::hrp2_14<LocalFloatType2> Robot_Model2;
typedef metapod::Nodes< Robot_Model2, Robot_Model2::BODY >::type Node2;
#endif


using namespace::PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;
using namespace std;

enum Profiles_t {
  PROFIL_HERDT_ONLINE_WALKING,                 // 1
  PROFIL_HERDT_EMERGENCY_STOP                  // 2
};

class TestHerdt2010: public TestObject
{

private:
  ComAndFootRealization * ComAndFootRealization_;
  SimplePluginManager * SPM ;
  int iteration ;

public:
  TestHerdt2010(int argc, char *argv[], string &aString, int TestProfile):
      TestObject(argc,argv,aString)
  {
    m_TestProfile = TestProfile;
    SPM = 0 ;
    ComAndFootRealization_ = 0 ;
    iteration = 0 ;
  }

  ~TestHerdt2010()
  {
    if ( ComAndFootRealization_ != 0 )
    {
      delete ComAndFootRealization_ ;
      ComAndFootRealization_ = 0 ;
    }
    if ( SPM != 0 )
    {
      delete SPM ;
      SPM = 0 ;
    }
  }

  typedef void (TestHerdt2010::* localeventHandler_t)(PatternGeneratorInterface &);

  struct localEvent
  {
    unsigned time;
    localeventHandler_t Handler ;
  };

  bool doTest(ostream &os)
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

  void init()
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

    SPM = new SimplePluginManager();

    ComAndFootRealization_ = new ComAndFootRealizationByGeometry( (PatternGeneratorInterfacePrivate*) SPM );
    ComAndFootRealization_->setHumanoidDynamicRobot(m_HDR);
    ComAndFootRealization_->SetStepStackHandler(new StepStackHandler(SPM));
    ComAndFootRealization_->SetHeightOfTheCoM(0.814);
    ComAndFootRealization_->setSamplingPeriod(0.005);
    ComAndFootRealization_->Initialization();

    initIK();

    {
      istringstream strm2(":setfeetconstraint XY 0.09 0.04");
      m_PGI->ParseCmd(strm2);
    }

  }

protected:

  void initIK()
  {
    MAL_VECTOR_DIM(BodyAngles,double,MAL_VECTOR_SIZE(InitialPosition));
    MAL_VECTOR_DIM(waist,double,6);
    for (int i = 0 ; i < 6 ; ++i )
    {
      waist(i) = 0;
    }
    for (unsigned int i = 0 ; i < (m_HDR->numberDof()-6) ; ++i )
    {
      BodyAngles(i) = InitialPosition(i);
    }
    MAL_S3_VECTOR(lStartingCOMState,double);

    lStartingCOMState(0) = m_OneStep.finalCOMPosition.x[0] ;
    lStartingCOMState(1) = m_OneStep.finalCOMPosition.y[0] ;
    lStartingCOMState(2) = m_OneStep.finalCOMPosition.z[0] ;
    ComAndFootRealization_->SetHeightOfTheCoM(0.814);
    ComAndFootRealization_->setSamplingPeriod(0.005);
    ComAndFootRealization_->Initialization();

    ComAndFootRealization_->InitializationCoM(BodyAngles,lStartingCOMState,
                                              waist,
                                              m_OneStep.LeftFootPosition, m_OneStep.RightFootPosition);
    ComAndFootRealization_->Initialization();
  }

  void fillInDebugFiles()
  {
    TestObject::fillInDebugFiles();
    if (m_DebugFGPI)
      {
        static int inc = 0 ;
        ofstream aof;
        string aFileName;
        aFileName = m_TestName;
        aFileName += "TestFGPIFull.dat";
        if (inc==0)
          {
            aof.open(aFileName.c_str(),ofstream::out);
          }
        inc = 1;
        aof.open(aFileName.c_str(),ofstream::app);
        aof.precision(8);
        aof.setf(ios::scientific, ios::floatfield);
        aof << filterprecision(m_OneStep.NbOfIt*0.005 ) << " "                            // 1
            << filterprecision(m_OneStep.finalCOMPosition.x[0] ) << " "                   // 2
            << filterprecision(m_OneStep.finalCOMPosition.y[0] ) << " "                   // 3
            << filterprecision(m_OneStep.finalCOMPosition.z[0] ) << " "                   // 4
            << filterprecision(m_OneStep.finalCOMPosition.yaw[0] ) << " "                 // 5
            << filterprecision(m_OneStep.finalCOMPosition.x[1] ) << " "                   // 6
            << filterprecision(m_OneStep.finalCOMPosition.y[1] ) << " "                   // 7
            << filterprecision(m_OneStep.finalCOMPosition.z[1] ) << " "                   // 8
            << filterprecision(m_OneStep.finalCOMPosition.yaw[1] ) << " "                 // 9
            << filterprecision(m_OneStep.finalCOMPosition.x[2] ) << " "                   // 10
            << filterprecision(m_OneStep.finalCOMPosition.y[2] ) << " "                   // 11
            << filterprecision(m_OneStep.finalCOMPosition.z[2] ) << " "                   // 12
            << filterprecision(m_OneStep.finalCOMPosition.yaw[2] ) << " "                 // 13
            << filterprecision(m_OneStep.ZMPTarget(0) ) << " "                            // 14
            << filterprecision(m_OneStep.ZMPTarget(1) ) << " "                            // 15
            << filterprecision(m_OneStep.ZMPTarget(2) ) << " "                            // 16
            << filterprecision(m_OneStep.LeftFootPosition.x  ) << " "                     // 17
            << filterprecision(m_OneStep.LeftFootPosition.y  ) << " "                     // 18
            << filterprecision(m_OneStep.LeftFootPosition.z  ) << " "                     // 19
            << filterprecision(m_OneStep.LeftFootPosition.dx  ) << " "                    // 20
            << filterprecision(m_OneStep.LeftFootPosition.dy  ) << " "                    // 21
            << filterprecision(m_OneStep.LeftFootPosition.dz  ) << " "                    // 22
            << filterprecision(m_OneStep.LeftFootPosition.ddx  ) << " "                   // 23
            << filterprecision(m_OneStep.LeftFootPosition.ddy  ) << " "                   // 24
            << filterprecision(m_OneStep.LeftFootPosition.ddz  ) << " "                   // 25
            << filterprecision(m_OneStep.LeftFootPosition.theta ) << " "                  // 26
            << filterprecision(m_OneStep.LeftFootPosition.dtheta ) << " "                 // 27
            << filterprecision(m_OneStep.LeftFootPosition.ddtheta ) << " "                // 28
            << filterprecision(m_OneStep.LeftFootPosition.omega  ) << " "                 // 29
            << filterprecision(m_OneStep.LeftFootPosition.omega2  ) << " "                // 30
            << filterprecision(m_OneStep.RightFootPosition.x ) << " "                     // 31
            << filterprecision(m_OneStep.RightFootPosition.y ) << " "                     // 32
            << filterprecision(m_OneStep.RightFootPosition.z ) << " "                     // 33
            << filterprecision(m_OneStep.RightFootPosition.dx ) << " "                    // 34
            << filterprecision(m_OneStep.RightFootPosition.dy ) << " "                    // 35
            << filterprecision(m_OneStep.RightFootPosition.dz ) << " "                    // 36
            << filterprecision(m_OneStep.RightFootPosition.ddx ) << " "                   // 37
            << filterprecision(m_OneStep.RightFootPosition.ddy ) << " "                   // 38
            << filterprecision(m_OneStep.RightFootPosition.ddz ) << " "                   // 39
            << filterprecision(m_OneStep.RightFootPosition.theta ) << " "                 // 40
            << filterprecision(m_OneStep.RightFootPosition.dtheta ) << " "                // 41
            << filterprecision(m_OneStep.RightFootPosition.ddtheta ) << " "               // 42
            << filterprecision(m_OneStep.RightFootPosition.omega  ) << " "                // 43
            << filterprecision(m_OneStep.RightFootPosition.omega2  ) << " "              ;// 44
        aof << endl;
        aof.close();
    }

    /// \brief calculate, from the CoM of computed by the preview control,
    ///    the corresponding articular position, velocity and acceleration
    /// ------------------------------------------------------------------
    MAL_VECTOR_DIM(aCOMState,double,6);
    MAL_VECTOR_DIM(aCOMSpeed,double,6);
    MAL_VECTOR_DIM(aCOMAcc,double,6);
    MAL_VECTOR_DIM(aLeftFootPosition,double,5);
    MAL_VECTOR_DIM(aRightFootPosition,double,5);

    aCOMState(0) = m_OneStep.finalCOMPosition.x[0];      aCOMSpeed(0) = m_OneStep.finalCOMPosition.x[1];      aCOMAcc(0) = m_OneStep.finalCOMPosition.x[2];
    aCOMState(1) = m_OneStep.finalCOMPosition.y[0];      aCOMSpeed(1) = m_OneStep.finalCOMPosition.y[1];      aCOMAcc(1) = m_OneStep.finalCOMPosition.y[2];
    aCOMState(2) = m_OneStep.finalCOMPosition.z[0];      aCOMSpeed(2) = m_OneStep.finalCOMPosition.z[1];      aCOMAcc(2) = m_OneStep.finalCOMPosition.z[2];
    aCOMState(3) = m_OneStep.finalCOMPosition.roll[0]*180/M_PI;   aCOMSpeed(3) = m_OneStep.finalCOMPosition.roll[1]*180/M_PI;   aCOMAcc(3) = m_OneStep.finalCOMPosition.roll[2]*180/M_PI;
    aCOMState(4) = m_OneStep.finalCOMPosition.pitch[0]*180/M_PI;  aCOMSpeed(4) = m_OneStep.finalCOMPosition.pitch[1]*180/M_PI;  aCOMAcc(4) = m_OneStep.finalCOMPosition.pitch[2]*180/M_PI;
    aCOMState(5) = m_OneStep.finalCOMPosition.yaw[0]*180/M_PI;    aCOMSpeed(5) = m_OneStep.finalCOMPosition.yaw[1]*180/M_PI;    aCOMAcc(5) = m_OneStep.finalCOMPosition.yaw[2]*180/M_PI;

    aLeftFootPosition(0) = m_OneStep.LeftFootPosition.x;      aRightFootPosition(0) = m_OneStep.RightFootPosition.x;
    aLeftFootPosition(1) = m_OneStep.LeftFootPosition.y;      aRightFootPosition(1) = m_OneStep.RightFootPosition.y;
    aLeftFootPosition(2) = m_OneStep.LeftFootPosition.z;      aRightFootPosition(2) = m_OneStep.RightFootPosition.z;
    aLeftFootPosition(3) = m_OneStep.LeftFootPosition.theta;  aRightFootPosition(3) = m_OneStep.RightFootPosition.theta;
    aLeftFootPosition(4) = m_OneStep.LeftFootPosition.omega;  aRightFootPosition(4) = m_OneStep.RightFootPosition.omega;
    ComAndFootRealization_->setSamplingPeriod(0.005);
    ComAndFootRealization_->ComputePostureForGivenCoMAndFeetPosture(aCOMState, aCOMSpeed, aCOMAcc,
                                                                    aLeftFootPosition,
                                                                    aRightFootPosition,
                                                                    m_CurrentConfiguration,
                                                                    m_CurrentVelocity,
                                                                    m_CurrentAcceleration,
                                                                    20,
                                                                    1);

    m_CurrentConfiguration(28)= 0.174532925 ;     // RARM_JOINT6
    m_CurrentConfiguration(35)= 0.174532925 ;     // LARM_JOINT6

    /// \brief Create file .hip .pos .zmp
    /// --------------------
    ofstream aof ;
    string root = "/opt/grx/HRP2LAAS/etc/mnaveau/" ;
    string aFileName = root + m_TestName + ".pos" ;
    if ( iteration == 0 )
    {
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    aof << filterprecision( iteration * 0.005 ) << " "  ; // 1
    for(unsigned int i = 6 ; i < m_CurrentConfiguration.size() ; i++){
      aof << filterprecision( m_CurrentConfiguration(i) ) << " "  ; // 2
    }
    for(unsigned int i = 0 ; i < 9 ; i++){
      aof << 0.0 << " "  ;
    }
    aof << 0.0  << endl ;
    aof.close();

    aFileName = root + m_TestName + ".hip" ;
    if ( iteration == 0 ){
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
      aof << filterprecision( iteration * 0.005 ) << " "  ;                           // 1
      aof << filterprecision( m_OneStep.finalCOMPosition.roll[0]) << " "  ;  // 2
      aof << filterprecision( m_OneStep.finalCOMPosition.pitch[0]) << " "  ;// 3
      aof << filterprecision( m_OneStep.finalCOMPosition.yaw[0]) ;          // 4
      aof << endl ;
    aof.close();

    aFileName = root + m_TestName + ".waist" ;
    if ( iteration == 0 ){
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
      aof << filterprecision( iteration * 0.005 ) << " "  ;                           // 1
      aof << filterprecision( m_OneStep.finalCOMPosition.roll[0]) << " "  ;  // 2
      aof << filterprecision( m_OneStep.finalCOMPosition.pitch[0]) << " "  ;// 3
      aof << filterprecision( m_OneStep.finalCOMPosition.yaw[0]) ;          // 4
      aof << endl ;
    aof.close();

    aFileName = root + m_TestName + ".zmp" ;
    if ( iteration == 0 ){
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
    FootAbsolutePosition aSupportState;
    if (m_OneStep.LeftFootPosition.stepType < 0 )
      aSupportState = m_OneStep.LeftFootPosition ;
    else
      aSupportState = m_OneStep.RightFootPosition ;

    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
      aof << filterprecision( iteration * 0.005 ) << " "  ;                                 // 1
      aof << filterprecision( m_OneStep.ZMPTarget(0) - m_CurrentConfiguration(0)) << " "  ; // 2
      aof << filterprecision( m_OneStep.ZMPTarget(1) - m_CurrentConfiguration(1) ) << " "  ;// 3
      aof << filterprecision( aSupportState.z  - m_CurrentConfiguration(2))  ;              // 4
      aof << endl ;
    aof.close();

    iteration++;
  }

  void SpecializedRobotConstructor(   CjrlHumanoidDynamicRobot *& aHDR, CjrlHumanoidDynamicRobot *& aDebugHDR )
  {
    dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
    Chrp2OptHumanoidDynamicRobot *aHRP2HDR = new Chrp2OptHumanoidDynamicRobot( &aRobotDynamicsObjectConstructor );
    aHDR = aHRP2HDR;
    aDebugHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
  }

  void startOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);

    {
      istringstream strm2(":SetAlgoForZmpTrajectory Herdt");
      aPGI.ParseCmd(strm2);

    }
    {
      istringstream strm2(":singlesupporttime 0.7");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":doublesupporttime 0.1");
      aPGI.ParseCmd(strm2);
    }
    {
      //istringstream strm2(":HerdtOnline");
      istringstream strm2(":HerdtOnline 0.2 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":numberstepsbeforestop 2");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":setfeetconstraint XY 0.09 0.04");
      m_PGI->ParseCmd(strm2);
    }
  }

  void startEmergencyStop(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);

    {
      istringstream strm2(":SetAlgoForZmpTrajectory Herdt");
      aPGI.ParseCmd(strm2);

    }
    {
      istringstream strm2(":singlesupporttime 0.7");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":doublesupporttime 0.1");
      aPGI.ParseCmd(strm2);
    }
    {
      //istringstream strm2(":HerdtOnline");
      istringstream strm2(":HerdtOnline 0.2 0.0 0.2");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":numberstepsbeforestop 2");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":setfeetconstraint XY 0.09 0.04");
      m_PGI->ParseCmd(strm2);
    }
  }

  void startTurningLeft(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.2 0.0 6.0832");
      aPGI.ParseCmd(strm2);
    }
  }

  void startTurningRight(PatternGeneratorInterface &aPGI)
  {
    {
      //istringstream strm2(":setVelReference  0.2 0.0 -0.2");
      istringstream strm2(":setVelReference  0.2 0.0 -6.0832");
      aPGI.ParseCmd(strm2);
    }
  }

  void startTurningRight2(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.2 0.0 -0.2");
      aPGI.ParseCmd(strm2);
    }
  }

  void startTurningLeft2(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 0.0 0.4");
      aPGI.ParseCmd(strm2);
    }
  }

  void startTurningLeftOnSpot(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 0.0 10.0");
      aPGI.ParseCmd(strm2);
    }
  }

  void startTurningRightOnSpot(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 0.0 -10.");
      aPGI.ParseCmd(strm2);
    }
  }

  void stop(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference 0.0 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
  }
  void walkForward1m_s(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.1 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
  }
  void walkForward2m_s(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.2 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
  }
  void walkForward3m_s(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.3 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
  }
  void walkSidewards1m_s(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 -0.1 0.0");
      aPGI.ParseCmd(strm2);
    }
  }
  void walkSidewards2m_s(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 -0.2 0.0");
      aPGI.ParseCmd(strm2);
    }
  }
  void walkSidewards3m_s(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 -0.3 0.0");
      aPGI.ParseCmd(strm2);
    }
  }
  void startWalkInDiagonal1m_s(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.1 0.1 0.0");
      aPGI.ParseCmd(strm2);
    }
  }
  void startWalkInDiagonal2m_s(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.2 0.2 0.0");
      aPGI.ParseCmd(strm2);
    }
  }

  void startWalkInDiagonal3m_s(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.3 0.3 0.0");
      aPGI.ParseCmd(strm2);
    }
  }

  void stopOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 0.0 0.0");
      aPGI.ParseCmd(strm2);
      istringstream strm3(":stoppg");
      aPGI.ParseCmd(strm3);
    }
  }

  void chooseTestProfile()
  {

    switch(m_TestProfile)
    {

    case PROFIL_HERDT_ONLINE_WALKING:
      startOnLineWalking(*m_PGI);
      break;
    case PROFIL_HERDT_EMERGENCY_STOP:
      startEmergencyStop(*m_PGI);
      break;
    default:
      throw("No correct test profile");
      break;
    }
  }


  void generateEventOnLineWalking()
  {
    struct localEvent
    {
      unsigned time;
      localeventHandler_t Handler ;
    };
#define localNbOfEvents 12
    struct localEvent events [localNbOfEvents] =
    { { 5*200,&TestHerdt2010::walkForward3m_s},
      {10*200,&TestHerdt2010::walkForward2m_s},
      {15*200,&TestHerdt2010::walkForward1m_s},
      {20*200,&TestHerdt2010::walkSidewards1m_s},
      {25*200,&TestHerdt2010::walkSidewards2m_s},
      {30*200,&TestHerdt2010::walkSidewards3m_s},
      {35*200,&TestHerdt2010::startTurningRightOnSpot},
      {40*200,&TestHerdt2010::stop},
      {45*200,&TestHerdt2010::stopOnLineWalking}};
    // Test when triggering event.
    for(unsigned int i=0;i<localNbOfEvents;i++)
      {
        if ( m_OneStep.NbOfIt==events[i].time)
          {
            ODEBUG3("********* GENERATE EVENT OLW ***********");
            (this->*(events[i].Handler))(*m_PGI);
          }
      }
  }
  void generateEventEmergencyStop()
  {
#define localNbOfEventsEMS 4
    struct localEvent events [localNbOfEventsEMS] =
    { {5*200,&TestHerdt2010::startTurningLeft2},
    {10*200,&TestHerdt2010::startTurningRight2},
    {15.2*200,&TestHerdt2010::stop},
    {20.8*200,&TestHerdt2010::stopOnLineWalking}};
    // Test when triggering event.
    for(unsigned int i=0;i<localNbOfEventsEMS;i++)
      {
        if ( m_OneStep.NbOfIt==events[i].time)
          {
            ODEBUG3("********* GENERATE EVENT EMS ***********");
            (this->*(events[i].Handler))(*m_PGI);
          }
      }
  }

  void generateEvent()
  {
    switch(m_TestProfile){
    case PROFIL_HERDT_ONLINE_WALKING:
      generateEventOnLineWalking();
      break;
    case PROFIL_HERDT_EMERGENCY_STOP:
      generateEventEmergencyStop();
      break;
    default:
      break;
    }
  }

};

int PerformTests(int argc, char *argv[])
{
#define NB_PROFILES 2
  std::string TestNames[NB_PROFILES] = { "TestHerdt2010DynamicFilter",
                                         "TestHerdt2010EmergencyStop"};
  int TestProfiles[NB_PROFILES] = { PROFIL_HERDT_ONLINE_WALKING,
                                    PROFIL_HERDT_EMERGENCY_STOP};

  for (unsigned int i=0;i<NB_PROFILES;i++){
    TestHerdt2010 aTH2010(argc,argv,TestNames[i],TestProfiles[i]);
    aTH2010.init();
    try{
      if (!aTH2010.doTest(std::cout)){
        cout << "Failed test " << i << endl;
        return -1;
      }
      else
        cout << "Passed test " << i << endl;
    }
    catch (const char * astr){
      cerr << "Failed on following error " << astr << std::endl;
      return -1; }
  }
  return 0;
}

int main(int argc, char *argv[])
{
  try
  {
    int ret = PerformTests(argc,argv);
    return ret ;
  }
  catch (const std::string& msg)
  {
    std::cerr << msg << std::endl;
  }
  return 1;
}



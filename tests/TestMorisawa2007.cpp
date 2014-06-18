/*
 * Copyright 2010,
 *
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of jrl-walkgen.
 * jrl-walkgen is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * jrl-walkgen is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with jrl-walkgen.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/* \file This file tests M. Morisawa's walking algorithm for
 * real-time CoM and ZMP trajectory generation
 */

#include "CommonTools.hh"
#include "TestObject.hh"

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
  PROFIL_ANALYTICAL_ONLINE_WALKING,                 // 1
  PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING          //  2
};

#define NBOFPREDEFONLINEFOOTSTEPS 11


double OnLineFootSteps[NBOFPREDEFONLINEFOOTSTEPS][3]={
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0}
};

class TestMorisawa2007: public TestObject
{

private:
  bool m_TestChangeFoot;
  unsigned long int m_NbStepsModified;
  // New time between two steps.
  double m_deltatime;

  // buffer to save all the zmps positions
  vector< vector<double> > errZMP_ ;
  Robot_Model2 robot_ ;
  ComAndFootRealization * ComAndFootRealization_;
  SimplePluginManager * SPM ;
  double dInitX, dInitY;
  int iteration,iteration_zmp ;
  bool once ;

public:
  TestMorisawa2007(int argc, char*argv[], string &aString, int TestProfile):
    TestObject(argc, argv, aString)
  {
    m_TestProfile = TestProfile;
    m_TestChangeFoot = true;
    m_NbStepsModified = 0;
    m_deltatime = 0;

    ComAndFootRealization_ = 0 ;
    dInitX = 0 ;
    dInitY = 0 ;
    iteration_zmp = 0 ;
    iteration = 0 ;
    once = true ;
  };

  ~TestMorisawa2007()
  {
    delete ComAndFootRealization_ ;
    delete SPM ;
  }

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
          ok = m_PGI->RunOneStepOfTheControlLoop( m_CurrentConfiguration,
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
          ComparingZMPs();
          ComputeAndDisplayAverageError(false);
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
    ComputeAndDisplayAverageError(true);
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
  }

protected:

  double filterprecision(double adb)
  {
    if (fabs(adb)<1e-7)
    return 0.0;

    double ladb2 = adb * 1e7;
    double lintadb2 = trunc(ladb2);
    return lintadb2/1e7;
  }

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

  void fillInDebugFiles( )
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
	  aof << filterprecision(m_OneStep.NbOfIt*0.005 ) << " "                            // 1
	      << filterprecision(m_OneStep.finalCOMPosition.x[0] ) << " "                   // 2
	      << filterprecision(m_OneStep.finalCOMPosition.y[0] ) << " "                   // 3
	      << filterprecision(m_OneStep.finalCOMPosition.z[0] ) << " "                   // 4
              << filterprecision(m_OneStep.finalCOMPosition.yaw[0] ) << " "                 // 5
	      << filterprecision(m_OneStep.finalCOMPosition.x[1] ) << " "                   // 6
	      << filterprecision(m_OneStep.finalCOMPosition.y[1] ) << " "                   // 7
	      << filterprecision(m_OneStep.finalCOMPosition.z[1] ) << " "                   // 8
	      << filterprecision(m_OneStep.ZMPTarget(0) ) << " "                            // 9
	      << filterprecision(m_OneStep.ZMPTarget(1) ) << " "                            // 10
	      << filterprecision(m_OneStep.LeftFootPosition.x  ) << " "                     // 11
	      << filterprecision(m_OneStep.LeftFootPosition.y  ) << " "                     // 12
	      << filterprecision(m_OneStep.LeftFootPosition.z  ) << " "                     // 13
	      << filterprecision(m_OneStep.LeftFootPosition.dx  ) << " "                    // 14
	      << filterprecision(m_OneStep.LeftFootPosition.dy  ) << " "                    // 15
	      << filterprecision(m_OneStep.LeftFootPosition.dz  ) << " "                    // 16
	      << filterprecision(m_OneStep.LeftFootPosition.ddx  ) << " "                   // 17
	      << filterprecision(m_OneStep.LeftFootPosition.ddy  ) << " "                   // 18
	      << filterprecision(m_OneStep.LeftFootPosition.ddz  ) << " "                   // 19
              << filterprecision(m_OneStep.LeftFootPosition.theta*M_PI/180 ) << " "         // 20
	      << filterprecision(m_OneStep.LeftFootPosition.omega  ) << " "                 // 21
	      << filterprecision(m_OneStep.LeftFootPosition.omega2  ) << " "                // 22
	      << filterprecision(m_OneStep.RightFootPosition.x ) << " "                     // 23
	      << filterprecision(m_OneStep.RightFootPosition.y ) << " "                     // 24
	      << filterprecision(m_OneStep.RightFootPosition.z ) << " "                     // 25
	      << filterprecision(m_OneStep.RightFootPosition.dx ) << " "                    // 26
	      << filterprecision(m_OneStep.RightFootPosition.dy ) << " "                    // 27
	      << filterprecision(m_OneStep.RightFootPosition.dz ) << " "                    // 28
	      << filterprecision(m_OneStep.RightFootPosition.ddx ) << " "                   // 29
	      << filterprecision(m_OneStep.RightFootPosition.ddy ) << " "                   // 30
	      << filterprecision(m_OneStep.RightFootPosition.ddz ) << " "                   // 31
	      << filterprecision(m_OneStep.RightFootPosition.theta*M_PI/180 ) << " "     // 32
	      << filterprecision(m_OneStep.RightFootPosition.omega  ) << " "                // 33
	      << filterprecision(m_OneStep.RightFootPosition.omega2  ) << " "               // 34
	      << filterprecision(m_OneStep.ZMPTarget(0)*cos(m_CurrentConfiguration(5)) -
				 m_OneStep.ZMPTarget(1)*sin(m_CurrentConfiguration(5))
				 +m_CurrentConfiguration(0) ) << " "                                          // 35
	      << filterprecision(m_OneStep.ZMPTarget(0)*sin(m_CurrentConfiguration(5)) +
				 m_OneStep.ZMPTarget(1)*cos(m_CurrentConfiguration(5))
				 +m_CurrentConfiguration(1) ) << " "                                          // 36
	      << filterprecision(m_CurrentConfiguration(0) ) << " "                         // 37
	      << filterprecision(m_CurrentConfiguration(1) ) << " ";                        // 38
        for (unsigned int i = 0 ; i < m_HDR->currentConfiguration().size() ; i++)
        {
          aof << filterprecision(m_HDR->currentConfiguration()(i)) << " " ;                  // 39 - 74
        }

	  aof << endl;
	  aof.close();
        }


      /// \brief Debug Purpose
      /// --------------------
      ofstream aof;
      string aFileName;
      ostringstream oss(std::ostringstream::ate);

      if ( iteration == 0 ){
        oss.str("/tmp/walk_mnaveau.pos");
        aFileName = oss.str();
        aof.open(aFileName.c_str(),ofstream::out);
        aof.close();
      }
      ///----
      oss.str("/tmp/walk_mnaveau.pos");
      aFileName = oss.str();
      aof.open(aFileName.c_str(),ofstream::app);
      aof.precision(8);
      aof.setf(ios::scientific, ios::floatfield);
      aof << filterprecision( iteration * 0.1 ) << " "  ; // 1
      for(unsigned int i = 6 ; i < m_CurrentConfiguration.size() ; i++){
        aof << filterprecision( m_CurrentConfiguration(i) ) << " "  ; // 1
      }
      for(unsigned int i = 0 ; i < 10 ; i++){
        aof << 0.0 << " "  ;
      }
      aof  << endl ;
      aof.close();

      if ( iteration == 0 ){
        oss.str("/tmp/walk_mnaveau.hip");
        aFileName = oss.str();
        aof.open(aFileName.c_str(),ofstream::out);
        aof.close();
      }
      oss.str("/tmp/walk_mnaveau.hip");
      aFileName = oss.str();
      aof.open(aFileName.c_str(),ofstream::app);
      aof.precision(8);
      aof.setf(ios::scientific, ios::floatfield);
      for(unsigned int j = 0 ; j < 20 ; j++){
        aof << filterprecision( iteration * 0.1 ) << " "  ; // 1
        aof << filterprecision( 0.0 ) << " "  ; // 1
        aof << filterprecision( 0.0 ) << " "  ; // 1
        aof << filterprecision( m_OneStep.finalCOMPosition.yaw[0] ) << " "  ; // 1
        aof << endl ;
      }
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

  void ComparingZMPs()
  {
		const int stage0 = 0 ;
    /// \brief calculate, from the CoM of computed by the preview control,
    ///    the corresponding articular position, velocity and acceleration
    /// ------------------------------------------------------------------
    MAL_VECTOR(CurrentConfiguration,double);
    MAL_VECTOR(CurrentVelocity,double);
    MAL_VECTOR(CurrentAcceleration,double);

    MAL_VECTOR_RESIZE(CurrentConfiguration, m_HDR->numberDof());
    MAL_VECTOR_RESIZE(CurrentVelocity, m_HDR->numberDof());
    MAL_VECTOR_RESIZE(CurrentAcceleration, m_HDR->numberDof());

    MAL_VECTOR_DIM(aCOMState,double,6);
    MAL_VECTOR_DIM(aCOMSpeed,double,6);
    MAL_VECTOR_DIM(aCOMAcc,double,6);
    MAL_VECTOR_DIM(aLeftFootPosition,double,5);
    MAL_VECTOR_DIM(aRightFootPosition,double,5);

    aCOMState(0) = m_OneStep.finalCOMPosition.x[0];      aCOMSpeed(0) = m_OneStep.finalCOMPosition.x[1];      aCOMAcc(0) = m_OneStep.finalCOMPosition.x[2];
    aCOMState(1) = m_OneStep.finalCOMPosition.y[0];      aCOMSpeed(1) = m_OneStep.finalCOMPosition.y[1];      aCOMAcc(1) = m_OneStep.finalCOMPosition.y[2];
    aCOMState(2) = m_OneStep.finalCOMPosition.z[0];      aCOMSpeed(2) = m_OneStep.finalCOMPosition.z[1];      aCOMAcc(2) = m_OneStep.finalCOMPosition.z[2];
    aCOMState(3) = m_OneStep.finalCOMPosition.roll[0];   aCOMSpeed(3) = m_OneStep.finalCOMPosition.roll[1]; 	aCOMAcc(3) = m_OneStep.finalCOMPosition.roll[2];
    aCOMState(4) = m_OneStep.finalCOMPosition.pitch[0];  aCOMSpeed(4) = m_OneStep.finalCOMPosition.pitch[1];	aCOMAcc(4) = m_OneStep.finalCOMPosition.pitch[2];
    aCOMState(5) = m_OneStep.finalCOMPosition.yaw[0];    aCOMSpeed(5) = m_OneStep.finalCOMPosition.yaw[1];  	aCOMAcc(5) = m_OneStep.finalCOMPosition.yaw[2];

    aLeftFootPosition(0) = m_OneStep.LeftFootPosition.x;      aRightFootPosition(0) = m_OneStep.RightFootPosition.x;
    aLeftFootPosition(1) = m_OneStep.LeftFootPosition.y;      aRightFootPosition(1) = m_OneStep.RightFootPosition.y;
    aLeftFootPosition(2) = m_OneStep.LeftFootPosition.z;      aRightFootPosition(2) = m_OneStep.RightFootPosition.z;
    aLeftFootPosition(3) = m_OneStep.LeftFootPosition.theta;  aRightFootPosition(3) = m_OneStep.RightFootPosition.theta;
    aLeftFootPosition(4) = m_OneStep.LeftFootPosition.omega;  aRightFootPosition(4) = m_OneStep.RightFootPosition.omega;
    ComAndFootRealization_->setSamplingPeriod(0.005);
    ComAndFootRealization_->ComputePostureForGivenCoMAndFeetPosture(aCOMState, aCOMSpeed, aCOMAcc,
                      aLeftFootPosition,
                      aRightFootPosition,
                      CurrentConfiguration,
                      CurrentVelocity,
                      CurrentAcceleration,
                      m_OneStep.NbOfIt,
                      stage0);

		/// \brief Debug Purpose
		/// --------------------
		ofstream aof;
		string aFileName;
		ostringstream oss(std::ostringstream::ate);
		oss.str("TestHerdt2010DynamicART2.dat");
		aFileName = oss.str();
		if ( iteration_zmp == 0 )
		{
			aof.open(aFileName.c_str(),ofstream::out);
			aof.close();
		}
		///----
		aof.open(aFileName.c_str(),ofstream::app);
		aof.precision(8);
		aof.setf(ios::scientific, ios::floatfield);
		for (unsigned int j = 0 ; j < CurrentConfiguration.size() ; ++j)
		{
			aof << filterprecision(CurrentConfiguration(j)) << " " ;
		}
		for (unsigned int j = 0 ; j < CurrentVelocity.size() ; ++j)
		{
			aof << filterprecision(CurrentVelocity(j)) << " " ;
		}
		for (unsigned int j = 0 ; j < CurrentAcceleration.size() ; ++j)
		{
			aof << filterprecision(CurrentAcceleration(j)) << " " ;
		}
		aof << endl ;


    /// \brief rnea, calculation of the multi body ZMP
    /// ----------------------------------------------
    Robot_Model2::confVector q, dq, ddq;
    for(unsigned int j = 0 ; j < CurrentConfiguration.size() ; j++ )
    {
      q(j,0) = CurrentConfiguration[j] ;
      dq(j,0) = CurrentVelocity[j] ;
      ddq(j,0) = CurrentAcceleration[j] ;
    }
    metapod::rnea< Robot_Model2, true >::run(robot_, q, dq, ddq);

    Node2 anode = boost::fusion::at_c<Robot_Model2::BODY>(robot_.nodes);
    Force2 aforce = anode.body.iX0.applyInv (anode.joint.f) ;

    double ZMPMB[2];

    ZMPMB[0] = - aforce.n()[1] / aforce.f()[2] ;
    ZMPMB[1] = aforce.n()[0] / aforce.f()[2] ;


    if (m_OneStep.NbOfIt<=10){
      dInitX = m_OneStep.ZMPTarget(0) - ZMPMB[0] ;
      dInitY = m_OneStep.ZMPTarget(1) - ZMPMB[1] ;
      {
        vector<double> tmp_zmp(2) ;
        tmp_zmp[0] =0.0 ;
        tmp_zmp[1] =0.0 ;
        errZMP_.push_back(tmp_zmp);
      }
    }

    if (m_OneStep.NbOfIt >= 10)
    {
      double errx = sqrt( ( m_OneStep.ZMPTarget(0) - ZMPMB[0] - dInitX )*( m_OneStep.ZMPTarget(0) - ZMPMB[0] - dInitX ) ) ;
      double erry = sqrt( ( m_OneStep.ZMPTarget(1) - ZMPMB[1] - dInitY )*( m_OneStep.ZMPTarget(1) - ZMPMB[1] - dInitY ) ) ;
      {
        vector<double> tmp_zmp(2) ;
        tmp_zmp[0] = errx ;
        tmp_zmp[1] = erry ;
        errZMP_.push_back(tmp_zmp);
      }
    }


    static double ecartmaxX = 0 ;
    static double ecartmaxY = 0 ;
    if ( abs(errZMP_.back()[0]) > ecartmaxX )
      ecartmaxX = abs(errZMP_.back()[0]) ;
    if ( abs(errZMP_.back()[1]) > ecartmaxY )
      ecartmaxY = abs(errZMP_.back()[1]) ;

//    cout << "ecartmaxX :" << ecartmaxX << endl ;
//    cout << "ecartmaxY :" << ecartmaxY << endl ;

    // Writing of the two zmps and the error.
    if (once)
    {
      aof.open("TestHerdt2010ErrorZMP.dat",ofstream::out);
      aof.close();
      once = false ;
    }
    aof.open("TestHerdt2010ErrorZMP.dat",ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    aof << filterprecision( iteration_zmp ) << " "          // 1
        << filterprecision( ZMPMB[0] + dInitX ) << " "      // 2
        << filterprecision( ZMPMB[1] + dInitY ) << " "      // 3
        << filterprecision(m_OneStep.ZMPTarget(0) ) << " "  // 4
        << filterprecision(m_OneStep.ZMPTarget(1) ) << " "  // 5
        << endl ;
    aof.close();

    iteration_zmp++;
    return ;
  }

  void ComputeAndDisplayAverageError(bool display){
    static int plot_it = 0 ;
    double moyErrX = 0 ;
    double moyErrY = 0 ;
    for (unsigned int i = 0 ; i < errZMP_.size(); ++i)
    {
      moyErrX += errZMP_[i][0] ;
      moyErrY += errZMP_[i][1] ;
    }
    moyErrX = moyErrX / errZMP_.size() ;
    moyErrY = moyErrY / errZMP_.size() ;
    if ( display )
    {
      cout << "moyErrX = " << moyErrX << endl
           << "moyErrY = " << moyErrY << endl ;
    }
    ofstream aof;
	  string aFileName;
	  aFileName = m_TestName;
	  aFileName += "MoyZMP.dat";
	  if(plot_it==0){
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
	  }
	  aof.open(aFileName.c_str(),ofstream::app);
	  aof.precision(8);
	  aof.setf(ios::scientific, ios::floatfield);
	  aof << filterprecision(moyErrX ) << " "        // 1
        << filterprecision(moyErrY ) << " "        // 2
        << endl ;
    aof.close();
    plot_it++;
  }


  void StartAnalyticalOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);

    {
      istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":onlinechangestepframe relative");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":SetAutoFirstStep false");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":StartOnLineStepSequencing 0.0 -0.095 0.0 0.0 0.19 0.0 0.0 -0.19 0.0 0.0 0.19 0.0");
      aPGI.ParseCmd(strm2);
    }
  }

  void StopOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    istringstream strm2(":StopOnLineStepSequencing");
    aPGI.ParseCmd(strm2);
  }

  void AnalyticalShortStraightWalking(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);
    {
      istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":stepseq 0.0 -0.105 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.0 -0.19 0.0");
      aPGI.ParseCmd(strm2);
    }

  }

  void chooseTestProfile()
  {

    switch(m_TestProfile)
      {
      case PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING:
	AnalyticalShortStraightWalking(*m_PGI);
	break;

      case PROFIL_ANALYTICAL_ONLINE_WALKING:
	StartAnalyticalOnLineWalking(*m_PGI);
	break;
      default:
	throw("No correct test profile");
	break;
      }
  }

  void generateEvent()
  {
    if (m_TestProfile==PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING)
      return;

    unsigned int StoppingTime = 70*200;


    double r = 100.0*(double)m_OneStep.NbOfIt/(double)StoppingTime;


    /* Stop after 30 seconds the on-line stepping */
    if (m_OneStep.NbOfIt>StoppingTime) 
      {
	StopOnLineWalking(*m_PGI);
      }
    else 
      {
	/* Stay on the spot during 5.0 s before stopping. */
	if (m_OneStep.NbOfIt<StoppingTime-200*5.0) 
	  {
	    if (m_OneStep.NbOfIt%200==0)
	      {
		cout << "Progress " << (unsigned int)r << " "<< "\r";
		cout.flush();
	      }
	
	    double triggertime = 9.8*200 + m_deltatime*200;
	    if ((m_OneStep.NbOfIt>triggertime) &&
		m_TestChangeFoot)
	      {
		PatternGeneratorJRL::FootAbsolutePosition aFAP;
		if (m_NbStepsModified<NBOFPREDEFONLINEFOOTSTEPS)
		  {
		    aFAP.x = OnLineFootSteps[m_NbStepsModified][0];
		    aFAP.y = OnLineFootSteps[m_NbStepsModified][1];
		    aFAP.theta = OnLineFootSteps[m_NbStepsModified][2];
		  }
		else
		  {
		    aFAP.x=0.1;
		    aFAP.y=0.0;
		    aFAP.theta=5.0;
		  }
		double newtime;
		bool stepHandledCorrectly=true;
		try 
		  {
		    m_PGI->ChangeOnLineStep(0.805,aFAP,newtime);
		  }
		catch(...)
		  {
		    cerr << "Step not well handled." << endl;
		    stepHandledCorrectly=false;
		  }
		if (stepHandledCorrectly)
		  {
		    m_deltatime += newtime+0.025;
		    m_TestChangeFoot=true;
		    m_NbStepsModified++;
		    if (m_NbStepsModified==360)
		      m_TestChangeFoot=false;
		  }
		else 
		  {
		    m_deltatime += 0.005;
		  }
	      }
	  }
      }
  };
  
};

int PerformTests(int argc, char *argv[])
{
  std::string CompleteName = string(argv[0]);
  unsigned found = CompleteName.find_last_of("/\\");
  std::string TestName =  CompleteName.substr(found+1);
  int TestProfiles[2] = { PROFIL_ANALYTICAL_ONLINE_WALKING,
			  PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING};
  int indexProfile=-1;

  if (TestName.compare(16,6,"OnLine")==0)
    indexProfile=0;
  if (TestName.compare(16,9,"ShortWalk")==0)
    indexProfile=1;

  if (indexProfile==-1)
    {
      std::cerr << "CompleteName: " << CompleteName << std::endl;
      std::cerr<< " TestName: " << TestName <<std::endl;
      std::cerr<< "Failure to find the proper indexFile:" << TestName.substr(17,6) << endl;
      exit(-1);
    }
  TestMorisawa2007 aTM2007(argc,argv,
                           TestName,
                           TestProfiles[indexProfile]);
  aTM2007.init();
  try
    {
      if (!aTM2007.doTest(std::cout))
        {
          cout << "Failed test " << indexProfile << endl;
          return -1;
        }
      else
        cout << "Passed test " << indexProfile << endl;
        }
  catch (const char * astr)
    { cerr << "Failed on following error " << astr << std::endl;
      return -1; }
  return 0;
}

int main(int argc, char *argv[])
{
  try
    {
      return PerformTests(argc,argv);
    }
  catch (const std::string& msg)
    {
      std::cerr << msg << std::endl;
    }
  return 1;
}



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


#ifndef METAPOD_INCLUDES
#define METAPOD_INCLUDES
// metapod includes
typedef double LocalFloatType;
#include <metapod/models/hrp2_14/hrp2_14.hh>
#include <metapod/tools/print.hh>
#include <metapod/tools/initconf.hh>
#include "metapod/algos/rnea.hh"
#include <Eigen/StdVector>
typedef metapod::hrp2_14<LocalFloatType> Robot_Model;
#endif

#include <MotionGeneration/ComAndFootRealizationByGeometry.hh>

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
  // buffer to save all the zmps positions
  ComAndFootRealizationByGeometry * comAndFootRealization ;
  double errZMP [2] ;

  public:
  TestHerdt2010(int argc, char *argv[], string &aString, int TestProfile):
    TestObject(argc,argv,aString)
  {
    m_TestProfile = TestProfile;
    comAndFootRealization = new ComAndFootRealizationByGeometry(NULL);
    errZMP[0]=0.0;
    errZMP[1]=0.0;
  };

  ~TestHerdt2010(){
    if (comAndFootRealization != NULL ){
      delete comAndFootRealization ;
      comAndFootRealization = NULL ;
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
          fillInDebugFiles();
          //ComparingZMPs(m_OneStep.NbOfIt);
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
    ComputeAndDisplayAverageError();
    return compareDebugFiles();
  }

protected:

  void CallToComAndFootRealization(COMState &acomp,
       FootAbsolutePosition &aLeftFAP,
       FootAbsolutePosition &aRightFAP,
       MAL_VECTOR_TYPE(double) &CurrentConfiguration,
       MAL_VECTOR_TYPE(double) &CurrentVelocity,
       MAL_VECTOR_TYPE(double) &CurrentAcceleration,
       int IterationNumber,
       int StageOfTheAlgorithm,
       ComAndFootRealizationByGeometry *comAndFootRealization)
   {

     // New scheme for WPG v3.0
     // We call the object in charge of generating the whole body
     // motion  ( for a given CoM and Feet points)  before applying the second filter.
     MAL_VECTOR_DIM(aCOMState,double,6);
     MAL_VECTOR_DIM(aCOMSpeed,double,6);
     MAL_VECTOR_DIM(aCOMAcc,double,6);

     aCOMState(0) = acomp.x[0];
     aCOMState(1) = acomp.y[0];
     aCOMState(2) = acomp.z[0];
     aCOMState(3) = acomp.roll[0];
     aCOMState(4) = acomp.pitch[0];
     aCOMState(5) = acomp.yaw[0];

     aCOMSpeed(0) = acomp.x[1];
     aCOMSpeed(1) = acomp.y[1];
     aCOMSpeed(2) = acomp.z[1];
     aCOMSpeed(3) = acomp.roll[1];
     aCOMSpeed(4) = acomp.roll[1];
     aCOMSpeed(5) = acomp.roll[1];

     aCOMAcc(0) = acomp.x[2];
     aCOMAcc(1) = acomp.y[2];
     aCOMAcc(2) = acomp.z[2];
     aCOMAcc(3) = acomp.roll[2];
     aCOMAcc(4) = acomp.roll[2];
     aCOMAcc(5) = acomp.roll[2];

     MAL_VECTOR_DIM(aLeftFootPosition,double,5);
     MAL_VECTOR_DIM(aRightFootPosition,double,5);

     aLeftFootPosition(0) = aLeftFAP.x;
     aLeftFootPosition(1) = aLeftFAP.y;
     aLeftFootPosition(2) = aLeftFAP.z;
     aLeftFootPosition(3) = aLeftFAP.theta;
     aLeftFootPosition(4) = aLeftFAP.omega;

     aRightFootPosition(0) = aRightFAP.x;
     aRightFootPosition(1) = aRightFAP.y;
     aRightFootPosition(2) = aRightFAP.z;
     aRightFootPosition(3) = aRightFAP.theta;
     aRightFootPosition(4) = aRightFAP.omega;

     /* Get the current configuration vector */
     CurrentConfiguration = m_HDR->currentConfiguration();

     /* Get the current velocity vector */
     CurrentVelocity = m_HDR->currentVelocity();

     /* Get the current acceleration vector */
     CurrentAcceleration = m_HDR->currentAcceleration();

     comAndFootRealization->ComputePostureForGivenCoMAndFeetPosture(aCOMState, aCOMSpeed, aCOMAcc,
                      aLeftFootPosition,
                      aRightFootPosition,
                      CurrentConfiguration,
                      CurrentVelocity,
                      CurrentAcceleration,
                      IterationNumber,
                      StageOfTheAlgorithm);
     if (StageOfTheAlgorithm==0)
       {
         /* Update the current configuration vector */
         m_HDR->currentConfiguration(CurrentConfiguration);

         /* Update the current velocity vector */
         m_HDR->currentVelocity(CurrentVelocity);

         /* Update the current acceleration vector */
         m_HDR->currentAcceleration(CurrentAcceleration);
       }
    }

  double filterprecision(double adb)
  {
    if (fabs(adb)<1e-7)
      return 0.0;

    double ladb2 = adb * 1e7;
    double lintadb2 = trunc(ladb2);
    return lintadb2/1e7;
  }

  void ComparingZMPs(int it)
  {
    // Read the robot VRML file model.
    comAndFootRealization->setHumanoidDynamicRobot(m_HDR);
    comAndFootRealization->SetHeightOfTheCoM(0.814);
    comAndFootRealization->setSamplingPeriod(0.1);
    comAndFootRealization->SetStepStackHandler(new StepStackHandler(new SimplePluginManager()));
    comAndFootRealization->Initialization();

    // \brief claculate, from the CoM of computed by the preview control,
    //    the corresponding articular position, velocity and acceleration
    // ------------------------------------------------------------------
    MAL_VECTOR_TYPE(double) configurationTraj ;
    MAL_VECTOR_TYPE(double) velocityTraj ;
    MAL_VECTOR_TYPE(double) accelerationTraj ;
    COMState aComSta ;
    for(unsigned int i=0;i<3;i++)
    {
      aComSta.x[i] = m_OneStep.finalCOMPosition.x[i];
      aComSta.y[i] = m_OneStep.finalCOMPosition.y[i];
      aComSta.z[i] = m_OneStep.finalCOMPosition.z[i];
    };
    aComSta.yaw[0]   = m_OneStep.finalCOMPosition.yaw;   aComSta.yaw[1]   = aComSta.yaw[2]   = 0.0;
    aComSta.pitch[0] = m_OneStep.finalCOMPosition.pitch; aComSta.pitch[1] = aComSta.pitch[2] = 0.0;
    aComSta.roll[0]  = m_OneStep.finalCOMPosition.roll;  aComSta.roll[1]  = aComSta.roll[2]  = 0.0;
    CallToComAndFootRealization(
        aComSta,
        m_OneStep.LeftFootPosition,
        m_OneStep.RightFootPosition,
        configurationTraj,
        velocityTraj,
        accelerationTraj,
        it,0,comAndFootRealization);

    // \brief rnea
    // -----------
    // Initialize the robot with the autogenerated files by MetapodFromUrdf
    Robot_Model robot;
    // Set configuration vectors (q, dq, ddq) to reference values.
    Robot_Model::confVector torques;
    Robot_Model::confVector q, dq, ddq;
    // Apply the RNEA to the metapod multibody and print the result in a log file.
    for( unsigned int j = 0 ; j < configurationTraj.size() ; j++ )
    {
      q(j,0) = configurationTraj[j] ;
      dq(j,0) = velocityTraj[j] ;
      ddq(j,0) = accelerationTraj[j] ;
    }
    metapod::rnea< Robot_Model, true >::run(robot, q, dq, ddq);
    getTorques(robot, torques);

    // Projection of the Torques on the ground, the result is the ZMP Multi-Body
    // -------------------------------------------------------------------------
    double factor = 1/(m_HDR->mass()*9.81);
    ZMPPosition ZMPMB;
    // Smooth ramp
    ZMPMB.px = torques(1,0) /*(4,0)*/ * factor ;
    ZMPMB.py = torques(0,0) /*(3,0)*/ * factor ;
    ZMPMB.pz = 0 ;
    ZMPMB.theta = 0.0;
    ZMPMB.time = m_clock.getStartOneIteration();
    ZMPMB.stepType = 0 ;

    double errx = fabs ( m_OneStep.ZMPTarget(0) - ZMPMB.px ) ;
    double erry = fabs ( m_OneStep.ZMPTarget(1) - ZMPMB.py ) ;

    errZMP[0] += errx ;
    errZMP[1] += erry ;

    // Writing of the two zmps and the error.
    ofstream aof;
	  string aFileName;
	  aFileName = m_TestName;
	  aFileName += "DeltaZMP.dat";
	  aof.open(aFileName.c_str(),ofstream::app);
	  aof.precision(8);
	  aof.setf(ios::scientific, ios::floatfield);
	  aof << filterprecision(m_OneStep.NbOfIt*0.005 ) << " "          // 1
        << filterprecision(m_OneStep.ZMPTarget(0) ) << " "          // 2
        << filterprecision(m_OneStep.ZMPTarget(1) ) << " "          // 3
        << filterprecision(ZMPMB.px) << " "                         // 4
        << filterprecision(ZMPMB.py) << " "                         // 5
        << filterprecision(errx) << " "                             // 6
        << filterprecision(erry) << " "                             // 7
        << filterprecision(m_OneStep.finalCOMPosition.x[0] ) << " " // 8
	      << filterprecision(m_OneStep.finalCOMPosition.y[0] ) << " " // 9
        << endl ;
    aof.close();
  }

  void ComputeAndDisplayAverageError(){
    double moyErrX = errZMP[0] / m_OneStep.NbOfIt ;
    double moyErrY = errZMP[1] / m_OneStep.NbOfIt ;
    cout << "moyErrX = " << moyErrX << endl
         << "moyErrY = " << moyErrY << endl ;

    // Writing of the two zmps and the error.
    ofstream aof;
	  string aFileName;
	  aFileName = m_TestName;
	  aFileName += "MoyZMP.dat";
	  aof.open(aFileName.c_str(),ofstream::app);
	  aof.precision(8);
	  aof.setf(ios::scientific, ios::floatfield);
	  aof << filterprecision(moyErrX ) << " "        // 1
        << filterprecision(moyErrY ) << " "        // 2
        << endl ;
    aof.close();
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
  void walkForward(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.2 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
  }
  void walkSidewards(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 0.2 0.0");
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
    { { 5*200,&TestHerdt2010::walkForward},
   //   {10*200,&TestHerdt2010::walkSidewards},
 //     {25*200,&TestHerdt2010::startTurningRightOnSpot},
//      {35*200,&TestHerdt2010::walkForward},
//      {45*200,&TestHerdt2010::startTurningLeftOnSpot},
//      {55*200,&TestHerdt2010::walkForward},
//      {65*200,&TestHerdt2010::startTurningRightOnSpot},
//      {75*200,&TestHerdt2010::walkForward},
//      {85*200,&TestHerdt2010::startTurningLeft},
//      {95*200,&TestHerdt2010::startTurningRight},
      {10*200,&TestHerdt2010::stop},
      {25*200,&TestHerdt2010::stopOnLineWalking}
    };

    // Test when triggering event.
    for(unsigned int i=0;i<localNbOfEvents;i++)
    {
	  if ( m_OneStep.NbOfIt==events[i].time){
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
      {20.8*200,&TestHerdt2010::stopOnLineWalking}
    };

    // Test when triggering event.
    for(unsigned int i=0;i<localNbOfEventsEMS;i++){
      if ( m_OneStep.NbOfIt==events[i].time){
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
      system("pause");
      return ret ;
    }
  catch (const std::string& msg)
    {
      std::cerr << msg << std::endl;
    }
  system("pause");
  return 1;
}



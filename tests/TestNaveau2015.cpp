/*
 * Copyright 2009, 2010, 2014
 *
 * Maximilien Naveau
 * Olivier  Stasse,
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
/*! \file TestNaveau2015.cpp
  \brief This Example shows you how to use the nmpc_genereator.cpp */
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl/dynamics/dynamicsfactory.hh>
#include <jrl/walkgen/patterngeneratorinterface.hh>
#include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>
#include <ZMPRefTrajectoryGeneration/ZMPVelocityReferencedSQP.hh>

#include <jrl/walkgen/config_private.hh>

using namespace std;
using namespace PatternGeneratorJRL;

#include "Debug.hh"
#include "CommonTools.hh"
#include "TestObject.hh"
#include <jrl/walkgen/pgtypes.hh>
#include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>
#include <MotionGeneration/ComAndFootRealizationByGeometry.hh>

#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>

#include <metapod/models/hrp2_14/hrp2_14.hh>
#include <ZMPRefTrajectoryGeneration/DynamicFilter.hh>
#include <metapod/algos/rnea.hh>
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
  PROFIL_NAVEAU_ONLINE_WALKING // 1
};

class IO_TextFile
{

private:
  string namefile;
  ifstream myfile_input;
  ofstream myfile_output;

public:
  vector<vector<double> > data ;

  int WriteOnFile_AddText(string text)
  {

    myfile_output.open(namefile.c_str(),ofstream::app);
    if (myfile_output.is_open())
    {
      myfile_output << text;
      myfile_output.close();
      return 0;
    }
    else
    {
      cout << "Unable to open file";
      return -1;
    }
  }

  int WriteOnFile_DeleteAndWrite(string text)
  {

    myfile_output.open(namefile.c_str(),ofstream::out);
    if (myfile_output.is_open())
    {
      myfile_output << text;
      myfile_output.close();
      return 0;
    }
    else
    {
      cout << "Unable to open file";
      return -1;
    }
  }

  bool ReadFromFile()
  {
    data.clear();
    string linetmp;
    myfile_input.open(namefile.c_str(),ifstream::in);
    if (myfile_input.is_open())
    {
      while ( getline (myfile_input,linetmp) )
      {
        stringstream line(linetmp);
//        cout << linetmp << endl ;
//        cout << line.str() << endl ;
        vector<double>tmpvector;
        for(unsigned i=0 ; i<5 ; ++i)
        {
          double tmpdouble ;
          line >> tmpdouble ;
          tmpvector.push_back(tmpdouble);
        }
        data.push_back(tmpvector);
      }
      myfile_input.close();
    }
    else
      return false ;

    return true ;
  }

  IO_TextFile(string namefile_init)
  {
    namefile = namefile_init;
  }

  ~IO_TextFile()
  {

  }
};

// Class TestNaveau2015
class TestNaveau2015: public TestObject
{

private:
  ComAndFootRealization * ComAndFootRealization_;
  SimplePluginManager * SPM ;
  int iteration ;
  vector<double> err_zmp_x ;
  vector<double> err_zmp_y ;
  int resetfiles ;
  IO_TextFile * dataFile_;

  /// Class that compute the dynamic and kinematic of the robot
  CjrlHumanoidDynamicRobot * cjrlHDR_ ;
  Robot_Model hrp2_14_ ;
  Robot_Model::confVector q_,dq_,ddq_;
  Force_HRP2_14 com_tensor_ ;

public:
  TestNaveau2015(int argc, char *argv[], string &aString, int TestProfile):
    TestObject(argc,argv,aString)
  {
    m_TestProfile = TestProfile;
    SPM = 0 ;
    ComAndFootRealization_ = 0 ;
    iteration = 0 ;
    err_zmp_x.clear() ;
    err_zmp_y.clear() ;
    resetfiles=0;

    m_DebugFGPI=false;
  }

  ~TestNaveau2015()
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

  typedef void (TestNaveau2015::* localeventHandler_t)(PatternGeneratorInterface &);

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

    std::random_device rd;
    std::mt19937 gen(rd());
    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    double d_std(0.000),d_bias(0);
    std::normal_distribution<> d(d_bias,d_std);

//    std::map<int, int> hist;
//    for(int n=0; n<10000; ++n) {
//      double randvar = d(gen);
//      cout << randvar << endl ;
//      ++hist[std::round(randvar)];
//    }
//    for(auto p : hist) {
//        std::cout << std::fixed << std::setprecision(1) << std::setw(2)
//                  << p.first << ' ' << std::string(p.second/200, '*') << '\n';
//    }

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
          m_OneStep.ZMPTarget(0) += d(gen);
          m_OneStep.ZMPTarget(1) += d(gen);
          m_OneStep.finalCOMPosition.x[0]+= d(gen);
          m_OneStep.finalCOMPosition.x[1]+= d(gen);
          m_OneStep.finalCOMPosition.x[2]+= d(gen);
          m_OneStep.finalCOMPosition.y[0]+= d(gen);
          m_OneStep.finalCOMPosition.y[1]+= d(gen);
          m_OneStep.finalCOMPosition.y[2]+= d(gen);
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

    ComputeAndDisplayZMPStatistic();
    string lProfileOutput= m_TestName;
    lProfileOutput +="TimeProfile.dat";
    m_clock.writeBuffer(lProfileOutput);
    m_clock.displayStatistics(os,m_OneStep);
    // Compare debugging files
    return compareDebugFiles();
  }

  void ComputeStat(vector<double> vec,double &avg, double &max_abs)
  {
    double total = 0.0 ;
    avg = 0.0 ;
    max_abs = 0.0 ;
    for (unsigned int i = 0 ; i < vec.size() ; ++i)
    {
      double abs_value = sqrt(vec[i]*vec[i]) ;
      if( abs_value > max_abs)
        max_abs = abs_value ;

      total += abs_value ;
    }
    avg = total/vec.size() ;
    return ;
  }

  void ComputeAndDisplayZMPStatistic()
  {
    cout << "Statistic for Dzmp in x : " << endl ;
    double moy_delta_zmp_x = 0.0 ;
    double max_abs_err_x = 0.0 ;
    ComputeStat(err_zmp_x,moy_delta_zmp_x,max_abs_err_x);
    cout << "average : " << moy_delta_zmp_x << endl ;
    cout << "maxx error : " << max_abs_err_x << endl ;

    cout << "Statistic for Dzmp in y : " << endl ;
    double moy_delta_zmp_y = 0.0 ;
    double max_abs_err_y = 0.0 ;
    ComputeStat(err_zmp_y,moy_delta_zmp_y,max_abs_err_y);
    cout << "average : " << moy_delta_zmp_y << endl ;
    cout << "maxx error : " << max_abs_err_y << endl ;
    return ;
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


    dataFile_ = new IO_TextFile ("/home/mnaveau/devel/ros_unstable/src/jrl/jrl-walkgen/tests/CommandedVelocity.txt");
    dataFile_->ReadFromFile();
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

  void prepareDebugFiles()
  {
    TestObject::prepareDebugFiles() ;
    if (m_DebugFGPI)
    {
      ofstream aof;
      string aFileName;
      aFileName = m_TestName;
      aFileName += "TestFGPIFull.dat";
      if (m_OneStep.NbOfIt==1)
      {
        aof.open(aFileName.c_str(),ofstream::out);
      }
    }
  }

  void fillInDebugFiles()
  {

    if (m_DebugFGPI)
    {
      TestObject::fillInDebugFiles();
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
      aCOMState(3) = m_OneStep.finalCOMPosition.roll[0]  * 180/M_PI  ;  aCOMSpeed(3) = m_OneStep.finalCOMPosition.roll[1] /** * 180/M_PI  */ ;  aCOMAcc(3) = m_OneStep.finalCOMPosition.roll[2]/** * 180/M_PI  */ ;
      aCOMState(4) = m_OneStep.finalCOMPosition.pitch[0] * 180/M_PI  ;  aCOMSpeed(4) = m_OneStep.finalCOMPosition.pitch[1]/** * 180/M_PI  */ ;  aCOMAcc(4) = m_OneStep.finalCOMPosition.pitch[2]/** * 180/M_PI  */ ;
      aCOMState(5) = m_OneStep.finalCOMPosition.yaw[0] *180/M_PI;  aCOMSpeed(5) = m_OneStep.finalCOMPosition.yaw[1]/** * 180/M_PI  */ ; aCOMAcc(5) = m_OneStep.finalCOMPosition.yaw[2] /** * 180/M_PI  */;

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

      // compute the 6D force applied at the CoM
      for(unsigned int i = 0 ; i < MAL_VECTOR_SIZE(m_CurrentConfiguration) ; ++i)
      {
        q_(i,0)   = m_CurrentConfiguration (i);
        dq_(i,0)  = m_CurrentVelocity      (i);
        ddq_(i,0) = m_CurrentAcceleration  (i);
      }
      metapod::rnea< Robot_Model, true >::run(hrp2_14_, q_, dq_, ddq_);
      vector<double> zmpmb = vector<double>(3,0.0);
      // extract the CoM momentum and forces
      RootNode & node_waist = boost::fusion::at_c< Robot_Model::BODY >(hrp2_14_.nodes);
      com_tensor_ = node_waist.body.iX0.applyInv(node_waist.joint.f);

      // compute the Multibody ZMP
      zmpmb[0] = - com_tensor_.n()[1] / com_tensor_.f()[2] ;
      zmpmb[1] =   com_tensor_.n()[0] / com_tensor_.f()[2] ;

      err_zmp_x.push_back(zmpmb[0]-m_OneStep.ZMPTarget(0)) ;
      err_zmp_y.push_back(zmpmb[1]-m_OneStep.ZMPTarget(1)) ;

      ofstream aof;
      string aFileName;
      aFileName = m_TestName;
      aFileName += "TestFGPIFull.dat";
      if (m_OneStep.NbOfIt==1)
      {
        aof.open(aFileName.c_str(),ofstream::out);
      }
      resetfiles = 1;
      aof.open(aFileName.c_str(),ofstream::app);
      aof.precision(8);
      aof.setf(ios::scientific, ios::floatfield);
      aof << filterprecision(m_OneStep.NbOfIt*0.005 ) << " "             // 1
          << filterprecision(m_OneStep.finalCOMPosition.x[0] ) << " "    // 2
          << filterprecision(m_OneStep.finalCOMPosition.y[0] ) << " "    // 3
          << filterprecision(m_OneStep.finalCOMPosition.z[0] ) << " "    // 4
          << filterprecision(m_OneStep.finalCOMPosition.yaw[0] ) << " "  // 5
          << filterprecision(m_OneStep.finalCOMPosition.x[1] ) << " "    // 6
          << filterprecision(m_OneStep.finalCOMPosition.y[1] ) << " "    // 7
          << filterprecision(m_OneStep.finalCOMPosition.z[1] ) << " "    // 8
          << filterprecision(m_OneStep.finalCOMPosition.yaw[1] ) << " "  // 9
          << filterprecision(m_OneStep.finalCOMPosition.x[2] ) << " "    // 10
          << filterprecision(m_OneStep.finalCOMPosition.y[2] ) << " "    // 11
          << filterprecision(m_OneStep.finalCOMPosition.z[2] ) << " "    // 12
          << filterprecision(m_OneStep.finalCOMPosition.yaw[2] ) << " "  // 13
          << filterprecision(m_OneStep.ZMPTarget(0) ) << " "             // 14
          << filterprecision(m_OneStep.ZMPTarget(1) ) << " "             // 15
          << filterprecision(m_OneStep.ZMPTarget(2) ) << " "             // 16
          << filterprecision(m_OneStep.LeftFootPosition.x  ) << " "      // 17
          << filterprecision(m_OneStep.LeftFootPosition.y  ) << " "      // 18
          << filterprecision(m_OneStep.LeftFootPosition.z  ) << " "      // 19
          << filterprecision(m_OneStep.LeftFootPosition.dx  ) << " "     // 20
          << filterprecision(m_OneStep.LeftFootPosition.dy  ) << " "     // 21
          << filterprecision(m_OneStep.LeftFootPosition.dz  ) << " "     // 22
          << filterprecision(m_OneStep.LeftFootPosition.ddx  ) << " "    // 23
          << filterprecision(m_OneStep.LeftFootPosition.ddy  ) << " "    // 24
          << filterprecision(m_OneStep.LeftFootPosition.ddz  ) << " "    // 25
          << filterprecision(m_OneStep.LeftFootPosition.theta ) << " "   // 26
          << filterprecision(m_OneStep.LeftFootPosition.dtheta ) << " "  // 27
          << filterprecision(m_OneStep.LeftFootPosition.ddtheta ) << " " // 28
          << filterprecision(m_OneStep.LeftFootPosition.omega  ) << " "  // 29
          << filterprecision(m_OneStep.LeftFootPosition.omega2  ) << " " // 30
          << filterprecision(m_OneStep.RightFootPosition.x ) << " "      // 31
          << filterprecision(m_OneStep.RightFootPosition.y ) << " "      // 32
          << filterprecision(m_OneStep.RightFootPosition.z ) << " "      // 33
          << filterprecision(m_OneStep.RightFootPosition.dx ) << " "     // 34
          << filterprecision(m_OneStep.RightFootPosition.dy ) << " "     // 35
          << filterprecision(m_OneStep.RightFootPosition.dz ) << " "     // 36
          << filterprecision(m_OneStep.RightFootPosition.ddx ) << " "    // 37
          << filterprecision(m_OneStep.RightFootPosition.ddy ) << " "    // 38
          << filterprecision(m_OneStep.RightFootPosition.ddz ) << " "    // 39
          << filterprecision(m_OneStep.RightFootPosition.theta ) << " "  // 40
          << filterprecision(m_OneStep.RightFootPosition.dtheta ) << " " // 41
          << filterprecision(m_OneStep.RightFootPosition.ddtheta ) << " "// 42
          << filterprecision(m_OneStep.RightFootPosition.omega  ) << " " // 43
          << filterprecision(m_OneStep.RightFootPosition.omega2  ) << " "// 44
          << filterprecision(zmpmb[0]) << " "                            // 45
          << filterprecision(zmpmb[1]) << " "                            // 46
          << filterprecision(zmpmb[2]) << " "                           ;// 47
      for(unsigned int k = 0 ; k < m_CurrentConfiguration.size() ; k++){ // 48-53 -> 54-83
        aof << filterprecision( m_CurrentConfiguration(k) ) << " "  ;
      }
      aof << endl;
      aof.close();

      /// \brief Create file .hip .pos .zmp
      /// ---------------------------------
      aFileName = m_TestName + ".pos" ;
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

      aFileName = m_TestName + ".hip" ;
      if ( iteration == 0 ){
        aof.open(aFileName.c_str(),ofstream::out);
        aof.close();
      }
      aof.open(aFileName.c_str(),ofstream::app);
      aof.precision(8);
      aof.setf(ios::scientific, ios::floatfield);
      aof << filterprecision( iteration * 0.005 ) << " "  ;                 // 1
      aof << filterprecision( m_OneStep.finalCOMPosition.roll[0]) << " "  ; // 2
      aof << filterprecision( m_OneStep.finalCOMPosition.pitch[0]) << " "  ;// 3
      aof << filterprecision( m_OneStep.finalCOMPosition.yaw[0]) ;          // 4
      aof << endl ;
      aof.close();

      aFileName = m_TestName + ".waist" ;
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

      aFileName = m_TestName + ".zmp" ;
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
  }

  void SpecializedRobotConstructor(   CjrlHumanoidDynamicRobot *& aHDR, CjrlHumanoidDynamicRobot *& aDebugHDR )
  {
    aHDR = NULL ;
    aDebugHDR = NULL ;

#ifdef WITH_HRP2DYNAMICS
    dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
    Chrp2OptHumanoidDynamicRobot *aHRP2HDR = new Chrp2OptHumanoidDynamicRobot( &aRobotDynamicsObjectConstructor );
    aHDR = aHRP2HDR;
    aDebugHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
#endif
  }

  void startOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);

    {
      istringstream strm2(":SetAlgoForZmpTrajectory Naveau");
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
      istringstream strm2(":NaveauOnline");
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
    {
      istringstream strm2(":stepheight 0.05");
      m_PGI->ParseCmd(strm2);
    }

    {
      istringstream strm2(":deleteallobstacles");
      m_PGI->ParseCmd(strm2);
    }

    {
      istringstream strm2(":feedBackControl true");
      //istringstream strm2(":feedBackControl false");
      m_PGI->ParseCmd(strm2);
    }



//    {
//      istringstream strm2(":addoneobstacle 9.0 0.5 0.23");
//      m_PGI->ParseCmd(strm2);
//    }


//    {
//      istringstream strm2(":addoneobstacle 1.0 0.5 0.23");
//      m_PGI->ParseCmd(strm2);
//    }

//    {
//      istringstream strm2(":addoneobstacle 1.5 -0.0 0.23");
//      m_PGI->ParseCmd(strm2);
//    }


//    {
//      istringstream strm2(":updateoneobstacle 1 1.5 -1.5 0.23");
//      m_PGI->ParseCmd(strm2);
//    }
  }

  void startEmergencyStop(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);

    {
      istringstream strm2(":SetAlgoForZmpTrajectory Naveau");
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
      istringstream strm2(":NaveauOnline");
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
    {
      istringstream strm2(":stepheight 0.05");
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
      istringstream strm2(":setVelReference  0.15 0.0 -0.08");
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

  void walkOnSpot(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.1 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
  }

  void perturbationForce(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":perturbationforce  -20.0 8.0 0.0");
      aPGI.ParseCmd(strm2);
    }
  }

  void chooseTestProfile()
  {

    switch(m_TestProfile)
    {

      case PROFIL_NAVEAU_ONLINE_WALKING:
        startOnLineWalking(*m_PGI);
        break;
      default:
        throw("No correct test profile");
        break;
    }
  }


//  void generateEventOnLineWalking()
//  {
//    double dx=0.0 ;
//    double dy=0.0 ;
//    double dtheta=0.0 ;
//    if(m_OneStep.NbOfIt<dataFile_->data.size())
//    {
//      dx=dataFile_->data[m_OneStep.NbOfIt][1];
//      dy=dataFile_->data[m_OneStep.NbOfIt][2];
//      dtheta=dataFile_->data[m_OneStep.NbOfIt][3];
//    }else if(m_OneStep.NbOfIt > dataFile_->data.size() + 15*200)
//    {
//      istringstream strm3(":stoppg");
//      m_PGI->ParseCmd(strm3);
//    }

//    stringstream ss ;
//    ss << ":setVelReference " << dx << " " << dy << " " << dtheta ;
//    cout << ss.str() << endl ;
//    istringstream strm(ss.str());
//    m_PGI->ParseCmd(strm);
//  }

  void generateEventOnLineWalking()
  {
    #define localNbOfEvents 20
    struct localEvent events [localNbOfEvents] =
    {
      //{ 1,&TestNaveau2015::walkForward2m_s},
      { 1,&TestNaveau2015::walkOnSpot},
      //{10*200,&TestNaveau2015::walkForward2m_s},
      //{15*200,&TestNaveau2015::walkForward3m_s},
      //{1*20+5*200,&TestNaveau2015::perturbationForce},
      //{2*20+5*200,&TestNaveau2015::perturbationForce},
      //{3*20+5*200,&TestNaveau2015::perturbationForce},
      //{4*20+5*200,&TestNaveau2015::perturbationForce},
      //{5*20+5*200,&TestNaveau2015::perturbationForce},
      //{6*20+5*200,&TestNaveau2015::perturbationForce},
      //{7*20+5*200,&TestNaveau2015::perturbationForce},
      //{8*20+5*200,&TestNaveau2015::perturbationForce},
      //{9*20+5*200,&TestNaveau2015::perturbationForce},
      //{10*20+5*200,&TestNaveau2015::perturbationForce},
      {5*200,&TestNaveau2015::walkForward2m_s},
      {10*200,&TestNaveau2015::walkForward3m_s},
      {15*200,&TestNaveau2015::walkSidewards1m_s},
      {20*200,&TestNaveau2015::walkSidewards2m_s},
      {25*200,&TestNaveau2015::startTurningRight2},
      {30*200,&TestNaveau2015::stop},
      {35*200,&TestNaveau2015::stopOnLineWalking}
    };
    // Test when triggering event.
    for(unsigned int i=0;i<localNbOfEvents;i++)
    {
      if ( m_OneStep.NbOfIt==events[i].time)
      {
        ODEBUG3("********* GENERATE EVENT EMS ***********");
        (this->*(events[i].Handler))(*m_PGI);
      }
    }
//    if(m_OneStep.NbOfIt>=5*200)
//    {
//      ostringstream oss ;
//      oss << ":perturbationforce "
//          //<< 15*sin((m_OneStep.NbOfIt-5*200)*0.005)
//          << -20 << " "
//          << -4 << " "
//          << " 0.0";
//      cout << oss.str() << endl ;
//      istringstream strm (oss.str()) ;
//      m_PGI->ParseCmd(strm);
//    }
  }

  void generateEventEmergencyStop()
  {
#define localNbOfEventsEMS 4
    struct localEvent events [localNbOfEventsEMS] =
    { {5*200,&TestNaveau2015::startTurningLeft2},
    {10*200,&TestNaveau2015::startTurningRight2},
    {15*200,&TestNaveau2015::stop},
    {20*200,&TestNaveau2015::stopOnLineWalking}};
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
      case PROFIL_NAVEAU_ONLINE_WALKING:
        generateEventOnLineWalking();
        break;
      default:
        break;
    }
  }

};

int PerformTests(int argc, char *argv[])
{
#define NB_PROFILES 1
  std::string TestNames[NB_PROFILES] = {"TestNaveau2015"};
  int TestProfiles[NB_PROFILES] = {PROFIL_NAVEAU_ONLINE_WALKING};


  for (unsigned int i=0;i<NB_PROFILES;i++){
    TestNaveau2015 aTN2015(argc,argv,TestNames[i],TestProfiles[i]);
    aTN2015.init();
    try{
      if (!aTN2015.doTest(std::cout)){
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



//  IO_TextFile TextFileInput("example.txt");

//    // Test method
//    TextFileInput.WriteOnFile_AddText("test \n");
//    std::cout << TextFileInput.ReadFromFile();


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

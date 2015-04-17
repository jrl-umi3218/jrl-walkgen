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
#include <ZMPRefTrajectoryGeneration/DynamicFilter.hh>



using namespace::PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;
using namespace std;

class TestInverseKinematics: public TestObject
{

private:
  DynamicFilter* dynamicfilter_;
  SimplePluginManager * SPM_ ;
  double dInitX, dInitY;
  bool once ;
  MAL_VECTOR(InitialPosition,double);

  vector<COMState> comPos ;

  vector<FootAbsolutePosition> lfFoot ;
  vector<FootAbsolutePosition> rfFoot ;

  vector<ZMPPosition> zmp ;




public:
  TestInverseKinematics(int argc, char *argv[], string &aString):
      TestObject(argc,argv,aString)
  {
    SPM_ = NULL ;
    dynamicfilter_ = NULL ;
    once = true ;
    MAL_VECTOR_RESIZE(InitialPosition,36);
  };

  ~TestInverseKinematics()
  {
    if ( dynamicfilter_ != 0 )
    {
      delete dynamicfilter_ ;
      dynamicfilter_ = 0 ;
    }
    if ( SPM_ != 0 )
    {
      delete SPM_ ;
      SPM_ = 0 ;
    }

    m_DebugHDR = 0;
  }

  typedef void (TestInverseKinematics::* localeventHandler_t)(PatternGeneratorInterface &);

  struct localEvent
  {
    unsigned time;
    localeventHandler_t Handler ;
  };

  bool doTest(ostream &os)
  {
    readData();


    int stage0 = 0 ;
    int stage1 = 1 ;
    double samplingPeriod = 0.005 ;
    dynamicfilter_->init(
                          samplingPeriod,
                          samplingPeriod,
                          (double)(comPos.size()-320)*samplingPeriod,
                          1.6,
                          comPos[0] );


    vector<vector<double> > zmpmb ( comPos.size() , vector<double>(2) );
    for (unsigned int i= 0 ; i < comPos.size() ; ++i )
    {
      dynamicfilter_->ComputeZMPMB( samplingPeriod, comPos[i],
                                      lfFoot[i], rfFoot[i],
                                      zmpmb[i] , stage0 , i);
    }

    deque<ZMPPosition> inputdeltaZMP_deq(comPos.size()) ;
    deque<COMState> outputDeltaCOMTraj_deq ;
    for (unsigned int i = 0 ; i < comPos.size() ; ++i)
    {
      inputdeltaZMP_deq[i].px = zmp[i].px - zmpmb[i][0] ;
      inputdeltaZMP_deq[i].py = zmp[i].py - zmpmb[i][1] ;
      inputdeltaZMP_deq[i].pz = 0.0 ;
      inputdeltaZMP_deq[i].theta = 0.0 ;
      inputdeltaZMP_deq[i].time = i * samplingPeriod ;
      inputdeltaZMP_deq[i].stepType = zmp[i].stepType ;
    }
    dynamicfilter_->OptimalControl(inputdeltaZMP_deq,outputDeltaCOMTraj_deq) ;

    cout << outputDeltaCOMTraj_deq.size() << endl ;
    for (unsigned int i = 0 ; i < comPos.size()-320 ; ++i)
    {
      for(int j=0;j<3;j++)
      {
//        comPos[i].x[j] += outputDeltaCOMTraj_deq[i].x[j] ;
//        comPos[i].y[j] += outputDeltaCOMTraj_deq[i].y[j] ;
      }
    }

    for (unsigned int i = 0 ; i < comPos.size()-320 ; ++i)
    {
      dynamicfilter_->InverseKinematics(comPos[i],lfFoot[i],rfFoot[i],
                                        m_CurrentConfiguration, m_CurrentVelocity, m_CurrentAcceleration,
                                        samplingPeriod,stage1,i);
      m_OneStep.finalCOMPosition = comPos[i] ;

      m_OneStep.ZMPTarget(0) = zmp[i].px ;
      m_OneStep.ZMPTarget(1) = zmp[i].py ;

      m_OneStep.LeftFootPosition = lfFoot[i];
      m_OneStep.RightFootPosition = rfFoot[i];

      fillInDebugFiles();
    }
    return true ;
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

    // Creating the humanoid robot.
    SpecializedRobotConstructor(m_HDR);
    if(m_HDR==0)
    {
      if (m_HDR!=0) delete m_HDR;
      dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
      m_HDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
    }
    // Parsing the file.
    dynamicsJRLJapan::parseOpenHRPVRMLFile(*m_HDR,RobotFileName,
                                           m_LinkJointRank,
                                           m_SpecificitiesFileName);
    // Create Pattern Generator Interface
    m_PGI = patternGeneratorInterfaceFactory(m_HDR);

    unsigned int lNbActuatedJoints = 30;
    double * dInitPos = new double[lNbActuatedJoints];
    ifstream aif;
    aif.open(m_InitConfig.c_str(),ifstream::in);
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
    bool conversiontoradneeded=true;
    if (conversiontoradneeded)
      for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
        InitialPosition(i) = dInitPos[i]*M_PI/180.0;
    else
      for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
        InitialPosition(i) = dInitPos[i];

    // This is a vector corresponding to ALL the DOFS of the robot:
    // free flyer + actuated DOFS.
    unsigned int lNbDofs = 36 ;
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

    delete [] dInitPos;

    MAL_VECTOR_RESIZE(m_CurrentConfiguration, m_HDR->numberDof());
    MAL_VECTOR_RESIZE(m_CurrentVelocity, m_HDR->numberDof());
    MAL_VECTOR_RESIZE(m_CurrentAcceleration, m_HDR->numberDof());

    MAL_VECTOR_RESIZE(m_PreviousConfiguration, m_HDR->numberDof());
    MAL_VECTOR_RESIZE(m_PreviousVelocity, m_HDR->numberDof());
    MAL_VECTOR_RESIZE(m_PreviousAcceleration, m_HDR->numberDof());

    SPM_ = new SimplePluginManager();

    dynamicfilter_ = new DynamicFilter(SPM_,m_HDR);
    FootAbsolutePosition supportFoot ;
    if (m_OneStep.LeftFootPosition.stepType<0)
    {
      supportFoot = m_OneStep.LeftFootPosition ;
    }
    else{
      supportFoot = m_OneStep.RightFootPosition ;
    }
    double samplingPeriod = 0.005;
    dynamicfilter_->init(samplingPeriod,samplingPeriod,0.1,
                         1.6,m_OneStep.finalCOMPosition);
    initIK();
    MAL_VECTOR_TYPE(double) UpperConfig = m_HDR->currentConfiguration() ;
    MAL_VECTOR_TYPE(double) UpperVel = m_HDR->currentVelocity() ;
    MAL_VECTOR_TYPE(double) UpperAcc = m_HDR->currentAcceleration() ;
    dynamicfilter_->setRobotUpperPart(UpperConfig,UpperVel,UpperAcc);
    dynamicfilter_->InverseKinematics(
        m_OneStep.finalCOMPosition,
        m_OneStep.LeftFootPosition,
        m_OneStep.RightFootPosition,
        m_CurrentConfiguration,
        m_CurrentVelocity,
        m_CurrentAcceleration,
        0.005,1,0);
  }

protected:
  int readData(){
      std::string dataPath = "/home/mnaveau/devel/ros_sot/src/jrl/jrl-walkgen/tests/" ;
      std::string dataFile = dataPath + "NMPCpython.csv" ;
      std::ifstream dataStream ;
      dataStream.open(dataFile.c_str(),std::ifstream::in);

      COMState acomPos ;
      ZMPPosition azmp ;
      FootAbsolutePosition alf ;
      FootAbsolutePosition arf ;

      while (dataStream.good()) {
          double value ;          
          dataStream >> value ;
          acomPos.x[0]=value ;
          dataStream >> value ;
          acomPos.x[1]=value ;
          dataStream >> value ;
          acomPos.x[2]=value ;
          dataStream >> value ;
          acomPos.y[0]=value ;
          dataStream >> value ;
          acomPos.y[1]=value ;
          dataStream >> value ;
          acomPos.y[2]=value ;

          acomPos.z[0] = 0.814 ;
          acomPos.z[1] = 0.0 ;
          acomPos.z[2] = 0.0 ;

          acomPos.roll[0]=0.0 ;
          acomPos.roll[1]=0.0 ;
          acomPos.roll[2]=0.0 ;
          acomPos.pitch[0]=0.0 ;
          acomPos.pitch[1]=0.0 ;
          acomPos.pitch[2]=0.0 ;

          dataStream >> value ;
          acomPos.yaw[0]=value * 180.0 / M_PI;
          dataStream >> value ;
          acomPos.yaw[1]=value * 180.0 / M_PI;
          dataStream >> value ;
          acomPos.yaw[2]=value * 180.0 / M_PI;

          dataStream >> value ;
          azmp.px=value ;
          dataStream >> value ;
          azmp.py=value ;
          dataStream >> value ;
          azmp.pz=value ;

          dataStream >> value ;
          arf.x=value ;
          dataStream >> value ;
          arf.y=value ;
          dataStream >> value ;
          arf.z=value ;
          dataStream >> value ;
          arf.theta=value * 180.0 / M_PI;
          arf.omega=0.0;

          dataStream >> value ;
          alf.x=value ;
          dataStream >> value ;
          alf.y=value ;
          dataStream >> value ;
          alf.z=value ;
          dataStream >> value ;
          alf.theta=value * 180.0 / M_PI;
          alf.omega=0.0;

          comPos.push_back(acomPos);
          lfFoot.push_back(alf);
          rfFoot.push_back(arf);
          zmp.push_back(azmp);
      }

      dataStream.close();
      return 0;
  }

  void chooseTestProfile()
  {return;}
  void generateEvent()
  {return;}

  void SpecializedRobotConstructor(CjrlHumanoidDynamicRobot *& aHDR)
  {
    dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
    Chrp2OptHumanoidDynamicRobot *aHRP2HDR = new Chrp2OptHumanoidDynamicRobot( &aRobotDynamicsObjectConstructor );
    aHDR = aHRP2HDR;
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
    double samplingPeriod = 0.005 ;
    ComAndFootRealizationByGeometry * CaFR = dynamicfilter_->getComAndFootRealization() ;
    CaFR->SetHeightOfTheCoM(0.814);
    CaFR->setSamplingPeriod(samplingPeriod);
    CaFR->SetStepStackHandler(new StepStackHandler(SPM_));
    CaFR->Initialization();
    CaFR->InitializationCoM(BodyAngles,lStartingCOMState,
                            waist, m_OneStep.LeftFootPosition,
                            m_OneStep.RightFootPosition);
    CaFR->Initialization();
    CaFR->SetPreviousConfigurationStage0(m_HDR->currentConfiguration());
    CaFR->SetPreviousVelocityStage0(m_HDR->currentVelocity());
  }

  void fillInDebugFiles( )
  {
    /// \brief Create file .hip .pos .zmp
    /// --------------------
    ofstream aof;
    string aFileName;
    static int iteration = 0 ;

    if ( iteration == 0 ){
      aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
      aFileName+=m_TestName;
      aFileName+=".pos";
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
    ///----
    aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
      aFileName+=m_TestName;
      aFileName+=".pos";
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

    if ( iteration == 0 ){
      aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
      aFileName+=m_TestName;
      aFileName+=".hip";
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
    aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
      aFileName+=m_TestName;
      aFileName+=".hip";
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
      aof << filterprecision( iteration * 0.005 ) << " "  ; // 1
      aof << filterprecision( m_OneStep.finalCOMPosition.roll[0] * M_PI /180) << " "  ; // 2
      aof << filterprecision( m_OneStep.finalCOMPosition.pitch[0] * M_PI /180 ) << " "  ; // 3
      aof << filterprecision( m_OneStep.finalCOMPosition.yaw[0] * M_PI /180 ) ; // 4
      aof << endl ;
    aof.close();

    if ( iteration == 0 ){
      aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
      aFileName+=m_TestName;
      aFileName+=".zmp";
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }

    FootAbsolutePosition aSupportState;
    if (m_OneStep.LeftFootPosition.stepType < 0 )
      aSupportState = m_OneStep.LeftFootPosition ;
    else
      aSupportState = m_OneStep.RightFootPosition ;

    aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
      aFileName+=m_TestName;
      aFileName+=".zmp";
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
      aof << filterprecision( iteration * 0.005 ) << " "  ; // 1
      aof << filterprecision( m_OneStep.ZMPTarget(0) - m_CurrentConfiguration(0)) << " "  ; // 2
      aof << filterprecision( m_OneStep.ZMPTarget(1) - m_CurrentConfiguration(1) ) << " "  ; // 3
      aof << filterprecision( aSupportState.z  - m_CurrentConfiguration(2))  ; // 4
      aof << endl ;
    aof.close();

    aFileName = "/opt/grx3.0/HRP2LAAS/log/mnaveau/";
      aFileName+="footpos";
      aFileName+=".zmp";
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    aof << filterprecision( iteration * 0.005 ) << " "  ; // 1
      aof << filterprecision( lfFoot[iteration].x ) << " "  ; // 2
      aof << filterprecision( lfFoot[iteration].y ) << " "  ; // 3
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

  double filterprecision(double adb)
  {
    if (fabs(adb)<1e-7)
      return 0.0;

    double ladb2 = adb * 1e7;
    double lintadb2 = trunc(ladb2);
    return lintadb2/1e7;
  }
};

int PerformTests(int argc, char *argv[])
{
#define NB_PROFILES 1
  std::string TestNames = "TestInverseKinematics" ;

  TestInverseKinematics aTIK(argc,argv,TestNames);
  aTIK.init();
  try{
    if (!aTIK.doTest(std::cout)){
      cout << "Failed test " << endl;
      return -1;
    }
    else
      cout << "Passed test " << endl;
  }
  catch (const char * astr){
    cerr << "Failed on following error " << astr << std::endl;
    return -1; }

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



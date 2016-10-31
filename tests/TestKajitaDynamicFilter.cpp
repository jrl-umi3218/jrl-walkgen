/*
 * Copyright 2010,
 *
 * Maximilien naveau
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
/* \file This file tests Kajita's dynamic filter on a simple case
 */
#include "Debug.hh"
#include "CommonTools.hh"
#include "TestObject.hh"
#include <jrl/walkgen/pgtypes.hh>
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
  MAL_VECTOR(InitialConfiguration,double);
  MAL_VECTOR(InitialVelocity,double);
  MAL_VECTOR(InitialAcceleration,double);
  MAL_S3_VECTOR(lStartingCOMState,double);

  deque<COMState> delta_com ;

  vector<FootAbsolutePosition> lfFoot ;
  vector<FootAbsolutePosition> rfFoot ;

  deque<ZMPPosition> delta_zmp ;

public:
  TestInverseKinematics(int argc, char *argv[], string &aString):
    TestObject(argc,argv,aString)
  {
    SPM_ = NULL ;
    dynamicfilter_ = NULL ;
    once = true ;
    MAL_VECTOR_RESIZE(InitialPosition,36);
    MAL_VECTOR_RESIZE(InitialConfiguration,36);
    MAL_VECTOR_RESIZE(InitialVelocity,36);
    MAL_VECTOR_RESIZE(InitialAcceleration,36);
    MAL_VECTOR_FILL(InitialVelocity,36);
    MAL_VECTOR_FILL(InitialAcceleration,36);
  }

  ~TestInverseKinematics()
  {
    if ( dynamicfilter_ != 0 )
    {
      delete dynamicfilter_ ;
      dynamicfilter_ = 0 ;
    }
  }

  typedef void (TestInverseKinematics::* localeventHandler_t)(PatternGeneratorInterface &);

  struct localEvent
  {
    unsigned time;
    localeventHandler_t Handler ;
  };

  bool doTest(ostream &os)
  {
    os << "<===============================================================>"<<endl;
    os << "Initialization..." << endl;

    double endTime = 8.0 ;
    double previewWindowSize = 1.6 ;
    double samplingPeriod = 0.005 ;
    unsigned int N = (unsigned int)round(endTime/samplingPeriod);
    unsigned int Nctrl = (unsigned int)round((endTime - previewWindowSize)/samplingPeriod);
    COMState com_init ;
    com_init.z[0]=lStartingCOMState(2);
    dynamicfilter_->init( samplingPeriod,
                          samplingPeriod,
                          endTime - previewWindowSize,
                          endTime,
                          previewWindowSize,
                          com_init );

    os << "<===============================================================>"<<endl;
    os << "Filtering..." << endl;
    delta_zmp.resize(N);
    delta_com.resize(Nctrl);
    MAL_VECTOR_FILL(InitialVelocity,0.0);
    MAL_VECTOR_FILL(InitialAcceleration,0.0);
    MAL_S3_VECTOR_TYPE(double) zmpmb ;
    dynamicfilter_->zmpmb(m_CurrentConfiguration,InitialVelocity,InitialAcceleration,zmpmb);
    for (unsigned int i = 0 ; i < N ; ++i )
    {
      delta_zmp[i].px = 0.0-zmpmb[0];
      delta_zmp[i].py = 0.0-zmpmb[1];
    }
    cout << m_CurrentConfiguration << endl;
    cout << InitialVelocity << endl;
    cout << InitialAcceleration << endl;
    cout << zmpmb[0] << " " << zmpmb[1] << endl ;

    dynamicfilter_->OptimalControl(delta_zmp,delta_com);

    os << "<===============================================================>"<<endl;
    os << "Dumping..." << endl;
    /// \brief Create file .hip .pos .zmp
    /// --------------------
    ofstream aof;
    string aFileName;
    static int iteration = 0 ;
    aFileName = "./TestDynamicFilter32.dat";
    if ( iteration == 0 ){
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    for(unsigned int i = 0 ; i < delta_com.size() ; i++){
      aof << filterprecision( lStartingCOMState(0)+delta_com[i].x[0] ) << " "  ; // 1
      aof << filterprecision( lStartingCOMState(1)+delta_com[i].y[0] ) << " "  ; // 2
      aof << filterprecision( lStartingCOMState(0) ) << " "  ;                   // 3
      aof << filterprecision( delta_com[i].x[0] ) << " "  ;                      // 4
      aof << filterprecision( lStartingCOMState(1) ) << " "  ;                   // 5
      aof << filterprecision( delta_com[i].x[1] ) << " "  ;                      // 6
      aof << endl ;
    }

    aof.close();

    ++iteration;

    return true ;
  }

  void init()
  {
    TestObject::init();

    dynamicfilter_ = new DynamicFilter(m_SPM,m_PR);
    ComAndFootRealizationByGeometry * cfr =
        dynamicfilter_->getComAndFootRealization();
    cfr->setPinocchioRobot(m_PR);
    cfr->SetStepStackHandler(new StepStackHandler(m_SPM));
    cfr->SetHeightOfTheCoM(0.814);
    cfr->setSamplingPeriod(0.005);
    cfr->Initialization();
    MAL_VECTOR_DIM(waist,double,6);
    for (int i = 0 ; i < 6 ; ++i )
    {
      waist(i) = 0;
    }

    lStartingCOMState(0) = m_OneStep.finalCOMPosition.x[0] ;
    lStartingCOMState(1) = m_OneStep.finalCOMPosition.y[0] ;
    lStartingCOMState(2) = m_OneStep.finalCOMPosition.z[0] ;
    cfr->setSamplingPeriod(0.005);
    cfr->Initialization();
    cfr->InitializationCoM(m_HalfSitting,lStartingCOMState,
                           waist,
                           m_OneStep.LeftFootPosition, m_OneStep.RightFootPosition);
    m_CurrentConfiguration(0) = waist(0);
    m_CurrentConfiguration(1) = waist(1);
    m_CurrentConfiguration(2) = waist(2);
    m_OneStep.finalCOMPosition.x[0] = lStartingCOMState(0) ;
    m_OneStep.finalCOMPosition.y[0] = lStartingCOMState(1) ;
    m_OneStep.finalCOMPosition.z[0] = lStartingCOMState(2) ;

    MAL_VECTOR_TYPE(double) UpperConfig = m_CurrentConfiguration ;
    MAL_VECTOR_TYPE(double) UpperVel = m_CurrentVelocity ;
    MAL_VECTOR_TYPE(double) UpperAcc = m_CurrentAcceleration ;
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

  void chooseTestProfile()
  {return;}
  void generateEvent()
  {return;}

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



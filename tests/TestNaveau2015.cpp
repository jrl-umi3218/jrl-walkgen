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
#include <iomanip>
#include <string>
#include <map>
#include <cmath>

#include <jrl/walkgen/config_private.hh>
#include <jrl/walkgen/pgtypes.hh>
#include "Debug.hh"
#include "CommonTools.hh"
#include "TestObject.hh"
#include "MotionGeneration/ComAndFootRealizationByGeometry.hh"

#include "pinocchio/multibody/parser/urdf.hpp"
#include "pinocchio/multibody/parser/srdf.hpp"

using namespace std;
using namespace PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;

enum Profiles_t {
  PROFIL_NAVEAU // 1
};

// Class TestNaveau2015
class TestNaveau2015: public TestObject
{

private:
  int resetfiles ;
  bool m_DebugFGPIFull ;
  ComAndFootRealizationByGeometry * ComAndFootRealization_ ;
  SimplePluginManager * SPM_ ;
  vector<double> m_err_zmp_x, m_err_zmp_y;
  MAL_VECTOR_TYPE(double) m_conf;
  MAL_VECTOR_TYPE(double) m_vel ;
  MAL_VECTOR_TYPE(double) m_acc ;
  int iteration;
  std::vector<se3::JointIndex> m_leftLeg  ;
  std::vector<se3::JointIndex> m_rightLeg ;
  std::vector<se3::JointIndex> m_leftArm  ;
  std::vector<se3::JointIndex> m_rightArm ;

public:
  TestNaveau2015(int argc, char *argv[], string &aString, int TestProfile):
    TestObject(argc,argv,aString)
  {
    m_TestProfile = TestProfile;
    resetfiles=0;
    m_DebugFGPIFull=true;
    m_DebugFGPI=true;
    ComAndFootRealization_ = NULL;
    SPM_ = NULL ;
    m_err_zmp_x.clear();
    m_err_zmp_y.clear();
    iteration=0;
    m_leftLeg .clear();
    m_rightLeg.clear();
    m_leftArm .clear();
    m_rightArm.clear();
  }

  ~TestNaveau2015()
  {
    if(ComAndFootRealization_!=NULL)
    {
      delete ComAndFootRealization_ ;
      ComAndFootRealization_ = NULL;
    }
    if(SPM_!=NULL)
    {
      delete SPM_ ;
      SPM_ = NULL;
    }
  }

  typedef void (TestObject:: * localeventHandler_t)(PatternGeneratorInterface &);

  struct localEvent
  {
    unsigned time;
    localeventHandler_t Handler ;
  };

  bool doTest(std::ostream &os)
  {
    TestObject::doTest(os);
    ComputeAndDisplayZMPStatistic();
  }

  bool init()
  {
    if(!TestObject::init())
      return false;
    SPM_ = new SimplePluginManager();
    ComAndFootRealization_ = new ComAndFootRealizationByGeometry(
          (PatternGeneratorInterfacePrivate*)SPM_ );
    ComAndFootRealization_->setPinocchioRobot(m_PR);
    ComAndFootRealization_->SetStepStackHandler(new StepStackHandler(SPM_));
    ComAndFootRealization_->SetHeightOfTheCoM(0.814);
    ComAndFootRealization_->setSamplingPeriod(0.005);
    ComAndFootRealization_->Initialization();

    MAL_VECTOR_DIM(waist,double,6);
    for (int i = 0 ; i < 6 ; ++i )
    {
      waist(i) = 0;
    }
    MAL_S3_VECTOR(lStartingCOMState,double);

    lStartingCOMState(0) = m_OneStep.finalCOMPosition.x[0] ;
    lStartingCOMState(1) = m_OneStep.finalCOMPosition.y[0] ;
    lStartingCOMState(2) = m_OneStep.finalCOMPosition.z[0] ;
    ComAndFootRealization_->setSamplingPeriod(0.005);
    ComAndFootRealization_->Initialization();
    ComAndFootRealization_->InitializationCoM(m_HalfSitting,lStartingCOMState,
                                              waist,
                                              m_OneStep.LeftFootPosition, m_OneStep.RightFootPosition);
    m_conf = m_CurrentConfiguration ;
    m_vel  = m_CurrentVelocity ;
    m_acc  = m_CurrentAcceleration ;
    {
      istringstream strm2(":setfeetconstraint XY 0.09 0.04");
      m_PGI->ParseCmd(strm2);
    }
    m_leftLeg =
        m_PR->jointsBetween(m_PR->waist(),m_PR->leftFoot()->associatedAnkle);
    m_rightLeg =
        m_PR->jointsBetween(m_PR->waist(),m_PR->rightFoot()->associatedAnkle);
    m_leftArm =
        m_PR->jointsBetween(m_PR->chest(),m_PR->leftWrist());
    m_rightArm =
        m_PR->jointsBetween(m_PR->chest(),m_PR->rightWrist());

    m_leftLeg.erase( m_leftLeg.begin() );
    m_rightLeg.erase( m_rightLeg.begin() );

    se3::JointModelVector & ActuatedJoints = m_PR->getActuatedJoints();
    for(unsigned i=0 ; i <m_leftLeg.size() ; ++i)
      m_leftLeg[i] = se3::idx_q(ActuatedJoints[m_leftLeg[i]])-1;
    for(unsigned i=0 ; i <m_rightLeg.size() ; ++i)
      m_rightLeg[i] = se3::idx_q(ActuatedJoints[m_rightLeg[i]])-1;
    for(unsigned i=0 ; i <m_leftArm.size() ; ++i)
      m_leftArm[i] = se3::idx_q(ActuatedJoints[m_leftArm[i]])-1;
    for(unsigned i=0 ; i <m_rightArm.size() ; ++i)
      m_rightArm[i] = se3::idx_q(ActuatedJoints[m_rightArm[i]])-1;

    return true ;
  }

protected:

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
    ComputeStat(m_err_zmp_x,moy_delta_zmp_x,max_abs_err_x);
    cout << "average : " << moy_delta_zmp_x << endl ;
    cout << "maxx error : " << max_abs_err_x << endl ;

    cout << "Statistic for Dzmp in y : " << endl ;
    double moy_delta_zmp_y = 0.0 ;
    double max_abs_err_y = 0.0 ;
    ComputeStat(m_err_zmp_y,moy_delta_zmp_y,max_abs_err_y);
    cout << "average : " << moy_delta_zmp_y << endl ;
    cout << "maxx error : " << max_abs_err_y << endl ;
    return ;
  }

  void analyticalInverseKinematics(MAL_VECTOR_TYPE(double) & conf,
                                   MAL_VECTOR_TYPE(double) & vel,
                                   MAL_VECTOR_TYPE(double) & acc)
  {
    /// \brief calculate, from the CoM of computed by the preview control,
    ///    the corresponding articular position, velocity and acceleration
    /// ------------------------------------------------------------------
    MAL_VECTOR_DIM(aCOMState,double,6);
    MAL_VECTOR_DIM(aCOMSpeed,double,6);
    MAL_VECTOR_DIM(aCOMAcc,double,6);
    MAL_VECTOR_DIM(aLeftFootPosition,double,5);
    MAL_VECTOR_DIM(aRightFootPosition,double,5);

    aCOMState(0) = m_OneStep.finalCOMPosition.x[0];
    aCOMState(1) = m_OneStep.finalCOMPosition.y[0];
    aCOMState(2) = m_OneStep.finalCOMPosition.z[0];
    aCOMState(3) = m_OneStep.finalCOMPosition.roll[0]  * 180/M_PI ;
    aCOMState(4) = m_OneStep.finalCOMPosition.pitch[0] * 180/M_PI ;
    aCOMState(5) = m_OneStep.finalCOMPosition.yaw[0]   * 180/M_PI ;

    aCOMSpeed(0) = m_OneStep.finalCOMPosition.x[1];
    aCOMSpeed(1) = m_OneStep.finalCOMPosition.y[1];
    aCOMSpeed(2) = m_OneStep.finalCOMPosition.z[1];
    aCOMSpeed(3) = m_OneStep.finalCOMPosition.roll[1] /** * 180/M_PI  */ ;
    aCOMSpeed(4) = m_OneStep.finalCOMPosition.pitch[1]/** * 180/M_PI  */ ;
    aCOMSpeed(5) = m_OneStep.finalCOMPosition.yaw[1]/** * 180/M_PI  */ ;

    aCOMAcc(0) = m_OneStep.finalCOMPosition.x[2];
    aCOMAcc(1) = m_OneStep.finalCOMPosition.y[2];
    aCOMAcc(2) = m_OneStep.finalCOMPosition.z[2];
    aCOMAcc(3) = m_OneStep.finalCOMPosition.roll[2]/** * 180/M_PI  */ ;
    aCOMAcc(4) = m_OneStep.finalCOMPosition.pitch[2]/** * 180/M_PI  */ ;
    aCOMAcc(5) = m_OneStep.finalCOMPosition.yaw[2] /** * 180/M_PI  */;

    aLeftFootPosition(0) = m_OneStep.LeftFootPosition.x;
    aLeftFootPosition(1) = m_OneStep.LeftFootPosition.y;
    aLeftFootPosition(2) = m_OneStep.LeftFootPosition.z;
    aLeftFootPosition(3) = m_OneStep.LeftFootPosition.theta;
    aLeftFootPosition(4) = m_OneStep.LeftFootPosition.omega;

    aRightFootPosition(0) = m_OneStep.RightFootPosition.x;
    aRightFootPosition(1) = m_OneStep.RightFootPosition.y;
    aRightFootPosition(2) = m_OneStep.RightFootPosition.z;
    aRightFootPosition(3) = m_OneStep.RightFootPosition.theta;
    aRightFootPosition(4) = m_OneStep.RightFootPosition.omega;
    ComAndFootRealization_->setSamplingPeriod(0.005);
    ComAndFootRealization_->ComputePostureForGivenCoMAndFeetPosture(
          aCOMState, aCOMSpeed, aCOMAcc, aLeftFootPosition, aRightFootPosition,
          conf, vel, acc, iteration, 1);
    ++iteration;

    conf(m_leftArm.back()+1) = 10.0*M_PI/180.0;
    conf(m_rightArm.back()+1) = 10.0*M_PI/180.0;
  }

  MAL_VECTOR_TYPE(double) parseFromURDFtoOpenHRPIndex()
  {
    MAL_VECTOR_TYPE(double) conf = m_conf;
    MAL_VECTOR_FILL(conf,0.0);

    for(unsigned int i = 0 ; i < 6 ; i++)
      conf(i) = m_conf(i);
    unsigned index=6;
    //RLEG
    for(unsigned int i = 0 ; i < m_rightLeg.size() ; i++)
      conf(index+i) = m_conf(m_rightLeg[i]);
    index+=m_rightLeg.size();
    //LLEG
    for(unsigned int i = 0 ; i < m_leftLeg.size() ; i++)
      conf(index+i) = m_conf(m_leftLeg[i]);
    index+=m_leftLeg.size();
    //CHEST
    for(unsigned int i = 0 ; i < 2 ; i++)
      conf(index+i) = 0.0 ;
    index+= 2 ;
    //HEAD
    for(unsigned int i = 0 ; i < 2 ; i++)
      conf(index+i) = 0.0 ;
    index+= 2 ;
    //RARM
    for(unsigned int i = 0 ; i < m_rightArm.size() ; i++)
      conf(index+i) = m_HalfSitting(m_rightArm[i]-6);
    index+=m_rightArm.size();
    conf(index) = 10*M_PI/180;
    ++index;
    //LARM
    for(unsigned int i = 0 ; i < m_leftArm.size() ; i++)
      conf(index+i) = m_HalfSitting(m_leftArm[i]-6);
    index+=m_leftArm.size();
    conf(index) = 10*M_PI/180;

    return conf ;
  }

  void createOpenHRPFiles()
  {
    /// \brief Create file .hip .pos .zmp
    /// ---------------------------------
    MAL_VECTOR_TYPE(double) conf = parseFromURDFtoOpenHRPIndex();
    ofstream aof ;
    string aPosFileName = /*"/tmp/" +*/ m_TestName + ".pos" ;
    string aZMPFileName = /*"/tmp/" +*/ m_TestName + ".zmp" ;
    string aWaistFileName = /*"/tmp/" +*/ m_TestName + ".waist" ;
    string aHipFileName = /*"/tmp/" +*/ m_TestName + ".hip" ;
    if ( iteration == 1 )
    {
      aof.open(aPosFileName.c_str(),ofstream::out);
      aof.close();
      aof.open(aZMPFileName.c_str(),ofstream::out);
      aof.close();
      aof.open(aWaistFileName.c_str(),ofstream::out);
      aof.close();
      aof.open(aHipFileName.c_str(),ofstream::out);
      aof.close();
    }

    aof.open(aPosFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    aof << filterprecision( (double)(iteration-1) * 0.005 ) << " "  ;// 1
    for(unsigned i=6 ; i<MAL_VECTOR_SIZE(conf) ; ++i)
      aof << filterprecision( conf(i) ) << " "  ;                    // 2-30
    for(unsigned i=0 ; i<9 ; ++i)
      aof << filterprecision( 0.0 ) << " "  ;                        // 31-40
    aof << 0.0  << endl ;                                            // 41
    aof.close();

     aof.open(aWaistFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    aof << filterprecision( (double)(iteration-1) * 0.005 ) << " "  ;     // 1
    aof << filterprecision( m_OneStep.finalCOMPosition.roll[0]) << " "  ; // 2
    aof << filterprecision( m_OneStep.finalCOMPosition.pitch[0]) << " "  ;// 3
    aof << filterprecision( m_OneStep.finalCOMPosition.yaw[0]) ;          // 4
    aof << endl ;
    aof.close();


    aof.open(aHipFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    aof << filterprecision( (double)(iteration-1) * 0.005 ) << " "  ;     // 1
    aof << filterprecision( m_OneStep.finalCOMPosition.roll[0]) << " "  ; // 2
    aof << filterprecision( m_OneStep.finalCOMPosition.pitch[0]) << " "  ;// 3
    aof << filterprecision( m_OneStep.finalCOMPosition.yaw[0]) ;          // 4
    aof << endl ;
    aof.close();


    FootAbsolutePosition aSupportState;
    if (m_OneStep.LeftFootPosition.stepType < 0 )
      aSupportState = m_OneStep.LeftFootPosition ;
    else
      aSupportState = m_OneStep.RightFootPosition ;

    aof.open(aZMPFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    aof << filterprecision( (double)(iteration-1) * 0.005 ) << " "  ;   // 1
    aof << filterprecision( m_OneStep.ZMPTarget(0) - conf(0)) << " "  ; // 2
    aof << filterprecision( m_OneStep.ZMPTarget(1) - conf(1) ) << " "  ;// 3
    aof << filterprecision( aSupportState.z  - conf(2))  ;              // 4
    aof << endl ;
    aof.close();

    iteration++;
  }

  void prepareDebugFiles()
  {
    TestObject::prepareDebugFiles() ;
    if (m_DebugFGPIFull)
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
    TestObject::fillInDebugFiles();

    if(m_DebugFGPIFull)
    {

      analyticalInverseKinematics(m_conf,m_vel,m_acc);
      if(iteration==1)
      {
        bool isHalfsitting = true ;
        for(unsigned i=0 ; i<MAL_VECTOR_SIZE(m_HalfSitting) ; ++i)
        {
          isHalfsitting &= abs(m_conf(6+i)-m_HalfSitting(i))<1e-3 ;
//          if(!(abs(m_conf(6+i)-m_HalfSitting(i))<1e-3))
//            cout << i << " " ;
        }
//        cout << endl ;
//        cout << m_conf << endl ;
//        cout << m_HalfSitting << endl ;
//        cout << std::boolalpha << isHalfsitting << endl ;
        assert(isHalfsitting);
      }

      se3::JointModelVector & ActuatedJoints = m_DebugPR->getActuatedJoints();
      se3::JointIndex leftHand = se3::idx_q(
            ActuatedJoints[m_DebugPR->leftWrist()+1]);
      se3::JointIndex rightHand = se3::idx_q(
            ActuatedJoints[m_DebugPR->rightWrist()+1]);
//      m_conf(leftHand) = 0.174532925 ;     // gripper openning
//      m_conf(rightHand)= 0.174532925 ;     // gripper openning

      m_DebugPR->computeInverseDynamics(m_conf,m_vel,m_acc);
      createOpenHRPFiles();
      MAL_S3_VECTOR(zmpmb,double);
      m_DebugPR->zeroMomentumPoint(zmpmb);
      m_err_zmp_x.push_back(zmpmb[0]-m_OneStep.ZMPTarget(0)) ;
      m_err_zmp_y.push_back(zmpmb[1]-m_OneStep.ZMPTarget(1)) ;

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
    }
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

  void chooseTestProfile()
  {

    switch(m_TestProfile)
    {

      case PROFIL_NAVEAU:
        startOnLineWalking(*m_PGI);
        break;
      default:
        throw("No correct test profile");
        break;
    }
  }

  void generateEventOnLineWalking()
  {
    #define localNbOfEvents 20
    struct localEvent events [localNbOfEvents] =
    {
      //{ 1,&TestObject::walkForward2m_s},
      { 5,&TestObject::walkOnSpot},
      //{10*200,&TestObject::walkForward2m_s},
      //{15*200,&TestObject::walkForward3m_s},
      //{20*200,&TestObject::walkForward2m_s},
      //{25*200,&TestObject::walkForward2m_s},
      //{30*200,&TestObject::walkSidewards1m_s},
      //{35*200,&TestObject::walkSidewards2m_s},
      {20*200,&TestObject::stop},
      {30*200,&TestObject::stopOnLineWalking}
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
//    if(m_OneStep.NbOfIt>=0*200)
//    {
//      ostringstream oss ;
//      oss << ":perturbationforce "
//          << 0.0 << " "
//          << 0.0 << " "
//          << " 0.0";
//      istringstream strm (oss.str()) ;
//      m_PGI->ParseCmd(strm);
//    }
  }

  void generateEvent()
  {
    switch(m_TestProfile){
      case PROFIL_NAVEAU:
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
  int TestProfiles[NB_PROFILES] = {PROFIL_NAVEAU};


  for (unsigned int i=0;i<NB_PROFILES;i++){
    TestNaveau2015 aTN2015(argc,argv,TestNames[i],TestProfiles[i]);
    if(!aTN2015.init())
    {
      cout << "pb on init" << endl;
      return -1;
    }
    try
    {
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

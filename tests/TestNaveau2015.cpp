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

using namespace std;
using namespace PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;

enum Profiles_t {
  PROFIL_NAVEAU,       // 0
  PROFIL_SIMPLE_NAVEAU // 1
};

typedef void (TestObject:: * localeventHandler_t)(PatternGeneratorInterface &);

struct localEvent
{
  unsigned time;
  localeventHandler_t Handler ;
};

struct setOfLocalEvents
{
  std::vector<localEvent> m_vecOfLocalEvents;
  PatternGeneratorInterface *m_PGI;
  
  setOfLocalEvents(PatternGeneratorInterface *aPGI)
  { m_PGI = aPGI;} 
    
  void initVecOfLocalEvents( struct localEvent * events,
			     unsigned int sizeOfEvents)
  {
    for(unsigned int i=0;i<sizeOfEvents;i++)
      {
	m_vecOfLocalEvents.push_back(events[i]);
      }
  }
  
  void evaluateEvents(struct OneStep &oneStep,
		      TestObject &aTestObject)
  {
    // Test when triggering event.
    for(unsigned int i=0;i<m_vecOfLocalEvents.size();i++)
      {
	if ( oneStep.NbOfIt==m_vecOfLocalEvents[i].time)
	  {
	    ODEBUG3("********* GENERATE EVENT EMS ***********");
	    (aTestObject.*(m_vecOfLocalEvents[i].Handler))(*m_PGI);
	  }
      }
  }
};

// Class TestNaveau2015
class TestNaveau2015: public TestObject
{

private:
  int resetfiles ;
  bool m_DebugFGPIFull ;
  vector<double> m_err_zmp_x, m_err_zmp_y;

  int iteration;
  std::vector<pinocchio::JointIndex> m_leftLeg  ;
  std::vector<pinocchio::JointIndex> m_rightLeg ;
  std::vector<pinocchio::JointIndex> m_leftArm  ;
  std::vector<pinocchio::JointIndex> m_rightArm ;
  pinocchio::JointIndex m_leftGripper  ;
  pinocchio::JointIndex m_rightGripper ;

  /// Object to generate events according to profile.
  setOfLocalEvents * m_setOfLocalEvents;
  std::vector<double> m_prevCoMp;
  std::vector<double> m_prevdCoMp;
  std::vector<double> m_prevddCoMp;
  std::vector<double> m_prevWaistOrien;  
  
  std::vector<double> m_prevLeftFootPos,
    m_prevLeftFootdPos,
    m_prevLeftFootddPos,
    m_prevLeftFootOrientation,
    m_prevLeftFootdOrientation,
    m_prevLeftFootddOrientation;
  
  std::vector<double> m_prevRightFootPos,
    m_prevRightFootdPos,
    m_prevRightFootddPos,
    m_prevRightFootOrientation,
    m_prevRightFootdOrientation,
    m_prevRightFootddOrientation;

  std::vector<double> m_prevZMPMB, m_prevZMPRef;
  
public:
  TestNaveau2015(int argc, char *argv[], string &aString, int TestProfile):
    TestObject(argc,argv,aString)
  {
    m_TestProfile = TestProfile;
    resetfiles=0;
    m_DebugFGPIFull=true;
    m_DebugFGPI=true;
    m_err_zmp_x.clear();
    m_err_zmp_y.clear();
    iteration=0;
    m_leftLeg .clear();
    m_rightLeg.clear();
    m_leftArm .clear();
    m_rightArm.clear();
    m_leftGripper  = 0 ;
    m_rightGripper = 0 ;
    m_setOfLocalEvents = 0;

    m_prevCoMp.resize(3);
    m_prevdCoMp.resize(3);
    m_prevddCoMp.resize(3);
    m_prevWaistOrien.resize(3);
    
    m_prevLeftFootPos.resize(3);
    m_prevLeftFootdPos.resize(3);
    m_prevLeftFootddPos.resize(3);
    m_prevLeftFootOrientation.resize(3);
    m_prevLeftFootdOrientation.resize(3);
    m_prevLeftFootddOrientation.resize(3);

    m_prevRightFootPos.resize(3);
    m_prevRightFootdPos.resize(3);
    m_prevRightFootddPos.resize(3);
    m_prevRightFootOrientation.resize(3);
    m_prevRightFootdOrientation.resize(3);
    m_prevRightFootddOrientation.resize(3);

    m_prevZMPMB.resize(3);
    m_prevZMPRef.resize(3);
  }



  bool doTest(std::ostream &os)
  {
    bool ret = TestObject::doTest(os);
    if(m_DebugFGPIFull)
      ComputeAndDisplayZMPStatistic();
    return ret ;
  }

  bool init()
  {
    if(!TestObject::init())
      return false;

    /// Initialize Configuration Velocity and Acceleration
    m_CurrentVelocity.setZero();
    m_CurrentAcceleration.setZero();
    m_setOfLocalEvents = new setOfLocalEvents(m_PGI);

    ODEBUG3("m_vel" << m_CurrentVelocity);
    ODEBUG3("m_acc" << m_CurrentAcceleration);
    if (m_PR->getName()=="hrp2_14_reduced")
      m_ComAndFootRealization->SetHeightOfTheCoM(0.814);
    else if (m_PR->getName()=="talos")
      m_ComAndFootRealization->SetHeightOfTheCoM(0.876681);
    

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

    pinocchio::JointModelVector & ActuatedJoints = m_PR->getActuatedJoints();
    for(unsigned i=0 ; i <m_leftLeg.size() ; ++i)
      m_leftLeg[i] = pinocchio::idx_q(ActuatedJoints[m_leftLeg[i]])-1;
    for(unsigned i=0 ; i <m_rightLeg.size() ; ++i)
      m_rightLeg[i] = pinocchio::idx_q(ActuatedJoints[m_rightLeg[i]])-1;
    for(unsigned i=0 ; i <m_leftArm.size() ; ++i)
      m_leftArm[i] = pinocchio::idx_q(ActuatedJoints[m_leftArm[i]])-1;
    for(unsigned i=0 ; i <m_rightArm.size() ; ++i)
      m_rightArm[i] = pinocchio::idx_q(ActuatedJoints[m_rightArm[i]])-1;

    if((m_robotModel.parents.size() >= m_rightArm.back()+1) &&
       m_robotModel.parents[m_rightArm.back()+1] == m_rightArm.back())
      m_rightGripper = m_rightArm.back()+1 ;
    else
      m_rightGripper = 0 ;

    if((m_robotModel.parents.size() >= m_leftArm.back()+1) &&
       m_robotModel.parents[m_leftArm.back()+1] == m_leftArm.back())
      m_leftGripper = m_leftArm.back()+1 ;
    else
      m_leftGripper = 0 ;

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
    avg = total/(double)(vec.size());
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

  void analyticalInverseKinematics(Eigen::VectorXd & conf,
                                   Eigen::VectorXd & vel,
                                   Eigen::VectorXd & acc)
  {
    /// \brief calculate, from the CoM of computed by the preview control,
    ///    the corresponding articular position, velocity and acceleration
    /// ------------------------------------------------------------------
    Eigen::VectorXd aCOMState(6);
    Eigen::VectorXd aCOMSpeed(6);
    Eigen::VectorXd aCOMAcc(6);
    Eigen::VectorXd aLeftFootPosition(5);
    Eigen::VectorXd aRightFootPosition(5);

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
    m_ComAndFootRealization->setSamplingPeriod(0.005);
    m_ComAndFootRealization->ComputePostureForGivenCoMAndFeetPosture(
								     aCOMState, aCOMSpeed, aCOMAcc, aLeftFootPosition, aRightFootPosition,
								     conf, vel, acc, iteration, 1);

    if(m_leftGripper!=0 && m_rightGripper!=0)
      {
	conf(m_leftGripper) = 10.0*M_PI/180.0;
	conf(m_rightGripper) = 10.0*M_PI/180.0;
      }
  }

  Eigen::VectorXd parseFromURDFtoOpenHRPIndex()
  {
    Eigen::VectorXd conf = m_conf;
    { for(unsigned int i=0;i<conf.size();conf[i++]=0.0);};

    for(unsigned int i = 0 ; i < 6 ; i++)
      conf(i) = m_conf(i);
    std::size_t index=6;
    //RLEG
    for(unsigned int i = 0 ; i < m_rightLeg.size() ; i++)
      conf(index+i) = m_conf(m_rightLeg[i]);
    index+=(unsigned)m_rightLeg.size();
    //LLEG
    for(unsigned int i = 0 ; i < m_leftLeg.size() ; i++)
      conf(index+i) = m_conf(m_leftLeg[i]);
    index+=(unsigned)m_leftLeg.size();
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
    index+=(unsigned)m_rightArm.size();
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
    Eigen::VectorXd conf = parseFromURDFtoOpenHRPIndex();
    ofstream aof ;
    string aPosFileName = /*"/tmp/" +*/ m_TestName + ".pos" ;
    string aZMPFileName = /*"/tmp/" +*/ m_TestName + ".zmp" ;
    string aWaistFileName = /*"/tmp/" +*/ m_TestName + ".waist" ;
    string aHipFileName = /*"/tmp/" +*/ m_TestName + ".hip" ;
    if ( iteration == 0 )
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
    aof << filterprecision( (double)iteration * 0.005 ) << " "  ;// 1
    for(unsigned i=6 ; i<conf.size() ; ++i)
      aof << filterprecision( conf(i) ) << " "  ;                    // 2-30
    for(unsigned i=0 ; i<9 ; ++i)
      aof << filterprecision( 0.0 ) << " "  ;                        // 31-40
    aof << 0.0  << endl ;                                            // 41
    aof.close();

    aof.open(aWaistFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    aof << filterprecision( (double)iteration * 0.005 ) << " "  ;     // 1
    aof << filterprecision( m_OneStep.finalCOMPosition.roll[0]) << " "  ; // 2
    aof << filterprecision( m_OneStep.finalCOMPosition.pitch[0]) << " "  ;// 3
    aof << filterprecision( m_OneStep.finalCOMPosition.yaw[0]) ;          // 4
    aof << endl ;
    aof.close();


    aof.open(aHipFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    aof << filterprecision( (double)iteration * 0.005 ) << " "  ;     // 1
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
    aof << filterprecision( (double)iteration * 0.005 ) << " "  ;   // 1
    aof << filterprecision( m_OneStep.ZMPTarget(0) - conf(0)) << " "  ; // 2
    aof << filterprecision( m_OneStep.ZMPTarget(1) - conf(1) ) << " "  ;// 3
    aof << filterprecision( aSupportState.z  - conf(2))  ;              // 4
    aof << endl ;
    aof.close();
  }

  void prepareDebugFiles()
  {
    TestObject::prepareDebugFiles() ;
    if (m_DebugFGPIFull)
      {
	ofstream aof;
	string aFileName;

	aFileName = m_TestName;	  
        aFileName += "TestFGPIFull_description.dat";

        aof.open(aFileName.c_str(),ofstream::out);

        string Titles[51] =
	  { "Time",                          // 1
	    "Com X",                         // 2
	    "Com Y" ,                        // 3
	    "Com Z" ,                        // 4
	    "Com Yaw",                       // 5
	    "Com dX" ,                       // 6
	    "Com dY" ,                       // 7
	    "Com dZ" ,                       // 8
	    "Com dYaw",	                     // 9
	    "Com ddX" ,                      // 10
	    "Com ddY" ,                      // 11
	    "Com ddZ" ,                      // 12
	    "Com ddYaw",                     // 13 
	    "ZMP X (world ref.)" ,           // 14
	    "ZMP Y (world ref.)" ,           // 15
	    "ZMP Z (world ref.)" ,	     // 16
	    "Left Foot X" ,                  // 17
	    "Left Foot Y" ,                  // 18
	    "Left Foot Z" ,                  // 19
	    "Left Foot dX" ,                 // 20
	    "Left Foot dY" ,                 // 21
	    "Left Foot dZ" ,                 // 22
	    "Left Foot ddX" ,                // 23
	    "Left Foot ddY" ,                // 24
	    "Left Foot ddZ" ,                // 25
	    "Left Foot Theta" ,              // 26
	    "Left Foot dTheta" ,             // 27
	    "Left Foot ddTheta" ,	     // 28
	    "Left Foot Omega" ,              // 29
	    "Left Foot Omega2" ,             // 30
	    "Right Foot X" ,                 // 31      
	    "Right Foot Y" ,                 // 32
	    "Right Foot Z" ,                 // 33
	    "Right Foot dX" ,                // 34
	    "Right Foot dY" ,                // 35
	    "Right Foot dZ" ,                // 36
	    "Right Foot ddX" ,               // 37 
	    "Right Foot ddY" ,               // 38
	    "Right Foot ddZ" ,               // 39
	    "Right Foot Theta" ,             // 40
	    "Right Foot dTheta" ,            // 41
	    "Right Foot ddTheta" ,	     // 42
	    "Right Foot Omega" ,             // 43
	    "Right Foot Omega2" ,            // 44
	    "ZMP MB X " ,                    // 45
	    "ZMP MB Y " ,                    // 46
	    "ZMP MB Z " ,	             // 47
	    "q " ,                           // 48 
	    "dq" ,                           // 48 + nq
	    "ddq "};                         // 48 + nq + nv
        for(unsigned int i=0;i<51;i++)
          aof << i+1 << ". " <<Titles[i] <<std::endl;
	
        aof.close();

	aFileName = m_TestName;
	aFileName += "TestFGPIFull.dat";
	if (m_OneStep.NbOfIt==1)
	  {
	    aof.open(aFileName.c_str(),ofstream::out);
	  }

      }
  }

  void prepareFile(ofstream &aof,string &prefix)
  {
    string aFileName;
    aFileName = prefix;
    aFileName += ".dat";
    if (m_OneStep.NbOfIt==1)
      {
	aof.open(aFileName.c_str(),ofstream::out);
      }
    resetfiles = 1;
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);

  }

  void fillFileWithSubsamplingAndClose
  (ofstream &aof,
   std::vector<double> &prev,
   std::vector<double> &next,
   double nb_subsampling=5.0)
  {
    for(double subsampling=1.0;
	subsampling <= nb_subsampling;
	subsampling+=1.0)
      {
	//	aof << filterprecision(((double)m_OneStep.NbOfIt)*dt
	//		       + subsampling*dt/nb_subsampling ) << " ";   // 1
	for(unsigned int i=0;
	    i<next.size();
	    i++)
	  {
	    double intermediate = prev[i] +
	      (next[i]-prev[i])*subsampling/nb_subsampling ;
	    aof << intermediate << " ";
	  }
	aof << std::endl;
      }
    aof.close();
    prev = next;
  }

  void fillInTests()
  {
    ofstream aof;
    string prefix= m_TestName + "CoM";
    std::vector<double> vec_db;

    /// CoM Position
    prepareFile(aof,prefix);
    vec_db.resize(3);
    vec_db[0] = filterprecision(m_OneStep.finalCOMPosition.x[0] );
    vec_db[1] = filterprecision(m_OneStep.finalCOMPosition.y[0] );
    vec_db[2] = filterprecision(m_OneStep.finalCOMPosition.z[0] );
    fillFileWithSubsamplingAndClose(aof,m_prevCoMp,vec_db);

    /// CoM velocity
    prefix= m_TestName + "dCoM";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.finalCOMPosition.x[1] );    // 6
    vec_db[1] = filterprecision(m_OneStep.finalCOMPosition.y[1] );    // 7
    vec_db[2] = filterprecision(m_OneStep.finalCOMPosition.z[1] );    // 8
    fillFileWithSubsamplingAndClose(aof,m_prevdCoMp,vec_db);

    /// CoM acceleration
    prefix= m_TestName + "ddCoM";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.finalCOMPosition.x[2] );    // 10
    vec_db[1] =  filterprecision(m_OneStep.finalCOMPosition.y[2] );    // 11
    vec_db[2] = filterprecision(m_OneStep.finalCOMPosition.z[2] );    // 12
    fillFileWithSubsamplingAndClose(aof,m_prevddCoMp,vec_db);    
      
    /// Waist Orientation
    prefix= m_TestName + "WaistOrientation";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.finalCOMPosition.yaw[0] );  // 5
    vec_db[1] = filterprecision(m_OneStep.finalCOMPosition.yaw[1] );  // 9
    vec_db[2] = filterprecision(m_OneStep.finalCOMPosition.yaw[2] );  // 13
    fillFileWithSubsamplingAndClose(aof,m_prevWaistOrien,vec_db);    
    
    /// ZMP Ref
    prefix= m_TestName + "ZMPRef";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.ZMPTarget(0) );             // 14
    vec_db[1] =  filterprecision(m_OneStep.ZMPTarget(1) );             // 15
    vec_db[2] =  filterprecision(m_OneStep.ZMPTarget(2) );           // 16
    fillFileWithSubsamplingAndClose(aof,m_prevZMPRef,vec_db);

    prefix= m_TestName + "LeftFootPos";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.LeftFootPosition.x  );      // 17
    vec_db[1] = filterprecision(m_OneStep.LeftFootPosition.y  );      // 18
    vec_db[2] = filterprecision(m_OneStep.LeftFootPosition.z  );      // 19
    fillFileWithSubsamplingAndClose(aof,m_prevLeftFootPos,vec_db);    
    
    prefix= m_TestName + "LeftFootdPos";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.LeftFootPosition.dx  );     // 20
    vec_db[1] =  filterprecision(m_OneStep.LeftFootPosition.dy  );     // 21
    vec_db[2] =filterprecision(m_OneStep.LeftFootPosition.dz  );   // 22
    fillFileWithSubsamplingAndClose(aof,m_prevLeftFootdPos,vec_db);
    
    prefix= m_TestName + "LeftFootddPos";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.LeftFootPosition.ddx  );    // 23
    vec_db[1] =  filterprecision(m_OneStep.LeftFootPosition.ddy  );    // 24
    vec_db[2] =filterprecision(m_OneStep.LeftFootPosition.ddz  );    // 25
    fillFileWithSubsamplingAndClose(aof,m_prevLeftFootddPos,vec_db);
    
    prefix= m_TestName + "LeftFootOrientation";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.LeftFootPosition.theta );   // 26
    vec_db[1] = filterprecision(m_OneStep.LeftFootPosition.omega  );  // 29
    vec_db[2] =filterprecision(m_OneStep.LeftFootPosition.omega2  ); // 30
    fillFileWithSubsamplingAndClose(aof,m_prevLeftFootOrientation,vec_db);

    prefix= m_TestName + "LeftFootdOrientation";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.LeftFootPosition.dtheta );
    vec_db[1] = 0.0; vec_db[2] =  0.0;  // 27
    fillFileWithSubsamplingAndClose(aof,m_prevLeftFootdOrientation,vec_db);

    prefix= m_TestName + "LeftFootddOrientation";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.LeftFootPosition.ddtheta );
    vec_db[1] = 0.0; vec_db[2] =  0.0;  // 27
    fillFileWithSubsamplingAndClose(aof,m_prevLeftFootddOrientation,vec_db);    
    
    prefix= m_TestName + "RightFootPos";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.RightFootPosition.x  );
    vec_db[1] = filterprecision(m_OneStep.RightFootPosition.y  );
    vec_db[2] = filterprecision(m_OneStep.RightFootPosition.z  );
    fillFileWithSubsamplingAndClose(aof,m_prevRightFootPos,vec_db);    
    
    prefix= m_TestName + "RightFootdPos";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.RightFootPosition.dx  );
    vec_db[1] = filterprecision(m_OneStep.RightFootPosition.dy  );
    vec_db[2] = filterprecision(m_OneStep.RightFootPosition.dz  );
    fillFileWithSubsamplingAndClose(aof,m_prevRightFootdPos,vec_db);        
    
    prefix= m_TestName + "RightFootddPos";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.RightFootPosition.ddx  );    
    vec_db[1] = filterprecision(m_OneStep.RightFootPosition.ddy  );    
    vec_db[2] = filterprecision(m_OneStep.RightFootPosition.ddz  );    
    fillFileWithSubsamplingAndClose(aof,m_prevRightFootddPos,vec_db);
    
    prefix= m_TestName + "RightFootOrientation";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.RightFootPosition.theta );   
    vec_db[1] = filterprecision(m_OneStep.RightFootPosition.omega  );  
    vec_db[2] = filterprecision(m_OneStep.RightFootPosition.omega2  ); 
    fillFileWithSubsamplingAndClose(aof,m_prevRightFootOrientation,vec_db);

    prefix= m_TestName + "RightFootdOrientation";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.RightFootPosition.dtheta );
    vec_db[1] = 0.0; vec_db[2] = 0.0; 
    fillFileWithSubsamplingAndClose(aof,m_prevRightFootdOrientation,vec_db);

    prefix= m_TestName + "RightFootddOrientation";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(m_OneStep.RightFootPosition.ddtheta );
    vec_db[1] = 0.0; vec_db[2] = 0.0;
    fillFileWithSubsamplingAndClose(aof,m_prevRightFootddOrientation,vec_db);    

    analyticalInverseKinematics(m_CurrentConfiguration,
				m_CurrentVelocity,
				m_CurrentAcceleration);
    m_DebugPR->computeInverseDynamics(m_CurrentConfiguration,
				      m_CurrentVelocity,
				      m_CurrentAcceleration);
    Eigen::Vector3d zmpmb;
    m_DebugPR->zeroMomentumPoint(zmpmb);

    prefix= m_TestName + "ZMPMB";
    prepareFile(aof,prefix);
    vec_db[0] = filterprecision(zmpmb[0]);
    vec_db[1] = filterprecision(zmpmb[1]);
    vec_db[2] = filterprecision(zmpmb[2]);
    fillFileWithSubsamplingAndClose(aof,m_prevZMPMB,vec_db);    


  }
  
  void fillInDebugFiles()
  {
    TestObject::fillInDebugFiles();

    fillInTests();
    if(m_DebugFGPIFull)
      {
	analyticalInverseKinematics(m_CurrentConfiguration,
				    m_CurrentVelocity,
				    m_CurrentAcceleration);
	if(iteration==0)
	  {

	    //        cout << endl ;
	    //        cout << m_conf << endl ;
	    //        cout << m_HalfSitting << endl ;
	    //        cout << std::boolalpha << isHalfsitting << endl ;
	    //        assert(isHalfsitting);
	  }

	m_DebugPR->computeInverseDynamics(m_CurrentConfiguration,
					  m_CurrentVelocity,
					  m_CurrentAcceleration);
	Eigen::Vector3d com, dcom, ddcom;
	m_DebugPR->CenterOfMass(com, dcom, ddcom);
	createOpenHRPFiles();
	Eigen::Vector3d zmpmb;
	m_DebugPR->zeroMomentumPoint(zmpmb);
	m_err_zmp_x.push_back(zmpmb[0]-m_OneStep.ZMPTarget(0)) ;
	m_err_zmp_y.push_back(zmpmb[1]-m_OneStep.ZMPTarget(1)) ;

	++iteration;

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
	aof << filterprecision(((double)m_OneStep.NbOfIt)*0.005 ) << " "   // 1
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
	for(unsigned int k = 0 ; k < m_conf.size() ; k++){ // 48-53 -> 54-83
	  aof << filterprecision( m_conf(k) ) << " "  ;
	}
	for(unsigned int k = 0 ; k < m_vel.size() ; k++){ // 84-89 -> 90-118
	  aof << filterprecision( m_vel(k) ) << " "  ;
	}
	for(unsigned int k = 0 ; k < m_acc.size() ; k++){ // 119-125 -> 125-155
	  aof << filterprecision( m_acc(k) ) << " "  ;
	}
	aof << endl;
	aof.close();
      }
  }

  void createFullEventsForHRP2()
  {
    ODEBUG3("createFullEventsForHRP2");
    localEvent events[8] =
      {
	{1*200,&TestObject::walkForwardSlow},
	{10*200,&TestObject::walkForward2m_s},
	{20*200,&TestObject::walkSidewards2m_s},
	{30*200,&TestObject::walkX05Y04},
	{40*200,&TestObject::startTurningRight2},
	{50*200,&TestObject::walkOnSpot},
	{66*200,&TestObject::stop},
	{76*200,&TestObject::stopOnLineWalking}
      };

    if (m_setOfLocalEvents!=0)
      m_setOfLocalEvents->initVecOfLocalEvents(events,8);
  }

  void createSimpleEventsForHRP2()
  {
    localEvent events[2] =
      {
	{1*200,&TestObject::walkOnSpot},	
	{10*200,&TestObject::stop},
      };
    
    if (m_setOfLocalEvents!=0)
      m_setOfLocalEvents->initVecOfLocalEvents(events,2);
  }

  
  
  void startHRP2OnLineWalking(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);

    {
      istringstream strm2(":SetAlgoForZmpTrajectory Naveau");
      aPGI.ParseCmd(strm2);

    }
    {
      //istringstream strm2(":singlesupporttime 1.4");
      istringstream strm2(":singlesupporttime 0.7");
      aPGI.ParseCmd(strm2);
    }
    {
      //istringstream strm2(":doublesupporttime 0.2");
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
      istringstream strm2(":setfeetconstraint XY 0.095 0.055");
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
      //istringstream strm2(":feedBackControl true");
      istringstream strm2(":feedBackControl false");
      m_PGI->ParseCmd(strm2);
    }

    {
      //istringstream strm2(":useDynamicFilter true");
      istringstream strm2(":useDynamicFilter false");
      m_PGI->ParseCmd(strm2);
    }

  }

  void startTalosOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);

    {
      istringstream strm2(":setDSFeetDistance 0.162");
      m_PGI->ParseCmd(strm2);
    }
    
    {
      istringstream strm2(":SetAlgoForZmpTrajectory Naveau");
      aPGI.ParseCmd(strm2);

    }
    {
      //istringstream strm2(":singlesupporttime 1.4");
      istringstream strm2(":singlesupporttime 0.7");
      aPGI.ParseCmd(strm2);
    }
    {
      //istringstream strm2(":doublesupporttime 0.2");
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
      istringstream strm2(":setfeetconstraint XY 0.091 0.0489");
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
      //istringstream strm2(":feedBackControl true");
      istringstream strm2(":feedBackControl false");
      m_PGI->ParseCmd(strm2);
    }

    {
      //istringstream strm2(":useDynamicFilter true");
      istringstream strm2(":useDynamicFilter false");
      m_PGI->ParseCmd(strm2);
    }


  }

  void chooseTestProfile()
  {
    ODEBUG3("ROBOT:" << m_PR->getName() <<
	    " Profile: " << m_TestProfile);
    switch(m_TestProfile)
      {
      
      case PROFIL_NAVEAU:
	createFullEventsForHRP2();
	if (m_PR->getName()=="hrp2_14_reduced")
	  startHRP2OnLineWalking(*m_PGI);
	else if (m_PR->getName()=="talos")
	  startTalosOnLineWalking(*m_PGI);
	else
	  throw("No valid robot");
        break;
	
      case PROFIL_SIMPLE_NAVEAU:
	createSimpleEventsForHRP2();
	if (m_PR->getName()=="hrp2_14_reduced")
	  startHRP2OnLineWalking(*m_PGI);
	else if (m_PR->getName()=="talos")
	  startTalosOnLineWalking(*m_PGI);
	else
	  throw("No valid robot");
        break;
	
      default:
        throw("No correct test profile");
        break;
      }
  }

  void generateEventOnLineWalking()
  {
    if (m_setOfLocalEvents!=0)
      m_setOfLocalEvents->evaluateEvents(m_OneStep,*this);
  }

  void generateEvent()
  {
    switch(m_TestProfile)
      {
      case PROFIL_NAVEAU:
        generateEventOnLineWalking();
        break;
      case PROFIL_SIMPLE_NAVEAU:
	generateEventOnLineWalking();
      default:
        break;
      }
  }

};

int PerformTests(int argc, char *argv[])
{
#define NB_PROFILES 2
  std::string CompleteName = string(argv[0]);
  std::size_t found = CompleteName.find_last_of("/\\");
  std::string TestName =  CompleteName.substr(found+1);

  
  int TestProfiles[NB_PROFILES] =
    {
      PROFIL_NAVEAU,
      PROFIL_SIMPLE_NAVEAU
    };

  int indexProfile=-1;
  
  if (TestName.compare(14,6,"Online")==0)
    indexProfile=PROFIL_NAVEAU;
  if (TestName.compare(14,12,"OnlineSimple")==0)
    indexProfile=PROFIL_SIMPLE_NAVEAU;

  if (indexProfile==-1)
    {
      std::cerr << "CompleteName: " << CompleteName << std::endl;
      std::cerr<< " TestName: " << TestName <<std::endl;
      std::cerr<< "Failure to find the proper indexFile:" << TestName.substr(14,6) << endl;
      exit(-1);
    }
  else
    { std::cout << "Index detected: " << indexProfile
		<< " PROFIL_NAVEAU: " << PROFIL_NAVEAU
		<< std::endl;}

  TestNaveau2015 aTN2015(argc,argv,TestName,TestProfiles[indexProfile]);
  if(!aTN2015.init())
    {
      cout << "pb on init" << endl;
      return -1;
    }
  try
    {
      if (!aTN2015.doTest(std::cout)){
	cout << "Failed test " << indexProfile << endl;
	return -1;
      }
      else
	cout << "Passed test " << indexProfile << endl;
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
      return PerformTests(argc,argv);
    }
  catch (const std::string& msg)
    {
      std::cerr << msg << std::endl;
    }
  return 0;
}

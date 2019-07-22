/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
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
#include <iostream>
#include <fstream>
#include <typeinfo>
#include "portability/gettimeofday.hh"

#ifdef WIN32
# include <Windows.h>
#endif /* WIN32 */

#include <time.h>

#include <Debug.hh>
#include <MotionGeneration/ComAndFootRealizationByGeometry.hh>


using namespace PatternGeneratorJRL;

ComAndFootRealizationByGeometry::
   ComAndFootRealizationByGeometry(PatternGeneratorInterfacePrivate *aPGI)
      : ComAndFootRealization(aPGI)
{

  m_WaistPlanner = 0;
  m_UpBody = 0;
  m_ZARM = -1.0;
  m_LeftShoulder = 0;
  m_RightShoulder = 0;
  ShiftFoot_ = true ;
  RegisterMethods();

  for(unsigned int i=0;i<3;i++)
    m_DiffBetweenComAndWaist[i] = 0.0;

  // By assumption on this implementation
  // the humanoid is assume to have 6 DOFs per leg.
  { for(unsigned int i=0;i<m_prev_Configuration.size();m_prev_Configuration[i++]=0.0);};
  { for(unsigned int i=0;i<m_prev_Configuration1.size();m_prev_Configuration1[i++]=0.0);};
  { for(unsigned int i=0;i<m_prev_Configuration2.size();m_prev_Configuration2[i++]=0.0);};
  { for(unsigned int i=0;i<m_prev_Velocity.size();m_prev_Velocity[i++]=0.0);};
  { for(unsigned int i=0;i<m_prev_Velocity1.size();m_prev_Velocity1[i++]=0.0);};
  { for(unsigned int i=0;i<m_prev_Velocity2.size();m_prev_Velocity2[i++]=0.0);};

  RESETDEBUG4("DebugDataVelocity.dat");

  RESETDEBUG4("LegsSpeed.dat");
  RESETDEBUG4("COMPC1.dat");
  RESETDEBUG4("DebugDataIK.dat");
  RESETDEBUG4("DebugDatamDtL.dat");
  RESETDEBUG4("DebugDataStartingCOM.dat");

  RESETDEBUG4("DebugDataCOMForHeuristic.txt");
  RESETDEBUG4("DebugDataIKArms.txt");
  RESETDEBUG4("DebugDataqArmsHeuristic.txt");
  RESETDEBUG4("DebugDataVelocity0.dat");
  RESETDEBUG4("DebugDataVelocity1.dat");

  RESETDEBUG4("DebugDataBodyP0.dat");
  RESETDEBUG4("DebugDataBodyP1.dat");
  RESETDEBUG4("FootPosRight.dat");
  RESETDEBUG4("FootPosLeft.dat");

  // Variables for stepping over upper body motion.
  m_UpperBodyMotion.resize(3);
  m_UpperBodyMotion[0]=0.0;
  m_UpperBodyMotion[1]=0.0;
  m_UpperBodyMotion[2]=0.0;

  m_COGInitialAnkles.setZero();
}

void ComAndFootRealizationByGeometry::
    RegisterMethods()
{
  string aMethodName[3] = {":armparameters",
                           ":UpperBodyMotionParameters",
                           ":samplingperiod"};
  for(int i=0;i<3;i++)
  {
    if (!RegisterMethod(aMethodName[i]))
    {
      std::cerr << "Unable to register " << aMethodName << std::endl;
    }
  }

}

void ComAndFootRealizationByGeometry::
InitializationMaps(std::vector<pinocchio::Index> &FromRootToJoint,
                   pinocchio::JointModelVector & actuatedJoints,
                   std::vector<int> &IndexinConfiguration,
                   std::vector<int> &IndexinVelocity)
{
  if (FromRootToJoint.size()!=0)
  {
    IndexinConfiguration.resize(FromRootToJoint.size());
    IndexinVelocity.resize(FromRootToJoint.size());

    int lindex =0;

    // Here we assume that they are in a decending order.
    for(unsigned int i=0;i<FromRootToJoint.size();i++)
    {
      // -1 because pinocchio uses quaternion instead of roll pitch yaw
      // assuming the model possess only revolute joint
      IndexinConfiguration[lindex] = pinocchio::idx_q(actuatedJoints[FromRootToJoint[i]])-1 ;
      IndexinVelocity[lindex] = pinocchio::idx_v(actuatedJoints[FromRootToJoint[i]]) ;
      lindex++;
    }
  }
}

void ComAndFootRealizationByGeometry::
    InitializeMapsForAHand(pinocchio::JointIndex aWrist,
                           pinocchio::JointModelVector & ActuatedJoints,
                           vector<int> & IndexesInConfiguration,
                           vector<int> & IndexesInVelocity,
                           pinocchio::JointIndex & associateShoulder)
{
  if (aWrist==0)
    return;

  // Find back the path from the shoulder to the left hand.
  pinocchio::JointIndex Chest = getPinocchioRobot()->chest();
  if (Chest==0)
    return;

  const pinocchio::JointIndex associatedWrist = aWrist;
  if (associatedWrist==0)
    return;

  std::vector<pinocchio::JointIndex> FromRootToJoint =
      getPinocchioRobot()->jointsBetween(Chest, associatedWrist);

  associateShoulder = FromRootToJoint[0];

  InitializationMaps(FromRootToJoint,ActuatedJoints,
                     IndexesInConfiguration,
                     IndexesInVelocity);
}

void ComAndFootRealizationByGeometry::
    InitializeMapForChest(pinocchio::JointModelVector & ActuatedJoints)
{

  pinocchio::JointIndex Chest = getPinocchioRobot()->chest();
  if (Chest==0)
    return;

  std::vector<pinocchio::JointIndex> /*FromRootToJoint2,*/FromRootToJoint;
  FromRootToJoint = getPinocchioRobot()->fromRootToIt(Chest);
  // erase the 1rst element as it is the root
  FromRootToJoint.erase (FromRootToJoint.begin());/*
  std::vector<pinocchio::JointIndex>::iterator itJoint = FromRootToJoint.begin();
  bool startadding=false;
  while(itJoint!=FromRootToJoint.end())
  {
    std::vector<pinocchio::JointIndex>::iterator current = itJoint;

    if (*current==Chest)
    {
      startadding=true;

    }
    else
    {
      if (startadding)
        FromRootToJoint2.push_back(*itJoint);
    }
    itJoint++;
  }*/
  InitializationMaps(FromRootToJoint,ActuatedJoints,
                     m_ChestIndexinConfiguration,m_ChestIndexinVelocity);
}

void ComAndFootRealizationByGeometry::
    Initialization()
{

  // Planners for stepping over.
  if (m_WaistPlanner == 0)
    m_WaistPlanner = new WaistHeightVariation();
  if (m_UpBody==0)
    m_UpBody = new UpperBodyMotion();

  // Take the right ankle position (should be equivalent)
  Eigen::Vector3d lAnklePositionRight,lAnklePositionLeft;
  PRFoot *LeftFoot, *RightFoot;
  LeftFoot = getPinocchioRobot()->leftFoot();
  if (LeftFoot==0)
    LTHROW("No left foot");

  RightFoot = getPinocchioRobot()->rightFoot();
  if (RightFoot==0)
    LTHROW("No right foot");

  lAnklePositionRight = RightFoot->anklePosition;
  lAnklePositionLeft = LeftFoot->anklePosition;

  m_AnklePositionRight[0] = lAnklePositionRight[0];
  m_AnklePositionLeft[0]  = lAnklePositionLeft[0];
  m_AnklePositionRight[1] = lAnklePositionRight[1];
  m_AnklePositionLeft[1]  = lAnklePositionLeft[1];
  m_AnklePositionRight[2] = lAnklePositionRight[2];
  m_AnklePositionLeft[2]  = lAnklePositionRight[2];

  // Update the index to change the configuration according
  // to the VRML ID.

  // Extract the indexes of the Right leg.
  pinocchio::JointIndex waist = getPinocchioRobot()->waist();

  if (RightFoot->associatedAnkle==0)
    LTHROW("No right ankle");

  // Build global map.
  pinocchio::JointModelVector & ActuatedJoints =
      getPinocchioRobot()->getActuatedJoints();

  // here we assume all revolute joint
  // -2 : because pinocchio assume a non actuated "universe joint"
  // and a non actuated base joint
  m_GlobalVRMLIDtoConfiguration.resize(ActuatedJoints.size()-2);
  for(unsigned int i=0; i<m_GlobalVRMLIDtoConfiguration.size(); ++i)
  {
    m_GlobalVRMLIDtoConfiguration[i] = pinocchio::idx_q(ActuatedJoints[i+2]);
  }

  // Build right and left leg map.
  // Remove the first element
  std::vector<pinocchio::JointIndex> FromRootToJoint =
      getPinocchioRobot()->jointsBetween(waist, RightFoot->associatedAnkle);
  FromRootToJoint.erase(FromRootToJoint.begin()); // be careful with that line potential bug
  InitializationMaps(FromRootToJoint,ActuatedJoints,
                     m_RightLegIndexinConfiguration,
                     m_RightLegIndexinVelocity);

  FromRootToJoint =
      getPinocchioRobot()->jointsBetween(waist, LeftFoot->associatedAnkle);
  FromRootToJoint.erase(FromRootToJoint.begin());
  InitializationMaps(FromRootToJoint,ActuatedJoints,
                     m_LeftLegIndexinConfiguration,
                     m_LeftLegIndexinVelocity);

  // Create maps for the left hand.
  InitializeMapsForAHand(getPinocchioRobot()->leftWrist(),
                         ActuatedJoints,
                         m_LeftArmIndexinConfiguration,
                         m_LeftArmIndexinVelocity,
                         m_LeftShoulder);

  // Create maps for the right hand.
  InitializeMapsForAHand(getPinocchioRobot()->rightWrist(),
                         ActuatedJoints,
                         m_RightArmIndexinConfiguration,
                         m_RightArmIndexinVelocity,
                         m_RightShoulder);

  InitializeMapForChest(ActuatedJoints);

  FromRootToJoint.clear();

  { for(unsigned int i=0;i<m_prev_Configuration.size();m_prev_Configuration[i++]=0.0);};
  { for(unsigned int i=0;i<m_prev_Configuration1.size();m_prev_Configuration1[i++]=0.0);};
  { for(unsigned int i=0;i<m_prev_Configuration2.size();m_prev_Configuration2[i++]=0.0);};
  { for(unsigned int i=0;i<m_prev_Velocity.size();m_prev_Velocity[i++]=0.0);};
  { for(unsigned int i=0;i<m_prev_Velocity1.size();m_prev_Velocity1[i++]=0.0);};
  { for(unsigned int i=0;i<m_prev_Velocity2.size();m_prev_Velocity2[i++]=0.0);};

}

ComAndFootRealizationByGeometry::~ComAndFootRealizationByGeometry()
{
  if (m_WaistPlanner!=0)
    delete m_WaistPlanner;

  if (m_UpBody!=0)
    delete m_UpBody;

}

bool ComAndFootRealizationByGeometry::
InitializationHumanoid(Eigen::VectorXd &BodyAnglesIni,
		       Eigen::Matrix<double, 6, 1> & lStartingWaistPose)
{
  // For initialization we read the current value inside
  // the model. But we do not use it.
  // it is used to resize the temporary vector
  PinocchioRobot *aPR =  getPinocchioRobot();
  Eigen::VectorXd CurrentConfig = aPR->currentRPYConfiguration();

  // Set to zero the free floating root.
  if(lStartingWaistPose.size())
  {
    CurrentConfig[0] = lStartingWaistPose(0);
    CurrentConfig[1] = lStartingWaistPose(1);
    CurrentConfig[2] = lStartingWaistPose(2);
    CurrentConfig[3] = lStartingWaistPose(3);
    CurrentConfig[4] = lStartingWaistPose(4);
    CurrentConfig[5] = lStartingWaistPose(5);
  }
  else
  {
    lStartingWaistPose.resize(6,0);
  }

  // Initialize the configuration vector.
  for(unsigned int i=0; i<BodyAnglesIni.size(); ++i)
  {
    CurrentConfig[i+pinocchio_robot::RPY_SIZE] =
      BodyAnglesIni[i];
  }

  aPR->currentRPYConfiguration(CurrentConfig);

  // Compensate for the static translation, not the WAIST position
  // but it is the body position which start on the ground.
  aPR->computeForwardKinematics();

  CurrentConfig = aPR->currentRPYConfiguration();

  // Set the waist position.
  lStartingWaistPose(0) = CurrentConfig(0); // 0.0
  lStartingWaistPose(1) = CurrentConfig(1); // 0.0
  lStartingWaistPose(2) = CurrentConfig(2);
  lStartingWaistPose(3) = CurrentConfig(3); //0.0;
  lStartingWaistPose(4) = CurrentConfig(4); //0.0;
  lStartingWaistPose(5) = CurrentConfig(5); //0.0;

  return true;

}

bool ComAndFootRealizationByGeometry::
    InitializationFoot(PRFoot * aFoot,
                       Eigen::Vector3d &m_AnklePosition,
                       FootAbsolutePosition & InitFootPosition)
{
  pinocchio::JointIndex AnkleJoint = aFoot->associatedAnkle;
  pinocchio::SE3 lAnkleSE3 = getPinocchioRobot()->Data()->oMi[AnkleJoint] ;
  Eigen::Vector3d translation ;
  translation << -m_AnklePosition[0], -m_AnklePosition[1], -m_AnklePosition[2];
  pinocchio::SE3 FootTranslation =
    pinocchio::SE3(Eigen::Matrix3d::Identity(),translation);

  pinocchio::SE3 lFootSE3 = lAnkleSE3.act(FootTranslation);

  InitFootPosition.x = lFootSE3.translation()[0];
  InitFootPosition.y = lFootSE3.translation()[1];
  InitFootPosition.z = lFootSE3.translation()[2];

  pinocchio::SE3 lAnkleInitSE3inv = getPinocchioRobot()
      ->DataInInitialePose()->oMi[AnkleJoint];
  Eigen::Matrix3d normalizedRotation =
      lFootSE3.rotation() * lAnkleInitSE3inv.rotation() ;
  // The foot *must be* flat on the floor....
  // Thus
  // lRightFootPose(0:2,0:2)=
  // coct    -st    -soct
  // cost     ct    -sost
  // so        0    co
  ODEBUG(normalizedRotation(2,1));
  assert((fabs(normalizedRotation(2,1))<2e-3) &&
         "Error in the walk pattern generator initialization:" &&
         " Initial foot position is not flat");

  InitFootPosition.omega =
      atan2(normalizedRotation(2,0),normalizedRotation(2,2))*180/M_PI;
  InitFootPosition.theta =
      atan2(-normalizedRotation(0,1),normalizedRotation(1,1))*180/M_PI;
  return true;
}

/*! This initialization phase does the following:
  1/ we take the current state of the robot
  to compute the current CoM value.
  2/ We deduce the difference between the CoM and the waist,
  which is suppose to be constant for the all duration of the motion.
*/
bool ComAndFootRealizationByGeometry::
InitializationCoM
(Eigen::VectorXd &BodyAnglesIni,
 Eigen::Vector3d & lStartingCOMPosition,
 Eigen::Matrix<double,6,1> & lStartingWaistPose,
 FootAbsolutePosition & InitLeftFootPosition,
 FootAbsolutePosition & InitRightFootPosition)
{
  /* Initialize properly the left and right initial positions of the feet. */
  memset((char *)&InitLeftFootPosition,0,sizeof(FootAbsolutePosition));
  memset((char *)&InitRightFootPosition,0,sizeof(FootAbsolutePosition));
  lStartingCOMPosition.setZero();
  lStartingWaistPose.setZero();

  // Compute the forward dynamics from the configuration vector provided by the user.
  // Initialize waist pose.
  InitializationHumanoid(BodyAnglesIni,lStartingWaistPose);

  // Initialise the right foot position.
  PinocchioRobot *aPR =  getPinocchioRobot();
  PRFoot * RightFoot = aPR->rightFoot();
  PRFoot * LeftFoot = aPR->leftFoot();

  // Initialize Feet.
  InitializationFoot(RightFoot, m_AnklePositionRight,InitRightFootPosition);

  InitializationFoot(LeftFoot,  m_AnklePositionLeft, InitLeftFootPosition);

  // Compute the center of gravity between the ankles.
  m_COGInitialAnkles[0] =
      0.5 * (InitRightFootPosition.x + InitLeftFootPosition.x);
  m_COGInitialAnkles[1] =
      0.5 * (InitRightFootPosition.y + InitLeftFootPosition.y);
  m_COGInitialAnkles[2] =
      0.5 * (InitRightFootPosition.z + InitLeftFootPosition.z);

  // Translate lStartingWaist Pose from ( 0.0 0.0 -lFootPosition[2])
  lStartingWaistPose(2) -= InitRightFootPosition.z;

  // CoM position
  getPinocchioRobot()->positionCenterOfMass(lStartingCOMPosition);
  ODEBUG4( "COM positions: "
	    << lStartingCOMPosition(0) << " "
	    << lStartingCOMPosition(1) << " "
	    << lStartingCOMPosition(2),"DebugDataStartingCOM.dat");
  ODEBUG( lStartingCOMPosition[0] << " "
	   << lStartingCOMPosition(1) << " "
	   << lStartingCOMPosition(2));
  lStartingCOMPosition[2] -= InitRightFootPosition.z;

  // Vector to go from CoM to Waist.
  m_DiffBetweenComAndWaist[0] =  lStartingWaistPose(0) - lStartingCOMPosition[0];
  m_DiffBetweenComAndWaist[1] =  lStartingWaistPose(1) - lStartingCOMPosition[1];
  m_DiffBetweenComAndWaist[2] =  lStartingWaistPose(2) - lStartingCOMPosition[2];
  ODEBUG("lFootPosition[2]: " <<InitRightFootPosition.z);
  ODEBUG( "Diff between Com and Waist" << m_DiffBetweenComAndWaist[0] << " "
          << m_DiffBetweenComAndWaist[1] << " "
          << m_DiffBetweenComAndWaist[2]);
  // This term is usefull if

  ODEBUG4("m_DiffBetweenComAndWaist :" << m_DiffBetweenComAndWaist,"DebugData.txt");
  // the initial position does not put z at Zc

  // The most important line of the method...
  // The one which initialize correctly the height of the pattern generator.
  for(int i=0;i<3;i++)
  {
    m_TranslationToTheLeftHip(i)  = m_DiffBetweenComAndWaist[i];
    m_TranslationToTheRightHip(i) = m_DiffBetweenComAndWaist[i];
  }

  // Initialize previous configuration vector
  m_prev_Configuration.setZero();
  m_prev_Configuration1.setZero();
  m_prev_Configuration2.setZero();
  m_prev_Velocity.setZero();
  m_prev_Velocity1.setZero();
  m_prev_Velocity2.setZero();


  for(unsigned int i = 0 ; i < BodyAnglesIni.size() ; ++i)
  {
    m_prev_Configuration (i) = BodyAnglesIni(i);
    m_prev_Configuration1(i) = BodyAnglesIni(i);
    m_prev_Configuration2(i) = BodyAnglesIni(i);
  }
  for(unsigned int i = 0 ; i < 6 ; ++i)
  {
    m_prev_Configuration (i) = lStartingWaistPose(i);
    m_prev_Configuration1(i) = lStartingWaistPose(i);
    m_prev_Configuration2(i) = lStartingWaistPose(i);
  }

  InitLeftFootPosition.z = 0.0;
  InitRightFootPosition.z = 0.0;

  return true;
}

bool ComAndFootRealizationByGeometry::
InitializationUpperBody
(deque<ZMPPosition> &inZMPPositions,
 deque<COMPosition> &inCOMBuffer,
 deque<RelativeFootPosition> lRelativeFootPositions)
{

  // Check pre-condition.
  if (getPinocchioRobot()==0)
  {
    cerr << "ComAndFootRealizationByGeometry::InitializationUpperBody "  << endl
        << "No Humanoid Specificites class given " << endl;
    return false;
  }

  //  FootAbsolutePosition InitLeftFootAbsPos, InitRightFootAbsPos;
  struct timeval begin,  time2, time3;// end, time1, time4, time5, time6;

  gettimeofday(&begin,0);

  ODEBUG4("FinishAndRealizeStepSequence() - 1","DebugGMFKW.dat");
  cout << __FUNCTION__ << ":"<< __LINE__ << ": You should implement something here"
      << endl;

  ODEBUG4("FinishAndRealizeStepSequence() - 3 ","DebugGMFKW.dat");

  gettimeofday(&time2,0);
  //this function calculates a buffer with COM values after a first preview round,
  // currently required to calculate the arm swing before "onglobal step of control"
  // in order to take the arm swing motion into account in the second preview loop
  deque<ZMPPosition> aZMPBuffer;

  aZMPBuffer.resize(inCOMBuffer.size());

  ODEBUG4("FinishAndRealizeStepSequence() - 4 ","DebugGMFKW.dat");

  // read upperbody data which has to be included in the patterngeneration second preview loop stability
  string BodyDat = "UpperBodyDataFile.dat";

  ODEBUG4("FinishAndRealizeStepSequence() - 5 ","DebugGMFKW.dat");
  // Link the current trajectory and GenerateMotionFromKineoWorks.

  // Specify the buffer.

  m_UpperBodyPositionsBuffer.clear();


  if (GetStepStackHandler()->GetWalkMode()==3)
  {
    ODEBUG4("Before Starting GMFKW","DebugGMFKW.dat");
    m_GMFKW->CreateBufferFirstPreview(inZMPPositions);

    // Map the path found by KineoWorks onto the ZMP buffer.
    m_ConversionForUpperBodyFromLocalIndexToRobotDOFs.clear();

    m_GMFKW->ComputeUpperBodyPosition
      (m_UpperBodyPositionsBuffer,
       m_ConversionForUpperBodyFromLocalIndexToRobotDOFs);
    ODEBUG4("After GMFKW","DebugGMFKW.dat");
  }
  else
  {
    // Create the stack of upper body motion.
    m_UpperBodyPositionsBuffer.resize(inZMPPositions.size());


    // Create the conversion array between the Upper body indexes
    // and the Joint indexes.
    /*
      int UpperBodyJointNb = getHumanoidDynamicRobot()->GetUpperBodyJointNb();
      m_ConversionForUpperBodyFromLocalIndexToRobotDOFs = getHumanoidDynamicRobot()->GetUpperBodyJoints();

      for(unsigned int i=0;i<m_ConversionForUpperBodyFromLocalIndexToRobotDOFs.size();i++)
	{

	  m_ConversionForUpperBodyFromLocalIndexToRobotDOFs[i] = i;
	}

      for(unsigned int i=0;i<m_UpperBodyPositionsBuffer.size();i++)
	{
	  m_UpperBodyPositionsBuffer[i].Joints.resize(UpperBodyJointNb);

	  Eigen::VectorXd currentConfiguration;
	  currentConfiguration = getHumanoidDynamicRobot()->currentConfiguration();

	  if (i==0)
	    {
	      for(int j=0;j<UpperBodyJointNb;j++)
		m_UpperBodyPositionsBuffer[i].Joints[j] = currentConfiguration
		  [m_ConversionForUpperBodyFromLocalIndexToRobotDOFs[j]];
	    }
	  // Initialize the upper body motion to the current stored value.
	  for(int j=0;j<UpperBodyJointNb;j++)
	    m_UpperBodyPositionsBuffer[i].Joints[j] = m_UpperBodyPositionsBuffer[0].Joints[j];
	}
      */

  }

  ODEBUG4("FinishAndRealizeStepSequence() - 6 ","DebugCG.ctx");


  gettimeofday(&time3,0);


  if ((GetStepStackHandler()->GetWalkMode()==1)	||
      (GetStepStackHandler()->GetWalkMode()==3)	)
  {
    m_WaistPlanner->PolyPlanner
      (inCOMBuffer,lRelativeFootPositions,inZMPPositions);
  }

  ODEBUG("inZMPPositions : " << inZMPPositions.size() << endl <<
         "m_COMBuffer : " << inCOMBuffer.size() << endl);

  return true;

}

bool ComAndFootRealizationByGeometry::
KinematicsForOneLeg
(Eigen::Matrix3d & Body_R,
 Eigen::Vector3d & Body_P,
 Eigen::VectorXd &aFoot,
 Eigen::Vector3d & lDt,
 Eigen::VectorXd &  aCoMPosition,
 Eigen::Vector3d & ToTheHip,
 int LeftOrRight,
 Eigen::VectorXd &lq,
 int Stage)
{

  // Foot Orientation
  Eigen::Matrix3d Foot_R;
  // Foot position
  Eigen::Vector3d Foot_P;

  // LEFT Foot.
  double FootPositiontheta = aFoot(3);
  double FootPositionomega = aFoot(4);
  double c,s,co,so;

  // This is just to make the flag WERROR happy.
  c = aCoMPosition(2) + ToTheHip(2) + lDt(2);


  ODEBUG("FootPositiontheta: " << FootPositiontheta);
  c = cos(FootPositiontheta*M_PI/180.0);
  s = sin(FootPositiontheta*M_PI/180.0);

  co = cos(FootPositionomega*M_PI/180.0);
  so = sin(FootPositionomega*M_PI/180.0);

  // Orientation
  Foot_R(0,0) = c*co;       Foot_R(0,1) = -s;       Foot_R(0,2) = c*so;
  Foot_R(1,0) = s*co;       Foot_R(1,1) =  c;       Foot_R(1,2) = s*so;
  Foot_R(2,0) = -so;        Foot_R(2,1) = 0;        Foot_R(2,2) = co;

  // position
  Foot_P(0) = aFoot(0);Foot_P(1) = aFoot(1); Foot_P(2) = aFoot(2);

  Eigen::Vector3d Foot_Shift;

  pinocchio::JointIndex Ankle = 0;
  if (LeftOrRight==-1)
  {
    Foot_Shift=Foot_R*m_AnklePositionRight;
    Ankle = getPinocchioRobot()->rightFoot()->associatedAnkle;
  }
  else if (LeftOrRight==1)
  {
    Foot_Shift=Foot_R*m_AnklePositionLeft;
    Ankle = getPinocchioRobot()->leftFoot()->associatedAnkle;
  }

  if(ShiftFoot_)
    {
      Foot_P = Foot_P + Foot_Shift;
    }
  //  Foot_P(2)-=(aCoMPosition(2) + ToTheHip(2));
  ODEBUG("Body_P:" << endl << Body_P);
  ODEBUG("Body_R:" << endl << Body_R);
  ODEBUG("Foot_P" << Foot_P);
  ODEBUG("Foot_R" << Foot_R);
  // Compute the inverse kinematics.
  if ((LeftOrRight==1) && (Stage==0))
  {
    ODEBUG4SIMPLE(Body_P[0] << " " <<
                  Body_P[1] << " " <<
                  Body_P[2] << " " <<
                  Foot_P[0] << " " <<
                  Foot_P[1] << " " <<
                  Foot_P[2] << " "
                  ,"DebugDataIK.dat");
  }

  ODEBUG4("Body_P " << Body_P,"DebugDataIK.dat");
  ODEBUG4("Body_R " << Body_R,"DebugDataIK.dat");
  ODEBUG4("Foot_P " << Foot_P,"DebugDataIK.dat");
  ODEBUG4("Foot_R " << Foot_R,"DebugDataIK.dat");
  ODEBUG4("lDt " << lDt,"DebugDataIK.dat");

  // Homogeneous matrix
  Eigen::Matrix4d BodyPose,FootPose;
  for(unsigned int i=0;i<3;i++)
  {
    for(unsigned int j=0;j<3;j++)
    {
      BodyPose(i,j) =
	Body_R(i,j);
      FootPose(i,j) =
	Foot_R(i,j);
    }
    BodyPose(i,3) = Body_P(i);
    FootPose(i,3) = Foot_P(i);
  }
  BodyPose(3,3) = 1.0 ;
  FootPose(3,3) = 1.0 ;

  pinocchio::JointIndex Waist = getPinocchioRobot()->waist();

  ODEBUG("Typeid of humanoid: " << typeid(getHumanoidDynamicRobot()).name() );
  // Call specialized dynamics.
  getPinocchioRobot()->ComputeSpecializedInverseKinematics(
        Waist,Ankle,BodyPose,FootPose,lq);
  ODEBUG4("lq " << lq,"DebugDataIK.dat");
  return true;
}

bool ComAndFootRealizationByGeometry::
KinematicsForTheLegs
(Eigen::VectorXd & aCoMPosition,
 Eigen::VectorXd & aLeftFoot,
 Eigen::VectorXd & aRightFoot,
 int Stage,
 Eigen::VectorXd & ql,
 Eigen::VectorXd & qr,
 Eigen::Vector3d & AbsoluteWaistPosition )
{

  // Body attitude
  Eigen::Matrix3d Body_R;
  // Body position
  Eigen::Vector3d Body_P;
  // To the hip
  Eigen::Vector3d ToTheHip;

  // Angles for the COM
  double COMtheta, COMomega;

  // Extract the angles from the input.
  COMtheta = aCoMPosition(5);
  COMomega = aCoMPosition(4);

  ODEBUG("COMtheta: " << COMtheta);
  // Cos and sinus for theta.
  double CosTheta,SinTheta;

  // Cos and sinus for Omega.
  double CosOmega,SinOmega;

  CosTheta = cos(COMtheta*M_PI/180.0);
  SinTheta = sin(COMtheta*M_PI/180.0);

  CosOmega = cos(COMomega*M_PI/180.0);
  SinOmega = sin(COMomega*M_PI/180.0);

  // COM Orientation
  Body_R(0,0) = CosTheta*CosOmega;
  Body_R(0,1) = -SinTheta;
  Body_R(0,2) = CosTheta*SinOmega;

  Body_R(1,0) = SinTheta*CosOmega;
  Body_R(1,1) = CosTheta;
  Body_R(1,2) = SinTheta*SinOmega;

  Body_R(2,0) = -SinOmega;
  Body_R(2,1) = 0;
  Body_R(2,2) = CosOmega;

  // COM position

  ToTheHip=Body_R*m_TranslationToTheLeftHip;
  Body_P(0) = aCoMPosition(0) + ToTheHip(0) ;
  Body_P(1) = aCoMPosition(1) + ToTheHip(1);
  Body_P(2) = aCoMPosition(2) + ToTheHip(2);

  ODEBUG(aCoMPosition(2) <<
         " Body_P : " << Body_P  << std::endl <<
         " aLeftFoot : " << aLeftFoot);
  /* If this is the second call, (stage =1)
     it is the final desired CoM */
  if (Stage==1)
  {
    m_FinalDesiredCOMPose(0,0) = CosTheta*CosOmega;
    m_FinalDesiredCOMPose(0,1) = -SinTheta;
    m_FinalDesiredCOMPose(0,2) = CosTheta*SinOmega;

    m_FinalDesiredCOMPose(1,0) = SinTheta*CosOmega;
    m_FinalDesiredCOMPose(1,1) = CosTheta;
    m_FinalDesiredCOMPose(1,2) = SinTheta*SinOmega;

    m_FinalDesiredCOMPose(2,0) = -SinOmega;
    m_FinalDesiredCOMPose(2,1)=  0;
    m_FinalDesiredCOMPose(2,2) = CosOmega;

    m_FinalDesiredCOMPose(0,3) = aCoMPosition(0);
    m_FinalDesiredCOMPose(1,3) = aCoMPosition(1);
    m_FinalDesiredCOMPose(2,3) = aCoMPosition(2);
    m_FinalDesiredCOMPose(3,3) = 1.0;

    ODEBUG4(Body_P ,"DebugDataBodyP1.dat");
  }
  else
  {
    ODEBUG4(Body_P ,"DebugDataBodyP0.dat");
  }


  // Kinematics for the left leg.
  ODEBUG4("Stage " << Stage,"DebugDataIK.dat");
  ODEBUG4("* Left Lego *","DebugDataIK.dat");
  KinematicsForOneLeg(Body_R,
		      Body_P,
		      aLeftFoot,
		      m_DtLeft,
		      aCoMPosition,
		      ToTheHip,1,ql,Stage);

  // Kinematics for the right leg.
  ToTheHip=Body_R*m_TranslationToTheRightHip;
  Body_P(0) = aCoMPosition(0) + ToTheHip(0);
  Body_P(1) = aCoMPosition(1) + ToTheHip(1);
  Body_P(2) = aCoMPosition(2) + ToTheHip(2);

  ODEBUG4("* Right Leg *","DebugDataIK.dat");
  KinematicsForOneLeg(Body_R,
		      Body_P,
		      aRightFoot,
		      m_DtRight,
		      aCoMPosition,
		      ToTheHip,-1,qr,Stage);

  ODEBUG4("**************","DebugDataIK.dat");
  /* Should compute now the Waist Position */
  m_ComAndWaistInRefFrame=Body_R*m_DiffBetweenComAndWaist;

  AbsoluteWaistPosition[0] = aCoMPosition(0) + m_ComAndWaistInRefFrame[0];
  AbsoluteWaistPosition[1] = aCoMPosition(1) + m_ComAndWaistInRefFrame[1];
  AbsoluteWaistPosition[2] = aCoMPosition(2) + ToTheHip(2) ;// - 0.705;

  return true;
}

bool ComAndFootRealizationByGeometry::
ComputePostureForGivenCoMAndFeetPosture
(Eigen::VectorXd & aCoMPosition,
 Eigen::VectorXd & aCoMSpeed,
 Eigen::VectorXd & aCoMAcc,
 Eigen::VectorXd & aLeftFoot,
 Eigen::VectorXd & aRightFoot,
 Eigen::VectorXd & CurrentConfiguration,
 Eigen::VectorXd & CurrentVelocity,
 Eigen::VectorXd & CurrentAcceleration,
 unsigned long int IterationNumber,
 int Stage)
{
  Eigen::Vector3d AbsoluteWaistPosition;
  Eigen::VectorXd lqr(6);
  Eigen::VectorXd lql(6);

  // Kinematics for the legs.
  KinematicsForTheLegs(aCoMPosition,
                       aLeftFoot,
                       aRightFoot,
                       Stage,
                       lql,
                       lqr,
                       AbsoluteWaistPosition);
  /// NOW IT IS ABOUT THE UPPER BODY... ////
  Eigen::VectorXd qArmr(6);
  Eigen::VectorXd qArml(6);

  for(unsigned int i=0;i<qArmr.size();i++)
  {
    qArmr[i] = 0.0;
    qArml[i] = 0.0;
  }

  if (GetStepStackHandler()->GetWalkMode()<3)
  {
    Eigen::VectorXd lAbsoluteWaistPosition(6);
    for(unsigned int i=0;i<3;i++)
    {
      lAbsoluteWaistPosition(i) = AbsoluteWaistPosition[i];
      lAbsoluteWaistPosition(i+3) = aCoMPosition(i+3);
    }
    ODEBUG("AbsoluteWaistPosition:" << lAbsoluteWaistPosition  <<
           " ComPosition" << aCoMPosition);

    ComputeUpperBodyHeuristicForNormalWalking
      (qArmr,
       qArml,
       lAbsoluteWaistPosition,
       aRightFoot,
       aLeftFoot);
  }

  // For stepping over modify the waist position and
  // according to parameters the arms motion.
  if(GetStepStackHandler()->GetWalkMode()==2)
  {

    /// this angle is introduced to rotate the upperbody
    ///  when the waist is rotated during stepover
    double qWaistYaw = -CurrentConfiguration
      (m_ChestIndexinConfiguration[0])*M_PI/180.0;
    ODEBUG4(qWaistYaw,"DebugDataWaistYaw.dat");
    //this is not correct yet since it uses COMPositionFromPC1.
    // theta which also changes when turning....
    // it will be modified in the near future
    // include waistrotation in dynamic model for second preview correction

    CurrentConfiguration[m_ChestIndexinConfiguration[0]] = qWaistYaw;

    if (m_UpperBodyMotion[0]!=0)
    {
      CurrentConfiguration[m_ChestIndexinConfiguration[1]] =
          m_UpperBodyMotion[0]*fabs(qWaistYaw);

    }
    if (m_UpperBodyMotion[1]!=0)
    {
      qArmr(0)=qArmr(0)-m_UpperBodyMotion[1]*fabs(qWaistYaw);
      qArml(0)=qArml(0)-m_UpperBodyMotion[1]*fabs(qWaistYaw);
    }

    if (m_UpperBodyMotion[2]!=0)
    {

      aCoMPosition(4) = m_UpperBodyMotion[2]*fabs(aCoMPosition(5));
    }

  }

  ODEBUG( "ComAndFoot: AbsoluteWaistPosition: "
	  << AbsoluteWaistPosition << endl
          << "CoMPosition: " << aCoMPosition );
  ODEBUG("Left FootPosition: " << aLeftFoot <<
         " Right FootPosition: " << aRightFoot );
  /* Update of the configuration and velocity vector */
  for(int i=0;i<3;i++)
    CurrentConfiguration[i] = AbsoluteWaistPosition(i);

  for(int i=3;i<6;i++)
    CurrentConfiguration[i] = aCoMPosition(i)*M_PI/180.0;

  for(unsigned int i=0;i<lqr.size();i++)
    CurrentConfiguration[m_RightLegIndexinConfiguration[i]] = lqr[i];

  for(unsigned int i=0;i<lql.size();i++)
    CurrentConfiguration[m_LeftLegIndexinConfiguration[i]] = lql[i];

  for(unsigned int i=0;i<qArmr.size();i++)
    CurrentConfiguration[m_RightArmIndexinConfiguration[i]] = qArmr[i];

  for(unsigned int i=0;i<qArml.size();i++)
    CurrentConfiguration[m_LeftArmIndexinConfiguration[i]] = qArml[i];

  // Update the speed values.
  /* If this is the first call ( stage = 0)
     we should update the current stored values.  */
  /* Initialize the acceleration */
  for(unsigned int i=0;i<CurrentAcceleration.size();i++)
  {
    CurrentVelocity[i]=0.0;
    CurrentAcceleration[i] = 0.0;
    /* Keep the new value for the legs. */
  }

  double ldt =  getSamplingPeriod();

  pinocchio::JointIndex diffVelSize = getPinocchioRobot()->getFreeFlyerSize()-
    getPinocchioRobot()->getFreeFlyerVelSize();

  if (Stage==0)
  {
    if (IterationNumber>0)
    {
      /* Compute the speed */
      for(unsigned int i=6;i<m_prev_Configuration.size()-diffVelSize;i++)
      {
        CurrentVelocity[i] = (CurrentConfiguration[i+diffVelSize]
			      - m_prev_Configuration[i+diffVelSize])/ ldt;
        /* Keep the new value for the legs. */
      }

      if (IterationNumber>1)
      {
        for(unsigned int i=6;i<m_prev_Velocity.size();i++)
          CurrentAcceleration[i] = (CurrentVelocity[i] - m_prev_Velocity[i])/ ldt;
      }
    }
    else
    {
      /* Compute the speed */
      for(unsigned int i=0;i<CurrentVelocity.size();i++)
      {
        CurrentVelocity[i] = 0.0;
        /* Keep the new value for the legs. */
      }
    }

    ODEBUG4(CurrentVelocity, "DebugDataVelocity0.dat");
    m_prev_Configuration = CurrentConfiguration;
    m_prev_Velocity = CurrentVelocity;
  }
  else if (Stage==1)
  {
    ODEBUG("lql: "<<lql<< " lqr: " <<lqr);
    if (IterationNumber>0)
    {
      /* Compute the speed */
      for(unsigned int i=6;i<m_prev_Configuration1.size()-diffVelSize;i++)
      {
        CurrentVelocity[i] = (CurrentConfiguration[i+diffVelSize] -
			      m_prev_Configuration1[i+diffVelSize])/ ldt;
        /* Keep the new value for the legs. */
      }
      if (IterationNumber>1)
      {
        for(unsigned int i=6;i<m_prev_Velocity1.size();i++)
          CurrentAcceleration[i] = (CurrentVelocity[i] - m_prev_Velocity1[i])/ ldt;
      }
    }
    else
    {
      CurrentVelocity.setZero();
    }
    ODEBUG4(CurrentVelocity, "DebugDataVelocity1.dat");
    m_prev_Configuration1 = CurrentConfiguration;
    m_prev_Velocity1 = CurrentVelocity;
  }
  else if (Stage==2)
  {
    ODEBUG("lql: "<<lql<< " lqr: " <<lqr);
    if (IterationNumber>0)
    {
      /* Compute the speed */
      for(unsigned int i=6;i<m_prev_Configuration2.size()-diffVelSize;i++)
      {
        CurrentVelocity[i] = (CurrentConfiguration[i+diffVelSize]
			      - m_prev_Configuration2[i+diffVelSize])/
	  getSamplingPeriod();
        /* Keep the new value for the legs. */
      }
      if (IterationNumber>1)
      {
        for(unsigned int i=6;i<m_prev_Velocity2.size()-diffVelSize;i++)
          CurrentAcceleration[i] =
	    (CurrentVelocity[i+diffVelSize]
	     - m_prev_Velocity2[i+diffVelSize])/ ldt;
      }
    }
    else
    {
      CurrentVelocity.setZero();
    }
    ODEBUG4(CurrentVelocity, "DebugDataVelocity1.dat");
    m_prev_Configuration2 = CurrentConfiguration;
    m_prev_Velocity2 = CurrentVelocity;
  }

  Eigen::Vector3d waistCom;
  for(int i=0;i<3;i++)
    waistCom(i) = aCoMPosition(i) - AbsoluteWaistPosition(i) ;

  // v_waist = v_com + waist-com x omega :
  CurrentVelocity[0] = aCoMSpeed(0) +
    (waistCom(1)*aCoMSpeed(5)  -  waistCom(2)*aCoMSpeed(4) ) ;
  CurrentVelocity[1] = aCoMSpeed(1) +
    (waistCom(2)*aCoMSpeed(3)  - waistCom(0)*aCoMSpeed(5) ) ;
  CurrentVelocity[2] = aCoMSpeed(2) +
    (waistCom(0)*aCoMSpeed(4)  - waistCom(1)*aCoMSpeed(3) ) ;

  // omega_waist = omega_com
  for(int i=3;i<6;i++)
    CurrentVelocity[i] = aCoMSpeed(i);


  // (omega x waist-com) x omega = waist-com ( omega . omega )
  // - omega ( omega . waist-com )
  Eigen::Vector3d coriolis;
  double omega_dot_omega =
    aCoMSpeed(3)*aCoMSpeed(3) +
    aCoMSpeed(4)*aCoMSpeed(4) +
    aCoMSpeed(5)*aCoMSpeed(5) ;
  double omega_dot_waistCom =
    aCoMSpeed(3)*waistCom(0) +
    aCoMSpeed(4)*waistCom(1) +
    aCoMSpeed(5)*waistCom(2) ;

  coriolis(0) = waistCom(0) * omega_dot_omega - aCoMSpeed(3) * omega_dot_waistCom ;
  coriolis(1) = waistCom(1) * omega_dot_omega - aCoMSpeed(4) * omega_dot_waistCom ;
  coriolis(2) = waistCom(2) * omega_dot_omega - aCoMSpeed(5) * omega_dot_waistCom ;

  // a_waist = a_com + waist-com x d omega/dt + (omega x waist-com) x omega
  CurrentAcceleration[0] = aCoMAcc(0) +
    (waistCom(1)*aCoMAcc(5)  - waistCom(2)*aCoMAcc(4) ) + coriolis(0) ;
  CurrentAcceleration[1] = aCoMAcc(1) +
    (waistCom(2)*aCoMAcc(3)  - waistCom(0)*aCoMAcc(5) ) + coriolis(1) ;
  CurrentAcceleration[2] = aCoMAcc(2) +
    (waistCom(0)*aCoMAcc(4)  - waistCom(1)*aCoMAcc(3) ) + coriolis(2) ;

  // d omega_waist /dt = d omega_com /dt
  //cout << "CFRG : " ;
  for(int i=3;i<6;i++)
  {
    //cout << aCoMAcc(i) << " "  ;
    CurrentAcceleration[i] = aCoMAcc(i);
  }//cout << endl ;


  ODEBUG( "CurrentVelocity :" << endl << CurrentVelocity);
  ODEBUG4("SamplingPeriod " << getSamplingPeriod(),"LegsSpeed.dat");

  string aDebugFileName;

  ODEBUG4( (1.0/M_PI)*180.0*lql[0] << " " <<
           (1.0/M_PI)*180.0*lql[1] << " " <<
           (1.0/M_PI)*180.0*lql[2] << " " <<
           (1.0/M_PI)*180.0*lql[3] << " " <<
           (1.0/M_PI)*180.0*lql[4] << " " <<
           (1.0/M_PI)*180.0*lql[5],(char *)aDebugFileName.c_str());

  ODEBUG4(CurrentVelocity,"DebugDataVelocity.dat");

  ODEBUG4( aCoMPosition[0]
           << " " <<
           aCoMPosition[1],"COMPC1.dat");

  return true;
}

int ComAndFootRealizationByGeometry::
    EvaluateStartingCoM(Eigen::VectorXd &BodyAngles,
                        Eigen::Vector3d &aStartingCOMPosition,
                        FootAbsolutePosition & InitLeftFootPosition,
                        FootAbsolutePosition & InitRightFootPosition)
{

  EvaluateCOMForStartingPosition(BodyAngles,
                                 0.0,0.0,
                                 m_StartingCOMPosition,
                                 InitLeftFootPosition,
                                 InitRightFootPosition);

  aStartingCOMPosition[0] = m_StartingCOMPosition[0];
  aStartingCOMPosition[1] = m_StartingCOMPosition[1];
  aStartingCOMPosition[2] = m_StartingCOMPosition[2];

  return 0;
}

int ComAndFootRealizationByGeometry::
    EvaluateStartingCoM(Eigen::VectorXd &BodyAngles,
                        Eigen::Vector3d &aStartingCOMPosition,
                        Eigen::Matrix<double, 6, 1> &aWaistPose,
                        FootAbsolutePosition & InitLeftFootPosition,
                        FootAbsolutePosition & InitRightFootPosition)
{
  Eigen::VectorXd WaistPose;
  InitializationCoM(BodyAngles,
                    m_StartingCOMPosition,
                    aWaistPose,
                    InitLeftFootPosition,
                    InitRightFootPosition);
  aStartingCOMPosition[0] = m_StartingCOMPosition[0];
  aStartingCOMPosition[1] = m_StartingCOMPosition[1];
  aStartingCOMPosition[2] = m_StartingCOMPosition[2];

  return 0;
}

int ComAndFootRealizationByGeometry::
    EvaluateCOMForStartingPosition( Eigen::VectorXd &BodyAngles,
                                    double , // omega,
                                    double , // theta,
                                    Eigen::Vector3d &lCOMPosition,
                                    FootAbsolutePosition & InitLeftFootPosition,
                                    FootAbsolutePosition & InitRightFootPosition)
{
  Eigen::Matrix<double, 6, 1> lWaistPose;
  return InitializationCoM(BodyAngles,
                           lCOMPosition,
                           lWaistPose,
                           InitLeftFootPosition,
			   InitRightFootPosition);

}


void ComAndFootRealizationByGeometry::
    GetCurrentPositionofWaistInCOMFrame(Eigen::VectorXd &CurPosWICF_homogeneous)
{
  for(int i=0;i<3;i++)
    CurPosWICF_homogeneous[i] = m_DiffBetweenComAndWaist[i];
  CurPosWICF_homogeneous[3] = 1.0;

}

void ComAndFootRealizationByGeometry::
    ComputeUpperBodyHeuristicForNormalWalking(Eigen::VectorXd & qArmr,
                                              Eigen::VectorXd & qArml,
                                              Eigen::VectorXd & aCOMPosition,
                                              Eigen::VectorXd & RFP,
                                              Eigen::VectorXd & LFP)
{

  ODEBUG4("aCOMPosition:" << aCOMPosition << endl <<
          "Right Foot Position:" << RFP << endl <<
          "Left Foot Position:" << LFP << endl, "DebugDataIKArms.txt");

  ODEBUG4(m_ZARM << " " << m_Xmax << " " << " " << m_GainFactor,"DebugDataIKArms.txt");
  double TempXL,TempXR,TempCos,TempSin,
  TempARight,TempALeft;

  // Compute the position of the hand according to the
  // leg.
  TempCos = cos(aCOMPosition(5)*M_PI/180.0);
  TempSin = sin(aCOMPosition(5)*M_PI/180.0);

  TempXR = TempCos * (RFP(0) +m_AnklePositionRight[0] - aCOMPosition(0) - m_COGInitialAnkles(0)) +
           TempSin * (RFP(1) +m_AnklePositionRight[1] - aCOMPosition(1) - m_COGInitialAnkles(1));
  TempXL = TempCos * (LFP(0)  +m_AnklePositionRight[0] - aCOMPosition(0) - m_COGInitialAnkles(0)) +
           TempSin * (LFP(1) +m_AnklePositionRight[1] - aCOMPosition(1) - m_COGInitialAnkles(1));

  ODEBUG4(aCOMPosition(0) << " " << aCOMPosition(1) << " " << aCOMPosition(3),"DebugDataIKArms.txt");
  ODEBUG4(RFP(0) << " " << RFP(1) ,"DebugDataIKArms.txt");
  ODEBUG4(LFP(0) << " " << LFP(1) ,"DebugDataIKArms.txt");

  TempARight = TempXR*-1.0;
  TempALeft = TempXL*-1.0;

  ODEBUG4("Values: TL " << TempALeft <<
          " TR " << TempARight <<
          " "    << m_ZARM <<
          " "    << m_Xmax ,"DebugDataIKArms.txt");
  ODEBUG("Values: TL " << TempALeft <<
         " TR " << TempARight <<
         " "    << m_ZARM <<
         " "    << m_Xmax );
  // Compute angles using inverse kinematics and the computed hand position.

  Eigen::Matrix4d jointRootPosition,jointEndPosition;
  jointRootPosition.setIdentity();
  jointEndPosition.setIdentity();

  jointEndPosition(0,3) =
      TempALeft * m_GainFactor / 0.2;
  jointEndPosition(2,3) = m_ZARM;

  getPinocchioRobot()->ComputeSpecializedInverseKinematics(
        m_LeftShoulder,
        getPinocchioRobot()->leftWrist(),
        jointRootPosition,
        jointEndPosition,
        qArml);
  ODEBUG4("ComputeHeuristicArm: Step 2 ","DebugDataIKArms.txt");
  ODEBUG4( "IK Left arm p:" << qArml(0)<< " " <<  qArml(1)  << " " << qArml(2)
           << " " << qArml(3) << "  " << qArml(4) << " " << qArml(5), "DebugDataIKArms.txt" );

  jointEndPosition(0,3) = TempARight;
  jointEndPosition(2,3) = m_ZARM;

  getPinocchioRobot()->ComputeSpecializedInverseKinematics(
        m_RightShoulder,
        getPinocchioRobot()->rightWrist(),
        jointRootPosition,
        jointEndPosition,
        qArmr);
  ODEBUG4( "IK Right arm p:" << qArmr(0)<< " " <<  qArmr(1)  << " " << qArmr(2)
           << " " << qArmr(3) << "  " << qArmr(4) << " " << qArmr(5), "DebugDataIKArms.txt" );

  ODEBUG4( qArml(0)<< " " <<  qArml(1)  << " " << qArml(2) << " "
           << qArml(3) << "  " << qArml(4) << " " << qArml(5) << " "
           << qArmr(0)<< " " <<  qArmr(1)  << " " << qArmr(2) << " "
           << qArmr(3) << "  " << qArmr(4) << " " << qArmr(5),
	   "DebugDataqArmsHeuristic.txt");
}

bool ComAndFootRealizationByGeometry::
    setPinocchioRobot(PinocchioRobot * aPinocchioRobot)
{
  ComAndFootRealization::setPinocchioRobot(aPinocchioRobot);
  PinocchioRobot *aPR =  aPinocchioRobot;

  m_prev_Configuration.resize(aPR->numberDof());
  m_prev_Configuration1.resize(aPR->numberDof());
  m_prev_Configuration2.resize(aPR->numberDof());

  for(unsigned int i=0;i<m_prev_Configuration.size();i++)
  {
    m_prev_Configuration[i] = 0.0;
    m_prev_Configuration1[i] = 0.0;
    m_prev_Configuration2[i] = 0.0;
  }
  return true;
}

void ComAndFootRealizationByGeometry::
    CallMethod(string &Method, istringstream &istrm)
{

  if (Method==":armparameters")
  {
    istrm >> m_GainFactor;
  }
  else if (Method==":UpperBodyMotionParameters")
  {
    if (!istrm.eof())
    {
      istrm >> m_UpperBodyMotion[0];
    }
    else
      if (!istrm.eof())
      {
	    istrm >> m_UpperBodyMotion[1];
	  }
    else
      if (!istrm.eof())
	    {
      istrm >> m_UpperBodyMotion[2];
    }
  }
  else if (Method==":samplingperiod")
  {
    double ldt;
    istrm >> ldt;
    setSamplingPeriod(ldt);
  }
}

Eigen::Matrix4d ComAndFootRealizationByGeometry::
    GetCurrentPositionofWaistInCOMFrame()
{
  Eigen::Matrix4d P;

  P.setIdentity();
  P(0,3) = m_DiffBetweenComAndWaist[0];
  P(1,3) = m_DiffBetweenComAndWaist[1];
  P(2,3) = m_DiffBetweenComAndWaist[2];

  return P;
}

Eigen::Vector3d ComAndFootRealizationByGeometry::GetCOGInitialAnkles()
{

  return m_COGInitialAnkles;
}

ostream& PatternGeneratorJRL::operator<<(ostream &os,const ComAndFootRealization &obj)
{

  SimplePluginManager * SPM = obj.getSimplePluginManager() ;
  os << "The Simple Plugin Manager is :\n" ;
  os << SPM << endl ;

  //  Eigen::Vector3d COGInitialAnkles = obj.GetCOGInitialAnkles() ;
  //  os << "The initial COG of ankles is :\n" ;
  //  os << COGInitialAnkles(0,0) << endl
  //  << COGInitialAnkles(1,0) << endl
  //  << COGInitialAnkles(2,0) << endl ;

  StepStackHandler* stepStackHandler = obj.GetStepStackHandler() ;
  os << "The Step Stack Handler pointer is :\n" ;
  os << stepStackHandler << endl ;

  double SamplingPeriod = obj.getSamplingPeriod() ;
  os << "The sampling period is :\n" ;
  os << SamplingPeriod << endl ;

  //  Eigen::Matrix4d P = obj.GetCurrentPositionofWaistInCOMFrame();
  //  os<< "The current position of waist in COM frame is :\n" ;
  //      << P[0,0]) << " " << P[0,1]) << " "
  //      << P[0,2]) << " " << P[0,3]) << endl
  //      << P[1,0]) << " " << P[1,1]) << " "
  //      << P[1,2]) << " " << P[1,3]) << endl
  //      << P[2,0]) << " " << P[2,1]) << " "
  //      << P[2,2]) << " " << P[2,3]) << endl
  //      << P[3,0]) << " " << P[3,1]) << " "
  //      << P[3,2]) << " " << P[3,3]) << endl ;

  PinocchioRobot * Robot = obj.getPinocchioRobot();
  os << "The PinocchioRobot is at the adress :\n" << Robot << endl ;

  double HeightOfTheCoM = obj.GetHeightOfTheCoM();
  os << "The height of the robot is :\n" << HeightOfTheCoM << endl << endl ;

  return os;
}

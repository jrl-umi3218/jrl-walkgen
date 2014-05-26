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
						
  RegisterMethods();

  for(unsigned int i=0;i<3;i++)
    m_DiffBetweenComAndWaist[i] = 0.0;

  // By assumption on this implementation
  // the humanoid is assume to have 6 DOFs per leg.
  MAL_VECTOR_FILL(m_prev_Configuration,0.0);
  MAL_VECTOR_FILL(m_prev_Configuration1,0.0);
  MAL_VECTOR_FILL(m_prev_Velocity,0.0);
  MAL_VECTOR_FILL(m_prev_Velocity1,0.0);

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

  MAL_S3_VECTOR_FILL(m_COGInitialAnkles,0.0);
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
InitializationMaps(std::vector<CjrlJoint *> &FromRootToJoint,
		   std::vector<CjrlJoint *> &ActuatedJoints,
		   std::vector<int> &IndexinConfiguration )
{
  if (FromRootToJoint.size()!=0)
    {
      ODEBUG("Enter 6.0 " << this);
      IndexinConfiguration.resize(FromRootToJoint.size());

      ODEBUG("Enter 6.1 " );      
      int lindex =0;
      
      // Here we assume that they are in a decending order.
      for(unsigned int i=0;i<FromRootToJoint.size();i++)
	{
	  IndexinConfiguration[lindex] = FromRootToJoint[i]->rankInConfiguration();
	  lindex++;
	}
    }
}

void ComAndFootRealizationByGeometry::
InitializeMapsForAHand(CjrlHand * aHand,
		       std::vector<CjrlJoint *> &ActuatedJoints,
		       vector<int> & IndexesInConfiguration,
		       CjrlJoint * & associateShoulder)
{
  if (aHand==0)
    return;

  // Find back the path from the shoulder to the left hand.
  //  CjrlHand *LeftHand = getHumanoidDynamicRobot()->leftHand();
  CjrlJoint *Chest = getHumanoidDynamicRobot()->chest();
  if (Chest==0)
    return;
  
  const CjrlJoint * associatedWrist = aHand->associatedWrist();
  if (associatedWrist==0)
    return;

  std::vector<CjrlJoint *> FromRootToJoint =
    getHumanoidDynamicRobot()->jointsBetween(*Chest, *associatedWrist);

  std::vector<CjrlJoint *>::iterator itJoint = FromRootToJoint.begin();
  associateShoulder = FromRootToJoint[0];
  InitializationMaps(FromRootToJoint,ActuatedJoints,
		     IndexesInConfiguration);
}

void ComAndFootRealizationByGeometry::
InitializeMapForChest(std::vector<CjrlJoint *> &ActuatedJoints)
{

  CjrlJoint *Chest = getHumanoidDynamicRobot()->chest();
  if (Chest==0)
    return;

  std::vector<CjrlJoint *> FromRootToJoint2,FromRootToJoint;
  FromRootToJoint = Chest->jointsFromRootToThis();
  std::vector<CjrlJoint *>::iterator itJoint = FromRootToJoint.begin();
  bool startadding=false;
  while(itJoint!=FromRootToJoint.end())
    {
      std::vector<CjrlJoint *>::iterator current = itJoint;
      
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
    }
  InitializationMaps(FromRootToJoint2,ActuatedJoints,m_ChestIndexinConfiguration);

}
void ComAndFootRealizationByGeometry::
Initialization()
{

  ODEBUG("Enter 0.0 ");

  // Planners for stepping over.
  if (m_WaistPlanner == 0)
    m_WaistPlanner = new WaistHeightVariation();
  if (m_UpBody==0)
    m_UpBody = new UpperBodyMotion();

  ODEBUG("Enter 1.0 ");
  
  ODEBUG("Enter 2.0 ");
  // Take the right ankle position (should be equivalent)
  vector3d lAnklePositionRight,lAnklePositionLeft;
  CjrlFoot *LeftFoot, *RightFoot;
  LeftFoot = getHumanoidDynamicRobot()->leftFoot();
  if (LeftFoot==0)
    LTHROW("No left foot");

  RightFoot = getHumanoidDynamicRobot()->rightFoot();
  if (RightFoot==0)
    LTHROW("No right foot");
    
  RightFoot->getAnklePositionInLocalFrame(lAnklePositionRight);
  LeftFoot->getAnklePositionInLocalFrame(lAnklePositionLeft);

  double lWidth,lHeight,lDepth;
  lDepth = lAnklePositionRight[2];
  LeftFoot->getSoleSize(lWidth,lHeight);

  m_AnklePositionRight[0] = lAnklePositionRight[0];
  m_AnklePositionLeft[0]  = lAnklePositionLeft[0];
  m_AnklePositionRight[1] = lAnklePositionRight[1];
  m_AnklePositionLeft[1]  = lAnklePositionLeft[1];
  m_AnklePositionRight[2] = lAnklePositionRight[2];
  m_AnklePositionLeft[2]  = lAnklePositionRight[2];
      
  ODEBUG4("m_AnklePositionRight: "<< m_AnklePositionRight[0] << " " <<
	 m_AnklePositionRight[1] << " " <<
	  m_AnklePositionRight[2],"DebugDataStartingCOM.dat");
  ODEBUG4("m_AnklePositionLeft: "<< m_AnklePositionLeft[0] << " " <<
	 m_AnklePositionLeft[1] << " " <<
	  m_AnklePositionLeft[2],"DebugDataStartingCOM.dat");
  ODEBUG("Enter 3.0 ");
  // Update the index to change the configuration according
  // to the VRML ID.
  ODEBUG("Enter 5.0 ");
  // Extract the indexes of the Right leg.
  CjrlJoint *waist = getHumanoidDynamicRobot()->waist();
  
  if (RightFoot->associatedAnkle()==0)
    LTHROW("No right ankle");

  std::vector<CjrlJoint *> FromRootToJoint2,FromRootToJoint = 
    getHumanoidDynamicRobot()->jointsBetween(*waist, *(RightFoot->associatedAnkle()));
  std::vector<CjrlJoint *> ActuatedJoints = 
    getHumanoidDynamicRobot()->getActuatedJoints();
  ODEBUG4("Size of ActuatedJoints"<<ActuatedJoints.size(),"DebugDataStartingCOM.dat");
  // Build global map.
  m_GlobalVRMLIDtoConfiguration.resize(ActuatedJoints.size());
  for(unsigned int j=0;j<ActuatedJoints.size();j++)
    {
      m_GlobalVRMLIDtoConfiguration[j] = ActuatedJoints[j]->rankInConfiguration();
    }


  // Build Right and left leg map.
  // Remove the first element
  FromRootToJoint.erase(FromRootToJoint.begin());
  InitializationMaps(FromRootToJoint,ActuatedJoints,
		     m_RightLegIndexinConfiguration);

  FromRootToJoint = 
    getHumanoidDynamicRobot()->jointsBetween(*waist, *(LeftFoot->associatedAnkle()));

  FromRootToJoint.erase(FromRootToJoint.begin());
  InitializationMaps(FromRootToJoint,ActuatedJoints,
		     m_LeftLegIndexinConfiguration);

  // Create maps for the left hand.
  InitializeMapsForAHand(getHumanoidDynamicRobot()->leftHand(),
			 ActuatedJoints,
			 m_LeftArmIndexinConfiguration,
			 m_LeftShoulder);
  
  // Create maps for the right hand.
  InitializeMapsForAHand(getHumanoidDynamicRobot()->rightHand(),
			 ActuatedJoints,
			 m_RightArmIndexinConfiguration,
			 m_RightShoulder);
  
  FromRootToJoint.clear();
  FromRootToJoint2.clear();


  
  ODEBUG("RightLegIndex: "
	 << m_RightLegIndexinConfiguration[0] << " "
	 << m_RightLegIndexinConfiguration[1] << " "
	 << m_RightLegIndexinConfiguration[2] << " "
	 << m_RightLegIndexinConfiguration[3] << " "
	 << m_RightLegIndexinConfiguration[4] << " "
	 << m_RightLegIndexinConfiguration[5] );
  ODEBUG("LeftLegIndex: "
	 << m_LeftLegIndexinConfiguration[0] << " "
	 << m_LeftLegIndexinConfiguration[1] << " "
	 << m_LeftLegIndexinConfiguration[2] << " "
	 << m_LeftLegIndexinConfiguration[3] << " "
	 << m_LeftLegIndexinConfiguration[4] << " "
	 << m_LeftLegIndexinConfiguration[5] );
  
  ODEBUG("Enter 12.0 ");
  
  MAL_VECTOR_FILL(m_prev_Configuration,0.0);
  MAL_VECTOR_FILL(m_prev_Configuration1,0.0);
  MAL_VECTOR_FILL(m_prev_Velocity,0.0);
  MAL_VECTOR_FILL(m_prev_Velocity1,0.0);

}

ComAndFootRealizationByGeometry::~ComAndFootRealizationByGeometry()
{
  if (m_WaistPlanner!=0)
    delete m_WaistPlanner;

  if (m_UpBody!=0)
    delete m_UpBody;

}

bool ComAndFootRealizationByGeometry::
InitializationHumanoid(MAL_VECTOR_TYPE(double) &BodyAnglesIni,
		   MAL_VECTOR_TYPE(double) & lStartingWaistPose)
{
  // For initialization we read the current value inside
  // the model. But we do not use it.
  MAL_VECTOR_TYPE(double) CurrentConfig = 
    getHumanoidDynamicRobot()->currentConfiguration();

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
  for(unsigned int i=0;i<m_GlobalVRMLIDtoConfiguration.size();i++)
    {
      CurrentConfig[m_GlobalVRMLIDtoConfiguration[i]] = BodyAnglesIni[i];
    }  

  CjrlHumanoidDynamicRobot *aDMB =  getHumanoidDynamicRobot();
  aDMB->currentConfiguration(CurrentConfig);

  // Compensate for the static translation, not the WAIST position
  // but it is the body position which start on the ground.
  {
    string inProperty[2]={"ComputeCoM",
			  "ComputeZMP"};
    string inValue[2]={"true","false"};
    for(unsigned int i=0;i<2;i++)
      aDMB->setProperty(inProperty[i],inValue[i]);

  }

  aDMB->computeForwardKinematics();

  CurrentConfig = aDMB->currentConfiguration();


  // Set the waist position.
  lStartingWaistPose(0) = CurrentConfig(0); // 0.0
  lStartingWaistPose(1) = CurrentConfig(1); // 0.0
  lStartingWaistPose(2) = CurrentConfig(2);
  lStartingWaistPose(3) = CurrentConfig(3); //0.0;
  lStartingWaistPose(4) = CurrentConfig(4); //0.0;
  lStartingWaistPose(5) = CurrentConfig(5); //0.0;

  ODEBUG("Current Config: " << CurrentConfig);
  return true;

}

bool ComAndFootRealizationByGeometry::
InitializationFoot(CjrlFoot * aFoot,
		   MAL_S3_VECTOR(& m_AnklePosition,double),
		   FootAbsolutePosition & InitFootPosition)
{
  const CjrlJoint * AnkleJoint = aFoot->associatedAnkle();
  matrix4d lFootPose = AnkleJoint->currentTransformation();
  MAL_S4x4_MATRIX_TYPE(double) FootTranslation;
  MAL_S4x4_MATRIX_SET_IDENTITY(FootTranslation);

  for(unsigned int i=0;i<3;i++)
    MAL_S4x4_MATRIX_ACCESS_I_J(FootTranslation, i,3) = -
      m_AnklePosition[i];
  
  lFootPose = MAL_S4x4_RET_A_by_B(lFootPose,FootTranslation);

  InitFootPosition.x = MAL_S4x4_MATRIX_ACCESS_I_J(lFootPose, 0,3);
  InitFootPosition.y = MAL_S4x4_MATRIX_ACCESS_I_J(lFootPose, 1,3);
  InitFootPosition.z = MAL_S4x4_MATRIX_ACCESS_I_J(lFootPose, 2,3);

  // Renormalize the feet orientation
  matrix4d initialRot;
  initialRot = AnkleJoint->initialPosition();
  matrix4d invrot;
  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      {
  	MAL_S4x4_MATRIX_ACCESS_I_J(invrot,i,j)=0.0;
  	for(unsigned int k=0;k<3;k++)
  	  {
  	    MAL_S4x4_MATRIX_ACCESS_I_J(invrot,i,j)+=
  	      MAL_S4x4_MATRIX_ACCESS_I_J(lFootPose,i,k) *
  	      MAL_S4x4_MATRIX_ACCESS_I_J(initialRot,j,k);
  	  }
      }
  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      MAL_S4x4_MATRIX_ACCESS_I_J(lFootPose,i,j) =
  	MAL_S4x4_MATRIX_ACCESS_I_J(invrot,i,j);

  // The foot *must be* flat on the floor...
  // Thus
  // lRightFootPose(0:2,0:2)=
  // coct    -st    -soct
  // cost     ct    -sost
  // so        0    co
  assert((MAL_S4x4_MATRIX_ACCESS_I_J(lFootPose, 2,1) == 0) &&
        "Error in the walk pattern generator initialization:" &&
        " Initial foot position is not flat");

  InitFootPosition.omega =
    atan2(MAL_S4x4_MATRIX_ACCESS_I_J(lFootPose, 2,0),
	  MAL_S4x4_MATRIX_ACCESS_I_J(lFootPose, 2,2))*180/M_PI;
  InitFootPosition.theta =
    atan2(-MAL_S4x4_MATRIX_ACCESS_I_J(lFootPose, 0,1),
	  MAL_S4x4_MATRIX_ACCESS_I_J(lFootPose, 1,1))*180/M_PI;

  return true;
}

/*! This initialization phase does the following:
  1/ we take the current state of the robot
  to compute the current CoM value.
  2/ We deduce the difference between the CoM and the waist,
  which is suppose to be constant for the all duration of the motion.
*/
bool ComAndFootRealizationByGeometry::
InitializationCoM(MAL_VECTOR_TYPE(double) &BodyAnglesIni,
		  MAL_S3_VECTOR_TYPE(double) & lStartingCOMPosition,
		  MAL_VECTOR_TYPE(double) & lStartingWaistPose,
		  FootAbsolutePosition & InitLeftFootPosition,
		  FootAbsolutePosition & InitRightFootPosition)
{

  /* Initialize properly the left and right initial positions of the feet. */
  memset((char *)&InitLeftFootPosition,0,sizeof(FootAbsolutePosition));
  memset((char *)&InitRightFootPosition,0,sizeof(FootAbsolutePosition));

  // Compute the forward dynamics from the configuration vector provided by the user. 
  // Initialize waist pose.
  InitializationHumanoid(BodyAnglesIni,lStartingWaistPose);

  // Initialise the right foot position.
  CjrlHumanoidDynamicRobot *aDMB =  getHumanoidDynamicRobot();
  CjrlFoot * RightFoot = aDMB->rightFoot();
  CjrlFoot * LeftFoot = aDMB->leftFoot();

  // Initialize Feet.
  InitializationFoot(RightFoot, m_AnklePositionRight,InitRightFootPosition);
  ODEBUG("InitRightFootPosition : " << InitRightFootPosition.x 
	  << " " << InitRightFootPosition.y 
	  << " " << InitRightFootPosition.z << std::endl
	  << " Ankle: " << m_AnklePositionRight);

  InitializationFoot(LeftFoot,  m_AnklePositionLeft, InitLeftFootPosition);
  ODEBUG("InitLeftFootPosition : " << InitLeftFootPosition.x 
	  << " " << InitLeftFootPosition.y 
	  << " " << InitLeftFootPosition.z << std::endl
	  << " Ankle: " << m_AnklePositionLeft);
  
  // Compute the center of gravity between the ankles.
  MAL_S3_VECTOR_ACCESS(m_COGInitialAnkles,0) = 
    0.5 * (InitRightFootPosition.x + InitLeftFootPosition.x);
  MAL_S3_VECTOR_ACCESS(m_COGInitialAnkles,1) = 
    0.5 * (InitRightFootPosition.y + InitLeftFootPosition.y);
  MAL_S3_VECTOR_ACCESS(m_COGInitialAnkles,2) = 
    0.5 * (InitRightFootPosition.z + InitLeftFootPosition.z);
  ODEBUG("COGInitialAnkle : "<<m_COGInitialAnkles);

  // Translate lStartingWaist Pose from ( 0.0 0.0 -lFootPosition[2])
  lStartingWaistPose(2) -= InitRightFootPosition.z;

  // CoM position
  lStartingCOMPosition = getHumanoidDynamicRobot()->positionCenterOfMass();
  ODEBUG4( "COM positions: "
	   << lStartingCOMPosition[0] << " "
	   << lStartingCOMPosition[1] << " "
	   << lStartingCOMPosition[2],"DebugDataStartingCOM.dat");
  ODEBUG( lStartingCOMPosition[0] << " "
	  << lStartingCOMPosition[1] << " "
	  << lStartingCOMPosition[2]);
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
      m_TranslationToTheLeftHip(i)  = 
	m_StaticToTheLeftHip(i)  + m_DiffBetweenComAndWaist[i];
      m_TranslationToTheRightHip(i) = 
	m_StaticToTheRightHip(i) + m_DiffBetweenComAndWaist[i];
    }

  // Initialize previous configuration vector
  MAL_VECTOR_FILL(m_prev_Configuration,0.0);
  MAL_VECTOR_FILL(m_prev_Configuration1,0.0);
  MAL_VECTOR_FILL(m_prev_Velocity,0.0);
  MAL_VECTOR_FILL(m_prev_Velocity1,0.0);

  InitLeftFootPosition.z = 0.0;
  InitRightFootPosition.z = 0.0;

  return true;
}

bool ComAndFootRealizationByGeometry::
InitializationUpperBody(deque<ZMPPosition> &inZMPPositions,
			deque<COMPosition> &inCOMBuffer,
			deque<RelativeFootPosition> lRelativeFootPositions)
{

  // Check pre-condition.
  if (getHumanoidDynamicRobot()==0)
    {
      cerr << "ComAndFootRealizationByGeometry::InitializationUpperBody "  << endl
	   << "No Humanoid Specificites class given " << endl;
      return false;
    }

  //  FootAbsolutePosition InitLeftFootAbsPos, InitRightFootAbsPos;
  struct timeval begin,  time2, time3;// end, time1, time4, time5, time6;

  gettimeofday(&begin,0);

  ODEBUG6("FinishAndRealizeStepSequence() - 1","DebugGMFKW.dat");
  cout << __FUNCTION__ << ":"<< __LINE__ << ": You should implement something here"
       << endl;
 
  ODEBUG6("FinishAndRealizeStepSequence() - 3 ","DebugGMFKW.dat");

  gettimeofday(&time2,0);
  //this function calculates a buffer with COM values after a first preview round,
  // currently required to calculate the arm swing before "onglobal step of control"
  // in order to take the arm swing motion into account in the second preview loop
  deque<ZMPPosition> aZMPBuffer;

  aZMPBuffer.resize(inCOMBuffer.size());

  ODEBUG6("FinishAndRealizeStepSequence() - 4 ","DebugGMFKW.dat");

  // read upperbody data which has to be included in the patterngeneration second preview loop stability
  string BodyDat = "UpperBodyDataFile.dat";

  ODEBUG6("FinishAndRealizeStepSequence() - 5 ","DebugGMFKW.dat");
  // Link the current trajectory and GenerateMotionFromKineoWorks.

  // Specify the buffer.

  m_UpperBodyPositionsBuffer.clear();


  if (GetStepStackHandler()->GetWalkMode()==3)
    {
      ODEBUG6("Before Starting GMFKW","DebugGMFKW.dat");
      m_GMFKW->CreateBufferFirstPreview(inZMPPositions);

      // Map the path found by KineoWorks onto the ZMP buffer.
      m_ConversionForUpperBodyFromLocalIndexToRobotDOFs.clear();

      m_GMFKW->ComputeUpperBodyPosition(m_UpperBodyPositionsBuffer,
					m_ConversionForUpperBodyFromLocalIndexToRobotDOFs);
      ODEBUG6("After GMFKW","DebugGMFKW.dat");
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

	  MAL_VECTOR_TYPE(double) currentConfiguration;
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

  ODEBUG6("FinishAndRealizeStepSequence() - 6 ","DebugCG.ctx");


  gettimeofday(&time3,0);


  if ((GetStepStackHandler()->GetWalkMode()==1)	||
      (GetStepStackHandler()->GetWalkMode()==3)	)
    {
      m_WaistPlanner->PolyPlanner(inCOMBuffer,lRelativeFootPositions,inZMPPositions);
    }

  ODEBUG("inZMPPositions : " << inZMPPositions.size() << endl <<
	 "m_COMBuffer : " << inCOMBuffer.size() << endl);

  return true;

}

bool ComAndFootRealizationByGeometry::
KinematicsForOneLeg(MAL_S3x3_MATRIX_TYPE(double) & Body_R,
		    MAL_S3_VECTOR_TYPE(double) & Body_P,
		    MAL_VECTOR_TYPE(double) &aFoot,
		    MAL_S3_VECTOR_TYPE(double) & lDt,
		    MAL_VECTOR_TYPE(double) &  aCoMPosition,
		    MAL_S3_VECTOR_TYPE(double) & ToTheHip,
		    int LeftOrRight,
		    MAL_VECTOR_TYPE(double) &lq,
		    int Stage)
{

  // Foot Orientation
  MAL_S3x3_MATRIX(Foot_R,double);
  // Foot position
  MAL_S3_VECTOR(Foot_P,double);

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

  //      cout << ZMPRefPositions[lindex].theta << " " << LeftFootPosition[lindex].theta;
  // Orientation
  Foot_R(0,0) = c*co;       Foot_R(0,1) = -s;       Foot_R(0,2) = c*so;
  Foot_R(1,0) = s*co;       Foot_R(1,1) =  c;       Foot_R(1,2) = s*so;
  Foot_R(2,0) = -so;        Foot_R(2,1) = 0;        Foot_R(2,2) = co;

  // position
  Foot_P(0) = aFoot(0);Foot_P(1) = aFoot(1); Foot_P(2) = aFoot(2);

  MAL_S3_VECTOR(Foot_Shift,double);

  CjrlJoint *Ankle = 0;
  if (LeftOrRight==-1)
    {
      MAL_S3x3_C_eq_A_by_B(Foot_Shift, Foot_R,m_AnklePositionRight);
      Ankle = getHumanoidDynamicRobot()->rightAnkle();
    }
  else if (LeftOrRight==1)
    {
      MAL_S3x3_C_eq_A_by_B(Foot_Shift, Foot_R,m_AnklePositionLeft);
      Ankle = getHumanoidDynamicRobot()->leftAnkle();
    }

  Foot_P = Foot_P + Foot_Shift;
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

  ODEBUG4("Body_R " << Body_R,"DebugDataIK.dat");
  ODEBUG4("Foot_P " << Foot_P,"DebugDataIK.dat");
  ODEBUG4("Foot_R " << Foot_R,"DebugDataIK.dat");
  ODEBUG4("lDt " << lDt,"DebugDataIK.dat");

  // Homogeneous matrix
  matrix4d BodyPose,FootPose;
  for(unsigned int i=0;i<3;i++)
    {
      for(unsigned int j=0;j<3;j++)
	{
	  MAL_S4x4_MATRIX_ACCESS_I_J(BodyPose,i,j) = 
	    MAL_S3x3_MATRIX_ACCESS_I_J(Body_R,i,j);
	  MAL_S4x4_MATRIX_ACCESS_I_J(FootPose,i,j) = 
	    MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R,i,j);
	}
      MAL_S4x4_MATRIX_ACCESS_I_J(BodyPose,i,3) = MAL_S3_VECTOR_ACCESS(Body_P,i);
      MAL_S4x4_MATRIX_ACCESS_I_J(FootPose,i,3) = MAL_S3_VECTOR_ACCESS(Foot_P,i);
    }


  CjrlJoint *Waist = getHumanoidDynamicRobot()->waist();
  
  ODEBUG("Typeid of humanoid: " << typeid(getHumanoidDynamicRobot()).name() );
  // Call specialized dynamics.
  getHumanoidDynamicRobot()->getSpecializedInverseKinematics(*Waist,*Ankle,BodyPose,FootPose,lq);

	  
  return true;
}

bool ComAndFootRealizationByGeometry::
KinematicsForTheLegs(MAL_VECTOR_TYPE(double) & aCoMPosition,
		     MAL_VECTOR_TYPE(double) & aLeftFoot,
		     MAL_VECTOR_TYPE(double) & aRightFoot,
		     int Stage,
		     MAL_VECTOR_TYPE(double) & ql,
		     MAL_VECTOR_TYPE(double) & qr,
		     MAL_S3_VECTOR_TYPE(double) & AbsoluteWaistPosition )

{

  // Body attitude
  MAL_S3x3_MATRIX_TYPE(double) Body_R;
  // Body position
  MAL_S3_VECTOR(Body_P,double);
  // To the hip
  MAL_S3_VECTOR(ToTheHip,double);

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

  MAL_S3x3_C_eq_A_by_B(ToTheHip,Body_R , m_TranslationToTheLeftHip);
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
      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 0,0) = CosTheta*CosOmega;
      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 0,1) = -SinTheta;
      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 0,2) = CosTheta*SinOmega;

      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 1,0) = SinTheta*CosOmega;
      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 1,1) = CosTheta;
      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 1,2) = SinTheta*SinOmega;

      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 2,0) = -SinOmega;
      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 2,1)=  0;
      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 2,2) = CosOmega;

      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 0,3) = aCoMPosition(0);
      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 1,3) = aCoMPosition(1);
      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 2,3) = aCoMPosition(2);
      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 3,3) = 1.0;

      ODEBUG4(Body_P ,"DebugDataBodyP1.dat");
    }
  else
    {
      ODEBUG4(Body_P ,"DebugDataBodyP0.dat");
    }


  // Kinematics for the left leg.
  ODEBUG4("Stage " << Stage,"DebugDataIK.dat");
  ODEBUG4("* Left Lego *","DebugDataIK.dat");
  KinematicsForOneLeg(Body_R,Body_P,aLeftFoot,m_DtLeft,aCoMPosition,ToTheHip,1,ql,Stage);


  // Kinematics for the right leg.
  MAL_S3x3_C_eq_A_by_B(ToTheHip, Body_R, m_TranslationToTheRightHip);
  Body_P(0) = aCoMPosition(0) + ToTheHip(0);
  Body_P(1) = aCoMPosition(1) + ToTheHip(1);
  Body_P(2) = aCoMPosition(2) + ToTheHip(2);

  ODEBUG4("* Right Leg *","DebugDataIK.dat");
  KinematicsForOneLeg(Body_R,Body_P,aRightFoot,m_DtRight,aCoMPosition,ToTheHip,-1,qr,Stage);

  ODEBUG4("**************","DebugDataIK.dat");
  /* Should compute now the Waist Position */
  MAL_S3x3_C_eq_A_by_B(m_ComAndWaistInRefFrame, Body_R, m_DiffBetweenComAndWaist);

  AbsoluteWaistPosition[0] = aCoMPosition(0) + m_ComAndWaistInRefFrame[0];
  AbsoluteWaistPosition[1] = aCoMPosition(1) + m_ComAndWaistInRefFrame[1];
  AbsoluteWaistPosition[2] = aCoMPosition(2) + ToTheHip(2) ;// - 0.705;

  return true;
}

bool ComAndFootRealizationByGeometry::
ComputePostureForGivenCoMAndFeetPosture(MAL_VECTOR_TYPE(double) & aCoMPosition,
					MAL_VECTOR_TYPE(double) & aCoMSpeed,
					MAL_VECTOR_TYPE(double) & aCoMAcc,
					MAL_VECTOR_TYPE(double) & aLeftFoot,
					MAL_VECTOR_TYPE(double) & aRightFoot,
					MAL_VECTOR_TYPE(double) & CurrentConfiguration,
					MAL_VECTOR_TYPE(double) & CurrentVelocity,
					MAL_VECTOR_TYPE(double) & CurrentAcceleration,
					int IterationNumber,
					int Stage)
{
  
  MAL_VECTOR(lqr,double);
  MAL_VECTOR(lql,double);

  MAL_S3_VECTOR(AbsoluteWaistPosition,double);

  // Kinematics for the legs.
  KinematicsForTheLegs(aCoMPosition,
		       aLeftFoot,
		       aRightFoot,
		       Stage,
		       lql,
		       lqr,
		       AbsoluteWaistPosition);
  /// NOW IT IS ABOUT THE UPPER BODY... ////
  MAL_VECTOR_DIM(qArmr,double,6);
  MAL_VECTOR_DIM(qArml,double,6);

  for(unsigned int i=0;i<MAL_VECTOR_SIZE(qArmr);i++)
    {
      qArmr[i] = 0.0;
      qArml[i] = 0.0;
    }

  if (GetStepStackHandler()->GetWalkMode()<3)
    {
      MAL_VECTOR_DIM(lAbsoluteWaistPosition,double,6);
      for(unsigned int i=0;i<3;i++)
	{
	  lAbsoluteWaistPosition(i) = MAL_S3_VECTOR_ACCESS(AbsoluteWaistPosition,i);
	  lAbsoluteWaistPosition(i+3) = aCoMPosition(i+3);
	}
      ODEBUG("AbsoluteWaistPosition:" << lAbsoluteWaistPosition  << 
	     " ComPosition" << aCoMPosition);
      
      ComputeUpperBodyHeuristicForNormalWalking(qArmr,
						qArml,
						lAbsoluteWaistPosition,
						aRightFoot,
						aLeftFoot);
    }

  // For stepping over modify the waist position and
  // according to parameters the arms motion.
  if(GetStepStackHandler()->GetWalkMode()==2)
    {

      ///this angle is introduced to rotate the upperbody when the waist is rotated during stepover
      double qWaistYaw = -CurrentConfiguration(m_ChestIndexinConfiguration[0])*M_PI/180.0;
      ODEBUG4(qWaistYaw,"DebugDataWaistYaw.dat");
      //this is not correct yet since it uses COMPositionFromPC1.theta which also changes when turning....
      //it will be modified in the near future
      //include waistrotation in dynamic model for second preview correction

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

  ODEBUG( "ComAndFoot: AbsoluteWaistPosition: " << AbsoluteWaistPosition << endl
	  << "CoMPosition: " << aCoMPosition );
  ODEBUG("Left FootPosition: " << aLeftFoot <<
	  " Right FootPosition: " << aRightFoot );
  /* Update of the configuration and velocity vector */
  for(int i=0;i<3;i++)
    CurrentConfiguration[i] = AbsoluteWaistPosition(i);

  for(int i=3;i<6;i++)
    CurrentConfiguration[i] = aCoMPosition(i)*M_PI/180.0;

  for(unsigned int i=0;i<MAL_VECTOR_SIZE(lqr);i++)
    CurrentConfiguration[m_RightLegIndexinConfiguration[i]] = lqr[i];

  for(unsigned int i=0;i<MAL_VECTOR_SIZE(lql);i++)
    CurrentConfiguration[m_LeftLegIndexinConfiguration[i]] = lql[i];

  for(unsigned int i=0;i<MAL_VECTOR_SIZE(qArmr);i++)
    CurrentConfiguration[m_RightArmIndexinConfiguration[i]] = qArmr[i];

  for(unsigned int i=0;i<MAL_VECTOR_SIZE(qArml);i++)
    CurrentConfiguration[m_LeftArmIndexinConfiguration[i]] = qArml[i];

  // Update the speed values.
  /* If this is the first call ( stage = 0)
     we should update the current stored values.  */
  /* Initialize the acceleration */
  for(unsigned int i=0;i<MAL_VECTOR_SIZE(CurrentAcceleration);i++)
    {
      CurrentVelocity[i]=0.0;
      CurrentAcceleration[i] = 0.0;
      /* Keep the new value for the legs. */
    }
  
  double ldt =  getSamplingPeriod();

  if (Stage==0)
    {
      if (IterationNumber>0)
	{
	  /* Compute the speed */
	  for(unsigned int i=6;i<MAL_VECTOR_SIZE(m_prev_Configuration);i++)
	    {
	      CurrentVelocity[i] = (CurrentConfiguration[i] - m_prev_Configuration[i])/ ldt;
	      /* Keep the new value for the legs. */
	    }

	  if (IterationNumber>1)
	    {
	      for(unsigned int i=6;i<MAL_VECTOR_SIZE(m_prev_Velocity);i++)
		CurrentAcceleration[i] = (CurrentVelocity[i] - m_prev_Velocity[i])/ ldt;
	    }
	}
      else
	{
	  /* Compute the speed */
	  for(unsigned int i=0;i<MAL_VECTOR_SIZE(CurrentVelocity);i++)
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
	  for(unsigned int i=6;i<MAL_VECTOR_SIZE(m_prev_Configuration1);i++)
	    {
	      CurrentVelocity[i] = (CurrentConfiguration[i] - m_prev_Configuration1[i])/ getSamplingPeriod();
	      /* Keep the new value for the legs. */
	    }
	  if (IterationNumber>1)
	    {
	      for(unsigned int i=6;i<MAL_VECTOR_SIZE(m_prev_Velocity1);i++)
		CurrentAcceleration[i] = (CurrentVelocity[i] - m_prev_Velocity1[i])/ ldt;
	    }
	}
      else
	{
	  /* Compute the speed */
	  for(unsigned int i=0;i<MAL_VECTOR_SIZE(m_prev_Configuration1);i++)
	    {
	      CurrentVelocity[i] = 0.0;
	      /* Keep the new value for the legs. */
	    }
	}
      ODEBUG4(CurrentVelocity, "DebugDataVelocity1.dat");
      m_prev_Configuration1 = CurrentConfiguration;
      m_prev_Velocity1 = CurrentVelocity;
    }


  for(int i=0;i<6;i++)
    CurrentVelocity[i] = aCoMSpeed(i);
  
  for(int i=0;i<6;i++)
    CurrentAcceleration[i] = aCoMAcc(i);

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
EvaluateStartingCoM(MAL_VECTOR(&BodyAngles,double),
		    MAL_S3_VECTOR(&aStartingCOMPosition,double),
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
EvaluateStartingCoM(MAL_VECTOR(&BodyAngles,double),
		    MAL_S3_VECTOR(&aStartingCOMPosition,double),
		    MAL_VECTOR(&aWaistPose,double),
		    FootAbsolutePosition & InitLeftFootPosition,
		    FootAbsolutePosition & InitRightFootPosition)
{
  MAL_VECTOR(WaistPose,double);
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
EvaluateCOMForStartingPosition( MAL_VECTOR( &BodyAngles,double),
				double , // omega, 
				double , // theta,
				MAL_S3_VECTOR( &lCOMPosition,double),
				FootAbsolutePosition & InitLeftFootPosition,
				FootAbsolutePosition & InitRightFootPosition)
{
  MAL_VECTOR(lWaistPose,double);
  return InitializationCoM(BodyAngles,
			   lCOMPosition,
			   lWaistPose,
			   InitLeftFootPosition, InitRightFootPosition);

}


void ComAndFootRealizationByGeometry::
GetCurrentPositionofWaistInCOMFrame(MAL_VECTOR_TYPE(double) &CurPosWICF_homogeneous)
{
  for(int i=0;i<3;i++)
    CurPosWICF_homogeneous[i] = m_DiffBetweenComAndWaist[i];
  CurPosWICF_homogeneous[3] = 1.0;

}

void ComAndFootRealizationByGeometry::
ComputeUpperBodyHeuristicForNormalWalking(MAL_VECTOR_TYPE(double) & qArmr,
					  MAL_VECTOR_TYPE(double) & qArml,
					  MAL_VECTOR_TYPE(double) & aCOMPosition,
					  MAL_VECTOR_TYPE(double) & RFP,
					  MAL_VECTOR_TYPE(double) & LFP)
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

  matrix4d jointRootPosition,jointEndPosition;
  MAL_S4x4_MATRIX_SET_IDENTITY(jointRootPosition);
  MAL_S4x4_MATRIX_SET_IDENTITY(jointEndPosition);
  
  MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,0,3) = 
    TempALeft * m_GainFactor / 0.2;
  MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,2,3) = m_ZARM;
  
  getHumanoidDynamicRobot()->getSpecializedInverseKinematics(*m_LeftShoulder,
							     *getHumanoidDynamicRobot()->leftWrist(),
							     jointRootPosition,
							     jointEndPosition,
							     qArml);

  ODEBUG4("ComputeHeuristicArm: Step 2 ","DebugDataIKArms.txt");
  ODEBUG4( "IK Left arm p:" << qArml(0)<< " " <<  qArml(1)  << " " << qArml(2)
	   << " " << qArml(3) << "  " << qArml(4) << " " << qArml(5), "DebugDataIKArms.txt" );

  MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,0,3) = TempARight;
  MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,2,3) = m_ZARM;
  
  getHumanoidDynamicRobot()->getSpecializedInverseKinematics(*m_RightShoulder,
							     *getHumanoidDynamicRobot()->rightWrist(),
							     jointRootPosition,
							     jointEndPosition,
							     qArmr);
  ODEBUG4( "IK Right arm p:" << qArmr(0)<< " " <<  qArmr(1)  << " " << qArmr(2)
	   << " " << qArmr(3) << "  " << qArmr(4) << " " << qArmr(5), "DebugDataIKArms.txt" );

  ODEBUG4( qArml(0)<< " " <<  qArml(1)  << " " << qArml(2) << " "
	   << qArml(3) << "  " << qArml(4) << " " << qArml(5) << " "
	   << qArmr(0)<< " " <<  qArmr(1)  << " " << qArmr(2) << " "
	   << qArmr(3) << "  " << qArmr(4) << " " << qArmr(5), "DebugDataqArmsHeuristic.txt");
}

bool ComAndFootRealizationByGeometry::
setHumanoidDynamicRobot(CjrlHumanoidDynamicRobot * aHumanoidDynamicRobot)
{
  ComAndFootRealization::setHumanoidDynamicRobot(aHumanoidDynamicRobot);
  CjrlHumanoidDynamicRobot *aHDMB =  aHumanoidDynamicRobot;

  MAL_VECTOR_RESIZE(m_prev_Configuration,aHDMB->numberDof());
  MAL_VECTOR_RESIZE(m_prev_Configuration1,aHDMB->numberDof());

  for(unsigned int i=0;i<MAL_VECTOR_SIZE(m_prev_Configuration);i++)
    {
      m_prev_Configuration[i] = 0.0;
      m_prev_Configuration1[i] = 0.0;
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

MAL_S4x4_MATRIX_TYPE(double) ComAndFootRealizationByGeometry::
GetCurrentPositionofWaistInCOMFrame()
{
  MAL_S4x4_MATRIX_TYPE(double) P;

  MAL_S4x4_MATRIX_SET_IDENTITY(P);
  MAL_S4x4_MATRIX_ACCESS_I_J(P, 0,3) = m_DiffBetweenComAndWaist[0];
  MAL_S4x4_MATRIX_ACCESS_I_J(P, 1,3) = m_DiffBetweenComAndWaist[1];
  MAL_S4x4_MATRIX_ACCESS_I_J(P, 2,3) = m_DiffBetweenComAndWaist[2];

  return P;
}

MAL_S3_VECTOR_TYPE(double) ComAndFootRealizationByGeometry::GetCOGInitialAnkles()
{
  
  return m_COGInitialAnkles;
}

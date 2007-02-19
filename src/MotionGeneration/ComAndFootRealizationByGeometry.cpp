#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <time.h>

#include <ComAndFootRealizationByGeometry.h>
#include <HumanoidDynamicMultiBody.h>


#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "CAFRBG: " << x << endl; DebugFile.close();}
#else
#define RESETDEBUG4(y) 
#define ODEBUG4(x,y) 
#endif

#define RESETDEBUG6(y) 
#define ODEBUG6(x,y) 

#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "CAFRBG: " << x << endl; DebugFile.close();}
#if 1
#define ODEBUG(x)
#else
#define ODEBUG(x)  std::cout << x << endl;
#endif

#define ODEBUG3(x)  std::cout << x << endl;

using namespace PatternGeneratorJRL;

ComAndFootRealizationByGeometry::ComAndFootRealizationByGeometry()
{
  
  m_ZARM = -1.0;
  m_InverseKinematics = 0;
  for(unsigned int i=0;i<3;i++)
    m_DiffBetweenComAndWaist[i] = 0.0;

  // By assumption on this implementation
  // the humanoid is assume to have 6 DOFs per leg.
 

  m_AnkleSoilDistance = 0.1;

  RESETDEBUG4("DebugDataqr_0.dat");
  RESETDEBUG4("DebugDataqr_1.dat");
  RESETDEBUG4("DebugDataql_0.dat");
  RESETDEBUG4("DebugDataql_1.dat");
  RESETDEBUG4("DebugDataVelocity.dat");
  
  RESETDEBUG4("LegsSpeed.dat");
  RESETDEBUG4("COMPC1.dat");
  RESETDEBUG4("DebugDataIKR.dat");
  RESETDEBUG4("DebugDataIKL_0.dat");
  RESETDEBUG4("DebugDataIKL_1.dat");
  RESETDEBUG4("DebugDatamDtL.dat");
  RESETDEBUG4("DebugDataStartingCOM.dat");
}
  
void ComAndFootRealizationByGeometry::Initialization()
{

  // Planners for stepping over.
  m_WaistPlanner = new WaistHeightVariation();
  m_UpBody = new UpperBodyMotion();
  
  if (m_HS!=0)
    {
      double AnklePosition[3];
      // Take the right ankle position (should be equivalent)
      m_HS->GetAnklePosition(-1,AnklePosition);
      m_AnkleSoilDistance = AnklePosition[2];
      ODEBUG("AnkleSoilDistnace =" << m_AnkleSoilDistance);

      // Update the index to change the configuration according
      // to the VRML ID.
      
      HumanoidDynamicMultiBody * aHDMB = (HumanoidDynamicMultiBody *)getHumanoidDynamicRobot();
      aHDMB->GetJointIDInConfigurationFromVRMLID(m_GlobalVRMLIDtoConfiguration);

      // Extract the indexes of the Left leg.
      int r=m_HS->GetLegJointNb(1) + m_HS->GetFootJointNb(1);
      if (r!=6)
	{
	  cerr << " The number of joints for the left leg in " << endl
	       << " HRP2 specificities are not sufficient " << endl
	       << r << " were detected. " << endl;
	}
      else 
	{
	  m_LeftLegIndexInVRML.resize(6);
	  m_LeftLegIndexinConfiguration.resize(6);
	  
	  std::vector<int> tmp = m_HS->GetLegJoints(1);
	  int lindex =0;
	  // Here we assume that they are in a decending order.
	  for(int i=0;i<tmp.size();i++)
	    {
	      m_LeftLegIndexInVRML[lindex] = tmp[i];
	      m_LeftLegIndexinConfiguration[lindex] = m_GlobalVRMLIDtoConfiguration[tmp[i]];
	      lindex++;
	    }

	  tmp = m_HS->GetFootJoints(1);
	  for(int i=0;i<tmp.size();i++)
	    {
	      m_LeftLegIndexInVRML[lindex] = tmp[i];
	      m_LeftLegIndexinConfiguration[lindex] = m_GlobalVRMLIDtoConfiguration[tmp[i]];
	      lindex++;
	    }
	}

      // Extract the indexes of the Right leg.
      r=m_HS->GetLegJointNb(-1) + m_HS->GetFootJointNb(-1);
      if (r!=6)
	{
	  cerr << " The number of joints for the right leg in " << endl
	       << " HRP2 specificities are not good  " << endl
	       << r << " were detected." << endl;
	}
      else
	{
	   m_RightLegIndexInVRML.resize(6);
	   m_RightLegIndexinConfiguration.resize(6);
	  
	  std::vector<int> tmp = m_HS->GetLegJoints(-1);
	  int lindex =0;
	  // Here we assume that they are in a decending order.
	  for(int i=0;i<tmp.size();i++)
	    {
	      m_RightLegIndexInVRML[lindex] = tmp[i];
	      m_RightLegIndexinConfiguration[lindex] = m_GlobalVRMLIDtoConfiguration[tmp[i]];
	      lindex++;
	    }
	  
	  tmp = m_HS->GetFootJoints(-1);
	  for(int i=0;i<tmp.size();i++)
	    {
	      m_RightLegIndexInVRML[lindex] = tmp[i];
	      m_RightLegIndexinConfiguration[lindex] = m_GlobalVRMLIDtoConfiguration[tmp[i]];
	      lindex++;
	    }
	}
      
      // Extract the indexes of the Left Arm.
      r=m_HS->GetArmJointNb(1);
      if (r!=7)
	{
	  cerr << " The number of joints for the left arm in " << endl
	       << " HRP2 specificities are not sufficient " << endl
	       << r << " were detected. " << endl;
	}
      else 
	{
	  m_LeftArmIndexInVRML.resize(7);
	  m_LeftArmIndexinConfiguration.resize(7);
	  
	  std::vector<int> tmp = m_HS->GetArmJoints(1);
	  int lindex =0;
	  // Here we assume that they are in a decending order.
	  for(int i=0;i<tmp.size();i++)
	    {
	      m_LeftArmIndexInVRML[lindex] = tmp[i];
	      m_LeftArmIndexinConfiguration[lindex] = m_GlobalVRMLIDtoConfiguration[tmp[i]];
	      lindex++;
	    }
	}

      // Extract the indexes of the Right Arm.
      r=m_HS->GetArmJointNb(-1);
      if (r!=7)
	{
	  cerr << " The number of joints for the right arm in " << endl
	       << " HRP2 specificities are not sufficient " << endl
	       << r << " were detected. " << endl;
	}
      else 
	{
	  m_RightArmIndexInVRML.resize(7);
	  m_RightArmIndexinConfiguration.resize(7);
	  
	  std::vector<int> tmp = m_HS->GetArmJoints(-1);
	  int lindex =0;
	  // Here we assume that they are in a decending order.
	  for(int i=0;i<tmp.size();i++)
	    {
	      m_RightArmIndexInVRML[lindex] = tmp[i];
	      m_RightArmIndexinConfiguration[lindex] = m_GlobalVRMLIDtoConfiguration[tmp[i]];
	      lindex++;
	    }
	}

    }
  
  ODEBUG("RightLegIndex: " 
	  << m_RightLegIndexInVRML[0] << " " 
	  << m_RightLegIndexInVRML[1] << " " 
	  << m_RightLegIndexInVRML[2] << " " 
	  << m_RightLegIndexInVRML[3] << " " 
	  << m_RightLegIndexInVRML[4] << " " 
	  << m_RightLegIndexInVRML[5] << " " 
	  << m_RightLegIndexinConfiguration[0] << " " 
	  << m_RightLegIndexinConfiguration[1] << " " 
	  << m_RightLegIndexinConfiguration[2] << " " 
	  << m_RightLegIndexinConfiguration[3] << " " 
	  << m_RightLegIndexinConfiguration[4] << " " 
	  << m_RightLegIndexinConfiguration[5] );
  ODEBUG("LeftLegIndex: "
	  << m_LeftLegIndexInVRML[0] << " " 
	  << m_LeftLegIndexInVRML[1] << " " 
	  << m_LeftLegIndexInVRML[2] << " " 
	  << m_LeftLegIndexInVRML[3] << " " 
	  << m_LeftLegIndexInVRML[4] << " " 
	  << m_LeftLegIndexInVRML[5] << " " 
	  << m_LeftLegIndexinConfiguration[0] << " " 
	  << m_LeftLegIndexinConfiguration[1] << " " 
	  << m_LeftLegIndexinConfiguration[2] << " " 
	  << m_LeftLegIndexinConfiguration[3] << " " 
	  << m_LeftLegIndexinConfiguration[4] << " " 
	  << m_LeftLegIndexinConfiguration[5] ); 
}

ComAndFootRealizationByGeometry::~ComAndFootRealizationByGeometry()
{
}

/*! This initialization phase does the following:
  1/ we take the current state of the robot
  to compute the current CoM value.
  2/ We deduce the difference between the CoM and the waist,
  which is suppose to be constant for the all duration of the motion. 
*/
bool ComAndFootRealizationByGeometry::InitializationCoM(MAL_VECTOR(,double) &BodyAnglesIni,
							MAL_S3_VECTOR(,double) & lStartingCOMPosition,
							FootAbsolutePosition & InitLeftFootPosition, 
							FootAbsolutePosition & InitRightFootPosition)
{
  if (m_InverseKinematics==0)
    {
      cerr << "ComAndFootRealizationByGeometry::InitializationUpperBody "  << endl
	   << "No Inverse Kinematics class given " << endl;
      return false;
    }
  


  MAL_VECTOR(,double) CurrentConfig = getHumanoidDynamicRobot()->currentConfiguration();
  MAL_VECTOR(,double) CurrentVelocity = getHumanoidDynamicRobot()->currentVelocity();

  // Update the velocity.
  MAL_S3_VECTOR(RootPosition,double);
  MAL_S3_VECTOR(RootVelocity,double);
  MAL_S3x3_MATRIX(Body_Rm3d,double);
    
  RootVelocity[0] = CurrentVelocity[0];
  RootVelocity[1] = CurrentVelocity[1];
  RootVelocity[2] = CurrentVelocity[2];
  
  RootPosition[0] = CurrentConfig[0];
  RootPosition[1] = CurrentConfig[1];
  RootPosition[2] = CurrentConfig[2]; 

  double omega = CurrentConfig[3], theta = CurrentConfig[4];
  double c,s,co,so;
  ODEBUG4( "omega: " << omega << " theta: " << theta ,"DebugDataStartingCOM.dat"); 

  c = cos(theta*M_PI/180.0);
  s = sin(theta*M_PI/180.0);

  co = cos(omega*M_PI/180.0);
  so = sin(omega*M_PI/180.0);
  
  // COM Orientation
  Body_Rm3d(0,0) = c*co;        Body_Rm3d(0,1) = -s;      Body_Rm3d(0,2) = c*so;
  Body_Rm3d(1,0) = s*co;        Body_Rm3d(1,1) =  c;      Body_Rm3d(1,2) = s*so;
  Body_Rm3d(2,0) = -so;         Body_Rm3d(2,1) = 0;       Body_Rm3d(2,2) = co;

  // TODO: Update according the current state vector of the humanoid robot.
  
  ODEBUG4("Size of body angles ini: " << MAL_VECTOR_SIZE(BodyAnglesIni),
	  "DebugDataStartingCOM.dat");
  for(int i=0;i<MAL_VECTOR_SIZE(BodyAnglesIni);i++)
    {
      CurrentConfig[m_GlobalVRMLIDtoConfiguration[i]] = BodyAnglesIni[i];
      ODEBUG4( m_GlobalVRMLIDtoConfiguration[i] << " " << CurrentConfig[m_GlobalVRMLIDtoConfiguration[i]]*180/M_PI  , 
	         "DebugDataStartingCOM.dat");
	
    }
  
  getHumanoidDynamicRobot()->currentConfiguration(CurrentConfig);

  ODEBUG("Forward Kinematics for InitializationCoM");

  // Compensate for the static translation, not the WAIST position
  // but it is the body position which start on the ground.
  getHumanoidDynamicRobot()->computeForwardKinematics();
  
  ODEBUG4("Root Position:" 
	  << RootPosition[0] << " "
	  << RootPosition[1] << " "
	  << RootPosition[2] , "DebugDataStartingCOM.dat");
  
  MAL_S4x4_MATRIX(,double) lFootPose;
  lFootPose = getHumanoidDynamicRobot()->rightFoot()->initialPosition();
  
  MAL_S3_VECTOR(lFootPosition,double);
  
  for(int i=0;i<3;i++)
    lFootPosition(i)  = lFootPose(i,3);  

  MAL_S3_VECTOR(WaistPosition,double);

  WaistPosition[0] = 0.0;
  WaistPosition[1] = 0.0;
  WaistPosition[2] = -lFootPosition[2]+m_AnkleSoilDistance;

  InitRightFootPosition.x = lFootPosition[0];
  InitRightFootPosition.y = lFootPosition[1];
  InitRightFootPosition.z = 0.0;

  
  ODEBUG4( "Right Foot Position: " 
	   << lFootPosition[0] << " "
	   << lFootPosition[1] << " "
	   << lFootPosition[2] ,"DebugDataStartingCOM.dat");

  // For now we assume that the position of the hip is given 
  // by the first joint of HRP2Specificities.xml
  int LeftHipIndex = m_HS->GetLegJoints(-1)[0];

  // Get the associate pose.
  MAL_S4x4_MATRIX(,double) lLeftHipPose = getHumanoidDynamicRobot()->jointVector()[LeftHipIndex]->initialPosition();

  MAL_S3_VECTOR(LeftHip,double);
  for(int i=0;i<3;i++)
    LeftHip(i) = lLeftHipPose(i,3);

  ODEBUG4( "Left Hip Position: " 
	   << LeftHip[0] << " "
	   << LeftHip[1] << " "
	   << LeftHip[2],"DebugDataStartingCOM.dat" );
  
  lStartingCOMPosition = getHumanoidDynamicRobot()->positionCenterOfMass();
  ODEBUG4( "COM positions: " 
	   << lStartingCOMPosition[0] << " "
	   << lStartingCOMPosition[1] << " "
	   << lStartingCOMPosition[2],"DebugDataStartingCOM.dat");

  m_DiffBetweenComAndWaist[0] =  -lStartingCOMPosition[0];
  m_DiffBetweenComAndWaist[1] =  -lStartingCOMPosition[1];
  m_DiffBetweenComAndWaist[2] =  -lStartingCOMPosition[2] 
    -(GetHeightOfTheCoM() + lFootPosition[2] - m_AnkleSoilDistance - lStartingCOMPosition[2]); // This term is usefull if

  ODEBUG("m_DiffBetweenComAndWaist :" << m_DiffBetweenComAndWaist);
  // the initial position does not put z at Zc
  
  // The most important line of the method...
  // The one which initialize correctly the height of the pattern generator.
  for(int i=0;i<3;i++)
    {
      m_TranslationToTheLeftHip(i) = m_StaticToTheLeftHip(i) + m_DiffBetweenComAndWaist[i];
      m_TranslationToTheRightHip(i) = m_StaticToTheRightHip(i) + m_DiffBetweenComAndWaist[i];
    }

  MAL_VECTOR_DIM(lqr,double,6);
  MAL_S3x3_MATRIX(Foot_R,double);
  MAL_S3x3_MATRIX(Body_R,double);
  MAL_S3_VECTOR(Body_P,double);
  MAL_S3_VECTOR(Foot_P,double);
  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      {
	if (i!=j)
	  Foot_R(i,j) = 
	    Body_R(i,j) = 0.0;
	else 
	  Foot_R(i,j) = 
	    Body_R(i,j) = 1.0;
      }  

  MAL_S3_VECTOR(ToTheHip,double);

  ODEBUG(Body_R << endl << Foot_R );
  MAL_S3x3_C_eq_A_by_B(ToTheHip,Body_R, m_TranslationToTheRightHip);
  ODEBUG4("m_StaticToTheRightHip " << m_TranslationToTheRightHip, "DebugDataStartingCOM.dat");
  ODEBUG4( "ToTheHip " << ToTheHip , "DebugDataStartingCOM.dat");

  Body_P(0)= LeftHip[0];
  Body_P(1)= LeftHip[1];
  Body_P(2)= 0.0;

  lFootPose = getHumanoidDynamicRobot()->leftFoot()->initialPosition();  
  
  for(int i=0;i<3;i++)
    lFootPosition(i) =  lFootPose(i,3);  

  InitLeftFootPosition.x = lFootPosition[0];
  InitLeftFootPosition.y = lFootPosition[1];
  InitLeftFootPosition.z = 0.0;
  
  ODEBUG( "Body_R " << Body_R );
  ODEBUG( "Body_P " << Body_P );
  Foot_P(0) = lFootPosition[0];
  Foot_P(1) = lFootPosition[1];
  Foot_P(2) = lFootPosition[2];

  InitRightFootPosition.z = 0.0;
  ODEBUG( "Foot_P " << Foot_P );
  ODEBUG( "Foot_R " << Foot_R );
  ODEBUG( "m_Dt" << m_Dt );

  // RIGHT FOOT.
  m_Dt(1) = -m_Dt(1);
  
  // Compute the inverse kinematics.
  m_InverseKinematics->ComputeInverseKinematics2ForLegs(Body_R,
							Body_P,
							m_Dt,
							Foot_R,
							Foot_P,
							lqr);

  m_Dt(1)= -m_Dt(1);

}

bool ComAndFootRealizationByGeometry::InitializationUpperBody(deque<ZMPPosition> &inZMPPositions,
							      deque<COMPosition> &inCOMBuffer,
							      deque<RelativeFootPosition> lRelativeFootPositions)
{

  // Check pre-condition.
  if (m_HS==0)
    {
      cerr << "ComAndFootRealizationByGeometry::InitializationUpperBody "  << endl
	   << "No Humanoid Specificites class given " << endl;
      return false;
    }

  if (m_InverseKinematics==0)
    {
      cerr << "ComAndFootRealizationByGeometry::InitializationUpperBody "  << endl
	   << "No Inverse Kinematics class given " << endl;
      return false;
    }

  FootAbsolutePosition InitLeftFootAbsPos, InitRightFootAbsPos;
  struct timeval begin, end, time1, time2, time3, time4, time5, time6;
  
  gettimeofday(&begin,0);
  
  ODEBUG6("FinishAndRealizeStepSequence() - 1","DebugGMFKW.dat");
  m_Xmax = m_InverseKinematics->ComputeXmax(m_ZARM); // Laaaaaazzzzzyyyyy guy...
  
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

      int UpperBodyJointNb = m_HS->GetUpperBodyJointNb();
      m_ConversionForUpperBodyFromLocalIndexToRobotDOFs = m_HS->GetUpperBodyJoints();

      for(unsigned int i=0;i<m_ConversionForUpperBodyFromLocalIndexToRobotDOFs.size();i++)
	{
	  
	  m_ConversionForUpperBodyFromLocalIndexToRobotDOFs[i] = i;
	}
      
      for(unsigned int i=0;i<m_UpperBodyPositionsBuffer.size();i++)
	{
	  m_UpperBodyPositionsBuffer[i].Joints.resize(UpperBodyJointNb);
	  
	  MAL_VECTOR(,double) currentConfiguration;
	  currentConfiguration = getHumanoidDynamicRobot()->currentConfiguration();

	  if (i==0)
	    {
	      for(unsigned int j=0;j<UpperBodyJointNb;j++)
		m_UpperBodyPositionsBuffer[i].Joints[j] = currentConfiguration
		  [m_ConversionForUpperBodyFromLocalIndexToRobotDOFs[j]];
	    }
	  // Initialize the upper body motion to the current stored value.
	  for(unsigned int j=0;j<UpperBodyJointNb;j++)
	    m_UpperBodyPositionsBuffer[i].Joints[j] = m_UpperBodyPositionsBuffer[0].Joints[j];
	}
      

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


  ODEBUG6("FinishAndRealizeStepSequence() - 7 ","DebugGMFKW.dat");
#if _DEBUG_4_ACTIVATED_
  ofstream DebugFile;
  DebugFile.open("DebugDataCOMInitialization.txt",ofstream::out);
  for(unsigned int i=0;i<inCOMBuffer.size();i++)
    {

      for(int j=0;j<3;j++)
	{
	  DebugFile << inCOMBuffer[i].x[j] << " ";
	}

      for(int j=0;j<3;j++)
	{
	  DebugFile << inCOMBuffer[i].y[j] << " ";
	}

      for(int j=0;j<3;j++)
	{
	  DebugFile << inCOMBuffer[i].z[j] << " ";
	}

      DebugFile << inCOMBuffer[i].omega << " "
		<< inCOMBuffer[i].theta << " "
		<< inCOMBuffer[i].hip;

      DebugFile << endl;
    }

  DebugFile.close();
  
#endif 

    

}

bool ComAndFootRealizationByGeometry::KinematicsForOneLeg(MAL_S3x3_MATRIX(,double) & Body_R,
							  MAL_S3_VECTOR(,double) & Body_P,
							  MAL_VECTOR(,double) &aFoot,
							  MAL_S3_VECTOR(,double) &lDt,
							  MAL_VECTOR(,double) &aCoMPosition,
							  MAL_S3_VECTOR(,double) &ToTheHip,
							  MAL_VECTOR(,double) &lq)
{
  // Foot attitude 
  MAL_S3x3_MATRIX(Foot_R,double);
  // Foot position
  MAL_S3_VECTOR(Foot_P,double);

  // LEFT Foot.
  double FootPositiontheta = aFoot(3);
  double FootPositionomega = aFoot(4);
  double c,s,co,so;

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
  Foot_P(0)=aFoot(0)+ m_AnkleSoilDistance*so;
  Foot_P(1)=aFoot(1);
  Foot_P(2)=aFoot(2)+ m_AnkleSoilDistance*co -(aCoMPosition(2) + ToTheHip(2));

  // Compute the inverse kinematics.
  m_InverseKinematics->ComputeInverseKinematics2ForLegs(Body_R,
							Body_P,
							lDt,
							Foot_R,
							Foot_P,
							lq);
  
  return true;
}

bool ComAndFootRealizationByGeometry::KinematicsForTheLegs(MAL_VECTOR(,double) & aCoMPosition,
							   MAL_VECTOR(,double) & aLeftFoot,
							   MAL_VECTOR(,double) & aRightFoot,
							   int Stage,
							   MAL_VECTOR(,double) & ql,
							   MAL_VECTOR(,double) & qr,
							   MAL_S3_VECTOR(,double) & AbsoluteWaistPosition
							   )

{

  // Body attitude
  MAL_S3x3_MATRIX(,double) Body_R;
  // Body position
  MAL_S3_VECTOR(Body_P,double);
  // To the hip
  MAL_S3_VECTOR(ToTheHip,double);

  // Angles for the COM
  double COMtheta, COMomega;

  // Extract the angles from the input.
  COMtheta = aCoMPosition(3);
  COMomega = aCoMPosition(4);

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
  Body_P(2) = 0.0; // aCOMPosition.z[0] + ToTheHip(2,0);


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
    }


  // Kinematics for the left leg.
  MAL_S3_VECTOR(,double) lDt;
  lDt = m_Dt;
  KinematicsForOneLeg(Body_R,Body_P,aLeftFoot,lDt,aCoMPosition,ToTheHip,ql);

  
  // Kinematics for the right leg.
  MAL_S3x3_C_eq_A_by_B(ToTheHip, Body_R, m_TranslationToTheRightHip);
  Body_P(0) = aCoMPosition(0) + ToTheHip(0) ;
  Body_P(1) = aCoMPosition(1) + ToTheHip(1);
  Body_P(2) = 0.0;// aCOMPosition.z[0] + ToTheHip(2,0);
  
  lDt = m_Dt;
  lDt(1) = -lDt(1);
  KinematicsForOneLeg(Body_R,Body_P,aRightFoot,lDt,aCoMPosition,ToTheHip,qr);

  /* Should compute now the Waist Position */
  MAL_S3x3_C_eq_A_by_B(AbsoluteWaistPosition, Body_R, m_DiffBetweenComAndWaist);
  AbsoluteWaistPosition[0] += aCoMPosition(0);
  AbsoluteWaistPosition[1] += aCoMPosition(1);
  AbsoluteWaistPosition[2] = aCoMPosition(2) + ToTheHip(2);// - 0.705; 

  return true;
}

bool ComAndFootRealizationByGeometry::ComputePostureForGivenCoMAndFeetPosture(MAL_VECTOR(,double) & aCoMPosition,
									      MAL_VECTOR(,double) & aCoMSpeed,
									      MAL_VECTOR(,double) & aLeftFoot,
									      MAL_VECTOR(,double) & aRightFoot,
									      MAL_VECTOR(,double) & CurrentConfiguration,
									      MAL_VECTOR(,double) & CurrentVelocity,					      						      int Stage)
{

  ODEBUG("CPFGCAFP: " << aLeftFoot << " " << aRightFoot );
  if (m_InverseKinematics==0)
    {
      cerr << "ComAndFootRealizationByGeometry::InitializationUpperBody "  << endl
	   << "No Inverse Kinematics class given " << endl;
      exit(0);
      return false;
    }


  MAL_VECTOR_DIM(lqr,double,6);
  MAL_VECTOR_DIM(lql,double,6);

  MAL_S3_VECTOR(AbsoluteWaistPosition,double);

  // Kinematics for the legs.
  KinematicsForTheLegs(aCoMPosition,
		       aLeftFoot,
		       aRightFoot,
		       Stage,
		       lql,
		       lqr,
		       AbsoluteWaistPosition);
  

  // Debugging
  if (Stage==0)
    {
      ODEBUG4( (1.0/M_PI)*180.0*lqr[0] << " " <<
	       (1.0/M_PI)*180.0*lqr[1] << " " <<
	       (1.0/M_PI)*180.0*lqr[2] << " " <<
	       (1.0/M_PI)*180.0*lqr[3] << " " <<
	       (1.0/M_PI)*180.0*lqr[4] << " " <<
	       (1.0/M_PI)*180.0*lqr[5],"DebugDataqr_0.dat");

      ODEBUG4( (1.0/M_PI)*180.0*lql[0] << " " <<
	       (1.0/M_PI)*180.0*lql[1] << " " <<
	       (1.0/M_PI)*180.0*lql[2] << " " <<
	       (1.0/M_PI)*180.0*lql[3] << " " <<
	       (1.0/M_PI)*180.0*lql[4] << " " <<
	       (1.0/M_PI)*180.0*lql[5],"DebugDataql_0.dat");      
    }
  else
    {
      ODEBUG4( (1.0/M_PI)*180.0*lqr[0] << " " <<
	       (1.0/M_PI)*180.0*lqr[1] << " " <<
	       (1.0/M_PI)*180.0*lqr[2] << " " <<
	       (1.0/M_PI)*180.0*lqr[3] << " " <<
	       (1.0/M_PI)*180.0*lqr[4] << " " <<
	       (1.0/M_PI)*180.0*lqr[5],"DebugDataqr_1.dat");

      ODEBUG4( (1.0/M_PI)*180.0*lql[0] << " " <<
	       (1.0/M_PI)*180.0*lql[1] << " " <<
	       (1.0/M_PI)*180.0*lql[2] << " " <<
	       (1.0/M_PI)*180.0*lql[3] << " " <<
	       (1.0/M_PI)*180.0*lql[4] << " " <<
	       (1.0/M_PI)*180.0*lql[5],"DebugDataql_1.dat");      
    }

  MAL_VECTOR_DIM(qArmr,double,7);
  MAL_VECTOR_DIM(qArml,double,7);

  for(int i=0;i<MAL_VECTOR_SIZE(qArmr);i++)
    {
      qArmr[i] = 0.0;
      qArml[i] = 0.0;
    }
  if (GetStepStackHandler()->GetWalkMode()==0)
    {
      ComputeUpperBodyHeuristicForNormalWalking(qArmr,
						qArml,
						aCoMPosition,
						aRightFoot,
						aLeftFoot);
    }

  
  
  ODEBUG( "AbsoluteWaistPosition: " << AbsoluteWaistPosition << endl
	   << "CoMPosition: " << aCoMPosition<< endl
	   << "ToTheHip(2) : " << ToTheHip(2) );

  

  /* Update of the configuration and velocity vector */
  for(int i=0;i<3;i++)
    CurrentConfiguration[i] = AbsoluteWaistPosition(i);

  for(int i=3;i<6;i++)
    CurrentConfiguration[i] = aCoMPosition(i);

  for(int i=0;i<MAL_VECTOR_SIZE(lql);i++)
    CurrentConfiguration[m_LeftLegIndexinConfiguration[i]] = lql[i];
  
  for(int i=0;i<MAL_VECTOR_SIZE(lqr);i++)
    CurrentConfiguration[m_RightLegIndexinConfiguration[i]] = lqr[i];

  for(int i=0;i<MAL_VECTOR_SIZE(qArml);i++)
    CurrentConfiguration[m_LeftArmIndexinConfiguration[i]] = qArml[i];
  
  for(int i=0;i<MAL_VECTOR_SIZE(qArmr);i++)
    CurrentConfiguration[m_RightArmIndexinConfiguration[i]] = qArmr[i];

  // Update the speed values.
  /* If this is the first call ( stage = 0) 
     we should update the current stored values.  */
  if (Stage==0)
    {
      /* Compute the speed */
      for(int i=6;i<MAL_VECTOR_SIZE(m_prev_Configuration);i++)
	{
	  CurrentVelocity[i] = (CurrentConfiguration[i] - m_prev_Configuration[i])/ getSamplingPeriod();
	  /* Keep the new value for the legs. */
	}

      m_prev_Configuration = CurrentConfiguration;
    }
  else if (Stage==1)
    {

      /* Compute the speed */
      for(int i=6;i<MAL_VECTOR_SIZE(m_prev_Configuration1);i++)
	{
	  CurrentVelocity[i] = (CurrentConfiguration[i] - m_prev_Configuration1[i])/ getSamplingPeriod();
	  /* Keep the new value for the legs. */
	}
      m_prev_Configuration1 = CurrentConfiguration;

    }
  
  for(int i=0;i<6;i++)
    CurrentVelocity[i] = aCoMSpeed(i);


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

  ODEBUG4( aCoMPosition[0] << " " <<
	   aCoMPosition[1],"COMPC1.dat");
  return true;
}

int ComAndFootRealizationByGeometry::EvaluateStartingCoM(MAL_VECTOR(&BodyAngles,double),
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

int ComAndFootRealizationByGeometry::EvaluateStartingCoM(MAL_VECTOR(&BodyAngles,double),
							 MAL_S3_VECTOR(&aStartingCOMPosition,double),
							 MAL_S3_VECTOR(&aWaistPosition,double),
							 FootAbsolutePosition & InitLeftFootPosition,
							 FootAbsolutePosition & InitRightFootPosition)
{
  MAL_S3_VECTOR(WaistPosition,double);
  InitializationCoM(BodyAngles,
		    m_StartingCOMPosition,
		    InitLeftFootPosition,
		    InitRightFootPosition);
  aWaistPosition[0] = WaistPosition[0];
  aWaistPosition[1] = WaistPosition[1];
  aWaistPosition[2] = WaistPosition[2];
  aStartingCOMPosition[0] = m_StartingCOMPosition[0];
  aStartingCOMPosition[1] = m_StartingCOMPosition[1];
  aStartingCOMPosition[2] = m_StartingCOMPosition[2];
  
  return 0;
}

int ComAndFootRealizationByGeometry::EvaluateCOMForStartingPosition( MAL_VECTOR( &BodyAngles,double),
								     double omega, double theta,
								     MAL_S3_VECTOR( &lCOMPosition,double),
								     FootAbsolutePosition & InitLeftFootPosition,
								     FootAbsolutePosition & InitRightFootPosition)
{
  MAL_S3_VECTOR(lWaistPosition,double);
  return InitializationCoM(BodyAngles,
			   lCOMPosition,
			   InitLeftFootPosition, InitRightFootPosition);
  
}
 
 
void ComAndFootRealizationByGeometry::GetCurrentPositionofWaistInCOMFrame(MAL_VECTOR(,double) &CurPosWICF_homogeneous)
{
  for(int i=0;i<3;i++)
    CurPosWICF_homogeneous[i] = m_DiffBetweenComAndWaist[i];
  CurPosWICF_homogeneous[3] = 1.0;
  
}
 
void ComAndFootRealizationByGeometry::ComputeUpperBodyHeuristicForNormalWalking(    MAL_VECTOR(,double) & qArmr, 
										    MAL_VECTOR(,double) & qArml,
										    MAL_VECTOR(,double) & aCOMPosition,
										    MAL_VECTOR(,double) & RFP,
										    MAL_VECTOR(,double) & LFP)
/*
  COMPosition aCOMPosition,
  FootAbsolutePosition RFP,
  FootAbsolutePosition LFP )*/
{
  if (m_InverseKinematics==0)
    {
      cerr << "ComAndFootRealizationByGeometry::InitializationUpperBody "  << endl
	   << "No Inverse Kinematics class given " << endl;
      return ;
    }

  m_Xmax = m_InverseKinematics->ComputeXmax(m_ZARM);
  
  double GainX = m_GainFactor * m_Xmax/0.2;
  
  // Arms Motion : Heuristic based.
  double Alpha,Beta;
  //Temporary variables
  double TempXL,TempXR,TempCos,TempSin,
    TempARight,TempALeft;
  
  // Compute the position of the hand according to the 
  // leg.
  TempCos = cos(aCOMPosition(3)*M_PI/180.0);
  TempSin = sin(aCOMPosition(3)*M_PI/180.0);
  
  TempXR = TempCos * (RFP(0)  - aCOMPosition(0)) + 
    TempSin * (RFP(1)  - aCOMPosition(1));
  TempXL = TempCos * (LFP(0)  - aCOMPosition(0)) + 
    TempSin * (LFP(1)  - aCOMPosition(1));
  
  ODEBUG4(aCOMPosition(0) << " " << aCOMPosition(1),"DebugDataCOMForHeuristic.txt");
  
  TempARight = TempXR*-1.0;
  TempALeft = TempXL*-1.0;
  
  ODEBUG4("Values:" << TempALeft * GainX << " " << m_ZARM << " " << m_Xmax,"DebugDataIKArms.txt");
  // Compute angles using inverse kinematics and the computed hand position.
  m_InverseKinematics->ComputeInverseKinematicsForArms(TempALeft * GainX,
						       m_ZARM,
						       Alpha,
						       Beta);
  
  ODEBUG6("ComputeHeuristicArm: Step 2 ","DebugDataIKArms.txt");
  qArml(0)= Alpha;
  qArml(1)= 10.0*M_PI/180.0;
  qArml(2)= 0.0;
  qArml(3)= Beta;
  qArml(4)= 0.0;
  qArml(5)= 0.0;
  qArml(6)= 10.0*M_PI/180.0;
  
  ODEBUG4( "IK Left arm p:" << qArml(0)<< " " <<  qArml(1)  << " " << qArml(2) 
	      << " " << qArml(3) << "  " << qArml(4) << " " << qArml(5), "DebugDataIKArms.txt" );
  
  
  m_InverseKinematics->ComputeInverseKinematicsForArms(TempARight * GainX,
						       m_ZARM,
						       Alpha,
						       Beta);
  qArmr(0)= Alpha;
  qArmr(1)= -10.0*M_PI/180.0;
  qArmr(2)= 0.0;
  qArmr(3)= Beta;
  qArmr(4)= 0.0;
  qArmr(5)= 0.0;
  qArmr(6)= 10.0*M_PI/180.0;;
  
  ODEBUG4( "IK Right arm p:" << qArmr(0)<< " " <<  qArmr(1)  << " " << qArmr(2) 
	   << " " << qArmr(3) << "  " << qArmr(4) << " " << qArmr(5), "DebugDataIKArms.txt" );
  
  
  ODEBUG4( qArml(0)<< " " <<  qArml(1)  << " " << qArml(2) << " " 
	   << qArml(3) << "  " << qArml(4) << " " << qArml(5) << " " 
	   << qArmr(0)<< " " <<  qArmr(1)  << " " << qArmr(2) << " "
	   << qArmr(3) << "  " << qArmr(4) << " " << qArmr(5), "DebugDataqArmsHeuristic.txt");
  
}  

bool ComAndFootRealizationByGeometry::setHumanoidDynamicRobot(const CjrlHumanoidDynamicRobot<MAL_MATRIX(,double), 
							      MAL_S4x4_MATRIX(,double),MAL_S3x3_MATRIX(,double),
							      MAL_VECTOR(,double),MAL_S3_VECTOR(,double)> * aHumanoidDynamicRobot)
{
  ComAndFootRealization::setHumanoidDynamicRobot(aHumanoidDynamicRobot);
  HumanoidDynamicMultiBody *aHDMB = (HumanoidDynamicMultiBody *)aHumanoidDynamicRobot;
  
  m_HS = aHDMB->getHumanoidSpecificities();

  if (m_InverseKinematics!=0)
    delete m_InverseKinematics;

  m_InverseKinematics = new InverseKinematics(m_HS);

  double WaistToHip[3];
  // Left hip;
  m_HS->GetWaistToHip(1,WaistToHip);
  m_StaticToTheLeftHip[0] = WaistToHip[0];
  m_StaticToTheLeftHip[1] = WaistToHip[1];
  m_StaticToTheLeftHip[2] = WaistToHip[2];

  m_HS->GetWaistToHip(-1,WaistToHip);
  m_StaticToTheRightHip[0] = WaistToHip[0];
  m_StaticToTheRightHip[1] = WaistToHip[1];
  m_StaticToTheRightHip[2] = WaistToHip[2];
  
  double HipLength[3];
  m_HS->GetHipLength(1,HipLength);
  m_Dt(0) = HipLength[0];
  m_Dt(1) = HipLength[1];
  m_Dt(2) = HipLength[2];

  MAL_VECTOR_RESIZE(m_prev_Configuration,aHDMB->numberDof());
  MAL_VECTOR_RESIZE(m_prev_Configuration1,aHDMB->numberDof());

  for(int i=0;i<MAL_VECTOR_SIZE(m_prev_Configuration);i++)
    {
      m_prev_Configuration[i] = 0.0;
      m_prev_Configuration1[i] = 0.0;
    }
  return true;
}

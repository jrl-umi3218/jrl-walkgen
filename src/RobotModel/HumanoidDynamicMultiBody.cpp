#include <DynamicMultiBody.h>
#include <HumanoidDynamicMultiBody.h>

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "HumanoidDynamicMultiBody: " << x << endl; DebugFile.close();}
#else
#define RESETDEBUG4(y) 
#define ODEBUG4(x,y) 
#endif

#define RESETDEBUG6(y) 
#define ODEBUG6(x,y) 

#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "HumanoidDynamicMultiBody: " << x << endl; DebugFile.close();}
#if 1
#define ODEBUG(x)
#else
#define ODEBUG(x)  std::cout << x << endl;
#endif

#define ODEBUG3(x)  std::cout << x << endl;

using namespace PatternGeneratorJRL;

HumanoidDynamicMultiBody::HumanoidDynamicMultiBody(CjrlDynamicRobot *aDMB,
						   string aFileNameForHumanoidSpecificities)
{
  m_DMB = aDMB;

  string aHumanoidName="HRP2JRL";
  m_HS = new HumanoidSpecificities();

  if (m_HS!=0)
    {
      m_HS->ReadXML(aFileNameForHumanoidSpecificities,aHumanoidName);
	
      double AnklePosition[3];
      // Take the right ankle position (should be equivalent)
      m_HS->GetAnklePosition(-1,AnklePosition);
      m_AnkleSoilDistance = AnklePosition[2];
      ODEBUG("AnkleSoilDistance =" << m_AnkleSoilDistance);

      // Lenght of the hip (necessary for 
      double HipLength[3];
      // Takes the left one.
      m_HS->GetHipLength(1,HipLength);

      ODEBUG(WaistToHip[0] << " "
	      << WaistToHip[1] << " "
	      << WaistToHip[2] << " ");
      m_Dt(0) = HipLength[0];
      m_Dt(1) = HipLength[1];
      m_Dt(2) = HipLength[2];

      MAL_S3_VECTOR(StaticToTheLeftHip,double);
      MAL_S3_VECTOR(StaticToTheRightHip,double);

      
      // Displacement between the hip and the waist.
      double WaistToHip[3];
      m_HS->GetWaistToHip(1,WaistToHip);
      m_StaticToTheLeftHip(0) = WaistToHip[0];
      m_StaticToTheLeftHip(1) = WaistToHip[1];
      m_StaticToTheLeftHip(2) = WaistToHip[2]; 

      m_TranslationToTheLeftHip = m_StaticToTheLeftHip;
      
      m_HS->GetWaistToHip(-1,WaistToHip);
      m_StaticToTheRightHip(0) = WaistToHip[0];
      m_StaticToTheRightHip(1) = WaistToHip[1];
      m_StaticToTheRightHip(2) = WaistToHip[2];
      m_TranslationToTheRightHip = m_StaticToTheRightHip;      
      
    }
  else
    {
      cerr << "Warning: No appropriate definition of Humanoid Specifities" << endl;
      cerr << "Use default value: " << 0.1 << endl;
      m_AnkleSoilDistance = 0.1;

      // Displacement between the hip and the waist.
      m_Dt(0) = 0.0;
      m_Dt(1) = 0.04;
      m_Dt(2) = 0.0;

    }
  
}

HumanoidDynamicMultiBody::~HumanoidDynamicMultiBody()
{
  if (m_HS!=0)
    delete m_HS;
}

void HumanoidDynamicMultiBody::LinkBetweenJointsAndEndEffectorSemantic()
{
  if (m_HS==0)
    return;

  // Link the correct joints.
  
  // Get the left hand.
  std::vector<int> JointForOneLimb = m_HS->GetArmJoints(1);
  int ListeJointsSize = JointForOneLimb.size();
  int EndIndex = JointForOneLimb[ListeJointsSize-1];
  DynamicMultiBody *m_SDMB = dynamic_cast<DynamicMultiBody *>(m_DMB);
  if (m_DMB!=0)
    m_LeftHandJoint = m_SDMB->GetJointFromVRMLID(EndIndex);
  
  // Get the right hand.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetArmJoints(-1);
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];
  if (m_DMB!=0)
    m_RightHandJoint = m_SDMB->GetJointFromVRMLID(EndIndex);
  
  
  // Get the left foot.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetFootJoints(1);
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];
  ODEBUG("Joints for the left foot:" << EndIndex);
  if (m_DMB!=0)
    m_LeftFootJoint = m_SDMB->GetJointFromVRMLID(EndIndex);
  
  // Get the right foot.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetFootJoints(-1);
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];
  ODEBUG("Joints for the right foot:" << EndIndex);
  if (m_DMB!=0)
    m_RightFootJoint = m_SDMB->GetJointFromVRMLID(EndIndex);

  
  // Get the gaze joint (head) of the humanoid.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetHeadJoints();
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];
  if (m_DMB!=0)
    m_GazeJoint = m_SDMB->GetJointFromVRMLID(EndIndex);

  
}

void HumanoidDynamicMultiBody::GetJointIDInConfigurationFromVRMLID(std::vector<int> &aVector)
{
  DynamicMultiBody *a_SDMB = dynamic_cast<DynamicMultiBody *>(m_DMB);
  if (a_SDMB!=0)
    a_SDMB->GetJointIDInConfigurationFromVRMLID(aVector);
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::zeroMomentumPoint() const
{

  return m_ZeroMomentumPoint;
}

void HumanoidDynamicMultiBody::ComputingZeroMomentumPoint()
{
  DynamicMultiBody * aDMB = (DynamicMultiBody *)m_DMB;
  m_ZeroMomentumPoint = aDMB->getZMP();
}

/* Methods related to the fixed joints */

bool HumanoidDynamicMultiBody::addFixedJoint(CjrlJoint *inFixedJoint)
{
  m_VectorOfFixedJoints.insert(m_VectorOfFixedJoints.end(),inFixedJoint);
  return true;
}

unsigned int HumanoidDynamicMultiBody::countFixedJoints() const
{
  return m_VectorOfFixedJoints.size();
}

bool HumanoidDynamicMultiBody::removeFixedJoint(CjrlJoint * inFixedJoint)
{
  std::vector<CjrlJoint *>::iterator it_Joint = m_VectorOfFixedJoints.begin();
  while((*it_Joint!= inFixedJoint) &&
	(it_Joint!=m_VectorOfFixedJoints.end()))
    it_Joint++;

  if (it_Joint!=m_VectorOfFixedJoints.end())
    m_VectorOfFixedJoints.erase(it_Joint);
  return true;
}

const CjrlJoint & HumanoidDynamicMultiBody::fixedJoint(unsigned int inJointRank) const
{
  if ((inJointRank>0) & (inJointRank<=m_VectorOfFixedJoints.size()))
      return *m_VectorOfFixedJoints[inJointRank];
}
/* End of Methods related to the fixed joints */


/***************************************************/
/* Implementation of the proxy design pattern for  */
/* the part inherited from jrlDynamicRobot.        */
/***************************************************/

void HumanoidDynamicMultiBody::rootJoint(CjrlJoint & inJoint)
{
  if (m_DMB!=0)
    m_DMB->rootJoint(inJoint);
}

CjrlJoint * HumanoidDynamicMultiBody::rootJoint() const
{
  if (m_DMB==0)
    return 0;
  return m_DMB->rootJoint();
}

std::vector< CjrlJoint *> HumanoidDynamicMultiBody::jointVector()
{  
  return  m_DMB->jointVector();
}

unsigned int HumanoidDynamicMultiBody::numberDof() const
{
  return m_DMB->numberDof();
}

bool HumanoidDynamicMultiBody::currentConfiguration(const MAL_VECTOR(,double) & inConfig)
{
  return m_DMB->currentConfiguration(inConfig);
}

const MAL_VECTOR(,double) & HumanoidDynamicMultiBody::currentConfiguration() const
{
  return m_DMB->currentConfiguration();
}

bool HumanoidDynamicMultiBody::currentVelocity(const MAL_VECTOR(,double) & inVelocity) 
{
  return m_DMB->currentVelocity(inVelocity);
}

const MAL_VECTOR(,double) & HumanoidDynamicMultiBody::currentVelocity()  const
{
  return m_DMB->currentVelocity();
}

bool HumanoidDynamicMultiBody::currentAcceleration(const MAL_VECTOR(,double) & inAcceleration)
{
  return m_DMB->currentAcceleration(inAcceleration);
}

const MAL_VECTOR(,double) & HumanoidDynamicMultiBody::currentAcceleration() const
{
  return m_DMB->currentAcceleration();
}

bool HumanoidDynamicMultiBody::computeForwardKinematics()
{
  bool r;
  r= m_DMB->computeForwardKinematics();
  ComputingZeroMomentumPoint();
  return r;

}

bool HumanoidDynamicMultiBody::computeCenterOfMassDynamics()
{
  return m_DMB->computeCenterOfMassDynamics();
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::positionCenterOfMass()
{
  return m_DMB->positionCenterOfMass();
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::velocityCenterOfMass()
{
  return m_DMB->velocityCenterOfMass();
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::accelerationCenterOfMass()
{
  return m_DMB->accelerationCenterOfMass();
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::linearMomentumRobot()
{
  return m_DMB->linearMomentumRobot();
  
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::derivativeLinearMomentum()
{
  return m_DMB->derivativeLinearMomentum();
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::angularMomentumRobot()
{
  return m_DMB->angularMomentumRobot();
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::derivativeAngularMomentum()
{
  return m_DMB->derivativeAngularMomentum();
}

void HumanoidDynamicMultiBody::computeJacobianCenterOfMass()
{
  return m_DMB->computeJacobianCenterOfMass();
}

const MAL_MATRIX(,double) & HumanoidDynamicMultiBody::jacobianCenterOfMass() const
{
  return m_DMB->jacobianCenterOfMass();
}

/***************************************************/
/* End of the implementation                       */
/***************************************************/

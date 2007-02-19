#include <string>

#include <Joint.h>
#include <HumanoidDynamicMultiBody.h>

using namespace std;
using namespace PatternGeneratorJRL;

void RecursiveDisplayOfJoints(CjrlJoint<MAL_MATRIX(,double), MAL_S4x4_MATRIX(,double),MAL_S3x3_MATRIX(,double),
			      MAL_VECTOR(,double),MAL_S3_VECTOR(,double)> *aJoint)
{
  if (aJoint==0)
    return;

  Joint *a2Joint=0;

  int NbChildren = aJoint->countChildJoints();

  a2Joint = dynamic_cast<Joint *>( aJoint);
  if (a2Joint==0)
    return;

  cout << "Name : " << a2Joint->getName() << endl;
  cout << "Number of child  :" << NbChildren << endl;
  for(int i=0;i<NbChildren;i++)
    {
      a2Joint = (Joint *)&aJoint->childJoint(i);

      cout << " Child " << i << " " <<a2Joint->getName() << endl;
    }


  cout << "Nb of degree of freedom " << 
    aJoint->numberDof() << endl;

  cout << "Initial Position " <<
    aJoint->initialPosition();

  cout << "CurrentTransformation " <<
    aJoint->currentTransformation() << endl;

  cout << " Joint from root to here:" << endl;
  std::vector<CjrlJoint<MAL_MATRIX(,double), MAL_S4x4_MATRIX(,double),MAL_S3x3_MATRIX(,double),
    MAL_VECTOR(,double),MAL_S3_VECTOR(,double)> *> JointsFromRootToHere = aJoint->jointsFromRootToThis();
  cout << " Nb of nodes: " << JointsFromRootToHere.size() << endl;
  for(int i=0;i<JointsFromRootToHere.size();i++)
    {
      Joint * a3Joint = dynamic_cast<Joint *>(JointsFromRootToHere[i]);
      if (a3Joint==0)
	continue;

      cout << a3Joint->getName() << endl;

    }
  CjrlRigidVelocity<MAL_S3_VECTOR(,double)> aRV = aJoint->jointVelocity();
  cout << " Linear Velocity " << aRV.linearVelocity() << endl;
  cout << " Angular Velocity " << aRV.rotationVelocity() << endl;
  CjrlRigidAcceleration<MAL_S3_VECTOR(,double)> aRA = aJoint->jointAcceleration();
  cout << " Linear Acceleration " << aRA.linearAcceleration() << endl;
  cout << " Angular Acceleration " << aRA.rotationAcceleration() << endl;

  cout << "***********************************************" << endl;
  cout << " Display Now information related to children :" << endl;
  for(int i=0;i<NbChildren;i++)
    {
      // Returns a const so we have to force the casting/
      RecursiveDisplayOfJoints((CjrlJoint<MAL_MATRIX(,double), MAL_S4x4_MATRIX(,double),MAL_S3x3_MATRIX(,double),
				MAL_VECTOR(,double),MAL_S3_VECTOR(,double)> *)&aJoint->childJoint(i)); 
    }
  cout << " End for Joint: " << a2Joint->getName() << endl;
}


void DisplayDynamicRobotInformation(CjrlDynamicRobot<MAL_MATRIX(,double), MAL_S4x4_MATRIX(,double),MAL_S3x3_MATRIX(,double),
				    MAL_VECTOR(,double),MAL_S3_VECTOR(,double)> *aDynamicRobot)
{
  std::vector<CjrlJoint<MAL_MATRIX(,double), MAL_S4x4_MATRIX(,double),MAL_S3x3_MATRIX(,double),
    MAL_VECTOR(,double),MAL_S3_VECTOR(,double)> *> aVec = aDynamicRobot->jointVector();
  int r = aVec.size();
  cout << "Number of joints :" << r << endl;
  for(int i=0;i<r;i++)
    {
      Joint * aJoint = dynamic_cast<Joint *>(aVec[i]);
      cout << aJoint->getName();
    }	

  
}

int main()
{
  string aFileName = "./data/HRP2Specificities.xml";
  DynamicMultiBody * aDMB = new DynamicMultiBody();
  string aPath="../../etc/HRP2JRL/";
  string aName="HRP2JRLmain.wrl";
  aDMB->parserVRML(aPath,aName,"");
  HumanoidDynamicMultiBody *aHDMB = new HumanoidDynamicMultiBody(aDMB,aFileName);

  // Display tree of the joints.
  CjrlJoint<MAL_MATRIX(,double), MAL_S4x4_MATRIX(,double),MAL_S3x3_MATRIX(,double),
    MAL_VECTOR(,double),MAL_S3_VECTOR(,double)> * rootJoint = aHDMB->rootJoint();  
  bool ok=true;

  // Test the tree.
  RecursiveDisplayOfJoints(rootJoint);


  // Tes the computation of the jacobian.
  double dInitPos[40] = { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  };

  int NbOfDofs = aDMB->numberDof();
  
  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  int lindex=0;
  for(int i=0;i<6;i++)
    aCurrentConf[lindex++] = 0.0;

  for(int i=0;i<40;i++)
    aCurrentConf[lindex++] = dInitPos[i];
  
  aDMB->currentConfiguration(aCurrentConf);

  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs); 
  lindex=0;
  for(int i=0;i<NbOfDofs;i++)
    aCurrentVel[lindex++] = 0.0;
  
  aDMB->currentVelocity(aCurrentVel);

  aDMB->computeForwardKinematics();

  std::vector<CjrlJoint<MAL_MATRIX(,double), MAL_S4x4_MATRIX(,double),MAL_S3x3_MATRIX(,double),
    MAL_VECTOR(,double),MAL_S3_VECTOR(,double)> *> aVec = aDMB->jointVector();
  
  Joint  * aJoint = (Joint *)aVec[22]; // Try to get the hand.

  cout << aJoint->getName() << endl;
  
  aJoint->computeJacobianJointWrtConfig();

  MAL_MATRIX(,double) aJ = aJoint->jacobianJointWrtConfig();
  
  for(int i=0;i<6;i++)
    {
      for(int j=0;j<MAL_MATRIX_NB_COLS(aJ);j++)
	{
	  if (aJ(i,j)==0.0)
	    printf("0 ");
	  else
	    printf("%10.5f ",aJ(i,j));
	}
      printf("\n");
    }
  
  delete aDMB;
  delete aHDMB;
  
}

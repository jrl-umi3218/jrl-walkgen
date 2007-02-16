#include "Joint.h"
#include "DynamicBody.h"

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "Joint :" << x << endl

#if 0
#define ODEBUG(x) cerr << "Joint :" <<  x << endl
#else
#define ODEBUG(x) 
#endif

#if 0

#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "WalkGenJRLIntegrate: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1 
#else
#define ODEBUG4(x,y)
#endif

using namespace PatternGeneratorJRL;

Joint::Joint(int ltype, MAL_S3_VECTOR(,double) & laxe, 
	     float lquantite, MAL_S4x4_MATRIX(,double) & lpose):
  m_type(ltype),
  m_axe(laxe),
  m_quantity(lquantite),
  m_pose(lpose),
  m_FatherJoint(0),
  m_IDinVRML(-1)
{

}

Joint::Joint(int ltype, MAL_S3_VECTOR(,double) & laxe, 
	     float lquantite, MAL_S3_VECTOR(,double) & translationStatic):
  m_type(ltype),
  m_axe(laxe),
  m_quantity(lquantite),
  m_FatherJoint(0),
  m_IDinVRML(-1)
{
  MAL_S4x4_MATRIX_SET_IDENTITY(m_pose);
  m_pose(0,3) = translationStatic[0];
  m_pose(1,3) = translationStatic[1];
  m_pose(2,3) = translationStatic[2];

  
}

Joint::Joint(int ltype, MAL_S3_VECTOR(,double) & laxe, 
	     float lquantite):
  m_type(ltype),
  m_axe(laxe),
  m_quantity(lquantite),
  m_FatherJoint(0),
  m_IDinVRML(-1)
{
  MAL_S4x4_MATRIX_SET_IDENTITY(m_pose);
}

Joint::Joint(const Joint &r)
{
  m_type = r.type();
  m_axe = r.axe();
  m_quantity=r.quantity();
  m_pose=r.pose();
  m_FatherJoint = 0;
  m_Name=r.getName();
  m_IDinVRML=r.getIDinVRML();

}

Joint::~Joint() 
{
}


Joint & Joint::operator=(const Joint & r) 
{
  m_type = r.type();
  m_axe = r.axe();
  m_quantity=r.quantity();
  m_pose=r.pose();
  m_Name = r.getName();
  m_IDinVRML = r.getIDinVRML();
  return *this;
};


/***********************************************/
/* Implementation of the generic JRL interface */
/***********************************************/

CjrlJoint & Joint::parentJoint() const
{
  return *m_FatherJoint;
}

bool Joint::addChildJoint(const CjrlJoint & aJoint)
{
  Joint * pjoint = (Joint *)&aJoint;
  m_Children.push_back(pjoint);
  return true;
}

unsigned int Joint::countChildJoints() const
{
  return m_Children.size();
}

const CjrlJoint & Joint::childJoint(unsigned int givenRank) const
{
  if ((givenRank>=0) && (givenRank<m_Children.size()))
    return *m_Children[givenRank];
  
  return *m_Children[0];
}

std::vector<CjrlJoint *> Joint::jointsFromRootToThis() const 
{
  return m_FromRootToThis;
}


const MAL_S4x4_MATRIX(,double) & Joint::currentTransformation() const
{
  MAL_S4x4_MATRIX(,double) * A = new MAL_S4x4_MATRIX(,double);

  MAL_S4x4_MATRIX_SET_IDENTITY((*A));

  if (m_Body!=0)
    {
      DynamicBody *m_DBody = (DynamicBody *) m_Body;

      for( unsigned int i=0;i<3;i++)
	for(unsigned int j=0;j<3;j++)
	  (*A)(i,j) = m_DBody->R(i,j);
      
      for( unsigned int i=0;i<3;i++)
	(*A)(i,3) = m_DBody->p(i);
    }
  return *A;
}

CjrlRigidVelocity Joint::jointVelocity()
{
  return m_RigidVelocity;
}

CjrlRigidAcceleration Joint::jointAcceleration()
{
  // TODO : Update the member of this object
  // TODO : when calling ForwardDynamics.
  // TODO : This will avoid the dynamic cast.
  MAL_S3_VECTOR(,double) a,b;

  if (m_Body!=0)
    {
      DynamicBody *m_DBody = dynamic_cast<DynamicBody *>(m_Body);

      a = m_DBody->dv;
      b = m_DBody->dw;
    }
  CjrlRigidAcceleration ajrlRA(a,b);
  
  return ajrlRA;

}

unsigned int Joint::numberDof() const
{
  unsigned int r=0;

  switch(m_type)
    {
    case (FREE_JOINT):
      r=6;
      break;
    case (FIX_JOINT):
      r=0;
      break;
    case (REVOLUTE_JOINT):
      r=1;
      break;
    case (PRISMATIC_JOINT):
      r=1;
      break;
    }
  return r;
}

MAL_MATRIX(,double) Joint::jacobianPositionJointWrtConfig() const
{
  cerr << "MAL_MATRIX(,double) Joint::jacobianPositionJointWrtConfig() " << endl;
  cerr << "This function should be implemented" << endl;
  MAL_MATRIX(,double) J;
  
  return J;
}

MAL_MATRIX(,double) Joint::computeJacobianJointWrtConfig()
{
  cerr << "MAL_MATRIX(,double) Joint::computeJacobianJointWrtConfig() " << endl;
  cerr << "This function should be implemented" << endl;
  MAL_MATRIX(,double) A;
  return A;

}

MAL_MATRIX(,double) Joint::jacobianPointWrtConfig(MAL_S3_VECTOR(,double) inPointJointFrame) const
{
  cerr << "MAL_MATRIX(,double) Joint::jacobianPointWrtConfig() " << endl;
  cerr << "This function should be implemented" << endl;
  MAL_MATRIX(,double) A;
  return A;

}

CjrlBody * Joint::linkedBody() const
{
  return m_Body;
}

int Joint::setLinkedBody(CjrlBody & inBody)
{
  m_Body = &inBody;
  return 1;
}

void Joint::SetFatherJoint(Joint *aFather)
{
  m_FatherJoint = aFather;

  if ((m_FromRootToThis.size()==0) &&
      (m_FatherJoint!=0))
    {
      CjrlJoint * aJoint;
      aJoint=m_FatherJoint;
      while(aJoint!=0)
	{
	  m_FromRootToThis.insert(m_FromRootToThis.begin(),aJoint);
	  aJoint = &(aJoint->parentJoint());
	}
    }

}

const MAL_S4x4_MATRIX(,double) & Joint::initialPosition()
{
  if (m_Body!=0)
    {
      DynamicBody *aDB = (DynamicBody *) m_Body;
      ODEBUG("Joint Name " << m_Name << " " << m_Body);
      for(int i=0;i<3;i++)
	for(int j=0;j<3;j++)
	  m_pose(i,j) = aDB->R(i,j);
      for(int i=0;i<3;i++)
	m_pose(i,3) = aDB->p(i);
      ODEBUG( m_pose);

    }
  return m_pose;
}

void Joint::UpdatePoseFrom6DOFsVector(MAL_VECTOR(,double) a6DVector)
{
  // Update the orientation of the joint.
  // Takes the three euler joints 

  m_pose(0,3) = a6DVector(0);
  m_pose(1,3) = a6DVector(1);
  m_pose(2,3) = a6DVector(2);  
  
  MAL_S3x3_MATRIX(,double) D,B,C,A;
  double CosTheta, SinTheta, 
    CosPhi, SinPhi,
    CosPsi, SinPsi;
  
  CosTheta = cos(a6DVector(3));
  SinTheta = sin(a6DVector(3));
  CosPsi = cos(a6DVector(4));
  SinPsi = sin(a6DVector(4));
  CosPhi = cos(a6DVector(5));
  SinPhi = sin(a6DVector(5));
  
  D(0,0) =  CosPhi; D(0,1) = SinPhi; D(0,2) = 0;
  D(1,0) = -SinPhi; D(1,1) = CosPhi; D(1,2) = 0;
  D(2,0) =       0; D(2,1) =      0; D(2,2) = 1;
  
  C(0,0) =  CosTheta; C(0,1) =        0; C(0,2) = -SinTheta;
  C(1,0) =         0; C(1,1) =        1; C(1,2) = 0;
  C(2,0) = -SinTheta; C(2,1) =        0; C(2,2) = CosTheta;
  
  B(0,0) =       1; B(0,1) =        0; B(0,2) = 0;
  B(1,0) =       0; B(1,1) =   CosPsi; B(1,2) = SinPsi;
  B(2,0) =       0; B(2,1) =  -SinPsi; B(2,2) = CosPsi;
  
  MAL_S3x3_MATRIX(,double) tmp;
  MAL_S3x3_C_eq_A_by_B(tmp,C,D);
  MAL_S3x3_C_eq_A_by_B(A,B,tmp);
  
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      m_pose(i,j) = A(i,j);
 
  ODEBUG("m_pose : " << m_pose << 
	  " A: "<<endl << A << 
	  " tmp " << endl << tmp <<
	  "C " << endl << C <<
	  "D " << endl << D << 
	  "B " << endl << B );
}

void Joint::UpdateVelocityFrom2x3DOFsVector(MAL_S3_VECTOR(,double) & aLinearVelocity,
					  MAL_S3_VECTOR(,double) & anAngularVelocity)
{
  m_RigidVelocity.linearVelocity(aLinearVelocity);
  m_RigidVelocity.rotationVelocity(anAngularVelocity);
}

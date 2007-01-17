#include "Joint.h"

using namespace PatternGeneratorJRL;

Joint::Joint(int ltype, MAL_S3_VECTOR(,double) laxe, 
	     float lquantite, MAL_S4x4_MATRIX(,double) lrotation):
  m_type(ltype),
  m_axe(laxe),
  m_quantity(lquantite),
  m_rotation(lrotation),
  m_FatherJoint(0)
{
}

Joint::Joint(int ltype, MAL_S3_VECTOR(,double) laxe, 
	     float lquantite):
  m_type(ltype),
  m_axe(laxe),
  m_quantity(lquantite),
  m_FatherJoint(0)
{
  MAL_S4x4_MATRIX_SET_IDENTITY(m_rotation);
}

Joint::Joint(const Joint &r)
{
  m_type = r.type();
  m_axe = r.axe();
  m_quantity=r.quantity();
  m_rotation=r.rotation();
  m_FatherJoint =0;
}

Joint::~Joint() 
{
}


Joint & Joint::operator=(const Joint & r) 
{
  m_type = r.type();
  m_axe = r.axe();
  m_quantity=r.quantity();
  m_rotation=r.rotation();
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
  std::vector<CjrlJoint *> avec;
  return avec;
}


const MAL_S4x4_MATRIX(,double) & Joint::currentTransformation() const
{
  MAL_S4x4_MATRIX(,double) * A = new MAL_S4x4_MATRIX(,double);

  return *A;
}

CjrlRigidVelocity Joint::jointVelocity()
{
  MAL_S3_VECTOR(,double) v,w;
  CjrlRigidVelocity ajrlRV(v,w);
  
  return ajrlRV;
}

CjrlRigidAcceleration Joint::jointAcceleration()
{
  MAL_S3_VECTOR(,double) a,b;
  CjrlRigidAcceleration aRigidAcceleration(a,b);
  return aRigidAcceleration;
}

unsigned int Joint::numberDof() const
{
  return 1;
}

MAL_MATRIX(,double) Joint::jacobianPositionJointWrtConfig() const
{
  MAL_MATRIX(,double) A;
  return A;
}

CjrlBody * Joint::linkedBody() const
{
  return 0;
}

int Joint::setLinkedBody(CjrlBody & inBody)
{
  return 1;

}

void Joint::SetFatherJoint(Joint *aFather)
{
  m_FatherJoint = aFather;
}

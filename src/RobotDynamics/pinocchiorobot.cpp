#include <jrl/walkgen/pinocchiorobot.hh>
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
using namespace PatternGeneratorJRL;

class Joint_shortname : public boost::static_visitor<std::string>
{
public:
  template<typename D>
  std::string operator()(const se3::JointModelBase<D> & jmodel) const
  { return jmodel.shortname(); }

  static std::string run( const se3::JointModelVariant & jmodel)
  { return boost::apply_visitor( Joint_shortname(), jmodel ); }
};
inline std::string shortname(const se3::JointModelVariant & jmodel)
{ return Joint_shortname::run(jmodel); }

PinocchioRobot::PinocchioRobot()
{
  // all the pointor are set to 0
  m_robotModel = 0 ;
  m_robotData = 0 ;
  m_robotDataInInitialePose = 0 ;

  // init quaternion as unit zero rotation
  m_quat = Eigen::Quaterniond(
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) ) ;
  m_quat.normalize();
  // resize by default
  m_q.resize(50,1);
  m_q(3)=m_quat.x();
  m_q(4)=m_quat.y();
  m_q(5)=m_quat.z();
  m_q(6)=m_quat.w();
  m_v.resize(50,1);
  m_a.resize(50,1);
  m_tau.resize(50,1);
  m_q.fill(0.0);
  m_v.fill(0.0);
  m_a.fill(0.0);
  m_tau.fill(0.0);
  MAL_VECTOR_RESIZE(m_qmal,50);
  MAL_VECTOR_RESIZE(m_vmal,50);
  MAL_VECTOR_RESIZE(m_amal,50);
  MAL_VECTOR_FILL(m_qmal,0.0);
  MAL_VECTOR_FILL(m_vmal,0.0);
  MAL_VECTOR_FILL(m_amal,0.0);

  m_f.fill(0.0);
  m_n.fill(0.0);
  m_com.fill(0.0);

  m_boolModel     = false ;
  m_boolData      = false ;
  m_boolLeftFoot  = false ;
  m_boolRightFoot = false ;
  m_isLegInverseKinematic = false ;
  m_isArmInverseKinematic = false ;

  m_chest = 0 ;
  m_waist = 0 ;
  m_leftShoulder = 0 ;
  m_rightShoulder = 0 ;
  m_leftWrist = 0 ;
  m_rightWrist = 0;
  m_mass = 0.0 ;
  memset(&m_leftFoot,0,sizeof(m_leftFoot));
  memset(&m_rightFoot,0,sizeof(m_rightFoot));

  m_femurLength = 0.0 ;
  m_tibiaLengthZ = 0.0 ;
  m_tibiaLengthY = 0.0 ;

}

PinocchioRobot::~PinocchioRobot()
{
  if (m_robotDataInInitialePose != 0)
  {
    delete m_robotDataInInitialePose ;
    m_robotDataInInitialePose = 0 ;
  }
}

bool PinocchioRobot::checkModel(se3::Model * robotModel)
{
  if(!robotModel->existFrame("r_ankle"))
  {
    m_boolModel=false;
    const std::string exception_message ("r_ankle is not a valid body name");
    throw std::invalid_argument(exception_message);
    return false ;
  }
  if(!robotModel->existFrame("l_ankle"))
  {
    m_boolModel=false;
    const std::string exception_message ("l_ankle is not a valid body name");
    throw std::invalid_argument(exception_message);
    return false ;
  }
  if(!robotModel->existFrame("BODY") && !robotModel->existFrame("body"))
  {
    m_boolModel=false;
    const std::string exception_message ("BODY is not a valid body name");
    throw std::invalid_argument(exception_message);
    return false ;
  }
  if(!robotModel->existFrame("torso"))
  {
    m_boolModel=false;
    const std::string exception_message ("torso is not a valid body name");
    throw std::invalid_argument(exception_message);
    return false ;
  }
  if(!robotModel->existFrame("r_wrist"))
  {
    m_boolModel=false;
    const std::string exception_message ("r_wrist is not a valid body name");
    throw std::invalid_argument(exception_message);
    return false ;
  }
  if(!robotModel->existFrame("l_wrist"))
  {
    const std::string exception_message ("l_wrist is not a valid body name");
    throw std::invalid_argument(exception_message);
    return false ;
  }
  return true ;
}

bool PinocchioRobot::initializeRobotModelAndData(se3::Model * robotModel,
                                                 se3::Data * robotData)
{
  m_boolModel=checkModel(robotModel);
  if(!m_boolModel)
    return false ;

  // initialize the model
  ///////////////////////
  m_robotModel = robotModel;

  // initialize the short cut for the joint ids
  se3::FrameIndex chest = m_robotModel->getFrameId("torso");
  m_chest = m_robotModel->frames[chest].parent ;
  se3::FrameIndex waist = (robotModel->existFrame("BODY"))?m_robotModel->getFrameId("BODY"):m_robotModel->getFrameId("body");
  m_waist = m_robotModel->frames[waist].parent ;
  se3::FrameIndex ra = m_robotModel->getFrameId("r_ankle");
  m_rightFoot.associatedAnkle = m_robotModel->frames[ra].parent ;
  se3::FrameIndex la = m_robotModel->getFrameId("l_ankle");
  m_leftFoot.associatedAnkle = m_robotModel->frames[la].parent ;
  se3::FrameIndex rw = m_robotModel->getFrameId("r_wrist");
  m_rightWrist = m_robotModel->frames[rw].parent ;
  se3::FrameIndex lw = m_robotModel->getFrameId("l_wrist");
  m_leftWrist = m_robotModel->frames[lw].parent ;

  DetectAutomaticallyShoulders();

  // intialize the "initial pose" (q=[0]) data
  m_robotDataInInitialePose = new se3::Data(*m_robotModel);
  m_robotDataInInitialePose->v[0] = se3::Motion::Zero();
  m_robotDataInInitialePose->a[0] = -m_robotModel->gravity;
  m_q.resize(m_robotModel->nq,1);
  m_q.fill(0.0);
  m_q[6]= 1.0 ;
  m_v.resize(m_robotModel->nv,1);
  m_a.resize(m_robotModel->nv,1);
  m_tau.resize(m_robotModel->nv,1);
  se3::forwardKinematics(*m_robotModel,*m_robotDataInInitialePose,m_q);

  MAL_VECTOR_RESIZE(m_qmal,m_robotModel->nv);
  MAL_VECTOR_RESIZE(m_vmal,m_robotModel->nv);
  MAL_VECTOR_RESIZE(m_amal,m_robotModel->nv);
  MAL_VECTOR_FILL(m_qmal,0.0);
  MAL_VECTOR_FILL(m_vmal,0.0);
  MAL_VECTOR_FILL(m_amal,0.0);

  // compute the global mass of the robot
  m_mass=0.0;
  for(unsigned i=0; i<m_robotModel->inertias.size() ; ++i)
  {
    m_mass += m_robotModel->inertias[i].mass();
  }

  // initialize the data
  //////////////////////
  if (robotData==0)
  {
    m_boolData = false ;
    return false;
  }
  else
    m_boolData=true;
  m_robotData = robotData;
  m_robotData->v[0] = se3::Motion::Zero();
  m_robotData->a[0] = -m_robotModel->gravity;

  if(testInverseKinematics())
    initializeInverseKinematics();

  return true ;
}

bool PinocchioRobot::initializeLeftFoot(PRFoot leftFoot)
{
  m_leftFoot = leftFoot ;
  m_boolLeftFoot = true ;
  return true ;
}

bool PinocchioRobot::initializeRightFoot(PRFoot rightFoot)
{
  m_rightFoot = rightFoot ;
  m_boolRightFoot = true ;
  return true ;
}

bool PinocchioRobot::testInverseKinematics()
{
  std::vector<se3::JointIndex> leftLeg =
      jointsBetween(m_waist,m_leftFoot.associatedAnkle);
  std::vector<se3::JointIndex> rightLeg =
      jointsBetween(m_waist,m_rightFoot.associatedAnkle);
  std::vector<se3::JointIndex> leftArm =
      jointsBetween(m_chest,m_leftWrist);
  std::vector<se3::JointIndex> rightArm =
      jointsBetween(m_chest,m_rightWrist);

  std::vector<std::string> leftLegJointName,rightLegJointName,
      leftArmJointName,rightArmJointName;
  leftLegJointName.push_back("JointModelFreeFlyer");
  leftLegJointName.push_back("JointModelRZ");
  leftLegJointName.push_back("JointModelRX");
  leftLegJointName.push_back("JointModelRY");
  leftLegJointName.push_back("JointModelRY");
  leftLegJointName.push_back("JointModelRY");
  leftLegJointName.push_back("JointModelRX");
  
  rightLegJointName.push_back("JointModelFreeFlyer");
  rightLegJointName.push_back("JointModelRZ");
  rightLegJointName.push_back("JointModelRX");
  rightLegJointName.push_back("JointModelRY");
  rightLegJointName.push_back("JointModelRY");
  rightLegJointName.push_back("JointModelRY");
  rightLegJointName.push_back("JointModelRX");
  
  leftArmJointName.push_back("JointModelRY");
  leftArmJointName.push_back("JointModelRX");
  leftArmJointName.push_back("JointModelRZ");
  leftArmJointName.push_back("JointModelRY");
  leftArmJointName.push_back("JointModelRZ");
  leftArmJointName.push_back("JointModelRY");
  rightArmJointName.push_back("JointModelRY");
  rightArmJointName.push_back("JointModelRX");
  rightArmJointName.push_back("JointModelRZ");
  rightArmJointName.push_back("JointModelRY");
  rightArmJointName.push_back("JointModelRZ");
  rightArmJointName.push_back("JointModelRY");

  m_isLegInverseKinematic = true ;
  m_isArmInverseKinematic = true ;
  for (unsigned  i=0 ; i<leftLegJointName.size() ; ++i)
  {
    std::string shortName = boost::apply_visitor(Joint_shortname(),
                                            m_robotModel->joints[leftLeg[i]]);
    m_isLegInverseKinematic &= (shortName == leftLegJointName[i]);
  }
  for (unsigned  i=0 ; i<rightLegJointName.size() ; ++i)
  {
    std::string shortName = boost::apply_visitor(Joint_shortname(),
                                            m_robotModel->joints[rightLeg[i]]);
    m_isLegInverseKinematic &= (shortName == rightLegJointName[i]);
  }
  for (unsigned  i=0 ; i<leftArmJointName.size() ; ++i)
  {
    std::string shortName = boost::apply_visitor(Joint_shortname(),
                                            m_robotModel->joints[leftArm[i]]);
    m_isArmInverseKinematic &= (shortName == leftArmJointName[i]);
  }
  for (unsigned  i=0 ; i<rightArmJointName.size() ; ++i)
  {
    std::string shortName = boost::apply_visitor(Joint_shortname(),
                                            m_robotModel->joints[rightArm[i]]);
    m_isArmInverseKinematic &= (shortName == rightArmJointName[i]);
  }

  return m_isLegInverseKinematic & m_isArmInverseKinematic;
}

void PinocchioRobot::initializeInverseKinematics()
{
  std::vector<se3::JointIndex> leftLeg =
      jointsBetween(m_waist,m_leftFoot.associatedAnkle);
  std::vector<se3::JointIndex> rightLeg =
      jointsBetween(m_waist,m_rightFoot.associatedAnkle);

  MAL_S3_VECTOR_CLEAR(m_leftDt);
  MAL_S3_VECTOR_CLEAR(m_rightDt);
  se3::SE3 waist_M_leftHip , waist_M_rightHip ;
  waist_M_leftHip = m_robotModel->jointPlacements[leftLeg[0]].act(
        m_robotModel->jointPlacements[leftLeg[1]]).act(
        m_robotModel->jointPlacements[leftLeg[2]]).act(
        m_robotModel->jointPlacements[leftLeg[3]]);
  waist_M_rightHip = m_robotModel->jointPlacements[rightLeg[0]].act(
        m_robotModel->jointPlacements[rightLeg[1]]).act(
        m_robotModel->jointPlacements[rightLeg[2]]).act(
        m_robotModel->jointPlacements[rightLeg[3]]);

  m_leftDt(0)=waist_M_leftHip.translation()(0);
  m_leftDt(1)=waist_M_leftHip.translation()(1);
  m_leftDt(2)=waist_M_leftHip.translation()(2);
  m_rightDt(0)=waist_M_rightHip.translation()(0);
  m_rightDt(1)=waist_M_rightHip.translation()(1);
  m_rightDt(2)=waist_M_rightHip.translation()(2);

  m_femurLength = m_robotModel->jointPlacements[rightLeg[4]]
    .translation().norm();

  m_tibiaLengthY =
      std::abs(m_robotModel->jointPlacements[rightLeg[5]].translation()[1]);
  m_tibiaLengthZ =
      std::abs(m_robotModel->jointPlacements[rightLeg[5]].translation()[2]);

  if(m_femurLength==0 || m_tibiaLengthZ==0)
    m_isLegInverseKinematic=false;

  return ;
}

void PinocchioRobot::RPYToSpatialFreeFlyer(Eigen::Vector3d & rpy,
                                           Eigen::Vector3d & drpy,
                                           Eigen::Vector3d & ddrpy,
                                           Eigen::Quaterniond & quat,
                                           Eigen::Vector3d & omega,
                                           Eigen::Vector3d & domega)
{
  quat = Eigen::Quaterniond(
        Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) ) ;
  quat.normalize();
  double c0,s0; SINCOS (rpy(2), &s0, &c0);
  double c1,s1; SINCOS (rpy(1), &s1, &c1);
  double c2,s2; SINCOS (rpy(0), &s2, &c2);
  m_S << -s1, 0., 1., c1 * s2, c2, 0, c1 * c2, -s2, 0;
  omega = m_S * drpy ;
  domega = m_S * ddrpy ;
  domega(0) += -c1 * drpy (0) * drpy (1);
  domega(1) += -s1 * s2 * drpy (0) * drpy (1) + c1 * c2 * drpy (0) * drpy (2) - s2 * drpy (1) * drpy (2);
  domega(2) += -s1 * c2 * drpy (0) * drpy (1) - c1 * s2 * drpy (0) * drpy (2) - c2 * drpy (1) * drpy (2);
}

void PinocchioRobot::computeForwardKinematics()
{
  computeForwardKinematics(m_qmal);
}

void PinocchioRobot::computeForwardKinematics(MAL_VECTOR_TYPE(double) & q)
{
  // euler to quaternion :
  m_quat = Eigen::Quaterniond(
        Eigen::AngleAxisd(q(5), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(q(4), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(q(3), Eigen::Vector3d::UnitX()) ) ;
  m_quat.normalize();
  // fill up m_q following the pinocchio standard : [pos quarternion DoFs]
  for(unsigned i=0; i<3 ; ++i)
  {
    m_q(i) = q(i);
  }
  m_q(3) = m_quat.x() ;
  m_q(4) = m_quat.y() ;
  m_q(5) = m_quat.z() ;
  m_q(6) = m_quat.w() ;
  for(int i=0; i<m_robotModel->nv-6 ; ++i)
  {
    m_q(7+i) = q(6+i);
  }
  se3::forwardKinematics(*m_robotModel,*m_robotData,m_q);
  se3::centerOfMass(*m_robotModel,*m_robotData,m_q);
}

void PinocchioRobot::computeInverseDynamics()
{
  PinocchioRobot::computeInverseDynamics(m_qmal,m_vmal,m_amal);
}

void PinocchioRobot::computeInverseDynamics(MAL_VECTOR_TYPE(double) & q,
                                            MAL_VECTOR_TYPE(double) & v,
                                            MAL_VECTOR_TYPE(double) & a)
{
//  for(unsigned i=0;i<3;++i)
//  {
//    m_rpy   (i) = q(3+i);
//    m_drpy  (i) = v(3+i);
//    m_ddrpy (i) = a(3+i);
//  }
//  RPYToSpatialFreeFlyer(m_rpy,m_drpy,m_ddrpy,
//                        m_quat,m_omega,m_domega);
  // euler to quaternion :
  m_quat = Eigen::Quaterniond(
        Eigen::AngleAxisd(q(5), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(q(4), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(q(3), Eigen::Vector3d::UnitX()) ) ;
  for(unsigned i=0; i<3 ; ++i)
  {
    m_q(i) = q(i);
    m_v(i) = v(i);
    m_a(i) = a(i);
  }
  m_rot = m_quat.toRotationMatrix().transpose() ;
  m_v.segment<3>(0) = m_rot * m_v.segment<3>(0) ;
  m_a.segment<3>(0) = m_rot * m_a.segment<3>(0) ;

  // fill up m_q following the pinocchio standard : [pos quarternion DoFs]
  m_q(3) = m_quat.x() ;
  m_q(4) = m_quat.y() ;
  m_q(5) = m_quat.z() ;
  m_q(6) = m_quat.w() ;

  // fill up the velocity and acceleration vectors
  //m_v.segment<3>(3) = m_omega ;
  //m_a.segment<3>(3) = m_domega ;

  for(int i=6; i<m_robotModel->nv-6 ; ++i)
  {
    m_q(1+i) = q(i);
    m_v(i)   = v(i);
    m_a(i)   = a(i);
  }
  // performing the inverse dynamics
  m_tau = se3::rnea(*m_robotModel,*m_robotData,m_q,m_v,m_a);
}

std::vector<se3::JointIndex> PinocchioRobot::fromRootToIt(se3::JointIndex it)
{
  std::vector<se3::JointIndex> fromRootToIt ;
  fromRootToIt.clear();
  se3::JointIndex i = it ;
  while(i!=0)
  {
    fromRootToIt.insert(fromRootToIt.begin(),i);
    i = m_robotModel->parents[i];
  }
  return fromRootToIt ;
}

std::vector<se3::JointIndex> PinocchioRobot::jointsBetween
( se3::JointIndex first, se3::JointIndex second)
{
  std::vector<se3::JointIndex> fromRootToFirst  = fromRootToIt(first);
  std::vector<se3::JointIndex> fromRootToSecond = fromRootToIt(second);

  std::vector<se3::JointIndex> out ;
  out.clear();
  se3::JointIndex lastCommonRank = 0 ;
  se3::JointIndex minChainLength =
      fromRootToFirst.size() < fromRootToSecond.size()
      ? fromRootToFirst.size() : fromRootToSecond.size() ;

  for(unsigned k=1 ; k<minChainLength ; ++k)
  {
    if(fromRootToFirst[k] == fromRootToSecond[k])
      ++lastCommonRank;
  }

  for(unsigned k=fromRootToFirst.size()-1; k>lastCommonRank ; --k)
  {
    out.push_back(fromRootToFirst[k]);
  }
  if(lastCommonRank==0)
  {
    out.push_back(fromRootToSecond[0]);
  }
  for(unsigned k=lastCommonRank+1 ; k<fromRootToSecond.size() ; ++k)
  {
    out.push_back(fromRootToSecond[k]);
  }

  return out ;
}

///////////////////////////////////////////////////////////////////////////////
bool PinocchioRobot::
ComputeSpecializedInverseKinematics(
    const se3::JointIndex &jointRoot,
    const se3::JointIndex &jointEnd,
    const MAL_S4x4_MATRIX_TYPE(double) & jointRootPosition,
    const MAL_S4x4_MATRIX_TYPE(double) & jointEndPosition,
    MAL_VECTOR_TYPE(double) &q )
{
  MAL_VECTOR_FILL(q,0.0);
  /*! Try to find out which kinematics chain the user
    send to the method.*/
  if (jointRoot==m_waist)
  {
    if(!m_isLegInverseKinematic)
      return false ;

    /* Consider here the legs. */
    if (jointEnd==m_leftFoot.associatedAnkle)
    {
      getWaistFootKinematics(jointRootPosition, jointEndPosition,
                             q, m_leftDt);
      return true;
    }
    else if (jointEnd==m_rightFoot.associatedAnkle)
    {
      getWaistFootKinematics(jointRootPosition, jointEndPosition,
                             q, m_rightDt);
      return true;
    }else
    {
      return false ;
    }
  }
  else
  {
    if(!m_isArmInverseKinematic)
      return false ;

    if ( (m_leftShoulder==0) || (m_rightShoulder==0) )
      DetectAutomaticallyShoulders();

    /* Here consider the arms */
    if (jointRoot==m_leftShoulder && jointEnd==m_leftWrist)
    {
      getShoulderWristKinematics(jointRootPosition,jointEndPosition,q,1);
      return true;
    }
    if (jointRoot==m_rightShoulder && jointEnd==m_rightWrist)
    {
      getShoulderWristKinematics(jointRootPosition,jointEndPosition,q,-1);
      return true;
    }
  }
  return false;
}

void PinocchioRobot::getWaistFootKinematics(const matrix4d & jointRootPosition,
                                            const matrix4d & jointEndPosition,
                                            vectorN &q,
                                            vector3d Dt)
{
  double _epsilon=1.0e-6;
  // definition des variables relatif au design du robot
  double A = m_femurLength;
  double B = m_tibiaLengthZ;
  //double C = 0.0;
  double c5 = 0.0;
  double q6a = 0.0;

  //vector3d r;

  /* Build sub-matrices */
  matrix3d Foot_R,Body_R;
  vector3d Foot_P,Body_P;
  for(unsigned int i=0;i<3;i++)
  {
    for(unsigned int j=0;j<3;j++)
    {
      MAL_S3x3_MATRIX_ACCESS_I_J(Body_R,i,j) =
          MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,j);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R,i,j) =
          MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,j);
    }
    Body_P(i) = MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,3);
    Foot_P(i) = MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,3);
  }

  matrix3d Foot_Rt;
  MAL_S3x3_TRANSPOSE_A_in_At(Foot_R,Foot_Rt);

  // Initialisation of q
  if (MAL_VECTOR_SIZE(q)!=6)
    MAL_VECTOR_RESIZE(q,6);

  for(unsigned int i=0;i<6;i++)
    q(i)=0.0;

  // if Dt(1)<0.0 then Opp=1.0 else Opp=-1.0
  double OppSignOfDtY = Dt(1) < 0.0 ? 1.0 : -1.0;

  vector3d d2,d3;
  d2 = Body_P + Body_R * Dt;
  d3 = d2 - Foot_P;

  double l0 = sqrt(d3(0)*d3(0)+d3(1)*d3(1)+d3(2)*d3(2)
                   - m_tibiaLengthY*m_tibiaLengthY);
  c5 = 0.5 * (l0*l0-A*A-B*B) / (A*B);
  if (c5 > 1.0-_epsilon)
  {
    q[3] = 0.0;
  }
  if (c5 < -1.0+_epsilon)
  {
    q[3] = M_PI;
  }
  if (c5 >= -1.0+_epsilon && c5 <= 1.0-_epsilon)
  {
    q[3] = acos(c5);
  }

  vector3d r3;
  r3 = Foot_Rt * d3;

  q6a = asin((A/l0)*sin(M_PI- q[3]));

  double l3 = sqrt(r3(1)*r3(1) + r3(2)*r3(2));
  double l4 = sqrt(l3*l3 - m_tibiaLengthY*m_tibiaLengthY);

  double phi = atan2(r3(0), l4);
  q[4] = -phi - q6a;

  double psi1 = atan2(r3(1), r3(2)) * OppSignOfDtY;
  double psi2 = 0.5*M_PI - psi1;
  double psi3 = atan2(l4, m_tibiaLengthY);
  q[5] = (psi3 - psi2) * OppSignOfDtY;

  if (q[5] > 0.5*M_PI)
  {
    q[5] -= M_PI;
  }
  else if (q[5] < -0.5*M_PI)
  {
    q[5] += M_PI;
  }

  matrix3d R;
  matrix3d BRt;
  MAL_S3x3_TRANSPOSE_A_in_At(Body_R,BRt);

  matrix3d Rroll;
  double c = cos(q[5]);
  double s = sin(q[5]);

  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,0,0) = 1.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,0,1) = 0.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,0,2) = 0.0;

  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,1,0) = 0.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,1,1) = c;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,1,2) = s;

  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,2,0) = 0.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,2,1) = -s;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,2,2) = c;

  matrix3d Rpitch;
  c = cos(q[4]+q[3]);
  s = sin(q[4]+q[3]);

  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,0,0) = c;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,0,1) = 0.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,0,2) = -s;

  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,1,0) = 0.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,1,1) = 1.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,1,2) = 0.0;

  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,2,0) = s;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,2,1) = 0.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,2,2) = c;

  R = BRt * Foot_R * Rroll * Rpitch;
  q[0] = atan2(-R(0,1),R(1,1));

  double cz = cos(q[0]);
  double sz = sin(q[0]);

  q[1] = atan2(R(2,1), -R(0,1)*sz+R(1,1)*cz);
  q[2] = atan2( -R(2,0), R(2,2));
}

double PinocchioRobot::ComputeXmax(double & Z)
{
  double A=0.25,
      B=0.25;
  double Xmax;
  if (Z<0.0)
    Z = 2*A*cos(15*M_PI/180.0);
  Xmax = sqrt(A*A - (Z - B)*(Z-B));
  return Xmax;
}

void PinocchioRobot::getShoulderWristKinematics(const matrix4d & jointRootPosition,
                                                const matrix4d & jointEndPosition,
                                                vectorN &q,
                                                int side)
{

  // Initialisation of q
  if (MAL_VECTOR_SIZE(q)!=6)
    MAL_VECTOR_RESIZE(q,6);

  double Alpha,Beta;
  for(unsigned int i=0;i<6;i++)
    q(i)=0.0;

  double X = MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,0,3)
      - MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,0,3);
  double Z = MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,2,3)
      - MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,2,3);

  double Xmax = ComputeXmax(Z);
  X = X*Xmax;

  double A=0.25, B=0.25; //UpperArmLength ForeArmLength

  double C=0.0,Gamma=0.0,Theta=0.0;
  C = sqrt(X*X+Z*Z);

  Beta = acos((A*A+B*B-C*C)/(2*A*B))- M_PI;
  Gamma = asin((B*sin(M_PI+Beta))/C);
  Theta = atan2(X,Z);
  Alpha = Gamma - Theta;

  // Fill in the joint values.
  q(0)= Alpha;
  q(1)= 10.0*M_PI/180.0;
  q(2)= 0.0;
  q(3)= Beta;
  q(4)= 0.0;
  q(5)= 0.0;

  if (side==-1)
    q(1) = -q(1);


}

void PinocchioRobot::DetectAutomaticallyShoulders()
{
  DetectAutomaticallyOneShoulder(m_leftWrist,m_leftShoulder);
  DetectAutomaticallyOneShoulder(m_rightWrist,m_rightShoulder);
}

void PinocchioRobot::DetectAutomaticallyOneShoulder(
    se3::JointIndex aWrist,
    se3::JointIndex & aShoulder)
{
  std::vector<se3::JointIndex>FromRootToJoint;

  FromRootToJoint.clear();
  FromRootToJoint = fromRootToIt(aWrist);

  std::vector<se3::JointIndex>::iterator itJoint = FromRootToJoint.begin();
  bool found=false;
  while(itJoint!=FromRootToJoint.end())
  {
    std::vector<se3::JointIndex>::iterator current = itJoint;
    if (*current==m_chest)
      found=true;
    else
    {
      if (found)
      {
        aShoulder = *current;
        return;
      }
    }
    itJoint++;
  }
}

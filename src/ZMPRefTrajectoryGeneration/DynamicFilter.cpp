#include "DynamicFilter.hh"
#include <metapod/algos/rnea.hh>

using namespace std;
using namespace PatternGeneratorJRL;
using namespace metapod;

DynamicFilter::DynamicFilter(
    SimplePluginManager *SPM,
    CjrlHumanoidDynamicRobot *aHS)
{
  currentTime_ = 0.0 ;
  controlPeriod_ = 0.0 ;
  interpolationPeriod_ = 0.0 ;
  previewWindowSize_ = 0.0 ;
  PG_T_ = 0.0 ;
  NbI_ = 0.0 ;
  NCtrl_ = 0.0;
  NbI_ = 0.0 ;

  comAndFootRealization_ = new ComAndFootRealizationByGeometry(
      (PatternGeneratorInterfacePrivate*) SPM );
  comAndFootRealization_->setHumanoidDynamicRobot(aHS);
  comAndFootRealization_->SetHeightOfTheCoM(CoMHeight_);
  comAndFootRealization_->setSamplingPeriod(interpolationPeriod_);
  comAndFootRealization_->Initialization();

  PC_ = new PreviewControl(
      SPM,OptimalControllerSolver::MODE_WITH_INITIALPOS,false);
  CoMHeight_ = 0.0 ;

  configurationTraj_.clear();
  velocityTraj_.clear();
  accelerationTraj_.clear();
  previousConfiguration_.clear();
  deltaZMP_deq_.clear();
  ZMPMB_vec_.clear();

  MAL_VECTOR_RESIZE(aCoMState_,6);
  MAL_VECTOR_RESIZE(aCoMSpeed_,6);
  MAL_VECTOR_RESIZE(aCoMAcc_,6);
  MAL_VECTOR_RESIZE(aLeftFootPosition_,5);
  MAL_VECTOR_RESIZE(aRightFootPosition_,5);
  MAL_MATRIX_RESIZE(deltax_,3,1);
  MAL_MATRIX_RESIZE(deltay_,3,1);

  previousConfiguration_ = aHS->currentConfiguration() ;
  previousVelocity_ = aHS->currentVelocity() ;
  previousAcceleration_ = aHS->currentAcceleration() ;

  comAndFootRealization_->SetPreviousConfigurationStage0(
      previousConfiguration_);
  comAndFootRealization_->SetPreviousVelocityStage0(
      previousVelocity_);

  Once_ = true ;
  DInitX_ = 0.0 ;
  DInitY_ = 0.0 ;

  jacobian_lf_ = Jacobian_LF::Jacobian::Zero();
  jacobian_rf_ = Jacobian_RF::Jacobian::Zero();

  sxzmp_ = 0.0 ;
  syzmp_ = 0.0 ;
  deltaZMPx_ = 0.0 ;
  deltaZMPy_ = 0.0 ;

  upperPartIndex.clear();

  walkingHeuristic_ = false ;
}

DynamicFilter::~DynamicFilter()
{
  if (PC_!=0){
    delete PC_;
    PC_ = 0 ;
  }
  if (comAndFootRealization_!=0){
    delete comAndFootRealization_;
    comAndFootRealization_ = 0 ;
  }
}

void DynamicFilter::ForwardKinematics(MAL_VECTOR_TYPE(double) & configuration,
                                      MAL_VECTOR_TYPE(double) & velocity,
                                      MAL_VECTOR_TYPE(double) & acceleration)
{
  Robot_Model::confVector q, dq, ddq ;
  for(unsigned int j = 0 ; j < configuration.size() ; j++ )
  {
    q(j,0) = configuration(j) ;
    dq(j,0) = velocity(j) ;
    ddq(j,0) = acceleration(j) ;
  }
  metapod::rnea< Robot_Model, true >::run(robot_, q, dq, ddq);
}

void DynamicFilter::setRobotUpperPart(MAL_VECTOR_TYPE(double) & configuration,
                                      MAL_VECTOR_TYPE(double) & velocity,
                                      MAL_VECTOR_TYPE(double) & acceleration)
{
  for ( unsigned int i = 0 ; i < upperPartIndex.size() ; ++i )
  {
    upperPartConfiguration_(upperPartIndex[i])  = configuration(upperPartIndex[i]);
    upperPartVelocity_(upperPartIndex[i])       = velocity(upperPartIndex[i]);
    upperPartAcceleration_(upperPartIndex[i])   = acceleration(upperPartIndex[i]);
  }
  return ;
}

/// \brief Initialse all objects, to be called just after the constructor
void DynamicFilter::init(
    double currentTime,
    double controlPeriod,
    double interpolationPeriod,
    double PG_T,
    double previewWindowSize,
    double CoMHeight,
    FootAbsolutePosition inputLeftFoot,
    COMState inputCoMState)
{
  currentTime_ = currentTime ;
  controlPeriod_ = controlPeriod ;
  interpolationPeriod_ = interpolationPeriod ;
  PG_T_ = PG_T ;
  previewWindowSize_ = previewWindowSize ;

  if (interpolationPeriod_>PG_T)
  {NbI_=1;}
  else
  {NbI_ = (int)(PG_T/interpolationPeriod_);}

  NCtrl_ = (int)(PG_T_/controlPeriod_) ;
  PG_N_ = (int)(previewWindowSize_/interpolationPeriod_) ;

  CoMHeight_ = CoMHeight ;
  PC_->SetPreviewControlTime (previewWindowSize_);
  PC_->SetSamplingPeriod (interpolationPeriod_);
  PC_->SetHeightOfCoM(CoMHeight_);
  PC_->ComputeOptimalWeights(OptimalControllerSolver::MODE_WITH_INITIALPOS);

  previousConfiguration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentConfiguration() ;
  previousVelocity_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentVelocity() ;
  previousAcceleration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentAcceleration() ;

  upperPartConfiguration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentConfiguration() ;
  previousUpperPartConfiguration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentConfiguration() ;
  upperPartVelocity_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentVelocity() ;
  previousUpperPartVelocity_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentVelocity() ;
  upperPartAcceleration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentAcceleration() ;

  ZMPMBConfiguration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentConfiguration() ;
  ZMPMBVelocity_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentVelocity() ;
  ZMPMBAcceleration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentAcceleration() ;

  configurationTraj_.resize( PG_N_, previousConfiguration_ ); ;
  velocityTraj_.resize( PG_N_, previousVelocity_ ); ;
  accelerationTraj_.resize( PG_N_, previousAcceleration_ ); ;

  for(unsigned int j = 0 ; j < ZMPMBConfiguration_.size() ; j++ )
  {
    q_(j,0) = ZMPMBConfiguration_(j) ;
    dq_(j,0) = ZMPMBVelocity_(j) ;
    ddq_(j,0) = ZMPMBAcceleration_(j) ;
  }

  if (inputLeftFoot.stepType<0)
  {
    PreviousSupportFoot_ = true ; // left foot is supporting
  }
  else
  {
    PreviousSupportFoot_ = false ; // right foot is supporting
  }
  prev_q_ = q_ ;
  prev_dq_ = dq_ ;
  prev_ddq_ = ddq_ ;
  jcalc<Robot_Model>::run(robot_,q_ ,dq_ );
  jcalc<Robot_Model>::run(robot_2,q_, dq_);

  deltaZMP_deq_.resize( PG_N_);
  ZMPMB_vec_.resize( PG_N_, vector<double>(2));

  comAndFootRealization_->setSamplingPeriod(interpolationPeriod_);
  comAndFootRealization_->Initialization();

  MAL_VECTOR_RESIZE(aCoMState_,6);
  MAL_VECTOR_RESIZE(aCoMSpeed_,6);
  MAL_VECTOR_RESIZE(aCoMAcc_,6);
  MAL_VECTOR_RESIZE(aLeftFootPosition_,5);
  MAL_VECTOR_RESIZE(aRightFootPosition_,5);
  MAL_MATRIX_RESIZE(deltax_,3,1);
  MAL_MATRIX_RESIZE(deltay_,3,1);

  RootNode & node_waist = boost::fusion::at_c<Robot_Model::BODY>(robot_.nodes);
  CWx = node_waist.body.iX0.r()(0,0) - inputCoMState.x[0] ;
  CWy = node_waist.body.iX0.r()(1,0) - inputCoMState.y[0] ;

  sxzmp_ = 0.0 ;
  syzmp_ = 0.0 ;
  deltaZMPx_ = 0.0 ;
  deltaZMPy_ = 0.0 ;

  for(int j=0;j<3;j++)
  {
    deltax_(j,0) = 0 ;
    deltay_(j,0) = 0 ;
  }

  upperPartIndex.resize(2+2+7+7);
  for (unsigned int i = 0 ; i < upperPartIndex.size() ; ++i )
  {
    upperPartIndex[i]=i+18;
  }
  return ;
}

int DynamicFilter::OffLinefilter(
    COMState & lastCtrlCoMState,
    FootAbsolutePosition & lastCtrlLeftFoot,
    FootAbsolutePosition & lastCtrlRightFoot,
    deque<COMState> & inputCOMTraj_deq_,
    deque<ZMPPosition> inputZMPTraj_deq_,
    deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
    deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
    deque<COMState> & outputDeltaCOMTraj_deq_)
{
  return 0;
}

int DynamicFilter::OnLinefilter(
    const deque<COMState> & ctrlCoMState,
    const deque<FootAbsolutePosition> & ctrlLeftFoot,
    const deque<FootAbsolutePosition> & ctrlRightFoot,
    const deque<COMState> & inputCOMTraj_deq_,
    const deque<ZMPPosition> inputZMPTraj_deq_,
    const deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
    const deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
    deque<COMState> & outputDeltaCOMTraj_deq_)
{
  return 0 ;
}

void DynamicFilter::InverseKinematics(
    const COMState & inputCoMState,
    const FootAbsolutePosition & inputLeftFoot,
    const FootAbsolutePosition & inputRightFoot,
    MAL_VECTOR_TYPE(double)& configuration,
    MAL_VECTOR_TYPE(double)& velocity,
    MAL_VECTOR_TYPE(double)& acceleration,
    double samplingPeriod,
    int stage,
    int iteration)
{
  aCoMState_(0) = inputCoMState.x[0];       aCoMSpeed_(0) = inputCoMState.x[1];
  aCoMState_(1) = inputCoMState.y[0];       aCoMSpeed_(1) = inputCoMState.y[1];
  aCoMState_(2) = inputCoMState.z[0];       aCoMSpeed_(2) = inputCoMState.z[1];
  aCoMState_(3) = inputCoMState.roll[0];    aCoMSpeed_(3) = inputCoMState.roll[1];
  aCoMState_(4) = inputCoMState.pitch[0];   aCoMSpeed_(4) = inputCoMState.pitch[1];
  aCoMState_(5) = inputCoMState.yaw[0];     aCoMSpeed_(5) = inputCoMState.yaw[1];

  aCoMAcc_(0) = inputCoMState.x[2];         aLeftFootPosition_(0) = inputLeftFoot.x;
  aCoMAcc_(1) = inputCoMState.y[2];         aLeftFootPosition_(1) = inputLeftFoot.y;
  aCoMAcc_(2) = inputCoMState.z[2];         aLeftFootPosition_(2) = inputLeftFoot.z;
  aCoMAcc_(3) = inputCoMState.roll[2];      aLeftFootPosition_(3) = inputLeftFoot.theta;
  aCoMAcc_(4) = inputCoMState.pitch[2];     aLeftFootPosition_(4) = inputLeftFoot.omega;
  aCoMAcc_(5) = inputCoMState.yaw[2];

  aRightFootPosition_(0) = inputRightFoot.x;
  aRightFootPosition_(1) = inputRightFoot.y;
  aRightFootPosition_(2) = inputRightFoot.z;
  aRightFootPosition_(3) = inputRightFoot.theta;
  aRightFootPosition_(4) = inputRightFoot.omega;

  comAndFootRealization_->setSamplingPeriod(samplingPeriod);
  comAndFootRealization_->ComputePostureForGivenCoMAndFeetPosture(
      aCoMState_, aCoMSpeed_, aCoMAcc_,
      aLeftFootPosition_, aRightFootPosition_,
      configuration, velocity, acceleration,
      iteration, stage);

  static int it = 0 ;

  if (walkingHeuristic_)
  {
    LankleNode & node_lankle = boost::fusion::at_c<Robot_Model::l_ankle>(robot_.nodes);
    RankleNode & node_rankle = boost::fusion::at_c<Robot_Model::r_ankle>(robot_.nodes);

    LhandNode & node_lhand = boost::fusion::at_c<Robot_Model::l_wrist>(robot_.nodes);
    RhandNode & node_rhand = boost::fusion::at_c<Robot_Model::r_wrist>(robot_.nodes);

    LshoulderNode & node_lshoulder = boost::fusion::at_c<Robot_Model::LARM_LINK0>(robot_.nodes);
    RshoulderNode & node_rshoulder = boost::fusion::at_c<Robot_Model::RARM_LINK0>(robot_.nodes);

    RootNode & node_waist = boost::fusion::at_c<Robot_Model::BODY>(robot_.nodes);

    Spatial::TransformT<LocalFloatType,Spatial::RotationMatrixTpl<LocalFloatType> > waistXlf ;
    Spatial::TransformT<LocalFloatType,Spatial::RotationMatrixTpl<LocalFloatType> > waistXrf ;
    waistXlf = node_waist.body.iX0 * node_lankle.body.iX0.inverse() ;
    waistXrf = node_waist.body.iX0 * node_rankle.body.iX0.inverse() ;
//
//    Spatial::TransformT<LocalFloatType,Spatial::RotationMatrixTpl<LocalFloatType> > shoulderXlh ;
//    Spatial::TransformT<LocalFloatType,Spatial::RotationMatrixTpl<LocalFloatType> > shoulderXrh ;
//    shoulderXlh = node_lshoulder.body.iX0 * node_lhand.body.iX0.inverse() ;
//    shoulderXrh = node_rshoulder.body.iX0 * node_rhand.body.iX0.inverse() ;

    // Homogeneous matrix
    matrix4d identity,leftHandPose, rightHandPose;
    MAL_S4x4_MATRIX_SET_IDENTITY(identity);
    MAL_S4x4_MATRIX_SET_IDENTITY(leftHandPose);
    MAL_S4x4_MATRIX_SET_IDENTITY(rightHandPose);

    MAL_S4x4_MATRIX_ACCESS_I_J(leftHandPose,0,3) = -4*waistXrf.r()(0);
    MAL_S4x4_MATRIX_ACCESS_I_J(leftHandPose,1,3) = 0.0;
    MAL_S4x4_MATRIX_ACCESS_I_J(leftHandPose,2,3) = -0.45;



    MAL_S4x4_MATRIX_ACCESS_I_J(rightHandPose,0,3) = -4*waistXlf.r()(0);
    MAL_S4x4_MATRIX_ACCESS_I_J(rightHandPose,1,3) = 0.0;
    MAL_S4x4_MATRIX_ACCESS_I_J(rightHandPose,2,3) = -0.45;

    MAL_VECTOR_DIM(larm_q, double, 6) ;
    MAL_VECTOR_DIM(rarm_q, double, 6) ;

    CjrlHumanoidDynamicRobot * HDR = comAndFootRealization_->getHumanoidDynamicRobot();
    int err1,err2;

    CjrlJoint* left_shoulder = NULL ;
    CjrlJoint* right_shoulder = NULL ;

    std::vector<CjrlJoint*> actuatedJoints = HDR->getActuatedJoints() ;
    for (unsigned int i = 0 ; i < actuatedJoints.size() ; ++i )
    {
      if ( actuatedJoints[i]->getName() == "LARM_JOINT0" )
        left_shoulder = actuatedJoints[i];
      if ( actuatedJoints[i]->getName() == "RARM_JOINT0" )
        right_shoulder = actuatedJoints[i];
    }


    err1 = HDR->getSpecializedInverseKinematics( *left_shoulder ,*(HDR->leftWrist()), identity, leftHandPose, larm_q );
    err2 = HDR->getSpecializedInverseKinematics( *right_shoulder ,*(HDR->rightWrist()), identity, rightHandPose, rarm_q );

    ofstream aof;

    string aFileName;
    if ( it == 0 ){
      aFileName = "larmrarm.dat";
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
    aFileName = "larmrarm.dat";
    aof.open(aFileName.c_str(),ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);

    aof << it * 0.005 << " "  ; // 1

    for(unsigned int i = 0 ; i < larm_q.size() ; i++){
      aof << rarm_q(i) << " "  ; // 2
    }

    for(unsigned int i = 0 ; i < rarm_q.size() ; i++){
      aof << larm_q(i) << " "  ; // 8
    }
    aof << waistXrf.r()(0) << " "
        << waistXlf.r()(0) << " ";

    aof  << endl ;
    aof.close();

    // swinging arms
    upperPartConfiguration_(upperPartIndex[0])= 0.0 ;             // CHEST_JOINT0
    upperPartConfiguration_(upperPartIndex[1])= 0.015 ;           // CHEST_JOINT1
    upperPartConfiguration_(upperPartIndex[2])= 0.0 ;             // HEAD_JOINT0
    upperPartConfiguration_(upperPartIndex[3])= 0.0 ;             // HEAD_JOINT1
    upperPartConfiguration_(upperPartIndex[4])= rarm_q(0) ;       // RARM_JOINT0
    upperPartConfiguration_(upperPartIndex[5])= rarm_q(1) ;       // RARM_JOINT1
    upperPartConfiguration_(upperPartIndex[6])= rarm_q(2) ;       // RARM_JOINT2
    upperPartConfiguration_(upperPartIndex[7])= rarm_q(3) ;       // RARM_JOINT3
    upperPartConfiguration_(upperPartIndex[8])= rarm_q(4) ;       // RARM_JOINT4
    upperPartConfiguration_(upperPartIndex[9])= rarm_q(5) ;       // RARM_JOINT5
    upperPartConfiguration_(upperPartIndex[10])= 0.174532925 ;    // RARM_JOINT6
    upperPartConfiguration_(upperPartIndex[11])= larm_q(0) ;      // LARM_JOINT0
    upperPartConfiguration_(upperPartIndex[12])= larm_q(1) ;      // LARM_JOINT1
    upperPartConfiguration_(upperPartIndex[13])= larm_q(2) ;      // LARM_JOINT2
    upperPartConfiguration_(upperPartIndex[14])= larm_q(3) ;      // LARM_JOINT3
    upperPartConfiguration_(upperPartIndex[15])= larm_q(4) ;      // LARM_JOINT4
    upperPartConfiguration_(upperPartIndex[16])= larm_q(5) ;      // LARM_JOINT5
    upperPartConfiguration_(upperPartIndex[17])= 0.174532925 ;    // LARM_JOINT6

    for (unsigned int i = 0 ; i < upperPartIndex.size() ; ++i)
    {
      upperPartVelocity_(upperPartIndex[i]) = (upperPartConfiguration_(upperPartIndex[i]) -
                                               previousUpperPartConfiguration_(upperPartIndex[i]))/samplingPeriod;
      upperPartAcceleration_(upperPartIndex[i]) = (upperPartVelocity_(upperPartIndex[i]) -
                                                   previousUpperPartVelocity_(upperPartIndex[i]))/samplingPeriod;
    }
    previousUpperPartConfiguration_ = upperPartConfiguration_ ;
    previousUpperPartVelocity_ = upperPartVelocity_ ;
  }

  for ( unsigned int i = 18 ; i < 36 ; ++i )
  {
    configuration(i)= upperPartConfiguration_(i);
    velocity(i) = upperPartVelocity_(i) ;
    acceleration(i) = upperPartAcceleration_(i) ;
  }


  /// \brief Create file .hip .pos .zmp
  /// --------------------
  ofstream aof;
  string aFileName;

  if ( it == 0 ){
    aFileName = "/tmp/goingDownWithWeightDFtest";
    aFileName+=".pos";
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
  }
  ///----
  aFileName = "/tmp/goingDownWithWeightDFtest";
    aFileName+=".pos";
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  aof << it * 0.005 << " "  ; // 1
  for(unsigned int i = 6 ; i < configuration.size() ; i++){
    aof << configuration(i) << " "  ; // 2
  }
  for(unsigned int i = 0 ; i < 10 ; i++){
    aof << 0.0 << " "  ;
  }
  aof  << endl ;
  aof.close();

  ++it;

  return;
}

void DynamicFilter::stage0INstage1()
{
  comAndFootRealization_->SetPreviousConfigurationStage1(comAndFootRealization_->GetPreviousConfigurationStage0());
  comAndFootRealization_->SetPreviousVelocityStage1(comAndFootRealization_->GetPreviousVelocityStage0());
  return ;
}

void DynamicFilter::ComputeZMPMB(
    double samplingPeriod,
    const COMState & inputCoMState,
    const FootAbsolutePosition & inputLeftFoot,
    const FootAbsolutePosition & inputRightFoot,
    vector<double> & ZMPMB,
    int stage,
    int iteration)
{
  InverseKinematics( inputCoMState, inputLeftFoot, inputRightFoot,
      ZMPMBConfiguration_, ZMPMBVelocity_, ZMPMBAcceleration_,
      samplingPeriod, stage, iteration) ;

  // Copy the angular trajectory data from "Boost" to "Eigen"
  for(unsigned int j = 0 ; j < ZMPMBConfiguration_.size() ; j++ )
  {
    q_(j,0) = ZMPMBConfiguration_(j) ;
    dq_(j,0) = ZMPMBVelocity_(j) ;
    ddq_(j,0) = ZMPMBAcceleration_(j) ;
  }

  //computeWaist( inputLeftFoot );
  RootNode & node_waist = boost::fusion::at_c<Robot_Model::BODY>(robot_.nodes);
  // Apply the RNEA on the robot model
  clock_.StartTiming();
  metapod::rnea< Robot_Model, true >::run(robot_, q_, dq_, ddq_);
  clock_.StopTiming();

  clock_.IncIteration();

  m_force = node_waist.body.iX0.applyInv(node_waist.joint.f);

  ZMPMB.resize(2);
  ZMPMB[0] = - m_force.n()[1] / m_force.f()[2] ;
  ZMPMB[1] =   m_force.n()[0] / m_force.f()[2] ;

  return ;
}

void DynamicFilter::ComputeZMPMB(
    MAL_VECTOR_TYPE(double) & configuration,
    MAL_VECTOR_TYPE(double) & velocity,
    MAL_VECTOR_TYPE(double) & acceleration,
    vector<double> & ZMPMB)
{
  // Copy the angular trajectory data from "Boost" to "Eigen"
  for(unsigned int j = 0 ; j < configuration.size() ; j++ )
  {
    q_(j,0) = configuration(j) ;
    dq_(j,0) = velocity(j) ;
    ddq_(j,0) = acceleration(j) ;
  }

  // Apply the RNEA on the robot model
  metapod::rnea< Robot_Model, true >::run(robot_, q_, dq_, ddq_);

  RootNode & node = boost::fusion::at_c<Robot_Model::BODY>(robot_.nodes);
  m_force = node.body.iX0.applyInv (node.joint.f);

  ZMPMB.resize(2);
  ZMPMB[0] = - m_force.n()[1] / m_force.f()[2] ;
  ZMPMB[1] =   m_force.n()[0] / m_force.f()[2] ;

  return ;
}

int DynamicFilter::OptimalControl(
    deque<ZMPPosition> & inputdeltaZMP_deq,
    deque<COMState> & outputDeltaCOMTraj_deq_)
{
  /// --------------------
  ofstream aof;
  string aFileName;
  ostringstream oss(std::ostringstream::ate);
  /// --------------------
  oss.str("ZMPDiscretisationErr.dat");
  aFileName = oss.str();
  aof.open(aFileName.c_str(),ofstream::out);
  aof.close();
  ///----
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);

  if(!PC_->IsCoherent())
    PC_->ComputeOptimalWeights(OptimalControllerSolver::MODE_WITH_INITIALPOS);

  outputDeltaCOMTraj_deq_.resize(NCtrl_);
  // calcul of the preview control along the "deltaZMP_deq_"
  for (unsigned i = 0 ; i < NCtrl_ ; i++ )
  {
    aof << i*controlPeriod_ << " "               // 1
        << sxzmp_ << " "                         // 2
        << syzmp_ << " "                         // 3
        << endl ;
    PC_->OneIterationOfPreview(deltax_,deltay_,
                               sxzmp_,syzmp_,
                               inputdeltaZMP_deq,i,
                               deltaZMPx_, deltaZMPy_, false);
    for(int j=0;j<3;j++)
    {
      outputDeltaCOMTraj_deq_[i].x[j] = deltax_(j,0);
      outputDeltaCOMTraj_deq_[i].y[j] = deltay_(j,0);
    }
  }

  aof.close();

  for (unsigned int i = 0 ; i < NCtrl_ ; i++)
  {
    for(int j=0;j<3;j++)
    {
      if ( outputDeltaCOMTraj_deq_[i].x[j] == outputDeltaCOMTraj_deq_[i].x[j] ||
           outputDeltaCOMTraj_deq_[i].y[j] == outputDeltaCOMTraj_deq_[i].y[j] )
      {}
      else{
        cout << "kajita2003 preview control diverged\n" ;
        return -1 ;
      }
    }
  }
  return 0 ;
}


void DynamicFilter::computeWaist(const FootAbsolutePosition & inputLeftFoot)
{
  Eigen::Matrix< LocalFloatType, 6, 1 > waist_speed, waist_acc ;
  Eigen::Matrix< LocalFloatType, 3, 1 > waist_theta ;
  // compute the speed and acceleration of the waist in the world frame
  if (PreviousSupportFoot_)
  {
    Jacobian_LF::run(robot_, prev_q_, Vector3dTpl<LocalFloatType>::Type(0,0,0), jacobian_lf_);
    waist_speed = jacobian_lf_ * prev_dq_ ;
    waist_acc = jacobian_lf_ * prev_ddq_ /* + d_jacobian_lf_ * prev_dq_*/ ;
  }else
  {
    Jacobian_RF::run(robot_, prev_q_, Vector3dTpl<LocalFloatType>::Type(0,0,0), jacobian_rf_);
    waist_speed = jacobian_rf_ * prev_dq_ ;
    waist_acc = jacobian_rf_ * prev_ddq_ /*+ d_jacobian_rf_ * prev_dq_*/ ;
  }
  for (unsigned int i = 0 ; i < 6 ; ++i)
  {
    dq_(i,0)   = waist_speed(i,0);
    ddq_(i,0)  = waist_acc(i,0);
  }
  // compute the position of the waist in the world frame
  RootNode & node_waist = boost::fusion::at_c<Robot_Model::BODY>(robot_.nodes);
  waist_theta(0,0) = prev_q_(3,0) ;
  waist_theta(1,0) = prev_dq_(4,0) ;
  waist_theta(2,0) = prev_ddq_(5,0) ;
  q_(0,0) = node_waist.body.iX0.inverse().r()(0,0) ;
  q_(1,0) = node_waist.body.iX0.inverse().r()(1,0) ;
  q_(2,0) = node_waist.body.iX0.inverse().r()(2,0) ;
  q_(3,0) = waist_theta(0,0) ;
  q_(4,0) = waist_theta(1,0) ;
  q_(5,0) = waist_theta(2,0) ;

  if (inputLeftFoot.stepType<0)
  {
    PreviousSupportFoot_ = true ; // left foot is supporting
  }
  else
  {
    PreviousSupportFoot_ = false ; // right foot is supporting
  }
  prev_q_ = q_ ;
  prev_dq_ = dq_ ;
  prev_ddq_ = ddq_ ;

  //  Robot_Model::confVector q, dq, ddq;
  //  for(unsigned int j = 0 ; j < 6 ; j++ )
  //  {
  //    q(j,0) = 0 ;
  //    dq(j,0) = 0 ;
  //    ddq(j,0) = 0 ;
  //  }
  //  for(unsigned int j = 6 ; j < ZMPMBConfiguration_.size() ; j++ )
  //  {
  //    q(j,0) = ZMPMBConfiguration_(j) ;
  //    dq(j,0) = ZMPMBVelocity_(j) ;
  //    ddq(j,0) = ZMPMBAcceleration_(j) ;
  //  }
  //
  //  metapod::rnea< Robot_Model, true >::run(robot_2, q, dq, ddq);
  //  LankleNode & node_lankle = boost::fusion::at_c<Robot_Model::l_ankle>(robot_2.nodes);
  //  RankleNode & node_rankle = boost::fusion::at_c<Robot_Model::r_ankle>(robot_2.nodes);
  //
  //  CWx = node_waist.body.iX0.r()(0,0) - inputCoMState.x[0] ;
  //  CWy = node_waist.body.iX0.r()(1,0) - inputCoMState.y[0] ;
  //
  //  // Debug Purpose
  //  // -------------
  //  ofstream aof;
  //  string aFileName;
  //  ostringstream oss(std::ostringstream::ate);
  //  static int it = 0;
  //
  //  // --------------------
  //  oss.str("DynamicFilterMetapodAccWaistSupportFoot.dat");
  //  aFileName = oss.str();
  //  if(it == 0)
  //  {
  //    aof.open(aFileName.c_str(),ofstream::out);
  //    aof.close();
  //  }
  //  ///----
  //  aof.open(aFileName.c_str(),ofstream::app);
  //  aof.precision(8);
  //  aof.setf(ios::scientific, ios::floatfield);
  //  aof << filterprecision( it*samplingPeriod) << " " ;     // 1
  //
  //  if (inputLeftFoot.stepType < 0)
  //  {
  //    aof << filterprecision( node_lankle.body.ai.v()(0,0) ) << " "  // 2
  //        << filterprecision( node_lankle.body.ai.v()(1,0) ) << " "  // 3
  //        << filterprecision( node_lankle.body.ai.v()(2,0) ) << " "  // 4
  //        << filterprecision( node_lankle.body.ai.w()(0,0) ) << " "  // 5
  //        << filterprecision( node_lankle.body.ai.w()(1,0) ) << " "  // 6
  //        << filterprecision( node_lankle.body.ai.w()(2,0) ) << " "; // 7
  //  }else
  //  {
  //    aof << filterprecision( node_rankle.body.ai.v()(0,0) ) << " "  // 2
  //        << filterprecision( node_rankle.body.ai.v()(1,0) ) << " "  // 3
  //        << filterprecision( node_rankle.body.ai.v()(2,0) ) << " "  // 4
  //        << filterprecision( node_rankle.body.ai.w()(0,0) ) << " "  // 5
  //        << filterprecision( node_rankle.body.ai.w()(1,0) ) << " "  // 6
  //        << filterprecision( node_rankle.body.ai.w()(2,0) ) << " " ;// 7
  //  }
  //
  //  aof << filterprecision( inputCoMState.x[2] ) << " "           // 8
  //      << filterprecision( inputCoMState.y[2] ) << " "           // 9
  //      << filterprecision( inputCoMState.z[2] ) << " "           // 10
  //      << filterprecision( inputCoMState.roll[2] ) << " "        // 11
  //      << filterprecision( inputCoMState.pitch[2] ) << " "       // 12
  //      << filterprecision( inputCoMState.yaw[2] ) << " "       ; // 13
  //
  //  if (inputLeftFoot.stepType < 0)
  //  {
  //    aof << filterprecision( node_lankle.body.vi.v()(0,0) ) << " "  // 14
  //        << filterprecision( node_lankle.body.vi.v()(1,0) ) << " "  // 15
  //        << filterprecision( node_lankle.body.vi.v()(2,0) ) << " "  // 16
  //        << filterprecision( node_lankle.body.vi.w()(0,0) ) << " "  // 17
  //        << filterprecision( node_lankle.body.vi.w()(1,0) ) << " "  // 18
  //        << filterprecision( node_lankle.body.vi.w()(2,0) ) << " " ;// 19
  //  }else
  //  {
  //    aof << filterprecision( node_rankle.body.vi.v()(0,0) ) << " "  // 14
  //        << filterprecision( node_rankle.body.vi.v()(1,0) ) << " "  // 15
  //        << filterprecision( node_rankle.body.vi.v()(2,0) ) << " "  // 16
  //        << filterprecision( node_rankle.body.vi.w()(0,0) ) << " "  // 17
  //        << filterprecision( node_rankle.body.vi.w()(1,0) ) << " "  // 18
  //        << filterprecision( node_rankle.body.vi.w()(2,0) ) << " "; // 19
  //  }
  //
  //  aof << filterprecision( inputCoMState.x[1] ) << " "           // 20
  //      << filterprecision( inputCoMState.y[1] ) << " "           // 21
  //      << filterprecision( inputCoMState.z[1] ) << " "           // 22
  //      << filterprecision( inputCoMState.roll[1] ) << " "        // 23
  //      << filterprecision( inputCoMState.pitch[1] ) << " "       // 24
  //      << filterprecision( inputCoMState.yaw[1] ) << " "     ;   // 25
  //
  //  aof << filterprecision( node_waist.joint.vj.v()(0,0) ) << " " // 26
  //      << filterprecision( node_waist.joint.vj.v()(1,0) ) << " "  // 27
  //      << filterprecision( node_waist.joint.vj.v()(2,0) ) << " "  // 28
  //      << filterprecision( node_waist.joint.vj.w()(0,0) ) << " "  // 29
  //      << filterprecision( node_waist.joint.vj.w()(1,0) ) << " "  // 30
  //      << filterprecision( node_waist.joint.vj.w()(2,0) ) << " "; // 31
  //
  //  aof << filterprecision( inputCoMState.x[0] ) << " "           // 32
  //      << filterprecision( inputCoMState.y[0] ) << " "           // 33
  //      << filterprecision( inputCoMState.z[0] ) << " "           // 34
  //      << filterprecision( inputCoMState.roll[0] ) << " "        // 35
  //      << filterprecision( inputCoMState.pitch[0] ) << " "       // 36
  //      << filterprecision( inputCoMState.yaw[0] ) << " "     ;   // 37
  //
  //  aof << filterprecision( node_waist.body.iX0.r()(0,0) ) << " " // 38
  //      << filterprecision( node_waist.body.iX0.r()(1,0) ) << " " // 39
  //      << filterprecision( node_waist.body.iX0.r()(2,0) ) << " ";// 40
  //
  //
  //  for(unsigned int j = 0 ; j < ZMPMBConfiguration_.size() ; j++ )//41
  //    aof << filterprecision( ZMPMBConfiguration_(j) ) << " " ;
  //  for(unsigned int j = 0 ; j < ZMPMBVelocity_.size() ; j++ )    //77
  //    aof << filterprecision( ZMPMBVelocity_(j) ) << " " ;
  //  for(unsigned int j = 0 ; j < ZMPMBAcceleration_.size() ; j++ )//113
  //    aof << filterprecision( ZMPMBAcceleration_(j) ) << " " ;
  //
  //
  //  aof << endl ;
  //  aof.close();
  //  ++it;

  return ;
}

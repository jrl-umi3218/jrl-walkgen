#include "DynamicFilter.hh"
#include <metapod/algos/rnea.hh>

using namespace std;
using namespace PatternGeneratorJRL;
using namespace metapod;

DynamicFilter::DynamicFilter(
    SimplePluginManager *SPM,
    CjrlHumanoidDynamicRobot *aHS): stage0_(0) , stage1_(1)
{
  controlPeriod_ = 0.0 ;
  interpolationPeriod_ = 0.0 ;
  previewWindowSize_ = 0.0 ;
  PG_T_ = 0.0 ;
  NbI_ = 0.0 ;
  NCtrl_ = 0.0;
  PG_N_ = 0.0 ;

  comAndFootRealization_ = new ComAndFootRealizationByGeometry((PatternGeneratorInterfacePrivate*) SPM );
  comAndFootRealization_->setHumanoidDynamicRobot(aHS);
  comAndFootRealization_->SetHeightOfTheCoM(CoMHeight_);
  comAndFootRealization_->setSamplingPeriod(interpolationPeriod_);
  comAndFootRealization_->Initialization();

  PC_ = new PreviewControl(
        SPM,OptimalControllerSolver::MODE_WITH_INITIALPOS,false);
  CoMHeight_ = 0.0 ;

  deltaZMP_deq_.clear();
  ZMPMB_vec_.clear();

  MAL_VECTOR_RESIZE(aCoMState_,6);
  MAL_VECTOR_RESIZE(aCoMSpeed_,6);
  MAL_VECTOR_RESIZE(aCoMAcc_,6);
  MAL_VECTOR_RESIZE(aLeftFootPosition_,5);
  MAL_VECTOR_RESIZE(aRightFootPosition_,5);
  MAL_MATRIX_RESIZE(deltax_,3,1);
  MAL_MATRIX_RESIZE(deltay_,3,1);

  comAndFootRealization_->SetPreviousConfigurationStage0(
        aHS->currentConfiguration());
  comAndFootRealization_->SetPreviousVelocityStage0(
        aHS->currentVelocity());

  Once_ = true ;
  DInitX_ = 0.0 ;
  DInitY_ = 0.0 ;

  jacobian_lf_ = Jac_LF::Jacobian::Zero();
  jacobian_rf_ = Jac_RF::Jacobian::Zero();

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

void DynamicFilter::setRobotUpperPart(const MAL_VECTOR_TYPE(double) & configuration,
                                      const MAL_VECTOR_TYPE(double) & velocity,
                                      const MAL_VECTOR_TYPE(double) & acceleration)
{
  for ( unsigned int i = 0 ; i < upperPartIndex.size() ; ++i )
    {
      upperPartConfiguration_(upperPartIndex[i])  = configuration(upperPartIndex[i]);
      upperPartVelocity_(upperPartIndex[i])       = velocity(upperPartIndex[i]);
      upperPartAcceleration_(upperPartIndex[i])   = acceleration(upperPartIndex[i]);
    }
  return ;
}

/// \brief Initialise all objects, to be called just after the constructor
void DynamicFilter::init(
    double controlPeriod,
    double interpolationPeriod,
    double PG_T,
    double previewWindowSize,
    double CoMHeight,
    FootAbsolutePosition inputLeftFoot,
    COMState inputCoMState)
{
  controlPeriod_ = controlPeriod ;
  interpolationPeriod_ = interpolationPeriod ;
  PG_T_ = PG_T ;
  previewWindowSize_ = previewWindowSize ;

  if (interpolationPeriod_>PG_T)
    {NbI_=1;}
  else
    {NbI_ = (int)(PG_T/interpolationPeriod_);}

  NCtrl_ = (int)(PG_T_/controlPeriod_) ;
  PG_N_ = (int)( (previewWindowSize_+PG_T_/controlPeriod*interpolationPeriod)/PG_T_ ) ;

  CoMHeight_ = CoMHeight ;
  PC_->SetPreviewControlTime (previewWindowSize_);
  PC_->SetSamplingPeriod (interpolationPeriod_);
  PC_->SetHeightOfCoM(CoMHeight_);
  PC_->ComputeOptimalWeights(OptimalControllerSolver::MODE_WITH_INITIALPOS);

  upperPartConfiguration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentConfiguration() ;
  previousUpperPartConfiguration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentConfiguration() ;
  upperPartVelocity_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentVelocity() ;
  previousUpperPartVelocity_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentVelocity() ;
  upperPartAcceleration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentAcceleration() ;

  ZMPMBConfiguration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentConfiguration() ;
  ZMPMBVelocity_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentVelocity() ;
  ZMPMBAcceleration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentAcceleration() ;

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

  deltaZMP_deq_.resize( PG_N_*NbI_);
  ZMPMB_vec_.resize( PG_N_*NbI_, vector<double>(2));

  comAndFootRealization_->setSamplingPeriod(interpolationPeriod_);
  comAndFootRealization_->Initialization();
  comAndFootRealization_->SetPreviousConfigurationStage0(ZMPMBConfiguration_);
  comAndFootRealization_->SetPreviousVelocityStage0(ZMPMBVelocity_);
  comAndFootRealization_->SetPreviousVelocityStage1(ZMPMBVelocity_);
  comAndFootRealization_->SetPreviousConfigurationStage1(ZMPMBConfiguration_);
  comAndFootRealization_->SetPreviousVelocityStage2(ZMPMBVelocity_);
  comAndFootRealization_->SetPreviousConfigurationStage2(ZMPMBConfiguration_);

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
    const double currentTime,
    const deque<COMState> &inputCOMTraj_deq_,
    const deque<ZMPPosition> &inputZMPTraj_deq_,
    const deque<FootAbsolutePosition> &inputLeftFootTraj_deq_,
    const deque<FootAbsolutePosition> &inputRightFootTraj_deq_,
    const vector< MAL_VECTOR_TYPE(double) > & UpperPart_q,
    const vector< MAL_VECTOR_TYPE(double) > & UpperPart_dq,
    const vector< MAL_VECTOR_TYPE(double) > & UpperPart_ddq,
    deque<COMState> & outputDeltaCOMTraj_deq_)
{
  unsigned int N = inputCOMTraj_deq_.size() ;
  ZMPMB_vec_.resize(N) ;
  deltaZMP_deq_.resize(N);

  setRobotUpperPart(UpperPart_q[0],UpperPart_dq[0],UpperPart_ddq[0]);

  for(unsigned int i = 0 ; i < N ; ++i )
    {
      ComputeZMPMB(interpolationPeriod_,inputCOMTraj_deq_[i],inputLeftFootTraj_deq_[i],
                   inputRightFootTraj_deq_[i], ZMPMB_vec_[i] , stage0_ , i);
    }
  for (unsigned int i = 0 ; i < N ; ++i)
    {
      deltaZMP_deq_[i].px = inputZMPTraj_deq_[i].px - ZMPMB_vec_[i][0] ;
      deltaZMP_deq_[i].py = inputZMPTraj_deq_[i].py - ZMPMB_vec_[i][1] ;
      deltaZMP_deq_[i].pz = 0.0 ;
      deltaZMP_deq_[i].theta = 0.0 ;
      deltaZMP_deq_[i].time = currentTime + i * interpolationPeriod_ ;
      deltaZMP_deq_[i].stepType = inputZMPTraj_deq_[i].stepType ;
    }
  OptimalControl(deltaZMP_deq_,outputDeltaCOMTraj_deq_) ;

  return 0;
}

int DynamicFilter::OnLinefilter(
    const double currentTime,
    const deque<COMState> & ctrlCoMState,
    const deque<FootAbsolutePosition> & ctrlLeftFoot,
    const deque<FootAbsolutePosition> & ctrlRightFoot,
    const deque<COMState> & inputCOMTraj_deq_,
    const deque<ZMPPosition> inputZMPTraj_deq_,
    const deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
    const deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
    deque<COMState> & outputDeltaCOMTraj_deq_)
{
  static int currentIteration = 0 ;
  vector< MAL_VECTOR_TYPE(double) > q_vec ;
  vector< MAL_VECTOR_TYPE(double) > dq_vec ;
  vector< MAL_VECTOR_TYPE(double) > ddq_vec ;
  for(unsigned int i = 0 ; i <= NCtrl_; ++i)
    {
      InverseKinematics( ctrlCoMState[i], ctrlLeftFoot[i],
                         ctrlRightFoot[i], ZMPMBConfiguration_, ZMPMBVelocity_,
                         ZMPMBAcceleration_, controlPeriod_, stage0_, currentIteration+i) ;
    }

  unsigned int N = PG_N_ * NbI_ ;
  ZMPMB_vec_.resize( N , vector<double>(2,0.0));
  for(unsigned int i = 0 ; i < N ; ++i)
  {
      ComputeZMPMB(interpolationPeriod_,
                   inputCOMTraj_deq_[i],
                   inputLeftFootTraj_deq_[i],
                   inputRightFootTraj_deq_[i],
                   ZMPMB_vec_[i],
                   stage1_,
                   currentIteration + i);
      q_vec.push_back(ZMPMBConfiguration_);
      dq_vec.push_back(ZMPMBVelocity_);
      ddq_vec.push_back(ZMPMBAcceleration_);
  }
  stage0INstage1();

  deltaZMP_deq_.resize(N);
  for (unsigned int i = 0 ; i < N ; ++i)
    {
      deltaZMP_deq_[i].px = inputZMPTraj_deq_[i].px - ZMPMB_vec_[i][0] ;
      deltaZMP_deq_[i].py = inputZMPTraj_deq_[i].py - ZMPMB_vec_[i][1] ;
      deltaZMP_deq_[i].pz = 0.0 ;
      deltaZMP_deq_[i].theta = 0.0 ;
      deltaZMP_deq_[i].time = currentTime + i * interpolationPeriod_ ;
      deltaZMP_deq_[i].stepType = inputZMPTraj_deq_[i].stepType ;
    }
  OptimalControl(deltaZMP_deq_,outputDeltaCOMTraj_deq_) ;

  currentIteration += NbI_ ;
//#############################################################################
  deque<COMState> CoM_tmp = ctrlCoMState ;
  for (unsigned int i = 0 ; i < NCtrl_ ; ++i)
  {
    for(int j=0;j<3;j++)
    {
      CoM_tmp[i].x[j] += outputDeltaCOMTraj_deq_[i].x[j] ;
      CoM_tmp[i].y[j] += outputDeltaCOMTraj_deq_[i].y[j] ;
    }
  }

  int stage2 = 2 ;
  vector< vector<double> > zmpmb_corr (NCtrl_,vector<double>(2,0.0));
  for(unsigned int i = 0 ; i < NCtrl_ ; ++i)
  {
      ComputeZMPMB(controlPeriod_,
                   CoM_tmp[i],
                   ctrlLeftFoot[i],
                   ctrlRightFoot[i],
                   zmpmb_corr[i],
                   stage2,
                   currentIteration + i);
  }

  ofstream aof;
  string aFileName;
  static int iteration_zmp = 0 ;
  ostringstream oss(std::ostringstream::ate);
  oss.str("zmpmb_herdt.txt");
  aFileName = oss.str();
  if ( iteration_zmp == 0 )
  {
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
  }

  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (unsigned int i = 0 ; i < NbI_ ; ++i)
  {
    aof << inputZMPTraj_deq_[i].px << " " ;           // 1
    aof << inputZMPTraj_deq_[i].py << " " ;           // 2

    aof << ZMPMB_vec_[i][0] << " " ;                  // 3
    aof << ZMPMB_vec_[i][1] << " " ;                  // 4

    aof << inputCOMTraj_deq_[i].x[0] << " " ;         // 5
    aof << inputCOMTraj_deq_[i].x[1] << " " ;         // 6
    aof << inputCOMTraj_deq_[i].x[2] << " " ;         // 7

    aof << inputLeftFootTraj_deq_[i].x << " " ;       // 8
    aof << inputLeftFootTraj_deq_[i].dx << " " ;      // 9
    aof << inputLeftFootTraj_deq_[i].ddx << " " ;     // 10

    aof << inputRightFootTraj_deq_[i].x << " " ;      // 11
    aof << inputRightFootTraj_deq_[i].dx << " " ;     // 12
    aof << inputRightFootTraj_deq_[i].ddx << " " ;    // 13

    aof << inputCOMTraj_deq_[i].y[0] << " " ;         // 14
    aof << inputCOMTraj_deq_[i].y[1] << " " ;         // 15
    aof << inputCOMTraj_deq_[i].y[2] << " " ;         // 16

    aof << inputLeftFootTraj_deq_[i].y << " " ;       // 17
    aof << inputLeftFootTraj_deq_[i].dy << " " ;      // 18
    aof << inputLeftFootTraj_deq_[i].ddy << " " ;     // 19

    aof << inputRightFootTraj_deq_[i].y << " " ;      // 20
    aof << inputRightFootTraj_deq_[i].dy << " " ;     // 21
    aof << inputRightFootTraj_deq_[i].ddy << " " ;    // 22

    aof << inputCOMTraj_deq_[i].yaw[0] << " " ;       // 23
    aof << inputCOMTraj_deq_[i].yaw[1] << " " ;       // 24
    aof << inputCOMTraj_deq_[i].yaw[2] << " " ;       // 25

    aof << inputLeftFootTraj_deq_[i].theta << " " ;   // 26
    aof << inputLeftFootTraj_deq_[i].dtheta << " " ;  // 27
    aof << inputLeftFootTraj_deq_[i].ddtheta << " " ; // 28

    aof << inputRightFootTraj_deq_[i].theta << " " ;  // 29
    aof << inputRightFootTraj_deq_[i].dtheta << " " ; // 30
    aof << inputRightFootTraj_deq_[i].ddtheta << " " ;// 31

    aof << inputCOMTraj_deq_[i].z[0] << " " ;         // 32
    aof << inputCOMTraj_deq_[i].z[1] << " " ;         // 33
    aof << inputCOMTraj_deq_[i].z[2] << " " ;         // 34

    aof << inputLeftFootTraj_deq_[i].z << " " ;       // 35
    aof << inputLeftFootTraj_deq_[i].dz << " " ;      // 36
    aof << inputLeftFootTraj_deq_[i].ddz << " " ;     // 37

    aof << inputRightFootTraj_deq_[i].z << " " ;      // 38
    aof << inputRightFootTraj_deq_[i].dz << " " ;     // 39
    aof << inputRightFootTraj_deq_[i].ddz << " " ;    // 40

    for(unsigned int j = 0 ; j < q_vec[0].size() ; ++j) // 41 -- 47
      {
        aof << q_vec[i][j] << " " ;
      }
    for(unsigned int j = 0 ; j < dq_vec[0].size() ; ++j) // 77 -- 83
      {
        aof << dq_vec[i][j] << " " ;
      }
    for(unsigned int j = 0 ; j < ddq_vec[0].size() ; ++j) // 113 -- 119
      {
        aof << ddq_vec[i][j] << " " ;
      }
    aof << endl ;
  }
  aof.close();

  aFileName = "zmpmb_corr_herdt.txt" ;
  if ( iteration_zmp == 0 )
  {
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
  }
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (unsigned int i = 0 ; i < NCtrl_ ; ++i)
  {
    aof << zmpmb_corr[i][0] << " " ;
    aof << zmpmb_corr[i][1] << " " ;
    aof << endl ;
  }
  aof.close();

  iteration_zmp++;

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

  // lower body
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

  // upper body
  if (walkingHeuristic_)
    {
      LankleNode & node_lankle = boost::fusion::at_c<Robot_Model::l_ankle>(robot_.nodes);
      RankleNode & node_rankle = boost::fusion::at_c<Robot_Model::r_ankle>(robot_.nodes);

      RootNode & node_waist = boost::fusion::at_c<Robot_Model::BODY>(robot_.nodes);

      Spatial::TransformT<LocalFloatType,Spatial::RotationMatrixTpl<LocalFloatType> > waistXlf ;
      Spatial::TransformT<LocalFloatType,Spatial::RotationMatrixTpl<LocalFloatType> > waistXrf ;
      waistXlf = node_waist.body.iX0 * node_lankle.body.iX0.inverse() ;
      waistXrf = node_waist.body.iX0 * node_rankle.body.iX0.inverse() ;

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

      HDR->getSpecializedInverseKinematics( *left_shoulder ,*(HDR->leftWrist()), identity, leftHandPose, larm_q );
      HDR->getSpecializedInverseKinematics( *right_shoulder ,*(HDR->rightWrist()), identity, rightHandPose, rarm_q );

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

  return;
}

void DynamicFilter::InverseDynamics(
    MAL_VECTOR_TYPE(double)& configuration,
    MAL_VECTOR_TYPE(double)& velocity,
    MAL_VECTOR_TYPE(double)& acceleration
    )
{
  // Copy the angular trajectory data from "Boost" to "Eigen"
  for(unsigned int j = 0 ; j < ZMPMBConfiguration_.size() ; j++ )
    {
      q_(j,0)   = configuration(j) ;
      dq_(j,0)  = velocity(j) ;
      ddq_(j,0) = acceleration(j) ;
    }

  //computeWaist( inputLeftFoot , q_ , dq_ , ddq_ );

  // Apply the RNEA on the robot model
  metapod::rnea< Robot_Model, true >::run(robot_, q_, dq_, ddq_);

  return ;
}

void DynamicFilter::ExtractZMP(vector<double> & ZMPMB)
{
  RootNode & node_waist = boost::fusion::at_c<Robot_Model::BODY>(robot_.nodes);

  // extract the CoM momentum and forces
  m_force = node_waist.body.iX0.applyInv(node_waist.joint.f);
  metapod::Vector3dTpl< LocalFloatType >::Type CoM_Waist_vec (node_waist.body.iX0.r() - com ()) ;
  metapod::Vector3dTpl< LocalFloatType >::Type CoM_torques (0.0,0.0,0.0);
  CoM_torques = m_force.n() + metapod::Skew<LocalFloatType>(CoM_Waist_vec) * m_force.f() ;

  // compute the Multibody ZMP
  ZMPMB.resize(2);
  ZMPMB[0] = - CoM_torques[1] / m_force.f()[2] ;
  ZMPMB[1] =   CoM_torques[0] / m_force.f()[2] ;
  return ;
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
    unsigned int stage,
    unsigned int iteration)
{
  InverseKinematics( inputCoMState, inputLeftFoot, inputRightFoot,
                     ZMPMBConfiguration_, ZMPMBVelocity_, ZMPMBAcceleration_,
                     samplingPeriod, stage, iteration) ;

  InverseDynamics(ZMPMBConfiguration_, ZMPMBVelocity_, ZMPMBAcceleration_);

  ExtractZMP(ZMPMB);

  return ;
}

int DynamicFilter::OptimalControl(
    deque<ZMPPosition> & inputdeltaZMP_deq,
    deque<COMState> & outputDeltaCOMTraj_deq_)
{
  if(!PC_->IsCoherent())
    PC_->ComputeOptimalWeights(OptimalControllerSolver::MODE_WITH_INITIALPOS);

  outputDeltaCOMTraj_deq_.resize(NCtrl_);
  // calcul of the preview control along the "deltaZMP_deq_"
  for (unsigned i = 0 ; i < NCtrl_ ; i++ )
    {
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

  // test to verify if the Kajita PC diverged
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

// TODO finish the implementation of a better waist tracking
void DynamicFilter::computeWaist(const FootAbsolutePosition & inputLeftFoot)
{
  Eigen::Matrix< LocalFloatType, 6, 1 > waist_speed, waist_acc ;
  Eigen::Matrix< LocalFloatType, 3, 1 > waist_theta ;
  // compute the speed and acceleration of the waist in the world frame
  if (PreviousSupportFoot_)
    {
      Jac_LF::run(robot_, prev_q_, Vector3dTpl<LocalFloatType>::Type(0,0,0), jacobian_lf_);
      waist_speed = jacobian_lf_ * prev_dq_ ;
      waist_acc = jacobian_lf_ * prev_ddq_ /* + d_jacobian_lf_ * prev_dq_*/ ;
    }else
    {
      Jac_RF::run(robot_, prev_q_, Vector3dTpl<LocalFloatType>::Type(0,0,0), jacobian_rf_);
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

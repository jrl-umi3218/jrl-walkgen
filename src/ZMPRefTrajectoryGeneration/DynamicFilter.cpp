#include "DynamicFilter.hh"
#include "Debug.hh"
#include <iomanip>
using namespace std;
using namespace PatternGeneratorJRL;
//using namespace metapod;

DynamicFilter::DynamicFilter(
                             SimplePluginManager *SPM,
                             PinocchioRobot *aPR):
  SimplePlugin(SPM),
  stage0_(0), stage1_(1),
  MODE_PC_(OptimalControllerSolver::MODE_WITH_INITIALPOS)
{
  controlPeriod_       = 0.0 ;
  interpolationPeriod_ = 0.0 ;
  controlWindowSize_   = 0.0 ;
  previewWindowSize_   = 0.0 ;
  kajitaPCwindowSize_  = 0.0 ;
  CoMHeight_           = 0.0 ;

  PR_ = aPR ;

  comAndFootRealization_ = new
    ComAndFootRealizationByGeometry((PatternGeneratorInterfacePrivate*) SPM );
  comAndFootRealization_->setPinocchioRobot(PR_);
  comAndFootRealization_->SetHeightOfTheCoM(CoMHeight_);
  comAndFootRealization_->ShiftFoot(true);
  comAndFootRealization_->setSamplingPeriod(interpolationPeriod_);
  comAndFootRealization_->Initialization();

  PC_ = new PreviewControl(SPM,MODE_PC_,false);

  deltaZMP_deq_.clear();
  ZMPMB_vec_.clear();

  aCoMState_.resize(6);
  aCoMSpeed_.resize(6);
  aCoMAcc_.resize(6);
  aLeftFootPosition_.resize(5);
  aRightFootPosition_.resize(5);
  deltax_.resize(3,1);
  deltay_.resize(3,1);

  comAndFootRealization_->
    SetPreviousConfigurationStage0(PR_->currentRPYConfiguration());
  comAndFootRealization_->
    SetPreviousVelocityStage0(PR_->currentRPYVelocity());

  sxzmp_.clear();
  syzmp_.clear();
  deltaZMPx_.clear();
  deltaZMPy_.clear();

  llegIdxq_.clear();
  rlegIdxq_.clear();

  larmIdxq_.clear();
  rarmIdxq_.clear();
  chestIdxq_.clear();

  walkingHeuristic_ = false ;
  useDynamicFilter_ = false ;

  // Register method to handle
  const unsigned int NbMethods = 1;
  const char *lMethodNames[NbMethods] =
    {":useDynamicFilter"};
  for(unsigned int i=0; i<NbMethods; i++)
    {
      std::string aMethodName(lMethodNames[i]);
      if (!RegisterMethod(aMethodName))
        {
          std::cerr << "Unable to register " << aMethodName << std::endl;
        }
    }
  RESETDEBUG5("/tmp/test_configuration.dat");
}

DynamicFilter::~DynamicFilter()
{
  if (PC_!=0)
    {
      delete PC_;
      PC_ = 0 ;
    }
  if (comAndFootRealization_!=0)
    {
      delete comAndFootRealization_;
      comAndFootRealization_ = 0 ;
    }
}

void DynamicFilter::CallMethod(string &Method, istringstream &strm)
{
  if (Method==":useDynamicFilter")
    {
      string useDynamicFilter ;
      strm >> useDynamicFilter;
      useDynamicFilter_ = useDynamicFilter=="true"? true:false ;
    }
}

void DynamicFilter::setRobotUpperPart(const Eigen::VectorXd & configuration,
                                      const Eigen::VectorXd & velocity,
                                      const Eigen::VectorXd & acceleration)
{
  for ( unsigned int i = 0 ; i < larmIdxq_.size() ; ++i )
    {
      upperPartConfiguration_(larmIdxq_[i])  =
        configuration(larmIdxq_[i]);
      upperPartVelocity_(larmIdxq_[i]) =
        velocity(larmIdxq_[i]);
      upperPartAcceleration_(larmIdxq_[i]) =
        acceleration(larmIdxq_[i]);
    }
  for ( unsigned int i = 0 ; i < rarmIdxq_.size() ; ++i )
    {
      upperPartConfiguration_(rarmIdxq_[i])  =
        configuration(rarmIdxq_[i]);
      upperPartVelocity_(rarmIdxq_[i]) =
        velocity(rarmIdxq_[i]);
      upperPartAcceleration_(rarmIdxq_[i]) =
        acceleration(rarmIdxq_[i]);
    }
  for ( unsigned int i = 0 ; i < chestIdxq_.size() ; ++i )
    {
      upperPartConfiguration_(chestIdxq_[i])  =
        configuration(chestIdxq_[i]);
      upperPartVelocity_(chestIdxq_[i]) =
        velocity(chestIdxq_[i]);
      upperPartAcceleration_(chestIdxq_[i]) =
        acceleration(chestIdxq_[i]);
    }
  return ;
}

/// \brief Initialise all objects, to be called just after the constructor
/// the filter takes a subsampled previewWindow,
/// interpolate it and use the kajita preview control on it
void DynamicFilter::
init
(double controlPeriod,
 double interpolationPeriod,
 double controlWindowSize,
 double previewWindowSize,
 double kajitaPCwindowSize,
 COMState inputCoMState)
{
  controlPeriod_       = controlPeriod       ;
  interpolationPeriod_ = interpolationPeriod ;
  controlWindowSize_   = controlWindowSize   ;
  previewWindowSize_   = previewWindowSize - interpolationPeriod ;
  kajitaPCwindowSize_  = kajitaPCwindowSize - interpolationPeriod ;

  CoMHeight_ = inputCoMState.z[0] ;
  PC_->SetPreviewControlTime (kajitaPCwindowSize_);
  PC_->SetSamplingPeriod (controlPeriod_);
  PC_->SetHeightOfCoM(CoMHeight_);
  PC_->ComputeOptimalWeights(MODE_PC_);

  ZMPMB_vec_.resize( (int)round( previewWindowSize / interpolationPeriod_ ));
  int inc = (int)round(interpolationPeriod_/controlPeriod_) ;
  zmpmb_i_.resize( (ZMPMB_vec_.size()-1)*inc +1);
  deltaZMP_deq_.resize( (int)round( previewWindowSize_ / controlPeriod_ ));

  /// Set CoM/LeftFoot/RightFoot/deltax/deltay sizes
  aCoMState_.resize(6);
  aCoMSpeed_.resize(6);
  aCoMAcc_.resize(6);
  aLeftFootPosition_.resize(5);
  aRightFootPosition_.resize(5);
  deltax_.resize(3,1);
  deltay_.resize(3,1);

  /// Set CoM/LeftFoot/RightFoot/deltax/deltay to Zero
  aCoMState_.setZero();
  aCoMSpeed_.setZero();
  aCoMAcc_.setZero();
  aLeftFootPosition_.setZero();
  aRightFootPosition_.setZero();
  deltax_.setZero();
  deltay_.setZero();

  /// Update previous values
  upperPartConfiguration_ = PR_->currentRPYConfiguration() ;
  previousUpperPartConfiguration_ = PR_->currentRPYConfiguration() ;
  upperPartVelocity_ = PR_->currentRPYVelocity() ;
  previousUpperPartVelocity_ = PR_->currentRPYVelocity() ;
  upperPartAcceleration_ = PR_->currentRPYAcceleration() ;

  ZMPMBConfiguration_ = PR_->currentRPYConfiguration() ;
  ZMPMBVelocity_      = PR_->currentRPYVelocity() ;
  ZMPMBAcceleration_  = PR_->currentRPYAcceleration() ;
  previousZMPMBConfiguration_ = PR_->currentRPYConfiguration() ;
  previousZMPMBVelocity_      = PR_->currentRPYVelocity() ;

  comAndFootRealization_->SetHeightOfTheCoM(CoMHeight_);
  comAndFootRealization_->ShiftFoot(true);
  comAndFootRealization_->setSamplingPeriod(interpolationPeriod_);
  comAndFootRealization_->Initialization();
  comAndFootRealization_->SetPreviousConfigurationStage0(ZMPMBConfiguration_);
  comAndFootRealization_->SetPreviousVelocityStage0(ZMPMBVelocity_);

  comAndFootRealization_->SetPreviousConfigurationStage1(ZMPMBConfiguration_);
  comAndFootRealization_->SetPreviousVelocityStage1(ZMPMBVelocity_);

  comAndFootRealization_->SetPreviousConfigurationStage2(ZMPMBConfiguration_);
  comAndFootRealization_->SetPreviousVelocityStage2(ZMPMBVelocity_);

  sxzmp_.resize( (int)round(controlWindowSize_/controlPeriod_),0.0);
  syzmp_.resize( (int)round(controlWindowSize_/controlPeriod_),0.0);
  deltaZMPx_.resize( (int)round(controlWindowSize_/controlPeriod_),0.0);
  deltaZMPy_.resize( (int)round(controlWindowSize_/controlPeriod_),0.0);

  comAndFootRealization_->leftLegIndexinConfiguration(llegIdxq_);
  comAndFootRealization_->rightLegIndexinConfiguration(rlegIdxq_);
  comAndFootRealization_->leftArmIndexinConfiguration(larmIdxq_);
  comAndFootRealization_->rightArmIndexinConfiguration(rarmIdxq_);
  comAndFootRealization_->chestIndexinConfiguration(chestIdxq_);

  comAndFootRealization_->leftLegIndexinVelocity(llegIdxv_);
  comAndFootRealization_->rightLegIndexinVelocity(rlegIdxv_);
  comAndFootRealization_->leftArmIndexinVelocity(larmIdxv_);
  comAndFootRealization_->rightArmIndexinVelocity(rarmIdxv_);
  comAndFootRealization_->chestIndexinVelocity(chestIdxv_);
  return ;
}

int DynamicFilter::
OffLinefilter
( const deque<COMState> &inputCOMTraj_deq_,
  const deque<ZMPPosition> &inputZMPTraj_deq_,
  const deque<FootAbsolutePosition> &inputLeftFootTraj_deq_,
  const deque<FootAbsolutePosition> &inputRightFootTraj_deq_,
  const vector< Eigen::VectorXd > & UpperPart_q,
  const vector< Eigen::VectorXd > & UpperPart_dq,
  const vector< Eigen::VectorXd > & UpperPart_ddq,
  deque<COMState> & outputDeltaCOMTraj_deq)
{
  unsigned int N = (unsigned int)inputCOMTraj_deq_.size() ;
  deltaZMP_deq_.resize(N);
  if(useDynamicFilter_)
    {
      ZMPMB_vec_.resize(N) ;
      setRobotUpperPart(UpperPart_q[0],UpperPart_dq[0],UpperPart_ddq[0]);

      for(unsigned int i = 0 ; i < N ; ++i )
        {
          ComputeZMPMB(interpolationPeriod_,inputCOMTraj_deq_[i],
                       inputLeftFootTraj_deq_[i],
                       inputRightFootTraj_deq_[i], ZMPMB_vec_[i], 1, i);
        }
      for (unsigned int i = 0 ; i < N ; ++i)
        {
          deltaZMP_deq_[i].px = inputZMPTraj_deq_[i].px - ZMPMB_vec_[i][0] ;
          deltaZMP_deq_[i].py = inputZMPTraj_deq_[i].py - ZMPMB_vec_[i][1] ;
        }
    }
  else
    {
      for (unsigned int i = 0 ; i < N ; ++i)
        {
          deltaZMP_deq_[i].px = 0.0 ;
          deltaZMP_deq_[i].py = 0.0 ;
        }
    }
  OptimalControl(deltaZMP_deq_,outputDeltaCOMTraj_deq) ;

  return 0;
}

int DynamicFilter::
OnLinefilter
(const deque<COMState> & inputCOMTraj_deq_,
 const deque<ZMPPosition> & inputZMPTraj_deq_,
 const deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
 const deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
 deque<COMState> & outputDeltaCOMTraj_deq_)
{
  unsigned int N = (unsigned int)inputRightFootTraj_deq_.size() ;
  int inc = (int)round(interpolationPeriod_/controlPeriod_) ;
  unsigned int N1 = (unsigned int)((ZMPMB_vec_.size()-1)*inc +1 );
  if(useDynamicFilter_)
    {
      for(unsigned int i = 0 ; i < N ; ++i)
        {
          ComputeZMPMB(interpolationPeriod_,
                       inputCOMTraj_deq_[i],
                       inputLeftFootTraj_deq_[i],
                       inputRightFootTraj_deq_[i],
                       ZMPMB_vec_[i],
                       stage1_,
                       //currentIteration
                       i);
        }
      ZMPMB_vec_[0][0]=inputZMPTraj_deq_[0].px;
      ZMPMB_vec_[0][1]=inputZMPTraj_deq_[0].py;
      ZMPMB_vec_[0][2]=0.0;
      if(false)
        {
          zmpmb_i_.resize( N1 ) ;
          for(unsigned int i = 0 ; i < N ; ++i)
            zmpmb_i_[i*inc] = ZMPMB_vec_[i] ;

          zmpmb_i_.back() = ZMPMB_vec_.back() ;
          unsigned limit = (unsigned int)(ZMPMB_vec_.size()-1);
          for(unsigned  i=0 ; i<limit  ; ++i)
            {
              double xA = zmpmb_i_[i*inc][0] ;
              double yA = zmpmb_i_[i*inc][1] ;
              double tA = i*inc ;
              double xB = zmpmb_i_[(i+1)*inc][0] ;
              double yB = zmpmb_i_[(i+1)*inc][1] ;
              double tB = (i+1)*inc ;
              for(int j = 1 ; j < inc ; ++j)
                {
                  double t = tA+j ;
                  zmpmb_i_[(i*inc)+j][0] = xA + (t-tA)*(xB-xA)/(tB-tA) ;
                  zmpmb_i_[(i*inc)+j][1] = yA + (t-tA)*(yB-yA)/(tB-tA) ;
                  zmpmb_i_[(i*inc)+j][2] = 0.0 ;
                }
            }
        }
      else
        {
          zmpmb_i_.resize( N1 ) ;
          for(unsigned int i = 0 ; i < N ; ++i)
            zmpmb_i_[i*inc] = ZMPMB_vec_[i] ;

          vector<vector<double> > dZMPMB_vec (N,vector<double>(2,0.0)) ;
          dZMPMB_vec[0][0] = (ZMPMB_vec_[1][0]  -ZMPMB_vec_[0][0]  )/inc;
          dZMPMB_vec[0][1] = (ZMPMB_vec_[1][1]  -ZMPMB_vec_[0][1]  )/inc;
          dZMPMB_vec[N-1][0] = (ZMPMB_vec_[N-1][0]-ZMPMB_vec_[N-2][0])/inc;
          dZMPMB_vec[N-1][1] = (ZMPMB_vec_[N-1][1]-ZMPMB_vec_[N-2][1])/inc;
          for(unsigned i=1 ; i<N-2  ; ++i)
            {
              dZMPMB_vec[i][0] =
                (ZMPMB_vec_[i+1][0]-ZMPMB_vec_[i-1][0])/(2*inc);
              dZMPMB_vec[i][1] =
                (ZMPMB_vec_[i+1][1]-ZMPMB_vec_[i-1][1])/(2*inc);
            }
          for(unsigned i=0 ; i<N-1  ; ++i)
            {
              Polynome5 polyX(1.0,0.0) ;
              Polynome5 polyY(1.0,0.0) ;
              polyX.SetParameters(inc,ZMPMB_vec_[i][0],dZMPMB_vec[i][0],0.0,
                                  ZMPMB_vec_[i+1][0],dZMPMB_vec[i+1][0],0.0);
              polyY.SetParameters(inc,ZMPMB_vec_[i][1],dZMPMB_vec[i][1],0.0,
                                  ZMPMB_vec_[i+1][1],dZMPMB_vec[i+1][1],0.0);

              for(int j = 1 ; j < inc ; ++j)
                {
                  zmpmb_i_[(i*inc)+j][0] = polyX.Compute(j) ;
                  zmpmb_i_[(i*inc)+j][1] = polyY.Compute(j) ;
                  zmpmb_i_[(i*inc)+j][2] = 0.0 ;
                }
            }
        }
      deltaZMP_deq_.resize(zmpmb_i_.size());
      for (unsigned int i = 0 ; i < N1 ; ++i)
        {
          deltaZMP_deq_[i].px = inputZMPTraj_deq_[i].px - zmpmb_i_[i][0] ;
          deltaZMP_deq_[i].py = inputZMPTraj_deq_[i].py - zmpmb_i_[i][1] ;
        }
    }
  else
    {
      deltaZMP_deq_.resize(N1);
      for (unsigned int i = 0 ; i < N1 ; ++i)
        {
          deltaZMP_deq_[i].px = 0.0 ;
          deltaZMP_deq_[i].py = 0.0 ;
        }
    }

  OptimalControl(deltaZMP_deq_,outputDeltaCOMTraj_deq_) ;

  return 0 ;
}

// #############################
int DynamicFilter::
zmpmb
(Eigen::VectorXd& configuration,
 Eigen::VectorXd& velocity,
 Eigen::VectorXd& acceleration,
 Eigen::Vector3d & zmpmb)
{
  PR_->computeInverseDynamics(configuration,velocity,acceleration);
  PR_->zeroMomentumPoint(zmpmb);
  return 0 ;
}

//##################################
void DynamicFilter::
InverseKinematics
(const COMState & inputCoMState,
 const FootAbsolutePosition & inputLeftFoot,
 const FootAbsolutePosition & inputRightFoot,
 Eigen::VectorXd& configuration,
 Eigen::VectorXd& velocity,
 Eigen::VectorXd& acceleration,
 double samplingPeriod,
 int stage,
 int iteration)
{

  // lower body !!!!! the angular quantities are set in degree !!!!!!
  aCoMState_(0) = inputCoMState.x[0];
  aCoMSpeed_(0) = inputCoMState.x[1];
  aCoMState_(1) = inputCoMState.y[0];
  aCoMSpeed_(1) = inputCoMState.y[1];
  aCoMState_(2) = inputCoMState.z[0];
  aCoMSpeed_(2) = inputCoMState.z[1];
  aCoMState_(3) = inputCoMState.roll[0];
  aCoMSpeed_(3) = inputCoMState.roll[1];
  aCoMState_(4) = inputCoMState.pitch[0];
  aCoMSpeed_(4) = inputCoMState.pitch[1];
  aCoMState_(5) = inputCoMState.yaw[0];
  aCoMSpeed_(5) = inputCoMState.yaw[1];

  aCoMAcc_(0) = inputCoMState.x[2];
  aLeftFootPosition_(0) = inputLeftFoot.x;
  aCoMAcc_(1) = inputCoMState.y[2];
  aLeftFootPosition_(1) = inputLeftFoot.y;
  aCoMAcc_(2) = inputCoMState.z[2];
  aLeftFootPosition_(2) = inputLeftFoot.z;
  aCoMAcc_(3) = inputCoMState.roll[2];
  aLeftFootPosition_(3) = inputLeftFoot.theta;
  aCoMAcc_(4) = inputCoMState.pitch[2];
  aLeftFootPosition_(4) = inputLeftFoot.omega;
  aCoMAcc_(5) = inputCoMState.yaw[2];

  aRightFootPosition_(0) = inputRightFoot.x;
  aRightFootPosition_(1) = inputRightFoot.y;
  aRightFootPosition_(2) = inputRightFoot.z;
  aRightFootPosition_(3) = inputRightFoot.theta;
  aRightFootPosition_(4) = inputRightFoot.omega;

  /*
    std::cout << "aCoMState_ :" << aCoMState_ << std::endl
    << " aCoMSpeed_ :" << aCoMSpeed_ << std::endl
    << " aCoMAcc_ :" << aCoMAcc_ << std::endl;
  */
  comAndFootRealization_->setSamplingPeriod(samplingPeriod);
  comAndFootRealization_->
    ComputePostureForGivenCoMAndFeetPosture
    (aCoMState_, aCoMSpeed_, aCoMAcc_,
     aLeftFootPosition_, aRightFootPosition_,
     configuration, velocity, acceleration,
     iteration, stage);
  
  //  std::cout << " configuration:" << configuration << std::endl;
  ODEBUG5SIMPLE(configuration,"/tmp/test_configuration.dat");

  // upper body
  if (walkingHeuristic_)
    {
      upperPartConfiguration_         = configuration           ;
      upperPartVelocity_              = velocity                ;
      upperPartAcceleration_          = acceleration            ;
      previousUpperPartConfiguration_ = upperPartConfiguration_ ;
      previousUpperPartVelocity_      = upperPartVelocity_      ;

      upperPartVelocity_.setZero();
      upperPartAcceleration_.setZero();
      previousUpperPartVelocity_.setZero() ;
    }

  for ( unsigned int i = 0 ; i < larmIdxq_.size() ; ++i )
    {
      configuration(larmIdxq_[i]) = upperPartConfiguration_(larmIdxq_[i]);
      velocity(larmIdxv_[i])      = upperPartVelocity_(larmIdxv_[i]);
      acceleration(larmIdxv_[i])  = upperPartAcceleration_(larmIdxv_[i]);
    }
  for ( unsigned int i = 0 ; i < rarmIdxq_.size() ; ++i )
    {
      configuration(rarmIdxq_[i]) = upperPartConfiguration_(rarmIdxq_[i]);
      velocity(rarmIdxv_[i])      = upperPartVelocity_(rarmIdxv_[i]);
      acceleration(rarmIdxv_[i])  = upperPartAcceleration_(rarmIdxv_[i]);
    }
  for ( unsigned int i = 0 ; i < chestIdxq_.size() ; ++i )
    {
      configuration(chestIdxq_[i]) = upperPartConfiguration_(chestIdxq_[i]);
      velocity(chestIdxv_[i])      = upperPartVelocity_(chestIdxv_[i]);
      acceleration(chestIdxv_[i])  = upperPartAcceleration_(chestIdxv_[i]);
    }
  return;
}

void DynamicFilter::stage0INstage1()
{
  comAndFootRealization_->
    SetPreviousConfigurationStage1
    (comAndFootRealization_->GetPreviousConfigurationStage0());
  comAndFootRealization_->
    SetPreviousVelocityStage1
    (comAndFootRealization_->GetPreviousVelocityStage0());
  return ;
}

void DynamicFilter::
ComputeZMPMB
(double samplingPeriod,
 const COMState & inputCoMState,
 const FootAbsolutePosition & inputLeftFoot,
 const FootAbsolutePosition & inputRightFoot,
 Eigen::Vector3d &ZMPMB,
 unsigned int stage,
 unsigned int iteration)
{
  InverseKinematics( inputCoMState, inputLeftFoot, inputRightFoot,
                     ZMPMBConfiguration_, ZMPMBVelocity_, ZMPMBAcceleration_,
                     samplingPeriod, stage, iteration) ;

  if(iteration>0)
    {
      PR_->computeInverseDynamics(ZMPMBConfiguration_,
                                  ZMPMBVelocity_,
                                  ZMPMBAcceleration_);

      PR_->zeroMomentumPoint(ZMPMB);
    }

  return ;
}

int DynamicFilter::
OptimalControl
( deque<ZMPPosition> & inputdeltaZMP_deq,
  deque<COMState> & outputDeltaCOMTraj_deq_)
{
  assert(PC_->IsCoherent());
  std::size_t Nctrl = (int)round(controlWindowSize_/controlPeriod_) ;

  assert(outputDeltaCOMTraj_deq_.size()==Nctrl);
  double deltaZMPx = 0.0 ;
  double deltaZMPy = 0.0 ;
  // computation of the preview control along the "deltaZMP_deq_"
  //  for(unsigned i=0;i<3;++i)
  //  {
  //    deltax_(i,0)=0.0;
  //    deltay_(i,0)=0.0;
  //  }
  for (std::size_t i = 0 ; i < Nctrl ; ++i )
    {
      PC_->OneIterationOfPreview(deltax_,deltay_,
                                 sxzmp_[0],syzmp_[0],
                                 inputdeltaZMP_deq,i,
                                 deltaZMPx, deltaZMPy,
                                 false);
      for(int j=0; j<3; ++j)
        {
          outputDeltaCOMTraj_deq_[i].x[j] = deltax_(j,0);
          outputDeltaCOMTraj_deq_[i].y[j] = deltay_(j,0);
        }
    }
  // test to verify if the Kajita PC diverged
  for (std::size_t i = 0 ; i < Nctrl ; ++i)
    {
      for(int j=0; j<3; ++j)
        {
          if ( (outputDeltaCOMTraj_deq_[i].x[j] ==
                outputDeltaCOMTraj_deq_[i].x[j]) ||
               (outputDeltaCOMTraj_deq_[i].y[j] ==
                outputDeltaCOMTraj_deq_[i].y[j]) )
            {}
          else
            {
              cout << "kajita2003 preview control diverged "
                   << outputDeltaCOMTraj_deq_[i].x[j] << " "
                   << outputDeltaCOMTraj_deq_[i].y[j] << " "
                   << deltaZMPx << " "
                   << deltaZMPy << "\n" ;
              return -1 ;
            }
        }
    }
  return 0 ;
}

// TODO finish the implementation of a better waist tracking
//void DynamicFilter::computeWaist(const FootAbsolutePosition & inputLeftFoot)
//{
//  Eigen::Matrix< LocalFloatType, 6, 1 > waist_speed, waist_acc ;
//  Eigen::Matrix< LocalFloatType, 3, 1 > waist_theta ;
//  // compute the speed and acceleration of the waist in the world frame
//  if (PreviousSupportFoot_)
//    {
//      Jac_LF::run(robot_, prev_q_, Vector3dTpl<LocalFloatType>::Type(0,0,0),
//  jacobian_lf_);
//      waist_speed = jacobian_lf_ * prev_dq_ ;
//      waist_acc = jacobian_lf_ * prev_ddq_ /* + d_jacobian_lf_ * prev_dq_*/ ;
//    }else
//    {
//      Jac_RF::run(robot_, prev_q_, Vector3dTpl<LocalFloatType>::Type(0,0,0),
// jacobian_rf_);
//      waist_speed = jacobian_rf_ * prev_dq_ ;
//      waist_acc = jacobian_rf_ * prev_ddq_ /*+ d_jacobian_rf_ * prev_dq_*/ ;
//    }
//  for (unsigned int i = 0 ; i < 6 ; ++i)
//    {
//      dq_(i,0)   = waist_speed(i,0);
//      ddq_(i,0)  = waist_acc(i,0);
//    }
//  // compute the position of the waist in the world frame
//  RootNode & node_waist = boost::fusion::at_c<Robot_Model::BODY>
// (robot_.nodes);
//  waist_theta(0,0) = prev_q_(3,0) ;
//  waist_theta(1,0) = prev_dq_(4,0) ;
//  waist_theta(2,0) = prev_ddq_(5,0) ;
//  q_(0,0) = node_waist.body.iX0.inverse().r()(0,0) ;
//  q_(1,0) = node_waist.body.iX0.inverse().r()(1,0) ;
//  q_(2,0) = node_waist.body.iX0.inverse().r()(2,0) ;
//  q_(3,0) = waist_theta(0,0) ;
//  q_(4,0) = waist_theta(1,0) ;
//  q_(5,0) = waist_theta(2,0) ;

//  if (inputLeftFoot.stepType<0)
//    {
//      PreviousSupportFoot_ = true ; // left foot is supporting
//    }
//  else
//    {
//      PreviousSupportFoot_ = false ; // right foot is supporting
//    }
//  prev_q_ = q_ ;
//  prev_dq_ = dq_ ;
//  prev_ddq_ = ddq_ ;

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
//  LankleNode & node_lankle =
// boost::fusion::at_c<Robot_Model::l_ankle>(robot_2.nodes);
//  RankleNode & node_rankle =
// boost::fusion::at_c<Robot_Model::r_ankle>(robot_2.nodes);
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

//  return ;
//}

void DynamicFilter::
Debug
(const deque<COMState> & ctrlCoMState,
 const deque<FootAbsolutePosition> & ctrlLeftFoot,
 const deque<FootAbsolutePosition> & ctrlRightFoot,
 const deque<COMState> & inputCOMTraj_deq_,
 const deque<ZMPPosition> inputZMPTraj_deq_,
 const deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
 const deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
 const deque<COMState> & outputDeltaCOMTraj_deq_)
{
  deque<COMState> CoM_tmp = ctrlCoMState ;
  int Nctrl = (int)round(controlWindowSize_/controlPeriod_) ;

  for (int i = 0 ; i < Nctrl ; ++i)
    {
      for(int j=0; j<3; j++)
        {
          CoM_tmp[i].x[j] += outputDeltaCOMTraj_deq_[i].x[j] ;
          CoM_tmp[i].y[j] += outputDeltaCOMTraj_deq_[i].y[j] ;
        }
    }

  vector < Eigen::VectorXd > conf ;
  vector < Eigen::VectorXd > vel ;
  vector < Eigen::VectorXd > acc ;
  vector< Eigen::Vector3d > zmpmb_corr (Nctrl);
  for(int i = 0 ; i < Nctrl ; ++i)
    {
      COMState com = ctrlCoMState[i];
      com.roll[0]  = com.roll[0]  * 180 / M_PI ;
      com.pitch[0] = com.pitch[0] * 180 / M_PI ;
      com.yaw[0]   = com.yaw[0]   * 180 / M_PI ;
      InverseKinematics( com, ctrlLeftFoot[i], ctrlRightFoot[i],
                         ZMPMBConfiguration_, ZMPMBVelocity_,
                         ZMPMBAcceleration_,
                         controlPeriod_, 2, 20) ;

      PR_->computeInverseDynamics(ZMPMBConfiguration_,
                                  ZMPMBVelocity_,
                                  ZMPMBAcceleration_);

      conf.push_back(ZMPMBConfiguration_);
      vel.push_back(ZMPMBVelocity_);
      acc.push_back(ZMPMBAcceleration_);
      //    cout << "DF debug : " ;
      //    cout << ZMPMBAcceleration_(3) << " "
      //         << ZMPMBAcceleration_(4) << " "
      //         << ZMPMBAcceleration_(5) << endl ;

      PR_->zeroMomentumPoint(zmpmb_corr[i]);
    }

  int inc = (int)round(interpolationPeriod_/controlPeriod_) ;
  ofstream aof;
  string aFileName;
  static int iteration_zmp = 0 ;
  ostringstream oss(std::ostringstream::ate);
  oss.str("/tmp/zmpmb_herdt.txt");
  aFileName = oss.str();
  if ( iteration_zmp == 0 )
    {
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }

  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  int NbI = (int)round(controlWindowSize_/interpolationPeriod_) ;
  for (int i = 0 ; i < NbI ; ++i)
    {
      aof << (iteration_zmp+i)*interpolationPeriod_ << " " ;       // 1

      aof << inputZMPTraj_deq_[i*inc].px << " " ;       // 1
      aof << inputZMPTraj_deq_[i*inc].py << " " ;       // 2

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

      aof << CoM_tmp[i*(int)round(interpolationPeriod_/
                                  controlPeriod_)].x[0] << " "
        ; // 41
      aof << CoM_tmp[i*(int)round(interpolationPeriod_/
                                  controlPeriod_)].x[1] << " "
        ; // 42
      aof << CoM_tmp[i*(int)round(interpolationPeriod_/
                                  controlPeriod_)].x[2] << " "
        ; // 43

      aof << CoM_tmp[i*(int)round(interpolationPeriod_/
                                  controlPeriod_)].y[0] << " "
        ; // 44
      aof << CoM_tmp[i*(int)round(interpolationPeriod_/
                                  controlPeriod_)].y[1]
          << " "
        ; // 45
      aof << CoM_tmp[i*(int)round(interpolationPeriod_/
                                  controlPeriod_)].y[2]
          << " "
        ; // 46

      //47
      int it_subsample = i*(int)round(interpolationPeriod_/controlPeriod_) ;
      for (unsigned int k = 0 ; k < conf[it_subsample].size() ; ++k)
        aof << conf[it_subsample](k) << " " ;

      //83
      for (unsigned int k = 0 ; k < vel[it_subsample].size() ; ++k)
        aof << vel[it_subsample](k) << " " ;

      //119
      for (unsigned int k = 0 ; k < acc[it_subsample].size() ; ++k)
        aof << acc[it_subsample](k) << " " ;

      aof << endl ;
    }
  aof.close();

  aFileName = "/tmp/zmpmb_corr_herdt.txt" ;
  if ( iteration_zmp == 0 )
    {
      aof.open(aFileName.c_str(),ofstream::out);
      aof.close();
    }
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (int i = 0 ; i < Nctrl ; ++i)
    {
      aof << zmpmb_corr[i][0] << " " ;                    // 1
      aof << zmpmb_corr[i][1] << " " ;                    // 2
      aof << outputDeltaCOMTraj_deq_[i].x[0] << " "  ;    // 3
      aof << outputDeltaCOMTraj_deq_[i].y[0] << " "  ;    // 4
      aof << outputDeltaCOMTraj_deq_[i].x[1] << " "  ;    // 5
      aof << outputDeltaCOMTraj_deq_[i].y[1] << " "  ;    // 6
      aof << outputDeltaCOMTraj_deq_[i].x[2] << " "  ;    // 7
      aof << outputDeltaCOMTraj_deq_[i].y[2] << " "  ;    // 8
      aof << deltaZMPx_[i] << " " ;                       // 9
      aof << deltaZMPy_[i] << " " ;                       // 10
      aof << sxzmp_[i] << " " ;                           // 11
      aof << syzmp_[i] << " " ;                           // 12
      aof << deltaZMP_deq_[i].px << " " ;                 // 13
      aof << deltaZMP_deq_[i].py << " " ;                 // 14
      for(unsigned j=0 ; j<acc[i].size() ; ++j)// 15+nq
        aof << conf[i][j] << " " ;
      for(unsigned j=0 ; j<acc[i].size() ; ++j)// 15+nq
        aof << vel[i][j] << " " ;
      for(unsigned j=0 ; j<acc[i].size() ; ++j)// 15+nq
        aof << acc[i][j] << " " ;                        //
      aof << endl ;
    }
  aof.close();

  oss.str("/tmp/buffer_");
  oss << setfill('0') << setw(3) << iteration_zmp << ".txt" ;
  aFileName = oss.str();
  aof.open(aFileName.c_str(),ofstream::out);
  aof.close();

  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  //for (int i = 0 ; i < zmpmb_i_.size() ; ++i)
  for (int i = 0 ; i < (int)zmpmb_i_.size() ; ++i)
    {
      aof << i << " " ; // 0
      aof << inputZMPTraj_deq_[i].px << " " ;           // 1
      aof << inputZMPTraj_deq_[i].py << " " ;           // 2

      aof << zmpmb_i_[i][0] << " " ;                    // 3
      aof << zmpmb_i_[i][1] << " " ;                    // 4

      aof << ctrlCoMState[i].x[0] << " " ;         // 5
      aof << ctrlCoMState[i].x[1] << " " ;         // 6
      aof << ctrlCoMState[i].x[2] << " " ;         // 7

      aof << ctrlLeftFoot[i].x << " " ;       // 8
      aof << ctrlLeftFoot[i].dx << " " ;      // 9
      aof << ctrlLeftFoot[i].ddx << " " ;     // 10

      aof << ctrlRightFoot[i].x << " " ;      // 11
      aof << ctrlRightFoot[i].dx << " " ;     // 12
      aof << ctrlRightFoot[i].ddx << " " ;    // 13

      aof << ctrlCoMState[i].y[0] << " " ;         // 14
      aof << ctrlCoMState[i].y[1] << " " ;         // 15
      aof << ctrlCoMState[i].y[2] << " " ;         // 16

      aof << ctrlLeftFoot[i].y << " " ;       // 17
      aof << ctrlLeftFoot[i].dy << " " ;      // 18
      aof << ctrlLeftFoot[i].ddy << " " ;     // 19

      aof << ctrlRightFoot[i].y << " " ;      // 20
      aof << ctrlRightFoot[i].dy << " " ;     // 21
      aof << ctrlRightFoot[i].ddy << " " ;    // 22

      aof << ctrlCoMState[i].yaw[0] << " " ;       // 23
      aof << ctrlCoMState[i].yaw[1] << " " ;       // 24
      aof << ctrlCoMState[i].yaw[2] << " " ;       // 25

      aof << ctrlLeftFoot[i].theta << " " ;   // 26
      aof << ctrlLeftFoot[i].dtheta << " " ;  // 27
      aof << ctrlLeftFoot[i].ddtheta << " " ; // 28

      aof << ctrlRightFoot[i].theta << " " ;  // 29
      aof << ctrlRightFoot[i].dtheta << " " ; // 30
      aof << ctrlRightFoot[i].ddtheta << " " ;// 31

      aof << ctrlCoMState[i].z[0] << " " ;         // 32
      aof << ctrlCoMState[i].z[1] << " " ;         // 33
      aof << ctrlCoMState[i].z[2] << " " ;         // 34

      aof << ctrlLeftFoot[i].z << " " ;       // 35
      aof << ctrlLeftFoot[i].dz << " " ;      // 36
      aof << ctrlLeftFoot[i].ddz << " " ;     // 37

      aof << ctrlRightFoot[i].z << " " ;      // 38
      aof << ctrlRightFoot[i].dz << " " ;     // 39
      aof << ctrlRightFoot[i].ddz << " " ;    // 40

      aof << deltaZMP_deq_[i].px << " " ;    // 41
      aof << deltaZMP_deq_[i].py << " " ;    // 42
      aof << endl ;
    }
  aof.close();
  iteration_zmp++;
  return ;
}

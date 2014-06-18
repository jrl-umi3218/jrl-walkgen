#include "DynamicFilter.hh"
#include <metapod/algos/rnea.hh>

using namespace std;
using namespace PatternGeneratorJRL;
using namespace metapod;

DynamicFilter::DynamicFilter(
    SimplePluginManager *SPM,
    CjrlHumanoidDynamicRobot *aHS
    )
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

/// \brief Initialse all objects, to be called just after the constructor
void DynamicFilter::init(
    double currentTime,
    double controlPeriod,
    double interpolationPeriod,
    double PG_T,
    double previewWindowSize,
    double CoMHeight
    )
{
  currentTime_ = currentTime ;
  controlPeriod_ = controlPeriod ;
  interpolationPeriod_ = interpolationPeriod ;
  PG_T_ = PG_T ;
  previewWindowSize_ = previewWindowSize ;

  if (PG_T>interpolationPeriod_)
  {NbI_=1;}
  else
  {NbI_ = PG_T/interpolationPeriod_;}

  NCtrl_ = (int)(PG_T_/controlPeriod_) ;
  PG_N_ = (int)(previewWindowSize_/interpolationPeriod_) ;

  CoMHeight_ = CoMHeight ;
  PC_->SetPreviewControlTime (previewWindowSize_ - PG_T/controlPeriod_ * interpolationPeriod_);
  PC_->SetSamplingPeriod (interpolationPeriod_);
  PC_->SetHeightOfCoM(CoMHeight_);

  previousConfiguration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentConfiguration() ;
  previousVelocity_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentVelocity() ;
  previousAcceleration_ = comAndFootRealization_->getHumanoidDynamicRobot()->currentAcceleration() ;

  configurationTraj_.resize( PG_N_, previousConfiguration_ ); ;
  velocityTraj_.resize( PG_N_, previousVelocity_ ); ;
  accelerationTraj_.resize( PG_N_, previousAcceleration_ ); ;

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
  return ;
}

int DynamicFilter::filter(
    deque<COMState> & inputCOMTraj_deq_,
    deque<ZMPPosition> inputZMPTraj_deq_,
    deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
    deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
    deque<COMState> & outputDeltaCOMTraj_deq_
    )
{
  InverseKinematics(
      inputCOMTraj_deq_,
      inputLeftFootTraj_deq_,
      inputRightFootTraj_deq_);


  //InverseDynamics(inputZMPTraj_deq_);

  //int error = OptimalControl(outputDeltaCOMTraj_deq_);

  printBuffers(inputCOMTraj_deq_,
             inputZMPTraj_deq_,
             inputLeftFootTraj_deq_,
             inputRightFootTraj_deq_,
             outputDeltaCOMTraj_deq_);
  int error = 0 ;
  return error ;
}

void DynamicFilter::InverseKinematics(
    deque<COMState> & inputCOMTraj_deq_,
    deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
    deque<FootAbsolutePosition> & inputRightFootTraj_deq_)
{
  const int stage0 = 0 ;
  int iteration = 2 ;
  comAndFootRealization_->SetPreviousConfigurationStage0(
      previousConfiguration_);
  comAndFootRealization_->SetPreviousVelocityStage0(
      previousVelocity_);

  for(unsigned int i = 0 ; i <  PG_N_ ; i++ )
  {
    const COMState & acomp = inputCOMTraj_deq_[i] ;
    const FootAbsolutePosition & aLeftFAP =
        inputLeftFootTraj_deq_ [i] ;
    const FootAbsolutePosition & aRightFAP =
        inputRightFootTraj_deq_ [i] ;

    aCoMState_(0) = acomp.x[0];      aCoMSpeed_(0) = acomp.x[1];
    aCoMState_(1) = acomp.y[0];      aCoMSpeed_(1) = acomp.y[1];
    aCoMState_(2) = acomp.z[0];      aCoMSpeed_(2) = acomp.z[1];
    aCoMState_(3) = acomp.roll[0];   aCoMSpeed_(3) = acomp.roll[1];
    aCoMState_(4) = acomp.pitch[0];  aCoMSpeed_(4) = acomp.pitch[1];
    aCoMState_(5) = acomp.yaw[0];		 aCoMSpeed_(5) = acomp.yaw[1];

    aCoMAcc_(0) = acomp.x[2];    aLeftFootPosition_(0) = aLeftFAP.x;
    aCoMAcc_(1) = acomp.y[2];    aLeftFootPosition_(1) = aLeftFAP.y;
    aCoMAcc_(2) = acomp.z[2];    aLeftFootPosition_(2) = aLeftFAP.z;
    aCoMAcc_(3) = acomp.roll[2]; aLeftFootPosition_(3) = aLeftFAP.theta;
    aCoMAcc_(4) = acomp.pitch[2];aLeftFootPosition_(4) = aLeftFAP.omega;
    aCoMAcc_(5) = acomp.yaw[2];

    aRightFootPosition_(0) = aRightFAP.x;
    aRightFootPosition_(1) = aRightFAP.y;
    aRightFootPosition_(2) = aRightFAP.z;
    aRightFootPosition_(3) = aRightFAP.theta;
    aRightFootPosition_(4) = aRightFAP.omega;

    comAndFootRealization_->ComputePostureForGivenCoMAndFeetPosture(
        aCoMState_, aCoMSpeed_, aCoMAcc_,
        aLeftFootPosition_, aRightFootPosition_,
        configurationTraj_[i],
        velocityTraj_[i],
        accelerationTraj_[i],
        iteration, stage0);
  }

//  tmpConfigurationTraj_[0] = ( ConfigurationTraj_[1]+ConfigurationTraj_[0]+PreviousConfiguration_ )/3;
//  tmpVelocityTraj_[0]      = ( VelocityTraj_[1]+VelocityTraj_[0]+PreviousVelocity_ )/3;
//  tmpAccelerationTraj_[0]  = ( AccelerationTraj_[1]+AccelerationTraj_[0]+PreviousAcceleration_ )/3;

  // saving the precedent state of the next QP_ computation
  previousConfiguration_ = configurationTraj_[NbI_-1] ;
  previousVelocity_ = velocityTraj_[NbI_-1] ;
  previousAcceleration_ = accelerationTraj_[NbI_-1] ;

//  for (unsigned int i = 1 ; i < N-1 ; ++i )
//  {
//    tmpConfigurationTraj_[i] = ( ConfigurationTraj_[i+1] + ConfigurationTraj_[i] + ConfigurationTraj_[i-1] )/3;
//    tmpVelocityTraj_[i] = ( VelocityTraj_[i+1] + VelocityTraj_[i] + VelocityTraj_[i-1] )/3;
//    tmpAccelerationTraj_[i] = ( AccelerationTraj_[i+1] + AccelerationTraj_[i] + AccelerationTraj_[i-1] )/3;
//  }
//
//  tmpConfigurationTraj_[N-1] = ( ConfigurationTraj_[N-1]+ConfigurationTraj_[N-2] )*0.5;
//  tmpVelocityTraj_[N-1]      = ( VelocityTraj_[N-1]+VelocityTraj_[N-2] )*0.5;
//  tmpAccelerationTraj_[N-1]  = ( AccelerationTraj_[N-1]+AccelerationTraj_[N-2] )*0.5;
//
//
//  ConfigurationTraj_ = tmpConfigurationTraj_ ;
//  VelocityTraj_ = tmpVelocityTraj_ ;
//  AccelerationTraj_ = tmpAccelerationTraj_ ;

  return ;
}

void DynamicFilter::InverseDynamics(
    deque<ZMPPosition> inputZMPTraj_deq_)
{
  for (unsigned int i = 0 ; i < PG_N_ ; i++ )
  {
    // Copy the angular trajectory data from "Boost" to "Eigen"
    for(unsigned int j = 0 ; j < configurationTraj_[i].size() ; j++ )
    {
      m_q(j,0) = configurationTraj_[i](j) ;
      m_dq(j,0) = velocityTraj_[i](j) ;
      m_ddq(j,0) = accelerationTraj_[i](j) ;
    }

    // Apply the RNEA on the robot model
    metapod::rnea< Robot_Model, true >::run(m_robot, m_q, m_dq, m_ddq);

    Node & node =
        boost::fusion::at_c<Robot_Model::BODY>(m_robot.nodes);
    m_force = node.body.iX0.applyInv (node.joint.f);

    if (Once_){
      DInitX_ = inputZMPTraj_deq_[0].px -
                ( - m_force.n()[1] / m_force.f()[2] ) ;
      DInitY_ = inputZMPTraj_deq_[0].py -
                (   m_force.n()[0] / m_force.f()[2] ) ;
      Once_ = false ;
    }

    ZMPMB_vec_[i][0] = - m_force.n()[1] / m_force.f()[2] + DInitX_ ;
    ZMPMB_vec_[i][1] =   m_force.n()[0] / m_force.f()[2] + DInitY_ ;

    deltaZMP_deq_[i].px = inputZMPTraj_deq_[i].px - ZMPMB_vec_[i][0] ;
    deltaZMP_deq_[i].py = inputZMPTraj_deq_[i].py - ZMPMB_vec_[i][1] ;
    deltaZMP_deq_[i].pz = 0.0 ;
    deltaZMP_deq_[i].theta = 0.0 ;
    deltaZMP_deq_[i].time = currentTime_ + i * interpolationPeriod_ ;
    deltaZMP_deq_[i].stepType = inputZMPTraj_deq_[i].stepType ;
  }
  return ;
}

int DynamicFilter::OptimalControl(
    deque<COMState> & outputDeltaCOMTraj_deq_)
{
  if(!PC_->IsCoherent())
    PC_->ComputeOptimalWeights(OptimalControllerSolver::MODE_WITH_INITIALPOS);


  double aSxzmp (0) , aSyzmp(0);
  double deltaZMPx (0) , deltaZMPy (0) ;

  // calcul of the preview control along the "deltaZMP_deq_"
  for (unsigned i = 0 ; i < NCtrl_ ; i++ )
  {
    for(int j=0;j<3;j++)
    {
      deltax_(j,0) = 0 ;
      deltay_(j,0) = 0 ;
    }
    PC_->OneIterationOfPreview(deltax_,deltay_,
                               aSxzmp,aSyzmp,
                               deltaZMP_deq_,i,
                               deltaZMPx, deltaZMPy, false);
    for(int j=0;j<3;j++)
    {
      outputDeltaCOMTraj_deq_[i].x[j] = deltax_(j,0);
      outputDeltaCOMTraj_deq_[i].y[j] = deltay_(j,0);
    }
  }

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

double DynamicFilter::filterprecision(double adb)
{
  if (fabs(adb)<1e-7)
    return 0.0;

  if (fabs(adb)>1e7)
    return 1e7 ;

  double ladb2 = adb * 1e7;
  double lintadb2 = trunc(ladb2);
  return lintadb2/1e7;
}

void DynamicFilter::printAlongTime(deque<COMState> & inputCOMTraj_deq_,
                               deque<ZMPPosition> inputZMPTraj_deq_,
                               deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
                               deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
                               deque<COMState> & outputDeltaCOMTraj_deq_
                               )
{  
  // Debug Purpose
  // -------------
  ofstream aof;
  string aFileName;
  ostringstream oss(std::ostringstream::ate);
  static int iteration = 0;
//  int iteration100 = (int)iteration/100;
//  int iteration10 = (int)(iteration - iteration100*100)/10;
//  int iteration1 = (int)(iteration - iteration100*100 - iteration10*10 );

  // --------------------
  oss.str("DynamicFilterAllVariablesAlongInTime.dat");
  aFileName = oss.str();
  if(iteration == 0)
  {
    aof.open(aFileName.c_str(),ofstream::out);
    aof.close();
  }
  ///----
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (unsigned int i = 0 ; i < NbI_ ; ++i )
  {
    aof << filterprecision( iteration*controlPeriod_ + i*interpolationPeriod_ ) << " " // 0
        << filterprecision( inputCOMTraj_deq_[i].x[0] ) << " "    // 1
        << filterprecision( inputCOMTraj_deq_[i].x[1] ) << " "    // 2
        << filterprecision( inputCOMTraj_deq_[i].x[2] ) << " "    // 3
        << filterprecision( inputCOMTraj_deq_[i].y[0] ) << " "    // 4
        << filterprecision( inputCOMTraj_deq_[i].y[1] ) << " "    // 5
        << filterprecision( inputCOMTraj_deq_[i].y[2] ) << " "    // 6
        << filterprecision( inputCOMTraj_deq_[i].z[0] ) << " "    // 7
        << filterprecision( inputCOMTraj_deq_[i].z[1] ) << " "    // 8
        << filterprecision( inputCOMTraj_deq_[i].z[2] ) << " "    // 9
        << filterprecision( inputCOMTraj_deq_[i].roll[0] ) << " " // 10
        << filterprecision( inputCOMTraj_deq_[i].roll[1] ) << " " // 11
        << filterprecision( inputCOMTraj_deq_[i].roll[2] ) << " " // 12
        << filterprecision( inputCOMTraj_deq_[i].pitch[0] ) << " "// 13
        << filterprecision( inputCOMTraj_deq_[i].pitch[1] ) << " "// 14
        << filterprecision( inputCOMTraj_deq_[i].pitch[2] ) << " "// 15
        << filterprecision( inputCOMTraj_deq_[i].yaw[0] ) << " "  // 16
        << filterprecision( inputCOMTraj_deq_[i].yaw[1] ) << " "  // 17
        << filterprecision( inputCOMTraj_deq_[i].yaw[2] ) << " "  // 18

        << filterprecision( inputZMPTraj_deq_[i].px ) << " "      // 19
        << filterprecision( inputZMPTraj_deq_[i].py ) << " "      // 20

        << filterprecision( ZMPMB_vec_[i][0] ) << " "                  // 21
        << filterprecision( ZMPMB_vec_[i][1] ) << " "                  // 22

        << filterprecision( inputLeftFootTraj_deq_[i].x ) << " "       // 23
        << filterprecision( inputLeftFootTraj_deq_[i].y ) << " "       // 24
        << filterprecision( inputLeftFootTraj_deq_[i].z ) << " "       // 25
        << filterprecision( inputLeftFootTraj_deq_[i].theta ) << " "   // 26
        << filterprecision( inputLeftFootTraj_deq_[i].omega ) << " "   // 27
        << filterprecision( inputLeftFootTraj_deq_[i].dx ) << " "      // 28
        << filterprecision( inputLeftFootTraj_deq_[i].dy ) << " "      // 29
        << filterprecision( inputLeftFootTraj_deq_[i].dz ) << " "      // 30
        << filterprecision( inputLeftFootTraj_deq_[i].dtheta ) << " "  // 31
        << filterprecision( inputLeftFootTraj_deq_[i].domega ) << " "  // 32
        << filterprecision( inputLeftFootTraj_deq_[i].ddx ) << " "     // 33
        << filterprecision( inputLeftFootTraj_deq_[i].ddy ) << " "     // 34
        << filterprecision( inputLeftFootTraj_deq_[i].ddz ) << " "     // 35
        << filterprecision( inputLeftFootTraj_deq_[i].ddtheta ) << " " // 36
        << filterprecision( inputLeftFootTraj_deq_[i].ddomega ) << " " // 37

        << filterprecision( inputRightFootTraj_deq_[i].x ) << " "      // 38
        << filterprecision( inputRightFootTraj_deq_[i].y ) << " "      // 39
        << filterprecision( inputRightFootTraj_deq_[i].z ) << " "      // 40
        << filterprecision( inputRightFootTraj_deq_[i].theta ) << " "  // 41
        << filterprecision( inputRightFootTraj_deq_[i].omega ) << " "  // 42
        << filterprecision( inputRightFootTraj_deq_[i].dx ) << " "     // 43
        << filterprecision( inputRightFootTraj_deq_[i].dy ) << " "     // 44
        << filterprecision( inputRightFootTraj_deq_[i].dz ) << " "     // 45
        << filterprecision( inputRightFootTraj_deq_[i].dtheta ) << " " // 46
        << filterprecision( inputRightFootTraj_deq_[i].domega ) << " " // 47
        << filterprecision( inputRightFootTraj_deq_[i].ddx ) << " "    // 48
        << filterprecision( inputRightFootTraj_deq_[i].ddy ) << " "    // 49
        << filterprecision( inputRightFootTraj_deq_[i].ddz ) << " "    // 50
        << filterprecision( inputRightFootTraj_deq_[i].ddtheta ) << " "// 51
        << filterprecision( inputRightFootTraj_deq_[i].ddomega ) << " ";// 52

    for(unsigned int j = 0 ; j < configurationTraj_[i].size() ; j++ )
      aof << filterprecision( configurationTraj_[i](j) ) << " " ;
    for(unsigned int j = 0 ; j < velocityTraj_[i].size() ; j++ )
      aof << filterprecision( velocityTraj_[i](j) ) << " " ;
    for(unsigned int j = 0 ; j < accelerationTraj_[i].size() ; j++ )
      aof << filterprecision( accelerationTraj_[i](j) ) << " " ;
    aof << endl ;
  }
  aof.close();

  ++iteration;

  static double ecartmaxX = 0 ;
  static double ecartmaxY = 0 ;
  for (unsigned int i = 0 ; i < deltaZMP_deq_.size() ; i++ )
  {
    if ( abs(deltaZMP_deq_[i].px) > ecartmaxX )
      ecartmaxX = abs(deltaZMP_deq_[i].px) ;
    if ( abs(deltaZMP_deq_[i].py) > ecartmaxY )
      ecartmaxY = abs(deltaZMP_deq_[i].py) ;
  }

  return ;
}

void DynamicFilter::printBuffers(deque<COMState> & inputCOMTraj_deq_,
                deque<ZMPPosition> inputZMPTraj_deq_,
                deque<FootAbsolutePosition> & inputLeftFootTraj_deq_,
                deque<FootAbsolutePosition> & inputRightFootTraj_deq_,
                deque<COMState> & outputDeltaCOMTraj_deq_
                )
{
  // Debug Purpose
  // -------------
  ofstream aof;
  string aFileName;
  ostringstream oss(std::ostringstream::ate);
  static int iteration = 0;
  int iteration100 = (int)iteration/100;
  int iteration10 = (int)(iteration - iteration100*100)/10;
  int iteration1 = (int)(iteration - iteration100*100 - iteration10*10 );

  // --------------------
  oss.str("DumpingData/DynamicFilterAllVariablesAlongInTime_");
  oss << iteration100 << iteration10 << iteration1 << ".dat" ;
  aFileName = oss.str();
  aof.open(aFileName.c_str(),ofstream::out);
  aof.close();

  ///----
  aof.open(aFileName.c_str(),ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  for (unsigned int i = 0 ; i < inputCOMTraj_deq_.size() ; ++i )
  {
    aof << filterprecision( iteration*controlPeriod_ + i*interpolationPeriod_ ) << " " // 0
        << filterprecision( inputCOMTraj_deq_[i].x[0] ) << " "    // 1
        << filterprecision( inputCOMTraj_deq_[i].x[1] ) << " "    // 2
        << filterprecision( inputCOMTraj_deq_[i].x[2] ) << " "    // 3
        << filterprecision( inputCOMTraj_deq_[i].y[0] ) << " "    // 4
        << filterprecision( inputCOMTraj_deq_[i].y[1] ) << " "    // 5
        << filterprecision( inputCOMTraj_deq_[i].y[2] ) << " "    // 6
        << filterprecision( inputCOMTraj_deq_[i].z[0] ) << " "    // 7
        << filterprecision( inputCOMTraj_deq_[i].z[1] ) << " "    // 8
        << filterprecision( inputCOMTraj_deq_[i].z[2] ) << " "    // 9
        << filterprecision( inputCOMTraj_deq_[i].roll[0] ) << " " // 10
        << filterprecision( inputCOMTraj_deq_[i].roll[1] ) << " " // 11
        << filterprecision( inputCOMTraj_deq_[i].roll[2] ) << " " // 12
        << filterprecision( inputCOMTraj_deq_[i].pitch[0] ) << " "// 13
        << filterprecision( inputCOMTraj_deq_[i].pitch[1] ) << " "// 14
        << filterprecision( inputCOMTraj_deq_[i].pitch[2] ) << " "// 15
        << filterprecision( inputCOMTraj_deq_[i].yaw[0] ) << " "  // 16
        << filterprecision( inputCOMTraj_deq_[i].yaw[1] ) << " "  // 17
        << filterprecision( inputCOMTraj_deq_[i].yaw[2] ) << " "  // 18

        << filterprecision( inputZMPTraj_deq_[i].px ) << " "      // 19
        << filterprecision( inputZMPTraj_deq_[i].py ) << " "      // 20

        << filterprecision( inputLeftFootTraj_deq_[i].x ) << " "       // 21
        << filterprecision( inputLeftFootTraj_deq_[i].y ) << " "       // 22
        << filterprecision( inputLeftFootTraj_deq_[i].z ) << " "       // 23
        << filterprecision( inputLeftFootTraj_deq_[i].theta ) << " "   // 24
        << filterprecision( inputLeftFootTraj_deq_[i].omega ) << " "   // 25
        << filterprecision( inputLeftFootTraj_deq_[i].dx ) << " "      // 26
        << filterprecision( inputLeftFootTraj_deq_[i].dy ) << " "      // 27
        << filterprecision( inputLeftFootTraj_deq_[i].dz ) << " "      // 28
        << filterprecision( inputLeftFootTraj_deq_[i].dtheta ) << " "  // 29
        << filterprecision( inputLeftFootTraj_deq_[i].domega ) << " "  // 30
        << filterprecision( inputLeftFootTraj_deq_[i].ddx ) << " "     // 31
        << filterprecision( inputLeftFootTraj_deq_[i].ddy ) << " "     // 32
        << filterprecision( inputLeftFootTraj_deq_[i].ddz ) << " "     // 33
        << filterprecision( inputLeftFootTraj_deq_[i].ddtheta ) << " " // 34
        << filterprecision( inputLeftFootTraj_deq_[i].ddomega ) << " " // 35

        << filterprecision( inputRightFootTraj_deq_[i].x ) << " "       // 36
        << filterprecision( inputRightFootTraj_deq_[i].y ) << " "       // 37
        << filterprecision( inputRightFootTraj_deq_[i].z ) << " "       // 38
        << filterprecision( inputRightFootTraj_deq_[i].theta ) << " "   // 39
        << filterprecision( inputRightFootTraj_deq_[i].omega ) << " "   // 40
        << filterprecision( inputRightFootTraj_deq_[i].dx ) << " "      // 41
        << filterprecision( inputRightFootTraj_deq_[i].dy ) << " "      // 42
        << filterprecision( inputRightFootTraj_deq_[i].dz ) << " "      // 43
        << filterprecision( inputRightFootTraj_deq_[i].dtheta ) << " "  // 44
        << filterprecision( inputRightFootTraj_deq_[i].domega ) << " "  // 45
        << filterprecision( inputRightFootTraj_deq_[i].ddx ) << " "     // 46
        << filterprecision( inputRightFootTraj_deq_[i].ddy ) << " "     // 47
        << filterprecision( inputRightFootTraj_deq_[i].ddz ) << " "     // 48
        << filterprecision( inputRightFootTraj_deq_[i].ddtheta ) << " " // 49
        << filterprecision( inputRightFootTraj_deq_[i].ddomega ) << " ";// 50

    //        << filterprecision( ZMPMB_vec_[i][0] ) << " "                  // 21
    //        << filterprecision( ZMPMB_vec_[i][1] ) << " "                  // 22


    for(unsigned int j = 0 ; j < configurationTraj_[i].size() ; j++ )
      aof << filterprecision( configurationTraj_[i](j) ) << " " ;
    for(unsigned int j = 0 ; j < velocityTraj_[i].size() ; j++ )
      aof << filterprecision( velocityTraj_[i](j) ) << " " ;
    for(unsigned int j = 0 ; j < accelerationTraj_[i].size() ; j++ )
      aof << filterprecision( accelerationTraj_[i](j) ) << " " ;

    aof << endl ;
  }
  aof.close();


  ++iteration;
  return ;
}

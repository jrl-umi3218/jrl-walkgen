#include "MultiContactHirukawa.hh"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include <Eigen/SVD>
//#define VERBOSE

using namespace std ;
using namespace PatternGeneratorJRL ;
using namespace Eigen;
using namespace se3;

MultiContactHirukawa::MultiContactHirukawa(se3::Model * model):
robot_model_(model),
q_(model->nv),
dq_(model->nv),
idx_r_wrist_( findIndex(model,"RARM_JOINT5") ),
idx_l_wrist_( findIndex(model,"LARM_JOINT5") ),
idx_r_ankle_( findIndex(model,"RLEG_JOINT5") ),
idx_l_ankle_( findIndex(model,"LLEG_JOINT5") ),
Jrh_(6,model->nv),
Jlh_(6,model->nv),
Jrf_(6,model->nv),
Jlf_(6,model->nv),
J_(4*6,model->nv),
Ve_(4*6),
Vb_(4*6),
Vrf_(6),
Vlf_(6),
Vrh_(6),
Vlh_(6),
svd_ ( JacobiSVD<MatrixXd>(4*6,model->nv,ComputeThinU | ComputeThinV) )
{
  robot_data_ = new se3::Data(*model);
  n_it_ = 5;                  // number of iteration max to converge
  sampling_period_ = 0.005;   // sampling period in seconds

  q_ .fill(0.0);
  dq_.fill(0.0);

  Jrh_.fill(0.0);
  Jlh_.fill(0.0);
  Jrf_.fill(0.0);
  Jlf_.fill(0.0);
  J_  .fill(0.0);

  Ve_ .fill(0.0);
  Vb_ .fill(0.0);
  Vrf_.fill(0.0);
  Vlf_.fill(0.0);
  Vrh_.fill(0.0);
  Vlh_.fill(0.0);

  omegab_.fill(0.0);
  vb_    .fill(0.0);
  b_rh_  .fill(0.0);
  b_lh_  .fill(0.0);
  b_rf_  .fill(0.0);
  b_lf_  .fill(0.0);

  P_.fill(0.0);
  L_.fill(0.0);
  prevP_.fill(0.0);
  prevL_.fill(0.0);
  dP_   .fill(0.0);
  dL_   .fill(0.0);

  // initialize the matrices for the svd computation
  int n(6),p(model->nv) ;
  int m = min(n,p) ;
  J_U_ = MatrixXd(n,m) ;
  J_V_ = MatrixXd(m,p) ;
  J_S_ = VectorXd(m)   ;
}

MultiContactHirukawa::~MultiContactHirukawa()
{
}

int MultiContactHirukawa::InverseKinematicsOnLimbs(FootAbsolutePosition & rf,
                                                   FootAbsolutePosition & lf,
                                                   HandAbsolutePosition & rh,
                                                   HandAbsolutePosition & lh,
                                                   COMState & base)
{
  Vrh_ << rh.dx, rh.dy, rh.dz, rh.domega, rh.domega2, rh.dtheta ;
  Vlh_ << lh.dx, lh.dy, lh.dz, lh.domega, lh.domega2, lh.dtheta ;
  Vrf_ << rf.dx, rf.dy, rf.dz, rf.domega, rf.domega2, rf.dtheta ;
  Vlf_ << lf.dx, lf.dy, lf.dz, lf.domega, lf.domega2, lf.dtheta ;
  Ve_ << Vrh_,Vlh_,Vrf_,Vlf_;

  omegab_<< base.roll[1], base.pitch[1], base.yaw[1] ;
  vb_    << base.x[1], base.y[1], base.z[1] ;
  b_rh_ << rh.x - base.x[0], rh.y - base.y[0] , rh.z - base.z[0] ;
  b_lh_ << lh.x - base.x[0], lh.y - base.y[0] , lh.z - base.z[0] ;
  b_rf_ << rf.x - base.x[0], rf.y - base.y[0] , rf.z - base.z[0] ;
  b_lf_ << lf.x - base.x[0], lf.y - base.y[0] , lf.z - base.z[0] ;
  Vb_ <<  vb_, b_rh_.cross(omegab_) ,
          vb_, b_lh_.cross(omegab_) ,
          vb_, b_rf_.cross(omegab_) ,
          vb_, b_lf_.cross(omegab_) ;

  computeJacobians  (*robot_model_,*robot_data_,q_);
  getJacobian<false>(*robot_model_,*robot_data_,idx_r_wrist_,Jrh_);
  getJacobian<false>(*robot_model_,*robot_data_,idx_l_wrist_,Jlh_);
  getJacobian<false>(*robot_model_,*robot_data_,idx_r_ankle_,Jrf_);
  getJacobian<false>(*robot_model_,*robot_data_,idx_l_ankle_,Jlf_);
  J_ << Jrh_, Jlh_, Jrf_, Jlf_ ;

  svd_.compute(J_);
  J_S_ = svd_.singularValues() ;
  J_U_ = svd_.matrixU() ;
  J_V_ = svd_.matrixV() ;

  MatrixXf::Index nonzeroSingVals (0) ;
  for(MatrixXd::Index i = 0; i < J_S_.size(); i++)
  {
    if(abs(J_S_(i)) < 1e-5 )
      J_S_(i) = 0.0 ;
    else
      ++nonzeroSingVals;
  }
  VectorXd::Index diagSize ( (std::min)(J_.rows(), J_.cols()) );
  VectorXd invertedSingVals(diagSize);
  invertedSingVals.head(nonzeroSingVals) = J_S_.head(nonzeroSingVals).array().inverse();
  invertedSingVals.tail(diagSize - nonzeroSingVals).setZero();

  dq_ = J_V_.leftCols(diagSize)
      * invertedSingVals.asDiagonal()
      * J_U_.leftCols(diagSize).adjoint()
      * (Ve_ - Vb_) ;

  return 0 ;
}

int MultiContactHirukawa::ForwardMomentum()
{
  robot_data_->M.fill(0.0);
  crba(*robot_model_,*robot_data_,q_);
  robot_data_->M.triangularView<Eigen::StrictlyLower>() =
    robot_data_->M.transpose().triangularView<Eigen::StrictlyLower>();

  prevL_ = L_ ;
  L_ = robot_data_->M.block(3,robot_model_->nv,3,0) * dq_ ;
  dL_ = (L_-prevL_)/sampling_period_ ;
  return 0 ;
}

int MultiContactHirukawa::ContactWrench()
{
  return 0 ;
}

int MultiContactHirukawa::InverseMomentum()
{
  return 0 ;
}

int MultiContactHirukawa::online(vector<COMState> & comState_deque,
                                 vector<FootAbsolutePosition> & rf_deque,
                                 vector<FootAbsolutePosition> & lf_deque,
                                 vector<HandAbsolutePosition> & rh_deque,
                                 vector<HandAbsolutePosition> & lh_deque)
{
  InverseKinematicsOnLimbs(rf_deque[0],lf_deque[0],rh_deque[0],lh_deque[0],comState_deque[0]);

  return 0 ;
}


//int DetermineContact(std::vector< FootAbsolutePosition > & rf,
//                     std::vector< FootAbsolutePosition > & lf,
//                     std::vector< HandAbsolutePosition > & rh,
//                     std::vector< HandAbsolutePosition > & lh)
//{
//  contactVec_.resize(rf.size());
//  for (unsigned int i = 0 ; i < contactVec_.size() ; ++i )
//  {
//    contactVec_[i].clear();
//    if ( rf[i].z == 0.0 )
//    {
//      Contact aContact ;
//      aContact.n(0) = 0.0 ;
//      aContact.n(1) = 0.0 ;
//      aContact.n(2) = 1.0 ;

//      aContact.p(0) = rf[i].x ;
//      aContact.p(1) = rf[i].y ;
//      aContact.p(2) = rf[i].z ;
//      contactVec_[i].push_back(aContact) ;
//    }
//    if ( lf[i].z == 0.0 )
//    {
//      Contact aContact ;
//      aContact.n(0) = 0.0 ;
//      aContact.n(1) = 0.0 ;
//      aContact.n(2) = 1.0 ;

//      aContact.p(0) = lf[i].x ;
//      aContact.p(1) = lf[i].y ;
//      aContact.p(2) = lf[i].z ;
//      contactVec_[i].push_back(aContact) ;
//    }
//    if ( rh[i].stepType < 0.0 )
//    {
//      Contact aContact ;
//      aContact.n(0) = 0.0 ;
//      aContact.n(1) = 0.0 ;
//      aContact.n(2) = 1.0 ;

//      aContact.p(0) = rh[i].x ;
//      aContact.p(1) = rh[i].y ;
//      aContact.p(2) = rh[i].z ;
//      contactVec_[i].push_back(aContact) ;
//    }
//    if ( lh[i].stepType < 0.0 )
//    {
//      Contact aContact ;
//      aContact.n(0) = 0.0 ;
//      aContact.n(1) = 0.0 ;
//      aContact.n(2) = 1.0 ;

//      aContact.p(0) = lh[i].x ;
//      aContact.p(1) = lh[i].y ;
//      aContact.p(2) = lh[i].z ;
//      contactVec_[i].push_back(aContact) ;
//    }
//  }

//#ifdef VERBOSE
//  cout << "contactVec_.size() = " << contactVec_.size() << endl ;
//  for ( unsigned int i=0 ; i < contactVec_.size() ; ++i )
//  {
//    for ( unsigned int j=0 ; j < contactVec_[i].size() ; ++j )
//    {
//      cout << j << " : ["
//           << contactVec_[i][j].p(0) << " , "
//           << contactVec_[i][j].p(1) << " , "
//           << contactVec_[i][j].p(2) << "] ";
//    }
//    cout << endl ;
//  }
//#endif
//  return 0 ;
//}
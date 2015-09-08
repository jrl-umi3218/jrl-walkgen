#include "MultiContactHirukawa.hh"
#include "pinocchio/algorithm/jacobian.hpp"
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
end_effectors_vel_(4*6),
rf_vel_(6),
lf_vel_(6),
rh_vel_(6),
lh_vel_(6),
v_com  (6)
{
  robot_data_ = new se3::Data(*model);
  n_it_ = 5;                  // number of iteration max to converge
  sampling_period_ = 0.005;   // sampling period in seconds
  Jrh_.fill(0.0);
  Jlh_.fill(0.0);
  Jrf_.fill(0.0);
  Jlf_.fill(0.0);
  J_  .fill(0.0);
}

MultiContactHirukawa::~MultiContactHirukawa()
{
}

int MultiContactHirukawa::InverseKinematicsOnLimbs(FootAbsolutePosition & rf,
                                                   FootAbsolutePosition & lf,
                                                   HandAbsolutePosition & rh,
                                                   HandAbsolutePosition & lh)
{
  rf_vel_(0) = rf.dx      ; lf_vel_(0) = lf.dx      ;
  rf_vel_(1) = rf.dy      ; lf_vel_(1) = lf.dy      ;
  rf_vel_(2) = rf.dz      ; lf_vel_(2) = lf.dz      ;
  rf_vel_(3) = rf.domega  ; lf_vel_(3) = lf.domega  ;
  rf_vel_(4) = rf.domega2 ; lf_vel_(4) = lf.domega2 ;
  rf_vel_(5) = rf.dtheta  ; lf_vel_(5) = lf.dtheta  ;

  rh_vel_(0) = rh.dx      ; lh_vel_(0) = lh.dx      ;
  rh_vel_(1) = rh.dy      ; lh_vel_(1) = lh.dy      ;
  rh_vel_(2) = rh.dz      ; lh_vel_(2) = lh.dz      ;
  rh_vel_(3) = rh.domega  ; lh_vel_(3) = lh.domega  ;
  rh_vel_(4) = rh.domega2 ; lh_vel_(4) = lh.domega2 ;
  rh_vel_(5) = rh.dtheta  ; lh_vel_(5) = lh.dtheta  ;

  computeJacobians  (*robot_model_,*robot_data_,q_);
  getJacobian<false>(*robot_model_,*robot_data_,idx_r_wrist_,Jrh_);
  getJacobian<false>(*robot_model_,*robot_data_,idx_l_wrist_,Jlh_);
  getJacobian<false>(*robot_model_,*robot_data_,idx_r_ankle_,Jrf_);
  getJacobian<false>(*robot_model_,*robot_data_,idx_l_ankle_,Jlf_);
  J_ << Jrh_, Jlh_, Jrf_, Jlf_ ;

  cout << "Jrh_ = \n" <<  Jrh_ << endl ;
  cout << "Jlh_ = \n" <<  Jlh_ << endl ;
  cout << "Jrf_ = \n" <<  Jrf_ << endl ;
  cout << "Jlf_ = \n" <<  Jlf_ << endl ;
  cout << "J_   = \n" <<  J_   << endl ;

  JacobiSVD<MatrixXf> svd(J_, ComputeThinU | ComputeThinV);
  cout << "Its singular values are:" << endl << svd.singularValues() << endl;
  cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
  cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
  Vector3f rhs(1, 0, 0);
  cout << "Now consider this rhs vector:" << endl << rhs << endl;
  cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(rhs) << endl;

  return 0 ;
}

int MultiContactHirukawa::ForwardMomentum()
{
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
  InverseKinematicsOnLimbs(rf_deque[0],lf_deque[0],rh_deque[0],lh_deque[0]);

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
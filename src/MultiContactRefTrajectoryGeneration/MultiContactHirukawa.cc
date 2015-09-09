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
V_(4*6),
Vrf_(6),
Vlf_(6),
Vrh_(6),
Vlh_(6),
Vcom_(6),
svd_ ( JacobiSVD<MatrixXd>(4*6,model->nv,ComputeThinU | ComputeThinV) )
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
  Vrf_(0) = rf.dx      ; Vlf_(0) = lf.dx      ;
  Vrf_(1) = rf.dy      ; Vlf_(1) = lf.dy      ;
  Vrf_(2) = rf.dz      ; Vlf_(2) = lf.dz      ;
  Vrf_(3) = rf.domega  ; Vlf_(3) = lf.domega  ;
  Vrf_(4) = rf.domega2 ; Vlf_(4) = lf.domega2 ;
  Vrf_(5) = rf.dtheta  ; Vlf_(5) = lf.dtheta  ;

  Vrh_(0) = rh.dx      ; Vlh_(0) = lh.dx      ;
  Vrh_(1) = rh.dy      ; Vlh_(1) = lh.dy      ;
  Vrh_(2) = rh.dz      ; Vlh_(2) = lh.dz      ;
  Vrh_(3) = rh.domega  ; Vlh_(3) = lh.domega  ;
  Vrh_(4) = rh.domega2 ; Vlh_(4) = lh.domega2 ;
  Vrh_(5) = rh.dtheta  ; Vlh_(5) = lh.dtheta  ;
  V_ << Vrh_,Vlh_,Vrf_,Vlf_;

  computeJacobians  (*robot_model_,*robot_data_,q_);
  getJacobian<false>(*robot_model_,*robot_data_,idx_r_wrist_,Jrh_);
  getJacobian<false>(*robot_model_,*robot_data_,idx_l_wrist_,Jlh_);
  getJacobian<false>(*robot_model_,*robot_data_,idx_r_ankle_,Jrf_);
  getJacobian<false>(*robot_model_,*robot_data_,idx_l_ankle_,Jlf_);
  J_ << Jrh_, Jlh_, Jrf_, Jlf_ ;

//  cout << "Jrh_ = \n" <<  Jrh_ << endl ;
//  cout << "Jlh_ = \n" <<  Jlh_ << endl ;
//  cout << "Jrf_ = \n" <<  Jrf_ << endl ;
//  cout << "Jlf_ = \n" <<  Jlf_ << endl ;
//  cout << "J_   = \n" <<  J_   << endl ;

  svd_.compute(J_);
//  cout << "Its singular values are:" << endl << svd_.singularValues() << endl;
//  cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd_.matrixU() << endl;
//  cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd_.matrixV() << endl;
//  cout << "Now consider this rhs vector:" << endl << V_ << endl;
//  cout << "A least-squares solution of m*x = rhs is:" << endl << svd_.solve(V_) << endl;
//  cout << "m*x - rhs is:" << endl << J_ * svd_.solve(V_) - V_  << endl;

//  svd_.singularValues();
//  MatrixXf::Index nonzeroSingVals = svd_.nonzeroSingularValues();
//  Index diagSize = (std::min)(dec().rows(), dec().cols());
//  typename JacobiSVDType::SingularValuesType invertedSingVals(diagSize);

//  Index nonzeroSingVals = dec().nonzeroSingularValues();
//  invertedSingVals.head(nonzeroSingVals) = dec().singularValues().head(nonzeroSingVals).array().inverse();
//  invertedSingVals.tail(diagSize - nonzeroSingVals).setZero();

//  dst = dec().matrixV().leftCols(diagSize)
//      * invertedSingVals.asDiagonal()
//      * dec().matrixU().leftCols(diagSize).adjoint()
//      * rhs();


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
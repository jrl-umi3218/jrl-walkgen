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
q_(model->nq),
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

  isInitialized_ = false ;

  contacts_.resize(4);
  contacts_[RightFoot].p.fill(0.0) ;
  contacts_[RightFoot].n.fill(0.0) ;
  contacts_[RightFoot].lambda = 1.0 ; // arbitrary value, Check the paper to get more intuition about it

  contacts_[LeftFoot ].p.fill(0.0) ;
  contacts_[LeftFoot ].n.fill(0.0) ;
  contacts_[LeftFoot ].lambda = 1.0 ; // arbitrary value, Check the paper to get more intuition about it

  contacts_[RightHand].p.fill(0.0) ;
  contacts_[RightHand].n.fill(0.0) ;
  contacts_[RightHand].lambda = 0.2 ; // arbitrary value, Check the paper to get more intuition about it

  contacts_[LeftHand ].p.fill(0.0) ;
  contacts_[LeftHand ].n.fill(0.0) ;
  contacts_[LeftHand ].lambda = 0.2 ; // arbitrary value, Check the paper to get more intuition about it

  epsilons_.resize(4,0.0);

  // at first walking on flat ground
  alpha_ = 0.0;

  robot_mass_ = 0.0 ;
  for(unsigned i=0 ; i<robot_data_->mass.size() ; ++i)
    robot_mass_ += robot_data_->mass[i];
}

MultiContactHirukawa::~MultiContactHirukawa()
{
}

int MultiContactHirukawa::InverseKinematicsOnLimbs(FootAbsolutePosition & rf,
                                                   FootAbsolutePosition & lf,
                                                   HandAbsolutePosition & rh,
                                                   HandAbsolutePosition & lh)
{
  Vrh_ << rh.dx, rh.dy, rh.dz, rh.domega, rh.domega2, rh.dtheta ;
  Vlh_ << lh.dx, lh.dy, lh.dz, lh.domega, lh.domega2, lh.dtheta ;
  Vrf_ << rf.dx, rf.dy, rf.dz, rf.domega, rf.domega2, rf.dtheta ;
  Vlf_ << lf.dx, lf.dy, lf.dz, lf.domega, lf.domega2, lf.dtheta ;
  Ve_ << Vrh_,Vlh_,Vrf_,Vlf_;
  cout << "Ve_ = " << Ve_ << endl ;

  vb_    << dq_(0), dq_(1), dq_(2) ;
  omegab_<< dq_(3), dq_(4), dq_(5) ;
  b_rh_ << rh.x - q_(0), rh.y - q_(1) , rh.z - q_(2) ;
  b_lh_ << lh.x - q_(0), lh.y - q_(1) , lh.z - q_(2) ;
  b_rf_ << rf.x - q_(0), rf.y - q_(1) , rf.z - q_(2) ;
  b_lf_ << lf.x - q_(0), lf.y - q_(1) , lf.z - q_(2) ;
  Vb_ <<  vb_, b_rh_.cross(omegab_) ,
          vb_, b_lh_.cross(omegab_) ,
          vb_, b_rf_.cross(omegab_) ,
          vb_, b_lf_.cross(omegab_) ;
  cout << "Vb_ = " << Vb_ << endl ;

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

  cout << "dq=" << dq_ << endl ;
  return 0 ;
}

int MultiContactHirukawa::ForwardMomentum()
{
  robot_data_->M.fill(0.0);
  crba(*robot_model_,*robot_data_,q_);
  robot_data_->M.triangularView<Eigen::StrictlyLower>() =
    robot_data_->M.transpose().triangularView<Eigen::StrictlyLower>();

  L_ = robot_data_->M.block(3,0,3,robot_model_->nv) * dq_ ;
  if(!isInitialized_)
  {
    dL_.fill(0.0) ;
  }
  else
  {
    dL_ = (L_-prevL_)/sampling_period_ ;
  }
  prevL_ = L_ ;

  cout << "L="<<L_ << endl ;
  cout << "dL="<<dL_ << endl ;
  return 0 ;
}

int MultiContactHirukawa::ContactWrench(COMState com_ref)
{
  double nbContacts = 0.0 ;
  double n_z_sum = 0.0 ;
  double lamba_nz_sum = 0.0 ;
  vector<double> lambda_ratio (contacts_.size());
  for(unsigned i=0 ; i<contacts_.size() ; ++i)
  {
    nbContacts += contacts_[i].n.squaredNorm() ;
    n_z_sum += contacts_[i].n(2) ;
    lamba_nz_sum += contacts_[i].lambda*contacts_[i].n(2) ;
  }
  for (unsigned i=0 ; i<contacts_.size() ; ++i)
    lambda_ratio[i] = contacts_[i].n.squaredNorm() * contacts_[i].lambda / lamba_nz_sum ;

  double g = robot_model_->gravity981(2) ;
  double ddc_z = com_ref.z[2] ;
  alpha_ = 1.0-1.0/nbContacts * n_z_sum ;

  for(unsigned i=0 ; i<contacts_.size() ; ++i )
    epsilons_[i] = (1-alpha_)*robot_mass_*(ddc_z+g)*lambda_ratio[i] ;

  return 0 ;
}

int MultiContactHirukawa::InverseMomentum()
{
  return 0 ;
}

int MultiContactHirukawa::oneIteration(
    COMState & comState,  // INPUT/OUTPUT
    COMState & baseState, // INPUT
    FootAbsolutePosition & rf, // INPUT
    FootAbsolutePosition & lf, // INPUT
    HandAbsolutePosition & rh, // INPUT
    HandAbsolutePosition & lh) // INPUT
{
  InverseKinematicsOnLimbs(rf,lf,rh,lh);
  ForwardMomentum();

  // update the contacts_ vector
  if(rf.z==0.0 && lf.z==0.0)
  {
    contacts_[RightFoot].p << rf.x, rf.y, rf.z;
    contacts_[LeftFoot] .p << lf.x, lf.y, lf.z;
    contacts_[RightFoot].n << 0.0, 0.0, 1.0;
    contacts_[LeftFoot] .n << 0.0, 0.0, 1.0;
  }else if(rf.z==0.0)
  {
    contacts_[RightFoot].p << rf.x, rf.y, rf.z;
    contacts_[RightFoot].n << 0.0, 0.0, 1.0;
    contacts_[LeftFoot] .n << 0.0, 0.0, 0.0;
  }else
  {
    contacts_[LeftFoot] .p << rf.x, rf.y, rf.z;
    contacts_[LeftFoot] .n << 0.0, 0.0, 1.0;
    contacts_[RightFoot].n << 0.0, 0.0, 0.0;
  }
  ContactWrench(comState);
  isInitialized_ = true ;
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
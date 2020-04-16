#include "MultiContactHirukawa.hh"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include <Eigen/SVD>
//#define VERBOSE

using namespace std;
using namespace PatternGeneratorJRL;
using namespace Eigen;
using namespace se3;

MultiContactHirukawa::MultiContactHirukawa(se3::Model *model)
    : robot_model_(model), q_(model->nq), dq_(model->nv), dqrh_(6), dqlh_(6),
      dqrf_(6), dqlf_(6), idx_r_wrist_(findIndex(model, "RARM_JOINT5")),
      idx_l_wrist_(findIndex(model, "LARM_JOINT5")),
      idx_r_ankle_(findIndex(model, "RLEG_JOINT5")),
      idx_l_ankle_(findIndex(model, "LLEG_JOINT5")), idx_r_hip_(30),
      idx_l_hip_(24), idx_r_shoulder_(17), idx_l_shoulder_(10),
      tmpJ_(6, model->nv), Jrh_(6, 6), Jlh_(6, 6), Jrf_(6, 6), Jlf_(6, 6),
      Jrh_1_(6, 6), Jlh_1_(6, 6), Jrf_1_(6, 6), Jlf_1_(6, 6), xirf_(6),
      xilf_(6), xirh_(6), xilh_(6),
      svd_(JacobiSVD<MatrixXd>(6, 6, ComputeThinU | ComputeThinV)) {
  robot_data_ = new se3::Data(*model);
  n_it_ = 5;                // number of iteration max to converge
  sampling_period_ = 0.005; // sampling period in seconds

  q_.fill(0.0);
  dq_.fill(0.0);
  dqrh_.fill(0.0);
  dqlh_.fill(0.0);
  dqrf_.fill(0.0);
  dqlf_.fill(0.0);

  tmpJ_.fill(0.0);
  Jrh_.fill(0.0);
  Jlh_.fill(0.0);
  Jrf_.fill(0.0);
  Jlf_.fill(0.0);
  Jrh_1_.fill(0.0);
  Jlh_1_.fill(0.0);
  Jrf_1_.fill(0.0);
  Jlf_1_.fill(0.0);

  xirf_.fill(0.0);
  xilf_.fill(0.0);
  xirh_.fill(0.0);
  xilh_.fill(0.0);

  xiB_.fill(0.0);
  bXrh_ = MatrixXd::Identity(6, 6);
  bXlh_ = MatrixXd::Identity(6, 6);
  bXrf_ = MatrixXd::Identity(6, 6);
  bXlf_ = MatrixXd::Identity(6, 6);

  P_.fill(0.0);
  L_.fill(0.0);
  prevL_.fill(0.0);
  prevP_.fill(0.0);
  dL_.fill(0.0);

  // initialize the matrices for the svd computations
  U_ = MatrixXd(6, 6);
  V_ = MatrixXd(6, 6);
  S_ = VectorXd(6);

  isInitialized_ = false;

  contacts_.resize(4);
  contacts_[RightFoot].p.fill(0.0);
  contacts_[RightFoot].n.fill(0.0);
  contacts_[RightFoot].lambda =
      1.0; // arbitrary value, Check the paper to get more intuition about it

  contacts_[LeftFoot].p.fill(0.0);
  contacts_[LeftFoot].n.fill(0.0);
  contacts_[LeftFoot].lambda =
      1.0; // arbitrary value, Check the paper to get more intuition about it

  contacts_[RightHand].p.fill(0.0);
  contacts_[RightHand].n.fill(0.0);
  contacts_[RightHand].lambda =
      0.2; // arbitrary value, Check the paper to get more intuition about it

  contacts_[LeftHand].p.fill(0.0);
  contacts_[LeftHand].n.fill(0.0);
  contacts_[LeftHand].lambda =
      0.2; // arbitrary value, Check the paper to get more intuition about it

  epsilons_.resize(4, 0.0);

  // at first walking on flat ground
  alpha_ = 0.0;
  epsilon_sum_ = 0.0;
  xC_ = yC_ = zC_ = 0.0;
  TauX = TauY = 0.0;

  robot_mass_ = 0.0;
  for (unsigned i = 1; i < robot_model_->inertias.size(); ++i)
    robot_mass_ += robot_model_->inertias[i].mass();

  A_ = MatrixXd::Zero(6, 6);
  B_ = VectorXd::Zero(6);
  xiB_ = VectorXd::Zero(6);
}

MultiContactHirukawa::~MultiContactHirukawa() {}

int MultiContactHirukawa::inverseKinematicsOnLimbs(FootAbsolutePosition &rf,
                                                   FootAbsolutePosition &lf,
                                                   HandAbsolutePosition &rh,
                                                   HandAbsolutePosition &lh) {
  xirf_ << rh.dx, rh.dy, rh.dz, rh.domega, rh.domega2, rh.dtheta;
  xilf_ << lh.dx, lh.dy, lh.dz, lh.domega, lh.domega2, lh.dtheta;
  xirh_ << rf.dx, rf.dy, rf.dz, rf.domega, rf.domega2, rf.dtheta;
  xilh_ << lf.dx, lf.dy, lf.dz, lf.domega, lf.domega2, lf.dtheta;

  xiB_ = dq_.head(6);
  VectorXd brh(3);
  brh << rh.x - q_(0), rh.y - q_(1), rh.z - q_(2);
  VectorXd blh(3);
  blh << lh.x - q_(0), lh.y - q_(1), lh.z - q_(2);
  VectorXd brf(3);
  brf << rf.x - q_(0), rf.y - q_(1), rf.z - q_(2);
  VectorXd blf(3);
  blf << lf.x - q_(0), lf.y - q_(1), lf.z - q_(2);
  bXrh_.topRightCorner(3, 3) = -hat(brh);
  bXlh_.topRightCorner(3, 3) = -hat(blh);
  bXrf_.topRightCorner(3, 3) = -hat(brf);
  bXlf_.topRightCorner(3, 3) = -hat(blf);

  computeJacobians(*robot_model_, *robot_data_, q_);
  tmpJ_.fill(0.0);
  getJacobian<false>(*robot_model_, *robot_data_, idx_r_wrist_, tmpJ_);
  Jrh_ = tmpJ_.block(0, idx_r_shoulder_, 6, 6);
  getJacobian<false>(*robot_model_, *robot_data_, idx_l_wrist_, tmpJ_);
  Jlh_ = tmpJ_.block(0, idx_l_shoulder_, 6, 6);
  getJacobian<false>(*robot_model_, *robot_data_, idx_r_ankle_, tmpJ_);
  Jrf_ = tmpJ_.block(0, idx_r_hip_, 6, 6);
  getJacobian<false>(*robot_model_, *robot_data_, idx_l_ankle_, tmpJ_);
  Jlf_ = tmpJ_.block(0, idx_l_hip_, 6, 6);

  invertMatrix(Jrh_, Jrh_1_);
  invertMatrix(Jlh_, Jlh_1_);
  invertMatrix(Jrf_, Jrf_1_);
  invertMatrix(Jlf_, Jlf_1_);

  dqrh_ = Jrh_1_ * (xirf_ - bXrh_ * xiB_);
  dqlh_ = Jlh_1_ * (xilf_ - bXlh_ * xiB_);
  dqrf_ = Jrf_1_ * (xirh_ - bXrf_ * xiB_);
  dqlf_ = Jlf_1_ * (xilh_ - bXlf_ * xiB_);

  dq_.segment(idx_r_shoulder_, 6) = dqrh_;
  dq_.segment(idx_l_shoulder_, 6) = dqlh_;
  dq_.segment(idx_r_hip_, 6) = dqrf_;
  dq_.segment(idx_l_hip_, 6) = dqlf_;

  //  cout << "dq=" << dq_ << endl ;
  return 0;
}

int MultiContactHirukawa::forwardMomentum() {
  robot_data_->M.fill(0.0);
  crba(*robot_model_, *robot_data_, q_);
  robot_data_->M.triangularView<Eigen::StrictlyLower>() =
      robot_data_->M.transpose().triangularView<Eigen::StrictlyLower>();

  L_ = robot_data_->M.block(3, 0, 3, robot_model_->nv) * dq_;
  if (!isInitialized_) {
    dL_.fill(0.0);
  } else {
    dL_ = (L_ - prevL_) / sampling_period_;
  }
  //  cout << "L="<<L_ << endl ;
  //  cout << "dL="<<dL_ << endl ;
  return 0;
}

int MultiContactHirukawa::contactWrench(COMState &com_ref) {
  // compute the repartition of the forces over the contacts
  double nbContacts = 0.0;
  double n_z_sum = 0.0;
  double lamba_nz_sum = 0.0;
  vector<double> lambda_ratio(contacts_.size());
  for (unsigned i = 0; i < contacts_.size(); ++i) {
    nbContacts += contacts_[i].n.squaredNorm();
    n_z_sum += contacts_[i].n(2);
    lamba_nz_sum += contacts_[i].lambda * contacts_[i].n(2);
  }
  for (unsigned i = 0; i < contacts_.size(); ++i)
    lambda_ratio[i] =
        contacts_[i].n.squaredNorm() * contacts_[i].lambda / lamba_nz_sum;

  double g = robot_model_->gravity981(2);
  double ddc_z = com_ref.z[2];
  alpha_ = 1.0 - 1.0 / nbContacts * n_z_sum;

  epsilon_sum_ = 0.0;
  for (unsigned i = 0; i < contacts_.size(); ++i) {
    epsilons_[i] = (1 - alpha_) * robot_mass_ * (ddc_z + g) * lambda_ratio[i];
    epsilon_sum_ += epsilons_[i];
  }

  // compute the virtual contact point Pc :
  xC_ = 0.0;
  yC_ = 0.0;
  zC_ = 0.0;
  for (unsigned i = 0; i < contacts_.size(); ++i) {
    xC_ += epsilons_[i] * contacts_[i].p(0);
    yC_ += epsilons_[i] * contacts_[i].p(1);
    zC_ += epsilons_[i] * contacts_[i].p(2);
  }
  xC_ *= alpha_ / epsilon_sum_;
  yC_ *= alpha_ / epsilon_sum_;
  zC_ *= (1 - alpha_) / epsilon_sum_;

  TauX = 0.0;
  TauY = 0.0;
  for (unsigned i = 0; i < contacts_.size(); ++i) {
    TauX += epsilons_[i] * (com_ref.x[0] * contacts_[i].n(2) -
                            com_ref.z[0] * contacts_[i].n(1));
    TauY += -epsilons_[i] * (com_ref.y[0] * contacts_[i].n(2) -
                             com_ref.z[0] * contacts_[i].n(0));
  }

  double *xG = com_ref.x;
  double *yG = com_ref.y;
  double *zG = com_ref.z;

  P_(0) = prevP_(0) +
          sampling_period_ / (zG[0] - zC_) *
              (TauY - dL_(1) + robot_mass_ * (zG[2] + g) * (xG[0] - xC_));
  P_(1) = prevP_(1) -
          sampling_period_ / (zG[0] - zC_) *
              (TauX - dL_(0) - robot_mass_ * (zG[2] + g) * (yG[0] - yC_));
  P_(2) = robot_mass_ * com_ref.z[1];

  // cout << "P = \n" << P_ << endl ;
  return 0;
}

int MultiContactHirukawa::inverseMomentum() {
  A_ = robot_data_->M.block(0, 0, 6, 6) -
       robot_data_->M.block(0, idx_r_shoulder_, 6, 6) * Jrh_1_ * bXrh_;
  -robot_data_->M.block(0, idx_l_shoulder_, 6, 6) * Jlh_1_ *bXlh_;
  -robot_data_->M.block(0, idx_r_hip_, 6, 6) * Jrf_1_ *bXrf_;
  -robot_data_->M.block(0, idx_l_hip_, 6, 6) * Jlf_1_ *bXlf_;
  B_ = (VectorXd(6) << P_, L_).finished() -
       robot_data_->M.block(0, idx_r_shoulder_, 6, 6) * Jrh_1_ * xirf_ -
       robot_data_->M.block(0, idx_l_shoulder_, 6, 6) * Jlh_1_ * xilf_ -
       robot_data_->M.block(0, idx_r_hip_, 6, 6) * Jrf_1_ * xirh_ -
       robot_data_->M.block(0, idx_l_hip_, 6, 6) * Jlf_1_ * xilh_;

  invertMatrix(A_, A_1_);
  xiB_ = A_1_ * B_;
  return 0;
}

int MultiContactHirukawa::oneIteration(COMState &comState,       // INPUT/OUTPUT
                                       FootAbsolutePosition &rf, // INPUT
                                       FootAbsolutePosition &lf, // INPUT
                                       HandAbsolutePosition &rh, // INPUT
                                       HandAbsolutePosition &lh) // INPUT
{
  cout << "dq_.head(6) = " << endl << dq_.head(6) << endl;
  for (unsigned i = 0; i < 500; ++i) {
    inverseKinematicsOnLimbs(rf, lf, rh, lh);
    forwardMomentum();

    // update the contacts_ vector
    if (rf.z == 0.0 && lf.z == 0.0) {
      contacts_[RightFoot].p << rf.x, rf.y, rf.z;
      contacts_[LeftFoot].p << lf.x, lf.y, lf.z;
      contacts_[RightFoot].n << 0.0, 0.0, 1.0;
      contacts_[LeftFoot].n << 0.0, 0.0, 1.0;
    } else if (rf.z == 0.0) {
      contacts_[RightFoot].p << rf.x, rf.y, rf.z;
      contacts_[RightFoot].n << 0.0, 0.0, 1.0;
      contacts_[LeftFoot].n << 0.0, 0.0, 0.0;
    } else {
      contacts_[LeftFoot].p << rf.x, rf.y, rf.z;
      contacts_[LeftFoot].n << 0.0, 0.0, 1.0;
      contacts_[RightFoot].n << 0.0, 0.0, 0.0;
    }
    contactWrench(comState);
    inverseMomentum();

    VectorXd error = dq_.head(3) - xiB_.head(3);
    dq_.head(3) = xiB_.head(3);
    cout << "i= " << i << " ; vB = " << xiB_.head(3)(0) << " "
         << xiB_.head(3)(1) << " " << xiB_.head(3)(2)
         << " ; error = " << error(0) << " " << error(1) << " " << error(2)
         << endl;
    double precision = 1e-5;
    if (abs(error(0)) < precision && abs(error(1)) < precision &&
        abs(error(2)) < precision) {
      break;
    }
  }

  dq_.head(6) = xiB_;
  //  q_ = integrate(dq_);

  isInitialized_ = true;
  prevP_ = P_;
  prevL_ = L_;
  return 0;
}

int MultiContactHirukawa::invertMatrix(Eigen::MatrixXd &A,
                                       Eigen::MatrixXd &A_1 // Right Hand Side
) {
  svd_.compute(A);
  S_ = svd_.singularValues();
  U_ = svd_.matrixU();
  V_ = svd_.matrixV();

  MatrixXf::Index nonzeroSingVals(0);
  for (MatrixXd::Index i = 0; i < S_.size(); i++) {
    if (abs(S_(i)) < 1e-5)
      S_(i) = 0.0;
    else
      ++nonzeroSingVals;
  }
  VectorXd::Index diagSize((std::min)(A.rows(), A.cols()));
  VectorXd invertedSingVals(diagSize);
  invertedSingVals.head(nonzeroSingVals) =
      S_.head(nonzeroSingVals).array().inverse();
  invertedSingVals.tail(diagSize - nonzeroSingVals).setZero();

  A_1 = V_.leftCols(diagSize) * invertedSingVals.asDiagonal() *
        U_.leftCols(diagSize).adjoint();
  return 0;
}

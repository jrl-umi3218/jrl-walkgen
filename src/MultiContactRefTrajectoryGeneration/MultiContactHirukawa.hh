#ifndef MULTICONTACTHIRUKAWA_HH
#define MULTICONTACTHIRUKAWA_HH

#include "pinocchio/multibody/model.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <jrl/walkgen/pgtypes.hh>
#include <sstream>

namespace PatternGeneratorJRL {

// repartition of the forces on the contacts
enum end_effector_e { RightFoot, LeftFoot, RightHand, LeftHand };

struct contact_t {
  // position of contact
  Eigen::Vector3d p;
  // noraml to the contact surface
  Eigen::Vector3d n;
  // force repartition factor
  double lambda;
};
typedef contact_t contact;

class MultiContactHirukawa {
public:
  MultiContactHirukawa(se3::Model *model);

  ~MultiContactHirukawa();

  int oneIteration(COMState &comState,        // INPUT
                   FootAbsolutePosition &rf,  // INPUT
                   FootAbsolutePosition &lf,  // INPUT
                   HandAbsolutePosition &rh,  // INPUT
                   HandAbsolutePosition &lh); // INPUT

private:
  int inverseKinematicsOnLimbs(FootAbsolutePosition &rf,
                               FootAbsolutePosition &lf,
                               HandAbsolutePosition &rh,
                               HandAbsolutePosition &lh);
  int forwardMomentum();
  int contactWrench(COMState &com_ref);
  int inverseMomentum();
  int invertMatrix(Eigen::MatrixXd &A, Eigen::MatrixXd &A_1);

  se3::Model::Index findIndex(se3::Model *model, std::string name) {
    return model->existBodyName(name) ? model->getBodyId(name)
                                      : (se3::Model::Index)(model->nbody - 1);
  }

  Eigen::MatrixXd hat(Eigen::VectorXd vec) {
    assert(vec.size() == 3);
    Eigen::MatrixXd mat(3, 3);
    mat << 0.0, -vec(2), vec(1), vec(2), 0.0, -vec(0), -vec(1), vec(0), 0.0;
    return mat;
  }

protected:
  // robot model an configurations
  se3::Model *robot_model_;
  se3::Data *robot_data_;

  Eigen::VectorXd q_, dq_;
  Eigen::VectorXd dqrh_, dqlh_, dqrf_, dqlf_;

  const se3::Model::Index idx_r_wrist_, idx_l_wrist_, idx_r_ankle_,
      idx_l_ankle_;
  const se3::Model::Index idx_r_hip_, idx_l_hip_, idx_r_shoulder_,
      idx_l_shoulder_;

  unsigned int n_it_;      // number of iteration max to converge
  double sampling_period_; // sampling period in seconds

  // all the Jacobians of the end effectors :
  // right hand, left hand, right foot, left foot
  Eigen::MatrixXd tmpJ_, Jrh_, Jlh_, Jrf_, Jlf_;
  Eigen::MatrixXd Jrh_1_, Jlh_1_, Jrf_1_, Jlf_1_;

  // 3D vector to change from base to end effector frames
  Eigen::VectorXd xiB_;
  Eigen::MatrixXd bXrh_, bXlh_, bXrf_, bXlf_;

  // 6D velocity of the end effectors
  Eigen::VectorXd xirf_, xilf_, xirh_, xilh_;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
  Eigen::MatrixXd U_, V_;
  Eigen::VectorXd S_;

  // linear "P" and angular "L" momentum
  Eigen::Vector3d P_, L_, prevP_, prevL_;

  // first derivative of the angular momentum :
  Eigen::Vector3d dL_;

  // initialize the finite differentiation
  bool isInitialized_;

  // contact planned
  std::vector<contact> contacts_; // 0:rf , 1:lf , 2:rh , 3:lh

  // forces applied on the contacts
  std::vector<double> epsilons_; // 0:rf , 1:lf , 2:rh , 3:lh
  double epsilon_sum_;

  // Virtual contact point :
  double xC_, yC_, zC_;

  // CoM torques
  double TauX, TauY;

  // average slope of the terrain
  double alpha_;

  // robot global mass
  double robot_mass_;

  // A xiB = B => xiB = A_1 B      equ.(33)
  Eigen::MatrixXd A_, A_1_;
  Eigen::VectorXd B_;

  // all the Inertia of the limbs : right hand, left hand,
  // right foot, left foot
  Eigen::MatrixXd Mrh_star_, Mlh_star_, Mrf_star_, Mlf_star_;

public:
  void q(Eigen::VectorXd &q) { q_ = q; }
};

} // namespace PatternGeneratorJRL
#endif // HIRUKAWA2007_HH

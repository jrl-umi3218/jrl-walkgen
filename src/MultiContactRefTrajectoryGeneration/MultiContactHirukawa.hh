#ifndef MULTICONTACTHIRUKAWA_HH
#define MULTICONTACTHIRUKAWA_HH

#include <sstream>
#include <fstream>
#include <Eigen/Dense>
#include <jrl/walkgen/pgtypes.hh>
#include "pinocchio/multibody/model.hpp"

namespace PatternGeneratorJRL {

class MultiContactHirukawa
{
public:
    MultiContactHirukawa(se3::Model * model);

    ~MultiContactHirukawa();

    int online(std::vector<COMState> & comState_deque, // OUTPUT
               std::vector<FootAbsolutePosition> & rf_deque, // INPUT
               std::vector<FootAbsolutePosition> & lf_deque, // INPUT
               std::vector<HandAbsolutePosition> & rh_deque, // INPUT
               std::vector<HandAbsolutePosition> & lh_deque);// INPUT
    int InverseKinematicsOnLimbs(FootAbsolutePosition &rf,
                                 FootAbsolutePosition &lf,
                                 HandAbsolutePosition &rh,
                                 HandAbsolutePosition &lh,
                                 COMState &base);
    int ForwardMomentum();
    int ContactWrench();
    int InverseMomentum();

    se3::Model::Index findIndex(se3::Model * model, std::string name)
    {
      return model->existBodyName(name)?model->getBodyId(name):(se3::Model::Index)(model->nbody-1) ;
    }

private :
    //robot model an configurations
    se3::Model * robot_model_ ;
    se3::Data  * robot_data_  ;

    Eigen::VectorXd q_,dq_ ;

    const se3::Model::Index idx_r_wrist_ ;
    const se3::Model::Index idx_l_wrist_ ;
    const se3::Model::Index idx_r_ankle_ ;
    const se3::Model::Index idx_l_ankle_ ;

    unsigned int n_it_ ;                // number of iteration max to converge
    double sampling_period_ ;           // sampling period in seconds

    // all the Jacobians of the end effectors : right hand, left hand, right foot, left foot
    Eigen::MatrixXd Jrh_, Jlh_, Jrf_, Jlf_, J_;
    // 3D vector to change from base to end effector frames
    Eigen::Vector3d omegab_, vb_, b_rh_, b_lh_, b_rf_, b_lf_;
    // task of the end effector : rh, lh, rf, lf
    Eigen::VectorXd Ve_ , Vb_ ;
    // 6D velocity of the end effectors
    Eigen::VectorXd Vrf_,Vlf_,Vrh_,Vlh_ ;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd_ ;
    Eigen::MatrixXd J_U_,J_V_;
    Eigen::VectorXd J_S_;

    // linear "P" and angular "L" momentum
    Eigen::Vector3d P_,L_ , prevP_,prevL_;

    // first derivative of the momentum :
    Eigen::Vector3d dP_,dL_ ;

public :
    void q(Eigen::VectorXd & q)
    {q_ = q;}
};

}
#endif // HIRUKAWA2007_HH

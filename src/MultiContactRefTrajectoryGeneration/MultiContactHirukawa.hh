#ifndef MULTICONTACTHIRUKAWA_HH
#define MULTICONTACTHIRUKAWA_HH

#include <sstream>
#include <fstream>
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
                                 HandAbsolutePosition &lh);
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
    Eigen::VectorXd end_effectors_vel_ ; // task of the end effector : rh, lh, rf, lf
    Eigen::VectorXd rf_vel_ ;
    Eigen::VectorXd lf_vel_ ;
    Eigen::VectorXd rh_vel_ ;
    Eigen::VectorXd lh_vel_ ;
    Eigen::VectorXd v_com ;

public :
    void q(Eigen::VectorXd & q)
    {q_ = q;}
};

}
#endif // HIRUKAWA2007_HH

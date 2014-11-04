#ifndef MULTICONTACTHIRUKAWA_HH
#define MULTICONTACTHIRUKAWA_HH

#include <sstream>
#include <fstream>
#include <metapod/models/hrp2_14/hrp2_14.hh>
#include <metapod/algos/crba.hh>
#include <metapod/algos/jac_point_chain.hh>
#include "Clock.hh"

typedef double LocalFloatType;
typedef metapod::Spatial::ForceTpl<LocalFloatType> Force_HRP2_14;
typedef metapod::hrp2_14<LocalFloatType> Robot_Model;

typedef metapod::Nodes< Robot_Model, Robot_Model::BODY >::type RootNode;
typedef metapod::Nodes< Robot_Model, Robot_Model::l_wrist >::type LhandNode;
typedef metapod::Nodes< Robot_Model, Robot_Model::r_wrist >::type RhandNode;
typedef metapod::Nodes< Robot_Model, Robot_Model::LARM_LINK0 >::type LshoulderNode;
typedef metapod::Nodes< Robot_Model, Robot_Model::RARM_LINK0 >::type RshoulderNode;

typedef metapod::Nodes< Robot_Model, Robot_Model::LLEG_LINK0 >::type LhipNode;
typedef metapod::Nodes< Robot_Model, Robot_Model::RLEG_LINK0 >::type RhipNode;
typedef metapod::Nodes< Robot_Model, Robot_Model::l_ankle >::type LankleNode;
typedef metapod::Nodes< Robot_Model, Robot_Model::r_ankle >::type RankleNode;

typedef metapod::jac_point_chain < Robot_Model,
Robot_Model::l_ankle, Robot_Model::LLEG_LINK0,0,true,false> Jac_LF;
typedef metapod::jac_point_chain < Robot_Model,
Robot_Model::r_ankle, Robot_Model::RLEG_LINK0,0,true,false> Jac_RF;

typedef metapod::jac_point_chain < Robot_Model,
Robot_Model::l_wrist, Robot_Model::LARM_LINK0,0,true,false> Jac_LH;
typedef metapod::jac_point_chain < Robot_Model,
Robot_Model::r_wrist, Robot_Model::RARM_LINK0,0,true,false> Jac_RH;

class MultiContactHirukawa
{
public:
    MultiContactHirukawa();

    ~MultiContactHirukawa();

    int online();
    int loadData();
    int retrieveCoMandContact();
    int InverseKinematicsOnLimbs();
    int ForwardMomentum();
    int SolvingTheDynamics();
    int InverseMomentum();

private :
    //robot model an configurations
    Robot_Model robot_ ;
    Robot_Model::confVector q_, dq_ ;

    //reading data
    unsigned int n_samples_;            // size of data to treat
    unsigned int n_it_ ;                // number of iteration max to converge
    double sampling_period_ ;           // sampling period in seconds
    std::vector< std::vector<double> > data_ ;    // data from planning
    std::vector< Eigen::MatrixXd > v_com ;

};

#endif // HIRUKAWA2007_HH

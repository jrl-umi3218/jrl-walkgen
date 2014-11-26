#ifndef MULTICONTACTHIRUKAWA_HH
#define MULTICONTACTHIRUKAWA_HH

#include <sstream>
#include <fstream>
#include <jrl/walkgen/pgtypes.hh>
#include <metapod/models/hrp2_14/hrp2_14.hh>
#include <metapod/algos/jac_point_chain.hh>
#include <metapod/tools/jcalc.hh>
# include <metapod/algos/jac.hh>

//#include "Clock.hh"

#ifndef METAPOD_TYPEDEF
#define METAPOD_TYPEDEF
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
        Robot_Model::l_ankle, Robot_Model::BODY,0,true,false> Jac_LF;

    typedef metapod::jac_point_chain < Robot_Model,
        Robot_Model::r_ankle, Robot_Model::RLEG_LINK0,0,true,false> Jac_RF;

    typedef metapod::jac_point_chain < Robot_Model,
        Robot_Model::l_wrist, Robot_Model::LARM_LINK0,0,true,false> Jac_LH;

    typedef metapod::jac_point_chain < Robot_Model,
        Robot_Model::r_wrist, Robot_Model::RARM_LINK0,0,true,false> Jac_RH;

    typedef Eigen::Matrix<LocalFloatType, 6 * Robot_Model::NBBODIES, Robot_Model::NBDOF> Robot_Jacobian;
#endif

typedef Eigen::Matrix<LocalFloatType,6,1> vector6d ;

namespace PatternGeneratorJRL {

struct Contact_s
{
    Eigen::Matrix<LocalFloatType,3,1> p ; // position of the contact
    Eigen::Matrix<LocalFloatType,3,1> n ; // normal vector of the contact surface
};
typedef struct Contact_s Contact;

class MultiContactHirukawa
{
public:
    MultiContactHirukawa();

    ~MultiContactHirukawa();

    int online(std::vector<COMState> & comPos_,
               std::vector<FootAbsolutePosition> & rf_,
               std::vector<FootAbsolutePosition> & lf_,
               std::vector<HandAbsolutePosition> & rh_,
               std::vector<HandAbsolutePosition> & lh_);
    int InverseKinematicsOnLimbs(std::vector<FootAbsolutePosition> & rf,
                                 std::vector<FootAbsolutePosition> & lf,
                                 std::vector<HandAbsolutePosition> & rh,
                                 std::vector<HandAbsolutePosition> & lh,
                                 unsigned int currentIndex);
    int DetermineContact(std::vector<FootAbsolutePosition> & rf,
                         std::vector<FootAbsolutePosition> & lf,
                         std::vector<HandAbsolutePosition> & rh,
                         std::vector<HandAbsolutePosition> & lh);
    int ForwardMomentum();
    int SolvingTheDynamics();
    int InverseMomentum();

private :
    //robot model an configurations
    Robot_Model * robot_ ;
    Robot_Model::confVector q_, dq_ ;
    Jac_LF::Jacobian jac_LF ;
    Jac_RF::Jacobian jac_RF ;
    Jac_LH::Jacobian jac_LH ;
    Jac_RH::Jacobian jac_RH ;
    //Robot_Jacobian jacobian_ ;

    unsigned int n_it_ ;                // number of iteration max to converge
    double sampling_period_ ;           // sampling period in seconds
    std::vector< std::vector< Contact > > contactVec_ ;
    std::vector< vector6d , Eigen::aligned_allocator<vector6d> > rf_vel_ ;
    std::vector< vector6d , Eigen::aligned_allocator<vector6d> > lf_vel_ ;
    std::vector< vector6d , Eigen::aligned_allocator<vector6d> > rh_vel_ ;
    std::vector< vector6d , Eigen::aligned_allocator<vector6d> > lh_vel_ ;
    std::vector< Eigen::MatrixXd > v_com ;

public :
    void q(Robot_Model::confVector & q)
    {q_ = q;}
};

}
#endif // HIRUKAWA2007_HH

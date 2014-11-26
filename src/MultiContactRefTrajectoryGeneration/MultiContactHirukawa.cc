#include "MultiContactHirukawa.hh"
//#define VERBOSE

using namespace std ;
using namespace PatternGeneratorJRL ;
MultiContactHirukawa::MultiContactHirukawa()
{
    n_it_ = 5;                  // number of iteration max to converge
    sampling_period_ = 0.005;   // sampling period in seconds
    jac_LF = Jac_LF::Jacobian::Zero(); // init the left  foot jacobian
    jac_RF = Jac_RF::Jacobian::Zero(); // init the right foot jacobian
    jac_LH = Jac_LH::Jacobian::Zero(); // init the left  arm  jacobian
    jac_RH = Jac_RH::Jacobian::Zero();; // init the right arm  jacobian
    //jacobian_ = Robot_Jacobian::Zero();
}

MultiContactHirukawa::~MultiContactHirukawa()
{
}

int MultiContactHirukawa::InverseKinematicsOnLimbs(std::vector< FootAbsolutePosition > & rf,
                                                   std::vector< FootAbsolutePosition > & lf,
                                                   std::vector< HandAbsolutePosition > & rh,
                                                   std::vector< HandAbsolutePosition > & lh,
                                                   unsigned int currentIndex)
{
    cout << "q = \n" << q_ << endl ;
    //metapod::jac< Robot_Model>::run(*robot_, jacobian_);
    Jac_LF::run(*robot_, q_, metapod::Vector3dTpl<LocalFloatType>::Type(0,0,0), jac_LF);
    Jac_RF::run(*robot_, q_, metapod::Vector3dTpl<LocalFloatType>::Type(0,0,0), jac_RF);
    Jac_LH::run(*robot_, q_, metapod::Vector3dTpl<LocalFloatType>::Type(0,0,0), jac_LH);
    Jac_RH::run(*robot_, q_, metapod::Vector3dTpl<LocalFloatType>::Type(0,0,0), jac_RH);

    cout << "jac_LF = \n" <<  jac_LF << endl ;
    cout << "jac_RF = \n" <<  jac_RF << endl ;
    cout << "jac_LH = \n" <<  jac_LH << endl ;
    cout << "jac_RH = \n" <<  jac_RH << endl ;
    return 0 ;
}

int MultiContactHirukawa::DetermineContact(std::vector< FootAbsolutePosition > & rf,
                                           std::vector< FootAbsolutePosition > & lf,
                                           std::vector< HandAbsolutePosition > & rh,
                                           std::vector< HandAbsolutePosition > & lh)
{
    contactVec_.resize(rf.size());
    for (unsigned int i = 0 ; i < contactVec_.size() ; ++i )
    {
        contactVec_[i].clear();
        if ( rf[i].z == 0.0 )
        {
            Contact aContact ;
            aContact.n(0) = 0.0 ;
            aContact.n(1) = 0.0 ;
            aContact.n(2) = 1.0 ;

            aContact.p(0) = rf[i].x ;
            aContact.p(1) = rf[i].y ;
            aContact.p(2) = rf[i].z ;
            contactVec_[i].push_back(aContact) ;
        }
        if ( lf[i].z == 0.0 )
        {
            Contact aContact ;
            aContact.n(0) = 0.0 ;
            aContact.n(1) = 0.0 ;
            aContact.n(2) = 1.0 ;

            aContact.p(0) = lf[i].x ;
            aContact.p(1) = lf[i].y ;
            aContact.p(2) = lf[i].z ;
            contactVec_[i].push_back(aContact) ;
        }
        if ( rh[i].stepType < 0.0 )
        {
            Contact aContact ;
            aContact.n(0) = 0.0 ;
            aContact.n(1) = 0.0 ;
            aContact.n(2) = 1.0 ;

            aContact.p(0) = rh[i].x ;
            aContact.p(1) = rh[i].y ;
            aContact.p(2) = rh[i].z ;
            contactVec_[i].push_back(aContact) ;
        }
        if ( lh[i].stepType < 0.0 )
        {
            Contact aContact ;
            aContact.n(0) = 0.0 ;
            aContact.n(1) = 0.0 ;
            aContact.n(2) = 1.0 ;

            aContact.p(0) = lh[i].x ;
            aContact.p(1) = lh[i].y ;
            aContact.p(2) = lh[i].z ;
            contactVec_[i].push_back(aContact) ;
        }
    }

#ifdef VERBOSE
    cout << "contactVec_.size() = " << contactVec_.size() << endl ;
    for ( unsigned int i=0 ; i < contactVec_.size() ; ++i )
    {
        for ( unsigned int j=0 ; j < contactVec_[i].size() ; ++j )
        {
            cout << j << " : ["
                 << contactVec_[i][j].p(0) << " , "
                 << contactVec_[i][j].p(1) << " , "
                 << contactVec_[i][j].p(2) << "] ";
        }
        cout << endl ;
    }
#endif
    return 0 ;
}

int MultiContactHirukawa::ForwardMomentum()
{
    return 0 ;
}

int MultiContactHirukawa::SolvingTheDynamics()
{
    return 0 ;
}

int MultiContactHirukawa::InverseMomentum()
{
    return 0 ;
}

int MultiContactHirukawa::online(vector<COMState> & comPos_,
                                 vector<FootAbsolutePosition> & rf_,
                                 vector<FootAbsolutePosition> & lf_,
                                 vector<HandAbsolutePosition> & rh_,
                                 vector<HandAbsolutePosition> & lh_)
{
    DetermineContact(rf_,lf_,rh_,lh_);
//    InverseKinematicsOnLimbs(rf_,lf_,rh_,lh_,0);

    return 0 ;
}
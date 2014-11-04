#include "MultiContactHirukawa.hh"

using namespace std ;

MultiContactHirukawa::MultiContactHirukawa()
{
    n_samples_ = 1000;          // size of data to treat
    n_it_ = 5;                  // number of iteration max to converge
    sampling_period_ = 0.005;   // sampling period in seconds
}

MultiContactHirukawa::~MultiContactHirukawa()
{
}

int MultiContactHirukawa::loadData()
{
    data_.clear() ;
    std::string astateFile =
    "/home/mnaveau/devel/ros_unstable/src/jrl/jrl-walkgen/_build-RELEASE/tests/\
    TestMorisawa2007ShortWalk32TestFGPI.dat";
    std::ifstream dataStream ;
    dataStream.open(astateFile.c_str(),std::ifstream::in);

    // reading all the data file
    while (dataStream.good()) {
        vector<double> oneLine(74) ;
        for (unsigned int i = 0 ; i < oneLine.size() ; ++i)
            dataStream >> oneLine[i];
        data_.push_back(oneLine);
    }
    dataStream.close();

    v_com.resize(data_.size());


    return 0 ;
}

int MultiContactHirukawa::retrieveCoMandContact()
{
    return 0 ;
}

int MultiContactHirukawa::InverseKinematicsOnLimbs()
{
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

int MultiContactHirukawa::online()
{
    loadData();

    for (unsigned int i = 0 ; i < data_.size() ; ++i )
    {   for (unsigned int j = 0 ; j < data_[0].size() ; ++j )
            cout << data_[i][j] << " " ;
        cout << endl ;
    }

    return 0 ;
}
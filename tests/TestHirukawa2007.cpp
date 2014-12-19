/*
 * Copyright 2010,
 *
 * Andrei Herdt
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/* \file This file tests A. Herdt's walking algorithm for
 * automatic foot placement giving an instantaneous CoM velocity reference.
 */
#include "Debug.hh"
#include "CommonTools.hh"
#include "TestObject.hh"
#include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>
#include <ZMPRefTrajectoryGeneration/DynamicFilter.hh>
#include <metapod/models/hrp2_14/hrp2_14.hh>
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/accumulate.hpp>
#include <MultiContactRefTrajectoryGeneration/MultiContactHirukawa.hh>

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
    Robot_Model::l_ankle, Robot_Model::LLEG_LINK0,0,true,false> Jac_LF;
    typedef metapod::jac_point_chain < Robot_Model,
    Robot_Model::r_ankle, Robot_Model::RLEG_LINK0,0,true,false> Jac_RF;

    typedef metapod::jac_point_chain < Robot_Model,
    Robot_Model::l_wrist, Robot_Model::LARM_LINK0,0,true,false> Jac_LH;
    typedef metapod::jac_point_chain < Robot_Model,
    Robot_Model::r_wrist, Robot_Model::RARM_LINK0,0,true,false> Jac_RH;
#endif

typedef metapod::Nodes< Robot_Model, Robot_Model::BODY >::type RootNode;

using namespace::PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;
using namespace std;

typedef Eigen::Matrix<double,6,1> vector6d ;

class TestHirukawa2007: public TestObject
{

private:

    SimplePluginManager * SPM_ ;
    double dInitX, dInitY;
    bool once ;
    MAL_VECTOR(InitialPosition,double);
    double samplingPeriod ;

    vector<COMState> comPos_ ;
    vector< FootAbsolutePosition > rf_ ;
    vector< FootAbsolutePosition > lf_ ;
    vector< HandAbsolutePosition > rh_ ;
    vector< HandAbsolutePosition > lh_ ;
    vector<ZMPPosition> zmp_ ;

    Robot_Model robot_ ;
    Robot_Model::confVector q_init_ ;

    vector< vector<double> > data_ ;

    MultiContactHirukawa MCHpg ;

public:
    TestHirukawa2007(int argc, char *argv[], string &aString):
        TestObject(argc,argv,aString)
    {
        SPM_ = NULL ;
        once = true ;
        MAL_VECTOR_RESIZE(InitialPosition,30);
        samplingPeriod = 0.005 ;
    }

    ~TestHirukawa2007()
    {
        if ( SPM_ != 0 )
        {
            delete SPM_ ;
            SPM_ = 0 ;
        }
        m_DebugHDR = 0;
    }

    typedef void (TestHirukawa2007::* localeventHandler_t)(PatternGeneratorInterface &);

    struct localEvent
    {
        unsigned time;
        localeventHandler_t Handler ;
    };

    bool doTest(ostream &os)
    {
        metapod::jcalc<Robot_Model>::run(robot_,q_init_,Robot_Model::confVector::Zero());
        metapod::bcalc<Robot_Model>::run(robot_,q_init_);

        ofstream aof;
        string aFileName;
        ostringstream oss(std::ostringstream::ate);
        oss.str("/home/mnaveau/devel/matlab_scripts/step_generator/testMetapod.txt");
        aFileName = oss.str();
        aof.open(aFileName.c_str(),ofstream::out);
        aof.close();

        boost::fusion::for_each(robot_.nodes ,  print_iXo() );

        double sum_mass = 0.0 ;
        metapod::Vector3dTpl< LocalFloatType >::Type com (0.0,0.0,0.0);

        sum_mass = boost::fusion::accumulate(robot_.nodes , sum_mass , MassSum() );
        com      = boost::fusion::accumulate(robot_.nodes , com      , MassbyComSum() );
        cout << "mass * com = \n" << com << endl ;
        cout << "mass = \n" << sum_mass << endl ;
        com      = com / sum_mass ;
        cout << "com = \n" << com << endl ;
















        data_.clear() ;
        std::string astateFile =
"/home/mnaveau/devel/ros_unstable/src/jrl/jrl-walkgen/_build-RELEASE/tests/TestMorisawa2007ShortWalk32TestFGPI.dat" ;
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

        comPos_.resize(data_.size()) ;
        rf_.resize(data_.size()) ;
        lf_.resize(data_.size()) ;
        rh_.resize(data_.size()) ;
        lh_.resize(data_.size()) ;
        zmp_   .resize(data_.size()) ;

        for (unsigned int i = 0 ; i < data_.size() ; ++i)
        {
            comPos_[i].x[0] = data_[i][1] ;
            comPos_[i].y[0] = data_[i][2] ;
            comPos_[i].z[0] = data_[i][3] ;
            comPos_[i].yaw[0] = data_[i][4] ;
            comPos_[i].x[1] = data_[i][5] ;
            comPos_[i].y[1] = data_[i][6] ;
            comPos_[i].z[1] = data_[i][7] ;

            rf_[i].x      = data_[i][22] ;
            rf_[i].y      = data_[i][23] ;
            rf_[i].z      = data_[i][24] ;
            rf_[i].omega  = 0.0 ;
            rf_[i].omega2 = 0.0 ;
            rf_[i].theta  = 0.0 ;

            rf_[i].dx      = data_[i][25] ;
            rf_[i].dy      = data_[i][26] ;
            rf_[i].dz      = data_[i][27] ;
            rf_[i].domega  = 0.0 ;
            rf_[i].domega2 = 0.0 ;
            rf_[i].dtheta  = 0.0 ;

            lf_[i].x      = data_[i][10] ;
            lf_[i].y      = data_[i][11] ;
            lf_[i].z      = data_[i][12] ;
            lf_[i].omega  = 0.0 ;
            lf_[i].omega2 = 0.0 ;
            lf_[i].theta  = 0.0 ;

            lf_[i].dx      = data_[i][13] ;
            lf_[i].dy      = data_[i][14] ;
            lf_[i].dz      = data_[i][15] ;
            lf_[i].domega  = 0.0 ;
            lf_[i].domega2 = 0.0 ;
            lf_[i].dtheta  = 0.0 ;

            rh_[i].dx      = data_[i][4] ;
            rh_[i].dy      = data_[i][5] ;
            rh_[i].dz      = data_[i][6] ;
            rh_[i].domega  = 0.0 ;
            rh_[i].domega2 = 0.0 ;
            rh_[i].dtheta  = 0.0 ;
            rh_[i].stepType  = 1.0 ;


            lh_[i].dx      = data_[i][4] ;
            lh_[i].dy      = data_[i][5] ;
            lh_[i].dz      = data_[i][6] ;
            lh_[i].domega  = 0.0 ;
            lh_[i].domega2 = 0.0 ;
            lh_[i].dtheta  = 0.0 ;
            lh_[i].stepType  = 1.0 ;
        }

        MCHpg.online(comPos_,rf_,lf_,rh_,lh_) ;

        vector<vector<int> > test_vector ;
        test_vector.clear();
        unsigned int dimension = 5 ;
        test_vector.resize(dimension);
        for (unsigned int i = 0 ; i < dimension ; ++i)
        {
            test_vector[i].clear();
            for (unsigned int j = 0 ; j < i ; ++j)
            {
                int nbr = j ;
                test_vector[i].push_back(nbr);
            }
        }


        for (unsigned int i = 0 ; i < test_vector.size() ; ++i)
        {
            for (unsigned int j = 0 ; j < test_vector[i].size() ; ++j)
            {
                cout << test_vector[i][j] << " ";
            }
            cout << endl ;
        }


        //fillInDebugFiles();
        return true ;
    }

    struct print_iXo
    {
        template <typename T>
        void operator()(T & x) const
        {
            ofstream aof;
            string aFileName;
            ostringstream oss(std::ostringstream::ate);
            oss.str("/home/mnaveau/devel/matlab_scripts/step_generator/testMetapod.txt");
            aFileName = oss.str();
            aof.open(aFileName.c_str(),ofstream::app);
            aof.precision(7);
            aof.setf(ios::fixed, ios::floatfield);
            aof << Robot_Model::inertias[x.id] ;
        }
    };

    struct MassSum
    {
        typedef LocalFloatType result_type;

        template <typename T>
        result_type operator()(const T & t , const result_type & sum_mass ) const
        {
            return ( sum_mass + Robot_Model::inertias[t.id].m() ) ;
        }

        template <typename T>
        result_type operator()(const result_type & sum_mass , const T & t ) const
        {
            return ( sum_mass + Robot_Model::inertias[t.id].m() ) ;
        }
    };

    struct MassbyComSum
    {
        typedef metapod::Vector3dTpl< LocalFloatType >::Type result_type;

        template <typename T>
        result_type operator()(const T & t , const result_type & sum_h ) const
        {
            double mass = Robot_Model::inertias[t.id].m() ;
            return ( sum_h + mass * t.body.iX0.r() + t.body.iX0.E() * Robot_Model::inertias[t.id].h() );
        }
        template <typename T>
        result_type operator()(const result_type & sum_h , const T & t ) const
        {
            double mass = Robot_Model::inertias[t.id].m() ;
            return ( sum_h + mass * t.body.iX0.r() + t.body.iX0.E() * Robot_Model::inertias[t.id].h() );
        }
    };

    void init()
    {
        // Instanciate and initialize.
        string RobotFileName = m_VRMLPath + m_VRMLFileName;

        bool fileExist = false;
        {
            std::ifstream file (RobotFileName.c_str ());
            fileExist = !file.fail ();
        }
        if (!fileExist)
            throw std::string ("failed to open robot model");

        // Creating the humanoid robot.
        SpecializedRobotConstructor(m_HDR);
        if(m_HDR==0)
        {
            if (m_HDR!=0) delete m_HDR;
            dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
            m_HDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
        }
        // Parsing the file.
        dynamicsJRLJapan::parseOpenHRPVRMLFile(*m_HDR,RobotFileName,
                                               m_LinkJointRank,
                                               m_SpecificitiesFileName);
        // Create Pattern Generator Interface
        m_PGI = patternGeneratorInterfaceFactory(m_HDR);

        unsigned int lNbActuatedJoints = 30;
        double * dInitPos = new double[lNbActuatedJoints];
        ifstream aif;
        aif.open(m_InitConfig.c_str(),ifstream::in);
        if (aif.is_open())
        {
            for(unsigned int i=0;i<lNbActuatedJoints;i++)
                aif >> dInitPos[i];
        }
        aif.close();

        bool DebugConfiguration = true;
        ofstream aofq;
        if (DebugConfiguration)
        {
            aofq.open("TestConfiguration.dat",ofstream::out);
            if (aofq.is_open())
            {
                for(unsigned int k=0;k<30;k++)
                {
                    aofq << dInitPos[k] << " ";
                }
                aofq << endl;
            }

        }

        // This is a vector corresponding to the DOFs actuated of the robot.
        bool conversiontoradneeded=true;
        if (conversiontoradneeded)
            for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
                InitialPosition(i) = dInitPos[i]*M_PI/180.0;
        else
            for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
                InitialPosition(i) = dInitPos[i];

        q_init_(0) = 0.0 ;
        q_init_(1) = 0.0 ;
        q_init_(2) = 0.6487 ;
        q_init_(3) = 0.0 ;
        q_init_(4) = 0.0 ;
        q_init_(5) = 0.0 ;
        for(unsigned int i=0 ; i<MAL_VECTOR_SIZE(InitialPosition) ; i++)
            q_init_(i+6,0) = InitialPosition(i) ;

        MCHpg.q(q_init_) ;

        // This is a vector corresponding to ALL the DOFS of the robot:
        // free flyer + actuated DOFS.
        unsigned int lNbDofs = 36 ;
        MAL_VECTOR_DIM(CurrentConfiguration,double,lNbDofs);
        MAL_VECTOR_DIM(CurrentVelocity,double,lNbDofs);
        MAL_VECTOR_DIM(CurrentAcceleration,double,lNbDofs);
        MAL_VECTOR_DIM(PreviousConfiguration,double,lNbDofs) ;
        MAL_VECTOR_DIM(PreviousVelocity,double,lNbDofs);
        MAL_VECTOR_DIM(PreviousAcceleration,double,lNbDofs);
        for(int i=0;i<6;i++)
        {
            PreviousConfiguration[i] = PreviousVelocity[i] = PreviousAcceleration[i] = 0.0;
        }

        for(unsigned int i=6;i<lNbDofs;i++)
        {
            PreviousConfiguration[i] = InitialPosition[i-6];
            PreviousVelocity[i] = PreviousAcceleration[i] = 0.0;
        }

        delete [] dInitPos;

        MAL_VECTOR_RESIZE(m_CurrentConfiguration, m_HDR->numberDof());
        MAL_VECTOR_RESIZE(m_CurrentVelocity, m_HDR->numberDof());
        MAL_VECTOR_RESIZE(m_CurrentAcceleration, m_HDR->numberDof());

        MAL_VECTOR_RESIZE(m_PreviousConfiguration, m_HDR->numberDof());
        MAL_VECTOR_RESIZE(m_PreviousVelocity, m_HDR->numberDof());
        MAL_VECTOR_RESIZE(m_PreviousAcceleration, m_HDR->numberDof());

        SPM_ = new SimplePluginManager();
    }

protected:

    void chooseTestProfile()
    {return;}
    void generateEvent()
    {return;}

    void SpecializedRobotConstructor(CjrlHumanoidDynamicRobot *& aHDR)
    {
        dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
        Chrp2OptHumanoidDynamicRobot *aHRP2HDR = new Chrp2OptHumanoidDynamicRobot( &aRobotDynamicsObjectConstructor );
        aHDR = aHRP2HDR;
    }

//    void fillInDebugFiles( )
//    {
//        /// \brief Create file .hip .pos .zmp
//        /// --------------------
//        ofstream aof;
//        string aFileName;
//        static int iteration = 0 ;

//        if ( iteration == 0 ){
//            aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
//            aFileName+=m_TestName;
//            aFileName+=".pos";
//            aof.open(aFileName.c_str(),ofstream::out);
//            aof.close();
//        }
//        ///----
//        aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
//        aFileName+=m_TestName;
//        aFileName+=".pos";
//        aof.open(aFileName.c_str(),ofstream::app);
//        aof.precision(8);
//        aof.setf(ios::scientific, ios::floatfield);
//        aof << filterprecision( iteration * 0.005 ) << " "  ; // 1
//        for(unsigned int i = 6 ; i < m_CurrentConfiguration.size() ; i++){
//            aof << filterprecision( m_CurrentConfiguration(i) ) << " "  ; // 2
//        }
//        for(unsigned int i = 0 ; i < 9 ; i++){
//            aof << 0.0 << " "  ;
//        }
//        aof << 0.0  << endl ;
//        aof.close();

//        if ( iteration == 0 ){
//            aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
//            aFileName+=m_TestName;
//            aFileName+=".hip";
//            aof.open(aFileName.c_str(),ofstream::out);
//            aof.close();
//        }
//        aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
//        aFileName+=m_TestName;
//        aFileName+=".hip";
//        aof.open(aFileName.c_str(),ofstream::app);
//        aof.precision(8);
//        aof.setf(ios::scientific, ios::floatfield);
//        aof << filterprecision( iteration * 0.005 ) << " "  ; // 1
//        aof << filterprecision( m_OneStep.finalCOMPosition.roll[0] * M_PI /180) << " "  ; // 2
//        aof << filterprecision( m_OneStep.finalCOMPosition.pitch[0] * M_PI /180 ) << " "  ; // 3
//        aof << filterprecision( m_OneStep.finalCOMPosition.yaw[0] * M_PI /180 ) ; // 4
//        aof << endl ;
//        aof.close();

//        if ( iteration == 0 ){
//            aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
//            aFileName+=m_TestName;
//            aFileName+=".zmp";
//            aof.open(aFileName.c_str(),ofstream::out);
//            aof.close();
//        }

//        FootAbsolutePosition aSupportState;
//        if (m_OneStep.LeftFootPosition.stepType < 0 )
//            aSupportState = m_OneStep.LeftFootPosition ;
//        else
//            aSupportState = m_OneStep.RightFootPosition ;

//        aFileName = "/opt/grx3.0/HRP2LAAS/etc/mnaveau/";
//        aFileName+=m_TestName;
//        aFileName+=".zmp";
//        aof.open(aFileName.c_str(),ofstream::app);
//        aof.precision(8);
//        aof.setf(ios::scientific, ios::floatfield);
//        aof << filterprecision( iteration * 0.005 ) << " "  ; // 1
//        aof << filterprecision( m_OneStep.ZMPTarget(0) - m_CurrentConfiguration(0)) << " "  ; // 2
//        aof << filterprecision( m_OneStep.ZMPTarget(1) - m_CurrentConfiguration(1) ) << " "  ; // 3
//        aof << filterprecision( aSupportState.z  - m_CurrentConfiguration(2))  ; // 4
//        aof << endl ;
//        aof.close();

//        aFileName = "/opt/grx3.0/HRP2LAAS/log/mnaveau/";
//        aFileName+="footpos";
//        aFileName+=".dat";
//        aof.open(aFileName.c_str(),ofstream::app);
//        aof.precision(8);
//        aof.setf(ios::scientific, ios::floatfield);
//        aof << filterprecision( iteration * 0.005 ) << " "  ; // 1
//        aof << filterprecision( lfFoot[iteration].x ) << " "  ; // 2
//        aof << filterprecision( lfFoot[iteration].y ) << " "  ; // 3
//        aof << endl ;
//        aof.close();

//        iteration++;
//    }

    void SpecializedRobotConstructor(   CjrlHumanoidDynamicRobot *& aHDR, CjrlHumanoidDynamicRobot *& aDebugHDR )
    {
        dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
        Chrp2OptHumanoidDynamicRobot *aHRP2HDR = new Chrp2OptHumanoidDynamicRobot( &aRobotDynamicsObjectConstructor );
        aHDR = aHRP2HDR;
        aDebugHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
    }

    double filterprecision(double adb)
    {
        if (fabs(adb)<1e-7)
            return 0.0;

        double ladb2 = adb * 1e7;
        double lintadb2 = trunc(ladb2);
        return lintadb2/1e7;
    }
};

int PerformTests(int argc, char *argv[])
{
#define NB_PROFILES 1
    std::string TestNames = "TestHirukawa2007" ;

    TestHirukawa2007 aTH2007(argc,argv,TestNames);
    aTH2007.init();
    try{
        if (!aTH2007.doTest(std::cout)){
            cout << "Failed test " << endl;
            return -1;
        }
        else
            cout << "Passed test " << endl;
    }
    catch (const char * astr){
        cerr << "Failed on following error " << astr << std::endl;
        return -1;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    try
    {
        int ret = PerformTests(argc,argv);
        return ret ;
    }
    catch (const std::string& msg)
    {
        std::cerr << msg << std::endl;
    }
    return 1;
}




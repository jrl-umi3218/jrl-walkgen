/*
 * Copyright 2010,
 *
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
/* \file Abstract Object test aim at testing various walking algorithms
 * Olivier Stasse
 */

#include <fstream>
#include "Debug.hh"
#include "TestObject.hh"
#include "pinocchio/multibody/parser/urdf.hpp"
#include "pinocchio/multibody/parser/srdf.hpp"
using namespace std;

#define NB_OF_FIELDS 39

#ifdef WIN32
double trunc (double x)
{
  return x < 0 ? ceil (x) : 0 < x ? floor (x) : x;
}
#endif /* WIN32 */

namespace PatternGeneratorJRL
{
  namespace TestSuite
  {
    double filterprecision(double adb)
    {
      if (fabs(adb)<1e-7)
        return 0.0;

      double ladb2 = adb * 1e7;
      double lintadb2 = trunc(ladb2);
      return lintadb2/1e7;
    }


    TestObject::TestObject(int argc, char *argv[],
                           string &aTestName,
                           int lPGIInterface)
    {
      m_TestName = aTestName;
      m_PGIInterface = lPGIInterface;
      m_OuterLoopNbItMax = 1;

      /*! default debug output */
      m_DebugFGPI = true;
      m_DebugZMP2 = false;
      m_TestProfile = 0 ;

      /*! Extract options and fill in members. */
      getOptions(argc,argv,
                 m_URDFPath,
                 m_SRDFPath,
                 m_TestProfile);

      m_PR = 0 ;
      m_robotData = 0 ;
      m_DebugPR = 0 ;
      m_DebugRobotData = 0 ;
      m_PGI = 0 ;
    }

    bool TestObject::checkFiles()
    {
      // Checking the files
      bool fileExist = false;
      bool correctExtension = false ;
      // check if URDF file exist
      {
        fileExist=false;
        std::ifstream file (m_URDFPath.c_str ());
        fileExist = !file.fail ();
        if (!fileExist)
          throw std::string ("failed to open robot urdf model");
      }
      // check if SRDF file exist
      {
        fileExist=false;
        std::ifstream file (m_SRDFPath.c_str ());
        fileExist = !file.fail ();
        if (!fileExist)
          throw std::string ("failed to open robot srdf model");
      }
      // check if file has .urdf extension
      {
        correctExtension = false ;
        std::size_t found = m_URDFPath.find_last_of('.');
        correctExtension = (m_URDFPath.substr(found) == ".urdf") ;
        if(!correctExtension)
          throw std::string("File is not an urdf, extension has to be .urdf");
      }
      // check if file has .srdf extension
      {
        correctExtension = false ;
        std::size_t found = m_SRDFPath.find_last_of('.');
        correctExtension = (m_SRDFPath.substr(found) == ".srdf") ;
        if(!correctExtension)
          throw std::string("File is not an srdf, extension has to be .srdf");
      }
      return correctExtension && fileExist ;
    }

    bool TestObject::init()
    {
      try{
        checkFiles();
      }catch(std::string e)
      {
        std::cout << "file problem, existance or extension incorrect"
                  << std::endl ;
        std::cout << e << std::endl ;
        return false;
      }

      // Instanciate and initialize.
      CreateAndInitializeHumanoidRobot(m_URDFPath,m_SRDFPath,m_PR,m_DebugPR);

      // Create Pattern Generator Interface
      m_PGI = patternGeneratorInterfaceFactory(m_PR);
      m_PGI->SetCurrentJointValues(m_HalfSitting);

      // Specify the walking mode: here the default one.
      istringstream strm2(":walkmode 0");
      m_PGI->ParseCmd(strm2);

      // This is a vector corresponding to ALL the DOFS of the robot:
      // free flyer + actuated DOFS.
      unsigned lNbDofs = m_PR->numberDof();
      MAL_VECTOR_RESIZE(m_CurrentConfiguration ,lNbDofs);
      MAL_VECTOR_RESIZE(m_CurrentVelocity      ,lNbDofs);
      MAL_VECTOR_RESIZE(m_CurrentAcceleration  ,lNbDofs);
      MAL_VECTOR_RESIZE(m_PreviousConfiguration,lNbDofs);
      MAL_VECTOR_RESIZE(m_PreviousVelocity     ,lNbDofs);
      MAL_VECTOR_RESIZE(m_PreviousAcceleration ,lNbDofs);
      for(int i=0;i<6;i++)
      {
        m_PreviousConfiguration[i] = 0.0 ;
        m_PreviousVelocity[i] = 0.0 ;
        m_PreviousAcceleration[i] = 0.0;
      }

      for(unsigned int i=6;i<lNbDofs;i++)
      {
        m_PreviousConfiguration[i] = m_HalfSitting[i-6];
        m_PreviousVelocity[i] = 0.0 ;
        m_PreviousAcceleration[i] = 0.0;
      }
      return true ;
    }

    TestObject::~TestObject()
    {

      if (m_PR!=0)
        delete m_PR;

      if (m_DebugPR!=0)
        delete m_DebugPR;

      if (m_robotData!=0)
        delete m_robotData;

      if (m_DebugRobotData!=0)
        delete m_DebugRobotData;

      if (m_PGI!=0)
        delete m_PGI;
    }

    void TestObject::CreateAndInitializeHumanoidRobot(
        std::string &URDFFile,
        std::string &SRDFFile,
        PinocchioRobot *& aPR,
        PinocchioRobot *& aDebugPR)
    {
      // Creating the humanoid robot via the URDF.
//      try{
        m_robotModel = se3::urdf::buildModel(URDFFile, se3::JointModelFreeFlyer());
        m_robotData = new se3::Data(m_robotModel) ;
        m_DebugRobotData = new se3::Data(m_robotModel) ;
//      }catch(std::invalid_argument e)
//      {
//        cout << e.what() ;
//        cout << "robot model or robot data not created properly" << endl ;
//        return ;
//      }

      if ((aPR==0) || (aDebugPR==0))
      {
        if (aPR!=0) delete aPR;
        if (aDebugPR!=0) delete aDebugPR;

        aPR = new PinocchioRobot();
        aDebugPR = new PinocchioRobot();
      }

      // initialize the model and data of the humanoid robot
      aPR->initializeRobotModelAndData(&m_robotModel,m_robotData);
      aDebugPR->initializeRobotModelAndData(&m_robotModel,m_DebugRobotData);

      // Parsing the SRDF file to initialize
      // the starting configuration and the robot specifities
      InitializeRobotWithSRDF(*aPR,SRDFFile);
      InitializeRobotWithSRDF(*aDebugPR,SRDFFile);
    }

    void TestObject::InitializeRobotWithSRDF(PinocchioRobot & aPR,
                                             const std::string & filename)
    {
      // manage the SRDF file
      //////////////////////////////////
      std::ifstream srdf_stream(filename.c_str());
      if (! srdf_stream.is_open())
      {
        const std::string exception_message (filename + " does not seem to be a valid file.");
        cerr << exception_message << endl ;
        throw std::invalid_argument(exception_message);
      }
      // Read xml stream
      using boost::property_tree::ptree;
      ptree pt;
      try{
        read_xml(srdf_stream, pt);
      }catch(...)
      {
        cerr << "problem while reading the srdf file. File corrupted?" << endl;
        return ;
      }

      // Get the starting configuration : half sitting
      MAL_VECTOR_RESIZE(m_HalfSitting,aPR.numberDof()-6);
      MAL_VECTOR_FILL(m_HalfSitting,0.0);
      se3::Model * aModel = aPR.Model();
      BOOST_FOREACH(const ptree::value_type & v, pt.get_child("robot.group_state"))
      {
        if(v.first=="joint")
        {
          const std::string jointName =
              v.second.get<std::string>("<xmlattr>.name");
          const double jointValue =
              v.second.get<double>("<xmlattr>.value");
          if(aModel->existJointName(jointName))
          {
            se3::JointIndex id = aModel->getJointId(jointName);
            unsigned idq = se3::idx_q(aModel->joints[id]);
            // we assume only revolute joint here.
            m_HalfSitting(idq-7) = jointValue ;
          }
        }
      } // BOOST_FOREACH
      bool DebugConfiguration = true;
      if (DebugConfiguration)
      {
        ofstream aofq;
        aofq.open("TestConfiguration.dat",ofstream::out);
        if (aofq.is_open())
        {
          for(unsigned int k=0;k<MAL_VECTOR_SIZE(m_HalfSitting);k++)
          {
            aofq << m_HalfSitting(k) << " ";
          }
          aofq << endl;
        }

      }

      // capture the details of the feet
      //////////////////////////////////

      // Initialize the Right Foot
      PRFoot aFoot ;
      string path = "robot.specificities.feet.right.size" ;
      BOOST_FOREACH(const ptree::value_type & v, pt.get_child(path.c_str()))
      {
        aFoot.soleHeight = v.second.get<double>("height");
        aFoot.soleWidth  = v.second.get<double>("width");
        aFoot.soleDepth  = v.second.get<double>("depth");
      }
      path = "robot.specificities.feet.right.anklePosition" ;
      BOOST_FOREACH(const ptree::value_type & v, pt.get_child(path.c_str()))
      {
        aFoot.anklePosition(0) = v.second.get<double>("x");
        aFoot.anklePosition(1) = v.second.get<double>("y");
        aFoot.anklePosition(2) = v.second.get<double>("z");
      }
      aFoot.associatedAnkle = aModel->getBodyId("r_ankle");
      aPR.initializeRightFoot(aFoot);

      // Initialize the Left Foot
      path = "robot.specificities.feet.left.size" ;
      BOOST_FOREACH(const ptree::value_type & v, pt.get_child(path.c_str()))
      {
        aFoot.soleHeight = v.second.get<double>("height");
        aFoot.soleWidth  = v.second.get<double>("width");
        aFoot.soleDepth  = v.second.get<double>("depth");
      }
      path = "robot.specificities.feet.left.anklePosition" ;
      BOOST_FOREACH(const ptree::value_type & v, pt.get_child(path.c_str()))
      {
        aFoot.anklePosition(0) = v.second.get<double>("x");
        aFoot.anklePosition(1) = v.second.get<double>("y");
        aFoot.anklePosition(2) = v.second.get<double>("z");
      }
      aFoot.associatedAnkle = aModel->getBodyId("l_ankle");
      aPR.initializeLeftFoot(aFoot);
    }

    void TestObject::prepareDebugFiles()
    {

      if (m_DebugZMP2)
      {
        ofstream aofzmpmb2;
        string aFileName = m_TestName;
        aFileName += "ZMPMBSTAGE2.dat";
        aofzmpmb2.open(aFileName.c_str(),ofstream::out);
      }


      if (m_DebugFGPI)
      {
        ofstream aof;
        string aFileName = m_TestName;
        aFileName += "TestFGPI_description.dat";

        aof.open(aFileName.c_str(),ofstream::out);

        string Titles[NB_OF_FIELDS] =
        { "Time",
          "Com X",
          "Com Y" ,
          "Com Z" ,
          "Com Yaw",
          "Com dX" ,
          "Com dY" ,
          "Com dZ" ,
          "ZMP X (world ref.)" ,
          "ZMP Y (world ref.)" ,
          "Left Foot X" ,
          "Left Foot Y" ,
          "Left Foot Z" ,
          "Left Foot dX" ,
          "Left Foot dY" ,
          "Left Foot dZ" ,
          "Left Foot ddX" ,
          "Left Foot ddY" ,
          "Left Foot ddZ" ,
          "Left Foot Theta" ,
          "Left Foot Omega" ,
          "Left Foot Omega2" ,
          "Right Foot X" ,
          "Right Foot Y" ,
          "Right Foot Z" ,
          "Right Foot dX" ,
          "Right Foot dY" ,
          "Right Foot dZ" ,
          "Right Foot ddX" ,
          "Right Foot ddY" ,
          "Right Foot ddZ" ,
          "Right Foot Theta" ,
          "Right Foot Omega" ,
          "Right Foot Omega2" ,
          "ZMP X (waist ref.)" ,
          "ZMP Y (waist ref.)" ,
          "Waist X (world ref.)" ,
          "Waist Y (world ref.)" ,
          "all configuration vector"};
        for(unsigned int i=0;i<NB_OF_FIELDS;i++)
          aof << i+1 << ". " <<Titles[i] <<std::endl;

        aof.close();

        aFileName = m_TestName;
        aFileName += "TestFGPI.dat";
        aof.open(aFileName.c_str(),ofstream::out);
        aof.close();
      }
    }


    void TestObject::fillInDebugFiles( )
    {
      if (m_DebugFGPI)
      {
        double localZMPx = m_OneStep.ZMPTarget(0)*cos(m_CurrentConfiguration(5)) -
            m_OneStep.ZMPTarget(1)*sin(m_CurrentConfiguration(5)) +
            m_CurrentConfiguration(0) ;
        double localZMPy = m_OneStep.ZMPTarget(0)*sin(m_CurrentConfiguration(5)) +
            m_OneStep.ZMPTarget(1)*cos(m_CurrentConfiguration(5)) +
            m_CurrentConfiguration(1) ;

        ofstream aof;
        string aFileName;
        aFileName = m_TestName;
        aFileName += "TestFGPI.dat";
        aof.open(aFileName.c_str(),ofstream::app);
        aof.precision(8);
        aof.setf(ios::scientific, ios::floatfield);
        aof << filterprecision(m_OneStep.NbOfIt*0.005 ) << " "                            // 1
            << filterprecision(m_OneStep.finalCOMPosition.x[0] ) << " "                   // 2
            << filterprecision(m_OneStep.finalCOMPosition.y[0] ) << " "                   // 3
            << filterprecision(m_OneStep.finalCOMPosition.z[0] ) << " "                   // 4
            << filterprecision(m_OneStep.finalCOMPosition.yaw[0] ) << " "                 // 5
            << filterprecision(m_OneStep.finalCOMPosition.x[1] ) << " "                   // 6
            << filterprecision(m_OneStep.finalCOMPosition.y[1] ) << " "                   // 7
            << filterprecision(m_OneStep.finalCOMPosition.z[1] ) << " "                   // 8
            << filterprecision(m_OneStep.ZMPTarget(0) ) << " "                            // 9
            << filterprecision(m_OneStep.ZMPTarget(1) ) << " "                            // 10
            << filterprecision(m_OneStep.LeftFootPosition.x  ) << " "                     // 11
            << filterprecision(m_OneStep.LeftFootPosition.y  ) << " "                     // 12
            << filterprecision(m_OneStep.LeftFootPosition.z  ) << " "                     // 13
            << filterprecision(m_OneStep.LeftFootPosition.dx  ) << " "                    // 14
            << filterprecision(m_OneStep.LeftFootPosition.dy  ) << " "                    // 15
            << filterprecision(m_OneStep.LeftFootPosition.dz  ) << " "                    // 16
            << filterprecision(m_OneStep.LeftFootPosition.ddx  ) << " "                   // 17
            << filterprecision(m_OneStep.LeftFootPosition.ddy  ) << " "                   // 18
            << filterprecision(m_OneStep.LeftFootPosition.ddz  ) << " "                   // 19
            << filterprecision(m_OneStep.LeftFootPosition.theta*M_PI/180 ) << " "         // 20
            << filterprecision(m_OneStep.LeftFootPosition.omega*M_PI/180 ) << " "         // 21
            << filterprecision(m_OneStep.LeftFootPosition.omega2*M_PI/180 ) << " "        // 22
            << filterprecision(m_OneStep.RightFootPosition.x ) << " "                     // 23
            << filterprecision(m_OneStep.RightFootPosition.y ) << " "                     // 24
            << filterprecision(m_OneStep.RightFootPosition.z ) << " "                     // 25
            << filterprecision(m_OneStep.RightFootPosition.dx ) << " "                    // 26
            << filterprecision(m_OneStep.RightFootPosition.dy ) << " "                    // 27
            << filterprecision(m_OneStep.RightFootPosition.dz ) << " "                    // 28
            << filterprecision(m_OneStep.RightFootPosition.ddx ) << " "                   // 29
            << filterprecision(m_OneStep.RightFootPosition.ddy ) << " "                   // 30
            << filterprecision(m_OneStep.RightFootPosition.ddz ) << " "                   // 31
            << filterprecision(m_OneStep.RightFootPosition.theta*M_PI/180 ) << " "        // 32
            << filterprecision(m_OneStep.RightFootPosition.omega*M_PI/180 ) << " "        // 33
            << filterprecision(m_OneStep.RightFootPosition.omega2*M_PI/180 ) << " "       // 34
            << filterprecision(localZMPx) << " "                                          // 35
            << filterprecision(localZMPy) << " "                                          // 36
            << filterprecision(m_CurrentConfiguration(0) ) << " "                         // 37
            << filterprecision(m_CurrentConfiguration(1) ) << " "			             ;// 38
        for (unsigned int i = 0 ; i < m_PR->currentConfiguration().size() ; i++)
        {
          aof << filterprecision(m_PR->currentConfiguration()(i)) << " " ;                // 39 - 39+dofs
        }
        aof << endl;
        aof.close();
      }

    }



    bool TestObject::compareDebugFiles( )
    {
      bool SameFile= false;
      if (m_DebugFGPI)
      {
        SameFile = true;
        ifstream alif;
        string aFileName;
        aFileName = m_TestName;
        aFileName += "TestFGPI.dat";
        ODEBUG("Report:" << aFileName);
        unsigned max_nb_of_pbs=100;
        unsigned nb_of_pbs = 0;

        alif.open(aFileName.c_str(),ifstream::in);
        if (!alif.is_open())
        {
          std::cerr << "Unable to open "<< aFileName << std::endl;
          return false;
        }

        ifstream arif;
        aFileName = m_TestName;
        aFileName += "TestFGPI.datref";
        arif.open(aFileName.c_str(),ifstream::in);
        ODEBUG("ReportRef:" << aFileName);

        if (!arif.is_open())
        {
          std::cerr << "Unable to open "<< aFileName << std::endl;
          return false;
        }


        ofstream areportof;
        aFileName = m_TestName;
        aFileName += "TestFGPI_report.dat";
        areportof.open(aFileName.c_str(),ofstream::out);

        // Time
        double LocalInput[NB_OF_FIELDS], ReferenceInput[NB_OF_FIELDS];
        bool finalreport = true;
        unsigned long int nblines = 0;
        bool endofinspection=false;

        // Find size of the two files.
        alif.seekg (0, alif.end);
        unsigned long int alif_length = (unsigned long int)alif.tellg();
        alif.seekg (0, alif.beg);

        arif.seekg (0, arif.end);
        unsigned long int arif_length = (unsigned long int)arif.tellg();
        arif.seekg (0, arif.beg);

        while ((!alif.eof()) &&
               (!arif.eof()) &&
               (!endofinspection))
        {
          for (unsigned int i=0;i<NB_OF_FIELDS;i++)
          {
            alif >> LocalInput[i];
            if (alif.eof())
            {
              endofinspection =true;
              break;
            }
          }
          if (endofinspection)
            break;

          for (unsigned int i=0;i<NB_OF_FIELDS;i++)
          {
            arif >> ReferenceInput[i];
            if (arif.eof())
            {
              endofinspection =true;
              break;
            }
          }
          if (endofinspection)
            break;


          for (unsigned int i=0;i<NB_OF_FIELDS;i++)
          {
            if  (fabs(LocalInput[i]-
                      ReferenceInput[i])>=1e-6)
            {
              finalreport = false;
              ostringstream oss;
              oss << "l: " << nblines
                  << " col:" << i
                  << " ref: " << ReferenceInput[i]
                     << " now: " << LocalInput[i]
                        << " " << nb_of_pbs
                        <<std::endl;
              areportof << oss.str();
              std::cout << oss.str();
              nb_of_pbs++;
              if(nb_of_pbs>max_nb_of_pbs)
              {
                endofinspection=true;
              }
            }
          }

          nblines++;
          if ((nblines*2> alif_length) ||
              (nblines*2> arif_length))
          {
            endofinspection=true;
            break;
          }
        }

        alif.close();
        arif.close();
        areportof.close();
        return finalreport;
      }
      return SameFile;
    }

    bool TestObject::doTest(ostream &os)
    {

      // Set time reference.
      m_clock.startingDate();

      /*! Open and reset appropriatly the debug files. */
      prepareDebugFiles();

      for (unsigned int lNbIt=0;lNbIt<m_OuterLoopNbItMax;lNbIt++)
      {
        os << "<===============================================================>"<<endl;
        os << "Iteration nb: " << lNbIt << endl;

        m_clock.startPlanning();

        /*! According to test profile initialize the current profile. */
        chooseTestProfile();

        m_clock.endPlanning();

        if (m_DebugPR!=0)
        {
          m_DebugPR->currentConfiguration(m_PreviousConfiguration);
          m_DebugPR->currentVelocity(m_PreviousVelocity);
          m_DebugPR->currentAcceleration(m_PreviousAcceleration);
          m_DebugPR->computeForwardKinematics();
        }

        bool ok = true;
        while(ok)
        {
          m_clock.startOneIteration();

          if (m_PGIInterface==0)
          {
            ok = m_PGI->RunOneStepOfTheControlLoop(m_CurrentConfiguration,
                                                   m_CurrentVelocity,
                                                   m_CurrentAcceleration,
                                                   m_OneStep.ZMPTarget,
                                                   m_OneStep.finalCOMPosition,
                                                   m_OneStep.LeftFootPosition,
                                                   m_OneStep.RightFootPosition);
          }
          else if (m_PGIInterface==1)
          {
            ok = m_PGI->RunOneStepOfTheControlLoop(m_CurrentConfiguration,
                                                   m_CurrentVelocity,
                                                   m_CurrentAcceleration,
                                                   m_OneStep.ZMPTarget);
          }

          m_OneStep.NbOfIt++;

          m_clock.stopOneIteration();

          m_PreviousConfiguration = m_CurrentConfiguration;
          m_PreviousVelocity = m_CurrentVelocity;
          m_PreviousAcceleration = m_CurrentAcceleration;

          /*! Call the reimplemented method to generate events. */
          if (ok)
          {
            m_clock.startModification();
            generateEvent();
            m_clock.stopModification();

            m_clock.fillInStatistics();


            /*! Fill the debug files with appropriate information. */
            fillInDebugFiles();
          }
          else
          {
            cerr << "Nothing to dump after " << m_OneStep.NbOfIt << endl;
          }

        }

        os << endl << "End of iteration " << lNbIt << endl;
        os << "<===============================================================>"<<endl;
      }

      string lProfileOutput= m_TestName;
      lProfileOutput +="TimeProfile.dat";
      m_clock.writeBuffer(lProfileOutput);
      m_clock.displayStatistics(os,m_OneStep);

      // Compare debugging files
      return compareDebugFiles();
    }

  }
}

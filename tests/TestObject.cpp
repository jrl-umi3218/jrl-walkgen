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
// System include for files
#include <fstream>
// System include for floating point errors
#include <fenv.h>

#include <pinocchio/fwd.hpp>

// Boost includes to parse XML files
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

// Internals includes
#include "Debug.hh"
#include "TestObject.hh"

// SoT includes: pinocchio
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
using namespace std;

#define NB_OF_FIELDS 39

#ifdef WIN32
double trunc(double x) { return x < 0 ? ceil(x) : 0 < x ? floor(x) : x; }
#endif /* WIN32 */

namespace PatternGeneratorJRL {
namespace TestSuite {

TestObject::TestObject(int argc, char *argv[], string &aTestName,
                       int lPGIInterface) {
  // Generates a signal when a NaN occurs.
#ifndef __APPLE_CC__
  feenableexcept(FE_INVALID | FE_OVERFLOW);
#endif

  m_TestName = aTestName;
  m_OneStep.m_TestName = m_TestName;
  m_PGIInterface = lPGIInterface;
  m_OuterLoopNbItMax = 1;

  /*! default debug output */
  m_DebugFGPI = true;
  m_DebugFGPIFull = true;
  m_DebugZMP2 = false;
  m_TestProfile = 0;

  /*! Extract options and fill in members. */
  getOptions(argc, argv, m_URDFPath, m_SRDFPath, m_TestProfile);

  m_PR = 0;
  m_robotData = 0;
  m_DebugPR = 0;
  m_DebugRobotData = 0;
  m_PGI = 0;
  m_PinoFreeFlyerSize = 7;
}

bool TestObject::checkFiles() {
  // Checking the files
  bool fileExist = false;
  bool correctExtension = false;
  // check if URDF file exist
  {
    fileExist = false;
    std::ifstream file(m_URDFPath.c_str());
    fileExist = !file.fail();
    if (!fileExist)
      throw std::string("failed to open robot urdf model");
  }
  // check if SRDF file exist
  {
    fileExist = false;
    std::ifstream file(m_SRDFPath.c_str());
    fileExist = !file.fail();
    if (!fileExist)
      throw std::string("failed to open robot srdf model");
  }
  // check if file has .urdf extension
  {
    correctExtension = false;
    std::size_t found = m_URDFPath.find_last_of('.');
    correctExtension = (m_URDFPath.substr(found) == ".urdf");
    if (!correctExtension)
      throw std::string("File is not an urdf, extension has to be .urdf");
  }
  // check if file has .srdf extension
  {
    correctExtension = false;
    std::size_t found = m_SRDFPath.find_last_of('.');
    correctExtension = (m_SRDFPath.substr(found) == ".srdf");
    if (!correctExtension)
      throw std::string("File is not an srdf, extension has to be .srdf");
  }
  return correctExtension && fileExist;
}

bool TestObject::init() {
  try {
    checkFiles();
  } catch (std::string e) {
    std::cerr << "file problem, existance or extension incorrect" << std::endl;
    std::cerr << e << std::endl;
    return false;
  }

  // Instanciate and initialize the model of the robot from URDF
  // and SRDF files.
  CreateAndInitializeHumanoidRobot(m_URDFPath, m_SRDFPath, m_PR, m_DebugPR);

  // Create Pattern Generator Interface
  m_OneStep.m_PR = m_PR;
  m_OneStep.m_DebugPR = m_DebugPR;

  m_PGI = patternGeneratorInterfaceFactory(m_PR);
  m_PGI->SetCurrentJointValues(m_HalfSitting);

  m_SPM = new SimplePluginManager();

  // Specify the walking mode: here the default one.
  istringstream strm2(":walkmode 0");
  m_PGI->ParseCmd(strm2);

  // Initialize m_CurrentConfiguration,
  // m_CurrentVelocity, m_CurrentAcceleration
  InitializeStateVectors();

  // Set m_CurrentConfiguration to halfSitting
  for (Eigen::Index i = 0; i < m_HalfSitting.size(); i++)
    m_CurrentConfiguration[i + m_PinoFreeFlyerSize] = m_HalfSitting[i];
  m_PR->currentPinoConfiguration(m_CurrentConfiguration);

  CreateAndInitializeComAndFootRealization();

  // initialize the reference to limbs
  InitializeLimbs();
  return true;
}

TestObject::~TestObject() {

  if (m_PR != 0)
    delete m_PR;

  if (m_DebugPR != 0)
    delete m_DebugPR;

  if (m_robotData != 0)
    delete m_robotData;

  if (m_DebugRobotData != 0)
    delete m_DebugRobotData;

  if (m_PGI != 0)
    delete m_PGI;

  if (m_SPM != 0)
    delete m_SPM;
}

void TestObject::InitializeStateVectors() {
  // This is a vector corresponding to ALL the DOFS of the robot:
  // free flyer + actuated DOFS.
  unsigned lNbDofs = m_PR->numberDof();
  m_CurrentConfiguration.resize(lNbDofs);
  unsigned lNbVelDofs = m_PR->numberVelDof();
  m_CurrentVelocity.resize(lNbVelDofs);
  m_CurrentAcceleration.resize(lNbVelDofs);
  m_PreviousConfiguration.resize(lNbDofs);
  m_PreviousVelocity.resize(lNbVelDofs);
  m_PreviousAcceleration.resize(lNbVelDofs);
  for (pinocchio::JointIndex i = 0; i < m_PinoFreeFlyerSize; i++) {
    m_PreviousConfiguration[i] = 0.0;
    m_PreviousVelocity[i] = 0.0;
    m_PreviousAcceleration[i] = 0.0;
  }

  for (Eigen::Index i = m_PinoFreeFlyerSize; i < lNbDofs; i++) {
    m_PreviousConfiguration[i] = m_HalfSitting[i - m_PinoFreeFlyerSize];
  }
  for (Eigen::Index i = m_PinoFreeFlyerSize; i < lNbVelDofs; i++) {
    m_PreviousVelocity[i] = 0.0;
    m_PreviousAcceleration[i] = 0.0;
  }
}

void TestObject::CreateAndInitializeComAndFootRealization() {
  m_ComAndFootRealization = new ComAndFootRealizationByGeometry(
      (PatternGeneratorInterfacePrivate *)m_SPM);

  m_ComAndFootRealization->setPinocchioRobot(m_PR);
  m_ComAndFootRealization->SetStepStackHandler(new StepStackHandler(m_SPM));
  m_ComAndFootRealization->SetHeightOfTheCoM(0.814);
  m_ComAndFootRealization->setSamplingPeriod(0.005);
  m_ComAndFootRealization->Initialization();
  Eigen::Matrix<double, 6, 1> waist;
  for (int i = 0; i < 6; ++i) {
    waist(i) = 0;
  }
  Eigen::Vector3d lStartingCOMState;

  lStartingCOMState(0) = m_OneStep.m_finalCOMPosition.x[0];
  lStartingCOMState(1) = m_OneStep.m_finalCOMPosition.y[0];
  lStartingCOMState(2) = m_OneStep.m_finalCOMPosition.z[0];
  m_ComAndFootRealization->setSamplingPeriod(0.005);
  m_ComAndFootRealization->Initialization();
  m_ComAndFootRealization->InitializationCoM(
      m_HalfSitting, lStartingCOMState, waist, m_OneStep.m_LeftFootPosition,
      m_OneStep.m_RightFootPosition);
  m_CurrentConfiguration(0) = waist(0);
  m_CurrentConfiguration(1) = waist(1);
  m_CurrentConfiguration(2) = waist(2);
}

void TestObject::InitializeLimbs() {
  m_leftLeg =
      m_PR->jointsBetween(m_PR->waist(), m_PR->leftFoot()->associatedAnkle);
  m_rightLeg =
      m_PR->jointsBetween(m_PR->waist(), m_PR->rightFoot()->associatedAnkle);
  m_leftArm = m_PR->jointsBetween(m_PR->chest(), m_PR->leftWrist());
  m_rightArm = m_PR->jointsBetween(m_PR->chest(), m_PR->rightWrist());

  m_leftLeg.erase(m_leftLeg.begin());
  m_rightLeg.erase(m_rightLeg.begin());

  pinocchio::JointModelVector &ActuatedJoints = m_PR->getActuatedJoints();
  for (unsigned i = 0; i < m_leftLeg.size(); ++i)
    m_leftLeg[i] = pinocchio::idx_q(ActuatedJoints[m_leftLeg[i]]) - 1;
  for (unsigned i = 0; i < m_rightLeg.size(); ++i)
    m_rightLeg[i] = pinocchio::idx_q(ActuatedJoints[m_rightLeg[i]]) - 1;
  for (unsigned i = 0; i < m_leftArm.size(); ++i)
    m_leftArm[i] = pinocchio::idx_q(ActuatedJoints[m_leftArm[i]]) - 1;
  for (unsigned i = 0; i < m_rightArm.size(); ++i)
    m_rightArm[i] = pinocchio::idx_q(ActuatedJoints[m_rightArm[i]]) - 1;

  if ((m_robotModel.parents.size() >= m_rightArm.back() + 1) &&
      m_robotModel.parents[m_rightArm.back() + 1] == m_rightArm.back())
    m_rightGripper = m_rightArm.back() + 1;
  else
    m_rightGripper = 0;

  if ((m_robotModel.parents.size() >= m_leftArm.back() + 1) &&
      m_robotModel.parents[m_leftArm.back() + 1] == m_leftArm.back())
    m_leftGripper = m_leftArm.back() + 1;
  else
    m_leftGripper = 0;
}

void TestObject::CreateAndInitializeHumanoidRobot(std::string &URDFFile,
                                                  std::string &SRDFFile,
                                                  PinocchioRobot *&aPR,
                                                  PinocchioRobot *&aDebugPR) {
  // Creating the humanoid robot via the URDF.
  //      try{
  pinocchio::urdf::buildModel(URDFFile, pinocchio::JointModelFreeFlyer(),
                              m_robotModel);
  m_robotData = new pinocchio::Data(m_robotModel);
  m_DebugRobotData = new pinocchio::Data(m_robotModel);

  if ((aPR == 0) || (aDebugPR == 0)) {
    if (aPR != 0)
      delete aPR;
    if (aDebugPR != 0)
      delete aDebugPR;

    aPR = new PinocchioRobot();
    aDebugPR = new PinocchioRobot();
  }

  // initialize the model and data of the humanoid robot
  aPR->initializeRobotModelAndData(&m_robotModel, m_robotData);
  aDebugPR->initializeRobotModelAndData(&m_robotModel, m_DebugRobotData);

  m_conf.resize(m_robotModel.nq);
  // Parsing the SRDF file to initialize
  // the starting configuration and the robot specifities
  InitializeRobotWithSRDF(*aPR, SRDFFile);
  InitializeRobotWithSRDF(*aDebugPR, SRDFFile);
}

void TestObject::InitializeRobotWithSRDF(PinocchioRobot &aPR,
                                         const std::string &filename) {
  // manage the SRDF file
  //////////////////////////////////
  std::ifstream srdf_stream(filename.c_str());
  if (!srdf_stream.is_open()) {
    const std::string exception_message(filename +
                                        " does not seem to be a valid file.");
    cerr << exception_message << endl;
    throw std::invalid_argument(exception_message);
  }
  // Read xml stream
  using boost::property_tree::ptree;
  ptree pt;
  try {
    read_xml(srdf_stream, pt);
  } catch (...) {
    cerr << "problem while reading the srdf file.";
    cerr << " File corrupted?" << endl;
    return;
  }

  // Get the starting configuration : half sitting
  // Uses the order
  m_HalfSitting.resize(aPR.numberDof() - m_PinoFreeFlyerSize);
  m_HalfSitting.setZero();

  std::vector<bool> setHalfSittingJoint(aPR.numberDof() - m_PinoFreeFlyerSize);
  for (std::size_t i = 0; i < setHalfSittingJoint.size(); i++)
    setHalfSittingJoint[i] = false;

  pinocchio::Model *aModel = aPR.Model();
  BOOST_FOREACH (const ptree::value_type &v,
                 pt.get_child("robot.group_state")) {
    if (v.first == "<xmlattr>") {
      std::string nameOfGroup = v.second.get<std::string>("name");
      if (nameOfGroup != "half_sitting")
        break;
    }

    if (v.first == "joint") {
      const std::string jointName = v.second.get<std::string>("<xmlattr>.name");
      if (jointName != "root_joint") {
        const double jointValue = v.second.get<double>("<xmlattr>.value");
        if (aModel->existJointName(jointName)) {
          pinocchio::JointIndex id = aModel->getJointId(jointName);
          unsigned idq = pinocchio::idx_q(aModel->joints[id]);
          // we assume only revolute joint here.
          m_HalfSitting(idq - m_PinoFreeFlyerSize) = jointValue;
          setHalfSittingJoint[idq - m_PinoFreeFlyerSize] = true;
        }
      }
    }
  } // BOOST_FOREACH

  for (std::size_t i = 0; i < setHalfSittingJoint.size(); i++) {
    if (setHalfSittingJoint[i] == false) {
      std::cerr << "Joint number " << i << " not initialized on "
                << setHalfSittingJoint.size() << " nb of joints" << std::endl;
      exit(-1);
    }
  }

  ODEBUG("Half sitting: " << m_HalfSitting);
  bool DebugConfiguration = true;
  if (DebugConfiguration) {
    ofstream aofq;
    aofq.open("TestConfiguration.dat", ofstream::out);
    if (aofq.is_open()) {
      for (unsigned int k = 0; k < m_HalfSitting.size(); k++) {
        aofq << m_HalfSitting(k) << " ";
      }
      aofq << endl;
    }
  }

  // capture the details of the feet
  //////////////////////////////////

  // Initialize the Right Foot
  PRFoot aFoot;
  string path = "robot.specificities.feet.right.size";
  BOOST_FOREACH (const ptree::value_type &v, pt.get_child(path.c_str())) {
    aFoot.soleHeight = v.second.get<double>("height");
    aFoot.soleWidth = v.second.get<double>("width");
    aFoot.soleDepth = v.second.get<double>("depth");
  }
  path = "robot.specificities.feet.right.anklePosition";
  BOOST_FOREACH (const ptree::value_type &v, pt.get_child(path.c_str())) {
    aFoot.anklePosition(0) = v.second.get<double>("x");
    aFoot.anklePosition(1) = v.second.get<double>("y");
    aFoot.anklePosition(2) = v.second.get<double>("z");
  }
  pinocchio::FrameIndex ra = aModel->getFrameId("r_ankle");
  aFoot.associatedAnkle = aModel->frames[ra].parent;
  aPR.initializeRightFoot(aFoot);

  // Initialize the Left Foot
  path = "robot.specificities.feet.left.size";
  BOOST_FOREACH (const ptree::value_type &v, pt.get_child(path.c_str())) {
    aFoot.soleHeight = v.second.get<double>("height");
    aFoot.soleWidth = v.second.get<double>("width");
    aFoot.soleDepth = v.second.get<double>("depth");
  }
  path = "robot.specificities.feet.left.anklePosition";
  BOOST_FOREACH (const ptree::value_type &v, pt.get_child(path.c_str())) {
    aFoot.anklePosition(0) = v.second.get<double>("x");
    aFoot.anklePosition(1) = v.second.get<double>("y");
    aFoot.anklePosition(2) = v.second.get<double>("z");
  }
  pinocchio::FrameIndex la = aModel->getFrameId("l_ankle");
  aFoot.associatedAnkle = aModel->frames[la].parent;
  aPR.initializeLeftFoot(aFoot);

  // path="robot.mapURDFToOpenHRP";
  unsigned int lindex = 0;
  BOOST_FOREACH (const ptree::value_type &v, pt.get_child("robot.group")) {
    // Check if the group is mapURDFToOpenHRP
    // This will only work if the first group has the name
    // mapURDFToOpenHRP.
    if (v.first == "<xmlattr>") {
      std::string nameOfGroup = v.second.get<std::string>("name");
      if (nameOfGroup != "mapURDFToOpenHRP")
        break;
    }
    if (v.first == "joint") {
      const std::string jointName = v.second.get<std::string>("<xmlattr>.name");
      pinocchio::JointIndex id = aModel->getJointId(jointName);
      unsigned idq = pinocchio::idx_q(aModel->joints[id]);
      m_fromURDFToOpenHRP.push_back(idq - 1);
    }
    lindex++;
  }
}

void TestObject::prepareDebugFiles() {

  if (m_DebugZMP2) {
    ofstream aofzmpmb2;
    string aFileName = m_TestName;
    aFileName += "ZMPMBSTAGE2.dat";
    aofzmpmb2.open(aFileName.c_str(), ofstream::out);
  }

  if (m_DebugFGPIFull) {
    ofstream aofFULL;
    string aFileName = m_TestName;
    aFileName += "TestFGPIFull.dat";
    aofFULL.open(aFileName.c_str(), ofstream::out);
  }
}

void TestObject::fillInDebugFiles() {
  if (m_DebugFGPI)
    m_OneStep.fillInDebugFile();
}

void TestObject::fillInDebugFilesFull() {
  if (m_DebugFGPIFull) {
    analyticalInverseKinematics(m_CurrentConfiguration, m_CurrentVelocity,
                                m_CurrentAcceleration);
    m_DebugPR->computeInverseDynamics(m_CurrentConfiguration, m_CurrentVelocity,
                                      m_CurrentAcceleration);
    Eigen::Vector3d zmpmb;
    m_DebugPR->zeroMomentumPoint(zmpmb);

    Eigen::VectorXd tau;

    tau = m_DebugPR->currentTau();

    ofstream aof;
    string aFileName;
    aFileName = m_TestName;
    aFileName += "TestFGPIFull.dat";
    aof.open(aFileName.c_str(), ofstream::app);
    aof.precision(8);
    aof.setf(ios::scientific, ios::floatfield);
    m_OneStep.fillInDebugFileContent(aof);
    aof << " " << filterprecision(zmpmb(0)) << " "     // 45
        << filterprecision(zmpmb(1)) << " "            // 46
        << filterprecision(zmpmb(2)) << " ";           // 47
    for (unsigned int k = 0; k < m_conf.size(); k++) { // 48-53 -> 54-83
      aof << filterprecision(m_conf(k)) << " ";
    }
    for (unsigned int k = 0; k < m_vel.size(); k++) { // 84-89 -> 90-118
      aof << filterprecision(m_vel(k)) << " ";
    }
    for (unsigned int k = 0; k < m_acc.size(); k++) { // 119-125 -> 125-155
      aof << filterprecision(m_acc(k)) << " ";
    }
    for (unsigned int k = 0; k < tau.size(); k++) { // 119-125 -> 125-155
      aof << filterprecision(tau(k)) << " ";
    }
    aof << endl;
    aof.close();
  }
}

bool TestObject::compareDebugFiles() {
  bool SameFile = false;
  if (m_DebugFGPI) {
    SameFile = true;
    ifstream alif;
    string aFileName;
    aFileName = m_TestName;
    aFileName += "TestFGPI.dat";
    ODEBUG("Report:" << aFileName);
    unsigned max_nb_of_pbs = 100;
    unsigned nb_of_pbs = 0;

    alif.open(aFileName.c_str(), ifstream::in);
    if (!alif.is_open()) {
      std::cerr << "Unable to open " << aFileName << std::endl;
      return false;
    }

    ifstream arif;
    aFileName = m_TestName;
    aFileName += "TestFGPI.datref";
    arif.open(aFileName.c_str(), ifstream::in);
    ODEBUG("ReportRef:" << aFileName);

    if (!arif.is_open()) {
      std::cerr << "Unable to open " << aFileName << std::endl;
      return false;
    }

    ofstream areportof;
    aFileName = m_TestName;
    aFileName += "TestFGPI_report.dat";
    areportof.open(aFileName.c_str(), ofstream::out);

    // Time
    double LocalInput[NB_OF_FIELDS], ReferenceInput[NB_OF_FIELDS];
    bool finalreport = true;
    unsigned long int nblines = 0;
    bool endofinspection = false;

    // Find size of the two files.
    alif.seekg(0, alif.end);
    unsigned long int alif_length = (unsigned long int)alif.tellg();
    alif.seekg(0, alif.beg);

    arif.seekg(0, arif.end);
    unsigned long int arif_length = (unsigned long int)arif.tellg();
    arif.seekg(0, arif.beg);

    while ((!alif.eof()) && (!arif.eof()) && (!endofinspection)) {
      for (unsigned int i = 0; i < NB_OF_FIELDS; i++) {
        alif >> LocalInput[i];
        if (alif.eof()) {
          endofinspection = true;
          break;
        }
      }
      if (endofinspection)
        break;

      for (unsigned int i = 0; i < NB_OF_FIELDS; i++) {
        arif >> ReferenceInput[i];
        if (arif.eof()) {
          endofinspection = true;
          break;
        }
      }
      if (endofinspection)
        break;

      for (unsigned int i = 0; i < NB_OF_FIELDS; i++) {
        if (fabs(LocalInput[i] - ReferenceInput[i]) >= 1e-6) {
          finalreport = false;
          ostringstream oss;
          oss << "l: " << nblines << " col:" << i
              << " ref: " << ReferenceInput[i] << " now: " << LocalInput[i]
              << " " << nb_of_pbs << std::endl;
          areportof << oss.str();
          nb_of_pbs++;
          if (nb_of_pbs > max_nb_of_pbs) {
            endofinspection = true;
          }
        }
      }

      nblines++;
      if ((nblines * 2 > alif_length) || (nblines * 2 > arif_length)) {
        endofinspection = true;
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

bool TestObject::doTest(ostream &os) {

  // Set time reference.
  m_clock.startingDate();

  /*! Open and reset appropriatly the debug files. */
  prepareDebugFiles();

  for (unsigned int lNbIt = 0; lNbIt < m_OuterLoopNbItMax; lNbIt++) {
    // os << "<====================================";
    // os << "===========================>"<<endl;
    // os << "Iteration nb: " << lNbIt << endl;

    m_clock.startPlanning();

    /*! According to test profile initialize the current profile. */
    chooseTestProfile();

    m_clock.endPlanning();

    if (m_DebugPR != 0) {
      m_DebugPR->currentPinoConfiguration(m_PreviousConfiguration);
      m_DebugPR->currentRPYVelocity(m_PreviousVelocity);
      m_DebugPR->currentRPYAcceleration(m_PreviousAcceleration);
      m_DebugPR->computeForwardKinematics();
    }

    bool ok = true;
    while (ok) {
      m_clock.startOneIteration();

      if (m_PGIInterface == 0) {
        ok = m_PGI->RunOneStepOfTheControlLoop(
            m_CurrentConfiguration, m_CurrentVelocity, m_CurrentAcceleration,
            m_OneStep.m_ZMPTarget, m_OneStep.m_finalCOMPosition,
            m_OneStep.m_LeftFootPosition, m_OneStep.m_RightFootPosition);
      } else if (m_PGIInterface == 1) {
        ok = m_PGI->RunOneStepOfTheControlLoop(
            m_CurrentConfiguration, m_CurrentVelocity, m_CurrentAcceleration,
            m_OneStep.m_ZMPTarget);
      }

      m_clock.stopOneIteration();

      m_PreviousConfiguration = m_CurrentConfiguration;
      m_PreviousVelocity = m_CurrentVelocity;
      m_PreviousAcceleration = m_CurrentAcceleration;

      /*! Call the reimplemented method to generate events. */
      if (ok) {
        /*! Fill the debug files with appropriate information. */
        fillInDebugFiles();
      } else {
        cerr << "Nothing to dump after " << m_OneStep.m_NbOfIt << endl;
      }

      m_OneStep.m_NbOfIt++;

      if (ok) {
        m_clock.startModification();
        generateEvent();
        m_clock.stopModification();

        m_clock.fillInStatistics();
      }
    }

    os << endl << "End of iteration " << lNbIt << endl;
    os << "<=============================";
    os << "==================================>" << endl;
  }

  string lProfileOutput = m_TestName;
  lProfileOutput += "TimeProfile.dat";
  m_clock.writeBuffer(lProfileOutput);
  m_clock.displayStatistics(os, m_OneStep);

  // Compare debugging files
  return compareDebugFiles();
}

void TestObject::setDirectorySeqplay(std::string &aDirectory) {
  m_DirectoryName = aDirectory;
}

void TestObject::analyticalInverseKinematics(Eigen::VectorXd &conf,
                                             Eigen::VectorXd &vel,
                                             Eigen::VectorXd &acc) {
  /// \brief calculate, from the CoM of computed by the preview control,
  ///    the corresponding articular position, velocity and acceleration
  /// ------------------------------------------------------------------
  Eigen::VectorXd aCOMState(6);
  Eigen::VectorXd aCOMSpeed(6);
  Eigen::VectorXd aCOMAcc(6);
  Eigen::VectorXd aLeftFootPosition(5);
  Eigen::VectorXd aRightFootPosition(5);

  aCOMState(0) = m_OneStep.m_finalCOMPosition.x[0];
  aCOMState(1) = m_OneStep.m_finalCOMPosition.y[0];
  aCOMState(2) = m_OneStep.m_finalCOMPosition.z[0];
  aCOMState(3) = m_OneStep.m_finalCOMPosition.roll[0] * 180 / M_PI;
  aCOMState(4) = m_OneStep.m_finalCOMPosition.pitch[0] * 180 / M_PI;
  aCOMState(5) = m_OneStep.m_finalCOMPosition.yaw[0] * 180 / M_PI;

  aCOMSpeed(0) = m_OneStep.m_finalCOMPosition.x[1];
  aCOMSpeed(1) = m_OneStep.m_finalCOMPosition.y[1];
  aCOMSpeed(2) = m_OneStep.m_finalCOMPosition.z[1];
  aCOMSpeed(3) = m_OneStep.m_finalCOMPosition.roll[1] /** * 180/M_PI  */;
  aCOMSpeed(4) = m_OneStep.m_finalCOMPosition.pitch[1] /** * 180/M_PI  */;
  aCOMSpeed(5) = m_OneStep.m_finalCOMPosition.yaw[1] /** * 180/M_PI  */;

  aCOMAcc(0) = m_OneStep.m_finalCOMPosition.x[2];
  aCOMAcc(1) = m_OneStep.m_finalCOMPosition.y[2];
  aCOMAcc(2) = m_OneStep.m_finalCOMPosition.z[2];
  aCOMAcc(3) = m_OneStep.m_finalCOMPosition.roll[2] /** * 180/M_PI  */;
  aCOMAcc(4) = m_OneStep.m_finalCOMPosition.pitch[2] /** * 180/M_PI  */;
  aCOMAcc(5) = m_OneStep.m_finalCOMPosition.yaw[2] /** * 180/M_PI  */;

  aLeftFootPosition(0) = m_OneStep.m_LeftFootPosition.x;
  aLeftFootPosition(1) = m_OneStep.m_LeftFootPosition.y;
  aLeftFootPosition(2) = m_OneStep.m_LeftFootPosition.z;
  aLeftFootPosition(3) = m_OneStep.m_LeftFootPosition.theta;
  aLeftFootPosition(4) = m_OneStep.m_LeftFootPosition.omega;

  aRightFootPosition(0) = m_OneStep.m_RightFootPosition.x;
  aRightFootPosition(1) = m_OneStep.m_RightFootPosition.y;
  aRightFootPosition(2) = m_OneStep.m_RightFootPosition.z;
  aRightFootPosition(3) = m_OneStep.m_RightFootPosition.theta;
  aRightFootPosition(4) = m_OneStep.m_RightFootPosition.omega;
  m_ComAndFootRealization->setSamplingPeriod(0.005);
  m_ComAndFootRealization->ComputePostureForGivenCoMAndFeetPosture(
      aCOMState, aCOMSpeed, aCOMAcc, aLeftFootPosition, aRightFootPosition,
      conf, vel, acc, ((int)m_OneStep.m_NbOfIt), 1);

  if (m_leftGripper != 0 && m_rightGripper != 0) {
    conf(m_leftGripper) = 10.0 * M_PI / 180.0;
    conf(m_rightGripper) = 10.0 * M_PI / 180.0;
  }
}

void TestObject::setFromURDFToOpenHRP(
    std::vector<unsigned int> &vfromURDFToOpenHRP) {
  m_fromURDFToOpenHRP = vfromURDFToOpenHRP;
}

void TestObject::parseFromURDFtoOpenHRPIndex(Eigen::VectorXd &conf) {
  if (((unsigned int)conf.size()) != m_fromURDFToOpenHRP.size()) {
    std::cerr << "" << std::endl;
    return;
  }

  std::cerr << "conf size():" << conf.size() << std::endl;
  std::cerr << "Current Configuration: " << m_CurrentConfiguration.size()
            << std::endl;
  conf.setZero();
  for (unsigned int i = 0; i < m_fromURDFToOpenHRP.size(); i++)
    conf(i) = m_CurrentConfiguration(m_fromURDFToOpenHRP[i]);
}

void TestObject::generateOpenHRPTrajectories() {

  analyticalInverseKinematics(m_CurrentConfiguration, m_CurrentVelocity,
                              m_CurrentAcceleration);

  /// \brief Create file .hip/.waist .pos .zmp
  /// --------------------
  Eigen::VectorXd conf;
  conf = m_CurrentConfiguration;
  parseFromURDFtoOpenHRPIndex(conf);

  ofstream aof;
  string aFileName;
  long int iteration = m_OneStep.m_NbOfIt - 1;

  if (iteration == 0) {
    aFileName = m_DirectoryName;
    aFileName += m_TestName;
    aFileName += ".pos";
    aof.open(aFileName.c_str(), ofstream::out);
    aof.close();
  }

  ///----
  aFileName = m_DirectoryName;
  aFileName += m_TestName;
  aFileName += ".pos";
  aof.open(aFileName.c_str(), ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  aof << filterprecision(((double)iteration) * 0.005) << " "; // 1
  for (unsigned int i = 6; i < conf.size(); i++) {
    aof << filterprecision(conf(i)) << " "; // 2
  }
  for (unsigned int i = 0; i < 9; i++) {
    aof << 0.0 << " ";
  }
  aof << 0.0 << endl;
  aof.close();

  if (iteration == 0) {
    aFileName = m_DirectoryName;
    aFileName += m_TestName;
    aFileName += ".hip";
    aof.open(aFileName.c_str(), ofstream::out);
    aof.close();
  }
  aFileName = m_DirectoryName;
  aFileName += m_TestName;
  aFileName += ".hip";
  aof.open(aFileName.c_str(), ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  aof << filterprecision(((double)iteration) * 0.005) << " "; // 1
  aof << filterprecision(m_OneStep.m_finalCOMPosition.roll[0] * M_PI / 180)
      << " "; // 2
  aof << filterprecision(m_OneStep.m_finalCOMPosition.pitch[0] * M_PI / 180)
      << " ";                                                               // 3
  aof << filterprecision(m_OneStep.m_finalCOMPosition.yaw[0] * M_PI / 180); // 4
  aof << endl;
  aof.close();

  if (iteration == 0) {
    aFileName = m_DirectoryName;
    aFileName += m_TestName;
    aFileName += ".zmp";
    aof.open(aFileName.c_str(), ofstream::out);
    aof.close();
  }

  FootAbsolutePosition aSupportState;
  if (m_OneStep.m_LeftFootPosition.stepType < 0)
    aSupportState = m_OneStep.m_LeftFootPosition;
  else
    aSupportState = m_OneStep.m_RightFootPosition;

  aFileName = m_DirectoryName;
  aFileName += m_TestName;
  aFileName += ".zmp";
  aof.open(aFileName.c_str(), ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  aof << filterprecision(((double)iteration) * 0.005) << " "; // 1
  aof << filterprecision(m_OneStep.m_ZMPTarget(0) - m_CurrentConfiguration(0))
      << " "; // 2
  aof << filterprecision(m_OneStep.m_ZMPTarget(1) - m_CurrentConfiguration(1))
      << " ";                                                          // 3
  aof << filterprecision(aSupportState.z - m_CurrentConfiguration(2)); // 4
  aof << endl;
  aof.close();
}

} // namespace TestSuite
} // namespace PatternGeneratorJRL

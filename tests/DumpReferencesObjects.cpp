// (c) Copyright LAAS CNRS 2019
// Olivier Stasse

#include "DumpReferencesObjects.hh"

using namespace PatternGeneratorJRL::TestSuite;
using namespace std;

DumpReferencesObjects::DumpReferencesObjects() {
  m_InternalFormat = 1;
  m_TimeOption = false;
  if (m_InternalFormat == 0) {
    m_prevCoMp.resize(3);
    m_prevdCoMp.resize(3);
    m_prevddCoMp.resize(3);
    m_prevWaistOrien.resize(3);

    m_prevLeftAnklePos.resize(3);
    m_prevLeftAnkledPos.resize(3);
    m_prevLeftAnkleddPos.resize(3);
    m_prevLeftAnkleOrientation.resize(3);
    m_prevLeftAnkledOrientation.resize(3);
    m_prevLeftAnkleddOrientation.resize(3);

    m_prevRightAnklePos.resize(3);
    m_prevRightAnkledPos.resize(3);
    m_prevRightAnkleddPos.resize(3);
    m_prevRightAnkleOrientation.resize(3);
    m_prevRightAnkledOrientation.resize(3);
    m_prevRightAnkleddOrientation.resize(3);

    m_prevZMPlocal.resize(2);
    m_prevZMPRef.resize(3);
    m_TimeOption = false;
  } else if (m_InternalFormat == 1) {
    m_prevCoMpF2.resize(9);
    m_prevWaistOrienF2.resize(9);
    m_prevZMPlocalF2.resize(9);
    m_prevLeftAnklePosF2.resize(18);
    m_prevRightAnklePosF2.resize(18);
  }
}

void DumpReferencesObjects::setAnklePositions(
    Eigen::Vector3d &AnklePositionRight, Eigen::Vector3d &AnklePositionLeft) {
  m_AnklePositionRight = AnklePositionRight;
  m_AnklePositionLeft = AnklePositionLeft;
}
void DumpReferencesObjects::prepareFile(ofstream &aof, string &prefix,
                                        OneStep &anOneStep) {
  string aFileName;
  aFileName = prefix;
  aFileName += ".dat";
  if (anOneStep.m_NbOfIt == 1) {
    aof.open(aFileName.c_str(), ofstream::out);
  }
  aof.open(aFileName.c_str(), ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
}

void DumpReferencesObjects::fillFileWithSubsamplingAndClose(
    FillingFileArgs_t &aSetOfFillingFileArgs, std::vector<double> &next,
    std::vector<double> &prev) {
  for (double subsampling = 1.0;
       subsampling <= aSetOfFillingFileArgs.nb_subsampling;
       subsampling += 1.0) {
    if (m_TimeOption) {
      aSetOfFillingFileArgs.aof
          << filterprecision(
                 ((double)aSetOfFillingFileArgs.anOneStep.m_NbOfIt) *
                     aSetOfFillingFileArgs.dt +
                 subsampling * aSetOfFillingFileArgs.dt /
                     aSetOfFillingFileArgs.nb_subsampling)
          << " "; // 1
    }
    for (unsigned int i = 0; i < next.size(); i++) {
      double intermediate = prev[i] + (next[i] - prev[i]) * subsampling /
                                          aSetOfFillingFileArgs.nb_subsampling;
      aSetOfFillingFileArgs.aof << intermediate;
      if (i < next.size() - 1)
        aSetOfFillingFileArgs.aof << " ";
    }
    aSetOfFillingFileArgs.aof << std::endl;
  }
  aSetOfFillingFileArgs.aof.close();
  prev = next;
}

void DumpReferencesObjects::fillInTests(
    std::string &aTestName, OneStep &anOneStep,
    Eigen::VectorXd &aCurrentConfiguration) {
  if (m_InternalFormat == 0)
    fillInTestsFormat1(aTestName, anOneStep, aCurrentConfiguration);
  else
    fillInTestsFormat2(aTestName, anOneStep, aCurrentConfiguration);
}

void DumpReferencesObjects::fillInTestsFormat1(
    std::string &aTestName, OneStep &anOneStep,
    Eigen::VectorXd &aCurrentConfiguration)

{

  ofstream aof;
  string prefix = aTestName + "CoM";
  std::vector<double> vec_db;
  double dt = 0.005;
  double nb_sampling = 5.0;

  FillingFileArgs_t aSetOfFillingFileArgs(aof, // File on which to write
                                          dt, nb_sampling,
                                          anOneStep); // Compute speed ?

  double localZMPx = anOneStep.m_ZMPTarget(0) * cos(aCurrentConfiguration(5)) -
                     anOneStep.m_ZMPTarget(1) * sin(aCurrentConfiguration(5)) +
                     aCurrentConfiguration(0);
  double localZMPy = anOneStep.m_ZMPTarget(0) * sin(aCurrentConfiguration(5)) +
                     anOneStep.m_ZMPTarget(1) * cos(aCurrentConfiguration(5)) +
                     aCurrentConfiguration(1);

  /// CoM Position
  prepareFile(aof, prefix, anOneStep);
  vec_db.resize(3);
  vec_db[0] = filterprecision(anOneStep.m_finalCOMPosition.x[0]);
  vec_db[1] = filterprecision(anOneStep.m_finalCOMPosition.y[0]);
  vec_db[2] = filterprecision(anOneStep.m_finalCOMPosition.z[0]);
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db, m_prevCoMp);

  /// CoM velocity
  prefix = aTestName + "dCoM";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_finalCOMPosition.x[1]); // 6
  vec_db[1] = filterprecision(anOneStep.m_finalCOMPosition.y[1]); // 7
  vec_db[2] = filterprecision(anOneStep.m_finalCOMPosition.z[1]); // 8
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db, m_prevdCoMp);

  /// CoM acceleration
  prefix = aTestName + "ddCoM";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_finalCOMPosition.x[2]); // 10
  vec_db[1] = filterprecision(anOneStep.m_finalCOMPosition.y[2]); // 11
  vec_db[2] = filterprecision(anOneStep.m_finalCOMPosition.z[2]); // 12
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db, m_prevddCoMp);

  /// Waist Orientation
  prefix = aTestName + "WaistOrientation";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_finalCOMPosition.yaw[0]); // 5
  vec_db[1] = filterprecision(anOneStep.m_finalCOMPosition.yaw[1]); // 9
  vec_db[2] = filterprecision(anOneStep.m_finalCOMPosition.yaw[2]); // 13
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevWaistOrien);

  /// ZMP Ref
  prefix = aTestName + "ZMPRef";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_ZMPTarget(0)); // 14
  vec_db[1] = filterprecision(anOneStep.m_ZMPTarget(1)); // 15
  vec_db[2] = filterprecision(anOneStep.m_ZMPTarget(2)); // 16
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevZMPlocal);

  prefix = aTestName + "LeftAnklePos";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_LeftFootPosition.x +
                              m_AnklePositionLeft[0]); // 17
  vec_db[1] = filterprecision(anOneStep.m_LeftFootPosition.y +
                              m_AnklePositionLeft[1]); // 18
  vec_db[2] = filterprecision(anOneStep.m_LeftFootPosition.z +
                              m_AnklePositionLeft[2]); // 19
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevLeftAnklePos);

  prefix = aTestName + "LeftAnkledPos";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_LeftFootPosition.dx); // 20
  vec_db[1] = filterprecision(anOneStep.m_LeftFootPosition.dy); // 21
  vec_db[2] = filterprecision(anOneStep.m_LeftFootPosition.dz); // 22
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevLeftAnkledPos);

  prefix = aTestName + "LeftAnkleddPos";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_LeftFootPosition.ddx); // 23
  vec_db[1] = filterprecision(anOneStep.m_LeftFootPosition.ddy); // 24
  vec_db[2] = filterprecision(anOneStep.m_LeftFootPosition.ddz); // 25
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevLeftAnkleddPos);

  prefix = aTestName + "LeftAnkleOrientation";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_LeftFootPosition.theta);  // 26
  vec_db[1] = filterprecision(anOneStep.m_LeftFootPosition.omega);  // 29
  vec_db[2] = filterprecision(anOneStep.m_LeftFootPosition.omega2); // 30
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevLeftAnkleOrientation);

  prefix = aTestName + "RightAnklePos";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_RightFootPosition.x +
                              m_AnklePositionRight[0]);
  vec_db[1] = filterprecision(anOneStep.m_RightFootPosition.y +
                              m_AnklePositionRight[1]);
  vec_db[2] = filterprecision(anOneStep.m_RightFootPosition.z +
                              m_AnklePositionRight[2]);
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevRightAnklePos);

  prefix = aTestName + "RightAnkledPos";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_RightFootPosition.dx);
  vec_db[1] = filterprecision(anOneStep.m_RightFootPosition.dy);
  vec_db[2] = filterprecision(anOneStep.m_RightFootPosition.dz);
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevRightAnkledPos);

  prefix = aTestName + "RightAnkleddPos";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_RightFootPosition.ddx);
  vec_db[1] = filterprecision(anOneStep.m_RightFootPosition.ddy);
  vec_db[2] = filterprecision(anOneStep.m_RightFootPosition.ddz);
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevRightAnkleddPos);

  prefix = aTestName + "RightAnkleOrientation";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_RightFootPosition.theta);
  vec_db[1] = filterprecision(anOneStep.m_RightFootPosition.omega);
  vec_db[2] = filterprecision(anOneStep.m_RightFootPosition.omega2);
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevRightAnkleOrientation);

  prefix = aTestName + "LeftAnkledOrientation";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_LeftFootPosition.dtheta);
  vec_db[1] = 0.0;
  vec_db[2] = 0.0; // 27
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevLeftAnkledOrientation);

  prefix = aTestName + "LeftAnkleddOrientation";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_LeftFootPosition.ddtheta);
  vec_db[1] = 0.0;
  vec_db[2] = 0.0; // 27
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevLeftAnkleddOrientation);

  prefix = aTestName + "RightAnkledOrientation";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_RightFootPosition.dtheta);
  vec_db[1] = 0.0;
  vec_db[2] = 0.0;
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevRightAnkledOrientation);

  prefix = aTestName + "RightAnkleddOrientation";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(anOneStep.m_RightFootPosition.ddtheta);
  vec_db[1] = 0.0;
  vec_db[2] = 0.0;
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevRightAnkleddOrientation);

  prefix = aTestName + "ZMPWorldRef";
  prepareFile(aof, prefix, anOneStep);
  vec_db.resize(2);
  vec_db[0] = filterprecision(localZMPx);
  vec_db[1] = filterprecision(localZMPy);
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevZMPlocal);
}

void DumpReferencesObjects::fillInTestsFormat2(
    std::string &aTestName, OneStep &anOneStep,
    Eigen::VectorXd &aCurrentConfiguration)

{
  ofstream aof;
  string prefix = aTestName + "CoM";
  std::vector<double> vec_db;
  double dt = 0.005;
  double nb_sampling = 5.0;

  double localZMPx = anOneStep.m_ZMPTarget(0) * cos(aCurrentConfiguration(5)) -
                     anOneStep.m_ZMPTarget(1) * sin(aCurrentConfiguration(5)) +
                     aCurrentConfiguration(0);
  double localZMPy = anOneStep.m_ZMPTarget(0) * sin(aCurrentConfiguration(5)) +
                     anOneStep.m_ZMPTarget(1) * cos(aCurrentConfiguration(5)) +
                     aCurrentConfiguration(1);

  /// CoM Position
  prepareFile(aof, prefix, anOneStep);
  vec_db.resize(9);
  vec_db[0] = filterprecision(anOneStep.m_finalCOMPosition.x[0]);
  vec_db[1] = filterprecision(anOneStep.m_finalCOMPosition.y[0]);
  vec_db[2] = filterprecision(anOneStep.m_finalCOMPosition.z[0]);
  vec_db[3] = filterprecision(anOneStep.m_finalCOMPosition.x[1]);
  vec_db[4] = filterprecision(anOneStep.m_finalCOMPosition.y[1]);
  vec_db[5] = filterprecision(anOneStep.m_finalCOMPosition.z[1]);
  vec_db[6] = filterprecision(anOneStep.m_finalCOMPosition.x[2]);
  vec_db[7] = filterprecision(anOneStep.m_finalCOMPosition.y[2]);
  vec_db[8] = filterprecision(anOneStep.m_finalCOMPosition.z[2]);

  TestSuite::FillingFileArgs_t aSetOfFillingFileArgs(aof, dt, nb_sampling,
                                                     anOneStep);
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db, m_prevCoMpF2);

  /// Waist Orientation
  prefix = aTestName + "WaistOrientation";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] =
      filterprecision(anOneStep.m_finalCOMPosition.yaw[0] * M_PI / 180.0);
  vec_db[1] = 0.0;
  vec_db[2] = 0.0;
  vec_db[3] = filterprecision(anOneStep.m_finalCOMPosition.yaw[1]);
  vec_db[4] = 0.0;
  vec_db[5] = 0.0;
  vec_db[6] = filterprecision(anOneStep.m_finalCOMPosition.yaw[2]);
  vec_db[7] = 0.0;
  vec_db[8] = 0.0;
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevWaistOrienF2);

  prefix = aTestName + "ZMPWorldRef";
  prepareFile(aof, prefix, anOneStep);
  vec_db[0] = filterprecision(localZMPx);
  vec_db[1] = filterprecision(localZMPy);
  vec_db[2] = 0.0;
  for (unsigned int j = 3; j < 9; j++)
    vec_db[j] = 0.0;

  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevZMPlocalF2);

  prefix = aTestName + "LeftFoot";
  prepareFile(aof, prefix, anOneStep);
  vec_db.resize(18);
  vec_db[0] =
      filterprecision(anOneStep.m_LeftFootPosition.x + m_AnklePositionLeft[0]);
  vec_db[1] =
      filterprecision(anOneStep.m_LeftFootPosition.y + m_AnklePositionLeft[1]);
  vec_db[2] =
      filterprecision(anOneStep.m_LeftFootPosition.z + m_AnklePositionLeft[2]);
  vec_db[3] =
      filterprecision(anOneStep.m_LeftFootPosition.theta * M_PI / 180.0);
  vec_db[4] = filterprecision(anOneStep.m_LeftFootPosition.omega);
  vec_db[5] = filterprecision(anOneStep.m_LeftFootPosition.omega2);
  vec_db[6] = filterprecision(anOneStep.m_LeftFootPosition.dx);
  vec_db[7] = filterprecision(anOneStep.m_LeftFootPosition.dy);
  vec_db[8] = filterprecision(anOneStep.m_LeftFootPosition.dz);
  vec_db[9] = filterprecision(anOneStep.m_LeftFootPosition.dtheta);
  vec_db[10] = 0;
  vec_db[11] = 0;
  vec_db[12] = filterprecision(anOneStep.m_LeftFootPosition.ddx);
  vec_db[13] = filterprecision(anOneStep.m_LeftFootPosition.ddy);
  vec_db[14] = filterprecision(anOneStep.m_LeftFootPosition.ddz);
  vec_db[15] = filterprecision(anOneStep.m_LeftFootPosition.ddtheta);
  vec_db[16] = 0.0;
  vec_db[17] = 0.0;

  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevLeftAnklePosF2);

  prefix = aTestName + "RightFoot";
  prepareFile(aof, prefix, anOneStep);
  vec_db.resize(18);
  vec_db[0] = filterprecision(anOneStep.m_RightFootPosition.x +
                              m_AnklePositionRight[0]);
  vec_db[1] = filterprecision(anOneStep.m_RightFootPosition.y +
                              m_AnklePositionRight[1]);
  vec_db[2] = filterprecision(anOneStep.m_RightFootPosition.z +
                              m_AnklePositionRight[2]);
  vec_db[3] =
      filterprecision(anOneStep.m_RightFootPosition.theta * M_PI / 180.0);
  vec_db[4] = filterprecision(anOneStep.m_RightFootPosition.omega);
  vec_db[5] = filterprecision(anOneStep.m_RightFootPosition.omega2);
  vec_db[6] = filterprecision(anOneStep.m_RightFootPosition.dx);
  vec_db[7] = filterprecision(anOneStep.m_RightFootPosition.dy);
  vec_db[8] = filterprecision(anOneStep.m_RightFootPosition.dz);
  vec_db[9] = filterprecision(anOneStep.m_RightFootPosition.dtheta);
  vec_db[10] = 0;
  vec_db[11] = 0;
  vec_db[12] = filterprecision(anOneStep.m_RightFootPosition.ddx);
  vec_db[13] = filterprecision(anOneStep.m_RightFootPosition.ddy);
  vec_db[14] = filterprecision(anOneStep.m_RightFootPosition.ddz);
  vec_db[15] = filterprecision(anOneStep.m_RightFootPosition.ddtheta);
  vec_db[16] = 0.0;
  vec_db[17] = 0.0;

  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevRightAnklePosF2);

  prefix = aTestName + "Torques";
  prepareFile(aof, prefix, anOneStep);
  Eigen::VectorXd &aTauVec = anOneStep.m_DebugPR->currentTau();
  vec_db.resize(aTauVec.size());
  if (m_prevTorquesF2.size() != (std::vector<double>::size_type)aTauVec.size())
    m_prevTorquesF2.resize(aTauVec.size());

  for (Eigen::VectorXd::Index i = 0; i < aTauVec.size(); i++)
    vec_db[i] = aTauVec[i];

  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevTorquesF2);

  // Store the phase.
  prefix = aTestName + "Phase";
  prepareFile(aof, prefix, anOneStep);
  vec_db.resize(3);
  vec_db[0] = 0;
  if (anOneStep.m_LeftFootPosition.stepType==-1)
    vec_db[0] = 1.0;
  else if (anOneStep.m_RightFootPosition.stepType==-1)
    vec_db[0] =-1.0;

  vec_db[1] = 0;
  vec_db[2] = 0;
  if (m_prevPhase.size()!=3)
    m_prevPhase.resize(3);
  aof.unsetf(std::ios::floatfield );
  aof.precision(1);
  fillFileWithSubsamplingAndClose(aSetOfFillingFileArgs, vec_db,
                                  m_prevPhase);

}

/*
 * Copyright 2010,
 *
 * Andrei  Herdt
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

#ifdef UNIX
#include <stdlib.h>
#include <sys/time.h>
#endif /*UNIX*/

#ifdef WIN32
#include "portability/gettimeofday.hh"
#include <Windows.h>
#endif /*WIN32*/

#include <fstream>
#include <sstream>

#include "CommonTools.hh"
#include "Debug.hh"

#include <jrl/walkgen/patterngeneratorinterface.hh>

#include "TestFootPrintPGInterfaceData.h"

using namespace std;

namespace PatternGeneratorJRL {
namespace TestSuite {

double filterprecision(double adb) {
  if (fabs(adb) < 1e-7)
    return 0.0;

  double ladb2 = adb * 1e7;
  double lintadb2 = trunc(ladb2);
  return lintadb2 / 1e7;
}

void CommonInitialization(PatternGeneratorInterface &aPGI) {
  const unsigned int nbMethod = 13;
  const char lBuffer[nbMethod][256] = {
      ":comheight 0.876681",
      ":samplingperiod 0.005",
      ":previewcontroltime 1.6",
      ":omega 0.0",
      ":stepheight 0.07",
      ":singlesupporttime 0.78",
      ":doublesupporttime 0.02",
      ":armparameters 0.5",
      ":LimitsFeasibility 0.0",
      ":ZMPShiftParameters 0.015 0.015 0.015 0.015",
      ":TimeDistributionParameters 2.0 3.7 1.7 3.0",
      ":UpperBodyMotionParameters -0.1 -1.0 0.0",
      ":useDynamicFilter false"};

  for (unsigned int i = 0; i < nbMethod; i++) {
    std::istringstream strm(lBuffer[i]);
    aPGI.ParseCmd(strm);
  }
  // Evaluate current state of the robot in the PG.
  COMState lStartingCOMPosition;
  Eigen::Vector3d lStartingZMPPosition;
  Eigen::Matrix<double, 6, 1> lStartingWaistPose;
  FootAbsolutePosition InitLeftFootAbsPos;
  FootAbsolutePosition InitRightFootAbsPos;

  aPGI.EvaluateStartingState(lStartingCOMPosition, lStartingZMPPosition,
                             lStartingWaistPose, InitLeftFootAbsPos,
                             InitRightFootAbsPos);

  ODEBUG("Starting COM Position: " << lStartingCOMPosition.x[0] << " "
                                   << lStartingCOMPosition.y[0] << " "
                                   << lStartingCOMPosition.z[0]);

  ODEBUG("Starting Waist Position: " << lStartingWaistPose[0] << " "
                                     << lStartingWaistPose[1] << " "
                                     << lStartingWaistPose[2]);

  ODEBUG("Starting ZMP Position: " << lStartingZMPPosition[0] << " "
                                   << lStartingZMPPosition[1] << " "
                                   << lStartingZMPPosition[2]);

  ODEBUG("Starting Left Foot Pos: "
         << InitLeftFootAbsPos.x << " " << InitLeftFootAbsPos.y << " "
         << InitLeftFootAbsPos.z << " " << InitLeftFootAbsPos.theta << " "
         << InitLeftFootAbsPos.omega << " " << InitLeftFootAbsPos.omega2);

  ODEBUG("Starting Right Foot Pos: "
         << InitRightFootAbsPos.x << " " << InitRightFootAbsPos.y << " "
         << InitRightFootAbsPos.z << " " << InitRightFootAbsPos.theta << " "
         << InitRightFootAbsPos.omega << " " << InitRightFootAbsPos.omega2);
}

void getOptions(int argc, char *argv[], string &urdfFullPath,
                string &srdfFullPath,
                unsigned int &) // TestProfil)
{
  std::cout << "Nb of entries: " << argc << std::endl;
  if (argc != 3) {
    urdfFullPath = URDF_FULL_PATH;
    srdfFullPath = SRDF_FULL_PATH;
  } else {
    urdfFullPath = argv[1];
    srdfFullPath = argv[2];
  }
}

OneStep::OneStep() {
  m_PR = 0;
  m_ZMPTarget.resize(3);
  m_NbOfIt = 0;
  memset(&m_LeftFootPosition, 0, sizeof(m_LeftFootPosition));
  memset(&m_RightFootPosition, 0, sizeof(m_RightFootPosition));
  memset(&m_finalCOMPosition, 0, sizeof(m_finalCOMPosition));

  fillInDebugVectorDoc();
}

void OneStep::fillInDebugVectorDoc() {
  /// Building vector of documentation strings.
  const char *docInit[15] = {"Time",
                             "CoM - X",
                             "CoM - Y",
                             "CoM - Z",
                             "Yaw",
                             "dCoM - X",
                             "dCoM - Y",
                             "dCoM - Z",
                             "dCoM - Yaw",
                             "ddCoM - X",
                             "ddCoM - Y",
                             "ddCoM - Z",
                             "ddCoM - Yaw",
                             "ZMP X in Waist Ref",
                             "ZMP Y in Waist Ref"};

  for (std::size_t i = 0; i < 15; i++)
    m_DebugStrings.push_back(docInit[i]);

  const char *footNames[2] = {"Left Foot", "Right Foot"};
  const char *docFootInit[14] = {
      " - Pos X",  " - Pos Y",   " - Pos Z",   " - Vel dX",  " - Vel dY",
      " - Vel dZ", " - Acc ddX", " - Acc ddY", " - Acc ddZ", " - Yaw",
      " - dYaw",   " - ddYaw",   " - Roll",    " - Pitch"};
  for (std::size_t footId = 0; footId < 2; footId++) {
    for (std::size_t docFootId = 0; docFootId < 14; docFootId++) {
      std::string aDoc = footNames[footId];
      aDoc = aDoc + " ";
      aDoc = aDoc + std::string(docFootInit[docFootId]);
      m_DebugStrings.push_back(aDoc);
    }
  }

  const char *docInit2[2] = {"ZMP Target X - world ref",
                             " ZMP Target Y - world ref"};

  for (std::size_t i = 0; i < 2; i++)
    m_DebugStrings.push_back(docInit2[i]);

  if (m_PR != 0) {
    Eigen::VectorXd &currentConfiguration = m_PR->currentRPYConfiguration();

    for (unsigned int i = 0; i < currentConfiguration.size(); i++) {
      std::ostringstream aDoc(" conf. ");
      aDoc << i;
      m_DebugStrings.push_back(aDoc.str());
    }
  }
}

void OneStep::fillInDebugVectorFoot(FootAbsolutePosition &aFootAbsolutePosition,
                                    std::size_t &lindex) {
  m_DebugVector[lindex++] = aFootAbsolutePosition.x;
  m_DebugVector[lindex++] = aFootAbsolutePosition.y;
  m_DebugVector[lindex++] = aFootAbsolutePosition.z;
  m_DebugVector[lindex++] = aFootAbsolutePosition.dx;
  m_DebugVector[lindex++] = aFootAbsolutePosition.dy;
  m_DebugVector[lindex++] = aFootAbsolutePosition.dz;
  m_DebugVector[lindex++] = aFootAbsolutePosition.ddx;
  m_DebugVector[lindex++] = aFootAbsolutePosition.ddy;
  m_DebugVector[lindex++] = aFootAbsolutePosition.ddz;
  m_DebugVector[lindex++] = aFootAbsolutePosition.theta;
  m_DebugVector[lindex++] = aFootAbsolutePosition.dtheta;
  m_DebugVector[lindex++] = aFootAbsolutePosition.ddtheta;
  m_DebugVector[lindex++] = aFootAbsolutePosition.omega;
  m_DebugVector[lindex++] = aFootAbsolutePosition.omega2;
}

void OneStep::fillInDebugVector() {
  std::size_t lindex = 0;
  Eigen::VectorXd &currentConfiguration = m_PR->currentRPYConfiguration();

  Eigen::Index nq = currentConfiguration.size();
  if (m_DebugVector.size() == 0)
    m_DebugVector.resize(17 + 14 * 2 + nq);

  /// Time
  m_DebugVector[lindex++] = ((double)m_NbOfIt) * 0.005;
  /// CoM Position
  m_DebugVector[lindex++] = m_finalCOMPosition.x[0];
  m_DebugVector[lindex++] = m_finalCOMPosition.y[0];
  m_DebugVector[lindex++] = m_finalCOMPosition.z[0];
  m_DebugVector[lindex++] = m_finalCOMPosition.yaw[0];
  m_DebugVector[lindex++] = m_finalCOMPosition.x[1];
  m_DebugVector[lindex++] = m_finalCOMPosition.y[1];
  m_DebugVector[lindex++] = m_finalCOMPosition.z[1];
  m_DebugVector[lindex++] = m_finalCOMPosition.yaw[1];
  m_DebugVector[lindex++] = m_finalCOMPosition.x[2];
  m_DebugVector[lindex++] = m_finalCOMPosition.y[2];
  m_DebugVector[lindex++] = m_finalCOMPosition.z[2];
  m_DebugVector[lindex++] = m_finalCOMPosition.yaw[2];

  /// ZMP Target
  m_DebugVector[lindex++] = m_ZMPTarget(0);
  m_DebugVector[lindex++] = m_ZMPTarget(1);
  /// Left Foot Position
  fillInDebugVectorFoot(m_LeftFootPosition, lindex);
  /// Right Foot position
  fillInDebugVectorFoot(m_RightFootPosition, lindex);

  assert(m_PR != NULL);

  /// ZMP Target in absolute reference
  m_DebugVector[lindex++] = m_ZMPTarget(0) * cos(currentConfiguration(5)) -
                            m_ZMPTarget(1) * sin(currentConfiguration(5)) +
                            currentConfiguration(0);
  m_DebugVector[lindex++] = m_ZMPTarget(0) * sin(currentConfiguration(5)) +
                            m_ZMPTarget(1) * cos(currentConfiguration(5)) +
                            currentConfiguration(1);

  /// Saving configuration
  for (unsigned int i = 0; i < currentConfiguration.size(); i++)
    m_DebugVector[lindex++] = currentConfiguration(i);
}

void OneStep::fillInDebugFileContent(std::ofstream &aof) {
  for (std::size_t i = 0; i < m_DebugVector.size(); i++)
    aof << filterprecision(m_DebugVector[i]) << " ";
}

void OneStep::fillInDebugFile() {
  /// Store data
  fillInDebugVector();

  ofstream aof;

  /// Create filename
  string aFileName;
  assert(!m_TestName.empty());
  aFileName = m_TestName;
  aFileName += "TestFGPI.dat";

  if (m_NbOfIt == 0) {
    /// Write description file if this is the first iteration
    writeDescriptionFile();

    /// Erase the file if we start.
    aof.open(aFileName.c_str(), ofstream::out);
    aof.close();
  }

  /// Open file additively
  aof.open(aFileName.c_str(), ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  /// Write down the file
  fillInDebugFileContent(aof);
  aof << endl;
  aof.close();
}

void OneStep::writeDescriptionFile() {
  ofstream aof;
  string aFileName;
  assert(!m_TestName.empty());
  aFileName = m_TestName;
  aFileName += "TestFGPI_description.dat";
  aof.open(aFileName.c_str(), ofstream::out);

  for (std::size_t i = 0; i < m_DebugStrings.size(); i++)
    aof << i << " - " << m_DebugStrings[i] << std::endl;
  aof.close();
}
} // namespace TestSuite
} // namespace PatternGeneratorJRL

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
#include <string>
#include <time.h>

#include "ClockCPUTime.hh"
#include <jrl/walkgen/patterngeneratorinterface.hh>

#ifndef _COMMON_TOOLS_PATTERN_GENERATOR_UTESTING_H_
#define _COMMON_TOOLS_PATTERN_GENERATOR_UTESTING_H_

namespace PatternGeneratorJRL {
namespace TestSuite {
double filterprecision(double adb);

void getOptions(int argc, char *argv[], std::string &urdfFullPath,
                std::string &srdfFullPath,
                unsigned int &); // TestProfil)

void CommonInitialization(PatternGeneratorJRL::PatternGeneratorInterface &aPGI);

/*! \brief Structure to handle information related
  to one step of each algorithm m_*/
class OneStep {
public:
  COMState m_finalCOMPosition;
  FootAbsolutePosition m_LeftFootPosition;
  FootAbsolutePosition m_RightFootPosition;
  Eigen::VectorXd m_ZMPTarget;
  unsigned long int m_NbOfIt;
  Eigen::VectorXd m_Tau;

  /// Vector of data to dump
  std::vector<double> m_DebugVector;
  /// Vector of documentation string
  std::vector<std::string> m_DebugStrings;
  /// TestName
  std::string m_TestName;

  /// Pinocchio Robot for the Pattern generator
  PinocchioRobot *m_PR;
  /// Pointer towards the debugging robot.
  PinocchioRobot *m_DebugPR;

  OneStep();

  /// Filling m_DebugVector
  void fillInDebugVectorFoot(FootAbsolutePosition &aFootAbsolutePosition,
                             size_t &index);

  /// Filling documentation vector
  void fillInDebugVectorDoc();

  /// Filling files based on m_DebugVector.
  void fillInDebugFileContent(std::ofstream &aof);

  /// Creates a file based on filename.
  void fillInDebugFile();

  /// Fill in the debug vector.
  void fillInDebugVector();

  /// Writes down the description vector.
  void writeDescriptionFile();
};
} // namespace TestSuite
} // namespace PatternGeneratorJRL
#endif /* _COMMON_TOOLS_PATTERN_GENERATOR_UTESTING_H_*/

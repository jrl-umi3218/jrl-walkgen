/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Florent Lamiraux
 * Mathieu Poirier
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
/** @doc This object is in charge of dealing with the upper body
    motion of the robot.
*/

#ifndef _UPPER_BODY_MOTION_
#define _UPPER_BODY_MOTION_

#include <string>
#include <vector>
//#define FULL_POLYNOME

using namespace ::std;

#include <Eigen/Dense>
#include <Mathematics/PolynomeFoot.hh>

namespace PatternGeneratorJRL {

class UpperBodyMotion {
public:
  /// Constructor
  UpperBodyMotion();

  /// Destructor
  ~UpperBodyMotion();

  void GenerateDataFile(string aFileName, int LenghtDataArray);

  void ReadDataFile(string aFileName, Eigen::MatrixXd &UpperBodyAngles);

  void WriteDataFile(string aFileName, Eigen::MatrixXd &UpperBodyAngles);

protected:
};
} // namespace PatternGeneratorJRL
#endif /* _UPPER_BODY_MOTION_*/

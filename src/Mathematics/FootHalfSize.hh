/*
 * Copyright 2010,
 *
 * Olivier    Stasse
 *
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

#ifndef _FOOT_HALF_SIZE_H_
#define _FOOT_HALF_SIZE_H_

namespace PatternGeneratorJRL {

/*! This class handles the size and the constraints related to the feet
 */
class FootHalfSize {
public:
  FootHalfSize();
  ~FootHalfSize();

  void setSize(double lWidth, double lHeight, double DSFeetDistance);

  void setConstraints(double OnX, double OnY, double DSFeetDistance);

protected:
  void updateHalfHeightDS(double DSFeetDistance);
  void updateHalfSize();

private:
  double m_HalfHeight;
  double m_HalfWidth;
  double m_HalfHeightDS;
  double m_Height;
  double m_Width;

  double m_ConstraintsOnX;
  double m_ConstraintsOnY;

public:
  double getHalfHeight() const;
  double getHalfWidth() const;
  double getHalfHeightDS() const;
};
} // namespace PatternGeneratorJRL

#endif /*  _FOOT_HALF_SIZE_H_*/

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

#include <Mathematics/FootHalfSize.hh>

using namespace PatternGeneratorJRL;
#include <iostream>
using namespace std;

FootHalfSize::FootHalfSize()
{
  m_HalfWidth = -1.0;
  m_HalfHeight = -1.0;
  m_Width = -1.0;
  m_Height = -1.0;
  m_HalfHeightDS = -1.0;

  // Variation around X < 1 cm by default.
  m_ConstraintsOnX = 0.01;

  // Variation around Y < 2 cm by default.
  m_ConstraintsOnY = 0.02;
}

FootHalfSize::~FootHalfSize()
{
}

void FootHalfSize::setSize(double lWidth,
                           double lHeight,
                           double DSFeetDistance)
{
  m_Height = lHeight;
  m_Width = lWidth;

  updateHalfSize();
  updateHalfHeightDS(DSFeetDistance);
}

void FootHalfSize::updateHalfSize()
{
  m_HalfWidth = 0.5 * m_Width;
  m_HalfHeight = 0.5 * m_Height;

  m_HalfWidth -= m_ConstraintsOnX;
  m_HalfHeight -= m_ConstraintsOnY;
}

void FootHalfSize::updateHalfHeightDS(double DSFeetDistance)
{
  m_HalfHeightDS = m_HalfHeight+DSFeetDistance/2.0;
}

void FootHalfSize::setConstraints(double OnX, double OnY, double DSFeetDistance)
{
  m_ConstraintsOnX = OnX;
  m_ConstraintsOnY = OnY;

  updateHalfSize();
  updateHalfHeightDS(DSFeetDistance);

}

double FootHalfSize::getHalfHeight() const
{
  return m_HalfHeight;
}

double FootHalfSize::getHalfWidth() const
{
  return m_HalfWidth;
}

double FootHalfSize::getHalfHeightDS() const
{
  return m_HalfHeightDS;
}


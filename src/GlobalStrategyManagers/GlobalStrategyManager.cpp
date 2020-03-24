/*
 * Copyright 2009, 2010,
 *
 * Olivier  Stasse
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
/*! \file GlobalStrategyManager.cpp
  \brief This object defines a global strategy abstract object to generate
  an output handled by the PatternGeneratorInterface object.
*/

#include <Debug.hh>
#include <GlobalStrategyManagers/GlobalStrategyManager.hh>
#include <deque>

using namespace PatternGeneratorJRL;

GlobalStrategyManager::GlobalStrategyManager(
    SimplePluginManager *aPluginManager)
    : SimplePlugin(aPluginManager) {}

void GlobalStrategyManager::SetBufferPositions(
    deque<ZMPPosition> *aZMPPositions, deque<COMState> *aCOMBuffer,
    deque<FootAbsolutePosition> *aLeftFootAbsolutePositions,
    deque<FootAbsolutePosition> *aRightFootAbsolutePositions) {
  m_ZMPPositions = aZMPPositions;
  m_COMBuffer = aCOMBuffer;
  m_LeftFootPositions = aLeftFootAbsolutePositions;
  m_RightFootPositions = aRightFootAbsolutePositions;
}

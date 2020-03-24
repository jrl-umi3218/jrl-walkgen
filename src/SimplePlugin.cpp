/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Nicolas    Perrin

 * Olivier    Stasse
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
/*! Abstract class to handle generic calls from the
  Pattern Generator Interface.
*/

#include <SimplePlugin.hh>
#include <SimplePluginManager.hh>

using namespace std;
using namespace PatternGeneratorJRL;
bool SimplePlugin::RegisterMethod(string &MethodName) {
  bool r = false;
  if (m_SimplePluginManager != 0)
    r = m_SimplePluginManager->RegisterMethod(MethodName, this);
  return r;
}

SimplePlugin::~SimplePlugin() {
  if (m_SimplePluginManager != 0) {
    m_SimplePluginManager->UnregisterPlugin(this);
  }
}

/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Paul       Evrard
 * Andrei     Herdt
 * Francois   Keith
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

/*! Abstract class to handle generic calls for the
  Pattern Generator Interface.*/

#include <iostream>

using namespace std;
#include <Debug.hh>
#include <SimplePlugin.hh>
#include <SimplePluginManager.hh>
#include <assert.h>

using namespace PatternGeneratorJRL;

SimplePluginManager::SimplePluginManager() { RESETDEBUG5("PgDebug.txt"); }

SimplePluginManager::~SimplePluginManager() {
  std::multimap<std::string, SimplePlugin *, ltstr>::iterator it_SP;

  it_SP = m_SimplePlugins.begin();
  while (it_SP != m_SimplePlugins.end()) {
    it_SP->second->m_SimplePluginManager = 0;
    it_SP++;
  }
}

void SimplePluginManager::UnregisterPlugin(SimplePlugin *aSimplePlugin) {
  std::multimap<std::string, SimplePlugin *, ltstr>::iterator it_SP,
      it_ToBeRemoved;

  it_SP = m_SimplePlugins.begin();
  while (it_SP != m_SimplePlugins.end()) {

    if (it_SP->second == aSimplePlugin) {
      it_ToBeRemoved = it_SP;
      it_SP++;

      m_SimplePlugins.erase(it_ToBeRemoved);
    } else
      it_SP++;
  }
}

void SimplePluginManager::Print() {
  std::multimap<std::string, SimplePlugin *, ltstr>::iterator it_SP;

  it_SP = m_SimplePlugins.begin();
  while (it_SP != m_SimplePlugins.end()) {
    cout << it_SP->first << endl;
    it_SP++;
  }
}

/*! \name Register the method for which this object can be called
  by a higher parser. */
bool SimplePluginManager::RegisterMethod(string &MethodName,
                                         SimplePlugin *aSP) {
  /* We have to copy the name, because we don't know
     how is handle the memory towards MethodName.c_str() */

  m_SimplePlugins.insert(pair<string, SimplePlugin *>(MethodName, aSP));

  ODEBUG5("Registered method " << MethodName << " for plugin " << aSP << endl,
          "PgDebug.txt");
  //  Print();
  return true;
}

/*! \name Call the method from the Method name. */
bool SimplePluginManager::CallMethod(string &MethodName, istringstream &istrm) {
  pair<std::multimap<std::string, SimplePlugin *, ltstr>::iterator,
       std::multimap<std::string, SimplePlugin *, ltstr>::iterator>
      RangeOfPlugins = m_SimplePlugins.equal_range(MethodName);

  std::multimap<std::string, SimplePlugin *, ltstr>::iterator CurrentPlugin;

  unsigned int NbPlugins = 0;
  for (CurrentPlugin = RangeOfPlugins.first;
       CurrentPlugin != RangeOfPlugins.second; ++NbPlugins, ++CurrentPlugin)
    ;

  ODEBUG5("Size of SimplePlugins: " << m_SimplePlugins.size() << " Found for "
                                    << MethodName << " : " << NbPlugins,
          "PgDebug.txt");
  bool FoundAPlugin = false;

  stringbuf *pbuf;
  pbuf = istrm.rdbuf();

  auto size = pbuf->in_avail();
  char aBuffer[65636];
  assert(size < 65635);

  memset(aBuffer, 0, size + 1);
  for (int i = 0; i < size; i++)
    aBuffer[i] = (char)pbuf->sbumpc();
  ODEBUG5(aBuffer, "PgDebug.txt");

  pbuf->pubsetbuf(aBuffer, size);

  for (CurrentPlugin = RangeOfPlugins.first;
       CurrentPlugin != RangeOfPlugins.second; ++CurrentPlugin) {
    istringstream iss(aBuffer);
    SimplePlugin *aSP = CurrentPlugin->second;
    ODEBUG5("Found the method " << MethodName << " for plugin :" << aSP << endl
                                << " Buffer: " << aBuffer,
            "PgDebug.txt");
    if (aSP != 0) {
      aSP->CallMethod(MethodName, iss);
      FoundAPlugin = true;
    } else {
      cout << "SimplePlugin empty " << endl;
    }
  }

  return FoundAPlugin;
}

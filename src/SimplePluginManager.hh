/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Florent    Lamiraux
 * Nicolas    Perrin
 * Olivier    Stasse
 * Eiichi     Yoshida
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
/* Abstract class to handle generic calls for the
   Pattern Generator Interface. */

#ifndef _PGI_SIMPLE_PLUGIN_MANAGER_H_
#define _PGI_SIMPLE_PLUGIN_MANAGER_H_

#include <string.h>

#include <map>
#include <iostream>
#include <string>
#include <sstream>



namespace PatternGeneratorJRL
{
  class SimplePlugin;


  /*! This class takes care of dispatching the call
    appropriatly.

  */
  class SimplePluginManager
  {


  public:

    struct ltstr
    {
      bool operator()(const std::string s1, const std::string s2) const
      {
        return strcmp(s1.c_str(), s2.c_str()) < 0;
      }
    };


  protected:

    /*! Set of plugins sorted by names */
    std::multimap<std::string, SimplePlugin *, ltstr>  m_SimplePlugins;

  public:

    /*! \brief Pointer towards the PGI which is handling this object. */
    SimplePluginManager();
    virtual ~SimplePluginManager();

    /*! Get the map of plugins. */
    const std::multimap<std::string, SimplePlugin *, ltstr>  &
    getSimplePlugins()
      const
    {
      return m_SimplePlugins;
    };

    /*! \name Register the method for which this object can be called
      by a higher parser. */
    bool RegisterMethod(std::string &MethodName, SimplePlugin *aSP);

    /*! \name Unregister a plugin */
    void UnregisterPlugin(SimplePlugin *aSP);

    /*! \name Call the method from the Method name. */
    bool CallMethod(std::string &MethodName, std::istringstream &istrm);

    /*! \name Operator to display in cout. */
    void Print();

  };
}

#include <SimplePlugin.hh>

#endif /* _PGI_SIMPLE_PLUGIN_H_ */

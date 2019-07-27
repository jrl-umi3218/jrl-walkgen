/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Florent    Lamiraux
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

#ifndef _PGI_SIMPLE_PLUGIN_H_
#define _PGI_SIMPLE_PLUGIN_H_

#include <string>
#include <sstream>


namespace PatternGeneratorJRL
{

  class SimplePluginManager;

  /*! Each object derivating of this class are supposed
    to populate a PGI instance to handle approprietly
    parsing from the PGI.
  */
  class SimplePlugin
  {

  private:
    SimplePluginManager * m_SimplePluginManager;
    friend class SimplePluginManager;

  public:

    /*! \brief Pointer towards the PGI which is handling this object. */
    inline SimplePlugin(SimplePluginManager * lSPM)
      : m_SimplePluginManager(lSPM)
    {};

    virtual ~SimplePlugin();

    /*! \name Register the method for which this object can be called
      by a higher parser. */
    bool RegisterMethod(std::string &MethodName);

    /*! \name Virtual method to redispatch the method. */
    virtual void CallMethod
    (std::string &Method, std::istringstream & astrm) = 0;

    /*! \name Get the simple plugin manager */
    SimplePluginManager * getSimplePluginManager() const
    {
      return m_SimplePluginManager;
    }
  };
}
#include <SimplePluginManager.hh>
#endif /* _PGI_SIMPLE_PLUGIN_H_ */

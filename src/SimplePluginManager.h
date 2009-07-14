/* Abstract class to handle generic calls for the 
   Pattern Generator Interface.

   Copyright (c) 2007-2009, 
   @author  Olivier Stasse, Francois Keith.
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see License.txt for further information on license.
*/

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
	return strcmp((char*)s1.c_str(), (char*)s2.c_str()) < 0;
      }
    };


  protected:

    /*! Set of plugins sorted by names */
    std::multimap<std::string, SimplePlugin *, ltstr>  m_SimplePlugins;

  public: 
    
    /*! \brief Pointer towards the PGI which is handling this object. */
    inline SimplePluginManager()
      {};
    virtual ~SimplePluginManager();
    
    /*! Get the map of plugins. */
    const std::multimap<std::string, SimplePlugin *, ltstr>  & getSimplePlugins() const
      { return m_SimplePlugins;};

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
};

#include <SimplePlugin.h>

#endif /* _PGI_SIMPLE_PLUGIN_H_ */

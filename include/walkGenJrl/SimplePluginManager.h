/* Abstract class to handle generic calls for the 
   Pattern Generator Interface.

   Copyright (c) 2007, 
   @author Francois Keith, Olivier Stasse.
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or 
   other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _PGI_SIMPLE_PLUGIN_MANAGER_H_
#define _PGI_SIMPLE_PLUGIN_MANAGER_H_

#include <string.h>

#include <map>
#include <iostream>
#include <string>
#include <sstream>

#include <walkGenJrl/walkGenJrl_API.h>

namespace PatternGeneratorJRL
{
  class SimplePlugin;

  
  /*! This class takes care of dispatching the call 
    appropriatly.
    
  */
  class WALK_GEN_JRL_EXPORT SimplePluginManager
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

#include <walkGenJrl/SimplePlugin.h>

#endif /* _PGI_SIMPLE_PLUGIN_H_ */

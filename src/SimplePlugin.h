/* Abstract class to handle generic calls from the 
   Pattern Generator Interface.

   Copyright (c) 2007, 
   @author Francois Keith, Olivier Stasse.
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   For more information on the license please look at License.txt 
   in the root directory.
   
*/

#ifndef _PGI_SIMPLE_PLUGIN_H_
#define _PGI_SIMPLE_PLUGIN_H_

#include <string>
#include <sstream>

#include <walkGenJrl/walkGenJrl_API.h>

namespace PatternGeneratorJRL
{
  
  class SimplePluginManager;

  /*! Each object derivating of this class are supposed
   to populate a PGI instance to handle approprietly
   parsing from the PGI.
  */
  class WALK_GEN_JRL_EXPORT SimplePlugin
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
    virtual void CallMethod(std::string &Method, std::istringstream & astrm) = 0;
    
    /*! \name Get the simple plugin manager */
    SimplePluginManager * getSimplePluginManager() const
      { return m_SimplePluginManager; } 
  };
};
#include <SimplePluginManager.h>
#endif /* _PGI_SIMPLE_PLUGIN_H_ */

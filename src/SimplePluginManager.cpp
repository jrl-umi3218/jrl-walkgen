/* Abstract class to handle generic calls for the 
   Pattern Generator Interface.

   Copyright (c) 2007, 
   @author Olivier Stasse.
   
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

#include <iostream>
#include <walkGenJrl/SimplePlugin.h>
#include <walkGenJrl/SimplePluginManager.h>

#if 0

#define RESETDEBUG4(y) { ofstream DebugFile; \
    DebugFile.open(y,ofstream::out); \
    DebugFile.close();}

#define ODEBUG4(x,y) { ofstream DebugFile; \
    DebugFile.open(y,ofstream::app); \
    DebugFile << "SimplePluginManager: " << x << endl; \
    DebugFile.close();}

#else
#define RESETDEBUG4(y) 
#define ODEBUG4(x,y) 
#endif

#define RESETDEBUG6(y) 
#define ODEBUG6(x,y) 

#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "SimplePluginManager: " << x << endl; DebugFile.close();}
#if 1
#define ODEBUG(x)
#else
#define ODEBUG(x)  std::cout << x << endl;
#endif

#define ODEBUG3(x)  std::cout << x << endl;

using namespace std;
using namespace PatternGeneratorJRL;


SimplePluginManager::~SimplePluginManager()
{
  /*
  std::map<char*, SimplePlugin * , ltstr>::iterator it_SP;
  

  it_SP =m_SimplePlugins.begin();
  while(it_SP!=m_SimplePlugins.end())
    {
      delete it_SP->first;
      it_SP++;
    }
  */
}

void SimplePluginManager::Print( )
{ 
  std::map<std::string, SimplePlugin * , ltstr>::iterator it_SP;
  

  it_SP =m_SimplePlugins.begin();
  while(it_SP!=m_SimplePlugins.end())
    {
      cout << it_SP->first << endl;
      it_SP++;
    }
}


/*! \name Register the method for which this object can be called
  by a higher parser. */
bool SimplePluginManager::RegisterMethod(string &MethodName, SimplePlugin *aSP)
{
  /* We have to copy the name, because we don't know
     how is handle the memory towards MethodName.c_str() */

  /*
  char * lMethodName = new char[MethodName.length()+1];
  memset(lMethodName,0,MethodName.length()+1);
  memcpy(lMethodName,MethodName.c_str(),MethodName.length());
  */
  m_SimplePlugins.insert(pair < string, SimplePlugin * > (MethodName,aSP));

  ODEBUG("Registered method " << MethodName << " for plugin " << aSP << endl);
  //  Print();
  return true;  
}

/*! \name Call the method from the Method name. */
bool SimplePluginManager::CallMethod(string &MethodName, istringstream &istrm)
{
  pair <std::multimap<std::string, SimplePlugin * , ltstr>::iterator,
    std::multimap<std::string, SimplePlugin * , ltstr>::iterator >
    RangeOfPlugins  = m_SimplePlugins.equal_range(MethodName);
  
  
  std::multimap<std::string, SimplePlugin * , ltstr>::iterator CurrentPlugin;

  
  unsigned int NbPlugins=0;
  for (CurrentPlugin = RangeOfPlugins.first;
       CurrentPlugin != RangeOfPlugins.second;++NbPlugins,
	 ++CurrentPlugin);
  
  ODEBUG("Size of SimplePlugins: " << m_SimplePlugins.size() 
	  << " Found for " << MethodName << " : " << NbPlugins);
  bool FoundAPlugin = false;

  stringbuf *pbuf;
  pbuf = istrm.rdbuf();

  int size = pbuf->in_avail();
  char aBuffer[32768];

  memset(aBuffer,0,size+1);
  for(int i=0;i<size;i++)
    aBuffer[i] = (char)pbuf->sbumpc();
  ODEBUG(aBuffer);
  
  pbuf->pubsetbuf(aBuffer,size);

  for (CurrentPlugin = RangeOfPlugins.first;
       CurrentPlugin != RangeOfPlugins.second;
       ++CurrentPlugin)
    {
      istringstream iss(aBuffer);
      SimplePlugin * aSP = CurrentPlugin->second;
      ODEBUG("Found the method " << MethodName << " for plugin :" << aSP ); 
      if (aSP!=0)
	{
	  aSP->CallMethod(MethodName,iss);
	  FoundAPlugin = true;
	}
      else
	{
	  cout << "SimplePlugin empty " << endl;
	}
    }

  return FoundAPlugin;
  
}

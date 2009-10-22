/* Abstract class to handle generic calls for the 
   Pattern Generator Interface.

   Copyright (c) 2007, 
   @author Olivier Stasse.
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please see License.txt for further information on license.            
   
*/

#include <iostream>
#include <SimplePlugin.h>
#include <SimplePluginManager.h>
#include <Debug.h>

using namespace std;
using namespace PatternGeneratorJRL;


SimplePluginManager::~SimplePluginManager()
{
  std::multimap<std::string, SimplePlugin * , ltstr>::iterator it_SP;
  

  it_SP =m_SimplePlugins.begin();
  while(it_SP!=m_SimplePlugins.end())
    {
      it_SP->second->m_SimplePluginManager = 0;
      it_SP++;
    }
}

void SimplePluginManager::UnregisterPlugin(SimplePlugin* aSimplePlugin)
{
  std::multimap<std::string, SimplePlugin * , ltstr>::iterator it_SP, it_ToBeRemoved;

  it_SP =m_SimplePlugins.begin();
  while(it_SP!=m_SimplePlugins.end())
    {

      if (it_SP->second==aSimplePlugin)
	{
	  it_ToBeRemoved = it_SP;
	  it_SP++;

	  m_SimplePlugins.erase(it_ToBeRemoved);
	}
      else 
	it_SP++;
    }
  
}

void SimplePluginManager::Print( )
{ 
  std::multimap<std::string, SimplePlugin * , ltstr>::iterator it_SP;
  

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
      ODEBUG("Found the method " << MethodName 
	      << " for plugin :" << aSP << endl
	      << " Buffer: " << aBuffer); 
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

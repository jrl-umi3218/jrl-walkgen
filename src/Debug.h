/* @doc File for debugging messages.

   Copyright (c) 2009, 
   Olivier Stasse,
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see License.txt for further information on license.      

*/


#define ODEBUG2(x)
#define ODEBUG3(x) std::cerr << __FILE__ << ":" \
                             << __FUNCTION__ << "(#" \
                             << __LINE__ << "):" << x << std::endl;
#define RESETDEBUG5(y) { std::ofstream DebugFile; \
DebugFile.open(y,ofstream::out); \
DebugFile.close();}
#define ODEBUG5(x,y) { std::ofstream DebugFile; \
DebugFile.open(y,ofstream::app); \
DebugFile << __FILE__ << ":" \
          << __FUNCTION__ << "(#" \
          << __LINE__ << "):" << x << std::endl; \
          DebugFile.close();}

#define ODEBUG3_NENDL(x) std::cerr << x

#ifdef _DEBUG_MODE_ON_
#define ODEBUG(x) std::cerr << __FILE__ << ":" \
                            << __FUNCTION__ << "(#" \
                            << __LINE__ << "):" << x << std::endl;

#define ODEBUG_NENDL(x) std::cerr << x

#define ODEBUG_CODE(x) x
#else
#define ODEBUG(x)
#define ODEBUG_NENDL(x) 
#define ODEBUG_CODE(x)
#endif

#ifdef _DEBUG_MODE_ON_
#define RESETDEBUG4(y) { std::ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { std::ofstream DebugFile; DebugFile.open(y,ofstream::app); \
    DebugFile << __FILE__ << ":" \
              << __FUNCTION__ << "(#" \
              << __LINE__ << "):" << x << std::endl; \
    DebugFile.close();}

#define _DEBUG_4_ACTIVATED_ 1
#else
#define RESETDEBUG4(y)
#define ODEBUG4(x,y)
#endif

#define RESETDEBUG6(x)
#define ODEBUG6(x,y)
#define CODEDEBUG6(x)

/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Olivier Stasse
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
/* @doc File for debugging messages. */
#include <iostream>
#include <fstream>
#include <exception>

#define LTHROW(x)                               \
  {                                             \
    class Exception: public std::exception      \
    {                                           \
      virtual const char * what() const throw() \
      {                                         \
        return x;                               \
      }                                         \
    };                                          \
                                                \
    Exception almsg;                            \
    throw almsg; }

#define ODEBUG2(x)
#define ODEBUG3(x) std::cerr << __FILE__ << ":" \
  << __FUNCTION__ << "(#"                       \
  << __LINE__ << "):" << x << std::endl;
#define RESETDEBUG5(y) { std::ofstream DebugFile;       \
    DebugFile.open(y,ofstream::out);                    \
    DebugFile.close();}
#define ODEBUG5(x,y) { std::ofstream DebugFile;         \
    DebugFile.open(y,ofstream::app);                    \
    DebugFile << __FILE__ << ":"                        \
              << __FUNCTION__ << "(#"                   \
              << __LINE__ << "):" << x << std::endl;    \
    DebugFile.close();}
#define ODEBUG5SIMPLE(x,y) { std::ofstream DebugFile;   \
    DebugFile.open(y,ofstream::app);                    \
    DebugFile << x << std::endl;                        \
    DebugFile.close();}

#define ODEBUG3_NENDL(x) std::cerr << x

#ifdef _DEBUG_MODE_ON_
#define ODEBUG(x) std::cerr << __FILE__ << ":"  \
  << __FUNCTION__ << "(#"                       \
  << __LINE__ << "):" << x << std::endl;

#define ODEBUG_NENDL(x) std::cerr << x

#define ODEBUG_CODE(x) x
#else
#define ODEBUG(x)
#define ODEBUG_NENDL(x)
#define ODEBUG_CODE(x)
#endif

#ifdef _DEBUG_MODE_ON_
#define RESETDEBUG4(y) { std::ofstream DebugFile; \
    DebugFile.open(y,ofstream::out); \
    DebugFile.close();}
#define ODEBUG4(x,y) { std::ofstream DebugFile; \
    DebugFile.open(y,ofstream::app);                                    \
    DebugFile << __FILE__ << ":"                                        \
              << __FUNCTION__ << "(#"                                   \
              << __LINE__ << "):" << x << std::endl;                    \
    DebugFile.close();}
#define ODEBUG4SIMPLE(x,y) { std::ofstream DebugFile;   \
    DebugFile.open(y,ofstream::app);                    \
    DebugFile << x << std::endl;                        \
    DebugFile.close();}

#define _DEBUG_4_ACTIVATED_ 1
#else
#define RESETDEBUG4(y)
#define ODEBUG4(x,y)
#define ODEBUG4SIMPLE(x,y)
#endif

#define RESETDEBUG6(x)
#define ODEBUG6(x,y)
#define ODEBUG6SIMPLE(x,y)
#define CODEDEBUG6(x)

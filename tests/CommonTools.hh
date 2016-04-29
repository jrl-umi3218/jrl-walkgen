/*
 * Copyright 2010,
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
#ifdef UNIX
#include <sys/time.h>
#include <stdlib.h>
#endif /*UNIX*/

#ifdef WIN32
#include <Windows.h>
#include "portability/gettimeofday.hh"
#endif /*WIN32*/

#include <time.h>
#include <sstream>
#include <fstream>
#include <string>


#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl/walkgen/patterngeneratorinterface.hh>
#include <jrl/walkgen/config_private.hh>
#include "ClockCPUTime.hh"

#ifndef _COMMON_TOOLS_PATTERN_GENERATOR_UTESTING_H_
#define _COMMON_TOOLS_PATTERN_GENERATOR_UTESTING_H_

namespace PatternGeneratorJRL
{
  namespace TestSuite
  {
    void getOptions(int argc,
                    char *argv[],
                    std::string &urdfFullPath,
                    std::string &srdfFullPath,
                    unsigned int &); // TestProfil)

    void CommonInitialization(PatternGeneratorJRL::PatternGeneratorInterface &aPGI);

    /*! \brief Structure to handle information related to one step of each algorithm */
    struct OneStep
    {
      COMState finalCOMPosition;
      FootAbsolutePosition LeftFootPosition;
      FootAbsolutePosition RightFootPosition;
      MAL_VECTOR(ZMPTarget,double);
      unsigned long int NbOfIt;

      OneStep()
      {
        MAL_VECTOR_RESIZE(ZMPTarget,3);
        NbOfIt = 0;
        memset(&LeftFootPosition,0,sizeof(LeftFootPosition));
        memset(&RightFootPosition,0,sizeof(RightFootPosition));
        memset(&finalCOMPosition,0,sizeof(finalCOMPosition));
      }
    };
  } /* end of TestSuite namespace */
} /* end of PatternGeneratorJRL namespace */
#endif /* _COMMON_TOOLS_PATTERN_GENERATOR_UTESTING_H_*/

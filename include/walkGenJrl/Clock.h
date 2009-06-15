/*! \file Clock.h
    \brief Defines an object for measuring the time spend in some code part.

    SVN Information:
   $Id$
   $Author$
   $Date$
   $Revision $
   $Source $
   $Log $


   Copyright (c) 2005-2008, 
   @author Olivier Stasse, Francois Keith
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS and AIST nor the names of its contributors 
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

#include <walkGenJrl/walkGenJrl_API.h>

#ifdef UNIX
#include <sys/time.h> 
#endif /*UNIX*/

#ifdef WIN32
#include <Windows.h>
#include "walkGenJrl/TimeUtilsWindows.h"
#endif

#include <time.h>
#include <vector>
#include <string>

#ifndef _HWPG_CLOCK_H_
#define _HWPG_CLOCK_H_
namespace PatternGeneratorJRL
{
  /*! \brief Measure time spend in some code.
    The object measure the time between StartTiming() and StopTiming() slots.
    The number of iteration is incremented by IncIteration().
    MaxTime() and AverageTime() returns the maximum time spend in one iteration,
    and the average time spends in one iteration respectively.
    TotalTime() returns the time spend in total in the code measured.
    The precision is expected to be in micro-second but is OS dependent.
  */
  class WALK_GEN_JRL_EXPORT Clock
  {
  public:

    /*! \brief Default constructor */
    Clock();

    /*! \brief Default destructor */
    ~Clock();

    /*! \brief Start Timing. */
    void StartTiming();

    /*! \brief End Timing. */
    void StopTiming();
    
    /*! \brief Increment Iteration by specifying the number of iterations*/
    void IncIteration(int lNbOfIts=1);

    /*! \brief Returns number of iteration. */
    unsigned long int NbOfIterations();

    /*! \brief Returns maximum time interval measured
      between two increment of iteration. */
    double MaxTime();

    /*! \brief Returns average time interval measured
      between two increment of iteration. */
    double AverageTime();

    /*! \brief Returns the total time measured. */
    double TotalTime();

    /*! \brief Reset the clock to restart a campaign
      of measures */
    void Reset();

    /*! \brief Display a brief description of the current status. */
    void Display();

    /*! \brief Record buffer of time consumption. */
    void RecordDataBuffer(std::string filename);

  private:
    
    /*! Storing begin and end timestamps. */
    struct timeval m_BeginTimeStamp, m_EndTimeStamp;

    /*! Starting time of the clock. */
    double m_StartingTime;

    /*! Number of iterations. */
    unsigned long int m_NbOfIterations;

    /*! Maximum time. */
    double m_MaximumTime;
    
    /*! Total time. */
    double m_TotalTime;
    
    /*! Buffer */
    std::vector<double> m_DataBuffer;

    
    
  };
};
#endif /* _HWPG_CLOCK_H_ */

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

   Please see License.txt for further information on license.
*/



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
  class  Clock
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

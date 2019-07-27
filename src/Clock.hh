/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Francois Keith
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

/*! \file Clock.h
  \brief Defines an object for measuring the time spend in some code part.
*/
#ifndef _HWPG_CLOCK_H_
# define _HWPG_CLOCK_H_
# include <time.h>
# include <vector>
# include <string>

# include "portability/gettimeofday.hh"

# ifdef WIN32
#  include <Windows.h>
# endif


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
}
#endif /* _HWPG_CLOCK_H_ */

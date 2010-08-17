/* \file Class to measure CPU computation time 
 * Olivier Stasse
 * (c) 2010
 * 
 */

#ifndef _CLOCK_CPUTIME_PATTERN_GERENATOR_UTESTING_H_
#define _CLOCK_CPUTIME_PATTERN_GERENATOR_UTESTING_H_

/* System includes */
#ifdef UNIX
#include <sys/time.h>
#include <stdlib.h>
#endif /*UNIX*/

#ifdef WIN32
#include <Windows.h>
#include "TimeUtilsWindows.h"
#endif /*WIN32*/

#include <time.h>
#include <ostream>
#include <vector>

#include "CommonTools.h"

namespace PatternGeneratorJRL {

  namespace TestSuite {
    
    /*! \brief Clock class for computation timing */
    class ClockCPUTime
    {
    public:
      ClockCPUTime();
      ~ClockCPUTime();

      /*! \name Measure starting date */
      void startingDate();

      /*! \name Methods related to off-line trajectories generation 
       @{ */
      void startPlanning();
      void endPlanning();
      /*! @} */

      /*! \name Methods related to on-line trajectories generation
	@{ */
      void startOneIteration();
      void stopOneIteration();
      /*! \brief Returns the date at which startOneIteration was called. */
      double getStartOneIteration();
      /*! @} */
      
      /*! \name Methods related to on-line modification
	@{ */
      void startModification();
      void stopModification();
      /*! @} */


      /*! \name Update data buffer regarding time statistics. */
      void fillInStatistics();
      
      /*! \brief Write buffers of timestamps and time consumtion
	in file aFileName */
      void writeBuffer(std::string &aFileName);

      /*! \brief Display mean time, max time, and nb of modifications. */
      void displayStatistics(std::ostream &os,
			     OneStep &aStep);
      
      /*! \brief Reset all counters */
      void Reset();

    private:

      /*! \brief Date at the measurement started, 
	i.e. the global reference */
      struct timeval m_startingtime;
      
      /*! \brief Date at which current computation started */
      struct timeval m_begin;
      
      /*! \brief The time spend in planning. */
      double m_totaltimeinplanning;

      /*! \brief Current total time spend in one iteration. */
      double m_currenttime;

      /*! \brief Total time spend in handling iterations. */
      double m_totaltime;

      /*! \brief The time spend in handling modification. */
      double m_modificationtime;

      /*! \brief Total time spend in handling modification during the test. */
      double m_totalmodificationtime;
      
      /*! \brief Number of modification done. */
      unsigned long int m_nbofmodifs;

      /*! \brief Maximum time spend in one iteration */
      double m_maxtime;

      /*! \brief Current position of the circular buffer. */
      unsigned long int m_TimeProfileIndex;

      /*! \brief Limit of the circular buffer. */
      unsigned long int m_TimeProfileUpperLimit;

      /*! \brief The circular buffer for time consumption. */
      std::vector<double> m_TimeProfile;

      /*! \brief The circular buffer for time stamps. */
      std::vector<double> m_TimeProfileTS;

      /*! \brief Nb of iteration really computed */
      unsigned long int m_NbOfItToCompute;
    };
  };
};
#endif /* _CLOCK_CPUTIME_PATTERN_GERENATOR_UTESTING_H_ */

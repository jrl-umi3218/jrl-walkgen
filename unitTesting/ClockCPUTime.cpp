/* Olivier Stasse
 * (c) 2010, JRL, CNRS-AIST
 * 
 */
#include <fstream>
#include <string>
#include "ClockCPUTime.h"

using namespace std;

namespace PatternGeneratorJRL
{
  namespace TestSuite
  {

    ClockCPUTime::ClockCPUTime()
    {
      Reset();
    }

    ClockCPUTime::~ClockCPUTime()
    {
    };

    void ClockCPUTime::Reset()
    {
      m_totaltimeinplanning = 0.0;
      m_currenttime = 0.0;
      m_TimeProfileIndex = 0;
      m_TimeProfile.resize(200*620);
      m_TimeProfileTS.resize(200*620);
      m_TimeProfileUpperLimit = 200 * 620;
      m_maxtime = 0.0;
      m_modificationtime = 0.0;
      m_totalmodificationtime = 0.0;
      m_NbOfItToCompute = 0;
      m_nbofmodifs = 0;
    };

    void ClockCPUTime::startingDate()
    {
      gettimeofday(&m_startingtime,0);
    }

    void ClockCPUTime::startPlanning()
    {
      gettimeofday(&m_begin,0);
    };

    void ClockCPUTime::endPlanning()
    {
      struct timeval end;
      
      gettimeofday(&end,0);
      double ltime = end.tv_sec-m_begin.tv_sec 
	+ 0.000001 * (end.tv_usec - m_begin.tv_usec);
      m_totaltimeinplanning+=ltime;

    };

    void ClockCPUTime::startOneIteration()
    {
      gettimeofday(&m_begin,0);
    }

    double ClockCPUTime::getStartOneIteration()
    {
      return m_begin.tv_sec + 0.000001 * m_begin.tv_usec ;
    }

    void ClockCPUTime::stopOneIteration()
    {
      struct timeval end;

      gettimeofday(&end,0);
      m_currenttime = end.tv_sec-m_begin.tv_sec 
	+ 0.000001 * (end.tv_usec - m_begin.tv_usec);
      if (m_maxtime<m_currenttime)
	m_maxtime = m_currenttime;
      
      if (m_currenttime>0.000300)
	{
	  m_totaltime += m_currenttime;
	  m_NbOfItToCompute++;
	}
    }

    void ClockCPUTime::startModification()
    {
      gettimeofday(&m_begin,0);
    }

    void ClockCPUTime::stopModification()
    {
      struct timeval end;

      gettimeofday(&end,0);
      m_modificationtime = end.tv_sec - m_begin.tv_sec 
	+ 0.000001 * (end.tv_usec - m_begin.tv_usec);

      m_totalmodificationtime += m_modificationtime;
    }
    
    void ClockCPUTime::fillInStatistics()
    {
      m_TimeProfile[m_TimeProfileIndex] = 
	m_currenttime + m_modificationtime;
      m_TimeProfileTS[m_TimeProfileIndex] = 
	m_begin.tv_sec + 0.000001 * m_begin.tv_usec;

      m_TimeProfileIndex++;
      if (m_TimeProfileIndex>m_TimeProfileUpperLimit)
	m_TimeProfileIndex = 0;

    }

    void ClockCPUTime::writeBuffer(string &aFileName)
    {
      ofstream lProfileOutput(aFileName.c_str(),ofstream::out);
      double dST = m_startingtime.tv_sec + 0.000001 * m_startingtime.tv_usec;
      for(unsigned int i=0;i<m_TimeProfileIndex;i++)
	lProfileOutput << " " << m_TimeProfileTS[i] - dST
		       << " " << m_TimeProfile[i] << std::endl;
      
      lProfileOutput.close();

    }

    void ClockCPUTime::displayStatistics(ostream &os,
					  struct OneStep &OneStep)
    {
        os << "Number of iterations " << OneStep.NbOfIt << " " << m_NbOfItToCompute << endl;
	os << "Time consumption: " << (double)m_totaltime/(double)m_NbOfItToCompute 
	     << " max time: " << m_maxtime << endl;
	os << "Time for modif: " << (double)m_totalmodificationtime/(double)m_nbofmodifs 
	     <<  " nb of modifs: " << m_nbofmodifs << endl ;
	os << "Time on ZMP ref planning " 
	   << m_totaltimeinplanning << " " << endl;
	if ((double)OneStep.NbOfIt!=0)
	  {
	    os << m_totaltimeinplanning*4/(double)OneStep.NbOfIt<< endl;
	  }

    }
    
  };
};

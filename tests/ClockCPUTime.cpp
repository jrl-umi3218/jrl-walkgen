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
#include <fstream>
#include <string>
#include "ClockCPUTime.hh"

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
    }

    void ClockCPUTime::Reset()
    {
      m_totaltimeinplanning = 0.0;
      m_currenttime = 0.0;
      m_totaltime = 0.0;
      m_TimeProfileIndex = 0;
      m_TimeProfile.resize(200*620);
      m_TimeProfileTS.resize(200*620);
      m_TimeProfileUpperLimit = 200 * 620;
      m_maxtime = 0.0;
      m_modificationtime = 0.0;
      m_totalmodificationtime = 0.0;
      m_NbOfItToCompute = 0;
      m_nbofmodifs = 0;
    }

    void ClockCPUTime::startingDate()
    {
      gettimeofday(&m_startingtime,0);
    }

    void ClockCPUTime::startPlanning()
    {
      gettimeofday(&m_begin,0);
    }

    void ClockCPUTime::endPlanning()
    {
      struct timeval end;
      
      gettimeofday(&end,0);
      double ltime = end.tv_sec-m_begin.tv_sec 
	+ 0.000001 * (end.tv_usec - m_begin.tv_usec);
      m_totaltimeinplanning+=ltime;

    }

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

      if (m_modificationtime> 0.0005)
	m_nbofmodifs++;

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
      // Shift all the measurement to the origin.
      double dST = m_startingtime.tv_sec + 0.000001 * m_startingtime.tv_usec;
      for(unsigned int i=0;i<m_TimeProfileIndex;i++)
	lProfileOutput << " " << m_TimeProfileTS[i] - dST
		       << " " << m_TimeProfile[i] << std::endl;
      
      lProfileOutput.close();

    }

    void ClockCPUTime::displayStatistics(ostream &os,
					 struct OneStep &OneStep)
    {
      
      os << " === " << endl;
      os << "Number of iterations " << OneStep.NbOfIt << endl
	 << "Number of iterations above 300 us:" << m_NbOfItToCompute << endl;
      if (m_NbOfItToCompute!=0)
	os << "Mean time consumption for one iteration above 300 us: " 
	   << (double)m_totaltime/(double)m_NbOfItToCompute 
	   << " (s) " << endl
	   << "Maximum time consumption for one iteration: " 
	   << m_maxtime << " (s) " << endl;
      else
	os << "No iteration above 300 us realized." << endl
	   << "The computation were done off-line or there is a problem." << endl;

      os << " === " << endl;
      if (m_nbofmodifs!=0)
	os << "Mean time for modifications: " 
	   << (double)m_totalmodificationtime/(double)m_nbofmodifs 
	   << " (s) " << endl
	   <<  "Number of modifications: " << m_nbofmodifs << endl ;
      else 
	os << "No modifications" << endl;
      
      os << "Time on ZMP reference planning " 
	 << m_totaltimeinplanning << " (s) " << endl;
      
    }
    
  }
}

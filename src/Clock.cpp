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
/*! \file Clock.cpp
  \brief This object allows to make time measurement on the code.

  Copyright (c) 2008,
  Olivier Stasse,

  JRL-Japan, CNRS/AIST

  All rights reserved.

  Please see license.txt for more information on license.
*/

#include <Clock.hh>
#include <fstream>
#include <iostream>

using namespace PatternGeneratorJRL;

Clock::Clock() {
  Reset();
  m_DataBuffer.resize(300000);

  struct timeval startingtime;
  gettimeofday(&startingtime, 0);
  m_StartingTime =
      (double)startingtime.tv_sec + 0.000001 * (double)startingtime.tv_usec;
}

Clock::~Clock() {}

void Clock::Reset() {
  m_NbOfIterations = 0;
  m_MaximumTime = 0.0;
  m_TotalTime = 0.0;

  struct timeval startingtime;
  gettimeofday(&startingtime, 0);
  m_StartingTime =
      (double)startingtime.tv_sec + 0.000001 * (double)startingtime.tv_usec;
}

void Clock::StartTiming() { gettimeofday(&m_BeginTimeStamp, 0); }

void Clock::StopTiming() {
  gettimeofday(&m_EndTimeStamp, 0);
  double ltime =
      (double)m_EndTimeStamp.tv_sec - (double)m_BeginTimeStamp.tv_sec +
      0.000001 * (double)(m_EndTimeStamp.tv_usec - m_BeginTimeStamp.tv_usec);

  m_MaximumTime = m_MaximumTime < ltime ? ltime : m_MaximumTime;
  m_TotalTime += ltime;

  m_DataBuffer[(m_NbOfIterations * 2) % 3000000] =
      (double)m_BeginTimeStamp.tv_sec +
      0.000001 * (double)m_BeginTimeStamp.tv_usec - m_StartingTime;
  m_DataBuffer[(m_NbOfIterations * 2 + 1) % 3000000] = ltime;
}

void Clock::IncIteration(int lNbOfIts) { m_NbOfIterations += lNbOfIts; }

void Clock::RecordDataBuffer(std::string filename) {
  std::ofstream aof(filename.c_str());
  for (unsigned int i = 0; i < 2 * m_NbOfIterations % 300000; i += 2)
    aof << m_DataBuffer[i] << " " << m_DataBuffer[i + 1] << std::endl;
  aof.close();
}
unsigned long int Clock::NbOfIterations() { return m_NbOfIterations; }

double Clock::MaxTime() { return m_MaximumTime; }

double Clock::TotalTime() { return m_TotalTime; }

double Clock::AverageTime() {
  if (m_NbOfIterations != 0)
    return m_TotalTime / (double)m_NbOfIterations;
  return 0.0;
}

void Clock::Display() {
  std::cout << "Average Time : " << AverageTime() << std::endl;
  std::cout << "Total Time : " << TotalTime() << std::endl;
  std::cout << "Max Time : " << MaxTime() << std::endl;
  std::cout << "Nb of iterations: " << NbOfIterations() << std::endl;
}

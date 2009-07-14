/*! \file Clock.cpp
  \brief This object allows to make time measurement on the code.

   Copyright (c) 2008, 
   Olivier Stasse,
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see license.txt for more information on license.
*/

#include <iostream>
#include <fstream>
#include <Clock.h>

using namespace PatternGeneratorJRL;

Clock::Clock()
{
  Reset();
  m_DataBuffer.resize(300000);

  struct timeval startingtime;
  gettimeofday(&startingtime,0);
  m_StartingTime = startingtime.tv_sec + 0.000001 * startingtime.tv_usec;
}


Clock::~Clock()
{
}

void Clock::Reset()
{
  m_NbOfIterations = 0;
  m_MaximumTime = 0.0;
  m_TotalTime=0.0;

  struct timeval startingtime;
  gettimeofday(&startingtime,0);
  m_StartingTime = startingtime.tv_sec + 0.000001 * startingtime.tv_usec;
}

void Clock::StartTiming()
{
  gettimeofday(&m_BeginTimeStamp,0);

}

void Clock::StopTiming()
{
  gettimeofday(&m_EndTimeStamp,0);
  double ltime = m_EndTimeStamp.tv_sec - m_BeginTimeStamp.tv_sec +
    0.000001 * (m_EndTimeStamp.tv_usec - m_BeginTimeStamp.tv_usec);

  m_MaximumTime = m_MaximumTime < ltime ? ltime : m_MaximumTime;
  m_TotalTime += ltime;

  m_DataBuffer[(m_NbOfIterations*2)%3000000]=m_BeginTimeStamp.tv_sec +
    0.000001 * m_BeginTimeStamp.tv_usec - m_StartingTime;
  m_DataBuffer[(m_NbOfIterations*2+1)%3000000]=ltime;
}

void Clock::IncIteration(int lNbOfIts)
{
  m_NbOfIterations += lNbOfIts;
}

void Clock::RecordDataBuffer(std::string filename)
{
  std::ofstream aof(filename.c_str());
  for(unsigned int i=0;i<2*m_NbOfIterations%300000;i+=2)
    aof << m_DataBuffer[i]<< " " << m_DataBuffer[i+1] << std::endl;
  aof.close();
}
unsigned long int Clock::NbOfIterations()
{
  return m_NbOfIterations;
}

double Clock::MaxTime()
{
  return m_MaximumTime;
}

double Clock::TotalTime()
{
  return m_TotalTime;
}

double Clock::AverageTime()
{
  if (m_NbOfIterations!=0)
    return m_TotalTime/(double)m_NbOfIterations;
  return 0.0;
}

void Clock::Display()
{
  std::cout << "Average Time : " << AverageTime() << std::endl;
  std::cout << "Total Time : " << TotalTime() << std::endl;
  std::cout << "Max Time : " << MaxTime() << std::endl;
  std::cout << "Nb of iterations: " << NbOfIterations() << std::endl;
  
}

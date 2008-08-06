/*! \file Clock.cpp
  \brief This object allows to make time measurement on the code.

   Copyright (c) 2008, 
   Olivier Stasse,
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
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

#include <iostream>

#include <walkGenJrl/Clock.h>

using namespace PatternGeneratorJRL;

Clock::Clock()
{
  Reset();
}


Clock::~Clock()
{
}

void Clock::Reset()
{
  m_NbOfIterations = 0;
  m_MaximumTime = 0.0;
  m_TotalTime=0.0;
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

}

void Clock::IncIteration(int lNbOfIts)
{
  m_NbOfIterations += lNbOfIts;
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

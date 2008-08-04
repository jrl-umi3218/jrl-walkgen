/* This object handles several intervals for the foot trajectory generation.

   Copyright (c) 2007, 
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

#define ODEBUG2(x)
#define ODEBUG3(x) std::cerr << "FootTrajectoryGenerationMultiple :" << x << std::endl
#define RESETDEBUG5(y) { std::ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { std::ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#if 0
#define ODEBUG(x) std::cerr << "FootTrajectoryGenerationMultiple :" <<  x << std::endl
#else
#define ODEBUG(x)
#endif

#if 0
#define RESETDEBUG4(y) { std::ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { std::ofstream DebugFile; DebugFile.open(y,ofstream::app); \
    DebugFile << "PGI: " << x << std::endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1
#else
#define RESETDEBUG4(y)
#define ODEBUG4(x,y)
#endif

#define ODEBUG6(x,y)

#include <walkGenJrl/FootTrajectoryGeneration/FootTrajectoryGenerationMultiple.h>

using namespace PatternGeneratorJRL;

FootTrajectoryGenerationMultiple::FootTrajectoryGenerationMultiple(SimplePluginManager *lSPM,
								   dynamicsJRLJapan::HumanoidSpecificities *aHS)
  : SimplePlugin(lSPM)
{
  m_HS = aHS;
}

FootTrajectoryGenerationMultiple::~FootTrajectoryGenerationMultiple()
{
}

void FootTrajectoryGenerationMultiple::SetNumberOfIntervals(int lNumberOfIntervals)
{
  if (m_SetOfFootTrajectoryGenerationObjects.size()==lNumberOfIntervals)
    return;
  
  for(unsigned int i=0;i<m_SetOfFootTrajectoryGenerationObjects.size();i++)
    {
      delete m_SetOfFootTrajectoryGenerationObjects[i];
    }
  
  m_SetOfFootTrajectoryGenerationObjects.resize(lNumberOfIntervals);
  for(unsigned int i=0;i<m_SetOfFootTrajectoryGenerationObjects.size();i++)
    {
      m_SetOfFootTrajectoryGenerationObjects[i] = new FootTrajectoryGenerationStandard(getSimplePluginManager(),m_HS);
      m_SetOfFootTrajectoryGenerationObjects[i]->InitializeInternalDataStructures();
    }  
}
  
int FootTrajectoryGenerationMultiple::GetNumberOfIntervals()
{
  return m_SetOfFootTrajectoryGenerationObjects.size();
}


void FootTrajectoryGenerationMultiple::SetTimeIntervals(vector<double> &lDeltaTj)
{
  m_DeltaTj = lDeltaTj;
  m_RefTime.resize(lDeltaTj.size());
  double reftime=0.0;
  for(unsigned int li=0;li<m_DeltaTj.size();li++)
    {
      m_RefTime[li] = reftime;
      reftime+=m_DeltaTj[li];
    }
  
}

void FootTrajectoryGenerationMultiple::GetTimeIntervals(vector<double> &lDeltaTj)
{
  lDeltaTj = m_DeltaTj;
}

bool FootTrajectoryGenerationMultiple::Compute(int axis, double t, double &result)
{
  t -= m_AbsoluteTimeReference;
  result = -1.0;
  double reftime=0;
  ODEBUG(" ====== CoM ====== ");
  for(unsigned int j=0;j<m_DeltaTj.size();j++)
    {
      ODEBUG("t: " << t << " reftime :" << reftime << " Tj["<<j << "]= "<< m_DeltaTj[j]);
      if ((t>=reftime) && (t<=reftime+m_DeltaTj[j]))
	{
	  double deltaj=0.0;
	  deltaj = t-reftime;
	  
	  if (m_SetOfFootTrajectoryGenerationObjects[j]!=0)
	    {
	      result = m_SetOfFootTrajectoryGenerationObjects[j]->Compute(axis,deltaj);
	    }
	  return true;
	}
      
      reftime+=m_DeltaTj[j];
    }
  return false;
}


bool FootTrajectoryGenerationMultiple::Compute(double t, FootAbsolutePosition & aFootAbsolutePosition, unsigned int IndexInterval)
{
  double deltaj = t - m_AbsoluteTimeReference - m_RefTime[IndexInterval];
  ODEBUG("IndexInterval : " << IndexInterval );
  m_SetOfFootTrajectoryGenerationObjects[IndexInterval]->ComputeAll(aFootAbsolutePosition,deltaj);
}

bool FootTrajectoryGenerationMultiple::Compute(double t, FootAbsolutePosition & aFootAbsolutePosition)
{
  t -= m_AbsoluteTimeReference;
  double reftime=0;
  ODEBUG(" ====== Foot ====== " << m_DeltaTj.size());
  for(unsigned int j=0;j<m_DeltaTj.size();j++)
    {

      if ((t>=reftime) && (t<=reftime+m_DeltaTj[j]))
	{
	  ODEBUG("t: " << t << " reftime :" << reftime << " Tj["<<j << "]= "<< m_DeltaTj[j]);
	  double deltaj=0.0;
	  deltaj = t-reftime;
	  
	  if (m_SetOfFootTrajectoryGenerationObjects[j]!=0)
	    {	      
	      m_SetOfFootTrajectoryGenerationObjects[j]->ComputeAll(aFootAbsolutePosition,deltaj);
	    }
	  ODEBUG("X: " << aFootAbsolutePosition.x << 
		  " Y: " << aFootAbsolutePosition.y << 
		  " Z: " << aFootAbsolutePosition.z << 
		  " Theta: " << aFootAbsolutePosition.theta <<
		  " Omega: " << aFootAbsolutePosition.omega);
	  return true;
	}
      
      reftime+=m_DeltaTj[j];
    }
  return false;
}

/*! This method specifies the parameters for each of the polynome used by this
  object. In this case, as it is used for the 3rd order polynome. The polynome to
  which those parameters are set is specified with PolynomeIndex. 
  @param PolynomeIndex: Set to which axis the parameters will be applied. 
  @param TimeInterval: Set the time base of the polynome.
  @param Position: Set the final position of the polynome at TimeInterval.
  @param InitPosition: Initial position when computing the polynome at t=0.0.
  @param InitSpeed: Initial speed when computing the polynome at t=0.0.
*/
int FootTrajectoryGenerationMultiple::SetParametersWithInitPosInitSpeed(unsigned int IntervalIndex,
									int AxisReference,
									double TimeInterval,
									double FinalPosition,
									double InitPosition,
									double InitSpeed)
{
  if ((IntervalIndex<0) || (IntervalIndex>=m_SetOfFootTrajectoryGenerationObjects.size()))
    return -1;


  m_SetOfFootTrajectoryGenerationObjects[IntervalIndex]->SetParametersWithInitPosInitSpeed(AxisReference,
											  TimeInterval,
											  FinalPosition,
											  InitPosition,
											  InitSpeed);
  return 0;
}


double FootTrajectoryGenerationMultiple::GetAbsoluteTimeReference()
{
  return m_AbsoluteTimeReference;
}

void FootTrajectoryGenerationMultiple::SetAbsoluteTimeReference(double lAbsoluteTimeReference)
{
  m_AbsoluteTimeReference = lAbsoluteTimeReference;
}

void FootTrajectoryGenerationMultiple::CallMethod(std::string &Method,
						  std::istringstream &strm)
{
  
}

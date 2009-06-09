/* This object handles several intervals for the foot trajectory generation.

   Copyright (c) 2007, 
   Olivier Stasse,
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   See License.txt for more details on license.txt
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
								   CjrlHumanoidDynamicRobot *aHS)
  : SimplePlugin(lSPM)
{
  m_HS = aHS;
}

FootTrajectoryGenerationMultiple::~FootTrajectoryGenerationMultiple()
{
  for(unsigned int i=0;i<m_SetOfFootTrajectoryGenerationObjects.size();i++)
    {
      delete m_SetOfFootTrajectoryGenerationObjects[i];
    }
}

void FootTrajectoryGenerationMultiple::SetNumberOfIntervals(int lNumberOfIntervals)
{
  if (m_SetOfFootTrajectoryGenerationObjects.size()==(unsigned int)lNumberOfIntervals)
    return;
  
  for(unsigned int i=0;i<m_SetOfFootTrajectoryGenerationObjects.size();i++)
    {
      delete m_SetOfFootTrajectoryGenerationObjects[i];
    }
  
  m_SetOfFootTrajectoryGenerationObjects.resize(lNumberOfIntervals);
  for(unsigned int i=0;i<m_SetOfFootTrajectoryGenerationObjects.size();i++)
    {
      m_SetOfFootTrajectoryGenerationObjects[i] = 
	new FootTrajectoryGenerationStandard(getSimplePluginManager(),m_HS);
      m_SetOfFootTrajectoryGenerationObjects[i]->InitializeInternalDataStructures();
    }  
  m_NatureOfIntervals.resize(lNumberOfIntervals);
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
  aFootAbsolutePosition.stepType = m_NatureOfIntervals[IndexInterval];
  return true;
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
	      aFootAbsolutePosition.stepType = m_NatureOfIntervals[j];
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

/*! This method specifies the nature of the interval. 
*/
int FootTrajectoryGenerationMultiple::SetNatureInterval(unsigned int IntervalIndex,
							int Nature)
{
  if ((IntervalIndex<0) || (IntervalIndex>=m_NatureOfIntervals.size()))
    return -1;
  m_NatureOfIntervals[IntervalIndex] = Nature;
  return 0;
  
}

/*! This method returns the nature of the interval. 
*/
int FootTrajectoryGenerationMultiple::GetNatureInterval(unsigned int IntervalIndex)
{
  if ((IntervalIndex<0) || (IntervalIndex>=m_NatureOfIntervals.size()))
    return -100;
  
  return  m_NatureOfIntervals[IntervalIndex];
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

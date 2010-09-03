/* This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of steps following a QP 
   formulation and a new QP solver as proposed by Herdt Advanced Robotics 2010. 
 
   Copyright (c) 2010, 
   Andrei Herdt,
   Olivier Stasse, 
   Mehdi Benallegue

   JRL-Japan, CNRS/AIST 
 
   All rights reserved. 
 
   See License.txt for more information on license. 

*/ 

 
#ifdef UNIX 
#include <sys/time.h> 
#endif /* UNIX */ 
 
#ifdef WIN32 
#include <Windows.h> 
#include <TimeUtilsWindows.h> 
#endif 
 
#include <time.h> 
 
#include <iostream> 
#include <fstream> 
 
#include <Mathematics/qld.h> 
#include <ZMPRefTrajectoryGeneration/OnlineStepPositionTrajectoryGeneration.h> 
 
#include <Debug.h> 
using namespace std; 
using namespace PatternGeneratorJRL; 
 
OnlineStepPositionTrajectoryGeneration::OnlineStepPositionTrajectoryGeneration(SimplePluginManager *lSPM,
						 string DataFile, 
						 CjrlHumanoidDynamicRobot *aHS) : 
  ZMPVelocityReferencedQP(lSPM,DataFile,aHS) 
{ 
  velocityMode_=true;
  RelativeFootPosition p;
  p.sx=
	  p.sy=
	  p.theta=0;

  stepPos_.push_back(p);
  m_BetaCache_=0.0;
 
} 
 
OnlineStepPositionTrajectoryGeneration::~OnlineStepPositionTrajectoryGeneration() 
{ 
 
 
} 

void OnlineStepPositionTrajectoryGeneration::SetStepsPositions(const std::deque<RelativeFootPosition>& s)
{
	if (s.size()>0)
		stepPos_=s;
	else
		throw std::runtime_error("Empty queue as an input");

}

const std::deque<RelativeFootPosition> &OnlineStepPositionTrajectoryGeneration::GetStepsPositions() const
{
	return stepPos_;
}

void OnlineStepPositionTrajectoryGeneration::SetVelocityMode(bool b)
{
	if (velocityMode_!=b)
	{
		velocityMode_=b;
		double tmp=m_Beta;
		m_Beta=m_BetaCache_;
		m_BetaCache_=tmp;
	}
}

void OnlineStepPositionTrajectoryGeneration::SetBeta(const double & b)
{
	if (velocityMode_)
		ZMPVelocityReferencedQP::SetBeta(b);
}
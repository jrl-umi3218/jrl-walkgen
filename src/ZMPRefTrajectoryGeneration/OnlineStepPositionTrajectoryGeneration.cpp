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


#include "portability/gettimeofday.hh"

#ifdef WIN32
# include <Windows.h>
#endif /* WIN32 */

#include <time.h> 

#include <iostream> 
#include <fstream> 

#include <Mathematics/qld.h> 
#include <ZMPRefTrajectoryGeneration/OnlineStepPositionTrajectoryGeneration.h> 




#include <Debug.h> 
using namespace std; 
using namespace PatternGeneratorJRL; 

OnlineStepPositionTrajectoryGeneration::OnlineStepPositionTrajectoryGeneration(SimplePluginManager* lSPM,
																			   string DataFile, 
																			   CjrlHumanoidDynamicRobot* aHS) : 
ZMPVelocityReferencedQP(lSPM,DataFile,aHS)/*m_fCALS_FP(lSPM,aHS,m_ConstraintOnX,m_ConstraintOnY),*/
{ 
	velocityMode_=true;
	betaCache_=0;
	yawCache_=0;
	
	
	
	
	//Pointer exchange
	if (VRQPGenerator_!=0x0)
	{
		VRQPGeneratorCopy_=new GeneratorStepPos(*VRQPGenerator_);
		delete VRQPGenerator_;
		VRQPGenerator_=VRQPGeneratorCopy_;
	}
	else
	{
		std::string s="Error creating OnlineStepPositionTrajectoryGeneration VRQPGenerator_ is null";
		throw std::runtime_error(s);
	}

	
	SetVelocityMode(false);

	

	// Register method to handle 
	string aMethodName[] = 
	{":setvelocitymode"}; 

	for(int i=0;i<1;i++) 
	{ 
		if (!RegisterMethod(aMethodName[i])) 
		{ 
			std::cerr << "Unable to register " << aMethodName << std::endl; 
		} 
	} 

} 

OnlineStepPositionTrajectoryGeneration::~OnlineStepPositionTrajectoryGeneration() 
{ 


} 

void OnlineStepPositionTrajectoryGeneration::SetStepsPositions(const RelativeStepPositionQueue& s)
{
	VRQPGeneratorCopy_->SetStepsPositions(s);

}

const RelativeStepPositionQueue &OnlineStepPositionTrajectoryGeneration::GetStepsPositions() const
{
	return VRQPGeneratorCopy_->GetStepsPositions();
}

void OnlineStepPositionTrajectoryGeneration::SetVelocityMode(bool b)
{
	if (velocityMode_!=b)
	{
		velocityMode_=b;
		if (b==false)
		{
			betaCache_=0;
			yawCache_=0;
		}	
		double beta=VRQPGenerator_->getPonderation( IntermedQPMat::INSTANT_VELOCITY);
		double dx,dy,dyaw;
		getVelReference(dx,dy,dyaw);
		VRQPGeneratorCopy_->explicitPonderation(betaCache_, IntermedQPMat::INSTANT_VELOCITY);
		setVelReference(dx,dy,yawCache_);
		betaCache_=beta;
		yawCache_=dyaw;
		VRQPGeneratorCopy_->setVelocityMode(b);
	}
	
}



void OnlineStepPositionTrajectoryGeneration::CallMethod(std::string & Method, std::istringstream &strm)
{
	if (Method==":setvelocitymode") 
	{
		int b;
		strm>> b; 
		SetVelocityMode(b!=0);
	} 
	
	else
		ZMPVelocityReferencedQP::CallMethod(Method,strm); 
}



void OnlineStepPositionTrajectoryGeneration::OnLine(double time, 
													deque<ZMPPosition> & FinalZMPTraj_deq, 
													deque<COMState> & FinalCOMTraj_deq, 
													deque<FootAbsolutePosition> &FinalLeftFootTraj_deq, 
													deque<FootAbsolutePosition> &FinalRightFootTraj_deq) 
{

	//if (velocityMode_)
	{

		//Calling the Parent overload of Online
		ZMPVelocityReferencedQP::OnLine(time,FinalZMPTraj_deq,FinalCOMTraj_deq,FinalLeftFootTraj_deq, FinalRightFootTraj_deq);
		return;
	}
	
} 





int OnlineStepPositionTrajectoryGeneration::OnLineFootChange(double time,
															 FootAbsolutePosition &aFootAbsolutePosition,
															 std::deque<ZMPPosition> & FinalZMPPositions,			     
															 std::deque<COMState> & COMStates,
															 std::deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
															 std::deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
															 StepStackHandler * aStepStackHandler)
{
	RelativeStepPosition r;
	r.x=aFootAbsolutePosition.x;
	r.y=aFootAbsolutePosition.y;
	r.theta=aFootAbsolutePosition.theta;

	return ChangeStepPosition(r,(unsigned)aFootAbsolutePosition.stepType);


}

int OnlineStepPositionTrajectoryGeneration::ChangeStepPosition(const RelativeStepPosition & r,
															   unsigned stepNumber)
{

	return VRQPGeneratorCopy_->ChangeStepPosition(r,stepNumber);

}

void OnlineStepPositionTrajectoryGeneration::OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
														   std::deque<ZMPPosition> & FinalZMPPositions,					     
														   std::deque<COMState> & COMStates,
														   std::deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
														   std::deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
														   bool EndSequence)
{
	RelativeStepPosition NewRelativeStepPosition;

	NewRelativeStepPosition.x=NewRelativeFootPosition.sx;
	NewRelativeStepPosition.y=NewRelativeFootPosition.sy;
	NewRelativeStepPosition.theta=NewRelativeFootPosition.theta;

	SetVelocityMode(false);
	AddStepPosition(NewRelativeStepPosition);



}

void OnlineStepPositionTrajectoryGeneration::setVelReference(double dx, double dy, double dyaw)
{
	if( fabs(dx)<SupportFSM::eps && fabs(dy)<SupportFSM::eps && fabs(dyaw)<SupportFSM::eps )
	{ 
		SetVelocityMode(true);
	}

	if (velocityMode_)
	{
		ZMPVelocityReferencedQP::setVelReference(dx,dy,dyaw);
	}
	else
	{
		yawCache_=dyaw;
		ZMPVelocityReferencedQP::setVelReference(dx,dy,0);
	}


}


int OnlineStepPositionTrajectoryGeneration::AddStepPosition(const RelativeStepPosition & r)
{
	return VRQPGeneratorCopy_->AddStepPosition(r);

}

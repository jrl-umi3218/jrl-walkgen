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
	
	
	VRQPGeneratorCopy_=new GeneratorStepPos(*VRQPGenerator_);

	
	//Pointer exchange
	if (VRQPGenerator_!=0x0)
	{
		delete VRQPGenerator_;
		VRQPGenerator_=VRQPGeneratorCopy_;
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
	
	//else//  StepPosMode
	{
		// If on-line mode not activated we go out.
		if (!m_OnLineMode)
		{ return; }

		// Testing if we are reaching the end of the online mode.
		if ((m_EndingPhase) &&
			(time>=m_TimeToStopOnLineMode))
		{ m_OnLineMode = false; }


		// Apply external forces if occured
		if(PerturbationOccured_ == true)
		{
			com_t com = CoM_();
			com.x(2) = com.x(2)+PerturbationAcceleration_(2);
			com.y(2) = com.y(2)+PerturbationAcceleration_(5);
			PerturbationOccured_ = false;
			CoM_(com);
		}

		// UPDATE WALKING TRAJECTORIES:
		// --------------------
		if(time + 0.00001 > m_UpperTimeLimitToUpdate)
		{
			double TotalAmountOfCPUTime=0.0,CurrentCPUTime=0.0;
			struct timeval start,end;
			gettimeofday(&start,0);


			// UPDATE INTERNAL DATA:
			// ---------------------
			VRQPGenerator_->Reference(VelRef_);
			VRQPGenerator_->setCurrentTime(time+TimeBuffer_);
			VRQPGenerator_->CoM(CoM_());


			// PREVIEW SUPPORT STATES FOR THE WHOLE PREVIEW WINDOW:
			// ----------------------------------------------------
			deque<support_state_t> PrwSupportStates_deq;
			VRQPGenerator_->preview_support_states(SupportFSM_, PrwSupportStates_deq);


			// DETERMINE CURRENT SUPPORT POSITION:
			// -----------------------------------
			support_state_t CurrentSupport = PrwSupportStates_deq.front();
			//Add a new support foot to the support feet history deque
			if(CurrentSupport.StateChanged == true)
			{
				FootAbsolutePosition FAP;
				if(CurrentSupport.Foot==1)
					FAP = FinalLeftFootTraj_deq.back();
				else
					FAP = FinalRightFootTraj_deq.back();
				CurrentSupport.x = FAP.x;
				CurrentSupport.y = FAP.y;
				CurrentSupport.yaw = FAP.theta*M_PI/180.0;
				CurrentSupport.StartTime = m_CurrentTime;
				VRQPGenerator_->SupportState( CurrentSupport );
			}


			// COMPUTE ORIENTATIONS OF FEET FOR WHOLE PREVIEW PERIOD:
			// ------------------------------------------------------
			deque<double> PreviewedSupportAngles_deq;
			OrientPrw_->preview_orientations(time+TimeBuffer_,
				VelRef_,
				SupportFSM_->StepPeriod(), CurrentSupport,
				FinalLeftFootTraj_deq, FinalRightFootTraj_deq,
				PreviewedSupportAngles_deq);


			// COMPUTE REFERENCE IN THE GLOBAL FRAME:
			// --------------------------------------
			VRQPGenerator_->compute_global_reference( FinalCOMTraj_deq );


			// BUILD CONSTANT PART OF THE OBJECTIVE:
			// -------------------------------------
			VRQPGenerator_->build_invariant_part( Problem_ );


			// BUILD VARIANT PART OF THE OBJECTIVE:
			// ------------------------------------
			VRQPGenerator_->update_problem( Problem_, PrwSupportStates_deq );


			// BUILD CONSTRAINTS:
			// ------------------
			VRQPGenerator_->build_constraints( Problem_,
				RFC_,
				FinalLeftFootTraj_deq,
				FinalRightFootTraj_deq,
				PrwSupportStates_deq,
				PreviewedSupportAngles_deq );


			// SOLVE PROBLEM:
			// --------------
			QPProblem_s::solution_t Result;
			Problem_.solve( QPProblem_s::QLD , Result );


			// INTERPOLATE THE NEXT COMPUTED COM STATE:
			// ----------------------------------------
			FinalCOMTraj_deq.resize((int)((QP_T_+TimeBuffer_)/m_SamplingPeriod));
			FinalZMPTraj_deq.resize((int)((QP_T_+TimeBuffer_)/m_SamplingPeriod));
			FinalLeftFootTraj_deq.resize((int)((QP_T_+TimeBuffer_)/m_SamplingPeriod));
			FinalRightFootTraj_deq.resize((int)((QP_T_+TimeBuffer_)/m_SamplingPeriod));
			int CurrentIndex = (int)(TimeBuffer_/m_SamplingPeriod)-1;
			CoM_.Interpolation(FinalCOMTraj_deq,
				FinalZMPTraj_deq,
				CurrentIndex,
				Result.Solution_vec[0],Result.Solution_vec[QP_N_]);
			CoM_.OneIteration(Result.Solution_vec[0],Result.Solution_vec[QP_N_]);


			// COMPUTE ORIENTATION OF TRUNK:
			// -----------------------------
			OrientPrw_->interpolate_trunk_orientation(time+TimeBuffer_, CurrentIndex,
				m_SamplingPeriod,
				CurrentSupport,
				FinalCOMTraj_deq);


			// INTERPOLATE THE COMPUTED FEET POSITIONS:
			// ----------------------------------------
			unsigned NumberStepsPrwd = PrwSupportStates_deq.back().StepNumber;
			OFTG_->interpolate_feet_positions(time+TimeBuffer_,
				CurrentIndex, CurrentSupport,
				Result.Solution_vec[2*QP_N_], Result.Solution_vec[2*QP_N_+NumberStepsPrwd],
				PreviewedSupportAngles_deq,
				FinalLeftFootTraj_deq, FinalRightFootTraj_deq);



			if(CurrentSupport.StepsLeft == 0)
				m_EndingPhase = true;
			// Specify that we are in the ending phase.
			if (m_EndingPhase==false)
			{
				// This should be done only during the transition EndingPhase=false -> EndingPhase=true
				m_TimeToStopOnLineMode = m_UpperTimeLimitToUpdate+QP_T_ * QP_N_;
				// Set the ZMP reference as very important.
				// It suppose to work because Gamma appears only during the non-constant
			}


			m_UpperTimeLimitToUpdate = m_UpperTimeLimitToUpdate+QP_T_;

			// Compute CPU consumption time.
			gettimeofday(&end,0);
			CurrentCPUTime = end.tv_sec - start.tv_sec +
				0.000001 * (end.tv_usec - start.tv_usec);
			TotalAmountOfCPUTime += CurrentCPUTime;
		}

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

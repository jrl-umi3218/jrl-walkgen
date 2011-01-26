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

OnlineStepPositionTrajectoryGeneration::OnlineStepPositionTrajectoryGeneration(SimplePluginManager *lSPM,
																			   string DataFile, 
																			   CjrlHumanoidDynamicRobot *aHS) : 
ZMPVelocityReferencedQP(lSPM,DataFile,aHS),/*m_fCALS_FP(lSPM,aHS,m_ConstraintOnX,m_ConstraintOnY),*/
removeQueueHead(false)
{ 
	velocityMode_=true;
	betaCache_=0;
	yawCache_=0;
	RelativeFootPosition p;
	p.sx=0;
	p.sy=0;
	p.theta=0;

	stepPos_.push_back(p);


	SetVelocityMode(false);

	// Register method to handle 
	string aMethodName[2] = 
	{":setstepspositions",
	":setvelocitymode"}; 

	for(int i=0;i<2;i++) 
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
		if (b==false)
		{
			betaCache_=0;
			yawCache_=0;
		}	
		double beta=VRQPGenerator_->getPonderation( IntermedQPMat::INSTANT_VELOCITY);
		double dx,dy,dyaw;
		getVelReference(dx,dy,dyaw);
		VRQPGenerator_->Ponderation(betaCache_, IntermedQPMat::INSTANT_VELOCITY);
		setVelReference(dx,dy,yawCache_);
		betaCache_=beta;
		yawCache_=dyaw;
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
	else if (Method==":setstepspositions") 
	{ 
		RelativeFootPosition p;
		strm>>p.sx;
		strm>>p.sy;
		strm>>p.theta;

		if (!strm.eof())
		{
			stepPos_.clear();

			do{
				stepPos_.push_back(p);
				strm>>p.sx;
				strm>>p.sy;
				strm>>p.theta;
			}while (!strm.eof());
		}


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

	if (velocityMode_)
	{

		//Calling the Parent overload of Online
		ZMPVelocityReferencedQP::OnLine(time,FinalZMPTraj_deq,FinalCOMTraj_deq,FinalLeftFootTraj_deq, FinalRightFootTraj_deq);



		return;
	}
	//else = StepPos Mode
	else
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
	RelativeFootPosition r;
	r.sx=aFootAbsolutePosition.x;
	r.sy=aFootAbsolutePosition.y;
	r.theta=aFootAbsolutePosition.theta;

	return ChangeStepPosition(r,(unsigned)aFootAbsolutePosition.stepType);


}

int OnlineStepPositionTrajectoryGeneration::ChangeStepPosition(const RelativeFootPosition & r,
															   unsigned stepNumber)
{

	if (stepNumber>=stepPos_.size())
	{	
		stepPos_.push_back(r);
		return stepPos_.size();
	}
	else
	{
		stepPos_[stepNumber]=r;
		return stepNumber;
	}


}

void OnlineStepPositionTrajectoryGeneration::OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
														   std::deque<ZMPPosition> & FinalZMPPositions,					     
														   std::deque<COMState> & COMStates,
														   std::deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
														   std::deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
														   bool EndSequence)
{

	AddStepPosition(NewRelativeFootPosition);



}

void OnlineStepPositionTrajectoryGeneration::setVelReference(double dx, double dy, double dyaw)
{
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


int OnlineStepPositionTrajectoryGeneration::AddStepPosition(const RelativeFootPosition & r)
{
	stepPos_.push_back(r);
	return stepPos_.size();

}

//Commented--------------------------------------------------------
//void OnlineStepPositionTrajectoryGeneration::OnLine(double time, 
//													deque<ZMPPosition> & FinalZMPPositions, 
//													deque<COMState> & FinalCOMStates, 
//													deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions, 
//													deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions) 
//{
//
//	if (velocityMode_)
//	{
//		//Calling the Parent overload of Online
//		ZMPVelocityReferencedQP::OnLine(time,FinalZMPPositions,FinalCOMStates,FinalLeftFootAbsolutePositions, FinalRightFootAbsolutePositions);
//		return;
//	}
//
//	//else = FootPos Mode
//	if(time + 0.00001 > m_UpperTimeLimitToUpdate) 
//	{ 
//		int NbOfConstraints=0; // Nb of constraints are not known in advance 
//
//		MAL_VECTOR_DIM(xk,double,6); 
//
//		int CriteriaToMaximize=1; 
//
//
//		deque<LinearConstraintInequality_t> QueueOfLConstraintInequalities; 
//		deque<LinearConstraintInequalityFreeFeet_t> QueueOfLConstraintInequalitiesFreeFeet; 
//		deque<LinearConstraintInequalityFreeFeet_t> QueueOfFeetPosInequalities; 
//
//		// pre compute the matrices needed for the optimization. 
//		double TotalAmountOfCPUTime=0.0,CurrentCPUTime=0.0; 
//		struct timeval start,end; 
//		//int li=0; 
//		//      double dinterval = m_QP_T /  m_SamplingPeriod; 
//		//int interval=(int)dinterval; 
//		bool StartingSequence = true; 
//
//		//int NumberOfRemovedConstraints =0; 
//
//		//----------"Real-time" loop--------- 
//		// 
//		// 
//		//----------------------------------- 
//		// printf("Inside the 'Real-time' loop: \n"); 
//
//		// printf("StartingTime: %f \n", StartingTime); 
//		gettimeofday(&start,0); 
//
//		m_OP->verifyAccelerationOfHipJoint(RefVel, m_TrunkState, 
//			m_TrunkStateT, m_Support); 
//
//		m_OP->previewOrientations(time+m_TimeBuffer, 
//			m_PreviewedSupportAngles, 
//			m_TrunkState, 
//			m_TrunkStateT, 
//			m_Support, 
//			FinalLeftFootAbsolutePositions, 
//			FinalRightFootAbsolutePositions); 
//
//		// Read the current state of the 2D Linearized Inverted Pendulum. 
//		m_2DLIPM->GetState(xk);
//
//		if(m_PerturbationOccured == true)
//		{
//			xk(2) = xk(2)+m_PerturbationAcceleration(2);
//			xk(5) = xk(5)+m_PerturbationAcceleration(5);
//			m_PerturbationOccured = false;
//		}
//
//		m_2DLIPM->setState(xk);
//
//		//TODO : Add a get function to read the state 
//		m_Support->setSupportState(time+m_TimeBuffer, 0, RefVel); 
//
//		//TODO : Temporary solution for the pldp solver. See above
//		bool CurrentStateChanged = m_Support->m_StateChanged;
//
//		//Add a new support foot to the support feet history deque
//		if(m_Support->m_StateChanged == true) 
//		{ 
//
//			deque<FootAbsolutePosition>::iterator FAP_it; 
//			SupportFeet_t newSF; 
//			if(m_Support->CurrentSupportFoot==1) 
//			{ 
//				FAP_it = FinalLeftFootAbsolutePositions.end(); 
//				FAP_it--; 
//			} 
//			else 
//			{ 
//				FAP_it = FinalRightFootAbsolutePositions.end(); 
//				FAP_it--; 
//			} 
//
//
//			newSF.x = FAP_it->x; 
//			newSF.y = FAP_it->y; 
//			newSF.theta = FAP_it->theta*M_PI/180.0; 
//			newSF.StartTime = time+m_TimeBuffer; 
//			newSF.SupportFoot = m_Support->CurrentSupportFoot; 
//
//			QueueOfSupportFeet.push_back(newSF); 
//		} 
//
//
//		m_fCALS_FP.buildLinearConstraintInequalities(FinalLeftFootAbsolutePositions, 
//			FinalRightFootAbsolutePositions, 
//			QueueOfLConstraintInequalitiesFreeFeet, 
//			QueueOfFeetPosInequalities, 
//			RefVel,
//			stepPos_,
//			removeQueueHead,
//			time+m_TimeBuffer, 
//			m_QP_N, 
//			m_Support, m_PreviewedSupportAngles,
//			NbOfConstraints); 
//
//
//		initializeProblem();
//
//		computeObjective(QueueOfLConstraintInequalitiesFreeFeet, QueueOfSupportFeet,  
//			NbOfConstraints, 0, CriteriaToMaximize, xk, time); 
//
//		if(m_FastFormulationMode == PLDPHerdt)
//		{
//			computeCholeskyOfQ(m_Pb.Q);
//			buildConstraintMatricesPLDPHerdt();
//		}
//
//
//		buildConstraintMatrices(m_Pb.DS,m_Pb.DU, 
//			m_QP_T, 
//			time+m_TimeBuffer, 
//			QueueOfLConstraintInequalitiesFreeFeet, 
//			QueueOfFeetPosInequalities, 
//			QueueOfSupportFeet, 
//			m_ComHeight,
//			NbOfConstraints,
//			xk); 
//
//		if(m_FullDebug>2) 
//			dumpProblem(m_Pb.Q, m_Pb.D, m_Pb.DU, m_Pb.m, m_Pb.DS, m_Pb.XL, m_Pb.XU, xk, time+m_TimeBuffer); 
//
//		double ldt = 0.0; 
//		//---------Solver------------ 
//		// 
//		// 
//		//--------------------------- 
//		// printf("Entering the solver \n"); 
//		if ((m_FastFormulationMode==QLDANDLQ)|| 
//			(m_FastFormulationMode==QLD)) 
//		{ 
//			struct timeval lbegin,lend; 
//			gettimeofday(&lbegin,0); 
//			ql0001_(&m_Pb.m, &m_Pb.me, &m_Pb.mmax, &m_Pb.n, &m_Pb.nmax, &m_Pb.mnn, 
//				m_Pb.Q, m_Pb.D, m_Pb.DU, m_Pb.DS, m_Pb.XL, m_Pb.XU, 
//				m_Pb.X, m_Pb.U, &m_Pb.iout, &m_Pb.ifail, &m_Pb.iprint, 
//				m_Pb.war, &m_Pb.lwar, m_Pb.iwar, &m_Pb.liwar, &m_Pb.Eps); 
//			gettimeofday(&lend,0); 
//
//			ldt = lend.tv_sec - lbegin.tv_sec + 
//				0.000001 * (lend.tv_usec - lbegin.tv_usec); 
//
//			int NbOfActivatedConstraints = 0; 
//			for(int lk=0;lk<m_Pb.m;lk++) 
//			{ 
//				if (m_Pb.U[lk]>0.0) 
//				{ 
//					NbOfActivatedConstraints++; 
//				} 
//			} 
//			ODEBUG6(NbOfActivatedConstraints,"InfosQLD.dat"); 
//			ODEBUG6(ldt,"dtQLD.dat"); 
//		} 
//		else if (m_FastFormulationMode==PLDPHerdt) 
//		{ 
//			ODEBUG("State: " << xk[0] << " " << xk[3] << " " << 
//				xk[1] << " " << xk[4] << " " << 
//				xk[2] << " " << xk[5] << " "); 
//			struct timeval lbegin,lend; 
//			gettimeofday(&lbegin,0); 
//
//
//			if(m_PLDPSolverHerdt==0)
//				m_PLDPSolverHerdt = new Optimization::Solver::PLDPSolverHerdt((unsigned int)m_QP_N, 
//				MAL_RET_MATRIX_DATABLOCK(m_iPu), 
//				MAL_RET_MATRIX_DATABLOCK(m_Px), 
//				m_Pu, 
//				MAL_RET_MATRIX_DATABLOCK(m_iLQ));
//
//
//			unsigned int NumberOfRemovedConstraints = 4; unsigned int NbRemovedFootCstr = 5;
//
//			m_Pb.ifail=m_PLDPSolverHerdt->SolveProblem(QueueOfLConstraintInequalitiesFreeFeet, QueueOfSupportFeet,
//				m_Pb.D, 
//				(unsigned int)m_Pb.m, 
//				m_Pb.DU, 
//				m_Pb.DS, 
//				MAL_RET_VECTOR_DATABLOCK(xk),m_Pb.X, 
//				NumberOfRemovedConstraints, NbRemovedFootCstr,
//				StartingSequence, 
//				(unsigned int)m_Support->StepNumber, 
//				CurrentStateChanged, time); 
//			StartingSequence = false; 
//			//NumberOfRemovedConstraints = NextNumberOfRemovedConstraints; 
//			gettimeofday(&lend,0); 
//			// 	  CODEDEBUG6(double ldt = lend.tv_sec - lbegin.tv_sec + 
//			// 		     0.000001 * (lend.tv_usec - lbegin.tv_usec);); 
//
//			// 	  ODEBUG6(ldt,"dtPLDP.dat"); 
//		} 
//
//		if (m_Pb.ifail!=0) 
//		{ 
//			cout << "IFAIL: " << m_Pb.ifail << " at time: " << time << endl; 
//			//return -1; 
//		} 
//
//		//------------------------ 
//		// 
//		// 
//		//------------------------- 
//
//		double *ptX=0; 
//		if ((m_FastFormulationMode==QLDANDLQ)|| 
//			(m_FastFormulationMode==PLDPHerdt)) 
//		{ 
//			/* Multiply the solution by the transpose of iLQ 
//			because it is a triangular matrix we do a specific 
//			multiplication. 
//			*/ 
//			memset(m_Pb.NewX,0,2*(m_QP_N+m_Support->StepNumber)*sizeof(double)); 
//
//			double *pm_iLQ = MAL_RET_MATRIX_DATABLOCK(m_iLQ); 
//			double *pNewX = m_Pb.NewX; 
//
//			for( int i=0;i<2*(m_QP_N+m_Support->StepNumber);i++) 
//			{ 
//				double *pX= m_Pb.X+i; 
//				double *piLQ = pm_iLQ+i*2*(m_QP_N+m_Support->StepNumber)+i; 
//				*pNewX = 0.0; 
//				for( int j=i;j<2*(m_QP_N+m_Support->StepNumber);j++) 
//				{ 
//					*pNewX+= (*piLQ) * (*pX++); 
//					piLQ+=2*(m_QP_N+m_Support->StepNumber); 
//				} 
//				pNewX++; 
//			} 
//			ptX=m_Pb.NewX; 
//		} 
//		else 
//			ptX=m_Pb.X; 
//
//		/* Simulation of the Single Point Mass model 
//		with the new command. 
//		*/ 
//		ODEBUG("X[0] " << X[0] << " X[N] :" << X[N]); 
//
//
//
//		FinalCOMStates.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod)); 
//		FinalZMPPositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod)); 
//		FinalLeftFootAbsolutePositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod)); 
//		FinalRightFootAbsolutePositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod)); 
//
//		int CurrentIndex = (int)(m_TimeBuffer/m_SamplingPeriod)
//			-1
//			//	-(int)(ldt/m_SamplingPeriod)-1 //<- This part is supposed to be equal to zero.
//			; 
//		ODEBUG("m_TimeBuffer: "<< m_TimeBuffer << 
//			" m_SamplingPeriod: "<< m_SamplingPeriod << 
//			" ldt: " << ldt);
//		ODEBUG("ldt: "<<ldt<<
//			"(int)(ldt/m_SamplingPeriod): "<<(int)(ldt/m_SamplingPeriod)<<
//			"(ldt/m_SamplingPeriod): "<<(ldt/m_SamplingPeriod)); 
//		// update the ZMP and COM positions. 
//		ODEBUG("m_TimeBuffer/m_SamplingPeriod: "<<
//			m_TimeBuffer/m_SamplingPeriod<<
//			"(int)(m_TimeBuffer/m_SamplingPeriod): "<<
//			(int)(m_TimeBuffer/m_SamplingPeriod)); 
//
//		m_2DLIPM->Interpolation(FinalCOMStates, 
//			FinalZMPPositions, 
//			CurrentIndex, 
//			ptX[0],ptX[m_QP_N]); 
//
//		m_2DLIPM->OneIteration(ptX[0],ptX[m_QP_N]); 
//
//
//		//Previewed position of the next foot 
//		if(m_Support->CurrentStepsLeft>0) 
//		{ 
//			if(fabs(ptX[2*m_QP_N])-0.00001>0.0) 
//			{ 
//				m_FPx = ptX[2*m_QP_N]; 
//				m_FPy = ptX[2*m_QP_N+m_Support->StepNumber]; 
//			} 
//		} 
//		else 
//		{//The solver isn't responsible for the feet positions anymore 
//			deque<SupportFeet_t>::iterator CurSF_it; 
//			CurSF_it = QueueOfSupportFeet.end(); 
//			CurSF_it--; 
//			while(CurSF_it->SupportFoot!=m_Support->CurrentSupportFoot) 
//				CurSF_it--; 
//			m_FPx = CurSF_it->x + double(CurSF_it->SupportFoot)*sin(CurSF_it->theta)*m_FeetDistanceDS; 
//			m_FPy = CurSF_it->y - double(CurSF_it->SupportFoot)*cos(CurSF_it->theta)*m_FeetDistanceDS;  
//		} 
//
//
//		if (m_FullDebug>2) 
//		{ 
//			ofstream aof; 
//
//			aof.open("/tmp/FootPositionsT.dat",ofstream::app);
//			aof<<" "<<m_FPx<<" "<<m_FPy<<endl;
//			aof.close();
//			char Buffer[1024]; 
//			if(m_FastFormulationMode == PLDPHerdt)
//				sprintf(Buffer,"/tmp/PLDPXff_%f.dat",time); 
//			else
//				sprintf(Buffer,"/tmp/Xff_%f.dat",time);
//			aof.open(Buffer,ofstream::out); 
//
//			for(int i=0;i<m_QP_N;i++) 
//			{ 
//				aof << ptX[i] << endl; 
//			} 
//			aof.close(); 
//			if(m_FastFormulationMode == PLDPHerdt)
//				sprintf(Buffer,"/tmp/PLDPYff_%f.dat",time); 
//			else
//				sprintf(Buffer,"/tmp/Yff_%f.dat",time);
//			aof.open(Buffer,ofstream::out); 
//			//aof << "State: " <<xk[0]<<" "<<xk[1]<< " " << xk[2] << " " << xk[3] << " "<<xk[4]<<" "<<xk[5]<<" "<<endl; 
//			for(int i=m_QP_N+m_Support->StepNumber;i<2*(m_QP_N+m_Support->StepNumber);i++) 
//			{ 
//				aof << ptX[i] << endl; 
//			} 
//			aof.close(); 
//			aof.open("/tmp/comHeight.dat",ofstream::app); 
//
//			aof << FinalCOMStates[CurrentIndex].x[0]<<" "
//				<< FinalCOMStates[CurrentIndex].y[0]<< " " 
//				<< FinalCOMStates[CurrentIndex].z[0] << " "
//				<< FinalCOMStates[CurrentIndex].roll <<  endl; 
//
//			aof.close(); 
//			aof.open("/tmp/CurrentIndex.dat",ofstream::app); 
//			aof<<CurrentIndex<<endl; 
//			aof.close(); 
//		} 
//
//		//TODO :Jumps of 5ms 
//		if(m_FullDebug>2) 
//		{ 
//			ofstream aof; 
//			aof.open("/tmp/time.dat",ios::app); 
//			aof<<time<<" "<<m_UpperTimeLimitToUpdate<<endl; 
//		} 
//
//		double LocalInterpolationTime = (time+m_TimeBuffer)-(m_Support->CurrentTimeLimit-m_Support->SSPeriod); 
//		// printf("FPx: %f FPy %f \n",FPx,FPy); 
//
//		// printf("Before interpolation \n"); 
//		double StepHeight = 0.05; 
//		// 
//		if(m_Support->CurrentSupportPhase == 1 && time+m_TimeBuffer+3.0/2.0*m_QP_T < m_Support->CurrentTimeLimit) 
//		{ 
//			//determine coefficients of interpolation polynom 
//			double ModulationSupportCoefficient = 0.9; 
//			double ModulatedSingleSupportTime = (m_Support->SSPeriod-m_QP_T) * ModulationSupportCoefficient; 
//			double EndOfLiftOff = ((m_Support->SSPeriod-m_QP_T)-ModulatedSingleSupportTime)*0.5; 
//			double InterpolationTimePassed = 0.0; 
//			if(LocalInterpolationTime>EndOfLiftOff) 
//				InterpolationTimePassed = LocalInterpolationTime-EndOfLiftOff; 
//
//			FootAbsolutePosition LastSwingFootPosition; 
//
//			if(m_Support->CurrentSupportFoot==1) 
//			{ 
//				LastSwingFootPosition = FinalRightFootAbsolutePositions[CurrentIndex]; 
//			} 
//			else 
//			{ 
//				LastSwingFootPosition = FinalLeftFootAbsolutePositions[CurrentIndex]; 
//			} 
//			//Set parameters for current polynomial 
//			m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::X_AXIS, 
//				ModulatedSingleSupportTime-InterpolationTimePassed,m_FPx, 
//				LastSwingFootPosition.x, 
//				LastSwingFootPosition.dx); 
//			m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::Y_AXIS, 
//				ModulatedSingleSupportTime-InterpolationTimePassed,m_FPy, 
//				LastSwingFootPosition.y, 
//				LastSwingFootPosition.dy); 
//
//			if(m_Support->m_StateChanged==true) 
//				m_FTGS->SetParameters(FootTrajectoryGenerationStandard::Z_AXIS, m_Support->SSPeriod-m_QP_T,StepHeight); 
//
//			m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::THETA_AXIS, 
//				ModulatedSingleSupportTime-InterpolationTimePassed,  
//				m_PreviewedSupportAngles[0]*180.0/M_PI, 
//				LastSwingFootPosition.theta, 
//				LastSwingFootPosition.dtheta); 
//			m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::OMEGA_AXIS, 
//				ModulatedSingleSupportTime-InterpolationTimePassed,0.0*180.0/M_PI, 
//				LastSwingFootPosition.omega, 
//				LastSwingFootPosition.domega); 
//			m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::OMEGA2_AXIS, 
//				ModulatedSingleSupportTime-InterpolationTimePassed,2*0.0*180.0/M_PI, 
//				LastSwingFootPosition.omega2, 
//				LastSwingFootPosition.domega2); 
//
//			//Set parameters for trunk interpolation 
//
//			m_c = 3.0*(m_TrunkStateT.yaw[1]-m_TrunkState.yaw[1])/(m_QP_T*m_QP_T); 
//			m_d = -2.0*m_c/(3.0*m_QP_T); 
//			m_a =  m_TrunkState.yaw[1]; 
//
//
//			double tT; 
//			double Theta = m_TrunkState.yaw[0]; 
//			//double dTheta = m_TrunkState.yaw[1]; 
//			//double ddTheta = m_TrunkState.yaw[2]; 
//			int StepType = 1; 
//
//			FinalCOMStates[CurrentIndex].yaw[0] = m_TrunkState.yaw[0]; 
//			//Interpolate the 
//			for(int k = 1; k<=(int)(m_QP_T/m_SamplingPeriod);k++) 
//			{ 
//				tT = (double)k*m_SamplingPeriod; 
//				//interpolate the orientation of the trunk 
//				if(fabs(m_TrunkStateT.yaw[1]-m_TrunkState.yaw[1])-0.000001 > 0) 
//				{ 
//					m_TrunkState.yaw[0] = (((1.0/4.0*m_d*tT+1.0/3.0*m_c)* 
//						tT)*tT+m_a)*tT+Theta; 
//					m_TrunkState.yaw[1] = ((m_d*tT+m_c)*tT)*tT+m_a; 
//					m_TrunkState.yaw[2] = (3.0*m_d*tT+2.0*m_c)*tT; 
//
//					m_QueueOfTrunkStates.push_back(m_TrunkState); 
//				} 
//				else 
//				{ 
//					m_TrunkState.yaw[0] += m_SamplingPeriod*m_TrunkStateT.yaw[1]; 
//
//					m_QueueOfTrunkStates.push_back(m_TrunkState);
//				}
//				FinalCOMStates[CurrentIndex+k].yaw[0] = m_TrunkState.yaw[0];
//				if(m_FullDebug>2)
//				{
//					ofstream aof;
//					aof.open("/tmp/Trunk.dat",ofstream::app);
//					aof<<time+k*m_SamplingPeriod<<" "<<m_TrunkState.yaw[0]<<" "<<m_TrunkState.yaw[1]<<" "<<m_TrunkState.yaw[2]<<endl; 
//					aof.close();
//				}
//
//				if (m_Support->CurrentSupportFoot==1)
//				{
//					m_FTGS->UpdateFootPosition(FinalLeftFootAbsolutePositions,
//						FinalRightFootAbsolutePositions,
//						CurrentIndex,k,
//						LocalInterpolationTime,
//						ModulatedSingleSupportTime,
//						StepType, -1);
//				} 
//				else 
//				{ 
//					m_FTGS->UpdateFootPosition(FinalRightFootAbsolutePositions, 
//						FinalLeftFootAbsolutePositions, 
//						CurrentIndex,k, 
//						LocalInterpolationTime, 
//						ModulatedSingleSupportTime, 
//						StepType, 1); 
//				} 
//				FinalLeftFootAbsolutePositions[CurrentIndex+k].time = 
//					FinalRightFootAbsolutePositions[CurrentIndex+k].time = time+m_TimeBuffer+k*m_SamplingPeriod; 
//
//
//				if(m_FullDebug>0) 
//				{ 
//					ofstream aoffeet; 
//					aoffeet.open("/tmp/Feet.dat",ios::app); 
//					aoffeet<<time+m_TimeBuffer+k*m_SamplingPeriod<<"    " 
//						<<FinalLeftFootAbsolutePositions[CurrentIndex+k].x<<"    " 
//						<<FinalLeftFootAbsolutePositions[CurrentIndex+k].y<<"    " 
//						<<FinalLeftFootAbsolutePositions[CurrentIndex+k].z<<"    " 
//						<<FinalLeftFootAbsolutePositions[CurrentIndex+k].stepType<<"    " 
//						<<FinalRightFootAbsolutePositions[CurrentIndex+k].x<<"    " 
//						<<FinalRightFootAbsolutePositions[CurrentIndex+k].y<<"    " 
//						<<FinalRightFootAbsolutePositions[CurrentIndex+k].z<<"    " 
//						<<FinalRightFootAbsolutePositions[CurrentIndex+k].stepType<<"    " 
//						<<endl; 
//					aoffeet.close(); 
//				} 
//
//			} 
//		} 
//		else if (m_Support->CurrentSupportPhase == 0 || time+m_TimeBuffer+3.0/2.0*m_QP_T > m_Support->CurrentTimeLimit) 
//		{ 
//			// printf("After parametrization SP == 0 \n"); 
//			for(int k = 0; k<=(int)(m_QP_T/m_SamplingPeriod);k++) 
//			{ 
//				FinalCOMStates[CurrentIndex+k].yaw[0] = m_TrunkState.yaw[0];
//
//				// cout<<"CurrentIndex+k"<<CurrentIndex+k<<FinalRightFootAbsolutePositions.size(); 
//				// cout<<" x ,y:"<<FinalRightFootAbsolutePositions[CurrentIndex+k-1].x<< 
//				// 	FinalRightFootAbsolutePositions[CurrentIndex+k-1].y<<endl; 
//				FinalRightFootAbsolutePositions[CurrentIndex+k]=FinalRightFootAbsolutePositions[CurrentIndex+k-1]; 
//				// cout<<"CurrentIndex+k"<<CurrentIndex+k<<endl; 
//				FinalLeftFootAbsolutePositions[CurrentIndex+k]=FinalLeftFootAbsolutePositions[CurrentIndex+k-1]; 
//				// cout<<"CurrentIndex+k"<<CurrentIndex+k<<endl; 
//				FinalLeftFootAbsolutePositions[CurrentIndex+k].time = 
//					FinalRightFootAbsolutePositions[CurrentIndex+k].time = time+m_TimeBuffer+k*m_SamplingPeriod; 
//				// cout<<"CurrentIndex+k"<<CurrentIndex+k<<endl; 
//				FinalLeftFootAbsolutePositions[CurrentIndex+k].stepType = 
//					FinalRightFootAbsolutePositions[CurrentIndex+k].stepType = 10; 
//				// cout<<"CurrentIndex+k"<<CurrentIndex+k<<endl; 
//				// cout<<"LFx: "<<FinalLeftFootAbsolutePositions[CurrentIndex+k].x<<"LFy: "<<FinalLeftFootAbsolutePositions[CurrentIndex+k].y<<"LFz: "<<FinalLeftFootAbsolutePositions[CurrentIndex+k].z<<endl; 
//				// cout<<"RFx: "<<FinalRightFootAbsolutePositions[CurrentIndex+k].x<<"RFy: "<<FinalRightFootAbsolutePositions[CurrentIndex+k].y<<"RFz: "<<FinalRightFootAbsolutePositions[CurrentIndex+k].z<<endl; 
//
//				if(m_FullDebug>0) 
//				{ 
//					ofstream aoffeet; 
//					aoffeet.open("/tmp/Feet.dat",ios::app); 
//					aoffeet<<time+m_TimeBuffer+k*m_SamplingPeriod<<"    " 
//						<<FinalLeftFootAbsolutePositions[CurrentIndex+k].x<<"    " 
//						<<FinalLeftFootAbsolutePositions[CurrentIndex+k].y<<"    " 
//						<<FinalLeftFootAbsolutePositions[CurrentIndex+k].z<<"    " 
//						<<FinalLeftFootAbsolutePositions[CurrentIndex+k].stepType<<"    " 
//						<<FinalRightFootAbsolutePositions[CurrentIndex+k].x<<"    " 
//						<<FinalRightFootAbsolutePositions[CurrentIndex+k].y<<"    " 
//						<<FinalRightFootAbsolutePositions[CurrentIndex+k].z<<"    " 
//						<<FinalRightFootAbsolutePositions[CurrentIndex+k].stepType<<"    " 
//						<<endl; 
//					aoffeet.close(); 
//				} 
//			} 
//		} 
//
//		if(m_UpperTimeLimitToUpdate==0.0) 
//			m_UpperTimeLimitToUpdate = time+m_QP_T; 
//		else 
//			m_UpperTimeLimitToUpdate = m_UpperTimeLimitToUpdate+m_QP_T; 
//
//		//cout<<m_TrunkState.yaw[0]<<"   "<<m_TrunkStateT.yaw[0]; 
//		//		m_TrunkState.yaw[0] = m_TrunkStateT.yaw[0]; 
//		//		m_TrunkState.yaw[1] = m_AngVelTrunkConst; 
//
//		ODEBUG6("uk:" << uk,"DebugPBW.dat"); 
//		ODEBUG6("xk:" << xk,"DebugPBW.dat"); 
//
//
//		if (m_FullDebug>2) 
//		{ 
//			ofstream aof; 
//			char Buffer[1024]; 
//			sprintf(Buffer,"/tmp/Xff_%f.dat",time); 
//			aof.open(Buffer,ofstream::out); 
//			//aof << "State: " <<xk[0]<<" "<<xk[1]<< " " << xk[2] << " " << xk[3] << " "<<xk[4]<<" "<<xk[5]<<" "<<endl; 
//			for( int i=0;i<2*(m_QP_N+m_Support->StepNumber);i++) 
//			{ 
//				aof << m_Pb.X[i] << endl; 
//			} 
//			aof.close(); 
//			// sprintf(Buffer,"Uff_%f.dat",StartingTime); 
//			// aof.open(Buffer,ofstream::out); 
//			// for(unsigned int i=0;i<2*(N+m_Support->StepNumber);i++) 
//			//   { 
//			//     aof << U[i] << endl; 
//			//   } 
//			// aof.close(); 
//		} 
//
//		if(m_FullDebug>2) 
//		{ 
//			//if(validateConstraints(m_Pb.DS, m_Pb.DU, m_Pb.m, li, m_Pb.X, time)<0) 
//			//  { 
//			//    cout << "Something is wrong with the constraints." << endl; 
//			//    exit(-1); 
//			//  } 
//		} 
//
//		// Compute CPU consumption time. 
//		gettimeofday(&end,0); 
//		CurrentCPUTime = end.tv_sec - start.tv_sec + 
//			0.000001 * (end.tv_usec - start.tv_usec); 
//		TotalAmountOfCPUTime += CurrentCPUTime; 
//		ODEBUG("Current Time : " <<time << " " << 
//			" Virtual time to simulate: " << QueueOfLConstraintInequalities.back()->EndingTime - time << 
//			"Computation Time " << CurrentCPUTime << " " << TotalAmountOfCPUTime); 
//
//		QueueOfLConstraintInequalitiesFreeFeet.clear(); 
//		QueueOfFeetPosInequalities.clear(); 
//
//		delete [] m_Pb.Q; 
//		delete [] m_Pb.D; 
//		delete [] m_Pb.DS; 
//		delete [] m_Pb.DU; 
//		delete [] m_Pb.XL; 
//		delete [] m_Pb.XU; 
//		delete [] m_Pb.X; 
//		delete [] m_Pb.NewX; 
//		delete [] m_Pb.iwar; // The Cholesky decomposition is done internally. 
//
//		delete [] m_Pb.war; 
//		free(m_Pb.U); 
//
//
//
//	} 
//
//	// printf("Leaving online \n"); 
//	//----------------------------------- 
//	// 
//	// 
//	//----------"Real-time" loop-------- 
//
//} 
//
//
//


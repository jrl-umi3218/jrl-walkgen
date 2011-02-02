/*
 * Copyright 2011,
 *
 * Mehdi Benallegue
 * Andrei   Herdt
 * Olivier  Stasse
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
/*! This object constructs a QP as proposed by Herdt IROS 2010.
 */

#include "portability/gettimeofday.hh"

#include <ZMPRefTrajectoryGeneration/generator-step-pos.hh>

using namespace std;
using namespace PatternGeneratorJRL;




GeneratorStepPos::GeneratorStepPos(SimplePluginManager *lSPM ) : GeneratorVelRef(lSPM)
{ 
	Initialization();
}

GeneratorStepPos::GeneratorStepPos(const GeneratorVelRef &genvr ) : GeneratorVelRef(genvr)
{ 
	Initialization();
}


void GeneratorStepPos::Initialization()
{
	velocityMode_=true;

	// Register method to handle
	string aMethodName[2] =
	{":setstepswidth",":setstepspositions"};

	for(int i=0;i<2;i++)
	{
		if (!RegisterMethod(aMethodName[i]))
		{
			std::cerr << "Unable to register " << aMethodName << std::endl;
		}
    }

	RelativeStepPosition p;
	p.x=0.1;
	p.y=0;
	p.theta=0;
	stepPos_.push_back(p);
}


void GeneratorStepPos::SetStepsPositions(const RelativeStepPositionQueue& s)
{
	if (s.size()>0)
		stepPos_=s;
	else
		throw std::runtime_error("GeneratorStepPos::SetStepsPositions : Empty queue as an input");

}

const RelativeStepPositionQueue &GeneratorStepPos::GetStepsPositions() const
{
	return stepPos_;
}



void
GeneratorStepPos::build_constraints( QPProblem & Pb,
				  RelativeFeetInequalities * RFI,
				  const std::deque< FootAbsolutePosition> & AbsoluteLeftFootPositions,
				  const std::deque<FootAbsolutePosition> & AbsoluteRightFootPositions,
				  const std::deque<support_state_t> & SupportStates_deq,
				  const std::deque<double> & PreviewedSupportAngles )
{
	if (velocityMode_)
	{
		GeneratorVelRef::build_constraints(Pb,RFI,AbsoluteLeftFootPositions,
			AbsoluteRightFootPositions,SupportStates_deq,PreviewedSupportAngles);
	}
	else
	{

		const IntermedQPMat::state_variant_t & State = Matrices_.State();
		int NbStepsPreviewed = SupportStates_deq.back().StepNumber;

		//Equality constraints for feet positions
		linear_inequality_t & EqStepPos = Matrices_.Inequalities(IntermedQPMat::EQ_STEP_POS);
		int equalities = build_equalities_step_pos(EqStepPos, AbsoluteLeftFootPositions, AbsoluteRightFootPositions,
			SupportStates_deq, PreviewedSupportAngles);
		build_constraints_step_pos(EqStepPos, State, NbStepsPreviewed, Pb);
		//Pb.setNbEqConstraints(equalities);

		//CoP constraints
		linear_inequality_t & IneqCoP = Matrices_.Inequalities(IntermedQPMat::INEQ_COP);
		build_inequalities_cop(IneqCoP, RFI,
			AbsoluteLeftFootPositions, AbsoluteRightFootPositions,
			SupportStates_deq, PreviewedSupportAngles);

		const IntermedQPMat::dynamics_t & CoP = Matrices_.Dynamics(IntermedQPMat::COP);
		build_constraints_cop(IneqCoP, CoP, State, NbStepsPreviewed, Pb);

		//Feet constraints
		//linear_inequality_t & IneqFeet = Matrices_.Inequalities(IntermedQPMat::INEQ_FEET);
		//build_inequalities_feet(IneqFeet, RFI,
		//	AbsoluteLeftFootPositions, AbsoluteRightFootPositions,
		//	SupportStates_deq, PreviewedSupportAngles);

		//build_constraints_feet(IneqFeet, State, NbStepsPreviewed, Pb);
	}
}

int
GeneratorStepPos::build_equalities_step_pos(linear_inequality_t & equalities,
				       const std::deque< FootAbsolutePosition> & AbsoluteLeftFootPositions,
				       const std::deque<FootAbsolutePosition> & AbsoluteRightFootPositions,
				       const std::deque<support_state_t> & SupportStates_deq,
				       const std::deque<double> & PreviewedSupportAngles)
{

	const support_state_t & CurrentSupport = SupportStates_deq.front();


	double CurrentSupportAngle;
	if( CurrentSupport.Foot==1 )
		CurrentSupportAngle = AbsoluteLeftFootPositions.back().theta*M_PI/180.0;
	else
		CurrentSupportAngle = AbsoluteRightFootPositions.back().theta*M_PI/180.0;


	//set constraints for the whole preview window
	double SupportAngle = CurrentSupportAngle;
	
	
	int NbStepsPreviewed = SupportStates_deq.back().StepNumber;
	equalities.resize(4*NbStepsPreviewed,NbStepsPreviewed, false);
	
	//fill stepPos by duplicating the last value
	for (int i=stepPos_.size();i<CurrentSupport.StepNumber+NbStepsPreviewed;i++)
	{
		stepPos_.push_back(stepPos_.back());
	}

	int nb_step = 0;
	RelativeStepPosition rsp;
	for( int i=1;i<=m_N;i++ )
	{

		const support_state_t & PrwSupport = SupportStates_deq[i];

		//foot positioning constraints
		if( PrwSupport.StateChanged && PrwSupport.StepNumber>0 && PrwSupport.Phase != 0)
		{

			SupportAngle = PreviewedSupportAngles[PrwSupport.StepNumber-1];

			if( PrwSupport.StepNumber == 1 )
				SupportAngle = CurrentSupportAngle;
			else
				SupportAngle = PreviewedSupportAngles[PrwSupport.StepNumber-2];

			rfe_.setVertice(rsp,stepPos_[CurrentSupport.StepNumber],PrwSupport,SupportAngle);

			equalities.x.D.push_back((PrwSupport.StepNumber-1)*4, (PrwSupport.StepNumber-1), 1);
			//equalities.y.D.push_back((PrwSupport.StepNumber-1)*4, (PrwSupport.StepNumber-1), 0);
			equalities.dc((PrwSupport.StepNumber-1)*4) =rsp.x ;
			
			equalities.x.D.push_back((PrwSupport.StepNumber-1)*4+1, (PrwSupport.StepNumber-1), -1);
			//equalities.y.D.push_back((PrwSupport.StepNumber-1)*4+1, (PrwSupport.StepNumber-1), 0);
			equalities.dc((PrwSupport.StepNumber-1)*4+1) =-(rsp.x-0.000001) ;

			//equalities.x.D.push_back((PrwSupport.StepNumber-1)*4+2, (PrwSupport.StepNumber-1), 0);
			equalities.y.D.push_back((PrwSupport.StepNumber-1)*4+2, (PrwSupport.StepNumber-1), 1);
			equalities.dc((PrwSupport.StepNumber-1)*4+2) =rsp.y ;

			//equalities.x.D.push_back((PrwSupport.StepNumber-1)*4+3, (PrwSupport.StepNumber-1), 0);
			equalities.y.D.push_back((PrwSupport.StepNumber-1)*4+3, (PrwSupport.StepNumber-1), -1);
			equalities.dc((PrwSupport.StepNumber-1)*4+3) =-(rsp.y-0.000001) ;
			
			nb_step++;
		}
	}
	return nb_step*4;
}




void
GeneratorStepPos::build_constraints_step_pos(const linear_inequality_t & eqSteps,
				      const IntermedQPMat::state_variant_t & State,
				      int NbStepsPreviewed, QPProblem & Pb)
{

  const int & NbConstraints = eqSteps.dc.size();

  boost_ublas::matrix<double> MM(NbConstraints,NbStepsPreviewed,false);

  // -D*V_f
  compute_term(MM,-1.0,eqSteps.x.D,State.V_f);
  Pb.add_term(MM,QPProblem::MATRIX_DU,4*m_N,2*m_N);
  compute_term(MM,-1.0,eqSteps.y.D,State.V_f);
  Pb.add_term(MM,QPProblem::MATRIX_DU,4*m_N,2*m_N+NbStepsPreviewed);

  // +dc
  Pb.add_term(eqSteps.dc,QPProblem::VECTOR_DS,4*m_N);

  // +D*Vc_f*FP
  boost_ublas::vector<double> MV(NbConstraints*NbStepsPreviewed,false);
  compute_term(MV, State.SupportState.x, eqSteps.x.D, State.Vc_f);
  Pb.add_term(MV,QPProblem::VECTOR_DS,4*m_N);
  compute_term(MV, State.SupportState.y, eqSteps.y.D, State.Vc_f);
  Pb.add_term(MV,QPProblem::VECTOR_DS,4*m_N);

}

void 
GeneratorStepPos::setVelocityMode(bool vm)
{
	velocityMode_=vm;
}

void 
GeneratorStepPos::Ponderation(double weight,int objective)
{
	if (velocityMode_)
	{
		GeneratorVelRef::Ponderation(weight,objective);
	}
}

void 
GeneratorStepPos::explicitPonderation(double weight,int objective)
{
	GeneratorVelRef::Ponderation(weight,objective);
	
}

 

int 
GeneratorStepPos::ChangeStepPosition(const RelativeStepPosition & r,
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

int GeneratorStepPos::AddStepPosition(const RelativeStepPosition & r)
{
	stepPos_.push_back(r);
	return stepPos_.size();

}

 void
GeneratorStepPos::CallMethod( std::string &Method, std::istringstream &Args )
{

	if ( Method==":setstepswidth" )
	{
		double stepWidth;

		Args>> stepWidth;
		rfe_.setStepWidth(stepWidth);
		cout << "stepWidth: " << stepWidth<< endl;

	}
	else if (Method==":setstepspositions") 
	{ 
		RelativeStepPosition p;
		Args>>p.x;
		Args>>p.y;
		Args>>p.theta;

		if (!Args.eof())
		{
			RelativeStepPositionQueue stepPos;

			do{
				stepPos.push_back(p);
				Args>>p.x;
				Args>>p.y;
				Args>>p.theta;
			}while (!Args.eof());

			SetStepsPositions(stepPos);
		}


	}
	else
	{
		GeneratorVelRef::CallMethod(Method,Args);
	}

}
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
	velocityMode_=true;

	// Register method to handle
	string aMethodName[] =
	{":setstepswidth"};

	for(int i=0;i<1;i++)
	{
		if (!RegisterMethod(aMethodName[i]))
		{
			std::cerr << "Unable to register " << aMethodName << std::endl;
		}
    }
}

GeneratorStepPos::GeneratorStepPos(const GeneratorVelRef &genvr ) : GeneratorVelRef(genvr)
{ 
	velocityMode_=true;
	
	
	// Register method to handle
	string aMethodName[] =
	{":setstepswidth"};

	for(int i=0;i<1;i++)
	{
		if (!RegisterMethod(aMethodName[i]))
		{
			std::cerr << "Unable to register " << aMethodName << std::endl;
		}
    }
}



void
GeneratorStepPos::build_constraints( QPProblem & Pb,
				  RelativeFeetInequalities * RFI,
				  const std::deque< FootAbsolutePosition> & AbsoluteLeftFootPositions,
				  const std::deque<FootAbsolutePosition> & AbsoluteRightFootPositions,
				  const std::deque<support_state_t> & SupportStates_deq,
				  const std::deque<double> & PreviewedSupportAngles )
{
	//Equality constraints for feet positions

	
	  //CoP constraints
  linear_inequality_t & IneqCoP = Matrices_.Inequalities(IntermedQPMat::INEQ_COP);
  build_inequalities_cop(IneqCoP, RFI,
		       AbsoluteLeftFootPositions, AbsoluteRightFootPositions,
		       SupportStates_deq, PreviewedSupportAngles);

  const IntermedQPMat::dynamics_t & CoP = Matrices_.Dynamics(IntermedQPMat::COP);
  const IntermedQPMat::state_variant_t & State = Matrices_.State();
  int NbStepsPreviewed = SupportStates_deq.back().StepNumber;
  build_constraints_cop(IneqCoP, CoP, State, NbStepsPreviewed, Pb);

  //Feet constraints
  linear_inequality_t & IneqFeet = Matrices_.Inequalities(IntermedQPMat::INEQ_FEET);
  build_inequalities_feet(IneqFeet, RFI,
			AbsoluteLeftFootPositions, AbsoluteRightFootPositions,
			SupportStates_deq, PreviewedSupportAngles);

  build_constraints_feet(IneqFeet, State, NbStepsPreviewed, Pb);

}

int
GeneratorStepPos::build_equalities_step_pos(linear_inequality_t & equalities,
				       RelativeFeetInequalities * RFI,
				       const std::deque< FootAbsolutePosition> & AbsoluteLeftFootPositions,
				       const std::deque<FootAbsolutePosition> & AbsoluteRightFootPositions,
				       const std::deque<support_state_t> & SupportStates_deq,
				       const std::deque<double> & PreviewedSupportAngles,
					   RelativeStepPositionQueue& stepPos ) const
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
	equalities.resize(2*NbStepsPreviewed,NbStepsPreviewed, false);
	
	//fill stepPos by duplicating the last value
	for (int i=stepPos.size();i<CurrentSupport.StepNumber+NbStepsPreviewed;i++)
	{
		stepPos.push_back(stepPos.back());
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

			rfe_.setVertice(rsp,stepPos[CurrentSupport.StepNumber],PrwSupport,SupportAngle);

			equalities.x.D.push_back((PrwSupport.StepNumber-1)*2, (PrwSupport.StepNumber-1), 1);
			equalities.dc((PrwSupport.StepNumber-1)*2) =rsp.x ;
			equalities.y.D.push_back((PrwSupport.StepNumber-1)*2+1, (PrwSupport.StepNumber-1), 1);
			equalities.dc((PrwSupport.StepNumber-1)*2+1) =rsp.y ;
			nb_step++;
		}
	}
	return nb_step*2;
}




void
GeneratorStepPos::build_constraints_step_pos(const linear_inequality_t & IneqFeet,
				      const IntermedQPMat::state_variant_t & State,
				      int NbStepsPreviewed, QPProblem & Pb)
{

  const int & NbConstraints = IneqFeet.dc.size();

  boost_ublas::matrix<double> MM(NbConstraints,NbStepsPreviewed,false);

  // -D*V_f
  compute_term(MM,-1.0,IneqFeet.x.D,State.V_f);
  Pb.add_term(MM,QPProblem::MATRIX_DU,4*m_N,2*m_N);
  compute_term(MM,-1.0,IneqFeet.y.D,State.V_f);
  Pb.add_term(MM,QPProblem::MATRIX_DU,4*m_N,2*m_N+NbStepsPreviewed);

  // +dc
  Pb.add_term(IneqFeet.dc,QPProblem::VECTOR_DS,4*m_N);

  // +D*Vc_f*FP
  boost_ublas::vector<double> MV(NbConstraints*NbStepsPreviewed,false);
  compute_term(MV, State.SupportState.x, IneqFeet.x.D, State.Vc_f);
  Pb.add_term(MV,QPProblem::VECTOR_DS,4*m_N);
  compute_term(MV, State.SupportState.y, IneqFeet.y.D, State.Vc_f);
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

}
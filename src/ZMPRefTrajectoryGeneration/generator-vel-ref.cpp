/*
 * Copyright 2010,
 *
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


#include <ZMPRefTrajectoryGeneration/generator-vel-ref.hh>

using namespace PatternGeneratorJRL;


GeneratorVelRef::GeneratorVelRef(SimplePluginManager *lSPM, std::string DataFile,
				 CjrlHumanoidDynamicRobot *aHS) : MPCTrajectoryGeneration(lSPM)
{
  //TODO:
}
	
		
GeneratorVelRef::~GeneratorVelRef()
{
  //TODO:
}

	
void
GeneratorVelRef::CallMethod(std::string &Method, std::istringstream &strm)
{
  GeneratorVelRef::CallMethod(Method,strm);
}

	
void 
GeneratorVelRef::setPonderation(double alpha, double beta, double gamma, double delta)
{
  //TODO:	
}


void 
GeneratorVelRef::setReference(std::istringstream &strm)
{
  //TODO:
}
	
	
void 
GeneratorVelRef::setReference(double dx, double dy, double dyaw)
{
  //TODO:
}


void 
GeneratorVelRef::buildConstantPartOfObjective(QPProblem & Pb)
{
  //TODO:
}


void 
GeneratorVelRef::addEqConstraint(std::deque<LinearConstraintInequalityFreeFeet_t> ConstraintsDeque,
				 MAL_MATRIX (&DU, double), MAL_MATRIX (&DS, double))
{
  //TODO:
}
	  

void
GeneratorVelRef::addIneqConstraint(std::deque<LinearConstraintInequalityFreeFeet_t> ConstraintsDeque,
				   MAL_MATRIX (&DU, double), MAL_MATRIX (&DS, double))
{
  //TODO:
}
	  

void 
GeneratorVelRef::generateZMPConstraints (CjrlFoot & Foot,
					 std::deque < double > & PreviewedSupportAngles,
					 SupportFSM & FSM,
					 SupportState_t & CurrentSupportState,
					 QPProblem & Pb)
{
  //TODO:
}


void 
GeneratorVelRef::generateFeetPosConstraints (CjrlFoot & Foot,
					     std::deque < double > & PreviewedSupportAngles,
					     SupportFSM & ,
					     SupportState_t &,
					     QPProblem & Pb)
{
  //TODO:
}


void 
GeneratorVelRef::computeObjective(QPProblem & Pb, IntermedQPMat & QPMat, std::deque<SupportState_t> PrwSupStates)
{
  //TODO:
}


int 
GeneratorVelRef::initConstants(QPProblem & Pb)
{
  //TODO:
}


void 
GeneratorVelRef::setCoMPerturbationForce(double Fx, double Fy, 
					 LinearizedInvertedPendulum2D & CoM)
{
  //TODO:
}
	  

void 
GeneratorVelRef::setCoMPerturbationForce(std::istringstream &strm,
					 LinearizedInvertedPendulum2D & CoM)
{
  //TODO:
}
	  

void
GeneratorVelRef::addLeastSquaresTerm(ObjectiveTerm_t & type, QPProblem & Pb, int NbSt)
{
  //TODO:
}
	  

int 
GeneratorVelRef::buildConstantPartOfConstraintMatrices()
{
  //TODO:
}

	
int 
GeneratorVelRef::buildConstantPartOfTheObjectiveFunction()
{
  //TODO:
}


int 
GeneratorVelRef::initializeMatrixPbConstants()
{
  //TODO:
}
	  
	  
int 
GeneratorVelRef::dumpProblem(MAL_VECTOR(& xk,double),
			     double Time)
{
  //TODO:
}

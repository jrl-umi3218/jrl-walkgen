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
GeneratorVelRef::updateObjective(QPProblem & Pb, IntermedQPMat & QPMatrices, std::deque<SupportState_t> PrwSupStates)
{

IntermedQPMat::standard_ls_form_t LSMat;

QPMatrices.getTermMatrices(LSMat, IntermedQPMat::INSTANT_VELOCITY);
updateLSObjTerm(QPMatrices, Pb.Q, Pb.D, LSMat, INSTANT_VELOCITY);

QPMatrices.getTermMatrices(LSMat, IntermedQPMat::COP_CENTERING);
updateLSObjTerm(QPMatrices, Pb.Q, Pb.D, LSMat, COP_CENTERING);

QPMatrices.getTermMatrices(LSMat, IntermedQPMat::JERK);
updateLSObjTerm(QPMatrices, Pb.Q, Pb.D, LSMat, JERK);

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
GeneratorVelRef::updateLSObjTerm(IntermedQPMat & QPMatrices, double * Q, double *p,
    IntermedQPMat::standard_ls_form_t LSMat, int ObjectiveType)
{

  // Quadratic part of the objective
  switch(ObjectiveType)
  {
  case COP_CENTERING:
    LSMat.weightM1TM2 = -LSMat.weight*MAL_RET_A_by_B(LSMat.UT,LSMat.V);
    addTerm(LSMat.weightM1TM2, Q, 0, 2*m_QP_N, m_QP_N, LSMat.nSt);
    addTerm(LSMat.weightM1TM2, Q, m_QP_N, 2*m_QP_N+LSMat.nSt, m_QP_N, LSMat.nSt);

    LSMat.weightM1TM2 = -LSMat.weight*MAL_RET_A_by_B(LSMat.VT,LSMat.U);
    addTerm(LSMat.weightM1TM2, Q, 2*m_QP_N, 0, LSMat.nSt, m_QP_N);
    addTerm(LSMat.weightM1TM2, Q, 2*m_QP_N+LSMat.nSt, m_QP_N, LSMat.nSt, m_QP_N);

    LSMat.weightM1TM2 = LSMat.weight*MAL_RET_A_by_B(LSMat.VT,LSMat.V);
    addTerm(LSMat.weightM1TM2, Q, 2*m_QP_N, 2*m_QP_N, LSMat.nSt, LSMat.nSt);
    addTerm(LSMat.weightM1TM2, Q, 2*m_QP_N+LSMat.nSt, 2*m_QP_N+LSMat.nSt, LSMat.nSt, LSMat.nSt);
  }

  // Linear part of the objective
  LSMat.weightM1TV1 = LSMat.weight*MAL_RET_A_by_B(LSMat.UT,LSMat.Sc_x);
  addTerm(LSMat.weightM1TV1, p, 0, m_QP_N);
  LSMat.weightM1TV1 = LSMat.weight*MAL_RET_A_by_B(LSMat.UT,LSMat.Sc_y);
  addTerm(LSMat.weightM1TV1, p, m_QP_N, m_QP_N);
  LSMat.weightM1TV1 = -LSMat.weight*MAL_RET_A_by_B(LSMat.UT,LSMat.ref_x);
  addTerm(LSMat.weightM1TV1, p, 0, m_QP_N);
  LSMat.weightM1TV1 = -LSMat.weight*MAL_RET_A_by_B(LSMat.UT,LSMat.ref_y);
  addTerm(LSMat.weightM1TV1, p, m_QP_N, m_QP_N);

  switch(ObjectiveType)
  {
  case COP_CENTERING:
    LSMat.weightM1TV1 = -LSMat.weight*MAL_RET_A_by_B(LSMat.VT,LSMat.Sc_x);
    addTerm(LSMat.weightM1TV1, p, 2*m_QP_N, LSMat.nSt);
    LSMat.weightM1TV1 = -LSMat.weight*MAL_RET_A_by_B(LSMat.VT,LSMat.Sc_y);
    addTerm(LSMat.weightM1TV1, p, 2*m_QP_N+LSMat.nSt, LSMat.nSt);
    LSMat.weightM1TV1 = LSMat.weight*MAL_RET_A_by_B(LSMat.UT,LSMat.ref_x);
    addTerm(LSMat.weightM1TV1, p, 2*m_QP_N, LSMat.nSt);
    LSMat.weightM1TV1 = LSMat.weight*MAL_RET_A_by_B(LSMat.UT,LSMat.ref_y);
    addTerm(LSMat.weightM1TV1, p, 2*m_QP_N+LSMat.nSt, LSMat.nSt);
  }

}


void
GeneratorVelRef::addTerm(MAL_MATRIX (&Mat, double), double * QPMat, int row, int col, int nrows, int ncols)
{
  for(int i = row;i < row+nrows; i++)
    for(int j = col;j < col+ncols; j++)
      QPMat[i+j*2*(m_QP_N+m_PrwSupport.StepNumber)] = Mat(i,j);
}


void
GeneratorVelRef::addTerm(MAL_VECTOR (&Vec, double), double * QPVec, int index, int nelem)
{
  for(int i = index;i < nelem; i++)
      QPVec[i] = Vec(i);
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


void
GeneratorVelRef::updateMatrices(IntermedQPMat & QPMatrices,
        const LinearizedInvertedPendulum2D & CoM,
        const std::deque<SupportState_t> PrwSupportStates,
        const ReferenceAbsoluteVelocity_t & RefVel,
        const COMState & Trunk,
        const std::deque<SupportFeet_t> & QueueOfSupportFeet)
{



}

	  
int 
GeneratorVelRef::dumpProblem(MAL_VECTOR(& xk,double),
			     double Time)
{
  //TODO:
}

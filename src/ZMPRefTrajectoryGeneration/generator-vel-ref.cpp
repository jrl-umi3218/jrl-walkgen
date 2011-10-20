/*
 * Copyright 2011,
 *
 * Olivier  Stasse
 * Andrei   Herdt
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

#include <ZMPRefTrajectoryGeneration/generator-vel-ref.hh>

using namespace std;
using namespace PatternGeneratorJRL;


GeneratorVelRef::GeneratorVelRef(SimplePluginManager *lSPM,
    IntermedQPMat * Data, RigidBodySystem * Robot, RelativeFeetInequalities * RFI )
: MPCTrajectoryGeneration(lSPM)
, IntermedData_ (Data)
, Robot_(Robot)
, RFI_(RFI)
, MM_(1,1,false)
, MM2_(1,1,false)
, MV_(1,false)
, MV2_(1,false)
{
}


GeneratorVelRef::~GeneratorVelRef()
{}


//void
//GeneratorVelRef::CallMethod(std::string &Method, std::istringstream &strm)
//{
//  //GeneratorVelRef::CallMethod(Method,strm);
//}


void 
GeneratorVelRef::Ponderation( double weight, objective_e type)
{

  IntermedQPMat::objective_variant_t & Objective = IntermedData_->Objective( type );
  Objective.weight = weight;

}	


void
GeneratorVelRef::preview_support_states( double time, const SupportFSM * FSM,
    const deque<FootAbsolutePosition> & FinalLeftFootTraj_deq, const deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
    deque<support_state_t> & SupportStates_deq )
{

  const FootAbsolutePosition * FAP = 0;

  // DETERMINE CURRENT SUPPORT STATE:
  // --------------------------------
  const reference_t & RefVel = IntermedData_->Reference();
  support_state_t & CurrentSupport = IntermedData_->SupportState();
  FSM->set_support_state( CurrentTime_, 0, CurrentSupport, RefVel );
  if( CurrentSupport.StateChanged == true )
    {
      if( CurrentSupport.Foot == LEFT )
        FAP = & FinalLeftFootTraj_deq.front();
      else
        FAP = & FinalRightFootTraj_deq.front();
      CurrentSupport.X = FAP->x;
      CurrentSupport.Y = FAP->y;
      CurrentSupport.Yaw = FAP->theta*M_PI/180.0;
      CurrentSupport.StartTime = time;
    }
  SupportStates_deq.push_back( CurrentSupport );
  IntermedData_->SupportState( CurrentSupport );


  // PREVIEW SUPPORT STATES:
  // -----------------------
  // initialize the previewed support state before previewing
  support_state_t PreviewedSupport = CurrentSupport;
  PreviewedSupport.StepNumber  = 0;
  for( unsigned spNb=1; spNb<=N_; spNb++ )
    {
      FSM->set_support_state( CurrentTime_, spNb, PreviewedSupport, RefVel );
      if( spNb == 1 && PreviewedSupport.StateChanged )//Foot down
        {
          if( PreviewedSupport.Foot == LEFT )
            FAP = & FinalLeftFootTraj_deq.back();
          else
            FAP = & FinalRightFootTraj_deq.back();
          PreviewedSupport.X = FAP->x;
          PreviewedSupport.Y = FAP->y;
          PreviewedSupport.Yaw = FAP->theta*M_PI/180.0;
          PreviewedSupport.StartTime = time+spNb*Tprw_;
        }
      SupportStates_deq.push_back( PreviewedSupport );
    }


  // GENERATE SUPPORT SELECTION MATRICES:
  // ------------------------------------
  generate_selection_matrices( SupportStates_deq );

}


void
GeneratorVelRef::generate_selection_matrices( const std::deque<support_state_t> & SupportStates_deq )
{

  IntermedQPMat::state_variant_t & State = IntermedData_->State();
  const int & NbPrwSteps = SupportStates_deq.back().StepNumber;

  bool Preserve = true;
  State.Vc.resize(N_,!Preserve);
  State.Vc.clear();
  State.V.resize(N_,NbPrwSteps,!Preserve);
  State.V.clear();
  State.VT.resize(NbPrwSteps,N_,!Preserve);
  State.VT.clear();
  State.Vc_f.resize(NbPrwSteps,!Preserve);
  State.Vc_f.clear();
  State.V_f.resize(NbPrwSteps,NbPrwSteps,!Preserve);
  State.V_f.clear();


  std::deque<support_state_t>::const_iterator SS_it;
  SS_it = SupportStates_deq.begin();//points at the cur. sup. st.

  SS_it++;
  for(unsigned int i=0;i<N_;i++)
    {
      if(SS_it->StepNumber>0)
        {
          State.V(i,SS_it->StepNumber-1) = State.VT(SS_it->StepNumber-1,i) = 1.0;
          if(SS_it->StepNumber==1)
            {
              State.Vc_f(0) = 1.0;
              State.V_f(0,0) = 1.0;
            }
          else if(SS_it->StepNumber>1)
            {
              State.V_f(SS_it->StepNumber-1,SS_it->StepNumber-2) = -1.0;
              State.V_f(SS_it->StepNumber-1,SS_it->StepNumber-1) = 1.0;
            }
        }
      else
        State.Vc(i) = 1.0;
      SS_it++;
    }

}


void 
GeneratorVelRef::compute_global_reference( const solution_t & Solution )
{

  reference_t & Ref = IntermedData_->Reference();

  Ref.Global.X_vec.resize(N_,false);
  Ref.Global.X_vec.clear();
  Ref.Global.Y_vec.resize(N_,false);
  Ref.Global.Y_vec.clear();
  double YawTrunk;
  for( unsigned int i=0;i<N_;i++ )
    {
      YawTrunk = Solution.TrunkOrientations_deq[i];
      Ref.Global.X_vec(i) = Ref.Local.X*cos(YawTrunk)-Ref.Local.Y*sin(YawTrunk);
      Ref.Global.Y_vec(i) = Ref.Local.Y*cos(YawTrunk)+Ref.Local.X*sin(YawTrunk);
    }

}


void
GeneratorVelRef::initialize_matrices(option_e option)
{

  linear_inequality_t & IneqCoP = IntermedData_->Inequalities( INEQ_COP );
  initialize_matrices( IneqCoP, option );


}


void
GeneratorVelRef::initialize_matrices( linear_inequality_t & Inequalities, option_e option)
{

  switch(Inequalities.type)
  {
  case INEQ_COP:
	int ineqSize=4;
	if (option==WITH_TWO_CONTRAINT_BOUNDS){
	  ineqSize=2;
	}
	Inequalities.D.x.resize(ineqSize*N_,N_,false);
    Inequalities.D.x.clear();
    Inequalities.D.y.resize(ineqSize*N_,N_,false);
    Inequalities.D.y.clear();
    Inequalities.dc.resize(ineqSize*N_,false);
    Inequalities.dc.clear();
    break;
  }

}


void 
GeneratorVelRef::build_inequalities_cop(linear_inequality_t & Inequalities,
    const std::deque<support_state_t> & SupportStates_deq,
    option_e option) const
{

  deque<support_state_t>::const_iterator prwSS_it = SupportStates_deq.begin();

  convex_hull_t ZMPFeasibilityEdges;
  RFI_->set_vertices( ZMPFeasibilityEdges,
      *prwSS_it,
      INEQ_COP );

  const unsigned nbEdges = 4;
  unsigned NbEdgesLoop= nbEdges ;
  if (option==WITH_TWO_CONTRAINT_BOUNDS){
	  NbEdgesLoop=2;
  }
  double D_x[nbEdges] = {0.0, 0.0, 0.0, 0.0};
  double D_y[nbEdges] = {0.0, 0.0, 0.0, 0.0};
  double dc[nbEdges] = {0.0, 0.0, 0.0, 0.0};

  prwSS_it++;//Point at the first previewed instant
  for( unsigned i=0; i<N_; i++ )
    {
        RFI_->set_vertices( ZMPFeasibilityEdges,*prwSS_it,
            INEQ_COP );
      	RFI_->compute_linear_system( ZMPFeasibilityEdges, D_x, D_y, dc, *prwSS_it );

      for( unsigned j = 0; j < NbEdgesLoop; j++ )
        {
          Inequalities.D.x.push_back(i*NbEdgesLoop+j,i,D_x[j]);
          Inequalities.D.y.push_back(i*NbEdgesLoop+j,i,D_y[j]);
          Inequalities.dc(i*NbEdgesLoop+j) = dc[j];
        }

      prwSS_it++;
    }


}


void
GeneratorVelRef::build_inequalities_feet( linear_inequality_t & Inequalities,
    const std::deque<support_state_t> & SupportStates_deq ) const
{

  // Arrays for the generated set of inequalities
  const unsigned nbEdges = 5;
  double Dx[nbEdges] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double Dy[nbEdges] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double dc[nbEdges] = {0.0, 0.0, 0.0, 0.0, 0.0};

  unsigned nbSteps = SupportStates_deq.back().StepNumber;
  Inequalities.resize(nbEdges*nbSteps,nbSteps, false);

  convex_hull_t FootFeasibilityEdges;

  deque<support_state_t>::const_iterator prwSS_it = SupportStates_deq.begin();
  prwSS_it++;//Point at the first previewed instant
  for( unsigned i=0; i<N_; i++ )
    {
      //foot positioning constraints
      if( prwSS_it->StateChanged && prwSS_it->StepNumber>0 && prwSS_it->Phase != DS)
        {
          prwSS_it--;//Take the support state before
          RFI_->set_vertices( FootFeasibilityEdges, *prwSS_it,
              INEQ_FEET );
          prwSS_it++;
          RFI_->compute_linear_system( FootFeasibilityEdges, Dx, Dy, dc, *prwSS_it );

          for( unsigned j = 0; j < nbEdges; j++ )
            {
              Inequalities.D.x.push_back((prwSS_it->StepNumber-1)*nbEdges+j, (prwSS_it->StepNumber-1), Dx[j]);
              Inequalities.D.y.push_back((prwSS_it->StepNumber-1)*nbEdges+j, (prwSS_it->StepNumber-1), Dy[j]);
              Inequalities.dc((prwSS_it->StepNumber-1)*nbEdges+j) = dc[j];
            }
        }

      prwSS_it++;
    }

}


void
GeneratorVelRef::build_constraints_cop(const linear_inequality_t & IneqCoP,
    unsigned int NbStepsPreviewed, QPProblem & Pb, option_e option)
{

		unsigned int NbConstraints = Pb.NbConstraints();

	  // -D*U
	  compute_term  ( MM_, -1.0, IneqCoP.D.x, Robot_->DynamicsCoPJerk().U   );
	  Pb.add_term_to( QPProblem::MATRIX_DU, MM_, NbConstraints, 0           );
	  compute_term  ( MM_, -1.0, IneqCoP.D.y, Robot_->DynamicsCoPJerk().U   );
	  Pb.add_term_to( QPProblem::MATRIX_DU, MM_, NbConstraints, N_          );

	  // +D*V
	  compute_term  ( MM_, 1.0, IneqCoP.D.x, IntermedData_->State().V 						);
	  // +  Robot_->LeftFoot().Dynamics(COP).U + Robot_->RightFoot().Dynamics(COP).U        );
	  Pb.add_term_to( QPProblem::MATRIX_DU, MM_, NbConstraints, 2*N_                        );
	  compute_term  ( MM_, 1.0, IneqCoP.D.y, IntermedData_->State().V  						);
	  // +  Robot_->LeftFoot().Dynamics(COP).U + Robot_->RightFoot().Dynamics(COP).U        );
	  Pb.add_term_to( QPProblem::MATRIX_DU, MM_, NbConstraints, 2*N_+NbStepsPreviewed       );

	  //constant part
	  // +dc
	  Pb.add_term_to( QPProblem::VECTOR_DS,IneqCoP.dc, NbConstraints );
	  if (option==WITH_TWO_CONTRAINT_BOUNDS){
		  Pb.add_term_to( QPProblem::VECTOR_DL,-IneqCoP.dc, NbConstraints );
	  }
	  // -D*S_z*x
	  compute_term  ( MV_, -1.0, IneqCoP.D.x, Robot_->DynamicsCoPJerk().S, IntermedData_->State().CoM.x          );
	  Pb.add_term_to( QPProblem::VECTOR_DS, MV_,  NbConstraints                                                  );
	  if (option==WITH_TWO_CONTRAINT_BOUNDS){
		  Pb.add_term_to( QPProblem::VECTOR_DL, MV_,  NbConstraints                                              );
	  }
	  /*
	   * Usefull for multibody dynamics
	   *
	  compute_term  ( MV_, -1.0, IneqCoP.D.x, Robot_->LeftFoot().Dynamics(COP).S, Robot_->LeftFoot().State().X   );
	  Pb.add_term_to( QPProblem::VECTOR_DS, MV_,  NbConstraints                                                  );
	  compute_term  ( MV_, -1.0, IneqCoP.D.x, Robot_->RightFoot().Dynamics(COP).S, Robot_->RightFoot().State().X );
	  Pb.add_term_to( QPProblem::VECTOR_DS, MV_,  NbConstraints                                                  );
	   */
	  compute_term  ( MV_, -1.0, IneqCoP.D.y,  Robot_->DynamicsCoPJerk().S, IntermedData_->State().CoM.y         );
	  Pb.add_term_to( QPProblem::VECTOR_DS, MV_,  NbConstraints                                                  );
	  if (option==WITH_TWO_CONTRAINT_BOUNDS){
		  Pb.add_term_to( QPProblem::VECTOR_DL, MV_,  NbConstraints                                              );
	  }
	  /*
	   * Usefull for multibody dynamics
	   *
	  compute_term  ( MV_, -1.0, IneqCoP.D.y, Robot_->LeftFoot().Dynamics(COP).S, Robot_->LeftFoot().State().Y   );
	  Pb.add_term_to( QPProblem::VECTOR_DS, MV_,  NbConstraints                                                  );
	  compute_term  ( MV_, -1.0, IneqCoP.D.y, Robot_->RightFoot().Dynamics(COP).S, Robot_->RightFoot().State().Y );
	  Pb.add_term_to( QPProblem::VECTOR_DS, MV_,  NbConstraints                                                  );
	   */

	  // +D*Vc*FP
	  compute_term  ( MV_, IntermedData_->State().SupportState.X, IneqCoP.D.x, IntermedData_->State().Vc    );
	  Pb.add_term_to( QPProblem::VECTOR_DS, MV_, NbConstraints                                              );
	  if (option==WITH_TWO_CONTRAINT_BOUNDS){
		  Pb.add_term_to( QPProblem::VECTOR_DL, MV_,  NbConstraints                                         );
	  }
	  compute_term  ( MV_, IntermedData_->State().SupportState.Y, IneqCoP.D.y, IntermedData_->State().Vc    );
	  Pb.add_term_to( QPProblem::VECTOR_DS, MV_, NbConstraints                                              );
	  if (option==WITH_TWO_CONTRAINT_BOUNDS){
		  Pb.add_term_to( QPProblem::VECTOR_DL, MV_,  NbConstraints                                         );
	  }

}


void
GeneratorVelRef::build_constraints_feet(const linear_inequality_t & IneqFeet,
    const IntermedQPMat::state_variant_t & State,
    int NbStepsPreviewed, QPProblem & Pb, option_e option)
{

  unsigned int NbConstraints = Pb.NbConstraints();

  // -D*V_f
  compute_term  ( MM_, -1.0, IneqFeet.D.x, State.V_f                              );
  Pb.add_term_to( QPProblem::MATRIX_DU, MM_, NbConstraints, 2*N_                  );
  compute_term  ( MM_, -1.0, IneqFeet.D.y, State.V_f                              );
  Pb.add_term_to( QPProblem::MATRIX_DU, MM_, NbConstraints, 2*N_+NbStepsPreviewed );

  // +dc
  Pb.add_term_to(  QPProblem::VECTOR_DS, IneqFeet.dc, NbConstraints );

  // D*Vc_f*FPc
  compute_term  ( MV_, State.SupportState.X, IneqFeet.D.x, State.Vc_f );
  Pb.add_term_to( QPProblem::VECTOR_DS, MV_, NbConstraints            );
  compute_term  ( MV_, State.SupportState.Y, IneqFeet.D.y, State.Vc_f );
  Pb.add_term_to( QPProblem::VECTOR_DS, MV_, NbConstraints            );

  if (option==WITH_TWO_CONTRAINT_BOUNDS){
	  unsigned int size=MV_.size();
	  MV2_.resize(size);
	  for(unsigned int i=0;i<size;++i){
		  MV2_(i)=-10e10;
	  }
	  Pb.add_term_to( QPProblem::VECTOR_DL, MV2_, NbConstraints       );
  }

}


void
GeneratorVelRef::build_eq_constraints_feet( const std::deque<support_state_t> & SupportStates_deq,
    unsigned int NbStepsPreviewed, QPProblem & Pb )
{

  if(SupportStates_deq.front().StateChanged)
    Robot_->SupportTrajectory().pop_front();
  std::deque<support_state_t>::const_iterator SPTraj_it = Robot_->SupportTrajectory().begin();

  boost_ublas::matrix<double> EqualityMatrix;
  boost_ublas::vector<double> EqualityVector;
  EqualityMatrix.resize(2,2*NbStepsPreviewed, false);
  EqualityMatrix.clear();
  EqualityVector.resize(2, false);
  EqualityVector.clear();
  Pb.NbEqConstraints(2*NbStepsPreviewed);
  for(unsigned int i = 0; i< NbStepsPreviewed; i++)
    {
      EqualityMatrix(0,i) = 1.0; EqualityVector(0) = -SPTraj_it->X;
      EqualityMatrix(1,NbStepsPreviewed+i) = 1.0; EqualityVector(1) = -SPTraj_it->Y;
      Pb.add_term_to( QPProblem::MATRIX_DU, EqualityMatrix, 2*i, 2*N_ );
      Pb.add_term_to( QPProblem::VECTOR_DS, EqualityVector, 2*i );
      EqualityMatrix.clear();
      EqualityVector.clear();
      SPTraj_it++;
    }

}


//void
//GeneratorVelRef::build_constraints_com( QPProblem & Pb, )


void
GeneratorVelRef::build_constraints( QPProblem & Pb, const solution_t & Solution, option_e option )
{

  unsigned nbStepsPreviewed = Solution.SupportStates_deq.back().StepNumber;
  //Equality constraints
  //  build_eq_constraints_feet( PrwSupportStates_deq, NbStepsPreviewed, Pb );

  //CoP constraints
  linear_inequality_t & IneqCoP = IntermedData_->Inequalities( INEQ_COP );
  build_inequalities_cop( IneqCoP, Solution.SupportStates_deq, option );
  build_constraints_cop( IneqCoP, nbStepsPreviewed, Pb, option );

  //Foot inequality constraints
  linear_inequality_t & IneqFeet = IntermedData_->Inequalities( INEQ_FEET );
  build_inequalities_feet( IneqFeet, Solution.SupportStates_deq );
  build_constraints_feet( IneqFeet, IntermedData_->State(), nbStepsPreviewed, Pb, option );


}


void 
GeneratorVelRef::build_invariant_part( QPProblem & Pb )
{

  const RigidBody & CoM = Robot_->CoM();

  //Constant terms in the Hessian
  // +a*U'*U
  const IntermedQPMat::objective_variant_t & Jerk = IntermedData_->Objective( JERK_MIN );
  const linear_dynamics_t & JerkDynamics = CoM.Dynamics( JERK );
  compute_term  ( MM_, Jerk.weight, JerkDynamics.UT, JerkDynamics.U     );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, 0, 0                        );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, N_, N_                      );

  // +a*U'*U
  const IntermedQPMat::objective_variant_t & InstVel = IntermedData_->Objective( INSTANT_VELOCITY );
  const linear_dynamics_t & VelDynamics = CoM.Dynamics( VELOCITY );
  compute_term  ( MM_, InstVel.weight, VelDynamics.UT, VelDynamics.U    );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, 0, 0                        );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, N_, N_                      );

  // +a*U'*U
  const IntermedQPMat::objective_variant_t & COPCent = IntermedData_->Objective( COP_CENTERING );
  compute_term  ( MM_, COPCent.weight, Robot_->DynamicsCoPJerk().UT, Robot_->DynamicsCoPJerk().U   );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, 0, 0                                                   );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, N_, N_                                                 );

}


void
GeneratorVelRef::update_problem( QPProblem & Pb, const std::deque<support_state_t> & SupportStates_deq )
{

  Pb.clear(QPProblem::VECTOR_D);

  const int NbStepsPreviewed = SupportStates_deq[N_].StepNumber;
  const IntermedQPMat::state_variant_t & State = IntermedData_->State();
  const RigidBody & CoM = Robot_->CoM();

  // Instant velocity terms
  const IntermedQPMat::objective_variant_t & InstVel = IntermedData_->Objective( INSTANT_VELOCITY );
  const linear_dynamics_t & VelDynamics = CoM.Dynamics( VELOCITY );
  // Linear part
  // +a*U'*S*x
  compute_term  ( MV_, InstVel.weight , VelDynamics.UT, VelDynamics.S, State.CoM.x      );
  Pb.add_term_to( QPProblem::VECTOR_D, MV_, 0                                           );
  compute_term  ( MV_ , InstVel.weight, VelDynamics.UT, VelDynamics.S, State.CoM.y      );
  Pb.add_term_to( QPProblem::VECTOR_D, MV_, N_                                          );
  // +a*U'*ref
  compute_term  ( MV_, -InstVel.weight, VelDynamics.UT, State.Ref.Global.X_vec  );
  Pb.add_term_to( QPProblem::VECTOR_D, MV_, 0                                   );
  compute_term  ( MV_, -InstVel.weight, VelDynamics.UT, State.Ref.Global.Y_vec  );
  Pb.add_term_to( QPProblem::VECTOR_D, MV_, N_                                  );

  // COP - centering terms
  const IntermedQPMat::objective_variant_t & COPCent = IntermedData_->Objective( COP_CENTERING );
  const linear_dynamics_t & CoPDynamics = Robot_->DynamicsCoPJerk( );
  //  const linear_dynamics_t & LFCoP = Robot_->LeftFoot().Dynamics(COP);
  //  const linear_dynamics_t & RFCoP = Robot_->RightFoot().Dynamics(COP);
  // Hessian
  // -a*U'*V
  compute_term  ( MM_, -COPCent.weight, CoPDynamics.UT, State.V         );
  Pb.add_term_to(  QPProblem::MATRIX_Q, MM_, 0, 2*N_                    );
  Pb.add_term_to(  QPProblem::MATRIX_Q, MM_, N_, 2*N_+NbStepsPreviewed  );

  // -a*V*U
  compute_term  ( MM_, -COPCent.weight, State.VT, CoPDynamics.U                           );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, 2*N_, 0                                       );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, 2*N_+NbStepsPreviewed, N_                     );
  //+a*V'*V
  compute_term  ( MM_, COPCent.weight, State.VT, State.V                                  );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, 2*N_, 2*N_                                    );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, 2*N_+NbStepsPreviewed, 2*N_+NbStepsPreviewed  );

  //Linear part
  // -a*V'*S*x
  compute_term  ( MV_, -COPCent.weight, State.VT, CoPDynamics.S, State.CoM.x    );
  Pb.add_term_to( QPProblem::VECTOR_D, MV_, 2*N_                                );
  compute_term  ( MV_, -COPCent.weight, State.VT, CoPDynamics.S, State.CoM.y    );
  Pb.add_term_to( QPProblem::VECTOR_D, MV_, 2*N_+NbStepsPreviewed               );
  // +a*V'*Vc*x
  compute_term  ( MV_, COPCent.weight, State.VT, State.Vc, State.SupportState.X  );
  Pb.add_term_to( QPProblem::VECTOR_D, MV_, 2*N_                                 );
  compute_term  ( MV_, COPCent.weight, State.VT, State.Vc, State.SupportState.Y  );
  Pb.add_term_to( QPProblem::VECTOR_D, MV_, 2*N_+NbStepsPreviewed                );

}


void
GeneratorVelRef::compute_warm_start( solution_t & Solution )
{

  // Initialize:
  // -----------
  unsigned int nbSteps = Solution.SupportStates_deq.back().StepNumber;
  unsigned int nbStepsMax = 4;

  // Copy current support
  support_state_t currentSupport = Solution.SupportStates_deq.front();

  // ZMP position vector
  boost_ublas::vector<double> zx(N_);
  boost_ublas::vector<double> zy(N_);

  Solution.initialSolution.resize(4*N_+2*nbSteps);

  // Compute previewed initial constraints:
  // ---------------------
  unsigned int size=Solution.initialConstraint.size();
  boost_ublas::vector<int> initialConstraintTmp;
  initialConstraintTmp = Solution.initialConstraint;
  if (size>=2*N_){
	  for(unsigned int i=0;i<2*(N_-1);++i){
		  Solution.initialConstraint(i)=initialConstraintTmp(i+2);
	  }
	  for(unsigned int i=2*(N_-1);i<2*N_;++i){
		  Solution.initialConstraint(i)=initialConstraintTmp(i);
	  }
	  for(unsigned int i=2*N_;i<2*N_+5*nbSteps;++i){
		  Solution.initialConstraint(i)=initialConstraintTmp(i);
	  }
	  for(unsigned int i=2*N_+5*nbSteps; i<2*N_+5*nbStepsMax; ++i){
		  Solution.initialConstraint(i)=0;
	  }
  }else{
	  Solution.initialConstraint.resize(2*N_+5*nbStepsMax);
	  for(unsigned int i=0;i<2*N_+5*nbStepsMax;++i){
		  Solution.initialConstraint(i)=0;
	  }
  }

  // Compute feasible initial ZMP and foot positions:
  // ---------------------------------------
  deque<support_state_t>::iterator prwSS_it = Solution.SupportStates_deq.begin();

  prwSS_it++;//Point at the first previewed support state

  unsigned int j = 0;
  convex_hull_t FootFeasibilityEdges, COPFeasibilityEdges;
  double shiftx,shifty;
  bool noActiveConstraints;

  for(unsigned int i=0; i<N_; i++)
    {
	  // Get COP convex hull for current support
	  RFI_->set_vertices( COPFeasibilityEdges, *prwSS_it, INEQ_COP );
      // Check if the support foot has changed
      if (currentSupport.Foot != prwSS_it->Foot)
        {
          currentSupport.Foot = prwSS_it->Foot;

          // Get feet convex hull for current support
          prwSS_it--;
          RFI_->set_vertices( FootFeasibilityEdges, *prwSS_it, INEQ_FEET );
          prwSS_it++;

          // Place the foot on active constraints
          shiftx=shifty=0;
          noActiveConstraints=true;
          for(unsigned int k=0;k<5;++k){
        	  if (Solution.initialConstraint(k+32+j*5)!=0){
        		  int k2=(k+1)%5; // k(4) = k(0)
        		  if (Solution.initialConstraint(k2+32+j*5)!=0){
					  shiftx=FootFeasibilityEdges.X[k2];
					  shifty=FootFeasibilityEdges.Y[k2];
				  }else{
					  shiftx=(FootFeasibilityEdges.X[k]+FootFeasibilityEdges.X[k2])/2;
					  shifty=(FootFeasibilityEdges.Y[k]+FootFeasibilityEdges.Y[k2])/2;
				  }
        		  noActiveConstraints=false;
        		  break;
        	  }
          }
          if (noActiveConstraints){
			  shiftx=(FootFeasibilityEdges.X[4]+FootFeasibilityEdges.X[0])/2;
			  shifty=(FootFeasibilityEdges.Y[4]+FootFeasibilityEdges.Y[2])/2;
          }

          currentSupport.X += shiftx;
          currentSupport.Y += shifty;

          // Set the new position into initial solution vector
          Solution.initialSolution(2*N_+j) = currentSupport.X;
          Solution.initialSolution(2*N_+nbSteps+j) = currentSupport.Y;
          ++j;
        }
      // Place the ZMP on active constraints
      shiftx=shifty=0;
      noActiveConstraints=true;
      int k1=-1;
      int k2=-1;
      if (Solution.initialConstraint(0+i*2)==1){
    	  if (Solution.initialConstraint(1+i*2)==1){
    		  k2=1;
    		  noActiveConstraints=false;
    	  }else if (Solution.initialConstraint(1+i*2)==2){
    		  k2=0;
    		  noActiveConstraints=false;
    	  }else if (Solution.initialConstraint(1+i*2)==0){
    		  k1=0;
    		  k2=1;
    		  noActiveConstraints=false;
    	  }
      }else if (Solution.initialConstraint(0+i*2)==2){
    	  if (Solution.initialConstraint(1+i*2)==1){
    		  k2=2;
    		  noActiveConstraints=false;
		  }else if (Solution.initialConstraint(1+i*2)==2){
			  k2=3;
			  noActiveConstraints=false;
		  }else if (Solution.initialConstraint(1+i*2)==0){
			  k1=3;
			  k2=2;
			  noActiveConstraints=false;
		  }
      }else if (Solution.initialConstraint(1+i*2)==1){
    	  k1=2;
    	  k2=1;
    	  noActiveConstraints=false;
      }else if (Solution.initialConstraint(1+i*2)==2){
    	  k1=0;
    	  k2=3;
    	  noActiveConstraints=false;
      }

      if (!noActiveConstraints){
    	  if (k1!=-1){
    		  shiftx=(COPFeasibilityEdges.X[k1]+COPFeasibilityEdges.X[k2])/2;
    		  shifty=(COPFeasibilityEdges.Y[k1]+COPFeasibilityEdges.Y[k2])/2;
    	  }else{
    		  shiftx=COPFeasibilityEdges.X[k2];
    		  shifty=COPFeasibilityEdges.Y[k2];
    	  }
      }


      zx(i) = currentSupport.X+shiftx;
      zy(i) = currentSupport.Y+shifty;
      ++prwSS_it;
    }

  // Compute initial jerk:
  // ---------------------
  boost_ublas::vector<double> X(N_);
  boost_ublas::vector<double> Y(N_);
  MV2_= prod( Robot_->DynamicsCoPJerk().S, IntermedData_->State().CoM.x  );
  X=    prod( Robot_->DynamicsCoPJerk().Um1, zx-MV2_                     );
  MV2_= prod( Robot_->DynamicsCoPJerk().S, IntermedData_->State().CoM.y  );
  Y=    prod( Robot_->DynamicsCoPJerk().Um1, zy-MV2_                     );

  for(unsigned int i=0;i<N_;i++)
    {
      Solution.initialSolution(i)=X(i);
      Solution.initialSolution(N_+i)=Y(i);
    }

}



void
GeneratorVelRef::compute_term(MAL_MATRIX (&weightMM, double), double weight,
    const MAL_MATRIX (&M1, double), const MAL_MATRIX (&M2, double))
{
  weightMM = MAL_RET_A_by_B(M1,M2);
  weightMM *= weight;
}


void
GeneratorVelRef::compute_term(MAL_VECTOR (&weightMV, double), double weight,
    const MAL_MATRIX (&M, double), const MAL_VECTOR (&V, double))
{
  weightMV = MAL_RET_A_by_B(M,V);
  weightMV *= weight;
}


void
GeneratorVelRef::compute_term(MAL_VECTOR (&weightMV, double),
    double weight, const MAL_MATRIX (&M, double),
    const MAL_VECTOR (&V, double), double scalar)
{
  weightMV = MAL_RET_A_by_B(M,V);
  weightMV *= weight*scalar;
}


void
GeneratorVelRef::compute_term(MAL_VECTOR (&weightMV, double),
    double weight, const MAL_MATRIX (&M1, double),
    const MAL_MATRIX (&M2, double), const MAL_VECTOR (&V2, double))
{
  MV2_ = MAL_RET_A_by_B(M2,V2);
  weightMV = MAL_RET_A_by_B(M1,MV2_);
  weightMV *= weight;
}



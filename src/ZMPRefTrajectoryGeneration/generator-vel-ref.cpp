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
    IntermedQPMat * Data, RigidBodySystem * Robot )
: MPCTrajectoryGeneration(lSPM)
, IntermedData_ (Data)
, Robot_(Robot)
, MV_(1,false)
, MM_(1,1,false)
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
GeneratorVelRef::Ponderation( double Weight, int Type)
{

  IntermedQPMat::objective_variant_t & Objective = IntermedData_->Objective( Type );
  Objective.weight = Weight;

}	


void
GeneratorVelRef::preview_support_states( double Time, const SupportFSM * FSM,
    const deque<FootAbsolutePosition> & FinalLeftFootTraj_deq, const deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
    deque<support_state_t> & SupportStates_deq )
{

  // DETERMINE CURRENT SUPPORT STATE:
  // --------------------------------
  const reference_t & RefVel = IntermedData_->Reference();
  support_state_t & CurrentSupport = IntermedData_->SupportState();
  FSM->set_support_state(CurrentTime_, 0, CurrentSupport, RefVel);
  if(CurrentSupport.StateChanged == true)
    {
      FootAbsolutePosition FAP;
      if(CurrentSupport.Foot == LEFT)
        FAP = FinalLeftFootTraj_deq.back();
      else
        FAP = FinalRightFootTraj_deq.back();
      CurrentSupport.X = FAP.x;
      CurrentSupport.Y = FAP.y;
      CurrentSupport.Yaw = FAP.theta*M_PI/180.0;
      CurrentSupport.StartTime = Time;
    }
  SupportStates_deq.push_back( CurrentSupport );
  IntermedData_->SupportState( CurrentSupport );


  // PREVIEW SUPPORT STATES:
  // -----------------------
  // initialize the previewed support state before previewing
  support_state_t PreviewedSupport = CurrentSupport;
  PreviewedSupport.StepNumber  = 0;
  for(unsigned int i=1;i<=N_;i++)
    {
      FSM->set_support_state( CurrentTime_, i, PreviewedSupport, RefVel );
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
GeneratorVelRef::initialize_matrices()
{

  linear_inequality_t & IneqCoP = IntermedData_->Inequalities( IntermedQPMat::INEQ_COP );
  initialize_matrices( IneqCoP );

}


void
GeneratorVelRef::initialize_matrices( linear_inequality_t & Inequalities)
{

  switch(Inequalities.type)
  {
  case IntermedQPMat::INEQ_COP:
    Inequalities.D.x.resize(4*N_,N_,false);
    Inequalities.D.x.clear();
    Inequalities.D.y.resize(4*N_,N_,false);
    Inequalities.D.y.clear();
    Inequalities.dc.resize(4*N_,false);
    Inequalities.dc.clear();
    break;
  }

}


void 
GeneratorVelRef::build_inequalities_cop(linear_inequality_t & Inequalities,
    RelativeFeetInequalities * RFI,
    const std::deque< FootAbsolutePosition> & LeftFootPositions_deq,
    const std::deque<FootAbsolutePosition> & RightFootPositions_deq,
    const std::deque<support_state_t> & SupportStates_deq,
    const std::deque<double> & PreviewedSupportAngles_deq) const
{

  const support_state_t & CurrentSupport = SupportStates_deq.front();
  double CurrentSupportAngle;
  if( CurrentSupport.Foot == LEFT )
    CurrentSupportAngle = LeftFootPositions_deq.back().theta*M_PI/180.0;
  else
    CurrentSupportAngle = RightFootPositions_deq.back().theta*M_PI/180.0;
  convex_hull_t ZMPFeasibilityEdges;
  RFI->set_vertices( ZMPFeasibilityEdges,
      CurrentSupportAngle,
      CurrentSupport,
      RelativeFeetInequalities::ZMP_CONSTRAINTS );

  //set constraints for the whole preview window
  double SupportAngle = CurrentSupportAngle;
  const int NbEdges = 4;
  double D_x[NbEdges] = {0.0, 0.0, 0.0, 0.0};
  double D_y[NbEdges] = {0.0, 0.0, 0.0, 0.0};
  double dc[NbEdges] = {0.0, 0.0, 0.0, 0.0};
  for( unsigned int i=1;i<=N_;i++ )// Only the previewed support states
    {
      const support_state_t & PrwSupport = SupportStates_deq[i];

      if( PrwSupport.StateChanged && PrwSupport.StepNumber>0 )
        SupportAngle = PreviewedSupportAngles_deq[PrwSupport.StepNumber-1];

      if( PrwSupport.StateChanged )
        RFI->set_vertices( ZMPFeasibilityEdges,
            SupportAngle,
            PrwSupport,
            RelativeFeetInequalities::ZMP_CONSTRAINTS );

      RFI->compute_linear_system( ZMPFeasibilityEdges, D_x, D_y, dc, PrwSupport );

      for(int j = 0;j < NbEdges; j++)
        {
          Inequalities.D.x.push_back((i-1)*NbEdges+j,i-1,D_x[j]);
          Inequalities.D.y.push_back((i-1)*NbEdges+j,i-1,D_y[j]);
          Inequalities.dc((i-1)*NbEdges+j) = dc[j];
        }
    }

}


void
GeneratorVelRef::build_inequalities_feet( linear_inequality_t & Inequalities,
    RelativeFeetInequalities * RFI,
    const std::deque< FootAbsolutePosition> & LeftFootPositions_deq,
    const std::deque<FootAbsolutePosition> & RightFootPositions_deq,
    const std::deque<support_state_t> & SupportStates_deq,
    const std::deque<double> & PreviewedSupportAngles_deq ) const
{

  // Initialize support angle
  const support_state_t & CurrentSupport = SupportStates_deq.front();
  double CurrentSupportAngle;
  if( CurrentSupport.Foot == LEFT )
    CurrentSupportAngle = LeftFootPositions_deq.back().theta*M_PI/180.0;
  else
    CurrentSupportAngle = RightFootPositions_deq.back().theta*M_PI/180.0;
  double SupportAngle = CurrentSupportAngle;

  // Arrays for the generated set of inequalities
  const int NbEdges = 5;
  double Dx[NbEdges] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double Dy[NbEdges] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double dc[NbEdges] = {0.0, 0.0, 0.0, 0.0, 0.0};

  int NbStepsPreviewed = SupportStates_deq.back().StepNumber;
  Inequalities.resize(NbEdges*NbStepsPreviewed,NbStepsPreviewed, false);

  convex_hull_t FootFeasibilityEdges;

  unsigned int StepNumber = 0;
  for(unsigned int i=1;i<=N_;i++ )
    {

      const support_state_t & PrwSupport = SupportStates_deq[i];

      //foot positioning constraints
      if( PrwSupport.StateChanged && PrwSupport.StepNumber>0 && PrwSupport.Phase != DS)
        {
          //SupportAngle = PreviewedSupportAngles_deq[PrwSupport.StepNumber-1];
          if( PrwSupport.StepNumber == 1 )
            SupportAngle = CurrentSupportAngle;
          else
            SupportAngle = PreviewedSupportAngles_deq[PrwSupport.StepNumber-2];

          RFI->set_vertices( FootFeasibilityEdges, SupportAngle, PrwSupport,
              RelativeFeetInequalities::FOOT_CONSTRAINTS );

          RFI->compute_linear_system( FootFeasibilityEdges, Dx, Dy, dc, PrwSupport );

          for(int j = 0;j < NbEdges; j++)
            {
              Inequalities.D.x.push_back((PrwSupport.StepNumber-1)*NbEdges+j, (PrwSupport.StepNumber-1), Dx[j]);
              Inequalities.D.y.push_back((PrwSupport.StepNumber-1)*NbEdges+j, (PrwSupport.StepNumber-1), Dy[j]);
              Inequalities.dc((PrwSupport.StepNumber-1)*NbEdges+j) = dc[j];
            }

          StepNumber++;
        }
    }

}


void
GeneratorVelRef::build_constraints_cop(const linear_inequality_t & IneqCoP,
    unsigned int NbStepsPreviewed, QPProblem & Pb)
{

  unsigned int NbConstraints = Pb.NbConstraints();

  // -D*U
  compute_term  ( MM_, -1.0, IneqCoP.D.x, Robot_->DynamicsCoPJerk().U   );
  Pb.add_term_to( QPProblem::MATRIX_DU, MM_, NbConstraints, 0           );
  compute_term  ( MM_, -1.0, IneqCoP.D.y, Robot_->DynamicsCoPJerk().U   );
  Pb.add_term_to( QPProblem::MATRIX_DU, MM_, NbConstraints, N_          );

  // +D*V
  compute_term  ( MM_, 1.0, IneqCoP.D.x, IntermedData_->State().V 						);
  // +  Robot_->LeftFoot().Dynamics(COP).U + Robot_->RightFoot().Dynamics(COP).U          );
  Pb.add_term_to( QPProblem::MATRIX_DU, MM_, NbConstraints, 2*N_                        );
  compute_term  ( MM_, 1.0, IneqCoP.D.y, IntermedData_->State().V  						);
  // +  Robot_->LeftFoot().Dynamics(COP).U + Robot_->RightFoot().Dynamics(COP).U          );
  Pb.add_term_to( QPProblem::MATRIX_DU, MM_, NbConstraints, 2*N_+NbStepsPreviewed       );

  //constant part
  // +dc
  Pb.add_term_to( QPProblem::VECTOR_DS,IneqCoP.dc, NbConstraints );

  // -D*S_z*x
  compute_term  ( MV_, -1.0, IneqCoP.D.x, Robot_->DynamicsCoPJerk().S, IntermedData_->State().CoM.x          );
  Pb.add_term_to( QPProblem::VECTOR_DS, MV_,  NbConstraints                                                  );
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
  compute_term  ( MV_, IntermedData_->State().SupportState.Y, IneqCoP.D.y, IntermedData_->State().Vc    );
  Pb.add_term_to( QPProblem::VECTOR_DS, MV_, NbConstraints                                              );


}


void
GeneratorVelRef::build_constraints_feet(const linear_inequality_t & IneqFeet,
    const IntermedQPMat::state_variant_t & State,
    int NbStepsPreviewed, QPProblem & Pb)
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


void
GeneratorVelRef::build_constraints( QPProblem & Pb, RelativeFeetInequalities * RFI,
    const std::deque< FootAbsolutePosition> & LeftFootPositions_deq,
    const std::deque<FootAbsolutePosition> & RightFootPositions_deq,
    const std::deque<support_state_t> & PrwSupportStates_deq,
    const std::deque<double> & PrwSupportAngles_deq )
{

  unsigned int NbStepsPreviewed = PrwSupportStates_deq.back().StepNumber;
  //Equality constraints
  //  build_eq_constraints_feet( PrwSupportStates_deq, NbStepsPreviewed, Pb );

  //CoP constraints
  linear_inequality_t & IneqCoP = IntermedData_->Inequalities(IntermedQPMat::INEQ_COP);
  build_inequalities_cop( IneqCoP, RFI,
      LeftFootPositions_deq, RightFootPositions_deq,
      PrwSupportStates_deq, PrwSupportAngles_deq );
  build_constraints_cop( IneqCoP, NbStepsPreviewed, Pb );

  //Foot inequality constraints
  linear_inequality_t & IneqFeet = IntermedData_->Inequalities(IntermedQPMat::INEQ_FEET);
  build_inequalities_feet( IneqFeet, RFI,
      LeftFootPositions_deq, RightFootPositions_deq,
      PrwSupportStates_deq, PrwSupportAngles_deq );
  build_constraints_feet( IneqFeet, IntermedData_->State(), NbStepsPreviewed, Pb );

}


void 
GeneratorVelRef::build_invariant_part( QPProblem & Pb )
{

  const RigidBody & CoM = Robot_->CoM();

  //Constant terms in the Hessian
  // +a*U'*U
  const IntermedQPMat::objective_variant_t & Jerk = IntermedData_->Objective(IntermedQPMat::JERK_MIN);
  const linear_dynamics_t & JerkDynamics = CoM.Dynamics( JERK );
  compute_term( MM_, Jerk.weight, JerkDynamics.UT, JerkDynamics.U );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, 0, 0              );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, N_, N_            );

  // +a*U'*U
  const IntermedQPMat::objective_variant_t & InstVel = IntermedData_->Objective(IntermedQPMat::INSTANT_VELOCITY);
  const linear_dynamics_t & VelDynamics = CoM.Dynamics( VELOCITY );
  compute_term  ( MM_, InstVel.weight, VelDynamics.UT, VelDynamics.U    );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, 0, 0                        );
  Pb.add_term_to( QPProblem::MATRIX_Q, MM_, N_, N_                      );

  // +a*U'*U
  const IntermedQPMat::objective_variant_t & COPCent = IntermedData_->Objective(IntermedQPMat::COP_CENTERING);
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
  const IntermedQPMat::objective_variant_t & InstVel = IntermedData_->Objective(IntermedQPMat::INSTANT_VELOCITY);
  const linear_dynamics_t & VelDynamics = CoM.Dynamics( VELOCITY );
  // Linear part
  // +a*U'*S*x
  compute_term  ( MV_, InstVel.weight , VelDynamics.UT, VelDynamics.S, State.CoM.x      );
  Pb.add_term_to( QPProblem::VECTOR_D, MV_, 0                                           );
  compute_term  ( MV_ , InstVel.weight, VelDynamics.UT, VelDynamics.S, State.CoM.y      );
  Pb.add_term_to( QPProblem::VECTOR_D, MV_, N_                                          );
  // +a*U'*ref
  compute_term  ( MV_, -InstVel.weight, VelDynamics.UT, State.Ref.Global.X_vec      );
  Pb.add_term_to( QPProblem::VECTOR_D, MV_, 0                                   );
  compute_term  ( MV_, -InstVel.weight, VelDynamics.UT, State.Ref.Global.Y_vec      );
  Pb.add_term_to( QPProblem::VECTOR_D, MV_, N_                                  );

  // COP - centering terms
  const IntermedQPMat::objective_variant_t & COPCent = IntermedData_->Objective(IntermedQPMat::COP_CENTERING);
  const linear_dynamics_t & CoPDynamics = Robot_->DynamicsCoPJerk( );
  //  const linear_dynamics_t & LFCoP = Robot_->LeftFoot().Dynamics(COP);
  //  const linear_dynamics_t & RFCoP = Robot_->RightFoot().Dynamics(COP);
  // Hessian
  // -a*U'*V
  compute_term  ( MM_    ,   -COPCent.weight    ,   CoPDynamics.UT    ,   State.V            );
  Pb.add_term_to(  QPProblem::MATRIX_Q, MM_, 0, 2*N_                                         );
  Pb.add_term_to(  QPProblem::MATRIX_Q, MM_, N_, 2*N_+NbStepsPreviewed                       );

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
GeneratorVelRef::compute_warm_start( solution_t & Solution , RelativeFeetInequalities * RFI )
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

  Solution.initialSolution.resize(2*N_+2*nbSteps);

  // Compute initial constraints:
  // ---------------------
  unsigned int size=Solution.initialConstraint.size();
  boost_ublas::vector<int> initialConstraintTmp;
  initialConstraintTmp = Solution.initialConstraint;
  if (size>=4*N_){
	  for(unsigned int i=0;i<4*(N_-1);++i){
		  Solution.initialConstraint(i)=initialConstraintTmp(i+4);
	  }
	  for(unsigned int i=4*(N_-1);i<4*N_;++i){
		  Solution.initialConstraint(i)=initialConstraintTmp(i);
	  }
	  for(unsigned int i=4*N_;i<4*N_+5*nbSteps;++i){
		  Solution.initialConstraint(i)=initialConstraintTmp(i);
	  }
	  for(unsigned int i=4*N_+5*nbSteps; i<4*N_+5*nbStepsMax; ++i){
		  Solution.initialConstraint(i)=0;
	  }
  }else{
	  Solution.initialConstraint.resize(4*N_+5*nbStepsMax);

	  for(unsigned int i=0;i<4*N_+5*nbStepsMax;++i){
		  Solution.initialConstraint(i)=0;
	  }
  }

  // Compute initial ZMP and foot positions:
  // ---------------------------------------
  deque<support_state_t>::iterator prwSS_it = Solution.SupportStates_deq.begin();
  deque<double>::iterator prwOr_it = Solution.SupportOrientations_deq.begin();

  prwSS_it++;//Point at the first previewed support state
  unsigned int j = 0;
  convex_hull_t FootFeasibilityEdges, COPFeasibilityEdges;
  double shiftx,shifty;
  bool noActiveConstraints;
  for(unsigned int i=0; i<N_; i++)
    {
	  RFI->set_vertices( COPFeasibilityEdges, currentSupport.Yaw, *prwSS_it, RelativeFeetInequalities::ZMP_CONSTRAINTS );
      // Check if the support foot has changed
      if (currentSupport.Foot != prwSS_it->Foot)
        {
          currentSupport.Foot = prwSS_it->Foot;
          // Place the foot on actives contraints
          shiftx=shifty=0;
          noActiveConstraints=true;
          RFI->set_vertices( FootFeasibilityEdges, currentSupport.Yaw, *prwSS_it, RelativeFeetInequalities::FOOT_CONSTRAINTS );
          for(unsigned int k=0;k<5;++k){
        	  if (Solution.initialConstraint(k+64+j*5)!=0){
        		  if (k==4){
        			  if (Solution.initialConstraint(0+64+j*5)!=0){
        				  shiftx=FootFeasibilityEdges.X[0];
        				  shifty=FootFeasibilityEdges.Y[0];
        			  }else{
        				  shiftx=(FootFeasibilityEdges.X[4]+FootFeasibilityEdges.X[0])/2;
        				  shifty=(FootFeasibilityEdges.Y[4]+FootFeasibilityEdges.Y[0])/2;
        			  }
        		  }else{
        			  if (Solution.initialConstraint(k+1+64+j*5)!=0){
        				  shiftx=FootFeasibilityEdges.X[k+1];
        				  shifty=FootFeasibilityEdges.Y[k+1];
        			  }else{
        				  shiftx=(FootFeasibilityEdges.X[k]+FootFeasibilityEdges.X[k+1])/2;
        				  shifty=(FootFeasibilityEdges.Y[k]+FootFeasibilityEdges.Y[k+1])/2;
        			  }
        		  }

        		  noActiveConstraints=false;
        		  break;
        	  }
          }
          if (noActiveConstraints){
			  shiftx=(FootFeasibilityEdges.X[4]+FootFeasibilityEdges.X[0])/2;
			  shifty=(FootFeasibilityEdges.Y[4]+FootFeasibilityEdges.Y[0])/2;
          }

          currentSupport.X += shiftx;
          currentSupport.Y += shifty;

          // Set the new position into initial solution vector
          Solution.initialSolution(2*N_+j) = currentSupport.X;
          Solution.initialSolution(2*N_+nbSteps+j) = currentSupport.Y;
          currentSupport.Yaw = *prwOr_it;
          ++prwOr_it;
          ++j;
        }
      // Set the ZMP at the midle of active constraints
      shiftx=shifty=0;
      for(unsigned int k=0;k<4;++k){
    	  if (Solution.initialConstraint(k+i*4)==1){
    		  if (k==3){
    			  if (Solution.initialConstraint(0+i*4)==1){
    				  shiftx=COPFeasibilityEdges.X[0];
    				  shifty=COPFeasibilityEdges.Y[0];
    			  }else{
    				  shiftx=(COPFeasibilityEdges.X[3]+COPFeasibilityEdges.X[0])/2;
    				  shifty=(COPFeasibilityEdges.Y[3]+COPFeasibilityEdges.Y[0])/2;
    			  }
    		  }else{
    			  if (Solution.initialConstraint(k+1+i*4)==1){
    				  shiftx=COPFeasibilityEdges.X[k+1];
    				  shifty=COPFeasibilityEdges.Y[k+1];
    			  }else{
    				  shiftx=(COPFeasibilityEdges.X[k]+COPFeasibilityEdges.X[k+1])/2;
    				  shifty=(COPFeasibilityEdges.Y[k]+COPFeasibilityEdges.Y[k+1])/2;
    			  }
    		  }
    		  break;
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



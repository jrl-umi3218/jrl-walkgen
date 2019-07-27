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


GeneratorVelRef::GeneratorVelRef
(SimplePluginManager *lSPM,
 IntermedQPMat * Data, RigidBodySystem * Robot,
 RelativeFeetInequalities * RFI )
  : MPCTrajectoryGeneration(lSPM)
  , IntermedData_ (Data)
  , Robot_(Robot)
  , RFI_(RFI)
  , LastFootSolX_(0.0)
  , LastFootSolY_(0.0)
  , MM_(1,1)
  , MV_(1)
  , MV2_(1)
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

  IntermedQPMat::objective_variant_t & Objective =
    IntermedData_->Objective(type );
  Objective.weight = weight;

}


void
GeneratorVelRef::preview_support_states
( double time, const SupportFSM * FSM,
  const deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
  const deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
  deque<support_state_t> & SupportStates_deq )
{

  const FootAbsolutePosition * FAP = NULL;

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
  for( unsigned pi=1; pi<=N_; pi++ )
    {
      FSM->set_support_state( CurrentTime_, pi, PreviewedSupport, RefVel );
      if( PreviewedSupport.StateChanged )
        {
          if( pi == 1  )//Foot down
            {
              if( PreviewedSupport.Foot == LEFT )
                FAP = & FinalLeftFootTraj_deq.back();
              else
                FAP = & FinalRightFootTraj_deq.back();
              PreviewedSupport.X = FAP->x;
              PreviewedSupport.Y = FAP->y;
              PreviewedSupport.Yaw = FAP->theta*M_PI/180.0;
              PreviewedSupport.StartTime = time+pi*Tprw_;
            }
          if( /*pi > 1 &&*/ PreviewedSupport.StepNumber > 0 )
            {
              PreviewedSupport.X = 0.0;
              PreviewedSupport.Y = 0.0;
            }
        }
      SupportStates_deq.push_back( PreviewedSupport );
    }


  // GENERATE SUPPORT SELECTION MATRICES:
  // ------------------------------------
  generate_selection_matrices( SupportStates_deq );

}


void
GeneratorVelRef::generate_selection_matrices
( const std::deque<support_state_t>
  & SupportStates_deq )
{

  IntermedQPMat::state_variant_t & State = IntermedData_->State();
  const unsigned & NbPrwSteps = SupportStates_deq.back().StepNumber;

  State.VcX.setZero();
  State.VcY.setZero();
  State.V.resize(N_,NbPrwSteps);
  State.V.setZero();
  State.VT.resize(NbPrwSteps,N_);
  State.VT.setZero();
  State.Vc_fX.resize(NbPrwSteps);
  State.Vc_fX.setZero();
  State.Vc_fY.resize(NbPrwSteps);
  State.Vc_fY.setZero();
  State.V_f.resize(NbPrwSteps,NbPrwSteps);
  State.V_f.setZero();


  std::deque<support_state_t>::const_iterator SS_it;
  SS_it = SupportStates_deq.begin();//points at the cur. sup. st.
  ++SS_it;
  for(unsigned i=0; i<N_; i++)
    {
      if(SS_it->StepNumber>0)
        {
          State.V(i,SS_it->StepNumber-1) =
            State.VT(SS_it->StepNumber-1,i) = 1.0;
          if( SS_it->StepNumber==1 &&
              SS_it->StateChanged && SS_it->Phase == SS )
            {
              --SS_it;
              State.Vc_fX(0) = SS_it->X;
              State.Vc_fY(0) = SS_it->Y;
              ++SS_it;

              State.V_f(0,0) = 1.0;
            }
          else if(SS_it->StepNumber>1)
            {
              State.V_f(SS_it->StepNumber-1,SS_it->StepNumber-2) = -1.0;
              State.V_f(SS_it->StepNumber-1,SS_it->StepNumber-1) = 1.0;
            }
        }
      else
        {
          State.VcX(i) = SS_it->X;
          State.VcY(i) = SS_it->Y;
        }
      ++SS_it;
    }


  State.VcshiftX.setZero();
  State.VcshiftY.setZero();
  State.Vshift.resize(N_,NbPrwSteps);
  State.Vshift.setZero();
  SS_it = SupportStates_deq.begin();
  State.VcshiftX(0) = SS_it->X;
  State.VcshiftY(0) = SS_it->Y;
  for(unsigned i=0; i<(N_-1); ++i)
    {
      for(unsigned j = 0; j < NbPrwSteps; ++j)
        {
          State.Vshift(i+1,j) = State.V(i,j);

        }
      State.VcshiftX(i+1) = State.VcX(i);
      State.VcshiftY(i+1) = State.VcY(i);
    }

}


void
GeneratorVelRef::compute_global_reference( const solution_t & Solution )
{

  reference_t & Ref = IntermedData_->Reference();

  Ref.Global.X_vec.resize(N_);
  Ref.Global.X_vec.setZero();
  Ref.Global.Y_vec.resize(N_);
  Ref.Global.Y_vec.setZero();
  double YawTrunk;
  for( unsigned i=0; i<N_; i++ )
    {
      YawTrunk = Solution.TrunkOrientations_deq[i];
      Ref.Global.X_vec(i) = Ref.Local.X*cos(YawTrunk)-Ref.Local.Y*sin(YawTrunk);
      Ref.Global.Y_vec(i) = Ref.Local.Y*cos(YawTrunk)+Ref.Local.X*sin(YawTrunk);
    }

}


void
GeneratorVelRef::initialize_matrices()
{

  linear_inequality_t & IneqCoP = IntermedData_->Inequalities( INEQ_COP );
  initialize_matrices( IneqCoP );
  linear_inequality_t & IneqCoM = IntermedData_->Inequalities( INEQ_COM );
  initialize_matrices( IneqCoM );

  IntermedQPMat::state_variant_t & State = IntermedData_->State();
  bool Preserve = true;
  State.VcshiftX.resize(N_);
  State.VcshiftX.setZero();
  State.VcshiftY.resize(N_);
  State.VcshiftY.setZero();
  State.VcX.resize(N_);
  State.VcX.setZero();
  State.VcY.resize(N_);
  State.VcY.setZero();

}


void
GeneratorVelRef::initialize_matrices( linear_inequality_t & Inequalities)
{

  switch(Inequalities.type)
    {
    case INEQ_COP:
      Inequalities.D.X_mat.resize(4*N_,N_);
      Inequalities.D.X_mat.setZero();
      Inequalities.D.Y_mat.resize(4*N_,N_);
      Inequalities.D.Y_mat.setZero();
      Inequalities.Dc_vec.resize(4*N_);
      Inequalities.Dc_vec.setZero();
      break;
    case INEQ_COM://TODO: fixed resize
      Inequalities.D.X_mat.resize(40,N_);
      Inequalities.D.X_mat.setZero();
      Inequalities.D.Y_mat.resize(40,N_);
      Inequalities.D.Y_mat.setZero();
      Inequalities.D.Z_mat.resize(40,1);
      Inequalities.D.Z_mat.setZero();
      Inequalities.Dc_vec.resize(40);
      Inequalities.Dc_vec.setZero();
      break;
    }

}


void
GeneratorVelRef::
build_inequalities_cop
(linear_inequality_t & Inequalities,
 const std::deque<support_state_t> & SupportStates_deq) const
{

  deque<support_state_t>::const_iterator prwSS_it = SupportStates_deq.begin();

  const unsigned nbEdges = 4;
  const unsigned nbIneq = 4;
  convex_hull_t CoPHull( nbEdges, nbIneq );
  RFI_->set_vertices( CoPHull, *prwSS_it, INEQ_COP );

  ++prwSS_it;//Point at the first previewed instant
  Inequalities.D.X_mat.reserve(N_*nbEdges);
  Inequalities.D.Y_mat.reserve(N_*nbEdges);

  for( unsigned i=0; i<N_; i++ )
    {
      if( prwSS_it->StateChanged )
        {
          RFI_->set_vertices( CoPHull, *prwSS_it, INEQ_COP );
          //        cout << "cop foot\n" ;
          //      if( prwSS_it->Foot == LEFT )
          //        cout << "LEFT \n" ;
          //      else
          //        cout << "RIGHT \n" ;
          //      cout << "convex hull : \n";
          //      for(unsigned int k = 0 ; k < CoPHull.X_vec.size() ; ++k)
          //        cout << CoPHull.X_vec[k] << " " << CoPHull.Y_vec[k]
          // << endl ;
        }
      RFI_->compute_linear_system( CoPHull, *prwSS_it );
      //      cout << "linear system \n";
      for( unsigned j = 0; j < nbEdges; j++ )
        {
          Inequalities.D.X_mat.insert( i*nbEdges+j, i)= CoPHull.A_vec[j] ;
          Inequalities.D.Y_mat.insert( i*nbEdges+j, i)= CoPHull.B_vec[j] ;
          Inequalities.Dc_vec( i*nbEdges+j ) = CoPHull.D_vec[j];
          //          cout << CoPHull.A_vec[j] << " " << CoPHull.B_vec[j]
          // << " " << CoPHull.D_vec[j] << endl;
        }

      ++prwSS_it;
    }
  //  cout << endl;
}


void
GeneratorVelRef::
build_inequalities_feet
( linear_inequality_t & Inequalities,
  const std::deque<support_state_t> & SupportStates_deq ) const
{

  // Arrays for the generated set of inequalities
  const unsigned nbEdges = 5;
  const unsigned nbIneq = 5;
  convex_hull_t FeetHull( nbEdges, nbIneq );

  unsigned nbSteps = SupportStates_deq.back().StepNumber;
  Inequalities.resize(nbEdges*nbSteps,nbSteps, false);

  deque<support_state_t>::const_iterator prwSS_it = SupportStates_deq.begin();
  prwSS_it++;//Point at the first previewed instant
  Inequalities.D.X_mat.reserve(N_*nbEdges);
  Inequalities.D.Y_mat.reserve(N_*nbEdges);
  for( unsigned i=0; i<N_; i++ )
    {
      //foot positioning constraints
      if( prwSS_it->StateChanged &&
          prwSS_it->StepNumber>0 && prwSS_it->Phase != DS)
        {
          prwSS_it--;//Take the support state before
          RFI_->set_vertices( FeetHull, *prwSS_it, INEQ_FEET );
          //          if( prwSS_it->Foot == LEFT )
          //            cout << "LEFT \n" ;
          //          else
          //            cout << "RIGHT \n" ;
          //          for(unsigned int k = 0 ; k < FeetHull.X_vec.size() ; ++k)
          //            cout << FeetHull.X_vec[k] << " " << FeetHull.Y_vec[k]
          // << endl ;

          prwSS_it++;

          RFI_->compute_linear_system( FeetHull, *prwSS_it );
          //cout << "linear system \n";
          for( unsigned j = 0; j < nbEdges; j++ )
            {
              Inequalities.D.X_mat.insert
                ((prwSS_it->StepNumber-1)*nbEdges+j,
                 (prwSS_it->StepNumber-1))= FeetHull.A_vec[j] ;
              Inequalities.D.Y_mat.insert
                ((prwSS_it->StepNumber-1)*nbEdges+j,
                 (prwSS_it->StepNumber-1))= FeetHull.B_vec[j] ;
              Inequalities.Dc_vec
                ( (prwSS_it->StepNumber-1)*nbEdges+j ) = FeetHull.D_vec[j];
              // cout << FeetHull.A_vec[j] << " " << FeetHull.B_vec[j]
              // << " " << FeetHull.D_vec[j] << endl;
            }
        }

      prwSS_it++;
    }
  //  cout << endl ;
}


void
GeneratorVelRef::
build_inequalities_com
(linear_inequality_t & Inequalities,
 const std::deque<support_state_t> & SupportStates_deq) const
{

  deque<support_state_t>::const_iterator prwSS_it = SupportStates_deq.begin();

  const unsigned nbEdges = 0;
  const unsigned nbIneq = 10;
  convex_hull_t CoPHull( nbEdges, nbIneq );
  RFI_->set_inequalities( CoPHull, *prwSS_it, INEQ_COM );

  ++prwSS_it;//Point at the first previewed instant
  unsigned nbIneqsSet = 0;
  Inequalities.D.X_mat.reserve(N_*nbIneq);

  for( unsigned i=0; i<N_; ++i )
    {
      if( prwSS_it->StateChanged )
        {
          //          RFI_->set_inequalities( CoPHull, *prwSS_it, INEQ_COP );

          for( unsigned j = 0; j < nbIneq; j++ )
            {
              Inequalities.D.X_mat.insert( nbIneqsSet+j, i)= CoPHull.A_vec[j] ;
              Inequalities.D.Y_mat.insert( nbIneqsSet+j, i)= CoPHull.B_vec[j] ;
              Inequalities.D.Z_mat.insert( nbIneqsSet+j, 0)= CoPHull.C_vec[j] ;
              Inequalities.Dc_vec( i*nbEdges+j ) = CoPHull.D_vec[j];
            }
          nbIneqsSet+=nbIneq;
        }

      ++prwSS_it;
    }

}


void
GeneratorVelRef::
build_constraints_cop
(const linear_inequality_t & IneqCoP,
 unsigned int NbStepsPreviewed, QPProblem & Pb)
{

  unsigned int NbConstraints = Pb.NbConstraints();

  // -D*U
  compute_term  ( MM_, -1.0, IneqCoP.D.X_mat,
                  Robot_->DynamicsCoPJerk().U       );
  Pb.add_term_to( MATRIX_DU, MM_, NbConstraints, 0  );
  compute_term  ( MM_, -1.0, IneqCoP.D.Y_mat,
                  Robot_->DynamicsCoPJerk().U       );
  Pb.add_term_to( MATRIX_DU, MM_, NbConstraints, N_ );

  // +D*V
  compute_term  ( MM_, 1.0, IneqCoP.D.X_mat, IntermedData_->State().V  );
  // +  Robot_->LeftFoot().Dynamics(COP).U +
  // Robot_->RightFoot().Dynamics(COP).U        );
  Pb.add_term_to( MATRIX_DU, MM_, NbConstraints,2*N_);

  //  cout << "IntermedData_->State().V  = "
  // << IntermedData_->State().V  << endl ;
  compute_term  ( MM_, 1.0, IneqCoP.D.Y_mat, IntermedData_->State().V );
  // +  Robot_->LeftFoot().Dynamics(COP).U +
  // Robot_->RightFoot().Dynamics(COP).U        );
  Pb.add_term_to( MATRIX_DU, MM_, NbConstraints,2*N_+NbStepsPreviewed );

  //constant part
  // +dc
  Pb.add_term_to( VECTOR_DS,IneqCoP.Dc_vec, NbConstraints);

  // -D*S_z*x
  compute_term  ( MV_, -1.0, IneqCoP.D.X_mat, Robot_->DynamicsCoPJerk().S,
                  IntermedData_->State().CoM.x );
  Pb.add_term_to( VECTOR_DS, MV_,NbConstraints);
  /*
   * Usefull for multibody dynamics
   *
   compute_term  ( MV_, -1.0, IneqCoP.D.x, Robot_->LeftFoot().Dynamics(COP).S,
   Robot_->LeftFoot().State().X   );
   Pb.add_term_to( VECTOR_DS, MV_, 
   NbConstraints                                                  );
   compute_term  ( MV_, -1.0, IneqCoP.D.x, Robot_->RightFoot().Dynamics(COP).S,
   Robot_->RightFoot().State().X );
   Pb.add_term_to( VECTOR_DS, MV_, 
   NbConstraints                                                  );
  */
  compute_term  ( MV_, -1.0, IneqCoP.D.Y_mat,  Robot_->DynamicsCoPJerk().S,
                  IntermedData_->State().CoM.y        );
  Pb.add_term_to( VECTOR_DS, MV_,NbConstraints);
  /*
   * Usefull for multibody dynamics
   *
   compute_term  ( MV_, -1.0, IneqCoP.D.y, Robot_->LeftFoot().Dynamics(COP).S,
   Robot_->LeftFoot().State().Y   );
   Pb.add_term_to( VECTOR_DS, MV_, 
   NbConstraints                                                  );
   compute_term  ( MV_, -1.0, IneqCoP.D.y, Robot_->RightFoot().Dynamics(COP).S,
   Robot_->RightFoot().State().Y );
   Pb.add_term_to( VECTOR_DS, MV_, 
   NbConstraints                                                  );
  */

  // +D*Vc*FP
  compute_term  ( MV_, 1.0, IneqCoP.D.X_mat, IntermedData_->State().VcX    );
  Pb.add_term_to( VECTOR_DS, MV_, NbConstraints                            );
  compute_term  ( MV_, 1.0, IneqCoP.D.Y_mat, IntermedData_->State().VcY    );
  Pb.add_term_to( VECTOR_DS, MV_, NbConstraints                            );
}


void
GeneratorVelRef::
build_constraints_feet
(const linear_inequality_t & IneqFeet,
 const IntermedQPMat::state_variant_t & State,
 int NbStepsPreviewed, QPProblem & Pb)
{
  unsigned int NbConstraints = Pb.NbConstraints();

  // -D*V_f
  compute_term  ( MM_, -1.0, IneqFeet.D.X_mat, State.V_f               );
  Pb.add_term_to( MATRIX_DU, MM_, NbConstraints, 2*N_                  );

  compute_term  ( MM_, -1.0, IneqFeet.D.Y_mat, State.V_f               );
  Pb.add_term_to( MATRIX_DU, MM_, NbConstraints, 2*N_+NbStepsPreviewed );

  // +dc
  Pb.add_term_to(  VECTOR_DS, IneqFeet.Dc_vec, NbConstraints           );

  // D*Vc_f*FPc
  compute_term  ( MV_, 1.0, IneqFeet.D.X_mat, State.Vc_fX              );
  Pb.add_term_to( VECTOR_DS, MV_, NbConstraints                        );

  compute_term  ( MV_, 1.0, IneqFeet.D.Y_mat, State.Vc_fY              );
  Pb.add_term_to( VECTOR_DS, MV_, NbConstraints                        );
}


void
GeneratorVelRef::
build_constraints_com
( const linear_inequality_t & IneqCoM,
  const support_state_t & CurrentSupport, QPProblem & Pb )
{

  const linear_dynamics_t & CoMDyn = Robot_->CoM().Dynamics( POSITION );
  const IntermedQPMat::state_variant_t & State = IntermedData_->State();
  const com_t & CoM = IntermedData_->State().CoM;

  unsigned nbConstraints = Pb.NbConstraints();

  // D*(S*c+U*ddd-(Vc*pc+V*p))+Dc > 0:
  // ---------------------------------
  // +Dx*U
  compute_term  ( MM_, 1.0, IneqCoM.D.X_mat, CoMDyn.U       );
  Pb.add_term_to( MATRIX_DU, MM_, nbConstraints, 0          );
  // +Dy*U
  compute_term  ( MM_, 1.0, IneqCoM.D.Y_mat, CoMDyn.U       );
  Pb.add_term_to( MATRIX_DU, MM_, nbConstraints, N_         );

  // -Dx*Vshift
  compute_term (MM_, -1.0, IneqCoM.D.X_mat, State.Vshift        );
  Pb.add_term_to( MATRIX_DU, MM_, nbConstraints, 2*N_           );//X
  // -Dy*Vshift
  compute_term (MM_, -1.0, IneqCoM.D.Y_mat, State.Vshift        );
  Pb.add_term_to( MATRIX_DU, MM_, nbConstraints, 2*N_           );//X

  // +Dx*(S*cx-Vc*pcx)
  compute_term  ( MV_, 1.0, IneqCoM.D.X_mat, CoMDyn.S, CoM.x          );
  Pb.add_term_to( VECTOR_DS, MV_,  nbConstraints                      );
  compute_term  ( MV_, -CurrentSupport.X, IneqCoM.D.X_mat,
                  State.VcshiftX );
  Pb.add_term_to( VECTOR_DS, MV_,  nbConstraints  );
  // +Dy*(S*cy-Vc*pcy)
  compute_term  ( MV_, 1.0, IneqCoM.D.Y_mat, CoMDyn.S, CoM.y        );
  Pb.add_term_to( VECTOR_DS, MV_,  nbConstraints );
  compute_term  ( MV_, -CurrentSupport.Y, IneqCoM.D.Y_mat,
                  State.VcshiftY  );
  Pb.add_term_to( VECTOR_DS, MV_,  nbConstraints);
  // +Dz*cz
  MM_ = IneqCoM.D.Z_mat*Robot_->CoMHeight();
  Pb.add_term_to( VECTOR_DS, MM_,  nbConstraints, 0    );
  // +dc
  Pb.add_term_to(  VECTOR_DS, IneqCoM.Dc_vec, nbConstraints  );


}


void
GeneratorVelRef::
build_eq_constraints_feet
( const std::deque<support_state_t> &
  SupportStates_deq,
  unsigned int NbStepsPreviewed, QPProblem & Pb )
{

  if(SupportStates_deq.front().StateChanged)
    Robot_->SupportTrajectory().pop_front();
  std::deque<support_state_t>::const_iterator SPTraj_it =
    Robot_->SupportTrajectory().begin();

  Eigen::MatrixXd EqualityMatrix;
  Eigen::VectorXd EqualityVector;
  EqualityMatrix.resize(2,2*NbStepsPreviewed);
  EqualityMatrix.setZero();
  EqualityVector.resize(2, false);
  EqualityVector.setZero();
  Pb.NbEqConstraints(2*NbStepsPreviewed);
  for(unsigned int i = 0; i< NbStepsPreviewed; i++)
    {
      EqualityMatrix(0,i) = 1.0;
      EqualityVector(0) = -SPTraj_it->X;
      EqualityMatrix(1,NbStepsPreviewed+i) = 1.0;
      EqualityVector(1) = -SPTraj_it->Y;
      Pb.add_term_to( MATRIX_DU, EqualityMatrix, 2*i, 2*N_ );
      Pb.add_term_to( VECTOR_DS, EqualityVector, 2*i );
      EqualityMatrix.setZero();
      EqualityVector.setZero();
      SPTraj_it++;
    }

}

void GeneratorVelRef::
build_eq_constraints_limitPosFeet
(const solution_t & Solution,QPProblem & Pb)
{
  std::deque<support_state_t>::const_iterator SPTraj_it =
    Solution.SupportStates_deq.begin();
  int ItBeforeLanding = 0 ;
  while(SPTraj_it!=Solution.SupportStates_deq.end())
    {
      ++SPTraj_it;
      if ( SPTraj_it->StateChanged !=1 )
        {
          ++ItBeforeLanding ;
        }
      else
        {
          break;
        }
    }
  int ItBeforeLandingThresh = 2 ;
  unsigned NbStepsPreviewed = Solution.SupportStates_deq.back().StepNumber;
  if( ItBeforeLanding <= ItBeforeLandingThresh && ItBeforeLanding > 0
      && Solution.SupportStates_deq.front().Phase == SS
      && Solution.SupportStates_deq.front().StateChanged != 1
      && NbStepsPreviewed > 0 )
    {
      unsigned int NbConstraints = Pb.NbConstraints();
      Eigen::MatrixXd EqualityMatrix;
      Eigen::VectorXd EqualityVector;

      EqualityMatrix.resize(2,2*N_+2*NbStepsPreviewed);
      EqualityMatrix.setZero();
      EqualityVector.resize(2, false);
      EqualityVector.setZero();

      EqualityMatrix(0,2*N_) =  1.0;
      EqualityVector(0) =  -LastFootSolX_ ;
      EqualityMatrix(1,2*N_+NbStepsPreviewed) =  1.0;
      EqualityVector(1) =  -LastFootSolY_ ;
      Pb.add_term_to( MATRIX_DU, EqualityMatrix, NbConstraints, 0 );
      Pb.add_term_to( VECTOR_DS, EqualityVector, NbConstraints );

      EqualityMatrix.setZero();
      EqualityVector.setZero();
      Pb.NbEqConstraints((unsigned int)(EqualityVector.size()+1));
    }
  else
    {
      Pb.NbEqConstraints(0);
    }
  return;
}

void
GeneratorVelRef::build_constraints( QPProblem & Pb, solution_t & Solution )
{
  unsigned nbStepsPreviewed = Solution.SupportStates_deq.back().StepNumber;

  //Equality constraints:
  //---------------------
  //build_eq_constraints_feet( Solution.SupportStates_deq, nbStepsPreviewed,
  //Pb );
  build_eq_constraints_limitPosFeet( Solution, Pb);

  // Polygonal constraints:
  // ----------------------
  //CoP constraints
  linear_inequality_t & IneqCoP = IntermedData_->Inequalities( INEQ_COP );
  build_inequalities_cop( IneqCoP, Solution.SupportStates_deq );
  build_constraints_cop( IneqCoP, nbStepsPreviewed, Pb );

  //Foot constraints
  linear_inequality_t & IneqFeet = IntermedData_->Inequalities( INEQ_FEET );
  build_inequalities_feet( IneqFeet, Solution.SupportStates_deq );
  build_constraints_feet( IneqFeet, IntermedData_->State(), nbStepsPreviewed,
                          Pb );

  // Polyhedric constraints:
  // -----------------------
  //  linear_inequality_t & IneqCoM = IntermedData_->Inequalities( INEQ_COM );
  //  build_inequalities_com( IneqCoM, Solution.SupportStates_deq );
  //  const support_state_t & CurrentSupport =
  //   Solution.SupportStates_deq.front();
  //  build_constraints_com( IneqCoM, CurrentSupport, Pb );

  if (Solution.useWarmStart)
    {
      compute_warm_start( Solution );
      //TODO: Move to update_problem or build_constraints?
    }
}


void
GeneratorVelRef::build_invariant_part( QPProblem & Pb )
{

  const RigidBody & CoM = Robot_->CoM();

  //Constant terms in the Hessian
  // +a*U'*U
  const IntermedQPMat::objective_variant_t & Jerk =
    IntermedData_->Objective(JERK_MIN );
  const linear_dynamics_t & JerkDynamics = CoM.Dynamics( JERK );
  compute_term  ( MM_, Jerk.weight, JerkDynamics.UT, JerkDynamics.U     );
  Pb.add_term_to( MATRIX_Q, MM_, 0, 0                                   );
  Pb.add_term_to( MATRIX_Q, MM_, N_, N_                                 );

  // +a*U'*U
  const IntermedQPMat::objective_variant_t & InstVel =
    IntermedData_->Objective(INSTANT_VELOCITY );
  const linear_dynamics_t & VelDynamics = CoM.Dynamics( VELOCITY );
  compute_term  ( MM_, InstVel.weight, VelDynamics.UT, VelDynamics.U    );
  Pb.add_term_to( MATRIX_Q, MM_, 0, 0                                   );
  Pb.add_term_to( MATRIX_Q, MM_, N_, N_                                 );

  // +a*U'*U
  const IntermedQPMat::objective_variant_t & COPCent =
    IntermedData_->Objective(COP_CENTERING );
  compute_term  ( MM_, COPCent.weight, Robot_->DynamicsCoPJerk().UT,
                  Robot_->DynamicsCoPJerk().U        );
  Pb.add_term_to( MATRIX_Q, MM_, 0,0);
  Pb.add_term_to( MATRIX_Q, MM_, N_,N_);

}


void
GeneratorVelRef::
update_problem
( QPProblem & Pb,
  const std::deque<support_state_t> & SupportStates_deq )
{

  Pb.clear(VECTOR_D);

  unsigned nbStepsPreviewed = SupportStates_deq.back().StepNumber;
  const IntermedQPMat::state_variant_t & State = IntermedData_->State();
  const RigidBody & CoM = Robot_->CoM();

  // Instant velocity terms
  const IntermedQPMat::objective_variant_t & InstVel =
    IntermedData_->Objective(INSTANT_VELOCITY );
  const linear_dynamics_t & VelDynamics = CoM.Dynamics( VELOCITY );
  // Linear part
  // +a*U'*S*x
  compute_term  ( MV_, InstVel.weight, VelDynamics.UT, VelDynamics.S,
                  State.CoM.x      );
  Pb.add_term_to( VECTOR_D, MV_,
                  0                                                      );
  compute_term  ( MV_, InstVel.weight, VelDynamics.UT, VelDynamics.S,
                  State.CoM.y      );
  Pb.add_term_to( VECTOR_D, MV_,
                  N_                                                     );
  // +a*U'*ref
  compute_term  ( MV_, -InstVel.weight, VelDynamics.UT,
                  State.Ref.Global.X_vec);
  Pb.add_term_to( VECTOR_D, MV_, 0  );
  compute_term  ( MV_, -InstVel.weight,
                  VelDynamics.UT, State.Ref.Global.Y_vec  );
  Pb.add_term_to( VECTOR_D, MV_, N_ );



  // COP - centering terms
  const IntermedQPMat::objective_variant_t & COPCent =
    IntermedData_->Objective(COP_CENTERING );
  const linear_dynamics_t & CoPDynamics = Robot_->DynamicsCoPJerk( );
  //  const linear_dynamics_t & LFCoP = Robot_->LeftFoot().Dynamics(COP);
  //  const linear_dynamics_t & RFCoP = Robot_->RightFoot().Dynamics(COP);
  // Hessian
  // -a*U'*V
  compute_term  ( MM_, -COPCent.weight, CoPDynamics.UT, State.V         );
  Pb.add_term_to(  MATRIX_Q, MM_, 0, 2*N_                               );
  Pb.add_term_to(  MATRIX_Q, MM_, N_, 2*N_+nbStepsPreviewed             );

  // -a*V*U
  compute_term  ( MM_, -COPCent.weight, State.VT, CoPDynamics.U         );
  Pb.add_term_to( MATRIX_Q, MM_, 2*N_, 0  );
  Pb.add_term_to( MATRIX_Q, MM_, 2*N_+nbStepsPreviewed, N_  );
  //+a*V'*V
  compute_term  ( MM_, COPCent.weight, State.VT, State.V );
  Pb.add_term_to( MATRIX_Q, MM_, 2*N_, 2*N_);
  Pb.add_term_to( MATRIX_Q, MM_, 2*N_+nbStepsPreviewed, 2*N_+nbStepsPreviewed);

  //Linear part
  // -a*V'*S*x
  compute_term  ( MV_, -COPCent.weight, State.VT, CoPDynamics.S, State.CoM.x );
  Pb.add_term_to( VECTOR_D, MV_, 2*N_ );
  compute_term  ( MV_, -COPCent.weight, State.VT, CoPDynamics.S, State.CoM.y );
  Pb.add_term_to( VECTOR_D, MV_, 2*N_+nbStepsPreviewed);
  // +a*V'*Vc*x
  compute_term  ( MV_, COPCent.weight, State.VT, State.VcX  );
  Pb.add_term_to( VECTOR_D, MV_, 2*N_                       );
  compute_term  ( MV_, COPCent.weight, State.VT, State.VcY  );
  Pb.add_term_to( VECTOR_D, MV_, 2*N_+nbStepsPreviewed      );
}


void
GeneratorVelRef::compute_warm_start( solution_t & Solution )
{

  // Initialize:
  // -----------
  unsigned int nbSteps = Solution.SupportStates_deq.back().StepNumber;

  // Copy current support
  support_state_t currentSupport = Solution.SupportStates_deq.front();

  // ZMP position vector
  Eigen::VectorXd zx(N_);
  Eigen::VectorXd zy(N_);

  double feetSpacing = 0.2;
  double sgn = 0;

  Solution.initialSolution.resize(2*N_+2*nbSteps);

  // Compute initial ZMP and foot positions:
  // ---------------------------------------
  deque<support_state_t>::iterator prwSS_it =
    Solution.SupportStates_deq.begin();
  prwSS_it++;//Point at the first previewed support state
  unsigned int j = 0;
  for(unsigned int i=0; i<N_; i++)
    {
      // Check if the support foot has changed
      if (currentSupport.Foot != prwSS_it->Foot)
        {
          currentSupport.Foot = prwSS_it->Foot;

          // Compute new feasible foot position
          if(prwSS_it->Foot == RIGHT)
            sgn=1;
          else
            sgn=-1;
          currentSupport.X += sgn*feetSpacing*sin(currentSupport.Yaw);
          currentSupport.Y -= sgn*feetSpacing*cos(currentSupport.Yaw);
          currentSupport.Yaw = Solution.SupportOrientations_deq[j];

          // Set the new position into initial solution vector
          Solution.initialSolution(2*N_+j) = currentSupport.X;
          Solution.initialSolution(2*N_+nbSteps+j) = currentSupport.Y;

          ++j;
        }

      // Set the ZMP at the center of the foot
      zx(i-1) = currentSupport.X;
      zy(i-1) = currentSupport.Y;

      prwSS_it++;
    }

  // Compute initial jerk:
  // ---------------------
  Eigen::VectorXd X(N_);
  Eigen::VectorXd Y(N_);
  MV2_=  Robot_->DynamicsCoPJerk().S * IntermedData_->State().CoM.x  ;
  X=     Robot_->DynamicsCoPJerk().Um1 * zx-MV2_                     ;
  MV2_=  Robot_->DynamicsCoPJerk().S * IntermedData_->State().CoM.y  ;
  Y=     Robot_->DynamicsCoPJerk().Um1* zy-MV2_                     ;

  for(unsigned int i=0; i<N_; i++)
    {
      Solution.initialSolution(i)=X(i);
      Solution.initialSolution(N_+i)=Y(i);
    }

}



void
GeneratorVelRef::
compute_term
(Eigen::MatrixXd &weightMM, double weight,
 const Eigen::MatrixXd &M1, const Eigen::MatrixXd &M2)
{
  weightMM = M1*M2;
  weightMM *= weight;
}


void
GeneratorVelRef::
compute_term
(Eigen::VectorXd &weightMV, double weight,
 const Eigen::MatrixXd &M, const Eigen::VectorXd &V)
{
  weightMV = M*V;
  weightMV *= weight;
}


void
GeneratorVelRef::
compute_term
(Eigen::VectorXd &weightMV,
 double weight, const Eigen::MatrixXd &M,
 const Eigen::VectorXd &V, double scalar)
{
  weightMV = M*V;
  weightMV *= weight*scalar;
}


void
GeneratorVelRef::
compute_term
(Eigen::VectorXd &weightMV,
 double weight, const Eigen::MatrixXd &M1,
 const Eigen::MatrixXd &M2, const Eigen::VectorXd &V2)
{
  MV2_ = M2*V2;
  weightMV = M1*MV2_;
  weightMV *= weight;
}



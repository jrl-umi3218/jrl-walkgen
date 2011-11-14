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
#include <iostream>
#include <fstream>
#include <cmath>
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
    const deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
    const deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
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
GeneratorVelRef::generate_selection_matrices( const std::deque<support_state_t> & SupportStates_deq )
{

  IntermedQPMat::state_variant_t & State = IntermedData_->State();
  const unsigned & NbPrwSteps = SupportStates_deq.back().StepNumber;

  State.VcX.clear();
  State.VcY.clear();
  State.V.resize(N_,NbPrwSteps,false);
  State.V.clear();
  State.VT.resize(NbPrwSteps,N_,false);
  State.VT.clear();
  State.Vc_fX.resize(NbPrwSteps,false);
  State.Vc_fX.clear();
  State.Vc_fY.resize(NbPrwSteps,false);
  State.Vc_fY.clear();
  State.V_f.resize(NbPrwSteps,NbPrwSteps,false);
  State.V_f.clear();


  std::deque<support_state_t>::const_iterator SS_it;
  SS_it = SupportStates_deq.begin();//points at the cur. sup. st.
  ++SS_it;
  for(unsigned i=0;i<N_;i++)
    {
      if(SS_it->StepNumber>0)
        {
          State.V(i,SS_it->StepNumber-1) = State.VT(SS_it->StepNumber-1,i) = 1.0;
          if( SS_it->StepNumber==1 && SS_it->StateChanged && SS_it->Phase == SS )
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


  State.VcshiftX.clear();
  State.VcshiftY.clear();
  State.Vshift.resize(N_,NbPrwSteps,false);
  State.Vshift.clear();
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

  Ref.Global.X_vec.resize(N_,false);
  Ref.Global.X_vec.clear();
  Ref.Global.Y_vec.resize(N_,false);
  Ref.Global.Y_vec.clear();
  double YawTrunk;
  for( unsigned i=0;i<N_;i++ )
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
  linear_inequality_t & IneqCoM = IntermedData_->Inequalities( INEQ_COM );
  initialize_matrices( IneqCoM );

  IntermedQPMat::state_variant_t & State = IntermedData_->State();
  bool Preserve = true;
  State.VcshiftX.resize(N_,!Preserve);
  State.VcshiftX.clear();
  State.VcshiftY.resize(N_,!Preserve);
  State.VcshiftY.clear();
  State.VcX.resize(N_,!Preserve);
  State.VcX.clear();
  State.VcY.resize(N_,!Preserve);
  State.VcY.clear();

}


void
GeneratorVelRef::initialize_matrices( linear_inequality_t & Inequalities, option_e option)
{

  switch(Inequalities.type)
  {
  case INEQ_COP:{

	int ineqSize=4;
	if (option==WITH_TWO_CONTRAINT_BOUNDS){
	  ineqSize=2;
	}
    Inequalities.D.X_mat.resize(ineqSize*N_,N_,false);
    Inequalities.D.X_mat.clear();
    Inequalities.D.Y_mat.resize(ineqSize*N_,N_,false);
    Inequalities.D.Y_mat.clear();
    Inequalities.Dc_vec.resize(ineqSize*N_,false);
    Inequalities.Dc_vec.clear();
    break;}
  case INEQ_COM:{//TODO: fixed resize
    Inequalities.D.X_mat.resize(40,N_,false);
    Inequalities.D.X_mat.clear();
    Inequalities.D.Y_mat.resize(40,N_,false);
    Inequalities.D.Y_mat.clear();
    Inequalities.D.Z_mat.resize(40,1,false);
    Inequalities.D.Z_mat.clear();
    Inequalities.Dc_vec.resize(40,false);
    Inequalities.Dc_vec.clear();
    break;}
  }

}


void 
GeneratorVelRef::build_inequalities_cop(linear_inequality_t & Inequalities,
    const std::deque<support_state_t> & SupportStates_deq,
    option_e option, QPProblem & Pb)
{

  deque<support_state_t>::const_iterator prwSS_it = SupportStates_deq.begin();

  const unsigned nbEdges = 4;

  unsigned NbEdgesLoop= nbEdges ;
  if (option==WITH_TWO_CONTRAINT_BOUNDS){
	  NbEdgesLoop=2;
  }

  const unsigned nbIneq = 4;
  convex_hull_t CoPHull( nbEdges, nbIneq );
  RFI_->set_vertices( CoPHull, *prwSS_it, INEQ_COP );

  if (option==WITH_NEW_FORMULATION){
	unsigned nbStepsPreviewed = SupportStates_deq.back().StepNumber;
	int size = 2*N_+2*nbStepsPreviewed;
	MV_.resize(size);
	MV2_.resize(size);
  }
//std::cout << std::endl;
  ++prwSS_it;//Point at the first previewed instant
  for( unsigned i=0; i<N_; i++ )
    {
      if( prwSS_it->StateChanged )
        RFI_->set_vertices( CoPHull, *prwSS_it, INEQ_COP );
/*
      if (prwSS_it->Phase==SS){
    	  std::cout << "SS";
      }else{
    	  std::cout << "DS";
      }
      if (prwSS_it->Foot==LEFT){
    	  std::cout << " L ";
      }else{
    	  std::cout << " R ";
      }
      if (prwSS_it->StateChanged){
    	  std::cout << "/ ";
      }else{
    	  std::cout << ": ";
      }*/
      if (option==WITH_NEW_FORMULATION){

  			MV_(i)     = min(CoPHull.X_vec[0],CoPHull.X_vec[3]);
  			MV2_(i)    = max(CoPHull.X_vec[0],CoPHull.X_vec[3]);

  			MV_(N_+i)  = min(CoPHull.Y_vec[0],CoPHull.Y_vec[1]);
  			MV2_(N_+i) = max(CoPHull.Y_vec[0],CoPHull.Y_vec[1]);

      }else{
    	  RFI_->compute_linear_system( CoPHull, *prwSS_it );
		  for( unsigned j = 0; j < NbEdgesLoop; j++ )
			{
			  Inequalities.D.X_mat.push_back( i*NbEdgesLoop+j, i, CoPHull.A_vec[j] );
			  Inequalities.D.Y_mat.push_back( i*NbEdgesLoop+j, i, CoPHull.B_vec[j] );
			  Inequalities.Dc_vec( i*NbEdgesLoop+j ) = CoPHull.D_vec[j];
			}


		}
      ++prwSS_it;
    }

  if (option==WITH_NEW_FORMULATION){
	  unsigned nbStepsPreviewed = SupportStates_deq.back().StepNumber;
	for(unsigned int i=2*N_;i<2*N_+2*nbStepsPreviewed;++i){
		MV_(i)=-10e10;
		MV2_(i)=10e10;
	}

	Pb.add_term_to( VECTOR_XL, MV_, 0  );
	Pb.add_term_to( VECTOR_XU, MV2_, 0  );

  }
}


void
GeneratorVelRef::build_inequalities_feet( linear_inequality_t & Inequalities,
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
  for( unsigned i=0; i<N_; i++ )
    {
      //foot positioning constraints
      if( prwSS_it->StateChanged && prwSS_it->StepNumber>0 && prwSS_it->Phase != DS)
        {
          prwSS_it--;//Take the support state before
          RFI_->set_vertices( FeetHull, *prwSS_it, INEQ_FEET );
          prwSS_it++;

          RFI_->compute_linear_system( FeetHull, *prwSS_it );

          for( unsigned j = 0; j < nbEdges; j++ )
            {
              Inequalities.D.X_mat.push_back( (prwSS_it->StepNumber-1)*nbEdges+j, (prwSS_it->StepNumber-1), FeetHull.A_vec[j] );
              Inequalities.D.Y_mat.push_back( (prwSS_it->StepNumber-1)*nbEdges+j, (prwSS_it->StepNumber-1), FeetHull.B_vec[j] );
              Inequalities.Dc_vec( (prwSS_it->StepNumber-1)*nbEdges+j ) = FeetHull.D_vec[j];
            }
        }

      prwSS_it++;
    }

}


void
GeneratorVelRef::build_inequalities_com(linear_inequality_t & Inequalities,
    const std::deque<support_state_t> & SupportStates_deq) const
{

  deque<support_state_t>::const_iterator prwSS_it = SupportStates_deq.begin();

  const unsigned nbEdges = 0;
  const unsigned nbIneq = 10;
  convex_hull_t CoPHull( nbEdges, nbIneq );
  RFI_->set_inequalities( CoPHull, *prwSS_it, INEQ_COM );

  ++prwSS_it;//Point at the first previewed instant
  unsigned nbIneqsSet = 0;
  for( unsigned i=0; i<N_; ++i )
    {
      if( prwSS_it->StateChanged )
        {
//          RFI_->set_inequalities( CoPHull, *prwSS_it, INEQ_COP );

          for( unsigned j = 0; j < nbIneq; j++ )
            {
              Inequalities.D.X_mat.push_back( nbIneqsSet+j, i, CoPHull.A_vec[j] );
              Inequalities.D.Y_mat.push_back( nbIneqsSet+j, i, CoPHull.B_vec[j] );
              Inequalities.D.Z_mat.push_back( nbIneqsSet+j, 0, CoPHull.C_vec[j] );
              Inequalities.Dc_vec( i*nbEdges+j ) = CoPHull.D_vec[j];
            }
          nbIneqsSet+=nbIneq;
        }

      ++prwSS_it;
    }

}


void
GeneratorVelRef::build_constraints_cop(const linear_inequality_t & IneqCoP,
    unsigned int NbStepsPreviewed, QPProblem & Pb, option_e option)
{

  unsigned int NbConstraints = Pb.NbConstraints();

  if (option==WITH_NEW_FORMULATION){




  }else if (option==NONE || option==WITH_TWO_CONTRAINT_BOUNDS){

	  // -D*U
	  compute_term  ( MM_, -1.0, IneqCoP.D.X_mat, Robot_->DynamicsCoPJerk().U       );
	  Pb.add_term_to( MATRIX_DU, MM_, NbConstraints, 0                              );
	  compute_term  ( MM_, -1.0, IneqCoP.D.Y_mat, Robot_->DynamicsCoPJerk().U       );
	  Pb.add_term_to( MATRIX_DU, MM_, NbConstraints, N_                             );


	  // +D*V
	  compute_term  ( MM_, 1.0, IneqCoP.D.X_mat, IntermedData_->State().V 			);
	  // +  Robot_->LeftFoot().Dynamics(COP).U + Robot_->RightFoot().Dynamics(COP).U        );
	  Pb.add_term_to( MATRIX_DU, MM_, NbConstraints, 2*N_                                   );
	  compute_term  ( MM_, 1.0, IneqCoP.D.Y_mat, IntermedData_->State().V  			);
	  // +  Robot_->LeftFoot().Dynamics(COP).U + Robot_->RightFoot().Dynamics(COP).U        );
	  Pb.add_term_to( MATRIX_DU, MM_, NbConstraints, 2*N_+NbStepsPreviewed                  );

	  //constant part
	  // +dc
	  Pb.add_term_to( VECTOR_DS,IneqCoP.Dc_vec, NbConstraints               );
	  if (option==WITH_TWO_CONTRAINT_BOUNDS){
		  Pb.add_term_to( VECTOR_DL,-IneqCoP.Dc_vec, NbConstraints               );
	  }

	  // -D*S_z*x
	  compute_term  ( MV_, -1.0, IneqCoP.D.X_mat, Robot_->DynamicsCoPJerk().S, IntermedData_->State().CoM.x         );
	  Pb.add_term_to( VECTOR_DS, MV_,  NbConstraints                                                                );
	  if (option==WITH_TWO_CONTRAINT_BOUNDS){
		  Pb.add_term_to( VECTOR_DL, MV_,  NbConstraints                                                                );
	  }
	  /*
	   * Usefull for multibody dynamics
	   *
	  compute_term  ( MV_, -1.0, IneqCoP.D.x, Robot_->LeftFoot().Dynamics(COP).S, Robot_->LeftFoot().State().X   );
	  Pb.add_term_to( VECTOR_DS, MV_,  NbConstraints                                                  );
	  compute_term  ( MV_, -1.0, IneqCoP.D.x, Robot_->RightFoot().Dynamics(COP).S, Robot_->RightFoot().State().X );
	  Pb.add_term_to( VECTOR_DS, MV_,  NbConstraints                                                  );
	   */
	  compute_term  ( MV_, -1.0, IneqCoP.D.Y_mat,  Robot_->DynamicsCoPJerk().S, IntermedData_->State().CoM.y        );
	  Pb.add_term_to( VECTOR_DS, MV_,  NbConstraints                                                                );
	  if (option==WITH_TWO_CONTRAINT_BOUNDS){
		  Pb.add_term_to( VECTOR_DL, MV_,  NbConstraints                                                                );
	  }
	  /*
	   * Usefull for multibody dynamics
	   *
	  compute_term  ( MV_, -1.0, IneqCoP.D.y, Robot_->LeftFoot().Dynamics(COP).S, Robot_->LeftFoot().State().Y   );
	  Pb.add_term_to( VECTOR_DS, MV_,  NbConstraints                                                  );
	  compute_term  ( MV_, -1.0, IneqCoP.D.y, Robot_->RightFoot().Dynamics(COP).S, Robot_->RightFoot().State().Y );
	  Pb.add_term_to( VECTOR_DS, MV_,  NbConstraints                                                  );
	   */

	  // +D*Vc*FP
	  compute_term  ( MV_, 1.0, IneqCoP.D.X_mat, IntermedData_->State().VcX    );
	  Pb.add_term_to( VECTOR_DS, MV_, NbConstraints                            );
	  if (option==WITH_TWO_CONTRAINT_BOUNDS){
		  Pb.add_term_to( VECTOR_DL, MV_, NbConstraints                            );
	  }
	  compute_term  ( MV_, 1.0, IneqCoP.D.Y_mat, IntermedData_->State().VcY    );
	  Pb.add_term_to( VECTOR_DS, MV_, NbConstraints                            );
	  if (option==WITH_TWO_CONTRAINT_BOUNDS){
		  Pb.add_term_to( VECTOR_DL, MV_, NbConstraints                            );
	  }

	  if (option==NONE){
		  unsigned int size=MV_.size();
		  MV2_.resize(size);
		  for(unsigned int i=0;i<size;++i){
			  MV2_(i)=-10e10;
		  }
		  Pb.add_term_to( VECTOR_DL, MV2_, NbConstraints       );
	  }
  }
}


void
GeneratorVelRef::build_constraints_feet(const linear_inequality_t & IneqFeet,
    const IntermedQPMat::state_variant_t & State,
    int NbStepsPreviewed, QPProblem & Pb, option_e option)
{

  unsigned int NbConstraints = Pb.NbConstraints();

  // -D*V_f
  compute_term  ( MM_, -1.0, IneqFeet.D.X_mat, State.V_f                        );
  Pb.add_term_to( MATRIX_DU, MM_, NbConstraints, 2*N_                           );
  compute_term  ( MM_, -1.0, IneqFeet.D.Y_mat, State.V_f                        );
  Pb.add_term_to( MATRIX_DU, MM_, NbConstraints, 2*N_+NbStepsPreviewed          );

  // +dc
  Pb.add_term_to(  VECTOR_DS, IneqFeet.Dc_vec, NbConstraints                    );

  // D*Vc_f*FPc
  compute_term  ( MV_, 1.0, IneqFeet.D.X_mat, State.Vc_fX                       );
  Pb.add_term_to( VECTOR_DS, MV_, NbConstraints                                 );
  compute_term  ( MV_, 1.0, IneqFeet.D.Y_mat, State.Vc_fY                       );
  Pb.add_term_to( VECTOR_DS, MV_, NbConstraints                                 );



  unsigned int size=MV_.size();
  MV2_.resize(size);
  for(unsigned int i=0;i<size;++i){
	  MV2_(i)=-10e10;
  }
  Pb.add_term_to( VECTOR_DL, MV2_, NbConstraints       );

}


void
GeneratorVelRef::build_constraints_com( const linear_inequality_t & IneqCoM,
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
  compute_term  ( MV_, 1.0, IneqCoM.D.X_mat, CoMDyn.S, CoM.x                    );
  Pb.add_term_to( VECTOR_DS, MV_,  nbConstraints                                );
  compute_term  ( MV_, -CurrentSupport.X, IneqCoM.D.X_mat, State.VcshiftX       );
  Pb.add_term_to( VECTOR_DS, MV_,  nbConstraints                                );
  // +Dy*(S*cy-Vc*pcy)
  compute_term  ( MV_, 1.0, IneqCoM.D.Y_mat, CoMDyn.S, CoM.y                    );
  Pb.add_term_to( VECTOR_DS, MV_,  nbConstraints                                );
  compute_term  ( MV_, -CurrentSupport.Y, IneqCoM.D.Y_mat, State.VcshiftY       );
  Pb.add_term_to( VECTOR_DS, MV_,  nbConstraints                                );
  // +Dz*cz
  MM_ = IneqCoM.D.Z_mat*Robot_->CoMHeight();
  Pb.add_term_to( VECTOR_DS, MM_,  nbConstraints, 0                             );
  // +dc
  Pb.add_term_to(  VECTOR_DS, IneqCoM.Dc_vec, nbConstraints                     );


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
      Pb.add_term_to( MATRIX_DU, EqualityMatrix, 2*i, 2*N_ );
      Pb.add_term_to( VECTOR_DS, EqualityVector, 2*i );
      EqualityMatrix.clear();
      EqualityVector.clear();
      SPTraj_it++;
    }

}


void
GeneratorVelRef::build_constraints( QPProblem & Pb, const solution_t & Solution, option_e option )
{

  unsigned nbStepsPreviewed = Solution.SupportStates_deq.back().StepNumber;

  //Equality constraints:
  //---------------------
  //  build_eq_constraints_feet( PrwSupportStates_deq, NbStepsPreviewed, Pb );


  // Polygonal constraints:
  // ----------------------
  //CoP constraints
  linear_inequality_t & IneqCoP = IntermedData_->Inequalities( INEQ_COP );
  build_inequalities_cop( IneqCoP, Solution.SupportStates_deq, option, Pb );
  build_constraints_cop( IneqCoP, nbStepsPreviewed, Pb, option );

  //Foot constraints
  linear_inequality_t & IneqFeet = IntermedData_->Inequalities( INEQ_FEET );
  build_inequalities_feet( IneqFeet, Solution.SupportStates_deq );
  build_constraints_feet( IneqFeet, IntermedData_->State(), nbStepsPreviewed, Pb, option );


  // Polyhedric constraints:
  // -----------------------
//  linear_inequality_t & IneqCoM = IntermedData_->Inequalities( INEQ_COM );
//  build_inequalities_com( IneqCoM, Solution.SupportStates_deq );
//  const support_state_t & CurrentSupport = Solution.SupportStates_deq.front();
//  build_constraints_com( IneqCoM, CurrentSupport, Pb );

}


void 
GeneratorVelRef::build_invariant_part( QPProblem & Pb, option_e option )
{
	if (option==WITH_NEW_FORMULATION){


	}else{
	  const RigidBody & CoM = Robot_->CoM();

	  //Constant terms in the Hessian
	  // +a*U'*U
	  const IntermedQPMat::objective_variant_t & Jerk = IntermedData_->Objective( JERK_MIN );
	  const linear_dynamics_t & JerkDynamics = CoM.Dynamics( JERK );
	  compute_term  ( MM_, Jerk.weight, JerkDynamics.UT, JerkDynamics.U     ); // U is identity, no ???
	  Pb.add_term_to( MATRIX_Q, MM_, 0, 0                                   );
	  Pb.add_term_to( MATRIX_Q, MM_, N_, N_                                 );

	  // +a*U'*U
	  const IntermedQPMat::objective_variant_t & InstVel = IntermedData_->Objective( INSTANT_VELOCITY );
	  const linear_dynamics_t & VelDynamics = CoM.Dynamics( VELOCITY );
	  compute_term  ( MM_, InstVel.weight, VelDynamics.UT, VelDynamics.U    );
	  Pb.add_term_to( MATRIX_Q, MM_, 0, 0                                   );
	  Pb.add_term_to( MATRIX_Q, MM_, N_, N_                                 );

	  // +a*U'*U
	  const IntermedQPMat::objective_variant_t & COPCent = IntermedData_->Objective( COP_CENTERING );
	  compute_term  ( MM_, COPCent.weight, Robot_->DynamicsCoPJerk().UT, Robot_->DynamicsCoPJerk().U        );
	  Pb.add_term_to( MATRIX_Q, MM_, 0, 0                                                                   );
	  Pb.add_term_to( MATRIX_Q, MM_, N_, N_                                                                 );
	}
}


void
GeneratorVelRef::update_problem( QPProblem & Pb, const std::deque<support_state_t> & SupportStates_deq, option_e option )
{

	if (option==WITH_NEW_FORMULATION){

		Pb.clear(VECTOR_D);
		Pb.clear(MATRIX_Q);
		const RigidBody & CoM = Robot_->CoM();
		const IntermedQPMat::objective_variant_t & Jerk = IntermedData_->Objective( JERK_MIN );
		const IntermedQPMat::objective_variant_t & InstVel = IntermedData_->Objective( INSTANT_VELOCITY );
		const IntermedQPMat::objective_variant_t & COPCent = IntermedData_->Objective( COP_CENTERING );
		const linear_dynamics_t & CoPDynamics = Robot_->DynamicsCoPJerk( );
		const linear_dynamics_t & VelDynamics = CoM.Dynamics( VELOCITY );
		const linear_dynamics_t & JerkDynamics = CoM.Dynamics( JERK );
		const IntermedQPMat::state_variant_t & State = IntermedData_->State();
		unsigned nbStepsPreviewed = SupportStates_deq.back().StepNumber;

		boost_ublas::matrix<double> Id_N(N_,N_);
		for(int i=0;i<N_;++i){
			for(int j=0;j<N_;++j){
				if(i==j){
					Id_N(i,j)=1;
				}else{
					Id_N(i,j)=0;
				}
			}
		}



		boost_ublas::matrix<double> G(N_,N_);
		boost_ublas::vector<double> H(N_);

		MM_ = prod(VelDynamics.U,CoPDynamics.Um1);
		MM_ = prod(VelDynamics.UT,MM_);
		MM_ = prod(CoPDynamics.Um1T,MM_);

		MM2_= prod(CoPDynamics.Um1T,CoPDynamics.Um1);

		G=  InstVel.weight*MM_ +Jerk.weight*MM2_  ;
		Pb.add_term_to( MATRIX_Q, G+COPCent.weight*Id_N, 0, 0 );
		Pb.add_term_to( MATRIX_Q, G+COPCent.weight*Id_N, N_, N_ );

		MM2_=prod(G , State.V);

		Pb.add_term_to( MATRIX_Q, MM2_, 0 , 2*N_ );
		Pb.add_term_to( MATRIX_Q, MM2_, N_, 2*N_+nbStepsPreviewed );

		MM2_=prod(State.VT , G);

		Pb.add_term_to( MATRIX_Q, MM2_, 2*N_ , 0 );
		Pb.add_term_to( MATRIX_Q, MM2_, 2*N_+nbStepsPreviewed, N_ );

		MM2_=prod(G,State.V);
		MM2_=prod(State.VT, MM2_);

		Pb.add_term_to( MATRIX_Q, MM2_, 2*N_ , 2*N_ );
		Pb.add_term_to( MATRIX_Q, MM2_, 2*N_+nbStepsPreviewed, 2*N_+nbStepsPreviewed );


		MM_ = prod(CoPDynamics.Um1, CoPDynamics.S);
		MM_ = VelDynamics.S - prod(VelDynamics.U,MM_);

		MV_ = prod(MM_,State.CoM.x);
		MV_-= State.Ref.Global.X_vec;


		MV_ = prod(VelDynamics.UT,MV_);
		MV_ = prod(CoPDynamics.Um1T,MV_);
		H   = InstVel.weight*MV_;

		MV_ = prod(CoPDynamics.S   , State.CoM.x);
		MV_ = prod(CoPDynamics.Um1 , MV_);
		MV_ = prod(CoPDynamics.Um1T, MV_);
		H   -= Jerk.weight * MV_;


		MV_ = prod(CoPDynamics.Um1, State.VcX);
		MV_ = prod(CoPDynamics.Um1T, MV_);
		H  += Jerk.weight*MV_;

		MV_ = prod(CoPDynamics.Um1, State.VcX);
		MV_ = prod(VelDynamics.U, MV_);
		MV_ = prod(VelDynamics.UT, MV_);
		MV_ = prod(CoPDynamics.Um1T, MV_);
		H  += InstVel.weight*MV_;

		Pb.add_term_to( VECTOR_D, H, 0 );
		MV_ = prod(State.VT,H);
		Pb.add_term_to( VECTOR_D, MV_, 2*N_ );


		MM_ = prod(CoPDynamics.Um1, CoPDynamics.S);
		MM_ = VelDynamics.S - prod(VelDynamics.U,MM_);

		MV_ = prod(MM_,State.CoM.y);
		MV_-= State.Ref.Global.Y_vec;


		MV_ = prod(VelDynamics.UT,MV_);
		MV_ = prod(CoPDynamics.Um1T,MV_);
		H   = InstVel.weight*MV_;

		MV_ = prod(CoPDynamics.S   , State.CoM.y);
		MV_ = prod(CoPDynamics.Um1 , MV_);
		MV_ = prod(CoPDynamics.Um1T, MV_);
		H   -= Jerk.weight * MV_;


		MV_ = prod(CoPDynamics.Um1, State.VcY);
		MV_ = prod(CoPDynamics.Um1T, MV_);
		H  += Jerk.weight*MV_;

		MV_ = prod(CoPDynamics.Um1, State.VcY);
		MV_ = prod(VelDynamics.U, MV_);
		MV_ = prod(VelDynamics.UT, MV_);
		MV_ = prod(CoPDynamics.Um1T, MV_);
		H  += InstVel.weight*MV_;

		Pb.add_term_to( VECTOR_D, H, N_ );
		MV_ = prod(State.VT,H);
		Pb.add_term_to( VECTOR_D, MV_, 2*N_+nbStepsPreviewed );

	  }else{

		MV_.resize(N_);
		for(int i=0;i<N_;++i){
			MV_(i)=-10e10;
		}
		Pb.add_term_to( VECTOR_XL, MV_, 0  );
		for(int i=0;i<N_;++i){
			MV_(i)=10e10;
		}
		Pb.add_term_to( VECTOR_XU, MV_, 0  );

	  Pb.clear(VECTOR_D);

	  unsigned nbStepsPreviewed = SupportStates_deq.back().StepNumber;
	  const IntermedQPMat::state_variant_t & State = IntermedData_->State();
	  const RigidBody & CoM = Robot_->CoM();

	  // Instant velocity terms
	  const IntermedQPMat::objective_variant_t & InstVel = IntermedData_->Objective( INSTANT_VELOCITY );
	  const linear_dynamics_t & VelDynamics = CoM.Dynamics( VELOCITY );
	  // Linear part
	  // +a*U'*S*x
	  compute_term  ( MV_, InstVel.weight , VelDynamics.UT, VelDynamics.S, State.CoM.x      );
	  Pb.add_term_to( VECTOR_D, MV_, 0                                                      );
	  compute_term  ( MV_ , InstVel.weight, VelDynamics.UT, VelDynamics.S, State.CoM.y      );
	  Pb.add_term_to( VECTOR_D, MV_, N_                                                     );
	  // +a*U'*ref
	  compute_term  ( MV_, -InstVel.weight, VelDynamics.UT, State.Ref.Global.X_vec  );
	  Pb.add_term_to( VECTOR_D, MV_, 0                                              );
	  compute_term  ( MV_, -InstVel.weight, VelDynamics.UT, State.Ref.Global.Y_vec  );
	  Pb.add_term_to( VECTOR_D, MV_, N_                                             );

	  // COP - centering terms
	  const IntermedQPMat::objective_variant_t & COPCent = IntermedData_->Objective( COP_CENTERING );
	  const linear_dynamics_t & CoPDynamics = Robot_->DynamicsCoPJerk( );
	  //  const linear_dynamics_t & LFCoP = Robot_->LeftFoot().Dynamics(COP);
	  //  const linear_dynamics_t & RFCoP = Robot_->RightFoot().Dynamics(COP);
	  // Hessian
	  // -a*U'*V
	  compute_term  ( MM_, -COPCent.weight, CoPDynamics.UT, State.V         );
	  Pb.add_term_to(  MATRIX_Q, MM_, 0, 2*N_                               );
	  Pb.add_term_to(  MATRIX_Q, MM_, N_, 2*N_+nbStepsPreviewed             );

	  // -a*V*U
	  compute_term  ( MM_, -COPCent.weight, State.VT, CoPDynamics.U                );
	  Pb.add_term_to( MATRIX_Q, MM_, 2*N_, 0                                       );
	  Pb.add_term_to( MATRIX_Q, MM_, 2*N_+nbStepsPreviewed, N_                     );
	  //+a*V'*V
	  compute_term  ( MM_, COPCent.weight, State.VT, State.V                       );
	  Pb.add_term_to( MATRIX_Q, MM_, 2*N_, 2*N_                                    );
	  Pb.add_term_to( MATRIX_Q, MM_, 2*N_+nbStepsPreviewed, 2*N_+nbStepsPreviewed  );

	  //Linear part
	  // -a*V'*S*x
	  compute_term  ( MV_, -COPCent.weight, State.VT, CoPDynamics.S, State.CoM.x    );
	  Pb.add_term_to( VECTOR_D, MV_, 2*N_                                           );
	  compute_term  ( MV_, -COPCent.weight, State.VT, CoPDynamics.S, State.CoM.y    );
	  Pb.add_term_to( VECTOR_D, MV_, 2*N_+nbStepsPreviewed                          );
	  // +a*V'*Vc*x
	  compute_term  ( MV_, COPCent.weight, State.VT, State.VcX  );
	  Pb.add_term_to( VECTOR_D, MV_, 2*N_                       );
	  compute_term  ( MV_, COPCent.weight, State.VT, State.VcY  );
	  Pb.add_term_to( VECTOR_D, MV_, 2*N_+nbStepsPreviewed      );
	}

}

void
GeneratorVelRef::convert_cop_to_jerk_formulation( solution_t & Solution ){

	boost_ublas::vector<double> zx(N_);
	boost_ublas::vector<double> zy(N_);

	for(int i=0;i<N_;++i){
		zx(i)=Solution.Solution_vec(i);
		zy(i)=Solution.Solution_vec(N_+i);
	}

	int nbSteps =  (Solution.Solution_vec.size()-2*N_)/2;

	boost_ublas::vector<double> px(nbSteps);
	boost_ublas::vector<double> py(nbSteps);
	boost_ublas::vector<double> Vpx(N_);
	boost_ublas::vector<double> Vpy(N_);
	for(int i=0;i<nbSteps;++i){
		px(i)=Solution.Solution_vec(2*N_+i);
		py(i)=Solution.Solution_vec(2*N_+nbSteps+i);
	}

	if (nbSteps>0){
		Vpx=prod(IntermedData_->State().V,px);
		Vpy=prod(IntermedData_->State().V,py);
	}else{
		for(int i=0;i<N_;++i){
			Vpx(i)=0;
			Vpy(i)=0;
		}
	}

	zx+=Vpx+IntermedData_->State().VcX;
	zy+=Vpy+IntermedData_->State().VcY;

	boost_ublas::vector<double> X(N_);
	boost_ublas::vector<double> Y(N_);
	MV2_= prod( Robot_->DynamicsCoPJerk().S, IntermedData_->State().CoM.x  );
	X=    prod( Robot_->DynamicsCoPJerk().Um1, zx-MV2_                     );
	MV2_= prod( Robot_->DynamicsCoPJerk().S, IntermedData_->State().CoM.y  );
	Y=    prod( Robot_->DynamicsCoPJerk().Um1, zy-MV2_                     );


	for(int i=0;i<N_;++i){
		Solution.Solution_vec(i)=X(i);
		Solution.Solution_vec(N_+i)=Y(i);

	}


}


void
GeneratorVelRef::compute_warm_start( solution_t & Solution )
{

  // Initialize:
  // -----------
  unsigned int nbSteps = Solution.SupportStates_deq.back().StepNumber;
  unsigned int nbStepsMax = 4;


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
  // Copy current support
  support_state_t currentSupport = Solution.SupportStates_deq.front();
  // if in transition phase
  if (prwSS_it->StateChanged){
	  currentSupport=*prwSS_it;
  }
  unsigned int j = 0;
  convex_hull_t FootFeasibilityEdges(5,5);
  convex_hull_t COPFeasibilityEdges(4,4);
  double shiftx,shifty;
  bool noActiveConstraints;
  for(unsigned int i=0; i<N_; i++)
    {
	  // Get COP convex hull for current support
	  RFI_->set_vertices( COPFeasibilityEdges, *prwSS_it, INEQ_COP );
      // Check if the support foot has changed
      if (prwSS_it->StateChanged && prwSS_it->StepNumber>0)
        {

          // Get feet convex hull for current support
		  prwSS_it--;
		  RFI_->set_vertices( FootFeasibilityEdges, *prwSS_it, INEQ_FEET );
		  prwSS_it++;



          // Place the foot on active constraints
          shiftx=shifty=0;
          noActiveConstraints=true;
          for(unsigned int k=0;k<5;++k){
        	  if (Solution.initialConstraint(k+2*N_+j*5)!=0){
        		  int k2=(k+1)%5; // k(4) = k(0)
        		  if (Solution.initialConstraint(k2+2*N_+j*5)!=0){
					  shiftx=FootFeasibilityEdges.X_vec[k2];
					  shifty=FootFeasibilityEdges.Y_vec[k2];
				  }else{
					  shiftx=(FootFeasibilityEdges.X_vec[k]+FootFeasibilityEdges.X_vec[k2])/2;
					  shifty=(FootFeasibilityEdges.Y_vec[k]+FootFeasibilityEdges.Y_vec[k2])/2;
				  }
        		  noActiveConstraints=false;
        		  break;
        	  }
          }
          if (noActiveConstraints){
			  shiftx=(FootFeasibilityEdges.X_vec[4]+FootFeasibilityEdges.X_vec[0])/2;
			  shifty=(FootFeasibilityEdges.Y_vec[4]+FootFeasibilityEdges.Y_vec[2])/2;
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
    		  shiftx=(COPFeasibilityEdges.X_vec[k1]+COPFeasibilityEdges.X_vec[k2])/2;
    		  shifty=(COPFeasibilityEdges.Y_vec[k1]+COPFeasibilityEdges.Y_vec[k2])/2;
    	  }else{
    		  shiftx=COPFeasibilityEdges.X_vec[k2];
    		  shifty=COPFeasibilityEdges.Y_vec[k2];
    	  }
      }


      Solution.initialSolution(i) = shiftx;
      Solution.initialSolution(N_+i) = shifty;
      ++prwSS_it;

    }


}

void GeneratorVelRef::amelif_preview_display(solution_t & Solution){
	std::ofstream data("pg-data-displayer.dat");
	boost_ublas::vector<double> ZX(N_);
	boost_ublas::vector<double> ZY(N_);
	boost_ublas::vector<double> CX(N_);
	boost_ublas::vector<double> CY(N_);
	boost_ublas::vector<double> X(N_);
	boost_ublas::vector<double> Y(N_);
	for(int i=0;i<N_;++i){
		X(i)=Solution.Solution_vec(i);
		Y(i)=Solution.Solution_vec(i+N_);
	}
	// Compute previewed ZMP
	ZX=prod(Robot_->DynamicsCoPJerk().S, IntermedData_->State().CoM.x)+
	   prod(Robot_->DynamicsCoPJerk().U,X);

	ZY=prod(Robot_->DynamicsCoPJerk().S, IntermedData_->State().CoM.y)+
	   prod(Robot_->DynamicsCoPJerk().U,Y);

	// Compute previewed COM
	CX=prod(Robot_->CoM().Dynamics(POSITION).S, IntermedData_->State().CoM.x)+
	   prod(Robot_->CoM().Dynamics(POSITION).U,X);

	CY=prod(Robot_->CoM().Dynamics(POSITION).S, IntermedData_->State().CoM.y)+
	   prod(Robot_->CoM().Dynamics(POSITION).U,Y);
	//display previewed ZMP
	for(int i=0;i<N_;++i){
		std::stringstream ssTmp;
		ssTmp << "TRAJ\t1\t\t0\t1\t1\t\t" << ZX(i) << "\t" << ZY(i) << "\t0\n";
		data.write(ssTmp.str().c_str(),ssTmp.str().size());
	}

	//display previewed COM
	for(int i=0;i<N_;++i){
		std::stringstream ssTmp;
		ssTmp << "TRAJ\t2\t\t1\t0\t0\t\t" << CX(i) << "\t" << CY(i) << "\t0\n";
		data.write(ssTmp.str().c_str(),ssTmp.str().size());
	}



	support_state_t currentSupport = Solution.SupportStates_deq.front();
	deque<support_state_t>::iterator prwSS_it = Solution.SupportStates_deq.begin();

	unsigned int j = 0;
	double x,y;
	convex_hull_t FootFeasibilityEdges(5,5);
	convex_hull_t COPFeasibilityEdges(4,4);
	unsigned int nbSteps = Solution.SupportStates_deq.back().StepNumber;

	//display current COP constraint
	RFI_->set_vertices( COPFeasibilityEdges, *prwSS_it, INEQ_COP );
	for(int k=0;k<4;++k){
		  std::stringstream ssTmp;
		  ssTmp << "BOUND\t-1\t\t0.5\t0.5\t0.5\t\t" <<
				  COPFeasibilityEdges.X_vec[k]+currentSupport.X << "\t" <<
				  COPFeasibilityEdges.Y_vec[k]+currentSupport.Y << "\t0\n";
		  data.write(ssTmp.str().c_str(),ssTmp.str().size());
	 }

	  prwSS_it++;
	for(unsigned int i=0; i<N_; i++)
	{

	  //display constraints
	  if (currentSupport.Foot != prwSS_it->Foot){
		  if (prwSS_it->StepNumber==0){
			  x=prwSS_it->X;
			  y=prwSS_it->Y;
		  }else if (prwSS_it->StepNumber>=nbSteps){
			  x=Solution.Solution_vec(2*N_+nbSteps-1);
			  y=Solution.Solution_vec(2*N_+2*nbSteps-1);
		  }else{
			  x=Solution.Solution_vec(2*N_+prwSS_it->StepNumber-1);
			  y=Solution.Solution_vec(2*N_+nbSteps+prwSS_it->StepNumber-1);
		  }
		  if (prwSS_it->StepNumber-1>0){
			  j=prwSS_it->StepNumber-1;
		  }else{
			  j=0;
		  }
		  if (prwSS_it->StepNumber-1>=nbSteps){
			  j=nbSteps-1;
		  }
		  currentSupport.Foot = prwSS_it->Foot;
		  RFI_->set_vertices( COPFeasibilityEdges, *prwSS_it, INEQ_COP );

		  RFI_->set_vertices( FootFeasibilityEdges, *prwSS_it, INEQ_FEET );

		  //display COP constraints
		  for(int k=0;k<4;++k){
			  std::stringstream ssTmp;
			  ssTmp << "BOUND\t" << j << "\t\t0.5\t0.5\t0.5\t\t" <<
					  COPFeasibilityEdges.X_vec[k]+x << "\t" <<
					  COPFeasibilityEdges.Y_vec[k]+y << "\t0\n";
			  data.write(ssTmp.str().c_str(),ssTmp.str().size());
		  }

		  //display feet constraints
		  for(int k=0;k<5;++k){
			  std::stringstream ssTmp;
			  ssTmp << "BOUND\t" << j+nbSteps << "\t\t0.5\t0.5\t0.5\t\t" <<
					  FootFeasibilityEdges.X_vec[k]+x << "\t" <<
					  FootFeasibilityEdges.Y_vec[k]+y << "\t0\n";
			  data.write(ssTmp.str().c_str(),ssTmp.str().size());
		  }

		  //display feet positions
		  std::stringstream ssTmp;
		  ssTmp << "POINT\t" << j+nbSteps << "\t\t1\t0\t0\t\t" <<
				  x << "\t" <<
				  y << "\t0\n";
		  data.write(ssTmp.str().c_str(),ssTmp.str().size());
		  j++;
	  }
	  prwSS_it++;
	}
	data.close();



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



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
{

  Matrices_ = Data;
  Robot_ = Robot;

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

  IntermedQPMat::objective_variant_t & Objective = Matrices_->Objective( Type );
  Objective.weight = Weight;

}	


void
GeneratorVelRef::preview_support_states( const SupportFSM * FSM, std::deque<support_state_t> & SupportStates_deq )
{

  // INITIALIZE QEUE OF SUPPORT STATES:
  // ----------------------------------
  const reference_t & RefVel = Matrices_->Reference();
  support_state_t & CurrentSupport = Matrices_->SupportState();
  FSM->set_support_state(CurrentTime_, 0, CurrentSupport, RefVel);
  SupportStates_deq.push_back(CurrentSupport);


  // PREVIEW SUPPORT STATES:
  // -----------------------
  //initialize the previewed support state before previewing
  support_state_t PreviewedSupport = CurrentSupport;
  PreviewedSupport.StepNumber  = 0;

  for(unsigned int i=1;i<=N_;i++)
    {
      FSM->set_support_state(CurrentTime_, i, PreviewedSupport, RefVel);
      SupportStates_deq.push_back(PreviewedSupport);
    }

  generate_selection_matrices(SupportStates_deq);

}


void
GeneratorVelRef::generate_selection_matrices( const std::deque<support_state_t> & SupportStates_deq )
{

  IntermedQPMat::state_variant_t & State = Matrices_->State();
  const int & NbPrwSteps = SupportStates_deq.back().StepNumber;

  bool preserve = true;
  State.Vc.resize(N_,!preserve);
  State.Vc.clear();
  State.V.resize(N_,NbPrwSteps,!preserve);
  State.V.clear();
  State.VT.resize(NbPrwSteps,N_,!preserve);
  State.VT.clear();
  State.Vc_f.resize(NbPrwSteps,!preserve);
  State.Vc_f.clear();
  State.V_f.resize(NbPrwSteps,NbPrwSteps,!preserve);
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
GeneratorVelRef::compute_global_reference( const deque<COMState> & TrunkStates_deq )
{

  reference_t & Ref = Matrices_->Reference();

  Ref.Global.X.resize(N_,false);
  Ref.Global.X.clear();
  Ref.Global.Y.resize(N_,false);
  Ref.Global.Y.clear();
  double YawTrunk;
  for( unsigned int i=0;i<N_;i++ )
    {
      YawTrunk = TrunkStates_deq[(int)(i+1)*T_Prw_/T_Ctr_].yaw[0];
      Ref.Global.X(i) = Ref.Local.x*cos(YawTrunk)-Ref.Local.y*sin(YawTrunk);
      Ref.Global.Y(i) = Ref.Local.y*cos(YawTrunk)+Ref.Local.x*sin(YawTrunk);
    }

}


void 
GeneratorVelRef::initialize_matrices()
{

  linear_inequality_t & IneqCoP = Matrices_->Inequalities( IntermedQPMat::INEQ_COP );
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
  for( unsigned int i=1;i<=N_;i++ )
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
GeneratorVelRef::build_inequalities_feet(linear_inequality_t & Inequalities,
				       RelativeFeetInequalities * RFI,
				       const std::deque< FootAbsolutePosition> & LeftFootPositions_deq,
				       const std::deque<FootAbsolutePosition> & RightFootPositions_deq,
				       const std::deque<support_state_t> & SupportStates_deq,
				       const std::deque<double> & PreviewedSupportAngles_deq) const
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

  int StepNumber = 0;
  for(unsigned int i=1;i<=N_;i++ )
    {

      const support_state_t & PrwSupport = SupportStates_deq[i];

      //foot positioning constraints
      if( PrwSupport.StateChanged && PrwSupport.StepNumber>0 && PrwSupport.Phase != DS)
	{
	  SupportAngle = PreviewedSupportAngles_deq[PrwSupport.StepNumber-1];

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
GeneratorVelRef::build_constraints_door(double Time, Door & Door,
    const std::deque<support_state_t> & SupportStates_deq, QPProblem & Pb)
{

  // Build rotation matrix R
  boost_ublas::matrix<double> R(2,N_,false);
  R.clear();
  Door.build_rotation_matrices(Time, N_, R);

  //	// build door linear equation
  //
  //	 double RrotHx = prod(Door.InvRx(),Rrot);
  //	 double RrotHy = prod(Door.InvRy(),Rrot);
  //	 double RrotHp = prod(Door.InvRp(),Rrot);

  // Build selection matrix W
  IntermedQPMat::state_variant_t & State = Matrices_->State();

  const int & NbPrwSteps = SupportStates_deq.back().StepNumber;
  boost_ublas::matrix<double> W(N_,NbPrwSteps,false);
  W.clear();
  bool Empty = true;
  for(int j=0;j<NbPrwSteps;j++)
    {
      for(unsigned int i=0;i<N_;i++)
        {
          if(State.V(i,j) == 1 && Empty == true)
            {
              Empty = false;
              W(i,j)=1;
            }
          else
            W(i,j)=0;
        }
      Empty = true;
    }


  int NbStepsPreviewed = SupportStates_deq.back().StepNumber;

  /// security distance distances from door's edges D1 and D2
  boost_ublas::vector<double> D1(N_,false);
  boost_ublas::vector<double> D2(N_,false);
  boost_ublas::vector<double> D3(N_,false);
  boost_ublas::vector<double> D4(N_,false);
  boost_ublas::vector<double> D5(NbStepsPreviewed,false);
  boost_ublas::vector<double> D6(NbStepsPreviewed,false);
  boost_ublas::vector<double> D7(NbStepsPreviewed,false);

  for(unsigned int i=0;i<N_;i++)
    {
      D1(i) = 0.5;
      D2(i) = 0.2;
      D3(i) = 0.45;
      D4(i) = 0.55;
    }


  for(int i=0;i< NbStepsPreviewed;i++)
    {
      D5(i) = 0.5;
      D6(i) = 0.3;
      D7(i) = 0.7;
    }

  const RigidBody & CoM = Robot_->CoM();
  const linear_dynamics_t & PosDynamics = CoM.Dynamics( POSITION );

  // Number of inequality constraints
  linear_inequality_t DoorConstraints;
  //	 DoorConstraints.resize(4*N_+3*NbPrwSteps,2*(N_+NbPrwSteps),false);

  DoorConstraints.resize(NbStepsPreviewed,2*(NbStepsPreviewed),false);
  DoorConstraints.clear();

  // Compute term diag(RH)Sx
  // -----------------------
  Door::frame_t & FrameBase = Door.FrameBase();
  boost_ublas::vector<double> HXT = FrameBase.InvRx;
  boost_ublas::vector<double> HXRT = trans(prod(HXT,R));


  // Build diagonal matrix out of HXRT
  boost_ublas::matrix<double> Diagx(N_,N_,false);
  Diagx.clear();
  for(unsigned int i=0;i<N_;i++)
    Diagx(i,i) = HXRT(i);

  boost_ublas::matrix<double> RHXSXi = prod(Diagx,PosDynamics.S);
  boost_ublas::vector<double> RHXSX = prod(RHXSXi,State.CoM.x);

  // Compute term diag(RH)Sy
  // -----------------------
  boost_ublas::vector<double> HYT = trans(FrameBase.InvRy);
  boost_ublas::vector<double> HYRT = trans(prod(HYT,R));

  boost_ublas::matrix<double> Diagy(N_,N_,false);
  Diagy.clear();


  for(unsigned int i=0;i<N_;i++)
    {
      Diagy(i,i) = HYRT(i);
    }

  boost_ublas::matrix<double> RHYSYi = prod(Diagy,PosDynamics.S);
  boost_ublas::vector<double> RHYSY = prod(RHYSYi,State.CoM.y);

  //compute the term HPTR
  boost_ublas::vector<double> HPT = trans(FrameBase.InvHp);
  boost_ublas::vector<double> HPTR = trans(prod(HPT,R));

  // COM constraints right side

  boost_ublas::vector<double> COM_CONSTR_R1 = D1+D2-RHXSX-RHYSY- HPTR;
  boost_ublas::vector<double> COM_CONSTR_R2 = D2-COM_CONSTR_R1;

  // COM constraints left side

  //compute term HxTRU
  boost_ublas::vector<double> HXTR = trans(HXRT);
  boost_ublas::matrix<double> HXTRU = prod(Diagx,PosDynamics.U);

  //compute term HyTRU

  boost_ublas::vector<double> HYTR = trans(HYRT);
  boost_ublas::matrix<double> HYTRU = prod(Diagy,PosDynamics.U);

  //Foot constraints with respect to the door
  boost_ublas::matrix<double> HXRTW = prod(Diagx,W);
  boost_ublas::matrix<double> HYRTW = prod(Diagy,W);
  boost_ublas::vector<double> WWH= prod(HPTR,W);


  boost_ublas::matrix<double> WWx(NbPrwSteps,NbPrwSteps,false);
  boost_ublas::matrix<double> WWy(NbPrwSteps,NbPrwSteps,false);

  int line = 0;
  for(unsigned int i=0;i<N_;i++)
    {
      for(int j=0;j<NbPrwSteps;j++)
        {
          if(W(i,j)==1 )
            {
              for(int p=0;p<NbPrwSteps;p++)
                {
                  WWx(line,p)= HXRTW(i,p);
                  WWy(line,p)= HYRTW(i,p);
                }
              line++;
            }
        }
    }


  // lateral constraints with D3 and D4:
  // -----------------------------------
  R.clear();
  Door.build_rotation_matrices( Time, N_, R );
  // Compute term diag(RH)Sx
  // -----------------------
  // boost_ublas::vector<double> HXRT2 = trans(prod(HXT,R));

  boost_ublas::matrix<double> RT = trans(R);
  boost_ublas::vector<double> HXRT2 = prod(RT,HXT);

  // Build diagonal matrix out of HXRT
  boost_ublas::matrix<double> Diagx1(N_,N_,false);
  Diagx1.clear();
  for(unsigned int i=0;i<N_;i++)
    Diagx1(i,i) = HXRT2(i);


  RHXSXi = prod(Diagx1,PosDynamics.S);
  boost_ublas::vector<double> RHXSX2 = prod(RHXSXi,State.CoM.x);

  // Compute term diag(RH)Sy:
  // ------------------------
  boost_ublas::vector<double> HYRT2 = trans(prod(HYT,R));
  boost_ublas::matrix<double> Diagy1(N_,N_,false);
  Diagy1.clear();
  for(unsigned int i=0;i<N_;i++)
    Diagy1(i,i) = HYRT2(i);


  RHYSYi = prod(Diagy1,PosDynamics.S);
  boost_ublas::vector<double> RHYSY2 = prod(RHYSYi,State.CoM.y);

  //compute the term HPTR
  boost_ublas::vector<double> HPTR2 = trans(prod(HPT,R));

  // COM constraints right side

  boost_ublas::vector<double> COM_CONSTR_R1_2 = D3+D4-RHXSX2-RHYSY2- HPTR2;
  boost_ublas::vector<double> COM_CONSTR_R2_2 = D4-COM_CONSTR_R1_2;

  // COM constraints left side

  //compute term HxTRU
  boost_ublas::vector<double> HXTR2 = trans(HXRT2);
  boost_ublas::matrix<double> HXTRU2 = prod(Diagx1,PosDynamics.U);
  //compute term HyTRU

  boost_ublas::vector<double> HYTR2 = trans(HYRT2);
  boost_ublas::matrix<double> HYTRU2 = prod(Diagy1,PosDynamics.U);


  //Foot constraints with respect to Cadre
  boost_ublas::matrix<double> HXRTW2=prod(Diagx1,W);
  boost_ublas::matrix<double> HYRTW2= prod(Diagy1,W);
  boost_ublas::vector<double> WWH2= prod(HPTR2,W);
  boost_ublas::matrix<double> WWx2(NbPrwSteps,NbPrwSteps,false);
  boost_ublas::matrix<double> WWy2(NbPrwSteps,NbPrwSteps,false);


  int line1 = 0;
  for(unsigned int i=0;i<N_;i++)
    {

      for(int j=0;j<NbPrwSteps;j++)
        {
          if(W(i,j)==1)
            {
              for(int p=0;p<NbPrwSteps;p++)
                {
                  WWx2(line1,p)= HXRTW2(i,p);
                  WWy2(line1,p)= HYRTW2(i,p);
                }
              line1++;
            }
        }
    }


  // Front of door 1
  int NbConstraints = Pb.NbConstraints();
  Pb.add_term(HXTRU,QPProblem::MATRIX_DU,NbConstraints,0);
  Pb.add_term(HYTRU,QPProblem::MATRIX_DU,NbConstraints,N_);
  Pb.add_term(COM_CONSTR_R2,QPProblem::VECTOR_DS,NbConstraints);

  //// in front of door2
  //
  //			 NbConstraints = Pb.NbConstraints();
  //			 Pb.add_term(-HXTRU,QPProblem::MATRIX_DU,NbConstraints,0);
  //			 Pb.add_term(-HYTRU,QPProblem::MATRIX_DU,NbConstraints,N_);
  //			 Pb.add_term(COM_CONSTR_R1,QPProblem::VECTOR_DS,NbConstraints);


  //int NbStepsPreviewed = SupportStates_deq.back().StepNumber;
  //
  //			 NbConstraints = Pb.NbConstraints();
  //			 Pb.add_term(HXTRU2,QPProblem::MATRIX_DU,NbConstraints,0);
  //			 Pb.add_term(HYTRU2,QPProblem::MATRIX_DU,NbConstraints,N_);
  //			 Pb.add_term(COM_CONSTR_R2_2,QPProblem::VECTOR_DS,NbConstraints);


  //			 NbConstraints = Pb.NbConstraints();
  //			 Pb.add_term(-HXTRU2,QPProblem::MATRIX_DU,NbConstraints,0);
  //			 Pb.add_term(-HYTRU2,QPProblem::MATRIX_DU,NbConstraints,N_);
  //			 Pb.add_term(COM_CONSTR_R1_2,QPProblem::VECTOR_DS,NbConstraints);

  NbConstraints = Pb.NbConstraints();
  D5=D5-WWH;
  Pb.add_term(WWx,QPProblem::MATRIX_DU,NbConstraints,2*N_);
  Pb.add_term(WWy,QPProblem::MATRIX_DU,NbConstraints,2*N_+NbStepsPreviewed);
  Pb.add_term(-D5,QPProblem::VECTOR_DS,NbConstraints);

//  NbConstraints = Pb.NbConstraints();
//  D6=D6-WWH2;
//  Pb.add_term(WWx2,QPProblem::MATRIX_DU,NbConstraints,2*N_);
//  Pb.add_term(WWy2,QPProblem::MATRIX_DU,NbConstraints,2*N_+NbStepsPreviewed);
//  Pb.add_term(-D6,QPProblem::VECTOR_DS,NbConstraints);
//
//
//  NbConstraints = Pb.NbConstraints();
//  D7=D7-WWH2;
//  Pb.add_term(-WWx2,QPProblem::MATRIX_DU,NbConstraints,2*N_);
//  Pb.add_term(-WWy2,QPProblem::MATRIX_DU,NbConstraints,2*N_+NbStepsPreviewed);
//  Pb.add_term(D7,QPProblem::VECTOR_DS,NbConstraints);


}


void
GeneratorVelRef::build_constraints_cop(const linear_inequality_t & IneqCoP, int NbStepsPreviewed, QPProblem & Pb)
{

  int NbInequalities = IneqCoP.dc.size();
  boost_ublas::matrix<double> MM(NbInequalities,N_,false);

  // -D*U
  compute_term(MM,-1.0,IneqCoP.D.x,Robot_->DynamicsCoPJerk().U);
  Pb.add_term(MM,QPProblem::MATRIX_DU,0,0);
  compute_term(MM,-1.0,IneqCoP.D.y,Robot_->DynamicsCoPJerk().U);
  Pb.add_term(MM,QPProblem::MATRIX_DU,0,N_);

  // +D*V
  compute_term(MM,1.0,IneqCoP.D.x,Matrices_->State().V+
      Robot_->LeftFoot().Dynamics(COP).U+Robot_->RightFoot().Dynamics(COP).U);
  Pb.add_term(MM,QPProblem::MATRIX_DU,0,2*N_);
  compute_term(MM,1.0,IneqCoP.D.y,Matrices_->State().V+
      Robot_->LeftFoot().Dynamics(COP).U+Robot_->RightFoot().Dynamics(COP).U);
  Pb.add_term(MM,QPProblem::MATRIX_DU,0,2*N_+NbStepsPreviewed);

  //constant part
  // +dc
  Pb.add_term(IneqCoP.dc,QPProblem::VECTOR_DS,0);

  boost_ublas::vector<double> MV(NbInequalities,false);
  boost_ublas::matrix<double> MM2(NbInequalities,3,false);

  // -D*S_z*x
  compute_term(MM2,1.0,IneqCoP.D.x,Robot_->DynamicsCoPJerk().S);
  compute_term(MV,-1.0,MM2,Matrices_->State().CoM.x);
  Pb.add_term(MV,QPProblem::VECTOR_DS,0);
  compute_term(MM2,1.0,IneqCoP.D.x,Robot_->LeftFoot().Dynamics(COP).S);
  compute_term(MV,-1.0,MM2,Robot_->LeftFoot().State().X);
  Pb.add_term(MV,QPProblem::VECTOR_DS,0);
  compute_term(MM2,1.0,IneqCoP.D.x,Robot_->RightFoot().Dynamics(COP).S);
  compute_term(MV,-1.0,MM2,Robot_->RightFoot().State().X);
  Pb.add_term(MV,QPProblem::VECTOR_DS,0);

  compute_term(MM2,1.0,IneqCoP.D.y,Robot_->DynamicsCoPJerk().S);
  compute_term(MV,-1.0,MM2,Matrices_->State().CoM.y);
  Pb.add_term(MV,QPProblem::VECTOR_DS,0);
  compute_term(MM2,1.0,IneqCoP.D.y,Robot_->LeftFoot().Dynamics(COP).S);
  compute_term(MV,-1.0,MM2,Robot_->LeftFoot().State().Y);
  Pb.add_term(MV,QPProblem::VECTOR_DS,0);
  compute_term(MM2,1.0,IneqCoP.D.y,Robot_->RightFoot().Dynamics(COP).S);
  compute_term(MV,-1.0,MM2,Robot_->RightFoot().State().Y);
  Pb.add_term(MV,QPProblem::VECTOR_DS,0);

  // +D*Vc*FP
  compute_term(MV, Matrices_->State().SupportState.x, IneqCoP.D.x, Matrices_->State().Vc);
  Pb.add_term(MV,QPProblem::VECTOR_DS,0);
  compute_term(MV, Matrices_->State().SupportState.y, IneqCoP.D.y, Matrices_->State().Vc);
  Pb.add_term(MV,QPProblem::VECTOR_DS,0);

}


void
GeneratorVelRef::build_constraints_feet(const linear_inequality_t & IneqFeet,
				      const IntermedQPMat::state_variant_t & State,
				      int NbStepsPreviewed, QPProblem & Pb)
{

  const int & NbConstraints = IneqFeet.dc.size();

  boost_ublas::matrix<double> MM(NbConstraints,NbStepsPreviewed,false);

  // -D*V_f
  compute_term(MM,-1.0,IneqFeet.D.x,State.V_f);
  Pb.add_term(MM,QPProblem::MATRIX_DU,4*N_,2*N_);
  compute_term(MM,-1.0,IneqFeet.D.y,State.V_f);
  Pb.add_term(MM,QPProblem::MATRIX_DU,4*N_,2*N_+NbStepsPreviewed);

  // +dc
  Pb.add_term(IneqFeet.dc,QPProblem::VECTOR_DS,4*N_);

  // +D*Vc_f*FP
  boost_ublas::vector<double> MV(NbConstraints*NbStepsPreviewed,false);
  compute_term(MV, State.SupportState.x, IneqFeet.D.x, State.Vc_f);
  Pb.add_term(MV,QPProblem::VECTOR_DS,4*N_);
  compute_term(MV, State.SupportState.y, IneqFeet.D.y, State.Vc_f);
  Pb.add_term(MV,QPProblem::VECTOR_DS,4*N_);

}


void
GeneratorVelRef::build_constraints( QPProblem & Pb,
				  RelativeFeetInequalities * RFI,
				  const std::deque< FootAbsolutePosition> & LeftFootPositions_deq,
				  const std::deque<FootAbsolutePosition> & RightFootPositions_deq,
				  const std::deque<support_state_t> & SupportStates_deq,
				  const std::deque<double> & PreviewedSupportAngles_deq )
{

  //CoP constraints
  linear_inequality_t & IneqCoP = Matrices_->Inequalities(IntermedQPMat::INEQ_COP);
  build_inequalities_cop( IneqCoP, RFI,
		       LeftFootPositions_deq, RightFootPositions_deq,
		       SupportStates_deq, PreviewedSupportAngles_deq );

  int NbStepsPreviewed = SupportStates_deq.back().StepNumber;
  build_constraints_cop( IneqCoP, NbStepsPreviewed, Pb );

  //Feet constraints
  linear_inequality_t & IneqFeet = Matrices_->Inequalities(IntermedQPMat::INEQ_FEET);
  build_inequalities_feet( IneqFeet, RFI,
			LeftFootPositions_deq, RightFootPositions_deq,
			SupportStates_deq, PreviewedSupportAngles_deq );

  build_constraints_feet( IneqFeet, Matrices_->State(), NbStepsPreviewed, Pb );

}


void 
GeneratorVelRef::build_invariant_part( QPProblem & Pb )
{

  const RigidBody & CoM = Robot_->CoM();
  boost_ublas::matrix<double> weightMTM(N_,N_,false);

  //Constant terms in the Hessian
  // +a*U'*U
  const IntermedQPMat::objective_variant_t & Jerk = Matrices_->Objective(IntermedQPMat::JERK_MIN);
  const linear_dynamics_t & JerkDynamics = CoM.Dynamics( JERK );
  compute_term( weightMTM, Jerk.weight, JerkDynamics.UT, JerkDynamics.U );
  Pb.add_term( weightMTM, QPProblem::MATRIX_Q, 0, 0 );
  Pb.add_term( weightMTM, QPProblem::MATRIX_Q, N_, N_ );

  // +a*U'*U
  const IntermedQPMat::objective_variant_t & InstVel = Matrices_->Objective(IntermedQPMat::INSTANT_VELOCITY);
  const linear_dynamics_t & VelDynamics = CoM.Dynamics( VELOCITY );
  compute_term(weightMTM, InstVel.weight, VelDynamics.UT, VelDynamics.U);
  Pb.add_term(weightMTM, QPProblem::MATRIX_Q, 0, 0);
  Pb.add_term(weightMTM, QPProblem::MATRIX_Q, N_, N_);

  // +a*U'*U
  const IntermedQPMat::objective_variant_t & COPCent = Matrices_->Objective(IntermedQPMat::COP_CENTERING);
  compute_term(weightMTM, COPCent.weight, Robot_->DynamicsCoPJerk().UT, Robot_->DynamicsCoPJerk().U);
  Pb.add_term(weightMTM, QPProblem::MATRIX_Q, 0, 0);
  Pb.add_term(weightMTM, QPProblem::MATRIX_Q, N_, N_);

}


void
GeneratorVelRef::update_problem( QPProblem & Pb, const std::deque<support_state_t> & SupportStates_deq )
{

  Pb.clear(QPProblem::VECTOR_D);

  //Intermediate vector
  boost_ublas::vector<double> MV(N_);
  MV.clear();
  // Final scaled products that are added to the QP
  MAL_MATRIX(weightMTM,double);
  MAL_VECTOR(weightMTV,double);

  const int NbStepsPreviewed = SupportStates_deq[N_].StepNumber;

  const IntermedQPMat::state_variant_t & State = Matrices_->State();
  const RigidBody & CoM = Robot_->CoM();

  // Instant velocity terms
  const IntermedQPMat::objective_variant_t & InstVel = Matrices_->Objective(IntermedQPMat::INSTANT_VELOCITY);
  const linear_dynamics_t & VelDynamics = CoM.Dynamics( VELOCITY );
  // Linear part
  // +a*U'*S*x
  compute_term(weightMTV, InstVel.weight, VelDynamics.UT, MV, VelDynamics.S, State.CoM.x);
  Pb.add_term(weightMTV, QPProblem::VECTOR_D, 0);
  compute_term(weightMTV, InstVel.weight, VelDynamics.UT, MV, VelDynamics.S, State.CoM.y);
  Pb.add_term(weightMTV, QPProblem::VECTOR_D, N_);
  // +a*U'*ref
  compute_term(weightMTV, -InstVel.weight, VelDynamics.UT, State.Ref.Global.X);
  Pb.add_term(weightMTV, QPProblem::VECTOR_D, 0);
  compute_term(weightMTV, -InstVel.weight, VelDynamics.UT, State.Ref.Global.Y);
  Pb.add_term(weightMTV, QPProblem::VECTOR_D, N_);

  // COP - centering terms
  const IntermedQPMat::objective_variant_t & COPCent = Matrices_->Objective(IntermedQPMat::COP_CENTERING);
  const linear_dynamics_t & CoPDynamics = Robot_->DynamicsCoPJerk( );
  const linear_dynamics_t & LFCoP = Robot_->LeftFoot().Dynamics(COP);
  const linear_dynamics_t & RFCoP = Robot_->RightFoot().Dynamics(COP);
  // Hessian
  // -a*U'*V
  compute_term(weightMTM, -COPCent.weight, CoPDynamics.UT, State.V);
  Pb.add_term(weightMTM, QPProblem::MATRIX_Q, 0, 2*N_);
  Pb.add_term(weightMTM, QPProblem::MATRIX_Q, N_, 2*N_+NbStepsPreviewed);
  // -a*V*U
  compute_term(weightMTM, -COPCent.weight, State.VT, CoPDynamics.U);
  Pb.add_term(weightMTM, QPProblem::MATRIX_Q, 2*N_, 0);
  Pb.add_term(weightMTM, QPProblem::MATRIX_Q, 2*N_+NbStepsPreviewed, N_);
  //+a*V'*V
  compute_term(weightMTM, COPCent.weight, State.VT, State.V);
  Pb.add_term(weightMTM, QPProblem::MATRIX_Q, 2*N_, 2*N_);
  Pb.add_term(weightMTM, QPProblem::MATRIX_Q, 2*N_+NbStepsPreviewed, 2*N_+NbStepsPreviewed);

  //Linear part
  // -a*V'*S*x
  compute_term(weightMTV, -COPCent.weight, State.VT, MV, CoPDynamics.S, State.CoM.x);
  Pb.add_term(weightMTV, QPProblem::VECTOR_D, 2*N_);
  compute_term(weightMTV, -COPCent.weight, State.VT, MV, CoPDynamics.S, State.CoM.y);
  Pb.add_term(weightMTV, QPProblem::VECTOR_D, 2*N_+NbStepsPreviewed);
  // +a*V'*Vc*x
  compute_term(weightMTV, COPCent.weight, State.VT, State.Vc, State.SupportState.x);
  Pb.add_term(weightMTV, QPProblem::VECTOR_D, 2*N_);
  compute_term(weightMTV, COPCent.weight, State.VT, State.Vc, State.SupportState.y);
  Pb.add_term(weightMTV, QPProblem::VECTOR_D, 2*N_+NbStepsPreviewed);

}


void
GeneratorVelRef::compute_term(MAL_MATRIX (&weightMM, double), double weight,
			     const MAL_MATRIX (&M1, double), const MAL_MATRIX (&M2, double))
{
  weightMM = weight*MAL_RET_A_by_B(M1,M2);
}


void
GeneratorVelRef::compute_term(MAL_MATRIX (&MM, double),
                             const MAL_MATRIX (&M1, double), const MAL_MATRIX (&M2, double))
{
  MM = MAL_RET_A_by_B(M1,M2);
}

void
GeneratorVelRef::compute_term(MAL_VECTOR (&weightMV, double), double weight,
			     const MAL_MATRIX (&M, double), const MAL_VECTOR (&V, double))
{
  weightMV = weight*MAL_RET_A_by_B(M,V);
}


void
GeneratorVelRef::compute_term(MAL_VECTOR (&weightMV, double),
			     double weight, const MAL_MATRIX (&M, double),
			     const MAL_VECTOR (&V, double), double scalar)
{
  weightMV = weight*scalar*MAL_RET_A_by_B(M,V);
}


void
GeneratorVelRef::compute_term(MAL_VECTOR (&weightMV, double),
			     double weight, const MAL_MATRIX (&M1, double), MAL_VECTOR (&V1, double),
			     const MAL_MATRIX (&M2, double), const MAL_VECTOR (&V2, double))
{
  V1 = MAL_RET_A_by_B(M2,V2);
  weightMV = weight*MAL_RET_A_by_B(M1,V1);
}

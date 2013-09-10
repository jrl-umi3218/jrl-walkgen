/*
 * Copyright 2010,
 *
 * Andrei Herdt
 * Francois Keith
 * Olivier Stasse
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

/*! This object generate all the values for the foot trajectories,
  and the desired ZMP based on a sequence of steps following a QP
  formulation and a new QP solver as proposed by Herdt Advanced Robotics 2010.

  Andrei Herdt,
  Olivier Stasse,
 */

#include "portability/gettimeofday.hh"

#ifdef WIN32
# include <Windows.h>
#endif /* WIN32 */

#include <time.h>

#include <iostream>
#include <fstream>

#include <Mathematics/qld.hh>
#include <ZMPRefTrajectoryGeneration/ZMPVelocityReferencedQP.hh>

#include <Debug.hh>
using namespace std;
using namespace PatternGeneratorJRL;



ZMPVelocityReferencedQP::ZMPVelocityReferencedQP(SimplePluginManager *SPM,
    string DataFile, CjrlHumanoidDynamicRobot *aHS) :
    ZMPRefTrajectoryGeneration(SPM),
    Robot_(0),SupportFSM_(0),OrientPrw_(0),VRQPGenerator_(0),IntermedData_(0),RFI_(0),Problem_(),Solution_()
{

  m_Pu = 0;
  m_FullDebug = -10;
  m_FastFormulationMode = QLD;

  m_QP_T = 0.1;
  m_QP_N = 16;

  m_SamplingPeriod = 0.005;
  PerturbationOccured_ = false;
  UpperTimeLimitToUpdate_ = 0.0;
  RobotMass_ = aHS->mass();
  Solution_.useWarmStart=false;

  // Create and initialize online interpolation of feet trajectories
  RFI_ = new RelativeFeetInequalities( SPM,aHS );

  // Create and initialize the finite state machine for support sequences
  SupportFSM_ = new SupportFSM();
  SupportFSM_->StepPeriod( 0.8 );
  SupportFSM_->DSPeriod( 1e9 );
  SupportFSM_->DSSSPeriod( 0.8 );
  SupportFSM_->NbStepsSSDS( 2 );
  SupportFSM_->SamplingPeriod( QP_T_ );

  // Create and initialize preview of orientations
  OrientPrw_ = new OrientationsPreview( aHS->rootJoint() );
  OrientPrw_->SamplingPeriod( QP_T_ );
  OrientPrw_->NbSamplingsPreviewed( QP_N_ );
  OrientPrw_->SSLength( SupportFSM_->StepPeriod() );
  COMState CurrentTrunkState;
  OrientPrw_->CurrentTrunkState( CurrentTrunkState );

  // Initialize  the 2D LIPM
  CoM_.SetSimulationControlPeriod( QP_T_ );
  CoM_.SetRobotControlPeriod( m_SamplingPeriod );
  CoM_.InitializeSystem();

  // Create and initialize simplified robot model
  Robot_ = new RigidBodySystem( SPM, aHS, SupportFSM_ );
  Robot_->Mass( aHS->mass() );
  Robot_->LeftFoot().Mass( 0.0 );
  Robot_->RightFoot().Mass( 0.0 );
  Robot_->NbSamplingsPreviewed( QP_N_ );
  Robot_->SamplingPeriodSim( QP_T_ );
  Robot_->SamplingPeriodAct( m_SamplingPeriod );
  Robot_->CoMHeight( 0.814 );
  Robot_->multiBody(false);
  Robot_->initialize( );


  IntermedData_ = new IntermedQPMat();

  VRQPGenerator_ = new GeneratorVelRef( SPM, IntermedData_, Robot_, RFI_ );
  VRQPGenerator_->NbPrwSamplings( QP_N_ );
  VRQPGenerator_->SamplingPeriodPreview( QP_T_ );
  VRQPGenerator_->SamplingPeriodControl( m_SamplingPeriod );
  VRQPGenerator_->ComHeight( 0.814 );
  VRQPGenerator_->initialize_matrices();
  VRQPGenerator_->Ponderation( 1.0, INSTANT_VELOCITY );
  VRQPGenerator_->Ponderation( 0.000001, COP_CENTERING );
  VRQPGenerator_->Ponderation( 0.00001, JERK_MIN );

  // Register method to handle
  const unsigned int NbMethods = 3;
  string aMethodName[NbMethods] =
      {":previewcontroltime",
          ":numberstepsbeforestop",
          ":stoppg"};

  for(unsigned int i=0;i<NbMethods;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
    }
  
  // Debug point.
  debugConstructor();

  //Feet distance in the DS phase
  m_FeetDistanceDS = 0.2;

  m_PerturbationOccured = false;
}


ZMPVelocityReferencedQP::~ZMPVelocityReferencedQP()
{

  if (m_2DLIPM!=0)
    delete m_2DLIPM;

  if (m_SupportFSM!=0)
    delete m_SupportFSM;

  if (RFI_!=0)
    delete RFI_;

  if (m_OP!=0)
    delete m_OP;

  if (m_PLDPSolverHerdt!=0)
    delete m_PLDPSolverHerdt;

  if (m_Pu!=0)
    delete [] m_Pu ;

  if (Robot_!=0)
    delete Robot_;

  if (IntermedData_!=0)
    delete IntermedData_;

}


void
ZMPVelocityReferencedQP::setCoMPerturbationForce(istringstream &strm)
{

  MAL_VECTOR_RESIZE(PerturbationAcceleration_,6);

  strm >> PerturbationAcceleration_(2);
  strm >> PerturbationAcceleration_(5);
  PerturbationAcceleration_(2) = PerturbationAcceleration_(2)/RobotMass_;
  PerturbationAcceleration_(5) = PerturbationAcceleration_(5)/RobotMass_;
  PerturbationOccured_ = true;

}


void
ZMPVelocityReferencedQP::setCoMPerturbationForce(double x,double y)
{

  MAL_VECTOR_RESIZE(PerturbationAcceleration_,6);

  m_PerturbationAcceleration(2) = x/m_RobotMass;
  m_PerturbationAcceleration(5) = y/m_RobotMass;
  m_PerturbationOccured = true;

}

void ZMPVelocityReferencedQP::interpolateFeet(deque<FootAbsolutePosition> &,
					      deque<FootAbsolutePosition> &)
{

  printf("To be implemented \n");
}

int ZMPVelocityReferencedQP::InitializeMatrixPbConstants()
{
  MAL_MATRIX_RESIZE(m_PPu,2*m_QP_N,2*m_QP_N);
  MAL_MATRIX_RESIZE(m_PZu,m_QP_N,m_QP_N);
  MAL_MATRIX_RESIZE(m_VPu,2*m_QP_N,2*m_QP_N);
  MAL_MATRIX_RESIZE(m_PPx,2*m_QP_N,6);
  MAL_MATRIX_RESIZE(m_PZx,m_QP_N,3);
  MAL_MATRIX_RESIZE(m_VPx,2*m_QP_N,6);

  for( int i=0;i<m_QP_N;i++)
    {
      // Compute VPx and PPx
      m_VPx(i,0)   = 0.0;   m_VPx(i,1) =     1.0; m_VPx(i,2)   = (i+1)*m_QP_T;
      m_VPx(i,3)   = 0.0;   m_VPx(i,4) =     0.0; m_VPx(i,5)   = 0.0;
      m_VPx(i+m_QP_N,0) = 0.0;   m_VPx(i+m_QP_N,1) =   0.0; m_VPx(i+m_QP_N,2) = 0.0;
      m_VPx(i+m_QP_N,3) = 0.0;   m_VPx(i+m_QP_N,4) =   1.0; m_VPx(i+m_QP_N,5) = (i+1)*m_QP_T;

      m_PPx(i,0) = 1.0; m_PPx(i,1)     = (i+1)*m_QP_T; m_PPx(i,2) = (i+1)*(i+1)*m_QP_T*m_QP_T*0.5;
      m_PPx(i,3) = 0.0; m_PPx(i,4)     =       0; m_PPx(i,5) = 0.;
      m_PPx(i+m_QP_N,0) = 0.0; m_PPx(i+m_QP_N,1) =     0.0; m_PPx(i+m_QP_N,2) = 0.0;
      m_PPx(i+m_QP_N,3) = 1.0; m_PPx(i+m_QP_N,4) = (i+1)*m_QP_T; m_PPx(i+m_QP_N,5) = (i+1)*(i+1)*m_QP_T*m_QP_T*0.5;

      //TODO: + or - m_ComHeight/9.81
      m_PZx(i,0) = 1.0; m_PZx(i,1)     = (i+1)*m_QP_T; m_PZx(i,2) = (i+1)*(i+1)*m_QP_T*m_QP_T*0.5-m_ComHeight/9.81;
      //m_PZx(i,3) = 0.0; m_PZx(i,4)     =       0; m_PZx(i,5) = 0.;
      //		m_PZx(i+m_QP_N,0) = 0.0; m_PZx(i+m_QP_N,1) =     0.0; m_PZx(i+m_QP_N,2) = 0.0;
      //		m_PZx(i+m_QP_N,3) = 1.0; m_PZx(i+m_QP_N,4) = (i+1)*m_QP_T; m_PZx(i+m_QP_N,5) = (i+1)*(i+1)*m_QP_T*m_QP_T*0.5+m_ComHeight/9.81;


      for( int j=0;j<m_QP_N;j++)
	{
	  m_PPu(i,j)=0;
	  m_PZu(i,j)=0;

	  if (j<=i)
	    {

	      m_VPu(i,j)= (2*(i-j)+1)*m_QP_T*m_QP_T*0.5 ;
	      m_VPu(i+m_QP_N,j+m_QP_N)= (2*(i-j)+1)*m_QP_T*m_QP_T*0.5 ;
	      m_VPu(i,j+m_QP_N)=0.0;
	      m_VPu(i+m_QP_N,j)=0.0;


	      m_PPu(i,j)= (1 + 3*(i-j) + 3*(i-j)*(i-j)) * m_QP_T*m_QP_T*m_QP_T/6.0;
	      m_PPu(i+m_QP_N,j+m_QP_N)= (1 + 3*(i-j) + 3*(i-j)*(i-j)) * m_QP_T*m_QP_T*m_QP_T/6.0;
	      m_PPu(i,j+m_QP_N)=0.0;
	      m_PPu(i+m_QP_N,j)=0.0;

	      m_PZu(i,j)= (1 + 3*(i-j) + 3*(i-j)*(i-j)) * m_QP_T*m_QP_T*m_QP_T/6.0 - m_QP_T*m_ComHeight/9.81;
	      //m_PZu(i+m_QP_N,j+m_QP_N)= (1 + 3*(i-j) + 3*(i-j)*(i-j)) * m_QP_T*m_QP_T*m_QP_T/6.0 + m_QP_T*m_ComHeight/9.81;
	      //m_PZu(i,j+m_QP_N)=0.0;
	      //m_PZu(i+m_QP_N,j)=0.0;
	    }
	  else
	    {

	      m_VPu(i,j) = 0.0;
	      m_VPu(i+m_QP_N,j+m_QP_N)=0.0;
	      m_VPu(i,j+m_QP_N)=0.0;
	      m_VPu(i+m_QP_N,j)=0.0;

	      m_PPu(i,j) = 0.0;
	      m_PPu(i+m_QP_N,j+m_QP_N)=0.0;
	      m_PPu(i,j+m_QP_N)=0.0;
	      m_PPu(i+m_QP_N,j)=0.0;

	      m_PZu(i,j) = 0.0;
	      //m_PZu(i+m_QP_N,j+m_QP_N)=0.0;
	      //m_PZu(i,j+m_QP_N)=0.0;
	      //m_PZu(i+m_QP_N,j)=0.0;
	    }

	}
    }

  // Build m_Px.
  MAL_MATRIX_RESIZE(m_Px,m_QP_N,3);

  for( int li=0;li<m_QP_N;li++)
    {
      m_Px(li,0) = 1.0;
      m_Px(li,1) = (double)(1.0+li)*m_QP_T;
      m_Px(li,2) = (li+1.0)*(li+1.0)*m_QP_T*m_QP_T*0.5-m_ComHeight/9.81;
    }
  if (m_FullDebug>2)
    {
      ofstream aof;
      aof.open("/tmp/VPx.dat");
      aof << m_VPx;
      aof.close();

      aof.open("/tmp/m_PPx.dat");
      aof << m_PPx;
      aof.close();

      aof.open("/tmp/VPu.dat");
      aof << m_VPu;
      aof.close();

      aof.open("/tmp/PPu.dat");
      aof << m_PPu;
      aof.close();

      aof.open("/tmp/PZu.dat");
      aof << m_PZu;
      aof.close();
    }

  return 0;
}

int ZMPVelocityReferencedQP::BuildingConstantPartOfTheObjectiveFunction()
{

  MAL_MATRIX(OptA,double);

  //  OptA = Id + alpha * VPu.Transpose() * VPu + beta * PPu.Transpose() * PPu;
  MAL_MATRIX(lterm1,double);
  lterm1 = MAL_RET_TRANSPOSE(m_PPu);
  lterm1 = MAL_RET_A_by_B(lterm1, m_PPu);
  lterm1 = m_Beta * lterm1;

  MAL_MATRIX(lterm2,double);
  lterm2 = MAL_RET_TRANSPOSE(m_VPu);
  lterm2 = MAL_RET_A_by_B(lterm2,m_VPu);
  // lterm2 = m_Alpha * lterm2;//TODO:: original pb
  lterm2 = m_Beta*lterm2;

  MAL_MATRIX_RESIZE(OptA,
		    MAL_MATRIX_NB_ROWS(lterm2),
		    MAL_MATRIX_NB_COLS(lterm2));
  MAL_MATRIX_SET_IDENTITY(OptA);
  OptA = m_Alpha*OptA;


  // OptA = OptA + lterm1 + lterm2;//TODO:: original problem
  OptA = OptA + lterm2;
  
  m_OptA = OptA;

  // Initialization of the matrice regarding the quadratic
  // part of the objective function.
  //TODO:: size of Q is 3*Nx3*N which means that there is place for N/2 feet variables

  /*! Compute constants of the linear part of the objective function. */
  lterm1 = MAL_RET_TRANSPOSE(m_PPu);
  lterm1 = MAL_RET_A_by_B(lterm1,m_PPx);
  m_OptB = MAL_RET_TRANSPOSE(m_VPu);
  m_OptB = MAL_RET_A_by_B(m_OptB,m_VPx);
  m_OptB = m_Alpha * m_OptB;
  m_OptB = m_OptB + m_Beta * lterm1;

  m_OptC = MAL_RET_TRANSPOSE(m_PPu);
  m_OptC = m_Beta * m_OptC;


  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"/tmp/OptB.dat");
      aof.open(Buffer,ofstream::out);
      for( unsigned int i=0;i<MAL_MATRIX_NB_ROWS(m_OptB);i++)
	{
	  for( unsigned int j=0;j<MAL_MATRIX_NB_COLS(m_OptB)-1;j++)
	    aof << m_OptB(i,j) << " ";
	  aof << m_OptB(i,MAL_MATRIX_NB_COLS(m_OptB)-1);
	  aof << endl;
	}
      aof.close();

    }

  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"/tmp/OptC.dat");
      aof.open(Buffer,ofstream::out);
      for( unsigned int i=0;i<MAL_MATRIX_NB_ROWS(m_OptC);i++)
	{
	  for( unsigned int j=0;j<MAL_MATRIX_NB_COLS(m_OptC)-1;j++)
	    aof << m_OptC(i,j) << " ";
	  aof << m_OptC(i,MAL_MATRIX_NB_COLS(m_OptC)-1);
	  aof << endl;
	}
      aof.close();

    }

  return 0;
}

int ZMPVelocityReferencedQP::BuildingConstantPartOfConstraintMatrices()
{
  if (m_Pu==0)
    m_Pu = new double[m_QP_N*m_QP_N];

  double * lInterPu=0;
  double * ptPu=0;

  if ((m_FastFormulationMode==QLDANDLQ)||
      (m_FastFormulationMode==PLDP))
    {
      lInterPu = new double[m_QP_N*m_QP_N];
      memset(lInterPu,0,m_QP_N*m_QP_N*sizeof(double));
      ptPu = lInterPu;
    }
  else
    ptPu = m_Pu;

  memset(m_Pu,0,m_QP_N*m_QP_N*sizeof(double));

  // Recursive multiplication of the system is applied.
  // we keep the transpose form, i.e. Pu'.
  for(int i=0;i<m_QP_N;i++)
    {

      for(int k=0;k<=i;k++)
	{
	  ptPu[k*m_QP_N+i] =
	    ((1+3*(i-k)+3*(i-k)*(i-k))*m_QP_T*m_QP_T*m_QP_T/6.0 - m_QP_T * m_ComHeight/9.81);
	}
    }

  // Consider QLDANDLQ formulation.
  if ((m_FastFormulationMode==QLDANDLQ) ||
      (m_FastFormulationMode==PLDPHerdt))
    {
      // Premultiplication by LQ-1
      // Indeed we have to provide qld transpose matrix,
      // therefore instead of providing D*Pu*iLQ (IROS 2008 p 28)
      // we provide its transpose:
      // (D*Pu*iLQ')' = iLQ*Pu'*D'
      // So here we compute iLQ*Pu'
      // Be careful with the two stages resolution.
      for(int i=0;i<m_QP_N;i++)
	{
	  for(int j=0;j<m_QP_N;j++)
	    {
	      m_Pu[i*m_QP_N+j] = 0;
	      for(int k=0;k<m_QP_N;k++)
		{
		  m_Pu[i*m_QP_N+j] += m_iLQ(i,k) * ptPu[k*m_QP_N+j];
		}
	    }
	}

      if (m_FastFormulationMode==PLDPHerdt)
	{
	  MAL_MATRIX_DIM(m_mal_Pu,double,m_QP_N,m_QP_N);
	  for(int j=0;j<m_QP_N;j++)
	    for(int k=0;k<m_QP_N;k++)
	      m_mal_Pu(j,k) = m_Pu[j*m_QP_N+k];
	  MAL_INVERSE(m_mal_Pu, m_iPu, double);
	}
    }

  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"/tmp/PuCst.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<m_QP_N;i++)
	{
	  for( int j=0;j<m_QP_N;j++)
	    aof << m_Pu[j+i*m_QP_N] << " " ;
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"/tmp/tmpPuCst.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<m_QP_N;i++)
	{
	  for( int j=0;j<m_QP_N;j++)
	    aof << ptPu[j+i*m_QP_N] << " " ;
	  aof << endl;
	}
      aof.close();

      if ((m_FastFormulationMode==QLDANDLQ) ||
	  (m_FastFormulationMode==PLDP))
	{
	  sprintf(Buffer,"/tmp/tmpiLQ.dat");
	  aof.open(Buffer,ofstream::out);
	  for( int i=0;i<m_QP_N;i++)
	    {
	      for( int j=0;j<m_QP_N;j++)
		aof << m_iLQ(i,j) << " " ;
	      aof << endl;
	    }
	  aof.close();
	}

    }

  delete [] lInterPu;
  return 0;
}


int ZMPVelocityReferencedQP::buildConstraintMatricesPLDPHerdt()
{
  m_Pu = new double[2*(m_QP_N+m_PrwSupport.StepNumber)*2*(m_QP_N+m_PrwSupport.StepNumber)];

  double * lInterPu=0;
  double * ptPu=0;

  if ((m_FastFormulationMode==QLDANDLQ)||
      (m_FastFormulationMode==PLDPHerdt))
    {
      lInterPu = new double[2*(m_QP_N+m_PrwSupport.StepNumber)*2*(m_QP_N+m_PrwSupport.StepNumber)];
      memset(lInterPu,0,2*(m_QP_N+m_PrwSupport.StepNumber)*2*(m_QP_N+m_PrwSupport.StepNumber)*sizeof(double));
      ptPu = lInterPu;
    }
  else
    ptPu = m_Pu;


  memset(m_Pu,0,2*(m_QP_N+m_PrwSupport.StepNumber)*2*(m_QP_N+m_PrwSupport.StepNumber)*sizeof(double));


  // Recursive multiplication of the system is applied.
  // we keep the transpose form, i.e. Pu'.
  for(int i=0;i<m_QP_N;i++)
    {
      for(int k=0;k<=i;k++)
	{
	  ptPu[k*2*(m_QP_N+m_PrwSupport.StepNumber)+i] =
	    ((1+3*(i-k)+3*(i-k)*(i-k))*m_QP_T*m_QP_T*m_QP_T/6.0 - m_QP_T * m_ComHeight/9.81);
	  ptPu[(k+m_QP_N)*2*(m_QP_N+m_PrwSupport.StepNumber)+m_QP_N+i] =
	    ((1+3*(i-k)+3*(i-k)*(i-k))*m_QP_T*m_QP_T*m_QP_T/6.0 - m_QP_T * m_ComHeight/9.81);
	}
    }
  for(int i=0;i<m_PrwSupport.StepNumber;i++)
    {
      ptPu[(2*m_QP_N+i)*2*(m_QP_N+m_PrwSupport.StepNumber)+2*m_QP_N+i] = 1.0;
      ptPu[(2*m_QP_N+m_PrwSupport.StepNumber+i)*2*(m_QP_N+m_PrwSupport.StepNumber)+2*m_QP_N+m_PrwSupport.StepNumber+i] = 1.0;
    }

  if ((m_FastFormulationMode==QLDANDLQ) ||
      (m_FastFormulationMode==PLDPHerdt))
    {
      // Premultiplication by LQ-1
      // Indeed we have to provide qld transpose matrix,
      // therefore instead of providing D*Pu*iLQ (IROS 2008 p 28)
      // we provide its transpose:
      // (D*Pu*iLQ')' = iLQ*Pu'*D'
      // So here we compute iLQ*Pu'
      // Be careful with the two stages resolution.
      for(int i=0;i<(2*m_QP_N+2*m_PrwSupport.StepNumber);i++)
	{
	  for(int j=0;j<(2*m_QP_N+2*m_PrwSupport.StepNumber);j++)
	    {
	      m_Pu[i*2*(m_QP_N+m_PrwSupport.StepNumber)+j] = 0;
	      for(int k=0;k<2*(m_QP_N+m_PrwSupport.StepNumber);k++)
		{
		  m_Pu[i*2*(m_QP_N+m_PrwSupport.StepNumber)+j] += m_iLQ(i,k) * ptPu[k*2*(m_QP_N+m_PrwSupport.StepNumber)+j];
		}
	    }
	}

      if (m_FastFormulationMode==PLDPHerdt)
	{
	  MAL_MATRIX_DIM(m_mal_Pu,double,2*(m_QP_N+m_PrwSupport.StepNumber),2*(m_QP_N+m_PrwSupport.StepNumber));
	  for(int j=0;j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
	    for(int k=0;k<2*(m_QP_N+m_PrwSupport.StepNumber);k++)
	      m_mal_Pu(j,k) = m_Pu[j*2*(m_QP_N+m_PrwSupport.StepNumber)+k];
	  MAL_INVERSE(m_mal_Pu, m_iPu, double);
	}
    }

  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"/tmp/PuVar.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<(2*m_QP_N+2*m_PrwSupport.StepNumber);i++)
	{
	  for( int j=0;j<(2*m_QP_N+2*m_PrwSupport.StepNumber);j++)
	    aof << m_Pu[j+i*(2*m_QP_N+2*m_PrwSupport.StepNumber)] << " " ;
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"/tmp/tmpPuVar.dat");
      aof.open(Buffer,ofstream::out);
      for(int i=0;i<(2*m_QP_N+2*m_PrwSupport.StepNumber);i++)
	{
	  for(int j=0;j<(2*m_QP_N+2*m_PrwSupport.StepNumber);j++)
	    aof << ptPu[j+i*(2*m_QP_N+2*m_PrwSupport.StepNumber)] << " " ;
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"/tmp/tmpPuVar.dat");
      aof.open(Buffer,ofstream::out);
      for(int i=0;i<(2*m_QP_N+2*m_PrwSupport.StepNumber);i++)
	{
	  for(int j=0;j<(2*m_QP_N+2*m_PrwSupport.StepNumber);j++)
	    aof << ptPu[j+i*(2*m_QP_N+2*m_PrwSupport.StepNumber)] << " " ;
	  aof << endl;
	}
      aof.close();

      if ((m_FastFormulationMode==QLDANDLQ) ||
	  (m_FastFormulationMode==PLDPHerdt))
	{
	  sprintf(Buffer,"/tmp/tmpiLQVar.dat");
	  aof.open(Buffer,ofstream::out);
	  for( int i=0;i<(2*m_QP_N+2*m_PrwSupport.StepNumber);i++)
	    {
	      for( int j=0;j<(2*m_QP_N+2*m_PrwSupport.StepNumber);j++)
		aof << m_iLQ(i,j) << " " ;
	      aof << endl;
	    }
	  aof.close();
	}

    }

  delete [] lInterPu;
  return 0;
}

void ZMPVelocityReferencedQP::initFeet()
{

  //Define the initial coordinates of the feet
  //This might be done when creating SupportState
  SupportFeet_t aSFLeft;
  SupportFeet_t aSFRight;
  aSFLeft.x = 0.0;
  aSFLeft.y = 0.1;//TODO:
  aSFLeft.theta = 0.0;
  aSFLeft.StartTime = 0.0;
  aSFLeft.SupportFoot = 1;
  aSFRight.x = 0.0;
  aSFRight.y = -0.1;//TODO:
  aSFRight.theta = 0.0;
  aSFRight.StartTime = 0.0;
  aSFRight.SupportFoot = -1;

  QueueOfSupportFeet.push_back(aSFLeft);
  QueueOfSupportFeet.push_back(aSFRight);

}


int ZMPVelocityReferencedQP::InitConstants()
{
  int r;
  if ((r=InitializeMatrixPbConstants())<0)
    return r;
  if(m_FastFormulationMode != PLDPHerdt)
    {
      if ((r=BuildingConstantPartOfTheObjectiveFunction())<0)
	return r;

      if ((r=BuildingConstantPartOfConstraintMatrices())<0)
	return r;
    }

  return 0;
}

void ZMPVelocityReferencedQP::SetAlpha(const double &anAlpha)
{
  m_Alpha = anAlpha;
}

const double & ZMPVelocityReferencedQP::GetAlpha() const
{
  return m_Alpha;
}

void ZMPVelocityReferencedQP::SetBeta(const double &anAlpha)
{
  m_Beta = anAlpha;
}

const double & ZMPVelocityReferencedQP::GetBeta() const
{
  return m_Beta;
}



//------------------new functions---
//
//
//----------------------------------

int ZMPVelocityReferencedQP::dumpProblem(MAL_VECTOR(& xk,double),
					 double Time)
{
  ofstream aof;

  char Buffer[1024];
  sprintf(Buffer,"/tmp/ProblemFF_%f.dat",Time);
  aof.open(Buffer,ofstream::out);

  // Dumping D.
  aof << endl <<"D: "<< "RefVel: "<< RefVel.x << " "<< RefVel.y << "xk: "<<xk<< endl;
  m_Pb.dumpProblem(aof);
  
  return 0;
}

int ZMPVelocityReferencedQP::buildConstraintMatrices(double * &,
						     double * &DU,
						     double StartingTime,
						     deque<LinearConstraintInequalityFreeFeet_t> &
						     QueueOfLConstraintInequalitiesFreeFeet,
						     deque<LinearConstraintInequalityFreeFeet_t> &
						     QueueOfFeetPosInequalities,
						     deque<SupportFeet_t> &
						     QueueOfSupportFeet,
						     double ,
						     int NbOfConstraints,
						     MAL_VECTOR(& xk,double))
{

  // Discretize the problem.
  ODEBUG(" N:" << m_QP_N << " T: " << T);

  if (m_FullDebug>2)
   {
      char Buffer[1024];
      sprintf(Buffer,"/tmp/PXD_%f.dat", StartingTime);
      RESETDEBUG4(Buffer);
      ODEBUG6("xk:" << xk << " Starting time: " <<StartingTime ,Buffer );
      char Buffer2[1024];
      sprintf(Buffer2,"/tmp/PXxD_%f.dat", StartingTime);
      RESETDEBUG4(Buffer2);

      char Buffer3[1024];
      sprintf(Buffer3,"/tmp/PXyD_%f.dat", StartingTime);
      RESETDEBUG4(Buffer3);

      RESETDEBUG6("/tmp/FFP.dat");
    }

  //Current support foot
  deque<LinearConstraintInequalityFreeFeet_t>::iterator LCIFF_it;
  deque<SupportFeet_t>::iterator CurSF_it;
  CurSF_it = QueueOfSupportFeet.end();
  CurSF_it--;
  while(CurSF_it->SupportFoot!=m_CurrentSupport.Foot)
    CurSF_it--;

  LCIFF_it = QueueOfLConstraintInequalitiesFreeFeet.begin();


  double FFPx, FFPy;

  int IndexConstraint = 0;
  ODEBUG("Starting Matrix to build the constraints. ");
  ODEBUG((LCIFF_it)->D );
  //ZMP constraints
  for( int i=0;i<m_QP_N;i++)
    {
      if(LCIFF_it->StepNumber==0)
	{//c'est pas bon ca
	  FFPx = CurSF_it->x;
	  FFPy = CurSF_it->y;
	}
      else
	{
	  FFPx = 0.0;
	  FFPy = 0.0;
	}

      // For each constraint.
      for(unsigned int j=0;j<MAL_MATRIX_NB_ROWS(LCIFF_it->D);j++)
	{
	  m_Pb.DS[IndexConstraint] =
	    // X Axis * A
	    (FFPx-xk[0] * m_Px(i,0)-
	     xk[1] * m_Px(i,1)-
	     xk[2] * m_Px(i,2))
	    * LCIFF_it->D(j,0)
	    +
	    // Y Axis * A
	    ( FFPy-xk[3] * m_Px(i,0)-
	      xk[4] * m_Px(i,1)-
	      xk[5] * m_Px(i,2))
	    * LCIFF_it->D(j,1)
	    // Constante part of the constraint
	    + LCIFF_it->Dc(j,0);

	  ODEBUG6(m_Pb.DS[IndexConstraint] << " " << (LCIFF_it)->D(j,0)  << " "
		  << (LCIFF_it)->D[j][1] << " " << (LCIFF_it)->Dc(j,0) ,Buffer);
	  ODEBUG6(1 << " " <<    T *(i+1) << " " <<    (i+1)*(i+1)*T*T/2 - Com_Height/9.81,Buffer2);
	  ODEBUG6(1 << " " <<    T *(i+1) << " " <<    (i+1)*(i+1)*T*T/2 - Com_Height/9.81,Buffer3);

	  if (m_FastFormulationMode==QLD)
	    {
	      // In this case, Pu is triangular.
	      // so we can speed up the computation.

	      for(int k=0;k<=i;k++)
		{
		  // X axis
		  DU[IndexConstraint+k*(NbOfConstraints+1)] =
		    -(LCIFF_it)->D(j,0)*m_Pu[k*m_QP_N+i];

		  // Y axis
		  DU[IndexConstraint+(k+m_QP_N)*(NbOfConstraints+1)] =
		    -(LCIFF_it)->D(j,1)*m_Pu[k*m_QP_N+i];
		}
	    }
	  else if ((m_FastFormulationMode==QLDANDLQ)||
		   (m_FastFormulationMode==PLDPHerdt))
	    {
	      // In this case, Pu is *NOT* triangular.
	      for(int k=0;k<m_QP_N;k++)
		{
		  // X axis
		  DU[IndexConstraint+k*(NbOfConstraints+1)] =
		    -(LCIFF_it)->D(j,0)*m_Pu[k*2*(m_QP_N+m_PrwSupport.StepNumber)+i];
		  // Y axis
		  DU[IndexConstraint+(k+m_QP_N)*(NbOfConstraints+1)] =
		    -(LCIFF_it)->D(j,1)*m_Pu[k*2*(m_QP_N+m_PrwSupport.StepNumber)+i];
		}
	    }

	  //Feet variables after jerk: [dddX,dddY,FPx,FPy]
	  if(LCIFF_it->StepNumber>0)
	    {
	      DU[IndexConstraint+(2*m_QP_N+(LCIFF_it)->StepNumber-1)*(NbOfConstraints+1)] = LCIFF_it->D(j,0);
	      DU[IndexConstraint+(2*m_QP_N+m_PrwSupport.StepNumber+(LCIFF_it)->StepNumber-1)*(NbOfConstraints+1)] = LCIFF_it->D(j,1);
	    }

	  ODEBUG("IC: " << IndexConstraint );
	  IndexConstraint++;
	}
      LCIFF_it++;
    }


  //Feet position constraints
  LCIFF_it = QueueOfFeetPosInequalities.begin();
  for( int i=0;i<m_PrwSupport.StepNumber;i++)
    {
      if(LCIFF_it->StepNumber==1)
	{
	  FFPx = CurSF_it->x;
	  FFPy = CurSF_it->y;
	}
      else
	{
	  FFPx = 0.0;
	  FFPy = 0.0;
	}


      // For each constraint.
      for(unsigned int j=0;j<MAL_MATRIX_NB_ROWS(LCIFF_it->D);j++)
	{
	  m_Pb.DS[IndexConstraint] =
	    // X Axis * A
	    FFPx * LCIFF_it->D(j,0)
	    +
	    // Y Axis * A
	    FFPy * LCIFF_it->D(j,1)
	    // Constante part of the constraint
	    + LCIFF_it->Dc(j,0);


	  //Foot variables after jerk: [dddX,dddY,FPx,FPy]
	  if((LCIFF_it)->StepNumber==1)
	    {
	      m_Pb.DU[IndexConstraint+(2*m_QP_N+(LCIFF_it)->StepNumber-1)*(NbOfConstraints+1)] =
		-LCIFF_it->D(j,0);
	      m_Pb.DU[IndexConstraint+(2*m_QP_N+m_PrwSupport.StepNumber+LCIFF_it->StepNumber-1)*(NbOfConstraints+1)] =
		-LCIFF_it->D(j,1);
	    }
	  if((LCIFF_it)->StepNumber>1)
	    {
	      m_Pb.DU[IndexConstraint+(2*m_QP_N+(LCIFF_it)->StepNumber-1)*(NbOfConstraints+1)] =
		-(LCIFF_it)->D(j,0);
	      m_Pb.DU[IndexConstraint+(2*m_QP_N+(LCIFF_it)->StepNumber-2)*(NbOfConstraints+1)] =
		(LCIFF_it)->D(j,0);
	      m_Pb.DU[IndexConstraint+(2*m_QP_N+m_PrwSupport.StepNumber+(LCIFF_it)->StepNumber-1)*(NbOfConstraints+1)] =
		-(LCIFF_it)->D(j,1);
	      m_Pb.DU[IndexConstraint+(2*m_QP_N+m_PrwSupport.StepNumber+(LCIFF_it)->StepNumber-2)*(NbOfConstraints+1)] =
		(LCIFF_it)->D(j,1);
	    }
	  ODEBUG("IC: " << IndexConstraint );
	  IndexConstraint++;
	}

      LCIFF_it++;
    }

  ODEBUG6("Index Constraint :"<< IndexConstraint,Buffer);
  static double localtime = -m_QP_T;
  localtime+=m_QP_T;

  ODEBUG("IndexConstraint:"<<IndexConstraint << " localTime :" << localtime);

  if (m_FullDebug>0)
    {
      
      ODEBUG("localtime: " <<localtime);
      m_Pb.dumpMatrix("/tmp/DU.dat",ProblemVelRef_s::MATRIX_DU);
      m_Pb.dumpVector("/tmp/m_Pb.DS.dat", ProblemVelRef_s::VECTOR_DS);

      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"/tmp/PuCst_%f.dat",StartingTime);
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
	{
	  for( int j=0;j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
	    aof << m_Pu[j+i*2*(m_QP_N+m_PrwSupport.StepNumber)] << " " ;
	  aof << endl;
	}
      aof.close();
      
      sprintf(Buffer,"/tmp/DPX_%f.dat", StartingTime);
      m_Pb.dumpMatrix(Buffer,ProblemVelRef_s::VECTOR_DS);
    }

  return 0;
}



//--------------------------------------
//
//
//-----------new functions--------------
void ZMPVelocityReferencedQP::CallMethod(std::string & Method, std::istringstream &strm)
{

  if (Method==":previewcontroltime")
    {
      strm >> m_PreviewControlTime;
    }
  if (Method==":numberstepsbeforestop")
    {
      support_state_t & CurrentSupport = IntermedData_->SupportState();
      strm >> CurrentSupport.NbStepsLeft;
      SupportFSM_->NbStepsSSDS(CurrentSupport.NbStepsLeft);
    }
  if (Method==":stoppg")
    {
      EndingPhase_ = true;
    }

  ZMPRefTrajectoryGeneration::CallMethod(Method,strm);

}

int
ZMPVelocityReferencedQP::InitOnLine(deque<ZMPPosition> & FinalZMPTraj_deq,
    deque<COMState> & FinalCoMPositions_deq,
    deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
    deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
    FootAbsolutePosition & InitLeftFootAbsolutePosition,
    FootAbsolutePosition & InitRightFootAbsolutePosition,
    deque<RelativeFootPosition> &, // RelativeFootPositions,
    COMState & lStartingCOMState,
    MAL_S3_VECTOR_TYPE(double) & lStartingZMPPosition)
{

  FootAbsolutePosition CurrentLeftFootAbsPos, CurrentRightFootAbsPos;

  // Set the internal state of the ZMPRefTrajectory object.
  m_OnLineMode = true;
  m_EndingPhase = false;
  m_TimeToStopOnLineMode = -1.0;

  // INITIALIZE FEET POSITIONS:
  // --------------------------
  CurrentLeftFootAbsPos = InitLeftFootAbsolutePosition;
  CurrentLeftFootAbsPos.z = 0.0;
  CurrentLeftFootAbsPos.dz = CurrentLeftFootAbsPos.ddz = 0.0;
  CurrentLeftFootAbsPos.time = 0.0;
  CurrentLeftFootAbsPos.theta = 0.0;

  CurrentRightFootAbsPos = InitRightFootAbsolutePosition;
  CurrentRightFootAbsPos.z = 0.0;
  CurrentRightFootAbsPos.dz = CurrentRightFootAbsPos.ddz = 0.0;
  CurrentRightFootAbsPos.time = 0.0;
  CurrentRightFootAbsPos.theta = 0.0;

  // FILL THE QUEUES:
  // ----------------
  int AddArraySize;
  {
    assert(m_SamplingPeriod > 0);
    double ldAddArraySize = TimeBuffer_/m_SamplingPeriod;
    AddArraySize = (int)ldAddArraySize;
  }

  FinalZMPTraj_deq.resize(AddArraySize);
  FinalCoMPositions_deq.resize(AddArraySize);
  FinalLeftFootTraj_deq.resize(AddArraySize);
  FinalRightFootTraj_deq.resize(AddArraySize);
  int CurrentZMPindex=0;
  for( unsigned int i=0;i<FinalZMPTraj_deq.size();i++ )
    {
      // Smooth ramp
      FinalZMPPositions[CurrentZMPindex].px = lStartingZMPPosition(0);
      FinalZMPPositions[CurrentZMPindex].py = lStartingZMPPosition(1);
      FinalZMPPositions[CurrentZMPindex].pz = lStartingZMPPosition(2);
      FinalZMPPositions[CurrentZMPindex].theta = 0.0;
      FinalZMPPositions[CurrentZMPindex].time = m_CurrentTime;
      FinalZMPPositions[CurrentZMPindex].stepType = 0;

      // Set CoM positions.
      FinalCoMPositions_deq[CurrentZMPindex] = lStartingCOMState;
      // Set Left Foot positions.
      FinalLeftFootTraj_deq[CurrentZMPindex] = CurrentLeftFootAbsPos;
      FinalRightFootTraj_deq[CurrentZMPindex] = CurrentRightFootAbsPos;
      FinalLeftFootTraj_deq[CurrentZMPindex].time =
          FinalRightFootTraj_deq[CurrentZMPindex].time = m_CurrentTime;
      FinalLeftFootTraj_deq[CurrentZMPindex].stepType =
          FinalRightFootTraj_deq[CurrentZMPindex].stepType = 10;

      m_CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;
    }

  // INITIAL SUPPORT STATE:
  // ----------------------
  support_state_t CurrentSupport;
  CurrentSupport.Phase = DS;
  CurrentSupport.Foot = LEFT;
  CurrentSupport.TimeLimit = 1000000000;
  CurrentSupport.NbStepsLeft = 1;
  CurrentSupport.StateChanged = false;
  CurrentSupport.X = 0.0;
  CurrentSupport.Y = 0.1;
  CurrentSupport.Yaw = 0.0;
  CurrentSupport.StartTime = 0.0;
  IntermedData_->SupportState(CurrentSupport);

  // INITIALIZE CENTER OF MASS:
  // --------------------------
  com_t CoM;
  CoM.x[0] = lStartingCOMState.x[0];
  CoM.x[1] = lStartingCOMState.x[1];
  CoM.x[2] = lStartingCOMState.x[2];
  CoM.y[0] = lStartingCOMState.y[0];
  CoM.y[1] = lStartingCOMState.y[1];
  CoM.y[2] = lStartingCOMState.y[2];
  CoM.z[0] = lStartingCOMState.z[0];
  CoM.z[1] = lStartingCOMState.z[1];
  CoM.z[2] = lStartingCOMState.z[2];
  CoM_.SetComHeight(lStartingCOMState.z[0]);
  CoM_.InitializeSystem();
  CoM_(CoM);
  IntermedData_->CoM(CoM_());

  // BUILD CONSTANT PART OF THE OBJECTIVE:
  // -------------------------------------
  Problem_.reset();
  Problem_.nbInvariantRows(2*QP_N_);
  Problem_.nbInvariantCols(2*QP_N_);
  VRQPGenerator_->build_invariant_part( Problem_ );

  return 0;
}



void
ZMPVelocityReferencedQP::OnLine(double time,
    deque<ZMPPosition> & FinalZMPTraj_deq,
    deque<COMState> & FinalCOMTraj_deq,
    deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
    deque<FootAbsolutePosition> & FinalRightFootTraj_deq)
{

  // If on-line mode not activated we go out.
  if (!m_OnLineMode)
    return;

  // Test if the end of the online mode has been reached.
  if ((EndingPhase_) &&
      (time>=TimeToStopOnLineMode_))
    { m_OnLineMode = false; }


  if(time + 0.00001 > m_UpperTimeLimitToUpdate)
    {



  // UPDATE WALKING TRAJECTORIES:
  // ----------------------------
  if(time + 0.00001 > UpperTimeLimitToUpdate_)
    {

      // UPDATE INTERNAL DATA:
      // ---------------------
      Problem_.reset_variant();
      Solution_.reset();
      VRQPGenerator_->CurrentTime( time );
      VelRef_=NewVelRef_;
      SupportFSM_->update_vel_reference(VelRef_, IntermedData_->SupportState());
      IntermedData_->Reference( VelRef_ );
      IntermedData_->CoM( CoM_() );


      // PREVIEW SUPPORT STATES FOR THE WHOLE PREVIEW WINDOW:
      // ----------------------------------------------------
      VRQPGenerator_->preview_support_states( time, SupportFSM_,
          FinalLeftFootTraj_deq, FinalRightFootTraj_deq, Solution_.SupportStates_deq );

	  newSF.x = FAP_it->x;
	  newSF.y = FAP_it->y;
	  newSF.theta = FAP_it->theta*M_PI/180.0;
	  newSF.StartTime = time+m_TimeBuffer;
	  newSF.SupportFoot = m_CurrentSupport.Foot;

      // COMPUTE ORIENTATIONS OF FEET FOR WHOLE PREVIEW PERIOD:
      // ------------------------------------------------------
      OrientPrw_->preview_orientations( time, VelRef_,
          SupportFSM_->StepPeriod(),
          FinalLeftFootTraj_deq, FinalRightFootTraj_deq,
          Solution_ );


      // UPDATE THE DYNAMICS:
      // --------------------
      Robot_->update( Solution_.SupportStates_deq,
          FinalLeftFootTraj_deq, FinalRightFootTraj_deq );


      // COMPUTE REFERENCE IN THE GLOBAL FRAME:
      // --------------------------------------
      VRQPGenerator_->compute_global_reference( Solution_ );


      // BUILD VARIANT PART OF THE OBJECTIVE:
      // ------------------------------------
      VRQPGenerator_->update_problem( Problem_, Solution_.SupportStates_deq );


      // BUILD CONSTRAINTS:
      // ------------------
      VRQPGenerator_->build_constraints( Problem_, Solution_ );

	  ldt = lend.tv_sec - lbegin.tv_sec +
	    0.000001 * (lend.tv_usec - lbegin.tv_usec);

      // SOLVE PROBLEM:
      // --------------
      if (Solution_.useWarmStart)
    	  VRQPGenerator_->compute_warm_start( Solution_ );//TODO: Move to update_problem or build_constraints?
      Problem_.solve( QLD, Solution_, NONE );
      if(Solution_.Fail>0)
          Problem_.dump( time );


      // INTERPOLATE THE NEXT COMPUTED COM STATE:
      // ----------------------------------------
      unsigned currentIndex = FinalCOMTraj_deq.size();
      FinalCOMTraj_deq.resize( (unsigned)(QP_T_/m_SamplingPeriod)+currentIndex );
      FinalZMPTraj_deq.resize( (unsigned)(QP_T_/m_SamplingPeriod)+currentIndex );
      CoM_.Interpolation( FinalCOMTraj_deq, FinalZMPTraj_deq, currentIndex,
          Solution_.Solution_vec[0], Solution_.Solution_vec[QP_N_] );
      CoM_.OneIteration( Solution_.Solution_vec[0],Solution_.Solution_vec[QP_N_] );


      // INTERPOLATE TRUNK ORIENTATION:
      // ------------------------------
      OrientPrw_->interpolate_trunk_orientation( time, currentIndex,
          m_SamplingPeriod, Solution_.SupportStates_deq,
          FinalCOMTraj_deq );

	  // Specify that we are in the ending phase.
	  if (m_EndingPhase==false)
	    {
	      // This should be done only during the transition EndingPhase=false -> EndingPhase=true
	      m_TimeToStopOnLineMode = m_UpperTimeLimitToUpdate+m_QP_T * m_QP_N;
	      // Set the ZMP reference as very important.
	      // It suppose to work because Gamma appears only during the non-constant 
	    }
	  m_EndingPhase = true;
	  
	}

      // INTERPOLATE THE COMPUTED FOOT POSITIONS:
      // ----------------------------------------
      Robot_->generate_trajectories( time, Solution_,
          Solution_.SupportStates_deq, Solution_.SupportOrientations_deq,
          FinalLeftFootTraj_deq, FinalRightFootTraj_deq );

      if (m_FullDebug>2)
	{
	  ofstream aof;

      // Specify that we are in the ending phase.
      if (EndingPhase_ == false)
        {
          TimeToStopOnLineMode_ = UpperTimeLimitToUpdate_ + QP_T_ * QP_N_;
        }
      UpperTimeLimitToUpdate_ = UpperTimeLimitToUpdate_ + QP_T_;

    }
  //-----------------------------------
  //
  //
  //----------"Real-time" loop--------

}


// TODO: New parent class needed
void ZMPVelocityReferencedQP::GetZMPDiscretization(deque<ZMPPosition> & ,
    deque<COMState> & ,
    deque<RelativeFootPosition> &,
    deque<FootAbsolutePosition> &,
    deque<FootAbsolutePosition> &,
    double ,
    COMState &,
    MAL_S3_VECTOR(&,double),
    FootAbsolutePosition & ,
    FootAbsolutePosition & )
{
  cout << "To be removed" << endl;
}


void ZMPVelocityReferencedQP::OnLineAddFoot(RelativeFootPosition & ,
    deque<ZMPPosition> & ,
    deque<COMState> & ,
    deque<FootAbsolutePosition> &,
    deque<FootAbsolutePosition> &,
    bool)
{
  cout << "To be removed" << endl;
}

int ZMPVelocityReferencedQP::OnLineFootChange(double ,
    FootAbsolutePosition &,
    deque<ZMPPosition> & ,
    deque<COMState> & ,
    deque<FootAbsolutePosition> &,
    deque<FootAbsolutePosition> &,
    StepStackHandler  *)
{
  cout << "To be removed" << endl;
  return -1;
}

void ZMPVelocityReferencedQP::EndPhaseOfTheWalking(deque<ZMPPosition> &, 
    deque<COMState> &,
    deque<FootAbsolutePosition> &,
    deque<FootAbsolutePosition> &)
{
  cout << "To be removed" << endl;
}

int ZMPVelocityReferencedQP::ReturnOptimalTimeToRegenerateAStep()
{
  int r = (int)(m_PreviewControlTime/m_SamplingPeriod);
  return 2*r;
}


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


#ifdef UNIX
#include <sys/time.h>
#endif /* UNIX */

#ifdef WIN32
#include <Windows.h>
#include <TimeUtilsWindows.h>
#endif

#include <time.h>

#include <iostream>
#include <fstream>

#include <Mathematics/qld.h>
#include <ZMPRefTrajectoryGeneration/ZMPVelocityReferencedQP.h>

#include <Debug.h>
using namespace std;
using namespace PatternGeneratorJRL;

ZMPVelocityReferencedQP::ZMPVelocityReferencedQP(SimplePluginManager *lSPM,
						 string DataFile,
						 CjrlHumanoidDynamicRobot *aHS) :
  ZMPRefTrajectoryGeneration(lSPM)
{

  m_Q = 0;
  m_Pu = 0;
  m_FullDebug = -10;
  m_FastFormulationMode = QLD;

  m_QP_T = 0.1;
  m_QP_N = 16;

  m_SamplingPeriod = 0.005;

  m_ComHeight = 0.814;

  /*! Getting the ZMP reference from Kajita's heuristic. */
  m_ZMPD = new ZMPDiscretization(lSPM,DataFile,aHS);

  /*! For simulating the linearized inverted pendulum in 2D. */
  m_2DLIPM = new LinearizedInvertedPendulum2D();


  /*! For computing the stability constraints from the feet positions. */
  m_ConstraintOnX = 0.04;
  m_ConstraintOnY = 0.04;
  m_fCALS = new FootConstraintsAsLinearSystemForVelRef(lSPM,aHS,m_ConstraintOnX,m_ConstraintOnY);

  //m_StartTime = 0.0;
  m_UpperTimeLimitToUpdate = 0.0;
  m_TimeBuffer = 0.040;

  m_FTGS = new FootTrajectoryGenerationStandard(lSPM,aHS->leftFoot());
  m_FTGS->InitializeInternalDataStructures();

  //TODO: Change the name of the class
  /* Initialize the FSM */
  m_SupportFSM = new SupportFSM(m_QP_T);

  //TODO: The initialization has to be exported.
  //Initialize the support state
  m_CurrentSupport.Phase = 0;
  m_CurrentSupport.Foot = 1;
  m_CurrentSupport.TimeLimit = 1000000000;
  m_CurrentSupport.StepsLeft = 1;
  m_CurrentSupport.SSSS = false;
  m_CurrentSupport.StateChanged = false;


  /* Orientations preview algorithm*/
  m_OP = new OrientationsPreview(m_QP_T, m_QP_N, m_SupportFSM->m_SSPeriod, aHS->rootJoint());

  m_RobotMass = aHS->mass();
  m_TrunkState.yaw[0]=m_TrunkState.yaw[1]=m_TrunkState.yaw[2]=0.0;

  InitConstants();

  m_PLDPSolverHerdt = 0;

  /* Initialize  the 2D LIPM */
  m_2DLIPM->SetSimulationControlPeriod(m_QP_T);
  m_2DLIPM->SetRobotControlPeriod(m_SamplingPeriod);
  m_2DLIPM->SetComHeight(m_ComHeight);
  m_2DLIPM->InitializeSystem();

  //Gains
  m_Alpha = 0.00001;//Jerk
  m_Beta = 1.0; //Velocity
  m_Gamma = 0.000001; //ZMP



  initFeet();

  // Register method to handle
  string aMethodName[2] =
    {":previewcontroltime",
     ":numberstepsbeforestop"};

  for(int i=0;i<2;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
    }

  if (m_FastFormulationMode==QLDANDLQ)
    {
      RESETDEBUG6("/tmp/dtQLD.dat");
      RESETDEBUG6("/tmp/InfosQLD.dat");
      RESETDEBUG6("/tmp/Check2DLIPM_QLDANDLQ.dat");
    }


  if (m_FastFormulationMode==PLDP)
    {
      RESETDEBUG6("/tmp/dtPLDP.dat");
      RESETDEBUG6("/tmp/Check2DLIPM_PLDP.dat");
    }

  if(m_FullDebug>2)
    {
      ofstream aof;
      aof.open("/tmp/Trunk.dat",ofstream::out);
      aof.close();
      aof.open("/tmp/time.dat",ofstream::out);
      aof.close();
      aof.open("/tmp/FootPositionsT.dat",ofstream::out);
      aof.close();
    }

  //Feet distance in the DS phase
  m_FeetDistanceDS = 0.2;

  m_PerturbationOccured = false;
}

ZMPVelocityReferencedQP::~ZMPVelocityReferencedQP()
{

  if (m_ZMPD!=0)
    delete m_ZMPD;

  if (m_2DLIPM!=0)
    delete m_2DLIPM;

  if (m_SupportFSM!=0)
    delete m_SupportFSM;

  if (m_fCALS!=0)
    delete m_fCALS;

  if (m_FTGS!=0)
    delete m_FTGS;

  if (m_Q!=0)
    delete [] m_Q;

  if (m_OP!=0)
    delete m_OP;

  if (m_PLDPSolverHerdt!=0)
    delete m_PLDPSolverHerdt;

  if (m_Pu!=0)
    delete [] m_Pu ;
}

void ZMPVelocityReferencedQP::setVelReference(istringstream &strm)
{
  strm >> RefVel.x;
  strm >> RefVel.y;
  strm >> RefVel.dYaw;
}

void ZMPVelocityReferencedQP::setVelReference(double x,
					      double y,
					      double yaw)
{
  RefVel.x = x;
  RefVel.y = y;
  RefVel.dYaw = yaw;
}

void ZMPVelocityReferencedQP::setCoMPerturbationForce(istringstream &strm)
{
  MAL_VECTOR_RESIZE(m_PerturbationAcceleration,6);

  strm >> m_PerturbationAcceleration(2);
  strm >> m_PerturbationAcceleration(5);
  m_PerturbationAcceleration(2) = m_PerturbationAcceleration(2)/m_RobotMass;
  m_PerturbationAcceleration(5) = m_PerturbationAcceleration(5)/m_RobotMass;
  m_PerturbationOccured = true;
}

void ZMPVelocityReferencedQP::setCoMPerturbationForce(double x,double y)
{
  MAL_VECTOR_RESIZE(m_PerturbationAcceleration,6);

  m_PerturbationAcceleration(2) = x/m_RobotMass;
  m_PerturbationAcceleration(5) = y/m_RobotMass;
  m_PerturbationOccured = true;

}

void ZMPVelocityReferencedQP::interpolateFeet(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					      deque<FootAbsolutePosition> &RightFootAbsolutePositions)
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


int ZMPVelocityReferencedQP::BuildingConstantPartOfTheObjectiveFunctionQLD(MAL_MATRIX(,double) &OptA)
{
  for( int i=0;i<2*m_QP_N;i++)
    for( int j=0;j<2*m_QP_N;j++)
      m_Q[i*2*m_QP_N+j] = OptA(j,i);

  return 0;
}
int ZMPVelocityReferencedQP::BuildingConstantPartOfTheObjectiveFunctionQLDANDLQ(MAL_MATRIX(,double) &OptA)
{

  /*! Build cholesky matrix of the optimum
    We copy only the upper corner of the OptA matrix
    because we know its specific structure.
  */
  double *localQ=new double[m_QP_N*m_QP_N];
  for( int i=0;i<m_QP_N;i++)
    for( int j=0;j<m_QP_N;j++)
      localQ[i*m_QP_N+j] = OptA(i,j);

  double *localLQ=new double[m_QP_N*m_QP_N];
  double *localiLQ=new double[m_QP_N*m_QP_N];

  memset(localLQ,0,m_QP_N*m_QP_N*sizeof(double));
  memset(localiLQ,0,m_QP_N*m_QP_N*sizeof(double));

  OptCholesky anOCD(m_QP_N,m_QP_N,OptCholesky::MODE_NORMAL);
  anOCD.SetA(localQ,m_QP_N);
  anOCD.SetL(localLQ);
  anOCD.SetiL(localiLQ);

  anOCD.ComputeNormalCholeskyOnANormal();
  anOCD.ComputeInverseCholeskyNormal(1);

  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"/tmp/localQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<m_QP_N;i++)
	{
	  for( int j=0;j<m_QP_N;j++)
	    aof << localQ[i*m_QP_N+j] << " ";
	  aof<<endl;
	}
      aof.close();

      sprintf(Buffer,"/tmp/localLQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<m_QP_N;i++)
	{
	  for( int j=0;j<m_QP_N;j++)
	    aof << localLQ[i*m_QP_N+j] << " ";
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"/tmp/localiLQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<m_QP_N;i++)
	{
	  for( int j=0;j<m_QP_N;j++)
	    aof << localiLQ[i*m_QP_N+j] << " ";
	  aof << endl;
	}
      aof.close();

    }


  MAL_MATRIX_RESIZE(m_LQ,2*m_QP_N,2*m_QP_N);
  MAL_MATRIX_RESIZE(m_iLQ,2*m_QP_N,2*m_QP_N);


  for( int i=0;i<m_QP_N;i++)
    {
      for( int j=0;j<m_QP_N;j++)
	{
	  m_LQ(i,j) = localLQ[i*m_QP_N+j];
	  m_LQ(i+m_QP_N,j+m_QP_N) = localLQ[i*m_QP_N+j];
	  m_LQ(i,j+m_QP_N) = 0.0;
	  m_LQ(i+m_QP_N,j) = 0.0;

	  m_iLQ(i,j) = localiLQ[i*m_QP_N+j];
	  m_iLQ(i+m_QP_N,j+m_QP_N) = localiLQ[i*m_QP_N+j];
	  m_iLQ(i,j+m_QP_N) = 0.0;
	  m_iLQ(i+m_QP_N,j) = 0.0;
	}
    }


  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];

      sprintf(Buffer,"/tmp/LQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<2*m_QP_N;i++)
	{
	  for( int j=0;j<2*m_QP_N;j++)
	    aof << m_LQ(i,j) << " ";
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"/tmp/iLQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<2*m_QP_N;i++)
	{
	  for( int j=0;j<2*m_QP_N;j++)
	    aof << m_iLQ(i,j) << " ";
	  aof << endl;
	}
      aof.close();
    }
  delete [] localQ;
  delete [] localLQ;
  delete [] localiLQ;

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

  // Initialization of the matrice regarding the quadratic
  // part of the objective function.
  //TODO:: size of Q is 3*Nx3*N which means that there is place for N/2 feet variables
  m_Q=new double[4*(m_QP_N)*(m_QP_N)];
  memset(m_Q,0,4*(m_QP_N)*(m_QP_N)*sizeof(double));
  // for( int i=0;i<2*m_QP_N;i++)
  //   m_Q[i*2*m_QP_N+i] = 1.0;


  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"/tmp/Q.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<2*(m_QP_N);i++)
	{
	  for( int j=0;j<2*(m_QP_N);j++)
	    aof << m_Q[i*m_QP_N*2+j] << " ";
	  aof << endl;
	}
      aof.close();
    }

  /*! Compute constants of the linear part of the objective function. */
  lterm1 = MAL_RET_TRANSPOSE(m_PPu);
  lterm1 = MAL_RET_A_by_B(lterm1,m_PPx);
  m_OptB = MAL_RET_TRANSPOSE(m_VPu);
  m_OptB = MAL_RET_A_by_B(m_OptB,m_VPx);
  m_OptB = m_Alpha * m_OptB;
  m_OptB = m_OptB + m_Beta * lterm1;

  m_OptC = MAL_RET_TRANSPOSE(m_PPu);
  m_OptC = m_Beta * m_OptC;



  if ((m_FastFormulationMode==QLDANDLQ) ||
      (m_FastFormulationMode==PLDP))
    {
      //BuildingConstantPartOfTheObjectiveFunctionQLDANDLQ(OptA);
    }
  else
    {
      BuildingConstantPartOfTheObjectiveFunctionQLD(OptA);
    }


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

int ZMPVelocityReferencedQP::validateConstraints(double * & DS,double * &DU,
						 int NbOfConstraints,  int li,
						 double *X, double time)
{
  // double lSizeMat = QueueOfLConstraintInequalities.back()->EndingTime/m_QP_T;
  MAL_MATRIX(vnlPx,double); MAL_MATRIX(vnlPu,double);
  MAL_MATRIX(vnlValConstraint,double);
  MAL_MATRIX(vnlX,double);// MAL_MATRIX(vnlStorePx,double);
  // MAL_MATRIX(vnlStoreX,double);
  MAL_VECTOR(ConstraintNb,int);

  MAL_MATRIX_RESIZE(vnlX,2*(m_QP_N+m_PrwSupport.StepNumber),1);

  // ConstraintNb[li] = NbOfConstraints;
  MAL_MATRIX_RESIZE(vnlPu,NbOfConstraints,2*(m_QP_N+m_PrwSupport.StepNumber));
  MAL_MATRIX_RESIZE(vnlPx,NbOfConstraints,1);


  for(int i=0; i<NbOfConstraints;i++)
    {
      vnlPx(i,0) = DS[i];
      // vnlStorePx(i,li) = DS[i];
    }

  for(int i=0; i<NbOfConstraints;i++)
    for( int j=0; j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
      vnlPu(i,j) = DU[j*(NbOfConstraints+1)+i];

  for( int i=0; i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
    {
      // vnlStoreX(i,li) = X[i];
      vnlX(i,0) = X[i];
    }

  vnlValConstraint = MAL_RET_A_by_B(vnlPu, vnlX)  + vnlPx;

  if (MAL_MATRIX_NB_COLS(vnlValConstraint)!=1)
    {
      cout << "Problem during validation of the constraints matrix: " << endl;
      cout << "   size for the columns different from 1" << endl;
      return -1;
    }


  for(int i=0;i<NbOfConstraints;i++)
    {
      int pbOnCurrent=0;
      if (vnlValConstraint(i,0)<-1e-8)
	{
	  ODEBUG3("Problem during validation of the constraints at time: "<<time);
	  ODEBUG3("  constraint " << i << " is not positive");
	  ODEBUG3(vnlValConstraint(i,0));
	  pbOnCurrent = 1;
	}

      if (pbOnCurrent)
	{
	  ODEBUG3("PbonCurrent: " << pbOnCurrent << " " << li
		  << " Contrainte " << i);
	  return -1;
	}

    }

  return 0;
}

int ZMPVelocityReferencedQP::dumpProblem(double * Q,
					 double * D,
					 double * DPu,
					 int NbOfConstraints,
					 double * Px,
					 double * XL,
					 double * XU,
					 MAL_VECTOR(& xk,double),
					 double Time)
{
  ofstream aof;

  char Buffer[1024];
  sprintf(Buffer,"/tmp/ProblemFF_%f.dat",Time);
  aof.open(Buffer,ofstream::out);

  //Somehow this has to be done
  NbOfConstraints++;

  // Dumping Q.
  aof << endl << "Q:"<< endl;
  for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
    {
      for( int j=0;j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
	{
	  aof <<Q[j*2*(m_QP_N+m_PrwSupport.StepNumber)+i]<< " ";
	}
      aof <<endl;
    }

  // Dumping D.
  aof << endl <<"D: "<< "RefVel: "<< RefVel.x << " "<< RefVel.y << "xk: "<<xk<< endl;
  for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
    {
      aof <<D[i]<< " ";
    }
  aof <<endl;

  // Dumping Pu.
  aof << "DU: "<< "NbOfConstr.: " << NbOfConstraints << endl;
  for(int i=0;i<NbOfConstraints;i++)
    {
      for( int j=0;j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
	{
	  aof << DPu[j*(NbOfConstraints)+i] << " ";
	}
      aof <<endl;
    }

  // Dumping Px.
  aof << endl<<"DS:"<< endl;
  for(int i=0;i<NbOfConstraints;i++)
    {
      aof << Px[i] << " ";
    }
  aof << endl;

  // Dumping XL.
  aof << endl << "XL:"<< endl;
  for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
    {
      aof << XL[i] << " ";
    }
  aof << endl;

  // Dumping XU.
  aof << endl << "XU:"<< endl;
  for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
    {
      aof << XU[i] << " ";
    }
  aof << endl;

  aof.close();
  return 0;
}

int ZMPVelocityReferencedQP::buildConstraintMatrices(double * &DS,double * &DU,
						     double T, double StartingTime,
						     deque<LinearConstraintInequalityFreeFeet_t> &
						     QueueOfLConstraintInequalitiesFreeFeet,
						     deque<LinearConstraintInequalityFreeFeet_t> &
						     QueueOfFeetPosInequalities,
						     deque<SupportFeet_t> &
						     QueueOfSupportFeet,
						     double Com_Height,
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

>>>>>>> topic/vel-ref-gen-cleaning-restructuring
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


  if (0)
    {
      ODEBUG("localtime: " <<localtime);
      ofstream aof;

      char Buffer[1024];
      sprintf(Buffer,"/tmp/DU.dat");
      aof.open(Buffer,ofstream::out);
      aof <<" 2*N+2*m_PrwSupport.StepNumber: "<<2*m_QP_N+2*m_PrwSupport.StepNumber<<" NbOfConstraints: "<<NbOfConstraints
	  << endl;
      for( int i=0;i<NbOfConstraints;i++)
	{
	  for( int j=0;j<2*m_QP_N+2*m_PrwSupport.StepNumber;j++)
	    aof << DU[j*NbOfConstraints+i] << " " ;
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"/tmp/m_Pb.DS.dat");
      aof.open(Buffer,ofstream::out);
      for( int j=0;j<NbOfConstraints;j++)
	aof << m_Pb.DS[j] << endl;
      // aof << endl;
      aof.close();

    }

  if (m_FullDebug>0)
    {

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


      if (0)
	{
	  sprintf(Buffer,"/tmp/DPX_%f.dat", StartingTime);
	  aof.open(Buffer,ofstream::out);
	  for( int i=0;i<IndexConstraint;i++)
	    {
	      aof << m_Pb.DS[i] << endl ;
	    }
	  aof.close();
	}
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
      strm >> m_Support->NbOfStepsSSDS;
    }

  ZMPRefTrajectoryGeneration::CallMethod(Method,strm);
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
      strm >> m_SupportFSM->m_NbOfStepsSSDS;
    }

  ZMPRefTrajectoryGeneration::CallMethod(Method,strm);
}

int ZMPVelocityReferencedQP::InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
					deque<COMState> & FinalCoMPositions,
					deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
					deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
					FootAbsolutePosition & InitLeftFootAbsolutePosition,
					FootAbsolutePosition & InitRightFootAbsolutePosition,
					deque<RelativeFootPosition> &RelativeFootPositions,
					COMState & lStartingCOMState,
					MAL_S3_VECTOR(,double) & lStartingZMPPosition)
{

  FootAbsolutePosition CurrentLeftFootAbsPos, CurrentRightFootAbsPos;


  ODEBUG4("ZMP::InitOnLine - Step 2 ","ZMDInitOnLine.txt");
  // Initialize position of the feet.
  CurrentLeftFootAbsPos = InitLeftFootAbsolutePosition;
  CurrentLeftFootAbsPos.z = 0.0;//m_FTGS->m_AnklePositionLeft[2];
  CurrentLeftFootAbsPos.time = 0.0;
  CurrentLeftFootAbsPos.theta = 0.0;


  ODEBUG4("CurrentLeftFootAbsPos.y: " << CurrentLeftFootAbsPos.y, "ZMDInitOnLine.txt");
  CurrentRightFootAbsPos = InitRightFootAbsolutePosition;
  CurrentRightFootAbsPos.z = 0.0;//m_FTGS->m_AnklePositionRight[2];
  CurrentRightFootAbsPos.time = 0.0;
  CurrentRightFootAbsPos.theta = 0.0;

  // V pre is the difference between
  // the current m_SupportFSM position and the precedent.
  ODEBUG4("ZMP::InitOnLine - Step 2.5 ","ZMDInitOnLine.txt");


  int AddArraySize;
  {
    assert(m_SamplingPeriod > 0);
    double ldAddArraySize = m_TimeBuffer/m_SamplingPeriod;
    AddArraySize = (int) ldAddArraySize;
  }

  ODEBUG(AddArraySize);
  FinalZMPPositions.resize(AddArraySize);
  FinalCoMPositions.resize(AddArraySize);
  FinalLeftFootAbsolutePositions.resize(AddArraySize);
  FinalRightFootAbsolutePositions.resize(AddArraySize);
  int CurrentZMPindex=0;

  //TODO:
  if(m_FullDebug>0)
    {
      //Feet coordinates for plot in scilab
      ofstream aoffeet;
      aoffeet.open("/tmp/Feet.dat",ios::out);
      aoffeet<<"#Time    "<<"LeftX    "<<"LeftY    "<<"LeftZ    "<<"RightX    "<<"RightY    "<<"RightZ    "<<endl;
      aoffeet.close();
    }

  for( unsigned int i=0;i<FinalZMPPositions.size();i++)
    {

      // Smooth ramp
      FinalZMPPositions[CurrentZMPindex].px =lStartingZMPPosition(0);
      FinalZMPPositions[CurrentZMPindex].py = lStartingZMPPosition(1);
      FinalZMPPositions[CurrentZMPindex].pz = lStartingZMPPosition(2);
      FinalZMPPositions[CurrentZMPindex].theta = 0.0;
      FinalZMPPositions[CurrentZMPindex].time = m_CurrentTime;
      FinalZMPPositions[CurrentZMPindex].stepType = 0;

      // Set CoM positions.
      FinalCoMPositions[CurrentZMPindex] = lStartingCOMState;

      // Set Left Foot positions.
      FinalLeftFootAbsolutePositions[CurrentZMPindex] = CurrentLeftFootAbsPos;
      FinalRightFootAbsolutePositions[CurrentZMPindex] = CurrentRightFootAbsPos;

      FinalLeftFootAbsolutePositions[CurrentZMPindex].time =
        FinalRightFootAbsolutePositions[CurrentZMPindex].time = m_CurrentTime;

      FinalLeftFootAbsolutePositions[CurrentZMPindex].stepType =
        FinalRightFootAbsolutePositions[CurrentZMPindex].stepType = 10;



      if(m_FullDebug>0)
	{
	  //Feet coordinates for plot in scilab
	  ofstream aoffeet;
	  aoffeet.open("/tmp/Feet.dat",ios::app);
	  aoffeet<<FinalLeftFootAbsolutePositions[CurrentZMPindex].time<<"    "
		 <<FinalLeftFootAbsolutePositions[CurrentZMPindex].x<<"    "
		 <<FinalLeftFootAbsolutePositions[CurrentZMPindex].y<<"    "
		 <<FinalLeftFootAbsolutePositions[CurrentZMPindex].z<<"    "
		 <<FinalLeftFootAbsolutePositions[CurrentZMPindex].stepType<<"    "
		 <<FinalRightFootAbsolutePositions[CurrentZMPindex].x<<"    "
		 <<FinalRightFootAbsolutePositions[CurrentZMPindex].y<<"    "
		 <<FinalRightFootAbsolutePositions[CurrentZMPindex].z<<"    "
		 <<FinalRightFootAbsolutePositions[CurrentZMPindex].stepType<<"    "<<endl;
	  aoffeet.close();
	}

      m_CurrentTime += m_SamplingPeriod;
      CurrentZMPindex++;

    }

  MAL_VECTOR_DIM(xk,double,6);

  xk[0] = lStartingCOMState.x[0];
  xk[1] = lStartingCOMState.x[1];
  xk[2] = lStartingCOMState.x[2];
  xk[3] = lStartingCOMState.y[0];
  xk[4] = lStartingCOMState.y[1];
  xk[5] = lStartingCOMState.y[2];

  m_2DLIPM->setState(xk);

  m_2DLIPM->GetState(xk);

  return 0;
}


void ZMPVelocityReferencedQP::initializeProblem()
{

  m_Pb.DS = new double[(8*m_QP_N+1)*2*(m_QP_N+m_PrwSupport.StepNumber)];

  m_Pb.DU = new double[(8*m_QP_N+1)*2*(m_QP_N+m_PrwSupport.StepNumber)];

  memset(m_Pb.DU,0,(8*m_QP_N+1)*2*(m_QP_N+m_PrwSupport.StepNumber)*sizeof(double));

  m_Pb.Q=new double[4*(m_QP_N+m_PrwSupport.StepNumber)*(m_QP_N+m_PrwSupport.StepNumber)];  //Quadratic part of the objective function
  m_Pb.D=new double[2*(m_QP_N+m_PrwSupport.StepNumber)];   // Linear part of the objective function
  m_Pb.XL=new double[2*(m_QP_N+m_PrwSupport.StepNumber)];  // Lower bound of the jerk.
  m_Pb.XU=new double[2*(m_QP_N+m_PrwSupport.StepNumber)];  // Upper bound of the jerk.
  m_Pb.X=new double[2*(m_QP_N+m_PrwSupport.StepNumber)];   // Solution of the system.
  m_Pb.NewX=new double[2*(m_QP_N+m_PrwSupport.StepNumber)];   // Solution of the system.

}

void ZMPVelocityReferencedQP::computeCholeskyOfQ(double * OptA)
{

  /*! Build cholesky matrix of the optimum
    We copy only the upper corner of the OptA matrix
    because we know its specific structure.
  */
  double *localQ=new double[2*(m_QP_N+m_PrwSupport.StepNumber)*2*(m_QP_N+m_PrwSupport.StepNumber)];
  for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
    for( int j=0;j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
      localQ[i*2*(m_QP_N+m_PrwSupport.StepNumber)+j] = OptA[i*2*(m_QP_N+m_PrwSupport.StepNumber)+j];

  double *localLQ=new double[2*(m_QP_N+m_PrwSupport.StepNumber)*2*(m_QP_N+m_PrwSupport.StepNumber)];
  double *localiLQ=new double[2*(m_QP_N+m_PrwSupport.StepNumber)*2*(m_QP_N+m_PrwSupport.StepNumber)];

  memset(localLQ,0,2*(m_QP_N+m_PrwSupport.StepNumber)*2*(m_QP_N+m_PrwSupport.StepNumber)*sizeof(double));
  memset(localiLQ,0,2*(m_QP_N+m_PrwSupport.StepNumber)*2*(m_QP_N+m_PrwSupport.StepNumber)*sizeof(double));

  OptCholesky anOCD(2*(m_QP_N+m_PrwSupport.StepNumber),2*(m_QP_N+m_PrwSupport.StepNumber),OptCholesky::MODE_NORMAL);
  anOCD.SetA(localQ,2*(m_QP_N+m_PrwSupport.StepNumber));
  anOCD.SetL(localLQ);
  anOCD.SetiL(localiLQ);

  anOCD.ComputeNormalCholeskyOnANormal();
  anOCD.ComputeInverseCholeskyNormal(1);

  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"/tmp/localQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
	{
	  for( int j=0;j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
	    aof << localQ[i*2*(m_QP_N+m_PrwSupport.StepNumber)+j] << " ";
	  aof<<endl;
	}
      aof.close();

      sprintf(Buffer,"/tmp/localLQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
	{
	  for( int j=0;j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
	    aof << localLQ[i*2*(m_QP_N+m_PrwSupport.StepNumber)+j] << " ";
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"/tmp/localiLQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
	{
	  for( int j=0;j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
	    aof << localiLQ[i*2*(m_QP_N+m_PrwSupport.StepNumber)+j] << " ";
	  aof << endl;
	}
      aof.close();

    }


  MAL_MATRIX_RESIZE(m_LQ,2*(m_QP_N+m_PrwSupport.StepNumber),2*(m_QP_N+m_PrwSupport.StepNumber));
  MAL_MATRIX_RESIZE(m_iLQ,2*(m_QP_N+m_PrwSupport.StepNumber),2*(m_QP_N+m_PrwSupport.StepNumber));


  for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
    {
      for( int j=0;j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
	{
	  m_LQ(i,j) = localLQ[i*2*(m_QP_N+m_PrwSupport.StepNumber)+j];
	  // 	  m_LQ(i+2*(m_QP_N+m_PrwSupport.StepNumber),j+2*(m_QP_N+m_PrwSupport.StepNumber)) = localLQ[i*2*(m_QP_N+m_PrwSupport.StepNumber)+j];
	  // 	  m_LQ(i,j+(m_QP_N+m_PrwSupport.StepNumber)) = 0.0;
	  // 	  m_LQ(i+(m_QP_N+m_PrwSupport.StepNumber),j) = 0.0;

	  m_iLQ(i,j) = localiLQ[i*2*(m_QP_N+m_PrwSupport.StepNumber)+j];
	  // 	  m_iLQ(i+(m_QP_N+m_PrwSupport.StepNumber),j+(m_QP_N+m_PrwSupport.StepNumber)) = localiLQ[i*(m_QP_N+m_PrwSupport.StepNumber)+j];
	  // 	  m_iLQ(i,j+(m_QP_N+m_PrwSupport.StepNumber)) = 0.0;
	  // 	  m_iLQ(i+(m_QP_N+m_PrwSupport.StepNumber),j) = 0.0;

	}
    }

  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];

      sprintf(Buffer,"/tmp/LQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
	{
	  for( int j=0;j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
	    aof << m_LQ(i,j) << " ";
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"/tmp/iLQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
	{
	  for( int j=0;j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
	    aof << m_iLQ(i,j) << " ";
	  aof << endl;
	}
      aof.close();
    }
  delete [] localQ;
  delete [] localLQ;
  delete [] localiLQ;

}

void ZMPVelocityReferencedQP::computeObjective(deque<LinearConstraintInequalityFreeFeet_t> &
					       QueueOfLConstraintInequalitiesFreeFeet,
					       deque<SupportFeet_t> &QueueOfSupportFeet,
					       int NbOfConstraints, int NbOfEqConstraints,
					       int & CriteriaToMaximize, MAL_VECTOR(& xk,double), double time)
{
  m_Pb.m=NbOfConstraints;
  m_Pb.me=NbOfEqConstraints;
  m_Pb.mmax=m_Pb.m+1;
  m_Pb.n=2*(m_QP_N+m_PrwSupport.StepNumber);
  m_Pb.nmax=m_Pb.n;
  m_Pb.mnn=m_Pb.m+2*m_Pb.n;

  m_Pb.iout=0;
  m_Pb.iprint=1;
  m_Pb.lwar=3*m_Pb.nmax*m_Pb.nmax/2+ 10*m_Pb.nmax  + 2*m_Pb.mmax + 20000;
  m_Pb.liwar=m_Pb.n;
  m_Pb.Eps=1e-8;

  m_Pb.war= new double[m_Pb.lwar];
  m_Pb.iwar = new int[m_Pb.liwar]; // The Cholesky decomposition is done internally.

  if (m_FastFormulationMode==QLDANDLQ)
    m_Pb.iwar[0]=0;
  else
    m_Pb.iwar[0]=1;

  m_Pb.U = (double *)malloc( sizeof(double)*(unsigned int)m_Pb.mnn); // Returns the Lagrange multipliers.;


  MAL_MATRIX(OptA,double);
  MAL_VECTOR(VRef,double);
  MAL_MATRIX(ltermVel,double);
  MAL_VECTOR_DIM(OptD,double,2*m_QP_N);
  MAL_VECTOR_RESIZE(VRef,2*m_QP_N);



  //ZMP -------------------------------
  //Q
  MAL_MATRIX(ltermPZuPZu,double);
  MAL_MATRIX(ltermPZuU,double);
  MAL_MATRIX(ltermUU,double);
  MAL_VECTOR_RESIZE(m_Uc,m_QP_N);
  deque<LinearConstraintInequalityFreeFeet_t>::iterator LCIFF_it;//, storeFF_it, VFF_it;
  LCIFF_it = QueueOfLConstraintInequalitiesFreeFeet.begin();

  ltermPZuPZu = MAL_RET_TRANSPOSE(m_PZu);
  ltermPZuPZu = MAL_RET_A_by_B(ltermPZuPZu,m_PZu);
  ltermPZuPZu = m_Gamma*ltermPZuPZu;

  if(m_PrwSupport.StepNumber>0)
    {
      MAL_MATRIX_RESIZE(m_U,m_QP_N,m_PrwSupport.StepNumber);
      for(int i=0;i<m_QP_N;i++)
	for(int j=0;j<m_PrwSupport.StepNumber;j++)
	  m_U(i,j) = 0.0;
    }
  for(int i=0;i<m_QP_N;i++)
    m_Uc(i) = 0.0;

  for(int i=0;i<m_QP_N;i++)
    {
      if(LCIFF_it->StepNumber>0)
	m_U(i,LCIFF_it->StepNumber-1) = 1.0;
      else
	m_Uc(i) = 1.0;
      LCIFF_it++;
    }

  if (m_FullDebug>2)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"/tmp/m_U_%f.dat",time);
      aof.open(Buffer,ofstream::out);
      aof<<m_U<<endl;
      aof.close();
    }

  ltermPZuU = MAL_RET_TRANSPOSE(m_PZu);
  ltermPZuU = MAL_RET_A_by_B(ltermPZuU,m_U);
  ltermPZuU = m_Gamma*ltermPZuU;
  ltermUU = MAL_RET_TRANSPOSE(m_U);
  ltermUU = MAL_RET_A_by_B(ltermUU,m_U);
  ltermUU = m_Gamma*ltermUU;

  //pT
  deque<SupportFeet_t>::iterator SF_it;//, storeFF_it, VFF_it;
  SF_it = QueueOfSupportFeet.end();
  SF_it--;
  //pTx
  MAL_VECTOR(lterm1ZMPx,double);
  MAL_VECTOR(lterm2ZMPx,double);

  MAL_VECTOR(xkT,double);
  MAL_VECTOR_RESIZE(xkT,3);
  for(int i=0;i<3;i++)
    xkT(i)=xk(i);

  MAL_C_eq_A_by_B(lterm1ZMPx,m_PZx,xkT);
  lterm2ZMPx = m_Uc*SF_it->x;

  //m_Uc = MAL_C_eq_A_by_B(lterm2ZMPx,m_Uc,SF_it->x);

  lterm1ZMPx -= lterm2ZMPx;
  lterm1ZMPx = MAL_RET_TRANSPOSE(lterm1ZMPx);
  MAL_VECTOR(lterm3ZMPx,double);
  lterm3ZMPx = MAL_RET_A_by_B(lterm1ZMPx,m_PZu);
  lterm3ZMPx = m_Gamma*lterm3ZMPx;
  MAL_VECTOR(lterm4ZMPx,double);
  lterm4ZMPx = MAL_RET_A_by_B(lterm1ZMPx,m_U);
  lterm4ZMPx = -m_Gamma*lterm4ZMPx;

  //pTy
  MAL_VECTOR(lterm1ZMPy,double);
  MAL_VECTOR(lterm2ZMPy,double);

  MAL_VECTOR(ykT,double);
  MAL_VECTOR_RESIZE(ykT,3);
  for(int i=0;i<3;i++)
    ykT(i)=xk(3+i);

  MAL_C_eq_A_by_B(lterm1ZMPy,m_PZx,ykT);
  lterm2ZMPy = m_Uc*SF_it->y;

  lterm1ZMPy -= lterm2ZMPy;
  lterm1ZMPy = MAL_RET_TRANSPOSE(lterm1ZMPy);
  MAL_VECTOR(lterm3ZMPy,double);
  lterm3ZMPy = MAL_RET_A_by_B(lterm1ZMPy,m_PZu);
  lterm3ZMPy = m_Gamma*lterm3ZMPy;
  MAL_VECTOR(lterm4ZMPy,double);
  lterm4ZMPy = MAL_RET_A_by_B(lterm1ZMPy,m_U);
  lterm4ZMPy = -m_Gamma*lterm4ZMPy;
  //---------------------------ZMP

  //Velocity
  ltermVel = MAL_RET_TRANSPOSE(m_VPu);
  ltermVel = MAL_RET_A_by_B(ltermVel,m_VPu);
  ltermVel = m_Beta*ltermVel;

  MAL_MATRIX_RESIZE(OptA,
		    MAL_MATRIX_NB_ROWS(ltermVel),
		    MAL_MATRIX_NB_COLS(ltermVel));
  MAL_MATRIX_SET_IDENTITY(OptA);

  //Jerk
  OptA = m_Alpha*OptA;

  //Final function
  OptA = OptA + ltermVel;

  //m_Pb.Q--
  memset(m_Pb.Q,0,4*(m_QP_N+m_PrwSupport.StepNumber)*(m_QP_N+m_PrwSupport.StepNumber)*sizeof(double));
  for( int i=0;i<2*m_QP_N;i++)
    for( int j=0;j<2*m_QP_N;j++)
      m_Pb.Q[i*2*(m_QP_N+m_PrwSupport.StepNumber)+j] = OptA(j,i);
  //ZMP----
  for( int i=0;i<m_QP_N;i++)
    {
      for( int j=0;j<m_QP_N;j++)
	{
	  m_Pb.Q[i*2*(m_QP_N+m_PrwSupport.StepNumber)+j] -= ltermPZuPZu(i,j);
	  m_Pb.Q[(m_QP_N+i)*2*(m_QP_N+m_PrwSupport.StepNumber)+m_QP_N+j] -= ltermPZuPZu(i,j);
	}
    }
  if(m_PrwSupport.StepNumber>0)
    {
      for( int i=0;i<m_QP_N;i++)
	{
	  for( int j=0;j<m_PrwSupport.StepNumber;j++)
	    {
	      m_Pb.Q[i*2*(m_QP_N+m_PrwSupport.StepNumber)+2*m_QP_N+j] -= ltermPZuU(i,j);
	      m_Pb.Q[(m_QP_N+i)*2*(m_QP_N+m_PrwSupport.StepNumber)+2*m_QP_N+m_PrwSupport.StepNumber+j] -= ltermPZuU(i,j);
	      m_Pb.Q[(2*m_QP_N+j)*2*(m_QP_N+m_PrwSupport.StepNumber)+i] -= ltermPZuU(i,j);
	      m_Pb.Q[(2*m_QP_N+m_PrwSupport.StepNumber+j)*2*(m_QP_N+m_PrwSupport.StepNumber)+m_QP_N+i] -= ltermPZuU(i,j);
	    }
	}
      for( int i=0;i<m_PrwSupport.StepNumber;i++)
	{
	  for( int j=0;j<m_PrwSupport.StepNumber;j++)
	    {
	      m_Pb.Q[(2*m_QP_N+i)*2*(m_QP_N+m_PrwSupport.StepNumber)+2*m_QP_N+j] += ltermUU(i,j);
	      m_Pb.Q[(2*m_QP_N+m_PrwSupport.StepNumber+i)*2*(m_QP_N+m_PrwSupport.StepNumber)+2*m_QP_N+m_PrwSupport.StepNumber+j] += ltermUU(i,j);
	    }
	}
    }

  //----ZMP

  //TODO: - only constant velocity
  //constant velocity for the whole preview window
  for( int i=0;i<m_QP_N;i++)
    VRef(i) = RefVel.x*cos(m_TrunkState.yaw[0]+m_TrunkStateT.yaw[1]*i*m_QP_T)-
      RefVel.y*sin(m_TrunkState.yaw[0]+m_TrunkStateT.yaw[1]*i*m_QP_T);
  for( int i=m_QP_N;i<2*m_QP_N;i++)
    VRef(i) = RefVel.y*cos(m_TrunkState.yaw[0]+m_TrunkStateT.yaw[1]*i*m_QP_T)+
      RefVel.x*sin(m_TrunkState.yaw[0]+m_TrunkStateT.yaw[1]*i*m_QP_T);

  m_OptB = MAL_RET_TRANSPOSE(m_VPu);
  m_OptB = MAL_RET_A_by_B(m_OptB,m_VPx);
  m_OptB = m_Beta * m_OptB;

  //TODO 2: The matrices of the value function have to go back where they come from
  //MAL_MATRIX(m_OptD,double);
  m_OptD = MAL_RET_TRANSPOSE(m_VPu);
  m_OptD = m_Beta * m_OptD;

  //m_Pb.D-
  memset(m_Pb.D,0,2*(m_QP_N+m_PrwSupport.StepNumber)*sizeof(double));

  //velocity
  MAL_VECTOR(lterm1v,double);
  MAL_C_eq_A_by_B(lterm1v,m_OptD,VRef);
  MAL_VECTOR_RESIZE(OptD,2*m_QP_N);
  MAL_C_eq_A_by_B(OptD,m_OptB,xk);
  OptD -= lterm1v;

  for( int i=0;i<2*m_QP_N;i++)
    m_Pb.D[i] += OptD(i);


  //zmp
  for( int i=0;i<m_QP_N;i++)
    {
      m_Pb.D[i] += lterm3ZMPx(i);
      m_Pb.D[m_QP_N+i] += lterm3ZMPy(i);
    }
  for( int i=0;i<m_PrwSupport.StepNumber;i++)
    {
      m_Pb.D[2*m_QP_N+i] += lterm4ZMPx(i);
      m_Pb.D[2*m_QP_N+m_PrwSupport.StepNumber+i] += lterm4ZMPy(i);
    }
  //----------m_Pb.D
  for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
    {
      m_Pb.XL[i] = -1e8;
      m_Pb.XU[i] = 1e8;
    }
  memset(m_Pb.X,0,2*(m_QP_N+m_PrwSupport.StepNumber)*sizeof(double));
}

void ZMPVelocityReferencedQP::interpolateTrunkState(double time, int CurrentIndex,
						    deque<COMState> & FinalCOMStates)
{
  if(m_CurrentSupport.Phase == 1 && time+m_TimeBuffer+3.0/2.0*m_QP_T < m_CurrentSupport.TimeLimit)
    {
      //Set parameters for trunk interpolation
      m_c = 3.0*(m_TrunkStateT.yaw[1]-m_TrunkState.yaw[1])/(m_QP_T*m_QP_T);
      m_d = -2.0*m_c/(3.0*m_QP_T);
      m_a =  m_TrunkState.yaw[1];

      double tT;
      double Theta = m_TrunkState.yaw[0];
      //double dTheta = m_TrunkState.yaw[1];
      //double ddTheta = m_TrunkState.yaw[2];

      FinalCOMStates[CurrentIndex].yaw[0] = m_TrunkState.yaw[0];
      //Interpolate the
      for(int k = 1; k<=(int)(m_QP_T/m_SamplingPeriod);k++)
	{
	  tT = (double)k*m_SamplingPeriod;
	  //interpolate the orientation of the trunk
	  if(fabs(m_TrunkStateT.yaw[1]-m_TrunkState.yaw[1])-0.000001 > 0)
	    {
	      m_TrunkState.yaw[0] = (((1.0/4.0*m_d*tT+1.0/3.0*m_c)*
				      tT)*tT+m_a)*tT+Theta;
	      m_TrunkState.yaw[1] = ((m_d*tT+m_c)*tT)*tT+m_a;
	      m_TrunkState.yaw[2] = (3.0*m_d*tT+2.0*m_c)*tT;

	      m_QueueOfTrunkStates.push_back(m_TrunkState);
	    }
	  else
	    {
	      m_TrunkState.yaw[0] += m_SamplingPeriod*m_TrunkStateT.yaw[1];

	      m_QueueOfTrunkStates.push_back(m_TrunkState);
	    }
	  FinalCOMStates[CurrentIndex+k].yaw[0] = m_TrunkState.yaw[0];
	  if(m_FullDebug>2)
	    {
	      ofstream aof;
	      aof.open("/tmp/Trunk.dat",ofstream::app);
	      aof<<time+k*m_SamplingPeriod<<" "<<m_TrunkState.yaw[0]<<" "<<m_TrunkState.yaw[1]<<" "<<m_TrunkState.yaw[2]<<endl;
	      aof.close();
	    }
	}
    }
  else if (m_CurrentSupport.Phase == 0 || time+m_TimeBuffer+3.0/2.0*m_QP_T > m_CurrentSupport.TimeLimit)
    {
      for(int k = 0; k<=(int)(m_QP_T/m_SamplingPeriod);k++)
	{
	  FinalCOMStates[CurrentIndex+k].yaw[0] = m_TrunkState.yaw[0];
	}
    }
}


void ZMPVelocityReferencedQP::interpolateFeetPositions(double time, int CurrentIndex,
						       deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
						       deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)
{
  double LocalInterpolationTime = (time+m_TimeBuffer)-(m_CurrentSupport.TimeLimit-m_SupportFSM->m_SSPeriod);

  double StepHeight = 0.05;
  int StepType = 1;

  if(m_CurrentSupport.Phase == 1 && time+m_TimeBuffer+3.0/2.0*m_QP_T < m_CurrentSupport.TimeLimit)
    {
      //determine coefficients of interpolation polynom
      double ModulationSupportCoefficient = 0.9;
      double ModulatedSingleSupportTime = (m_SupportFSM->m_SSPeriod-m_QP_T) * ModulationSupportCoefficient;
      double EndOfLiftOff = ((m_SupportFSM->m_SSPeriod-m_QP_T)-ModulatedSingleSupportTime)*0.5;
      double InterpolationTimePassed = 0.0;
      if(LocalInterpolationTime>EndOfLiftOff)
	InterpolationTimePassed = LocalInterpolationTime-EndOfLiftOff;

      FootAbsolutePosition LastSwingFootPosition;

      if(m_CurrentSupport.Foot==1)
	{
	  LastSwingFootPosition = FinalRightFootAbsolutePositions[CurrentIndex];
	}
      else
	{
	  LastSwingFootPosition = FinalLeftFootAbsolutePositions[CurrentIndex];
	}
      //Set parameters for current polynomial
      m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::X_AXIS,
						ModulatedSingleSupportTime-InterpolationTimePassed,m_FPx,
						LastSwingFootPosition.x,
						LastSwingFootPosition.dx);
      m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::Y_AXIS,
						ModulatedSingleSupportTime-InterpolationTimePassed,m_FPy,
						LastSwingFootPosition.y,
						LastSwingFootPosition.dy);

      if(m_CurrentSupport.StateChanged==true)
	m_FTGS->SetParameters(FootTrajectoryGenerationStandard::Z_AXIS, m_SupportFSM->m_SSPeriod-m_QP_T,StepHeight);

      m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::THETA_AXIS,
						ModulatedSingleSupportTime-InterpolationTimePassed,
						m_PreviewedSupportAngles[0]*180.0/M_PI,
						LastSwingFootPosition.theta,
						LastSwingFootPosition.dtheta);
      m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::OMEGA_AXIS,
						ModulatedSingleSupportTime-InterpolationTimePassed,0.0*180.0/M_PI,
						LastSwingFootPosition.omega,
						LastSwingFootPosition.domega);
      m_FTGS->SetParametersWithInitPosInitSpeed(FootTrajectoryGenerationStandard::OMEGA2_AXIS,
						ModulatedSingleSupportTime-InterpolationTimePassed,2*0.0*180.0/M_PI,
						LastSwingFootPosition.omega2,
						LastSwingFootPosition.domega2);

      for(int k = 1; k<=(int)(m_QP_T/m_SamplingPeriod);k++)
	{
	  if (m_CurrentSupport.Foot==1)
	    {
	      m_FTGS->UpdateFootPosition(FinalLeftFootAbsolutePositions,
					 FinalRightFootAbsolutePositions,
					 CurrentIndex,k,
					 LocalInterpolationTime,
					 ModulatedSingleSupportTime,
					 StepType, -1);
	    }
	  else
	    {
	      m_FTGS->UpdateFootPosition(FinalRightFootAbsolutePositions,
					 FinalLeftFootAbsolutePositions,
					 CurrentIndex,k,
					 LocalInterpolationTime,
					 ModulatedSingleSupportTime,
					 StepType, 1);
	    }
	  FinalLeftFootAbsolutePositions[CurrentIndex+k].time =
	    FinalRightFootAbsolutePositions[CurrentIndex+k].time = time+m_TimeBuffer+k*m_SamplingPeriod;


	  if(m_FullDebug>0)
	    {
	      ofstream aoffeet;
	      aoffeet.open("/tmp/Feet.dat",ios::app);
	      aoffeet<<time+m_TimeBuffer+k*m_SamplingPeriod<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].x<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].y<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].z<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].stepType<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].x<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].y<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].z<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].stepType<<"    "
		     <<endl;
	      aoffeet.close();
	    }

	}
    }
  else if (m_CurrentSupport.Phase == 0 || time+m_TimeBuffer+3.0/2.0*m_QP_T > m_CurrentSupport.TimeLimit)
    {
      for(int k = 0; k<=(int)(m_QP_T/m_SamplingPeriod);k++)
	{
	  FinalRightFootAbsolutePositions[CurrentIndex+k]=FinalRightFootAbsolutePositions[CurrentIndex+k-1];
	  FinalLeftFootAbsolutePositions[CurrentIndex+k]=FinalLeftFootAbsolutePositions[CurrentIndex+k-1];
	  FinalLeftFootAbsolutePositions[CurrentIndex+k].time =
	    FinalRightFootAbsolutePositions[CurrentIndex+k].time = time+m_TimeBuffer+k*m_SamplingPeriod;
	  FinalLeftFootAbsolutePositions[CurrentIndex+k].stepType =
	    FinalRightFootAbsolutePositions[CurrentIndex+k].stepType = 10;

	  if(m_FullDebug>0)
	    {
	      ofstream aoffeet;
	      aoffeet.open("/tmp/Feet.dat",ios::app);
	      aoffeet<<time+m_TimeBuffer+k*m_SamplingPeriod<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].x<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].y<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].z<<"    "
		     <<FinalLeftFootAbsolutePositions[CurrentIndex+k].stepType<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].x<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].y<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].z<<"    "
		     <<FinalRightFootAbsolutePositions[CurrentIndex+k].stepType<<"    "
		     <<endl;
	      aoffeet.close();
	    }
	}
    }
}


void ZMPVelocityReferencedQP::OnLine(double time,
				     deque<ZMPPosition> & FinalZMPPositions,
				     deque<COMState> & FinalCOMStates,
				     deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
				     deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)
{

  if(time + 0.00001 > m_UpperTimeLimitToUpdate)
    {
      int NbOfConstraints=0; // Nb of constraints are not known in advance

      MAL_VECTOR_DIM(xk,double,6);

      int CriteriaToMaximize=1;


      deque<LinearConstraintInequality_t> QueueOfLConstraintInequalities;
      deque<LinearConstraintInequalityFreeFeet_t> QueueOfLConstraintInequalitiesFreeFeet;
      deque<LinearConstraintInequalityFreeFeet_t> QueueOfFeetPosInequalities;

      // pre compute the matrices needed for the optimization.
      double TotalAmountOfCPUTime=0.0,CurrentCPUTime=0.0;
      struct timeval start,end;

      bool StartingSequence = true;

      //----------"Real-time" loop---------
      //
      //
      //-----------------------------------
      gettimeofday(&start,0);


      m_OP->verifyAccelerationOfHipJoint(RefVel, m_TrunkState,
					 m_TrunkStateT, m_CurrentSupport);


      m_OP->previewOrientations(time+m_TimeBuffer,
				m_PreviewedSupportAngles,
				m_TrunkState,
				m_TrunkStateT,
				m_SupportFSM, m_CurrentSupport,
				FinalLeftFootAbsolutePositions,
				FinalRightFootAbsolutePositions);


      // Read the current state of the 2D Linearized Inverted Pendulum.
      m_2DLIPM->GetState(xk);


      //Apply external forces
      if(m_PerturbationOccured == true)
	{
	  xk(2) = xk(2)+m_PerturbationAcceleration(2);
	  xk(5) = xk(5)+m_PerturbationAcceleration(5);
	  m_PerturbationOccured = false;
	}
      m_2DLIPM->setState(xk);


      //TODO : Add a get function to read the state
      m_SupportFSM->setSupportState(time+m_TimeBuffer, 0, m_CurrentSupport, RefVel);

	  deque<FootAbsolutePosition>::iterator FAP_it;
	  SupportFeet_t newSF;
	  if(m_Support->CurrentSupportFoot==1)
	    {
	      FAP_it = FinalLeftFootAbsolutePositions.end();
	      FAP_it--;
	    }
	  else
	    {
	      FAP_it = FinalRightFootAbsolutePositions.end();
	      FAP_it--;
	    }


	  newSF.x = FAP_it->x;
	  newSF.y = FAP_it->y;
	  newSF.theta = FAP_it->theta*M_PI/180.0;
	  newSF.StartTime = time+m_TimeBuffer;
	  newSF.SupportFoot = m_Support->CurrentSupportFoot;

	  QueueOfSupportFeet.push_back(newSF);
	}

      //      //TODO : Temporary solution for the pldp solver. See above
      //      bool CurrentStateChanged = m_SupportFSM->m_StateChanged;
      if (m_FullDebug>2)
	{
	  ofstream aof;
	  char Buffer[1024];
	  sprintf(Buffer,"/tmp/m_CurrentSupport_%f.dat",time);
	  aof.open(Buffer,ofstream::out);
	  aof<<"Phase: "<<m_CurrentSupport.Phase<<" Foot: "<<m_CurrentSupport.Foot<<
	    " TimeLimit: "<<m_CurrentSupport.TimeLimit<<" StepNumber: "<<m_CurrentSupport.StepNumber<<
	    " SSSS: "<<m_CurrentSupport.SSSS<<" StateChanged: "<<m_CurrentSupport.StateChanged<<endl;
	  aof.close();
	}

      m_fCALS->buildLinearConstraintInequalities(FinalLeftFootAbsolutePositions,
						 FinalRightFootAbsolutePositions,
						 QueueOfLConstraintInequalitiesFreeFeet,
						 QueueOfFeetPosInequalities,
						 RefVel,
						 time+m_TimeBuffer,
						 m_QP_N,
						 m_Support, m_PreviewedSupportAngles,
						 NbOfConstraints);

      //Add a new support foot to the support feet history deque
      if(m_CurrentSupport.StateChanged == true)
	{
	  deque<FootAbsolutePosition>::iterator FAP_it;
	  SupportFeet_t newSF;
	  if(m_CurrentSupport.Foot==1)
	    {
	      FAP_it = FinalLeftFootAbsolutePositions.end();
	      FAP_it--;
	    }
	  else
	    {
	      FAP_it = FinalRightFootAbsolutePositions.end();
	      FAP_it--;
	    }

	  newSF.x = FAP_it->x;
	  newSF.y = FAP_it->y;
	  newSF.theta = FAP_it->theta*M_PI/180.0;
	  newSF.StartTime = time+m_TimeBuffer;
	  newSF.SupportFoot = m_CurrentSupport.Foot;

	  QueueOfSupportFeet.push_back(newSF);
	}


      m_fCALS->buildLinearConstraintInequalities(FinalLeftFootAbsolutePositions,
						 FinalRightFootAbsolutePositions,
						 QueueOfLConstraintInequalitiesFreeFeet,
						 QueueOfFeetPosInequalities,
						 RefVel,
						 time+m_TimeBuffer,
						 m_QP_N,
						 m_SupportFSM, m_CurrentSupport, m_PrwSupport, m_PreviewedSupportAngles,
						 NbOfConstraints);


      initializeProblem();


      computeObjective(QueueOfLConstraintInequalitiesFreeFeet, QueueOfSupportFeet,
		       NbOfConstraints, 0, CriteriaToMaximize, xk, time);


      if(m_FastFormulationMode == PLDPHerdt)
	{
	  computeCholeskyOfQ(m_Pb.Q);
	  buildConstraintMatricesPLDPHerdt();
	}

      interpolateTrunkState(time, CurrentIndex,
          		FinalCOMStates);

      buildConstraintMatrices(m_Pb.DS,m_Pb.DU,
			      m_QP_T,
			      time+m_TimeBuffer,
			      QueueOfLConstraintInequalitiesFreeFeet,
			      QueueOfFeetPosInequalities,
			      QueueOfSupportFeet,
			      m_ComHeight,
			      NbOfConstraints,
			      xk);
      if(m_FullDebug>2)
	dumpProblem(m_Pb.Q, m_Pb.D, m_Pb.DU, m_Pb.m, m_Pb.DS, m_Pb.XL, m_Pb.XU, xk, time+m_TimeBuffer);


      double ldt = 0.0;
      //---------Solver------------
      //
      //
      //---------------------------
      if ((m_FastFormulationMode==QLDANDLQ)||
	  (m_FastFormulationMode==QLD))
	{
	  struct timeval lbegin,lend;
	  gettimeofday(&lbegin,0);
	  ql0001_(&m_Pb.m, &m_Pb.me, &m_Pb.mmax, &m_Pb.n, &m_Pb.nmax, &m_Pb.mnn,
		  m_Pb.Q, m_Pb.D, m_Pb.DU, m_Pb.DS, m_Pb.XL, m_Pb.XU,
		  m_Pb.X, m_Pb.U, &m_Pb.iout, &m_Pb.ifail, &m_Pb.iprint,
		  m_Pb.war, &m_Pb.lwar, m_Pb.iwar, &m_Pb.liwar, &m_Pb.Eps);
	  gettimeofday(&lend,0);

	  ldt = lend.tv_sec - lbegin.tv_sec +
	    0.000001 * (lend.tv_usec - lbegin.tv_usec);

	  int NbOfActivatedConstraints = 0;
	  for(int lk=0;lk<m_Pb.m;lk++)
	    {
	      if (m_Pb.U[lk]>0.0)
		{
		  NbOfActivatedConstraints++;
		}
	    }
	  ODEBUG6(NbOfActivatedConstraints,"/tmp/InfosQLD.dat");
	  ODEBUG6(ldt,"/tmp/dtQLD.dat");
	}
      else if (m_FastFormulationMode==PLDPHerdt)
	{
	  ODEBUG("State: " << xk[0] << " " << xk[3] << " " <<
		 xk[1] << " " << xk[4] << " " <<
		 xk[2] << " " << xk[5] << " ");
	  struct timeval lbegin,lend;
	  gettimeofday(&lbegin,0);


	  if(m_PLDPSolverHerdt==0)
	    m_PLDPSolverHerdt = new Optimization::Solver::PLDPSolverHerdt((unsigned int)m_QP_N,
									  MAL_RET_MATRIX_DATABLOCK(m_iPu),
									  MAL_RET_MATRIX_DATABLOCK(m_Px),
									  m_Pu,
									  MAL_RET_MATRIX_DATABLOCK(m_iLQ));


	  unsigned int NumberOfRemovedConstraints = 4; unsigned int NbRemovedFootCstr = 5;

	  m_Pb.ifail=m_PLDPSolverHerdt->SolveProblem(QueueOfLConstraintInequalitiesFreeFeet, QueueOfSupportFeet,
						     m_Pb.D,
						     (unsigned int)m_Pb.m,
						     m_Pb.DU,
						     m_Pb.DS,
						     MAL_RET_VECTOR_DATABLOCK(xk),m_Pb.X,
						     NumberOfRemovedConstraints, NbRemovedFootCstr,
						     StartingSequence,
						     (unsigned int)m_PrwSupport.StepNumber,
						     m_CurrentSupport.StateChanged, time);
	  StartingSequence = false;
	  //NumberOfRemovedConstraints = NextNumberOfRemovedConstraints;
	  gettimeofday(&lend,0);
	  // 	  CODEDEBUG6(double ldt = lend.tv_sec - lbegin.tv_sec +
	  // 		     0.000001 * (lend.tv_usec - lbegin.tv_usec););

	  // 	  ODEBUG6(ldt,"/tmp/dtPLDP.dat");
	}
      if (m_Pb.ifail!=0)
	{
	  cout << "IFAIL: " << m_Pb.ifail << " at time: " << time << endl;
	  //return -1;
	}

      double *ptX=0;
      if ((m_FastFormulationMode==QLDANDLQ)||
	  (m_FastFormulationMode==PLDPHerdt))
	{
	  /* Multiply the solution by the transpose of iLQ
      	     because it is a triangular matrix we do a specific
      	     multiplication.
	  */
	  memset(m_Pb.NewX,0,2*(m_QP_N+m_PrwSupport.StepNumber)*sizeof(double));

	  double *pm_iLQ = MAL_RET_MATRIX_DATABLOCK(m_iLQ);
	  double *pNewX = m_Pb.NewX;

	  for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
	    {
	      double *pX= m_Pb.X+i;
	      double *piLQ = pm_iLQ+i*2*(m_QP_N+m_PrwSupport.StepNumber)+i;
	      *pNewX = 0.0;
	      for(int j=i;j<2*(m_QP_N+m_PrwSupport.StepNumber);j++)
		{
		  *pNewX+= (*piLQ) * (*pX++);
		  piLQ+=2*(m_QP_N+m_PrwSupport.StepNumber);
		}
	      pNewX++;
	    }
	  ptX=m_Pb.NewX;
	}
      else
	ptX=m_Pb.X;
      //------------------------
      //
      //
      //-------------------------


      FinalCOMStates.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      FinalZMPPositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      FinalLeftFootAbsolutePositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      FinalRightFootAbsolutePositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));

      //TODO: The variable CurrentIndex might be obsolete as it introduces a buffer which might be unnecessary.
      int CurrentIndex = (int)(m_TimeBuffer/m_SamplingPeriod)
	-1
	//	-(int)(ldt/m_SamplingPeriod)-1 //<- This part is supposed to be equal to zero.
	;
      ODEBUG("m_TimeBuffer: "<< m_TimeBuffer <<
	     " m_SamplingPeriod: "<< m_SamplingPeriod <<
	     " ldt: " << ldt);
      ODEBUG("ldt: "<<ldt<<
	     "(int)(ldt/m_SamplingPeriod): "<<(int)(ldt/m_SamplingPeriod)<<
	     "(ldt/m_SamplingPeriod): "<<(ldt/m_SamplingPeriod));
      // update the ZMP and COM positions.
      ODEBUG("m_TimeBuffer/m_SamplingPeriod: "<<
	     m_TimeBuffer/m_SamplingPeriod<<
	     "(int)(m_TimeBuffer/m_SamplingPeriod): "<<
	     (int)(m_TimeBuffer/m_SamplingPeriod));


      m_2DLIPM->Interpolation(FinalCOMStates,
			      FinalZMPPositions,
			      CurrentIndex,
			      ptX[0],ptX[m_QP_N]);
      m_2DLIPM->OneIteration(ptX[0],ptX[m_QP_N]);


      //The robot is supposed to stop always with the feet aligned in the lateral plane.
      if(m_CurrentSupport.StepsLeft>0)
	{
	  if(fabs(ptX[2*m_QP_N])-0.00001<0.0)
	    {
	      cout<<"Previewed foot position zero at time: "<<time<<endl;
	    }
	  else if (m_CurrentSupport.TimeLimit-time-m_QP_T/2.0>0)
	    {//The landing position is yet determined by the solver because the robot finds himself still in the single support phase
	      m_FPx = ptX[2*m_QP_N];
	      m_FPy = ptX[2*m_QP_N+m_PrwSupport.StepNumber];
	    }
	}
      else
	{//The solver isn't responsible for the feet positions anymore
	  deque<SupportFeet_t>::iterator CurSF_it;
	  CurSF_it = QueueOfSupportFeet.end();
	  CurSF_it--;
	  while(CurSF_it->SupportFoot!=m_CurrentSupport.Foot)
	    CurSF_it--;
	  m_FPx = CurSF_it->x + double(CurSF_it->SupportFoot)*sin(CurSF_it->theta)*m_FeetDistanceDS;
	  m_FPy = CurSF_it->y - double(CurSF_it->SupportFoot)*cos(CurSF_it->theta)*m_FeetDistanceDS;
	}


      if (m_FullDebug>2)
	{
	  ofstream aof;

	  aof.open("/tmp/FootPositionsT.dat",ofstream::app);
	  aof<<" "<<m_FPx<<" "<<m_FPy<<endl;
	  aof.close();
	  char Buffer[1024];
	  if(m_FastFormulationMode == PLDPHerdt)
	    sprintf(Buffer,"/tmp/PLDPXff_%f.dat",time);
	  else
	    sprintf(Buffer,"/tmp/Xff_%f.dat",time);
	  aof.open(Buffer,ofstream::out);

	  for(int i=0;i<m_QP_N;i++)
	    {
	      aof << ptX[i] << endl;
	    }
	  aof.close();
	  if(m_FastFormulationMode == PLDPHerdt)
	    sprintf(Buffer,"/tmp/PLDPYff_%f.dat",time);
	  else
	    sprintf(Buffer,"/tmp/Yff_%f.dat",time);
	  aof.open(Buffer,ofstream::out);
	  for(int i=m_QP_N+m_PrwSupport.StepNumber;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
	    {
	      aof << ptX[i] << endl;
	    }
	  aof.close();
	  aof.open("/tmp/comHeight.dat",ofstream::app);

	  aof << FinalCOMStates[CurrentIndex].x[0]<<" "
	      << FinalCOMStates[CurrentIndex].y[0]<< " "
	      << FinalCOMStates[CurrentIndex].z[0] << " "
	      << FinalCOMStates[CurrentIndex].roll <<  endl;

	  aof.close();
	  aof.open("/tmp/CurrentIndex.dat",ofstream::app);
	  aof<<CurrentIndex<<endl;
	  aof.close();
	}

      //TODO :Jumps of 5ms
      if(m_FullDebug>2)
	{
	  ofstream aof;
	  aof.open("/tmp/time.dat",ios::app);
	  aof<<time<<" "<<m_UpperTimeLimitToUpdate<<endl;
	}

      if(m_FullDebug>2)
	{
	  //if(validateConstraints(m_Pb.DS, m_Pb.DU, m_Pb.m, li, m_Pb.X, time)<0)
	  //  {
	  //    cout << "Something is wrong with the constraints." << endl;
	  //    exit(-1);
	  //  }
	}

      interpolateTrunkState(time, CurrentIndex,
			    FinalCOMStates);
      interpolateFeetPositions(time, CurrentIndex,
			       FinalLeftFootAbsolutePositions,
			       FinalRightFootAbsolutePositions);


      if(m_UpperTimeLimitToUpdate==0.0)
	m_UpperTimeLimitToUpdate = time+m_QP_T;
      else
	m_UpperTimeLimitToUpdate = m_UpperTimeLimitToUpdate+m_QP_T;

      ODEBUG6("uk:" << uk,"/tmp/DebugPBW.dat");
      ODEBUG6("xk:" << xk,"/tmp/DebugPBW.dat");


      if (m_FullDebug>2)
	{
	  ofstream aof;
	  char Buffer[1024];
	  sprintf(Buffer,"/tmp/Xff_%f.dat",time);
	  aof.open(Buffer,ofstream::out);
	  for( int i=0;i<2*(m_QP_N+m_PrwSupport.StepNumber);i++)
	    {
	      aof << m_Pb.X[i] << endl;
	    }
	  aof.close();
	}

      if(m_FullDebug>2)
	{
	  //if(validateConstraints(m_Pb.DS, m_Pb.DU, m_Pb.m, li, m_Pb.X, time)<0)
	  //  {
	  //    cout << "Something is wrong with the constraints." << endl;
	  //    exit(-1);
	  //  }
	}

      // Compute CPU consumption time.
      gettimeofday(&end,0);
      CurrentCPUTime = end.tv_sec - start.tv_sec +
        0.000001 * (end.tv_usec - start.tv_usec);
      TotalAmountOfCPUTime += CurrentCPUTime;
      ODEBUG("Current Time : " <<time << " " <<
	     " Virtual time to simulate: " << QueueOfLConstraintInequalities.back()->EndingTime - time <<
	     "Computation Time " << CurrentCPUTime << " " << TotalAmountOfCPUTime);

      QueueOfLConstraintInequalitiesFreeFeet.clear();
      QueueOfFeetPosInequalities.clear();

      delete [] m_Pb.Q;
      delete [] m_Pb.D;
      delete [] m_Pb.DS;
      delete [] m_Pb.DU;
      delete [] m_Pb.XL;
      delete [] m_Pb.XU;
      delete [] m_Pb.X;
      delete [] m_Pb.NewX;
      delete [] m_Pb.iwar;

      delete [] m_Pb.war;
      free(m_Pb.U);

    }
  //-----------------------------------
  //
  //
  //----------"Real-time" loop--------

}


void ZMPVelocityReferencedQP::GetZMPDiscretization(deque<ZMPPosition> & ZMPPositions,
						   deque<COMState> & COMStates,
						   deque<RelativeFootPosition> &RelativeFootPositions,
						   deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
						   deque<FootAbsolutePosition> &RightFootAbsolutePositions,
						   double Xmax,
						   COMState & lStartingCOMState,
						   MAL_S3_VECTOR(&,double) lStartingZMPPosition,
						   FootAbsolutePosition & InitLeftFootAbsolutePosition,
						   FootAbsolutePosition & InitRightFootAbsolutePosition)
{
}


void ZMPVelocityReferencedQP::OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
					    deque<ZMPPosition> & FinalZMPPositions,
					    deque<COMState> & FinalCOMStates,
					    deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
					    deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
					    bool EndSequence)
{
  cout << "To be implemented" << endl;
}

int ZMPVelocityReferencedQP::OnLineFootChange(double time,
					      FootAbsolutePosition &aFootAbsolutePosition,
					      deque<ZMPPosition> & FinalZMPPositions,
					      deque<COMState> & CoMPositions,
					      deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
					      deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
					      StepStackHandler  *aStepStackHandler)
{
  cout << "To be implemented" << endl;
  return -1;
}

void ZMPVelocityReferencedQP::EndPhaseOfTheWalking(deque<ZMPPosition> &ZMPPositions,
						   deque<COMState> &FinalCOMStates,
						   deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
						   deque<FootAbsolutePosition> &RightFootAbsolutePositions)
{
  cout << "To be implemented" << endl;
  return -1;

}

int ZMPVelocityReferencedQP::ReturnOptimalTimeToRegenerateAStep()
{
  int r = (int)(m_PreviewControlTime/m_SamplingPeriod);
  return 2*r;
}



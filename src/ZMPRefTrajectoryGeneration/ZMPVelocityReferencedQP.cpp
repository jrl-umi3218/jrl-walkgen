/* This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of steps following a QP
   formulation and a new QP solver as proposed by Dimitrov ICRA 2009.

   Copyright (c) 2009,
   Olivier Stasse,

   JRL-Japan, CNRS/AIST

   All rights reserved.

   See License.txt for more information on license.

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
  // printf("Entered ZMPVelocityReferencedQP \n");
  m_Q = 0;
  m_Pu = 0;
  m_FullDebug = 3;
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
  m_fCALS = new footConstraintsAsLinearSystem(lSPM,aHS,m_ConstraintOnX,m_ConstraintOnY);

  m_StartTime = 0.0;
  m_UpperTimeLimitToUpdate = 0.0;
  m_TimeBuffer = 0.040;

  m_FTGS = new footTrajectoryGenerationStandard(lSPM,aHS->leftFoot());
  m_FTGS->InitializeInternalDataStructures();

  /* Initialize the FSM */
  m_Support = new SupportState(m_QP_T);

  /* Orientations preview algorithm*/
  m_OP = new OrientationsPreview(m_QP_T, m_QP_N, m_Support->SSPeriod, aHS->rootJoint());

  m_TrunkState.yaw[0]=m_TrunkState.yaw[1]=m_TrunkState.yaw[2]=0.0;

  //	m_PreviewedSupportAngles = new double[(int)ceil((m_QP_N+1)*m_QP_T/m_Support->SSPeriod)];
  //	memset(m_PreviewedSupportAngles,0,(int)ceil((m_QP_N+1)*m_QP_T/m_Support->SSPeriod)*sizeof(double));



  // if (m_FastFormulationMode==PLDP)
  //   m_PLDPSolverHerdt = new Optimization::Solver::PLDPSolver(m_QP_N,
  // 							MAL_RET_MATRIX_DATABLOCK(m_iPu),
  // 							MAL_RET_MATRIX_DATABLOCK(m_Px),
  // 							m_Pu,
  // 							MAL_RET_MATRIX_DATABLOCK(m_iLQ));
  // else
  // m_PLDPSolverHerdt =0;

  /* Initialize  the 2D LIPM */
  m_2DLIPM->SetSimulationControlPeriod(m_QP_T);
  m_2DLIPM->SetRobotControlPeriod(m_SamplingPeriod);
  m_2DLIPM->SetComHeight(m_ComHeight);
  m_2DLIPM->InitializeSystem();

  m_Alpha = 0.00001;
  m_Beta = 1.0;
  m_Gamma =0.0;//0.000001;

  InitConstants();

  initFeet();
  // PLDP Solver needs iPu and Px.

  m_SimilarConstraints.resize(8*m_QP_N);


  // Register method to handle
  string aMethodName[1] =
    {":previewcontroltime"};

  for(int i=0;i<1;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
    }

  if (m_FastFormulationMode==QLDANDLQ)
    {
      RESETDEBUG6("dtQLD.dat");
      RESETDEBUG6("InfosQLD.dat");
      RESETDEBUG6("Check2DLIPM_QLDANDLQ.dat");
    }


  if (m_FastFormulationMode==PLDP)
    {
      RESETDEBUG6("dtPLDP.dat");
      RESETDEBUG6("Check2DLIPM_PLDP.dat");
    }

  if(m_FullDebug>2)
    {
      ofstream aof;
      aof.open("Trunk.dat",ofstream::out);
      aof.close();
      aof.open("time.dat",ofstream::out);
      aof.close();
    }

  //Feet distance in the DS phase
  m_FeetDistanceDS = 0.2;
  // printf("Leaving ZMPVelocityReferencedQP \n");
}

ZMPVelocityReferencedQP::~ZMPVelocityReferencedQP()
{

  //	printf("Entered ~ZMPVelocityReferencedQP \n");

  if (m_ZMPD!=0)
    delete m_ZMPD;

  if (m_2DLIPM!=0)
    delete m_2DLIPM;

  if (m_Support!=0)
    delete m_Support;

  if (m_fCALS!=0)
    delete m_fCALS;

  if (m_FTGS!=0)
    delete m_FTGS;

  if (m_Q!=0)
    delete [] m_Q;

  //	if (m_PreviewedSupportAngles!=0)
  //		delete [] m_PreviewedSupportAngles;

  if (m_OP!=0)
    delete m_OP;



  //if (m_Pb!=0)
  //		delete m_Pb;

  // if (m_PLDPSolverHerdt!=0)
  //   delete m_PLDPSolverHerdt;

  if (m_Pu!=0)
    delete [] m_Pu ;


  //	printf("Leaving ~ZMPVelocityReferencedQP \n");
}


void ZMPVelocityReferencedQP::setVelReference(istringstream &strm)
{
  strm >> RefVel.x;
  strm >> RefVel.y;
  strm >> RefVel.dYaw;
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
		aof.open("VPx.dat");
		aof << m_VPx;
		aof.close();

		aof.open("m_PPx.dat");
		aof << m_PPx;
		aof.close();

		aof.open("VPu.dat");
		aof << m_VPu;
		aof.close();

		aof.open("PPu.dat");
		aof << m_PPu;
		aof.close();

		aof.open("PZu.dat");
		aof << m_PZu;
		aof.close();
	}

	return 0;
}


// int ZMPVelocityReferencedQP::InitializeMatrixPbConstants()
// {
//   MAL_MATRIX_RESIZE(m_PPu,2*m_QP_N,2*m_QP_N);
//   MAL_MATRIX_RESIZE(m_PZu,m_QP_N,m_QP_N);
//   MAL_MATRIX_RESIZE(m_VPu,2*m_QP_N,2*m_QP_N);
//   MAL_MATRIX_RESIZE(m_PPx,2*m_QP_N,6);
//   MAL_MATRIX_RESIZE(m_PZx,2*m_QP_N,6);
//   MAL_MATRIX_RESIZE(m_VPx,2*m_QP_N,6);

//   for( int i=0;i<m_QP_N;i++)
//     {
//       // Compute VPx and PPx
//       m_VPx(i,0)   = 0.0;   m_VPx(i,1) =     1.0; m_VPx(i,2)   = (i+1)*m_QP_T;
//       m_VPx(i,3)   = 0.0;   m_VPx(i,4) =     0.0; m_VPx(i,5)   = 0.0;
//       m_VPx(i+m_QP_N,0) = 0.0;   m_VPx(i+m_QP_N,1) =   0.0; m_VPx(i+m_QP_N,2) = 0.0;
//       m_VPx(i+m_QP_N,3) = 0.0;   m_VPx(i+m_QP_N,4) =   1.0; m_VPx(i+m_QP_N,5) = (i+1)*m_QP_T;

//       m_PPx(i,0) = 1.0; m_PPx(i,1)     = (i+1)*m_QP_T; m_PPx(i,2) = (i+1)*(i+1)*m_QP_T*m_QP_T*0.5;
//       m_PPx(i,3) = 0.0; m_PPx(i,4)     =       0; m_PPx(i,5) = 0.;
//       m_PPx(i+m_QP_N,0) = 0.0; m_PPx(i+m_QP_N,1) =     0.0; m_PPx(i+m_QP_N,2) = 0.0;
//       m_PPx(i+m_QP_N,3) = 1.0; m_PPx(i+m_QP_N,4) = (i+1)*m_QP_T; m_PPx(i+m_QP_N,5) = (i+1)*(i+1)*m_QP_T*m_QP_T*0.5;


//       for( int j=0;j<m_QP_N;j++)
// 	{
// 	  m_PPu(i,j)=0;

// 	  if (j<=i)
// 	    {

// 	      m_VPu(i,j)= (2*(i-j)+1)*m_QP_T*m_QP_T*0.5 ;
// 	      m_VPu(i+m_QP_N,j+m_QP_N)= (2*(i-j)+1)*m_QP_T*m_QP_T*0.5 ;
// 	      m_VPu(i,j+m_QP_N)=0.0;
// 	      m_VPu(i+m_QP_N,j)=0.0;


// 	      m_PPu(i,j)= (1 + 3*(i-j) + 3*(i-j)*(i-j)) * m_QP_T*m_QP_T*m_QP_T/6.0;
// 	      m_PPu(i+m_QP_N,j+m_QP_N)= (1 + 3*(i-j) + 3*(i-j)*(i-j)) * m_QP_T*m_QP_T*m_QP_T/6.0;
// 	      m_PPu(i,j+m_QP_N)=0.0;
// 	      m_PPu(i+m_QP_N,j)=0.0;

// 	    }
// 	  else
// 	    {

// 	      m_VPu(i,j) = 0.0;
// 	      m_VPu(i+m_QP_N,j+m_QP_N)=0.0;
// 	      m_VPu(i,j+m_QP_N)=0.0;
// 	      m_VPu(i+m_QP_N,j)=0.0;

// 	      m_PPu(i,j) = 0.0;
// 	      m_PPu(i+m_QP_N,j+m_QP_N)=0.0;
// 	      m_PPu(i,j+m_QP_N)=0.0;
// 	      m_PPu(i+m_QP_N,j)=0.0;

// 	    }

// 	}
//     }

//   // Build m_Px.
//   MAL_MATRIX_RESIZE(m_Px,m_QP_N,3);

//   for( int li=0;li<m_QP_N;li++)
//     {
//       m_Px(li,0) = 1.0;
//       m_Px(li,1) = (double)(1.0+li)*m_QP_T;
//       m_Px(li,2) = (li+1.0)*(li+1.0)*m_QP_T*m_QP_T*0.5-m_ComHeight/9.81;
//     }
//   if (m_FullDebug>2)
//     {
//       ofstream aof;
//       aof.open("VPx.dat");
//       aof << m_VPx;
//       aof.close();

//       aof.open("m_PPx.dat");
//       aof << m_PPx;
//       aof.close();

//       aof.open("VPu.dat");
//       aof << m_VPu;
//       aof.close();

//       aof.open("PPu.dat");
//       aof << m_PPu;
//       aof.close();
//     }

//   return 0;
// }


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
      sprintf(Buffer,"localQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<m_QP_N;i++)
	{
	  for( int j=0;j<m_QP_N;j++)
	    aof << localQ[i*m_QP_N+j] << " ";
	  aof<<endl;
	}
      aof.close();

      sprintf(Buffer,"localLQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<m_QP_N;i++)
	{
	  for( int j=0;j<m_QP_N;j++)
	    aof << localLQ[i*m_QP_N+j] << " ";
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"localiLQ.dat");
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


  // New formulation (Dimitar08)
  m_OptB = MAL_RET_A_by_B(m_iLQ,m_OptB);

  // New formulation (Dimitar08)
  m_OptC = MAL_RET_A_by_B(m_iLQ,m_OptC);

  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];

      sprintf(Buffer,"LQ.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<2*m_QP_N;i++)
	{
	  for( int j=0;j<2*m_QP_N;j++)
	    aof << m_LQ(i,j) << " ";
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"iLQ.dat");
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
  // lterm2 = m_Alpha * lterm2;//Andremize: original pb
  lterm2 = m_Beta*lterm2;

  MAL_MATRIX_RESIZE(OptA,
		    MAL_MATRIX_NB_ROWS(lterm2),
		    MAL_MATRIX_NB_COLS(lterm2));
  MAL_MATRIX_SET_IDENTITY(OptA);
  OptA = m_Alpha*OptA;


  // OptA = OptA + lterm1 + lterm2;//Andremize: original problem
  OptA = OptA + lterm2;

  // Initialization of the matrice regarding the quadratic
  // part of the objective function.
  //Andremize: size of Q is 3*Nx3*N which means that there is place for N/2 feet variables
  m_Q=new double[4*(m_QP_N)*(m_QP_N)];
  memset(m_Q,0,4*(m_QP_N)*(m_QP_N)*sizeof(double));
  // for( int i=0;i<2*m_QP_N;i++)
  //   m_Q[i*2*m_QP_N+i] = 1.0;


  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"Q.dat");
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
      BuildingConstantPartOfTheObjectiveFunctionQLDANDLQ(OptA);
    }
  else
    {
      BuildingConstantPartOfTheObjectiveFunctionQLD(OptA);
    }


  if (m_FullDebug>0)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"OptB.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<MAL_MATRIX_NB_ROWS(m_OptB);i++)
	{
	  for( int j=0;j<MAL_MATRIX_NB_COLS(m_OptB)-1;j++)
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
      sprintf(Buffer,"OptC.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<MAL_MATRIX_NB_ROWS(m_OptC);i++)
	{
	  for( int j=0;j<MAL_MATRIX_NB_COLS(m_OptC)-1;j++)
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
      (m_FastFormulationMode==PLDP))
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

      if (m_FastFormulationMode==PLDP)
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
      sprintf(Buffer,"PuCst.dat");
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<m_QP_N;i++)
	{
	  for( int j=0;j<m_QP_N;j++)
	    aof << m_Pu[j+i*m_QP_N] << " " ;
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"tmpPuCst.dat");
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
	  sprintf(Buffer,"tmpiLQ.dat");
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

void ZMPVelocityReferencedQP::initFeet()
{

  //Define the initial coordinates of the feet
  //This might be done when creating SupportState
  SupportFeet_t aSFLeft;
  SupportFeet_t aSFRight;
  aSFLeft.x = 0.0;
  aSFLeft.y = 0.1;//Andremize
  aSFLeft.theta = 0.0;
  aSFLeft.StartTime = 0.0;
  aSFLeft.SupportFoot = 1;
  aSFRight.x = 0.0;
  aSFRight.y = -0.1;//Andremize
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

  if ((r=BuildingConstantPartOfTheObjectiveFunction())<0)
    return r;

  if ((r=BuildingConstantPartOfConstraintMatrices())<0)
    return r;

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
						 double *X
						 )
{
  // double lSizeMat = QueueOfLConstraintInequalities.back()->EndingTime/m_QP_T;
  MAL_MATRIX(vnlPx,double); MAL_MATRIX(vnlPu,double);
  MAL_MATRIX(vnlValConstraint,double);
  MAL_MATRIX(vnlX,double);// MAL_MATRIX(vnlStorePx,double);
  // MAL_MATRIX(vnlStoreX,double);
  MAL_VECTOR(ConstraintNb,int);

  MAL_MATRIX_RESIZE(vnlX,2*(m_QP_N+m_Support->StepNumber),1);
  // MAL_MATRIX_RESIZE(vnlStorePx,
  // 		    NbOfConstraints,
  // 		    //6*N,
  // 		    1+(unsigned int)lSizeMat);

  // for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(vnlStorePx);i++)
  //   {
  //     for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(vnlStorePx);j++)
  // 	{
  // 	  vnlStorePx(i,j) =0.0;
  // 	}
  //   }
  // MAL_MATRIX_RESIZE(vnlStoreX,
  // 		    2*m_QP_N,1+(unsigned int)lSizeMat);

  // // for(unsigned int i=0;i<2*m_QP_N;i++)
  // //   vnlStoreX(i,0) = 0.0;

  // MAL_VECTOR_RESIZE(ConstraintNb,
  // 		    1+(unsigned int)lSizeMat);



  // ConstraintNb[li] = NbOfConstraints;
  MAL_MATRIX_RESIZE(vnlPu,NbOfConstraints,2*(m_QP_N+m_Support->StepNumber));
  MAL_MATRIX_RESIZE(vnlPx,NbOfConstraints,1);


  for(int i=0; i<NbOfConstraints;i++)
    {
      vnlPx(i,0) = DS[i];
      // vnlStorePx(i,li) = DS[i];
    }

  for(int i=0; i<NbOfConstraints;i++)
    for( int j=0; j<2*(m_QP_N+m_Support->StepNumber);j++)
      vnlPu(i,j) = DU[j*(NbOfConstraints+1)+i];

  for( int i=0; i<2*(m_QP_N+m_Support->StepNumber);i++)
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
	  ODEBUG3("Problem during validation of the constraint: ");
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

  // if (m_FullDebug>2)
  //   {
  //     ofstream aof;
  //     aof.open("StorePx.dat",ofstream::out);

  //     for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(vnlStorePx);i++)
  // 	{
  // 	  for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(vnlStorePx);j++)
  // 	    {
  // 	      aof << vnlStorePx(i,j) << " ";
  // 	    }
  // 	  aof << endl;
  // 	}
  //     aof.close();


  //     char lBuffer[1024];
  //     sprintf(lBuffer,"StoreX.dat");
  //     aof.open(lBuffer,ofstream::out);

  //     for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(vnlStoreX);i++)
  // 	{
  // 	  for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(vnlStoreX);j++)
  // 	    {
  // 	      aof << vnlStoreX(i,j) << " ";
  // 	    }
  // 	  aof << endl;
  // 	}
  //     aof.close();

  //     aof.open("Cnb.dat",ofstream::out);
  //     for(unsigned int i=0;i<MAL_VECTOR_SIZE(ConstraintNb);i++)
  // 	{
  // 	  aof << ConstraintNb[i]<<endl;
  // 	}
  //     aof.close();
  //   }
  return 0;
}

int ZMPVelocityReferencedQP::dumpProblem(double * Q,
					 double * D,
					 double * DPu,
					 int NbOfConstraints,
					 double * Px,
					 double * XL,
					 double * XU,
					 double Time)
{
  ofstream aof;

  char Buffer[1024];
  sprintf(Buffer,"ProblemFF_%f.dat",Time);
  aof.open(Buffer,ofstream::out);

  //Somehow this has to be done
  NbOfConstraints++;

  // Dumping Q.
  aof << "Q:"<< endl;
  for( int i=0;i<2*(m_QP_N+m_Support->StepNumber);i++)
    {
      for( int j=0;j<2*(m_QP_N+m_Support->StepNumber);j++)
	{
	  aof <<Q[j*2*(m_QP_N+m_Support->StepNumber)+i]<< " ";
	}
      aof <<endl;
    }

  // Dumping D.
  aof << "D:"<< endl;
  for( int i=0;i<2*(m_QP_N+m_Support->StepNumber);i++)
    {
      aof <<D[i]<< " ";
    }
  aof <<endl;

  // Dumping Pu.
  //	cout<<"NbOfConstraints in dumpProblem: "<<NbOfConstraints<<endl;
  //	aof << "DU: "<< "NbOfConstr.: " << NbOfConstraints << endl;
  for(int i=0;i<NbOfConstraints;i++)
    {
      for( int j=0;j<2*(m_QP_N+m_Support->StepNumber);j++)
	{
	  aof << DPu[j*(NbOfConstraints)+i] << " ";
	}
      aof <<endl;
    }

  // Dumping Px.
  aof << "DS:"<< endl;
  for(int i=0;i<NbOfConstraints;i++)
    {
      aof << Px[i] << " ";
    }
  aof << endl;

  // Dumping XL.
  aof << "XL:"<< endl;
  for( int i=0;i<2*(m_QP_N+m_Support->StepNumber);i++)
    {
      aof << XL[i] << " ";
    }
  aof << endl;

  // Dumping XU.
  aof << "XU:"<< endl;
  for( int i=0;i<2*(m_QP_N+m_Support->StepNumber);i++)
    {
      aof << XU[i] << " ";
    }
  aof << endl;

  aof.close();
  return 0;
}

int ZMPVelocityReferencedQP::buildConstraintMatrices(double * &DS,double * &DU,
						     int N, double T,
						     double StartingTime,
						     deque<LinearConstraintInequalityFreeFeet_t> &
						     QueueOfLConstraintInequalitiesFreeFeet,
						     deque<LinearConstraintInequalityFreeFeet_t> &
						     QueueOfFeetPosInequalities,
						     deque<SupportFeet_t> &
						     QueueOfSupportFeet,
						     double Com_Height,
						     int &NbOfConstraints,
						     MAL_VECTOR(& xk,double))
{

  // Discretize the problem.
  ODEBUG(" N:" << N << " T: " << T);

  // Creates the matrices.
  // The memory will be bounded to 4 constraints per
  // Support foot.
  // Will be probably all the time smaller.



  //deque<LinearConstraintInequality_t>::iterator LCI_it, store_it;//Olivier

  //LCI_it = QueueOfLConstraintInequalities.begin();//Olivier
  /*//LCI_it starts always at the beginning of the queue
    while (LCI_it!=QueueOfLConstraintInequalitiesFreeFeet.end())
    {
    if ((StartingTime>=(*LCI_it)->StartingTime) &&
    (StartingTime<=(*LCI_it)->EndingTime))
    {
    break;
    }
    LCI_it++;
    }
  */
  //store_it = LCI_it;//Olivier
  // storeFF_it = LCIFF_it;

  /*See above
  // Did not find the appropriate Linear Constraint.
  if (LCI_it==QueueOfLConstraintInequalitiesFreeFeet.end())
  {
  cout << "HERE 3" << endl;
  return -1;
  }
  */

  if (m_FullDebug>2)
    {
      char Buffer[1024];
      sprintf(Buffer,"PXD_%f.dat", StartingTime);
      RESETDEBUG4(Buffer);
      ODEBUG6("xk:" << xk << " Starting time: " <<StartingTime ,Buffer );
      char Buffer2[1024];
      sprintf(Buffer2,"PXxD_%f.dat", StartingTime);
      RESETDEBUG4(Buffer2);

      char Buffer3[1024];
      sprintf(Buffer3,"PXyD_%f.dat", StartingTime);
      RESETDEBUG4(Buffer3);

      RESETDEBUG6("FFP.dat");
    }

  deque<LinearConstraintInequalityFreeFeet_t>::iterator LCIFF_it;//, storeFF_it, VFF_it;

  // Is better kept for the case when the number of constraints is less predictable
  // Compute first the number of constraint.
  LCIFF_it = QueueOfLConstraintInequalitiesFreeFeet.begin();
  int IndexConstraint=0;
  for( int i=0;i<m_QP_N;i++)
    {
      if (LCIFF_it==QueueOfLConstraintInequalitiesFreeFeet.end())
	{
	  break;
	}
      IndexConstraint += MAL_MATRIX_NB_ROWS((LCIFF_it)->D);
      LCIFF_it++;
    }
  if(m_Support->StepNumber>0)
    {
      LCIFF_it = QueueOfFeetPosInequalities.begin();
      for( int i=0;i<m_QP_N;i++)
	{
	  if (LCIFF_it==QueueOfFeetPosInequalities.end())
	    {
	      break;
	    }
	  IndexConstraint += MAL_MATRIX_NB_ROWS((LCIFF_it)->D);
	  LCIFF_it++;
	}
    }

  NbOfConstraints = IndexConstraint;

  MAL_MATRIX(lD,double);
  MAL_MATRIX_RESIZE(lD,NbOfConstraints,2*(N+m_Support->StepNumber));

  MAL_VECTOR_DIM(lb,double,NbOfConstraints);



  // //Fixed foot positions: V*f_k
  // MAL_VECTOR_DIM(FFPx,double,N);
  // MAL_VECTOR_DIM(FFPy,double,N);

  // VFF_it = QueueOfLConstraintInequalitiesFreeFeet.begin();

  // //Current support foot
  // deque<SupportFeet_t>::iterator CurSF_it;
  // CurSF_it = QueueOfSupportFeet.end();
  // CurSF_it--;
  // for(unsigned int i=0;i<N;i++)
  //   {
  //     if((*VFF_it)->StepNumber==0)
  // 	{
  // 	  FFPx(i) = (CurSF_it)->x;
  // 	  FFPy(i) = (CurSF_it)->y;
  // 	}
  //     else
  // 	{
  // 	  FFPx(i) = 0.0;
  // 	  FFPy(i) = 0.0;
  // 	}
  //     // ODEBUG6("FFPx:" << FFPx(i) << " " << "FFPy:" << FFPy(i), "FFP.dat");
  //   }


  // aof.open("FFP.dat",ios::app);
  // for(unsigned int j=0;j<N;j++)
  // 	aof << "FFPx: "<<FFPx[j] << " " << "FFPy: "<<FFPy[j] << " "<<endl ;
  // aof << endl;
  // aof.close();

  // Store the number of constraint to be generated for the first
  // slot of time control of the algorithm.
  //NextNumberOfRemovedConstraints = MAL_MATRIX_NB_ROWS((LCIFF_it)->D);//Andrei


  // double v = 0;//Andremize: needed?

  //Current support foot
  deque<SupportFeet_t>::iterator CurSF_it;
  CurSF_it = QueueOfSupportFeet.end();
  CurSF_it--;
  while(CurSF_it->SupportFoot!=m_Support->CurrentSupportFoot)
    CurSF_it--;

  LCIFF_it = QueueOfLConstraintInequalitiesFreeFeet.begin();
  // cout << "QOLCIFF.size: " << (int) QueueOfLConstraintInequalitiesFreeFeet.size() << endl;

  // ofstream aof;//Andremize

  // for(unsigned int i=0;i<N;i++)
  //   {
  //     aof.open("LCIFF.dat",ios::app);
  //     aof << "i: "<<i<< " " << "SN: "<<(LCIFF_it)->StepNumber ;
  //     aof << endl;
  //     aof.close();
  //     LCIFF_it++;
  //   }

  // LCIFF_it = QueueOfLConstraintInequalitiesFreeFeet.begin();



  double FFPx, FFPy;

  IndexConstraint = 0;
  ODEBUG("Starting Matrix to build the constraints. ");
  ODEBUG((LCIFF_it)->D );
  //ZMP constraints
  for( int i=0;i<N;i++)
    {
      if((LCIFF_it)->StepNumber==0)
	{//c'est pas bon ca
	  FFPx = CurSF_it->x;
	  FFPy = CurSF_it->y;
	  // cout<<FFPx<<" "<<FFPy<<endl;
	}
      else
	{
	  FFPx = 0.0;
	  FFPy = 0.0;
	}

      // aof.open("FFP.dat",ios::app);
      // aof << "FFPx: "<<FFPx << " " << "FFPy: "<<FFPy<<"(LCIFF_it)->StepNumber: "<<(LCIFF_it)->StepNumber;
      // aof << endl;
      // aof.close();

      // For each constraint.

      for(int j=0;j<MAL_MATRIX_NB_ROWS(LCIFF_it->D);j++)
	{
	  // cout<<" D("<<j<<",0): " <<(LCIFF_it)->D(j,0);
	  // cout<<" D("<<j<<",1): " <<(LCIFF_it)->D(j,1);
	  // cout<<" Dc("<<j<<"): " <<(LCIFF_it)->Dc(j,0)<<" FFPx"<<IndexConstraint<<" :"<<FFPx<<" FFPy"<<IndexConstraint<<" :"<<FFPy;
	  // Verification of constraints.
	  m_Pb.DS[IndexConstraint] =
	    // X Axis * A
	    (FFPx-xk[0] * m_Px(i,0)-
	     xk[1] * m_Px(i,1)-
	     xk[2] * m_Px(i,2))
	    * (LCIFF_it)->D(j,0)
	    +
	    // Y Axis * A
	    ( FFPy-xk[3] * m_Px(i,0)-
	      xk[4] * m_Px(i,1)-
	      xk[5] * m_Px(i,2))
	    * (LCIFF_it)->D(j,1)
	    // Constante part of the constraint
	    + (LCIFF_it)->Dc(j,0);

	  // cout<<" DS"<<IndexConstraint<<" :"<<DS[IndexConstraint]<<endl;
	  ODEBUG6(m_Pb.DS[IndexConstraint] << " " << (LCIFF_it)->D(j,0)  << " "
		  << (LCIFF_it)->D[j][1] << " " << (LCIFF_it)->Dc(j,0) ,Buffer);
	  ODEBUG6(1 << " " <<    T *(i+1) << " " <<    (i+1)*(i+1)*T*T/2 - Com_Height/9.81,Buffer2);
	  ODEBUG6(1 << " " <<    T *(i+1) << " " <<    (i+1)*(i+1)*T*T/2 - Com_Height/9.81,Buffer3);

	  //m_SimilarConstraints[IndexConstraint]=(LCIFF_it)->SimilarConstraints[j];

	  if (m_FastFormulationMode==QLD)
	    {
	      // In this case, Pu is triangular.
	      // so we can speed up the computation.

	      for(int k=0;k<=i;k++)
		{
		  // X axis
		  DU[IndexConstraint+k*(NbOfConstraints+1)] =
		    -(LCIFF_it)->D(j,0)*m_Pu[k*N+i];

		  // Y axis
		  DU[IndexConstraint+(k+N)*(NbOfConstraints+1)] =
		    -(LCIFF_it)->D(j,1)*m_Pu[k*N+i];

		}
	    }
	  else if ((m_FastFormulationMode==QLDANDLQ)||
		   (m_FastFormulationMode==PLDP))
	    {
	      // In this case, Pu is *NOT* triangular.
	      for(int k=0;k<N;k++)
		{
		  // X axis
		  DU[IndexConstraint+k*(NbOfConstraints+1)] =
		    -(LCIFF_it)->D(j,0)*m_Pu[k*N+i];
		  // Y axis
		  DU[IndexConstraint+(k+N)*(NbOfConstraints+1)] =
		    -(LCIFF_it)->D(j,1)*m_Pu[k*N+i];
		}
	    }

	  //Foot variables after jerk: [dddX,dddY,FPx,FPy]
	  if((LCIFF_it)->StepNumber>0)
	    {
	      DU[IndexConstraint+(2*N+(LCIFF_it)->StepNumber-1)*(NbOfConstraints+1)] =
		(LCIFF_it)->D(j,0);
	      DU[IndexConstraint+(2*N+m_Support->StepNumber+(LCIFF_it)->StepNumber-1)*(NbOfConstraints+1)] =
		(LCIFF_it)->D(j,1);
	    }

	  ODEBUG("IC: " << IndexConstraint );
	  IndexConstraint++;
	}
      // printf("DUindex: %d  ",N+(LCIFF_it)->StepNumber);
      LCIFF_it++;
    }


  //Feet position constraints
  LCIFF_it = QueueOfFeetPosInequalities.begin();
  for( int i=0;i<m_Support->StepNumber;i++)
    {
      if(LCIFF_it->StepNumber==1)
	{
	  FFPx = CurSF_it->x;
	  FFPy = CurSF_it->y;
	  //cout<<FFPx<<" "<<FFPy<<endl;
	}
      else
	{
	  FFPx = 0.0;
	  FFPy = 0.0;
	}

      // aof.open("FFP.dat",ios::app);
      // aof << "FFPx: "<<FFPx << " " << "FFPy: "<<FFPy<<"(LCIFF_it)->StepNumber: "<<(LCIFF_it)->StepNumber;
      // aof << endl;
      // aof.close();



      // For each constraint.
      for(int j=0;j<MAL_MATRIX_NB_ROWS((LCIFF_it)->D);j++)
	{
	  // cout<<" D("<<j<<",0): " <<(LCIFF_it)->D(j,0);
	  // cout<<" D("<<j<<",1): " <<(LCIFF_it)->D(j,1);
	  // cout<<" Dc("<<j<<"): " <<(LCIFF_it)->Dc(j,0)<<" FFPx"<<IndexConstraint<<" :"<<FFPx<<" FFPy"<<IndexConstraint<<" :"<<FFPy;
	  // Verification of constraints.
	  m_Pb.DS[IndexConstraint] =
	    // X Axis * A
	    FFPx * (LCIFF_it)->D(j,0)
	    +
	    // Y Axis * A
	    FFPy * (LCIFF_it)->D(j,1)
	    // Constante part of the constraint
	    + (LCIFF_it)->Dc(j,0);
	  // cout<<" m_Pb.DS"<<IndexConstraint<<" :"<<DS[IndexConstraint]<<endl;
	  //   ODEBUG6(DS[IndexConstraint] << " " << (LCIFF_it)->D(j,0)  << " "
	  // 	    << (LCIFF_it)->D[j][1] << " " << (LCIFF_it)->Dc(j,0) ,Buffer);
	  // ODEBUG6(1 << " " <<    T *(i+1) << " " <<    (i+1)*(i+1)*T*T/2 - Com_Height/9.81,Buffer2);
	  // ODEBUG6(1 << " " <<    T *(i+1) << " " <<    (i+1)*(i+1)*T*T/2 - Com_Height/9.81,Buffer3);

	  //m_SimilarConstraints[IndexConstraint]=(LCIFF_it)->SimilarConstraints[j];

	  // if (m_FastFormulationMode==QLD)
	  //   {
	  //     // In this case, Pu is triangular.
	  //     // so we can speed up the computation.
	  //     for(unsigned k=0;k<=i;k++)
	  // 	{
	  // 	  // X axis
	  // 	  DU[IndexConstraint+k*(NbOfConstraints+1)] =
	  // 	    -(LCIFF_it)->D(j,0)*m_Pu[k*N+i];

	  // 	  // Y axis
	  // 	  DU[IndexConstraint+(k+N)*(NbOfConstraints+1)] =
	  // 	    -(LCIFF_it)->D(j,1)*m_Pu[k*N+i];
	  // 	}
	  //   }
	  // else if ((m_FastFormulationMode==QLDANDLQ)||
	  // 	   (m_FastFormulationMode==PLDP))
	  //   {
	  //     // In this case, Pu is *NOT* triangular.
	  //     for(unsigned k=0;k<N;k++)
	  // 	{
	  // 	  // X axis
	  // 	  DU[IndexConstraint+k*(NbOfConstraints+1)] =
	  // 	    -(LCIFF_it)->D(j,0)*m_Pu[k*N+i];
	  // 	  // Y axis
	  // 	  DU[IndexConstraint+(k+N)*(NbOfConstraints+1)] =
	  // 	    -(LCIFF_it)->D(j,1)*m_Pu[k*N+i];
	  // 	}
	  //   }

	  //Foot variables after jerk: [dddX,dddY,FPx,FPy]
	  if((LCIFF_it)->StepNumber==1)
	    {
	      m_Pb.DU[IndexConstraint+(2*N+(LCIFF_it)->StepNumber-1)*(NbOfConstraints+1)] =
		-LCIFF_it->D(j,0);
	      m_Pb.DU[IndexConstraint+(2*N+m_Support->StepNumber+LCIFF_it->StepNumber-1)*(NbOfConstraints+1)] =
		-LCIFF_it->D(j,1);
	    }
	  if((LCIFF_it)->StepNumber>1)
	    {
	      m_Pb.DU[IndexConstraint+(2*N+(LCIFF_it)->StepNumber-1)*(NbOfConstraints+1)] =
		-(LCIFF_it)->D(j,0);
	      m_Pb.DU[IndexConstraint+(2*N+(LCIFF_it)->StepNumber-2)*(NbOfConstraints+1)] =
		(LCIFF_it)->D(j,0);
	      m_Pb.DU[IndexConstraint+(2*N+m_Support->StepNumber+(LCIFF_it)->StepNumber-1)*(NbOfConstraints+1)] =
		-(LCIFF_it)->D(j,1);
	      m_Pb.DU[IndexConstraint+(2*N+m_Support->StepNumber+(LCIFF_it)->StepNumber-2)*(NbOfConstraints+1)] =
		(LCIFF_it)->D(j,1);
	    }
	  ODEBUG("IC: " << IndexConstraint );
	  IndexConstraint++;
	}

      // printf("DUindex: %d  ",N+(LCIFF_it)->StepNumber);
      LCIFF_it++;
    }

  ODEBUG6("Index Constraint :"<< IndexConstraint,Buffer);
  static double localtime = -m_QP_T;
  localtime+=m_QP_T;

  ODEBUG("IndexConstraint:"<<IndexConstraint << " localTime :" << localtime);


  if (0)
    {
      ODEBUG3("localtime: " <<localtime);
      ofstream aof;

      char Buffer[1024];
      sprintf(Buffer,"DU.dat");
      aof.open(Buffer,ofstream::out);
      aof <<" 2*N+2*m_Support->StepNumber: "<<2*N+2*m_Support->StepNumber<<" NbOfConstraints: "<<NbOfConstraints
	  << endl;
      for( int i=0;i<NbOfConstraints;i++)
	{
	  for( int j=0;j<2*N+2*m_Support->StepNumber;j++)
	    aof << DU[j*(NbOfConstraints)+i] << " " ;
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"m_Pb.DS.dat");
      aof.open(Buffer,ofstream::out);
      for( int j=0;j<NbOfConstraints;j++)
	aof << m_Pb.DS[j] << endl;
      // aof << endl;
      aof.close();


      // sprintf(Buffer,"lD.dat");
      // aof.open(Buffer,ofstream::out);
      // ODEBUG3(MAL_MATRIX_NB_ROWS(lD) << " " << MAL_MATRIX_NB_COLS(lD) << " " );
      // for(unsigned int lj=0;lj<MAL_MATRIX_NB_ROWS(lD);lj++)
      // 	{
      // 	  for(unsigned int k=0;k<MAL_MATRIX_NB_COLS(lD);k++)
      // 	    aof << lD(lj,k) << " " ;
      // 	  aof << endl;
      // 	}
      // aof.close();

      // sprintf(Buffer,"lb.dat");
      // aof.open(Buffer,ofstream::out);
      // for(unsigned int j=0;j<IndexConstraint;j++)
      // 	aof << lb(j) << " " ;
      // aof << endl;
      // aof.close();

      //      exit(0);
    }

  //  if (m_FullDebug>0)
  if (0)
    {

      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"PuCst_%f.dat",StartingTime);
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<m_QP_N;i++)
	{
	  for( int j=0;j<m_QP_N;j++)
	    aof << m_Pu[j+i*m_QP_N] << " " ;
	  aof << endl;
	}
      aof.close();

      sprintf(Buffer,"D_%f.dat",StartingTime);
      aof.open(Buffer,ofstream::out);
      for( int i=0;i<2*m_QP_N;i++)
	{
	  for( int j=0;j<NbOfConstraints;j++)
	    aof << lD(i,j) << " " ;
	  aof << endl;
	}
      aof.close();

      if (0)
	{
	  sprintf(Buffer,"DPX_%f.dat", StartingTime);
	  aof.open(Buffer,ofstream::out);
	  for( int i=0;i<IndexConstraint;i++)
	    {
	      aof << m_Pb.DS[i] << endl ;
	    }
	  aof.close();
	}
    }

  //	 printf("Leavin buildConstraints \n");

  return 0;
}







int ZMPVelocityReferencedQP::buildZMPTrajectoryFromFootTrajectory(deque<FootAbsolutePosition>
								  &LeftFootAbsolutePositions,
								  deque<FootAbsolutePosition>
								  &RightFootAbsolutePositions,
								  deque<ZMPPosition> &ZMPRefPositions,
								  deque<COMPosition> &COMPositions,
								  double ConstraintOnX,
								  double ConstraintOnY,
								  double T,
								  int N)
{


  int NbOfConstraints; // Nb of constraints are not known in advance



  MAL_VECTOR(VRef,double);
  MAL_VECTOR(ZMPRef,double);
  MAL_VECTOR_DIM(OptD,double,2*N);



  int CriteriaToMaximize=1;


  RESETDEBUG4("DebugInterpol.dat");
  MAL_VECTOR_RESIZE(ZMPRef,2*N);
  MAL_VECTOR_RESIZE(VRef,2*N);

  int m;
  int me;
  int mmax;
  int n;
  int nmax; // Size of the matrix to compute the cost function.
  int mnn;

  double Eps=1e-8;
  //double *U = (double *)malloc( sizeof(double)*mnn); // Returns the Lagrange multipliers.;

  int iout=0;
  int ifail;
  int iprint=1;
  int lwar;
  // double *war= (double *)malloc(sizeof(double)*lwar);
  int liwar = n; //
  // int *iwar = new int[liwar]; // The Cholesky decomposition is done internally.

  //deque<LinearConstraintInequality_t> QueueOfLConstraintInequalities;
  deque<LinearConstraintInequalityFreeFeet_t> QueueOfLConstraintInequalitiesFreeFeet;
  deque<LinearConstraintInequalityFreeFeet_t> QueueOfFeetPosInequalities;

  double FPx, FPy, FPtheta;
  FPx = 0.0;
  FPy = 0.0;
  FPtheta = 0.0;

  // if (m_FullDebug>0)
  //   {
  //     RESETDEBUG4("DebugPBW.dat");
  //     RESETDEBUG4("DebugPBW_Pb.dat");

  //     ODEBUG6("A:" << m_A << endl << "B:" << m_B, "DebugPBW_Pb.dat");

  //     ofstream aof("FFP.dat");//Andremize
  //     aof.open("LCIFF.dat");
  //   }

  //	deque<LinearConstraintInequality_t>::iterator LCI_it;
  //	LCI_it = QueueOfLConstraintInequalities.begin();
  //	while(LCI_it!=QueueOfLConstraintInequalities.end())
  //	{
  //		LCI_it++;
  //	}

  // pre computes the matrices needed for the optimization.

  double TotalAmountOfCPUTime=0.0,CurrentCPUTime=0.0;
  struct timeval start,end;
  int li=0;
  double dinterval = T /  m_SamplingPeriod;
  int interval=(int)dinterval;
  bool StartingSequence = true;

  MAL_VECTOR_DIM(xk,double,6);

  //ODEBUG3("0.0 " << QueueOfLConstraintInequalities.back().EndingTime-	N*T << " "
  //		<< " T: " << T << " N: " << N << " interval " << interval);
  int NumberOfRemovedConstraints =0;

  //Andremize
  //(Re)initialize the LIPM
  m_2DLIPM->InitializeSystem();




  //----------"Real-time" loop---------
  //
  //
  //-----------------------------------
  // printf("Inside the 'Real-time' loop: \n");
  for(double StartingTime=0.0;
      StartingTime<= 11.0;
      StartingTime+=T,li++)
    {

      // printf("FPx: %f FPy %f \n",FPx,FPy);
      double *DS=0,*DU=0;

      // printf("StartingTime: %f \n", StartingTime);
      gettimeofday(&start,0);

      // Read the current state of the 2D Linearized Inverted Pendulum.
      m_2DLIPM->GetState(xk);

      ODEBUG("State: " << xk[0] << " " << xk[3] << " " <<
	     xk[1] << " " << xk[4] << " " <<
	     xk[2] << " " << xk[5] << " ");
      if (m_FastFormulationMode==QLDANDLQ)
	{
	  ODEBUG6(xk[0] << " " << xk[3] << " " <<
		  xk[1] << " " << xk[4] << " " <<
		  xk[2] << " " << xk[5] << " ", "Check2DLIPM_QLDANDLQ.dat");
	}
      else if (m_FastFormulationMode==PLDP)
	{
	  ODEBUG6(xk[0] << " " << xk[3] << " " <<
		  xk[1] << " " << xk[4] << " " <<
		  xk[2] << " " << xk[5] << " ", "Check2DLIPM_PLDP.dat");
	}


      m_Support->setSupportState(StartingTime, 0, RefVel);

      deque<SupportFeet_t>::iterator SF_it;
      if(m_Support->StateChanged == 1)
	{
	  SupportFeet_t newSF;
	  if(m_Support->SSSS == 0)//SS->DS or DS->SS
	    {
	      SF_it = QueueOfSupportFeet.end();
	      SF_it--;
	      //The Support foot does not change
	      if((SF_it)->SupportFoot != m_Support->CurrentSupportFoot)
		SF_it--;
	      FPx = (SF_it)->x;
	      FPy = (SF_it)->y;
	      FPtheta = (SF_it)->theta;
	    }

	  newSF.x = FPx;
	  newSF.y = FPy;
	  // printf("newSF -> FPx: %f FPy %f \n",FPx,FPy);
	  newSF.theta = FPtheta;
	  newSF.StartTime = StartingTime;
	  newSF.SupportFoot = m_Support->CurrentSupportFoot;

	  QueueOfSupportFeet.push_back(newSF);

	  // delete newSF;
	}


      // printf("Before buildLinearConstraintInequalities \n");
      m_fCALS->buildLinearConstraintInequalities(LeftFootAbsolutePositions,
						 RightFootAbsolutePositions, QueueOfLConstraintInequalitiesFreeFeet,
						 QueueOfFeetPosInequalities, RefVel,
						 StartingTime,
						 m_QP_N,
						 m_Support,m_PreviewedSupportAngles);



      // printf("buildConstraintMatrices");
      buildConstraintMatrices(DS,DU,
			      N,T,
			      StartingTime,
			      QueueOfLConstraintInequalitiesFreeFeet,
			      QueueOfFeetPosInequalities,
			      QueueOfSupportFeet,
			      m_ComHeight,
			      NbOfConstraints,
			      xk);


      //-------------Prepare the data for the solver-------
      //
      //
      //---------------------------------------------------

      m = NbOfConstraints;
      me= 0;
      mmax = m+1;
      n = 2*(N+m_Support->StepNumber);
      nmax = n; // Size of the matrix to compute the cost function.
      mnn = m+n+n;

      lwar=3*nmax*nmax/2+ 10*nmax  + 2*mmax + 20000;
      liwar = n;

      //Andremize
      //Variable matrices due to variable foot step number
      double *m_Qff=new double[4*(m_QP_N+m_Support->StepNumber)*(m_QP_N+m_Support->StepNumber)];  //Quadratic part of the objective function
      double *D=new double[2*(N+m_Support->StepNumber)];   // Linear part of the objective function
      double *XL=new double[2*(N+m_Support->StepNumber)];  // Lower bound of the jerk.
      double *XU=new double[2*(N+m_Support->StepNumber)];  // Upper bound of the jerk.
      double *X=new double[2*(N+m_Support->StepNumber)];   // Solution of the system.
      double *NewX=new double[2*(N+m_Support->StepNumber)];   // Solution of the system.
      double *U = (double *)malloc( sizeof(double)*mnn); // Returns the Lagrange multipliers.;
      // double *war= (double *)malloc(sizeof(double)*lwar);
      double *war= new double[lwar];
      int *iwar = new int[liwar]; // The Cholesky decomposition is done internally.

      if (m_FastFormulationMode==QLDANDLQ)
	iwar[0]=0;
      else
	iwar[0]=1;

      //Objective
      //Andremize: There are constant parts which should be put in separate functions
      MAL_MATRIX(OptA,double);

      MAL_MATRIX(lterm2,double);
      lterm2 = MAL_RET_TRANSPOSE(m_VPu);
      lterm2 = MAL_RET_A_by_B(lterm2,m_VPu);
      lterm2 = m_Beta*lterm2;

      MAL_MATRIX_RESIZE(OptA,
			MAL_MATRIX_NB_ROWS(lterm2),
			MAL_MATRIX_NB_COLS(lterm2));
      MAL_MATRIX_SET_IDENTITY(OptA);
      OptA = m_Alpha*OptA;

      OptA = OptA + lterm2;


      memset(m_Qff,0,4*(m_QP_N+m_Support->StepNumber)*(m_QP_N+m_Support->StepNumber)*sizeof(double));
      for( int i=0;i<2*(m_QP_N);i++)
	for( int j=0;j<2*(m_QP_N);j++)
	  m_Qff[i*2*(m_QP_N+m_Support->StepNumber)+j] = OptA(j,i);

      m_OptB = MAL_RET_TRANSPOSE(m_VPu);
      m_OptB = MAL_RET_A_by_B(m_OptB,m_VPx);
      m_OptB = m_Beta * m_OptB;

      //Andremize - has to go back where it comes from
      //MAL_MATRIX(m_OptD,double);
      m_OptD = MAL_RET_TRANSPOSE(m_VPu);
      m_OptD = m_Beta * m_OptD;



      memset(D,0,2*(m_QP_N+m_Support->StepNumber)*sizeof(double));
      if (CriteriaToMaximize==1)
	{
	  MAL_VECTOR(lterm1v,double);
	  MAL_C_eq_A_by_B(lterm1v,m_OptD,VRef);
	  MAL_VECTOR_RESIZE(OptD,2*N);
	  MAL_C_eq_A_by_B(OptD,m_OptB,xk);
	  OptD -= lterm1v;
	  for( int i=0;i<2*N;i++)
	    D[i] = OptD(i);

	  if (m_FullDebug>0)
	    {
	      ofstream aof;
	      char Buffer[1024];
	      sprintf(Buffer,"Dff_%f.dat",StartingTime);
	      aof.open(Buffer,ofstream::out);
	      for( int i=0;i<2*(N+m_Support->StepNumber);i++)
		{
		  aof << OptD[i] << endl;
		}
	      aof.close();
	    }

	}
      else
	{
	  // Default : set D to zero.
	  for( int i=0;i<2*(N+m_Support->StepNumber);i++)
	    D[i] = 0.0;
	}

      for( int i=0;i<2*(N+m_Support->StepNumber);i++)
	{
	  XL[i] = -1e8;
	  XU[i] = 1e8;
	}
      memset(X,0,2*(N+m_Support->StepNumber)*sizeof(double));


      ODEBUG("m: " << m);
      if(m_FullDebug>2)
	dumpProblem(m_Qff, D, DU, m, DS, XL, XU, StartingTime);


      //---------Solver------------
      //
      //
      //---------------------------
      // printf("Entering the solver \n");
      if ((m_FastFormulationMode==QLDANDLQ)||
	  (m_FastFormulationMode==QLD))
	{
	  struct timeval lbegin,lend;
	  gettimeofday(&lbegin,0);
	  ql0001_(&m, &me, &mmax, &n, &nmax, &mnn,
		  m_Qff, D, DU, DS, XL, XU,
		  X, U, &iout, &ifail, &iprint,
		  war, &lwar, iwar, &liwar, &Eps);
	  gettimeofday(&lend,0);




	  CODEDEBUG6(double ldt = lend.tv_sec - lbegin.tv_sec +
		     0.000001 * (lend.tv_usec - lbegin.tv_usec););
	  // printf("Solver has finished,  \n");
	  int NbOfActivatedConstraints = 0;
	  for(int lk=0;lk<m;lk++)
	    {
	      if (U[lk]>0.0)
		{
		  NbOfActivatedConstraints++;
		}
	    }
	  ODEBUG6(NbOfActivatedConstraints,"InfosQLD.dat");
	  ODEBUG6(ldt,"dtQLD.dat");
	}


      // else if (m_FastFormulationMode==PLDP)
      // 	{
      // 	  ODEBUG("State: " << xk[0] << " " << xk[3] << " " <<
      // 		  xk[1] << " " << xk[4] << " " <<
      // 		  xk[2] << " " << xk[5] << " ");
      // 	  struct timeval lbegin,lend;
      // 	  gettimeofday(&lbegin,0);

      // 	  ifail=m_PLDPSolverHerdt->SolveProblem(D,
      // 					   (unsigned int)m,
      // 					   DU,
      // 					   DS,
      // 					   MAL_RET_VECTOR_DATABLOCK(ZMPRef),
      // 					   MAL_RET_VECTOR_DATABLOCK(xk),X,
      // 					   m_SimilarConstraints,
      // 					   NumberOfRemovedConstraints,
      // 					   StartingSequence);
      // 	  StartingSequence = false;
      // 	  NumberOfRemovedConstraints = NextNumberOfRemovedConstraints;
      // 	  gettimeofday(&lend,0);
      // 	  CODEDEBUG6(double ldt = lend.tv_sec - lbegin.tv_sec +
      // 		     0.000001 * (lend.tv_usec - lbegin.tv_usec););

      // 	  ODEBUG6(ldt,"dtPLDP.dat");
      // 	}

      if (ifail!=0)
	{
	  cout << "IFAIL: " << ifail << " at time: " << StartingTime << endl;
	  //return -1;
	}

      //------------------------
      //
      //
      //-------------------------

      double *ptX=0;
      if ((m_FastFormulationMode==QLDANDLQ)||
	  (m_FastFormulationMode==PLDP))
	{
	  /* Multiply the solution by the transpose of iLQ
      	     because it is a triangular matrix we do a specific
      	     multiplication.
	  */
	  memset(NewX,0,2*N*sizeof(double));

	  double *pm_iLQ = MAL_RET_MATRIX_DATABLOCK(m_iLQ);
	  double *pNewX = NewX;

	  for( int i=0;i<2*N;i++)
	    {
	      double *pX= X+i;
	      double *piLQ = pm_iLQ+i*2*N+i;
	      *pNewX = 0.0;
	      for( int j=i;j<2*N;j++)
		{
		  *pNewX+= (*piLQ) * (*pX++);
		  piLQ+=2*N;
		}
	      pNewX++;
	    }
	  ptX=NewX;
	}
      else
	ptX=X;

      /* Simulation of the Single Point Mass model
      	 with the new command.
      */
      ODEBUG("X[0] " << X[0] << " X[N] :" << X[N]);

      // Calling this method will automatically
      // update the ZMPRefPositions.
      m_2DLIPM->Interpolation(COMPositions,
			      ZMPRefPositions,
			      li*interval,
			      ptX[0],ptX[N]);

      m_2DLIPM->OneIteration(ptX[0],ptX[N]);

      //Previewed position of the next foot
      FPx = ptX[2*N];
      FPy = ptX[2*N+m_Support->StepNumber];

      // printf("FPx: %f FPy %f \n",FPx,FPy);

      ODEBUG6("uk:" << uk,"DebugPBW.dat");
      ODEBUG6("xk:" << xk,"DebugPBW.dat");


      if (m_FullDebug>2)
	{
	  ofstream aof;
	  char Buffer[1024];
	  sprintf(Buffer,"Xff_%f.dat",StartingTime);
	  aof.open(Buffer,ofstream::out);
	  aof << "State: " <<xk[0]<<" "<<xk[1]<< " " << xk[2] << " " << xk[3] << " "<<xk[4]<<" "<<xk[5]<<" "<<endl;
	  for( int i=0;i<2*(N+m_Support->StepNumber);i++)
	    {
	      aof << X[i] << endl;
	    }
	  aof.close();
	  // sprintf(Buffer,"Uff_%f.dat",StartingTime);
	  // aof.open(Buffer,ofstream::out);
	  // for(unsigned int i=0;i<2*(N+m_Support->StepNumber);i++)
	  //   {
	  //     aof << U[i] << endl;
	  //   }
	  // aof.close();
	}

      if(m_FullDebug>2)
	{
	  if(validateConstraints(DS, DU, m, li, X)<0)
	    {
	      cout << "Something is wrong with the constraints." << endl;
	      exit(-1);
	    }
	}

      // Compute CPU consumption time.
      gettimeofday(&end,0);
      CurrentCPUTime = end.tv_sec - start.tv_sec +
	0.000001 * (end.tv_usec - start.tv_usec);
      TotalAmountOfCPUTime += CurrentCPUTime;
      //ODEBUG("Current Time : " << StartingTime << " " <<
      //		" Virtual time to simulate: " << QueueOfLConstraintInequalities.back()->EndingTime - StartingTime <<
      //		"Computation Time " << CurrentCPUTime << " " << TotalAmountOfCPUTime);

      QueueOfLConstraintInequalitiesFreeFeet.clear();
      QueueOfFeetPosInequalities.clear();

      delete [] m_Qff;
      delete [] D;
      delete [] DS;
      delete [] DU;
      delete [] XL;
      delete [] XU;
      delete [] X;
      delete [] NewX;
      delete [] iwar; // The Cholesky decomposition is done internally.

      delete [] war;
      free(U);
    }
  //-----------------------------------
  //
  //
  //----------"Real-time" loop--------

  /*  cout << "Size of PX: " << MAL_MATRIX_NB_ROWS(vnlStorePx) << " "
      << MAL_MATRIX_NB_COLS(vnlStorePx) << " " << endl; */


  // Clean the queue of Linear Constraint Inequalities.
  //QueueOfLConstraintInequalities.clear();
  QueueOfSupportFeet.clear();

  //	printf("Leaving buildZMPTrajectoryFromFeetTrajectory \n");
  return 0;
}



//--------------------------------------
//
//
//-----------new functions--------------



void ZMPVelocityReferencedQP::GetZMPDiscretization(deque<ZMPPosition> & ZMPPositions,
						   deque<COMPosition> & COMPositions,
						   deque<RelativeFootPosition> &RelativeFootPositions,
						   deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
						   deque<FootAbsolutePosition> &RightFootAbsolutePositions,
						   double Xmax,
						   COMPosition & lStartingCOMPosition,
						   MAL_S3_VECTOR(&,double) lStartingZMPPosition,
						   FootAbsolutePosition & InitLeftFootAbsolutePosition,
						   FootAbsolutePosition & InitRightFootAbsolutePosition)
{

  if (m_ZMPD==0)
    return;

  //printf("Entered GetZMPDiscretization \n");

  m_ZMPD->GetZMPDiscretization(ZMPPositions,
			       COMPositions,
			       RelativeFootPositions,
			       LeftFootAbsolutePositions,
			       RightFootAbsolutePositions,
			       Xmax,
			       lStartingCOMPosition,
			       lStartingZMPPosition,
			       InitLeftFootAbsolutePosition,
			       InitRightFootAbsolutePosition);


  ODEBUG3("Dimitrov algo set on");


  buildZMPTrajectoryFromFootTrajectory(LeftFootAbsolutePositions,
				       RightFootAbsolutePositions,
				       ZMPPositions,
				       COMPositions,
				       m_ConstraintOnX,
				       m_ConstraintOnY,
				       m_QP_T,
				       m_QP_N);

  if (m_FullDebug>0)
    {
      ofstream aof;
      aof.open("DebugDimitrovZMP.dat",ofstream::out);
      for( int i=0;i<ZMPPositions.size();i++)
	{
	  aof << ZMPPositions[i].px << " " << ZMPPositions[i].py << endl;
	}
      aof.close();

    }
  printf("finished GetZMPDiscretization \n");
}

void ZMPVelocityReferencedQP::CallMethod(std::string & Method, std::istringstream &strm)
{
  if (Method==":previewcontroltime")
    {
      strm >> m_PreviewControlTime;
    }


  ZMPRefTrajectoryGeneration::CallMethod(Method,strm);
}

int ZMPVelocityReferencedQP::InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
					deque<COMPosition> & FinalCoMPositions,
					deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
					deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
					FootAbsolutePosition & InitLeftFootAbsolutePosition,
					FootAbsolutePosition & InitRightFootAbsolutePosition,
					deque<RelativeFootPosition> &RelativeFootPositions,
					COMPosition & lStartingCOMPosition,
					MAL_S3_VECTOR(,double) & lStartingZMPPosition)
{
  // COMPosition aCOMPos;
  // ZMPPosition aZMPPos;

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
  // the current m_Support position and the precedent.
  ODEBUG4("ZMP::InitOnLine - Step 2.5 ","ZMDInitOnLine.txt");



  int AddArraySize;
  {
    assert(m_SamplingPeriod > 0);
    double ldAddArraySize = m_TimeBuffer/m_SamplingPeriod;
    AddArraySize = (int) ldAddArraySize;
  }

  cout<<"AddArraySize:"<<AddArraySize<<endl;
  ODEBUG(AddArraySize);
  FinalZMPPositions.resize(AddArraySize);
  FinalCoMPositions.resize(AddArraySize);
  FinalLeftFootAbsolutePositions.resize(AddArraySize);
  FinalRightFootAbsolutePositions.resize(AddArraySize);
  int CurrentZMPindex=0;

  //Andremize
  if(m_FullDebug>0)
    {
      //Feet coordinates for plot in scilab
      ofstream aoffeet;
      aoffeet.open("Feet.dat",ios::out);
      aoffeet<<"#Time    "<<"LeftX    "<<"LeftY    "<<"LeftZ    "<<"RightX    "<<"RightY    "<<"RightZ    "<<endl;
      aoffeet.close();
    }

  for( int i=0;i<FinalZMPPositions.size();i++)
    {

      // Smooth ramp
      FinalZMPPositions[CurrentZMPindex].px =0.0;
      FinalZMPPositions[CurrentZMPindex].py = 0.0;
      FinalZMPPositions[CurrentZMPindex].pz = 0.0;
      FinalZMPPositions[CurrentZMPindex].theta = 0.0;
      FinalZMPPositions[CurrentZMPindex].time = m_CurrentTime;
      FinalZMPPositions[CurrentZMPindex].stepType = 0;

      // Set CoM positions.
      FinalCoMPositions[CurrentZMPindex].z[0] = m_ComHeight;
      FinalCoMPositions[CurrentZMPindex].z[1] = 0.0;
      FinalCoMPositions[CurrentZMPindex].z[2] = 0.0;
      FinalCoMPositions[CurrentZMPindex].pitch = 0.0;
      FinalCoMPositions[CurrentZMPindex].roll = 0.0;
      FinalCoMPositions[CurrentZMPindex].yaw = 0.0;

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
	  aoffeet.open("Feet.dat",ios::app);
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

  return 0;
}

void ZMPVelocityReferencedQP::OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
					    deque<ZMPPosition> & FinalZMPPositions,
					    deque<COMPosition> & FinalCOMPositions,
					    deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
					    deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
					    bool EndSequence)
{
  cout << "To be implemented" << endl;
}


void ZMPVelocityReferencedQP::initializeProblem()
{

  m_Pb.DS = new double[(8*m_QP_N+1)*2*(m_QP_N+m_Support->StepNumber)];

  m_Pb.DU = new double[(8*m_QP_N+1)*2*(m_QP_N+m_Support->StepNumber)];

  memset(m_Pb.DU,0,(8*m_QP_N+1)*2*(m_QP_N+m_Support->StepNumber)*sizeof(double));

  m_Pb.Q=new double[4*(m_QP_N+m_Support->StepNumber)*(m_QP_N+m_Support->StepNumber)];  //Quadratic part of the objective function
  m_Pb.D=new double[2*(m_QP_N+m_Support->StepNumber)];   // Linear part of the objective function
  m_Pb.XL=new double[2*(m_QP_N+m_Support->StepNumber)];  // Lower bound of the jerk.
  m_Pb.XU=new double[2*(m_QP_N+m_Support->StepNumber)];  // Upper bound of the jerk.
  m_Pb.X=new double[2*(m_QP_N+m_Support->StepNumber)];   // Solution of the system.
  m_Pb.NewX=new double[2*(m_QP_N+m_Support->StepNumber)];   // Solution of the system.

}

void ZMPVelocityReferencedQP::setProblem(deque<LinearConstraintInequalityFreeFeet_t> & QueueOfLConstraintInequalitiesFreeFeet,
		deque<SupportFeet_t> &QueueOfSupportFeet,
		int NbOfConstraints, int NbOfEqConstraints, int & CriteriaToMaximize, MAL_VECTOR(& xk,double))
{
	m_Pb.m=NbOfConstraints;
	m_Pb.me=NbOfEqConstraints;
	m_Pb.mmax=m_Pb.m+1;
	m_Pb.n=2*(m_QP_N+m_Support->StepNumber);
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

	m_Pb.U = (double *)malloc( sizeof(double)*m_Pb.mnn); // Returns the Lagrange multipliers.;


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

	if(m_Support->StepNumber>0)
		MAL_MATRIX_RESIZE(m_U,m_QP_N,m_Support->StepNumber);
	for(int i=0;i<m_QP_N;i++)
	{
		if(LCIFF_it->StepNumber>0)
			m_U(i,LCIFF_it->StepNumber-1) = 1.0;
		else
			m_Uc(i) = 1.0;
		LCIFF_it++;
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

	//Velocity----
	ltermVel = MAL_RET_TRANSPOSE(m_VPu);
	ltermVel = MAL_RET_A_by_B(ltermVel,m_VPu);
	ltermVel = m_Beta*ltermVel;
	//----Velocity

	MAL_MATRIX_RESIZE(OptA,
			MAL_MATRIX_NB_ROWS(ltermVel),
			MAL_MATRIX_NB_COLS(ltermVel));
	MAL_MATRIX_SET_IDENTITY(OptA);
	OptA = m_Alpha*OptA;

	OptA = OptA + ltermVel;

	//m_Pb.Q------------------------
	memset(m_Pb.Q,0,4*(m_QP_N+m_Support->StepNumber)*(m_QP_N+m_Support->StepNumber)*sizeof(double));
	for( int i=0;i<2*m_QP_N;i++)
		for( int j=0;j<2*m_QP_N;j++)
			m_Pb.Q[i*2*(m_QP_N+m_Support->StepNumber)+j] = OptA(j,i);
	//ZMP----
	for( int i=0;i<m_QP_N;i++)
	{
		for( int j=0;j<m_QP_N;j++)
		{
			m_Pb.Q[i*2*(m_QP_N+m_Support->StepNumber)+j] -= ltermPZuPZu(i,j);
			m_Pb.Q[(m_QP_N+i)*2*(m_QP_N+m_Support->StepNumber)+m_QP_N+j] -= ltermPZuPZu(i,j);
		}
	}
	if(m_Support->StepNumber>0)
	{
		for( int i=0;i<m_QP_N;i++)
		{
			for( int j=0;j<m_Support->StepNumber;j++)
			{
				m_Pb.Q[i*2*(m_QP_N+m_Support->StepNumber)+2*m_QP_N+j] -= ltermPZuU(i,j);
				m_Pb.Q[(m_QP_N+i)*2*(m_QP_N+m_Support->StepNumber)+2*m_QP_N+m_Support->StepNumber+j] -= ltermPZuU(i,j);
				m_Pb.Q[(2*m_QP_N+j)*2*(m_QP_N+m_Support->StepNumber)+i] -= ltermPZuU(i,j);
				m_Pb.Q[(2*m_QP_N+m_Support->StepNumber+j)*2*(m_QP_N+m_Support->StepNumber)+m_QP_N+i] -= ltermPZuU(i,j);
			}
		}
		for( int i=0;i<m_Support->StepNumber;i++)
		{
			for( int j=0;j<m_Support->StepNumber;j++)
			{
				m_Pb.Q[(2*m_QP_N+i)*2*(m_QP_N+m_Support->StepNumber)+2*m_QP_N+j] += ltermUU(i,j);
				m_Pb.Q[(2*m_QP_N+m_Support->StepNumber+i)*2*(m_QP_N+m_Support->StepNumber)+2*m_QP_N+m_Support->StepNumber+j] += ltermUU(i,j);
			}
		}
	}
	//cout<<"m_Support->StepNumber"<<m_Support->StepNumber<<endl;
	//----ZMP

	//Andremize - only constant velocity
	//constant velocity for the whole preview window
	for( int i=0;i<m_QP_N;i++)
		VRef(i) = RefVel.x;
	for( int i=m_QP_N;i<2*m_QP_N;i++)
		VRef(i) = RefVel.y;

	m_OptB = MAL_RET_TRANSPOSE(m_VPu);
	m_OptB = MAL_RET_A_by_B(m_OptB,m_VPx);
	m_OptB = m_Beta * m_OptB;

	//TODO 2: The matrices of the value function have to go back where they come from
	//MAL_MATRIX(m_OptD,double);
	m_OptD = MAL_RET_TRANSPOSE(m_VPu);
	m_OptD = m_Beta * m_OptD;

	//m_Pb.D---------------
	memset(m_Pb.D,0,2*(m_QP_N+m_Support->StepNumber)*sizeof(double));

	//velocity
	MAL_VECTOR(lterm1v,double);
	MAL_C_eq_A_by_B(lterm1v,m_OptD,VRef);
	MAL_VECTOR_RESIZE(OptD,2*m_QP_N);
	MAL_C_eq_A_by_B(OptD,m_OptB,xk);
	OptD -= lterm1v;

	for( int i=0;i<2*m_QP_N;i++)
	  m_Pb.D[i] += 2*OptD(i);


	//zmp
	for( int i=0;i<m_QP_N;i++)
	  {
	   m_Pb.D[i] += 2*lterm3ZMPx(i);
	   m_Pb.D[m_QP_N+i] += 2*lterm3ZMPy(i);
	  }
	for( int i=0;i<m_Support->StepNumber;i++)
	  {
	   m_Pb.D[2*m_QP_N+i] += 2*lterm4ZMPx(i);
	   m_Pb.D[2*m_QP_N+m_Support->StepNumber+i] += 2*lterm4ZMPy(i);
	  }
	//----------m_Pb.D
	for( int i=0;i<2*(m_QP_N+m_Support->StepNumber);i++)
	{
		m_Pb.XL[i] = -1e8;
		m_Pb.XU[i] = 1e8;
	}
	memset(m_Pb.X,0,2*(m_QP_N+m_Support->StepNumber)*sizeof(double));

}

// void ZMPVelocityReferencedQP::setProblem(int NbOfConstraints,  int NbOfEqConstraints, int &CriteriaToMaximize, MAL_VECTOR(& xk,double))
// {
//   m_Pb.m=NbOfConstraints;
//   m_Pb.me=NbOfEqConstraints;
//   m_Pb.mmax=m_Pb.m+1;
//   m_Pb.n=2*(m_QP_N+m_Support->StepNumber);
//   m_Pb.nmax=m_Pb.n;
//   m_Pb.mnn=m_Pb.m+2*m_Pb.n;

//   m_Pb.iout=0;
//   m_Pb.
//     iprint=1;
//   m_Pb.lwar=3*m_Pb.nmax*m_Pb.nmax/2+ 10*m_Pb.nmax  + 2*m_Pb.mmax + 20000;
//   m_Pb.liwar=m_Pb.n;
//   m_Pb.Eps=1e-8;

//   m_Pb.war= new double[m_Pb.lwar];
//   m_Pb.iwar = new int[m_Pb.liwar]; // The Cholesky decomposition is done internally.

//   if (m_FastFormulationMode==QLDANDLQ)
//     m_Pb.iwar[0]=0;
//   else
//     m_Pb.iwar[0]=1;

//   m_Pb.U = (double *)malloc( sizeof(double)*m_Pb.mnn); // Returns the Lagrange multipliers.;


//   MAL_MATRIX(OptA,double);
//   MAL_VECTOR(VRef,double);
//   MAL_MATRIX(lterm2,double);
//   MAL_VECTOR_DIM(OptD,double,2*m_QP_N);
//   MAL_VECTOR_RESIZE(VRef,2*m_QP_N);

//   lterm2 = MAL_RET_TRANSPOSE(m_VPu);
//   lterm2 = MAL_RET_A_by_B(lterm2,m_VPu);
//   lterm2 = m_Beta*lterm2;

//   MAL_MATRIX_RESIZE(OptA,
// 		    MAL_MATRIX_NB_ROWS(lterm2),
// 		    MAL_MATRIX_NB_COLS(lterm2));
//   MAL_MATRIX_SET_IDENTITY(OptA);
//   OptA = m_Alpha*OptA;

//   OptA = OptA + lterm2;


//   memset(m_Pb.Q,0,4*(m_QP_N+m_Support->StepNumber)*(m_QP_N+m_Support->StepNumber)*sizeof(double));
//   for( int i=0;i<2*(m_QP_N);i++)
//     for( int j=0;j<2*(m_QP_N);j++)
//       m_Pb.Q[i*2*(m_QP_N+m_Support->StepNumber)+j] = OptA(j,i);

//   m_OptB = MAL_RET_TRANSPOSE(m_VPu);
//   m_OptB = MAL_RET_A_by_B(m_OptB,m_VPx);
//   m_OptB = m_Beta * m_OptB;

//   //TODO 2: The matrices of the value function have to go back where they come from
//   //MAL_MATRIX(m_OptD,double);
//   m_OptD = MAL_RET_TRANSPOSE(m_VPu);
//   m_OptD = m_Beta * m_OptD;

//   //Andremize - only constant velocity
//   //constant velocity for the whole preview window
//   for( int i=0;i<m_QP_N;i++)
//     VRef(i) = RefVel.x;
//   for( int i=m_QP_N;i<2*m_QP_N;i++)
//     VRef(i) = RefVel.y;

//   memset(m_Pb.D,0,2*(m_QP_N+m_Support->StepNumber)*sizeof(double));
//   if (CriteriaToMaximize==1)
//     {
//       MAL_VECTOR(lterm1v,double);
//       MAL_C_eq_A_by_B(lterm1v,m_OptD,VRef);
//       MAL_VECTOR_RESIZE(OptD,2*m_QP_N);
//       MAL_C_eq_A_by_B(OptD,m_OptB,xk);
//       OptD -= lterm1v;

//       for( int i=0;i<2*m_QP_N;i++)
// 	m_Pb.D[i] = OptD(i);



//     }
//   else
//     {
//       // Default : set D to zero.
//       for( int i=0;i<2*(m_QP_N+m_Support->StepNumber);i++)
// 	m_Pb.D[i] = 0.0;
//     }

//   for( int i=0;i<2*(m_QP_N+m_Support->StepNumber);i++)
//     {
//       m_Pb.XL[i] = -1e8;
//       m_Pb.XU[i] = 1e8;
//     }
//   memset(m_Pb.X,0,2*(m_QP_N+m_Support->StepNumber)*sizeof(double));

// }

void ZMPVelocityReferencedQP::OnLine(double time,
				     deque<ZMPPosition> & FinalZMPPositions,
				     deque<COMPosition> & FinalCOMPositions,
				     deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
				     deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)
{

  if(time + 0.00001 > m_UpperTimeLimitToUpdate)
    {
      int NbOfConstraints; // Nb of constraints are not known in advance

      MAL_VECTOR_DIM(xk,double,6);

      int CriteriaToMaximize=1;

      deque<LinearConstraintInequality_t> QueueOfLConstraintInequalities;
      deque<LinearConstraintInequalityFreeFeet_t> QueueOfLConstraintInequalitiesFreeFeet;
      deque<LinearConstraintInequalityFreeFeet_t> QueueOfFeetPosInequalities;

      // pre compute the matrices needed for the optimization.
      double TotalAmountOfCPUTime=0.0,CurrentCPUTime=0.0;
      struct timeval start,end;
      int li=0;
      double dinterval = m_QP_T /  m_SamplingPeriod;
      int interval=(int)dinterval;
      bool StartingSequence = true;

      int NumberOfRemovedConstraints =0;

      //----------"Real-time" loop---------
      //
      //
      //-----------------------------------
      // printf("Inside the 'Real-time' loop: \n");

      // printf("StartingTime: %f \n", StartingTime);
      gettimeofday(&start,0);

      m_OP->verifyAccelerationOfHipJoint(RefVel, m_TrunkState,
					 m_TrunkStateT, m_Support);

      m_OP->previewOrientations(time+m_TimeBuffer, m_PreviewedSupportAngles, m_TrunkState, m_TrunkStateT, m_Support,
				FinalLeftFootAbsolutePositions, FinalRightFootAbsolutePositions);

      // Read the current state of the 2D Linearized Inverted Pendulum.
      m_2DLIPM->GetState(xk);

      //TODO : Add a get function to read the state
      m_Support->setSupportState(time+m_TimeBuffer, 0, RefVel);


      if(m_Support->StateChanged == 1)
	{
	  //			deque<SupportFeet_t>::iterator SF_it;
	  deque<FootAbsolutePosition>::iterator FAP_it;
	  SupportFeet_t newSF;
	  //			if(m_Support->SSSS == 0)//SS->DS or DS->SS
	  //			{
	  //				SF_it = QueueOfSupportFeet.end();
	  //				SF_it--;
	  //				//The m_Support foot does not change
	  //				if((SF_it)->SupportFoot != m_Support->CurrentSupportFoot)
	  //					SF_it--;
	  //				m_FPx = (SF_it)->x;
	  //				m_FPy = (SF_it)->y;
	  //				m_FPtheta = (SF_it)->theta;
	  //			}
	  //			else
	  //			{
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
	  //			}

	  newSF.x = FAP_it->x;
	  newSF.y = FAP_it->y;
	  newSF.theta = FAP_it->theta;
	  newSF.StartTime = time+m_TimeBuffer;
	  newSF.SupportFoot = m_Support->CurrentSupportFoot;

	  QueueOfSupportFeet.push_back(newSF);
	}


      m_fCALS->buildLinearConstraintInequalities(FinalLeftFootAbsolutePositions,
						 FinalRightFootAbsolutePositions,
						 QueueOfLConstraintInequalitiesFreeFeet,
						 QueueOfFeetPosInequalities,
						 RefVel,
						 time+m_TimeBuffer,
						 m_QP_N,
						 m_Support, m_PreviewedSupportAngles);


      initializeProblem();

      buildConstraintMatrices(m_Pb.DS,m_Pb.DU,
			      m_QP_N,m_QP_T,
			      time+m_TimeBuffer,
			      QueueOfLConstraintInequalitiesFreeFeet,
			      QueueOfFeetPosInequalities,
			      QueueOfSupportFeet,
			      m_ComHeight,
			      NbOfConstraints,
			      xk);

      setProblem(QueueOfLConstraintInequalitiesFreeFeet, QueueOfSupportFeet,
				NbOfConstraints, 0, CriteriaToMaximize, xk);
      // setProblem(NbOfConstraints, 0, CriteriaToMaximize, xk);

      if (m_FullDebug>0)
	{
	  ofstream aof;
	  char Buffer[1024];
	  sprintf(Buffer,"Dff_%f.dat",time+m_TimeBuffer);
	  aof.open(Buffer,ofstream::out);
	  for( int i=0;i<2*m_QP_N;i++)
	    {
	      aof << m_Pb.D[i] << endl;
	    }
	  aof.close();
	}

      if(m_FullDebug>2)
	dumpProblem(m_Pb.Q, m_Pb.D, m_Pb.DU, m_Pb.m, m_Pb.DS, m_Pb.XL, m_Pb.XU, time+m_TimeBuffer);

      double ldt = 0.0;
      //---------Solver------------
      //
      //
      //---------------------------
      // printf("Entering the solver \n");
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
	  // printf("Solver has finished,  \n");
	  int NbOfActivatedConstraints = 0;
	  for(int lk=0;lk<m_Pb.m;lk++)
	    {
	      if (m_Pb.U[lk]>0.0)
		{
		  NbOfActivatedConstraints++;
		}
	    }
	  ODEBUG6(NbOfActivatedConstraints,"InfosQLD.dat");
	  ODEBUG6(ldt,"dtQLD.dat");
	}
      // else if (m_FastFormulationMode==PLDP)
      // 	{
      // 	  ODEBUG("State: " << xk[0] << " " << xk[3] << " " <<
      // 		  xk[1] << " " << xk[4] << " " <<
      // 		  xk[2] << " " << xk[5] << " ");
      // 	  struct timeval lbegin,lend;
      // 	  gettimeofday(&lbegin,0);

      // 	  ifail=m_PLDPSolverHerdt->SolveProblem(D,
      // 					   (unsigned int)m,
      // 					   DU,
      // 					   DS,
      // 					   MAL_RET_VECTOR_DATABLOCK(ZMPRef),
      // 					   MAL_RET_VECTOR_DATABLOCK(xk),X,
      // 					   m_SimilarConstraints,
      // 					   NumberOfRemovedConstraints,
      // 					   StartingSequence);
      // 	  StartingSequence = false;
      // 	  NumberOfRemovedConstraints = NextNumberOfRemovedConstraints;
      // 	  gettimeofday(&lend,0);
      // 	  CODEDEBUG6(double ldt = lend.tv_sec - lbegin.tv_sec +
      // 		     0.000001 * (lend.tv_usec - lbegin.tv_usec););

      // 	  ODEBUG6(ldt,"dtPLDP.dat");
      // 	}

      if (m_Pb.ifail!=0)
	{
	  cout << "IFAIL: " << m_Pb.ifail << " at time: " << time << endl;
	  //return -1;
	}

      //------------------------
      //
      //
      //-------------------------

      double *ptX=0;
      if ((m_FastFormulationMode==QLDANDLQ)||
	  (m_FastFormulationMode==PLDP))
	{
	  /* Multiply the solution by the transpose of iLQ
      	     because it is a triangular matrix we do a specific
      	     multiplication.
	  */
	  memset(m_Pb.NewX,0,2*m_QP_N*sizeof(double));

	  double *pm_iLQ = MAL_RET_MATRIX_DATABLOCK(m_iLQ);
	  double *pNewX = m_Pb.NewX;

	  for( int i=0;i<2*m_QP_N;i++)
	    {
	      double *pX= m_Pb.X+i;
	      double *piLQ = pm_iLQ+i*2*m_QP_N+i;
	      *pNewX = 0.0;
	      for( int j=i;j<2*m_QP_N;j++)
		{
		  *pNewX+= (*piLQ) * (*pX++);
		  piLQ+=2*m_QP_N;
		}
	      pNewX++;
	    }
	  ptX=m_Pb.NewX;
	}
      else
	ptX=m_Pb.X;

      /* Simulation of the Single Point Mass model
      	 with the new command.
      */
      ODEBUG("X[0] " << X[0] << " X[N] :" << X[N]);

   

      FinalCOMPositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      FinalZMPPositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      FinalLeftFootAbsolutePositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));
      FinalRightFootAbsolutePositions.resize((int)((m_QP_T+m_TimeBuffer)/m_SamplingPeriod));

      int CurrentIndex = (int)(m_TimeBuffer/m_SamplingPeriod)-(int)(ldt/m_SamplingPeriod)-1;
      //cout<<"ldt: "<<ldt<<"(int)(ldt/m_SamplingPeriod): "<<(int)(ldt/m_SamplingPeriod)<<"(ldt/m_SamplingPeriod): "<<(ldt/m_SamplingPeriod)<<endl;
      // update the ZMP and COM positions.
      //cout<<"m_TimeBuffer/m_SamplingPeriod: "<<m_TimeBuffer/m_SamplingPeriod<<"(int)(m_TimeBuffer/m_SamplingPeriod): "<<(int)(m_TimeBuffer/m_SamplingPeriod)<<endl;
      m_2DLIPM->Interpolation(FinalCOMPositions,
			      FinalZMPPositions,
			      CurrentIndex,
			      ptX[0],ptX[m_QP_N]);

      m_2DLIPM->OneIteration(ptX[0],ptX[m_QP_N]);


      //Previewed position of the next foot
      if(m_Support->CurrentStepsLeft>0)
	{
	  if(fabs(ptX[2*m_QP_N])-0.00001>0.0)
	    {
	      m_FPx = ptX[2*m_QP_N];
	      m_FPy = ptX[2*m_QP_N+m_Support->StepNumber];
	    }
	}
      else
	{//The solver isn't responsible for the feet positions anymore
	  deque<SupportFeet_t>::iterator CurSF_it;
	  CurSF_it = QueueOfSupportFeet.end();
	  CurSF_it--;
	  while(CurSF_it->SupportFoot!=m_Support->CurrentSupportFoot)
	    CurSF_it--;
	  m_FPx = CurSF_it->x + double(CurSF_it->SupportFoot)*sin(CurSF_it->theta)*m_FeetDistanceDS;
	  m_FPy = CurSF_it->y - double(CurSF_it->SupportFoot)*cos(CurSF_it->theta)*m_FeetDistanceDS; 
	}

      if (m_FullDebug>2)
	{
	  ofstream aof;
	  char Buffer[1024];
	  sprintf(Buffer,"Xff_%f.dat",time);
	  aof.open(Buffer,ofstream::out);
	  //aof << "State: " <<xk[0]<<" "<<xk[1]<< " " << xk[2] << " " << xk[3] << " "<<xk[4]<<" "<<xk[5]<<" "<<endl;
	  for(int i=0;i<m_QP_N;i++)
	    {
	      aof << m_Pb.X[i] << endl;
	    }
	  aof.close();
	  sprintf(Buffer,"Yff_%f.dat",time);
	  aof.open(Buffer,ofstream::out);
	  //aof << "State: " <<xk[0]<<" "<<xk[1]<< " " << xk[2] << " " << xk[3] << " "<<xk[4]<<" "<<xk[5]<<" "<<endl;
	  for(int i=m_QP_N+m_Support->StepNumber;i<2*(m_QP_N+m_Support->StepNumber);i++)
	    {
	      aof << m_Pb.X[i] << endl;
	    }
	  aof.close();
	  aof.open("comHeight.dat",ofstream::app);

	  aof << FinalCOMPositions[CurrentIndex].x[0]<<" "<< FinalCOMPositions[CurrentIndex].y[0]<< FinalCOMPositions[CurrentIndex].z[0] << endl;

	  aof.close();
	  aof.open("CurrentIndex.dat",ofstream::app);
	  aof<<CurrentIndex<<endl;
	  aof.close();
	}

      //TODO :Jumps of 5ms
      if(m_FullDebug>2)
	{
	  ofstream aof;
	  aof.open("time.dat",ios::app);
	  aof<<time<<" "<<m_UpperTimeLimitToUpdate<<endl;
	}

      double LocalInterpolationTime = (time+m_TimeBuffer)-(m_Support->CurrentTimeLimit-m_Support->SSPeriod);
      // printf("FPx: %f FPy %f \n",FPx,FPy);

      // printf("Before interpolation \n");
      double StepHeight = 0.05;
      //
      if(m_Support->CurrentSupportPhase == 1 && time+m_TimeBuffer+3.0/2.0*m_QP_T < m_Support->CurrentTimeLimit)
	{
	  //determine coefficients of interpolation polynom
	  double ModulationSupportCoefficient = 0.9;
	  double ModulatedSingleSupportTime = (m_Support->SSPeriod-m_QP_T) * ModulationSupportCoefficient;
	  double EndOfLiftOff = ((m_Support->SSPeriod-m_QP_T)-ModulatedSingleSupportTime)*0.5;
	  double InterpolationTimePassed = 0.0;
	  if(LocalInterpolationTime>EndOfLiftOff)
	    InterpolationTimePassed = LocalInterpolationTime-EndOfLiftOff;

	  FootAbsolutePosition LastSwingFootPosition;

	  if(m_Support->CurrentSupportFoot==1)
	    {
	      LastSwingFootPosition = FinalRightFootAbsolutePositions[CurrentIndex];
	    }
	  else
	    {
	      LastSwingFootPosition = FinalLeftFootAbsolutePositions[CurrentIndex];
	    }
	  //Set parameters for current polynomial
	  m_FTGS->SetParametersWithInitPosInitSpeed(footTrajectoryGenerationStandard::X_AXIS,
						    ModulatedSingleSupportTime-InterpolationTimePassed,m_FPx,
						    LastSwingFootPosition.x,
						    LastSwingFootPosition.dx);
	  m_FTGS->SetParametersWithInitPosInitSpeed(footTrajectoryGenerationStandard::Y_AXIS,
						    ModulatedSingleSupportTime-InterpolationTimePassed,m_FPy,
						    LastSwingFootPosition.y,
						    LastSwingFootPosition.dy);

	  if(m_Support->StateChanged==1)
	    m_FTGS->SetParameters(footTrajectoryGenerationStandard::Z_AXIS, m_Support->SSPeriod-m_QP_T,StepHeight);

	  m_FTGS->SetParametersWithInitPosInitSpeed(footTrajectoryGenerationStandard::THETA_AXIS,
						    ModulatedSingleSupportTime-InterpolationTimePassed, 
						    m_PreviewedSupportAngles[0],
						    LastSwingFootPosition.theta,
						    LastSwingFootPosition.dtheta);
	  m_FTGS->SetParametersWithInitPosInitSpeed(footTrajectoryGenerationStandard::OMEGA_AXIS,
						    ModulatedSingleSupportTime-InterpolationTimePassed,0.0,
						    LastSwingFootPosition.omega,
						    LastSwingFootPosition.domega);
	  m_FTGS->SetParametersWithInitPosInitSpeed(footTrajectoryGenerationStandard::OMEGA2_AXIS,
						    ModulatedSingleSupportTime-InterpolationTimePassed,2*0.0,
						    LastSwingFootPosition.omega2,
						    LastSwingFootPosition.domega2);

	  //Set parameters for trunk interpolation

	  m_c = 3.0*(m_TrunkStateT.yaw[1]-m_TrunkState.yaw[1])/(m_QP_T*m_QP_T);
	  m_d = -2.0*m_c/(3.0*m_QP_T);
	  m_a =  m_TrunkState.yaw[1];


	  double tT;
	  double Theta = m_TrunkState.yaw[0];
	  double dTheta = m_TrunkState.yaw[1];
	  double ddTheta = m_TrunkState.yaw[2];
	  int StepType = 1;

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

	      if(m_FullDebug>2)
		{
		  ofstream aof;
		  aof.open("Trunk.dat",ofstream::app);
		  aof<<time+k*m_SamplingPeriod<<" "<<m_TrunkState.yaw[0]<<" "<<m_TrunkState.yaw[1]<<" "<<m_TrunkState.yaw[2]<<endl;
		  aof.close();
		}
	      if (m_Support->CurrentSupportFoot==1)
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
		  aoffeet.open("Feet.dat",ios::app);
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
      else if (m_Support->CurrentSupportPhase == 0 || time+m_TimeBuffer+3.0/2.0*m_QP_T > m_Support->CurrentTimeLimit)
	{
	  // printf("After parametrization SP == 0 \n");
	  for(int k = 0; k<=(int)(m_QP_T/m_SamplingPeriod);k++)
	    {
	      // cout<<"CurrentIndex+k"<<CurrentIndex+k<<FinalRightFootAbsolutePositions.size();
	      // cout<<" x ,y:"<<FinalRightFootAbsolutePositions[CurrentIndex+k-1].x<<
	      // 	FinalRightFootAbsolutePositions[CurrentIndex+k-1].y<<endl;
	      FinalRightFootAbsolutePositions[CurrentIndex+k]=FinalRightFootAbsolutePositions[CurrentIndex+k-1];
	      // cout<<"CurrentIndex+k"<<CurrentIndex+k<<endl;
	      FinalLeftFootAbsolutePositions[CurrentIndex+k]=FinalLeftFootAbsolutePositions[CurrentIndex+k-1];
	      // cout<<"CurrentIndex+k"<<CurrentIndex+k<<endl;
	      FinalLeftFootAbsolutePositions[CurrentIndex+k].time =
		FinalRightFootAbsolutePositions[CurrentIndex+k].time = time+m_TimeBuffer+k*m_SamplingPeriod;
	      // cout<<"CurrentIndex+k"<<CurrentIndex+k<<endl;
	      FinalLeftFootAbsolutePositions[CurrentIndex+k].stepType =
		FinalRightFootAbsolutePositions[CurrentIndex+k].stepType = 10;
	      // cout<<"CurrentIndex+k"<<CurrentIndex+k<<endl;
	      // cout<<"LFx: "<<FinalLeftFootAbsolutePositions[CurrentIndex+k].x<<"LFy: "<<FinalLeftFootAbsolutePositions[CurrentIndex+k].y<<"LFz: "<<FinalLeftFootAbsolutePositions[CurrentIndex+k].z<<endl;
	      // cout<<"RFx: "<<FinalRightFootAbsolutePositions[CurrentIndex+k].x<<"RFy: "<<FinalRightFootAbsolutePositions[CurrentIndex+k].y<<"RFz: "<<FinalRightFootAbsolutePositions[CurrentIndex+k].z<<endl;

	      if(m_FullDebug>0)
		{
		  ofstream aoffeet;
		  aoffeet.open("Feet.dat",ios::app);
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

      if(m_UpperTimeLimitToUpdate==0.0)
	m_UpperTimeLimitToUpdate = time+m_QP_T;
      else
	m_UpperTimeLimitToUpdate = m_UpperTimeLimitToUpdate+m_QP_T;

      //cout<<m_TrunkState.yaw[0]<<"   "<<m_TrunkStateT.yaw[0];
      //		m_TrunkState.yaw[0] = m_TrunkStateT.yaw[0];
      //		m_TrunkState.yaw[1] = m_AngVelTrunkConst;

      ODEBUG6("uk:" << uk,"DebugPBW.dat");
      ODEBUG6("xk:" << xk,"DebugPBW.dat");


      if (m_FullDebug>2)
	{
	  ofstream aof;
	  char Buffer[1024];
	  sprintf(Buffer,"Xff_%f.dat",time);
	  aof.open(Buffer,ofstream::out);
	  aof << "State: " <<xk[0]<<" "<<xk[1]<< " " << xk[2] << " " << xk[3] << " "<<xk[4]<<" "<<xk[5]<<" "<<endl;
	  for( int i=0;i<2*(m_QP_N+m_Support->StepNumber);i++)
	    {
	      aof << m_Pb.X[i] << endl;
	    }
	  aof.close();
	  // sprintf(Buffer,"Uff_%f.dat",StartingTime);
	  // aof.open(Buffer,ofstream::out);
	  // for(unsigned int i=0;i<2*(N+m_Support->StepNumber);i++)
	  //   {
	  //     aof << U[i] << endl;
	  //   }
	  // aof.close();
	}

      if(0)
	{
	  if(validateConstraints(m_Pb.DS, m_Pb.DU, m_Pb.m, li, m_Pb.X)<0)
	    {
	      cout << "Something is wrong with the constraints." << endl;
	      exit(-1);
	    }
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
      delete [] m_Pb.iwar; // The Cholesky decomposition is done internally.

      delete [] m_Pb.war;
      free(m_Pb.U);



    }

  // printf("Leaving online \n");
  //-----------------------------------
  //
  //
  //----------"Real-time" loop--------

}



int ZMPVelocityReferencedQP::OnLineFootChange(double time,
					      FootAbsolutePosition &aFootAbsolutePosition,
					      deque<ZMPPosition> & FinalZMPPositions,
					      deque<COMPosition> & CoMPositions,
					      deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
					      deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
					      StepStackHandler  *aStepStackHandler)
{
  cout << "To be implemented" << endl;
  return -1;
}

void ZMPVelocityReferencedQP::EndPhaseOfTheWalking(deque<ZMPPosition> &ZMPPositions,
						   deque<COMPosition> &FinalCOMPositions,
						   deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
						   deque<FootAbsolutePosition> &RightFootAbsolutePositions)
{

}

int ZMPVelocityReferencedQP::ReturnOptimalTimeToRegenerateAStep()
{
  int r = (int)(m_PreviewControlTime/m_SamplingPeriod);
  return 2*r;
}



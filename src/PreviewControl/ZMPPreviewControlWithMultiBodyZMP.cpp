/*! \file ZMPPreviewControlWithMultiBodyZMP.cpp
  \brief This object generate all the values for the foot trajectories,
  and the desired ZMP based on a sequence of steps.
  
  Copyright (c) 2005-2006,
  @author Olivier Stasse, Ramzi Sellouati
  
  JRL-Japan, CNRS/AIST

  All rights reserved.
  
  Please see License.txt for further information on license.
*/

#include <iostream>
#include <sstream>
#include <fstream>

#include <Debug.h>

#include <PreviewControl/ZMPPreviewControlWithMultiBodyZMP.h>

using namespace PatternGeneratorJRL;


ZMPPreviewControlWithMultiBodyZMP::ZMPPreviewControlWithMultiBodyZMP(SimplePluginManager *lSPM)
  : SimplePlugin(lSPM)
{

  m_ComAndFootRealization = 0;
  m_HumanoidDynamicRobot = 0;

  m_StageStrategy = ZMPCOM_TRAJECTORY_FULL;

  RESETDEBUG4("DebugData.txt");
  RESETDEBUG4("DebugDataqrql.txt");
  RESETDEBUG4("DebugDataDiffZMP.txt");
  RESETDEBUG4("DebugDataCOMPC1.txt");
  RESETDEBUG4("DebugDataWaistZMP.txt");
  RESETDEBUG4("DebugDataZMPMB1.txt");
  RESETDEBUG4("DebugDataDeltaCOM.txt");
  RESETDEBUG4("DebugDataStartingCOM.dat");
  RESETDEBUG4("DebugDataUB.txt");
  RESETDEBUG4("DebugDatadUB.txt");
  RESETDEBUG4("DebugDataIKL.dat");
  RESETDEBUG4("DebugDataIKR.dat");
  RESETDEBUG4("DebugDataWP.txt");
  RESETDEBUG4("Dump.dat");
  RESETDEBUG4("DebugConfSO.dat");
  RESETDEBUG4("DebugConfSV.dat");
  RESETDEBUG4("DebugConfSA.dat");
  RESETDEBUG4("DebugDataCheckZMP1.txt");
  RESETDEBUG4("2ndStage.dat");
  RESETDEBUG4("ZMPPCWMZOGSOC.dat");
  // Sampling period.
  m_SamplingPeriod = -1;
  
  RegisterMethods();

  // Initialization of the first and second preview controls.
  MAL_MATRIX_RESIZE(m_PC1x,3,1);  MAL_MATRIX_RESIZE(m_PC1y,3,1);
  MAL_MATRIX_RESIZE(m_Deltax,3,1);  MAL_MATRIX_RESIZE(m_Deltay,3,1);

  m_PC = new PreviewControl(lSPM,
			    OptimalControllerSolver::MODE_WITHOUT_INITIALPOS,
			    true);
  m_StartingNewSequence = true;

  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, i,j) =0.0;


  m_NumberOfIterations = 0;
}


ZMPPreviewControlWithMultiBodyZMP::~ZMPPreviewControlWithMultiBodyZMP()
{
}

void ZMPPreviewControlWithMultiBodyZMP::SetPreviewControl(PreviewControl *aPC)
{
  m_PC = aPC;
  m_SamplingPeriod = m_PC->SamplingPeriod();
  m_PreviewControlTime = m_PC->PreviewControlTime();
  m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);
}

void ZMPPreviewControlWithMultiBodyZMP::CallToComAndFootRealization(COMPosition &acomp,
								    FootAbsolutePosition &aLeftFAP,
								    FootAbsolutePosition &aRightFAP,
								    MAL_VECTOR(,double) &CurrentConfiguration,
								    MAL_VECTOR(,double) &CurrentVelocity,
								    MAL_VECTOR(,double) &CurrentAcceleration,
								    int IterationNumber,
								    int StageOfTheAlgorithm)
{

  // New scheme for WPG v3.0
  // We call the object in charge of generating the whole body
  // motion  ( for a given CoM and Feet points)  before applying the second filter.
  MAL_VECTOR_DIM(aCOMPosition,double,6);
  MAL_VECTOR_DIM(aCOMSpeed,double,6);
  MAL_VECTOR_DIM(aCOMAcc,double,6);
  
  aCOMPosition(0) = acomp.x[0];
  aCOMPosition(1) = acomp.y[0];
  aCOMPosition(2) = acomp.z[0];
  aCOMPosition(3) = 0;
  aCOMPosition(4) = acomp.pitch;
  aCOMPosition(5) = acomp.yaw;

  aCOMSpeed(0) = acomp.x[1];
  aCOMSpeed(1) = acomp.y[1];
  aCOMSpeed(2) = acomp.z[1];
  aCOMSpeed(3) = 0;
  aCOMSpeed(4) = 0;
  aCOMSpeed(5) = 0;

  aCOMAcc(0) = acomp.x[2];
  aCOMAcc(1) = acomp.y[2];
  aCOMAcc(2) = acomp.z[2];
  aCOMAcc(3) = 0;
  aCOMAcc(4) = 0;
  aCOMAcc(5) = 0;

  MAL_VECTOR_DIM(aLeftFootPosition,double,5);
  MAL_VECTOR_DIM(aRightFootPosition,double,5);

  aLeftFootPosition(0) = aLeftFAP.x;
  aLeftFootPosition(1) = aLeftFAP.y;
  aLeftFootPosition(2) = aLeftFAP.z;
  aLeftFootPosition(3) = aLeftFAP.theta;
  aLeftFootPosition(4) = aLeftFAP.omega;

  aRightFootPosition(0) = aRightFAP.x;
  aRightFootPosition(1) = aRightFAP.y;
  aRightFootPosition(2) = aRightFAP.z;
  aRightFootPosition(3) = aRightFAP.theta;
  aRightFootPosition(4) = aRightFAP.omega;

  /* Get the current configuration vector */
  CurrentConfiguration = m_HumanoidDynamicRobot->currentConfiguration();

  /* Get the current velocity vector */
  CurrentVelocity = m_HumanoidDynamicRobot->currentVelocity();

  /* Get the current acceleration vector */
  CurrentAcceleration = m_HumanoidDynamicRobot->currentAcceleration();
  ODEBUG(aCOMPosition(0)<< " " <<
	 aCOMPosition(1) << " " <<
	 aCOMPosition(2) );
  m_ComAndFootRealization->ComputePostureForGivenCoMAndFeetPosture(aCOMPosition, aCOMSpeed, aCOMAcc,
								   aLeftFootPosition,
								   aRightFootPosition,
								   CurrentConfiguration,
								   CurrentVelocity,
								   CurrentAcceleration,
								   IterationNumber,
								   StageOfTheAlgorithm);

  if (StageOfTheAlgorithm==0)
    {
      ODEBUG("StageOfTheAlgorithm=0 - CurrentConfiguration: " << CurrentConfiguration );
      ODEBUG("StageOfTheAlgorithm=0 - CurrentVelocity:" << CurrentVelocity );
      ODEBUG4(CurrentConfiguration(0) << " " << CurrentConfiguration(1) << " " << CurrentConfiguration(2) ,
	      "DebugDataWaistZMP.txt");
      /* Update the current configuration vector */
      m_HumanoidDynamicRobot->currentConfiguration(CurrentConfiguration);

      /* Update the current velocity vector */
      m_HumanoidDynamicRobot->currentVelocity(CurrentVelocity);

      /* Update the current acceleration vector */
      //m_HumanoidDynamicRobot->currentVelocity(CurrentAcceleration);
    }
}

/* Removed lqr and lql, now they should be set automatically by
   m_ComAndFootRealization */
int ZMPPreviewControlWithMultiBodyZMP::OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
							      FootAbsolutePosition &RightFootPosition,
							      ZMPPosition &NewZMPRefPos,
							      COMPosition &refandfinalCOMPosition,
							      MAL_VECTOR(,double) & CurrentConfiguration,
							      MAL_VECTOR(,double) & CurrentVelocity,
							      MAL_VECTOR(,double) & CurrentAcceleration)
{

  //  m_FIFOZMPRefPositions.push_back(NewZMPRefPos);

  ODEBUG("FirstStage " << CurrentConfiguration);
  ODEBUG("LFP : " << LeftFootPosition.x<< " " << LeftFootPosition.y << " " << LeftFootPosition.z);
  ODEBUG("RFP : " << RightFootPosition.x<< " " << RightFootPosition.y << " " << RightFootPosition.z);
  
  FirstStageOfControl(LeftFootPosition,RightFootPosition,refandfinalCOMPosition);
  // This call is suppose to initialize
  // correctly the current configuration, speed and acceleration.
  COMPosition acompos = m_FIFOCOMPositions[m_NL];
  FootAbsolutePosition aLeftFAP = m_FIFOLeftFootPosition[m_NL];
  FootAbsolutePosition aRightFAP = m_FIFORightFootPosition[m_NL];

  ODEBUG4SIMPLE(m_FIFOZMPRefPositions[0].px << " " <<
		m_FIFOZMPRefPositions[0].py << " " <<
		m_FIFOZMPRefPositions[0].pz << " " <<
		acompos.x[0] << " " <<
		acompos.y[0] << " " <<
		acompos.z[0] << " " <<
		aLeftFAP.x << " " <<
		aLeftFAP.y << " " <<
		aLeftFAP.z << " " <<
		aRightFAP.x << " " <<
		aRightFAP.y << " " <<
		aRightFAP.z,
		"ZMPPCWMZOGSOC.dat");
  ODEBUG("Before First CallToComAndFootRealization: " << CurrentConfiguration);
  CallToComAndFootRealization(acompos,aLeftFAP,aRightFAP,
			      CurrentConfiguration,
			      CurrentVelocity,
			      CurrentAcceleration,
			      m_NumberOfIterations,
			      0);

  ODEBUG("After First CallToComAndFootRealization: " << CurrentConfiguration);
  ODEBUG("1-refandfinalCOMPosition: x: " << refandfinalCOMPosition.x[0] << 
	 " y: " << refandfinalCOMPosition.y[0] <<
	 " z: " << refandfinalCOMPosition.z[0]);

  if (m_StageStrategy!=ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY)
    EvaluateMultiBodyZMP(-1);
  
  aLeftFAP = m_FIFOLeftFootPosition[0];
  aRightFAP = m_FIFORightFootPosition[0];
  
  SecondStageOfControl(refandfinalCOMPosition);
  ODEBUG4SIMPLE(refandfinalCOMPosition.x[0] << 
		" " << refandfinalCOMPosition.y[0] <<
		" " << refandfinalCOMPosition.z[0] <<
		" " << aLeftFAP.x <<
		" " << aLeftFAP.y <<
		" " << aLeftFAP.z << 
		" " << aLeftFAP.stepType << 
		" " << aRightFAP.x <<
		" " << aRightFAP.y <<
		" " << aRightFAP.z << 
		" " << aRightFAP.stepType
		, "2ndStage.dat");
  if (m_StageStrategy!=ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY)
    {
      CallToComAndFootRealization(refandfinalCOMPosition,aLeftFAP,aRightFAP,
				  CurrentConfiguration,
				  CurrentVelocity,
				  CurrentAcceleration,
				  m_NumberOfIterations - m_NL,
				  1);
      ODEBUG("After Second CallToComAndFootRealization: " << CurrentConfiguration);
    }
  ODEBUG("Current Configuration: "<< CurrentConfiguration);
  // Here it is assumed that the 4x4 CoM matrix 
  // is the orientation of the free flyer and
  // its position.
  double c,co,s,so;
  c = cos(CurrentConfiguration(5));
  s = sin(CurrentConfiguration(5));

  co = cos(CurrentConfiguration(4));
  so = sin(CurrentConfiguration(4));

  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 0,0) = c*co;
  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 0,1) = -s;
  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 0,2) = c*so;

  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 1,0) = s*co;
  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 1,1) =  c;
  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 1,2) = s*so;

  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 2,0) =  -so;
  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 2,1)=  0;
  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 2,2) = co;

  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 0,3) = refandfinalCOMPosition.x[0];
  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 1,3) = refandfinalCOMPosition.y[0];
  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 2,3) = refandfinalCOMPosition.z[0];
  MAL_S4x4_MATRIX_ACCESS_I_J(m_FinalDesiredCOMPose, 3,3) = 1.0;
  ODEBUG("End of second stage");

  ODEBUG4SIMPLE(CurrentConfiguration[6]<< " " <<
		CurrentConfiguration[7]<< " " <<
		CurrentConfiguration[8]<< " " <<
		CurrentConfiguration[9]<< " " <<
		CurrentConfiguration[10]<< " " <<
		CurrentConfiguration[11]<< " " <<
		CurrentConfiguration[12]<< " " <<
		CurrentConfiguration[13]<< " " <<
		CurrentConfiguration[14]<< " " <<
		CurrentConfiguration[15]<< " " <<
		CurrentConfiguration[16]<< " " <<
		CurrentConfiguration[17]<< " " <<
		CurrentConfiguration[18]<< " ",
		"DebugDataqrql.txt");
  m_NumberOfIterations++;
  return 1;
}


COMPosition ZMPPreviewControlWithMultiBodyZMP::GetLastCOMFromFirstStage()
{
  COMPosition aCOM;
  aCOM = m_FIFOCOMPositions.back();
  return aCOM;
}

int ZMPPreviewControlWithMultiBodyZMP::SecondStageOfControl(COMPosition &finalCOMPosition)
{
  double Deltazmpx2,Deltazmpy2;
  // Inverse Kinematics variables.

  COMPosition aCOMPosition = m_FIFOCOMPositions[0];
  ODEBUG("aCOMPosition: x: " << aCOMPosition.x[0] <<
	  " y: " << aCOMPosition.y[0] <<
	  " z: " << aCOMPosition.z[0]);
  FootAbsolutePosition LeftFootPosition, RightFootPosition;

  LeftFootPosition = m_FIFOLeftFootPosition[0];
  RightFootPosition = m_FIFORightFootPosition[0];

  // Preview control on delta ZMP.
  if ((m_StageStrategy==ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY)||
      (m_StageStrategy==ZMPCOM_TRAJECTORY_FULL))
    {
      ODEBUG(m_FIFODeltaZMPPositions[0].px << " " <<
	      m_FIFODeltaZMPPositions[0].py);

      ODEBUG("Second Stage Size of FIFODeltaZMPPositions: "<< m_FIFODeltaZMPPositions.size()
	      << " " << m_Deltax 
	      << " " << m_Deltay
	      << " " << m_sxDeltazmp
	      << " " << m_syDeltazmp
	      << " " << Deltazmpx2
	      << " " << Deltazmpy2);
      
      m_PC->OneIterationOfPreview(m_Deltax,m_Deltay, 
				  m_sxDeltazmp, m_syDeltazmp,
				  m_FIFODeltaZMPPositions,0,
				  Deltazmpx2,Deltazmpy2,
				  true);
      
      
      // Correct COM position    but be carefull this is the COM for NL steps behind.
      for(int i=0;i<3;i++)
	{
	  aCOMPosition.x[i] += m_Deltax(i,0);
	  aCOMPosition.y[i] += m_Deltay(i,0);
	}
    }

  ODEBUG("Delta :" 
	  << m_Deltax(0,0) << " " << m_Deltay(0,0) << " "
	  << aCOMPosition.x[0] << " " << aCOMPosition.y[0]);
  // Update finalCOMPosition
  finalCOMPosition = aCOMPosition;

  if ((m_StageStrategy==ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY)||
      (m_StageStrategy==ZMPCOM_TRAJECTORY_FULL))
    {

      m_FIFODeltaZMPPositions.pop_front();
    }
  m_FIFOCOMPositions.pop_front();
  m_FIFOLeftFootPosition.pop_front();
  m_FIFORightFootPosition.pop_front();
  
  ODEBUG("End");
  return 1;
}

int ZMPPreviewControlWithMultiBodyZMP::FirstStageOfControl( FootAbsolutePosition &LeftFootPosition,
							    FootAbsolutePosition &RightFootPosition,
							    COMPosition& afCOMPosition )

{


  double zmpx2, zmpy2;
  COMPosition acomp;
  acomp.yaw = 0.0;
  acomp.pitch = 0.0;
  if ((m_StageStrategy==ZMPCOM_TRAJECTORY_FULL)
      || (m_StageStrategy==ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY))
    {
      ODEBUG("First Stage "<< m_FIFOZMPRefPositions.size());
      ODEBUG4( m_PC1x(0,0) << " " <<
	       m_PC1x(1,0) << " " <<
	       m_PC1x(2,0) << " " <<
	       m_PC1y(0,0) << " " <<
	       m_PC1y(1,0) << " " <<
	       m_PC1y(2,0) << " " <<
	       m_sxzmp << " " << 
	       m_syzmp << " " << 
	       zmpx2 << " " <<
	       zmpy2 , "DebugDataCheckZMP1.txt" );
      m_PC->OneIterationOfPreview(m_PC1x,m_PC1y,
				  m_sxzmp,m_syzmp,
				  m_FIFOZMPRefPositions,0,
				  zmpx2, zmpy2, true);
      for(unsigned j=0;j<3;j++)
	acomp.x[j] = m_PC1x(j,0);

      for(unsigned j=0;j<3;j++)
	acomp.y[j] = m_PC1y(j,0);

      for(unsigned j=0;j<3;j++)
	acomp.z[j] = afCOMPosition.z[j];

      acomp.yaw = afCOMPosition.yaw;
      acomp.pitch = afCOMPosition.pitch;

    }
  else  if (m_StageStrategy==ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY)
    {
      for(unsigned j=0;j<3;j++)
	acomp.x[j] = m_PC1x(j,0)= afCOMPosition.x[j];

      for(unsigned j=0;j<3;j++)
	acomp.y[j] = m_PC1y(j,0)= afCOMPosition.y[j];

      for(unsigned j=0;j<3;j++)
	acomp.z[j] = afCOMPosition.z[j];

      acomp.yaw = afCOMPosition.yaw;
      acomp.pitch = afCOMPosition.pitch;
    }


  // Update of the FIFOs
  m_FIFOCOMPositions.push_back(acomp);
  m_FIFORightFootPosition.push_back(RightFootPosition);
  m_FIFOLeftFootPosition.push_back(LeftFootPosition);
  
  ODEBUG("FIFOs COM:"<< m_FIFOCOMPositions.size() <<
	 " RF: "<< m_FIFORightFootPosition.size() <<
	 " LF: "<< m_FIFOLeftFootPosition.size());
  m_FIFOZMPRefPositions.pop_front();

  return 1;
}

int ZMPPreviewControlWithMultiBodyZMP::EvaluateMultiBodyZMP(int StartingIteration)
{
  string sComputeZMP("ComputeZMP");
  string sZMPtrue("true");

  ODEBUG("Start EvaluateMultiBodyZMP");
  // Call the  Dynamic Multi Body computation of the dynamic parameters.
  m_HumanoidDynamicRobot->setProperty(sComputeZMP,sZMPtrue);
  m_HumanoidDynamicRobot->computeForwardKinematics();

  // Call the Humanoid Dynamic Multi Body robot model to
  // compute the ZMP related to the motion found by CoMAndZMPRealization.
  MAL_S3_VECTOR(,double) ZMPmultibody;
  ZMPmultibody = m_HumanoidDynamicRobot->zeroMomentumPoint();
  ODEBUG4(ZMPmultibody[0] << " " << ZMPmultibody[1], "DebugDataCheckZMP1.txt");
  MAL_S3_VECTOR(,double) CoMmultibody;
  CoMmultibody = m_HumanoidDynamicRobot->positionCenterOfMass();
  ODEBUG("Stage 2");
  // Fill the delta ZMP FIFO for the second stage of the control.
  ZMPPosition aZMPpos;
  aZMPpos.px = m_FIFOZMPRefPositions[0].px - ZMPmultibody[0];
  aZMPpos.py = m_FIFOZMPRefPositions[0].py - ZMPmultibody[1];
  aZMPpos.pz = 0.0;
  aZMPpos.theta = 0.0;
  aZMPpos.stepType = 1;
  aZMPpos.time = m_FIFOZMPRefPositions[0].time;
  ODEBUG("Stage 3");
  string inProperty("Iteration");
  string inValue("-1");
  m_HumanoidDynamicRobot->getProperty(inProperty,inValue);
  MAL_VECTOR(,double) CurrentConfiguration;
  /* Get the current configuration vector */
  CurrentConfiguration = m_HumanoidDynamicRobot->currentConfiguration();

  ODEBUG("Stage 4");
  m_FIFODeltaZMPPositions.push_back(aZMPpos);
  m_StartingNewSequence = false;
  ODEBUG("Final");
  return 1;
}



int ZMPPreviewControlWithMultiBodyZMP::Setup(deque<ZMPPosition> &ZMPRefPositions,
					     deque<COMPosition> &COMPositions,
					     deque<FootAbsolutePosition> &LeftFootPositions,
					     deque<FootAbsolutePosition> &RightFootPositions)
{
  m_NumberOfIterations = 0;
  MAL_VECTOR(,double) CurrentConfiguration = m_HumanoidDynamicRobot->currentConfiguration();
  MAL_VECTOR(,double) CurrentVelocity = m_HumanoidDynamicRobot->currentVelocity();
  MAL_VECTOR(,double) CurrentAcceleration = m_HumanoidDynamicRobot->currentAcceleration();
  
  m_PC->ComputeOptimalWeights(OptimalControllerSolver::MODE_WITHOUT_INITIALPOS);

  string inProperty[5]={"TimeStep","ComputeAcceleration",
			"ComputeBackwardDynamics", "ComputeZMP",
			"ResetIteration"};
  ostringstream oss;
  oss << m_SamplingPeriod;
  string inValue[5]={oss.str(),"false","false","true","true"};
  
  for(unsigned int i=0;i<5;i++)
    m_HumanoidDynamicRobot->setProperty(inProperty[i],
					inValue[i]);

  SetupFirstPhase(ZMPRefPositions,
		  COMPositions,
		  LeftFootPositions,
		  RightFootPositions);
  for(unsigned int i=0;i<m_NL;i++)
    SetupIterativePhase(ZMPRefPositions,
			COMPositions,
			LeftFootPositions,
			RightFootPositions,
			CurrentConfiguration,
			CurrentVelocity,
			CurrentAcceleration,
			i);
  ODEBUG4("<========================================>","ZMPPCWMZOGSOC.dat");
  return 0;
}

int ZMPPreviewControlWithMultiBodyZMP::SetupFirstPhase(deque<ZMPPosition> &ZMPRefPositions,
						       deque<COMPosition> &COMPositions,
						       deque<FootAbsolutePosition> &LeftFootPositions,
						       deque<FootAbsolutePosition> &RightFootPositions)
{
  ODEBUG6("Beginning of Setup 0 ","DebugData.txt");
  ODEBUG("Setup");
  double zmpx2, zmpy2;


  m_sxzmp =0.0;
  m_syzmp =0.0;
  m_sxDeltazmp = 0.0;
  m_syDeltazmp = 0.0;


  m_StartingNewSequence = true;

  // Fill the Fifo
  m_FIFOZMPRefPositions.resize(m_NL);
  m_FIFOLeftFootPosition.resize(m_NL);
  m_FIFORightFootPosition.resize(m_NL);
  for(unsigned int i=0;i<m_NL;i++)
    {
      m_FIFOZMPRefPositions[i] = ZMPRefPositions[i];
      m_FIFOLeftFootPosition[i] = LeftFootPositions[i];
      m_FIFORightFootPosition[i] = RightFootPositions[i];
    }

  ODEBUG6("After EvaluateCOM","DebugData.txt");

  ODEBUG6("Beginning of Setup 1 ","DebugData.txt");
  m_PC1x(0,0)= m_StartingCOMPosition[0];
  ODEBUG("COMPC1 init X: " <<  m_PC1x(0,0));
  //m_PC1x(0,0) = 0.0;
  m_PC1x(1,0)= 0.0;
  m_PC1x(2,0)= 0.0;

  m_PC1y(0,0)= m_StartingCOMPosition[1];
  ODEBUG("COMPC1 init Y: " <<  m_PC1y(0,0));
  //m_PC1y(0,0) = 0.0;
  m_PC1y(1,0)= 0;
  m_PC1y(2,0)= 0;

  m_Deltax(0,0)= 0.0; //-StartingCOMPosition[0];
  m_Deltax(1,0)= 0;    m_Deltax(2,0)= 0;
  m_Deltay(0,0)= 0.0; //-StartingCOMPosition[1];
  m_Deltay(1,0)= 0;    m_Deltay(2,0)= 0;

  //  m_sxzmp=-StartingCOMPosition[0];m_syzmp=-StartingCOMPosition[1];
  //  zmpx2=StartingCOMPosition[0];zmpy2=StartingCOMPosition[1];
  m_sxzmp = 0.0; m_syzmp =0.0;
  zmpx2 = 0.0; zmpy2 = 0.0;

  m_FIFODeltaZMPPositions.clear();
  m_FIFOCOMPositions.clear();


  MAL_VECTOR(CurrentConfiguration,double);
  MAL_VECTOR(CurrentVelocity,double);
  MAL_VECTOR(CurrentAcceleration,double);
  /* Get the current configuration vector */
  CurrentConfiguration = m_HumanoidDynamicRobot->currentConfiguration();
  CurrentVelocity = m_HumanoidDynamicRobot->currentVelocity();
  CurrentAcceleration = m_HumanoidDynamicRobot->currentAcceleration();

  for(unsigned int i=0;i<MAL_VECTOR_SIZE(CurrentVelocity);i++)
    {
      CurrentVelocity[i] = CurrentAcceleration[i] = 0.0;
    }
  m_HumanoidDynamicRobot->currentVelocity(CurrentVelocity);
  m_HumanoidDynamicRobot->currentAcceleration(CurrentAcceleration);

  for(unsigned int i=0;i<0;i++)
    {
      string sComputeZMP("ComputeZMP");
      string sZMPtrue("true");
      // Call the  Dynamic Multi Body computation of the dynamic parameters.
      m_HumanoidDynamicRobot->setProperty(sComputeZMP,sZMPtrue);
      m_HumanoidDynamicRobot->computeForwardKinematics();
    }


#ifdef _DEBUG_MODE_ON_
  m_FIFOTmpZMPPosition.clear();
#endif

  return 0;
}


int ZMPPreviewControlWithMultiBodyZMP::SetupIterativePhase(deque<ZMPPosition> &ZMPRefPositions,
							   deque<COMPosition> &COMPositions,
							   deque<FootAbsolutePosition> &LeftFootPositions,
							   deque<FootAbsolutePosition> &RightFootPositions,
							   MAL_VECTOR(,double) & CurrentConfiguration,
							   MAL_VECTOR(,double) & CurrentVelocity,
							   MAL_VECTOR(,double) & CurrentAcceleration,
							   int localindex)
{

  ODEBUG("SetupIterativePhase " << localindex << " " << CurrentConfiguration );
  ODEBUG("COMPosition["<<localindex<<"]=" << COMPositions[localindex].x[0] << " " << 
	  COMPositions[localindex].y[0] << " " << COMPositions[localindex].z[0] <<
	  " COMPositions.size()=" <<COMPositions.size());
  FirstStageOfControl(LeftFootPositions[localindex],RightFootPositions[localindex],COMPositions[localindex]);
  ODEBUG("m_FIFOCOMPositions["<<localindex<<"]=" << m_FIFOCOMPositions[localindex].x[0] << " " << 
	  m_FIFOCOMPositions[localindex].y[0] << " " << m_FIFOCOMPositions[localindex].z[0] <<
	  " m_FIFOCOMPositions.size()=" <<m_FIFOCOMPositions.size());
  COMPosition acompos = m_FIFOCOMPositions[localindex];
  FootAbsolutePosition aLeftFAP = m_FIFOLeftFootPosition[localindex];
  FootAbsolutePosition aRightFAP = m_FIFORightFootPosition[localindex];

  ODEBUG4SIMPLE(m_FIFOZMPRefPositions[0].px << " " <<
		m_FIFOZMPRefPositions[0].py << " " <<
		m_FIFOZMPRefPositions[0].pz << " " <<
		acompos.x[0] << " " <<
		acompos.y[0] << " " <<
		acompos.z[0] << " " <<
		aLeftFAP.x << " " <<
		aLeftFAP.y << " " <<
		aLeftFAP.z << " " <<
		aRightFAP.x << " " <<
		aRightFAP.y << " " <<
		aRightFAP.z,
		"ZMPPCWMZOGSOC.dat");

  CallToComAndFootRealization(m_FIFOCOMPositions[localindex],
			      m_FIFORightFootPosition[localindex],
			      m_FIFOLeftFootPosition[localindex],
			      CurrentConfiguration,
			      CurrentVelocity,
			      CurrentAcceleration,
			      m_NumberOfIterations,
			      0);


  EvaluateMultiBodyZMP(localindex);

  m_FIFOZMPRefPositions.push_back(ZMPRefPositions[localindex+1+m_NL]);

  m_NumberOfIterations++;
  return 0;
}
void ZMPPreviewControlWithMultiBodyZMP::CreateExtraCOMBuffer(deque<COMPosition> &m_ExtraCOMBuffer,
							     deque<ZMPPosition> &m_ExtraZMPBuffer,
							     deque<ZMPPosition> &m_ExtraZMPRefBuffer)

{
  deque<ZMPPosition> aFIFOZMPRefPositions;
  MAL_MATRIX(aPC1x,double);
  MAL_MATRIX(aPC1y,double);
  double aSxzmp, aSyzmp;
  double aZmpx2, aZmpy2;

  //initialize ZMP FIFO
  for (unsigned int i=0;i<m_NL;i++)
    aFIFOZMPRefPositions.push_back(m_ExtraZMPRefBuffer[i]);

  //use accumulated zmp error  of preview control so far
  aSxzmp = m_sxzmp;
  aSyzmp = m_syzmp;

  aPC1x = m_PC1x;
  aPC1y = m_PC1y;

  //create the extra COMbuffer

#ifdef _DEBUG_MODE_ON_
  ofstream aof_ExtraCOM;
  static unsigned char FirstCall=1;
  if (FirstCall)
    {
      aof_ExtraCOM.open("CartExtraCOM_1.dat",ofstream::out);
    }
  else
    {
      aof_ExtraCOM.open("CartExtraCOM_1.dat",ofstream::app);
    }

  if (FirstCall)
    FirstCall = 0;
#endif

  for (unsigned int i=0;i<m_ExtraCOMBuffer.size();i++)
    {
      aFIFOZMPRefPositions.push_back(m_ExtraZMPRefBuffer[i]);
      m_PC->OneIterationOfPreview(aPC1x,aPC1y,
				  aSxzmp,aSyzmp,
				  aFIFOZMPRefPositions,0,
				  aZmpx2, aZmpy2, true);

      for(unsigned j=0;j<3;j++)
	{
	  m_ExtraCOMBuffer[i].x[j] = aPC1x(j,0);
	  m_ExtraCOMBuffer[i].y[j] = aPC1y(j,0);
	}

      m_ExtraZMPBuffer[i].px=aZmpx2;
      m_ExtraZMPBuffer[i].py=aZmpy2;

      m_ExtraCOMBuffer[i].yaw = m_ExtraZMPRefBuffer[i].theta;

      aFIFOZMPRefPositions.pop_front();


#ifdef _DEBUG_MODE_ON_
      if (aof_ExtraCOM.is_open())
	{
	  aof_ExtraCOM << m_ExtraZMPRefBuffer[i].time << " "
		       << m_ExtraZMPRefBuffer[i].px << " "
		       << aPC1x(0,0)<< " "
		       << aPC1y(0,0) << endl;
	}
#endif

    }
  ODEBUG("ik ben hier b" << aFIFOZMPRefPositions.size() );
  ODEBUG("ik ben hier c" << m_ExtraZMPRefBuffer.size());

#ifdef _DEBUG_MODE_ON_
  if (aof_ExtraCOM.is_open())
    {
      aof_ExtraCOM.close();
    }
#endif

}




void ZMPPreviewControlWithMultiBodyZMP::UpdateTheZMPRefQueue(ZMPPosition NewZMPRefPos)
{
  m_FIFOZMPRefPositions.push_back(NewZMPRefPos);
}

void ZMPPreviewControlWithMultiBodyZMP::SetStrategyForStageActivation(int aZMPComTraj)
{
  switch(aZMPComTraj)
    {
    case ZMPCOM_TRAJECTORY_FULL:
      m_StageStrategy = ZMPCOM_TRAJECTORY_FULL;
      break;
    case ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY:
      m_StageStrategy = ZMPCOM_TRAJECTORY_SECOND_STAGE_ONLY;
      break;
    case ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY:
      m_StageStrategy = ZMPCOM_TRAJECTORY_FIRST_STAGE_ONLY;
      break;

    default:
      break;
    }
}

int ZMPPreviewControlWithMultiBodyZMP::GetStrategyForStageActivation()
{
  return m_StageStrategy;
}

// TODO : Compute the position of the waist inside the COM Frame.
MAL_S4x4_MATRIX(,double) ZMPPreviewControlWithMultiBodyZMP::GetCurrentPositionofWaistInCOMFrame()
{
  MAL_S4x4_MATRIX(,double) PosOfWaistInCoMFrame;
  PosOfWaistInCoMFrame = m_ComAndFootRealization->GetCurrentPositionofWaistInCOMFrame();
  //  cerr << " Should implement: ZMPPreviewControlWithMultiBodyZMP::GetCurrentPositionOfWaistInCOMFrame()" << endl;
  return PosOfWaistInCoMFrame;
}

MAL_S4x4_MATRIX(,double)  ZMPPreviewControlWithMultiBodyZMP::GetFinalDesiredCOMPose()
{
   return m_FinalDesiredCOMPose;
}


int ZMPPreviewControlWithMultiBodyZMP::EvaluateStartingState(MAL_VECTOR(&BodyAnglesInit,double),
							     MAL_S3_VECTOR(&aStartingCOMPosition,double),
							     MAL_S3_VECTOR(&,double) aStartingZMPPosition,
							     MAL_VECTOR(&,double) aStartingWaistPosition,
							     FootAbsolutePosition & InitLeftFootPosition,
							     FootAbsolutePosition & InitRightFootPosition)
{
  int r = EvaluateStartingCoM(BodyAnglesInit,aStartingCOMPosition,
			      aStartingWaistPosition,
			      InitLeftFootPosition, InitRightFootPosition);
  aStartingZMPPosition= m_ComAndFootRealization->GetCOGInitialAnkles();
  return r;
}
int ZMPPreviewControlWithMultiBodyZMP::EvaluateStartingCoM(MAL_VECTOR(&BodyAnglesInit,double),
							   MAL_S3_VECTOR(&aStartingCOMPosition,double),
							   MAL_VECTOR(&,double) aStartingWaistPosition,
							   FootAbsolutePosition & InitLeftFootPosition,
							   FootAbsolutePosition & InitRightFootPosition)
{
  ODEBUG("EvaluateStartingCOM: BodyAnglesInit :" << BodyAnglesInit);

  m_ComAndFootRealization->InitializationCoM(BodyAnglesInit,m_StartingCOMPosition,
					     aStartingWaistPosition,
					     InitLeftFootPosition, InitRightFootPosition);  
  ODEBUG("EvaluateStartingCOM: m_StartingCOMPosition: " << m_StartingCOMPosition);
  aStartingCOMPosition[0] = m_StartingCOMPosition[0];
  aStartingCOMPosition[1] = m_StartingCOMPosition[1];
  aStartingCOMPosition[2] = m_StartingCOMPosition[2];

  return 0;
}

void ZMPPreviewControlWithMultiBodyZMP::SetStrategyForPCStages(int Strategy)
{
  m_StageStrategy = Strategy;
}

int ZMPPreviewControlWithMultiBodyZMP::GetStrategyForPCStages()
{
  return m_StageStrategy;
}


void ZMPPreviewControlWithMultiBodyZMP::RegisterMethods()
{
  std::string aMethodName[3] = 
    {":samplingperiod",
     ":previewcontroltime",
     ":comheight"};
  
  for(int i=0;i<3;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
      else
	{
	  ODEBUG("Succeed in registering " << aMethodName[i]);
	}

    }

}

void ZMPPreviewControlWithMultiBodyZMP::SetSamplingPeriod(double lSamplingPeriod)
{
  m_SamplingPeriod = lSamplingPeriod;

  m_NL=0.0;
  if (m_SamplingPeriod!=0.0)
    m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);
  
}

void ZMPPreviewControlWithMultiBodyZMP::SetPreviewControlTime(double lPreviewControlTime)
{
  m_PreviewControlTime = lPreviewControlTime;

  m_NL=0.0;
  if (m_SamplingPeriod!=0.0)
    m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);

}

void ZMPPreviewControlWithMultiBodyZMP::CallMethod(std::string &Method,
						   std::istringstream &strm)
{
  if (Method==":samplingperiod")
    {
      std::string aws;
      if (strm.good())
	{
	  double lSamplingPeriod;
	  strm >> lSamplingPeriod;
	  SetSamplingPeriod(lSamplingPeriod);
	}
    }
  else if (Method==":previewcontroltime")
    {
      std::string aws;
      if (strm.good())
	{
	  double lpreviewcontroltime;
	  strm >> lpreviewcontroltime;
	  SetPreviewControlTime(lpreviewcontroltime);
	}
    }

}

/** @doc This object generate all the values for the foot trajectories,
    and the desired ZMP based on a sequence of steps.
   
    CVS Information:
    $Id: ZMPPreviewControlWithMultiBodyZMP.cpp,v 1.4 2006-01-18 06:34:58 stasse Exp $
    $Author: stasse $
    $Date: 2006-01-18 06:34:58 $
    $Revision: 1.4 $
    $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/ZMPPreviewControlWithMultiBodyZMP.cpp,v $
    $Log: ZMPPreviewControlWithMultiBodyZMP.cpp,v $
    Revision 1.4  2006-01-18 06:34:58  stasse
    OS: Updated the names of the contributors, the documentation
    and added a sample file for WalkPlugin


    Copyright (c) 2005-2006, 
    @author Olivier Stasse, Ramzi Sellouati
   
    JRL-Japan, CNRS/AIST

    All rights reserved.
   
    Redistribution and use in source and binary forms, with or without modification, 
    are permitted provided that the following conditions are met:
   
    * Redistributions of source code must retain the above copyright notice, 
    this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, 
    this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    * Neither the name of the <ORGANIZATION> nor the names of its contributors 
    may be used to endorse or promote products derived from this software without specific prior written permission.
   
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
    OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
    AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
    OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
    OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
    STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
    IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//#define _DEBUG_
#include <iostream>
#include <fstream>
#include <ZMPPreviewControlWithMultiBodyZMP.h>

using namespace PatternGeneratorJRL;

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "ZMPPCWMBZ: " << x << endl; DebugFile.close();}
#else
#define RESETDEBUG4(y) 
#define ODEBUG4(x,y) 
#endif

#define RESETDEBUG6(y) 
#define ODEBUG6(x,y) 

#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "ZMPPCWMBZ: " << x << endl; DebugFile.close();}
#if 1
#define ODEBUG(x)
#else
#define ODEBUG(x)  std::cout << x << endl;
#endif

#define ODEBUG3(x)  std::cout << x << endl;

ZMPPreviewControlWithMultiBodyZMP::ZMPPreviewControlWithMultiBodyZMP(HumanoidSpecificities *aHS)
{
  m_HS = aHS;
  m_ZMPCoMTrajectoryAlgorithm = 1;

  if (m_HS!=0)
    {
      double AnklePosition[3];
      // Take the right ankle position (should be equivalent)
      m_HS->GetAnklePosition(-1,AnklePosition);
      m_AnkleSoilDistance = AnklePosition[2];
      ODEBUG("AnkleSoilDistnace =" << m_AnkleSoilDistance);
    }
  else
    {
      cerr << "Warning: No appropriate definition of Humanoid Specifities" << endl;
      cerr << "Use default value: " << 0.1 << endl;
      m_AnkleSoilDistance = 0.1;
    }

  RESETDEBUG4("DebugData.txt");
  RESETDEBUG4("DebugDataqr.txt");
  RESETDEBUG4("DebugDataql.txt");
  RESETDEBUG4("DebugDatadqr.txt");
  RESETDEBUG4("DebugDatadql.txt");
  RESETDEBUG4("DebugDataDiffZMP.txt");
  RESETDEBUG4("DebugDataCOMPC1.txt");
  RESETDEBUG4("DebugDataCOMPC2.txt");
  RESETDEBUG4("DebugDataZMPMB1.txt");
  RESETDEBUG4("DebugDataDeltaCOM.txt");
  RESETDEBUG4("DebugDataStartingCOM.dat");
  RESETDEBUG4("DebugDataUB.txt");
  RESETDEBUG4("DebugDatadUB.txt");
  RESETDEBUG4("DebugDataIKL.dat");
  RESETDEBUG4("DebugDataIKR.dat");
  RESETDEBUG4("DebugDataWP.txt");

  m_DiffBetweenComAndWaist.Resize(3);

  for(unsigned int i=0;i<3;i++)
    m_DiffBetweenComAndWaist[i] = 0.0;
  m_Dt.Resize(3,1);

#if 1
  // Displacement between the hip and the waist.
  if (m_HS==0)
    {
      m_Dt(0,0) = 0.0;
      m_Dt(1,0) = 0.04;
      m_Dt(2,0) = 0.0;
    }
  else
    {
      double WaistToHip[3];
      // Takes the left one.
      m_HS->GetWaistToHip(1,WaistToHip);
      ODEBUG3(WaistToHip[0] << " "
	      << WaistToHip[1] << " "
	      << WaistToHip[2] << " ");
      m_Dt(0,0) = WaistToHip[0];
      m_Dt(1,0) = WaistToHip[1];
      m_Dt(2,0) = WaistToHip[2];
    }
#else
  m_Dt(0,0) = 0.0;
  m_Dt(1,0) = 0.035;
  m_Dt(2,0) = 0.0;
#endif
  ODEBUG3( "m_DT: " << m_Dt);

  m_StaticToTheLeftHip.Resize(3,1);
  m_StaticToTheRightHip.Resize(3,1);
  m_TranslationToTheLeftHip.Resize(3,1);
  m_TranslationToTheRightHip.Resize(3,1);

  // Displacement between the COM and the waist
  // WARNING : Specific to HRP2 !

  VNL::Matrix<double> StaticToTheLeftHip(3,1),StaticToTheRightHip(3,1);
  m_StaticToTheLeftHip(0,0) = 0.0;
  m_StaticToTheLeftHip(1,0) = 0.06;
  m_StaticToTheLeftHip(2,0) = 0.0; // BE CAREFUL HAS TO BEEN INITIALIZED by EvaluateCOM DiffBetweenComAndWaist;
  m_TranslationToTheLeftHip = m_StaticToTheLeftHip;

  m_StaticToTheRightHip(0,0) = 0.0;
  m_StaticToTheRightHip(1,0) = -0.06;
  m_StaticToTheRightHip(2,0) = 0.0; // BE CAREFUL HAS TO BEEN INITIALIZED by EvaluateCOM DiffBetweenComAndWaist;
  m_TranslationToTheRightHip = m_StaticToTheRightHip;


  // Initialize the joint related variables.
  m_prev_ql.Resize(6,1);
  m_prev_qr.Resize(6,1);
	
  m_prev_UpperBodyAngles.Resize(28,1);

  // Initialize the size of the final leg joint values.
  Finalql.Resize(6,1);
  Finalqr.Resize(6,1);

  // Sampling period.
  m_SamplingPeriod = -1;


  // Initialization of the first and second preview controls.
  m_PC1x.Resize(3,1);  m_PC1y.Resize(3,1);
  m_Deltax.Resize(3,1);  m_Deltay.Resize(3,1);

  m_DMB = 0;
  m_PC = 0;
  m_IK = 0;
  m_StartingNewSequence = true;

  m_FinalDesiredCOMPose.Resize(4,4);	
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)	
      m_FinalDesiredCOMPose(i,j) =0.0;

  
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

void ZMPPreviewControlWithMultiBodyZMP::SetDynamicMultiBodyModel(DynamicMultiBody *aDMB)
{
  m_DMB = aDMB;
  for(int i=0;i<m_DMB->NbOfLinks();i++)
    m_DMB->Setdq(i,0.0);

}

void ZMPPreviewControlWithMultiBodyZMP::SetInverseKinematics(InverseKinematics *anIK)
{
  m_IK = anIK;
}



int ZMPPreviewControlWithMultiBodyZMP::OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
							      FootAbsolutePosition &RightFootPosition,
							      ZMPPosition &NewZMPRefPos,
							      VNL::Matrix<double> & lqr, 
							      VNL::Matrix<double> & lql, 
							      COMPosition & refCOMPosition,
							      COMPosition & finalCOMPosition,
							      VNL::Matrix<double> &UpperBodyAngles)
{
  VNL::Matrix<double> BodyAttitude(3,3,0.0);
  VNL::Matrix<double> qr(6,1,0.0), ql(6,1,0.0);

  m_FIFOZMPRefPositions.push_back(NewZMPRefPos);
  FirstStageOfControl(LeftFootPosition,RightFootPosition,refCOMPosition,ql,qr,BodyAttitude);
  EvaluateMultiBodyZMP(ql,qr,UpperBodyAngles,BodyAttitude,-1);
  SecondStageOfControl(lqr,lql,finalCOMPosition);
  
  return 1;
}

int ZMPPreviewControlWithMultiBodyZMP::OneGlobalStepOfControl(FootAbsolutePosition &LeftFootPosition,
							      FootAbsolutePosition &RightFootPosition,
							      ZMPPosition &NewZMPRefPos,
							      VNL::Matrix<double> & lqr, 
							      VNL::Matrix<double> & lql, 
							      COMPosition & refandfinalCOMPosition,
							      VNL::Matrix<double> &UpperBodyAngles)
{
  VNL::Matrix<double> BodyAttitude(3,3,0.0);
  VNL::Matrix<double> qr(6,1,0.0), ql(6,1,0.0);

  m_FIFOZMPRefPositions.push_back(NewZMPRefPos);
  FirstStageOfControl(LeftFootPosition,RightFootPosition,refandfinalCOMPosition,ql,qr,BodyAttitude);
  EvaluateMultiBodyZMP(ql,qr,UpperBodyAngles,BodyAttitude,-1);
  SecondStageOfControl(lqr,lql,refandfinalCOMPosition);
  
  return 1;
}

COMPosition ZMPPreviewControlWithMultiBodyZMP::GetLastCOMFromFirstStage()
{
  COMPosition aCOM;
  aCOM = m_FIFOCOMPositions.back();
  return aCOM;
}

int ZMPPreviewControlWithMultiBodyZMP::SecondStageOfControl(VNL::Matrix<double> & lqr, VNL::Matrix<double> & lql, 
							    COMPosition & afCOMPosition)
{
  double Deltazmpx2,Deltazmpy2;
  // Inverse Kinematics variables.

  VNL::Matrix<double> Body_R(3,3), Body_P(3,1);
  VNL::Matrix<double> Foot_R(3,3), Foot_P(3,1);
  float c,s,co,so;
  VNL::Matrix<double> ToTheHip(3,1);
  COMPosition aCOMPosition = m_FIFOCOMPositions[0];
  FootAbsolutePosition LeftFootPosition, RightFootPosition;

  LeftFootPosition = m_FIFOLeftFootPosition[0];
  RightFootPosition = m_FIFORightFootPosition[0];

  // Preview control on delta ZMP.

    
  m_PC->OneIterationOfPreview(m_Deltax,m_Deltay, m_sxDeltazmp, m_syDeltazmp,
			      m_FIFODeltaZMPPositions,0,
			      Deltazmpx2,Deltazmpy2,
			      true);
  
  ODEBUG4(m_Deltax[0][0] << " " << m_Deltay[0][0] , "DebugDataDeltaCOM.txt");
  ODEBUG6("m_TranslationToTheLeft" << m_TranslationToTheLeft ,"DebugData.txt");
  // Correct COM position    but be carefull this is the COM for NL steps behind.
  for(int i=0;i<3;i++)
    {
      aCOMPosition.x[i] += m_Deltax(i,0);
      aCOMPosition.y[i] += m_Deltay(i,0);
	
      // afCOMPosition.x[i] = aCOMPosition.x[i];
      //	afCOMPosition.y[i] = aCOMPosition.y[i];
    }
  ODEBUG4(aCOMPosition.x[0]<< " "
	  << aCOMPosition.y[0] <<  " " 
	  << aCOMPosition.z[0] << " " 
	  << aCOMPosition.theta << " " 
	  << aCOMPosition.omega ,"DebugDataCOMPC2.txt");

  //  afCOMPosition.hip =  m_PC->GetHeightOfCoM() + ToTheHip(3,1) ;//-0.705;
  c = cos(aCOMPosition.theta*M_PI/180.0);
  s = sin(aCOMPosition.theta*M_PI/180.0);
	
  co = cos(aCOMPosition.omega*M_PI/180.0);
  so = sin(aCOMPosition.omega*M_PI/180.0);

  // COM Orientation
  Body_R(0,0) =  m_FinalDesiredCOMPose(0,0) = c*co;       
  Body_R(0,1) =  m_FinalDesiredCOMPose(0,1) = -s;       
  Body_R(0,2) =  m_FinalDesiredCOMPose(0,2) = c*so;
  
  Body_R(1,0) =  m_FinalDesiredCOMPose(1,0) = s*co;       
  Body_R(1,1) =  m_FinalDesiredCOMPose(1,1) =  c;       
  Body_R(1,2) =  m_FinalDesiredCOMPose(1,2) = s*so;

  Body_R(2,0) =  m_FinalDesiredCOMPose(2,0) =  -so;       
  Body_R(2,1) =  m_FinalDesiredCOMPose(2,1)=  0;        
  Body_R(2,2) =  m_FinalDesiredCOMPose(2,2) = co;  

  /*
  // COM Orientation
  Body_R(0,0) = c;       Body_R(0,1) = -s;       Body_R(0,2) = 0;
  Body_R(1,0) = s;       Body_R(1,1) =  c;       Body_R(1,2) = 0;
  Body_R(2,0) = 0;       Body_R(2,1) = 0;        Body_R(2,2) = 1;
  */ 
  // COM position
  ToTheHip = Body_R * m_TranslationToTheLeftHip;
  Body_P(0,0) = aCOMPosition.x[0] + ToTheHip(0,0) ;
  Body_P(1,0) = aCOMPosition.y[0] + ToTheHip(1,0);
  Body_P(2,0) = 0.0; // aCOMPosition.z[0] + ToTheHip(2,0);

  m_FinalDesiredCOMPose(0,3) = aCOMPosition.x[0];
  m_FinalDesiredCOMPose(1,3) = aCOMPosition.y[0];
  m_FinalDesiredCOMPose(2,3) = aCOMPosition.z[0];	
  m_FinalDesiredCOMPose(3,3) = 1.0;

    //  aCOMPosition.hip =   Body_P(2,0);

  // Left Foot.
  c = cos(LeftFootPosition.theta*M_PI/180.0);
  s = sin(LeftFootPosition.theta*M_PI/180.0);
  co = cos(LeftFootPosition.omega*M_PI/180.0);
  so = sin(LeftFootPosition.omega*M_PI/180.0);
 


  //      cout << ZMPRefPositions[lindex].theta << " " << LeftFootPosition[lindex].theta;
  // Orientation
  Foot_R(0,0) = c*co;       Foot_R(0,1) = -s;       Foot_R(0,2) = c*so;
  Foot_R(1,0) = s*co;       Foot_R(1,1) =  c;       Foot_R(1,2) = s*so;
  Foot_R(2,0) = -so;        Foot_R(2,1) = 0;        Foot_R(2,2) = co;
  
  // position
  Foot_P(0,0)=LeftFootPosition.x+ m_AnkleSoilDistance*so;
  Foot_P(1,0)=LeftFootPosition.y;
  Foot_P(2,0)=LeftFootPosition.z+ m_AnkleSoilDistance*co -(aCOMPosition.z[0] + ToTheHip(2,0));
  
  // Compute the inverse kinematics.
  m_IK->ComputeInverseKinematics2ForLegs(Body_R,
					 Body_P,
					 m_Dt,
					 Foot_R,
					 Foot_P,
					 lql);

  // RIGHT FOOT //
  m_Dt(1,0) = -m_Dt(1,0);

  // Right Foot.
  c = cos(RightFootPosition.theta*M_PI/180.0);
  s = sin(RightFootPosition.theta*M_PI/180.0);
  co = cos(RightFootPosition.omega*M_PI/180.0);
  so = sin(RightFootPosition.omega*M_PI/180.0);
  // Orientation
  Foot_R(0,0) = c*co;       Foot_R(0,1) = -s;       Foot_R(0,2) = c*so;
  Foot_R(1,0) = s*co;       Foot_R(1,1) =  c;       Foot_R(1,2) = s*so;
  Foot_R(2,0) =  -so;       Foot_R(2,1) =  0;       Foot_R(2,2) = co;
  
  // position
  Foot_P(0,0)=RightFootPosition.x+m_AnkleSoilDistance*so;
  Foot_P(1,0)=RightFootPosition.y;
  Foot_P(2,0)=RightFootPosition.z+m_AnkleSoilDistance*co - (aCOMPosition.z[0] + ToTheHip(2,0));;
  
  // COM position
  ToTheHip = Body_R * m_TranslationToTheRightHip;
  Body_P(0,0) = aCOMPosition.x[0] + ToTheHip(0,0) ;
  Body_P(1,0) = aCOMPosition.y[0] + ToTheHip(1,0);
  Body_P(2,0) = 0.0;// aCOMPosition.z[0] + ToTheHip(2,0);
  
  m_IK->ComputeInverseKinematics2ForLegs(Body_R,
					 Body_P,
					 m_Dt,
					 Foot_R,
					 Foot_P,
					 lqr);
  m_Dt(1,0) = -m_Dt(1,0);

#ifdef _DEBUG_
  ofstream aof_LastZMP;
  static unsigned char FirstCall=1;
  if (FirstCall)
    {
      aof_LastZMP.open("LastZMP_1.txt",ofstream::out);
      FirstCall = 0;
    }
  else 
    {
      aof_LastZMP.open("LastZMP_1.txt",ofstream::app);
    }

  if (aof_LastZMP.is_open())
    {
      aof_LastZMP << m_FIFOTmpZMPPosition[0].time << " " 
		  << m_FIFOTmpZMPPosition[0].px +  Deltazmpx2 << " " 
		  << m_FIFOTmpZMPPosition[0].py +  Deltazmpy2 << " "
		  << aCOMPosition.x[0] << " " 
		  << aCOMPosition.y[0] << " "
		  << aCOMPosition.z[0] << " "
	          << m_sxDeltazmp << " "
	          << m_syDeltazmp << " "
		  << m_FIFODeltaZMPPositions[0].px << " "
		  << m_FIFODeltaZMPPositions[0].py << " "
		  << m_Deltax(0,0) << " "
		  << m_Deltay(0,0) << " "
		  << m_Deltax(1,0) << " "
		  << m_Deltay(1,0) << " "
		  << Deltazmpx2 << " "
		  << Deltazmpy2 << " "
		  << endl;
      aof_LastZMP.close();
    }
  m_FIFOTmpZMPPosition.pop_front();
#endif

  m_FIFODeltaZMPPositions.pop_front();
  m_FIFOCOMPositions.pop_front();
  m_FIFOLeftFootPosition.pop_front();
  m_FIFORightFootPosition.pop_front();

  afCOMPosition = aCOMPosition;
  return 1;
}

int ZMPPreviewControlWithMultiBodyZMP::FirstStageOfControl( FootAbsolutePosition &LeftFootPosition,
							    FootAbsolutePosition &RightFootPosition,
							    COMPosition & afCOMPosition,
							    VNL::Matrix<double> &ql,
							    VNL::Matrix<double> &qr,
							    VNL::Matrix<double> &BodyAttitude)

{

	
  double zmpx2, zmpy2;
  // Inverse Kinematics variables.
  matrix3d Body_Rm3d;
  VNL::Matrix<double> Body_R(3,3,0.0), Body_P(3,1,0.0);
  VNL::Matrix<double> Foot_R(3,3,0.0), Foot_P(3,1,0.0);
  float c,s,co,so;

  /* int LINKSFORRARM[7] = { 16, 17, 18, 19, 20, 21, 22};
     int LINKSFORLARM[7] = { 23, 24, 25, 26, 27, 28, 29};
     int LINKSFORUPPERBODY[2] = { 12, 13};
     int LINKSFORHEAD[2] = { 14, 15};
     int LINKSFORRHAND[5] = { 30, 31, 32, 33, 34};
     int LINKSFORLHAND[5] = { 35, 36, 37, 38, 39};	
  */
 
  VNL::Matrix<double> ToTheHip(3,1);


#ifdef _DEBUG_
  ofstream aof_CartZMP;
  static unsigned char FirstCall=1;
  if (FirstCall)
    {
      aof_CartZMP.open("CartZMP_1.dat",ofstream::out);
    }
  else 
    {
      aof_CartZMP.open("CartZMP_1.dat",ofstream::app);
    }

  ofstream aof_CartCOM;
  if (FirstCall)
    {
      aof_CartCOM.open("CartCOM_1.dat",ofstream::out);
      aof_CartCOM << m_FIFOZMPRefPositions[0].time << " " << m_PC1x(0,0)<< " " << m_PC1y(0,0) << endl;
    }
  else 
    {
      aof_CartCOM.open("CartCOM_1.dat",ofstream::app);
    }

  ofstream aof_IK1;
  if (FirstCall)
    {
      aof_IK1.open("IK_1.dat",ofstream::out);
    }
  else 
    {
      aof_IK1.open("IK_1.dat",ofstream::app);
    }

  ofstream aof_Momentums;
  if (FirstCall)
    {
      aof_Momentums.open("Momentums_1.dat",ofstream::out);
    }
  else 
    {
      aof_Momentums.open("Momentums_1.dat",ofstream::app);
    }

  ofstream aof_WCOMdMom;
  if (FirstCall)
    {
      aof_WCOMdMom.open("WCOMdMom_1.dat",ofstream::out);
    }
  else 
    {
      aof_WCOMdMom.open("WCOMdMom_1.dat",ofstream::app);
    }

  if (FirstCall)
    FirstCall = 0;
  
#endif

  COMPosition acomp;

  if (m_ZMPCoMTrajectoryAlgorithm==ZMPCOM_TRAJECTORY_KAJITA)
    {
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
  
    }
  else  if (m_ZMPCoMTrajectoryAlgorithm==ZMPCOM_TRAJECTORY_WIEBER) 
    {
      for(unsigned j=0;j<3;j++)
	acomp.x[j] = afCOMPosition.x[j];
      
      for(unsigned j=0;j<3;j++)
	acomp.y[j] = afCOMPosition.y[j];
      
    }
  
      
  ODEBUG6("First Stage: CoM "<< m_PC1x[0][0] << " " << m_PC1y[0][0] , "DebugData.txt");
#ifdef _DEBUG_
  if (aof_CartZMP.is_open())
    {
      aof_CartZMP << m_FIFOZMPRefPositions[0].time << " " << zmpx2 << " " << zmpy2 << endl;
    }

  if (aof_CartCOM.is_open())
    {
      aof_CartCOM << m_FIFOZMPRefPositions[0].time << " " << m_PC1x(0,0)<< " " << m_PC1y(0,0) << endl;
    }
      
#endif




  {
    ZMPPosition lZMPPos = m_FIFOZMPRefPositions[0];
    ODEBUG4(acomp.x[0]<< " "
	    << acomp.y[0] <<  " " 
	    << acomp.z[0] << " "
	    << zmpx2 << " " 
	    << zmpy2 << " " 
	    << lZMPPos.px << " " 
	    << lZMPPos.py << " "
	    << acomp.x[1]<< " "
	    << acomp.y[1] <<  " " 
	    << acomp.z[1] << " "
	    << acomp.x[2]<< " "
	    << acomp.y[2] <<  " " 
	    << acomp.z[2] << " "

	    ,"DebugDataCOMPC1.txt");
  }

  acomp.omega = afCOMPosition.omega;
  acomp.theta = afCOMPosition.theta;

 // c = cos((m_FIFOZMPRefPositions[0].theta+afCOMPosition.theta)*M_PI/180.0);
  //s = sin((m_FIFOZMPRefPositions[0].theta+afCOMPosition.theta)*M_PI/180.0);

  c = cos((afCOMPosition.theta)*M_PI/180.0);
  s = sin((afCOMPosition.theta)*M_PI/180.0);

  co = cos(afCOMPosition.omega*M_PI/180.0);
  so = sin(afCOMPosition.omega*M_PI/180.0);

  // COM Orientation
  Body_R(0,0) = Body_Rm3d.m[0] = c*co;       Body_R(0,1) = Body_Rm3d.m[1] = -s;       Body_R(0,2) = Body_Rm3d.m[2] = c*so;
  Body_R(1,0) = Body_Rm3d.m[3] = s*co;       Body_R(1,1) = Body_Rm3d.m[4] =  c;       Body_R(1,2) = Body_Rm3d.m[5] = s*so;
  Body_R(2,0) = Body_Rm3d.m[6] = -so;       Body_R(2,1) = Body_Rm3d.m[7] = 0;        Body_R(2,2) = Body_Rm3d.m[8] = co;
  
  /* // COM Orientation
     Body_R(0,0) = Body_Rm3d.m[0] = c;       Body_R(0,1) = Body_Rm3d.m[1] = -s;       Body_R(0,2) = Body_Rm3d.m[2] = 0;
     Body_R(1,0) = Body_Rm3d.m[3] = s;       Body_R(1,1) = Body_Rm3d.m[4] =  c;       Body_R(1,2) = Body_Rm3d.m[5] = 0;
     Body_R(2,0) = Body_Rm3d.m[6] = 0;       Body_R(2,1) = Body_Rm3d.m[7] = 0;        Body_R(2,2) = Body_Rm3d.m[8] = 1;
  */
  // HIP position
  ToTheHip = Body_R * m_TranslationToTheLeftHip;
  Body_P(0,0) = acomp.x[0] + ToTheHip(0,0) ;
  Body_P(1,0) = acomp.y[0] + ToTheHip(1,0);
  Body_P(2,0) = 0.0; // afCOMPosition.z[0] + ToTheHip(2,0);

  acomp.hip=Body_P(2,0);

  // Left Foot.
  c = cos(LeftFootPosition.theta*M_PI/180.0);
  s = sin(LeftFootPosition.theta*M_PI/180.0);
  co = cos(LeftFootPosition.omega*M_PI/180.0);
  so = sin(LeftFootPosition.omega*M_PI/180.0);
  
  //      cout << ZMPRefPositions[lindex].theta << " " << LeftFootPosition[lindex].theta;
  // Orientation
  Foot_R(0,0) = c*co;       Foot_R(0,1) = -s;       Foot_R(0,2) = c*so;
  Foot_R(1,0) = s*co;       Foot_R(1,1) =  c;       Foot_R(1,2) = s*so;
  Foot_R(2,0) = -so;        Foot_R(2,1) = 0;        Foot_R(2,2) = co;

  // position
  Foot_P(0,0)=LeftFootPosition.x+m_AnkleSoilDistance*so;
  Foot_P(1,0)=LeftFootPosition.y;
  Foot_P(2,0)=LeftFootPosition.z+m_AnkleSoilDistance*co - (afCOMPosition.z[0] + ToTheHip(2,0));

  ODEBUG4( Body_P(0,0) <<  " " << Body_P(1,0) <<  " " << Body_P(2,0)  << " " 
	   << Foot_P(0,0) <<  " " << Foot_P(1,0) <<  " " << Foot_P(2,0)  << " "
	   << acomp.x[0] << " " << acomp.y[0] << " " << ToTheHip(0,0) << " " << ToTheHip(1,0)<< " " << ToTheHip(2,0)<< " " 
	   << so << " " << co << " " << LeftFootPosition.omega,
	   "DebugDataIKL.dat");
  
  // Compute the inverse kinematics.
  m_IK->ComputeInverseKinematics2ForLegs(Body_R,
					 Body_P,
					 m_Dt,
					 Foot_R,
					 Foot_P,
					 ql);

  ODEBUG4( ql(0,0)*180/M_PI << " "  <<  
	   ql(1,0)*180/M_PI << " " <<  
	   ql(2,0)*180/M_PI << " " <<  
	   ql(3,0)*180/M_PI << "  " <<  
	   ql(4,0)*180/M_PI << " " <<  
	   ql(5,0)*180/M_PI, "DebugDataql.txt" );

  m_Dt(1,0) = -m_Dt(1,0);


  // Right Foot.
  c = cos(RightFootPosition.theta*M_PI/180.0);
  s = sin(RightFootPosition.theta*M_PI/180.0);
  co = cos(RightFootPosition.omega*M_PI/180.0);
  so = sin(RightFootPosition.omega*M_PI/180.0);

  // Orientation
  Foot_R(0,0) = c*co;       Foot_R(0,1) = -s;       Foot_R(0,2) = c*so;
  Foot_R(1,0) = s*co;       Foot_R(1,1) =  c;       Foot_R(1,2) = s*so;
  Foot_R(2,0) =  -so;       Foot_R(2,1) =  0;       Foot_R(2,2) = co;
  
  // position
  Foot_P(0,0)=RightFootPosition.x+m_AnkleSoilDistance*so;
  Foot_P(1,0)=RightFootPosition.y;
  Foot_P(2,0)=RightFootPosition.z+m_AnkleSoilDistance*co - (afCOMPosition.z[0] + ToTheHip(2,0));;
  
  // COM position
  ToTheHip = Body_R * m_TranslationToTheRightHip;
  Body_P(0,0)=acomp.x[0] + ToTheHip(0,0);
  Body_P(1,0)=acomp.y[0] + ToTheHip(1,0);
  Body_P(2,0)=0.0; //afCOMPosition.z[0] + ToTheHip(2,0);

  ODEBUG4( Body_P(0,0) <<  " " << Body_P(1,0) <<  " " << Body_P(2,0)  << " " 
	   << Foot_P(0,0) <<  " " << Foot_P(1,0) <<  " " << Foot_P(2,0)  << " "
	   << acomp.x[0] << " " << acomp.y[0] << " " << ToTheHip(0,0) << " " << ToTheHip(1,0)<< " " 
	   << so << " " << co ,
	   "DebugDataIKR.dat");
  
  m_IK->ComputeInverseKinematics2ForLegs(Body_R,
					 Body_P,
					 m_Dt,
					 Foot_R,
					 Foot_P,
					 qr);

  ODEBUG4( qr(0,0)*180/M_PI << " "  <<  
	   qr(1,0)*180/M_PI << " " <<  
	   qr(2,0)*180/M_PI << " " <<  
	   qr(3,0)*180/M_PI << "  " <<  
	   qr(4,0)*180/M_PI << " " <<  
	   qr(5,0)*180/M_PI, "DebugDataqr.txt" );

  BodyAttitude = Body_R;

  // Fill the COM positions FIFO.
 // acomp.theta = m_FIFOZMPRefPositions[0].theta+afCOMPosition.theta;

 acomp.theta = afCOMPosition.theta;

  
 // Update the value of the input parameter.
 afCOMPosition = acomp;

 // Update of the FIFOs
 m_FIFOCOMPositions.push_back(acomp);
 m_FIFORightFootPosition.push_back(RightFootPosition);
 m_FIFOLeftFootPosition.push_back(LeftFootPosition);

#ifdef _DEBUG_
  m_FIFOTmpZMPPosition.push_back(m_FIFOZMPRefPositions[0]);
#endif
  m_FIFOZMPRefPositions.pop_front();
  
  return 1;
}

int ZMPPreviewControlWithMultiBodyZMP::EvaluateMultiBodyZMP(VNL::Matrix<double> &ql,
							    VNL::Matrix<double> &qr,
							    VNL::Matrix<double> &UpperBodyAngles,
							    VNL::Matrix<double> &BodyAttitude,
							    int StartingIteration)
{
  // Forward Kinematics variables.
  int LINKSFORRLEG[6] = { 0, 1, 2, 3,  4, 5};
  int LINKSFORLLEG[6] = { 6, 7, 8, 9, 10, 11};

  // For computing ZMP multi body
  double ZMPz=0.0;

  // Momentum vectors.
  vector3d P(0,0,0),L(0,0,0),  dP(0,0,0), dL(0,0,0);

  VNL::Matrix<double> dql(6,1,0.0), dqr(6,1,0.0);
  VNL::Matrix<double> dUpperBodyAngles(28,1,0.0);
  
  // Update the Dynamic multi body model 
  for(unsigned int j=0;j<6;j++)
    {
      m_DMB->Setq(LINKSFORLLEG[j],ql(j,0));
      if (!m_StartingNewSequence)
	dql(j,0) = (ql(j,0)-m_prev_ql(j,0))/m_SamplingPeriod;
      else
	dql(j,0) = 0;

      m_DMB->Setdq(LINKSFORLLEG[j],dql(j,0));
      m_prev_ql(j,0) = ql(j,0);
#ifdef _DEBUG_
      if (aof_IK1.is_open())
	{
	  aof_IK1 << ql(j,0)<< " " << dql(j,0) << " ";
	}
#endif
    }
  ODEBUG4( dql(0,0)<< " " <<  dql(1,0)  << " " << dql(2,0) << " " 
	   << dql(3,0) << "  " << dql(4,0) << " " << dql(5,0), "DebugDatadql.txt" );
  
  // Update the Dynamic multi body model 
  for(unsigned int j=0;j<6;j++)
    {
      m_DMB->Setq(LINKSFORRLEG[j],qr(j,0));
      if (!m_StartingNewSequence)
	dqr(j,0) = (qr(j,0)-m_prev_qr(j,0))/m_SamplingPeriod;
      else
	dqr(j,0) = 0;
      m_DMB->Setdq(LINKSFORRLEG[j],dqr(j,0));
      m_prev_qr(j,0) = qr(j,0);
#ifdef _DEBUG_
      if (aof_IK1.is_open())
	{
	  aof_IK1 << qr(j,0)<< " " << dqr(j,0) << " ";
	}
#endif

    }
  ODEBUG4( dqr(0,0)<< " " <<  dqr(1,0)  << " " << dqr(2,0) << " " 
	   << dqr(3,0) << "  " << dqr(4,0) << " " << dqr(5,0), "DebugDatadqr.txt" );
  
  

  // Update the Dynamic multi body model with upperbody motion
  for(int j=0;j<28;j++)
    {
      m_DMB->Setq(12+j,UpperBodyAngles(j,0));
      ODEBUG6( "q"<< j << " : "<< UpperBodyAngles(j,0), "DebugData.txt" );  
      if (!m_StartingNewSequence)
	dUpperBodyAngles(j,0) = (UpperBodyAngles(j,0)- m_prev_UpperBodyAngles(j,0))/m_SamplingPeriod;
      else
	dUpperBodyAngles(j,0) = 0;
      m_DMB->Setdq(12+j,dUpperBodyAngles(j,0));

      ODEBUG6( "dq"<< j << " : " << dUpperBodyAngles(j,0), "DebugData.txt" );  
      m_prev_UpperBodyAngles(j,0) = UpperBodyAngles(j,0);
#ifdef _DEBUG_
      if (aof_IK1.is_open())
	{
	  aof_IK1 << UpperBodyAngles(j,0)<< " " 
		  << dUpperBodyAngles(j,0) << " ";
	}
#endif
    }
  ODEBUG4( UpperBodyAngles(0,0) << " " <<
	   UpperBodyAngles(1,0) << " " <<
	   UpperBodyAngles(2,0) << " " <<
	   UpperBodyAngles(3,0) << " " <<
	   UpperBodyAngles(4,0) << " " <<
	   UpperBodyAngles(5,0) << " " <<
	   UpperBodyAngles(6,0) << " " <<
	   UpperBodyAngles(7,0) << " " <<
	   UpperBodyAngles(8,0) << " " <<
	   UpperBodyAngles(9,0) << " " <<
	   UpperBodyAngles(10,0) << " " <<
	   UpperBodyAngles(11,0) << " " <<
	   UpperBodyAngles(12,0) << " " <<
	   UpperBodyAngles(13,0) << " " <<
	   UpperBodyAngles(14,0) << " " <<
	   UpperBodyAngles(15,0) << " " <<
	   UpperBodyAngles(16,0) << " " <<
	   UpperBodyAngles(17,0) << " " <<
	   UpperBodyAngles(18,0) << " " <<
	   UpperBodyAngles(19,0) << " " <<
	   UpperBodyAngles(20,0) << " " <<
	   UpperBodyAngles(21,0) << " " <<
	   UpperBodyAngles(22,0) << " " <<
	   UpperBodyAngles(23,0) << " " <<
	   UpperBodyAngles(24,0) << " " <<
	   UpperBodyAngles(25,0) << " " <<
	   UpperBodyAngles(26,0) << " " <<
	   UpperBodyAngles(27,0) << " " 
	   , "DebugDataUB.txt" );
  ODEBUG4( dUpperBodyAngles(0,0) << " " <<
	   dUpperBodyAngles(1,0) << " " <<
	   dUpperBodyAngles(2,0) << " " <<
	   dUpperBodyAngles(3,0) << " " <<
	   dUpperBodyAngles(4,0) << " " <<
	   dUpperBodyAngles(5,0) << " " <<
	   dUpperBodyAngles(6,0) << " " <<
	   dUpperBodyAngles(7,0) << " " <<
	   dUpperBodyAngles(8,0) << " " <<
	   dUpperBodyAngles(9,0) << " " <<
	   dUpperBodyAngles(10,0) << " " <<
	   dUpperBodyAngles(11,0) << " " <<
	   dUpperBodyAngles(12,0) << " " <<
	   dUpperBodyAngles(13,0) << " " <<
	   dUpperBodyAngles(14,0) << " " <<
	   dUpperBodyAngles(15,0) << " " <<
	   dUpperBodyAngles(16,0) << " " <<
	   dUpperBodyAngles(17,0) << " " <<
	   dUpperBodyAngles(18,0) << " " <<
	   dUpperBodyAngles(19,0) << " " <<
	   dUpperBodyAngles(20,0) << " " <<
	   dUpperBodyAngles(21,0) << " " <<
	   dUpperBodyAngles(22,0) << " " <<
	   dUpperBodyAngles(23,0) << " " <<
	   dUpperBodyAngles(24,0) << " " <<
	   dUpperBodyAngles(25,0) << " " <<
	   dUpperBodyAngles(26,0) << " " <<
	   dUpperBodyAngles(27,0) << " " 
	   , "DebugDatadUB.txt" );

  
  m_Dt(1,0) = -m_Dt(1,0);

  // Compute the waist position in a similar manner than for the hip.
  VNL::Vector<double> AbsoluteWaistPosition(3);

  AbsoluteWaistPosition = BodyAttitude* m_DiffBetweenComAndWaist;
  AbsoluteWaistPosition[0] += m_PC1x(0,0);
  AbsoluteWaistPosition[1] += m_PC1y(0,0);

  vector3d WaistPosition;
  vector3d WaistVelocity;
	
  COMPosition afCOMPosition = m_FIFOCOMPositions.back();
  WaistVelocity.x = m_PC1x(1,0);
  WaistVelocity.y = m_PC1y(1,0);
  WaistVelocity.z = afCOMPosition.z[1];
  
#if 0
  WaistPosition.x = m_PC1x(0,0);
  WaistPosition.y = m_PC1y(0,0);
#else
  WaistPosition.x = AbsoluteWaistPosition[0];
  WaistPosition.y = AbsoluteWaistPosition[1];
#endif
  VNL::Matrix<double> ToTheHip(3,1);
  ToTheHip = BodyAttitude * m_TranslationToTheLeftHip;
  WaistPosition.z = afCOMPosition.z[0] + ToTheHip(2,0) - 0.705; 
  // Compensate for the static translation, not the WAIST position

  // but it is the body position which start on the ground.
  
  ODEBUG4( WaistPosition.x << " " << WaistPosition.y << " " << WaistPosition.z << " " << 
	   WaistVelocity.x << " " << WaistVelocity.y << " " << WaistVelocity.z << " "
	   << m_DiffBetweenComAndWaist[0] << " "
	   << m_DiffBetweenComAndWaist[1] << " "
	   << m_DiffBetweenComAndWaist[2] << " " 
	   << m_PC1x(1,0) << " " << m_PC1y(1,0), "DebugDataWP.txt");

  // COM Orientation
  matrix3d Body_Rm3d;
  int k=0;
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      Body_Rm3d.m[k++] = BodyAttitude(i,j);

  m_DMB->ForwardVelocity(WaistPosition,Body_Rm3d, WaistVelocity);

  // Get angular and linear momentum.
  if ((StartingIteration>0) || 
      (StartingIteration==-1))
    {
      m_DMB->GetPandL(P,L);

      if ((StartingIteration>1) ||
	  (StartingIteration==-1))
	{
	  if ((StartingIteration==1) ||
	      (StartingIteration==2))
	    {
	      // For the first iteration, we assume that the previous values were the same.
	      m_prev_P = P;
	      m_prev_L = L;
	    } 
	  // Approximate first order derivative of linear and angular momentum.
	  dP = (P - m_prev_P)/m_SamplingPeriod;
	  dL = (L - m_prev_L)/m_SamplingPeriod;
	}

    }

      
  double ZMPmultibody[2];


  ODEBUG4( "dP : " << dP << " m_prev_P " << m_prev_P << " dL: " << dL<< " m_prev_L " << m_prev_L << 
	   " P : " << P << " L : " << L , "DebugData.txt");

  // Compute ZMP
  m_DMB->CalculateZMP(ZMPmultibody[0],ZMPmultibody[1],
		      dP, dL, ZMPz);
  
#ifdef _DEBUG_
  aof_WCOMdMom 
    << dP.x << " " << dP.y << " " << dP.z  << " " 
    << dL.x << " " << dL.y << " " << dL.z  << " " 
    << ZMPz << " " 
    << m_DMB->positionCoMPondere.x << " " 
    << m_DMB->positionCoMPondere.y << " " 
    << m_DMB->positionCoMPondere.z << " "
    << m_DMB->getMasse() << endl;
    

  aof_Momentums << m_FIFOZMPRefPositions[0].time << " "
		<< P.x << " " << P.y << " " << P.z << " "
		<< L.x << " " << L.y << " " << L.z << endl;
  ofstream aof_ZMPMB;
  static unsigned char FirstCallZMPMB=1;
  if (FirstCallZMPMB)
    {
      aof_ZMPMB.open("ZMPMB.dat",ofstream::out);
      FirstCallZMPMB = 0;
    }
  else 
    {
      aof_ZMPMB.open("ZMPMB.dat",ofstream::app);
    }
  if (aof_ZMPMB.is_open())
    {
      aof_ZMPMB << m_FIFOZMPRefPositions[0].time << " " 
	        << ZMPmultibody[0] << " " << ZMPmultibody[1] <<  " "
	        << m_FIFOZMPRefPositions[0].px << " " 
	        << m_FIFOZMPRefPositions[0].py << " " 
		<< WaistPosition.x << " " << WaistPosition.y << " " << WaistPosition.z << " "  <<endl;
      aof_ZMPMB.close();
    }
  ofstream aof_ZMPPreviewControl;
  if (FirstCallZMPMB)
    {
      aof_ZMPPreviewControl.open("PosPreviewControlFS.dat",ofstream::out);
      FirstCallZMPMB = 0;
    }
  else 
    {
      aof_ZMPPreviewControl.open("PosPreviewControlFS.dat",ofstream::app);
    }
  for(int i=0;i<m_DMB->NbOfLinks();i++)
    {
      vector3d avec = m_DMB->Getp(i);
      aof_ZMPPreviewControl << avec.x << " " <<  avec.y << " "  << avec.z << " " ;
    }
  aof_ZMPPreviewControl << endl;
  aof_ZMPPreviewControl.close();      
  
#endif

  ODEBUG4( StartingIteration << " " << ZMPmultibody[0] << " " << ZMPmultibody[1] << " " 
	   << P.x << " " << P.y << " " << P.z << " " 
	   << L.x << " " << L.y << " " << P.z << " " 
	   << dP.x << " " << dP.y << " " << dP.z <<" " 
	   << dL.x << " " << dL.y << " " << dL.z << " " << dL ,"DebugDataZMPMB1.txt");
  //COMFromMB = m_DMB->getPositionCoM();
  
  m_prev_P = P;
  m_prev_L = L;
  
  // Fill the delta ZMP FIFO
  ZMPPosition aZMPpos;
  aZMPpos.px = m_FIFOZMPRefPositions[0].px - ZMPmultibody[0];
  aZMPpos.py = m_FIFOZMPRefPositions[0].py - ZMPmultibody[1];
  aZMPpos.time = m_FIFOZMPRefPositions[0].time;
  ODEBUG4(aZMPpos.px << " " << aZMPpos.py << " " 
	  << m_FIFOZMPRefPositions[0].px << " " << m_FIFOZMPRefPositions[0].py  << " " 
	  << ZMPmultibody[0] << " " << ZMPmultibody[1] , "DebugDataDiffZMP.txt");
  m_FIFODeltaZMPPositions.push_back(aZMPpos);

  m_StartingNewSequence = false;

#ifdef _DEBUG_
  if (aof_WCOMdMom.is_open())
    aof_WCOMdMom.close();

  if (aof_Momentums.is_open())
    aof_Momentums.close();

  if (aof_CartZMP.is_open())
    {
      aof_CartZMP.close();
    }

  if (aof_IK1.is_open())
    {
      aof_IK1<< endl;
      aof_IK1.close();
    }
#endif 
  return 1;
}


int ZMPPreviewControlWithMultiBodyZMP::EvaluateStartingCoM(VNL::Matrix<double> &BodyAngles,
							   VNL::Vector<double> &aStartingCOMPosition,
							   FootAbsolutePosition & InitLeftFootPosition,
							   FootAbsolutePosition & InitRightFootPosition)
{
  
  EvaluateCOM(BodyAngles,
	      0.0,0.0,
	      m_StartingCOMPosition,
	      InitLeftFootPosition,
	      InitRightFootPosition);
  
  aStartingCOMPosition[0] = m_StartingCOMPosition.x;
  aStartingCOMPosition[1] = m_StartingCOMPosition.y;
  aStartingCOMPosition[2] = m_StartingCOMPosition.z;
  
  return 0;
}

int ZMPPreviewControlWithMultiBodyZMP::EvaluateStartingCoM(VNL::Matrix<double> &BodyAngles,
							   VNL::Vector<double> &aStartingCOMPosition,
							   VNL::Vector<double> &aWaistPosition,
							   FootAbsolutePosition & InitLeftFootPosition,
							   FootAbsolutePosition & InitRightFootPosition)
{
  vector3d WaistPosition;
  EvaluateCOM(BodyAngles,
	      0.0,0.0,
	      m_StartingCOMPosition,
	      WaistPosition,
	      InitLeftFootPosition,
	      InitRightFootPosition);
  
  aWaistPosition[0] = WaistPosition.x;
  aWaistPosition[1] = WaistPosition.y;
  aWaistPosition[2] = WaistPosition.z;
  aStartingCOMPosition[0] = m_StartingCOMPosition.x;
  aStartingCOMPosition[1] = m_StartingCOMPosition.y;
  aStartingCOMPosition[2] = m_StartingCOMPosition.z;
  
  return 0;
}

int ZMPPreviewControlWithMultiBodyZMP::Setup(deque<ZMPPosition> &ZMPRefPositions,
					     deque<COMPosition> &COMPositions,
					     deque<FootAbsolutePosition> &LeftFootPositions,
					     deque<FootAbsolutePosition> &RightFootPositions,
					     VNL::Matrix<double> &BodyAngles)
{
  SetupFirstPhase(ZMPRefPositions,
		  COMPositions,
		  LeftFootPositions,
		  RightFootPositions,
		  BodyAngles);

  for(unsigned int i=0;i<m_NL;i++)
    SetupIterativePhase(ZMPRefPositions,
			COMPositions,
			LeftFootPositions,
			RightFootPositions,
			BodyAngles,i);
  return 0;
}

int ZMPPreviewControlWithMultiBodyZMP::SetupFirstPhase(deque<ZMPPosition> &ZMPRefPositions,
						       deque<COMPosition> &COMPositions,
						       deque<FootAbsolutePosition> &LeftFootPositions,
						       deque<FootAbsolutePosition> &RightFootPositions,
						       VNL::Matrix<double> &BodyAngles)
{
  ODEBUG6("Beginning of Setup 0 ","DebugData.txt");
  ODEBUG("Setup");
  double zmpx2, zmpy2;
  
  
  m_sxzmp =0.0;
  m_syzmp =0.0;
  m_sxDeltazmp = 0.0;
  m_syDeltazmp = 0.0;

  // Initialization of previous momentum
  m_prev_P.x = m_prev_P.y = m_prev_P.z =
    m_prev_L.x = m_prev_L.y = m_prev_L.z = 0.0;

  m_StartingNewSequence = true;

  // Fill the Fifo 
  m_FIFOZMPRefPositions.resize(m_NL);
  for(unsigned int i=0;i<m_NL;i++)
    {
      m_FIFOZMPRefPositions[i] = ZMPRefPositions[i];
    }
  
  ODEBUG6("After EvaluateCOM","DebugData.txt");

  VNL::Matrix<double> UpperBodyAngles(28,1,0.0);
  for(int i=0;i<28;i++)
    UpperBodyAngles[i][0] = BodyAngles[12+i][0];

  ODEBUG6("Beginning of Setup 1 ","DebugData.txt");	
  m_PC1x(0,0)= m_StartingCOMPosition.x;
  ODEBUG("COMPC1 init X: " <<  m_PC1x(0,0));
  //m_PC1x(0,0) = 0.0;
  m_PC1x(1,0)= 0.0;
  m_PC1x(2,0)= 0.0;
  
  m_PC1y(0,0)= m_StartingCOMPosition.y;
  ODEBUG("COMPC1 init Y: " <<  m_PC1y(0,0));
  //m_PC1y(0,0) = 0.0;
  m_PC1y(1,0)= 0;    
  m_PC1y(2,0)= 0;

  m_Deltax(0,0)= 0.0; //-StartingCOMPosition.x;    
  m_Deltax(1,0)= 0;    m_Deltax(2,0)= 0;
  m_Deltay(0,0)= 0.0; //-StartingCOMPosition.y;    
  m_Deltay(1,0)= 0;    m_Deltay(2,0)= 0;
   
  //  m_sxzmp=-StartingCOMPosition.x;m_syzmp=-StartingCOMPosition.y;
  //  zmpx2=StartingCOMPosition.x;zmpy2=StartingCOMPosition.y;
  m_sxzmp = 0.0; m_syzmp =0.0;
  zmpx2 = 0.0; zmpy2 = 0.0;
      
  m_FIFODeltaZMPPositions.clear();
  m_FIFOCOMPositions.clear();
  m_FIFOLeftFootPosition.clear();
  m_FIFORightFootPosition.clear();
#ifdef _DEBUG_
  m_FIFOTmpZMPPosition.clear();
#endif

  return 0;
}


int ZMPPreviewControlWithMultiBodyZMP::SetupIterativePhase(deque<ZMPPosition> &ZMPRefPositions,
							   deque<COMPosition> &COMPositions,
							   deque<FootAbsolutePosition> &LeftFootPositions,
							   deque<FootAbsolutePosition> &RightFootPositions,
							   VNL::Matrix<double> &BodyAngles,
							   int localindex)
{
  VNL::Matrix<double> UpperBodyAngles(28,1,0.0);
  for(int i=0;i<28;i++)
    UpperBodyAngles[i][0] = BodyAngles[12+i][0];
  
  VNL::Matrix<double> BodyAttitude(3,3,0.0);
  VNL::Matrix<double> qr(6,1,0.0), ql(6,1,0.0);

  FirstStageOfControl(LeftFootPositions[localindex],RightFootPositions[localindex],COMPositions[localindex],
		      ql,qr,BodyAttitude);
  EvaluateMultiBodyZMP(ql,qr,UpperBodyAngles,BodyAttitude,localindex);

  m_FIFOZMPRefPositions.push_back(ZMPRefPositions[localindex+1+m_NL]);
  return 0;
}
void ZMPPreviewControlWithMultiBodyZMP::CreateExtraCOMBuffer(deque<COMPosition> &m_ExtraCOMBuffer,deque<ZMPPosition> &m_ExtraZMPBuffer, deque<ZMPPosition> &m_ExtraZMPRefBuffer)

{
  deque<ZMPPosition> aFIFOZMPRefPositions;
  VNL::Matrix<double> aPC1x,aPC1y; 
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
	
#ifdef _DEBUG_
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

      m_ExtraCOMBuffer[i].theta = m_ExtraZMPRefBuffer[i].theta;
		
      aFIFOZMPRefPositions.pop_front();
	
		
#ifdef _DEBUG_
      if (aof_ExtraCOM.is_open())
	{
	  aof_ExtraCOM << m_ExtraZMPRefBuffer[i].time << " " << m_ExtraZMPRefBuffer[i].px << " "<< aPC1x(0,0)<< " " << aPC1y(0,0) << endl;
	}
#endif
		
    }
  ODEBUG("ik ben hier b" << aFIFOZMPRefPositions.size() );
  ODEBUG("ik ben hier c" << m_ExtraZMPRefBuffer.size());	

#ifdef _DEBUG_
  if (aof_ExtraCOM.is_open())
    {
      aof_ExtraCOM.close();
    }
#endif 
  	
}


int ZMPPreviewControlWithMultiBodyZMP::EvaluateCOM( VNL::Matrix<double> BodyAngles,
						    double omega, double theta,
						    vector3d &lCOMPosition,
						    FootAbsolutePosition & InitLeftFootPosition,
						    FootAbsolutePosition & InitRightFootPosition)
{
  vector3d lWaistPosition;
  return EvaluateCOM(BodyAngles,
	      omega,theta,
	      lCOMPosition, lWaistPosition,
	      InitLeftFootPosition, InitRightFootPosition);
  
}
int ZMPPreviewControlWithMultiBodyZMP::EvaluateCOM( VNL::Matrix<double> BodyAngles,
						    double omega, double theta,
						    vector3d &lCOMPosition,
						    vector3d &WaistPosition,
						    FootAbsolutePosition & InitLeftFootPosition,
						    FootAbsolutePosition & InitRightFootPosition)

{
  matrix3d Body_Rm3d;

  memset(&InitLeftFootPosition,1,sizeof(FootAbsolutePosition));
  memset(&InitRightFootPosition,1,sizeof(FootAbsolutePosition));

  // Update the Dynamic multi body model 
  for(unsigned int j=0;j<BodyAngles.Rows();j++)
    {
      ODEBUG4( "EvalueCOM2 :" <<  j << " " << (BodyAngles[j][0]*180/M_PI), "DebugDataStartingCOM.dat");
      m_DMB->Setq(j,BodyAngles[j][0]);
      m_DMB->Setdq(j,0.0);
    }

  // Update the velocity.
  vector3d RootPosition;
  vector3d RootVelocity;
  RootVelocity.x = 0.0;
  RootVelocity.y = 0.0;
  RootVelocity.z = 0.0;
  
  RootPosition.x = 0.0;
  RootPosition.y = 0.0;
  RootPosition.z = -0.705; 

  double c,s,co,so;
  ODEBUG4( "omega: " << omega << " theta: " << theta ,"DebugDataStartingCOM.dat"); 

  c = cos(theta*M_PI/180.0);
  s = sin(theta*M_PI/180.0);

  co = cos(omega*M_PI/180.0);
  so = sin(omega*M_PI/180.0);
  
  // COM Orientation
  Body_Rm3d.m[0] = c*co;        Body_Rm3d.m[1] = -s;      Body_Rm3d.m[2] = c*so;
  Body_Rm3d.m[3] = s*co;        Body_Rm3d.m[4] =  c;      Body_Rm3d.m[5] = s*so;
  Body_Rm3d.m[6] = -so;         Body_Rm3d.m[7] = 0;       Body_Rm3d.m[8] = co;

  // Compensate for the static translation, not the WAIST position
  // but it is the body position which start on the ground.
  
  m_DMB->ForwardVelocity(RootPosition,Body_Rm3d, RootVelocity);
  
  ODEBUG4("Root Position:" << RootPosition.x << " "
	 << RootPosition.y << " "
	  << RootPosition.z , "DebugDataStartingCOM.dat");
  vector3d lFootPosition;
  lFootPosition  = m_DMB->Getp(5);  

  WaistPosition.x = 0.0;
  WaistPosition.y = 0.0;
  WaistPosition.z = -lFootPosition.z+m_AnkleSoilDistance;

  InitRightFootPosition.x = lFootPosition.x;
  InitRightFootPosition.y = lFootPosition.y;
  InitRightFootPosition.z = 0.0;


  ODEBUG( "Right Foot Position: " << lFootPosition.x << " "
       << lFootPosition.y << " "
       << lFootPosition.z );
  vector3d LeftHip;
  LeftHip = m_DMB->Getp(6);
  ODEBUG( "Left Hip Position: " << LeftHip.x << " "
       << LeftHip.y << " "
       << LeftHip.z );
  
  lCOMPosition = m_DMB->getPositionCoM();
  ODEBUG4( "COM positions: " << lCOMPosition.x << " "
	  << lCOMPosition.y << " "
	   << lCOMPosition.z,"DebugDataStartingCOM.dat");

  m_DiffBetweenComAndWaist[0] =  -lCOMPosition.x;
  m_DiffBetweenComAndWaist[1] =  -lCOMPosition.y;
  m_DiffBetweenComAndWaist[2] =  -lCOMPosition.z 
    -(m_PC->GetHeightOfCoM() + lFootPosition.z - m_AnkleSoilDistance - lCOMPosition.z); // This term is usefull if
  // the initial position does not put z at Zc
  
  // The most important line of the method...
  // The one which initialize correctly the height of the pattern generator.
  for(int i=0;i<3;i++)
    {
      m_TranslationToTheLeftHip(i,0) = m_StaticToTheLeftHip(i,0) + m_DiffBetweenComAndWaist[i];
      m_TranslationToTheRightHip(i,0) = m_StaticToTheRightHip(i,0) + m_DiffBetweenComAndWaist[i];
    }

  VNL::Matrix<double> lqr(6,1);
  VNL::Matrix<double> Foot_R(3,3),Body_R(3,3);
  VNL::Matrix<double> Body_P(3,1),Foot_P(3,1);
  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      {
	if (i!=j)
	  Foot_R[i][j] = 
	    Body_R[i][j] = 0.0;
	else 
	  Foot_R[i][j] = 
	    Body_R[i][j] = 1.0;
      }  
  VNL::Matrix<double> ToTheHip(3,1);

  ODEBUG( Body_R << endl
	  << Foot_R );
  ToTheHip = Body_R * m_TranslationToTheRightHip;
  ODEBUG("m_StaticToTheRightHip " << m_TranslationToTheRightHip);
  ODEBUG( "ToTheHip " << ToTheHip );

  Body_P(0, 0)= LeftHip.x;
  Body_P(1, 0)= LeftHip.y;
  Body_P(2, 0)= 0.0;


  lFootPosition =  m_DMB->Getp(11);  
  InitLeftFootPosition.x = lFootPosition.x;
  InitLeftFootPosition.y = lFootPosition.y;
  InitLeftFootPosition.z = 0.0;
  
  ODEBUG( "Body_R " << Body_R );
  ODEBUG( "Body_P  " << Body_P );
  Foot_P(0,0) = lFootPosition.x;
  Foot_P(1,0) = lFootPosition.y;
  Foot_P(2,0) = lFootPosition.z;

  InitRightFootPosition.z = 0.0;
  ODEBUG( "Foot_P  " << Foot_P );
  ODEBUG( "Foot_R  " << Foot_R );
  ODEBUG( "m_Dt" << m_Dt );

  // RIGHT FOOT.
  m_Dt(1,0) = -m_Dt(1,0);
  
  // Compute the inverse kinematics.
  m_IK->ComputeInverseKinematics2ForLegs(Body_R,
					 Body_P,
					 m_Dt,
					 Foot_R,
					 Foot_P,
					 lqr);

  m_Dt(1,0)= -m_Dt(1,0);



#if 0
  cout << "Right leg \t Initial Position" << endl;
  for(unsigned int i=0;i<6;i++)
    cout << (lqr[i][0]*180.0/M_PI) <<  " \t " 
	 << (BodyAngles[i][0]*180/M_PI) << endl ;
  cout << endl;
#endif

  return 1;
}

void ZMPPreviewControlWithMultiBodyZMP::GetDifferenceBetweenComAndWaist(double lDiffBetweenComAndWaist[3] )
{
  for(unsigned int i =0;i<3;i++)
    lDiffBetweenComAndWaist[i] =    m_DiffBetweenComAndWaist[i];
}

VNL::Matrix<double> ZMPPreviewControlWithMultiBodyZMP::GetFinalDesiredCOMPose()
{
   return m_FinalDesiredCOMPose;
}

VNL::Vector<double>  ZMPPreviewControlWithMultiBodyZMP::GetCurrentPositionofWaistInCOMFrame()
{
  VNL::Vector<double> CurPosWICF_homogeneous(4);
  for(int i=0;i<3;i++)
     CurPosWICF_homogeneous[i] = m_DiffBetweenComAndWaist[i];
  CurPosWICF_homogeneous[3] = 1.0;

  return CurPosWICF_homogeneous;
}

void ZMPPreviewControlWithMultiBodyZMP::UpdateTheZMPRefQueue(ZMPPosition NewZMPRefPos)
{
  m_FIFOZMPRefPositions.push_back(NewZMPRefPos);
}
 
void ZMPPreviewControlWithMultiBodyZMP::SetAlgorithmForZMPAndCoMTrajectoryGeneration(int aZMPComTraj)
{
  switch(aZMPComTraj)
    {
    case ZMPCOM_TRAJECTORY_KAJITA:
      m_ZMPCoMTrajectoryAlgorithm = ZMPCOM_TRAJECTORY_KAJITA;
      break;
    case ZMPCOM_TRAJECTORY_WIEBER:
      m_ZMPCoMTrajectoryAlgorithm = ZMPCOM_TRAJECTORY_WIEBER;
      break;
    default:
      break;
    }
}

int ZMPPreviewControlWithMultiBodyZMP::GetAlgorithmForZMPAndCoMTrajectoryGeneration()
{
  return m_ZMPCoMTrajectoryAlgorithm;
}

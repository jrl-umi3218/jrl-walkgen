/* This object generate all the values for the foot trajectories,

   Copyright (c) 2007-2009, 
   Olivier Stasse,Francois Keith 
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see License.txt for further information on license.      
*/
#include <iostream>
#include <fstream>

#include <Debug.h>

#include "FootTrajectoryGenerationStandard.h"


using namespace PatternGeneratorJRL;

FootTrajectoryGenerationStandard::FootTrajectoryGenerationStandard(SimplePluginManager *lSPM,
								   CjrlFoot *aFoot)
  : FootTrajectoryGenerationAbstract(lSPM,aFoot)
{
  /* Initialize the pointers to polynomes. */
  m_PolynomeX = 0;
  m_PolynomeY = 0;
  m_PolynomeZ = 0;
  m_PolynomeOmega = 0;
  m_PolynomeOmega2 = 0;
  m_PolynomeTheta = 0;

  /* Computes information on foot dimension 
     from humanoid specific informations. */
  double lWidth,lHeight,lDepth;
  m_Foot->getSoleSize(lWidth,lHeight);
  vector3d AnklePosition;
  m_Foot->getAnklePositionInLocalFrame(AnklePosition);
  lDepth = AnklePosition[2];

  /*! Compute information for omega. */
  m_FootB =  AnklePosition[0];
  m_FootH = m_AnklePositionRight[2] = AnklePosition[2];
  m_AnklePositionRight[1] = AnklePosition[1];
  m_FootF = lDepth-AnklePosition[0];

  m_AnklePositionRight[0] = -lDepth*0.5 + AnklePosition[0];
  m_AnklePositionRight[1] = lWidth*0.5 - AnklePosition[1];
  m_AnklePositionRight[2] = AnklePosition[2];
  
  /* Compute Left foot coordinates */
  m_Foot->getAnklePositionInLocalFrame(AnklePosition);
  m_Foot->getSoleSize(lWidth,lHeight);

  m_AnklePositionLeft[0] = -lDepth*0.5 + AnklePosition[0];
  m_AnklePositionLeft[1] = -lWidth*0.5 + AnklePosition[1];
  m_AnklePositionLeft[2] = AnklePosition[2];

  RESETDEBUG4("GeneratedFoot.dat");
}

FootTrajectoryGenerationStandard::~FootTrajectoryGenerationStandard()
{
  if (m_PolynomeX!=0)
    delete m_PolynomeX;
  
  if (m_PolynomeY!=0)
    delete m_PolynomeY;

  if (m_PolynomeZ!=0)
    delete m_PolynomeZ;

  if (m_PolynomeOmega!=0)
    delete m_PolynomeOmega;

  if (m_PolynomeOmega2!=0)
    delete m_PolynomeOmega2;

  if (m_PolynomeTheta!=0)
    delete m_PolynomeTheta;
}

void FootTrajectoryGenerationStandard::InitializeInternalDataStructures()
{
  FreeInternalDataStructures();
  
  m_PolynomeX = new Polynome3(0,0);
  m_PolynomeY = new Polynome3(0,0);
  m_PolynomeZ = new Polynome4(0,0);
  m_PolynomeOmega = new Polynome3(0,0);
  m_PolynomeOmega2 = new Polynome3(0,0);
  m_PolynomeTheta = new Polynome3(0,0);  
}

void FootTrajectoryGenerationStandard::FreeInternalDataStructures()
{
  if (m_PolynomeX!=0)
    delete m_PolynomeX;

  if (m_PolynomeY!=0)
    delete m_PolynomeY;

  if (m_PolynomeZ!=0)
    delete m_PolynomeZ;

  if (m_PolynomeOmega!=0)
    delete m_PolynomeOmega;

  if (m_PolynomeOmega2!=0)
    delete m_PolynomeOmega2;

  if (m_PolynomeTheta!=0)
    delete m_PolynomeTheta;

}

int FootTrajectoryGenerationStandard::SetParameters(int PolynomeIndex,
						     double TimeInterval,
						     double Position)
{
 switch (PolynomeIndex)
   {
     
   case X_AXIS:
     m_PolynomeX->SetParameters(TimeInterval,Position);
     break;
     
   case Y_AXIS:
     m_PolynomeY->SetParameters(TimeInterval,Position);
     break;

   case Z_AXIS:
     m_PolynomeZ->SetParameters(TimeInterval,Position);
     break;

   case THETA_AXIS:
     m_PolynomeTheta->SetParameters(TimeInterval,Position);
     break;

   case OMEGA_AXIS:
     m_PolynomeOmega->SetParameters(TimeInterval,Position);
     break;

   case OMEGA2_AXIS:
     m_PolynomeOmega2->SetParameters(TimeInterval,Position);
     break;

   default:
     return -1;
     break;
   }
 return 0;
}

int FootTrajectoryGenerationStandard::SetParametersWithInitPosInitSpeed(int PolynomeIndex,
									double TimeInterval,
									double FinalPosition,
									double InitPosition,
									double InitSpeed)
{
 switch (PolynomeIndex)
   {
     
   case X_AXIS:
     ODEBUG("Initspeed: " << InitSpeed << " ");
     m_PolynomeX->SetParametersWithInitPosInitSpeed(TimeInterval,FinalPosition,InitPosition,InitSpeed);
     break;
     
   case Y_AXIS:
     m_PolynomeY->SetParametersWithInitPosInitSpeed(TimeInterval,FinalPosition,InitPosition,InitSpeed);
     break;

   case Z_AXIS:
     m_PolynomeZ->SetParametersWithInitPosInitSpeed(TimeInterval,FinalPosition,InitPosition,InitSpeed);
     break;

   case THETA_AXIS:
     m_PolynomeTheta->SetParametersWithInitPosInitSpeed(TimeInterval,FinalPosition,InitPosition,InitSpeed);
     break;

   case OMEGA_AXIS:
     m_PolynomeOmega->SetParametersWithInitPosInitSpeed(TimeInterval,FinalPosition,InitPosition,InitSpeed);
     break;

   case OMEGA2_AXIS:
     m_PolynomeOmega2->SetParametersWithInitPosInitSpeed(TimeInterval,FinalPosition,InitPosition,InitSpeed);
     break;

   default:
     return -1;
     break;
   }
 return 0;
}

double FootTrajectoryGenerationStandard::ComputeAll(FootAbsolutePosition & aFootAbsolutePosition,
						    double Time)
{
  aFootAbsolutePosition.x = m_PolynomeX->Compute(Time);
  aFootAbsolutePosition.dx = m_PolynomeX->ComputeDerivative(Time);
  ODEBUG("t: " << Time << " : " << aFootAbsolutePosition.x); 

  aFootAbsolutePosition.y = m_PolynomeY->Compute(Time);
  aFootAbsolutePosition.dy = m_PolynomeY->ComputeDerivative(Time);

  aFootAbsolutePosition.z = m_PolynomeZ->Compute(Time);
  aFootAbsolutePosition.dz = m_PolynomeZ->ComputeDerivative(Time);

  aFootAbsolutePosition.theta = m_PolynomeTheta->Compute(Time);
  aFootAbsolutePosition.dtheta = m_PolynomeTheta->ComputeDerivative(Time);

  aFootAbsolutePosition.omega = m_PolynomeOmega->Compute(Time);
  aFootAbsolutePosition.domega = m_PolynomeOmega->ComputeDerivative(Time);

  aFootAbsolutePosition.omega2 = m_PolynomeOmega2->Compute(Time);
  aFootAbsolutePosition.domega2 = m_PolynomeOmega2->ComputeDerivative(Time);
  return Time;
}

double FootTrajectoryGenerationStandard::Compute(unsigned int PolynomeIndex, double Time)
{
  double r=0.0;

  switch (PolynomeIndex)
   {
     
   case X_AXIS:
     r=m_PolynomeX->Compute(Time);
     break;
     
   case Y_AXIS:
     r=m_PolynomeY->Compute(Time);
     break;

   case Z_AXIS:
     r=m_PolynomeZ->Compute(Time);
     break;

   case THETA_AXIS:
     r=m_PolynomeTheta->Compute(Time);
     break;

   case OMEGA_AXIS:
     r=m_PolynomeOmega->Compute(Time);
     break;

   case OMEGA2_AXIS:
     r=m_PolynomeOmega2->Compute(Time);
     break;

   default:
     return -1.0;
     break;
   }
 return r;
}

void FootTrajectoryGenerationStandard::UpdateFootPosition(deque<FootAbsolutePosition> &SupportFootAbsolutePositions,
							  deque<FootAbsolutePosition> &NoneSupportFootAbsolutePositions,
							  int CurrentAbsoluteIndex,  
							  int IndexInitial, 
							  double ModulatedSingleSupportTime,
							  int StepType, int LeftOrRight)
{
  unsigned int k = CurrentAbsoluteIndex - IndexInitial;
  // Local time
  double LocalTime = k*m_SamplingPeriod;
  double EndOfLiftOff = (m_TSingle-ModulatedSingleSupportTime)*0.5;
  double StartLanding = EndOfLiftOff + ModulatedSingleSupportTime;

  // The foot support does not move.
  SupportFootAbsolutePositions[CurrentAbsoluteIndex] = 
    SupportFootAbsolutePositions[CurrentAbsoluteIndex-1];

  SupportFootAbsolutePositions[CurrentAbsoluteIndex].stepType = (-1)*StepType;

  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].stepType = StepType;
  
  if (LocalTime < EndOfLiftOff)
    {
      // Do not modify x, y and theta while liftoff.
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].x = 
	NoneSupportFootAbsolutePositions[IndexInitial].x;

      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].y = 
	NoneSupportFootAbsolutePositions[IndexInitial].y;
       
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].theta = 
	NoneSupportFootAbsolutePositions[IndexInitial].theta;
    }
  else if (LocalTime < StartLanding)
    {
      // DO MODIFY x, y and theta the remaining time.
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].x = 
	m_PolynomeX->Compute(LocalTime - EndOfLiftOff) + 
	NoneSupportFootAbsolutePositions[IndexInitial].x;

      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].y = 
	m_PolynomeY->Compute(LocalTime - EndOfLiftOff) + 
	NoneSupportFootAbsolutePositions[IndexInitial].y;

      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].theta = 
	m_PolynomeTheta->Compute(LocalTime - EndOfLiftOff) + 
	NoneSupportFootAbsolutePositions[IndexInitial].theta;
	  
    }
  else 
    {
      // Do not modify x, y and theta while landing.
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].x = 
	m_PolynomeX->Compute(ModulatedSingleSupportTime) + 
	NoneSupportFootAbsolutePositions[IndexInitial].x;
       
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].y = 
	m_PolynomeY->Compute(ModulatedSingleSupportTime) + 
	NoneSupportFootAbsolutePositions[IndexInitial].y;

      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].theta = 
	m_PolynomeTheta->Compute(ModulatedSingleSupportTime) + 
	NoneSupportFootAbsolutePositions[IndexInitial].theta;
    }

  

  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].z = 
    m_PolynomeZ->Compute(LocalTime) + 
    NoneSupportFootAbsolutePositions[IndexInitial].z;
  
  bool ProtectionNeeded=false;

  // Treat Omega with the following strategy:
  // First treat the lift-off.
  if (LocalTime<EndOfLiftOff)
    {
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].omega = 
	m_PolynomeOmega->Compute(LocalTime) ;      
      ProtectionNeeded=true;
    }
  // Prepare for the landing.
  else if (LocalTime<StartLanding)
    {
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].omega = 
	m_Omega - m_PolynomeOmega2->Compute(LocalTime-EndOfLiftOff);
    }
  // Realize the landing.
  else 
    {
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].omega = 
	m_PolynomeOmega->Compute(LocalTime - StartLanding)  - m_Omega;
      ProtectionNeeded=true;
    }
  double dFX=0,dFY=0,dFZ=0;
  double lOmega = 0.0;
  lOmega = NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].omega*M_PI/180.0;
  double lTheta = 0.0;
  lTheta = NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].theta*M_PI/180.0;

  double c = cos(lTheta);
  double s = sin(lTheta);

  {
    // Make sure the foot is not going inside the floor.
    double dX=0,Z1=0,Z2=0,X1=0,X2=0;
    double B=m_FootB,H=m_FootH,F=m_FootF; 

    if (lOmega<0)
      {
	X1 = B*cos(-lOmega);
	X2 = H*sin(-lOmega);
	Z1 = H*cos(-lOmega);
	Z2 = B*sin(-lOmega);
	dX = -(B - X1 + X2);
	dFZ = Z1 + Z2 - H;
      }
    else
      {
	X1 = F*cos(lOmega);
	X2 = H*sin(lOmega);
	Z1 = H*cos(lOmega);
	Z2 = F*sin(lOmega);
	dX = (F - X1 + X2);
	dFZ = Z1 + Z2 - H;
      }
    dFX = c*dX;
    dFY = s*dX;
  }

#if _DEBUG_4_ACTIVATED_
  ofstream aoflocal;
  aoflocal.open("Corrections.dat",ofstream::app);
  aoflocal << dFX << " " << dFY << " " << dFZ << " " << lOmega << endl;
  aoflocal.close();
#endif
  MAL_S3_VECTOR(Foot_Shift,double);
#if 0
  double co,so;

  co = cos(lOmega);
  so = sin(lOmega);
  
  // COM Orientation
  MAL_S3x3_MATRIX(Foot_R,double);

  Foot_R(0,0) = c*co;        Foot_R(0,1) = -s;      Foot_R(0,2) = c*so;
  Foot_R(1,0) = s*co;        Foot_R(1,1) =  c;      Foot_R(1,2) = s*so;
  Foot_R(2,0) = -so;         Foot_R(2,1) = 0;       Foot_R(2,2) = co;

  if (LeftOrRight==-1)
    {
      MAL_S3x3_C_eq_A_by_B(Foot_Shift, Foot_R,m_AnklePositionRight);
    }
  else if (LeftOrRight==1)
    MAL_S3x3_C_eq_A_by_B(Foot_Shift, Foot_R,m_AnklePositionLeft);

  // Modification of the foot position.
  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].x += (dFX + Foot_Shift(0));
  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].y += (dFY + Foot_Shift(1));
  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].z += (dFZ + Foot_Shift(2));
#else
  // Modification of the foot position.
  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].x += dFX ;
  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].y += dFY ;
  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].z += dFZ ;
#endif
 
  ODEBUG4( "Foot Step:" << StepType << "Foot Shift: "<< Foot_Shift 
	   << " ( " << dFX<< " , " << dFY<< " , " << " , " << dFZ << " )" 
	   << NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].x << " "
	   << NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].y << " "
	   << NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].z << " "
	   ,"GeneratedFoot.dat");

}

void FootTrajectoryGenerationStandard::ComputingAbsFootPosFromQueueOfRelPos(deque<RelativeFootPosition> &RelativeFootPositions,
									    deque<FootAbsolutePosition> &AbsoluteFootPositions )
{
  
  if (AbsoluteFootPositions.size()==0)
    AbsoluteFootPositions.resize(RelativeFootPositions.size());

  /*! Compute the absolute coordinates of the steps.  */
  double CurrentAbsTheta=0.0,c=0.0,s=0.0;
  MAL_MATRIX_DIM(MM,double,2,2);
  MAL_MATRIX_DIM(CurrentSupportFootPosition,double,3,3);
  MAL_MATRIX_DIM(Orientation,double,2,2);
  MAL_MATRIX_SET_IDENTITY(CurrentSupportFootPosition);
  MAL_MATRIX_DIM(v,double,2,1);
  MAL_MATRIX_DIM(v2,double,2,1);

  for(unsigned int i=0;i<RelativeFootPositions.size();i++)
    {
      /*! Compute Orientation matrix related to the relative orientation
	of the support foot */
      c = cos(RelativeFootPositions[i].theta*M_PI/180.0);
      s = sin(RelativeFootPositions[i].theta*M_PI/180.0);
      MM(0,0) = c;      MM(0,1) = -s;
      MM(1,0) = s;      MM(1,1) = c;
	
      /*! Update the orientation */
      CurrentAbsTheta+= RelativeFootPositions[i].theta;
      CurrentAbsTheta = fmod(CurrentAbsTheta,180.0);
	
      /*! Extract the current absolute orientation matrix. */
      for(int k=0;k<2;k++)
	for(int l=0;l<2;l++)
	  Orientation(k,l) = CurrentSupportFootPosition(k,l);
	
      /*! Put in a vector form the translation of the relative foot. */
      v(0,0) = RelativeFootPositions[i].sx;
      v(1,0) = RelativeFootPositions[i].sy;
	
      /*! Compute the new orientation of the foot vector. */
      Orientation = MAL_RET_A_by_B(MM , Orientation);
      v2 = MAL_RET_A_by_B(Orientation, v);
	
      /*! Update the world coordinates of the support foot. */
      for(int k=0;k<2;k++)
	for(int l=0;l<2;l++)
	  CurrentSupportFootPosition(k,l) = Orientation(k,l);
	
      for(int k=0;k<2;k++)
	CurrentSupportFootPosition(k,2) += v2(k,0);

      AbsoluteFootPositions[i].x = v2(0,0);
      AbsoluteFootPositions[i].y = v2(1,0);
      AbsoluteFootPositions[i].theta = CurrentAbsTheta;
    }
}




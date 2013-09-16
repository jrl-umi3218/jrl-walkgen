/*
 * Copyright 2008, 2009, 2010, 
 *
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
/* This object generate all the values for the foot trajectories, */
#include <iostream>
#include <fstream>

#include <Debug.hh>

#include "FootTrajectoryGenerationStandard.hh"


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
  if (m_Foot!=0)
    m_Foot->getSoleSize(lWidth,lHeight);
  else
    {
      cerr << "Pb no ref Foot." << endl;
    }
  vector3d AnklePosition;
  if (m_Foot!=0)
    m_Foot->getAnklePositionInLocalFrame(AnklePosition);
  else
    {
      cerr << "Pb no ref Foot." << endl;
    }

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
  if (m_Foot!=0)
    {
      m_Foot->getAnklePositionInLocalFrame(AnklePosition);
      m_Foot->getSoleSize(lWidth,lHeight);
    }
  else
    {
      cerr << "Pb no ref Foot." << endl;
    }

  m_AnklePositionLeft[0] = -lDepth*0.5 + AnklePosition[0];
  m_AnklePositionLeft[1] = -lWidth*0.5 + AnklePosition[1];
  m_AnklePositionLeft[2] = AnklePosition[2];

  RESETDEBUG5("GeneratedFoot.dat");
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
  
  m_PolynomeX = new Polynome5(0,0);
  m_PolynomeY = new Polynome5(0,0);
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
     ODEBUG2("Initspeed: " << InitSpeed << " ");
     m_PolynomeX->SetParameters(TimeInterval,FinalPosition,InitPosition,InitSpeed,0.0);
     break;
     
   case Y_AXIS:
     m_PolynomeY->SetParameters(TimeInterval,FinalPosition,InitPosition,InitSpeed,0.0);
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

int FootTrajectoryGenerationStandard::SetParameters(int PolynomeIndex, double TimeInterval,
    double FinalPosition, double InitPosition, double InitSpeed, double InitAcc)
{
 switch (PolynomeIndex)
   {

   case X_AXIS:
     m_PolynomeX->SetParameters(TimeInterval,FinalPosition,InitPosition,InitSpeed,InitAcc);
     break;

   case Y_AXIS:
     m_PolynomeY->SetParameters(TimeInterval,FinalPosition,InitPosition,InitSpeed,InitAcc);
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

int FootTrajectoryGenerationStandard::GetParametersWithInitPosInitSpeed(int PolynomeIndex,
									double &TimeInterval,
									double &FinalPosition,
									double &InitPosition,
									double &InitSpeed)
{
 switch (PolynomeIndex)
   {
     
   case X_AXIS:
     ODEBUG2("Initspeed: " << InitSpeed << " ");
     m_PolynomeX->GetParametersWithInitPosInitSpeed(TimeInterval,FinalPosition,InitPosition,InitSpeed);
     break;
     
   case Y_AXIS:
     m_PolynomeY->GetParametersWithInitPosInitSpeed(TimeInterval,FinalPosition,InitPosition,InitSpeed);
     break;

   case Z_AXIS:
     m_PolynomeZ->GetParametersWithInitPosInitSpeed(TimeInterval,FinalPosition,InitPosition,InitSpeed);
     break;

   case THETA_AXIS:
     m_PolynomeTheta->GetParametersWithInitPosInitSpeed(TimeInterval,FinalPosition,InitPosition,InitSpeed);
     break;

   case OMEGA_AXIS:
     m_PolynomeOmega->GetParametersWithInitPosInitSpeed(TimeInterval,FinalPosition,InitPosition,InitSpeed);
     break;

   case OMEGA2_AXIS:
     m_PolynomeOmega2->GetParametersWithInitPosInitSpeed(TimeInterval,FinalPosition,InitPosition,InitSpeed);
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
  //  aFootAbsolutePosition.ddx = m_PolynomeX->ComputeSecDerivative(Time);
  ODEBUG2("t: " << Time << " : " << aFootAbsolutePosition.x); 

  aFootAbsolutePosition.y = m_PolynomeY->Compute(Time);
  aFootAbsolutePosition.dy = m_PolynomeY->ComputeDerivative(Time);
  //  aFootAbsolutePosition.ddy = m_PolynomeY->ComputeSecDerivative(Time);

  aFootAbsolutePosition.z = m_PolynomeZ->Compute(Time);
  aFootAbsolutePosition.dz = m_PolynomeZ->ComputeDerivative(Time);
  //  aFootAbsolutePosition.ddz = m_PolynomeZ->ComputeSecDerivative(Time);

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

double FootTrajectoryGenerationStandard::ComputeSecDerivative(unsigned int PolynomeIndex, double Time)
{
  double r=0.0;

  switch (PolynomeIndex)
   {

   case X_AXIS:
     r=m_PolynomeX->ComputeSecDerivative(Time);
     break;

   case Y_AXIS:
     r=m_PolynomeY->ComputeSecDerivative(Time);
     break;

   case Z_AXIS:
     r=m_PolynomeZ->ComputeSecDerivative(Time);
     break;

   case THETA_AXIS:
     r=m_PolynomeTheta->ComputeSecDerivative(Time);
     break;

   case OMEGA_AXIS:
     r=m_PolynomeOmega->ComputeSecDerivative(Time);
     break;

   case OMEGA2_AXIS:
     r=m_PolynomeOmega2->ComputeSecDerivative(Time);
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
							  int StepType, 
							  int /* LeftOrRight */)
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

  FootAbsolutePosition & curr_NSFAP = NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex];
  const FootAbsolutePosition & init_NSFAP = NoneSupportFootAbsolutePositions[IndexInitial];

  curr_NSFAP.stepType = StepType;
  
  if (LocalTime < EndOfLiftOff)
    {
      // Do not modify x, y and theta while liftoff.
      curr_NSFAP.x = init_NSFAP.x;
      curr_NSFAP.y = init_NSFAP.y;
      curr_NSFAP.theta = init_NSFAP.theta;
    }
  else if (LocalTime < StartLanding)
    {
      // DO MODIFY x, y and theta the remaining time.
      curr_NSFAP.x     = init_NSFAP.x     + m_PolynomeX->Compute(LocalTime - EndOfLiftOff);
      curr_NSFAP.y     = init_NSFAP.y     + m_PolynomeY->Compute(LocalTime - EndOfLiftOff);
      curr_NSFAP.theta = init_NSFAP.theta + m_PolynomeTheta->Compute(LocalTime - EndOfLiftOff);
    }
  else 
    {
      // Do not modify x, y and theta while landing.
      curr_NSFAP.x     = init_NSFAP.x     + m_PolynomeX->Compute(ModulatedSingleSupportTime);
      curr_NSFAP.y     = init_NSFAP.y     + m_PolynomeY->Compute(ModulatedSingleSupportTime);
      curr_NSFAP.theta = init_NSFAP.theta + m_PolynomeTheta->Compute(ModulatedSingleSupportTime);
    }

  curr_NSFAP.z = init_NSFAP.z + m_PolynomeZ->Compute(LocalTime);
  ODEBUG2("x:" << curr_NSFAP.x << " LocalTime - EndOfLiftOff" << LocalTime - EndOfLiftOff
          << " " << m_PolynomeX->Compute(LocalTime - EndOfLiftOff));
  //  m_PolynomeX->print();
  
  bool ProtectionNeeded=false;

  // Treat Omega with the following strategy:
  // First treat the lift-off.
  if (LocalTime<EndOfLiftOff)
    {
      curr_NSFAP.omega = m_PolynomeOmega->Compute(LocalTime) ;
      ProtectionNeeded=true;
    }
  // Prepare for the landing.
  else if (LocalTime<StartLanding)
    {
      curr_NSFAP.omega =
	m_Omega - m_PolynomeOmega2->Compute(LocalTime-EndOfLiftOff);
    }
  // Realize the landing.
  else 
    {
      curr_NSFAP.omega =
	m_PolynomeOmega->Compute(LocalTime - StartLanding)  - m_Omega;
      ProtectionNeeded=true;
    }
  double dFX=0,dFY=0,dFZ=0;
  double lOmega = 0.0;
  lOmega = curr_NSFAP.omega*M_PI/180.0;
  double lTheta = 0.0;
  lTheta = curr_NSFAP.theta*M_PI/180.0;

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
  curr_NSFAP.x += (dFX + Foot_Shift(0));
  curr_NSFAP.y += (dFY + Foot_Shift(1));
  curr_NSFAP.z += (dFZ + Foot_Shift(2));
#else
  // Modification of the foot position.
  curr_NSFAP.x += dFX ;
  curr_NSFAP.y += dFY ;
  curr_NSFAP.z += dFZ ;
#endif
 
  ODEBUG4( "Foot Step:" << StepType << "Foot Shift: "<< Foot_Shift 
          << " ( " << dFX<< " , " << dFY<< " , " << " , " << dFZ << " )" 
          << curr_NSFAP.x << " "
          << curr_NSFAP.y << " "
          << curr_NSFAP.z << " "
          ,"GeneratedFoot.dat");

}

void FootTrajectoryGenerationStandard::UpdateFootPosition(deque<FootAbsolutePosition> &SupportFootAbsolutePositions,
							  deque<FootAbsolutePosition> &NoneSupportFootAbsolutePositions,
							  int StartIndex, int k,
							  double LocalInterpolationStartTime,
							  double ModulatedSingleSupportTime,
							  int StepType, int /* LeftOrRight */)
{//TODO 0:Update foot position needs to be verified and cleaned

  // unsigned int k = CurrentAbsoluteIndex - IndexInitial;
  // Local time
  double InterpolationTime = (double)k*m_SamplingPeriod;
  int CurrentAbsoluteIndex = k+StartIndex;
  // unsigned int IndexInitial = CurrentAbsoluteIndex-1;
  double EndOfLiftOff = (m_TSingle-ModulatedSingleSupportTime)*0.5;
  double StartLanding = EndOfLiftOff + ModulatedSingleSupportTime;

  // The foot support does not move.
  SupportFootAbsolutePositions[CurrentAbsoluteIndex] = 
    SupportFootAbsolutePositions[StartIndex-1];

  SupportFootAbsolutePositions[CurrentAbsoluteIndex].stepType = (-1)*StepType;

  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].stepType = StepType;
  cout<<"LocalInterpolationStartTime+InterpolationTime: "<<LocalInterpolationStartTime+InterpolationTime;
  if (LocalInterpolationStartTime +InterpolationTime <= EndOfLiftOff || LocalInterpolationStartTime +InterpolationTime >= StartLanding)
    {
      // Do not modify x, y and theta while liftoff.
      // cout<<"no change"<<endl;
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].x = 
	NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex-1].x;

      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].y = 
	NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex-1].y;
       
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].theta = 
	NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex-1].theta;
    }
  else if (LocalInterpolationStartTime < EndOfLiftOff && LocalInterpolationStartTime +InterpolationTime > EndOfLiftOff)
    {
      // cout<<"rest changes"<<endl;
      // DO MODIFY x, y and theta the remaining time.
      // x, dx
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].x = 
	m_PolynomeX->Compute(LocalInterpolationStartTime + InterpolationTime - EndOfLiftOff);// +
	// NoneSupportFootAbsolutePositions[StartIndex-1].x;
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].dx = 
	m_PolynomeX->ComputeDerivative(LocalInterpolationStartTime + InterpolationTime - EndOfLiftOff);// +
	// NoneSupportFootAbsolutePositions[StartIndex-1].dx;
      //y, dy
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].y = 
	m_PolynomeY->Compute(LocalInterpolationStartTime + InterpolationTime - EndOfLiftOff);//  + 
	// NoneSupportFootAbsolutePositions[StartIndex-1].y;
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].dy = 
	m_PolynomeY->ComputeDerivative(LocalInterpolationStartTime + InterpolationTime - EndOfLiftOff); // +
	// NoneSupportFootAbsolutePositions[StartIndex-1].dy;
      //theta, dtheta
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].theta = 
	m_PolynomeTheta->Compute(LocalInterpolationStartTime + InterpolationTime - EndOfLiftOff);// + 
	//NoneSupportFootAbsolutePositions[StartIndex].theta;
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].dtheta = 
	m_PolynomeTheta->ComputeDerivative(LocalInterpolationStartTime + InterpolationTime - EndOfLiftOff);
	// +NoneSupportFootAbsolutePositions[StartIndex].dtheta;
    }
  else 
    {
      // cout<<"all changes";
      // DO MODIFY x, y and theta all the time.
      // x, dx
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].x = 
	m_PolynomeX->Compute(InterpolationTime);
      //+NoneSupportFootAbsolutePositions[StartIndex-1].x;
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].dx = 
	m_PolynomeX->ComputeDerivative(InterpolationTime);
      //+NoneSupportFootAbsolutePositions[StartIndex-1].dx;
      //y, dy
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].y = 
	m_PolynomeY->Compute(InterpolationTime);
      //+NoneSupportFootAbsolutePositions[StartIndex].y;
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].dy = 
	m_PolynomeY->ComputeDerivative(InterpolationTime);
      //+NoneSupportFootAbsolutePositions[StartIndex].dy;
      //theta, dtheta
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].theta = 
	m_PolynomeTheta->Compute( InterpolationTime );
      // +NoneSupportFootAbsolutePositions[StartIndex].theta;
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].dtheta = 
	m_PolynomeTheta->ComputeDerivative(InterpolationTime);
      // + NoneSupportFootAbsolutePositions[StartIndex].dtheta;
    }

  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].z = 
    m_PolynomeZ->Compute(LocalInterpolationStartTime+InterpolationTime);//+
    //m_AnklePositionRight[2];
  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].dz = 
    m_PolynomeZ->Compute(LocalInterpolationStartTime+InterpolationTime);//+
    //m_AnklePositionRight[2];
  
  bool ProtectionNeeded=false;

  // Treat Omega with the following strategy:
  // First treat the lift-off.
  if (LocalInterpolationStartTime+InterpolationTime<EndOfLiftOff)
    {
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].omega = 
	m_PolynomeOmega->Compute(InterpolationTime); // + 
    // NoneSupportFootAbsolutePositions[StartIndex-1].omega; 
      
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].domega = 
	m_PolynomeOmega->Compute(InterpolationTime);//  + 
    // NoneSupportFootAbsolutePositions[StartIndex-1].domega;  
      
      ProtectionNeeded=true;
    }
  // Prepare for the landing.
  else if (LocalInterpolationStartTime+InterpolationTime<StartLanding)
    {
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].omega = 
	m_Omega - m_PolynomeOmega2->Compute(LocalInterpolationStartTime+InterpolationTime-EndOfLiftOff)-
	NoneSupportFootAbsolutePositions[StartIndex-1].omega2;
    }
  // Realize the landing.
  else 
    {
      NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].omega = 
	m_PolynomeOmega->Compute(LocalInterpolationStartTime+InterpolationTime - StartLanding) + 
	NoneSupportFootAbsolutePositions[StartIndex-1].omega - m_Omega;
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

void FootTrajectoryGenerationStandard::print()
{
  std::cout << "Polynome X:" <<std::endl;
  m_PolynomeX->print();
  std::cout << "Polynome Y:" <<std::endl;
  m_PolynomeY->print();
  std::cout << "Polynome Z:" <<std::endl;
  m_PolynomeZ->print();
  std::cout << "Polynome Roll:" <<std::endl;
  m_PolynomeOmega->print();
  std::cout << "Polynome Pitch:" <<std::endl;
  m_PolynomeOmega2->print();
  std::cout << "Polynome Yaw:" <<std::endl;
  m_PolynomeTheta->print();
  
}


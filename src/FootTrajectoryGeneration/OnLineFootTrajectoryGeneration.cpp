/*
 * Copyright 2011
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
/* This object generate all the values for the foot trajectories, */
#include <iostream>
#include <fstream>

#include <privatepgtypes.hh>

#include "OnLineFootTrajectoryGeneration.h"


using namespace PatternGeneratorJRL;

OnLineFootTrajectoryGeneration::OnLineFootTrajectoryGeneration(SimplePluginManager *lSPM,
    CjrlFoot *aFoot)
: FootTrajectoryGenerationStandard(lSPM,aFoot)
{

}

OnLineFootTrajectoryGeneration::~OnLineFootTrajectoryGeneration()
{

}


void
OnLineFootTrajectoryGeneration::UpdateFootPosition(deque<FootAbsolutePosition> &SupportFootAbsolutePositions,
    deque<FootAbsolutePosition> &NoneSupportFootAbsolutePositions,
    int StartIndex, int k,
    double LocalInterpolationStartTime,
    double UnlockedSwingPeriod,
    int StepType, int /* LeftOrRight */)
{
  // Local time
  double InterpolationTime = (double)k*m_SamplingPeriod;
  int CurrentAbsoluteIndex = k+StartIndex;
  double EndOfLiftOff = (m_TSingle-UnlockedSwingPeriod)*0.5;
  double StartLanding = EndOfLiftOff + UnlockedSwingPeriod;

  FootAbsolutePosition & curr_NSFAP = NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex];
  const FootAbsolutePosition & prev_NSFAP = NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex-1];

  // The foot support does not move.
  SupportFootAbsolutePositions[CurrentAbsoluteIndex] = 
      SupportFootAbsolutePositions[StartIndex-1];
  SupportFootAbsolutePositions[CurrentAbsoluteIndex].stepType = (-1)*StepType;


  curr_NSFAP.stepType = StepType;

  if (LocalInterpolationStartTime +InterpolationTime <= EndOfLiftOff || LocalInterpolationStartTime +InterpolationTime >= StartLanding)
    {
      // Do not modify x, y and theta while liftoff.
      curr_NSFAP.x = prev_NSFAP.x;
      curr_NSFAP.y = prev_NSFAP.y;
      curr_NSFAP.theta = prev_NSFAP.theta;
    }
  else if (LocalInterpolationStartTime < EndOfLiftOff && LocalInterpolationStartTime +InterpolationTime > EndOfLiftOff)
    {
      // DO MODIFY x, y and theta the remaining time.
	  double remainingTime = LocalInterpolationStartTime + InterpolationTime - EndOfLiftOff;
      // x, dx
      curr_NSFAP.x   = m_PolynomeX->Compute(remainingTime);
      curr_NSFAP.dx  = m_PolynomeX->ComputeDerivative(remainingTime);
      if(m_PolynomeX->Degree() > 4)
        curr_NSFAP.ddx  = m_PolynomeX->ComputeSecDerivative(remainingTime);
      //y, dy
      curr_NSFAP.y   = m_PolynomeY->Compute(remainingTime);
      curr_NSFAP.dy  = m_PolynomeY->ComputeDerivative(remainingTime);
      if(m_PolynomeY->Degree() > 4)
        curr_NSFAP.ddy  = m_PolynomeY->ComputeSecDerivative(remainingTime);
      //theta, dtheta
      curr_NSFAP.theta   = m_PolynomeTheta->Compute(remainingTime);
      curr_NSFAP.dtheta  = m_PolynomeTheta->ComputeDerivative(remainingTime);
      if(m_PolynomeTheta->Degree() > 4)
        curr_NSFAP.ddtheta = m_PolynomeTheta->ComputeSecDerivative(remainingTime);
    }
  else 
    {
      // DO MODIFY x, y and theta all the time.
      // x, dx
      curr_NSFAP.x  = m_PolynomeX->Compute(InterpolationTime);
      curr_NSFAP.dx = m_PolynomeX->ComputeDerivative(InterpolationTime);
      if(m_PolynomeX->Degree() > 4)
        curr_NSFAP.ddx  = m_PolynomeX->ComputeSecDerivative(InterpolationTime);
      //y, dy
      curr_NSFAP.y  = m_PolynomeY->Compute(InterpolationTime);
      curr_NSFAP.dy = m_PolynomeY->ComputeDerivative(InterpolationTime);
      if(m_PolynomeY->Degree() > 4)
        curr_NSFAP.ddy = m_PolynomeY->ComputeSecDerivative(InterpolationTime);
      //theta, dtheta
      curr_NSFAP.theta = m_PolynomeTheta->Compute( InterpolationTime );
      curr_NSFAP.dtheta = m_PolynomeTheta->ComputeDerivative(InterpolationTime);
      if(m_PolynomeTheta->Degree() > 4)
        curr_NSFAP.ddtheta = m_PolynomeTheta->ComputeSecDerivative(InterpolationTime);
    }

  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].z = 
      m_PolynomeZ->Compute(LocalInterpolationStartTime+InterpolationTime);
  NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex].dz = 
      m_PolynomeZ->ComputeDerivative(LocalInterpolationStartTime+InterpolationTime);

  bool ProtectionNeeded=false;

  // Treat Omega with the following strategy:
  // First treat the lift-off.
  if (LocalInterpolationStartTime+InterpolationTime<EndOfLiftOff)
    {
      curr_NSFAP.omega   = m_PolynomeOmega->Compute(InterpolationTime);
//TODO      curr_NSFAP.domega  = m_PolynomeOmega->Compute(InterpolationTime);
      curr_NSFAP.domega  = m_PolynomeOmega->ComputeDerivative(InterpolationTime);
      if(m_PolynomeOmega->Degree() > 4)
        curr_NSFAP.ddomega = m_PolynomeOmega->ComputeSecDerivative(InterpolationTime);

      ProtectionNeeded=true;
    }
  // Prepare for the landing.
  else if (LocalInterpolationStartTime+InterpolationTime<StartLanding)
    {
      curr_NSFAP.omega =
          m_Omega - m_PolynomeOmega2->Compute(LocalInterpolationStartTime+InterpolationTime-EndOfLiftOff)-
          NoneSupportFootAbsolutePositions[StartIndex-1].omega2;
    }
  // Realize the landing.
  else 
    {
      curr_NSFAP.omega =
          m_PolynomeOmega->Compute(LocalInterpolationStartTime+InterpolationTime - StartLanding) +
          NoneSupportFootAbsolutePositions[StartIndex-1].omega - m_Omega;
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


  MAL_S3_VECTOR(Foot_Shift,double);

  // Modification of the foot position.
  curr_NSFAP.x += dFX ;
  curr_NSFAP.y += dFY ;
  curr_NSFAP.z += dFZ ;
}


void
OnLineFootTrajectoryGeneration::interpret_solution( double CurrentTime, const solution_t & Solution,
    const support_state_t & CurrentSupport, unsigned int NbSteps, double & X, double & Y )
{

  double Sign;
  if(CurrentSupport.Foot == LEFT)
    Sign = 1.0;
  else
    Sign = -1.0;
  if(CurrentSupport.NbStepsLeft > 0 && NbSteps > 0 )
    {
      X = Solution.Solution_vec[2*QP_N_];
      Y = Solution.Solution_vec[2*QP_N_+NbSteps];
      if(fabs(X)+fabs(Y)-0.00001<0.0)
        {
          //cout<<"Previewed foot x-position zero at time: "<<CurrentTime<<endl;
        }
      else if (CurrentSupport.TimeLimit-CurrentTime-QP_T_/2.0 > 0.0)
        {//The landing position is yet determined by the solver because the robot finds himself still in the single support phase
          //do nothing
        }
    }
  else
    {//The solver isn't responsible for the feet positions anymore
      //The robot is supposed to stop always with the feet aligned in the lateral plane.
      X = CurrentSupport.X + Sign*sin(CurrentSupport.Yaw)*FeetDistanceDS_;
      Y = CurrentSupport.Y - Sign*cos(CurrentSupport.Yaw)*FeetDistanceDS_;
    }

}


void
OnLineFootTrajectoryGeneration::interpolate_feet_positions(double Time,
    const deque<support_state_t> & PrwSupportStates_deq,
    const solution_t & Solution,
    const deque<double> & PreviewedSupportAngles_deq,
    deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
    deque<FootAbsolutePosition> & FinalRightFootTraj_deq)
{

  support_state_t CurrentSupport = PrwSupportStates_deq.front();

  double FPx, FPy;
  if(CurrentSupport.Phase != DS)
    {
      unsigned int NbStepsPrwd = PrwSupportStates_deq.back().StepNumber;
      interpret_solution( Time, Solution, CurrentSupport, NbStepsPrwd, FPx, FPy );
    }

  double LocalInterpolationTime = Time-(CurrentSupport.TimeLimit-(m_TDouble+m_TSingle));

  int StepType = 1;
  unsigned int CurrentIndex = FinalLeftFootTraj_deq.size()-1;
  FinalLeftFootTraj_deq.resize((unsigned int)(QP_T_/m_SamplingPeriod)+CurrentIndex+1);
  FinalRightFootTraj_deq.resize((unsigned int)(QP_T_/m_SamplingPeriod)+CurrentIndex+1);
  if(CurrentSupport.Phase == SS && Time+3.0/2.0*QP_T_ < CurrentSupport.TimeLimit)
    {
      //determine coefficients of interpolation polynome
      double ModulationSupportCoefficient = 0.9;
      double UnlockedSwingPeriod = m_TSingle * ModulationSupportCoefficient;
      double EndOfLiftOff = (m_TSingle-UnlockedSwingPeriod)*0.5;
      double SwingTimePassed = 0.0;
      if(LocalInterpolationTime>EndOfLiftOff)
        SwingTimePassed = LocalInterpolationTime-EndOfLiftOff;

      FootAbsolutePosition * LastSFP; //LastSwingFootPosition

      if(CurrentSupport.Foot == LEFT)
        {
    	  LastSFP = &(FinalRightFootTraj_deq[CurrentIndex]);
        }
      else
        {
    	  LastSFP = &(FinalLeftFootTraj_deq[CurrentIndex]);
        }

      //Set parameters for current polynomial
      double TimeInterval = UnlockedSwingPeriod-SwingTimePassed;
      SetParameters(
    	  FootTrajectoryGenerationStandard::X_AXIS,
          TimeInterval,FPx,
          LastSFP->x, LastSFP->dx, LastSFP->ddx
          );
      SetParameters(
    	  FootTrajectoryGenerationStandard::Y_AXIS,
          TimeInterval,FPy,
          LastSFP->y, LastSFP->dy, LastSFP->ddy
          );

      if(CurrentSupport.StateChanged==true)
        SetParameters(FootTrajectoryGenerationStandard::Z_AXIS, m_TSingle, StepHeight_);

      SetParametersWithInitPosInitSpeed(
          FootTrajectoryGenerationStandard::THETA_AXIS,
          TimeInterval, PreviewedSupportAngles_deq[0]*180.0/M_PI,
          LastSFP->theta, LastSFP->dtheta);
      SetParametersWithInitPosInitSpeed(
          FootTrajectoryGenerationStandard::OMEGA_AXIS,
          TimeInterval,0.0*180.0/M_PI,
          LastSFP->omega, LastSFP->domega);
      SetParametersWithInitPosInitSpeed(
          FootTrajectoryGenerationStandard::OMEGA2_AXIS,
          TimeInterval,2*0.0*180.0/M_PI,
          LastSFP->omega2, LastSFP->domega2);

      for(int k = 1; k<=(int)(QP_T_/m_SamplingPeriod);k++)
        {
          if (CurrentSupport.Foot == LEFT)
            {
              UpdateFootPosition(FinalLeftFootTraj_deq,
                  FinalRightFootTraj_deq,
                  CurrentIndex,k,
                  LocalInterpolationTime,
                  UnlockedSwingPeriod,
                  StepType, -1);
            }
          else
            {
              UpdateFootPosition(FinalRightFootTraj_deq,
                  FinalLeftFootTraj_deq,
                  CurrentIndex,k,
                  LocalInterpolationTime,
                  UnlockedSwingPeriod,
                  StepType, 1);
            }
          FinalLeftFootTraj_deq[CurrentIndex+k].time =
              FinalRightFootTraj_deq[CurrentIndex+k].time = Time+k*m_SamplingPeriod;
        }
    }
  else if (CurrentSupport.Phase == DS || Time+3.0/2.0*QP_T_ > CurrentSupport.TimeLimit)
    {
      for(int k = 0; k<=(int)(QP_T_/m_SamplingPeriod);k++)
        {
          FinalRightFootTraj_deq[CurrentIndex+k]=               FinalRightFootTraj_deq[CurrentIndex+k-1];
          FinalLeftFootTraj_deq[CurrentIndex+k]=                FinalLeftFootTraj_deq[CurrentIndex+k-1];
          FinalLeftFootTraj_deq[CurrentIndex+k].time =
              FinalRightFootTraj_deq[CurrentIndex+k].time =     Time+k*m_SamplingPeriod;
          FinalLeftFootTraj_deq[CurrentIndex+k].stepType =
              FinalRightFootTraj_deq[CurrentIndex+k].stepType = 10;
        }
    }

}




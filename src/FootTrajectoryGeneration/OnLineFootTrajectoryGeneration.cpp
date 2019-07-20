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
#include <iomanip>

#include "Debug.hh"
#include <privatepgtypes.hh>

#include "OnLineFootTrajectoryGeneration.h"


using namespace PatternGeneratorJRL;

OnLineFootTrajectoryGeneration::
OnLineFootTrajectoryGeneration
(SimplePluginManager *lSPM,
 PRFoot *aFoot)
  : FootTrajectoryGenerationStandard(lSPM,aFoot)
{
  QP_T_ = 0.0 ;
  QP_N_ = 0 ;
  FeetDistanceDS_ = 0.0 ;
  HalfTimePassed_ = false;
  FPx_ = 0.0 ;
  FPy_ = 0.0 ;
  FirstPrvSuppFootX_vec.clear();
  FirstPrvSuppFootY_vec.clear();
}

OnLineFootTrajectoryGeneration::~OnLineFootTrajectoryGeneration()
{

}

void OnLineFootTrajectoryGeneration::
ComputeXYThetaFootPosition
(double t, FootAbsolutePosition& curr_NSFAP)
{
  // x, dx, ddx, dddx
  curr_NSFAP.x   = m_PolynomeX->Compute(t);
  curr_NSFAP.dx  = m_PolynomeX->ComputeDerivative(t);
  if(m_PolynomeX->Degree() > 4)
    curr_NSFAP.ddx  = m_PolynomeX->ComputeSecDerivative(t);
  if(m_PolynomeX->Degree() > 6)
    curr_NSFAP.dddx  = m_PolynomeX->ComputeJerk(t);

  //y, dy, ddy, dddy
  curr_NSFAP.y   = m_PolynomeY->Compute(t);
  curr_NSFAP.dy  = m_PolynomeY->ComputeDerivative(t);
  if(m_PolynomeY->Degree() > 4)
    curr_NSFAP.ddy  = m_PolynomeY->ComputeSecDerivative(t);
  if(m_PolynomeY->Degree() > 6)
    curr_NSFAP.dddy  = m_PolynomeY->ComputeJerk(t);

  //theta, dtheta, ddtheta, dddtheta
  curr_NSFAP.theta   = m_PolynomeTheta->Compute(t);
  curr_NSFAP.dtheta  = m_PolynomeTheta->ComputeDerivative(t);
  if(m_PolynomeTheta->Degree() > 4)
    curr_NSFAP.ddtheta = m_PolynomeTheta->ComputeSecDerivative(t);
  if(m_PolynomeTheta->Degree() > 6)
    curr_NSFAP.dddtheta  = m_PolynomeTheta->ComputeJerk(t);
}

void
OnLineFootTrajectoryGeneration::
UpdateFootPosition
(deque<FootAbsolutePosition> &SupportFootAbsolutePositions,
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

  FootAbsolutePosition & curr_NSFAP =
    NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex];
  const FootAbsolutePosition & prev_NSFAP =
    NoneSupportFootAbsolutePositions[CurrentAbsoluteIndex-1];

  // The foot support does not move.
  SupportFootAbsolutePositions[CurrentAbsoluteIndex] =
    SupportFootAbsolutePositions[StartIndex];
  SupportFootAbsolutePositions[CurrentAbsoluteIndex].stepType = (-1)*StepType;


  curr_NSFAP.stepType = StepType;

  if (LocalInterpolationStartTime +InterpolationTime <= EndOfLiftOff
      || LocalInterpolationStartTime +InterpolationTime >= StartLanding)
    {
      // Do not modify x, y and theta while liftoff.
      curr_NSFAP.x        = prev_NSFAP.x;
      curr_NSFAP.y        = prev_NSFAP.y;
      curr_NSFAP.theta    = prev_NSFAP.theta;
      curr_NSFAP.dx       = 0.0;
      curr_NSFAP.dy       = 0.0;
      curr_NSFAP.dtheta   = 0.0;
      curr_NSFAP.ddx      = 0.0;
      curr_NSFAP.ddy      = 0.0;
      curr_NSFAP.ddtheta  = 0.0;
      curr_NSFAP.ddx      = 0.0;
      curr_NSFAP.ddy      = 0.0;
      curr_NSFAP.ddtheta  = 0.0;
      curr_NSFAP.dddx     = 0.0;
      curr_NSFAP.dddy     = 0.0;
      curr_NSFAP.dddtheta = 0.0;
      // And all the derivatives are null
    }
  else if (LocalInterpolationStartTime < EndOfLiftOff)
    {
      // DO MODIFY x, y and theta the remaining time.
      double remainingTime = InterpolationTime - EndOfLiftOff;
      ComputeXYThetaFootPosition(remainingTime,curr_NSFAP);
    }
  else
    {
      // DO MODIFY x, y and theta the rest of the time.
      ComputeXYThetaFootPosition(InterpolationTime,curr_NSFAP);
    }

  curr_NSFAP.z = m_PolynomeZ->
    Compute(LocalInterpolationStartTime+InterpolationTime);
  curr_NSFAP.dz = m_PolynomeZ->
    ComputeDerivative(LocalInterpolationStartTime+InterpolationTime);
  curr_NSFAP.ddz = m_PolynomeZ->
    ComputeSecDerivative(LocalInterpolationStartTime+InterpolationTime);


  //bool ProtectionNeeded=false;

  // Treat Omega with the following strategy:
  // First treat the lift-off.
  if (LocalInterpolationStartTime+InterpolationTime<EndOfLiftOff)
    {
      curr_NSFAP.omega   = m_PolynomeOmega->Compute(InterpolationTime);
      //TODO curr_NSFAP.domega  = m_PolynomeOmega->Compute(InterpolationTime);
      curr_NSFAP.domega  = m_PolynomeOmega->
	ComputeDerivative(InterpolationTime);
      if(m_PolynomeOmega->Degree() > 4)
	curr_NSFAP.ddomega = m_PolynomeOmega->
	  ComputeSecDerivative(InterpolationTime);

      //ProtectionNeeded=true;
    }
  // Prepare for the landing.
  else if (LocalInterpolationStartTime+InterpolationTime<StartLanding)
    {
      curr_NSFAP.omega =
	m_Omega - m_PolynomeOmega2->
	Compute(LocalInterpolationStartTime+InterpolationTime-EndOfLiftOff)-
	NoneSupportFootAbsolutePositions[StartIndex].omega2;
    }
  // Realize the landing.
  else
    {
      curr_NSFAP.omega =
	m_PolynomeOmega->Compute
	(LocalInterpolationStartTime+InterpolationTime - StartLanding) +
	NoneSupportFootAbsolutePositions[StartIndex].omega - m_Omega;
      //ProtectionNeeded=true;
    }

  // Make sure the foot is not going inside the floor.
  double dFX=0,dFY=0,dFZ=0;
  {
    double lOmega = 0.0;
    lOmega = curr_NSFAP.omega*M_PI/180.0;
    double lTheta = 0.0;
    lTheta = curr_NSFAP.theta*M_PI/180.0;
    double c = cos(lTheta);
    double s = sin(lTheta);

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


  //MAL_S3_VECTOR(Foot_Shift,double);
  if ( abs(dFX) + abs(dFY) + abs(dFZ) )
    // Modification of the foot position.
    curr_NSFAP.x += dFX ;
  curr_NSFAP.y += dFY ;
  curr_NSFAP.z += dFZ ;
}


void
OnLineFootTrajectoryGeneration::
interpret_solution
( double CurrentTime, const solution_t & Solution,
  const support_state_t & CurrentSupport,
  unsigned int NbSteps, double & X, double & Y )
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
          ODEBUG("Previewed foot x-position zero at time: "<<CurrentTime);
        }
      else if (CurrentSupport.TimeLimit-CurrentTime-QP_T_/2.0 > 0.0)
        {
	 //The landing position is yet determined by the solver
	 // because the robot finds himself still in the single support phase
         //do nothing
        }
    }
  else
    {
      //The solver isn't responsible for the feet positions anymore
      //The robot is supposed to stop always with the feet aligned
      // in the lateral plane.
      X = CurrentSupport.X + Sign*sin(CurrentSupport.Yaw)*FeetDistanceDS_;
      Y = CurrentSupport.Y - Sign*cos(CurrentSupport.Yaw)*FeetDistanceDS_;
    }

}




void
OnLineFootTrajectoryGeneration::
interpolate_feet_positions
(double Time,
 const deque<support_state_t> & PrwSupportStates_deq,
 const solution_t & Solution,
 const deque<double> & PreviewedSupportAngles_deq,
 deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
 deque<FootAbsolutePosition> & FinalRightFootTraj_deq)
{
  support_state_t CurrentSupport = PrwSupportStates_deq.front();

  double FPx(0.0), FPy(0.0);
  if(CurrentSupport.Phase != DS)
    {
      unsigned int NbStepsPrwd = PrwSupportStates_deq.back().StepNumber;
      interpret_solution( Time, Solution, CurrentSupport,
			  NbStepsPrwd, FPx, FPy );
    }
  double LocalInterpolationStartTime = Time-
    (CurrentSupport.TimeLimit-(m_TDouble+m_TSingle));

  int StepType = 1;
  unsigned int CurrentIndex =
    (unsigned int)(FinalLeftFootTraj_deq.size()-1);


  FootAbsolutePosition * LastSFP; //LastSwingFootPosition
  if(CurrentSupport.Foot == LEFT)
    {
      LastSFP = &(FinalRightFootTraj_deq[CurrentIndex]);
    }
  else
    {
      LastSFP = &(FinalLeftFootTraj_deq[CurrentIndex]);
    }

  FinalLeftFootTraj_deq.resize((unsigned int)
			       (QP_T_/m_SamplingPeriod)+CurrentIndex+1);
  FinalRightFootTraj_deq.resize((unsigned int)
				(QP_T_/m_SamplingPeriod)+CurrentIndex+1);

  if(CurrentSupport.Phase == SS &&
     Time+1.5*QP_T_ < CurrentSupport.TimeLimit)
    {
      //determine coefficients of interpolation polynome
      double ModulationSupportCoefficient = 0.9;
      double UnlockedSwingPeriod = m_TSingle * ModulationSupportCoefficient;
      double EndOfLiftOff = (m_TSingle-UnlockedSwingPeriod)*0.5;
      double SwingTimePassed = 0.0;
      if(LocalInterpolationStartTime>EndOfLiftOff)
	SwingTimePassed = LocalInterpolationStartTime-EndOfLiftOff;

      //Set parameters for current polynomial
      double TimeInterval = UnlockedSwingPeriod-SwingTimePassed;
      SetParameters
	(FootTrajectoryGenerationStandard::X_AXIS,
	 TimeInterval,FPx,
	 LastSFP->x, LastSFP->dx, LastSFP->ddx, LastSFP->dddx);
      SetParameters
	(FootTrajectoryGenerationStandard::Y_AXIS,
	 TimeInterval,FPy,
	 LastSFP->y, LastSFP->dy, LastSFP->ddy, LastSFP->dddy);
      if(CurrentSupport.StateChanged==true)
	{
	  SetParameters
	    (FootTrajectoryGenerationStandard::Z_AXIS,
	     m_TSingle,/*m_AnklePositionLeft[2]*/0.0,
	     LastSFP->z, LastSFP->dz, LastSFP->ddz);
	}

      int index_orientation = PrwSupportStates_deq[1].StepNumber ;
      SetParameters
	(FootTrajectoryGenerationStandard::THETA_AXIS,
	 TimeInterval, PreviewedSupportAngles_deq[index_orientation]*180.0/M_PI,
	 LastSFP->theta, LastSFP->dtheta, LastSFP->ddtheta);

      SetParametersWithInitPosInitSpeed
	(FootTrajectoryGenerationStandard::OMEGA_AXIS,
	 TimeInterval,0.0*180.0/M_PI,
	 LastSFP->omega, LastSFP->domega);
      SetParametersWithInitPosInitSpeed
	(FootTrajectoryGenerationStandard::OMEGA2_AXIS,
	 TimeInterval,2*0.0*180.0/M_PI,
	 LastSFP->omega2, LastSFP->domega2);

      for(int k = 1; k<=(int)(QP_T_/m_SamplingPeriod);k++)
	{
	  if (CurrentSupport.Foot == LEFT)
	    {
	      UpdateFootPosition
		(FinalLeftFootTraj_deq,
		 FinalRightFootTraj_deq,
		 CurrentIndex,k,
		 LocalInterpolationStartTime,
		 UnlockedSwingPeriod,
		 StepType, -1);
	    }
	  else
	    {
	      UpdateFootPosition
		(FinalRightFootTraj_deq,
		 FinalLeftFootTraj_deq,
		 CurrentIndex,k,
		 LocalInterpolationStartTime,
		 UnlockedSwingPeriod,
		 StepType, 1);
	    }
	  FinalLeftFootTraj_deq[CurrentIndex+k].time =
	    FinalRightFootTraj_deq[CurrentIndex+k].time =
	    Time+k*m_SamplingPeriod;
	}
    }
  else if (CurrentSupport.Phase == DS ||
	   Time+3.0/2.0*QP_T_ > CurrentSupport.TimeLimit)
    {
      for(int k = 1;
	  k<=(int)(QP_T_/m_SamplingPeriod);
	  k++)
        {
          FinalRightFootTraj_deq[CurrentIndex+k] =
	    FinalRightFootTraj_deq[CurrentIndex+k-1];
          FinalLeftFootTraj_deq [CurrentIndex+k] =
	    FinalLeftFootTraj_deq [CurrentIndex+k-1];

          FinalLeftFootTraj_deq [CurrentIndex+k].dx      = 0.0 ;
          FinalLeftFootTraj_deq [CurrentIndex+k].dy      = 0.0 ;
          FinalLeftFootTraj_deq [CurrentIndex+k].dz      = 0.0 ;
          FinalLeftFootTraj_deq [CurrentIndex+k].domega  = 0.0 ;
          FinalLeftFootTraj_deq [CurrentIndex+k].domega2 = 0.0 ;
          FinalLeftFootTraj_deq [CurrentIndex+k].dtheta  = 0.0 ;

          FinalLeftFootTraj_deq [CurrentIndex+k].ddx       = 0.0 ;
          FinalLeftFootTraj_deq [CurrentIndex+k].ddy       = 0.0 ;
          FinalLeftFootTraj_deq [CurrentIndex+k].ddz       = 0.0 ;
          FinalLeftFootTraj_deq [CurrentIndex+k].ddomega   = 0.0 ;
          FinalLeftFootTraj_deq [CurrentIndex+k].ddomega2  = 0.0 ;
          FinalLeftFootTraj_deq [CurrentIndex+k].ddtheta   = 0.0 ;

          FinalRightFootTraj_deq [CurrentIndex+k].dx      = 0.0 ;
          FinalRightFootTraj_deq [CurrentIndex+k].dy      = 0.0 ;
          FinalRightFootTraj_deq [CurrentIndex+k].dz      = 0.0 ;
          FinalRightFootTraj_deq [CurrentIndex+k].domega  = 0.0 ;
          FinalRightFootTraj_deq [CurrentIndex+k].domega2 = 0.0 ;
          FinalRightFootTraj_deq [CurrentIndex+k].dtheta  = 0.0 ;

          FinalRightFootTraj_deq [CurrentIndex+k].ddx       = 0.0 ;
          FinalRightFootTraj_deq [CurrentIndex+k].ddy       = 0.0 ;
          FinalRightFootTraj_deq [CurrentIndex+k].ddz       = 0.0 ;
          FinalRightFootTraj_deq [CurrentIndex+k].ddomega   = 0.0 ;
          FinalRightFootTraj_deq [CurrentIndex+k].ddomega2  = 0.0 ;
          FinalRightFootTraj_deq [CurrentIndex+k].ddtheta   = 0.0 ;

          FinalLeftFootTraj_deq[CurrentIndex+k].time =
	    FinalRightFootTraj_deq[CurrentIndex+k].time = Time+k*m_SamplingPeriod;
          FinalLeftFootTraj_deq[CurrentIndex+k].stepType =
	    FinalRightFootTraj_deq[CurrentIndex+k].stepType = 10;
        }
    }

}

void OnLineFootTrajectoryGeneration::
interpolate_feet_positions
(double Time, unsigned CurrentIndex,
 const support_state_t & CurrentSupport,
 std::vector<double> FootStepX,
 std::vector<double> FootStepY,
 std::vector<double> FootStepYaw,
 deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
 deque<FootAbsolutePosition> & FinalRightFootTraj_deq)
{
  --CurrentIndex;
  int StepType = 1;
  FootAbsolutePosition * LastSFP; //LastSwingFootPosition
  if(CurrentSupport.Foot == LEFT)
    {
      LastSFP = &(FinalRightFootTraj_deq[CurrentIndex]);
    }
  else
    {
      LastSFP = &(FinalLeftFootTraj_deq[CurrentIndex]);
    }

  if(CurrentSupport.Phase == SS && Time+m_TDouble < CurrentSupport.TimeLimit)
    {
      double LocalInterpolationStartTime = Time-CurrentSupport.StartTime;
      //determine coefficients of interpolation polynome
      double ModulationSupportCoefficient = 0.9;
      double UnlockedSwingPeriod = m_TSingle * ModulationSupportCoefficient;
      double EndOfLiftOff = (m_TSingle-UnlockedSwingPeriod)*0.5;
      double SwingTimePassed = 0.0;
      if(LocalInterpolationStartTime>EndOfLiftOff)
	SwingTimePassed = LocalInterpolationStartTime-EndOfLiftOff;

      //Set parameters for current polynomial
      double TimeInterval = UnlockedSwingPeriod-SwingTimePassed;
      SetParameters(FootTrajectoryGenerationStandard::X_AXIS,
		    TimeInterval,FootStepX[CurrentSupport.StepNumber],
		    LastSFP->x, LastSFP->dx, LastSFP->ddx);
      SetParameters(FootTrajectoryGenerationStandard::Y_AXIS,
		    TimeInterval,FootStepY[CurrentSupport.StepNumber],
		    LastSFP->y, LastSFP->dy, LastSFP->ddy);
      if(LocalInterpolationStartTime<0.001)
	{
	  SetParameters(FootTrajectoryGenerationStandard::Z_AXIS,
			m_TSingle,/*m_AnklePositionLeft[2]*/0.0,
			LastSFP->z, LastSFP->dz, LastSFP->ddz);
	}

      SetParameters
	(FootTrajectoryGenerationStandard::THETA_AXIS,
	 TimeInterval, FootStepYaw[CurrentSupport.StepNumber]*180.0/M_PI,
	 LastSFP->theta, LastSFP->dtheta, LastSFP->ddtheta);

      SetParametersWithInitPosInitSpeed
	(FootTrajectoryGenerationStandard::OMEGA_AXIS,
	 TimeInterval,0.0*180.0/M_PI,
	 LastSFP->omega, LastSFP->domega);
      SetParametersWithInitPosInitSpeed
	(FootTrajectoryGenerationStandard::OMEGA2_AXIS,
	 TimeInterval,2*0.0*180.0/M_PI,
	 LastSFP->omega2, LastSFP->domega2);

      for(int k = 1; k<=(int)(QP_T_/m_SamplingPeriod);k++)
	{
	  if (CurrentSupport.Foot == LEFT)
	    {
	      UpdateFootPosition(FinalLeftFootTraj_deq,
				 FinalRightFootTraj_deq,
				 CurrentIndex,k,
				 LocalInterpolationStartTime,
				 UnlockedSwingPeriod,
				 StepType, -1);
	    }
	  else
	    {
	      UpdateFootPosition(FinalRightFootTraj_deq,
				 FinalLeftFootTraj_deq,
				 CurrentIndex,k,
				 LocalInterpolationStartTime,
				 UnlockedSwingPeriod,
				 StepType, 1);
	    }
	  FinalLeftFootTraj_deq[CurrentIndex+k].time =
	    FinalRightFootTraj_deq[CurrentIndex+k].time = Time+k*m_SamplingPeriod;
	}
    }
  else if (CurrentSupport.Phase == DS ||
	   Time+m_TDouble >= CurrentSupport.TimeLimit)
    {
      for(int k = 1; k<=(int)(m_TDouble/m_SamplingPeriod);k++)
	{
	  FinalRightFootTraj_deq[CurrentIndex+k] =
	    FinalRightFootTraj_deq[CurrentIndex+k-1];
	  FinalLeftFootTraj_deq [CurrentIndex+k] =
	    FinalLeftFootTraj_deq [CurrentIndex+k-1];

	  FinalLeftFootTraj_deq [CurrentIndex+k].dx      = 0.0 ;
	  FinalLeftFootTraj_deq [CurrentIndex+k].dy      = 0.0 ;
	  FinalLeftFootTraj_deq [CurrentIndex+k].dz      = 0.0 ;
	  FinalLeftFootTraj_deq [CurrentIndex+k].domega  = 0.0 ;
	  FinalLeftFootTraj_deq [CurrentIndex+k].domega2 = 0.0 ;
	  FinalLeftFootTraj_deq [CurrentIndex+k].dtheta  = 0.0 ;

	  FinalLeftFootTraj_deq [CurrentIndex+k].ddx       = 0.0 ;
	  FinalLeftFootTraj_deq [CurrentIndex+k].ddy       = 0.0 ;
	  FinalLeftFootTraj_deq [CurrentIndex+k].ddz       = 0.0 ;
	  FinalLeftFootTraj_deq [CurrentIndex+k].ddomega   = 0.0 ;
	  FinalLeftFootTraj_deq [CurrentIndex+k].ddomega2  = 0.0 ;
	  FinalLeftFootTraj_deq [CurrentIndex+k].ddtheta   = 0.0 ;

	  FinalRightFootTraj_deq [CurrentIndex+k].dx      = 0.0 ;
	  FinalRightFootTraj_deq [CurrentIndex+k].dy      = 0.0 ;
	  FinalRightFootTraj_deq [CurrentIndex+k].dz      = 0.0 ;
	  FinalRightFootTraj_deq [CurrentIndex+k].domega  = 0.0 ;
	  FinalRightFootTraj_deq [CurrentIndex+k].domega2 = 0.0 ;
	  FinalRightFootTraj_deq [CurrentIndex+k].dtheta  = 0.0 ;

	  FinalRightFootTraj_deq [CurrentIndex+k].ddx       = 0.0 ;
	  FinalRightFootTraj_deq [CurrentIndex+k].ddy       = 0.0 ;
	  FinalRightFootTraj_deq [CurrentIndex+k].ddz       = 0.0 ;
	  FinalRightFootTraj_deq [CurrentIndex+k].ddomega   = 0.0 ;
	  FinalRightFootTraj_deq [CurrentIndex+k].ddomega2  = 0.0 ;
	  FinalRightFootTraj_deq [CurrentIndex+k].ddtheta   = 0.0 ;

	  FinalLeftFootTraj_deq[CurrentIndex+k].time =
	    FinalRightFootTraj_deq[CurrentIndex+k].time = Time+k*m_SamplingPeriod;
	  FinalLeftFootTraj_deq[CurrentIndex+k].stepType =
	    FinalRightFootTraj_deq[CurrentIndex+k].stepType = 10;
	}
    }
}

/*
 * Copyright 2010, 
 *
 * Andrei   Herdt
 * Francois Keith
 * Olivier  Stasse
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
/*
 * OrientationsPreview.cpp
 */

#include <iostream>
#include <fstream>

#include <ZMPRefTrajectoryGeneration/OrientationsPreview.h>

using namespace PatternGeneratorJRL;
using namespace std;

const double OrientationsPreview::M_EPS = 0.00000001;

OrientationsPreview::OrientationsPreview( CjrlJoint *aRootJoint)
{

  m_lLimitLeftHipYaw = aRootJoint->childJoint(1)->lowerBound(0);//-30.0/180.0*M_PI;
  m_uLimitLeftHipYaw  = aRootJoint->childJoint(1)->upperBound(0);//45.0/180.0*M_PI;
  m_lLimitRightHipYaw = aRootJoint->childJoint(0)->lowerBound(0);//-45.0/180.0*M_PI;
  m_uLimitRightHipYaw = aRootJoint->childJoint(0)->upperBound(0);//30.0/180.0*M_PI;

  m_uvLimitFoot = fabs(aRootJoint->childJoint(0)->upperVelocityBound(0));

  //Acceleration limit not given by HRP2JRLmain.wrl
  m_uaLimitHipYaw = 0.1;
  //Maximal cross angle between the feet
  m_uLimitFeet = 5.0/180.0*M_PI;
}


OrientationsPreview::~OrientationsPreview()
{
}


void
OrientationsPreview::preview_orientations(double Time,
    const reference_t & Ref,
    double StepDuration, const support_state_t & CurrentSupport,
    std::deque<FootAbsolutePosition> & LeftFootAbsolutePositions,
    std::deque<FootAbsolutePosition> & RightFootAbsolutePositions,
    std::deque<double> & PreviewedSupportAngles)
{

  verify_acceleration_hip_joint(Ref, CurrentSupport);

  const FootAbsolutePosition & LeftFoot = LeftFootAbsolutePositions.back();
  const FootAbsolutePosition & RightFoot = RightFootAbsolutePositions.back();

  bool TrunkVelOK = false;
  bool TrunkAngleOK = false;

  // In case of double support the next support angle is fixed
  // ds -> FirstFootPreviewed == 0
  // ss -> FirstFootPreviewed == 1
  double FirstFootPreviewed = 0;

  m_signRotVelTrunk = (TrunkStateT_.yaw[1] < 0.0)?-1.0:1.0;

  unsigned StepNumber = 0;

  // Fourth order polynomial parameters
  double a,b,c,d,e;

  // Trunkangle at the end of the current support phase
  double PreviewedTrunkAngleEnd;

  while(!TrunkVelOK)
    {
      // Initialize support orientation:
      // -------------------------------
      double CurrentSupportAngle;
      if (CurrentSupport.Foot == 1)
	  CurrentSupportAngle = LeftFootAbsolutePositions[0].theta*M_PI/180.0;
      else
	  CurrentSupportAngle = RightFootAbsolutePositions[0].theta*M_PI/180.0;


      // (Re)Compute the trunk orientation at the end of the acceleration phase:
      // -----------------------------------------------------------------------
      if(CurrentSupport.Phase != 0)
	{
	  TrunkAngleOK = false;
	  while(!TrunkAngleOK)
	    {
	      if (fabs(TrunkStateT_.yaw[1]-TrunkState_.yaw[1]) > M_EPS)
		{
		  a = TrunkState_.yaw[0];
		  b = TrunkState_.yaw[1];
		  c = 0.0;
		  d = 3.0*(TrunkStateT_.yaw[1]-TrunkState_.yaw[1]) / (m_T*m_T);
		  e = -2.0*d/(3.0*m_T);
		  TrunkStateT_.yaw[0] = a + b*m_T+1.0/2.0*c*m_T*m_T+1.0/3.0*d*m_T*m_T*m_T+1.0/4.0*e*m_T*m_T*m_T*m_T;
		}
	      else
		  TrunkStateT_.yaw[0] = TrunkState_.yaw[0] + TrunkState_.yaw[1]*m_T;
	      //Compute the trunk angle at the end of the support phase
	      m_SupportTimePassed = CurrentSupport.TimeLimit-Time;
	      PreviewedTrunkAngleEnd = TrunkStateT_.yaw[0] + TrunkStateT_.yaw[1]*(m_SupportTimePassed-m_T);

	      //Verify the angle between the support foot and the trunk at the end of the current support period
	      TrunkAngleOK = verify_angle_hip_joint(CurrentSupport, PreviewedTrunkAngleEnd, TrunkState_, TrunkStateT_, CurrentSupportAngle, StepNumber);
	    }
	}
      else//The trunk does not rotate in the DS phase
	{
	  m_SupportTimePassed = CurrentSupport.TimeLimit+SSPeriod_-Time;
	  FirstFootPreviewed = 1;
	  PreviewedSupportAngles.push_back(CurrentSupportAngle);
	  TrunkStateT_.yaw[0] = PreviewedTrunkAngleEnd = TrunkState_.yaw[0];
	}

      // Initialize variables in the orientations preview loop:
      // ------------------------------------------------------
      double PreviousSupportAngle = CurrentSupportAngle;
      double PreviewedSupportFoot = CurrentSupport.Foot;
      double CurrentLeftFootAngle = LeftFoot.theta*M_PI/180.0;
      double CurrentRightFootAngle = RightFoot.theta*M_PI/180.0;
      double CurrentLeftFootVelocity = LeftFoot.dtheta*M_PI/180.0;
      double CurrentRightFootVelocity = RightFoot.dtheta*M_PI/180.0;

      // Preview of orientations:
      // -----------------------.
      for(StepNumber = FirstFootPreviewed;
	  StepNumber <= (unsigned)((int)ceil((m_N+1)*m_T/StepDuration));
	  StepNumber++)
	{
	  PreviewedSupportFoot = -PreviewedSupportFoot;
	  //compute the optimal support orientation
	  double PreviewedSupportAngle = PreviewedTrunkAngleEnd + TrunkStateT_.yaw[1]*SSPeriod_/2.0;

	 verify_velocity_hip_joint(Time, TrunkStateT_,
				   PreviewedSupportFoot,
				   PreviewedSupportAngle,
				   StepNumber, CurrentSupport,
				   CurrentRightFootAngle, CurrentLeftFootAngle,
				   CurrentLeftFootVelocity, CurrentRightFootVelocity);

	  //Check the feet angles to avoid self-collision:
	  if ((double)PreviewedSupportFoot*(PreviousSupportAngle-PreviewedSupportAngle)-M_EPS > m_uLimitFeet)
	      PreviewedSupportAngle = PreviousSupportAngle+(double)m_signRotVelTrunk*m_uLimitFeet;
	  //not being able to catch-up for a rectangular DS phase
	  else if (fabs(PreviewedSupportAngle-PreviousSupportAngle) > m_uvLimitFoot*SSPeriod_)
	      PreviewedSupportAngle = PreviousSupportAngle+(double)PreviewedSupportFoot * m_uvLimitFoot*(SSPeriod_-m_T);

	  TrunkAngleOK = verify_angle_hip_joint( CurrentSupport, PreviewedTrunkAngleEnd,
                                                 TrunkState_, TrunkStateT_,
                                                 CurrentSupportAngle, StepNumber);
	  if(!TrunkAngleOK)
	    {
	    PreviewedSupportAngles.clear();
	    break;
	  }
	  else
	    PreviewedSupportAngles.push_back(PreviewedSupportAngle);

	  //Prepare for the next step
	  PreviewedTrunkAngleEnd = PreviewedTrunkAngleEnd + SSPeriod_*TrunkStateT_.yaw[1];
	  PreviousSupportAngle = PreviewedSupportAngle;

	  if(PreviewedSupportFoot == 1)
	    CurrentLeftFootAngle = PreviewedSupportAngle;
	  else
	    CurrentRightFootAngle = PreviewedSupportAngle;

	  TrunkVelOK = true;
	}
    }
}


void
OrientationsPreview::verify_acceleration_hip_joint(const reference_t &Ref,
						       const support_state_t & CurrentSupport)
{
  if(CurrentSupport.Phase!=0)
      //Verify change in velocity against the maximal acceleration
      if(fabs(Ref.local.yaw-TrunkState_.yaw[1]) > 2.0/3.0*m_T*m_uaLimitHipYaw)
	{
	  double signRotAccTrunk = (Ref.local.yaw-TrunkState_.yaw[1] < 0.0)?-1.0:1.0;
	  TrunkStateT_.yaw[1] = TrunkState_.yaw[1] + signRotAccTrunk * 2.0/3.0*m_T* m_uaLimitHipYaw;
	}
      else
	TrunkStateT_.yaw[1] = Ref.local.yaw;
  else//No rotations in a double support phase
      TrunkStateT_.yaw[1] = 0.0;
}


bool
OrientationsPreview::verify_angle_hip_joint(support_state_t CurrentSupport,
                                                double PreviewedTrunkAngleEnd,
						const COMState &TrunkState_, COMState &TrunkStateT_,
						double CurrentSupportFootAngle,
						unsigned StepNumber)
{

  //Which limitation is relevant in the current situation?
  double uJointLimit, lJointLimit, JointLimit;
  if(CurrentSupport.Foot == 1)
    {
      uJointLimit = m_uLimitLeftHipYaw;
      lJointLimit = m_lLimitLeftHipYaw;
    }
  else
    {
      uJointLimit = m_uLimitRightHipYaw;
      lJointLimit = m_lLimitRightHipYaw;
    }
  JointLimit = (TrunkStateT_.yaw[1] < 0.0)?lJointLimit:uJointLimit;


  if (fabs(PreviewedTrunkAngleEnd - CurrentSupportFootAngle)>fabs(JointLimit))
    {
      TrunkStateT_.yaw[1] = (CurrentSupportFootAngle+JointLimit-TrunkState_.yaw[0]-TrunkState_.yaw[1]*m_T/2.0)/(m_SupportTimePassed+StepNumber*SSPeriod_-m_T/2.0);
      return false;
    }
  else
      return true;
}


void
OrientationsPreview::verify_velocity_hip_joint(double Time, COMState &,
						   double PreviewedSupportFoot, double PreviewedSupportAngle,
						   unsigned StepNumber, support_state_t CurrentSupport,
						   double CurrentRightFootAngle, double CurrentLeftFootAngle,
						   double CurrentLeftFootVelocity, double CurrentRightFootVelocity)
{
  double CurrentAngle;
  if(PreviewedSupportFoot==1)
    CurrentAngle = CurrentLeftFootAngle;
  else
    CurrentAngle = CurrentRightFootAngle;

  double a,b,c,d,T;

  //To be implemented
  //For the
  if(StepNumber>0 && CurrentSupport.Phase==1)
    {
      //verify the necessary, maximal, relative foot velocity
      double MeanFootVelDifference = (PreviewedSupportAngle-CurrentAngle)/(SSPeriod_-m_T);
      //If necessary reduce the velocity to the maximum
      if (3.0/2.0*fabs(MeanFootVelDifference) > m_uvLimitFoot)
	{
	  MeanFootVelDifference = 2.0/3.0*(double)m_signRotVelTrunk * m_uvLimitFoot;
	  //Compute the resulting angle
	  PreviewedSupportAngle = CurrentAngle+MeanFootVelDifference*(SSPeriod_-m_T);
	}
    }
  else if((StepNumber==0 && CurrentSupport.Phase==1) || (StepNumber==1 && CurrentSupport.Phase==0))
    {
      T = CurrentSupport.TimeLimit-Time-m_T;
      //Previewed polynome
      a = CurrentAngle;
      if(PreviewedSupportFoot==1)
	b = CurrentLeftFootVelocity;
      else
	b = CurrentRightFootVelocity;
      c = (3.0*PreviewedSupportAngle-3.0*a-2.0*b*T)/(T*T);
      d = (-b*T+2*a-2*PreviewedSupportAngle)/(T*T*T);

      //maximal speed violated
      if(df(a,b,c,d,-1.0/3.0*c/d)>m_uvLimitFoot)
	{
	  a = 0;
	  c = -1.0/(2.0*T)*(2.0*b-2.0*m_uvLimitFoot+2.0*sqrt(m_uvLimitFoot*m_uvLimitFoot-b*m_uvLimitFoot));
	  d = (-2.0*c-b/T)/(3.0*T);
	  PreviewedSupportAngle = f(a,b,c,d,T);
	}
    }

}


void
OrientationsPreview::interpolate_trunk_orientation(double time, int CurrentIndex,
                                                   double NewSamplingPeriod,
                                                    const support_state_t & CurrentSupport,
                                                    deque<COMState> & FinalCOMTraj_deq)
{
  if(CurrentSupport.Phase == 1 && time+3.0/2.0*m_T < CurrentSupport.TimeLimit)
    {
      //Fourth order polynomial parameters
      double a =  TrunkState_.yaw[1];
      double c = 3.0*(TrunkStateT_.yaw[1]-TrunkState_.yaw[1])/(m_T*m_T);
      double d = -2.0*c/(3.0*m_T);

      double tT;
      double Theta = TrunkState_.yaw[0];

      FinalCOMTraj_deq[CurrentIndex].yaw[0] = TrunkState_.yaw[0];
      //Interpolate the
      for(int k = 1; k<=(int)(m_T/NewSamplingPeriod);k++)
        {
          tT = (double)k*NewSamplingPeriod;
          //interpolate the orientation of the trunk
          if(fabs(TrunkStateT_.yaw[1]-TrunkState_.yaw[1])-0.000001 > 0)
            {
              TrunkState_.yaw[0] = (((1.0/4.0*d*tT+1.0/3.0*c)*
                                      tT)*tT+a)*tT+Theta;
              TrunkState_.yaw[1] = ((d*tT+c)*tT)*tT+a;
              TrunkState_.yaw[2] = (3.0*d*tT+2.0*c)*tT;
            }
          else
            {
              TrunkState_.yaw[0] += NewSamplingPeriod*TrunkStateT_.yaw[1];
            }
          FinalCOMTraj_deq[CurrentIndex+k].yaw[0] = TrunkState_.yaw[0];
        }
    }
  else if (CurrentSupport.Phase == 0 || time+3.0/2.0*m_T > CurrentSupport.TimeLimit)
    {
      for(int k = 0; k<=(int)(m_T/NewSamplingPeriod);k++)
        {
          FinalCOMTraj_deq[CurrentIndex+k].yaw[0] = TrunkState_.yaw[0];
        }
    }

}

double OrientationsPreview::f(double a,double b,double c,double d,double x){return a+b*x+c*x*x+d*x*x*x;}
double OrientationsPreview::df(double ,double b,double c,double d,double x){return b+2*c*x+3.0*d*x*x;}




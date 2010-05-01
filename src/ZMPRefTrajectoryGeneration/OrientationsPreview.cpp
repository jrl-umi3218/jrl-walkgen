/*
 * OrientationsPreview.cpp
 *
 *  Created on: Apr 26, 2010
 *      Author: andrei
 */

#include <iostream>
#include <fstream>
#include <Debug.h>



#include <ZMPRefTrajectoryGeneration/OrientationsPreview.h>

using namespace PatternGeneratorJRL;
using namespace std;

OrientationsPreview::OrientationsPreview(const double & SamplingPeriod,
		const unsigned int & SamplingsPreviewed, const double & SSPeriod)
{
	m_T = SamplingPeriod;
	m_N = SamplingsPreviewed;
	m_SSPeriod = SSPeriod;

	//TODO 1: Angular and velocity bounds of the Hip joint should be read from the aHDR object.
	m_lLimitLeftHipYaw = -30.0/180.0*M_PI;
	m_uLimitLeftHipYaw  = 45.0/180.0*M_PI;
	m_lLimitRightHipYaw = -45.0/180.0*M_PI;
	m_uLimitRightHipYaw = 30.0/180.0*M_PI;

	m_uvLimitFoot = 3.54108;

	//Acceleration limit not given by HRP2JRLmain.wrl
	m_uaLimitHipYaw = 0.1;
	//Maximal cross angle between the feet
	m_uLimitFeet = 5.0/180.0*M_PI;



	m_FullDebug = 3;

	//TODO 1: How does ODEBUG/RESETDEBUG get activated?
	if(m_FullDebug>2)
	{
		ofstream aof;
		aof.open("previewOrientations.dat",ofstream::out);
		aof.close();
		aof.open("verifyAccelerationOfHipJoint.dat",ofstream::out);
		aof.close();
	}

	//	RESETDEBUG4("OrientationsPreview.dat");
}

OrientationsPreview::~OrientationsPreview() {
	// TODO Auto-generated destructor stub
}


void OrientationsPreview::previewOrientations(const double &Time,
		double *PreviewedSupportAngles,
		double &AngVelTrunkConst, double &PreviewedTrunkAngleT,
		const COMState_t &TrunkState, const SupportState * Support,
		deque<SupportFeet_t *> &QueueOfSupportFeet,
		deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
		deque<FootAbsolutePosition> &RightFootAbsolutePositions)
{
	//TODO 2: Replace AngVelTrunkConst by TrunkStateT

	deque<FootAbsolutePosition>::iterator lF_it, rF_it;
	lF_it = LeftFootAbsolutePositions.end();
	lF_it--;
	rF_it = RightFootAbsolutePositions.end();
	rF_it--;

	if(m_FullDebug>2)
	{
		ofstream aof;
		aof.open("previewOrientations.dat",ofstream::app);
		aof<<endl<<endl;
		aof<<"Time: "<<Time<<" LeftFootAbsolutePositions[0].theta: "<<LeftFootAbsolutePositions[0].theta<<
				" RightFootAbsolutePositions[0].theta: "<<RightFootAbsolutePositions[0].theta
				<<" Last LeftFootAbsolutePosition: "<<lF_it->theta<<
				" Last RightFootAbsolutePosition: "<<rF_it->theta<<endl;
		aof.close();
	}

	m_TrunkVelOK = false;
	m_FirstPreviewedFoot = 0;
	//PreviewedTrunkAngleT = 0;

	double CurrentLeftFootAngle, CurrentRightFootAngle, CurrentLeftFootVelocity, CurrentRightFootVelocity;

	m_signRotVelTrunk = (AngVelTrunkConst < 0.0)?-1.0:1.0;

	unsigned int StepNumber = 0;

	while(!m_TrunkVelOK)
	{

		//Initialize the preview loop
		if (Support->CurrentSupportFoot == 1)
		{//TODO 2: Too many variables
			m_CurrentSupportAngle = LeftFootAbsolutePositions[0].theta;

			//m_PreviewedSupportAngle = RightFootAbsolutePositions[0].theta;
		}
		else
		{
			m_CurrentSupportAngle = RightFootAbsolutePositions[0].theta;
			//m_PreviewedSupportAngle = LeftFootAbsolutePositions[0].theta;
		}



		if(Support->CurrentSupportPhase != 0)
		{
			m_TrunkAngleOK = false;
			while(!m_TrunkAngleOK)
			{
				//(Re)Compute the trunk angle at the end of the acceleration phase
				if (fabs(AngVelTrunkConst-TrunkState.yaw[1]) > M_EPS)
				{
					m_a = TrunkState.yaw[0];
					m_b = TrunkState.yaw[1];
					m_c = 0.0;
					m_d = 3.0*(AngVelTrunkConst-TrunkState.yaw[1]) / (m_T*m_T);
					m_e = -2.0*m_d/(3.0*m_T);
					PreviewedTrunkAngleT = m_a + m_b*m_T+1.0/2.0*m_c*m_T*m_T+1.0/3.0*m_d*m_T*m_T*m_T+1.0/4.0*m_e*m_T*m_T*m_T*m_T;

					if(m_FullDebug>2)
					{
						ofstream aof;
						aof.open("previewOrientations.dat",ofstream::app);
						aof<<" Trunk accelerated because AngVelTrunkConst = "<<AngVelTrunkConst<<" TrunkState.yaw[1] = "<<TrunkState.yaw[1]<<endl;
						aof.close();
					}
				}
				else
				{
					PreviewedTrunkAngleT = TrunkState.yaw[0] + TrunkState.yaw[1]*m_T;

					if(m_FullDebug>2)
					{
						ofstream aof;
						aof.open("previewOrientations.dat",ofstream::app);
						aof<<" Trunk velocity constant because AngVelTrunkConst = "<<AngVelTrunkConst<<" TrunkState.yaw[1] = "<<TrunkState.yaw[1]<<endl;
						aof.close();
					}

				}
				//Compute the trunk angle at the end of the support phase
				m_SupportTimePassed = Support->CurrentTimeLimit-Time;
				m_PreviewedTrunkAngleEnd = PreviewedTrunkAngleT + AngVelTrunkConst*(m_SupportTimePassed-m_T);


				//Verify the angle between the support foot and the trunk at the end of the current support period
				m_TrunkAngleOK = verifyAngleOfHipJoint(AngVelTrunkConst,
						PreviewedTrunkAngleT, Support,
						TrunkState, m_CurrentSupportAngle, StepNumber);
			}

		}
		else//The trunk does not rotate in the DS phase
		{
			m_SupportTimePassed = Support->CurrentTimeLimit+m_SSPeriod-Time;
			m_FirstPreviewedFoot = 1;
			PreviewedSupportAngles[0] = m_CurrentSupportAngle;
			PreviewedTrunkAngleT = m_PreviewedTrunkAngleEnd = TrunkState.yaw[0];
		}

		//TODO : SupportState can replace some of the native members

		if(m_FullDebug>2)
		{
			ofstream aof;
			aof.open("previewOrientations.dat",ofstream::app);
			//			aof<<"fabs(AngVelTrunkConst-TrunkState.yaw[1]): "<<fabs(AngVelTrunkConst-TrunkState.yaw[1])<<" PreviewedTrunkAngleT: "<<PreviewedTrunkAngleT<<
			//					" m_d "<<m_d<<" m_e "<<m_e<<" 1/3*c*m_T*m_T*m_T "<<1.0/3.0*m_d*m_T*m_T*m_T<<" 1/4*m_e*m_T*m_T*m_T*m_T: "
			//					<<1.0/4.0*m_e*m_T*m_T*m_T*m_T<<endl;
			aof<<" PreviewedTrunkAngleT: "<<PreviewedTrunkAngleT<<
					" m_PreviewedTrunkAngleEnd: "<<m_PreviewedTrunkAngleEnd<<endl;
			aof.close();
		}

		m_PreviousSupportAngle = m_CurrentSupportAngle;
		m_PreviewedSupportFoot = Support->CurrentSupportFoot;


		if(m_FullDebug>2)
		{
			ofstream aof;
			aof.open("previewOrientations.dat",ofstream::app);
			aof<<"Preview loop initialized: "<<" m_PreviewedSupportFoot: "<<m_PreviewedSupportFoot<<
					" m_PreviousSupportAngle: "<<m_PreviousSupportAngle<<endl;
			aof.close();
		}

		CurrentLeftFootAngle = lF_it->theta;
		CurrentRightFootAngle = rF_it->theta;
		CurrentLeftFootVelocity = lF_it->dtheta;
		CurrentRightFootVelocity = rF_it->dtheta;

		//Preview
		for(StepNumber = m_FirstPreviewedFoot; StepNumber <= (int)ceil((m_N+1)*m_T/Support->SSPeriod); StepNumber++)
		{
			m_PreviewedSupportFoot = -m_PreviewedSupportFoot;
			//compute the optimal support orientation
			m_PreviewedSupportAngle = m_PreviewedTrunkAngleEnd + AngVelTrunkConst*m_SSPeriod/2.0;

			if(m_FullDebug>2)
			{
				ofstream aof;
				aof.open("previewOrientations.dat",ofstream::app);
				aof<<"PreviewedTrunkAngleEnd: "<<m_PreviewedTrunkAngleEnd<<
						" AngVelTrunkConst "<<AngVelTrunkConst<<endl;
				aof.close();
			}

			verifyVelocityOfHipJoint(Time, AngVelTrunkConst,
					m_PreviewedSupportFoot, StepNumber, Support,
					CurrentRightFootAngle, CurrentLeftFootAngle,
					CurrentLeftFootVelocity, CurrentRightFootVelocity);

			//verifyVelocityOfHipJoint(Ref, AngVelTrunkConst, TrunkState, Support, StepNumber);

			//Check the feet angles to avoid self-collision:
			if ((double)m_PreviewedSupportFoot*(m_PreviousSupportAngle-m_PreviewedSupportAngle)-M_EPS > m_uLimitFeet)
			{
				m_PreviewedSupportAngle = m_PreviousSupportAngle+(double)m_signRotVelTrunk*m_uLimitFeet;

				if(m_FullDebug>2)
				{
					ofstream aof;
					aof.open("previewOrientations.dat",ofstream::app);
					aof<<"Self collision occured - m_PreviewedSupportAngle: "<<m_PreviewedSupportAngle
							<<" m_PreviousSupportAngle: "<<m_PreviousSupportAngle<<endl;
					aof.close();
				}
			}
			//not being able to catch-up for a rectangular DS phase
			else if (fabs(m_PreviewedSupportAngle-m_PreviousSupportAngle) > m_uvLimitFoot*m_SSPeriod)
			{
				m_PreviewedSupportAngle = m_PreviousSupportAngle+(double)m_PreviewedSupportFoot * m_uvLimitFoot*(m_SSPeriod-m_T);

				if(m_FullDebug>2)
				{
					ofstream aof;
					aof.open("previewOrientations.dat",ofstream::app);
					aof<<"Angle too big for ds phase - m_PreviewedSupportAngle: "<<m_PreviewedSupportAngle
							<<" m_PreviousSupportAngle: "<<m_PreviousSupportAngle<<endl;
					aof.close();
				}
			}

			m_TrunkAngleOK = verifyAngleOfHipJoint(AngVelTrunkConst,
					PreviewedTrunkAngleT, Support,
					TrunkState, m_CurrentSupportAngle, StepNumber);
			if(!m_TrunkAngleOK)
				break;
			else
				PreviewedSupportAngles[StepNumber] = m_PreviewedSupportAngle;


			if(m_FullDebug>2)
			{
				ofstream aof;
				aof.open("previewOrientations.dat",ofstream::app);
				aof<<"PreviewedSupportAngles: "<<PreviewedSupportAngles[StepNumber-m_FirstPreviewedFoot]<<
						" StepNumber "<<StepNumber<<
						" m_FirstPreviewedFoot "<<m_FirstPreviewedFoot<<endl;
				aof.close();
			}



			//Prepare for the next step
			m_PreviewedTrunkAngleEnd = m_PreviewedTrunkAngleEnd + m_SSPeriod*AngVelTrunkConst;
			m_PreviousSupportAngle = m_PreviewedSupportAngle;

			if(m_PreviewedSupportFoot == 1)
				CurrentLeftFootAngle = m_PreviewedSupportAngle;
			else
				CurrentRightFootAngle = m_PreviewedSupportAngle;

			m_TrunkVelOK = true;
		}
	}
}

void OrientationsPreview::verifyAccelerationOfHipJoint(const ReferenceAbsoluteVelocity_t &Ref, double &AngVelTrunkConst,
		const COMState_t &TrunkState, const SupportState * Support)
{
	if(Support->CurrentSupportPhase!=0)
	{
		//Verify change in velocity against the maximal acceleration
		if(fabs(Ref.dYaw-TrunkState.yaw[1]) > 2.0/3.0*m_T*m_uaLimitHipYaw)
		{
			double signRotAccTrunk = (Ref.dYaw-TrunkState.yaw[1] < 0.0)?-1.0:1.0;
			AngVelTrunkConst = TrunkState.yaw[1] + signRotAccTrunk * 2.0/3.0*m_T* m_uaLimitHipYaw;


			if(m_FullDebug>2)
			{
				ofstream aof;
				aof.open("verifyAccelerationOfHipJoint.dat",ofstream::app);
				aof<<" AngVelTrunkConst: "<<AngVelTrunkConst <<" "
						<<" Ref.dYaw: "<<Ref.dYaw <<" "
						<<" m_signRotAccTrunk: "<<m_signRotAccTrunk <<" "
						<<" 2.0/3.0*m_T* m_uaLimitHipYaw: "<<2.0/3.0*m_T* m_uaLimitHipYaw <<" "
						<<endl;
				aof.close();
			}
		}
		else
			AngVelTrunkConst = Ref.dYaw;

	}
	else//No rotations in a double support phase
	{
		AngVelTrunkConst = 0.0;
	}
}


bool OrientationsPreview::verifyAngleOfHipJoint(double &AngVelTrunkConst,
		double &PreviewedTrunkAngleT, const SupportState * Support,
		const COMState_t &TrunkState, double CurrentSupportFootAngle,
		unsigned int StepNumber)
{//TODO 2: The current support angle is taken from QueueOfSupportFeet which should be replaced by QueueOfSupportStates

	//Which limitation is relevant in the current situation?
	double uJointLimit, lJointLimit, JointLimit;
	if(Support->CurrentSupportFoot == 1)
	{
		uJointLimit = m_uLimitLeftHipYaw;
		lJointLimit = m_lLimitLeftHipYaw;
	}
	else
	{
		uJointLimit = m_uLimitRightHipYaw;
		lJointLimit = m_lLimitRightHipYaw;
	}
	JointLimit = (AngVelTrunkConst < 0.0)?lJointLimit:uJointLimit;


	if (fabs(m_PreviewedTrunkAngleEnd - CurrentSupportFootAngle)>fabs(JointLimit))
	{
		AngVelTrunkConst = (CurrentSupportFootAngle+JointLimit-TrunkState.yaw[0]-TrunkState.yaw[1]*m_T/2.0)/(m_SupportTimePassed+StepNumber*m_SSPeriod-m_T/2.0);

		if(m_FullDebug>2)
		{
			ofstream aof;
			aof.open("previewOrientations.dat",ofstream::app);
			aof<<"Limitation reached - new AngVelTrunkConst :"<<AngVelTrunkConst<<endl;
			aof.close();
		}
		return false;
	}
	else
	{
		return true;
	}

}


void OrientationsPreview::verifyVelocityOfHipJoint(const double &Time, double &AngVelTrunkConst,
		const double &PreviewedSupportFoot, const unsigned int &StepNumber,
		const SupportState * Support,
		const double &CurrentRightFootAngle, const double &CurrentLeftFootAngle,
		const double &CurrentLeftFootVelocity,
		const double &CurrentRightFootVelocity)
{
	double CurrentAngle;
	if(PreviewedSupportFoot==1)
		CurrentAngle = CurrentLeftFootAngle;
	else
		CurrentAngle = CurrentRightFootAngle;

	double a,b,c,d,T;
	//To be implemented
	//For the
	if(StepNumber>0 && Support->CurrentSupportPhase==1)
	{
		//verify the necessary, maximal, relative foot velocity
		m_MeanFootVelDifference = (m_PreviewedSupportAngle-CurrentAngle)/(m_SSPeriod-m_T);
		//If necessary reduce the velocity to the maximum
		if (3.0/2.0*fabs(m_MeanFootVelDifference) > m_uvLimitFoot)
		{
			m_MeanFootVelDifference = 2.0/3.0*(double)m_signRotVelTrunk * m_uvLimitFoot;
			//Compute the resulting angle
			m_PreviewedSupportAngle = CurrentAngle+m_MeanFootVelDifference*(m_SSPeriod-m_T);
		}
	}
	else if(StepNumber==0 && Support->CurrentSupportPhase==1 || StepNumber==1 && Support->CurrentSupportPhase==0)
	{

		T = Support->CurrentTimeLimit-Time-m_T;
		//Previewed polynome
		a = CurrentAngle;
		if(PreviewedSupportFoot==1)
			b = CurrentLeftFootVelocity;
		else
			b = CurrentRightFootVelocity;
		c = (3.0*m_PreviewedSupportAngle-3.0*a-2.0*b*T)/(T*T);
		d = (-b*T+2*a-2*m_PreviewedSupportAngle)/(T*T*T);

		//maximal speed violated
		if(df(a,b,c,d,T)>m_uvLimitFoot)
		{
			a = 0;
			b = b;
			c = -1.0/(2.0*T)*(2.0*b-2.0*m_uvLimitFoot+2.0*sqrt(m_uvLimitFoot*m_uvLimitFoot-b*m_uvLimitFoot));
			d = (-2.0*c-b/T)/(3.0*T);
			m_PreviewedSupportAngle = f(a,b,c,d,T);
		}
//		f(0) = 0, df(0)=v0, df(1/3*c/d)=vmax, df(T)=0
//		a = 0, b = v0, c = -1/(2*T)*(2*v0-2*vmax+2*sqrt(vmax^2-vo*vmax), d = (-2*c-v0/T)/(3*T);
	}

}

double OrientationsPreview::f(double a,double b,double c,double d,double x){return a+b*x+c*x*x+d*x*x*x;}
double OrientationsPreview::df(double a,double b,double c,double d,double x){return b+2*c*x+3.0*d*x*x;}




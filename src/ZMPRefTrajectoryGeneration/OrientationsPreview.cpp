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
		aof.open("OrientationsPreview.dat",ofstream::out);
		aof.close();
		aof.open("verifyAccelerationOfHipJoint.dat",ofstream::out);
		aof.close();
	}

//	RESETDEBUG4("OrientationsPreview.dat");
}

OrientationsPreview::~OrientationsPreview() {
	// TODO Auto-generated destructor stub
}


void OrientationsPreview::previewOrientations(double &Time,
		deque<double> &PreviewedSupportAngles,
		double &AngVelTrunkConst, double &PreviewedTrunkAngle,
		COMState_t &TrunkState, SupportState * Support,
		deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
		deque<FootAbsolutePosition> &RightFootAbsolutePositions)
{
	if(m_FullDebug>2)
	{
		ofstream aof;
		aof.open("OrientationsPreview.dat",ofstream::app);
		aof<<"Time: "<<Time<<endl;
		aof.close();
		//	printf ("Double variables: %f %f %f %f %f %f %f %f %f %f %f \n", PreviewedFeetAngles[0], *AngVelCoH, *PreviewedTrunkAngle,
		//			*CurAngVelCoH, *TrunkAngle, *TimeLimit, *Time, *SupportAngle, *RightFootAngle, *LeftFootAngle, *T);
		//	printf ("Integers: %d %d \n", *SupportFoot, *SupportPhase);
	}
}

void OrientationsPreview::verifyAccelerationOfHipJoint(const ReferenceAbsoluteVelocity_t &Ref, double &AngVelTrunkConst,
		const COMState_t &TrunkState, const SupportState * Support)
{

	//	//Which Limitation is relevant in the current situation (There can be only one)
	//	double uJointLimit, lJointLimit, JointLimit;
	//	if(Support->CurrentSupportFoot == 1)
	//	{
	//		uJointLimit = m_uLimitLeftHipYaw;
	//		lJointLimit = m_lLimitLeftHipYaw;
	//	}
	//	else
	//	{
	//		uJointLimit = m_uLimitRightHipYaw;
	//		lJointLimit = m_lLimitRightHipYaw;
	//	}
	//	if()


	//Verify change in velocity against the maximal acceleration

	if(fabs(Ref.dYaw-TrunkState.yaw[1]) > 2.0/3.0*m_T*m_uaLimitHipYaw)
	{
		double signAcc = (Ref.dYaw-TrunkState.yaw[1] < 0.0)?-1.0:1.0;
		AngVelTrunkConst = TrunkState.yaw[1] + signAcc * 2.0/3.0*m_T* m_uaLimitHipYaw;


		if(m_FullDebug>2)
		{
			ofstream aof;
			aof.open("verifyAccelerationOfHipJoint.dat",ofstream::app);
			aof<<"AngVelTrunkConst :"<<AngVelTrunkConst <<" "
					<<"Ref.dYaw :"<<Ref.dYaw <<" "
					<<"signAcc :"<<signAcc <<" "
					<<"2.0/3.0*m_T* m_uaLimitHipYaw :"<<2.0/3.0*m_T* m_uaLimitHipYaw <<" "
					<<endl;
			aof.close();
		}
	}
	else
		AngVelTrunkConst = Ref.dYaw;



}




/*
 * OrientationsPreview.cpp
 *
 *  Created on: Apr 26, 2010
 *      Author: andrei
 */

#include <ZMPRefTrajectoryGeneration/OrientationsPreview.h>

using namespace PatternGeneratorJRL;
using namespace std;

OrientationsPreview::OrientationsPreview(const double & SamplingPeriod,
		const unsigned int & SamplingsPreviewed,
		const double & SSPeriod)
{
	// TODO Reading the limitations from the VRML File
	m_T = SamplingPeriod;
	m_N = SamplingsPreviewed;
	m_SSPeriod = SSPeriod;
}

OrientationsPreview::~OrientationsPreview() {
	// TODO Auto-generated destructor stub
}


void OrientationsPreview::previewOrientations(double Time,
			deque<double> PreviewedSupportAngles,
			double AngVelTrunk, double PreviewedTrunkAngle,
			/*in:*/ COMState_t TrunkState, SupportState Support,
			deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			deque<FootAbsolutePosition> &RightFootAbsolutePositions)
{

//TODO 1: previewOrientations()
}

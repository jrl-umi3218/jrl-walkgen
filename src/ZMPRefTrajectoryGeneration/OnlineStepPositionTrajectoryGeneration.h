/* This object provides the generation of ZMP and CoM trajectory
using a new formulation of the stability problem.

Copyright (c) 2005-2010, 
Andrei Herdt, Olivier Stasse, Francois Keith, Mehdi Benallegue

JRL-Japan, CNRS/AIST

All rights reserved.

Please see Licence.txt for details on the license.
*/

#ifndef _ONLINE_STEP_POSISTION_TRJECTORY_GENERATION_H_
#define _ONLINE_STEP_POSISTION_TRJECTORY_GENERATION_H_

#include <deque>

#include <PreviewControl/LinearizedInvertedPendulum2D.h>
#include <Mathematics/FootConstraintsAsLinearSystemForVelRef.h>
#include <Mathematics/OptCholesky.h>
#include <ZMPRefTrajectoryGeneration/ZMPVelocityReferencedQP.h>
#include <Mathematics/PLDPSolverHerdt.h> 
#include <PreviewControl/SupportState.h>
#include <FootTrajectoryGeneration/FootTrajectoryGenerationStandard.h>
#include <ZMPRefTrajectoryGeneration/OrientationsPreview.h>


namespace PatternGeneratorJRL
{
	class ZMPDiscretization;
	class  OnlineStepPositionTrajectoryGeneration  : public ZMPVelocityReferencedQP
	{

	public:

		/* constructor. */
		OnlineStepPositionTrajectoryGeneration(SimplePluginManager *lSPM, string DataFile, 
			CjrlHumanoidDynamicRobot *aHS=0);

		/* Default destructor. */
		virtual ~OnlineStepPositionTrajectoryGeneration();

		const std::deque<RelativeFootPosition>& GetStepsPositions() const;
		
		void SetStepsPositions(const std::deque<RelativeFootPosition>&);

		void SetBeta(const double &);
		
		/*true is on CoM velocity reference mode false is on foosteps mode*/
		void SetVelocityMode(bool);

		virtual void OnLine(double time,
		deque<ZMPPosition> & FinalZMPPositions,		
		deque<COMState> & CoMStates,			     
		deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
		deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions);

		
	protected:
		std::deque<RelativeFootPosition> stepPos_;

		bool velocityMode_;

		double m_BetaCache_;

	};
};

#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.h>
#endif

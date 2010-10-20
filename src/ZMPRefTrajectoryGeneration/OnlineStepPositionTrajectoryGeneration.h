/* This object provides the generation of ZMP and CoM trajectory
using a new formulation of the stability problem.

Copyright (c) 2005-2010, 
Mehdi Benallegue, Andrei Herdt, Olivier Stasse, Francois Keith, Mehdi Benallegue

JRL-Japan, CNRS/AIST

All rights reserved.

Please see Licence.txt for details on the license.
*/

#ifndef _ONLINE_STEP_POSISTION_TRJECTORY_GENERATION_H_
#define _ONLINE_STEP_POSISTION_TRJECTORY_GENERATION_H_

#include <deque>

#include <PreviewControl/LinearizedInvertedPendulum2D.h>
#include <Mathematics/FootConstraintsAsLinearSystemForStepPos.h>
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

    /*! Reimplement the interface of SimplePluginManager 
	\param[in] Method: The method to be called.
	\param[in] Args: Arguments of the methods.
       */
      virtual void CallMethod(std::string & Method, std::istringstream &Args);

	  /*! Caution, this overload doesn't aim the same goal as the virtual interface, but to change any step
		in the steps stack, the number of the step to change is set in the aFootAbsolutePosition.stepType*
	  */
	  virtual int OnLineFootChange(double time,
				 FootAbsolutePosition &aFootAbsolutePosition,
				 std::deque<ZMPPosition> & FinalZMPPositions,			     
				 std::deque<COMState> & COMStates,
				 std::deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
				 std::deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
				 StepStackHandler * aStepStackHandler);

	  virtual void OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
			       std::deque<ZMPPosition> & FinalZMPPositions,					     
			       std::deque<COMState> & COMStates,
			       std::deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
			       std::deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
			       bool EndSequence);

	  virtual int buildZMPTrajectoryFromFootTrajectory(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
    		deque<FootAbsolutePosition> &RightFootAbsolutePositions,
    		deque<ZMPPosition> &ZMPRefPositions,
    		deque<COMState> &COMStates,
    		double ConstraintOnX,
    		double ConstraintOnY,
    		double T,
    		 int N);

		
	protected:
		RelativeFootPositionQueue stepPos_;

		bool velocityMode_;

		double m_BetaCache_;

		FootConstraintsAsLinearSystemForStepPos m_fCALS_FP;

	};
};

#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.h>
#endif

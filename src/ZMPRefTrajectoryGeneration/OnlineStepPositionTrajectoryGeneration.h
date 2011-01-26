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

#include <jrl/walkgen/pgtypes.hh>
#include <PreviewControl/LinearizedInvertedPendulum2D.h>
//#include <Mathematics/FootConstraintsAsLinearSystemForStepPos.h>
#include <Mathematics/OptCholesky.h>
#include <ZMPRefTrajectoryGeneration/ZMPVelocityReferencedQP.h>
#include <Mathematics/PLDPSolverHerdt.h> 
//#include <PreviewControl/SupportState.h>
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

		//void SetBeta(const double &);
		
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

	  /*! Change a step position 
		\param[in] aFootAbsolutePosition: The RELATIVE position of the step.
	   Caution, this overload doesn't aim the same goal as the virtual interface, but to change any step
		in the steps stack, the number of the step to change is set in the aFootAbsolutePosition.stepType
	  */
	  virtual int OnLineFootChange(double time,
				 FootAbsolutePosition &aFootAbsolutePosition,
				 std::deque<ZMPPosition> & FinalZMPPositions,			     
				 std::deque<COMState> & COMStates,
				 std::deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
				 std::deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
				 StepStackHandler * aStepStackHandler);
	  
	  /*! Change a step position 
		\param[in] r: The relative position of the new step.
		\param[in] stepNumber: the number of the step to change.
	  */
	  virtual int ChangeStepPosition(const RelativeFootPosition & r,
				unsigned stepNumber);

	   /*! add a step position 
		\param[in] NewRelativeFootPosition: The RELATIVE position of the step.
	   Caution, this overload doesn't aim the same goal as the virtual interface, but to add any step
		in the steps stack.
	  */
	  virtual void OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
			       std::deque<ZMPPosition> & FinalZMPPositions,					     
			       std::deque<COMState> & COMStates,
			       std::deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
			       std::deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
			       bool EndSequence);

	   /*! Add a step position 
		\param[in] r: The relative position of the new step.
		
	  */
	  virtual int AddStepPosition(const RelativeFootPosition & r);

	  /// \name Accessors
	  /// \{
	  /*! Set the velocity reference from external reference */
	  virtual void setVelReference(double dx,double dy, double dyaw);
	  	  
	  /*! Set the velocity reference from string */
	  inline virtual void setVelReference(istringstream &strm)
	  {ZMPVelocityReferencedQP::setVelReference(strm);}
	  
	  /// \}

		
	protected:

		RelativeFootPositionQueue stepPos_;
		bool removeQueueHead;

		bool velocityMode_;

		double betaCache_;
		double yawCache_;

		//FootConstraintsAsLinearSystemForStepPos m_fCALS_FP;

	};
};

#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.h>
#endif

/* This object generate matrix representation of linear
   constraint based on foot position.
   It handles a stack of constraint on a sliding mode 
   for QP solving.

   Copyright (c) 2009, 
   Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

*/

#ifndef _fOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_FOR_STEP_POS_H_
#define _fOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_FOR_STEP_POS_H_

#include <vector>
#include <deque>
#include <string>
#include <sstream>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <robotDynamics/jrlHumanoidDynamicRobot.h>


#include <PGTypes.h>
#include <Mathematics/ConvexHull.h>
#include <Mathematics/FootConstraintsAsLinearSystemForVelRef.h>
#include <SimplePlugin.h>
#include <PreviewControl/SupportState.h>

namespace PatternGeneratorJRL
{
  /*! This class generates matrix representation of linear
   constraint based on foot position.
   It handles a stack of constraint on a sliding mode 
   for QP solving.
   */
  class FootConstraintsAsLinearSystemForStepPos : public FootConstraintsAsLinearSystemForVelRef
    {
    public:

      /*! Constructor */
      FootConstraintsAsLinearSystemForStepPos(SimplePluginManager *aSPM, 
				    CjrlHumanoidDynamicRobot *aHS,
				    double ConstraintOnX,
				    double ConstraintOnY);

      /*! Destructor */
      ~FootConstraintsAsLinearSystemForStepPos();

      /*! Compute the linear system \f${\bf A}{\bf x} \geq {\bf b}\f$ associated with the 
	set of points specified by aVecOfPoints. aVecOfPoints is supposed to represent
	the convex hull of the robot contact points with the ground.
       */
      int ComputeLinearSystem(std::vector<CH_Point> aVecOfPoints,
			      MAL_MATRIX(&A,double),
			      MAL_MATRIX(&B,double),
			      MAL_VECTOR(&C,double));



      /*!  Build a queue of constraint Inequalities based on a list of Foot Absolute
	Position.
       */
      int buildLinearConstraintInequalities(std::deque< FootAbsolutePosition> &LeftFootAbsolutePositions,
					    std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					    std::deque<LinearConstraintInequalityFreeFeet_t> &
					    QueueOfLConstraintInequalities,
					    std::deque<LinearConstraintInequalityFreeFeet_t> &
					    QueueOfFeetPosInequalities,
						ReferenceAbsoluteVelocity & RefVel,
					    RelativeFootPositionQueue & RefFP,
					    double StartingTime,
					    double m_QP_N,
					    SupportState * Support, std::deque<double> &PreviewedSupportAngles,
					    int & NbOfConstraints);

	protected :
		 std::vector<CH_Point> ConvexHullFPRef;

    
    };
};
#endif /* _fOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_ */

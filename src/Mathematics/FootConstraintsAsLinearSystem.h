/* This object generate matrix representation of linear
   constraint based on foot position.
   It handles a stack of constraint on a sliding mode 
   for QP solving.

   Copyright (c) 2009, 
   Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

*/

#ifndef _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_
#define _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_

#include <vector>
#include <deque>
#include <string>
#include <sstream>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <robotDynamics/jrlHumanoidDynamicRobot.h>


#include <PGTypes.h>
#include <Mathematics/ConvexHull.h>
#include <SimplePlugin.h>

namespace PatternGeneratorJRL
{
  /*! This class generates matrix representation of linear
   constraint based on foot position.
   It handles a stack of constraint on a sliding mode 
   for QP solving.
   */
  class  FootConstraintsAsLinearSystem: public SimplePlugin
    {
    public:

      /*! Constructor */
      FootConstraintsAsLinearSystem(SimplePluginManager *aSPM, 
				    CjrlHumanoidDynamicRobot *aHS);

      /*! Destructor */
      ~FootConstraintsAsLinearSystem();

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
      int BuildLinearConstraintInequalities(std::deque< FootAbsolutePosition> &LeftFootAbsolutePositions,
					    std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					    std::deque<LinearConstraintInequality_t *> &
					    QueueOfLConstraintInequalities,
					    double ConstraintOnX,
					    double ConstraintOnY);

      /*!  Build a queue of constraint Inequalities based on a list of Foot Absolute Position.  */
      int BuildLinearConstraintInequalities2(std::deque< FootAbsolutePosition> &LeftFootAbsolutePositions,
					     std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					     std::deque<LinearConstraintInequality_t *> &
					     QueueOfLConstraintInequalities,
					     double ConstraintOnX,
					     double ConstraintOnY);

      /*! Find Similar Constraints */
      int FindSimilarConstraints(MAL_MATRIX(&A,double),
				 std::vector<int> &SimilarConstraints);

      /*! Reimplement the interface of SimplePluginManager 
	\param[in] Method: The method to be called.
	\param[in] Args: Arguments of the methods.
       */
      virtual void CallMethod(std::string & Method, std::istringstream &Args);

    private:

      /* ! Reference on the Humanoid Specificities. */
      CjrlHumanoidDynamicRobot * m_HS;
      
    };
};
#endif /* _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_ */

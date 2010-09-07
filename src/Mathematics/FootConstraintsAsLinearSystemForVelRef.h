/* This object generate matrix representation of linear
   constraint based on foot position.
   It handles a stack of constraint on a sliding mode 
   for QP solving.

   Copyright (c) 2009, 
   Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

*/

#ifndef _fOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_
#define _fOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_

#include <vector>
#include <deque>
#include <string>
#include <sstream>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <robotDynamics/jrlHumanoidDynamicRobot.h>


#include <PGTypes.h>
#include <Mathematics/ConvexHull.h>
#include <SimplePlugin.h>
#include <PreviewControl/SupportState.h>

namespace PatternGeneratorJRL
{
  /*! This class generates matrix representation of linear
   constraint based on foot position.
   It handles a stack of constraint on a sliding mode 
   for QP solving.
   */
  class  FootConstraintsAsLinearSystemForVelRef: public SimplePlugin
    {
    public:

      /*! Constructor */
      FootConstraintsAsLinearSystemForVelRef(SimplePluginManager *aSPM, 
				    CjrlHumanoidDynamicRobot *aHS,
				    double ConstraintOnX,
				    double ConstraintOnY);

      /*! Destructor */
      virtual ~FootConstraintsAsLinearSystemForVelRef();

      /*! Compute the linear system \f${\bf A}{\bf x} \geq {\bf b}\f$ associated with the 
	set of points specified by aVecOfPoints. aVecOfPoints is supposed to represent
	the convex hull of the robot contact points with the ground.
       */
      int ComputeLinearSystem(std::vector<CH_Point> aVecOfPoints,
			      MAL_MATRIX(&A,double),
			      MAL_MATRIX(&B,double),
			      MAL_VECTOR(&C,double));

      /*! Compute the linear system \f${\bf D}{\bf x} \geq {\bf b}\f$ associated with the 
	set of points specified by aVecOfPoints. aVecOfPoints is supposed can represent
	either the convex hull of the robot contact points with the ground or the constraints on the 
	placement of the feet.
       */
      int computeLinearSystem(std::vector<CH_Point> aVecOfPoints,
			      MAL_MATRIX(&D,double),
			      MAL_MATRIX(&Dc,double),
			      SupportState * Support);

      /*!  Build a queue of constraint Inequalities based on a list of Foot Absolute
	Position.
       */
     virtual int buildLinearConstraintInequalities(std::deque< FootAbsolutePosition> &LeftFootAbsolutePositions,
					    std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					    std::deque<LinearConstraintInequalityFreeFeet_t> &
					    QueueOfLConstraintInequalities,
					    std::deque<LinearConstraintInequalityFreeFeet_t> &
					    QueueOfFeetPosInequalities,
					    ReferenceAbsoluteVelocity & RefVel,
					    double StartingTime,
					    double m_QP_N,
					    SupportState * Support, std::deque<double> &PreviewedSupportAngles,
					    int & NbOfConstraints);

      /*!  Build a queue of constraint Inequalities based on a list of Foot Absolute Position.  */
      int BuildLinearConstraintInequalities2(std::deque< FootAbsolutePosition> &LeftFootAbsolutePositions,
					     std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					     std::deque<LinearConstraintInequality_t> &
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

    protected:

      void initFPConstrArrays();

      /* ! Reference on the Humanoid Specificities. */
      CjrlHumanoidDynamicRobot * m_HS;
      
      std::vector<CH_Point> ConvexHullFP;
	  


      //Andremize: Caused memory errors when initialized in the constructor
      /* double CHLeftFPosConstrArrayX[5]; */
      /* double CHLeftFPosConstrArrayY[5]; */
      
      /* double CHRightFPosConstrArrayX[5]; */
      /* double CHRightFPosConstrArrayY[5]; */
      
      double* CHFPosConstrArrayX; 
      double* CHFPosConstrArrayY;

      // Find the convex hull for each of the position,
      // in order to create the corresponding trajectory.
      double lLeftFootHalfWidth,lLeftFootHalfHeight,lLeftFootHalfHeightDS,
	lRightFootHalfWidth,lRightFootHalfHeight,lRightFootHalfHeightDS,lZ;
      
      double DSFeetDistance;
      
      // Read humanoid specificities.
      CjrlFoot * lRightFoot;
      vector3d AnklePosition;
      CjrlFoot * lLeftFoot;
  
      int State; // State for the system 0:start, 1: Right Support Foot, 2: Left Support Foot,
      // 3: Double Support.
      int ComputeCH;
      double lx, ly;
      
      double prev_xmin, prev_xmax, prev_ymin, prev_ymax;
      
      double s_t,c_t;
      
      unsigned int m_FullDebug;
    };
};
#endif /* _fOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_ */

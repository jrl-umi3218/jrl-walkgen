/*
 * Copyright 2010, 
 *
 * Mehdi      Benallegue
 * Andrei     Herdt
 * Olivier    Stasse
 * 
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
/* This object generate matrix representation of linear
   constraint based on foot position.
   It handles a stack of constraint on a sliding mode 
   for QP solving. */

#ifndef _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_FOR_VEL_REF_H_
#define _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_FOR_VEL_REF_H_

#include <vector>
#include <deque>
#include <string>
#include <sstream>

#include <jrl/mal/matrixabstractlayer.hh>

#include <abstract-robot-dynamics/jrlhumanoiddynamicrobot.hh>


#include <jrl/walkgen/pgtypes.hh>
#include <privatepgtypes.h>
#include <Mathematics/ConvexHull.h>
#include <Mathematics/FootHalfSize.hh>
#include <SimplePlugin.h>
#include <PreviewControl/SupportFSM.h>

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
      ~FootConstraintsAsLinearSystemForVelRef();

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
			      SupportState_t PrwSupport);

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
					    double StartingTime,
					    double m_QP_N,
					    SupportFSM * SupportFSM, SupportState_t CurrentSupport, SupportState_t & PrwSupport,
					    std::deque<double> &PreviewedSupportAngles,
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

    private:

      void initFPConstrArrays();

      /* ! Reference on the Humanoid Specificities. */
      CjrlHumanoidDynamicRobot * m_HS;
      
      std::vector<CH_Point> ConvexHullFP;
      
      float* CHFPosConstrArrayX; 
      float* CHFPosConstrArrayY;

      // Find the convex hull for each of the position,
      // in order to create the corresponding trajectory.
      FootHalfSize m_LeftFootSize, m_RightFootSize;
      double m_Z;
      
      double DSFeetDistance;
      
      // Read humanoid specificities.
      CjrlFoot * m_RightFoot;
      vector3d AnklePosition;
      CjrlFoot * m_LeftFoot;
  
      int State; // State for the system 0:start, 1: Right Support Foot, 2: Left Support Foot,
      // 3: Double Support.
      int ComputeCH;
      float lx, ly;
      
      double prev_xmin, prev_xmax, prev_ymin, prev_ymax;
      
      double s_t,c_t;
      
      unsigned int m_FullDebug;

      /*! \brief Additional constraints on the feet size.
	m_ConstraintOnX  is removed from the length of the feet along the X-axis.
	m_ConstraintOnY  is removed from the length of the feet along the Y-axis.
       */
      double m_ConstraintOnX, m_ConstraintOnY;
    };
};
#endif /* _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_FOR_VEL_REF_H_ */

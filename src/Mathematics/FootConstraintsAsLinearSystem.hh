/*
 * Copyright 2009, 2010,
 *
 * Francois   Keith
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

#ifndef _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_
#define _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_

#include <deque>
#include <sstream>
#include <string>
#include <vector>

#include <jrl/walkgen/pinocchiorobot.hh>

#include <Mathematics/ConvexHull.hh>
#include <SimplePlugin.hh>
#include <jrl/walkgen/pgtypes.hh>

namespace PatternGeneratorJRL {
/*! This class generates matrix representation of linear
  constraint based on foot position.
  It handles a stack of constraint on a sliding mode
  for QP solving.
*/
class FootConstraintsAsLinearSystem : public SimplePlugin {
public:
  /*! Constructor */
  FootConstraintsAsLinearSystem(SimplePluginManager *aSPM, PinocchioRobot *aPR);

  /*! Destructor */
  ~FootConstraintsAsLinearSystem();

  /*! Compute the linear system \f${\bf A}{\bf x} \geq {\bf b}\f$
    associated with the
    set of points specified by aVecOfPoints. aVecOfPoints is
    supposed to represent
    the convex hull of the robot contact points with the ground.
  */
  int ComputeLinearSystem(std::vector<CH_Point> aVecOfPoints,
                          Eigen::MatrixXd &A, Eigen::MatrixXd &B,
                          Eigen::VectorXd &C);

  /*!  Build a queue of constraint Inequalities based on a list of
    Foot Absolute Position.
  */
  int BuildLinearConstraintInequalities(
      std::deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
      std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
      std::deque<LinearConstraintInequality_t *>
          &QueueOfLConstraintInequalities,
      double ConstraintOnX, double ConstraintOnY);

  /*!  Build a queue of constraint Inequalities based on a list
    of Foot Absolute Position.  */
  int BuildLinearConstraintInequalities2(
      std::deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
      std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
      std::deque<LinearConstraintInequality_t *>
          &QueueOfLConstraintInequalities,
      double ConstraintOnX, double ConstraintOnY);

  /*! Find Similar Constraints */
  int FindSimilarConstraints(Eigen::MatrixXd &A,
                             std::vector<int> &SimilarConstraints);

  /*! Reimplement the interface of SimplePluginManager
    \param[in] Method: The method to be called.
    \param[in] Args: Arguments of the methods.
  */
  virtual void CallMethod(std::string &Method, std::istringstream &Args);

private:
  /* ! Reference on the Humanoid Specificities. */
  PinocchioRobot *m_PR;
};
} // namespace PatternGeneratorJRL
#endif /* _FOOT_CONSTRAINTS_AS_LINEAR_SYSTEM_H_ */

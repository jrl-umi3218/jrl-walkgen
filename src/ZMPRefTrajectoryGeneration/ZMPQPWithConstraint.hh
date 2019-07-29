/*
 * Copyright 2010,
 *
 * Francois Keith
 * Olivier Stasse
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
/* This object provides the QP formulation with constraint on the ZMP
   of  the preview control method developed by Kaita..
   This formulation has been proposed by PB Wieber in Humanoids 2006. */

#ifndef _ZMPQP_WITH_CONSTRAINT_H_
#define _ZMPQP_WITH_CONSTRAINT_H_


#include <Mathematics/ConvexHull.hh>
#include <ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.hh>

namespace PatternGeneratorJRL
{
  class ZMPDiscretization;
  class  ZMPQPWithConstraint : public ZMPRefTrajectoryGeneration
  {

  public:

    /* Default constructor. */
    ZMPQPWithConstraint(SimplePluginManager *lSPM, string DataFile,
                        PinocchioRobot *aPR=0);

    /* Default destructor. */
    ~ZMPQPWithConstraint();

    /*! This method builds a set of linear constraint inequalities based
      on the foot trajectories given as an input.
      The result is a set Linear Constraint Inequalities. */
    int BuildLinearConstraintInequalities
    (deque<FootAbsolutePosition>
     &LeftFootAbsolutePositions,
     deque<FootAbsolutePosition> &RightFootAbsolutePositions,
     deque<LinearConstraintInequality_t *> & QueueOfLConstraintInequalities,
     double ConstraintOnX,
     double ConstraintOnY);
    
    /** Generate ZMP discreatization from a vector of foot position.
        ASSUME A COMPLETE MOTION FROM END TO START, and GENERATE EVERY VALUE.

        @param[out] ZMPPositions: Returns the ZMP reference values for 
        the overall motion.
        Those are absolute position in the world reference frame. The origin is 
        the initial
        position of the robot. The relative foot position specified are added.

        @param[out] CoMStates: Returns the CoM reference values for the overall 
        motion.
        Those are absolute position in the world reference frame. The origin is 
        the initial
        position of the robot. The relative foot position specified are added.

        @param[in] RelativeFootPositions: The set of
        relative steps to be performed by the robot.

        @param[out] LeftFootAbsolutePositions: Returns the absolute position of 
        the left foot.
        According to the macro FULL_POLYNOME the trajectory will follow a third 
        order
        polynom or a fifth order. By experience it is wise to put a third order.
        A null acceleration might cause problem for the compensation of the 
        Z-axis momentum.

        @param[out] RightFootAbsolutePositions: Returns the absolute position of
        the right foot.

        @param[in] Xmax: The maximal distance of a hand along the X axis in the 
        waist coordinates.

        @param[in] lStartingCOMState: The initial position of the CoM.

        @param[in] lStartingZMPPosition: The initial position of the ZMP.

        @param[in] InitLeftFootAbsolutePosition: The initial position of the 
        left foot.

        @param[in] InitRightFootAbsolutePosition: The initial position of the 
        right foot.


    */
    void GetZMPDiscretization
    (deque<ZMPPosition> & ZMPPositions,
     deque<COMState> & CoMStates,
     deque<RelativeFootPosition> &RelativeFootPositions,
     deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
     deque<FootAbsolutePosition> &RightFootAbsolutePositions,
     double Xmax,
     COMState & lStartingCOMState,
     Eigen::Vector3d & lStartingZMPPosition,
     FootAbsolutePosition & InitLeftFootAbsolutePosition,
     FootAbsolutePosition & InitRightFootAbsolutePosition);

    /*! This method is a new way of computing the ZMP trajectory from
      foot trajectory. */
    int BuildZMPTrajectoryFromFootTrajectory
    (deque<FootAbsolutePosition>
     &LeftFootAbsolutePositions,
     deque<FootAbsolutePosition> &RightFootAbsolutePositions,
     deque<ZMPPosition> &ZMPRefPositions,
     deque<COMState> &COMStates,
     double ConstraintOnX,
     double ConstraintOnY,
     double T,
     unsigned int N);

    /*! Build the necessary matrices for the QP problem under linear inequality 
      constraints. */
    int BuildMatricesPxPu
    (double * &Px, double * &Pu,
     unsigned N, double T,
     double StartingTime,
     deque<LinearConstraintInequality_t *>
     & QueueOfLConstraintInequalities,
     double Com_Height,
     unsigned int &NbOfConstraints,
     Eigen::VectorXd &xk);

    /*! This method helps to build a linear system for constraining the ZMP. */
    int ComputeLinearSystem(std::vector<CH_Point> aVecOfPoints,
                            Eigen::MatrixXd &A,
                            Eigen::MatrixXd &B);

    /*! This method get the COM buffer computed by the QP in off-line mode. */
    void GetComBuffer(deque<COMState> &aCOMBuffer);

    /*! Call method to handle the plugins. */
    void CallMethod(std::string &Method, std::istringstream &strm);

    /*! Call method to handle on-line generation of ZMP reference trajectory. 
      @{*/

    /*! Methods for on-line generation. (First version)
      The queues will be updated as follows:
      - The first values necessary to start walking will be inserted.
      - The initial positions of the feet will be taken into account
      according to InitLeftFootAbsolutePosition and 
      InitRightFootAbsolutePosition.
      - The RelativeFootPositions stack will be taken into account,
      - The starting COM Position.
      Returns the number of steps which has been completely put inside
      the queue of ZMP, and foot positions.
    */
    std::size_t InitOnLine
    (deque<ZMPPosition> & FinalZMPPositions,
     deque<COMState> & CoMStates,
     deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
     deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
     FootAbsolutePosition & InitLeftFootAbsolutePosition,
     FootAbsolutePosition & InitRightFootAbsolutePosition,
     deque<RelativeFootPosition> &RelativeFootPositions,
     COMState & lStartingCOMState,
                           Eigen::Vector3d & lStartingZMPPosition);

    /* ! Methods to update the stack on-line by inserting 
       a new foot position. */
    void OnLineAddFoot
    (RelativeFootPosition & NewRelativeFootPosition,
     deque<ZMPPosition> & FinalZMPPositions,
     deque<COMState> & CoMStates,
     deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
     deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
     bool EndSequence);

    /* ! \brief Method to update the stacks on-line */
    void OnLine
    (double time,
     deque<ZMPPosition> & FinalZMPPositions,
     deque<COMState> & CoMStates,
     deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
     deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions);

    /* ! \brief Method to change on line the landing position of a foot.
       @return If the method failed it returns -1, 0 otherwise.
    */
    int OnLineFootChange
    (double time,
     FootAbsolutePosition &aFootAbsolutePosition,
     deque<ZMPPosition> & FinalZMPPositions,
     deque<COMState> & CoMStates,
     deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
     deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
     StepStackHandler * aStepStackHandler=0);

    /*! \brief Method to stop walking.
      @param[out] ZMPPositions: The queue of ZMP reference positions.
      @param[out] FinalCOMStates: The queue of COM reference positions.
      @param[out] LeftFootAbsolutePositions: 
      The queue of left foot absolute positions.
      @param[out] RightFootAbsolutePositions: 
      The queue of right foot absolute positions.
    */
    void EndPhaseOfTheWalking
    (deque<ZMPPosition> &ZMPPositions,
     deque<COMState> &FinalCOMStates,
     deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
     deque<FootAbsolutePosition> &RightFootAbsolutePositions);
    

    /*! \brief Return the time at which it is optimal to regenerate a step in 
      online mode.
     */
    int ReturnOptimalTimeToRegenerateAStep();

    /* @} */


  protected:
    /* ! Reference on the Humanoid Specificities. */
    PinocchioRobot * m_PR;

    /* !  Matrices for the dynamical system.
       @{
    */
    /* ! Matrix regarding the state of the CoM (pos, velocity, acceleration) */
    Eigen::MatrixXd m_A;
    /* ! Vector for the command */
    Eigen::MatrixXd m_B;
    /* ! Vector for the ZMP. */
    Eigen::MatrixXd m_C;
    /* ! @} */

    /*! Uses a ZMPDiscretization scheme to get the usual Kajita heuristic. */
    ZMPDiscretization * m_ZMPD;

    /*! Constraint on X and Y */
    double m_ConstraintOnX, m_ConstraintOnY;

    /*! Sampling of the QP. */
    double m_QP_T;

    /*! Preview window */
    unsigned int m_QP_N;

  };
}

#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.hh>
#endif /* _ZMPQP_WITH_CONSTRAINT_H_ */

/*
 * Copyright 2010,
 *
 * Medhi    Benallegue
 * Andrei   Herdt
 * Francois Keith
 * Olivier  Stasse
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
/*! This object provides the generation of ZMP and CoM trajectory
  using a new formulation of the stability problem.
*/

#ifndef _ZMPVELOCITYREFERENCEDQP_WITH_CONSTRAINT_H_
#define _ZMPVELOCITYREFERENCEDQP_WITH_CONSTRAINT_H_



#include <PreviewControl/LinearizedInvertedPendulum2D.h>
#include <Mathematics/relative-feet-inequalities.hh>
#include <Mathematics/OptCholesky.h>
#include <ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.h>
#include <Mathematics/PLDPSolverHerdt.h>
#include <PreviewControl/SupportFSM.h>
#include <FootTrajectoryGeneration/OnLineFootTrajectoryGeneration.h>
#include <ZMPRefTrajectoryGeneration/OrientationsPreview.h>
#include <ZMPRefTrajectoryGeneration/qp-problem.hh>
#include <privatepgtypes.h>
#include <ZMPRefTrajectoryGeneration/generator-vel-ref.hh>
#include <Mathematics/intermediate-qp-matrices.hh>
#include <jrl/walkgen/pgtypes.hh>

namespace PatternGeneratorJRL
{


  class ZMPDiscretization;
  class  ZMPVelocityReferencedQP : public ZMPRefTrajectoryGeneration
  {

    //
    // Public methods:
    //
  public:

    /* Default constructor. */
    ZMPVelocityReferencedQP(SimplePluginManager *lSPM, string DataFile,
			    CjrlHumanoidDynamicRobot *aHS=0);

    /* Default destructor. */
    ~ZMPVelocityReferencedQP();


    /*! Call method to handle the plugins (SimplePlugin interface) . */
    void CallMethod(std::string &Method, std::istringstream &strm);

    /*! \name Call method to handle on-line generation of ZMP reference trajectory.
      @{*/

    /*! Methods for on-line generation. (First version!)
      The queues will be updated as follows:
      - The first values necessary to start walking will be inserted.
      - The initial positions of the feet will be taken into account
      according to InitLeftFootAbsolutePosition and InitRightFootAbsolutePosition.
      - The RelativeFootPositions stack will NOT be taken into account,
      - The starting COM Position will NOT be taken into account.
      Returns the number of steps which has been completely put inside
      the queue of ZMP, and foot positions.
    */
    int InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
		   deque<COMState> & CoMStates,
		   deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
		   deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
		   FootAbsolutePosition & InitLeftFootAbsolutePosition,
		   FootAbsolutePosition & InitRightFootAbsolutePosition,
		   deque<RelativeFootPosition> &RelativeFootPositions,
		   COMState & lStartingCOMState,
		   MAL_S3_VECTOR_TYPE(double) & lStartingZMPPosition);


    /* ! \brief Method to update the stacks on-line */
    void OnLine(double time,
		deque<ZMPPosition> & FinalZMPPositions,
		deque<COMState> & CoMStates,
		deque<FootAbsolutePosition> &FinalLeftFootTraj_deq,
		deque<FootAbsolutePosition> &FinalRightFootTraj_deq);


    int validateConstraints(double * & DS,double * &DU,
			    int NbOfConstraints,  int li,
			    double *X, double time);


    /// \name Accessors
    /// \{
    /*! Set the velocity reference */
    void setVelReference(istringstream &strm);

    /*! Set the velocity reference from external reference */
    void setVelReference(double dx,double dy, double dyaw);

    /*! Set the velocity reference from external reference */
    void setCoMPerturbationForce(double x,double y);

    void setCoMPerturbationForce(istringstream &strm);
    /// \}

    reference_t VelRef_;

  private:

    double RobotMass_;
    bool PerturbationOccured_;
    
    bool EndingPhase_;
    double TimeToStopOnLineMode_;

    double UpperTimeLimitToUpdate_;

    double TimeBuffer_;

    /// \brief 2D LIPM to simulate the evolution of the robot.
    LinearizedInvertedPendulum2D CoM_;

    /// \brief Finite State Machine to simulate the evolution of the support states.
    SupportFSM * SupportFSM_;

    /// \brief Deecoupled optimization problem to compute the evolution of feet angles.
    OrientationsPreview * OrientPrw_;

    /// \brief Generator of QP problem
    GeneratorVelRef * VRQPGenerator_;

    /// \brief Object creating linear inequalities relative to feet centers.
    RelativeFeetInequalities * RFC_;

    /// \brief Standard polynomial trajectories for the feet.
    OnLineFootTrajectoryGeneration * OFTG_;

    /// \brief Final optimization problem
    QPProblem Problem_;

    /// \brief Additional term on the acceleration of the CoM
    MAL_VECTOR(PerturbationAcceleration_,double);

    /// \brief QP-sampling period
    double QP_T_;

    /// \brief Nb samlings inside preview window
    int QP_N_;


    /*! \brief Fast formulations mode. */
    unsigned m_FastFormulationMode;

    
  public:

    /*! Methods to comply with the initial interface of ZMPRefTrajectoryGeneration.
      TODO: Change the internal structure to make those methods not mandatory
      for compiling.
    */

    void GetZMPDiscretization(std::deque<ZMPPosition> & ZMPPositions,
			      std::deque<COMState> & COMStates,
			      std::deque<RelativeFootPosition> &RelativeFootPositions,
			      std::deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			      std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
			      double Xmax,
			      COMState & lStartingCOMState,
			      MAL_S3_VECTOR_TYPE(double) & lStartingZMPPosition,
			      FootAbsolutePosition & InitLeftFootAbsolutePosition,
			      FootAbsolutePosition & InitRightFootAbsolutePosition);

    void OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
		       std::deque<ZMPPosition> & FinalZMPPositions,
		       std::deque<COMState> & COMStates,
		       std::deque<FootAbsolutePosition> &FinalLeftFootTraj_deq,
		       std::deque<FootAbsolutePosition> &FinalRightFootTraj_deq,
		       bool EndSequence);

    int OnLineFootChange(double time,
			 FootAbsolutePosition &aFootAbsolutePosition,
			 deque<ZMPPosition> & FinalZMPPositions,
			 deque<COMState> & CoMPositions,
			 deque<FootAbsolutePosition> &FinalLeftFootTraj_deq,
			 deque<FootAbsolutePosition> &FinalRightFootTraj_deq,
			 StepStackHandler  *aStepStackHandler);

    void EndPhaseOfTheWalking(deque<ZMPPosition> &ZMPPositions,
			      deque<COMState> &FinalCOMTraj_deq,
			      deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			      deque<FootAbsolutePosition> &RightFootAbsolutePositions);

    int ReturnOptimalTimeToRegenerateAStep();
  };
}

#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.h>
#endif /* _ZMPQP_WITH_CONSTRAINT_H_ */

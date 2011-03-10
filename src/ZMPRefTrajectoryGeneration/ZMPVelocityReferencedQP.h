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
#include <Mathematics/FootConstraintsAsLinearSystemForVelRef.h>
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

namespace PatternGeneratorJRL
{


  class ZMPDiscretization;
  class  ZMPVelocityReferencedQP : public ZMPRefTrajectoryGeneration
  {

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


    /*! \name Setter and getter for the objective function parameters
      @{
    */

    /*! Set the velocity reference */
    void setVelReference(istringstream &strm);

    /*! Set the velocity reference from external reference */
    void setVelReference(double dx,double dy, double dyaw);

    /*! Set the velocity reference from external reference */
    void setCoMPerturbationForce(double x,double y);

    void setCoMPerturbationForce(istringstream &strm);

    void interpolateFeet(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			 deque<FootAbsolutePosition> &RightFootAbsolutePositions);

    reference_t m_VelRef;

    static const unsigned int QLD=0;
    static const unsigned int QLDANDLQ=1;
    static const unsigned int PLDP=2;
    static const unsigned int PLDPHerdt = 3;

  private:

    double m_RobotMass;
    bool m_PerturbationOccured;
    double m_FeetDistanceDS;
    
    bool m_EndingPhase;
    double m_TimeToStopOnLineMode;

    double m_FPx, m_FPy, m_FPtheta;
    double m_StartTime;

    double m_UpperTimeLimitToUpdate;

    double m_TimeBuffer;

    /*! Uses a ZMPDiscretization scheme to get the usual Kajita heuristic. */
    ZMPDiscretization * m_ZMPD;

    /*! Uses a 2D LIPM to simulate the evolution of the robot. */
    LinearizedInvertedPendulum2D m_CoM;

    /*! Uses a Finite State Machine to simulate the evolution of the support states. */
    SupportFSM * SupportFSM_;

    /*! Deecoupled optimization problem to compute the evolution of feet angles. */
    OrientationsPreview * m_OP;

    GeneratorVelRef * m_GenVR;

    /*! \brief Object creating Linear inequalities constraints
      based on the foot position. Those constraints are *NOT* the
      one put in the QP, but they are a necessary intermediate step. */
    FootConstraintsAsLinearSystemForVelRef * m_fCALS;

    /*! \brief Standard polynomial trajectories for the feet. */
    OnLineFootTrajectoryGeneration * OFTG_;

    /*! Constraint on X and Y */
    double m_ConstraintOnX, m_ConstraintOnY;

    /*! Com height */
    double m_ComHeight;

    /*! Current state of the trunk and the trunk state after m_QP_T*/
    COMState m_TrunkState, m_TrunkStateT;

    deque<COMState> m_QueueOfTrunkStates;

    double m_a, m_TrunkPolCoeffB, m_c, m_d, m_TrunkPolCoeffE;

    //Additional term on the acceleration of the CoM
    MAL_VECTOR(m_PerturbationAcceleration,double);

    /*! Sampling of the QP. */
    double m_QP_T;

    /*! Preview window */
    int m_QP_N;

    //Final optimization problem
    QPProblem m_Pb;


    /*! \brief Cholesky decomposition of the initial objective function $Q$ */
    MAL_MATRIX(m_LQ,double);

    /*! \brief Cholesky decomposition of the initial objective function $Q$ */
    MAL_MATRIX(m_iLQ,double);

    /*! \brief Optimized cholesky decomposition */
    OptCholesky * m_OptCholesky;

    /*! \brief Sub matrix to compute the linear part of the objective function $p^{\top}_k$. */
    MAL_MATRIX(m_OptA,double);
    MAL_MATRIX(m_OptB,double);
    MAL_MATRIX(m_OptC,double);
    MAL_MATRIX(m_OptD,double);

    /* Constant parts of the linear constraints. */
    MAL_MATRIX(m_iPu,double);

    /* Constant parts of the dynamical system. */
    MAL_MATRIX(m_Px,double);

    /*! \brief Debugging variable: dump everything is set to 1 */
    int m_FullDebug;

    /*! \brief Fast formulations mode. */
    unsigned int m_FastFormulationMode;

    bool m_InvariantPartInitialized;

    void initializeProblem();


    void interpolateTrunkState(double time, int CurrentIndex,
                               const support_state_t & CurrentSupport,
			       deque<COMState> & FinalCOMTraj_deq);

    void interpolateFeetPositions(double time, int CurrentIndex,
                                  const support_state_t & CurrentSupport,
                                  const deque<double> & PreviewedSupportAngles_deq,
                                  deque<FootAbsolutePosition> &FinalLeftFootTraj_deq,
                                  deque<FootAbsolutePosition> &FinalRightFootTraj_deq);

    
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

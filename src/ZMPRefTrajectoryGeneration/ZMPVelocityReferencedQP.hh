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
/*! Generate ZMP and CoM trajectories using Herdt2010IROS
 */

#ifndef _ZMPVELOCITYREFERENCEDQP_WITH_CONSTRAINT_H_
#define _ZMPVELOCITYREFERENCEDQP_WITH_CONSTRAINT_H_

#include <PreviewControl/PreviewControl.hh>
#include <PreviewControl/LinearizedInvertedPendulum2D.hh>
#include <PreviewControl/rigid-body-system.hh>
#include <Mathematics/relative-feet-inequalities.hh>
#include <ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.hh>
#include <PreviewControl/SupportFSM.hh>
#include <ZMPRefTrajectoryGeneration/OrientationsPreview.hh>
#include <ZMPRefTrajectoryGeneration/qp-problem.hh>
#include <privatepgtypes.hh>
#include <ZMPRefTrajectoryGeneration/generator-vel-ref.hh>
#include <Mathematics/intermediate-qp-matrices.hh>
#include <jrl/walkgen/pgtypes.hh>
#include <ZMPRefTrajectoryGeneration/DynamicFilter.hh>

namespace PatternGeneratorJRL
{


  class ZMPDiscretization;
  class  ZMPVelocityReferencedQP : public ZMPRefTrajectoryGeneration
  {

    //
    // Public methods:
    //
  public:

    ZMPVelocityReferencedQP(SimplePluginManager *SPM, string DataFile,
                            PinocchioRobot *aPR=0 );

    ~ZMPVelocityReferencedQP();


    /// \brief Handle plugins (SimplePlugin interface)
    void CallMethod(std::string &Method, std::istringstream &strm);

    /*! \name Call method to handle on-line generation of ZMP 
      reference trajectory.
      @{*/

    /*! Methods for on-line generation. (First version!)
      The queues will be updated as follows:
      - The first values necessary to start walking will be inserted.
      - The initial positions of the feet will be taken into account
      according to InitLeftFootAbsolutePosition and 
      InitRightFootAbsolutePosition.
      - The RelativeFootPositions stack will NOT be taken into account,
      - The starting COM Position will NOT be taken into account.
      Returns the number of steps which has been completely put inside
      the queue of ZMP, and foot positions.
    */
    std::size_t InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
                           deque<COMState> & FinalCoMPositions_deq,
                           deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
                           deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
                           FootAbsolutePosition & InitLeftFootAbsolutePosition,
                           FootAbsolutePosition & InitRightFootAbsolutePosition,
                           deque<RelativeFootPosition> &RelativeFootPositions,
                           COMState & lStartingCOMState,
                           Eigen::Vector3d & lStartingZMPPosition);


    /// \brief Update the stacks on-line
    void OnLine(double time,
                deque<ZMPPosition> & FinalZMPPositions,
                deque<COMState> & FinalCOMTraj_deq,
                deque<FootAbsolutePosition> &FinalLeftFootTraj_deq,
                deque<FootAbsolutePosition> &FinalRightFootTraj_deq);


    /// \name Accessors and mutators
    /// \{
    /// \brief Set the reference (velocity only as for now) through
    /// the Interface (slow)
    void Reference(istringstream &strm)
    {
      strm >> NewVelRef_.Local.X;
      strm >> NewVelRef_.Local.Y;
      strm >> NewVelRef_.Local.Yaw;
    }
    inline void Reference(double dx, double dy, double dyaw)
    {
      NewVelRef_.Local.X = dx;
      NewVelRef_.Local.Y = dy;
      NewVelRef_.Local.Yaw = dyaw;
    }

    inline bool Running()
    {
      return Running_;
    }

    /// \brief Set the final-stage trigger
    inline void EndingPhase(bool EndingPhase)
    {
      EndingPhase_ = EndingPhase;
    }

    void setCoMPerturbationForce(double x,double y);
    void setCoMPerturbationForce(istringstream &strm);

    solution_t & Solution()
    {
      return Solution_;
    }

    inline const int & QP_N(void) const
    {
      return QP_N_;
    }

    /// \brief Setter and getter for the ComAndZMPTrajectoryGeneration.
    inline ComAndFootRealization * getComAndFootRealization()
    {
      return dynamicFilter_->getComAndFootRealization();
    };
    /// \}

    inline double InterpolationPeriod()
    {
      return InterpolationPeriod_ ;
    }
    inline void InterpolationPeriod( double T )
    {
      InterpolationPeriod_ = T ;
    }


    //
    // Private members:
    //
  private:

    /// \brief (Updated) Reference
    reference_t VelRef_;
    /// \brief Temporary (updating) reference
    reference_t NewVelRef_;

    /// \brief Total mass of the robot
    double RobotMass_;

    /// \brief Perturbation trigger
    bool PerturbationOccured_;

    /// \brief Final stage trigger
    bool EndingPhase_;

    /// \brief PG running
    bool Running_;

    /// \brief Time at which the online mode will stop
    double TimeToStopOnLineMode_;

    /// \brief Time at which the problem should be updated
    double UpperTimeLimitToUpdate_;

    /// \brief Security margin for trajectory queues
    double TimeBuffer_;

    /// \brief Additional term on the acceleration of the CoM
    Eigen::VectorXd PerturbationAcceleration_;

    /// \brief Sampling period considered in the QP
    double QP_T_;

    /// \brief Nb. samplings inside preview window
    int QP_N_;

    /// \brief Size of the preview for filtering
    double previewDuration_ ;

    /// \brief Duration of the preview for filtering
    int previewSize_ ;

    /// \brief 2D LIPM to simulate the evolution of the robot's CoM.
    LinearizedInvertedPendulum2D LIPM_ ;
    LinearizedInvertedPendulum2D LIPM_subsampled_ ;
    LinearizedInvertedPendulum2D CoM_;

    /// \brief Simplified robot model
    RigidBodySystem * Robot_ ;

    /// \brief Finite State Machine to simulate the evolution of
    /// the support states.
    SupportFSM * SupportFSM_;

    /// \brief Decoupled optimization problem to compute the evolution
    /// of feet angles.
    OrientationsPreview * OrientPrw_;
    OrientationsPreview * OrientPrw_DF_;

    /// \brief Generator of QP problem
    GeneratorVelRef * VRQPGenerator_;

    /// \brief Intermediate QP data
    IntermedQPMat * IntermedData_;

    /// \brief Object creating linear inequalities relative to feet centers.
    RelativeFeetInequalities * RFI_;

    /// \brief Final optimization problem
    QPProblem Problem_;

    /// \brief Previewed Solution
    solution_t Solution_;

    /// \brief Copy of the QP_ solution
    solution_t solution_ ;

    /// \brief HDR allow the computation of the dynamic filter
    PinocchioRobot * PR_ ;

    /// \brief Buffers for the Kajita's dynamic filter
    deque<COMState> deltaCOMTraj_deq_ ;

    deque<ZMPPosition> ZMPTraj_deq_ ;
    deque<COMState> COMTraj_deq_ ;
    deque<FootAbsolutePosition> LeftFootTraj_deq_ ;
    deque<FootAbsolutePosition> RightFootTraj_deq_ ;

    deque<ZMPPosition> ZMPTraj_deq_ctrl_ ;
    deque<COMState> COMTraj_deq_ctrl_ ;
    deque<FootAbsolutePosition> LeftFootTraj_deq_ctrl_ ;
    deque<FootAbsolutePosition> RightFootTraj_deq_ctrl_ ;

    /// \brief used to predict the next step using the current solution
    /// allow the computation of the complete preview
    vector< vector<double> > FootPrw_vec ;

    /// \brief Index where to begin the interpolation
    unsigned CurrentIndex_ ;

    /// \brief Interpolation Period for the dynamic filter
    double InterpolationPeriod_ ;

    /// \brief Step Period of the robot
    double StepPeriod_ ;

    /// \brief Period where the robot is on ONE feet
    double SSPeriod_ ;

    /// \brief Period where the robot is on TWO feet
    double DSPeriod_ ;

    /// \brief Maximum distance between the feet
    double FeetDistance_ ;

    /// \brief Maximum height of the feet
    double StepHeight_ ;

    /// \brief Height of the CoM
    double CoMHeight_ ;

    /// \brief Number of interpolated point needed for control computed
    /// during QP_T_
    unsigned NbSampleControl_ ;

    /// \brief Number of interpolated point needed for the dynamic filter
    /// computed during QP_T_
    unsigned NbSampleInterpolation_ ;

    COMState InitStateLIPM_ ;
    COMState InitStateOrientPrw_ ;
    COMState FinalCurrentStateOrientPrw_ ;
    COMState FinalPreviewStateOrientPrw_ ;

    /// \brief Standard polynomial trajectories for the feet.
    OnLineFootTrajectoryGeneration * OFTG_DF_ ;
    OnLineFootTrajectoryGeneration * OFTG_control_ ;

    DynamicFilter * dynamicFilter_ ;

  public:

    void GetZMPDiscretization
    (std::deque<ZMPPosition> & ZMPPositions,
     std::deque<COMState> & COMStates,
     std::deque<RelativeFootPosition> &RelativeFootPositions,
     std::deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
     std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
     double Xmax,
     COMState & lStartingCOMState,
     Eigen::Vector3d & lStartingZMPPosition,
     FootAbsolutePosition & InitLeftFootAbsolutePosition,
     FootAbsolutePosition & InitRightFootAbsolutePosition);

    void OnLineAddFoot
    (RelativeFootPosition & NewRelativeFootPosition,
     std::deque<ZMPPosition> & FinalZMPPositions,
     std::deque<COMState> & COMStates,
     std::deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
     std::deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
     bool EndSequence);

    int OnLineFootChange
    (double time,
     FootAbsolutePosition & aFootAbsolutePosition,
     deque<ZMPPosition> & FinalZMPPositions,
     deque<COMState> & CoMPositions,
     deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
     deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
     StepStackHandler * aStepStackHandler);
    
    void EndPhaseOfTheWalking
    (deque<ZMPPosition> & ZMPPositions,
     deque<COMState> & FinalCOMTraj_deq,
     deque<FootAbsolutePosition> & LeftFootAbsolutePositions,
     deque<FootAbsolutePosition> & RightFootAbsolutePositions);
    
    int ReturnOptimalTimeToRegenerateAStep();

    /// \brief Interpolation form the com jerk the position of the com and the
    /// zmp corresponding to the kart table model
    void CoMZMPInterpolation
    (std::deque<ZMPPosition> & ZMPPositions,                     // OUTPUT
     std::deque<COMState> & COMTraj_deq,                         // OUTPUT
     const std::deque<FootAbsolutePosition> & LeftFootTraj_deq,  // INPUT
     const std::deque<FootAbsolutePosition> & RightFootTraj_deq, // INPUT
     const solution_t * Solution,                                // INPUT
     LinearizedInvertedPendulum2D * LIPM,                        // INPUT/OUTPUT
     const unsigned numberOfSample,                              // INPUT
     const int IterationNumber,                                  // INPUT
     const unsigned int currentIndex);                           // INPUT
    
    /// \brief Interpolate just enough data to pilot the robot (period of
    ///    interpolation = QP_T_)
    void ControlInterpolation
    (std::deque<COMState> & FinalCOMTraj_deq,                      // OUTPUT
     std::deque<ZMPPosition> & FinalZMPTraj_deq,                   // OUTPUT
     std::deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,     // OUTPUT
     std::deque<FootAbsolutePosition> & FinalRightFootTraj_deq,    // OUTPUT
     double time);                                                 // INPUT
  
    /// \brief Interpolation everything on the whole preview
    void DynamicFilterInterpolation(double time);

    /// \brief Define the position of an additionnal foot step outside
    /// the preview to interpolate the position of the swinging feet in 3D
    void InterpretSolutionVector();

    /// \brief Prepare the vecteur containing the solution for the interpolation
    void PrepareSolution();

    /// \brief Project the found third foot step on the constraints
    void ProjectionOnConstraints(double & X, double & Y);

  };
}

#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.hh>
#endif /* _ZMPQP_WITH_CONSTRAINT_H_ */

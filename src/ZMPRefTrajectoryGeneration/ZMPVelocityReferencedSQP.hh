/*
 * Copyright 2015,
 *
 * Naveau Maximilien
 * Stasse Olivier
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
/*! Generate ZMP and CoM trajectories using Naveau2015RALETTER
 */

#ifndef ZMPVELOCITYREFERENCEDSQP_H
#define ZMPVELOCITYREFERENCEDSQP_H

#include <PreviewControl/LinearizedInvertedPendulum2D.hh>
#include <PreviewControl/rigid-body-system.hh>
#include <ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.hh>
#include <privatepgtypes.hh>
#include <jrl/walkgen/pgtypes.hh>
#include <ZMPRefTrajectoryGeneration/nmpc_generator.hh>
#include <ZMPRefTrajectoryGeneration/DynamicFilter.hh>

namespace PatternGeneratorJRL
{


  class ZMPDiscretization;
  class ZMPVelocityReferencedSQP : public ZMPRefTrajectoryGeneration
  {
    //
    // Public methods:
    //
  public:

    ZMPVelocityReferencedSQP(SimplePluginManager *SPM, string DataFile,
                            CjrlHumanoidDynamicRobot *aHS=0 );

    ~ZMPVelocityReferencedSQP();


    /// \brief Handle plugins (SimplePlugin interface)
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
                   deque<COMState> & FinalCoMPositions_deq,
                   deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
                   deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
                   FootAbsolutePosition & InitLeftFootAbsolutePosition,
                   FootAbsolutePosition & InitRightFootAbsolutePosition,
                   deque<RelativeFootPosition> &RelativeFootPositions,
                   COMState & lStartingCOMState,
                   MAL_S3_VECTOR_TYPE(double) & lStartingZMPPosition);


    /// \brief Update the stacks on-line
    void OnLine(double time,
                deque<ZMPPosition> & FinalZMPPositions,
                deque<COMState> & FinalCOMTraj_deq,
                deque<FootAbsolutePosition> &FinalLeftFootTraj_deq,
                deque<FootAbsolutePosition> &FinalRightFootTraj_deq);


    /// \name Accessors and mutators
    /// \{
    /// \brief Set the reference (velocity only as for now) through the Interface (slow)
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
    { return Running_; }

    /// \brief Set the final-stage trigger
    inline void EndingPhase(bool EndingPhase)
    { EndingPhase_ = EndingPhase;}

    void setCoMPerturbationForce(double x,double y);
    void setCoMPerturbationForce(istringstream &strm);

    inline const int & SQP_N(void) const
    { return SQP_N_; }

    /// \brief Setter and getter for the ComAndZMPTrajectoryGeneration.
    inline ComAndFootRealization * getComAndFootRealization()
    { return dynamicFilter_->getComAndFootRealization();}
    /// \}

    //
    // Private members:
    //
  private:

    /// \brief (Updated) Reference
    reference_t VelRef_;
    /// \brief Temporary (updating) reference
    reference_t NewVelRef_;

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
    MAL_VECTOR(PerturbationAcceleration_,double);

    /// \brief Sampling period considered in the QP
    double SQP_T_;

    /// \brief Nb. samplings inside preview window
    int SQP_N_;

    /// \brief 2D LIPM to simulate the evolution of the robot's CoM.
    LinearizedInvertedPendulum2D LIPM_ ;

    /// \brief Index where to begin the interpolation
    unsigned CurrentIndex_ ;

    /// \brief Generator of QP problem
    NMPCgenerator * NMPCgenerator_;

    /// \brief Previewed Solution
    solution_t solution_;

    /// \brief HDR, humanoid dyamic robot
    CjrlHumanoidDynamicRobot * HDR_ ;
    double RobotMass_ ;

    /// \brief Buffers for the Kajita's dynamic filter
    std::deque<COMState> deltaCOMTraj_deq_ ;
    // subsampled trajectory m_interpolationPeriod
    std::deque<ZMPPosition> ZMPTraj_deq_ ;
    std::deque<COMState> COMTraj_deq_ ;
    std::deque<FootAbsolutePosition> LeftFootTraj_deq_ ;
    std::deque<FootAbsolutePosition> RightFootTraj_deq_ ;
    // full trajectory (m_samplingPeriod)
    std::deque<ZMPPosition> ZMPTraj_deq_ctrl_ ;
    std::deque<COMState> COMTraj_deq_ctrl_ ;
    std::deque<FootAbsolutePosition> LeftFootTraj_deq_ctrl_ ;
    std::deque<FootAbsolutePosition> RightFootTraj_deq_ctrl_ ;
    /// \brief Duration of the preview for filtering
    double previewDuration_ ;
    /// \brief Size of the preview for filtering
    int previewSize_ ;
    /// \brief Number of interpolated point needed for control computed during QP_T_
    unsigned NbSampleControl_ ;
    /// \brief Number of interpolated point needed for the dynamic filter computed during QP_T_
    unsigned NbSampleInterpolation_ ;
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

    /// \brief Standard polynomial trajectories for the feet.
    OnLineFootTrajectoryGeneration * OFTG_ ;

    DynamicFilter * dynamicFilter_ ;

  public:

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
                       std::deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
                       std::deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
                       bool EndSequence);

    int OnLineFootChange(double time,
                         FootAbsolutePosition & aFootAbsolutePosition,
                         deque<ZMPPosition> & FinalZMPPositions,
                         deque<COMState> & CoMPositions,
                         deque<FootAbsolutePosition> & FinalLeftFootTraj_deq,
                         deque<FootAbsolutePosition> & FinalRightFootTraj_deq,
                         StepStackHandler * aStepStackHandler);

    void EndPhaseOfTheWalking(deque<ZMPPosition> & ZMPPositions,
                              deque<COMState> & FinalCOMTraj_deq,
                              deque<FootAbsolutePosition> & LeftFootAbsolutePositions,
                              deque<FootAbsolutePosition> & RightFootAbsolutePositions);

    int ReturnOptimalTimeToRegenerateAStep();

    /// \brief Interpolate just enough data to pilot the robot (period of interpolation = QP_T_)
    /// uses
    void FullTrajectoryInterpolation(double time); // INPUT
    /// \brief Interpolation form the com jerk the position of the com and the zmp corresponding to the kart table model
    void CoMZMPInterpolation(
        std::vector<double> JerkX,           // INPUT
        std::vector<double> JerkY,           // INPUT
        LinearizedInvertedPendulum2D * LIPM, // INPUT/OUTPUT
        const unsigned numberOfSample,       // INPUT
        const int IterationNumber,           // INPUT
        const unsigned int currentIndex);    // INPUT
  };
}

#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.hh>
#endif // ZMPVELOCITYREFERENCESQP_H

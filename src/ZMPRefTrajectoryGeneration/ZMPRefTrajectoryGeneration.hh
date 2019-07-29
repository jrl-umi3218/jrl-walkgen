/*
 * Copyright 2010,
 *
 * Paul Evrard
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
/*! \file ZMPRefTrajectoryGeneration.h
  \brief  This object provides the basic functionnalities
  for ZMP reference trajectory generation. */
#ifndef _ZMPREF_TRAJ_GEN_H_
#define _ZMPREF_TRAJ_GEN_H_


#include <deque>
#include <string>
//#define FULL_POLYNOME


#include <jrl/walkgen/pgtypes.hh>
#include <SimplePlugin.hh>

namespace PatternGeneratorJRL
{
  class StepStackHandler;

  /*! This class defines an abstract interface to generate 
    ZMP reference trajectory
    and its associate CoM trajectory.

    This class handles \f$ T_{DS} \f$ and \f$ T_{SS} \f$ which are
    respectively the double support time (m_Tdble) and the single phase time
    (m_Tsingle).

    The control sampling time \f$ \Delta T_{control} \f$ is handled by
    m_SamplingPeriod.

    The modulation of the single support phase defines how much time is spent
    in the landing and the take off.


  */
  class  ZMPRefTrajectoryGeneration : public SimplePlugin
  {
  protected:

    /* ! \brief Time for single support. */
    double m_Tsingle;

    /* ! \brief Time for double support. */
    double m_Tdble;

    /* ! \brief Sampling period */
    double m_SamplingPeriod;

    /* ! \brief ModulationSupportCoefficient coeeficient to wait a little
       before foot is of the ground */
    double m_ModulationSupportCoefficient;

    /* ! \brief The foot orientation for the lift off and the landing */
    double m_Omega;

    /* ! \brief Preview control window in second. */
    double m_PreviewControlTime;

    /* ! \brief Step height for the walking pattern. */
    double m_StepHeight;

    /* ! \brief Current time in the control loop (in seconds). */
    double m_CurrentTime;

    /*! \brief Specifies if we are or not in on line mode. */
    bool m_OnLineMode;

    /*! \brief Specifies Com Height. */
    double m_ComHeight;

  public:

    /* ! \brief Default constructor */
    ZMPRefTrajectoryGeneration(SimplePluginManager * lSPM);

    /* ! \brief Default destructor. */
    virtual ~ZMPRefTrajectoryGeneration() {};

    /* ! \brief Set the Preview control time window. */
    inline void SetTimeWindowPreviewControl(const double & aTW)
    {
      m_PreviewControlTime = aTW;
    };

    /* ! \brief Get the preview control time window. */
    inline const double & GetTimeWindowPreviewControl( ) const
    {
      return m_PreviewControlTime;
    };

    /* ! \brief Set for the foot angle on landing and taking off. */
    inline void SetOmega(const double & anOmega)
    {
      m_Omega = anOmega;
    };

    /* ! \brief Get the foot angle on landing and taking off. */
    inline const double & GetOmega(void) const
    {
      return m_Omega;
    };

    /* ! \brief Returns the Com Height. */
    inline const double & GetComHeight() const
    {
      return m_ComHeight;
    };

    /* ! \brief Returns the Com Height. */
    inline void SetComHeight(const double & aComHeight)
    {
      m_ComHeight = aComHeight;
    };

    /* ! \brief Returns the single support time. */
    inline const double & GetTSingleSupport() const
    {
      return m_Tsingle;
    };

    /* ! \brief Set the single support time. */
    inline void SetTSingleSupport(const double & aTSingleSupport)
    {
      m_Tsingle = aTSingleSupport;
    };

    /* ! \brief Returns the double support time. */
    inline const double & GetTDoubleSupport() const
    {
      return m_Tdble;
    };

    /* ! \brief Set the double support time. */
    inline void SetTDoubleSupport(const double & aTdble)
    {
      m_Tdble = aTdble;
    };

    /* ! \brief Get the sampling period for the control, 
       set to 0.005 by default. */
    inline const double & GetSamplingPeriod() const
    {
      return m_SamplingPeriod;
    };

    /* ! \brief Set the sampling period for the control. */
    inline void SetSamplingPeriod(const double &aSamplingPeriod)
    {
      m_SamplingPeriod = aSamplingPeriod;
    };

    /* ! \brief Returns the step height. */
    inline const double & GetStepHeight() const
    {
      return m_StepHeight;
    };

    /* ! \brief Specify the step height. */
    inline void SetStepHeight(const double & aSSH)
    {
      m_StepHeight = aSSH;
    };

    /* ! \brief Returns the ModulationSupportCoefficient. */
    inline const double &GetModulationSupportCoefficient() const
    {
      return m_ModulationSupportCoefficient;
    }

    /* !  \brief Specifies the modulation support coefficient. */
    inline void  SetModulationSupportCoefficient(const double &af)
    {
      m_ModulationSupportCoefficient = af;
    }

    /* !  \brief Set the ModulationSupportCoefficient.*/
    void SetModulationSupportCoefficient(double);


    /**  \brief Generate ZMP discreatization from a vector of foot position.
         ASSUME A COMPLETE MOTION FROM END TO START, and GENERATE EVERY VALUE.

         @param[out] ZMPPositions: Returns the ZMP reference values 
         for the overall motion.
         Those are absolute position in the world reference frame. 
         The origin is the initial
         position of the robot. The relative foot position specified are added.

         @param[out] COMStates: Returns the COM reference values for 
         the overall motion.
         Those are absolute position in the world reference frame. 
         The origin is the initial
         position of the robot. The relative foot position specified are added.

         @param[in] RelativeFootPositions: The main entry to this method: 
         the set of
         relative steps to be performed by the robot.

         @param[out] LeftFootAbsolutePositions: Returns the absolute 
         position of the left foot.
         According to the macro FULL_POLYNOME the trajectory will 
         follow a third order
         polynom or a fifth order. By experience it is wise to put 
         a third order.
         A null acceleration might cause problem for the compensation 
         of the Z-axis momentum.

         @param[out] RightFootAbsolutePositions: Returns 
         the absolute position of the right foot.


         @param Xmax: Returns the maximal distance of a hand along 
         the X axis in the waist coordinates.

         @param[in] lStartingCOMState: The initial position of the CoM.

         @param[in] lStartingZMPPosition: The initial position of the ZMP.

         @param[in] InitLeftFootAbsolutePosition: The initial position of 
         the left foot.

         @param[in] InitRightFootAbsolutePosition: The initial position of 
         the right foot.


    */
    virtual void GetZMPDiscretization
    (std::deque<ZMPPosition> & ZMPPositions,
     std::deque<COMState> & COMStates,
     std::deque<RelativeFootPosition> &RelativeFootPositions,
     std::deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
     std::deque<FootAbsolutePosition> &RightFootAbsolutePositions,
     double Xmax,
     COMState & lStartingCOMState,
     Eigen::Vector3d & lStartingZMPPosition,
     FootAbsolutePosition & InitLeftFootAbsolutePosition,
     FootAbsolutePosition & InitRightFootAbsolutePosition) = 0;
    
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

      @param[out] ZMPPositions: The queue of ZMP reference positions.
      @param[out] COMStates: The queue of COM reference positions.
      @param[out] FinalLeftFootAbsolutePositions: The queue of 
      left foot absolute positions.
      @param[out] FinalRightFootAbsolutePositions: The queue of 
      right foot absolute positions.
      @param[in] InitLeftFootAbsolutePosition: The initial position of 
      the left foot.
      @param[in] InitRightFootAbsolutePosition: The initial position of 
      the right foot.
      @param[in] RelativeFootPositions: The set of relative positions 
      to be taken into account.
      @param[in] lStartingCOMState: The initial position of the CoM 
      given as a 3D vector.
      @param[in] lStartingZMPPosition: The initial position of the 
      ZMP given as a 3D vector.
    */
    virtual std::size_t InitOnLine
    (std::deque<ZMPPosition> & ZMPPositions,
     std::deque<COMState> & COMStates,
     std::deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
     std::deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
     FootAbsolutePosition & InitLeftFootAbsolutePosition,
     FootAbsolutePosition & InitRightFootAbsolutePosition,
     std::deque<RelativeFootPosition> &RelativeFootPositions,
     COMState & lStartingCOMState,
     Eigen::Vector3d & lStartingZMPPosition)   = 0;




    /* ! Methods to update the stack on-line by inserting a new foot position.
       This method is only ADDING a new step position, this is not a 
       replacement.
       It is assumed that the removal of the foot is done outside this method.
       @param[in] NewRelativeFootPosition: The relative foot position 
       to be added.
       @param[out] FinalZMPPositions: The ZMP reference trajectory 
       deduced from the foot
       position.
       @param[out] FinalLeftFootAbsolutePositions: The left foot absolute
       trajectory obtained
       from the new foot positions.
       @param[out] FinalRightFootAbsolutePositions: The right foot absolute
       trajectory
       obtained from the new foot trajectories.
    */
    virtual void OnLineAddFoot
    (RelativeFootPosition & NewRelativeFootPosition,
     std::deque<ZMPPosition> & FinalZMPPositions,
     std::deque<COMState> & COMStates,
     std::deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
     std::deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
     bool EndSequence) = 0;
    
    /* ! \brief Method to change to update on line the queues necessary 
       of the system.
       @param[in] time : Current time.
       @param[out] FinalZMPPositions: The ZMP reference trajectory deduced 
       from the foot position.
       @param[out] COMStates: The CoM reference trajectory deduced from 
       the foot
       position.
       @param[out] FinalLeftFootAbsolutePositions: The left foot absolute
       trajectory obtained from the new foot positions.
       @param[out] FinalRightFootAbsolutePositions: The right foot absolute
       trajectory obtained from the new foot trajectories.

       @return If the method failed it returns -1, 0 otherwise.
    */
    virtual void OnLine
    (double time,
     std::deque<ZMPPosition> & FinalZMPPositions,
     std::deque<COMState> & COMStates,
     std::deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
     std::deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)=0;
    
    /*! \brief Method to stop walking.
      @param[out] ZMPPositions: The queue of ZMP reference positions.
      @param[out] FinalCOMStates: The queue of COM reference positions.
      @param[out] LeftFootAbsolutePositions: The queue of left 
      foot absolute positions.
      @param[out] RightFootAbsolutePositions: The queue of right 
      foot absolute positions.
    */
    virtual void EndPhaseOfTheWalking
    (std::deque<ZMPPosition> &ZMPPositions,
     std::deque<COMState> &FinalCOMStates,
     std::deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
     std::deque<FootAbsolutePosition> &RightFootAbsolutePositions)=0;
    

    /* ! \brief Method to change on line the landing position of a foot.
       @param[in] time : Current time.
       @param[out] FinalZMPPositions: The ZMP reference trajectory 
       deduced from the foot position.
       @param[out] COMStates: The CoM reference trajectory deduced from the foot
       position.
       @param[out] FinalLeftFootAbsolutePositions: The left foot absolute
       trajectory obtained from the new foot positions.
       @param[out] FinalRightFootAbsolutePositions: The right foot absolute
       trajectory obtained from the new foot trajectories.

       @return If the method failed it returns -1, 0 otherwise.
    */
    virtual int OnLineFootChange
    (double time,
     FootAbsolutePosition &aFootAbsolutePosition,
     std::deque<ZMPPosition> & FinalZMPPositions,
     std::deque<COMState> & COMStates,
     std::deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
     std::deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
     StepStackHandler * aStepStackHandler)=0;
    

    /*! \brief Return the time at which it is optimal to regenerate 
      a step in online mode.
     */
    virtual int ReturnOptimalTimeToRegenerateAStep()=0;

    /*! Handling methods for the plugin mecanism. */
    virtual void CallMethod(std::string & Method, std::istringstream &strm);


    /*! \name Methods related to the current time of the control loop.
      @{
    */

    /*! Set the current time. */
    void SetCurrentTime(const double & aTime)
    {
      m_CurrentTime = aTime;
    }

    /*! Get the current time. */
    double GetCurrentTime()
    {
      return m_CurrentTime;
    }
    /*! @} */

    /*! \name Methods related to the on line status generation of the ZMP.
      @{
    */
    /*! \brief Returns the current status of the ZMP trajectory generator.
      The online mode is determines internally.
      A ZMP-generator can be still in on-line mode even the step-generator
      is not because the ZMP-generator is generating the ending phase.
    */
    bool GetOnLineMode();
    /*! @}  */



  };
}

#include <StepStackHandler.hh>
#endif /* _ZMPREF_TRAJ_GEN_H_ */

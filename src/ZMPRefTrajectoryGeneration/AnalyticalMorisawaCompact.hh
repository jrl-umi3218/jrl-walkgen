/*
 * Copyright 2008, 2009, 2010,
 *
 * Alireza Nakhaei
 * Olivier  Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of jrl-walkgen.
 * jrl-walkgen is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * jrl-walkgen is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with jrl-walkgen.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/*! \file AnalyticalMorisawaCompact.h
  \brief Compact form of the analytical solution to generate the ZMP 
  and the CoM.

  This object generate the reference value for the
  ZMP based on a polynomial representation
  of the ZMP following
  "Experimentation of Humanoid Walking Allowing Immediate
  Modification of Foot Place Based on Analytical Solution"
  Morisawa, Harada, Kajita, Nakaoka, Fujiwara, Kanehiro, Hirukawa,
  ICRA 2007, 3989--39994
*/
#ifndef _ANALYTICAL_MORISAWA_FULL_H_
#define _ANALYTICAL_MORISAWA_FULL_H_



#include <Clock.hh>
#include <Mathematics/PolynomeFoot.hh>
#include <Mathematics/ConvexHull.hh>
#include <Mathematics/AnalyticalZMPCOGTrajectory.hh>
#include <PreviewControl/PreviewControl.hh>
#include <ZMPRefTrajectoryGeneration/AnalyticalMorisawaAbstract.hh>
#include "FilteringAnalyticalTrajectoryByPreviewControl.hh"
#include "LeftAndRightFootTrajectoryGenerationMultiple.hh"
#include <ZMPRefTrajectoryGeneration/DynamicFilter.hh>

namespace PatternGeneratorJRL
{
  /*!     @ingroup analyticalformulation
    Structure to specify the parameter of a trajectory. */
  typedef struct
  {
    /* ! Initial value of the CoM for the spline representation. */
    double InitialCoM;

    /*! Initial value of the CoM speed for the spline representation */
    double InitialCoMSpeed;

    /*! Final value of the CoM for the spline representation */
    double FinalCoMPos;

    /*! ZMP profil for the chosen axis. */
    std::vector<double> ZMPProfil;

    /*! Height ZMP profil.*/
    std::vector<double> ZMPZ;

    /*! Height COM profil. */
    std::vector<double> CoMZ;

  } CompactTrajectoryInstanceParameters;

  /*!     @ingroup analyticalformulation
    This structure is used to compute the resultant fluctuation
    to move from one set of ZMP-COM trajectories to an other. */
  typedef struct
  {
    double CoMInit, CoMNew;
    double CoMSpeedInit, CoMSpeedNew;
    double ZMPInit, ZMPNew;
    double ZMPSpeedInit, ZMPSpeedNew;
  } FluctuationParameters;

  /*! \brief Class to compute analytically in a compact 
    form the trajectories of both the ZMP and the CoM.
    @ingroup analyticalformulation
  */
  class  AnalyticalMorisawaCompact: public AnalyticalMorisawaAbstract
  {

  public:

    /*! \name Constants to handle errors
      when changing foot steps.
      @{ */
    const static int ERROR_WRONG_FOOT_TYPE = -1;
    const static int ERROR_TOO_LATE_FOR_MODIFICATION = -2;
    /*! @} */

    /*! \name Constants to handle reference frame when 
      changing the feet on-line
      @{ */
    const static unsigned int ABSOLUTE_FRAME = 0;
    const static unsigned int RELATIVE_FRAME = 1;
    /*! @} */
    /*! Constructor */
    AnalyticalMorisawaCompact(SimplePluginManager * lSPM,
                              PinocchioRobot *aPR);

    /*! Destructor */
    virtual ~AnalyticalMorisawaCompact();

    /*! \name Methods inherited from ZMPRefTrajectoryGeneration 
      and reimplemented
      @{ */

    /*! Returns the CoM and ZMP trajectory for some relative foot positions.
      Generate ZMP discreatization from a vector of foot position.
      ASSUME A COMPLETE MOTION FROM END TO START, and GENERATE EVERY VALUE.

      @param[out] ZMPPositions: Returns the ZMP reference values 
      for the overall motion.
      Those are absolute position in the world reference frame. 
      The origin is the initial
      position of the robot. The relative foot position specified are added.

      @param[out] CoMStates: Returns the CoM reference values for 
      the overall motion.
      Those are absolute position in the world reference frame. 
      The origin is the initial
      position of the robot. The relative foot position specified are added.

      @param[in] RelativeFootPositions: The set of
      relative steps to be performed by the robot.

      @param[out] LeftFootAbsolutePositions: Returns the 
      absolute position of the left foot.
      According to the macro FULL_POLYNOME the trajectory will 
      follow a third order
      polynom or a fifth order. By experience it is wise to put a third order.
      A null acceleration might cause problem for the compensation of 
      the Z-axis momentum.

      @param[out] RightFootAbsolutePositions: Returns the 
      absolute position of the right foot.


      @param[in] Xmax: The maximal distance of a hand along the 
      X axis in the waist coordinates.

      @param[in] lStartingCOMState: The initial position of the CoM.

      @param[in] lStartingZMPPosition: The initial position of the ZMP.

      @param[in] InitLeftFootAbsolutePosition: The initial position 
      of the left foot.

      @param[in] InitRightFootAbsolutePosition: The initial position 
      of the right foot.
    */
    void GetZMPDiscretization
    (deque<ZMPPosition> & ZMPPositions,
     deque<COMState> & CoMStates,
     deque<RelativeFootPosition> &RelativeFootPositions,
     deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
     deque<FootAbsolutePosition> &RightFootAbsolutePositions,
     double Xmax,
     COMState & lStartingCOMState,
     Eigen::Vector3d &lStartingZMPPosition,
     FootAbsolutePosition & InitLeftFootAbsolutePosition,
     FootAbsolutePosition & InitRightFootAbsolutePosition);
    
    /*! \brief Methods for on-line generation.
      The queues will be updated as follows:
      - The first values necessary to start walking will be inserted.
      - The initial positions of the feet will be taken into account
      according to InitLeftFootAbsolutePosition and
      InitRightFootAbsolutePosition.
      - The RelativeFootPositions stack will be taken into account,
      in this case only three steps will be removed from the stack,
      - The starting COM Position.
      Returns the number of steps which has been completely put inside
      the queue of ZMP, and foot positions.

      @param[out] FinalZMPPositions: The queue of ZMP reference positions.
      @param[out] CoMStates: The queue of COM reference positions.
      @param[out] FinalLeftFootAbsolutePositions: 
      The queue of left foot absolute positions.
      @param[out] FinalRightFootAbsolutePositions: 
      The queue of right foot absolute positions.
      @param[in] InitLeftFootAbsolutePosition: 
      The initial position of the left foot.
      @param[in] InitRightFootAbsolutePosition: 
      The initial position of the right foot.
      @param[in] RelativeFootPositions: 
      The set of relative positions to be taken into account.
      @param[in] lStartingCOMState: 
      The initial position of the CoM given as a 3D vector.
      @param[in] lStartingZMPPosition: 
      The initial position of the ZMP given as a 3D vector.
    */
    std::size_t InitOnLine
    (deque<ZMPPosition> & FinalZMPPositions,
     deque<COMState> & CoMStates,
     deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
     deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
     FootAbsolutePosition & InitLeftFootAbsolutePosition,
     FootAbsolutePosition & InitRightFootAbsolutePosition,
     deque<RelativeFootPosition> &RelativeFootPositions,
     COMState &lStartingCOMState,
     Eigen::Vector3d & lStartingZMPPosition
     );

    /* ! \brief Methods to update the stack on-line by inserting a 
       new foot position.
       The foot is put right at the end of the stack. This method 
       is supposed to be called
       when the first foot in the stack is finished.

       @param[in] NewRelativeFootPosition: The new foot to put in the stack.
       @param[out] FinalZMPPositions: The stack of final ZMP positions 
       to be updated
       with the new relative foot position.
       @param[out] FinalLeftFootAbsolutePositions: 
       The stack of final absolute left foot positions
       according to the new relative foot position.
       @param[out] FinalRightFootAbsolutePositions: 
       The stack of final absolute right foot positions
       according to the new relative foot position.
       @param[in] EndSequence: Inherited from abstract interface and unused.

    */
    void OnLineAddFoot
    (RelativeFootPosition & NewRelativeFootPosition,
     deque<ZMPPosition> & FinalZMPPositions,
     deque<COMState> & CoMStates,
     deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
     deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
     bool EndSequence);
    
    /* ! \brief Method to update the stacks on-line */
    void OnLine(double time,
                deque<ZMPPosition> & FinalZMPPositions,
                deque<COMState> & CoMStates,
                deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
                deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions);

    /* ! \brief Method to change on line the landing position of a foot.
       @return If the method failed it returns -1, 0 otherwise.
    */
    int OnLineFootChange
    (double time,
     FootAbsolutePosition &aFootPosition,
     deque<ZMPPosition> & FinalZMPPositions,
     deque<COMState> & CoMStates,
     deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
     deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
     StepStackHandler *aStepStackHandler=0);
    
    /* ! \brief Method to change on line the landing position of several feet.
       @return If the method failed it returns -1, 0 otherwise.
    */
    int OnLineFootChanges
    (double time,
     deque<FootAbsolutePosition> &FeetPosition,
     deque<ZMPPosition> & FinalZMPPositions,
     deque<COMState> & CoMStates,
     deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
     deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
     StepStackHandler *aStepStackHandler=0);

    /*! \brief Method to stop walking.
      @param[out] ZMPPositions: The queue of ZMP reference positions.
      @param[out] FinalCOMStates: The queue of COM reference positions.
      @param[out] LeftFootAbsolutePositions: The queue of left foot 
      absolute positions.
      @param[out] RightFootAbsolutePositions: The queue of right foot 
      absolute positions.
    */
    void EndPhaseOfTheWalking
    (deque<ZMPPosition> &ZMPPositions,
     deque<COMState> &FinalCOMStates,
     deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
     deque<FootAbsolutePosition> &RightFootAbsolutePositions);
    


    /*! \brief Return the time at which it is optimal to regenerate 
      a step in online mode.
     */
    int ReturnOptimalTimeToRegenerateAStep();

    /*! @} */

    /*! \name Methods specifics to our current implementation.
      @{ */
    /*! \name Methods for the full linear system
      @{ */

    /*! \brief Building the Z matrix to be inverted.
      @param[in] lCoMZ: Profile of the CoM's height trajectory 
      for each interval.
      @param[in] lZMPZ: Profile of the ZMP's height trajectory 
      for each interval.
    */
    void BuildingTheZMatrix(std::vector<double> &lCoMZ,
                            std::vector<double> &lZMPZ );

    /*! \brief Building the Z matrix to be inverted. */
    void BuildingTheZMatrix();

    /*! \brief Building the w vector.
      It is currently assume that all ZMP's speed will be
      set to zero, as well as the final COM's speed.
      The sequence of ZMPSequence is the final value of the
      ZMP. As a special case, the first interval being set
      as a single support phase
      @param[in] InitialCoMPos: Initial Position of the CoM.
      @param[in] InitialComSpeed: Initial Speed of the CoM,
      @param[in] ZMPPosSequence: Set of position of the ZMP 
      for the end of each interval.
      @param[in] FinalCoMPos: Final position of the CoM.
      @param[in] aAZCT: The analytical trajectory which store 
      the coefficients we need
      to compute appropriatly the coefficients of \f[ w \f].

    */
    void ComputeW(double InitialCoMPos,
                  double InitialComSpeed,
                  std::vector<double> &ZMPPosSequence,
                  double FinalCoMPos,
                  AnalyticalZMPCOGTrajectory &aAZCT);

    /*! \brief Transfert the computed weights to an Analytical ZMP COG
      Trajectory able to generate the appropriate intermediates values.
      @param aAZCT: The object to be filled with the appropriate 
      intermediate values.
      @param lCoMZ: The height trajectory of the CoM.
      @param lZMPZ: The height trajectory of the ZMP.
      @param lZMPInit: The initial value of the ZMP.
      @param lZMPEnd : The final value of the ZMP.
      @param InitializeaAZCT: This boolean is ignored as aAZCT should
      be already initialized.
    */
    void TransfertTheCoefficientsToTrajectories
    (AnalyticalZMPCOGTrajectory &aAZCT,
     std::vector<double> &lCoMZ,
     std::vector<double> &lZMPZ,
     double &lZMPInit,
     double &lZMPEnd,
     bool InitializeaAZCT);
    /*! @} */

    /*! \brief Initialize automatically Polynomial degrees, 
      and temporal intervals.
      @return True if succeedeed, false otherwise.
    */
    bool InitializeBasicVariables();


    /*! \brief Compute the polynomial weights. */
    void ComputePolynomialWeights();

    /*! \brief Compute the polynomial weights. */
    void ComputePolynomialWeights2();

    /*! \brief Compute a trajectory with the given parameters.
      This method assumes that a Z matrix has already been computed. */
    void ComputeTrajectory(CompactTrajectoryInstanceParameters &aCTIP,
                           AnalyticalZMPCOGTrajectory &aAZCT);

    /*! \brief Reset internal variables to compute a new
      problem */
    void ResetTheResolutionOfThePolynomial();

    /*! \brief For the current time t, we will change the foot position
      (NewPosX, NewPosY) during time interval IndexStep and IndexStep+1, using
      the AnalyticalZMPCOGTrajectory objects and their parameters.
      IndexStep has to be a double support phase, because it determines
      the landing position. It is also assumes that m_RelativeFootPositions
      and m_AbsoluteSupportFootPositions are set for the new values.

      @param[in] t : The current time.
      @param[in] IndexStep: The index of the interval where 
      the modification will start.
      The modification of the foot position is done over 2 intervals.
      @param[in] NewFootAbsPos: The new foot position in absolute coordinates.
      @param[out] aAZCTX: The analytical trajectory along 
      the X axis which will be modified
      (if possible) to comply with the new position.
      @param[out] aCTIPX: Embed the initial and final conditions
      to generate the ZMP and CoM trajectories along the X axis.
      @param[out] aAZCTY: The analytical trajectory along 
      the Y axis which will be modified
      (if possible) to comply with the new position.
      @param[out] aCTIPY: Embed the initial and final conditions
      to generate the ZMP and CoM trajectories along the Y axis.
      @param[in] TemporalShift : If true, this authorize 
      the method to shift the time for the modified interval.
      @param[in] aStepStackHandler: Access to the stack of steps.
      @param[in] AddingAFootStep: In this the foot step specified in
      NewFootAbsPos is added at the end of the preview window.

      @return : Returns an error index if the operation was not feasible. 
      You should use
      string error message to get the corresponding error message.

    */
    int ChangeFootLandingPosition
    (double t,
     vector<unsigned int> & IndexStep,
     vector<FootAbsolutePosition> & NewFootAbsPos,
     AnalyticalZMPCOGTrajectory &aAZCTX,
     CompactTrajectoryInstanceParameters &aCTIPX,
     AnalyticalZMPCOGTrajectory &aAZCTY,
     CompactTrajectoryInstanceParameters &aCTIPY,
     bool TemporalShift,
     bool ResetFilters,
     StepStackHandler *aStepStackHandler,
     bool AddingAFootStep);

    /*! \brief For the current time t, we will change the foot position
      (NewPosX, NewPosY) during time interval IndexStep and IndexStep+1.
      IndexStep has to be a double support phase, because it determines
      the landing position.

      @param[in] t : The current time.
      @param[in] IndexStep: The index of the interval where 
      the modification will start.
      The modification of the foot position is done over 2 intervals.
      @param[in] NewFootAbsPos: The new foot position in the 
      absolute frame coordinates.
      @return : Returns an error index if the operation was 
      not feasible. You should use
      string error message to get the corresponding error message.

    */
    int ChangeFootLandingPosition
    (double t,
     vector<unsigned int> & IndexStep,
     vector<FootAbsolutePosition> & NewFootAbsPos);


    /*! @} */

    /*! Put an error messages string in ErrorMessage,
      according to ErrorIndex. */
    void StringErrorMessage(int ErrorIndex, string & ErrorMessage);

    /*! \brief This method filter out the orthogonal trajectory 
      to minimize the
      fluctuation involved by the time shift.
    */
    void FilterOutOrthogonalDirection
    (AnalyticalZMPCOGTrajectory & aAZCT,
     CompactTrajectoryInstanceParameters &aCTIP,
     deque<double> & ZMPTrajectory,
     deque<double> & CoGTrajectory);

    /*! \name Feet Trajectory Generator methods
      @{ */
    /*! Set the feet trajectory generator */
    void SetFeetTrajectoryGenerator
    (LeftAndRightFootTrajectoryGenerationMultiple *
     aFeetTrajectoryGenerator);

    /*! Get the feet trajectory generator */
    LeftAndRightFootTrajectoryGenerationMultiple *
    GetFeetTrajectoryGenerator();

    /*!  Setter and getter for the ComAndZMPTrajectoryGeneration.  */
    inline ComAndFootRealization * getComAndFootRealization()
    {
      return m_kajitaDynamicFilter->getComAndFootRealization();
    };

    /*! @} */

    /*! Simple plugin interfaces
      @{
    */
    /*! Register methods. */
    void RegisterMethods();

    /*! Call methods according to the arguments. */
    void CallMethod(std::string & Method, std::istringstream &strm);

    /*! @} */
  protected:


    /*! \name Internal Methods to compute the full linear
      system.
      @{
    */

    /*! \brief Building the Z1 Matrix */
    void ComputeZ1(unsigned int &lindex);

    /*! \brief Building the Zj Matrix
      @param intervalindex: Index of the interval,
      @param colindex: Index of the column inside the matrix,
      @param rowindex: Index of the row inside the matrix. */
    void ComputeZj(unsigned int intervalindex,
                   unsigned int &colindex,
                   unsigned int &rowindex);

    /*! \brief Building the Zm Matrix */
    void ComputeZm(unsigned int intervalindex,
                   unsigned int &colindex,
                   unsigned int &rowindex);

    /*! \brief Considering the current time given by LocalTime,
      it identifies by IndexStartingInterval which interval is concerned 
      by LocalTime.
      A new duration of the IndexStartingInterval is proposed in NewTj
      to be m_Tj[StartingIndexInterval]-LocalTime -
      sum[m_Tj[0..StartingIndexInterval-1]].
      LocalTime should be given in the local reference time.
    */
    int TimeChange(double LocalTime,
                   unsigned int IndexStep,
                   unsigned int &IndexStartingInterval,
                   double &FinalTime,
                   double &NewTj);

    /*! \brief Recomputing all the m_DeltaTj according to NewTime,
      and the index of the first interval. */
    void NewTimeIntervals(unsigned IndexStartingInterval,
                          double NewTime);

    /*! \brief Recompute the trajectories based on the current time (LocalTime),
      the new landing position and the time interval (IndexStep) when the
      modification should take place.
    */
    void ConstraintsChange(double LocalTime,
                           FluctuationParameters FPX,
                           FluctuationParameters FPY,
                           CompactTrajectoryInstanceParameters &aCTIPX,
                           CompactTrajectoryInstanceParameters &aCTIPY,
                           unsigned int IndexStartingInterval,
                           StepStackHandler *aStepStackHandler=0);

    /*! \brief Compute the time to compensate for the ZMP fluctuation. */
    double TimeCompensationForZMPFluctuation(FluctuationParameters
                                             &aFluctuationParameters,
                                             double DeltaTInit);

    /*! @} */

    /*! \name Internal Methods to generate steps and create the associated
      problem.

      @{
    */

    /*! \brief Build and solve the linear problem associated with a 
      set of relative footholds.
      @param[in] lStartingCOMState: Specify the initial condition of 
      the CoM \f$(x,y,z)\f$ for the
      resolution of the linear problem. The matrix is defined as:
      \f[
      \left(
      \begin{matrix}
      x & \dot{x} & \ddot{x} \\
      y & \dot{y} & \ddot{y} \\
      z & \dot{z} & \ddot{z} \\
      \end{matrix}
      \right)
      \f]
      @param[in] LeftFootInitialPosition: The initial position of the 
      left foot in the <b>absolute</b>
      reference frame.
      @param[in] RightFootInitialPosition: The initial position of the 
      right foot in the <b>absolute</b>
      reference frame.
      the absolute initial feet positions and the queue of relative foot
      positions. This is not
      a set of trajectories at each 5 ms, but the support foot absolute
      positions.
      Thus the size of this queue should be the same than the relative 
      foot positions.

      @param[in] IgnoreFirstRelativeFoot: Boolean to ignore 
      the first relative foot.
      @param[in] DoNotPrepareLastFoot:  Boolean to not perform for 
      the ending sequence.


    */
    int BuildAndSolveCOMZMPForASetOfSteps
    (Eigen::Matrix3d & lStartingCOMState,
     FootAbsolutePosition &LeftFootInitialPosition,
     FootAbsolutePosition &RightFootInitialPosition,
     bool IgnoreFirstRelativeFoot,
     bool DoNotPrepareLastFoot);

    /*! Change the profil of the ZMP profil according to the index of 
      the interval.
      \param IndexStep : Index of the interval to be changed. 
      The index can be higher than
      the size of the preview window (+1). In this case, the 
      step available in m_AbsPosition are
      used.
      \param aNewFootAbsPos : The absolute position of the step to be changed.
      \param aCTIPX : The trajectory parameters along the X axis.
      \param aCTIPY : The trajectory parameters along the Y axis.
    */
    void ChangeZMPProfil(vector<unsigned int> & IndexStep,
                         vector<FootAbsolutePosition> &aNewFootAbsPos,
                         CompactTrajectoryInstanceParameters &aCTIPX,
                         CompactTrajectoryInstanceParameters &aCTIPY);
    /*! @} */

    /*! \brief Fill the queues of CoM, ZMP and feet trajectories.
      The queues are filled by using the polynomial stored in
      m_AnalyticalZMPCoGTrajectoryX and
      m_AnalyticalZMPCoGTrajectoryY. The interval time used for this 
      is specified by StartingTime
      and EndTime. The period used to sample the trajectory is m_SamplingPeriod.

      \param StartingTime: The starting time to fill in the queues.
      \param EndTime: The ending time to fill in the queues.
      \param FinalZMPPositions: The queue of ZMP positions. 
      More specifically fill in \f$px\f$ and \f$py\f$.
      \param FinalCoMPositions: The queue of CoM positions. 
      More specifically fill in \f$x,\dot{x},y,\dot{y}, z\f$,
      \param FinalLeftFootAbsolutePositions: 
      The queue of Left Foot Absolute positions.
      \param FinalRightFootAbsolutePositions: 
      The queue of Right Foot Absolute positions.
    */
    void FillQueues
    (double StartingTime,
     double EndTime,
     deque<ZMPPosition> & FinalZMPPositions,
     deque<COMState> & FinalCoMPositions,
     deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
     deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions);
    
    void ComputeZMPz(double t,
                     ZMPPosition &ZMPz,
                     unsigned int IndexInterval);

    void ComputeCoMz(COMState & CoM,
                     FootAbsolutePosition & LeftFoot,
                     FootAbsolutePosition & RightFoot);
    void ComputeCoMz(double t,
                     unsigned int lIndexInterval,
                     COMState &CoMz,
                     deque<COMState> & FinalCoMPositions);

    void FillQueues
    (double samplingPeriod,
     double StartingTime,
     double EndTime,
     deque<ZMPPosition> & FinalZMPPositions,
     deque<COMState> & FinalCoMPositions,
     deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
     deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions);
    
    void ComputeOneElementOfTheQueue
    (unsigned int & lIndexInterval,
     unsigned int & lPrevIndexInterval,
     double t,
     ZMPPosition & FinalZMPPosition,
     COMState & FinalCoMPosition,
     FootAbsolutePosition & FinalLeftFootAbsolutePosition,
     FootAbsolutePosition & FinalRightFootAbsolutePosition);

    /*! \brief LU decomposition of the Z matrix. */
    Eigen::MatrixXd m_AF;

    /*! \brief Pivots of the Z matrix LU decomposition. */
    Eigen::Matrix<int, Eigen::Dynamic, 1> m_IPIV;

    /*! \brief Boolean on the need to reset to the
      precomputed Z matrix LU decomposition */
    bool m_NeedToReset;

    /*! \brief Pointer to the preview control object used to
      filter out the orthogonal direction . */
    PreviewControl *  m_PreviewControl;

    /*! \name Object to handle trajectories.
      @{
    */
    /*! \brief Analytical sagital trajectories */
    AnalyticalZMPCOGTrajectory *m_AnalyticalZMPCoGTrajectoryX;

    /*! \brief Analytical sagital trajectories */
    AnalyticalZMPCOGTrajectory *m_AnalyticalZMPCoGTrajectoryY;

    /*! \brief Analytical sagital trajectories */
    //  AnalyticalZMPCOGTrajectory *m_AnalyticalZMPCoGTrajectoryZ;

    /*! \brief Foot Trajectory Generator */
    LeftAndRightFootTrajectoryGenerationMultiple * m_FeetTrajectoryGenerator;
    LeftAndRightFootTrajectoryGenerationMultiple *
    m_BackUpm_FeetTrajectoryGenerator;
    /*! @} */

    /*! @} */


    /*! \brief Stores the relative foot positions currently in the buffer */
    deque<RelativeFootPosition> m_RelativeFootPositions;

    /*! \brief Stores the absolute support foot positions 
      currently in the buffer */
    deque<FootAbsolutePosition> m_AbsoluteSupportFootPositions;

    /*! \brief Store the currently realized support foot position.
      \warning This field makes sense only direct ON-LINE mode.
    */
    FootAbsolutePosition m_AbsoluteCurrentSupportFootPosition;

    /*! \name Stores the current ZMP profil, 
      initial and final conditions for the trajectories.
      @{ */

    /*! Along the X-axis. */
    CompactTrajectoryInstanceParameters m_CTIPX;

    /*! Along the Y-axis. */
    CompactTrajectoryInstanceParameters m_CTIPY;

    /*! \brief Upper time limit over which the stacks are not updated anymore
      when calling OnLine(). */
    double m_UpperTimeLimitToUpdateStacks;
    /*! @} */

    /*! \brief Clocks for code measurement in OnLine()*/
    Clock m_Clock1, m_Clock2, m_Clock3, m_Clock4;

    /*! \brief Boolean value to check if there is a new step
      in the stack. */
    bool m_NewStepInTheStackOfAbsolutePosition;

    /*! \brief Height of the initial CoM position. */
    double m_InitialPoseCoMHeight;

    /*! \brief On-line change step mode */
    unsigned int m_OnLineChangeStepMode;

    /*! \brief Filtering the axis through a preview control. */
    FilteringAnalyticalTrajectoryByPreviewControl * m_FilterXaxisByPC,
      * m_FilterYaxisByPC;
    DynamicFilter * m_kajitaDynamicFilter ;
    // deque sampled at m_SamplingPeriod
    deque<FootAbsolutePosition> ctrlLF_ ;
    deque<FootAbsolutePosition> ctrlRF_ ;
    deque<COMState>             ctrlCoM_ ;
    deque<ZMPPosition>          ctrlZMP_ ;
    // deque sampled at interpolation time
    deque<COMState>             intCoM_ ;
    deque<FootAbsolutePosition> intLF_  ;
    deque<FootAbsolutePosition> intRF_  ;
    // output of the filter
    deque<COMState>     outputDeltaCoM_ ;
    // size of the Dynamic Filter preview
    double DFpreviewWindowSize_ ;

    /*! \brief Activate or desactivate the filtering. */
    bool m_FilteringActivate;

    /*! \brief End phase */
    bool m_EndPhase;

    PatternGeneratorJRL::BSplinesFoot * m_CoMbsplinesZ ;
    PatternGeneratorJRL::Polynome3 * m_ZMPpolynomeZ ;

  public:
    /*! \name Methods related to the Preview Control object used
      by this class. @{ */
    /*! Allows to set the preview control object. */
    int SetPreviewControl(PreviewControl * aPreviewControl);

    /*! Get the preview control object. */
    PreviewControl * GetPreviewControl();
    /*! @} */

    /*! \brief Propagate Absolute Reference Time */
    void PropagateAbsoluteReferenceTime(double x);

  };
}
#endif

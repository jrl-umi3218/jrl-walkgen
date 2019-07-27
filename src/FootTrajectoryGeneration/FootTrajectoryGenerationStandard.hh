/*
 * Copyright 2008, 2009, 2010,
 *
 *
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
/*! \file FootTrajectoryGenerationStandard.h
  \brief This object generate all the values for the foot trajectories.
  @ingroup foottrajectorygeneration */


#ifndef _FOOT_TRAJECTORY_GENERATION_STANDARD_H_
#define _FOOT_TRAJECTORY_GENERATION_STANDARD_H_

/* Walking pattern generation related inclusions */

#include <FootTrajectoryGeneration/FootTrajectoryGenerationAbstract.hh>
#include <Mathematics/PolynomeFoot.hh>
#include <Mathematics/Bsplines.hh>
namespace PatternGeneratorJRL
{

  /** @ingroup foottrajectorygeneration
      This class generates a trajectory for the swinging foot during single 
      support phase.
      It uses a classical approach relying in polynome of 3rd orders for the
      position in the
      orthogonal plan as well as the direction.For the height modification 
      a 4th order polynome
      is used. Finally a landing and take off phase using an angular value
      (\f$\omega\f$).
  */
  class  FootTrajectoryGenerationStandard : public
  FootTrajectoryGenerationAbstract
  {
  public:

    /*!\name  Constants related to the direction for the generation of the 
      polynomes.
      @{ */

    /*! \brief along the frontal direction */
    const static unsigned int X_AXIS = 0;
    /*! \brief along the left of the robot */
    const static unsigned int Y_AXIS = 1;
    /*! \brief along the vertical axis of the robot. */
    const static unsigned int Z_AXIS = 2;
    /*! \brief Along the direction of the robot*/
    const static unsigned int THETA_AXIS = 3;
    /*! \brief Angle used by the swinging foot for taking off. */
    const static unsigned int OMEGA_AXIS = 4;
    /*! \brief Angle used by the swinging foot for landing */
    const static unsigned int OMEGA2_AXIS = 5;

    /* @} */

    /*! Constructor: In order to compute some appropriate strategies,
      this class needs to extract specific details from the humanoid model. */
    FootTrajectoryGenerationStandard(SimplePluginManager *lSPM, PRFoot *aFoot);

    /*! Default destructor. */
    virtual ~FootTrajectoryGenerationStandard();

    /*! This method computes the position of the swinging foot during 
      single support phase,
      and maintain a constant position for the support foot.
      It uses polynomial of 3rd order for the X-axis, Y-axis,
      orientation in the X-Z axis, and orientation in the X-Y axis,
      and finally it uses a 4th order polynome for the Z-axis.

      @param SupportFootAbsolutePositions: Queue of absolute position for 
      the support foot.
      This method will set the foot position at index CurrentAbsoluteIndex 
      of the queue.
      This position is supposed to be constant.
      @param NoneSupportFootAbsolutePositions: Queue of absolute position 
      for the swinging
      foot. This method will set the foot position at index
      NoneSupportFootAbsolutePositions
      of the queue.
      @param CurrentAbsoluteIndex: Index in the queues of the foot 
      position to be set.
      @param IndexInitial: Index in the queues which correspond to the 
      starting point of the current single support phase.
      @param ModulatedSingleSupportTime: Amount of time where the foot is flat.
      @param StepType: Type of steps (for book-keeping).
      @param LeftOrRight: Specify if it is left (1) or right (-1).
    */
    virtual void UpdateFootPosition
    (deque<FootAbsolutePosition>
     &SupportFootAbsolutePositions,
     deque<FootAbsolutePosition> &NoneSupportFootAbsolutePositions,
                                    int CurrentAbsoluteIndex,
                                    int IndexInitial,
                                    double ModulatedSingleSupportTime,
                                    int StepType,int LeftOrRight);

    virtual void UpdateFootPosition
    (deque<FootAbsolutePosition>
     &SupportFootAbsolutePositions,
     deque<FootAbsolutePosition> &NoneSupportFootAbsolutePositions,
     int StartIndex, int k,
     double LocalInterpolationStartTime,
     double ModulatedSingleSupportTime,
     int StepType, int LeftOrRight);


    /*! Initialize internal data structures.
      In this specific case, it is in charge of 
      creating the polynomial structures.
    */
    virtual void InitializeInternalDataStructures();

    /*! Free internal data structures.
      In this specific case, it is in charge of freeing 
      the polynomial data structures.
    */
    virtual void FreeInternalDataStructures();

    /*! This method specifies the parameters for each of the polynome 
      used by this
      object. In this case, as it is used for the 3rd order polynome. 
      The polynome to
      which those parameters are set is specified with PolynomeIndex.
      It assumes an initial position and an initial speed set to zero.
      @param[in] AxisReference: Set to which axis the parameters will 
      be applied.
      @param[in] TimeInterval: Set the time base of the polynome.
      @param[in] Position: Set the final position of the polynome at 
      TimeInterval.
    */
    int SetParameters(int AxisReference,
                      double TimeInterval,
                      double Position);

    /*! This method specifies the parameters for each of the polynome 
      used by this
      object. In this case, as it is used for the 3rd order polynome. 
      The polynome to
      which those parameters are set is specified with PolynomeIndex.
      @param[in] AxisReference: Set to which axis the parameters 
      will be applied.
      @param[in] TimeInterval: Set the time base of the polynome.
      @param[in] FinalPosition: Set the final position of the polynome at
      TimeInterval.
      @param[in] InitPosition: Initial position when computing the polynome at 
      t=0.0.
      @param[in] InitSpeed: Initial speed when computing the polynome at t=0.0.
    */
    int SetParametersWithInitPosInitSpeed
    (int AxisReference,
     double TimeInterval,
     double FinalPosition,
     double InitPosition,
     double InitSpeed,
     vector<double> MiddlePos=vector<double>(3,-1));

    /*! Overloading -- BSPlines Init Function
      This method specifies the parameters for each of the Bsplines used by 
      this object.
      @param PolynomeIndex: Set to which axis the parameters will be applied.
      @param AxisReference: Index to the axis to be used.
      @param TimeInterval: Set the time base of the polynome.
      @param FinalTime: The final time of the BSpline
      @param FinalPosition: Set the final position of the spline at FinalTime.
      @param TimeMaxPosition: Set when the spline reaches the highest value.
      @param MaxPosition: Max value of the function.
    */

    int SetParametersWithInitPosInitSpeed(int AxisReference,
                                          double TimeInterval,
                                          double FinalTime,
                                          double FinalPosition,
                                          double TimeMaxPosition,
                                          double MaxPosition,
                                          double InitSpeed,
                                          double InitPosition);

    /*! This method get the parameters for each of the polynome used by this
      object. In this case, as it is used for the 3rd order polynome. 
      The polynome to
      which those parameters are set is specified with PolynomeIndex.
      @param[in] AxisReference: Set to which axis the parameters 
      will be applied.
      @param[in] TimeInterval: Set the time base of the polynome.
      @param[in] FinalPosition: Set the final position of the polynome 
      at TimeInterval.
      @param[in] InitPosition: Initial position when computing the polynome at 
      t=0.0.
      @param[in] InitSpeed: Initial speed when computing the polynome at t=0.0.
    */
    int GetParametersWithInitPosInitSpeed(int AxisReference,
                                          double &TimeInterval,
                                          double &FinalPosition,
                                          double &InitPosition,
                                          double &InitSpeed);



    /// \brief Set parameters considering initial position, speed, acceleration.
    ///
    /// \param[in] PolynomeIndex
    /// \param[in] TimeInterval
    /// \param[in] FinalPosition
    /// \param[in] InitPosition
    /// \param[in] InitSpeed
    /// \param[in] InitAcc
    int SetParameters(int PolynomeIndex, double TimeInterval,
                      double FinalPosition,
                      double InitPosition,
                      double InitSpeed,
                      double InitAcc,
                      double InitJerk);

    /// \brief Set parameters considering initial position, speed, acceleration.
    ///
    /// \param[in] PolynomeIndex
    /// \param[in] TimeInterval
    /// \param[in] FinalPosition
    /// \param[in] InitPosition
    /// \param[in] InitSpeed
    /// \param[in] InitAcc
    int SetParameters(int PolynomeIndex, double TimeInterval,
                      double FinalPosition, double InitPosition,
                      double InitSpeed, double InitAcc,
                      std::vector<double> MiddlePos=vector<double>(3,-1) );

    /*! Fill an absolute foot position structure for a given time. */
    // Using Polynoms
    double ComputeAllWithPolynom(FootAbsolutePosition & aFootAbsolutePosition,
                                 double Time);

    // Using BSplines
    double ComputeAllWithBSplines(FootAbsolutePosition & aFootAbsolutePosition,
                                  double Time);

    /*! Compute the value for a given polynome. */
    double Compute(unsigned int PolynomeIndex, double Time);

    /*! Compute the value for a given polynome's second derivative. */
    double ComputeSecDerivative(unsigned int PolynomeIndex, double Time);

    /*! Compute the absolute foot position from the queue of relative positions.
      There is not direct dependency with time.
    */
    void ComputingAbsFootPosFromQueueOfRelPos
    (deque<RelativeFootPosition>
     &RelativeFootPositions,
     deque<FootAbsolutePosition> &AbsoluteFootPositions);

    /*! Methods to compute a set of positions for the feet according to the 
      discrete time given in parameters and the phase of walking.
      @{
    */

    /*! @} */

    void print();

    void copyPolynomesFromFTGS (FootTrajectoryGenerationStandard * FTGS);

  protected:

    /*! \brief Polynomes for X and Y axis positions*/
    Polynome5 *m_PolynomeX,*m_PolynomeY;

    /*! \brief Polynome for X-Y orientation */
    Polynome5 *m_PolynomeTheta;

    /*! \brief Polynome for Y-Z orientation */
    Polynome3 *m_PolynomeOmega, *m_PolynomeOmega2;

    /*! \brief Polynome for Z axis position. */
    Polynome6 *m_PolynomeZ;

    /*! \brief Bsplines for Z axis position. */
    BSplinesFoot *m_BsplinesZ;

    /** Definition of BSplines X Y*/
    BSplinesFoot *m_BsplinesX;
    BSplinesFoot *m_BsplinesY;


    /*! \brief Foot dimension. */
    double m_FootB, m_FootH, m_FootF;

    /*! \brief Position of the ankle in the left foot. */
    Eigen::Vector3d m_AnklePositionLeft;

    /*! \brief Position of the ankle in the right foot. */
    Eigen::Vector3d m_AnklePositionRight;


  };


}
#endif /* _FOOT_TRAJECTORY_GENERATION_ABSTRACT_H_ */


/*
 * Copyright 2008, 2009, 2010,
 *
 * Torea Foissotte
 * Olivier Stasse
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

/*! \file FootTrajectoryGenerationMultiple.h
  \brief This object is in charge of maintaining the foot trajectory
  generation for several intervals.
  It relies on the FootTrajectoryGenerationStandard class.

  @ingroup foottrajectorygeneration
*/

#ifndef _FOOT_TRAJECTORY_GENERATION_MULTIPLE_H_

/* Walking pattern generation related inclusions */

#include "FootTrajectoryGeneration/FootTrajectoryGenerationStandard.hh"

namespace PatternGeneratorJRL {

/*! @ingroup foottrajectorygeneration

  This class generates a trajectory for a complete leg relying on a
  set of description of the intervals.
  More precisely this object handles a set of FootTrajectoryGenerationStandard
  objects.
  Thus it acts as a container, and allow a coherent interface to a set
  of foot trajectory.

  Each interval is described by a time duration \f[ \Delta T_j \f],
  its nature which can be double support and in  single support
  two subcategories exist: support foot and flying.


*/
class FootTrajectoryGenerationMultiple : public SimplePlugin {
public:
  /*! \name Constants to define the nature of the foot trajectory.
    @{
  */
  /*! Double support foot. */
  const static int DOUBLE_SUPPORT = 0;

  /*! \name Single support foot subcategories
    @{ */
  /*! \brief The robot is in single support and the foot
    considered is the support foot */
  const static int SINGLE_SUPPORT_SUPPORT = 1;
  /*! \brief The robot is in single support and
    the foot considered is flying. */
  const static int SINGLE_SUPPORT_FLYING = 2;

  /*! @} */
  /*! @} */
  /*! Constructor: In order to compute some appropriate strategies,
    this class needs to extract specific details from the humanoid model. */
  FootTrajectoryGenerationMultiple(SimplePluginManager *lSPM, PRFoot *aFoot);

  // Default destructor
  ~FootTrajectoryGenerationMultiple();

  /*! \brief Reimplementation of the call method for the plugin manager.
    More explicitly this object will deal with the call which initialize
    the feet behaviors (\f$omega\f$, \f$ stepheight \f$) .
  */
  virtual void CallMethod(std::string &Method, std::istringstream &strm);

  /*! \name Methods related to the handling of the intervals.
    @{
  */
  /*! \brief Set number of intervals. */
  void SetNumberOfIntervals(int lNumberOfIntervals);

  /*! \brief Get number of intervals. */
  int GetNumberOfIntervals() const;

  /*! \brief Set the time for each interval. */
  void SetTimeIntervals(const std::vector<double> &lDeltaTj);

  /*! \brief Get the time for each interval */
  void GetTimeIntervals(std::vector<double> &lDeltaTj) const;

  /*! \brief Set nature of interval */
  int SetNatureInterval(unsigned int IntervalIndex, int Nature);

  /*! \brief Get nature of interval */
  int GetNatureInterval(unsigned int IntervalIndex) const;

  /*! \brief Display intervals time. */
  int DisplayIntervals() const;

  /*! @} */

  /*! \brief Compute the value asked for according to :
    @param[in] axis: the axis along which the computation is done,
    @param[in] t: the time,
    @param[out] r: the result.
  */
  bool Compute(int axis, double t, double &r);

  /*! \brief Compute the value asked for according to :
    @param[in] t: the time,
    @param[out] aFootAbsolutePosition: a foot absolute position.
  */
  bool Compute(double t, FootAbsolutePosition &aFootAbsolutePosition);

  /*! \brief Compute the value asked for according to :
    @param[in] t: the time,
    @param[in] IndexInterval: Index of the interval to be used
    for the computation.
    @param[out] aFootAbsolutePosition: a foot absolute position.
  */
  bool Compute(double t, FootAbsolutePosition &aFootAbsolutePosition,
               unsigned int IndexInterval);

  /*! This method specifies the parameters for each of the polynome
    used by this
    object. In this case, as it is used for the 3rd order polynome.
    The polynome to which those parameters are set is specified
    with PolynomeIndex.
    @param PolynomeIndex: Set to which axis the parameters will be applied.
    @param TimeInterval: Set the time base of the polynome.
    @param Position: Set the final position of the polynome at TimeInterval.
  */
  int SetParameters(unsigned int IntervalIndex, int AxisReference,
                    double TimeInterval, double FinalPosition);

  /*! This method specifies the parameters for each of the polynome used
    by this object.
    In this case, as it is used for the 3rd order polynome. The polynome to
    which those parameters are set is specified with PolynomeIndex.
    @param PolynomeIndex: Set to which axis the parameters will be applied.
    @param AxisReference: Index to the axis to be used.
    @param TimeInterval: Set the time base of the polynome.
    @param FinalPosition: Set the final position of the polynome at
    TimeInterval.
    @param InitPosition: Initial position when computing the polynome at
    t= m_AbsoluteTimeReference.
    @param InitSpeed: Initial speed when computing the polynome at
    t=m_AbsoluteTimeReference.
  */
  int SetParametersWithInitPosInitSpeed(
      unsigned int PolynomeIndex, int AxisReference, double TimeInterval,
      double FinalPosition, double InitPosition, double InitSpeed,
      vector<double> MiddlePos = vector<double>(3, -1));

  /*! This method specifies the parameters for each of the polynome used by
    this object.
    In this case, as it is used for the 3rd order polynome. The polynome to
    which those parameters are set is specified with PolynomeIndex.
    @param PolynomeIndex: Set to which axis the parameters will be applied.
    @param AxisReference: Index to the axis to be used.
    @param TimeInterval: Set the time base of the polynome.
    @param FinalPosition: Set the final position of the polynome at
    TimeInterval.
    @param InitPosition: Initial position when computing the polynome at
    t= m_AbsoluteTimeReference.
    @param InitSpeed: Initial speed when computing the polynome at
    t=m_AbsoluteTimeReference.
    @param InitAcc: Initial speed when computing the polynome at
    t=m_AbsoluteTimeReference.
  */
  int SetParametersWithInitPosInitSpeedInitAcc(
      unsigned int PolynomeIndex, int AxisReference, double TimeInterval,
      double FinalPosition, double InitPosition, double InitSpeed,
      double InitAcc, vector<double> middlePos = vector<double>(3, -1));

  /*! This method gets the parameters for each of the polynome used by this
    object. In this case, as it is used for the 3rd order polynome.
    The polynome to which those parameters are set is specified with
    PolynomeIndex.
    @param PolynomeIndex: Set to which axis the parameters will be applied.
    @param AxisReference: Index to the axis to be used.
    @param TimeInterval: Set the time base of the polynome.
    @param FinalPosition: Set the final position of the polynome at
    TimeInterval.
    @param InitPosition: Initial position when computing the polynome at
    t= m_AbsoluteTimeReference.
    @param InitSpeed: Initial speed when computing the polynome at
    t=m_AbsoluteTimeReference.
  */
  int GetParametersWithInitPosInitSpeed(unsigned int PolynomeIndex,
                                        int AxisReference, double &TimeInterval,
                                        double &FinalPosition,
                                        double &InitPosition,
                                        double &InitSpeed);

  /*! \name Methods related to the Absolute Time Reference.
    This time specifies the beginning of the trajectory.
    @{ */

  /*! Returns the time when the trajectory starts. */
  double GetAbsoluteTimeReference() const;

  /*! Set the time when the trajectory starts.  */
  void SetAbsoluteTimeReference(double lAbsoluteTimeReference);

  /*! @} */

  FootTrajectoryGenerationMultiple &
  operator=(const FootTrajectoryGenerationMultiple &aFTGM);

protected:
  /*! \brief Handle a set of object allowing the generation of the foot
    trajectory.*/
  std::vector<FootTrajectoryGenerationStandard *>
      m_SetOfFootTrajectoryGenerationObjects;

  /*! \brief Reference of humanoid specificities. */
  PRFoot *m_Foot;

  /*! \brief Set the absolute reference time for this set of intervals. */
  double m_AbsoluteTimeReference;

  /*! \brief Set of interval times. */
  std::vector<double> m_DeltaTj;

  /*! \brief Nature of the interval. */
  std::vector<int> m_NatureOfIntervals;

  /*! \brief Reference time for the polynomials. */
  std::vector<double> m_RefTime;

  /*! \brief Sensitivity to numerical unstability when using time. */
  double m_Sensitivity;
};
} // namespace PatternGeneratorJRL
#endif /* _FOOT_TRAJECTORY_GENERATION_MULTIPLE_H_ */

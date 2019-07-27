/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Francois   Keith
 * Olivier    Stasse
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
/* \file StepStackHandler.h
   \brief This object handle the step stack of the pattern generator.
   It allows also to create automatically stack of steps according to
   some high level functionnalities.
*/

#ifndef _STEP_STACK_HANDLER_H_
#define _STEP_STACK_HANDLER_H_

#include <deque>


#include <SimplePlugin.hh>
#include <jrl/walkgen/pgtypes.hh>

namespace PatternGeneratorJRL
{
  class StepOverPlanner;

  /*! @ingroup pgjrl
    This class is in charge of handling the stack of footprints.
    There is two modes currently:
    - An off-line mode, where the complete stack is send to
    the ZMP reference trajectory generator object, and created off-line.
    - An on-line mode, where one step at a time is send to
    the ZMP reference trajectory generator object.
  */
  class  StepStackHandler : public SimplePlugin
  {
  public:
    /*! \brief Constructor */
    StepStackHandler(SimplePluginManager *lSPM);

    /*! \brief Destructor */
    ~StepStackHandler();


    /*! \brief Specify the walking mode:
      0: Normal walking.
      1: Lower the height.
      2: Stepping over.
      3: Read a path.
      4: Freeze the upper body.
    */
    void SetWalkMode(int lWalkMode);

    /*! \brief Get the walking mode. */
    int GetWalkMode();


    /*! \brief Set the link towards an instance of Step Over planner. */
    void SetStepOverPlanner(StepOverPlanner *aSOP);

    /*! \brief Take a serie of string as an input and
      read the steps according to the chosen walkmode. */

    void ReadStepSequenceAccordingToWalkMode(std::istringstream &strm);
    void ReadStepStairSequenceAccordingToWalkMode(std::istringstream &strm);

    /*! \brief Real a partial sequence of steps
      without termination and immediate execution. */
    void m_PartialStepSequence(std::istringstream &strm);

    void m_PartialStepStairSequence(std::istringstream &strm);

    /*! \brief Set the single time support. */
    void SetSingleTimeSupport(double aSingleSupportTime);

    /*! \brief Get the time for single support. */
    double GetSingleTimeSupport();

    /*! \brief Set the time for double support. */
    void SetDoubleTimeSupport(double aDoubleSupportTime);

    /*! \brief Get the time for double support. */
    double GetDoubleTimeSupport();

    /*! \brief Prepare the stack to start for a specific support foot. */
    void PrepareForSupportFoot(int SupportFoot);

    /*! \brief To force the last generated support foot. */
    void FinishOnTheLastCorrectSupportFoot();

    /*! \brief Creates a copy of relative foot positions 
      and reset the stack (or not).
      \param[in] lRelativeFootPositions The stack of relative foot to be copied.
      \param[in] PerformClean Reset the stack if PerformClean is true, 
      otherwise does nothing.
    */
    void CopyRelativeFootPosition(std::deque<RelativeFootPosition> &
                                  lRelativeFootPositions,
                                  bool PerformClean);

    /*! \name Method related to online stepping.
      @{
    */

    /*! \brief Start On Line stepping. */
    void StartOnLineStep();

    /*! \brief Stop On Line stepping. */
    void StopOnLineStep();

    /*! \brief Add a standard step on the stack. */
    void AddStandardOnLineStep(bool NewStep,
                               double NewStepX,
                               double NewStepY,
                               double Theta);


    /*! \brief Returns current state for on line stepping. */
    bool IsOnLineSteppingOn();
    /*! @} */

    /*! \brief Methods to handle the stack.
      @{
    */

    /*! \brief Remove the first step in the stack.
      @return Returns true if this is the end of the sequence. */
    bool RemoveFirstStepInTheStack();

    /*! \brief Add a step in the stack. */
    void AddStepInTheStack(double sx, double sy,
                           double theta, double sstime,
                           double dstime);

    void AddStepStairInTheStack(double sx, double sy, double sz,
                                double theta, double sstime,
                                double dstime);

    /*! \brief Push a step in front of the stack. */
    void PushFrontAStepInTheStack(RelativeFootPosition &aRFP);

    /*! \brief Returns the last step of the stack. */
    RelativeFootPosition ReturnBackFootPosition();

    /*! \brief Returns the first step of the stack. */
    bool ReturnFrontFootPosition(RelativeFootPosition &aRFP);

    /*! \brief Returns the size of the stack. */
    std::size_t ReturnStackSize();

    /*! @} */

    /*! \name High level methods to create stack of steps for large motion.
      @{
    */
    /*! \brief Create a sequence of step to realize an arc of rayon R,
      for arc_deg degrees, starting with the support foot defined
      by SupportFoot. The direction of the robot is towards
      the  center of the arc.
    */
    void CreateArcCenteredInStepStack(  double R,
                                        double arc_deg,
                                        int SupportFoot);
    /*! \brief Create a sequence of steps to realize an arc of rayon R,
      for arc_deg degrees, starting with the support foot defined
      by SupportFoot. The direction of the robot is tangent
      to the arc centered in x and y.
    */
    void CreateArcInStepStack(  double x,double y, double R,
                                double arc_deg, int SupportFoot);
    /*! @} */

    /*! \brief Handling methods for the plugin mecanism.
      This method is a reimplementation of the interface inherited
      by SimplePlugin.
    */
    virtual void CallMethod(std::string &Method, std::istringstream &strm);

  protected:

    /*! Vector of relative foot position. */
    std::deque<RelativeFootPosition> m_RelativeFootPositions;

    /*! Keep the last correct support foot. */
    int m_KeepLastCorrectSupportFoot;

    /*! Keeps the current walking mode. */
    int m_WalkMode;

    /*! Link to the step over planner. */
    StepOverPlanner *m_StOvPl;

    /*! Default value for Single time support and double time support. */
    double m_SingleSupportTime, m_DoubleSupportTime;

    /*! Variable for delta feasibility limit */
    double m_DeltaFeasibilityLimit;

    /*! On line step stack handling. */
    bool m_OnLineSteps;

    /*! Transition for finishing on line stepping. */
    bool m_TransitionFinishOnLine;
  };

}
#include <MotionGeneration/StepOverPlanner.hh>
#endif /* _FOOT_PRINT_H_*/

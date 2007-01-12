
/* This object handle the step stack of the pattern generator.
    It allows also to create automatically stack of steps according to 
    some high level functionnalities.


   Copyright (c) 2005-2006, 
   @author Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _STEP_STACK_HANDLER_H_
#define _STEP_STACK_HANDLER_H_

#include <MatrixAbstractLayer.h>
#include <StepOverPlanner.h>

namespace PatternGeneratorJRL
{
  
  /*! @ingroup pgjrl
    This class is in charge of handling the stack of footprints.
    There is two modes currently:
    - An off-line mode, where the complete stack is send to 
    the ZMP reference trajectory generator object, and created off-line.
    - An on-line mode, where one step at a time is send to 
    the ZMP reference trajectory generator object. 
   */
  class StepStackHandler
  {
  public:
    /*! Constructor */
    StepStackHandler();

    /*! Destructor */
    ~StepStackHandler();

    /*! Create a sequence of step to realize an arc of rayon R,
     for arc_deg degrees, starting with the support foot defined
     by SupportFoot. The direction of the robot is towards
     the  center of the arc.
    */
    void CreateArcCenteredInStepStack(  double R,
					double arc_deg, 
					int SupportFoot);
    /*! Create a sequence of steps to realize an arc of rayon R,
      for arc_deg degrees, starting with the support foot defined
      by SupportFoot. The direction of the robot is tangent
      to the arc centered in x and y.
     */
    void CreateArcInStepStack(  double x,double y, double R,
				double arc_deg, int SupportFoot);

    /*! Specify the walking mode:
      0: Normal walking.
      1: Lower the height.
      2: Stepping over.
      3: Read a path.
      4: Freeze the upper body.
    */
    void SetWalkMode(int lWalkMode);

    /*! Get the walking mode. */
    int GetWalkMode();


    /*! Set the link towards an instance of Step Over planner. */
    void SetStepOverPlanner(StepOverPlanner *aSOP);

    /*! Take a serie of string as an input and 
      read the steps according to the chosen walkmode. */
    void ReadStepSequenceAccordingToWalkMode(istringstream &strm);

    /*! Real a partial sequence of steps
      without termination and immediate execution. */ 
    void m_PartialStepSequence(istringstream &strm);

    /*! Set the single time support. */
    void SetSingleTimeSupport(double aSingleSupportTime);

    /*! Get the time for single support. */
    double GetSingleTimeSupport();
    
    /*! Set the time for double support. */
    void SetDoubleTimeSupport(double aDoubleSupportTime);
    
    /*! Get the time for double support. */
    double GetDoubleTimeSupport();

    /*! Prepare the stack to start for a specific support foot. */
    void PrepareForSupportFoot(int SupportFoot);
    
    /*! Prepare the stack to start for a specific support foot. */
    /*! (The parser interface). */
    void m_PrepareForSupportFoot(istringstream & strm);

    /*! To force the last generated support foot. */
    void FinishOnTheLastCorrectSupportFoot();

    /*! To create a copy and perform a clean (or not)
      depending on the value of the boolean b_clean. */
    void CopyRelativeFootPosition(deque<RelativeFootPosition> & lRelativeFootPositions,
				  bool PerformClean);
    
    /*! Start On Line stepping. */
    void StartOnLineStep();

    /*! Add a standard step on the stack. */
    void AddStandardOnLineStep(bool NewStep, 
			       double NewStepX,
			       double NewStepY,
			       double Theta);

    /*! Returns current state for on line stepping. */
    bool IsOnLineSteppingOn();
 
    /*! Remove the first step in the stack. */
    /*! Returns true if this is the end of the sequence. */
    bool RemoveFirstStepInTheStack();

    /*! Add a step in the stack. */
    void AddStepInTheStack(double sx, double sy,
			   double theta, double sstime,
			   double dstime);
    
    /*! Stop On Line stepping. */
    void StopOnLineStep();

    /*! Returns the first step of the stack. */
    RelativeFootPosition ReturnBackFootPosition();

  protected:

    /*! Vector of relative foot position. */
    deque<RelativeFootPosition> m_RelativeFootPositions;

    /*! Keep the last correct support foot. */
    int m_KeepLastCorrectSupportFoot;

    /*! Keeps the current walking mode. */
    int m_WalkMode;

    /*! Link to the step over planner. */
    StepOverPlanner *m_StOvPl;	

    /*! Default value for Single time support and double time support. */
    double m_SingleSupportTime, m_DoubleSupportTime;

    /*! Position and parameters related to the obstacle. */
    ObstaclePar m_ObstaclePars;

    /*! Variable for delta feasibility limit */
    double m_DeltaFeasibilityLimit;	
    
    /*! On line step stack handling. */
    bool m_OnLineSteps;
    
    /*! Transition for finishing on line stepping. */
    bool m_TransitionFinishOnLine;
  };

};
#endif /* _FOOT_PRINT_H_*/

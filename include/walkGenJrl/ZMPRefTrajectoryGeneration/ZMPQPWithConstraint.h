/* This object provides the QP formulation with constraint on the ZMP 
   of  the preview control method developed by Kaita..
   This formulation has been proposed by PB Wieber in Humanoids 2006.

   Copyright (c) 2005-2007, 
   Francois Keith, Olivier Stasse,
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
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

#ifndef _ZMPQP_WITH_CONSTRAINT_H_
#define _ZMPQP_WITH_CONSTRAINT_H_

#include <dynamicsJRLJapan/HumanoidSpecificities.h>

#include <walkGenJrl/walkGenJrl_API.h>
#include <walkGenJrl/Mathematics/ConvexHull.h>
#include <walkGenJrl/ZMPRefTrajectoryGeneration/ZMPRefTrajectoryGeneration.h>

using namespace dynamicsJRLJapan;
namespace PatternGeneratorJRL
{
  class ZMPDiscretization;
  class WALK_GEN_JRL_EXPORT ZMPQPWithConstraint : public ZMPRefTrajectoryGeneration
  {
    
  public:

    /* Default constructor. */
    ZMPQPWithConstraint(SimplePluginManager *lSPM, string DataFile, HumanoidSpecificities *aHS=0);

    /* Default destructor. */
    ~ZMPQPWithConstraint();
    
    /*! This method builds a set of linear constraint inequalities based
      on the foot trajectories given as an input.
      The result is a set Linear Constraint Inequalities. */
    int BuildLinearConstraintInequalities(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					  deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					  deque<LinearConstraintInequality_t *> & QueueOfLConstraintInequalities,
					  double ConstraintOnX,
					  double ConstraintOnY);

      /** Generate ZMP discreatization from a vector of foot position.
	  ASSUME A COMPLETE MOTION FROM END TO START, and GENERATE EVERY VALUE.
	  
	  @param[out] ZMPPositions: Returns the ZMP reference values for the overall motion.
	  Those are absolute position in the world reference frame. The origin is the initial
	  position of the robot. The relative foot position specified are added.

	  @param[out] CoMPositions: Returns the CoM reference values for the overall motion.
	  Those are absolute position in the world reference frame. The origin is the initial
	  position of the robot. The relative foot position specified are added.

	  @param[in] RelativeFootPositions: The set of 
	  relative steps to be performed by the robot.

	  @param[out] LeftFootAbsolutePositions: Returns the absolute position of the left foot.
	  According to the macro FULL_POLYNOME the trajectory will follow a third order
	  polynom or a fifth order. By experience it is wise to put a third order. 
	  A null acceleration might cause problem for the compensation of the Z-axis momentum.

	  @param[out] RightFootAbsolutePositions: Returns the absolute position of the right foot.
	  
	  @param[in] Xmax: The maximal distance of a hand along the X axis in the waist coordinates.

	  @param[in] lStartingCOMPosition: The initial position of the CoM.
	  
	  @param[in] lStartingZMPPosition: The initial position of the ZMP.
	  
	  @param[in] InitLeftFootAbsolutePosition: The initial position of the left foot.
	  
	  @param[in] InitRightFootAbsolutePosition: The initial position of the right foot.

	  
	   */
    void GetZMPDiscretization(deque<ZMPPosition> & ZMPPositions,
			      deque<COMPosition> & CoMPositions,
			      deque<RelativeFootPosition> &RelativeFootPositions,
			      deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			      deque<FootAbsolutePosition> &RightFootAbsolutePositions,
			      double Xmax,
			      COMPosition & lStartingCOMPosition,
			      MAL_S3_VECTOR(,double) & lStartingZMPPosition,
			      FootAbsolutePosition & InitLeftFootAbsolutePosition,
			      FootAbsolutePosition & InitRightFootAbsolutePosition);

    /*! This method is a new way of computing the ZMP trajectory from
      foot trajectory. */
    int BuildZMPTrajectoryFromFootTrajectory(deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					     deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					     deque<ZMPPosition> &ZMPRefPositions,		       
					     deque<COMPosition> &COMPositions,
					     double ConstraintOnX,
					     double ConstraintOnY,
					     double T,
					     unsigned int N);
    
    /*! Build the necessary matrices for the QP problem under linear inequality constraints. */
    int BuildMatricesPxPu(double * &Px, double * &Pu,
			  unsigned N, double T,
			  double StartingTime,
			  deque<LinearConstraintInequality_t *> 
			  & QueueOfLConstraintInequalities,
			  double Com_Height,
			  unsigned int &NbOfConstraints,
			  MAL_VECTOR(&xk,double));
    
    /*! This method helps to build a linear system for constraining the ZMP. */
    int ComputeLinearSystem(vector<CH_Point> aVecOfPoints,
			    MAL_MATRIX(&A,double),
			    MAL_MATRIX(&B,double));

    /*! This method get the COM buffer computed by the QP in off-line mode. */
    void GetComBuffer(deque<COMPosition> &aCOMBuffer);

    /*! Call method to handle the plugins. */
    void CallMethod(std::string &Method, std::istringstream &strm);

    /*! Call method to handle on-line generation of ZMP reference trajectory. @{*/
        
    /*! Methods for on-line generation. (First version)
      The queues will be updated as follows:
      - The first values necessary to start walking will be inserted.
      - The initial positions of the feet will be taken into account
      according to InitLeftFootAbsolutePosition and InitRightFootAbsolutePosition.
      - The RelativeFootPositions stack will be taken into account,
      - The starting COM Position.
      Returns the number of steps which has been completely put inside 
      the queue of ZMP, and foot positions.
    */
    int InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
		   deque<COMPosition> & CoMPositions,		   
		   deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
		   deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
		   FootAbsolutePosition & InitLeftFootAbsolutePosition,
		   FootAbsolutePosition & InitRightFootAbsolutePosition,
		   deque<RelativeFootPosition> &RelativeFootPositions,
		   COMPosition & lStartingCOMPosition,
		   MAL_S3_VECTOR(,double) & lStartingZMPPosition);
    
    /* ! Methods to update the stack on-line by inserting a new foot position. */
    void OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
		       deque<ZMPPosition> & FinalZMPPositions,		
		       deque<COMPosition> & CoMPositions,			     
		       deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
		       deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
		       bool EndSequence);

    /* ! \brief Method to update the stacks on-line */
    void OnLine(double time,
		deque<ZMPPosition> & FinalZMPPositions,		
		deque<COMPosition> & CoMPositions,			     
		deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
		deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions);

    /* ! \brief Method to change on line the landing position of a foot.
       @return If the method failed it returns -1, 0 otherwise.
    */
    int OnLineFootChange(double time,
			 FootAbsolutePosition &aFootAbsolutePosition,
			 deque<ZMPPosition> & FinalZMPPositions,			     
			 deque<COMPosition> & CoMPositions,
			 deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
			 deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
			 StepStackHandler * aStepStackHandler=0);

    /*! \brief Method to stop walking.
      @param[out] ZMPPositions: The queue of ZMP reference positions.
      @param[out] FinalCOMPositions: The queue of COM reference positions.
      @param[out] LeftFootAbsolutePositions: The queue of left foot absolute positions.
      @param[out] RightFootAbsolutePositions: The queue of right foot absolute positions.
    */
    void EndPhaseOfTheWalking(deque<ZMPPosition> &ZMPPositions,
			      deque<COMPosition> &FinalCOMPositions,
			      deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
			      deque<FootAbsolutePosition> &RightFootAbsolutePositions);


    /*! \brief Return the time at which it is optimal to regenerate a step in online mode. 
     */
    int ReturnOptimalTimeToRegenerateAStep();
      
    /* @} */

    /*! Set the preview control object. */
    void SetPreviewControl(PreviewControl *aPC);

  protected:
    /* ! Reference on the Humanoid Specificities. */
    HumanoidSpecificities * m_HS;

    /* !  Matrices for the dynamical system. 
       @{
     */
    /* ! Matrix regarding the state of the CoM (pos, velocity, acceleration) */
    MAL_MATRIX(m_A,double);
    /* ! Vector for the command */
    MAL_MATRIX(m_B,double);
    /* ! Vector for the ZMP. */
    MAL_MATRIX(m_C,double);
    /* ! @} */

    /*! Uses a ZMPDiscretization scheme to get the usual Kajita heuristic. */
    ZMPDiscretization * m_ZMPD;

    /*! Constraint on X and Y */
    double m_ConstraintOnX, m_ConstraintOnY;

    /*! Sampling of the QP. */
    double m_QP_T;
    
    /*! Preview window */
    unsigned int m_QP_N;

  };
};

#include <walkGenJrl/ZMPRefTrajectoryGeneration/ZMPDiscretization.h>
#endif /* _ZMPQP_WITH_CONSTRAINT_H_ */

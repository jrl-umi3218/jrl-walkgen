/* \file AnalyticalMorisawaFull.h
   \brief Full formulation of the analytical method to generate ZMP and COM.

   Copyright (c) 2007, 
   Olivier Stasse,
   
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
#ifndef _ANALYTICAL_MORISAWA_FULL_H_
#define _ANALYTICAL_MORISAWA_FULL_H_

#include <dynamicsJRLJapan/HumanoidSpecificities.h>

#include <walkGenJrl/Mathematics/PolynomeFoot.h>
#include <walkGenJrl/Mathematics/ConvexHull.h>
#include <walkGenJrl/Mathematics/AnalyticalZMPCOGTrajectory.h>
#include <walkGenJrl/PreviewControl/PreviewControl.h>
#include <walkGenJrl/ZMPRefTrajectoryGeneration/AnalyticalMorisawaAbstract.h>



namespace PatternGeneratorJRL
{
  /*! \brief Class to compute analytically the trajectories of both the ZMP and the CoM.
    @ingroup analyticalformulation
    This class generate the reference value for the
    ZMP based on a polynomail representation
    of the ZMP following the algorithm proposed by Morisawa and al. \ref Morisawa2007
    
    It uses a full formulation of the linear problem described 
    in AnalyticalMorizawaAbstract. 
    
    Basically the size of
    the matrix for a \f$ nb_{steps}=3 \f$ steps horizon the number of intervals
    is \f$ nb_{intervals}= 2 nb_{steps} \f$.
    
    The size \f$ (m,n) \f$ of the \f$ {\bf Z} \f$ matrix is given by:
    \f{eqnarray*}
    m = 6 \; nb_{intervals} + 2 \\
    n = 14 + 6 (nb_{intervals}-2) \\
    \f}

   */
  class AnalyticalMorisawaFull: public AnalyticalMorisawaAbstract
    {
      
    public:
      /*! Constructor */
      AnalyticalMorisawaFull(SimplePluginManager * lSPM);
      
      /*! Destructor */
      virtual ~AnalyticalMorisawaFull();

      /*! \name Methods inherited from ZMPRefTrajectoryGeneration and reimplemented
	@{ */

      /*! Returns the CoM and ZMP trajectory for some relative foot positions. 
	Generate ZMP discreatization from a vector of foot position.
	ASSUME A COMPLETE MOTION FROM END TO START, and GENERATE EVERY VALUE.
	  	
	@param[out] ZMPPositions: Returns the ZMP reference values for the overall motion.
	Those are absolute position in the world reference frame. The origin is the initial
	position of the robot. The relative foot position specified are added.

	@param[out] COMPositions: Returns the CoM reference values for the overall motion.
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

	@param[in] InitLeftFootAbsolutePosition: The initial position of the left foot.
	
	@param[in] InitRightFootAbsolutePosition: The initial position of the right foot.

      */
      void GetZMPDiscretization(deque<ZMPPosition> & ZMPPositions,
				deque<COMPosition> & COMPositions,
				deque<RelativeFootPosition> &RelativeFootPositions,
				deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
				deque<FootAbsolutePosition> &RightFootAbsolutePositions,
				double Xmax,
				COMPosition & lStartingCOMPosition,
				MAL_S3_VECTOR(&,double) lStartingZMPPosition,
				FootAbsolutePosition & InitLeftFootAbsolutePosition,
				FootAbsolutePosition & InitRightFootAbsolutePosition);
      
      /*! Methods for on-line generation. (First version)
	The queues will be updated as follows:
	\li \c The first values necessary to start walking will be inserted.
	\li \c The initial positions of the feet will be taken into account
	according to InitLeftFootAbsolutePosition and InitRightFootAbsolutePosition.
	\li \c The RelativeFootPositions stack will be taken into account,
	\li \c The starting COM Position.
	Returns the number of steps which has been completely put inside 
	the queue of ZMP, and foot positions.
      */
      int InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
		     deque<COMPosition> & FinalCOMPositions,
		     deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
		     deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
		     FootAbsolutePosition & InitLeftFootAbsolutePosition,
		     FootAbsolutePosition & InitRightFootAbsolutePosition,
		     deque<RelativeFootPosition> &RelativeFootPositions,
		     COMPosition & lStartingCOMPosition,
		     MAL_S3_VECTOR(&,double) lStartingZMPPosition);
      
      /* ! Methods to update the stack on-line by inserting a new foot position. */
      void OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
			 deque<ZMPPosition> & FinalZMPPositions,					     
			 deque<COMPosition> & FinalCOMPositions,					     
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
			   StepStackHandler *aStepStackHandler=0);

      /*! \brief Return the time at which it is optimal to regenerate a step in online mode. 
       */
      int ReturnOptimalTimeToRegenerateAStep();
      
      /*! @} */
      
      /*! \name Methods specifics to our current implementation.
       @{ */
      /*! \name Methods for the full linear system 
	@{ */

      /*! \brief Building the Z matrix to be inverted. 
	@param[in] lCoMZ: Profile of the CoM's height trajectory for each interval.
	@param[in] lZMPZ: Profile of the ZMP's height trajectory for each interval.
       */
      void BuildingTheZMatrix(vector<double> &lCoMZ, vector<double> &lZMPZ );

      /*! \brief Building the w vector. 
	It is currently assume that all ZMP's speed will be
	set to zero, as well as the final COM's speed.
	The sequence of ZMPSequence is the final value of the
	ZMP. As a special case, the first interval being set
	as a single support phase
       */
      void ComputeW(double InitialCoMPos,
		    double InitialComSpeed,
		    vector<double> &ZMPPosSequence,
		    double FinalCoMPos,
		    AnalyticalZMPCOGTrajectory & aAZCT);

      /*! \brief Transfert the computed weights to an Analytical ZMP COG
	Trajectory able to generate the appropriate intermediates values.
      @param[out] aAZCT: The object to be filled with the appropriate intermediate values.
      @param[in] lCoMZ: The height trajectory of the CoM.
      @param[in] lZMPZ: The height trajectory of the ZMP.
      @param[in] lZMPInit: The initial value of the ZMP.
      @param[in] lZMPEnd : The final value of the ZMP.
      @param[in] InitializeaAZCT: If set to true this variable trigger the initialization
      of the object aAZCT, does nothing otherwise.
      */
      void TransfertTheCoefficientsToTrajectories(AnalyticalZMPCOGTrajectory &aAZCT,
						  vector<double> &lCoMZ,
						  vector<double> &lZMPZ,
						  double &lZMPInit,
						  double &lZMPEnd,
						  bool InitializeaAZCT);

      /*! \brief Transfert the computed weights to an Analytical ZMP COG
	Trajectory able to generate the appropriate intermediates values.
	@param aAZCT: The object to be filled with the appropriate intermediate values.
	@param lCoMZ: The height trajectory of the CoM.
	@param lZMPZ: The height trajectory of the ZMP.
	@param lZMPInit: The initial value of the ZMP.
	@param lZMPEnd : The final value of the ZMP.
      */
      void TransfertTheCompactCoefficientsToTrajectories(AnalyticalZMPCOGTrajectory &aAZCT,
							 vector<double> &lCoMZ, 
							 vector<double> &lZMPZ,
							 double &lZMPInit,
							 double &lZMPEnd);
      /*! @} */

      /*! \brief Initialize automatically Polynomial degrees, and temporal intervals. 
	@return True if succeedeed, false otherwise.
       */
      bool InitializeBasicVariables();

      /*! \brief Compute the polynomial weights. */
      void ComputePolynomialWeights();

      /*! \brief Same than previously but wit a second method. */
      void ComputePolynomialWeights2();
	
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

      /*! @} */


      /*! @} */
      
    };
};
#endif

/* This object provides the .

   Copyright (c) 2005-2009, 
   Olivier Stasse,Francois Keith
   
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

#ifndef _LINEAR_INVERTED_PENDULUM_2D_H_
#define _LINEAR_INVERTED_PENDULUM_2D_H_

/*! STL includes */
#include <deque>

/*! Framework includes */
#include <walkGenJrl/walkGenJrl_API.h>
#include <walkGenJrl/PGTypes.h>

namespace PatternGeneratorJRL
{
  class WALK_GEN_JRL_EXPORT LinearizedInvertedPendulum2D
    {
    public:
      /*! Constructor */
      LinearizedInvertedPendulum2D();

      /*! Destructor */
      ~LinearizedInvertedPendulum2D();

      /*! \brief Initialize the system.
	\return 0 if the initialization is fine,
	-1 if the control period is not initialized,
	-2 if the Com height is not initialized.
      */
      int InitializeSystem();

      /*! \brief Interpolation during a simulation period with control parameters.
	\param[out]: NewFinalZMPPositions: queue of ZMP positions interpolated.
	\param[out]: COMPositions: queue of COM positions interpolated.
	\param[in]: ZMPRefPositions: Reference positions of ZMP (Kajita's heuristic every 5 ms).
	\param[in]: CurrentPosition: index of the current position of the ZMP reference
	(this allow to propagate some parameters define by a heuristic to the CoM positions).
	\param[in]: CX: command parameter in the forward direction.
	\param[in]: CY: command parameter in the perpendicular direction.
       */
      int Interpolation(std::deque<ZMPPosition> &NewFinalZMPPositions,
			std::deque<COMPosition> &COMPositions,
			std::deque<ZMPPosition> &ZMPRefPositions,
			int CurrentPosition,
			double CX, double CY);

      
      /*! \brief Simulate one iteration of the LIPM
       \param[in] CX: control value in the forward direction.
       \param[in] CY: control value in the left-right direction.
       \return 0 if the object has been properly initialized -1, otherwise.
      */
      int OneIteration(double CX, double CY);

    private:

      /*! \name Internal parameters. 
	@{
       */
      /*! \brief Control period */
      double m_T;

      /*! \brief Height of the com. */
      double m_ComHeight;

      /*! \brief Interval for interpolation */
      double m_InterpolationInterval;
      
      /*! \brief Interval for robot control */
      double m_SamplingPeriod;
      
      /*! @}*/
      /* !  Matrices for the dynamical system. 
	 @{
      */
      /* ! Matrix regarding the state of the CoM (pos, velocity, acceleration) */
      MAL_MATRIX(m_A,double);
      /* ! Vector for the command */
      MAL_MATRIX(m_B,double);
      /* ! Vector for the ZMP. */
      MAL_MATRIX(m_C,double);

      /*! \brief State of the LIPM at the \f$k\f$ eme iteration
	\f$ x_k = [ c_x \dot{c}_x \ddot{c}_x c_y \dot{c}_y \ddot{c}_y\f$ */
      MAL_VECTOR(m_xk,double);

      /* ! \brief Vector of ZMP  */
      MAL_VECTOR(m_zk,double);
      
      /* ! @} */

    public:
      
      /*! \name Getter and setter of variables 
	@{
       */
      /*! Getter for the CoM height */
      const double & GetComHeight() const;
      
      /*! Setter for the CoM height */
      void SetComHeight(const double &);

      /*! Getter for the simulation period specifically*/
      const double & GetSimulationControlPeriod() const;
      
      /*! Setter for the simulation period specifically*/
      void SetSimulationControlPeriod(const double &);

      /*! Getter for the control period of the robot (for interpolation) . */
      const double & GetRobotControlPeriod();

      /*! Setter for the control period of the robot (for interpolation) . */
      void SetRobotControlPeriod(const double &);

      /*! Get state. */
      void GetState(MAL_VECTOR(,double) &lxk);
      /*! @} */

    };
};
#endif /* _LINEAR_INVERTED_PENDULUM_2D_H_ */

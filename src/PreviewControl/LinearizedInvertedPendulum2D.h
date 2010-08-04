/* This object provides the .

   Copyright (c) 2005-2009, 
   Olivier Stasse,Francois Keith
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see License.txt for further information on license.
*/

#ifndef _LINEAR_INVERTED_PENDULUM_2D_H_
#define _LINEAR_INVERTED_PENDULUM_2D_H_


/*! STL includes */
#include <deque>

/*! Framework includes */

#include <walkGenJrl/PGTypes.h>

namespace PatternGeneratorJRL
{
  class  LinearizedInvertedPendulum2D
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
	\param[out]: COMStates: queue of COM positions interpolated.
	\param[in]: ZMPRefPositions: Reference positions of ZMP (Kajita's heuristic every 5 ms).
	\param[in]: CurrentPosition: index of the current position of the ZMP reference
	(this allow to propagate some parameters define by a heuristic to the CoM positions).
	\param[in]: CX: command parameter in the forward direction.
	\param[in]: CY: command parameter in the perpendicular direction.
       */
      int Interpolation(std::deque<COMState> &COMStates,
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

      /*! Get state. */
      void setState(MAL_VECTOR(,double) lxk);
      /*! @} */

    };
};
#endif /* _LINEAR_INVERTED_PENDULUM_2D_H_ */

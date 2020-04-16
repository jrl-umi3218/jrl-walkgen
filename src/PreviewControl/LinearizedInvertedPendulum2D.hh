/*
 * Copyright 2009, 2010,
 *
 * Andrei Herdt
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

/* \doc This object simulate a 2D Linearized Inverted Pendulum
   with a control at the jerk level. */

#ifndef _LINEAR_INVERTED_PENDULUM_2D_H_
#define _LINEAR_INVERTED_PENDULUM_2D_H_

/*! STL includes */
#include <deque>

/*! Framework includes */

#include <jrl/walkgen/pgtypes.hh>
#include <privatepgtypes.hh>

namespace PatternGeneratorJRL {
class LinearizedInvertedPendulum2D {
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
    \param[in]: ZMPRefPositions: Reference positions of ZMP
    (Kajita's heuristic every 5 ms).
    \param[in]: CurrentPosition: index of the current position of
    the ZMP reference
    (this allow to propagate some parameters define by a heuristic
    to the CoM positions).
    \param[in]: CX: command parameter in the forward direction.
    \param[in]: CY: command parameter in the perpendicular direction.
  */
  int Interpolation(std::deque<COMState> &COMStates,
                    std::deque<ZMPPosition> &ZMPRefPositions,
                    int CurrentPosition, double CX, double CY);

  /*! \brief Simulate one iteration of the LIPM
    \param[in] CX: control value in the forward direction.
    \param[in] CY: control value in the left-right direction.
    \return 0 if the object has been properly initialized -1, otherwise.
  */
  com_t OneIteration(double CX, double CY);

private:
  /*! \name Internal parameters.
    @{
  */
  /*! \brief Control period */
  double m_T;

  /*! \brief Height of the com. */
  double m_ComHeight;

  /*! \brief Interval for interpolation */
  int m_InterpolationInterval;

  /*! \brief Interval for robot control */
  double m_SamplingPeriod;

  /*! @}*/
  /* !  Matrices for the dynamical system.
     @{
  */
  /* ! Matrix regarding the state of the CoM (pos, velocity, acceleration) */
  Eigen::MatrixXd m_A;
  /* ! Vector for the command */
  Eigen::MatrixXd m_B;
  /* ! Vector for the ZMP. */
  Eigen::MatrixXd m_C;

  /*! \brief State of the LIPM at the \f$k\f$ eme iteration
    \f$ x_k = [ c_x \dot{c}_x \ddot{c}_x c_y \dot{c}_y \ddot{c}_y\f$ */
  Eigen::VectorXd m_xk;

  com_t m_CoM;

  /* ! \brief Vector of ZMP  */
  Eigen::VectorXd m_zk;

  /* ! @} */

public:
  /*! \name Getter and setter of variables
    @{
  */
  /*! Getter for the CoM height */
  const double &GetComHeight() const;

  /*! Setter for the CoM height */
  void SetComHeight(const double &);

  /*! Getter for the simulation period specifically*/
  const double &GetSimulationControlPeriod() const;

  /*! Setter for the simulation period specifically*/
  void SetSimulationControlPeriod(const double &);

  /*! Getter for the control period of the robot (for interpolation) . */
  const double &GetRobotControlPeriod();

  /*! Setter for the control period of the robot (for interpolation) . */
  void SetRobotControlPeriod(const double &);

  /// \brief Accessor
  inline const com_t operator()() const { return m_CoM; };

  /// \brief Accessor
  inline void operator()(com_t CoM) { m_CoM = CoM; };

  /*! Get state. */
  void GetState(Eigen::VectorXd &lxk);
  COMState GetState();

  inline com_t getState() { return m_CoM; }

  /*! Set state. */
  void setState(com_t aCoM);
  void setState(COMState &aCoM);
  /*! @} */
};
} // namespace PatternGeneratorJRL
#endif /* _LINEAR_INVERTED_PENDULUM_2D_H_ */

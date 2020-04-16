/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 *
 * Florent    Lamiraux
 * Alireza    Nakhaei
 * Mathieu    Poirier
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
/** \file StepOverPolynome.h
    \brief Polynomes object for generating foot and hip trajectories
    while stepping over. */

#ifndef _STEPOVER_POLYNOME_H_
#define _STEPOVER_POLYNOME_H_

#include <vector>

#include <Eigen/Dense>
#include <Mathematics/Polynome.hh>

namespace PatternGeneratorJRL {
/*! @ingroup steppingover
  @brief Polynome used for Z trajectory during stepover. */
class StepOverPolynomeFoot : public Polynome {
public:
  /*! Constructor:
    boundCond: the different boundary conditions begin,
    intermediate and end of polynomial
    timeDistr: vector with time instants for
    intermediate boundary conditions and end time */
  StepOverPolynomeFoot();

  /*! Set the parameters */
  void SetParameters(Eigen::VectorXd boundCond, std::vector<double> timeDistr);

  /*! Destructor. */
  ~StepOverPolynomeFoot();
};

/*! @ingroup steppingover
  @brief Polynome used for Z trajectory during stepover. */
class StepOverPolynomeFootZtoX : public Polynome {
public:
  /*! Constructor:
    Zpos: vector with Zpos
    Xpos: vector Xpos */
  StepOverPolynomeFootZtoX();

  /*! Set the parameters */
  void SetParameters(Eigen::VectorXd Zpos, std::vector<double> Xpos);

  /*! Destructor. */
  ~StepOverPolynomeFootZtoX();
};

/*! @ingroup stepping over
  @brief Polynome used for X trajectory in function of time
  to combine with StepOverPolynomeFootZtoX.*/
class StepOverPolynomeFootXtoTime : public Polynome {
public:
  /*! Constructor:
    Zpos: vector with Zpos */
  StepOverPolynomeFootXtoTime();

  /*! Set the parameters */
  void SetParameters(Eigen::VectorXd Xbound, std::vector<double> timedistr);

  /*! Destructor. */
  ~StepOverPolynomeFootXtoTime();
};

/*! @ingroup steppingover
  @brief Polynome for the hip trajectory.
*/
class StepOverPolynomeHip4 : public Polynome {
public:
  /*! Constructor:
    boundCond: the different boundary conditions begin,
    intermediate and end of polynomial
    timeDistr: vector with time instants for intermediate
    boundary conditions and end time
  */
  StepOverPolynomeHip4();

  // Set the parameters
  void SetParameters(Eigen::VectorXd boundCond, std::vector<double> timeDistr);

  /*! Destructor. */
  ~StepOverPolynomeHip4();
};

/*! @ingroup steppingover
  @brief spline function calculation
  class to calculate cubic splines */
class StepOverSpline {
public:
  /*! Constructor: */
  StepOverSpline();

  /*! Set the parameters */
  void SetParameters(Eigen::VectorXd Points);

  double GetValueSpline(Eigen::VectorXd TimePoints, double CurrentLocalTime);

  void print();

  /*! Destructor. */
  ~StepOverSpline();

protected:
  unsigned long int m_number;
  Eigen::MatrixXd m_Coefficients;
};

/*! @ingroup steppingover
  class to calculate Clamped Cubic splines */
class StepOverClampedCubicSpline {
public:
  /*! Constructor: */
  StepOverClampedCubicSpline();

  /*! Set the parameters */
  void SetParameters(Eigen::VectorXd Points, Eigen::VectorXd TimePoints,
                     Eigen::VectorXd DerivativeEndPoints);

  double GetValueSpline(Eigen::VectorXd TimePoints, double CurrentLocalTime);

  void print();
  /*! Destructor. */
  ~StepOverClampedCubicSpline();

protected:
  unsigned long int m_number;
  Eigen::MatrixXd m_Coefficients;
};

} // namespace PatternGeneratorJRL

#endif /* _STEPOVER_POLYNOME_H_ */

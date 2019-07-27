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
/** \file Polynome.h
    \brief Polynomes object for trajectories.
    Initial polynome. */

#ifndef _POLYNOME_H_
#define _POLYNOME_H_

#include <vector>
#include <iostream>

using namespace::std;

namespace PatternGeneratorJRL
{

  /** Class for computing trajectories */
  class  Polynome
  {

  public:

    /*! Constructor */
    Polynome(int Degree);

    /*! Destructor */
    ~Polynome();

    /*! Compute the value. */
    double Compute(double t);

    /*! Compute the value of the derivative. */
    double ComputeDerivative(double t);

    /*! Compute the value of the second derivative. */
    double ComputeSecDerivative(double t);

    /*! Compute the value of the third derivative (jerk). */
    double ComputeJerk(double t);

    /*! Get the coefficients. */
    void GetCoefficients(std::vector<double> &lCoefficients) const;

    /*! Set the coefficients. */
    void SetCoefficients(const std::vector<double> &lCoefficients);

    inline int Degree()
    {
      return m_Degree;
    };


    /*! Print the coefficient. */
    void print() const;

  protected:

    /// Degree of the polynome
    int m_Degree;

    /// Vector of coefficients.
    std::vector<double> m_Coefficients;
  };
}
#endif /* _POLYNOME_H_*/

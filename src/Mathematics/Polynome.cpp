/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 *
 * Florent    Lamiraux
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
/* Polynomes object for trajectories. */

#include <iostream>
#include <Mathematics/Polynome.hh>

using namespace::PatternGeneratorJRL;

Polynome::Polynome(int Degree)
{
  m_Coefficients.clear();
  m_Coefficients.resize(Degree+1);
  m_Degree = Degree;
}

Polynome::~Polynome()
{
}


double Polynome::Compute(double t)
{
  double r=0.0,pt=1.0;
  for(unsigned int i=0; i<m_Coefficients.size(); i++)
    {
      r += m_Coefficients[i]*pt;
      pt *=t;
    }
  return r;
}

double Polynome::ComputeDerivative(double t)
{
  double r=0,pt=1;
  for(unsigned int i=1; i<m_Coefficients.size(); i++)
    {
      r += i*m_Coefficients[i]*pt;
      pt *=t;
    }
  return r;
}

double Polynome::ComputeSecDerivative(double t)
{
  double r=0,pt=1;
  for(unsigned int i=2; i<m_Coefficients.size(); i++)
    {
      r += i*(i-1)*m_Coefficients[i]*pt;
      pt *=t;
    }
  return r;
}

double Polynome::ComputeJerk(double t)
{
  double r=0,pt=1;
  for(unsigned int i=3; i<m_Coefficients.size(); i++)
    {
      r += i*(i-1)*(i-2)*m_Coefficients[i]*pt;
      pt *=t;
    }
  return r;
}

void Polynome::GetCoefficients(vector<double> &lCoefficients) const
{
  lCoefficients = m_Coefficients;
}


void Polynome::SetCoefficients(const vector<double> &lCoefficients)
{
  m_Coefficients = lCoefficients;
}

void Polynome::print() const
{
  for(unsigned int i=0; i<m_Coefficients.size(); i++)
    cout << m_Coefficients[i] << " " ;
  cout << endl;
}

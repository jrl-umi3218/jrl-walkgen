/* Polynomes object for trajectories.
      
   CVS Information:
   $Id: Polynome.cpp,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/Polynome.cpp,v $
   $Log: Polynome.cpp,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
   Olivier Stasse,
   Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   Please see License.txt for more details on the license.
*/

#include <iostream>
#include <Mathematics/Polynome.h>

using namespace::PatternGeneratorJRL;

Polynome::Polynome(int Degree)
{
  m_Coefficients.clear();
  m_Coefficients.resize(Degree+1);
}

Polynome::~Polynome()
{
}


double Polynome::Compute(double t)
{
  double r=0.0,pt=1.0;
  for(unsigned int i=0;i<m_Coefficients.size();i++)
    {
      r += m_Coefficients[i]*pt;
      pt *=t;
    }
  return r;
}

double Polynome::ComputeDerivative(double t)
{
  double r=0,pt=1;
  for(unsigned int i=1;i<m_Coefficients.size();i++)
    {
      r += i*m_Coefficients[i]*pt;
      pt *=t;
    }
  return r;
}

void Polynome::GetCoefficients(vector<double> &lCoefficients) const
{
 lCoefficients = m_Coefficients;
}


void Polynome::SetCoefficients(vector<double> &lCoefficients)
{
  m_Coefficients = lCoefficients;
}

void Polynome::print()
{
  for(unsigned int i=0;i<m_Coefficients.size();i++)
    cout << m_Coefficients[i] << " " ;
  cout << endl;
}

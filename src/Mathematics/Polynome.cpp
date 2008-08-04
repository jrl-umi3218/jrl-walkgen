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

#include <iostream>
#include <walkGenJrl/Mathematics/Polynome.h>

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
  double r=0,pt=1;
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

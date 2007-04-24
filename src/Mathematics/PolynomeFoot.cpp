/* Polynomes object for generating foot trajectories.

   CVS Information:
   $Id: PolynomeFoot.cpp,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/PolynomeFoot.cpp,v $
   $Log: PolynomeFoot.cpp,v $
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
#include <Mathematics/PolynomeFoot.h>
#include <vector>



using namespace::std;
using namespace::PatternGeneratorJRL;

Polynome3::Polynome3(double FT, double FP) :Polynome(3)
{
  SetParameters(FT,FP);
}

void Polynome3::SetParameters(double FT, double FP)
{
  double tmp;
  m_Coefficients[0] = 0.0;
  m_Coefficients[1] = 0.0;
  tmp = FT*FT;
  m_Coefficients[2] = 3.0*FP/tmp;
  m_Coefficients[3] = -2.0*FP/(tmp*FT);
}

Polynome3::~Polynome3()
{}
Polynome4::Polynome4(double FT, double FP) :Polynome(4)
{
  SetParameters(FT,FP);
}

void Polynome4::SetParameters(double FT, double MP)
{
  double tmp;
  m_Coefficients[0] = 0.0;
  m_Coefficients[1] = 0.0;
  tmp = FT*FT;
  m_Coefficients[2] = 16.0*MP/tmp;
  tmp=tmp*FT;
  m_Coefficients[3] = -32.0*MP/tmp;
  tmp=tmp*FT;
  m_Coefficients[4] = 16.0*MP/tmp;
}

Polynome4::~Polynome4()
{}
Polynome5::Polynome5(double FT, double FP) :Polynome(5)
{
  SetParameters(FT,FP);
}

void Polynome5::SetParameters(double FT, double FP)
{
  double tmp;
  m_Coefficients[0] = 0.0;
  m_Coefficients[1] = 0.0;
  m_Coefficients[2] = 0.0;
  tmp = FT*FT*FT;
  m_Coefficients[3] = 10*FP/tmp;
  tmp *=FT;
  m_Coefficients[4] = -15*FP/tmp;
  tmp*=FT;
  m_Coefficients[5] = 6*FP/tmp;
}

Polynome5::~Polynome5()
{}

Polynome6::Polynome6(double FT, double MP) :Polynome(6)
{
  SetParameters(FT,MP);
}

void Polynome6::SetParameters(double FT, double MP)
{
  double tmp;
  m_Coefficients[0] = 0.0;
  m_Coefficients[1] = 0.0;
  m_Coefficients[2] = 0.0;
  tmp = FT*FT*FT;
  m_Coefficients[3] = 64*MP/tmp;
  tmp *=FT;
  m_Coefficients[4] = -192*MP/tmp;
  tmp *=FT;
  m_Coefficients[5] = 192*MP/tmp;
  tmp *=FT;
  m_Coefficients[6] = -64*MP/tmp;
}

Polynome6::~Polynome6()
{  
}


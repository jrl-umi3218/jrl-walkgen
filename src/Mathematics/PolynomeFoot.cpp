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

   Please see License.txt for more details on the license.   
*/
#include <iostream>
#include <vector>

#include <Mathematics/PolynomeFoot.h>



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

void Polynome3::SetParametersWithInitPosInitSpeed(double FT,
						  double FP,
						  double InitPos,
						  double InitSpeed)
{
  double tmp;
  m_Coefficients[0] = InitPos;
  m_Coefficients[1] = InitSpeed;
  tmp = FT*FT;
  m_Coefficients[2] = (3*FP - 3*InitPos - 2*InitSpeed*FT)/tmp;
  m_Coefficients[3] = (InitSpeed*FT+ 2*InitPos - 2*FP)/(tmp*FT);
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

void Polynome4::SetParametersWithInitPosInitSpeed(double FT,
						  double MP,
						  double InitPos,
						  double InitSpeed)
{
  double tmp;
  m_Coefficients[0] = InitPos;
  m_Coefficients[1] = InitSpeed;
  tmp = FT*FT;
  m_Coefficients[2] = (-4.0*InitSpeed*FT - 11.0*InitPos + 16.0*MP)/tmp;
  tmp=tmp*FT;
  m_Coefficients[3] = ( 5.0*InitSpeed*FT + 18.0*InitPos - 32.0*MP)/tmp;
  tmp=tmp*FT;
  m_Coefficients[4] = (-2.0*InitSpeed*FT - 8.0 *InitPos + 16.0*MP)/tmp;
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


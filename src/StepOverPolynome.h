/** @doc Polynomes object for generating foot and hip trajectories while stepping over.

   $Id: PolynomeFoot.h,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/PolynomeFoot.h,v $
   $Log: PolynomeFoot.h,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati
   
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


#ifndef _STEPOVER_POLYNOME_H_
#define _STEPOVER_POLYNOME_H_

#include <Polynome.h>
#include <VNL/matrix.h>
#include <vector>
#include <VNL/vector.h>

namespace PatternGeneratorJRL
{
   /// Polynome used for Z trajectory during stepover.
  class StepOverPolynomeFoot : public Polynome
    {
    public:
      /// Constructor:
      /// boundCond: the different boundary conditions begin, intermediate and end of polynomial
      /// timeDistr: vector with time instants for intermediate boundary conditions and end time
    
      StepOverPolynomeFoot();
  
      /// Set the parameters
      void SetParameters(VNL::Vector<double> boundCond, vector<double> timeDistr);

      /// Destructor.
      ~StepOverPolynomeFoot();
    };



 /// Polynome used for Z trajectory during stepover.
  class StepOverPolynomeFootZtoX : public Polynome
    {
    public:
      /// Constructor:
      /// Zpos: vector with Zpos
      /// Xpos: vector Xpos
    
      StepOverPolynomeFootZtoX();
  
      /// Set the parameters
      void SetParameters(VNL::Vector<double> Zpos, vector<double> Xpos);

      /// Destructor.
      ~StepOverPolynomeFootZtoX();
    };


/// Polynome used for X trajectory in function of time to combine with StepOverPolynomeFootZtoX.
  class StepOverPolynomeFootXtoTime : public Polynome
    {
    public:
      /// Constructor:
      /// Zpos: vector with Zpos
    
    
      StepOverPolynomeFootXtoTime();
  
      /// Set the parameters
      void SetParameters(VNL::Vector<double> Xbound, vector<double> timedistr);

      /// Destructor.
      ~StepOverPolynomeFootXtoTime();
    };







class StepOverPolynomeHip4 : public Polynome
    {
    public:
      /// Constructor:
      /// boundCond: the different boundary conditions begin, intermediate and end of polynomial
      /// timeDistr: vector with time instants for intermediate boundary conditions and end time
    
      StepOverPolynomeHip4();
  
      /// Set the parameters
      void SetParameters(VNL::Vector<double> boundCond, vector<double> timeDistr);

      /// Destructor.
      ~StepOverPolynomeHip4();
    };




/// spline function calculation
 
///class to calculate cubic splines

 class StepOverSpline 
    {
    public:
      /// Constructor:
        
    
      StepOverSpline();
  
      /// Set the parameters
      void SetParameters(VNL::Vector<double> Points);

	double GetValueSpline(VNL::Vector<double> TimePoints, double CurrentLocalTime);

	void print();
      /// Destructor.
      ~StepOverSpline();
    protected:
	unsigned int number;
	VNL::Matrix<double> Coefficients;
    };

};




///class to calculate Clamped Cubic splines

 class StepOverClampedCubicSpline 
    {
    public:
      /// Constructor:
        
    
      StepOverClampedCubicSpline();
  
      /// Set the parameters
	void SetParameters(VNL::Vector<double> Points,VNL::Vector<double> TimePoints,VNL::Vector<double> DerivativeEndPoints);

	double GetValueSpline(VNL::Vector<double> TimePoints, double CurrentLocalTime);

	void print();
      /// Destructor.
      ~StepOverClampedCubicSpline();
    protected:
	unsigned int number;
	VNL::Matrix<double> Coefficients;
    };





#endif /* _STEPOVER_POLYNOME_H_ */

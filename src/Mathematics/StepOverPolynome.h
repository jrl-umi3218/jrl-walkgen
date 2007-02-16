/** Polynomes object for generating foot and hip trajectories while stepping over.
    
Copyright (c) 2005-2006, 
@author Bjorn Verrelst, Olivier Stasse
   
JRL-Japan, CNRS/AIST

All rights reserved.
   
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:
   
* Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.
* Neither the name of the CNRS/AIST nor the names of its contributors 
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

#include <MatrixAbstractLayer.h>
#include <Polynome.h>
#include <vector>


namespace PatternGeneratorJRL
{
  /*! @ingroup steppingover 
    @brief Polynome used for Z trajectory during stepover. */
  class StepOverPolynomeFoot : public Polynome
  {
  public:
    /*! Constructor:
      boundCond: the different boundary conditions begin, intermediate and end of polynomial
      timeDistr: vector with time instants for intermediate boundary conditions and end time */
    StepOverPolynomeFoot();
  
    /*! Set the parameters */
    void SetParameters(MAL_VECTOR(boundCond,double), 
		       vector<double> timeDistr);

    /*! Destructor. */
    ~StepOverPolynomeFoot();
  };



  /*! @ingroup steppingover
    @brief Polynome used for Z trajectory during stepover. */
  class StepOverPolynomeFootZtoX : public Polynome
  {
  public:
    /*! Constructor:
      Zpos: vector with Zpos
      Xpos: vector Xpos */
    StepOverPolynomeFootZtoX();
  
    /*! Set the parameters */
    void SetParameters(MAL_VECTOR(Zpos,double), 
		       vector<double> Xpos);

    /*! Destructor. */
    ~StepOverPolynomeFootZtoX();
  };


  /*! @ingroup stepping over
    @brief Polynome used for X trajectory in function of time 
    to combine with StepOverPolynomeFootZtoX.*/
  class StepOverPolynomeFootXtoTime : public Polynome
  {
  public:
    /*! Constructor:
      Zpos: vector with Zpos */
    StepOverPolynomeFootXtoTime();
  
    /*! Set the parameters */
    void SetParameters(MAL_VECTOR(Xbound,double), 
		       vector<double> timedistr);

    /*! Destructor. */
    ~StepOverPolynomeFootXtoTime();
  };

  /*! @ingroup steppingover 
    @brief Polynome for the hip trajectory. 
  */
  class StepOverPolynomeHip4 : public Polynome
  {
  public:
    /*! Constructor:
      boundCond: the different boundary conditions begin, intermediate and end of polynomial
      timeDistr: vector with time instants for intermediate boundary conditions and end time 
    */
    StepOverPolynomeHip4();
    
    // Set the parameters
    void SetParameters(MAL_VECTOR( boundCond,double),
		       vector<double> timeDistr);
    
    /*! Destructor. */
    ~StepOverPolynomeHip4();
  };
  



  /*! @ingroup steppingover 
    @brief spline function calculation
    class to calculate cubic splines */
  class StepOverSpline 
  {
  public:
    /*! Constructor: */
    StepOverSpline();
    
    /*! Set the parameters */
    void SetParameters(MAL_VECTOR( Points,double));
    
    double GetValueSpline(MAL_VECTOR( TimePoints,double),
			  double CurrentLocalTime);
    
    void print();

    /*! Destructor. */
    ~StepOverSpline();

  protected:
    unsigned int number;
    MAL_MATRIX( Coefficients,double);
  };

  /*! @ingroup steppingover
    class to calculate Clamped Cubic splines */
  class StepOverClampedCubicSpline 
  {
  public:
    /*! Constructor: */
    StepOverClampedCubicSpline();
    
    /*! Set the parameters */
    void SetParameters(MAL_VECTOR(Points,double),
		       MAL_VECTOR(TimePoints,double),
		       MAL_VECTOR(DerivativeEndPoints,double));
    
    double GetValueSpline(MAL_VECTOR(TimePoints,double), 
			  double CurrentLocalTime);
    
    void print();
    /*! Destructor. */
    ~StepOverClampedCubicSpline();
    
  protected:
    unsigned int number;
    MAL_MATRIX( Coefficients,double);
    
  };
  
};

#endif /* _STEPOVER_POLYNOME_H_ */

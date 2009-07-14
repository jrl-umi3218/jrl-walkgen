/** \file StepOverPolynome.h
    \brief Polynomes object for generating foot and hip trajectories while stepping over.
    
    Copyright (c) 2005-2006, 
    @author Bjorn Verrelst, Olivier Stasse, Francois Keith
    
    JRL-Japan, CNRS/AIST
    
    All rights reserved.
    
    Please see License.txt for further information on license.
*/


#ifndef _STEPOVER_POLYNOME_H_
#define _STEPOVER_POLYNOME_H_

#include <vector>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>


#include <Mathematics/Polynome.h>



namespace PatternGeneratorJRL
{
  /*! @ingroup steppingover 
    @brief Polynome used for Z trajectory during stepover. */
  class  StepOverPolynomeFoot : public Polynome
  {
  public:
    /*! Constructor:
      boundCond: the different boundary conditions begin, intermediate and end of polynomial
      timeDistr: vector with time instants for intermediate boundary conditions and end time */
    StepOverPolynomeFoot();
  
    /*! Set the parameters */
    void SetParameters(MAL_VECTOR(boundCond,double), 
		       std::vector<double> timeDistr);

    /*! Destructor. */
    ~StepOverPolynomeFoot();
  };



  /*! @ingroup steppingover
    @brief Polynome used for Z trajectory during stepover. */
  class  StepOverPolynomeFootZtoX : public Polynome
  {
  public:
    /*! Constructor:
      Zpos: vector with Zpos
      Xpos: vector Xpos */
    StepOverPolynomeFootZtoX();
  
    /*! Set the parameters */
    void SetParameters(MAL_VECTOR(Zpos,double), 
		       std::vector<double> Xpos);

    /*! Destructor. */
    ~StepOverPolynomeFootZtoX();
  };


  /*! @ingroup stepping over
    @brief Polynome used for X trajectory in function of time 
    to combine with StepOverPolynomeFootZtoX.*/
  class  StepOverPolynomeFootXtoTime : public Polynome
  {
  public:
    /*! Constructor:
      Zpos: vector with Zpos */
    StepOverPolynomeFootXtoTime();
  
    /*! Set the parameters */
    void SetParameters(MAL_VECTOR(Xbound,double), 
		       std::vector<double> timedistr);

    /*! Destructor. */
    ~StepOverPolynomeFootXtoTime();
  };

  /*! @ingroup steppingover 
    @brief Polynome for the hip trajectory. 
  */
  class  StepOverPolynomeHip4 : public Polynome
  {
  public:
    /*! Constructor:
      boundCond: the different boundary conditions begin, intermediate and end of polynomial
      timeDistr: vector with time instants for intermediate boundary conditions and end time 
    */
    StepOverPolynomeHip4();
    
    // Set the parameters
    void SetParameters(MAL_VECTOR( boundCond,double),
		       std::vector<double> timeDistr);
    
    /*! Destructor. */
    ~StepOverPolynomeHip4();
  };
  



  /*! @ingroup steppingover 
    @brief spline function calculation
    class to calculate cubic splines */
  class  StepOverSpline 
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
  class  StepOverClampedCubicSpline 
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

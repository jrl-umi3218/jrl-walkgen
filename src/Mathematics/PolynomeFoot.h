/** \file PolynomeFoot.h
    \brief Polynomes object for trajectories.
   All references are from Kajita san's book

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
   @author Olivier Stasse, Ramzi Sellouati, Francois Keith
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see License.txt for further information on license.   
*/


#ifndef _POLYNOME_FOOT_H_
#define _POLYNOME_FOOT_H_

#ifdef max
#undef max
#undef min
#endif

#include <vector>

#include <Mathematics/Polynome.h>

namespace PatternGeneratorJRL
{

  /// Polynome used for X,Y and Theta trajectories.
  class  Polynome3 : public Polynome
    {
    public:
      /** Constructor:
       FT: Final time
       FP: Final position */
      Polynome3(double FT, double FP);

      /*!  Set the parameters 
	This method assumes implictly a position
	set to zero, and a speed equals to zero.
       */
      void SetParameters(double FT, double FP);

      /*! Set the parameters such that
	the initial position, and initial speed
	are different from zero.
       */
      void SetParametersWithInitPosInitSpeed(double FT,
					     double FP,
					     double InitPos,
					     double InitSpeed);

      /// Destructor.
      ~Polynome3();
    };

  /// Polynome used for Z trajectory.
  class  Polynome4 : public Polynome
    {
    public:
      /** Constructor:
       FT: Final time
       MP: Middle position */
      Polynome4(double FT, double MP);
  
      /// Set the parameters
      void SetParameters(double FT, double MP);

      /*! Set the parameters such that
	the initial position, and initial speed
	are different from zero.
       */
      void SetParametersWithInitPosInitSpeed(double FT,
					     double MP,
					     double InitPos,
					     double InitSpeed);


      /// Destructor.
      ~Polynome4();
    };

  /// Polynome used for X,Y and Theta trajectories.
  class  Polynome5 : public Polynome
    {
    public:
      /** Constructor:
	  FT: Final time
	  FP: Final position */
      Polynome5(double FT, double FP);

      /// Set the parameters
      void SetParameters(double FT, double FP);
      /// Destructor.
      ~Polynome5();
    };

  /// Polynome used for Z trajectory.
  class  Polynome6 : public Polynome
    {
    public:
      /// Constructor:
      /// FT: Final time
      /// MP: Middle position
      Polynome6(double FT, double MP);
  
      /// Set the parameters
      void SetParameters(double FT, double MP);

      /// Destructor.
      ~Polynome6();
    };

};
#endif /* _POLYNOME_FOOT_H_ */

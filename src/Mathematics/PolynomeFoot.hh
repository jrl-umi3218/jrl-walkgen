/*
 * Copyright 2006, 2007, 2008, 2009, 2010, 
 *
 * Andrei     Herdt
 * Florent    Lamiraux
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
/** \file PolynomeFoot.h
    \brief Polynomes object for trajectories.
   All references are from Kajita san's book. */


#ifndef _POLYNOME_FOOT_H_
#define _POLYNOME_FOOT_H_

#ifdef max
#undef max
#undef min
#endif

#include <vector>

#include <Mathematics/Polynome.hh>

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
	This method assumes implicitly a position
	set to zero, and a speed equals to zero.
	Final velocity is 0
       */
      void SetParameters(double FT, double FP);

      /*! Set the parameters such that
	the initial position, and initial speed
	are different from zero.
	Final velocity is 0
       */
      void SetParametersWithInitPosInitSpeed(double &FT,
					     double &FP,
					     double &InitPos,
					     double &InitSpeed);

      void GetParametersWithInitPosInitSpeed(double &FT,
					     double &FP,
					     double &InitPos,
					     double &InitSpeed);
      /// Destructor.
      ~Polynome3();
      
    private:
      /*! Store final time and final position. */
      double m_FT, m_FP;
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
      // Initial velocity and position are 0
      // Final velocity and position are 0
      void SetParameters(double FT, double MP);

      /*! Set the parameters such that
	the initial position, and initial speed
	are different from zero.
	Final velocity and position are 0
       */
      void SetParametersWithInitPosInitSpeed(double FT,
					     double MP,
					     double InitPos,
					     double InitSpeed);


      /*! Get the parameters */
      void GetParametersWithInitPosInitSpeed(double &FT,
					     double &MP,
					     double &InitPos,
					     double &InitSpeed);

      /// Destructor.
      ~Polynome4();
      
    private:
      /*! Store final time and middle position. */
      double m_FT, m_MP;
      
    };

  /// Polynome used for X,Y and Theta trajectories.
  class  Polynome5 : public Polynome
    {
    private:
      double FT_, FP_, InitPos_, InitSpeed_,InitAcc_;
    public:
      /** Constructor:
	  FT: Final time
	  FP: Final position */
      Polynome5(double FT, double FP);

      /// Set the parameters
      void SetParameters(double FT, double FP);

      /*! Set the parameters such that
        the initial position, and initial speed
        are different from zero.
       */
      void SetParametersWithInitPosInitSpeed(double FT,
                                             double FP,
                                             double InitPos,
                                             double InitSpeed);
      /*! Set the parameters such that
        the initial position, and initial speed
        are different from zero.
       */
      void GetParametersWithInitPosInitSpeed(double &FT,
                                             double &FP,
                                             double &InitPos,
                                             double &InitSpeed);

      /// \brief Set parameters considering initial position, velocity, acceleration
      void SetParameters(double FT, double FP,
          double InitPos, double InitSpeed, double InitAcc);
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
      // Initial acceleration, velocity and position by default 0
      // Final acceleration, velocity and position are 0
      void SetParameters(double FT, double MP);
      void SetParameters(double FT, double PM,
		  	  double InitPos, double InitSpeed, double InitAcc);

      /// Destructor.
      ~Polynome6();
    };

}
#endif /* _POLYNOME_FOOT_H_ */

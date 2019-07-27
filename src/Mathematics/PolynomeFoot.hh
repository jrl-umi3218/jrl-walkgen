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
  class PolynomeFoot : public Polynome
  {
  protected :
    /*! Store final time */
    double FT_;

  public :

    PolynomeFoot(int degree=0.0, double FT=0.0) : Polynome(degree), FT_(FT)
    {};

    /*! Compute the value. */
    double Compute(double t);

    /*! Compute the value of the derivative. */
    double ComputeDerivative(double t);

    /*! Compute the value of the second derivative. */
    double ComputeSecDerivative(double t);

    /*! Compute the value of the third derivative (jerk). */
    double ComputeJerk(double t);

  };

  /// Polynome used for X,Y and Theta trajectories.
  class  Polynome3 : public PolynomeFoot
  {
  public:
    /** Constructor:
        FT: Final time
        FP: Final position */
    Polynome3(double FT, double FP);
    /** Additionnal Constructor:
     */
    Polynome3(double FT,
              double IP, double IS,
              double FP, double FS);

    /*!  Set the parameters
      This method assumes implicitly a position
      set to zero, and a speed equals to zero.
      Final velocity is 0
    */
    void SetParameters(double FT, double FP);

    /*!  Set the parameters
     */
    void SetParameters(double FT,
                       double IP, double IS,
                       double FP, double FS);

    /*! Set the parameters such that
      the initial position, and initial speed
      are different from zero.
      Final velocity is 0
    */
    void SetParametersWithInitPosInitSpeed(double FT,
                                           double FP,
                                           double InitPos,
                                           double InitSpeed);

    void GetParametersWithInitPosInitSpeed(double &FT,
                                           double &FP,
                                           double &InitPos,
                                           double &InitSpeed);
    /// Destructor.
    ~Polynome3();

  private:
    /*! Store final time and final position. */
    double FP_;
  };

  /// Polynome used for Z trajectory.
  class  Polynome4 : public PolynomeFoot
  {
  public:
    /** Constructor:
        FT: Final time
        MP: Middle position */
    Polynome4(double FT, double MP, double FP=0.0);

    /// Set the parameters
    // Initial velocity and position are 0
    // Final velocity and position are 0
    void SetParameters(double FT, double MP, double FP=0.0);

    /// Set the parameters
    // time horizon
    // Initial Position
    // Initial velocity (IS)
    // Initial Acceleration
    // Final velocity
    // Final Acceleration
    void SetParameters(double FT,
                       double InitPos,
                       double InitSpeed,
                       double InitAcc,
                       double FinalSpeed,
                       double FinalAcc);

    /*! Set the parameters such that
      the initial position, and initial speed
      are different from zero.
      Final velocity and position are 0
    */
    void SetParametersWithInitPosInitSpeed(double FT,
                                           double MP,
                                           double InitPos,
                                           double InitSpeed,
                                           double FP = 0.0);


    /*! Get the parameters */
    void GetParametersWithInitPosInitSpeed(double &FT,
                                           double &MP,
                                           double &FP,
                                           double &InitPos,
                                           double &InitSpeed);

    /// Destructor.
    ~Polynome4();

  private:
    /*! Store final time and middle position. */
    double MP_;
    double FP_;

  };

  /// Polynome used for X,Y and Theta trajectories.
  class  Polynome5 : public PolynomeFoot
  {
  private:
    double InitPos_, InitSpeed_, InitAcc_, FinalPos_, FinalSpeed_, FinalAcc_;
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

    /// \brief Set parameters considering initial position, velocity,
    /// acceleration
    void SetParameters
    (double FT, double FP,
     double InitPos, double InitSpeed, double InitAcc,
     double InitJerk = 0.0);

    /// \brief Set parameters considering initial position, velocity,
    /// acceleration,
    /// and final poistion, velocity and acceleration
    void SetParameters(double FT,
                       double InitPos, double InitSpeed, double InitAcc,
                       double FinalPos, double FinalSpeed, double FinalAcc);

    /// Destructor.
    ~Polynome5();

  };

  /// Polynome used for Z trajectory.
  class  Polynome6 : public PolynomeFoot
  {
  private:
    double MP_, FP_, InitPos_, InitSpeed_,InitAcc_;
  public:
    /// Constructor:
    /// FT: Final time
    /// MP: Middle position
    Polynome6(double FT, double MP, double FP=0.0);

    /// Set the parameters
    // Initial acceleration, velocity and position by default 0
    // Final acceleration, velocity and position are 0
    void SetParameters(double FT, double MP, double FP = 0.0);
    void SetParametersWithMiddlePos
    (double FT, double MP,
     double InitPos, double InitSpeed, double InitAcc=0.0, double FP = 0.0);
    
    void GetParametersWithInitPosInitSpeed(double &TimeInterval,
                                           double &MiddlePosition,
                                           double &FinalPosition,
                                           double &InitPosition,
                                           double &InitSpeed);
    /// Destructor.
    ~Polynome6();
  };

  /// Polynome used for X,Y and Theta trajectories.
  class  Polynome7 : public PolynomeFoot
  {
  private:
    double FP_, InitPos_, InitSpeed_,InitAcc_,InitJerk_;
  public:
    /** Constructor:
        FT: Final time
        FP: Final position */
    Polynome7(double FT, double FP);

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

    /// \brief Set parameters considering initial position,
    /// velocity, acceleration, jerk
    void SetParameters
    (double FT, double FP,
     double InitPos, double InitSpeed, double InitAcc,
     double InitJerk=0.0);
    

    /*! Set the parameters such that
      the initial position, and initial speed
      are different from zero.
    */
    void GetParametersWithInitPosInitSpeed(double &FT,
                                           double &FP,
                                           double &InitPos,
                                           double &InitSpeed);



    /// Destructor.
    ~Polynome7();

  };

}
#endif /* _POLYNOME_FOOT_H_ */

/*
 * Copyright 2010,
 *
 * Andrei Herdt
 * Olivier  Stasse
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
/*! \file privatepgtypes.h
  \brief Defines basic types for the Humanoid Walking Pattern Generator.
*/

#ifndef _PATTERN_GENERATOR_INTERNAL_PRIVATE_H_
#define  _PATTERN_GENERATOR_INTERNAL_PRIVATE_H_

#include <jrl/mal/matrixabstractlayer.hh>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

namespace PatternGeneratorJRL
{

  /// \brief State of the center of mass
  struct com_s
  {
    MAL_VECTOR(x,double);
    MAL_VECTOR(y,double);
    MAL_VECTOR(z,double);

    struct com_s & operator=(const com_s &aCS);

    void reset();

    com_s();
  };
  typedef struct com_s com_t;

  // Support state of the robot at a certain point in time
  struct trunk_s
  {
    MAL_VECTOR(x,double);
    MAL_VECTOR(y,double);
    MAL_VECTOR(z,double);

    MAL_VECTOR(yaw,double);
    MAL_VECTOR(pitch,double);
    MAL_VECTOR(roll,double);

    struct trunk_s & operator=(const trunk_s &aTS);

    void reset();

    trunk_s();
  };
  typedef struct trunk_s trunk_t;

  //State of the feet on the ground
  struct supportfoot_s
  {
    double x,y,theta,StartTime;
    int SupportFoot;
  };
  typedef struct supportfoot_s
    supportfoot_t;

  /// Absolute reference.
  struct reference_s
  {
    struct frame_s
    {
      /// \brief Constant reference
      double x,y, yaw;

      /// \brief Reference vectors
      MAL_VECTOR(X,double);
      MAL_VECTOR(Y,double);
      MAL_VECTOR(YAW,double);
    };
    typedef struct frame_s frame_t;

    frame_t Global, Local;

  };
  typedef struct reference_s reference_t;

  /// \brief Linear inequality with free foot placement.
  struct linear_inequality_ff_s
  {
    MAL_MATRIX(D,double);
    MAL_MATRIX(Dc,double);
    int StepNumber;
  };
  typedef struct linear_inequality_ff_s
    linear_inequality_ff_t;

  /// \brief Linear constraints
  struct linear_constraint_s
  {
    boost_ublas::compressed_vector<double> A;
    double b;
  };
  typedef struct linear_constraint_s
  linear_constraint_t;

  /// \brief Set of 2-dimensional points
  struct convex_hull_s
  {

    boost_ublas::vector<double> X;
    boost_ublas::vector<double> Y;

    /// \brief Rotate the points around the origin by angle
    ///
    /// \param[in] Angle
    void rotate( double Angle );

    /// \brief Resize members to the desired number of points
    ///
    /// \param[in] size
    void resize( int Size );

    /// \brief Set the vectors from arrays
    ///
    /// \param[in] X
    /// \param[in] Y
    void set(const double * X_a, const double * Y_a);

    /// \brief Set all points to zero
    void reset();

    convex_hull_s( int Size );
    convex_hull_s();

  };
  typedef struct convex_hull_s convex_hull_t;


  /// \brief Linear inequalities set
  struct linear_inequality_s
  {
    struct coordinate_s
    {
      boost_ublas::compressed_matrix<double, boost_ublas::row_major> x;
      boost_ublas::compressed_matrix<double, boost_ublas::row_major> y;
    };
    struct coordinate_s D;

    boost_ublas::vector<double> dc;

    /// \brief Classifier
    int type;

    /// \brief Fill all elements with zeros
    void clear();

    /// \brief Resize all elements
    void resize( int NbRows, int NbCols, bool Preserve );
  };
  typedef struct linear_inequality_s
    linear_inequality_t;

  /// \brief Solution
  struct solution_s
  {

    /// \brief Size of the solution array
    unsigned int NbVariables;

    /// \brief Number of constraints (lagrange multipliers)
    unsigned int NbConstraints;

    /// \brief SHOWS THE TERMINATION REASON.
    ///   IFAIL = 0 :   SUCCESSFUL RETURN.
    ///   IFAIL = 1 :   TOO MANY ITERATIONS (MORE THAN 40*(N+M)).
    ///   IFAIL = 2 :   ACCURACY INSUFFICIENT TO SATISFY CONVERGENCE
    ///                 CRITERION.
    ///   IFAIL = 5 :   LENGTH OF A WORKING ARRAY IS TOO SHORT.
    ///   IFAIL > 10 :  THE CONSTRAINTS ARE INCONSISTENT.
    int Fail;

    /// \brief OUTPUT CONTROL.
    ///   IPRINT = 0 :  NO OUTPUT OF QL0001.
    ///   IPRINT > 0 :  BRIEF OUTPUT IN ERROR CASES.
    int Print;

    /// \brief Solution vector
    boost_ublas::vector<double> Solution_vec;

    /// \brief Lagrange multipliers of the constraints
    boost_ublas::vector<double> ConstrLagr_vec;
    /// \brief Lagrange multipliers of the lower bounds
    boost_ublas::vector<double> LBoundsLagr_vec;
    /// \brief Lagrange multipliers of the upper bounds
    boost_ublas::vector<double> UBoundsLagr_vec;

    /// \brief Resize solution containers
    void resize( unsigned int NbVariables, unsigned int NbConstraints );

    /// \brief Dump solution
    /// \param Filename
    void dump( const char * Filename );
    /// \brief Print_ solution
    /// \param aos
    void print( std::ostream & aos);

  };
  typedef struct solution_s solution_t;

  enum FootType
  {
    LEFT, RIGHT
  };

  enum PhaseType
  {
    SS, DS
  };

  /// \brief Support state of the robot at a certain point in time
  struct support_state_s
  {

    /// \brief Support phase
    PhaseType Phase;
    /// \brief Support foot
    FootType Foot;
    /// \brief Number steps left before double support
    unsigned int NbStepsLeft;
    /// \brief Number of step previewed
    unsigned int StepNumber;

    /// \brief Time until StateChanged == true
    double TimeLimit;
    /// \brief start time
    double StartTime;
    /// \brief Position and orientation on a plane
    double X,Y,Yaw;

    /// \brief (true) -> New single support state
    bool StateChanged;

    /// \brief Number of samplings passed in this phase
    unsigned int NbInstants;

    struct support_state_s & operator = (const support_state_s &aSS);

    void reset();

    support_state_s();
  };
  typedef struct support_state_s support_state_t;

}

#endif /* _PATTERN_GENERATOR_INTERNAL_PRIVATE_H_ */

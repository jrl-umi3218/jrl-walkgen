/*
 * Copyright 2011,
 *
 * Olivier  Stasse
 * Andrei   Herdt
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


#ifndef INTERMEDQPMAT_HH_
#define INTERMEDQPMAT_HH_

#include <jrl/mal/matrixabstractlayer.hh>
#include <privatepgtypes.h>

#include <iostream>
#include <fstream>

#include <PreviewControl/rigid-body-system.hh>


namespace PatternGeneratorJRL
{
  /// \brief Custom (value based) container providing intermediate elements for the construction of a QP.
  class  IntermedQPMat
  {
    //
    //Public types
    //
  public:

    const static int MEAN_VELOCITY = 0;
    const static int INSTANT_VELOCITY = 1;
    const static int COP_CENTERING = 2;
    const static int JERK_MIN = 3;

    const static int INEQ_COP = 20;
    const static int INEQ_COM = 21;
    const static int INEQ_FEET = 22;

    /// \name Matrices defining the evolution
    /// \{
    struct dynamics_s
    {
      /// \brief Matrix of the quadratic part
      boost_ublas::matrix<double> U;
      /// \brief Transpose of U
      boost_ublas::matrix<double> UT;

      /// \brief Matrix of the linear part
      boost_ublas::matrix<double> S;

      int type;
    };
    typedef dynamics_s dynamics_t;

    /// \name QP elements that are objective independent
    /// \{
    struct state_variant_s
    {
      /// \brief Objective independent QP elements

      /// \brief Reference
      reference_t Ref;

      /// \brief State of the Center of Mass
      com_t CoM;

      /// \brief TrunkState
      trunk_t Trunk;

      /// \brief Selection matrix for the previewed feet positions
      boost_ublas::matrix<double> V;
      /// \brief Transpose of V
      boost_ublas::matrix<double> VT;
      /// \brief Selection matrix for the current feet position
      boost_ublas::vector<double> Vc;
      /// \brief Selection matrix for the current foot position
      boost_ublas::vector<double> Vc_f;
      /// \brief Selection matrix for relative feet positions
      boost_ublas::matrix<double> V_f;
      /// \brief Current support state
      support_state_t SupportState;
    };
    typedef state_variant_s state_variant_t;
    /// \}

    /// \name Least square objective's dependent elements
    /// \{
    struct objective_variant_s
    {
      /// \brief Ponderation
      double weight;

      /// \brief Minimization objective type
      int type;

      std::ostream& print (std::ostream& o) const;
      void dump(const char * filename) const;
    };
    typedef objective_variant_s objective_variant_t;
    /// \}

    //
    //Public methods
    //
  public:
    /// \name Constructors and destructors.
    /// \{
    IntermedQPMat();
    ~IntermedQPMat();
    /// \}

    /// \name Accessors
    /// \{
    /// \brief Getter and setter

    inline state_variant_t const & State() const
    { return StateMatrices_; };
    inline state_variant_t & State()
    { return StateMatrices_; };

    objective_variant_t const & Objective( int type ) const;
    objective_variant_t & Objective( int type );

    dynamics_t const & Dynamics( int type ) const;
    dynamics_t & Dynamics( int type );

    linear_inequality_t const & Inequalities( int type ) const;
    linear_inequality_t & Inequalities( int type );

    inline com_t const & CoM() const
    { return StateMatrices_.CoM; };
    inline void CoM( const com_t & CoM )
    { StateMatrices_.CoM = CoM; };

    inline reference_t const & Reference() const
    { return StateMatrices_.Ref; };
    inline reference_t & Reference()
    { return StateMatrices_.Ref; };
    inline void Reference( const reference_t & Ref )
    { StateMatrices_.Ref = Ref; };

    inline support_state_t const & SupportState() const
    { return StateMatrices_.SupportState; };
    inline support_state_t & SupportState()
    { return StateMatrices_.SupportState; };
    inline void SupportState( const support_state_t & SupportState )
    { StateMatrices_.SupportState = SupportState; };
    /// \}

    /// \name Displaying
    /// \{
    /// \brief Dump
    void dump_objective( const int ObjectiveType, std::ostream &aos );
    void dump_state( std::ostream &aos );
    void dump_objective(const char * filename, const int Objectivetype);
    void dump_state(const char * filename);
    /// \}

    //
    //Private members
    //
  private:

    objective_variant_t
    MeanVelocity_,
    InstantVelocity_,
    COPCentering_,
    JerkMin_;

    state_variant_t
    StateMatrices_;

    linear_inequality_t
    IneqCoP_,
    IneqCoM_,
    IneqFeet_;

  };

  std::ostream& operator<< (std::ostream& o, const IntermedQPMat::objective_variant_s& r);
}



#endif /* INTERMEDQPMAT_HH_ */

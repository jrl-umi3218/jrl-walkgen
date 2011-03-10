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
/*! Custom (value based) container providing intermediate elements for the construction of a QP.
 */

#ifndef INTERMEDQPMAT_HH_
#define INTERMEDQPMAT_HH_

#include <jrl/mal/matrixabstractlayer.hh>
#include <privatepgtypes.h>

#include <iostream>
#include <fstream>


namespace PatternGeneratorJRL
{

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

    const static int POSITION = 10;
    const static int VELOCITY = 11;
    const static int ACCELERATION = 12;
    const static int JERK = 13;
    const static int COP = 14;

    const static int INEQ_COP = 20;
    const static int INEQ_COM = 21;
    const static int INEQ_FEET = 22;

    /// \name Matrices defining the evolution
    /// \{
    struct dynamics_s
    {
      /// \brief Matrix of the quadratic part
      boost_ublas::matrix<double> U;
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
      /// \brief Reference
      reference_t Ref;

      /// \brief State of the Center of Mass
      com_t CoM;

      /// \brief TrunkState
      trunk_t Trunk;

      /// \name Selection matrices
      /// \{
      ///
      /// \brief Selection matrix for the previewed feet positions
      boost_ublas::matrix<double> V;
      boost_ublas::matrix<double> VT;
      /// \brief Selection matrix for the current feet position
      boost_ublas::vector<double> Vc;
      /// \brief Selection matrix for the current foot position
      boost_ublas::vector<double> Vc_f;
      /// \brief Selection matrix for relative feet positions
      boost_ublas::matrix<double> V_f;
      /// \}

      /// \brief Current support foot position
      supportfoot_t SupportFoot;

      /// \}
    };
    typedef state_variant_s state_variant_t;
    /// \}

    /// \name Least square objective's dependent elements
    /// \{
    struct objective_variant_s
    {
      double weight;

      dynamics_t * dyn;

      /// \brief Minimization objective
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
    /// \brief Accessors to the state matrices
    inline state_variant_t const & State() const
    { return m_StateMatrices; };
    inline state_variant_t & State()
    { return m_StateMatrices; };

    /// \brief Accessors to the objective dependent matrices
    objective_variant_t const & Objective( const int type ) const;
    objective_variant_t & Objective( const int type );

    /// \brief Accessors to the dynamics
    dynamics_t const & Dynamics( const int type ) const;
    dynamics_t & Dynamics( const int type );

    /// \brief Accessors to the inequalities
    linear_inequality_t const & Inequalities( const int type ) const;
    linear_inequality_t & Inequalities( const int type );

    /// \brief Accessors to the Center of Mass
    inline com_t const & CoM() const
    { return m_StateMatrices.CoM; };
    inline void CoM( const com_t & CoM )
    { m_StateMatrices.CoM = CoM; };

    /// \brief Accessors to the reference
    inline reference_t const & Reference() const
    { return m_StateMatrices.Ref; };
    inline reference_t & Reference()
    { return m_StateMatrices.Ref; };
    inline void Reference( const reference_t & Ref )
    { m_StateMatrices.Ref = Ref; };

    /// \brief Accessors to the Center of Mass
    inline supportfoot_t const & SupportFoot() const
    { return m_StateMatrices.SupportFoot; };
    inline void SupportFoot( const supportfoot_t & SupportFoot )
    { m_StateMatrices.SupportFoot = SupportFoot; };
    /// \}

    /// \name Displaying
    /// \{
    /// \brief Dump objective matrices
    void dumpObjective( const int ObjectiveType, std::ostream &aos );
    void dumpState( std::ostream &aos );
    void dumpObjective(const char * filename, const int Objectivetype);
    void dumpState(const char * filename);
    /// \}

    //
    //Private members
    //
  private:

    objective_variant_t
    m_MeanVelocity,
      m_InstantVelocity,
      m_COPCentering,
      m_JerkMin;

    state_variant_t
    m_StateMatrices;

    dynamics_t
    m_Position,
      m_Velocity,
      m_Acceleration,
      m_Jerk,
      m_CoP;

    linear_inequality_t
    m_IneqCoP,
      m_IneqCoM,
      m_IneqFeet;

    /// \brief Cholesky decomposition of the initial objective function $Q$
    MAL_MATRIX(m_LQ,double);
    /// \brief Cholesky decomposition of the initial objective function $Q$
    MAL_MATRIX(m_iLQ,double);
    /// \brief Constant part of the constraint matrices.
    MAL_MATRIX(m_iDu,double);
    /// \brief Constant part of the constraint matrices.
    MAL_MATRIX(m_Ds,double);
    /// \brief Sub matrices to compute the linear part of the objective function $p^{\top}_k$.
    MAL_MATRIX(m_OptA,double);
    MAL_MATRIX(m_OptB,double);
    MAL_MATRIX(m_OptC,double);
    MAL_MATRIX(m_OptD,double);

  };

  std::ostream& operator<< (std::ostream& o, const IntermedQPMat::objective_variant_s& r);
}



#endif /* INTERMEDQPMAT_HH_ */

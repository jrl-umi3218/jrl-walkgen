/*
 * Copyright 2010,
 *
 * Andrei   Herdt
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
/*! This object provides intermediate elements and operations that are necessary to construct a QP.
 */

#ifndef INTERMEDQPMAT_HH_
#define INTERMEDQPMAT_HH_

//# include <boost/numeric/ublas/matrix.hpp>
//# include <boost/numeric/ublas/vector.hpp>
#include <jrl/mal/matrixabstractlayer.hh>
#include <privatepgtypes.h>

#include <iostream>
#include <fstream>

namespace PatternGeneratorJRL
{
  //namespace ublas = boost::numeric::ublas; 

  class  IntermedQPMat
  {
    //
    //Public types
    //
  public:

    const static int MEAN_VELOCITY = 0;
    const static int INSTANT_VELOCITY = 1;
    const static int COP_CENTERING = 2;
    const static int JERK = 3;

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

      /// \brief Selection matrix for the previewed and current feet positions.
      MAL_MATRIX(V,double);
      MAL_MATRIX(VT,double);
      MAL_VECTOR(Vc,double);

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
      /// \brief Matrix of the quadratic part
      MAL_MATRIX(U,double);
      MAL_MATRIX(UT,double);

      /// \brief Matrix of the linear part
      MAL_MATRIX(S,double);

      /// \brief Minimization objective
      int type;

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

    /// \brief Accessors to the state matrices
    inline state_variant_t const & State() const
    { return m_StateMatrices; };
    inline state_variant_t & State()
    { return m_StateMatrices; };

    /// \brief Accessors to the objective dependent matrices
    objective_variant_t const & Objective( const int aObjectiveType ) const;
    objective_variant_t & Objective( const int aObjectiveType );

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

    /// \brief Dumping
    void dumpObjective( const int ObjectiveType, std::ostream &aos );
    void dumpState( std::ostream &aos );
    void dumpObjective(const char * filename, const int Objectivetype);
    void dumpState(const char * filename);

    //
    //Private members
    //
  private:

    objective_variant_t
      m_MeanVelocity,
      m_InstantVelocity,
      m_COPCentering,
      m_Jerk;

    state_variant_t m_StateMatrices;

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
}



#endif /* INTERMEDQPMAT_HH_ */

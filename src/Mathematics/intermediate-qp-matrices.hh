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


    /// \name Standardized least square elements
    /// \{
    /// \brief Standard form: \f$ Q = \alpha M1^{\top}M2, p = \alpha U^{\top}(S*c-ref) \f$
    struct standard_ls_form_s
    {
      double weight;
      /// \brief Matrix of the quadratic part
      MAL_MATRIX(U,double);
      MAL_MATRIX(UT,double);
      MAL_MATRIX(V,double);
      MAL_MATRIX(VT,double);

      /// \brief Reference
      MAL_VECTOR(Sc_x,double);
      MAL_VECTOR(Sc_y,double);

      /// \brief Reference
      MAL_VECTOR(ref_x,double);
      MAL_VECTOR(ref_y,double);

      /// \brief Final products (the ones that are added to the final QP)
      MAL_MATRIX(weightM1TM2,double);
      MAL_VECTOR(weightM1TV1,double);

      /// \brief Number of previewed steps
      int nSt;
    };
    typedef standard_ls_form_s standard_ls_form_t;
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

    /// \brief Get matrices in the standard Least Squares form:
    /// \f$ Q = \alpha U^{\top}U, p = \alpha U^{\top}(S*c-ref) \f$
    void getTermMatrices(standard_ls_form_t & TMat, int ObjectiveType);
	  
    //
    //Private members
    //
  private:

    /// \name Least square objective's dependent elements
    /// \{
    struct invariant_objective_part_s
    {
      double weight;
      /// \brief Matrix of the quadratic part
      MAL_MATRIX(U,double);
      MAL_MATRIX(UT,double);

      /// \brief Matrix of the linear part
      MAL_MATRIX(S,double);
    };
    typedef invariant_objective_part_s invariant_objective_part_t;
    /// \}
    invariant_objective_part_t
      m_MeanVelocity,
      m_InstantVelocity,
      m_COPCentering,
      m_Jerk;

    /// \name QP elements that are objective independent
    /// \{
    struct variant_objective_part_s
    {
      /// reference values for the whole preview window
      MAL_VECTOR(RefX,double);
      MAL_VECTOR(RefY,double);
      MAL_VECTOR(RefTheta,double);

      /// \brief State of the Center of Mass
      MAL_VECTOR(CoMX,double);
      MAL_VECTOR(CoMY,double);
      MAL_VECTOR(CoMZ,double);

      /// \brief Selection matrix for the previewed and current feet positions.
      MAL_MATRIX(V,double);
      MAL_MATRIX(VT,double);
      MAL_VECTOR(Vc,double);

      /// \brief Current support foot position
      double fx, fy;

      /// \brief Number of free steps in the preview window
      int NbStepsPrw;
      /// \}
    };
    typedef variant_objective_part_s variant_objective_part_t;
    /// \}
    variant_objective_part_t m_StateMatrices;


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

	  
    enum MatrixType  { M_UP, M_UZ, M_UV, M_SP,
		       M_SZ, M_SV, M_U,
		       M_LQ, M_ILQ};
    enum VectorType  { M_UC,M_OPTA, M_OPTB, M_OPTC, M_OPTD };


  };
}



#endif /* INTERMEDQPMAT_HH_ */

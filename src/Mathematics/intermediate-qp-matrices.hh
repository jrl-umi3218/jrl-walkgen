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
    //Public constants
    //
  public:
    const static int MEAN_VELOCITY = 0;
    const static int INSTANT_VELOCITY = 1;
    const static int FOOT_CENTERING = 2;
    const static int JERK = 3;

    //
    //Public methods
    //
  public:
    /// \name Constructors and destructors.
    /// \{
    IntermedQPMat();
    ~IntermedQPMat();
    /// \}

    /// \brief Get matrices corresponding to the desired Least Squares term
    void getLSTerm(ObjectiveTerm_t LST, int ObjectiveType);
	
    //
    //Private methods
    //
  private:

    /// \brief \f$ M1 = \alpha M2^{\top}M3 \f$
    void weightedTransProd(MAL_MATRIX (&M1, double), double alpha, MAL_MATRIX (&M2, double), MAL_MATRIX (&M3, double));
	  
    /// \brief \f$ M1 = \alpa M2^{\top}V \f$
    void weightedTransProd(MAL_MATRIX (&M1, double), double alpha, MAL_MATRIX (&M2, double), MAL_VECTOR (&V, double));  
	  
    /// \brief \f$ M1 = \alpa M2^{\top}M3V \f$
    void weightedTransProd(MAL_MATRIX (&M1, double), double alpha, MAL_VECTOR (&M2, double), MAL_VECTOR (&M3, double), MAL_VECTOR (&V, double));    
	  
	  
    //
    //Private members
    //
  private:

	  
    /// \brief Current state of the point mass model.
    MAL_VECTOR(m_xk,double);
	  
    /// \brief Matrix relating the command and the CoM position.
    MAL_MATRIX(m_Up,double);

    /// \brief Matrix relating the command and the ZMP position.
    MAL_MATRIX(m_Uz,double);

    /// \brief Matrix relating the command and the CoM speed.
    MAL_MATRIX(m_Uv,double);

    /// \brief Matrix relating the CoM state and the CoM position.
    MAL_MATRIX(m_Sp,double);

    /// \brief Matrix relating the CoM state and the ZMP position.
    MAL_MATRIX(m_Sz,double);

    /// \brief Matrix relating the CoM state and the CoM speed.
    MAL_MATRIX(m_Sv,double);

    /// \brief Selection matrix for the previewed feet positions.
    MAL_MATRIX(m_U,double);

    /// \brief Selection matrix for the support feet.
    MAL_VECTOR(m_Uc,double);

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
    enum VectorType  { m_UC,M_OPTA, M_OPTB, M_OPTC, M_OPTD };


  };
}



#endif /* INTERMEDQPMAT_HH_ */

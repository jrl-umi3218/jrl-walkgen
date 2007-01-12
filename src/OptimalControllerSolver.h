/** @doc Object to compute the weights of the preview control
    @ingroup previewcontrol
    @endgroup


   Copyright (c) 2005-2006, 
   @author Olivier Stass
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef _OPTIMAL_CONTROLLER_SOLVER_H_
#define _OPTIMAL_CONTROLLER_SOLVER_H_

#include <MatrixAbstractLayer.h>

namespace PatternGeneratorJRL
{
  /**
     This class computes the gains for preview control for a given 
     discrete system. The discrete system is defined by three matrix A, b, c
     such as :
     \f{eqnarray*}

     {\bf x}_{k+1} & =& {\bf A} x_k + {\bf b} u_k \\
     p_k &=& {\bf cx}_k\\
     \f}

     The optimal critera considered here is :
     \f[
     J = \sum^{\infty}_{j=1} \{ Q(p^{ref}_j -p_j)^2 + Ru_j^2 \}
     \f]

     where \f$ Q \f$ and \f$ R \f$ are also given as inputs.

     the solution is then 
     \f{eqnarray*}
     {\bf K} & \equiv & (R + {\bf b}^T{\bf Pb})^{-1}{\bf b}^T{\bf PA} \\
     f_i & \equiv & (R + {\bf b}^T{\bf Pb})^{-1}{\bf b}^T({\bf A}-{\bf bK})^{T*(i-1)}{\bf c}^TQ \\
     \f}

     where \f$ {\bf P} \f$ is solution of the following Riccati equation:
     \f[
     {\bf P} = {\bf A}^T {\bf PA} + {\bf c}^TQ{\bf c} - {\bf A}^T{\bf Pb}(R + {\bf b}^T{\bf Pb})^{-1}{\bf b}^T{\bf PA}
     \f]
     

     The resolution of the Riccati equation is taken from \ref Laub1979, and is
     based on a Schur form .

     \anchor Laub1979
     Alan J. Laub A Schur method for solving Algebraic Riccati Equations, IEEE Transaction on Automatic Control,
     Vol AC-24, No.6 December 1979
   */
  class OptimalControllerSolver
  {
  public:
    /*! A constructor */
    OptimalControllerSolver(MAL_MATRIX(&A,double), 
			    MAL_MATRIX(&b,double), 
			    MAL_MATRIX(&c,double),
			    double Q, double R, 
			    unsigned int Nl);

    /*! Destructor. */
    ~OptimalControllerSolver();

    /*! Compute the weights */
    void ComputeWeights();
    
    /*! Display the weights */
    void DisplayWeights();

    bool GeneralizedSchur(MAL_MATRIX(&A,double),
			  MAL_MATRIX(&B,double),
			  MAL_VECTOR(&alphar,double),
			  MAL_VECTOR(&alphai,double),
			  MAL_VECTOR(&beta,double),
			  MAL_MATRIX(&L,double),
			  MAL_MATRIX(&R,double));
  protected:
    
    /*! The matrices needed for the dynamical system such as
      \f{eqnarray*}
      
      {\bf x}_{k+1} & =& {\bf A} x_k + {\bf b} u_k \\
      p_k &=& {\bf cx}_k\\
      \f}
      
     */
    MAL_MATRIX(m_A,double);
    MAL_MATRIX(m_b,double);
    MAL_MATRIX(m_c,double);

    /*! The coefficent of the index criteria:
     \f[
     J = \sum^{\infty}_{j=1} \{ Q(p^{ref}_j -p_j)^2 + Ru_j^2 \}
     \f]
     */
    double m_Q, m_R;

    
    /*! The weights themselves */
    MAL_MATRIX(m_K,double); 
    MAL_MATRIX(m_F,double);
			  
    /*! The size of the window for the preview */
    int m_Nl;
  };
};
#endif /* _OPTIMAL_CONTROLLER_SOLVER_H_ */

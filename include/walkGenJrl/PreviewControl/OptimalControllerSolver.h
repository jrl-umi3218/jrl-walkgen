/*! \file OptimalControllerSolver.h
  \brief Object to compute the weights of the preview control
    @ingroup previewcontrol

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

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

namespace PatternGeneratorJRL
{
  /*! @ingroup previewcontrol
      \brief  This class computes the gains for preview control for a given 
     discrete system. 
    The discrete system is defined by three matrix A, b, c
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

    static const unsigned int MODE_WITHOUT_INITIALPOS=1;
    static const unsigned int MODE_WITH_INITIALPOS=0;

    /*! A constructor */
    OptimalControllerSolver(MAL_MATRIX(&A,double), 
			    MAL_MATRIX(&b,double), 
			    MAL_MATRIX(&c,double),
			    double Q, double R, 
			    unsigned int Nl);

    /*! Destructor. */
    ~OptimalControllerSolver();

    /*! Compute the weights 
      Following the mode, there is a the inclusion 
      of the P matrix inside the weights. 
     */
    void ComputeWeights(unsigned int Mode);
    
    /*! Display the weights */
    void DisplayWeights();

    bool GeneralizedSchur(MAL_MATRIX(&A,double),
			  MAL_MATRIX(&B,double),
			  MAL_VECTOR(&alphar,double),
			  MAL_VECTOR(&alphai,double),
			  MAL_VECTOR(&beta,double),
			  MAL_MATRIX(&L,double),
			  MAL_MATRIX(&R,double));

    /*! To take matrix F aka the weights of the preview window . */
    void GetF(MAL_MATRIX(& LF,double) );
    
    /*! To take matrix K, aka the weight of the other part of the command */
    void GetK(MAL_MATRIX(& LK,double) );

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

  /*! 
    \defgroup pageexampleoptimalweights Computing optimal weights for the preview control.
    \ingroup codesourceexamples						

  \dontinclude TestRiccatiEquation.cpp
  First you need to include the header file:
  \skipline OptimalControllerSolver.h
  In the following we will use a pointer towards an instance
  of class OptimalControllerSolver.
  \skip main
  \until anOCS
  You have then to build the linear discrete system of the linear
  inverted pendulum such that:
  \f{eqnarray*}
  {\bf x}_{k+1} &=& {\bf A}{\bf x}_k + {\bf B} u_{k} \\
  p_k &=& {\bf C} {\bf x}_{k} \\
  \f}

  For this we need to declare the associated matrices:
  \skip Declare the linear system 
  \until lF;
  and the weights of the function to be minimized:
  \skip double Q
  \until double T
  
  Then we have to initialize the discretized linear system with
  
  \f{eqnarray*}
  {\bf A} & \equiv &
  \left[
  \begin{matrix}
  1 & T & T^2/2 \\
  0 & 1 & T \\
  0 & 0 & 1
  \end{matrix}
  \right] \\
 
  {\bf B} & \equiv &
  \left[
  \begin{matrix}
  T^3/6 \\
  T^2/2 \\
  T
  \end{matrix}
  \right] \\

  {\bf C} & \equiv &
  \left[
  1 \; 0 \; \frac{-z_c}{g}
  \right]
  \f}
  \skip Build the initial
  \until -0.814
  
  We then have to initialize the weights of the
  index function:
  \f[
  J = \sum^{NL}_{j=1} \{ Q(p^{ref}_j -p_j)^2 + Ru_j^2 \}
  \f]

  \skip Q
  \until Nl

  To suppress the problem of the initial CoM position,
  we can reformulate the discrete problem with:
  
  \f{eqnarray*}

  \widetilde{\bf A} &\equiv &
  \left[
  \begin{matrix}
  1 & {\bf cA} \\
  {\bf 0} & {\bf A} \\
  \end{matrix}
  \right] \\
  \tilde{\bf b} & \equiv &
  \left[
  \begin{matrix}
  {\bf cb} \\
  {\bf c}
  \end{matrix}
  \right] \\
  \tilde{\bf c} & 
  \equiv & [ 1 \; 0 \; 0 \; 0] \\

  \f}
  
  Then the subsequent code performs this operation and displays
  the associated matrices:
  \skip Build the derivated 
  \until cx:

  To create the instance of the object solving the Riccati Equation:
  \skipline PatternGeneratorJRL::OptimalControllerSolver

  The computation of the weights is done by calling ComputeWeights().
  There is only one parameter to specify, but it is important
  as the weights are computed differently according to this parameter.
  If you use the mode without initial position please uses MODE_WITH_INITIALPOS.
  \skipline PatternGeneratorJRL::OptimalControllerSolver

  To display the weights in the standard output
  \skipline DisplayWeights
  
  It is possible to retrieve the weights in a vector:
  \skipline GetF
  To compute the same weight with a specific initial position and the 
  original linear system you can use :
  \skip new PatternGeneratorJRL::OptimalControllerSolver
  \until MODE_WITH_INITIALPOS
  
  */
};
#endif /* _OPTIMAL_CONTROLLER_SOLVER_H_ */

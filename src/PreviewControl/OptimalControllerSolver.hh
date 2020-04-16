/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 *
 * Florent Lamiraux
 * Mathieu Poirier
 * Olivier Stasse
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
/*! \file OptimalControllerSolver.h
  \brief Object to compute the weights of the preview control
  @ingroup previewcontrol
*/
#ifndef _OPTIMAL_CONTROLLER_SOLVER_H_
#define _OPTIMAL_CONTROLLER_SOLVER_H_

#include <Eigen/Dense>

namespace PatternGeneratorJRL {
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

  the solution is then:
  \f{eqnarray*}
  u_j = - {\bf K}x_k + [ f_1, f_2, \cdots, f_N]
  \left[
  \begin{matrix}
  p^{ref}_{k+1} \\
  \vdots \\
  p^{ref}_{k+N}
  \end{matrix}
  \right]
  \f}

  \f{eqnarray*}
  {\bf K} & \equiv & (R + {\bf b}^T{\bf Pb})^{-1}{\bf b}^T{\bf PA} \\
  K_p(i) & \equiv & (R + {\bf b}^T{\bf Pb})^{-1}{\bf b}^T({\bf A}-
  {\bf bK})^{T*(i-1)}{\bf c}^TQ \             \
  \f}
  where \f$ {\bf P} \f$ is solution of the following Riccati equation:
  \f[
  {\bf P} = {\bf A}^T {\bf PA} + {\bf c}^TQ{\bf c} -
  {\bf A}^T{\bf Pb}(R + {\bf b}^T{\bf Pb})^{-1}{\bf b}^T{\bf PA}
  \f]


  The resolution of the Riccati equation is taken from \ref Laub1979, and is
  based on a Schur form .

  To suppress the problem of the initial CoM position,
  we can reformulate the discrete problem by posing the following:
  \f{eqnarray*}
  \begin{matrix}
  {\bf x}^*_{k+1} &= \widetilde{\bf A} {\bf x}^*_{k} +
  \widetilde{\bf b}\Delta u_k \               \
  p_k &= \widetilde{\bf c}{\bf x}^*_{k}
  \end{matrix}
  \f}
  with
  \f{eqnarray*}
  \Delta u_k \equiv u_k - u_{k-1} & \Delta {\bf x}_k \equiv {\bf x}_k
  - {\bf x}_{k-1}\                            \
  {\bf x}_k \equiv \left[
  \begin{matrix}
  p_k\\
  \Delta {\bf x}_k
  \end{matrix}
  \right]
  \f}
  The augmented system is then
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
  with the following cost function:
  \f[
  J = \sum^{\infty}_{j=1} \{ Q(p^{ref}_j -p_j)^2 + R \Delta u_j^2 \}
  \f]

  the solution is then:
  \f{eqnarray*}
  u_j = - K_1 \sum_{i=0}^k e(i) - {\bf K}_2 x(k) - \sum_{j=1}^{N_L}
  K_p(j)p^{ref}_j(k+j)
  \f}

  where \f{eqnarray*}
  \left[ \begin{matrix} K_1 \\ {\bf K}_2 \\ \end{matrix} \right]=
  \widetilde{\bf K}
  \f}


  \anchor Laub1979
  Alan J. Laub A Schur method for solving Algebraic Riccati Equations,
  IEEE Transaction on Automatic Control,
  Vol AC-24, No.6 December 1979
*/
class OptimalControllerSolver {
public:
  static const unsigned int MODE_WITHOUT_INITIALPOS = 1;
  static const unsigned int MODE_WITH_INITIALPOS = 0;

  /*! A constructor */
  OptimalControllerSolver(Eigen::MatrixXd &A, Eigen::MatrixXd &b,
                          Eigen::MatrixXd &c, double Q, double R,
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

  /*! To take matrix F aka the weights of the preview window . */
  void GetF(Eigen::MatrixXd &LF);

  /*! To take matrix K, aka the weight of the other part of the command */
  void GetK(Eigen::MatrixXd &LK);

protected:
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      MatrixRXd;
  /*! The matrices needed for the dynamical system such as
    \f{eqnarray*}

    {\bf x}_{k+1} & =& {\bf A} x_k + {\bf b} u_k \\
    p_k &=& {\bf cx}_k\\
    \f}

  */
  MatrixRXd m_A;
  MatrixRXd m_b;
  MatrixRXd m_c;

  /*! The coefficent of the index criteria:
    \f[
    J = \sum^{\infty}_{j=1} \{ Q(p^{ref}_j -p_j)^2 + Ru_j^2 \}
    \f]
  */
  double m_Q, m_R;

  /*! The weights themselves */
  MatrixRXd m_K;
  MatrixRXd m_F;

  /*! The size of the window for the preview */
  int m_Nl;

  bool GeneralizedSchur(MatrixRXd &A, MatrixRXd &B, Eigen::VectorXd &alphar,
                        Eigen::VectorXd &alphai, Eigen::VectorXd &beta,
                        MatrixRXd &L, MatrixRXd &R);
};

/*!
  \defgroup pageexampleoptimalweights Computing optimal weights
  for the preview control.
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
  we can reformulate the discrete problem by posing the following:
  \f{eqnarray*}
  {\bf x}^*_{k+1} = \widetilde{\bf A} {\bf x}^*_{k} + \widetilde{\bf b}
  \Delta u_k
  p_k = \widetilde{\bf c}{\bf x}^*_{k}
  \f}
  with
  \f{eqnarray*
  \Delta u_k \equiv u_k - u_{k-1} & \Delta {\bf x}_k \equiv {\bf x}_k -
  {\bf x}_{k-1}
  {\bf x}_k \equiv \left[
  \begin{matrix}
  p_k\\
  \Delta {\bf x}_k
  \end{matrix}
  \right]
  \f}

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
  If you use the mode without initial position please uses
  MODE_WITH_INITIALPOS.
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
} // namespace PatternGeneratorJRL
#endif /* _OPTIMAL_CONTROLLER_SOLVER_H_ */

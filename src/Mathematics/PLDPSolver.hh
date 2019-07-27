/*
 * Copyright 2009, 2010,
 *
 * Francois   Keith
 * Nicolas    Mansard
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
/*! \file PLDPSolver.cpp
  \brief This file implements the optimized QP solver proposed by Dimitar 2009.
  On the Application of Linear Model Predictive Control
  for Walking Pattern Generation in the Presence of Strong Disturbances
  D. Dimitrov and P.-B. Wieber and H. Diedam and O. Stasse
*/

#ifndef _PLDP_SOLVER_H_

#include <vector>
#include <Mathematics/OptCholesky.hh>

namespace Optimization
{
  namespace Solver
  {
    /*! This class implements a two stage strategy to solve the 
      following optimal problem:
     */
    class PLDPSolver
    {
    public:
      /*! \brief Constructor */
      PLDPSolver(unsigned int CardU,
                 double * iPu,
                 double * Px,
                 double * Pu,
                 double * iLQ);

      /*! \brief Destructor */
      ~PLDPSolver();

      /*! \brief Solve the optimization problem
       */
      int SolveProblem(double *CstPartOfTheCostFunction,
                       unsigned int NbOfConstraints,
                       double *LinearPartOfConstraints,
                       double *CstPartOfConstraints,
                       double *ZMPRef,
                       double *XkYk,
                       double *X,
                       std::vector<int> &SimilarConstraint,
                       unsigned int NumberOfRemovedConstraints,
                       bool StartingSequence);
    protected:

      /*! \name Initial solution methods related
        @{
      */
      /*! Compute the initial solution */
      int ComputeInitialSolution(double *ZMPRef,
                                 double *XkYk,
                                 bool StartingSequence);

      /*! Precompite iPuPx */
      int PrecomputeiPuPx();
      /*! @} */


      /*! Initialize the internal variables of
        the class. */
      void InitializeSolver();

      /*! Allocate memory for solver. */
      void AllocateMemoryForSolver();

      /*! \name Projected descent direction methods related
        @{
      */
      /*! \brief Compute Projected descent direction. */
      int ComputeProjectedDescentDirection();

      /*! \brief Forward substitution.
        First Phase
        EE^t v2 = v1 <-> LL^t v2 = v1
        Now solving L y = v1

      */
      int ForwardSubstitution();

      /*! \brief Compute v2 q (14b) in Dimitrov 2009.
        Second phase a
        Now solving
        LL^t v2 = v1 <-> L y = v1 with L^t v2 = y
        y solved with first phase.
        So now we are looking for v2.
      */
      int BackwardSubstitution();

      /*! @} */

      /*! Detecting violated constraints */
      double ComputeAlpha(vector<unsigned int> &NewActivatedConstraints,
                          vector<int> &SimilarConstraint);

      /*! Store the current ZMP solution for hot start purposes. */
      void StoreCurrentZMPSolution(double *XkYk);

      /*! Write current ZMP ref trajectory associated with
        current value of m_Vk. */
      void WriteCurrentZMPSolution(string filename,
                                   double *XkYk);

      /*! \name Methods related to a limited amount of computational time
        @{ */

    private:

      /*! \brief Store Pu */
      double *m_Pu;

      /*! Store the inverse of Pu. */
      double * m_iPu;

      /*! Store Px. */
      double * m_Px;

      /*! Store the inverse of Pu. */
      double * m_iPuPx;

      /*! Store Vk */
      double *m_Vk;

      /*! Store the constart part of the cost function. */
      double *m_CstPartOfCostFunction;

      /*! Store the unconstrained descent direction. */
      double *m_UnconstrainedDescentDirection;

      /*! Store the cholesky decomposition of EE^t. */
      double * m_L;

      /*! Store the inverse of the cholesky decomposition. */
      double *m_iL;

      /*! Store iLQ for debuggin purposes. */
      double *m_iLQ;

      /*! Store the projector of descent  */
      double *m_d;

      /*! Store some temporary variables  */
      double *m_v1, *m_v2, *m_y;

      /*! Store some temporary variables  */
      double *m_tmp1, *m_tmp2;

      /*! Store the linear part of the constraints. */
      double * m_A;

      /*! Store the cst part of the constraints. */
      double * m_b;


      /*! Store if the A*Vk values has been computed  */
      bool * m_ConstraintsValueComputed;


      /*! Store the maximum number of Constraints.
        It is also the dimension of L in its maximal storage
        form. */
      unsigned int m_NbMaxOfConstraints;

      /*! Store the current number of Constraints of matrix A. */
      unsigned int m_NbOfConstraints;

      /*! Store the size of the control vector. */
      unsigned int m_CardV;


      /*! \name Debugging fields
        @{
      */
      /*! Level of verbosity */
      unsigned int m_DebugMode;

      /*! Number of iterations */
      int m_ItNb;
      /*! @} */

      /*! Cholesky decomposition optimized for QP solving
        ( specifically this one). */
      PatternGeneratorJRL::OptCholesky * m_OptCholesky;

      /*! List of activated constraints. */
      vector<unsigned int> m_ActivatedConstraints;

      /*! List of previously activated constraints. */
      vector<unsigned int> m_PreviouslyActivatedConstraints;

      /*! Boolean to perform a hotstart */
      bool m_HotStart;

      /*! Store the current ZMP solution. */
      double * m_PreviousZMPSolution;

      /*! Double internal time. */
      double m_InternalTime;

      /*! Tolerance for zero value */
      double m_tol;

      /*! \name Data related to a limited amount of computational time
        @{
      */
      /*! Is the algorithm limited in time. */
      bool m_LimitedComputationTime;

      /*! Amount of limited */
      double m_AmountOfLimitedComputationTime;

      /*! @} */
    };
  }
}
#endif /* _PLDPSOLVER_H_*/

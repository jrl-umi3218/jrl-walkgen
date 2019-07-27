/*
 * Copyright 2010,
 *
 * Medhi    Benallegue
 * Andrei   Herdt
 * Francois Keith
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
#ifndef _PROBLEM_VEL_REF_H_
#define _PROBLEM_VEL_REF_H_




namespace PatternGeneratorJRL
{

  /*! \brief Final optimization problem to handle velocity reference.
    This object store a standardized optimization quadratic problem.
  */
  struct ProblemVelRef_s
  {
    int m, me, mmax, n, nmax, mnn;
    double *Q, *D, *DU, *DS, *XL, *XU, *X, *NewX, *U, *war;//For COM
    int *iwar;
    int iout, ifail, iprint, lwar, liwar;
    double Eps;

    /// \brief Initialize by default an empty problem.
    ProblemVelRef_s();

    /// \brief Release the memory at the end only.
    ~ProblemVelRef_s();

    /// \brief Set the dimensions of the problem.
    /// This method has an internal logic to
    /// allocate the memory. It is done only
    /// when the problem gets bigger. When it shrinks
    /// down the memory is kept to avoid overhead.
    void setDimensions(int NbOfConstraints,
                       int NbOfEqConstraints,
                       int StepNumber,
                       int QP_N);

    /// \brief Dump on disk a problem.
    void dumpProblem(const char *filename);
    void dumpProblem(std::ostream &);

    /// \brief Dump on disk a matrix defined by type.
    void dumpMatrix(const char *filename,int type);
    void dumpMatrix(std::ostream &, int type);
    /// \brief Dump on disk a vector defined by type.
    void dumpVector(const char *filename,int type);
    void dumpVector(std::ostream &,int type);

    /// \brief Initialize the problem
    void initializeProblem();

    const static int MATRIX_Q=0;
    const static int MATRIX_DU=1;
    const static int VECTOR_D=2;
    const static int VECTOR_DL=3;
    const static int VECTOR_XL=4;
    const static int VECTOR_XU=5;
    const static int VECTOR_DS=6;

  protected:

    /// The method doing the real job of releasing the memory.
    void ReleaseMemory();

    /// The method allocating the memory.
    /// Called when setting the dimensions of the problem.
    void AllocateMemory();

  private:
    /// Previous set of step number considered.
    int m_stepNumber;

    /// Previous size of the preview.
    int m_QP_N;

  };
  typedef struct ProblemVelRef_s Problem;
}
#endif /* _PROBLEM_VEL_REF_H_ */

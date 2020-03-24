/*
 * Copyright 2009, 2010,
 *
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
/*! \file TestOptCholesky.cpp
  \brief Example to compute a cholesky decomposition using
  an optimized implementation for QP solving.
*/

#include <stdlib.h>

#include <fstream>
#include <iostream>

#include <math.h>

#include "Mathematics/OptCholesky.hh"

using namespace std;

void MatrixMatrixT(double *A, double *AAT, int lNbOfConstraints, int lCardU) {
  for (int i = 0; i < lNbOfConstraints; i++) {
    for (int j = 0; j < lNbOfConstraints; j++) {
      AAT[i * lNbOfConstraints + j] = 0.0;
      for (int k = 0; k < lCardU; k++)
        AAT[i * lNbOfConstraints + j] += A[i * lCardU + k] * A[j * lCardU + k];
    }
  }
}

void DisplayMatrix(double *A, int rows, int columns, string name, int format) {
  cout.precision(5);
  cout.setf(ios::fixed, ios::floatfield);
  cout << name << "=[";

  for (int i = 0; i < rows; i++) {
    cout << "[";
    for (int j = 0; j < columns; j++) {
      cout << A[i * rows + j] << " ";
    }
    cout << "];";
    if (format == 0) // Human readable.
      cout << endl;
  }
  cout << "]" << endl;
}
double CheckCholeskyDecomposition(double *A, double *L, int n) {
  double *LLT = new double[n * n];
  MatrixMatrixT(L, LLT, n, n);

  double distance = 0.0;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      double r = A[i * n + j] - LLT[i * n + j];
      distance += r * r;
    }
  }

  delete LLT;
  distance = sqrt(distance);

  return distance;
}

int main() {
  PatternGeneratorJRL::OptCholesky *anOptCholesky;

  unsigned int lNbOfConstraints = 12;
  unsigned int lCardU = 15;
  unsigned int verbose = 2;

  /* Declare the linear system */
  double *A = new double[lNbOfConstraints * lCardU];
  double *L = new double[lNbOfConstraints * lNbOfConstraints];
  double *iL = new double[lNbOfConstraints * lNbOfConstraints];

  /* Create the object for optimized Cholesky computation */
  anOptCholesky = new PatternGeneratorJRL::OptCholesky(
      lNbOfConstraints, lCardU, PatternGeneratorJRL::OptCholesky::MODE_NORMAL);

  /* Build the test matrix */
  srand(0);
  for (unsigned int i = 0; i < lNbOfConstraints; i++) {
    for (unsigned int j = 0; j < lCardU; j++) {
      A[i * lCardU + j] = (double)rand() / (double)RAND_MAX;
    }
  }
  if (verbose > 1)
    DisplayMatrix(A, lNbOfConstraints, lCardU, string("A"), 0);

  anOptCholesky->SetA(A, PatternGeneratorJRL::OptCholesky::MODE_NORMAL);
  anOptCholesky->SetL(L);
  anOptCholesky->SetiL(iL);

  for (unsigned int i = 0; i < lNbOfConstraints; i++)
    anOptCholesky->AddActiveConstraint(i);

  if (verbose > 1)
    DisplayMatrix(L, lNbOfConstraints, lNbOfConstraints, string("L"), 0);

  double *AAT = new double[lNbOfConstraints * lNbOfConstraints];

  MatrixMatrixT(A, AAT, lNbOfConstraints, lCardU);

  if (verbose > 1)
    DisplayMatrix(AAT, lNbOfConstraints, lNbOfConstraints, string("AAT"), 0);

  int return_value = 0;

  double r = CheckCholeskyDecomposition(AAT, L, lNbOfConstraints);
  if (r > 1e-6) {
    cout << "Optimized Cholesky decomposition for QP pb:" << r << endl;
    return_value = -1;
  }

  delete anOptCholesky;

  /* Create the object for normal Cholesky computation */
  anOptCholesky = new PatternGeneratorJRL::OptCholesky(
      lNbOfConstraints, lNbOfConstraints,
      PatternGeneratorJRL::OptCholesky::MODE_NORMAL);

  anOptCholesky->SetA(AAT, PatternGeneratorJRL::OptCholesky::MODE_NORMAL);
  anOptCholesky->SetL(L);
  anOptCholesky->SetiL(iL);

  anOptCholesky->ComputeNormalCholeskyOnANormal();

  r = CheckCholeskyDecomposition(AAT, L, lNbOfConstraints);
  if (verbose > 1)
    DisplayMatrix(L, lNbOfConstraints, lNbOfConstraints, string("L"), 0);

  if (r > 1e-6) {
    cout << "Normal Cholesky decomposition pb:" << r << endl;
    return_value = -1;
  }

  anOptCholesky->ComputeInverseCholeskyNormal(1);
  if (verbose > 1)
    DisplayMatrix(iL, lNbOfConstraints, lNbOfConstraints, string("iL"), 0);

  delete anOptCholesky;
  delete[] L;
  delete[] A;

  if (return_value == -1) {
    cout << "Failed test" << endl;
  } else {
    cout << "Passed test" << endl;
  }

  return return_value;
}

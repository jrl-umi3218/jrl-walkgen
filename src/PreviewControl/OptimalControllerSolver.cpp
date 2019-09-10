/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 *
 * Florent Lamiraux
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
/** @doc Object to compute the weights of the preview control
    @ingroup previewcontrol
    @endgroup
*/

#include <PreviewControl/OptimalControllerSolver.hh>
#include <iostream>

#include <Debug.hh>

using namespace PatternGeneratorJRL;
using namespace std;

typedef long int logical;
typedef double doublereal;
typedef logical (*L_fp)(...);
typedef int integer;
extern "C" {
extern doublereal dlapy2_(doublereal *, doublereal *);
extern double dlamch_(char *);
extern /* Subroutine */
    int
    dgges_(char *, char *, char *, L_fp, integer *, doublereal *, integer *,
           doublereal *, integer *, integer *, doublereal *, doublereal *,
           doublereal *, doublereal *, integer *, doublereal *, integer *,
           doublereal *, integer *, logical *, integer *);
}

logical sb02ox(double *_alphar, double *_alphai, double *_beta) {
  logical r;
  double w = dlapy2_(_alphar, _alphai);
  double w2 = fabs(*_beta);
  r = w < w2;
  return r;
}

/*
  Inspired from scilab 4.0...


  To select the stable generalized eigenvalues for continuous-time.

  alphar: double precision,
  the real part of the numerator of the current eigenvalue considered.

  alphai: double precision,
  The imaginary part of the numerator of the current
  eigenvalue considered.

  beta: double precision
  The (real) denominator of the current eigenvalue
  considered. It is assumed that beta <> 0 (regular case).

  description:

  The function value SB02OW is set to true for a stable eigenvalue
  and to false, otherwise.
*/
logical sb02ow(double *_alphar, double * /* _alphai */, double *_beta) {
  char lp[2] = "p";
  logical r = ((*_alphar < 0.0) && (*_beta > 0.0)) ||

              (((*_alphar > 0.0) && (*_beta < 0.0)) &&
               ((fabs(*_beta) > fabs(*_alphar) * dlamch_(lp))));
  ;
  return r;
}

OptimalControllerSolver::OptimalControllerSolver(Eigen::MatrixXd &A,
                                                 Eigen::MatrixXd &b,
                                                 Eigen::MatrixXd &c, double Q,
                                                 double R, unsigned int Nl) {
  m_A.resize(A.rows(), A.cols());
  for (unsigned int i = 0; i < A.rows(); i++)
    for (unsigned int j = 0; j < A.cols(); j++)
      m_A(i, j) = A(i, j);

  m_b.resize(b.rows(), b.cols());
  for (unsigned int i = 0; i < b.rows(); i++)
    for (unsigned int j = 0; j < b.cols(); j++)
      m_b(i, j) = b(i, j);

  m_c.resize(c.rows(), c.cols());
  for (unsigned int i = 0; i < c.rows(); i++)
    for (unsigned int j = 0; j < c.cols(); j++)
      m_c(i, j) = c(i, j);

  m_Q = Q;
  m_R = R;
  m_Nl = Nl;
}

OptimalControllerSolver::~OptimalControllerSolver() {}

bool OptimalControllerSolver::GeneralizedSchur(MatrixRXd &A, MatrixRXd &B,
                                               Eigen::VectorXd &alphar,
                                               Eigen::VectorXd &alphai,
                                               Eigen::VectorXd &beta,
                                               MatrixRXd &L, MatrixRXd &R) {
  ODEBUG("A:" << A);
  ODEBUG("B:" << A);
  int n = (int)A.rows();

  alphar.resize(n);
  alphar.setZero();
  alphai.resize(n);
  alphai.setZero();
  beta.resize(n);
  beta.setZero();
  L.resize(n, n);
  L.setZero();
  R.resize(n, n);
  R.setZero();

  int sdim = 0;
  int lwork = 1000 + (8 * (int)n + 16);
  double *work = new double[lwork]; // std::vector<double> work(lwork);
  for (int i = 0; i < lwork; i++)
    work[i] = 0.0;
  int info = 0;
  logical *bwork = new logical[2 * n];
  for (int i = 0; i < 2 * n; i++)
    bwork[i] = 0;

  char lV[2] = "V";
  char lS[2] = "S";
  for (int i = 0; i < 2 * n; i++)
    bwork[i] = 0;
  A.transposeInPlace();
  B.transposeInPlace();
  ODEBUG("A:" << A);
  ODEBUG("A:" << B);

  dgges_(lV, lV, lS, (logical(*)(...))sb02ox, &n, A.data(), &n, B.data(), &n,
         &sdim, alphar.data(), alphai.data(), beta.data(), L.data(), &n,
         R.data(), &n, &work[0], &lwork, bwork, &info);

  A.transposeInPlace();
  B.transposeInPlace();
  L.transposeInPlace();
  R.transposeInPlace();

  delete[] work;
  delete[] bwork;
  if (info != 0) {
    std::cout << ": info = " << info << " n = " << n << std::endl;
    return false;
  } else {
    return true;
  }
}

void OptimalControllerSolver::ComputeWeights(unsigned int Mode) {
  // Compute the symplectic matrix related
  // to the discrete dynamical system given in parameters.

  // H is the 2n x 2n matrix
  // H = [[ A , 0 ]
  //      [-c'Qc E]]
  //  ODEBUG("m_A:" << m_A);
  // And each sub-matrix is n x n matrix.
  MatrixRXd H;
  MatrixRXd tm_b;

  // Store the transpose of m_b;
  tm_b = m_b.transpose();

  ODEBUG(" ROWS(A): " << m_A.rows() << " COLS(A): " << m_A.cols());
  H.resize(2 * m_A.rows(), 2 * m_A.cols());

  H.setIdentity();

  // Build the upper left sub-block of H
  Eigen::Index n = m_A.rows();

  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++)
      H(i, j) = m_A(i, j);

  MatrixRXd H21;
  H21 = m_c.transpose();
  ODEBUG("H21 (1):" << H21);
  H21 = H21 * m_Q;
  ODEBUG("H21 (2):" << H21);
  H21 = H21 * m_c;
  ODEBUG("H21 (3):" << H21);
  H21 = -H21;

  ODEBUG("H21:" << H21);
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++)
      H(i + n, j) = H21(i, j);

  ODEBUG("H:" << endl << H);
  MatrixRXd E(2 * n, 2 * n);
  E.setIdentity();

  MatrixRXd G;
  G = (m_b * (1 / m_R)) * tm_b;

  MatrixRXd At;
  At = m_A.transpose();
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++) {
      E(i, j + n) = G(i, j);
      E(i + n, j + n) = At(i, j);
    }

  ODEBUG("E:" << endl << E);
  // Computes S the Schur form of Laub_Z
  MatrixRXd ZH(2 * n, 2 * n); // The matrix of schur vectors
  MatrixRXd ZE(2 * n, 2 * n); // The matrix of Schur vectors.
  Eigen::VectorXd WR(2 * n);
  Eigen::VectorXd WI(2 * n);
  // The eigenvalues ( a matrix to handle complex eigenvalues).
  Eigen::VectorXd GS(2 * n);

  if (!GeneralizedSchur(H, E, WR, WI, GS, ZH, ZE)) {
    std::cerr << "Something is wrong with the weights "
              << "for the preview control !" << std::endl;
  }

  ODEBUG("Hx:" << H);
  ODEBUG("Ex:" << E);
  ODEBUG("ZH:" << ZH);
  ODEBUG("ZE:" << ZE);

  // Computes P the solution of the Riccati equation.
  MatrixRXd P;
  MatrixRXd Z11(n, n);
  MatrixRXd Z21(n, n);

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      Z11(i, j) = ZE(i, j);
      Z21(i, j) = ZE(i + n, j);
    }
  }
  ODEBUG("Z11:" << endl << Z11 << endl << "Z21:" << endl << Z21);

  MatrixRXd iZ11;
  iZ11 = Z11.inverse();
  P = Z21 * iZ11;

  ODEBUG("P: " << endl << P);

  // Compute the weights.
  MatrixRXd r;

  double la;
  r = tm_b;           // b^T
  r = r * P;          // b^T P
  r = r * m_b;        // b^T P b
  la = m_R + r(0, 0); // R + b^T P b
  la = 1 / la;

  // Computes the weights for the accumulated difference
  // and the speed.
  m_K = P * m_A;
  m_K = tm_b * m_K;
  m_K = m_K * la;

  ODEBUG("K: " << endl << m_K);

  // Computes the weights for the future.
  MatrixRXd PreMatrix;
  MatrixRXd Recursive;
  MatrixRXd BaseOfRecursion;
  MatrixRXd PostMatrix;
  MatrixRXd Intermediate;

  PreMatrix = la * tm_b;
  BaseOfRecursion = m_A - m_b * m_K;
  BaseOfRecursion.transposeInPlace();

  PostMatrix = m_c.transpose();
  PostMatrix = PostMatrix * m_Q;
  if (Mode == MODE_WITHOUT_INITIALPOS)
    PostMatrix = P * PostMatrix;

  Recursive = PostMatrix;

  ODEBUG("BaseOfRecursion:" << endl
                            << BaseOfRecursion << endl
                            << "Recursive:" << endl
                            << Recursive << endl
                            << "PreMatrix:" << endl
                            << PreMatrix << endl
                            << "PostMatrix:" << endl
                            << PostMatrix << endl);

  m_F.resize(m_Nl, 1);
  for (int k = 0; k < m_Nl; k++) {
    Intermediate = PreMatrix * Recursive;
    m_F(k, 0) = Intermediate(0, 0);
    Recursive = BaseOfRecursion * Recursive;
  }
}

void OptimalControllerSolver::DisplayWeights() {
  std::cout << "K:" << m_K << std::endl;
  std::cout << "F:" << m_F << std::endl;
}

void OptimalControllerSolver::GetF(Eigen::MatrixXd &lF) { lF = m_F; }

void OptimalControllerSolver::GetK(Eigen::MatrixXd &lK) { lK = m_K; }

/*
 * Copyright 2010,
 *
 * Andrei Herdt
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
/* This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of steps following a QP
   formulation and a new QP solver as proposed by Dimitrov ICRA 2009. */

#include "portability/gettimeofday.hh"

#ifdef WIN32
#include <Windows.h>
#endif /* WIN32 */

#include <time.h>

#include <fstream>
#include <iostream>

#include <Mathematics/qld.hh>
#include <ZMPRefTrajectoryGeneration/ZMPConstrainedQPFastFormulation.hh>

#include <Debug.hh>
using namespace std;
using namespace PatternGeneratorJRL;

ZMPConstrainedQPFastFormulation::ZMPConstrainedQPFastFormulation(
    SimplePluginManager *lSPM, string DataFile, PinocchioRobot *aPR)
    : ZMPRefTrajectoryGeneration(lSPM) {
  m_Q = 0;
  m_Pu = 0;
  m_FullDebug = 0;
  m_FastFormulationMode = PLDP;

  /*! Getting the ZMP reference from Kajita's heuristic. */
  m_ZMPD = new ZMPDiscretization(lSPM, DataFile, aPR);

  /*! For simulating the linearized inverted pendulum in 2D. */
  m_2DLIPM = new LinearizedInvertedPendulum2D();

  /*! For computing the stability constraints from the feet positions. */
  m_FCALS = new FootConstraintsAsLinearSystem(lSPM, aPR);

  // Register method to handle
  string aMethodName[1] = {":setdimitrovconstraint"};

  for (int i = 0; i < 1; i++) {
    if (!RegisterMethod(aMethodName[i])) {
      std::cerr << "Unable to register " << aMethodName << std::endl;
    }
  }

  m_ConstraintOnX = 0.04;
  m_ConstraintOnY = 0.04;

  //  m_QP_T = 0.02;
  m_QP_T = 0.1;
  m_QP_N = 16;

  m_SamplingPeriod = 0.005;

  m_ComHeight = 0.80;

  /* Initialize  the 2D LIPM */
  m_2DLIPM->SetSimulationControlPeriod(m_QP_T);
  m_2DLIPM->SetRobotControlPeriod(m_SamplingPeriod);
  m_2DLIPM->SetComHeight(m_ComHeight);
  m_2DLIPM->InitializeSystem();

  m_Alpha = 200.0;
  m_Beta = 1000.0;

  InitConstants();

  // PLDP Solver needs iPu and Px.

  m_SimilarConstraints.resize(8 * m_QP_N);

  if (m_FastFormulationMode == PLDP)
    m_PLDPSolver = new Optimization::Solver::PLDPSolver(
        m_QP_N, &m_iPu(0), &m_Px(0), m_Pu, &m_iLQ(0));
  else
    m_PLDPSolver = 0;

  if (m_FastFormulationMode == QLDANDLQ) {
    RESETDEBUG6("dtQLD.dat");
    RESETDEBUG6("InfosQLD.dat");
    RESETDEBUG6("Check2DLIPM_QLDANDLQ.dat");
  }

  if (m_FastFormulationMode == PLDP) {
    RESETDEBUG6("dtPLDP.dat");
    RESETDEBUG6("Check2DLIPM_PLDP.dat");
  }
}

ZMPConstrainedQPFastFormulation::~ZMPConstrainedQPFastFormulation() {

  if (m_ZMPD != 0)
    delete m_ZMPD;

  if (m_2DLIPM != 0)
    delete m_2DLIPM;

  if (m_FCALS != 0)
    delete m_FCALS;

  if (m_Q != 0)
    delete[] m_Q;

  if (m_PLDPSolver != 0)
    delete m_PLDPSolver;

  if (m_Pu != 0)
    delete[] m_Pu;
}

void ZMPConstrainedQPFastFormulation::SetPreviewControl(PreviewControl *) {
  // m_ZMPD->SetPreviewControl(aPC);
}

int ZMPConstrainedQPFastFormulation::InitializeMatrixPbConstants() {
  m_PPu.resize(2 * m_QP_N, 2 * m_QP_N);
  m_VPu.resize(2 * m_QP_N, 2 * m_QP_N);
  m_PPx.resize(2 * m_QP_N, 6);
  m_VPx.resize(2 * m_QP_N, 6);

  for (unsigned int i = 0; i < m_QP_N; i++) {
    // Compute VPx and PPx
    m_VPx(i, 0) = 0.0;
    m_VPx(i, 1) = 1.0;
    m_VPx(i, 2) = (i + 1) * m_QP_T;
    m_VPx(i, 3) = 0.0;
    m_VPx(i, 4) = 0.0;
    m_VPx(i, 5) = 0.0;
    m_VPx(i + m_QP_N, 0) = 0.0;
    m_VPx(i + m_QP_N, 1) = 0.0;
    m_VPx(i + m_QP_N, 2) = 0.0;
    m_VPx(i + m_QP_N, 3) = 0.0;
    m_VPx(i + m_QP_N, 4) = 1.0;
    m_VPx(i + m_QP_N, 5) = (i + 1) * m_QP_T;

    m_PPx(i, 0) = 1.0;
    m_PPx(i, 1) = (i + 1) * m_QP_T;
    m_PPx(i, 2) = (i + 1) * (i + 1) * m_QP_T * m_QP_T * 0.5;
    m_PPx(i, 3) = 0.0;
    m_PPx(i, 4) = 0;
    m_PPx(i, 5) = 0.;
    m_PPx(i + m_QP_N, 0) = 0.0;
    m_PPx(i + m_QP_N, 1) = 0.0;
    m_PPx(i + m_QP_N, 2) = 0.0;
    m_PPx(i + m_QP_N, 3) = 1.0;
    m_PPx(i + m_QP_N, 4) = (i + 1) * m_QP_T;
    m_PPx(i + m_QP_N, 5) = (i + 1) * (i + 1) * m_QP_T * m_QP_T * 0.5;

    for (unsigned int j = 0; j < m_QP_N; j++) {
      m_PPu(i, j) = 0;

      if (j <= i) {

        m_VPu(i, j) = (2 * (i - j) + 1) * m_QP_T * m_QP_T * 0.5;
        m_VPu(i + m_QP_N, j + m_QP_N) =
            (2 * (i - j) + 1) * m_QP_T * m_QP_T * 0.5;
        m_VPu(i, j + m_QP_N) = 0.0;
        m_VPu(i + m_QP_N, j) = 0.0;

        m_PPu(i, j) = (1 + 3 * (i - j) + 3 * (i - j) * (i - j)) * m_QP_T *
                      m_QP_T * m_QP_T / 6.0;
        m_PPu(i + m_QP_N, j + m_QP_N) =
            (1 + 3 * (i - j) + 3 * (i - j) * (i - j)) * m_QP_T * m_QP_T *
            m_QP_T / 6.0;
        m_PPu(i, j + m_QP_N) = 0.0;
        m_PPu(i + m_QP_N, j) = 0.0;

      } else {

        m_VPu(i, j) = 0.0;
        m_VPu(i + m_QP_N, j + m_QP_N) = 0.0;
        m_VPu(i, j + m_QP_N) = 0.0;
        m_VPu(i + m_QP_N, j) = 0.0;

        m_PPu(i, j) = 0.0;
        m_PPu(i + m_QP_N, j + m_QP_N) = 0.0;
        m_PPu(i, j + m_QP_N) = 0.0;
        m_PPu(i + m_QP_N, j) = 0.0;
      }
    }
  }

  // Build m_Px.
  m_Px.resize(m_QP_N, 3);

  for (unsigned int li = 0; li < m_QP_N; li++) {
    m_Px(li, 0) = 1.0;
    m_Px(li, 1) = (double)(1.0 + li) * m_QP_T;
    m_Px(li, 2) =
        (li + 1.0) * (li + 1.0) * m_QP_T * m_QP_T * 0.5 - m_ComHeight / 9.81;
  }
  if (m_FullDebug > 2) {
    ofstream aof;
    aof.open("VPx.dat");
    aof << m_VPx;
    aof.close();

    aof.open("m_PPx.dat");
    aof << m_PPx;
    aof.close();

    aof.open("VPu.dat");
    aof << m_VPu;
    aof.close();

    aof.open("PPu.dat");
    aof << m_PPu;
    aof.close();
  }

  return 0;
}

int ZMPConstrainedQPFastFormulation::ValidationConstraints(
    double *&DPx, double *&DPu, int NbOfConstraints,
    deque<LinearConstraintInequality_t *> &QueueOfLConstraintInequalities,
    unsigned int li, double *X, double StartingTime) {
  double lSizeMat = QueueOfLConstraintInequalities.back()->EndingTime / m_QP_T;
  Eigen::MatrixXd vnlPx;
  Eigen::MatrixXd vnlPu;
  Eigen::MatrixXd vnlValConstraint;
  Eigen::MatrixXd vnlX;
  Eigen::MatrixXd vnlStorePx;
  Eigen::MatrixXd vnlStoreX;
  Eigen::Matrix<int, Eigen::Dynamic, 1> ConstraintNb;

  vnlX.resize(2 * m_QP_N, 1);
  vnlStorePx.resize(NbOfConstraints, 1 + (unsigned int)lSizeMat);

  for (unsigned int i = 0; i < vnlStorePx.rows(); i++) {
    for (unsigned int j = 0; j < vnlStorePx.cols(); j++) {
      vnlStorePx(i, j) = 0.0;
    }
  }
  vnlStoreX.resize(2 * m_QP_N, 1 + (unsigned int)lSizeMat);

  for (unsigned int i = 0; i < 2 * m_QP_N; i++)
    vnlStoreX(i, 0) = 0.0;

  ConstraintNb.resize(1 + (unsigned int)lSizeMat, 1);

  ConstraintNb[li] = NbOfConstraints;
  vnlPu.resize(NbOfConstraints, 2 * m_QP_N);
  vnlPx.resize(NbOfConstraints, 1);

  for (int i = 0; i < NbOfConstraints; i++) {
    vnlPx(i, 0) = vnlStorePx(i, li) = DPx[i];
  }

  for (int i = 0; i < NbOfConstraints; i++)
    for (unsigned int j = 0; j < 2 * m_QP_N; j++)
      vnlPu(i, j) = DPu[j * (NbOfConstraints + 1) + i];

  for (unsigned int i = 0; i < 2 * m_QP_N; i++) {
    vnlStoreX(i, li) = X[i];
    vnlX(i, 0) = X[i];
  }

  vnlValConstraint = vnlPu * vnlX + vnlPx;

  if (vnlValConstraint.cols() != 1) {
    cout << "Problem during validation of the constraints matrix: " << endl;
    cout << "   size for the columns different from 1" << endl;
    return -1;
  }

  for (int i = 0; i < NbOfConstraints; i++) {
    unsigned int pbOnCurrent = 0;
    if (vnlValConstraint(i, 0) < -1e-8) {
      std::cerr << "Problem during validation of the constraint: " << std::endl;
      std::cerr << "  constraint " << i << " is not positive" << std::endl;
      std::cerr << vnlValConstraint(i, 0) << std::endl;
      pbOnCurrent = 1;
    }

    if (pbOnCurrent) {
      std::cerr << "PbonCurrent: " << pbOnCurrent << " " << li << " Contrainte "
                << i << " StartingTime :" << StartingTime << std::endl;
      if (pbOnCurrent) {
        return -1;
      }
    }
  }

  if (m_FullDebug > 2) {
    ofstream aof;
    aof.open("StorePx.dat", ofstream::out);

    for (unsigned int i = 0; i < vnlStorePx.rows(); i++) {
      for (unsigned int j = 0; j < vnlStorePx.cols(); j++) {
        aof << vnlStorePx(i, j) << " ";
      }
      aof << endl;
    }
    aof.close();

    char lBuffer[1024];
    sprintf(lBuffer, "StoreX.dat");
    aof.open(lBuffer, ofstream::out);

    for (unsigned int i = 0; i < vnlStoreX.rows(); i++) {
      for (unsigned int j = 0; j < vnlStoreX.cols(); j++) {
        aof << vnlStoreX(i, j) << " ";
      }
      aof << endl;
    }
    aof.close();

    aof.open("Cnb.dat", ofstream::out);
    for (unsigned int i = 0; i < ConstraintNb.size(); i++) {
      aof << ConstraintNb[i] << endl;
    }
    aof.close();
  }
  return 0;
}
int ZMPConstrainedQPFastFormulation::
    BuildingConstantPartOfTheObjectiveFunctionQLD(Eigen::MatrixXd &OptA) {
  for (unsigned int i = 0; i < 2 * m_QP_N; i++)
    for (unsigned int j = 0; j < 2 * m_QP_N; j++)
      m_Q[i * m_QP_N * 2 + j] = OptA(j, i);

  return 0;
}
int ZMPConstrainedQPFastFormulation::
    BuildingConstantPartOfTheObjectiveFunctionQLDANDLQ(Eigen::MatrixXd &OptA) {

  /*! Build cholesky matrix of the optimum
    We copy only the upper corner of the OptA matrix
    because we know its specific structure.
  */
  double *localQ = new double[m_QP_N * m_QP_N];
  for (unsigned int i = 0; i < m_QP_N; i++)
    for (unsigned int j = 0; j < m_QP_N; j++)
      localQ[i * m_QP_N + j] = OptA(i, j);

  double *localLQ = new double[m_QP_N * m_QP_N];
  double *localiLQ = new double[m_QP_N * m_QP_N];

  memset(localLQ, 0, m_QP_N * m_QP_N * sizeof(double));
  memset(localiLQ, 0, m_QP_N * m_QP_N * sizeof(double));

  OptCholesky anOCD(m_QP_N, m_QP_N, OptCholesky::MODE_NORMAL);
  anOCD.SetA(localQ, m_QP_N);
  anOCD.SetL(localLQ);
  anOCD.SetiL(localiLQ);

  anOCD.ComputeNormalCholeskyOnANormal();
  anOCD.ComputeInverseCholeskyNormal(1);

  if (m_FullDebug > 0) {
    ofstream aof;
    char Buffer[1024];
    sprintf(Buffer, "localQ.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < m_QP_N; i++) {
      for (unsigned int j = 0; j < m_QP_N; j++)
        aof << localQ[i * m_QP_N + j] << " ";
      aof << endl;
    }
    aof.close();

    sprintf(Buffer, "localLQ.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < m_QP_N; i++) {
      for (unsigned int j = 0; j < m_QP_N; j++)
        aof << localLQ[i * m_QP_N + j] << " ";
      aof << endl;
    }
    aof.close();

    sprintf(Buffer, "localiLQ.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < m_QP_N; i++) {
      for (unsigned int j = 0; j < m_QP_N; j++)
        aof << localiLQ[i * m_QP_N + j] << " ";
      aof << endl;
    }
    aof.close();
  }

  m_LQ.resize(2 * m_QP_N, 2 * m_QP_N);
  m_iLQ.resize(2 * m_QP_N, 2 * m_QP_N);

  for (unsigned int i = 0; i < m_QP_N; i++) {
    for (unsigned int j = 0; j < m_QP_N; j++) {
      m_LQ(i, j) = localLQ[i * m_QP_N + j];
      m_LQ(i + m_QP_N, j + m_QP_N) = localLQ[i * m_QP_N + j];
      m_LQ(i, j + m_QP_N) = 0.0;
      m_LQ(i + m_QP_N, j) = 0.0;

      m_iLQ(i, j) = localiLQ[i * m_QP_N + j];
      m_iLQ(i + m_QP_N, j + m_QP_N) = localiLQ[i * m_QP_N + j];
      m_iLQ(i, j + m_QP_N) = 0.0;
      m_iLQ(i + m_QP_N, j) = 0.0;
    }
  }

  // New formulation (Dimitar08)
  m_OptB = m_iLQ * m_OptB;

  // New formulation (Dimitar08)
  m_OptC = m_iLQ * m_OptC;

  if (m_FullDebug > 0) {
    ofstream aof;
    char Buffer[1024];

    sprintf(Buffer, "LQ.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < 2 * m_QP_N; i++) {
      for (unsigned int j = 0; j < 2 * m_QP_N; j++)
        aof << m_LQ(i, j) << " ";
      aof << endl;
    }
    aof.close();

    sprintf(Buffer, "iLQ.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < 2 * m_QP_N; i++) {
      for (unsigned int j = 0; j < 2 * m_QP_N; j++)
        aof << m_iLQ(i, j) << " ";
      aof << endl;
    }
    aof.close();
  }
  delete[] localQ;
  delete[] localLQ;
  delete[] localiLQ;

  return 0;
}

int ZMPConstrainedQPFastFormulation::
    BuildingConstantPartOfTheObjectiveFunction() {

  Eigen::MatrixXd OptA;
  Eigen::MatrixXd tmp;

  //  OptA = Id + alpha * VPu.Transpose() * VPu + beta * PPu.Transpose() * PPu;
  Eigen::MatrixXd lterm1;
  lterm1 = m_PPu.transpose();
  tmp = lterm1 * m_PPu;
  lterm1 = m_Beta * tmp;

  Eigen::MatrixXd lterm2;
  lterm2 = m_VPu.transpose();
  tmp = lterm2 * m_VPu;
  lterm2 = m_Alpha * lterm2;

  OptA.resize(lterm1.rows(), lterm1.cols());
  OptA.setIdentity();
  OptA = OptA + lterm1 + lterm2;

  // Initialization of the matrice regarding the quadratic
  // part of the objective function.
  m_Q = new double[4 * m_QP_N * m_QP_N];
  memset(m_Q, 0, 4 * m_QP_N * m_QP_N * sizeof(double));
  for (unsigned int i = 0; i < 2 * m_QP_N; i++)
    m_Q[i * 2 * m_QP_N + i] = 1.0;

  if (m_FullDebug > 0) {
    ofstream aof;
    char Buffer[1024];
    sprintf(Buffer, "Q.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < 2 * m_QP_N; i++) {
      for (unsigned int j = 0; j < 2 * m_QP_N; j++)
        aof << m_Q[i * m_QP_N * 2 + j] << " ";
      aof << endl;
    }
    aof.close();
  }

  /*! Compute constants of the linear part of the objective function. */
  lterm1 = m_PPu.transpose();
  lterm1 = lterm1 * m_PPx;
  m_OptB = m_VPu.transpose();
  tmp = m_OptB * m_VPx;
  m_OptB = m_Alpha * tmp;
  m_OptB = m_OptB + m_Beta * lterm1;

  m_OptC = m_PPu.transpose();
  m_OptC = m_Beta * m_OptC;

  if ((m_FastFormulationMode == QLDANDLQ) || (m_FastFormulationMode == PLDP)) {
    BuildingConstantPartOfTheObjectiveFunctionQLDANDLQ(OptA);
  } else {
    BuildingConstantPartOfTheObjectiveFunctionQLD(OptA);
  }

  if (m_FullDebug > 0) {
    ofstream aof;
    char Buffer[1024];
    sprintf(Buffer, "OptB.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < m_OptB.rows(); i++) {
      for (unsigned int j = 0; j < m_OptB.cols() - 1; j++)
        aof << m_OptB(i, j) << " ";
      aof << m_OptB(i, m_OptB.cols() - 1);
      aof << endl;
    }
    aof.close();
  }

  if (m_FullDebug > 0) {
    ofstream aof;
    char Buffer[1024];
    sprintf(Buffer, "OptC.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < m_OptC.rows(); i++) {
      for (unsigned int j = 0; j < m_OptC.cols() - 1; j++)
        aof << m_OptC(i, j) << " ";
      aof << m_OptC(i, m_OptC.cols() - 1);
      aof << endl;
    }
    aof.close();
  }

  return 0;
}

int ZMPConstrainedQPFastFormulation::
    BuildingConstantPartOfConstraintMatrices() {
  if (m_Pu == 0)
    m_Pu = new double[m_QP_N * m_QP_N];

  double *lInterPu = 0;
  double *ptPu = 0;

  if ((m_FastFormulationMode == QLDANDLQ) || (m_FastFormulationMode == PLDP)) {
    lInterPu = new double[m_QP_N * m_QP_N];
    memset(lInterPu, 0, m_QP_N * m_QP_N * sizeof(double));
    ptPu = lInterPu;
  } else
    ptPu = m_Pu;

  memset(m_Pu, 0, m_QP_N * m_QP_N * sizeof(double));

  // Recursive multiplication of the system is applied.
  // we keep the transpose form, i.e. Pu'.
  for (unsigned i = 0; i < m_QP_N; i++) {

    for (unsigned k = 0; k <= i; k++) {
      ptPu[k * m_QP_N + i] = ((1 + 3 * (i - k) + 3 * (i - k) * (i - k)) *
                                  m_QP_T * m_QP_T * m_QP_T / 6.0 -
                              m_QP_T * m_ComHeight / 9.81);
    }
  }

  // Consider QLDANDLQ formulation.
  if ((m_FastFormulationMode == QLDANDLQ) || (m_FastFormulationMode == PLDP)) {
    // Premultiplication by LQ-1
    // Indeed we have to provide qld transpose matrix,
    // therefore instead of providing D*Pu*iLQ (IROS 2008 p 28)
    // we provide its transpose:
    // (D*Pu*iLQ')' = iLQ*Pu'*D'
    // So here we compute iLQ*Pu'
    // Be careful with the two stages resolution.
    for (unsigned i = 0; i < m_QP_N; i++) {

      for (unsigned j = 0; j < m_QP_N; j++) {
        m_Pu[i * m_QP_N + j] = 0;
        for (unsigned k = 0; k < m_QP_N; k++) {
          m_Pu[i * m_QP_N + j] += m_iLQ(i, k) * ptPu[k * m_QP_N + j];
        }
      }
    }

    if (m_FastFormulationMode == PLDP) {
      Eigen::MatrixXd m_mal_Pu(m_QP_N, m_QP_N);
      for (unsigned j = 0; j < m_QP_N; j++)
        for (unsigned k = 0; k < m_QP_N; k++)
          m_mal_Pu(j, k) = m_Pu[j * m_QP_N + k];
      m_iPu = m_mal_Pu.inverse();
    }
  }

  if (m_FullDebug > 0) {
    ofstream aof;
    char Buffer[1024];
    sprintf(Buffer, "PuCst.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < m_QP_N; i++) {
      for (unsigned int j = 0; j < m_QP_N; j++)
        aof << m_Pu[j + i * m_QP_N] << " ";
      aof << endl;
    }
    aof.close();

    sprintf(Buffer, "tmpPuCst.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < m_QP_N; i++) {
      for (unsigned int j = 0; j < m_QP_N; j++)
        aof << ptPu[j + i * m_QP_N] << " ";
      aof << endl;
    }
    aof.close();

    sprintf(Buffer, "tmpiLQ.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < m_QP_N; i++) {
      for (unsigned int j = 0; j < m_QP_N; j++)
        aof << m_iLQ(i, j) << " ";
      aof << endl;
    }
    aof.close();
  }

  delete[] lInterPu;
  return 0;
}

int ZMPConstrainedQPFastFormulation::InitConstants() {
  int r;
  if ((r = InitializeMatrixPbConstants()) < 0)
    return r;

  if ((r = BuildingConstantPartOfTheObjectiveFunction()) < 0)
    return r;

  if ((r = BuildingConstantPartOfConstraintMatrices()) < 0)
    return r;

  return 0;
}

void ZMPConstrainedQPFastFormulation::SetAlpha(const double &anAlpha) {
  m_Alpha = anAlpha;
}

const double &ZMPConstrainedQPFastFormulation::GetAlpha() const {
  return m_Alpha;
}

void ZMPConstrainedQPFastFormulation::SetBeta(const double &anAlpha) {
  m_Beta = anAlpha;
}

const double &ZMPConstrainedQPFastFormulation::GetBeta() const {
  return m_Beta;
}

int ZMPConstrainedQPFastFormulation::BuildConstraintMatrices(
    double *&DPx, double *&DPu, unsigned N, double T, double StartingTime,
    deque<LinearConstraintInequality_t *> &QueueOfLConstraintInequalities,
    double, // Com_Height,
    unsigned int &NbOfConstraints, Eigen::VectorXd &xk, Eigen::VectorXd &ZMPRef,
    unsigned int &NextNumberOfRemovedConstraints) {
  // Discretize the problem.
  ODEBUG(" N:" << N << " T: " << T);

  // Creates the matrices.
  // The memory will be bounded to 8 constraints per
  // support foot (double support case).
  // Will be probably all the time smaller.
  if (DPx == 0)
    DPx = new double[8 * N + 1];

  if (DPu == 0)
    DPu = new double[(8 * N + 1) * 2 * N];

  memset(DPu, 0, (8 * N + 1) * 2 * N * sizeof(double));

  deque<LinearConstraintInequality_t *>::iterator LCI_it, store_it;
  LCI_it = QueueOfLConstraintInequalities.begin();
  while (LCI_it != QueueOfLConstraintInequalities.end()) {
    if ((StartingTime >= (*LCI_it)->StartingTime) &&
        (StartingTime <= (*LCI_it)->EndingTime)) {
      break;
    }
    LCI_it++;
  }
  store_it = LCI_it;

  // Did not find the appropriate Linear Constraint.
  if (LCI_it == QueueOfLConstraintInequalities.end()) {
    cout << "HERE 3" << endl;
    return -1;
  }

  if (m_FullDebug > 2) {
    char Buffer[1024];
    sprintf(Buffer, "PXD_%f.dat", StartingTime);
    RESETDEBUG4(Buffer);
    ODEBUG6("xk:" << xk << " Starting time: " << StartingTime, Buffer);
    char Buffer2[1024];
    sprintf(Buffer2, "PXxD_%f.dat", StartingTime);
    RESETDEBUG4(Buffer2);

    char Buffer3[1024];
    sprintf(Buffer3, "PXyD_%f.dat", StartingTime);
    RESETDEBUG4(Buffer3);
  }

  // Compute first the number of constraint.
  unsigned int IndexConstraint = 0;
  for (unsigned int i = 0; i < N; i++) {

    double ltime = StartingTime + i * T;
    if (ltime > (*LCI_it)->EndingTime)
      LCI_it++;

    if (LCI_it == QueueOfLConstraintInequalities.end()) {
      break;
    }
    IndexConstraint += (unsigned int)((*LCI_it)->A.rows());
  }
  NbOfConstraints = IndexConstraint;

  Eigen::MatrixXd lD;
  lD.resize(NbOfConstraints, 2 * N);

  Eigen::VectorXd lb(NbOfConstraints);

  LCI_it = store_it;

  // Store the number of constraint to be generated for the first
  // slot of time control of the algorithm.
  NextNumberOfRemovedConstraints = (unsigned int)((*LCI_it)->A.rows());

  IndexConstraint = 0;
  ODEBUG("Starting Matrix to build the constraints. ");
  ODEBUG((*LCI_it)->A);
  for (unsigned int i = 0; i < N; i++) {

    double ltime = StartingTime + i * T;
    if (ltime > (*LCI_it)->EndingTime) {
      LCI_it++;
    }
    ZMPRef[i] = (*LCI_it)->Center(0);
    ZMPRef[i + N] = (*LCI_it)->Center(1);

    // For each constraint.
    for (unsigned j = 0; j < (*LCI_it)->A.rows(); j++) {

      // Verification of constraints.
      DPx[IndexConstraint] =
          // X Axis * A
          (xk[0] * m_Px(i, 0) + xk[1] * m_Px(i, 1) + xk[2] * m_Px(i, 2)) *
              (*LCI_it)->A(j, 0) +
          // Y Axis * A
          (xk[3] * m_Px(i, 0) + xk[4] * m_Px(i, 1) + xk[5] * m_Px(i, 2)) *
              (*LCI_it)->A(j, 1)
          // Constante part of the constraint
          + (*LCI_it)->B(j, 0);

      ODEBUG6(DPx[IndexConstraint] << " " << (*LCI_it)->A(j, 0) << " "
                                   << (*LCI_it)->A[j][1] << " "
                                   << (*LCI_it)->B(j, 0),
              Buffer);
      ODEBUG6(1 << " " << T * (i + 1) << " "
                << (i + 1) * (i + 1) * T * T / 2 - Com_Height / 9.81,
              Buffer2);
      ODEBUG6(1 << " " << T * (i + 1) << " "
                << (i + 1) * (i + 1) * T * T / 2 - Com_Height / 9.81,
              Buffer3);

      m_SimilarConstraints[IndexConstraint] = (*LCI_it)->SimilarConstraints[j];

      if (m_FastFormulationMode == QLD) {
        // In this case, Pu is triangular.
        // so we can speed up the computation.
        for (unsigned k = 0; k <= i; k++) {
          // X axis
          DPu[IndexConstraint + k * (NbOfConstraints + 1)] =
              (*LCI_it)->A(j, 0) * m_Pu[k * N + i];
          // Y axis
          DPu[IndexConstraint + (k + N) * (NbOfConstraints + 1)] =
              (*LCI_it)->A(j, 1) * m_Pu[k * N + i];
        }
      } else if ((m_FastFormulationMode == QLDANDLQ) ||
                 (m_FastFormulationMode == PLDP)) {
        // In this case, Pu is *NOT* triangular.
        for (unsigned k = 0; k < N; k++) {
          // X axis
          DPu[IndexConstraint + k * (NbOfConstraints + 1)] =
              (*LCI_it)->A(j, 0) * m_Pu[k * N + i];
          // Y axis
          DPu[IndexConstraint + (k + N) * (NbOfConstraints + 1)] =
              (*LCI_it)->A(j, 1) * m_Pu[k * N + i];
        }
      }
      ODEBUG("IC: " << IndexConstraint);
      IndexConstraint++;
    }
  }

  ODEBUG6("Index Constraint :" << IndexConstraint, Buffer);
  static double localtime = -m_QP_T;
  localtime += m_QP_T;

  ODEBUG("IndexConstraint:" << IndexConstraint << " localTime :" << localtime);

  //  if (localtime>=1.96)
  if (0) {
    ODEBUG("localtime: " << localtime);
    ofstream aof;

    char Buffer[1024];
    sprintf(Buffer, "DPu.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < IndexConstraint; i++) {
      for (unsigned int j = 0; j < 2 * N; j++)
        aof << DPu[j * (NbOfConstraints + 1) + i] << " ";
      aof << endl;
    }
    aof.close();

    sprintf(Buffer, "DPx.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int j = 0; j < IndexConstraint; j++)
      aof << DPx[j] << " ";
    aof << endl;
    aof.close();

    sprintf(Buffer, "CZMPRef.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int j = 0; j < 2 * N; j++)
      aof << ZMPRef[j] << " ";
    aof << endl;
    aof.close();

    sprintf(Buffer, "lD.dat");
    aof.open(Buffer, ofstream::out);
    ODEBUG(lD.rows() << " " << lD.cols() << " ");
    for (unsigned int lj = 0; lj < lD.rows(); lj++) {
      for (unsigned int k = 0; k < lD.cols(); k++)
        aof << lD(lj, k) << " ";
      aof << endl;
    }
    aof.close();

    sprintf(Buffer, "lb.dat");
    aof.open(Buffer, ofstream::out);
    for (unsigned int j = 0; j < IndexConstraint; j++)
      aof << lb(j) << " ";
    aof << endl;
    aof.close();

    //      exit(0);
  }

  //  if (m_FullDebug>0)
  if (0) {

    ofstream aof;
    char Buffer[1024];
    sprintf(Buffer, "PuCst_%f.dat", StartingTime);
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < m_QP_N; i++) {
      for (unsigned int j = 0; j < m_QP_N; j++)
        aof << m_Pu[j + i * m_QP_N] << " ";
      aof << endl;
    }
    aof.close();

    sprintf(Buffer, "D_%f.dat", StartingTime);
    aof.open(Buffer, ofstream::out);
    for (unsigned int i = 0; i < 2 * m_QP_N; i++) {
      for (unsigned int j = 0; j < NbOfConstraints; j++)
        aof << lD(i, j) << " ";
      aof << endl;
    }
    aof.close();

    if (0) {
      sprintf(Buffer, "DPX_%f.dat", StartingTime);
      aof.open(Buffer, ofstream::out);
      for (unsigned int i = 0; i < IndexConstraint; i++) {
        aof << DPx[i] << endl;
      }
      aof.close();
    }
  }

  return 0;
}

int ZMPConstrainedQPFastFormulation::DumpProblem(double *Q, double *D,
                                                 double *DPu,
                                                 unsigned int NbOfConstraints,
                                                 double *Px, double *XL,
                                                 double *XU, double Time) {
  ofstream aof;

  char Buffer[1024];
  sprintf(Buffer, "Problem_%f.dat", Time);
  aof.open(Buffer, ofstream::out);

  // Dumping Q.
  aof << "Q:" << endl;
  for (unsigned int i = 0; i < 2 * m_QP_N; i++) {
    for (unsigned int j = 0; j < 2 * m_QP_N; j++) {
      aof << Q[j * m_QP_N * 2 + i] << " ";
    }
    aof << endl;
  }

  // Dumping D.
  aof << "D:" << endl;
  for (unsigned int i = 0; i < 2 * m_QP_N; i++) {
    aof << D[i] << " ";
  }
  aof << endl;

  // Dumping Pu.
  aof << "DPu:" << endl;
  for (unsigned int i = 0; i < NbOfConstraints; i++) {
    for (unsigned int j = 0; j < 2 * m_QP_N; j++) {
      aof << DPu[j * (NbOfConstraints + 1) + i] << " ";
    }
    aof << endl;
  }

  // Dumping Px.
  aof << "Px:" << endl;
  for (unsigned int i = 0; i < NbOfConstraints; i++) {
    aof << Px[i] << " ";
  }
  aof << endl;

  // Dumping XL.
  aof << "XL:" << endl;
  for (unsigned int i = 0; i < 2 * m_QP_N; i++) {
    aof << XL[i] << " ";
  }
  aof << endl;

  // Dumping XU.
  aof << "XU:" << endl;
  for (unsigned int i = 0; i < 2 * m_QP_N; i++) {
    aof << XU[i] << " ";
  }
  aof << endl;

  aof.close();
  return 0;
}
int ZMPConstrainedQPFastFormulation::BuildZMPTrajectoryFromFootTrajectory(
    deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
    deque<FootAbsolutePosition> &RightFootAbsolutePositions,
    deque<ZMPPosition> &ZMPRefPositions, deque<COMState> &COMStates,
    double ConstraintOnX, double ConstraintOnY, double T, unsigned int N) {

  double *DPx = 0, *DPu = 0;
  unsigned int NbOfConstraints = 8 * N;
  // Nb of constraints to be taken into account
  // for each iteration

  Eigen::VectorXd ZMPRef;
  Eigen::VectorXd OptD(2 * N);

  int CriteriaToMaximize = 1;

  RESETDEBUG4("DebugInterpol.dat");
  ZMPRef.resize(2 * N);

  int m = NbOfConstraints;
  int me = 0;
  int mmax = NbOfConstraints + 1;
  int n = 2 * N;
  int nmax = 2 * N; // Size of the matrix to compute the cost function.
  int mnn = m + n + n;

  double *D = new double[2 * N];    // Constant part of the objective function
  double *XL = new double[2 * N];   // Lower bound of the jerk.
  double *XU = new double[2 * N];   // Upper bound of the jerk.
  double *X = new double[2 * N];    // Solution of the system.
  double *NewX = new double[2 * N]; // Solution of the system.
  double Eps = 1e-8;
  double *U = (double *)malloc(sizeof(double) *
                               mnn); // Returns the Lagrange multipliers.;

  int iout = 0;
  int ifail;
  int iprint = 1;
  int lwar = 3 * nmax * nmax / 2 + 10 * nmax + 2 * mmax + 20000;
  ;
  double *war = (double *)malloc(sizeof(double) * lwar);
  int liwar = n;              //
  int *iwar = new int[liwar]; // The Cholesky decomposition is done internally.

  deque<LinearConstraintInequality_t *> QueueOfLConstraintInequalities;

  if (m_FullDebug > 0) {
    RESETDEBUG4("DebugPBW.dat");
    RESETDEBUG4("DebugPBW_Pb.dat");

    ODEBUG6("A:" << m_A << endl << "B:" << m_B, "DebugPBW_Pb.dat");
  }

  // Build a set of linear constraint inequalities.
  m_FCALS->BuildLinearConstraintInequalities(
      LeftFootAbsolutePositions, RightFootAbsolutePositions,
      QueueOfLConstraintInequalities, ConstraintOnX, ConstraintOnY);

  deque<LinearConstraintInequality_t *>::iterator LCI_it;
  LCI_it = QueueOfLConstraintInequalities.begin();
  while (LCI_it != QueueOfLConstraintInequalities.end()) {
    //      cout << *LCI_it << endl;
    //      cout << (*LCI_it)->StartingTime << " "
    // << (*LCI_it)->EndingTime << endl;
    LCI_it++;
  }

  // pre computes the matrices needed for the optimization.

  double TotalAmountOfCPUTime = 0.0, CurrentCPUTime = 0.0;
  struct timeval start, end;
  int li = 0;
  double dinterval = T / m_SamplingPeriod;
  int interval = (int)dinterval;
  bool StartingSequence = true;

  Eigen::VectorXd xk(6);

  ODEBUG("0.0 " << QueueOfLConstraintInequalities.back()->EndingTime - N * T
                << " "
                << " T: " << T << " N: " << N << " interval " << interval);
  unsigned int NumberOfRemovedConstraints = 0,
               NextNumberOfRemovedConstraints = 0;
  for (double StartingTime = 0.0;
       StartingTime < QueueOfLConstraintInequalities.back()->EndingTime - N * T;
       StartingTime += T, li++) {
    gettimeofday(&start, 0);

    // Read the current state of the 2D Linearized Inverted Pendulum.
    m_2DLIPM->GetState(xk);

    ODEBUG("State: " << xk[0] << " " << xk[3] << " " << xk[1] << " " << xk[4]
                     << " " << xk[2] << " " << xk[5] << " ");
    if (m_FastFormulationMode == QLDANDLQ) {
      ODEBUG6(xk[0] << " " << xk[3] << " " << xk[1] << " " << xk[4] << " "
                    << xk[2] << " " << xk[5] << " ",
              "Check2DLIPM_QLDANDLQ.dat");
    } else if (m_FastFormulationMode == PLDP) {
      ODEBUG6(xk[0] << " " << xk[3] << " " << xk[1] << " " << xk[4] << " "
                    << xk[2] << " " << xk[5] << " ",
              "Check2DLIPM_PLDP.dat");
    }
    // Build the related matrices.
    BuildConstraintMatrices(DPx, DPu, N, T, StartingTime,
                            QueueOfLConstraintInequalities, m_ComHeight,
                            NbOfConstraints, xk, ZMPRef,
                            NextNumberOfRemovedConstraints);

    m = NbOfConstraints;

    mmax = NbOfConstraints + 1;
    lwar = 3 * nmax * nmax / 2 + 10 * nmax + 2 * mmax + 20000;
    mnn = m + n + n;

    // Call to QLD (a linearly constrained quadratic problem solver)

    // Prepare D.
    //      PrepareZMPRef(ZMPRef,StartingTime,QueueOfLConstraintInequalities);

    if (m_FullDebug > 2) {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer, "ZMPRef_%f.dat", StartingTime);
      aof.open(Buffer, ofstream::out);
      for (unsigned int i = 0; i < 2 * N; i++) {
        aof << ZMPRef[i] << endl;
      }
      aof.close();
    }

    if (CriteriaToMaximize == 1) {
      Eigen::VectorXd lterm1v;
      lterm1v = m_OptC + ZMPRef;
      OptD.resize(2 * N);
      OptD = m_OptB + xk;
      OptD -= lterm1v;
      for (unsigned int i = 0; i < 2 * N; i++)
        D[i] = OptD[i];

      if (m_FullDebug > 0) {
        ofstream aof;
        char Buffer[1024];
        sprintf(Buffer, "D_%f.dat", StartingTime);
        aof.open(Buffer, ofstream::out);
        for (unsigned int i = 0; i < 2 * N; i++) {
          aof << OptD[i] << endl;
        }
        aof.close();
      }

    } else {
      // Default : set D to zero.
      for (unsigned int i = 0; i < 2 * N; i++)
        D[i] = 0.0;
    }

    for (unsigned int i = 0; i < 2 * N; i++) {
      XL[i] = -1e8;
      XU[i] = 1e8;
    }
    memset(X, 0, 2 * N * sizeof(double));

    if (m_FastFormulationMode == QLDANDLQ)
      iwar[0] = 0;
    else
      iwar[0] = 1;

    ODEBUG("m: " << m);
    //      DumpProblem(m_Q, D, DPu, m, DPx,XL,XU,StartingTime);

    if ((m_FastFormulationMode == QLDANDLQ) || (m_FastFormulationMode == QLD)) {
      struct timeval lbegin, lend;
      gettimeofday(&lbegin, 0);
      ql0001_(&m, &me, &mmax, &n, &nmax, &mnn, m_Q, D, DPu, DPx, XL, XU, X, U,
              &iout, &ifail, &iprint, war, &lwar, iwar, &liwar, &Eps);
      gettimeofday(&lend, 0);
      CODEDEBUG6(double ldt = lend.tv_sec - lbegin.tv_sec +
                              0.000001 * (lend.tv_usec - lbegin.tv_usec););

      unsigned int NbOfActivatedConstraints = 0;
      for (int lk = 0; lk < m; lk++) {
        if (U[lk] > 0.0) {
          NbOfActivatedConstraints++;
        }
      }
      ODEBUG6(NbOfActivatedConstraints, "InfosQLD.dat");
      ODEBUG6(ldt, "dtQLD.dat");
    } else if (m_FastFormulationMode == PLDP) {
      ODEBUG("State: " << xk[0] << " " << xk[3] << " " << xk[1] << " " << xk[4]
                       << " " << xk[2] << " " << xk[5] << " ");
      struct timeval lbegin, lend;
      gettimeofday(&lbegin, 0);

      ifail = m_PLDPSolver->SolveProblem(
          D, (unsigned int)m, DPu, DPx, &ZMPRef(0), &xk(0), X,
          m_SimilarConstraints, NumberOfRemovedConstraints, StartingSequence);
      StartingSequence = false;
      NumberOfRemovedConstraints = NextNumberOfRemovedConstraints;
      gettimeofday(&lend, 0);
      CODEDEBUG6(double ldt = lend.tv_sec - lbegin.tv_sec +
                              0.000001 * (lend.tv_usec - lbegin.tv_usec););

      ODEBUG6(ldt, "dtPLDP.dat");
    }

    if (ifail != 0) {
      cout << "IFAIL: " << ifail << " at time: " << StartingTime << endl;
      return -1;
    }

    double *ptX = 0;
    if ((m_FastFormulationMode == QLDANDLQ) ||
        (m_FastFormulationMode == PLDP)) {
      /* Multiply the solution by the transpose of iLQ
         because it is a triangular matrix we do a specific
         multiplication.
      */
      memset(NewX, 0, 2 * N * sizeof(double));

      double *pm_iLQ = &m_iLQ(0);
      double *pNewX = NewX;

      for (unsigned int i = 0; i < 2 * N; i++) {
        double *pX = X + i;
        double *piLQ = pm_iLQ + i * 2 * N + i;
        *pNewX = 0.0;
        for (unsigned int j = i; j < 2 * N; j++) {
          *pNewX += (*piLQ) * (*pX++);
          piLQ += 2 * N;
        }
        pNewX++;
      }
      ptX = NewX;
    } else
      ptX = X;

    /* Simulation of the Single Point Mass model
       with the new command.
    */
    ODEBUG("X[0] " << X[0] << " X[N] :" << X[N]);

    // Calling this method will automatically
    // update the ZMPRefPositions.
    m_2DLIPM->Interpolation(COMStates, ZMPRefPositions, li * interval, ptX[0],
                            ptX[N]);

    m_2DLIPM->OneIteration(ptX[0], ptX[N]);

    ODEBUG6("uk:" << uk, "DebugPBW.dat");
    ODEBUG6("xk:" << xk, "DebugPBW.dat");

    /* Constraint validation */
    if (0) {
      if (ValidationConstraints(DPx, DPu, m, QueueOfLConstraintInequalities, li,
                                X, StartingTime) < 0) {
        cout << "Something is wrong with the constraints." << endl;
        //   exit(-1);
      }
    }

    if (m_FullDebug > 2) {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer, "X_%f.dat", StartingTime);
      aof.open(Buffer, ofstream::out);
      for (unsigned int i = 0; i < 2 * N; i++) {
        aof << X[i] << endl;
      }
      aof.close();
    }

    // Compute CPU consumption time.
    gettimeofday(&end, 0);
    CurrentCPUTime = (double)(end.tv_sec - start.tv_sec) +
                     0.000001 * (double)(end.tv_usec - start.tv_usec);
    TotalAmountOfCPUTime += CurrentCPUTime;
    ODEBUG("Current Time : "
           << StartingTime << " "
           << " Virtual time to simulate: "
           << QueueOfLConstraintInequalities.back()->EndingTime - StartingTime
           << "Computation Time " << CurrentCPUTime << " "
           << TotalAmountOfCPUTime);
  }

  /*  cout << "Size of PX: " << vnlStorePx.rows() << " "
      << vnlStorePx.cols() << " " << endl; */
  delete[] D;
  delete[] XL;
  delete[] XU;
  delete[] X;
  free(war);
  free(U);
  delete[] iwar;
  // Clean the queue of Linear Constraint Inequalities.
  //  deque<LinearConstraintInequality_t *>::iterator LCI_it;
  LCI_it = QueueOfLConstraintInequalities.begin();
  while (LCI_it != QueueOfLConstraintInequalities.end()) {
    //      cout << *LCI_it << endl;
    //      cout << (*LCI_it)->StartingTime << " " << (*LCI_it)->EndingTime
    //      << endl;
    delete *(LCI_it);
    LCI_it++;
  }
  QueueOfLConstraintInequalities.clear();

  return 0;
}

void ZMPConstrainedQPFastFormulation::GetZMPDiscretization(
    deque<ZMPPosition> &ZMPPositions, deque<COMState> &COMStates,
    deque<RelativeFootPosition> &RelativeFootPositions,
    deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
    deque<FootAbsolutePosition> &RightFootAbsolutePositions, double Xmax,
    COMState &lStartingCOMState, Eigen::Vector3d &lStartingZMPPosition,
    FootAbsolutePosition &InitLeftFootAbsolutePosition,
    FootAbsolutePosition &InitRightFootAbsolutePosition) {
  if (m_ZMPD == 0)
    return;

  m_ZMPD->GetZMPDiscretization(
      ZMPPositions, COMStates, RelativeFootPositions, LeftFootAbsolutePositions,
      RightFootAbsolutePositions, Xmax, lStartingCOMState, lStartingZMPPosition,
      InitLeftFootAbsolutePosition, InitRightFootAbsolutePosition);

  ODEBUG("Dimitrov algo set on");

  BuildZMPTrajectoryFromFootTrajectory(
      LeftFootAbsolutePositions, RightFootAbsolutePositions, ZMPPositions,
      COMStates, m_ConstraintOnX, m_ConstraintOnY, m_QP_T, m_QP_N);
  if (m_FullDebug > 0) {
    ofstream aof;
    aof.open("DebugDimitrovZMP.dat", ofstream::out);
    for (unsigned int i = 0; i < ZMPPositions.size(); i++) {
      aof << ZMPPositions[i].px << " " << ZMPPositions[i].py << endl;
    }
    aof.close();
  }
}

void ZMPConstrainedQPFastFormulation::CallMethod(std::string &Method,
                                                 std::istringstream &strm) {
  if (Method == ":setdimitrovconstraint") {
    string PBWCmd;
    strm >> PBWCmd;
    if (PBWCmd == "XY") {
      strm >> m_ConstraintOnX;
      strm >> m_ConstraintOnY;
      cout << "Constraint On X: " << m_ConstraintOnX
           << " Constraint On Y: " << m_ConstraintOnY << endl;
    } else if (PBWCmd == "T") {
      strm >> m_QP_T;
      cout << "Sampling for the QP " << m_QP_T << endl;
    } else if (PBWCmd == "N") {
      strm >> m_QP_N;
      cout << "Preview window for the QP " << m_QP_N << endl;
    }
  }

  ZMPRefTrajectoryGeneration::CallMethod(Method, strm);
}

std::size_t ZMPConstrainedQPFastFormulation::InitOnLine(
    deque<ZMPPosition> &,          // FinalZMPPositions,
    deque<COMState> &,             // FinalCOMStates,
    deque<FootAbsolutePosition> &, // FinalLeftFootAbsolutePositions,
    deque<FootAbsolutePosition> &, // FinalRightFootAbsolutePositions,
    FootAbsolutePosition &,        // InitLeftFootAbsolutePosition,
    FootAbsolutePosition &,        // InitRightFootAbsolutePosition,
    deque<RelativeFootPosition> &, // RelativeFootPositions,
    COMState &,                    // lStartingCOMState,
    Eigen::Vector3d &)             // lStartingZMPPosition)
{
  cout << "To be implemented" << endl;
  return 0;
}

void ZMPConstrainedQPFastFormulation::OnLineAddFoot(
    RelativeFootPosition &,        // NewRelativeFootPosition,
    deque<ZMPPosition> &,          // FinalZMPPositions,
    deque<COMState> &,             // FinalCOMStates,
    deque<FootAbsolutePosition> &, // FinalLeftFootAbsolutePositions,
    deque<FootAbsolutePosition> &, // FinalRightFootAbsolutePositions,
    bool)                          // EndSequence)
{
  cout << "To be implemented" << endl;
}

void ZMPConstrainedQPFastFormulation::OnLine(
    double,                        // time,
    deque<ZMPPosition> &,          // FinalZMPPositions,
    deque<COMState> &,             // FinalCOMStates,
    deque<FootAbsolutePosition> &, // FinalLeftFootAbsolutePositions,
    deque<FootAbsolutePosition> &) // FinalRightFootAbsolutePositions)
{
  cout << "To be implemented" << endl;
}

int ZMPConstrainedQPFastFormulation::OnLineFootChange(
    double,                        // time,
    FootAbsolutePosition &,        // aFootAbsolutePosition,
    deque<ZMPPosition> &,          // FinalZMPPositions,
    deque<COMState> &,             // CoMPositions,
    deque<FootAbsolutePosition> &, // FinalLeftFootAbsolutePositions,
    deque<FootAbsolutePosition> &, // FinalRightFootAbsolutePositions,
    StepStackHandler *)            // aStepStackHandler)
{
  cout << "To be implemented" << endl;
  return -1;
}

void ZMPConstrainedQPFastFormulation::EndPhaseOfTheWalking(
    deque<ZMPPosition> &,          // ZMPPositions,
    deque<COMState> &,             // FinalCOMStates,
    deque<FootAbsolutePosition> &, // LeftFootAbsolutePositions,
    deque<FootAbsolutePosition> &) // RightFootAbsolutePositions)
{}

int ZMPConstrainedQPFastFormulation::ReturnOptimalTimeToRegenerateAStep() {
  int r = (int)(m_PreviewControlTime / m_SamplingPeriod);
  return 2 * r;
}

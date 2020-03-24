/*
 * Copyright 2009, 2010,
 *
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
/*! \file TestRiccatiEquation.cpp
  \brief Example to solve the Riccati Equation for the preview control. */

#define NB_OF_FIELDS 1

#include "PreviewControl/OptimalControllerSolver.hh"
#include <Debug.hh>
#include <fstream>
#include <iostream>

using namespace std;

bool compareDebugFiles(string fileName) {
  ifstream alif;
  unsigned max_nb_of_pbs = 100;
  unsigned nb_of_pbs = 0;
  ODEBUG("Report : " << fileName);
  alif.open(fileName.c_str(), ifstream::in);
  if (!alif.is_open()) {
    std::cerr << "Unable to open " << fileName << std::endl;
    return -1;
  }

  ifstream arif;
  string refFileName = fileName + "ref";
  arif.open(refFileName.c_str(), ifstream::in);
  ODEBUG("ReportRef : " << refFileName);

  if (!arif.is_open()) {
    std::cerr << "Unable to open " << refFileName << std::endl;
    return -1;
  }

  ofstream areportof;
  string aFileName;
  aFileName = fileName;
  aFileName += ".report";
  areportof.open(aFileName.c_str(), ofstream::out);

  // Time
  double LocalInput[NB_OF_FIELDS], ReferenceInput[NB_OF_FIELDS];
  bool finalreport = true;
  unsigned long int nblines = 0;
  bool endofinspection = false;

  // Find size of the two files.
  alif.seekg(0, alif.end);
  unsigned long int alif_length = (unsigned long int)alif.tellg();
  alif.seekg(0, alif.beg);

  arif.seekg(0, arif.end);
  unsigned long int arif_length = (unsigned long int)arif.tellg();
  arif.seekg(0, arif.beg);

  while ((!alif.eof()) && (!arif.eof()) && (!endofinspection)) {
    for (unsigned int i = 0; i < NB_OF_FIELDS; i++) {
      alif >> LocalInput[i];
      if (alif.eof()) {
        endofinspection = true;
        break;
      }
    }
    if (endofinspection)
      break;

    for (unsigned int i = 0; i < NB_OF_FIELDS; i++) {
      arif >> ReferenceInput[i];
      if (arif.eof()) {
        endofinspection = true;
        break;
      }
    }
    if (endofinspection)
      break;

    for (unsigned int i = 0; i < NB_OF_FIELDS; i++) {
      if (fabs(LocalInput[i] - ReferenceInput[i]) >= 1e-6) {
        finalreport = false;
        ostringstream oss;
        oss << "l: " << nblines << " col:" << i << " ref: " << ReferenceInput[i]
            << " now: " << LocalInput[i] << " " << nb_of_pbs << std::endl;
        areportof << oss.str();
        std::cout << oss.str();
        nb_of_pbs++;
        if (nb_of_pbs > max_nb_of_pbs) {
          endofinspection = true;
        }
      }
    }

    nblines++;
    if ((nblines * 2 > alif_length) || (nblines * 2 > arif_length)) {
      endofinspection = true;
      break;
    }
  }

  alif.close();
  arif.close();
  areportof.close();
  return finalreport;
}

int main() {
  PatternGeneratorJRL::OptimalControllerSolver *anOCS;

  /* Declare the linear system */
  Eigen::MatrixXd A(3, 3);
  Eigen::MatrixXd b(3, 1);
  Eigen::MatrixXd c(1, 3);
  Eigen::MatrixXd lF;
  Eigen::MatrixXd lK;

  /* Declare the weights for the function */
  ofstream aof;
  double Q, R;
  int Nl;
  double T = 0.005;

  /* Build the initial discrete system
     regarding the CoM and the ZMP. */
  A(0, 0) = 1.0;
  A(0, 1) = T;
  A(0, 2) = T * T / 2.0;
  A(1, 0) = 0.0;
  A(1, 1) = 1.0;
  A(1, 2) = T;
  A(2, 0) = 0.0;
  A(2, 1) = 0.0;
  A(2, 2) = 1;

  b(0, 0) = T * T * T / 6.0;
  b(1, 0) = T * T / 2.0;
  b(2, 0) = T;

  c(0, 0) = 1.0;
  c(0, 1) = 0.0;
  c(0, 2) = -0.814 / 9.8;

  Q = 1.0;
  R = 1e-6;

  Nl = (int)(1.6 / T);

  // Build the derivated system
  Eigen::MatrixXd Ax(4, 4);
  Eigen::MatrixXd tmpA;
  Eigen::MatrixXd bx(4, 1);
  Eigen::MatrixXd tmpb;
  Eigen::MatrixXd cx(1, 4);

  tmpA = c * A;
  cout << "tmpA :" << tmpA << endl;

  Ax(0, 0) = 1.0;
  for (int i = 0; i < 3; i++) {
    Ax(0, i + 1) = tmpA(0, i);
    Ax(i + 1, 0) = 0.;
    for (int j = 0; j < 3; j++)
      Ax(i + 1, j + 1) = A(i, j);
  }
  cout << "Ax: " << endl << Ax << endl;

  tmpb = c * b;
  bx(0, 0) = tmpb(0, 0);
  for (int i = 0; i < 3; i++) {
    bx(i + 1, 0) = b(i, 0);
  }

  cout << "bx: " << endl << bx << endl;

  cx(0, 0) = 1.0;
  cx(0, 1) = 0.0;
  cx(0, 2) = 0.0;
  cx(0, 3) = 0.0;
  cout << "cx: " << endl << cx << endl;

  anOCS =
      new PatternGeneratorJRL::OptimalControllerSolver(Ax, bx, cx, Q, R, Nl);

  anOCS->ComputeWeights(
      PatternGeneratorJRL::OptimalControllerSolver::MODE_WITHOUT_INITIALPOS);
  anOCS->DisplayWeights();
  anOCS->GetF(lF);
  anOCS->GetK(lK);

  aof.open("TestRiccatiEquationWeightsWithoutInitialPose.dat", ofstream::out);
  for (unsigned int li = 0; li < lK.rows(); li++) {
    aof << lK(li, 0) << endl;
  }
  for (unsigned int li = 0; li < lF.rows(); li++) {
    aof << lF(li, 0) << endl;
  }
  aof.close();
  bool sameFile =
      compareDebugFiles("TestRiccatiEquationWeightsWithoutInitialPose.dat");
  delete anOCS;

  // Build the initial discrete system
  // regarding the CoM and the ZMP.
  T = 0.01;
  A(0, 0) = 1.0;
  A(0, 1) = T;
  A(0, 2) = T * T / 2.0;
  A(1, 0) = 0.0;
  A(1, 1) = 1.0;
  A(1, 2) = T;
  A(2, 0) = 0.0;
  A(2, 1) = 0.0;
  A(2, 2) = 1;

  b(0, 0) = T * T * T / 6.0;
  b(1, 0) = T * T / 2.0;
  b(2, 0) = T;

  c(0, 0) = 1.0;
  c(0, 1) = 0.0;
  c(0, 2) = -0.814 / 9.8;

  // Q = 10000000.000000;
  //  R = 1.000000 ;
  Q = 1.0;
  R = 1e-5;

  Nl = (int)(1.6 / T);

  cout << "A: " << endl << A << endl;
  cout << "b: " << endl << b << endl;
  cout << "c: " << endl << c << endl;
  cout << "Nl: " << Nl << endl;
  anOCS = new PatternGeneratorJRL::OptimalControllerSolver(A, b, c, Q, R, Nl);

  anOCS->ComputeWeights(
      PatternGeneratorJRL::OptimalControllerSolver::MODE_WITH_INITIALPOS);

  anOCS->DisplayWeights();

  anOCS->GetF(lF);
  anOCS->GetK(lK);

  aof.open("TestRiccatiEquationWeightsWithInitialPose.dat", ofstream::out);
  for (unsigned int li = 0; li < lK.rows(); li++) {
    aof << lK(li, 0) << endl;
  }
  for (unsigned int li = 0; li < lF.rows(); li++) {
    aof << lF(li, 0) << endl;
  }
  aof.close();

  bool sameFile2 =
      compareDebugFiles("TestRiccatiEquationWeightsWithInitialPose.dat");
  delete anOCS;

  if (sameFile && sameFile2) {
    cout << "Passed test " << endl;
    return 0;
  } else {
    cout << "Failed test " << endl;
    return -1;
  }
}

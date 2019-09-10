/*
 * Copyright 2009, 2010, 2014
 *
 *
 * Olivier  Stasse, Huynh Ngoc Duc
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
/*! \file TestBsplines.cpp
  \brief This Example shows you how to use Bsplines to create a foot trajectory
  on Z . */
#include "Mathematics/Bsplines.hh"
#include <fstream>
#include <iostream>
#include <stdlib.h>

using namespace std;

int PerformTests(int, char *[]) {
  // Test Bspline without way point
  /////////////////////////////////
  double testIP(0.0), testIS(0.0), testIA(0.0), testMP(0.0), testMS(0.0),
      testMA(0.0), testMP2(0.0), testMS2(0.0), testMA2(0.0), testFP(0.0),
      testFS(0.0), testFA(0.0);
  double FT = 1.0;
  double IP = 0.209;
  double IS = 0.009;
  double IA = 1.0;
  double FP = 0.209;
  double FS = 0.009;
  double FA = 0.009;
  vector<double> MP;
  vector<double> ToMP;
  MP.clear();
  ToMP.clear();

  PatternGeneratorJRL::BSplinesFoot *bsplineNoWayPoint;
  bsplineNoWayPoint = new PatternGeneratorJRL::BSplinesFoot(FT, IP, FP, ToMP,
                                                            MP, IS, IA, FS, FA);

  // results
  bsplineNoWayPoint->Compute(0.0, testIP, testIS, testIA);
  bsplineNoWayPoint->Compute(FT, testFP, testFS, testFA);
  bool test_IP = sqrt((IP - testIP) * (IP - testIP)) < 1e-08;
  bool test_IS = sqrt((IS - testIS) * (IS - testIS)) < 1e-08;
  bool test_IA = sqrt((IA - testIA) * (IA - testIA)) < 1e-08;
  bool test_FP = sqrt((FP - testFP) * (FP - testFP)) < 1e-08;
  bool test_FS = sqrt((FS - testFS) * (FS - testFS)) < 1e-08;
  bool test_FA = sqrt((FA - testFA) * (FA - testFA)) < 1e-08;

  bool testBsplinenowayPoint =
      test_IP && test_IS && test_IA && test_FP && test_FS && test_FA;

  if (!testBsplinenowayPoint) {
    std::cerr << "Error unexpected behaviour of bspline generation\n"
              << "bspline with no way point corrupted" << std::endl;
  }
  delete bsplineNoWayPoint;
  bsplineNoWayPoint = NULL;

  // Test Bspline with one way point
  //////////////////////////////////
  MP.clear();
  MP.push_back(0.5);
  ToMP.clear();
  ToMP.push_back(FT / 2.0);
  FT = 1.0;
  IP = 0.1;
  IS = 0.0;
  IA = 0.0;
  FP = 0.2;
  FS = 0.0;
  FA = 0.0;
  PatternGeneratorJRL::BSplinesFoot *bsplineOneWayPoint;
  bsplineOneWayPoint = new PatternGeneratorJRL::BSplinesFoot(
      FT, IP, FP, ToMP, MP, IS, IA, FS, FA);

  bsplineOneWayPoint->Compute(0.0, testIP, testIS, testIA);
  bsplineOneWayPoint->Compute(FT / 2, testMP, testMS, testMA);
  bsplineOneWayPoint->Compute(FT, testFP, testFS, testFA);
  test_IP = sqrt((IP - testIP) * (IP - testIP)) < 1e-08;
  test_IS = sqrt((IS - testIS) * (IS - testIS)) < 1e-08;
  test_IA = sqrt((IA - testIA) * (IA - testIA)) < 1e-08;
  test_FP = sqrt((FP - testFP) * (FP - testFP)) < 1e-08;
  test_FS = sqrt((FS - testFS) * (FS - testFS)) < 1e-08;
  test_FA = sqrt((FA - testFA) * (FA - testFA)) < 1e-08;
  bool test_MP = sqrt((MP[0] - testMP) * (MP[0] - testMP)) < 1e-08;
  bool testBsplineOneWayPoint =
      test_IP && test_IS && test_IA && test_MP && test_FP && test_FS && test_FA;

  if (!bsplineOneWayPoint) {
    std::cerr << "Error unexpected behaviour of bspline generation\n"
              << "bspline with one way point corrupted" << std::endl;
  }
  delete bsplineOneWayPoint;
  bsplineOneWayPoint = NULL;

  // Test Bspline with two way point
  //////////////////////////////////
  FT = 1.0;
  IP = 0.1;
  IS = 0.0;
  IA = 0.0;
  FP = 0.2;
  FS = 0.0;
  FA = 0.0;
  MP.clear();
  MP.push_back(0.35);
  MP.push_back(0.25);
  ToMP.clear();
  ToMP.push_back(FT / 3.0);
  ToMP.push_back(2.0 * FT / 3.0);
  PatternGeneratorJRL::BSplinesFoot *bsplineTwoWayPoint;
  bsplineTwoWayPoint = new PatternGeneratorJRL::BSplinesFoot(
      FT, IP, FP, ToMP, MP, IS, IA, FS, FA);

  bsplineTwoWayPoint->Compute(0.0, testIP, testIS, testIA);
  bsplineTwoWayPoint->Compute(FT / 3.0, testMP, testMS, testMA);
  bsplineTwoWayPoint->Compute(2.0 * FT / 3.0, testMP2, testMS2, testMA2);
  bsplineTwoWayPoint->Compute(FT, testFP, testFS, testFA);
  test_IP = sqrt((IP - testIP) * (IP - testIP)) < 1e-08;
  test_IS = sqrt((IS - testIS) * (IS - testIS)) < 1e-08;
  test_IA = sqrt((IA - testIA) * (IA - testIA)) < 1e-08;
  test_FP = sqrt((FP - testFP) * (FP - testFP)) < 1e-08;
  test_FS = sqrt((FS - testFS) * (FS - testFS)) < 1e-08;
  test_FA = sqrt((FA - testFA) * (FA - testFA)) < 1e-08;
  test_MP = sqrt((MP[0] - testMP) * (MP[0] - testMP)) < 1e-08;
  bool test_MP2 = sqrt((MP[1] - testMP2) * (MP[1] - testMP2)) < 1e-08;
  bool testBsplineTwoWayPoint = test_IP && test_IS && test_IA && test_MP &&
                                test_MP2 && test_FP && test_FS && test_FA;

  if (!testBsplineTwoWayPoint) {
    std::cerr << "Error unexpected behaviour of bspline generation\n"
              << "bspline with two way point corrupted" << std::endl;
  }
  delete bsplineTwoWayPoint;
  bsplineTwoWayPoint = NULL;

  // Test Bspline with knots and control points
  ///////////////////////////////////////////
  /// \brief bsplineKnotsControl
  ofstream aof;
  string aFileName = "bsplineKnotsControl.dat";
  aof.open(aFileName.c_str(), ofstream::out);
  aof.close();
  aof.open(aFileName.c_str(), ofstream::app);
  aof.precision(8);
  aof.setf(ios::scientific, ios::floatfield);
  // aof << filterprecision(m_OneStep.NbOfIt*0.005 ) << " "
  PatternGeneratorJRL::BSplinesFoot *bsplineKnotsControl;
  bsplineKnotsControl = new PatternGeneratorJRL::BSplinesFoot(
      1.4, IP, FP, ToMP, MP, IS, IA, FS, FA);

  deque<double> knot;
  double TimeInterval = 1.4;
  for (int i = 0; i < 8; ++i) {
    knot.push_back(0);
  }
  double pourcentTime = 0.05;
  knot.push_back(pourcentTime);
  knot.push_back(pourcentTime);
  for (int i = 0; i < 8; ++i) {
    knot.push_back(1.0);
  }
  bsplineKnotsControl->SetKnotVector(knot);
  vector<double> controlPoints;
  double InitPosition = 0.0;
  double FinalPosition = 0.1;
  double WayPoint_z = 0.07;
  controlPoints.push_back(InitPosition);
  controlPoints.push_back(InitPosition);
  controlPoints.push_back(InitPosition);
  controlPoints.push_back(InitPosition);
  controlPoints.push_back(InitPosition);
  controlPoints.push_back(FinalPosition);
  controlPoints.push_back(FinalPosition + WayPoint_z);
  controlPoints.push_back(FinalPosition);
  controlPoints.push_back(FinalPosition);
  controlPoints.push_back(FinalPosition);
  controlPoints.push_back(FinalPosition);
  controlPoints.push_back(FinalPosition);
  // controlPoints.push_back(FinalPosition);
  bsplineKnotsControl->SetControlPoints(controlPoints);
  bsplineKnotsControl->FT(TimeInterval);
  bsplineKnotsControl->GenerateDegree();
  bsplineKnotsControl->PrintControlPoints();
  bsplineKnotsControl->PrintDegree();
  bsplineKnotsControl->PrintKnotVector();

  FT = 1.4;
  IP = 0.0;
  IS = 0.0;
  IA = 0.0;
  FP = 0.1;
  FS = 0.0;
  FA = 0.0;
  bsplineKnotsControl->Compute(0.0, testIP, testIS, testIA);
  bsplineKnotsControl->Compute(TimeInterval * pourcentTime, testMP, testMS,
                               testMA);
  bsplineKnotsControl->Compute(TimeInterval, testFP, testFS, testFA);
  test_IP = sqrt((IP - testIP) * (IP - testIP)) < 1e-08;
  test_IS = sqrt((IS - testIS) * (IS - testIS)) < 1e-08;
  test_IA = sqrt((IA - testIA) * (IA - testIA)) < 1e-08;
  test_FP = sqrt((FP - testFP) * (FP - testFP)) < 1e-08;
  test_FS = sqrt((FS - testFS) * (FS - testFS)) < 1e-08;
  test_FA = sqrt((FA - testFA) * (FA - testFA)) < 1e-08;
  bool testbsplineKnotsControl =
      test_IP && test_IS && test_IA && test_FP && test_FS && test_FA;
  std::cout << "testFP : " << testFP << "testbsplineKnotsControl "
            << testbsplineKnotsControl << std::endl;
  std::cout << "MP : [" << TimeInterval << ";" << FinalPosition << "]"
            << std::endl;

  for (double i = 0; i < 1400; ++i) {
    bsplineKnotsControl->Compute(FT * i / 1400.0, testMP, testMS, testMA);
    aof << testMP << endl;
  }

  if (!bsplineKnotsControl || !testBsplineTwoWayPoint) {
    std::cerr << "Error unexpected behaviour of bspline generation\n"
              << "bspline with knots and control points" << std::endl;
  }
  delete bsplineKnotsControl;
  bsplineKnotsControl = NULL;

  return (testBsplinenowayPoint && testBsplineOneWayPoint &&
          testBsplineTwoWayPoint)
             ? 1
             : 0;
}

int main(int argc, char *argv[]) {
  try {
    int ret = PerformTests(argc, argv);
    cout << "return " << ret << endl;
    return ret;
  } catch (const std::string &msg) {
    std::cerr << msg << std::endl;
  }
  return 1;
}

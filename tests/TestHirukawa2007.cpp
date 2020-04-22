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
/* \file This file tests A. Herdt's walking algorithm for
 * automatic foot placement giving an instantaneous CoM velocity reference.
 */
#include "Debug.hh"
#include <MultiContactRefTrajectoryGeneration/MultiContactHirukawa.hh>

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/parser/urdf.hpp"

using namespace std;
using namespace PatternGeneratorJRL;
using namespace Eigen;
using namespace se3;

VectorXd HalfSittingPos(se3::Model model);
void readData(vector<COMState> &comState_, vector<FootAbsolutePosition> &rf_,
              vector<FootAbsolutePosition> &lf_,
              vector<HandAbsolutePosition> &rh_,
              vector<HandAbsolutePosition> &lh_, vector<ZMPPosition> &zmp_);

int main(int argc, char *argv[]) {
  std::string filename;
  if (argc >= 2) {
    filename = argv[1];
  } else {
    filename = "/home/mnaveau/devel/ros_unstable/stacks/inverse_kinematics/"
               "pgmax/hrp2014.urdf";
  }

  std::cout << "Parse filename \"" << filename << "\"" << std::endl;
  se3::Model model = se3::urdf::buildModel(filename, true);
  se3::Data data(model);

  VectorXd q = HalfSittingPos(model);

  vector<COMState> comState_deque;
  vector<FootAbsolutePosition> rf_deque;
  vector<FootAbsolutePosition> lf_deque;
  vector<HandAbsolutePosition> rh_deque;
  vector<HandAbsolutePosition> lh_deque;
  vector<ZMPPosition> zmp_deque;
  readData(comState_deque, rf_deque, lf_deque, rh_deque, lh_deque, zmp_deque);

  MultiContactHirukawa aMCH(&model);
  aMCH.q(q);

  aMCH.oneIteration(comState_deque[0], rf_deque[0], lf_deque[0], rh_deque[0],
                    lh_deque[0]);

  return 1;
}

VectorXd HalfSittingPos(se3::Model model) {
  VectorXd halfsitting = VectorXd::Zero(model.nq);
  ifstream aif;
  aif.open("/home/mnaveau/devel/ros_unstable/install/share/hrp2-14/"
           "HRP2JRLInitConfigSmall.dat",
           ifstream::in);
  if (aif.is_open()) {
    for (int i = 0; i < model.nq; i++) {
      aif >> halfsitting(i);
      halfsitting(i) *= M_PI / 180;
    }
  }
  aif.close();

  halfsitting(2) = 0.6487;
  return halfsitting;
}

void readData(vector<COMState> &comPos_, vector<FootAbsolutePosition> &rf_,
              vector<FootAbsolutePosition> &lf_,
              vector<HandAbsolutePosition> &rh_,
              vector<HandAbsolutePosition> &lh_, vector<ZMPPosition> &zmp_) {
  vector<vector<double> > data_;
  data_.clear();
  std::string astateFile =
      "/home/mnaveau/devel/ros_unstable/src/jrl/jrl-walkgen/_build-RELEASE/"
      "tests/TestMorisawa2007ShortWalk32TestFGPI.datref";
  std::ifstream dataStream;
  dataStream.open(astateFile.c_str(), std::ifstream::in);

  // reading all the data file
  while (dataStream.good()) {
    vector<double> oneLine(74);
    for (unsigned int i = 0; i < oneLine.size(); ++i)
      dataStream >> oneLine[i];
    data_.push_back(oneLine);
  }
  dataStream.close();

  comPos_.resize(data_.size());
  rf_.resize(data_.size());
  lf_.resize(data_.size());
  rh_.resize(data_.size());
  lh_.resize(data_.size());
  zmp_.resize(data_.size());

  for (unsigned int i = 0; i < data_.size(); ++i) {
    comPos_[i].x[0] = data_[i][1];
    comPos_[i].y[0] = data_[i][2];
    comPos_[i].z[0] = data_[i][3];
    comPos_[i].yaw[0] = data_[i][4];
    comPos_[i].x[1] = data_[i][5];
    comPos_[i].y[1] = data_[i][6];
    comPos_[i].z[1] = data_[i][7];

    rf_[i].x = data_[i][22];
    rf_[i].y = data_[i][23];
    rf_[i].z = data_[i][24];
    rf_[i].omega = 0.0;
    rf_[i].omega2 = 0.0;
    rf_[i].theta = 0.0;

    rf_[i].dx = data_[i][25];
    rf_[i].dy = data_[i][26];
    rf_[i].dz = data_[i][27];
    rf_[i].domega = 0.0;
    rf_[i].domega2 = 0.0;
    rf_[i].dtheta = 0.0;

    lf_[i].x = data_[i][10];
    lf_[i].y = data_[i][11];
    lf_[i].z = data_[i][12];
    lf_[i].omega = 0.0;
    lf_[i].omega2 = 0.0;
    lf_[i].theta = 0.0;

    lf_[i].dx = data_[i][13];
    lf_[i].dy = data_[i][14];
    lf_[i].dz = data_[i][15];
    lf_[i].domega = 0.0;
    lf_[i].domega2 = 0.0;
    lf_[i].dtheta = 0.0;

    rh_[i].dx = 0.0;
    rh_[i].dy = 0.0;
    rh_[i].dz = 0.0;
    rh_[i].domega = 0.0;
    rh_[i].domega2 = 0.0;
    rh_[i].dtheta = 0.0;
    rh_[i].stepType = 1.0;

    lh_[i].dx = 0.0;
    lh_[i].dy = 0.0;
    lh_[i].dz = 0.0;
    lh_[i].domega = 0.0;
    lh_[i].domega2 = 0.0;
    lh_[i].dtheta = 0.0;
    lh_[i].stepType = 1.0;
  }
}

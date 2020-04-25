/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
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
/* \file GenerationMotionFromKineoWorks
   This class can inherited for creating the trajectory of the robot's joints
   given a path provided by KineoWorks. This path can be based on a partial
   model of the robot. The link between this partial model is given by
   an auxiliary file, and the redefinition of the virtual
   functions of this class.
*/
#include <fstream>
#include <iostream>

#include <MotionGeneration/GenerateMotionFromKineoWorks.hh>
#include <PreviewControl/PreviewControl.hh>
#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.hh>

#define _DEBUG_
namespace PatternGeneratorJRL {

GenerateMotionFromKineoWorks::GenerateMotionFromKineoWorks() {
  m_NbOfDOFsFromKW = 0;
}

GenerateMotionFromKineoWorks::~GenerateMotionFromKineoWorks() {}

int GenerateMotionFromKineoWorks::ReadPartialModel(string aFileName) {
  std::ifstream aif;

  aif.open(aFileName.c_str(), std::ifstream::in);
  if (aif.is_open()) {
    aif >> m_NbOfDOFsFromKW;
    if (m_NbOfDOFsFromKW < 0) {
      m_NbOfDOFsFromKW = 0;
      return -1;
    }

    m_IndexFromKWToRobot.resize(m_NbOfDOFsFromKW);
    for (int i = 0; i < m_NbOfDOFsFromKW; i++) {
      aif >> m_IndexFromKWToRobot[i];
    }
    aif.close();
  }
  return 0;
}

int GenerateMotionFromKineoWorks::ReadKineoWorksPath(string aFileName) {
  std::ifstream aif;

  aif.open(aFileName.c_str(), std::ifstream::in);
  if (aif.is_open()) {
    string atmp;
    aif >> atmp;
    if (atmp != "KWVersion") {
      cerr << "Not a Kineoworks' path file. Aborted." << endl;
      return -1;
    }

    double version;
    aif >> version;
    if (version != 2.0) {
      cerr << "Not version 2.0. Do not understand the information. "
           << "Aborted. " << endl;
      return -1;
    }
    aif >> atmp;
    if (atmp != "{") {
      cerr << "{ expected. Aborted. " << endl;
      return -1;
    }

    aif >> atmp;
    if (atmp != "SteeringMethodSelect") {
      cerr << "SteeringMethodSelect expected. Aborted. " << endl;
      return -1;
    }

    aif >> m_SteeringMethod;

    aif >> atmp;

    if (atmp != "Nodes") {
      cerr << "Nodes expected. Aborted. " << endl;
      return -1;
    }

    aif >> atmp;

    if (atmp != "[") {
      cerr << "] expected. Aborted. " << endl;
      return -1;
    }

    do {
      aif >> atmp;

      if (atmp[0] == '(') {
        string FirstJt = atmp.substr(1, atmp.length() - 1);
        KWNode aNode;
        aNode.Joints.resize(m_NbOfDOFsFromKW);
        aNode.Joints[0] = atof(FirstJt.c_str());
        for (int j = 1; j < m_NbOfDOFsFromKW; j++) {
          aif >> aNode.Joints[j];
        }
        aif >> atmp; // read ")"
        if (atmp != ")") {
          cerr << " ) expected. Aborted. " << atmp << endl;
          return -1;
        }
        m_Path.insert(m_Path.end(), aNode);
      }
    } while (atmp != "]");

    aif >> atmp;

    if (atmp != "}") {
      cerr << "} expected. Aborted. " << endl;
      return -1;
    }

    aif.close();
  }
  return 0;
}

void GenerateMotionFromKineoWorks::DisplayModelAndPath() {
  cout << "Partial model " << endl;
  cout << "Nb of DOFs: " << m_NbOfDOFsFromKW << endl;
  cout << "Correspondance between local DOFs and robot's ones." << endl;
  for (unsigned int i = 0; i < m_IndexFromKWToRobot.size(); i++) {
    cout << i << " : " << m_IndexFromKWToRobot[i] << endl;
  }

  cout << "KW Path " << endl;
  cout << "Steering Path " << m_SteeringMethod << endl;
  for (unsigned int i = 0; i < m_Path.size(); i++) {
    for (unsigned int j = 0; j < m_Path[i].Joints.size(); j++)
      cout << m_Path[i].Joints[j] << " ";
    cout << endl;
  }
}

void GenerateMotionFromKineoWorks::CreateBufferFirstPreview(
    deque<ZMPPosition> &ZMPRefBuffer) {
  deque<ZMPPosition> aFIFOZMPRefPositions;
  Eigen::MatrixXd aPC1x;
  Eigen::MatrixXd aPC1y;
  double aSxzmp, aSyzmp;
  double aZmpx2, aZmpy2;

  // Initialize local and object scope buffers.
  for (unsigned int i = 0; i < m_NL; i++)
    aFIFOZMPRefPositions.push_back(ZMPRefBuffer[i]);

  m_COMBuffer.resize(ZMPRefBuffer.size() - m_NL);

  // use accumulated zmp error  of preview control so far
  aSxzmp = 0.0; // m_sxzmp;
  aSyzmp = 0.0; // m_syzmp;

  aPC1x.resize(3, 1);
  aPC1y.resize(3, 1);

  aPC1x(0, 0) = 0;
  aPC1x(1, 0) = 0;
  aPC1x(2, 0) = 0;
  aPC1y(0, 0) = 0;
  aPC1y(1, 0) = 0;
  aPC1y(2, 0) = 0;

  // create the extra COMbuffer

#ifdef _DEBUG_
  ofstream aof_COMBuffer;
  static unsigned char FirstCall = 1;
  if (FirstCall) {
    aof_COMBuffer.open("CartCOMBuffer_1.dat", ofstream::out);
  } else {
    aof_COMBuffer.open("CartCOMBuffer_1.dat", ofstream::app);
  }

  if (FirstCall)
    FirstCall = 0;
#endif

  for (unsigned int i = 0; i < ZMPRefBuffer.size() - m_NL; i++) {

    aFIFOZMPRefPositions.push_back(ZMPRefBuffer[i + m_NL]);

    m_PC->OneIterationOfPreview(aPC1x, aPC1y, aSxzmp, aSyzmp,
                                aFIFOZMPRefPositions, 0, aZmpx2, aZmpy2, true);

    for (unsigned j = 0; j < 3; j++) {
      m_COMBuffer[i].x[j] = aPC1x(j, 0);
      m_COMBuffer[i].y[j] = aPC1y(j, 0);
    }

    m_COMBuffer[i].yaw = ZMPRefBuffer[i].theta;

    aFIFOZMPRefPositions.pop_front();

#ifdef _DEBUG_
    if (aof_COMBuffer.is_open()) {
      aof_COMBuffer << ZMPRefBuffer[i].time << " " << ZMPRefBuffer[i].px << " "
                    << m_COMBuffer[i].x[0] << " " << m_COMBuffer[i].y[0]
                    << endl;
    }
#endif
  }

#ifdef _DEBUG_
  if (aof_COMBuffer.is_open()) {
    aof_COMBuffer.close();
  }
#endif
}

void GenerateMotionFromKineoWorks::SetPreviewControl(PreviewControl *aPC) {

  m_PC = aPC;
  m_SamplingPeriod = m_PC->SamplingPeriod();
  m_PreviewControlTime = m_PC->PreviewControlTime();
  if (m_SamplingPeriod == 0)
    m_NL = 0;
  else
    m_NL = (unsigned int)(m_PreviewControlTime / m_SamplingPeriod);
}

void GenerateMotionFromKineoWorks::ComputeUpperBodyPosition(
    deque<KWNode> &UpperBodyPositionsBuffer,
    vector<int> &ConversionFromLocalToRobotDOFsIndex) {
  vector<int> ConversionFromLocalToKW;
  int count = 0;
  //! Find the sizes for the buffer, and the conversion array..
  //! First dimension: count the number of DOFs which are -1
  // inside the array of indexes.
  int NbOfUsedDOFs = 0;
  KWNode deltaJoints;

  for (unsigned int i = 0; i < m_IndexFromKWToRobot.size(); i++)
    if (m_IndexFromKWToRobot[i] != -1)
      NbOfUsedDOFs++;
  ConversionFromLocalToRobotDOFsIndex.resize(NbOfUsedDOFs);
  deltaJoints.Joints.resize(NbOfUsedDOFs);
  ConversionFromLocalToKW.resize(NbOfUsedDOFs);

  //! Second dimension: directly the number of elements inside
  // the COM's buffer of position.
  UpperBodyPositionsBuffer.resize(m_COMBuffer.size());
  for (unsigned int i = 0; i < UpperBodyPositionsBuffer.size(); i++) {
    UpperBodyPositionsBuffer[i].Joints.resize(NbOfUsedDOFs);
  }

  //! First initialize the first upper body position at the
  // beginning of the motion.
  //! and fill the conversion array.
  int k = 0;
  for (unsigned int i = 0; i < m_Path[0].Joints.size(); i++) {
    int IdDOF = 0;
    if ((IdDOF = m_IndexFromKWToRobot[i]) != -1) {

      UpperBodyPositionsBuffer[count].Joints[k] = m_Path[0].Joints[i];
      ConversionFromLocalToRobotDOFsIndex[k] = IdDOF;
      ConversionFromLocalToKW[k] = i;
      k++;
    }
  }
  count++;

  //! For each way-point of the path
  for (unsigned int IdWayPoint = 1; IdWayPoint < m_Path.size(); IdWayPoint++) {
    int CountTarget = -1;
    double lX = 0.0, lY=0.0, lZ=0.0, dist = 1000000.0;

    //! The references are specific to the current hybrid model.
    lX = m_Path[IdWayPoint].Joints[6];
    lY = m_Path[IdWayPoint].Joints[7];
    lZ = m_Path[IdWayPoint].Joints[8];

    //! Find the closest (X,Y,Z) position in the remaining
    // part of the CoM buffer.
    for (unsigned int i = count; i < m_COMBuffer.size(); i++) {
      double ldist = (lX - m_COMBuffer[i].x[0]) * (lX - m_COMBuffer[i].x[0]) +
                     (lY - m_COMBuffer[i].y[0])*(lY-m_COMBuffer[i].y[0]) +
                     (lZ - m_COMBuffer[i].z[0])*(lZ-m_COMBuffer[i].z[0]);
      
      if (ldist < dist) {
        dist = ldist;
        CountTarget = i;
      }
    }

    /* cout << "lX: " << lX <<" lY: " << lY << " lZ: " << lZ << endl
       << dist << " " << CountTarget << " " << count << endl; */
    /*! Create the linear interpolation between the previous
      reference value and the newly found. */

    //! Computes the delta for each joint and for each 0.005 ms
    for (unsigned int i = 0; i < ConversionFromLocalToRobotDOFsIndex.size();
         i++) {
      // int j = ConversionFromLocalToRobotDOFsIndex[i];
      int k = ConversionFromLocalToKW[i];

      deltaJoints.Joints[i] =
          (m_Path[IdWayPoint].Joints[k] - m_Path[IdWayPoint - 1].Joints[k]) /
          (double)(CountTarget - count);
    }

    //! Fill the buffer with linear interpolation.
    while (count <= CountTarget) {
      for (unsigned int i = 0; i < ConversionFromLocalToRobotDOFsIndex.size();
           i++) {
        UpperBodyPositionsBuffer[count].Joints[i] =
            UpperBodyPositionsBuffer[count - 1].Joints[i] +
            deltaJoints.Joints[i];
      }

      count++;
    }
  }
}

} // namespace PatternGeneratorJRL

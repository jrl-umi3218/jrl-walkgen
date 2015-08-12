/*
 * Copyright 2009, 2010, 2014
 *
 * Maximilien Naveau
 * Olivier  Stasse,
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
/*! \file TestNaveau2015.cpp
  \brief This Example shows you how to use the nmpc_genereator.cpp */
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl/dynamics/dynamicsfactory.hh>
#include <jrl/walkgen/patterngeneratorinterface.hh>
#include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>
#include <ZMPRefTrajectoryGeneration/nmpc_generator.hh>

using namespace std;
using namespace PatternGeneratorJRL;

int main()
{
#ifdef WITH_HRP2DYNAMICS
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
  Chrp2OptHumanoidDynamicRobot * aHRP2HDR = new Chrp2OptHumanoidDynamicRobot( &aRobotDynamicsObjectConstructor );
  Chrp2OptHumanoidDynamicRobot * aHDR = aHRP2HDR ;
#endif

  string RobotFileName ("/home/mnaveau/devel/ros_unstable/install/share/hrp2-14/HRP2LAAS_main_small.wrl");
  string LinkJointRank ("/home/mnaveau/devel/ros_unstable/install/share/hrp2-14/HRP2LinkJointRankSmall.xml");
  string SpecificitiesFileName ("/home/mnaveau/devel/ros_unstable/install/share/hrp2-14/HRP2SpecificitiesSmall.xml");
  string InitConfigFileName ("/home/mnaveau/devel/ros_unstable/install/share/hrp2-14/HRP2JRLInitConfigSmall.dat");


  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHRP2HDR,
                                         RobotFileName,
                                         LinkJointRank,
                                         SpecificitiesFileName);
  PatternGeneratorInterface * aPGI = patternGeneratorInterfaceFactory(aHRP2HDR);

  vector<CjrlJoint *> actuatedJoints = aHDR->getActuatedJoints();
  unsigned int lNbActuatedJoints = actuatedJoints.size();

  double * dInitPos = new double[lNbActuatedJoints];

  ifstream aif;
  aif.open(InitConfigFileName.c_str(),ifstream::in);
  if (aif.is_open())
  {
    for(unsigned int i=0;i<lNbActuatedJoints;i++)
      aif >> dInitPos[i];
  }
  aif.close();
  MAL_VECTOR_DIM(InitialPosition,double,lNbActuatedJoints);
  for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
    InitialPosition(i) = dInitPos[i]*M_PI/180.0;
  aPGI->SetCurrentJointValues(InitialPosition);
  istringstream strm2(":walkmode 0");
  aPGI->ParseCmd(strm2);

  delete [] dInitPos;

  SimplePluginManager * aSPM = new SimplePluginManager();

  NMPC_generator nmpc_generator (aSPM,aHRP2HDR) ;
  vector<double>local_vel_ref(3,0.0);
  local_vel_ref[0] = 0.2 ;
  local_vel_ref[1] = 0.0 ;
  local_vel_ref[2] = 0.2 ;
  nmpc_generator.initNMPC_generator();


  for(unsigned i=0 ; i<10 ; ++i)
  {
    nmpc_generator.setVelocityReference(local_vel_ref);
    nmpc_generator.solve();
  }

  return 0;
}



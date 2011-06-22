/*
 * Copyright 2011
 *
 * Andrei Herdt
 *
 * JRL, CNRS/AIST, INRIA Grenoble-Rhone-Alpes
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
 */

/// Simulate a rigid body

#include <PreviewControl/rigid-body-system.hh>

using namespace PatternGeneratorJRL;
using namespace std;

RigidBodySystem::RigidBodySystem()
{

}


RigidBodySystem::~RigidBodySystem()
{
  
}


void
RigidBodySystem::initialize()
{

  RigidBody::linear_dynamics_t & Velocity = CoM_.Dynamics( RigidBody::VELOCITY );
  initialize_dynamics( Velocity );
  RigidBody::linear_dynamics_t & COP = CoM_.Dynamics( RigidBody::COP );
  initialize_dynamics( COP );
  RigidBody::linear_dynamics_t & Jerk = CoM_.Dynamics( RigidBody::JERK );
  initialize_dynamics( Jerk );

}


void
RigidBodySystem::initialize_dynamics( RigidBody::linear_dynamics_t & Dynamics)
{

  bool preserve = true;
  Dynamics.U.resize(N_,N_,!preserve);
  Dynamics.U.clear();
  Dynamics.UT.resize(N_,N_,!preserve);
  Dynamics.UT.clear();
  Dynamics.S.resize(N_,3,!preserve);
  Dynamics.S.clear();

  switch(Dynamics.Type)
    {
    case RigidBody::VELOCITY:
      for(unsigned int i=0;i<N_;i++)
        {
          Dynamics.S(i,0) = 0.0; Dynamics.S(i,1) = 1.0; Dynamics.S(i,2) = (i+1)*T_;
          for(unsigned int j=0;j<N_;j++)
            if (j<=i)
              Dynamics.U(i,j) = Dynamics.UT(j,i) = (2*(i-j)+1)*T_*T_*0.5 ;
            else
              Dynamics.U(i,j) = Dynamics.UT(j,i) = 0.0;
        }
      break;

    case RigidBody::COP:
      for(unsigned int i=0;i<N_;i++)
        {
          Dynamics.S(i,0) = 1.0; Dynamics.S(i,1) = (i+1)*T_; Dynamics.S(i,2) = (i+1)*(i+1)*T_*T_*0.5-CoMHeight_/9.81;
          for(unsigned int j=0;j<N_;j++)
            if (j<=i)
              Dynamics.U(i,j) = Dynamics.UT(j,i) = (1 + 3*(i-j) + 3*(i-j)*(i-j)) * T_*T_*T_/6.0 - T_*CoMHeight_/9.81;
            else
              Dynamics.U(i,j) = Dynamics.UT(j,i) = 0.0;
        }
      break;

    case RigidBody::JERK:
      for(unsigned int i=0;i<N_;i++)
        {
          Dynamics.S(i,0) = 0.0; Dynamics.S(i,1) = 0.0; Dynamics.S(i,2) = 0.0;
          for(unsigned int j=0;j<N_;j++)
            if (j==i)
              Dynamics.U(i,j) = Dynamics.UT(j,i) = 1.0;
            else
              Dynamics.U(i,j) = Dynamics.UT(j,i) = 0.0;
        }
      break;

    }

}


int 
RigidBodySystem::interpolate(deque<COMState> &COMStates,
						deque<ZMPPosition> &ZMPRefPositions,
						int CurrentPosition,
						double CX, double CY)
{
  
  return 0;
  
}


void
RigidBodySystem::increment_state(double Control)
{

}

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

#include <PreviewControl/rigid-body.hh>

using namespace PatternGeneratorJRL;
using namespace std;

RigidBody::RigidBody():
    T_(0),Ta_(0),N_(0),Mass_(0)
{

}


RigidBody::~RigidBody()
{
  
}


int 
RigidBody::initialize()
{
  
  return 0;

}


int 
RigidBody::interpolate(deque<COMState> &COMStates,
						deque<ZMPPosition> &ZMPRefPositions,
						int CurrentPosition,
						double CX, double CY)
{
  
  return 0;
  
}


RigidBody::rigid_body_state_t
RigidBody::increment_state(double Control)
{

  return State_;
  
}


// ACCESSORS:
// ----------
RigidBody::linear_dynamics_t const &
RigidBody::Dynamics( const int type ) const
{
  switch(type)
  {
  case VELOCITY:
    return VelocityDynamics_;
  case JERK:
    return JerkDynamics_;
  }

  // Default
  return VelocityDynamics_;
}

RigidBody::linear_dynamics_t &
RigidBody::Dynamics( const int type )
{
  switch(type)
  {
  case VELOCITY:
    return VelocityDynamics_;
  case JERK:
    return JerkDynamics_;
  }

  // Default
  return VelocityDynamics_;
}


// INTERNAL TYPE METHODS:
// ----------------------
RigidBody::rigid_body_state_s::rigid_body_state_s()
{

  reset();

}


struct RigidBody::rigid_body_state_s &
RigidBody::rigid_body_state_t::operator=(const RigidBody::rigid_body_state_s & RB)
{

  for(unsigned int i=0;i<3;i++)
    {
      X(i) = RB.X(i);
      Y[i] = RB.Y[i];
      Z[i] = RB.Z[i];

      Yaw[i] = RB.Yaw[i];
      Pitch[i] = RB.Pitch[i];
      Roll[i] = RB.Roll[i];
    };
  return *this;

}


void 
RigidBody::rigid_body_state_t::reset()
{

  // Redimension
  X.resize(3,false);
  Y.resize(3,false);
  Z.resize(3,false);
  Yaw.resize(3,false);
  Pitch.resize(3,false);
  Roll.resize(3,false);
  // Set to zero
  X.clear();
  Y.clear();
  Z.clear();
  Yaw.clear();
  Pitch.clear();
  Roll.clear();

}

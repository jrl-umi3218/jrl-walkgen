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
  T_(0),Tr_(0),Ta_(0),N_(16),Mass_(0)
{

  PositionDynamics_.Type = POSITION;
  VelocityDynamics_.Type = VELOCITY;
  AccelerationDynamics_.Type = ACCELERATION;
  JerkDynamics_.Type = JERK;

}


RigidBody::~RigidBody()
{

}


int RigidBody::initialize()
{

  PositionDynamics_.S.resize( N_,3);
  PositionDynamics_.S.setZero();
  VelocityDynamics_.S.resize( N_,3 );
  VelocityDynamics_.S.setZero();
  AccelerationDynamics_.S.resize( N_,3);
  AccelerationDynamics_.S.setZero();
  CoPDynamics_.S.resize( N_,3);
  CoPDynamics_.S.setZero();

  Trajectory_.resize( N_ );

  return 0;

}

// TODO: RigidBody::interpolate RigidBody::increment_state
//int
//RigidBody::interpolate(deque<COMState> &COMStates,
//                                        deque<ZMPPosition> &ZMPRefPositions,
//                                        int CurrentPosition,
//                                        double CX, double CY)
//{
//
//  return 0;
//
//}
//
//
//RigidBody::rigid_body_state_t
//RigidBody::increment_state(double Control)
//{
//
//  return State_;
//
//}


// ACCESSORS:
// ----------
linear_dynamics_t const &
RigidBody::Dynamics( dynamics_e type ) const
{

  switch(type)
    {
    case POSITION:
      return PositionDynamics_;
    case VELOCITY:
      return VelocityDynamics_;
    case ACCELERATION:
      return AccelerationDynamics_;
    case JERK:
      return JerkDynamics_;
    case COP_POSITION:
      return CoPDynamics_;
    }

  return VelocityDynamics_;

}

linear_dynamics_t &
RigidBody::Dynamics( dynamics_e type )
{

  switch(type)
    {
    case POSITION:
      return PositionDynamics_;
    case VELOCITY:
      return VelocityDynamics_;
    case ACCELERATION:
      return AccelerationDynamics_;
    case JERK:
      return JerkDynamics_;
    case COP_POSITION:
      return CoPDynamics_;
    }

  return VelocityDynamics_;

}


//
// Private methods
//


//
// Internal type methods:
//
rigid_body_state_s::rigid_body_state_s()
{

  reset();

}


struct rigid_body_state_s &
rigid_body_state_t::operator=(const rigid_body_state_s & RB)
{

  for(unsigned int i=0; i<3; i++)
    {
      X[i] = RB.X[i];
      Y[i] = RB.Y[i];
      Z[i] = RB.Z[i];

      Yaw[i] = RB.Yaw[i];
      Pitch[i] = RB.Pitch[i];
      Roll[i] = RB.Roll[i];
    };
  return *this;

}


void
rigid_body_state_t::reset()
{

  X.resize(3);
  Y.resize(3);
  Z.resize(3);
  Yaw.resize(3);
  Pitch.resize(3);
  Roll.resize(3);

  X.setZero();
  Y.setZero();
  Z.setZero();
  Yaw.setZero();
  Pitch.setZero();
  Roll.setZero();

}

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
        T_(0),Tr_(0),Ta_(0),N_(0),Mass_(0)
{

  PositionDynamics_.Type = POSITION;
  VelocityDynamics_.Type = VELOCITY;
  AccelerationDynamics_.Type = ACCELERATION;
  JerkDynamics_.Type = JERK;

}


RigidBody::~RigidBody()
{

}


int
RigidBody::initialize()
{

  // Initialize dynamics
  // -------------------
  initialize_dynamics( PositionDynamics_ );
  initialize_dynamics( VelocityDynamics_ );
  initialize_dynamics( JerkDynamics_ );

  return 0;

}


int
RigidBody::initialize_dynamics( linear_dynamics_t & Dynamics )
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
  case POSITION:
    for(unsigned int i=0;i<N_;i++)
      {
        Dynamics.S(i,0) = 1; Dynamics.S(i,1) =(i+1)* T_; Dynamics.S(i,2) = ((i+1)* T_)*((i+1)* T_)/2;
        for(unsigned int j=0;j<N_;j++)
          if (j<=i)
            Dynamics.U(i,j) = Dynamics.UT(j,i) =(1+3*(i-j)+3*(i-j)*(i-j))*(T_*T_*T_)/6 ;
          else
            Dynamics.U(i,j) = Dynamics.UT(j,i) = 0.0;
      }
    break;
  case VELOCITY:
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
  case ACCELERATION:
    break;
  case JERK:
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
  case COP:
    break;

  }

  return 0;

}


// TODO: RigidBody::interpolate RigidBody::increment_state
//int
//RigidBody::interpolate(deque<COMState> &COMStates,
//						deque<ZMPPosition> &ZMPRefPositions,
//						int CurrentPosition,
//						double CX, double CY)
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
RigidBody::Dynamics( DynamicsType Type ) const
{

  switch(Type)
  {
  case POSITION:
    return PositionDynamics_;
  case VELOCITY:
    return VelocityDynamics_;
  case ACCELERATION:
    return AccelerationDynamics_;
  case JERK:
    return JerkDynamics_;
  case COP:
    break;
  }

  return VelocityDynamics_;

}

linear_dynamics_t &
RigidBody::Dynamics( DynamicsType Type )
{

  switch(Type)
  {
  case POSITION:
    return PositionDynamics_;
  case VELOCITY:
    return VelocityDynamics_;
  case ACCELERATION:
    return AccelerationDynamics_;
  case JERK:
    return JerkDynamics_;
  case COP:
    break;
  }

  return VelocityDynamics_;

}


// INTERNAL TYPE METHODS:
// ----------------------
rigid_body_state_s::rigid_body_state_s()
{

  reset();

}


struct rigid_body_state_s &
rigid_body_state_t::operator=(const rigid_body_state_s & RB)
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
rigid_body_state_t::reset()
{

  X.resize(3,false);
  Y.resize(3,false);
  Z.resize(3,false);
  Yaw.resize(3,false);
  Pitch.resize(3,false);
  Roll.resize(3,false);

  X.clear();
  Y.clear();
  Z.clear();
  Yaw.clear();
  Pitch.clear();
  Roll.clear();

}

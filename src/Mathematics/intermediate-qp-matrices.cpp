/*
 * Copyright 2011,
 *
 * Andrei   Herdt
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
/*! Custom (value based) container providing intermediate elements for the construction of a QP.
 */

#include <Mathematics/intermediate-qp-matrices.hh>

using namespace std;
using namespace PatternGeneratorJRL;


IntermedQPMat::IntermedQPMat()
{

  m_MeanVelocity.type = MEAN_VELOCITY;
  m_Velocity.type = VELOCITY;
  m_MeanVelocity.dyn = & m_Velocity;

  m_InstantVelocity.type = INSTANT_VELOCITY;
  m_InstantVelocity.dyn = & m_Velocity;

  m_COPCentering.type = COP_CENTERING;
  m_CoP.type = COP;
  m_COPCentering.dyn = & m_CoP;

  m_JerkMin.type = JERK_MIN;
  m_Jerk.type = JERK;
  m_JerkMin.dyn = & m_Jerk;

  m_Position.type = POSITION;
  m_Acceleration.type = ACCELERATION;

  m_IneqCoP.type = INEQ_COP;
  m_IneqCoM.type = INEQ_COM;
  m_IneqFeet.type = INEQ_FEET;

}


IntermedQPMat::~IntermedQPMat()
{
}


IntermedQPMat::objective_variant_t const &
IntermedQPMat::Objective( const int type ) const
{
  switch(type)
    {
    case MEAN_VELOCITY:
      return m_MeanVelocity;
    case INSTANT_VELOCITY:
      return m_InstantVelocity;
    case COP_CENTERING:
      return m_COPCentering;
    case JERK_MIN:
      return m_JerkMin;
    }
  /* Default behavior return Mean velocity. */
  return m_MeanVelocity;
}
IntermedQPMat::objective_variant_t &
IntermedQPMat::Objective( const int type )
{
  switch(type)
    {
    case MEAN_VELOCITY:
      return m_MeanVelocity;
    case INSTANT_VELOCITY:
      return m_InstantVelocity;
    case COP_CENTERING:
      return m_COPCentering;
    case JERK_MIN:
      return m_JerkMin;
    }
  /* Default behavior return Mean velocity. */
  return m_MeanVelocity;
}


IntermedQPMat::dynamics_t const &
IntermedQPMat::Dynamics( const int type ) const
{
  switch(type)
    {
    case VELOCITY:
      return m_Velocity;
    case COP:
      return m_CoP;
    case JERK:
      return m_Jerk;
    }
  /* Default behavior return velocity. */
  return m_Velocity;
}
IntermedQPMat::dynamics_t &
IntermedQPMat::Dynamics( const int type )
{
  switch(type)
    {
    case VELOCITY:
      return m_Velocity;
    case COP:
      return m_CoP;
    case JERK:
      return m_Jerk;
    }
  /*! Default behavior return velocity. */
  return m_Velocity;
}


linear_inequality_t const &
IntermedQPMat::Inequalities( const int type ) const
{
  switch(type)
    {
    case INEQ_COP:
      return m_IneqCoP;
    case INEQ_COM:
      return m_IneqCoM;
    case INEQ_FEET:
      return m_IneqFeet;
    }
  /* Default behavior return an inequality on CoP */
  return m_IneqCoP;
}

linear_inequality_t &
IntermedQPMat::Inequalities( const int type )
{
  switch(type)
    {
    case INEQ_COP:
      m_IneqCoP.clear();
      return m_IneqCoP;
    case INEQ_COM:
      m_IneqCoM.clear();
      return m_IneqCoM;
    case INEQ_FEET:
      m_IneqFeet.clear();
      return m_IneqFeet;
    }
  /* Default behavior return an inequality on CoP */
  m_IneqCoP.clear();
  return m_IneqCoP;
}


void
IntermedQPMat::dumpObjective( const int aObjectiveType, std::ostream &aos )
{

  switch(aObjectiveType)
    {
    case INSTANT_VELOCITY:
      m_InstantVelocity.print(aos);
      break;

    case JERK_MIN:
      m_JerkMin.print(aos);
      break;

    case COP_CENTERING:
      m_COPCentering.print(aos);
      break;
    }

}


void
IntermedQPMat::dumpState( std::ostream &aos )
{
  aos << "dumpState" << std::endl;
  aos << "=========" << std::endl;
}


void
IntermedQPMat::dumpObjective(const char * filename,
			     const int type)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  dumpObjective(type,aof);
  aof.close();
}


void
IntermedQPMat::dumpState(const char * filename)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  dumpState(aof);
  aof.close();
}

std::ostream&
IntermedQPMat::objective_variant_s::print (std::ostream& o) const
{
  o   << endl
      << "weight: " << weight << endl
      << "U: " << dyn->U << endl << endl
      << "UT: " << dyn->UT << endl<< endl
      << "S: " << dyn->S << endl;

  return o ;
}

void
IntermedQPMat::objective_variant_s::dump(const char * filename) const
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  print(aof);
  aof.close();
}

std::ostream&
operator << (std::ostream& o, const IntermedQPMat::objective_variant_s& r)
{
  return r.print(o);
}


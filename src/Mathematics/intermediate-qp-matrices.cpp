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
/*! \brief Custom (value based) container providing intermediate elements 
  for the construction of a QP.
 */

#include <Mathematics/intermediate-qp-matrices.hh>

using namespace std;
using namespace PatternGeneratorJRL;


IntermedQPMat::IntermedQPMat()
{

  InstantVelocity_.type = INSTANT_VELOCITY;
  COPCentering_.type = COP_CENTERING;
  JerkMin_.type = JERK_MIN;

  IneqCoP_.type = INEQ_COP;
  IneqCoM_.type = INEQ_COM;
  IneqFeet_.type = INEQ_FEET;

}


IntermedQPMat::~IntermedQPMat()
{
}


IntermedQPMat::objective_variant_t const &
IntermedQPMat::Objective( objective_e type ) const
{
  switch(type)
    {
    case INSTANT_VELOCITY:
      return InstantVelocity_;
    case COP_CENTERING:
      return COPCentering_;
    case JERK_MIN:
      return JerkMin_;
    }
  /* Default behavior return Mean velocity. */
  return MeanVelocity_;
}

IntermedQPMat::objective_variant_t &
IntermedQPMat::Objective( objective_e type )
{
  switch(type)
    {
    case INSTANT_VELOCITY:
      return InstantVelocity_;
    case COP_CENTERING:
      return COPCentering_;
    case JERK_MIN:
      return JerkMin_;
    }
  /* Default behavior return Mean velocity. */
  return MeanVelocity_;
}

linear_inequality_t const &
IntermedQPMat::Inequalities( ineq_e type ) const
{
  switch(type)
    {
    case INEQ_COP:
      return IneqCoP_;
    case INEQ_COM:
      return IneqCoM_;
    case INEQ_FEET:
      return IneqFeet_;
    }
  /* Default behavior return an inequality on CoP */
  return IneqCoP_;
}

linear_inequality_t &
IntermedQPMat::Inequalities( ineq_e type )
{
  switch(type)
    {
    case INEQ_COP:
      IneqCoP_.clear();
      return IneqCoP_;
    case INEQ_COM:
      IneqCoM_.clear();
      return IneqCoM_;
    case INEQ_FEET:
      IneqFeet_.clear();
      return IneqFeet_;
    }
  /* Default behavior return an inequality on CoP */
  IneqCoP_.clear();
  return IneqCoP_;
}


void
IntermedQPMat::dump_objective( objective_e type, std::ostream &aos )
{

  switch(type)
    {
    case INSTANT_VELOCITY:
      InstantVelocity_.print(aos);
      break;

    case JERK_MIN:
      JerkMin_.print(aos);
      break;

    case COP_CENTERING:
      COPCentering_.print(aos);
      break;
    }

}


void
IntermedQPMat::dump_state( std::ostream &aos )
{
  aos << "dumpState" << std::endl;
  aos << "=========" << std::endl;
}


void
IntermedQPMat::dump_objective(const char * filename,
                              objective_e type)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  dump_objective(type,aof);
  aof.close();
}


void
IntermedQPMat::dump_state(const char * filename)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  dump_state(aof);
  aof.close();
}

std::ostream&
IntermedQPMat::objective_variant_s::print (std::ostream& o) const
{
  o   << endl
      << "weight: " << weight << endl;

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


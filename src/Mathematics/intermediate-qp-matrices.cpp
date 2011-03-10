/*
 * Copyright 2010,
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
/*! This object provides intermediate elements and operations for the construction of a QP.
 */



#include <Mathematics/intermediate-qp-matrices.hh>

using namespace PatternGeneratorJRL;



IntermedQPMat::IntermedQPMat()
{

  m_MeanVelocity.type = MEAN_VELOCITY;
  m_InstantVelocity.type = INSTANT_VELOCITY;
  m_COPCentering.type = COP_CENTERING;
  m_Jerk.type = JERK;

}


IntermedQPMat::~IntermedQPMat()
{
  //TODO:
}


IntermedQPMat::objective_variant_t const &
IntermedQPMat::operator ()( const int aObjectiveType ) const
{
  switch(aObjectiveType)
  {
    case MEAN_VELOCITY:
      return m_MeanVelocity;
    case INSTANT_VELOCITY:
      return m_InstantVelocity;
    case COP_CENTERING:
      return m_COPCentering;
    case JERK:
      return m_Jerk;
  }
}


IntermedQPMat::objective_variant_t &
IntermedQPMat::operator ()( const int aObjectiveType )
{
  switch(aObjectiveType)
  {
    case MEAN_VELOCITY:
      return m_MeanVelocity;
    case INSTANT_VELOCITY:
      return m_InstantVelocity;
    case COP_CENTERING:
      return m_COPCentering;
    case JERK:
      return m_Jerk;
  }
}


void
IntermedQPMat::printObjective( const int ObjectiveType, std::ostream &aos )
{
  IntermedQPMat::objective_variant_t objective;
  switch(ObjectiveType)
    {
    case INSTANT_VELOCITY:
      objective = m_InstantVelocity;
      break;
    case JERK:
      objective = m_Jerk;
      break;
    case COP_CENTERING:
      objective = m_COPCentering;
      break;
    }

  aos << "Ponderation: " << std::endl;
  aos << objective.weight << std::endl<< std::endl;
  aos << "U: " << std::endl;
  aos << objective.U << std::endl<< std::endl;
  aos << "trans(U): " << std::endl;
  aos << objective.UT << std::endl<< std::endl;
  aos << "S: " << std::endl;
  aos << objective.S << std::endl<< std::endl;
}


void
IntermedQPMat::printState( std::ostream &aos )
{

}


void
IntermedQPMat::printObjective(const char * filename,
                          const int type)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  printObjective(type,aof);
  aof.close();
}


void
IntermedQPMat::printState(const char * filename)
{
  std::ofstream aof;
  aof.open(filename,std::ofstream::out);
  printState(aof);
  aof.close();
}



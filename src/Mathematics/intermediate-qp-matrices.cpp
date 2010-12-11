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



#include <Mathematics/intermediate-qp-matrices.hh> //TODO: Merge with ZMPVelocityReferencedQPDebug.cpp


using namespace PatternGeneratorJRL;



IntermedQPMat::IntermedQPMat()
{
  //TODO:
}


IntermedQPMat::~IntermedQPMat()
{
  //TODO:
}


void 
IntermedQPMat::getLSTerm(ObjectiveTerm_t LST, int ObjectiveType)
{
  //TODO:
}
	  
	
//
//Private methods
//
void 
IntermedQPMat::weightedTransProd(MAL_MATRIX (&M1, double), double alpha, MAL_MATRIX (&M2, double), MAL_MATRIX (&M3, double))
{
  //TODO:
}
	  

void 
IntermedQPMat::weightedTransProd(MAL_MATRIX (&M1, double), double alpha, MAL_MATRIX (&M2, double), MAL_VECTOR (&V, double))
{
  //TODO:
}
	  

void 
IntermedQPMat::weightedTransProd(MAL_MATRIX (&M1, double), double alpha, MAL_VECTOR (&M2, double), MAL_VECTOR (&M3, double), MAL_VECTOR (&V, double))
{
  //TODO:
}
	  

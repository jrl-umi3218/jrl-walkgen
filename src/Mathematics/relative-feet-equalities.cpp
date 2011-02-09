/*
 * Copyright 2010,
 *
 * Mehdi      Benallegue
 * Andrei     Herdt
 * Olivier    Stasse
 *
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
/** \file FootConstraintAsLinearSystemForVelRef.cpp
    \brief This object builds linear constraints relative to the current and the previewed feet positions.
*/

#include <iostream>
#include <fstream>

#include <Mathematics/relative-feet-equalities.hh>

using namespace std;
using namespace PatternGeneratorJRL;

#include <Debug.h>



RelativeFeetEqualities::RelativeFeetEqualities( )
 
{

	stepWidth_ = 0.195;//meters	

	
}

RelativeFeetEqualities::~RelativeFeetEqualities()
{

}



int
RelativeFeetEqualities::setVertice(RelativeStepPosition& StepOut, 
								   const RelativeStepPosition& StepIn, 
								   const support_state_t & PrwSupport,
								   double Orientation)const
{

	int parity; //left=1 or right=-1 
	//Prepare the computation of the convex hull
	if( PrwSupport.Foot == 1 )
	{
		parity=1;
	}
	else
	{
		parity=-1;
	}

	double zero=parity*stepWidth_;

	// rotate(Orientation);
	double c_a = cos(Orientation);
	double s_a = sin(Orientation);

	StepOut.x = ( StepIn.x*c_a - (StepIn.y+zero)*s_a );
	StepOut.y = ( StepIn.x*s_a + (StepIn.y+zero)*c_a );
	StepOut.theta = StepIn.theta+Orientation;
	return 0;

}

void RelativeFeetEqualities::setStepWidth(double width)
{
	stepWidth_=width;
}


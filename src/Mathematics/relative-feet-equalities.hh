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
/* This object generate matrix representation of linear
constraint based on foot position.
It handles a stack of constraint on a sliding mode 
for QP solving. */

#ifndef _RELATIVE_FEET_EQUALITIES_
#define _RELATIVE_FEET_EQUALITIES_

#include <string>
#include <sstream>

#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl/walkgen/pgtypes.hh>
#include <privatepgtypes.h>


namespace PatternGeneratorJRL
{

	/// \brief Generate a stack of equalities relative to feet positionning constraints
	class RelativeFeetEqualities
	{


		//
		// Public member functions
		//
	public:

		/// \name Constructors and destructors.
		/// \{
		RelativeFeetEqualities ();
		~RelativeFeetEqualities ();
		/// \}


		/// \brief Adapt vertice to the support foot and its orientation
		///
		/// \param[out] StepOut the relative support foot adapted to the support and orientation
		/// \param[in] StepIn the original relqtive step position
		/// \param[in] PrwSupport Support State
		/// \param[in] Orientation
		/// \return 0
		int setVertice(RelativeStepPosition& StepOut, 
			const RelativeStepPosition& StepIn, 
			const support_state_t & PrwSupport,
			double Orientation) const;

		/// \brief Sets the distance between feet in zero position
		///
		/// \param[in] width in meters
		void setStepWidth(double width);
		
		//
		// Private members
		//
	private:


		double stepWidth_;

	};
}
#endif /* _RELATIVE_FEET_EQUALITIES_ */

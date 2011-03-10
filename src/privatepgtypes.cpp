/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010, 
 *
 * Andrei Herdt
 * Olivier Stasse
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
#include <privatepgtypes.h>

namespace PatternGeneratorJRL
{

  struct com_s & com_t::operator=(const com_s &aCS)
  {
    for(unsigned int i=0;i<3;i++)
      {
	x[i] = aCS.x[i];
	y[i] = aCS.y[i];
	z[i] = aCS.z[i];
      };
    return *this;
  }
      
  void com_t::reset()
  {
    for(unsigned int i=0;i<3;i++)
      { 
        MAL_VECTOR_RESIZE(x,3);
        MAL_VECTOR_RESIZE(y,3);
        MAL_VECTOR_RESIZE(z,3);
	x[i] = 0.0;
	y[i] = 0.0;
	z[i] = 0.0;
      }
  }

  com_s::com_s()
  {
    reset();
  }

}

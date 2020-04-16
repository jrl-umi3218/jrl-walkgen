/*
 * Copyright 2011,
 *
 * Medhi    Benallegue
 * Andrei   Herdt
 * Francois Keith
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
#ifndef _QP_PROBLEM_HXX_
#define _QP_PROBLEM_HXX_

namespace PatternGeneratorJRL {

/// \brief Reallocate array
///
/// \param[in] array
/// \param[in] old_size
/// \param[in] new_size
template <typename type> int resize(type *&array, int old_size, int new_size) {
  try {
    type *NewArray = new type[new_size];
    initialize(NewArray, new_size, (type)0);
    for (int i = 0; i < old_size; i++) {
      NewArray[i] = array[i];
    }
    if (array != 0)
      delete[] array;
    array = NewArray;
  } catch (std::bad_alloc &ba) {
    std::cerr << "bad_alloc caught: " << ba.what() << std::endl;
  }
  return 0;
}

} // namespace PatternGeneratorJRL

#endif /* _QP_PROBLEM_HXX_ */

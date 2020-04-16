/*
 * Copyright 2007, 2008, 2009, 2010,
 *
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
/** \file ConvexHull.h
    \brief This object allow to build a convex hull on a list of 2D points. */
#ifndef _CONVEX_HULL_COMPUTATION_H_
#define _CONVEX_HULL_COMPUTATION_H_

#include <vector>

namespace PatternGeneratorJRL {

/*! @struct CH_Point
  @ingroup geometry
  Structure to store points for the convex hull computation.
*/
typedef struct {
  double col, row; /* col: x, row : y */
} CH_Point;

typedef std::vector<CH_Point> ConvexHullList;

/*! This class compute the convex hull in 2D. */
class ComputeConvexHull {
public:
  ComputeConvexHull();
  ~ComputeConvexHull();

  /*! Compute the convex hull
    by applying Graham's algorithm.
    @param aVecOfPoints:
    The set of 2D points on which the convex hull is computed.
    @param TheConvexHull:
    The set of 2D points which give the convex hull. */
  void DoComputeConvexHull(std::vector<CH_Point> aVecOfPoints,
                           std::vector<CH_Point> &TheConvexHull);
};
} // namespace PatternGeneratorJRL
#endif

/** \file ConvexHull.h
    \brief This object allow to build a convex hull on a list of 2D points.


   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Francois Keith
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see License.txt for further information on license.
*/
#ifndef _CONVEX_HULL_COMPUTATION_H_
#define _CONVEX_HULL_COMPUTATION_H_

#include <vector>


namespace PatternGeneratorJRL
{

  /*! @struct CH_Point
      @ingroup geometry
      Structure to store points for the convex hull computation.
  */
  typedef struct 
  {
    float col,row;  /* col: x, row : y */
  } CH_Point;
  
  typedef std::vector<CH_Point> ConvexHullList;

  /*! This class compute the convex hull in 2D. */
  class  ComputeConvexHull
  {
  public: 
    ComputeConvexHull();
    ~ComputeConvexHull();

    /*! Compute the convex hull
      by applying Graham's algorithm. 
    @param aVecOfPoints: The set of 2D points on which the convex hull is computed. 
    @param TheConvexHull: The set of 2D points which give the convex hull. */
    void DoComputeConvexHull(std::vector<CH_Point> aVecOfPoints,
		      std::vector<CH_Point> &TheConvexHull);
    
  };
};
#endif

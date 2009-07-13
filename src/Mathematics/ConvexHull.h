/** \file ConvexHull.h
    \brief This object allow to build a convex hull on a list of 2D points.

    SVN Information:
   $Id$
   $Author$
   $Date$
   $Revision $
   $Source $
   $Log $


   Copyright (c) 2005-2006, 
   @author Francois Keith, Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS and AIST nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef _CONVEX_HULL_COMPUTATION_H_
#define _CONVEX_HULL_COMPUTATION_H_

#include <vector>
#include <walkGenJrl/walkGenJrl_API.h>

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
  class WALK_GEN_JRL_EXPORT ComputeConvexHull
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

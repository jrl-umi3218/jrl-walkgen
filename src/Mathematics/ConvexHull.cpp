/*
 * Copyright 2007, 2008, 2009, 2010,
 *
 * Mehdi Benallegue
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
#include <set>
#include <math.h>

#include <Mathematics/ConvexHull.hh>
#include <Debug.hh>

using namespace std;

namespace PatternGeneratorJRL
{
  CH_Point HRP2CIO_GlobalP0;
  struct ltCH_Point
  {
    bool operator() (const CH_Point & s1, const CH_Point &s2)
    {
      double x1,x2,y1,y2;
      x1 = s1.col - HRP2CIO_GlobalP0.col;
      x2 = s2.col - HRP2CIO_GlobalP0.col;
      y1 = s1.row - HRP2CIO_GlobalP0.row;
      y2 = s2.row - HRP2CIO_GlobalP0.row;

      return (x1*y2 - x2*y1)>0.0;
    }
  };

  void DistanceCHRep(CH_Point & s1,CH_Point & s2,
                     double & distance1, double &distance2)
  {
    double x1,x2,y1,y2;
    x1 = s1.col - HRP2CIO_GlobalP0.col;
    x2 = s2.col - HRP2CIO_GlobalP0.col;
    y1 = s1.row - HRP2CIO_GlobalP0.row;
    y2 = s2.row - HRP2CIO_GlobalP0.row;

    distance1 = sqrt(x1*x1 + y1*y1);
    distance2 = sqrt(x2*x2 + y2*y2);

  }

  double CompareCBRep(CH_Point & s1,CH_Point &s2)
  {
    double x1,x2,y1,y2;
    x1 = s1.col - HRP2CIO_GlobalP0.col;
    x2 = s2.col - HRP2CIO_GlobalP0.col;
    y1 = s1.row - HRP2CIO_GlobalP0.row;
    y2 = s2.row - HRP2CIO_GlobalP0.row;

    return (x1*y2 - x2*y1);
  }

  ComputeConvexHull::ComputeConvexHull()
  {

  }

  ComputeConvexHull::~ComputeConvexHull()
  {

  }

  void ComputeConvexHull::DoComputeConvexHull(vector <CH_Point> aVecOfPoints,
                                              vector <CH_Point> & TheConvexHull)
  {

    if (aVecOfPoints.size()==0)
      return;

    CH_Point p0 = aVecOfPoints[0];

    // Detects the point with the smallest value for y.
    for(unsigned int i=0; i<aVecOfPoints.size(); i++)
      if (aVecOfPoints[i].row < p0.row)
        p0 = aVecOfPoints[i];


    HRP2CIO_GlobalP0 = p0;
    // Create the list of pixels sortes according to their
    // polar coordinates regarding p0.

    ODEBUG2(" VecOfPoints : " << aVecOfPoints.size());
    set<CH_Point, ltCH_Point> ListOfPointinPolarCoord;
    for(unsigned int i=0; i<aVecOfPoints.size(); i++)
      {
        unsigned char bInsert=1;

        // Check if the current point already exist with the same angle.
        // Cannot use find because of some pb with gcc version 2.95.
        set<CH_Point, ltCH_Point>::iterator it_PtinPolarCoord;

        it_PtinPolarCoord = ListOfPointinPolarCoord.begin();
        while (it_PtinPolarCoord!=ListOfPointinPolarCoord.end())
          {
            bool ToBeDeleted = false;
            CH_Point Current = *it_PtinPolarCoord;
            if (CompareCBRep(Current,aVecOfPoints[i])==0.0)
              {
                double distance1, distance2;
                DistanceCHRep(Current,aVecOfPoints[i],
                              distance1, distance2);
                if (distance1<=distance2)
                  ToBeDeleted = true;
                else
                  bInsert=0;
              }
            set<CH_Point, ltCH_Point>::iterator it_ToBeDeleted =
              it_PtinPolarCoord;

            it_PtinPolarCoord++;
            if(ToBeDeleted)
              ListOfPointinPolarCoord.erase(it_ToBeDeleted);

          }

        if (bInsert)
          {
            ListOfPointinPolarCoord.insert(aVecOfPoints[i]);
          }

      }

    // Apply the Graham's scan in O(n log n)

    // Creates the stack.

    set<CH_Point, ltCH_Point>::iterator it_LPPC;
    ODEBUG2( "LPPC: " <<ListOfPointinPolarCoord.size());


    it_LPPC = ListOfPointinPolarCoord.begin();
    TheConvexHull.insert(TheConvexHull.end(),p0);
    TheConvexHull.insert(TheConvexHull.end(),(*it_LPPC));
    it_LPPC++;
    TheConvexHull.insert(TheConvexHull.end(),(*it_LPPC));
    it_LPPC++;


    // Create the convex hull.
    while(it_LPPC!=ListOfPointinPolarCoord.end())
      {
        CH_Point pi = *it_LPPC;
        unsigned char ok=0;
        double x1,x2,y1,y2;
        CH_Point s1;
        CH_Point s2;

        do
          {
            if (TheConvexHull.size()>=2)
              {
                s1 = TheConvexHull[TheConvexHull.size()-1];
                s2 = TheConvexHull[TheConvexHull.size()-2];

                x1 = s1.col - s2.col;
                x2 = pi.col - s2.col;
                y1 = s1.row - s2.row;
                y2 = pi.row - s2.row;

                if ((x1*y2 - x2*y1)>0.0)
                  ok = 1;
                else
                  ok = 0;

              }
            else
              {
                ok=1;
              }

            if (!ok)
              TheConvexHull.pop_back();
          }
        while(!ok);


        TheConvexHull.push_back(*it_LPPC);
        it_LPPC++;
      }

  }
}

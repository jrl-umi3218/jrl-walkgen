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

#include <Mathematics/relative-feet-inequalities.hh>

using namespace std;
using namespace PatternGeneratorJRL;

#include <Debug.h>



RelativeFeetInequalities::RelativeFeetInequalities( SimplePluginManager *aSPM,
    CjrlHumanoidDynamicRobot *aHS ) :
    SimplePlugin(aSPM)
{

  DSFeetDistance_ = 0.2;
  SecurityMarginX_ = 0.04;
  SecurityMarginY_ = 0.04;

  set_feet_dimensions( aHS );

  init_convex_hulls();

  // Register method to handle
  string aMethodName[] =
    {":setfeetconstraint"};

  for(int i=0;i<1;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
    }
}


RelativeFeetInequalities::~RelativeFeetInequalities()
{

}


int
RelativeFeetInequalities::init_convex_hulls()
{

  double lxcoefsRight[4] = { 1.0, 1.0, -1.0, -1.0};
  double lycoefsRight[4] = {-1.0, 1.0,  1.0, -1.0};
  double lxcoefsLeft[4] = { 1.0, 1.0, -1.0, -1.0};
  double lycoefsLeft[4] = { 1.0, -1.0, -1.0, 1.0};

  FootPosEdges_.LeftDS.resize(5);
  FootPosEdges_.LeftSS.resize(5);
  double LeftFPosEdgesX[5] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  double LeftFPosEdgesY[5] = {0.2, 0.3, 0.4, 0.3, 0.2};
  FootPosEdges_.LeftDS.set(LeftFPosEdgesX,LeftFPosEdgesY);
  FootPosEdges_.LeftSS.set(LeftFPosEdgesX,LeftFPosEdgesY);

  FootPosEdges_.RightDS.resize(5);
  FootPosEdges_.RightSS.resize(5);
  double RightFPosEdgesX[5] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  double RightFPosEdgesY[5] = {-0.2, -0.3, -0.4, -0.3, -0.2};
  FootPosEdges_.RightDS.set(RightFPosEdgesX,RightFPosEdgesY);
  FootPosEdges_.RightSS.set(RightFPosEdgesX,RightFPosEdgesY);

  ZMPPosEdges_.LeftDS.resize(4);
  ZMPPosEdges_.LeftSS.resize(4);
  ZMPPosEdges_.RightDS.resize(4);
  ZMPPosEdges_.RightSS.resize(4);
  for( unsigned j=0;j<4;j++ )
    {
      //Left single support phase
      ZMPPosEdges_.LeftSS.X[j] = lxcoefsLeft[j]*LeftFootSize_.getHalfWidth();
      ZMPPosEdges_.LeftSS.Y[j] = lycoefsLeft[j]*LeftFootSize_.getHalfHeight();
      //Right single support phase
      ZMPPosEdges_.RightSS.X[j] = lxcoefsRight[j]*RightFootSize_.getHalfWidth();
      ZMPPosEdges_.RightSS.Y[j] = lycoefsRight[j]*RightFootSize_.getHalfHeight();
      //Left DS phase
      ZMPPosEdges_.LeftDS.X[j] = lxcoefsLeft[j]*LeftFootSize_.getHalfWidth();
      ZMPPosEdges_.LeftDS.Y[j] = lycoefsLeft[j]*LeftFootSize_.getHalfHeightDS()-DSFeetDistance_/2.0;
      //Right DS phase
      ZMPPosEdges_.RightDS.X[j] = lxcoefsRight[j]*RightFootSize_.getHalfWidth();
      ZMPPosEdges_.RightDS.Y[j] = lycoefsRight[j]*RightFootSize_.getHalfHeightDS()+DSFeetDistance_/2.0;
    }

  return 0;

}


int
RelativeFeetInequalities::set_feet_dimensions( CjrlHumanoidDynamicRobot *aHS )
{

  // Read feet specificities.
  double HeightHalf,WidthHalf;
  CjrlFoot * RightFoot = aHS->rightFoot();
  if (RightFoot==0)
    {
      cerr << "Problem with the reading of the right foot"<< endl;
      return 0;
    }
  RightFoot->getSoleSize( WidthHalf,HeightHalf );

  CjrlFoot * LeftFoot = aHS->leftFoot();
  if (RightFoot==0)
    {
      cerr << "Problem while reading of the left foot"<< endl;
      return 0;
    }
  LeftFoot->getSoleSize( WidthHalf,HeightHalf );

  LeftFootSize_.setHalfSizeInit( WidthHalf,HeightHalf );
  LeftFootSize_.setConstraints( SecurityMarginX_,SecurityMarginY_ );
  RightFootSize_.setHalfSizeInit( WidthHalf,HeightHalf );
  RightFootSize_.setConstraints( SecurityMarginX_,SecurityMarginY_ );

  return 0;

}


int
RelativeFeetInequalities::set_vertices( convex_hull_t & ConvexHull,
    double Orientation,
    const support_state_t & PrwSupport,
    int Type)
{

  edges_s * ConvexHull_p = 0;

  switch(Type)
    {
    case ZMP_CONSTRAINTS:
      ConvexHull_p = & ZMPPosEdges_;
      break;
    case FOOT_CONSTRAINTS:
      ConvexHull_p = & FootPosEdges_;
      break;

    }
  //Prepare the computation of the convex hull
  if( PrwSupport.Foot == LEFT )
    {
      if( PrwSupport.Phase == DS )
        ConvexHull = ConvexHull_p->LeftDS;
      else
        ConvexHull = ConvexHull_p->LeftSS;
    }
  else
    {
      if( PrwSupport.Phase == DS )
        ConvexHull = ConvexHull_p->RightDS;
      else
        ConvexHull = ConvexHull_p->RightSS;
    }

  ConvexHull.rotate(Orientation);

  return 0;

}


int
RelativeFeetInequalities::compute_linear_system( const convex_hull_t & ConvexHull,
    MAL_MATRIX(&D,double),
    MAL_MATRIX(&Dc,double),
    const support_state_t & PrwSupport ) const
{

  double dx,dy,dc,x1,y1,x2,y2;
  unsigned n = ConvexHull.X.size();
  MAL_MATRIX_RESIZE( D,ConvexHull.X.size(),2 );
  MAL_MATRIX_RESIZE( Dc,ConvexHull.X.size(),1 );

  double Sign;
  if(PrwSupport.Foot == LEFT)
    Sign = 1.0;
  else
    Sign = -1.0;
  for( unsigned i=0;i<n-1;i++ )//first n-1 inequalities
    {
      y1 = ConvexHull.Y[i];
      y2 = ConvexHull.Y[i+1];
      x1 = ConvexHull.X[i];
      x2 = ConvexHull.X[i+1];

      dx = y1-y2;
      dy = x2-x1;
      dc = dx*x1+dy*y1;

      //symmetrical constraints
      dx = Sign*dx;
      dy = Sign*dy;
      dc = Sign*dc;

      D(i,0) = dx; D(i,1)= dy;
      Dc(i,0) = dc;
    }

  {
    //Last inequality
    unsigned i = n-1;

    y1 = ConvexHull.Y[i];
    y2 = ConvexHull.Y[0];
    x1 = ConvexHull.X[i];
    x2 = ConvexHull.X[0];

    dx = y1-y2;
    dy = x2-x1;
    dc = dx*x1+dy*y1;

    //for symmetrical constraints
    dx = Sign*dx;
    dy = Sign*dy;
    dc = Sign*dc;

    D(i,0) = dx; D(i,1)= dy;
    Dc(i,0) = dc;
  }

  return 0;

}


int
RelativeFeetInequalities::compute_linear_system (const convex_hull_t & ConvexHull,
    double * Dx, double * Dy, double * Dc,
    const support_state_t & PrwSupport) const
{

  double dx,dy,dc,x1,y1,x2,y2;
  unsigned nrows = ConvexHull.X.size();

  double Sign;
  if(PrwSupport.Foot == LEFT)
    Sign = 1.0;
  else
    Sign = -1.0;
  for( unsigned i=0;i<nrows-1;i++ )//first n-1 inequalities
    {
      y1 = ConvexHull.Y[i];
      y2 = ConvexHull.Y[i+1];
      x1 = ConvexHull.X[i];
      x2 = ConvexHull.X[i+1];

      dx = y1-y2;
      dy = x2-x1;
      dc = dx*x1+dy*y1;

      //symmetrical constraints
      dx = Sign*dx;
      dy = Sign*dy;
      dc = Sign*dc;

      Dx[i] = dx; Dy[i]= dy;
      Dc[i] = dc;
    }

  {
    //Last inequality
    unsigned i = nrows-1;

    y1 = ConvexHull.Y[i];
    y2 = ConvexHull.Y[0];
    x1 = ConvexHull.X[i];
    x2 = ConvexHull.X[0];

    dx = y1-y2;
    dy = x2-x1;
    dc = dx*x1+dy*y1;

    //for symmetrical constraints
    dx = Sign*dx;
    dy = Sign*dy;
    dc = Sign*dc;

    Dx[i] = dx; Dy[i]= dy;
    Dc[i] = dc;
  }

  return 0;

}


void
RelativeFeetInequalities::CallMethod( std::string &Method, std::istringstream &Args )
{

  if ( Method==":setfeetconstraint" )
    {
      string lCmd;
      Args >> lCmd;

      if (lCmd=="XY")
        {
          Args >> SecurityMarginX_;
          Args >> SecurityMarginY_;

          RightFootSize_.setConstraints( SecurityMarginX_, SecurityMarginY_ );
          LeftFootSize_.setConstraints( SecurityMarginX_, SecurityMarginY_ );
          cout << "Security margin On X: " << SecurityMarginX_
               << " Security margin On Y: " << SecurityMarginX_ << endl;
        }
    }

}

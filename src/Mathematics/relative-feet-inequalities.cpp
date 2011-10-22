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

  const unsigned nbVertCoP = 4;
  const unsigned nbVertFeet = 5;


  // Feet hulls:
  // -----------
  double LeftFPosEdgesX[nbVertFeet] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  double LeftFPosEdgesY[nbVertFeet] = {-0.2, -0.3, -0.4, -0.3, -0.2};
  double RightFPosEdgesX[nbVertFeet] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  double RightFPosEdgesY[nbVertFeet] = {0.2, 0.3, 0.4, 0.3, 0.2};

  FootPosEdges_.LeftDS.resize(nbVertFeet);
  FootPosEdges_.LeftSS.resize(nbVertFeet);
  FootPosEdges_.RightDS.resize(nbVertFeet);
  FootPosEdges_.RightSS.resize(nbVertFeet);

  FootPosEdges_.LeftDS.set_vertices(LeftFPosEdgesX,LeftFPosEdgesY);
  FootPosEdges_.LeftSS.set_vertices(LeftFPosEdgesX,LeftFPosEdgesY);
  FootPosEdges_.RightDS.set_vertices(RightFPosEdgesX,RightFPosEdgesY);
  FootPosEdges_.RightSS.set_vertices(RightFPosEdgesX,RightFPosEdgesY);


  // ZMP hulls:
  // ----------
  double lxcoefsRight[nbVertCoP] = { 1.0, 1.0, -1.0, -1.0};
  double lycoefsRight[nbVertCoP] = {-1.0, 1.0,  1.0, -1.0};
  double lxcoefsLeft[nbVertCoP] = { 1.0, 1.0, -1.0, -1.0};
  double lycoefsLeft[nbVertCoP] = { 1.0, -1.0, -1.0, 1.0};

  ZMPPosEdges_.LeftDS.resize(nbVertCoP);
  ZMPPosEdges_.LeftSS.resize(nbVertCoP);
  ZMPPosEdges_.RightDS.resize(nbVertCoP);
  ZMPPosEdges_.RightSS.resize(nbVertCoP);
  for( unsigned j = 0; j < nbVertCoP; j++ )
    {
      //Left single support phase
      ZMPPosEdges_.LeftSS.X_vec[j] = lxcoefsLeft[j]*LeftFootSize_.getHalfWidth();
      ZMPPosEdges_.LeftSS.Y_vec[j] = lycoefsLeft[j]*LeftFootSize_.getHalfHeight();
      //Right single support phase
      ZMPPosEdges_.RightSS.X_vec[j] = lxcoefsRight[j]*RightFootSize_.getHalfWidth();
      ZMPPosEdges_.RightSS.Y_vec[j] = lycoefsRight[j]*RightFootSize_.getHalfHeight();
      //Left DS phase
      ZMPPosEdges_.LeftDS.X_vec[j] = lxcoefsLeft[j]*LeftFootSize_.getHalfWidth();
      ZMPPosEdges_.LeftDS.Y_vec[j] = lycoefsLeft[j]*LeftFootSize_.getHalfHeightDS()-DSFeetDistance_/2.0;
      //Right DS phase
      ZMPPosEdges_.RightDS.X_vec[j] = lxcoefsRight[j]*RightFootSize_.getHalfWidth();
      ZMPPosEdges_.RightDS.Y_vec[j] = lycoefsRight[j]*RightFootSize_.getHalfHeightDS()+DSFeetDistance_/2.0;
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


void
RelativeFeetInequalities::set_vertices( convex_hull_t & ConvexHull,
    const support_state_t & Support, ineq_e type)
{

  edges_s * ConvexHull_p = 0;

  switch(type)
    {
    case INEQ_COP:
      ConvexHull_p = & ZMPPosEdges_;
      break;
    case INEQ_FEET:
      ConvexHull_p = & FootPosEdges_;
      break;
    case INEQ_COM:
      break;

    }
  //Choose edges
  if( Support.Foot == LEFT )
    {
      if( Support.Phase == DS )
        {
          ConvexHull.X_vec = ConvexHull_p->LeftDS.X_vec;
          ConvexHull.Y_vec = ConvexHull_p->LeftDS.Y_vec;
        }
      else
        {
          ConvexHull.X_vec = ConvexHull_p->LeftSS.X_vec;
          ConvexHull.Y_vec = ConvexHull_p->LeftSS.Y_vec;
        }
    }
  else
    {
      if( Support.Phase == DS )
        {
          ConvexHull.X_vec = ConvexHull_p->RightDS.X_vec;
          ConvexHull.Y_vec = ConvexHull_p->RightDS.Y_vec;
        }
      else
        {
          ConvexHull.X_vec = ConvexHull_p->RightSS.X_vec;
          ConvexHull.Y_vec = ConvexHull_p->RightSS.Y_vec;
        }
    }

  ConvexHull.rotate( YAW, Support.Yaw);

}


void
RelativeFeetInequalities::compute_linear_system ( convex_hull_t & ConvexHull,
    const support_state_t & PrwSupport ) const
{

  double dx,dy,dc,x1,y1,x2,y2;
  unsigned nbRows = ConvexHull.X_vec.size();

  double sign;
  if( PrwSupport.Foot == LEFT )
    sign = 1.0;
  else
    sign = -1.0;
  for( unsigned i=0; i<nbRows-1;i++ )//first n-1 inequalities
    {
      y1 = ConvexHull.Y_vec[i];
      y2 = ConvexHull.Y_vec[i+1];
      x1 = ConvexHull.X_vec[i];
      x2 = ConvexHull.X_vec[i+1];

      dx = y1-y2;
      dy = x2-x1;
      dc = dx*x1+dy*y1;

      //symmetrical constraints
      dx = sign*dx;
      dy = sign*dy;
      dc = sign*dc;

      ConvexHull.A_vec[i] = dx; ConvexHull.B_vec[i]= dy;
      ConvexHull.D_vec[i] = dc;
    }

  {
    //Last inequality
    unsigned i = nbRows-1;

    y1 = ConvexHull.Y_vec[i];
    y2 = ConvexHull.Y_vec[0];
    x1 = ConvexHull.X_vec[i];
    x2 = ConvexHull.X_vec[0];

    dx = y1-y2;
    dy = x2-x1;
    dc = dx*x1+dy*y1;

    //for symmetrical constraints
    dx = sign*dx;
    dy = sign*dy;
    dc = sign*dc;

    ConvexHull.A_vec[i] = dx; ConvexHull.B_vec[i]= dy;
    ConvexHull.D_vec[i] = dc;
  }

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
          init_convex_hulls();
          cout << "Security margin On X: " << SecurityMarginX_
               << " Security margin On Y: " << SecurityMarginY_ << endl;
        }
    }

}

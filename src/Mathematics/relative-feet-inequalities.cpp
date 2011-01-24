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

#include <Mathematics/FootConstraintsAsLinearSystemForVelRef.h>

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

  setFeetDimensions( aHS );

  initConvexHulls();

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
RelativeFeetInequalities::initConvexHulls()
{

  double lxcoefsRight[4] = { 1.0, 1.0, -1.0, -1.0};
  double lycoefsRight[4] = {-1.0, 1.0,  1.0, -1.0};
  double lxcoefsLeft[4] = { 1.0, 1.0, -1.0, -1.0};
  double lycoefsLeft[4] = { 1.0, -1.0, -1.0, 1.0};

  FootPosEdges_.leftDS.resize(5);
  FootPosEdges_.leftSS.resize(5);
  double LeftFPosEdgesX[5] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  double LeftFPosEdgesY[5] = {0.2, 0.3, 0.4, 0.3, 0.2};
  FootPosEdges_.leftDS.set(LeftFPosEdgesX,LeftFPosEdgesY);
  FootPosEdges_.leftSS.set(LeftFPosEdgesX,LeftFPosEdgesY);

  FootPosEdges_.rightDS.resize(5);
  FootPosEdges_.rightSS.resize(5);
  double RightFPosEdgesX[5] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  double RightFPosEdgesY[5] = {-0.2, -0.3, -0.4, -0.3, -0.2};
  FootPosEdges_.rightDS.set(RightFPosEdgesX,RightFPosEdgesY);
  FootPosEdges_.rightSS.set(RightFPosEdgesX,RightFPosEdgesY);

  ZMPPosEdges_.leftDS.resize(4);
  ZMPPosEdges_.leftSS.resize(4);
  ZMPPosEdges_.rightDS.resize(4);
  ZMPPosEdges_.rightSS.resize(4);
  for( unsigned j=0;j<4;j++ )
    {
      //Left single support phase
      ZMPPosEdges_.leftSS.X[j] = lxcoefsLeft[j]*LeftFootSize_.getHalfWidth();
      ZMPPosEdges_.leftSS.Y[j] = lycoefsLeft[j]*LeftFootSize_.getHalfHeight();
      //Right single support phase
      ZMPPosEdges_.rightSS.X[j] = lxcoefsRight[j]*RightFootSize_.getHalfWidth();
      ZMPPosEdges_.rightSS.Y[j] = lycoefsRight[j]*RightFootSize_.getHalfHeight();
      //Left DS phase
      ZMPPosEdges_.leftDS.X[j] = lxcoefsLeft[j]*LeftFootSize_.getHalfWidth();
      ZMPPosEdges_.leftDS.Y[j] = lycoefsLeft[j]*LeftFootSize_.getHalfHeightDS()-DSFeetDistance_/2.0;
      //Right DS phase
      ZMPPosEdges_.rightDS.X[j] = lxcoefsRight[j]*RightFootSize_.getHalfWidth();
      ZMPPosEdges_.rightDS.Y[j] = lycoefsRight[j]*RightFootSize_.getHalfHeightDS()+DSFeetDistance_/2.0;
    }

  return 0;

}



int
RelativeFeetInequalities::setFeetDimensions( CjrlHumanoidDynamicRobot *aHS )
{

  // Read feet specificities.
  double lHalfHeightInit,lHalfWidthInit;
  CjrlFoot * RightFoot = aHS->rightFoot();
  if (RightFoot==0)
    {
      cerr << "Problem with the reading of the right foot"<< endl;
      return 0;
    }
  RightFoot->getSoleSize( lHalfWidthInit,lHalfHeightInit );

  CjrlFoot * LeftFoot = aHS->leftFoot();
  if (RightFoot==0)
    {
      cerr << "Problem while reading of the left foot"<< endl;
      return 0;
    }
  LeftFoot->getSoleSize( lHalfWidthInit,lHalfHeightInit );

  LeftFootSize_.setHalfSizeInit( lHalfWidthInit,lHalfHeightInit );
  LeftFootSize_.setConstraints( SecurityMarginX_,SecurityMarginY_ );
  RightFootSize_.setHalfSizeInit( lHalfWidthInit,lHalfHeightInit );
  RightFootSize_.setConstraints( SecurityMarginX_,SecurityMarginY_ );

  return 0;

}



int
RelativeFeetInequalities::setVertices( convex_hull_t & ConvexHull,
						     double Orientation,
						     const support_state_t & PrwSupport,
						     int constraints_type)
{

  edges_s * conv_hulls = 0;
  switch(constraints_type)
    {
    case ZMP_CONSTRAINTS:
      conv_hulls = & ZMPPosEdges_;
      break;
    case FOOT_CONSTRAINTS:
      conv_hulls = & FootPosEdges_;
      break;

    }
  //Prepare the computation of the convex hull
  if( PrwSupport.Foot == 1 )
    {
      if( PrwSupport.Phase == 0 )
        ConvexHull = conv_hulls->leftDS;
      else
        ConvexHull = conv_hulls->leftSS;
    }
  else
    {
      if( PrwSupport.Phase == 0 )
        ConvexHull = conv_hulls->rightDS;
      else
        ConvexHull = conv_hulls->rightSS;
    }

  ConvexHull.rotate(Orientation);

  return 0;

}



int
RelativeFeetInequalities::computeLinearSystem( const convex_hull_t & ConvexHull,
                                                             MAL_MATRIX(&D,double),
                                                             MAL_MATRIX(&Dc,double),
                                                             const support_state_t & PrwSupport ) const
{

  double dx,dy,dc,x1,y1,x2,y2;
  unsigned n = ConvexHull.X.size();
  MAL_MATRIX_RESIZE( D,ConvexHull.X.size(),2 );
  MAL_MATRIX_RESIZE( Dc,ConvexHull.X.size(),1 );


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
      dx = (double)PrwSupport.Foot*dx;
      dy = (double)PrwSupport.Foot*dy;
      dc = (double)PrwSupport.Foot*dc;

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
    dx = (double)PrwSupport.Foot*dx;
    dy = (double)PrwSupport.Foot*dy;
    dc = (double)PrwSupport.Foot*dc;

    D(i,0) = dx; D(i,1)= dy;
    Dc(i,0) = dc;
  }

  return 0;

}


int
RelativeFeetInequalities::computeLinearSystem (const convex_hull_t & ConvexHull,
							     double * D_x, double * D_y, double * d,
							     const support_state_t & PrwSupport) const
{

  double dx,dy,dc,x1,y1,x2,y2;
  unsigned nrows = ConvexHull.X.size();

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
      dx = (double)PrwSupport.Foot*dx;
      dy = (double)PrwSupport.Foot*dy;
      dc = (double)PrwSupport.Foot*dc;

      D_x[i] = dx; D_y[i]= dy;
      d[i] = dc;
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
    dx = (double)PrwSupport.Foot*dx;
    dy = (double)PrwSupport.Foot*dy;
    dc = (double)PrwSupport.Foot*dc;

    D_x[i] = dx; D_y[i]= dy;
    d[i] = dc;
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

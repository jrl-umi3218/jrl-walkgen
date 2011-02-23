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



FootConstraintsAsLinearSystemForVelRef::FootConstraintsAsLinearSystemForVelRef( SimplePluginManager *aSPM,
                                                                                CjrlHumanoidDynamicRobot *aHS ) :
  SimplePlugin(aSPM)
{

  //TODO: Hard coded values
  m_DSFeetDistance = 0.2;
  m_SecurityMarginX = 0.04;
  m_SecurityMarginY = 0.04;

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

FootConstraintsAsLinearSystemForVelRef::~FootConstraintsAsLinearSystemForVelRef()
{

}



int
FootConstraintsAsLinearSystemForVelRef::initConvexHulls()
{

  double m_lxcoefsRight[4] = { 1.0, 1.0, -1.0, -1.0};
  double m_lycoefsRight[4] = {-1.0, 1.0,  1.0, -1.0};
  double m_lxcoefsLeft[4] = { 1.0, 1.0, -1.0, -1.0};
  double m_lycoefsLeft[4] = { 1.0, -1.0, -1.0, 1.0};

  m_FootPosEdges.leftDS.resize(5);
  m_FootPosEdges.leftSS.resize(5);
  double LeftFPosEdgesX[5] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  double LeftFPosEdgesY[5] = {0.2, 0.3, 0.4, 0.3, 0.2};
  m_FootPosEdges.leftDS.set(LeftFPosEdgesX,LeftFPosEdgesY);
  m_FootPosEdges.leftSS.set(LeftFPosEdgesX,LeftFPosEdgesY);

  m_FootPosEdges.rightDS.resize(5);
  m_FootPosEdges.rightSS.resize(5);
  double RightFPosEdgesX[5] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  double RightFPosEdgesY[5] = {-0.2, -0.3, -0.4, -0.3, -0.2};
  m_FootPosEdges.rightDS.set(RightFPosEdgesX,RightFPosEdgesY);
  m_FootPosEdges.rightSS.set(RightFPosEdgesX,RightFPosEdgesY);

  m_ZMPPosEdges.leftDS.resize(4);
  m_ZMPPosEdges.leftSS.resize(4);
  m_ZMPPosEdges.rightDS.resize(4);
  m_ZMPPosEdges.rightSS.resize(4);
  for( unsigned j=0;j<4;j++ )
    {
      //Left single support phase
      m_ZMPPosEdges.leftSS.X[j] = m_lxcoefsLeft[j]*m_LeftFootSize.getHalfWidth();
      m_ZMPPosEdges.leftSS.Y[j] = m_lycoefsLeft[j]*m_LeftFootSize.getHalfHeight();
      //Right single support phase
      m_ZMPPosEdges.rightSS.X[j] = m_lxcoefsRight[j]*m_RightFootSize.getHalfWidth();
      m_ZMPPosEdges.rightSS.Y[j] = m_lycoefsRight[j]*m_RightFootSize.getHalfHeight();
      //Left DS phase
      m_ZMPPosEdges.leftDS.X[j] = m_lxcoefsLeft[j]*m_LeftFootSize.getHalfWidth();
      m_ZMPPosEdges.leftDS.Y[j] = m_lycoefsLeft[j]*m_LeftFootSize.getHalfHeightDS()-m_DSFeetDistance/2.0;
      //Right DS phase
      m_ZMPPosEdges.rightDS.X[j] = m_lxcoefsRight[j]*m_RightFootSize.getHalfWidth();
      m_ZMPPosEdges.rightDS.Y[j] = m_lycoefsRight[j]*m_RightFootSize.getHalfHeightDS()+m_DSFeetDistance/2.0;
    }

  return 0;

}



int
FootConstraintsAsLinearSystemForVelRef::setFeetDimensions( CjrlHumanoidDynamicRobot *aHS )
{

  // Read feet specificities.
  double lHalfHeightInit,lHalfWidthInit;
  m_RightFoot = aHS->rightFoot();
  if (m_RightFoot==0)
    {
      cerr << "Problem with the reading of the right foot"<< endl;
      return 0;
    }
  m_RightFoot->getSoleSize( lHalfWidthInit,lHalfHeightInit );

  m_LeftFoot = aHS->leftFoot();
  if (m_RightFoot==0)
    {
      cerr << "Problem while reading of the left foot"<< endl;
      return 0;
    }
  m_LeftFoot->getSoleSize( lHalfWidthInit,lHalfHeightInit );

  m_LeftFootSize.setHalfSizeInit( lHalfWidthInit,lHalfHeightInit );
  m_LeftFootSize.setConstraints( m_SecurityMarginX,m_SecurityMarginY );
  m_RightFootSize.setHalfSizeInit( lHalfWidthInit,lHalfHeightInit );
  m_RightFootSize.setConstraints( m_SecurityMarginX,m_SecurityMarginY );

  return 0;

}



int
FootConstraintsAsLinearSystemForVelRef::setVertices( convex_hull_t & ConvexHull,
						     double Orientation,
						     const support_state_t & PrwSupport,
						     int constraints_type)
{

  edges_s * conv_hulls=0;
  switch(constraints_type)
    {
    case ZMP_CONSTRAINTS:
      conv_hulls = & m_ZMPPosEdges;
      break;
    case FOOT_CONSTRAINTS:
      conv_hulls = & m_FootPosEdges;
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
FootConstraintsAsLinearSystemForVelRef::computeLinearSystem( const convex_hull_t & ConvexHull,
                                                             MAL_MATRIX(&D,double),
                                                             MAL_MATRIX(&Dc,double),
                                                             const support_state_t & PrwSupport )
{

  double dx,dy,dc,x1,y1,x2,y2;
  unsigned int n = ConvexHull.X.size();
  MAL_MATRIX_RESIZE( D,ConvexHull.X.size(),2 );
  MAL_MATRIX_RESIZE( Dc,ConvexHull.X.size(),1 );


  for( unsigned int i=0;i<n-1;i++ )//first n-1 inequalities
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
    unsigned int i = n-1;

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
FootConstraintsAsLinearSystemForVelRef::computeLinearSystem (const convex_hull_t & ConvexHull,
							     double * D_x, double * D_y, double * d,
							     const support_state_t & PrwSupport)
{

  double dx,dy,dc,x1,y1,x2,y2;
  unsigned int nrows = ConvexHull.X.size();

  for( unsigned int i=0;i<nrows-1;i++ )//first n-1 inequalities
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
    unsigned int i = nrows-1;

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
FootConstraintsAsLinearSystemForVelRef::CallMethod( std::string &Method, std::istringstream &Args )
{

  if ( Method==":setfeetconstraint" )
    {
      string lCmd;
      Args >> lCmd;

      if (lCmd=="XY")
        {
          Args >> m_SecurityMarginX;
          Args >> m_SecurityMarginY;

          m_RightFootSize.setConstraints( m_SecurityMarginX, m_SecurityMarginY );
          m_LeftFootSize.setConstraints( m_SecurityMarginX, m_SecurityMarginY );
          cout << "Security margin On X: " << m_SecurityMarginX
               << " Security margin On Y: " << m_SecurityMarginX << endl;
        }
    }

}

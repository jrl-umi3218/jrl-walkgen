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

  m_LeftFPosConstrVerticesX[0] = -0.28;m_LeftFPosConstrVerticesX[1]=-0.2;m_LeftFPosConstrVerticesX[2]=0.0;
  m_LeftFPosConstrVerticesX[3]=0.2;m_LeftFPosConstrVerticesX[4]=0.28;
  m_LeftFPosConstrVerticesY[0] = 0.2;m_LeftFPosConstrVerticesY[1]=0.3;m_LeftFPosConstrVerticesY[2]=0.4;
  m_LeftFPosConstrVerticesY[3]=0.3;m_LeftFPosConstrVerticesY[4]=0.2;
  m_RightFPosConstrVerticesX[0]=-0.28;m_RightFPosConstrVerticesX[1]=-0.2;m_RightFPosConstrVerticesX[2]=0.0;
  m_RightFPosConstrVerticesX[3]=0.2;m_RightFPosConstrVerticesX[4]=0.28;
  m_RightFPosConstrVerticesY[0] = -0.2; m_RightFPosConstrVerticesY[1] = -0.3; m_RightFPosConstrVerticesY[2] = -0.4;
  m_RightFPosConstrVerticesY[3] = -0.3; m_RightFPosConstrVerticesY[4] = -0.2;


  for( unsigned j=0;j<4;j++ )
    {
      //Left single support phase
      m_LeftZMPConstrVerticesX[j] = m_lxcoefsLeft[j]*m_LeftFootSize.getHalfWidth();
      m_LeftZMPConstrVerticesY[j] = m_lycoefsLeft[j]*m_LeftFootSize.getHalfHeight();
      //Right single support phase
      m_RightZMPConstrVerticesX[j] = m_lxcoefsRight[j]*m_RightFootSize.getHalfWidth();
      m_RightZMPConstrVerticesY[j] = m_lycoefsRight[j]*m_RightFootSize.getHalfHeight();
      //Left DS phase
      m_LeftDSZMPConstrVerticesX[j] = m_lxcoefsLeft[j]*m_LeftFootSize.getHalfWidth();
      m_LeftDSZMPConstrVerticesY[j] = m_lycoefsLeft[j]*m_LeftFootSize.getHalfHeightDS()-m_DSFeetDistance/2.0;
      //Right DS phase
      m_RightDSZMPConstrVerticesX[j] = m_lxcoefsRight[j]*m_RightFootSize.getHalfWidth();
      m_RightDSZMPConstrVerticesY[j] = m_lycoefsRight[j]*m_RightFootSize.getHalfHeightDS()+m_DSFeetDistance/2.0;
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
      cerr << "Problem with the reading of the left foot"<< endl;
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
FootConstraintsAsLinearSystemForVelRef::setVertices( std::vector<CH_Point> & ZMPConstrVertices,
                                                     std::vector<CH_Point> & FeetPosConstrVertices,
                                                     double ZMPConvHullAngle,
                                                     double FeetPosConvHullAngle,
                                                     support_state_t & PrwSupport)
{

  //compute the cos and sin of the angle
  double c_ZMP = cos(ZMPConvHullAngle);
  double s_ZMP = sin(ZMPConvHullAngle);
  double c_FP = cos(FeetPosConvHullAngle);
  double s_FP = sin(FeetPosConvHullAngle);

  //Prepare the computation of the convex hull
  if( PrwSupport.Foot == 1 )
    {
      m_FPosConstrVerticesX = m_LeftFPosConstrVerticesX;
      m_FPosConstrVerticesY = m_LeftFPosConstrVerticesY;
      if( PrwSupport.Phase == 0 )
        {
          m_ZMPConstrVerticesX = m_LeftDSZMPConstrVerticesX;
          m_ZMPConstrVerticesY = m_LeftDSZMPConstrVerticesY;
        }
      else
        {
          m_ZMPConstrVerticesX = m_LeftZMPConstrVerticesX;
          m_ZMPConstrVerticesY = m_LeftZMPConstrVerticesY;
        }
    }
  else
    {
      m_FPosConstrVerticesX = m_RightFPosConstrVerticesX;
      m_FPosConstrVerticesY = m_RightFPosConstrVerticesY;
      if( PrwSupport.Phase == 0 )
        {
          m_ZMPConstrVerticesX = m_RightDSZMPConstrVerticesX;
          m_ZMPConstrVerticesY = m_RightDSZMPConstrVerticesY;
        }
      else
        {
          m_ZMPConstrVerticesX = m_RightZMPConstrVerticesX;
          m_ZMPConstrVerticesY = m_RightZMPConstrVerticesY;
        }
    }

  //Set the convex hull of the ZMP constraints
  for( int j=0;j<4;j++ )
    {
      ZMPConstrVertices[j].col = ( m_ZMPConstrVerticesX[j] * c_ZMP - m_ZMPConstrVerticesY[j] * s_ZMP );
      ZMPConstrVertices[j].row = ( m_ZMPConstrVerticesX[j] * s_ZMP + m_ZMPConstrVerticesY[j] * c_ZMP );
    }
  //Set the convex hull of the feet constraints
  //TODO: The interior border does not yet depend on the angle
  for( int j=0;j<5;j++ )
    {
      FeetPosConstrVertices[j].col = ( m_FPosConstrVerticesX[j] * c_FP - m_FPosConstrVerticesY[j] * s_FP );
      FeetPosConstrVertices[j].row = ( m_FPosConstrVerticesX[j] * s_FP + m_FPosConstrVerticesY[j] * c_FP );
    }

  return 0;

}



int
FootConstraintsAsLinearSystemForVelRef::computeLinearSystem( vector<CH_Point> aVecOfPoints,
                                                             MAL_MATRIX(&D,double),
                                                             MAL_MATRIX(&Dc,double),
                                                             support_state_t & PrwSupport )
{

  double dx,dy,dc,x1,y1,x2,y2;
  unsigned int n = aVecOfPoints.size();
  MAL_MATRIX_RESIZE( D,aVecOfPoints.size(),2 );
  MAL_MATRIX_RESIZE( Dc,aVecOfPoints.size(),1 );


  for( unsigned int i=0;i<n-1;i++ )//first n-1 inequalities
    {
      y1 = aVecOfPoints[i].row;
      y2 = aVecOfPoints[i+1].row;
      x1 = aVecOfPoints[i].col;
      x2 = aVecOfPoints[i+1].col;

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

    y1 = aVecOfPoints[i].row;
    y2 = aVecOfPoints[0].row;
    x1 = aVecOfPoints[i].col;
    x2 = aVecOfPoints[0].col;

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
FootConstraintsAsLinearSystemForVelRef::buildConstraintInequalities( deque< FootAbsolutePosition> &LeftFootAbsolutePositions,
                                                                     deque<FootAbsolutePosition> &RightFootAbsolutePositions,
                                                                     deque<LinearConstraintInequalityFreeFeet_t> & ZMPInequalitiesDeque,
                                                                     deque<LinearConstraintInequalityFreeFeet_t> & FeetPosInequalitiesDeque,
                                                                     reference_t & RefVel, double StartingTime, double QP_N,
                                                                     SupportFSM * SupportFSM, support_state_t & CurrentSupport, support_state_t & PrwSupport,
                                                                     deque<double> &PreviewedSupportAngles, int & NbConstraints )
{

  vector<CH_Point> ZMPConstrVertices;
  ZMPConstrVertices.resize(4);
  vector<CH_Point> FeetPosConstrVertices;
  FeetPosConstrVertices.resize(5);

  double s_t=0,c_t=0;
  double lx = 0.0,ly =0.0;

  //determine the current support angle
  deque<FootAbsolutePosition>::iterator FAP_it;

  //define the current support angle
  if( CurrentSupport.Foot==1 )
    {
      FAP_it = LeftFootAbsolutePositions.end();
      FAP_it--;
    }
  else
    {
      FAP_it = RightFootAbsolutePositions.end();
      FAP_it--;
    }
  double CurrentSupportAngle = FAP_it->theta*M_PI/180.0;


  //initialize the previewed support state before previewing
  PrwSupport.Phase  = CurrentSupport.Phase;
  PrwSupport.Foot  = CurrentSupport.Foot;
  PrwSupport.StepsLeft  = CurrentSupport.StepsLeft;
  PrwSupport.TimeLimit = CurrentSupport.TimeLimit;
  PrwSupport.StepNumber  = 0;

  double ZMPConvHullOrientation = CurrentSupportAngle;
  double FPConvHullOrientation = CurrentSupportAngle;

  //set current constraints
  setVertices( ZMPConstrVertices, FeetPosConstrVertices,
               ZMPConvHullOrientation, FPConvHullOrientation,
               PrwSupport );

  //set constraints for the whole preview window
  for( int i=1;i<=QP_N;i++ )
    {
      SupportFSM->setSupportState( StartingTime, i, PrwSupport, RefVel );

      if( PrwSupport.StateChanged )
        setVertices( ZMPConstrVertices, FeetPosConstrVertices,
                     ZMPConvHullOrientation, FPConvHullOrientation, PrwSupport );

      if( PrwSupport.StateChanged && PrwSupport.StepNumber>0 )
          FPConvHullOrientation = PreviewedSupportAngles[PrwSupport.StepNumber-1];

      //foot positioning constraints
      //
      if( PrwSupport.StateChanged && PrwSupport.StepNumber>0 && PrwSupport.Phase != 0)
        {
          ZMPConvHullOrientation = PreviewedSupportAngles[PrwSupport.StepNumber-1];

          if( PrwSupport.StepNumber==1 )
              FPConvHullOrientation = CurrentSupportAngle;
          else
              FPConvHullOrientation = PreviewedSupportAngles[PrwSupport.StepNumber-2];

          setVertices( ZMPConstrVertices, FeetPosConstrVertices,
                       ZMPConvHullOrientation, FPConvHullOrientation, PrwSupport );

          LinearConstraintInequalityFreeFeet_t aLCIFP;
          computeLinearSystem( FeetPosConstrVertices, aLCIFP.D, aLCIFP.Dc, PrwSupport );
          aLCIFP.StepNumber = PrwSupport.StepNumber;
          FeetPosInequalitiesDeque.push_back( aLCIFP );
          NbConstraints += MAL_MATRIX_NB_ROWS( aLCIFP.D );
        }

      LinearConstraintInequalityFreeFeet_t aLCI;
      computeLinearSystem( ZMPConstrVertices, aLCI.D, aLCI.Dc, PrwSupport );
      aLCI.StepNumber = PrwSupport.StepNumber;
      ZMPInequalitiesDeque.push_back( aLCI );
      NbConstraints += MAL_MATRIX_NB_ROWS( aLCI.D );
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

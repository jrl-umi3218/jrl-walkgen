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
    \brief This object build linear constraints based on feet positions.
*/

#include <iostream>
#include <fstream>

#include <Mathematics/FootConstraintsAsLinearSystemForVelRef.h>

using namespace std;
using namespace PatternGeneratorJRL;

#include <Debug.h>



FootConstraintsAsLinearSystemForVelRef::
FootConstraintsAsLinearSystemForVelRef(SimplePluginManager *aSPM, 
				       CjrlHumanoidDynamicRobot *aHS, 
				       double ConstraintOnX, 
				       double ConstraintOnY) :
  SimplePlugin(aSPM)
{
  m_HS = aHS;
  // Read humanoid specificities.
  lRightFoot = m_HS->rightFoot();
  if (lRightFoot==0)
    {
      cerr << "Problem with the reading of the right foot"<< endl;
    }
  lRightFoot->getSoleSize(lRightFootHalfWidth,lRightFootHalfHeight);
  lRightFoot->getAnklePositionInLocalFrame(AnklePosition);
  lZ = AnklePosition[2];
  lLeftFoot = m_HS->leftFoot();
  if (lRightFoot==0)
    {
      cerr << "Problem with the reading of the left foot"<< endl;
    }
    
  lLeftFoot->getSoleSize(lLeftFootHalfWidth,lLeftFootHalfHeight);


  lRightFootHalfWidth *= 0.5;
  lRightFootHalfHeight *= 0.5;
  lLeftFootHalfWidth *= 0.5;
  lLeftFootHalfHeight *= 0.5;


  //Andremize: Scilab feet
  lLeftFootHalfHeight = 0.069-0.059;
  lRightFootHalfHeight = 0.069-0.059;

  lLeftFootHalfWidth = 0.1206-0.1006;
  lRightFootHalfWidth = 0.1206-0.1006;

  DSFeetDistance = 0.2;
  lLeftFootHalfHeightDS = lLeftFootHalfHeight+DSFeetDistance/2.0;
  lRightFootHalfHeightDS = lRightFootHalfHeight+DSFeetDistance/2.0;

  ConvexHullFP.resize(5);

  //initFPConstrArrays();
  //TODO 0: find another condition
  if(0)
    RESETDEBUG4("Constraints-fCSALS.dat");

  m_FullDebug = 0;

  //TODO 1: How does ODEBUG/RESETDEBUG get activated?
  if(m_FullDebug>2)
    {
      ofstream aof;
      aof.open("/tmp/SupportOrientations.dat",ofstream::out);
      aof.close();
      aof.open("/tmp/ConvexHull.dat",ofstream::out);
      aof.close();
    }

}

FootConstraintsAsLinearSystemForVelRef::
~FootConstraintsAsLinearSystemForVelRef()
{

}


int FootConstraintsAsLinearSystemForVelRef::FindSimilarConstraints(MAL_MATRIX(&A,double),
							  vector<int> &SimilarConstraints)
{

  SimilarConstraints.resize(MAL_MATRIX_NB_ROWS(A));
  SimilarConstraints[0] = 0;
  SimilarConstraints[1] = 0;
  if(MAL_MATRIX_NB_ROWS(A)==4)
    {
      if ((A(0,0)==-A(2,0)) &&
	  (A(0,1)==-A(2,1)))
	SimilarConstraints[2]=-2;
      if ((A(1,0)==-A(3,0)) &&
	  (A(1,1)==-A(3,1)))
	SimilarConstraints[3]=-2;

    }
  else if(MAL_MATRIX_NB_ROWS(A)==6)
    {
      SimilarConstraints[2] = 0;
      if ((A(0,0)==-A(3,0)) &&
	  (A(0,1)==-A(3,1)))
	SimilarConstraints[3]=-3;
      if ((A(1,0)==-A(4,0)) &&
	  (A(1,1)==-A(4,1)))
	SimilarConstraints[4]=-3;
      if ((A(2,0)==-A(5,0)) &&
	  (A(2,1)==-A(5,1)))
	SimilarConstraints[5]=-3;
    }

  if (0)
    for(unsigned int i=0;i<SimilarConstraints.size();i++)
      cout << "Similar (" << i << ")=" << SimilarConstraints[i] << endl;

  return 0;
}




// Assuming that the points are going counter-clockwise
int FootConstraintsAsLinearSystemForVelRef::computeLinearSystem(vector<CH_Point> aVecOfPoints,
						       MAL_MATRIX(&D,double),
						       MAL_MATRIX(&Dc,double),
						       SupportState_t PrwSupport)
{

  double dx,dy,dc,x1,y1,x2,y2;
  unsigned int n = aVecOfPoints.size();
  MAL_MATRIX_RESIZE(D,aVecOfPoints.size(),2);
  MAL_MATRIX_RESIZE(Dc,aVecOfPoints.size(),1);


  for(unsigned int i=0;i<n-1;i++)//first n-1 inequalities
    {
      ODEBUG("(x["<< i << "],y["<<i << "]): " << aVecOfPoints[i].col << " " <<  aVecOfPoints[i].row << " "
	     << aVecOfPoints[i+1].col << " "  << aVecOfPoints[i+1].row );

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
    ODEBUG("(x["<< i << "],y["<<i << "]): " << aVecOfPoints[i].col << " " <<  aVecOfPoints[i].row << " "
	   << aVecOfPoints[i+1].col << " "  << aVecOfPoints[i+1].row );

    y1 = aVecOfPoints[i].row;
    y2 = aVecOfPoints[0].row;
    x1 = aVecOfPoints[i].col;
    x2 = aVecOfPoints[0].col;

    dx = y1-y2;
    dy = x2-x1;
    dc = dx*x1+dy*y1;

    //symmetrical constraints cannot be achieved without knowledge of the support foot
    dx = (double)PrwSupport.Foot*dx;
    dy = (double)PrwSupport.Foot*dy;
    dc = (double)PrwSupport.Foot*dc;

    D(i,0) = dx; D(i,1)= dy;
    Dc(i,0) = dc;
  }

  return 0;
}


int FootConstraintsAsLinearSystemForVelRef::buildLinearConstraintInequalities(deque< FootAbsolutePosition> &LeftFootAbsolutePositions,
								     deque<FootAbsolutePosition> &RightFootAbsolutePositions,
								     deque<LinearConstraintInequalityFreeFeet_t> &
								     QueueOfLConstraintInequalitiesFreeFeet,
								     deque<LinearConstraintInequalityFreeFeet_t> &
								     QueueOfFeetPosInequalities,
								     ReferenceAbsoluteVelocity & RefVel,
								     double StartingTime, double m_QP_N,
								     SupportFSM * SupportFSM, SupportState_t CurrentSupport, SupportState_t & PrwSupport,
								     deque<double> &PreviewedSupportAngles,
								     int & NbOfConstraints)
{

  //For symmetrical constraints: The points of the left foot are counted clockwise.
  //The
  float lxcoefsRight[4] = { 1.0, 1.0, -1.0, -1.0};
  float lycoefsRight[4] = {-1.0, 1.0,  1.0, -1.0};
  float lxcoefsLeft[4] = { 1.0, 1.0, -1.0, -1.0};
  float lycoefsLeft[4] = { 1.0, -1.0, -1.0, 1.0};

  float *lxcoefs, *lycoefs;

  float CHLeftFPosConstrArrayX[5] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  float CHLeftFPosConstrArrayY[5] = {0.2, 0.3, 0.4, 0.3, 0.2};
  float CHRightFPosConstrArrayX[5] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  float CHRightFPosConstrArrayY[5] = {-0.2, -0.3, -0.4, -0.3, -0.2};

  vector<CH_Point> TheConvexHull;

  //determine the current support angle
  deque<FootAbsolutePosition>::iterator FAP_it;


  if(CurrentSupport.Foot==1)
    {
      FAP_it = LeftFootAbsolutePositions.end();
      FAP_it--;
    }
  else
    {
      FAP_it = RightFootAbsolutePositions.end();
      FAP_it--;
    }

  s_t = sin(FAP_it->theta*M_PI/180.0);
  c_t = cos(FAP_it->theta*M_PI/180.0);

  if(m_FullDebug>2)
    {
      ofstream aof;
      aof.open("/tmp/SupportOrientations.dat",ofstream::app);
      aof<<endl<<"Time: "<<StartingTime<<" CurrentSupAngle "<<FAP_it->theta*M_PI/180.0;
      aof.close();
    }

  //initialize the previewed support state before previewing
  PrwSupport.Phase  = CurrentSupport.Phase;
  PrwSupport.Foot  = CurrentSupport.Foot;
  PrwSupport.StepsLeft  = CurrentSupport.StepsLeft;
  PrwSupport.TimeLimit = CurrentSupport.TimeLimit;
  PrwSupport.StepNumber  = 0;

  for(unsigned int i=1;i<=m_QP_N;i++)
    {

      SupportFSM->setSupportState(StartingTime, i, PrwSupport, RefVel);

      ComputeCH=0;

      TheConvexHull.resize(4);//As for now only ZMP constraints

      //TODO: theta = 0 as for now

      if(PrwSupport.StateChanged && PrwSupport.StepNumber>0)
	{
	  s_t = sin(PreviewedSupportAngles[PrwSupport.StepNumber-1]);
	  c_t = cos(PreviewedSupportAngles[PrwSupport.StepNumber-1]);

	  if(m_FullDebug>2)
	    {
	      ofstream aof;
	      aof.open("/tmp/SupportOrientations.dat",ofstream::app);
	      aof<<" PreviewedAngle: "<<PreviewedSupportAngles[PrwSupport.StepNumber-1];
	      aof.close();
	    }
	}

      double FootHalfWidth, FootHalfHeight;

      //Prepare the computation of the convex hull
      if(PrwSupport.Phase == 0)
	{
	  //TODO: theta = 0
	  lx = 0.0;
	  ly = -(double)PrwSupport.Foot*DSFeetDistance/2.0;

	  if(PrwSupport.Foot == 1)
	    {
	      FootHalfWidth = lLeftFootHalfWidth;
	      FootHalfHeight = lLeftFootHalfHeightDS;

	      lxcoefs = lxcoefsLeft;
	      lycoefs = lycoefsLeft;
	    }
	  else
	    {
	      FootHalfWidth = lRightFootHalfWidth;
	      FootHalfHeight = lRightFootHalfHeightDS;

	      lxcoefs = lxcoefsRight;
	      lycoefs = lycoefsRight;
	    }
	}
      else
	{
	  //TODO: theta = 0
	  lx = 0.0;
	  ly = 0.0;

	  if(PrwSupport.Foot == 1)
	    {
	      FootHalfWidth = lLeftFootHalfWidth;
	      FootHalfHeight = lLeftFootHalfHeight;

	      lxcoefs = lxcoefsLeft;
	      lycoefs = lycoefsLeft;
	    }
	  else
	    {
	      FootHalfWidth = lRightFootHalfWidth;
	      FootHalfHeight = lRightFootHalfHeight;

	      lxcoefs = lxcoefsRight;
	      lycoefs = lycoefsRight;
	    }
	}

      //Compute the convex hull
      for(unsigned j=0;j<4;j++)
	{
	  TheConvexHull[j].col = (lx+lxcoefs[j]*FootHalfWidth)*c_t -
	    (ly+lycoefs[j]*FootHalfHeight)*s_t;
	  TheConvexHull[j].row = (lx+lxcoefs[j]*FootHalfWidth)*s_t +
	    (ly+lycoefs[j]*FootHalfHeight)*c_t;

	  if((m_FullDebug>2) && (i==1))
	    {
		    
	      ofstream aof;
	      aof.open("/tmp/ConvexHull.dat",ofstream::app);
	      aof<<TheConvexHull[j].col<<" "<<TheConvexHull[j].row<<" "<<endl;
	      aof.close();
	    }
		  
	}

      //foot positionning constraints
      if(PrwSupport.StateChanged && PrwSupport.StepNumber>0)
	{
	  //TODO: theta == 0
	  lx = 0.0;
	  ly = 0.0;
	  //TODO: Has to be the angle of the previous foot

	  if(PrwSupport.StepNumber==1)
	    {
	      s_t = sin(FAP_it->theta*M_PI/180.0);
	      c_t = cos(FAP_it->theta*M_PI/180.0);

	      if(m_FullDebug>2)
		{
		  ofstream aof;
		  aof.open("/tmp/SupportOrientations.dat",ofstream::app);
		  aof<<" AngleFootConstraints: "<<FAP_it->theta*M_PI/180.0;
		  aof.close();
		}
	    }
	  else
	    {
	      s_t = sin(PreviewedSupportAngles[PrwSupport.StepNumber-2]);
	      c_t = cos(PreviewedSupportAngles[PrwSupport.StepNumber-2]);

	      if(m_FullDebug>2)
		{
		  ofstream aof;
		  aof.open("/tmp/SupportOrientations.dat",ofstream::app);
		  aof<<" AngleFootConstraints: "<<PreviewedSupportAngles[PrwSupport.StepNumber-2];
		  aof.close();
		}
	    }
	  if(PrwSupport.Foot == 1)
	    {
	      CHFPosConstrArrayX = CHLeftFPosConstrArrayX;
	      CHFPosConstrArrayY = CHLeftFPosConstrArrayY;
	    }
	  else
	    {
	      CHFPosConstrArrayX = CHRightFPosConstrArrayX;
	      CHFPosConstrArrayY = CHRightFPosConstrArrayY;
	    }

	  //TODO: The interior border does not yet depend on the angle
	  for(unsigned j=0;j<5;j++)
	    {
	      ConvexHullFP[j].col = lx + ( CHFPosConstrArrayX[j] * c_t - CHFPosConstrArrayY[j] * s_t );
	      ConvexHullFP[j].row = ly + ( CHFPosConstrArrayX[j] * s_t + CHFPosConstrArrayY[j] * c_t );
	    }

	  LinearConstraintInequalityFreeFeet_t aLCIFP;

	  computeLinearSystem(ConvexHullFP, aLCIFP.D, aLCIFP.Dc, PrwSupport);

	  aLCIFP.StepNumber = PrwSupport.StepNumber;

	  QueueOfFeetPosInequalities.push_back(aLCIFP);

	}

      // Linear Constraint Inequality
      LinearConstraintInequalityFreeFeet_t aLCI;
      
      // Building those constraints
      computeLinearSystem(TheConvexHull, aLCI.D, aLCI.Dc, PrwSupport);

      //For selection vectors and matrices when computing the constraints
      aLCI.StepNumber = PrwSupport.StepNumber;

      QueueOfLConstraintInequalitiesFreeFeet.push_back(aLCI);

      //Determine the number of constraints 
      deque<LinearConstraintInequalityFreeFeet_t>::iterator LCIFF_it;
      LCIFF_it = QueueOfLConstraintInequalitiesFreeFeet.begin(); 
      int IndexConstraint=0; 
      for( int i=0;i<m_QP_N;i++) 
	{ 
	  if (LCIFF_it==QueueOfLConstraintInequalitiesFreeFeet.end()) 
	    { 
	      break; 
	    } 
	  IndexConstraint += MAL_MATRIX_NB_ROWS(LCIFF_it->D); 
	  LCIFF_it++; 
	} 
		
      if(PrwSupport.StepNumber>0)
	{ 
	  LCIFF_it = QueueOfFeetPosInequalities.begin(); 
		    
	  IndexConstraint += double(PrwSupport.StepNumber)*MAL_MATRIX_NB_ROWS(LCIFF_it->D);
	} 
		
      NbOfConstraints = IndexConstraint; 

   }


  ODEBUG("Size of the 5 ms array: "<< LeftFootAbsolutePositions.size());
  ODEBUG("Size of the queue of Linear Constraint Inequalities " << QueueOfLConstraintInequalitiesFreeFeet.size());
  return 0;
}

void FootConstraintsAsLinearSystemForVelRef::CallMethod(std::string &Method, std::istringstream &Args)
{
  // TO BE EXTENDED.
}

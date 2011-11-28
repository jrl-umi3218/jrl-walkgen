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

#include <Mathematics/FootConstraintsAsLinearSystemForVelRef.hh>

using namespace std;
using namespace PatternGeneratorJRL;

#include <Debug.hh>



FootConstraintsAsLinearSystemForVelRef::
FootConstraintsAsLinearSystemForVelRef(SimplePluginManager *aSPM,
				       CjrlHumanoidDynamicRobot *aHS,
				       double ConstraintOnX,
				       double ConstraintOnY) :
  SimplePlugin(aSPM)
{
  double lHalfHeightInit,lHalfWidthInit;

  m_HS = aHS;
  // Read humanoid specificities.
  m_RightFoot = m_HS->rightFoot();
  if (m_RightFoot==0)
    {
      cerr << "Problem with the reading of the right foot"<< endl;
    }
  m_RightFoot->getSoleSize(lHalfWidthInit,lHalfHeightInit);

  m_RightFootSize.setHalfSizeInit(lHalfWidthInit,lHalfHeightInit);
  m_RightFootSize.setConstraints(ConstraintOnX,ConstraintOnY);
  vector3d AnklePosition;
  m_RightFoot->getAnklePositionInLocalFrame(AnklePosition);
  m_Z = AnklePosition[2];
  m_LeftFoot = m_HS->leftFoot();
  if (m_RightFoot==0)
    {
      cerr << "Problem with the reading of the left foot"<< endl;
    }

  m_LeftFoot->getSoleSize(lHalfWidthInit,lHalfHeightInit);
  m_LeftFootSize.setHalfSizeInit(lHalfWidthInit,lHalfHeightInit);
  m_LeftFootSize.setConstraints(ConstraintOnX,ConstraintOnY);
  
  m_DSFeetDistance = 0.2;


  m_ConvexHullFP.resize(5);

  //initFPConstrArrays();
  //TODO 0: find another condition
  if(0)
    {
      RESETDEBUG4("Constraints-fCSALS.dat");
    }

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
  double lxcoefsRight[4] = { 1.0, 1.0, -1.0, -1.0};
  double lycoefsRight[4] = {-1.0, 1.0,  1.0, -1.0};
  double lxcoefsLeft[4] = { 1.0, 1.0, -1.0, -1.0};
  double lycoefsLeft[4] = { 1.0, -1.0, -1.0, 1.0};

  double *lxcoefs, *lycoefs;

  double CHLeftFPosConstrArrayX[5] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  double CHLeftFPosConstrArrayY[5] = {0.2, 0.3, 0.4, 0.3, 0.2};
  double CHRightFPosConstrArrayX[5] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  double CHRightFPosConstrArrayY[5] = {-0.2, -0.3, -0.4, -0.3, -0.2};

  vector<CH_Point> TheConvexHull;

  double s_t=0,c_t=0;
  double lx = 0.0,ly =0.0;

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
	  ly = -(double)PrwSupport.Foot*m_DSFeetDistance/2.0;

	  if(PrwSupport.Foot == 1)
	    {
	      FootHalfWidth = m_LeftFootSize.getHalfWidth();
	      FootHalfHeight = m_LeftFootSize.getHalfHeightDS();

	      lxcoefs = lxcoefsLeft;
	      lycoefs = lycoefsLeft;
	    }
	  else
	    {
	      FootHalfWidth = m_RightFootSize.getHalfWidth();
	      FootHalfHeight = m_RightFootSize.getHalfHeightDS();

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
	      FootHalfWidth = m_LeftFootSize.getHalfWidth();
	      FootHalfHeight = m_LeftFootSize.getHalfHeight();

	      lxcoefs = lxcoefsLeft;
	      lycoefs = lycoefsLeft;
	    }
	  else
	    {
	      FootHalfWidth = m_RightFootSize.getHalfWidth();
	      FootHalfHeight = m_RightFootSize.getHalfHeight();

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
	      m_CHFPosConstrArrayX = CHLeftFPosConstrArrayX;
	      m_CHFPosConstrArrayY = CHLeftFPosConstrArrayY;
	    }
	  else
	    {
	      m_CHFPosConstrArrayX = CHRightFPosConstrArrayX;
	      m_CHFPosConstrArrayY = CHRightFPosConstrArrayY;
	    }

	  //TODO: The interior border does not yet depend on the angle
	  for(unsigned j=0;j<5;j++)
	    {
	      m_ConvexHullFP[j].col = lx + ( m_CHFPosConstrArrayX[j] * c_t - m_CHFPosConstrArrayY[j] * s_t );
	      m_ConvexHullFP[j].row = ly + ( m_CHFPosConstrArrayX[j] * s_t + m_CHFPosConstrArrayY[j] * c_t );
	    }

	  LinearConstraintInequalityFreeFeet_t aLCIFP;

	  computeLinearSystem(m_ConvexHullFP, aLCIFP.D, aLCIFP.Dc, PrwSupport);

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

	  IndexConstraint += PrwSupport.StepNumber*MAL_MATRIX_NB_ROWS(LCIFF_it->D);
	}

      NbOfConstraints = IndexConstraint;

   }


  ODEBUG("Size of the 5 ms array: "<< LeftFootAbsolutePositions.size());
  ODEBUG("Size of the queue of Linear Constraint Inequalities " << QueueOfLConstraintInequalitiesFreeFeet.size());
  return 0;
}

void FootConstraintsAsLinearSystemForVelRef::CallMethod(std::string &Method, std::istringstream &Args)
{
  if (Method==":setfeetconstraint")
    {
      string lCmd;
      Args >> lCmd;

      if (lCmd=="XY")

	{

	  Args >> m_ConstraintOnX;
	  Args >> m_ConstraintOnY;

	  m_RightFootSize.setConstraints(m_ConstraintOnX, m_ConstraintOnY);
	  m_LeftFootSize.setConstraints(m_ConstraintOnX, m_ConstraintOnY);

	}

    }
}

/** \file FootConstraintAsLinearSystemForVelRef.cpp
    \brief This object build linear constraints based on feet positions.

    Copyright (c) 2005-2010,
    @author Andrei Herdt, Olivier Stasse

    JRL-Japan, CNRS/AIST

    All rights reserved.

    See License.txt file for more information on license
*/

#include <iostream>
#include <fstream>

#include <Mathematics/FootConstraintsAsLinearSystemForStepPos.h>

using namespace std;
using namespace PatternGeneratorJRL;

#include <Debug.h>



FootConstraintsAsLinearSystemForStepPos::
FootConstraintsAsLinearSystemForStepPos(SimplePluginManager *aSPM, 
				       CjrlHumanoidDynamicRobot *aHS, 
				       double ConstraintOnX, 
				       double ConstraintOnY) :
 FootConstraintsAsLinearSystemForVelRef(aSPM,aHS,ConstraintOnX,ConstraintOnY)
{
}

FootConstraintsAsLinearSystemForStepPos::
~FootConstraintsAsLinearSystemForStepPos()
{
  
}



// Assuming that the points are going counter-clockwise
int FootConstraintsAsLinearSystemForStepPos::buildLinearConstraintInequalities(deque< FootAbsolutePosition> &LeftFootAbsolutePositions,
								     deque<FootAbsolutePosition> &RightFootAbsolutePositions,
								     deque<LinearConstraintInequalityFreeFeet_t> &
								     QueueOfLConstraintInequalitiesFreeFeet,
								     deque<LinearConstraintInequalityFreeFeet_t> &
								     QueueOfFeetPosInequalities,
									 ReferenceAbsoluteVelocity & RefVel,
								     RelativeFootPositionQueue & RefPos,
								     double StartingTime, double m_QP_N,
								     SupportState * Support, 
								     deque<double> &PreviewedSupportAngles,
								     int & NbOfConstraints)
{

  //  printf("Entered buildLinearConstraintInequalities \n");

  // ComputeCH=0;
  // lx=0.0, ly=0.0;

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


  if(Support->CurrentSupportFoot==1)
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

  for(unsigned int i=1;i<=m_QP_N;i++)
    {

      Support->setSupportState(StartingTime, i, RefVel);

      ComputeCH=0;

      TheConvexHull.resize(4);//As for now only ZMP constraints

      //Andremize: theta = 0 as for now

      if(Support->m_StateChanged && Support->StepNumber>0)
	{
	  s_t = sin(PreviewedSupportAngles[Support->StepNumber-1]);
	  c_t = cos(PreviewedSupportAngles[Support->StepNumber-1]);

	  if(m_FullDebug>2)
	    {
	      ofstream aof;
	      aof.open("/tmp/SupportOrientations.dat",ofstream::app);
	      aof<<" PreviewedAngle: "<<PreviewedSupportAngles[Support->StepNumber-1];
	      aof.close();
	    }
	}

      double FootHalfWidth, FootHalfHeight;

      //Prepare the computation of the convex hull
      if(Support->PrwSupportPhase == 0)
	{
	  //Andremize: theta == 0
	  lx = 0.0;
	  ly = -(double)Support->PrwSupportFoot*DSFeetDistance/2.0;

	  if(Support->PrwSupportFoot == 1)
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
	  //Andremize: theta == 0
	  lx = 0.0;
	  ly = 0.0;

	  if(Support->PrwSupportFoot == 1)
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
      if(Support->m_StateChanged && Support->StepNumber>0)
	{

		
	  //Andremize: theta == 0
	  lx = 0.0;
	  ly = 0.0;
	  //Andremize: Has to be the angle of the previous foot

	  if(Support->StepNumber==1)
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
	      s_t = sin(PreviewedSupportAngles[Support->StepNumber-2]);
	      c_t = cos(PreviewedSupportAngles[Support->StepNumber-2]);

	      if(m_FullDebug>2)
		{
		  ofstream aof;
		  aof.open("/tmp/SupportOrientations.dat",ofstream::app);
		  aof<<" AngleFootConstraints: "<<PreviewedSupportAngles[Support->StepNumber-2];
		  aof.close();
		}
	    }//Left foot or rigth foot support
	  if(Support->PrwSupportFoot == 1)
	    {
	      CHFPosConstrArrayX = CHLeftFPosConstrArrayX;
	      CHFPosConstrArrayY = CHLeftFPosConstrArrayY;
	    }
	  else
	    {
	      CHFPosConstrArrayX = CHRightFPosConstrArrayX;
	      CHFPosConstrArrayY = CHRightFPosConstrArrayY;
	    }

	  //Andremize: The interior border does not yet depend on the angle
	  for(unsigned j=0;j<5;j++)
	    {
	      ConvexHullFP[j].col = lx + ( CHFPosConstrArrayX[j] * c_t - CHFPosConstrArrayY[j] * s_t );
	      ConvexHullFP[j].row = ly + ( CHFPosConstrArrayX[j] * s_t + CHFPosConstrArrayY[j] * c_t );
	    }

	  LinearConstraintInequalityFreeFeet_t aLCIFP;

	  computeLinearSystem(ConvexHullFP, aLCIFP.D, aLCIFP.Dc, Support);

	  aLCIFP.StepNumber = Support->StepNumber;

	  QueueOfFeetPosInequalities.push_back(aLCIFP);

	}

      // Linear Constraint Inequality
      LinearConstraintInequalityFreeFeet_t aLCI;
      

      // Building those constraints.
      //ComputeLinearSystem(TheConvexHull, aLCI->A, aLCI->B, aLCI->Center);
      //
      computeLinearSystem(TheConvexHull, aLCI.D, aLCI.Dc, Support);

      //For selection vectors and matrices when computing the constraints
      aLCI.StepNumber = Support->StepNumber;

      // Finding the similar one (i.e. Ai identical).
      //FindSimilarConstraints(aLCI->A,aLCI->SimilarConstraints);

      //aLCI->StartingTime = LeftFootAbsolutePositions[i].time;//???What to do

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
		
      if(Support->StepNumber>0) 
	{ 
	  LCIFF_it = QueueOfFeetPosInequalities.begin(); 
		    
	  IndexConstraint += double(Support->StepNumber)*  MAL_MATRIX_NB_ROWS(LCIFF_it->D); 
	} 
		
      NbOfConstraints = IndexConstraint; 

   }


  ODEBUG("Size of the 5 ms array: "<< LeftFootAbsolutePositions.size());
  ODEBUG("Size of the queue of Linear Constraint Inequalities " << QueueOfLConstraintInequalitiesFreeFeet.size());

  // printf("Leaving buildLinearConstraintInequalities \n");

  return 0;
}


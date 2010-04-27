/** \file footConstraintAsLinearSystem.cpp
    \brief This object build linear constraints based on feet positions.

   Copyright (c) 2005-2010,
   @author Andrei Herdt, Olivier Stasse

   JRL-Japan, CNRS/AIST

   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
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

#include <iostream>
#include <fstream>

#include <Mathematics/footConstraintsAsLinearSystem.h>

using namespace std;
using namespace PatternGeneratorJRL;


#if 1
#define RESETDEBUG4(y) { ofstream DebugFile; \
                         DebugFile.open(y,ofstream::out); \
                         DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; \
                       DebugFile.open(y,ofstream::app); \
                       DebugFile <<  x << endl; DebugFile.close();}
#else
#define RESETDEBUG4(y)
#define ODEBUG4(x,y)
#endif

#define RESETDEBUG6(y)
#define ODEBUG6(x,y)

#define RESETDEBUG5(y) { ofstream DebugFile; \
                         DebugFile.open(y,ofstream::out);\
                         DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; \
                       DebugFile.open(y,ofstream::app); \
                       DebugFile << x << endl; \
                       DebugFile.close();}
#define ODEBUG5NOE(x,y) { ofstream DebugFile; \
                          DebugFile.open(y,ofstream::app); \
                          DebugFile << x ; DebugFile.close();}
#if 1
#define ODEBUG(x)
#else
#define ODEBUG(x)  std::cout << "FootConstraintAsLinearSystem: " << x << endl;
#endif

#define ODEBUG3(x)  std::cout << "FootConstraintAsLinearSystem: " << x << endl;


footConstraintsAsLinearSystem::footConstraintsAsLinearSystem(SimplePluginManager *aSPM, CjrlHumanoidDynamicRobot *aHS, double ConstraintOnX, double ConstraintOnY) :
  SimplePlugin(aSPM)
{

  // printf("Entered footConstraintsAsLinearSystem \n");
  m_HS = aHS;
  // Read humanoid specificities.
  lRightFoot = m_HS->rightFoot();

  lRightFoot->getSoleSize(lRightFootHalfWidth,lRightFootHalfHeight);
  lRightFoot->getAnklePositionInLocalFrame(AnklePosition);
  lZ = AnklePosition[2];
  lLeftFoot = m_HS->leftFoot();
  lLeftFoot->getSoleSize(lLeftFootHalfWidth,lLeftFootHalfHeight);


  lRightFootHalfWidth *= 0.5;
  lRightFootHalfHeight *= 0.5;
  lLeftFootHalfWidth *= 0.5;
  lLeftFootHalfHeight *= 0.5;

  // lLeftFootHalfHeight -= ConstraintOnY;
  // lRightFootHalfHeight -= ConstraintOnY;

  // lLeftFootHalfWidth -= ConstraintOnX;
  // lRightFootHalfWidth -= ConstraintOnX;

  //Andremize: Scilab feet
  lLeftFootHalfHeight = 0.069-0.02;
  lRightFootHalfHeight = 0.069-0.02;

  lLeftFootHalfWidth = 0.1206-0.02;
  lRightFootHalfWidth = 0.1206-0.02;

  DSFeetDistance = 0.2;
  lLeftFootHalfHeightDS = lLeftFootHalfHeight+DSFeetDistance/2.0;
  lRightFootHalfHeightDS = lRightFootHalfHeight+DSFeetDistance/2.0;

   ConvexHullFP.resize(5);

  //initFPConstrArrays();
   //TODO 0: find another condition
   if(0)
	   RESETDEBUG4("Constraints-fCSALS.dat");

//   printf("Leaving footConstraintsAsLinearSystem \n");
}

footConstraintsAsLinearSystem::~footConstraintsAsLinearSystem()
{
  // if (CHLeftFPosConstrArrayX!=0)
  //   delete [] CHLeftFPosConstrArrayX;
  // if (CHLeftFPosConstrArrayY!=0)
  //   delete [] CHLeftFPosConstrArrayY;
  // if (CHRightFPosConstrArrayX!=0)
  //   delete [] CHRightFPosConstrArrayX;
  // if (CHRightFPosConstrArrayY!=0)
  //   delete [] CHRightFPosConstrArrayY;
  //  if (CHFPosConstrArrayX!=0)
  //   delete [] CHFPosConstrArrayX;
  // if (CHFPosConstrArrayY!=0)
  //   delete [] CHFPosConstrArrayY;
}

// void footConstraintsAsLinearSystem::initFPConstrArrays()
// {

//   // CHLeftFPosConstrArrayX[0]=-0.3;CHLeftFPosConstrArrayX[1]=-0.2;CHLeftFPosConstrArrayX[2]=0.0;
//   // CHLeftFPosConstrArrayX[3]=0.2;CHLeftFPosConstrArrayX[4]=0.3;

//   CHLeftFPosConstrArrayY[0]=0.15;CHLeftFPosConstrArrayY[1]=0.3;CHLeftFPosConstrArrayY[2]=0.4;
//   CHLeftFPosConstrArrayY[3]=0.3;CHLeftFPosConstrArrayY[4]=0.15;

//   CHRightFPosConstrArrayX[0]=-0.3;CHRightFPosConstrArrayX[1]=-0.2;CHRightFPosConstrArrayX[2]=0.0;
//   CHRightFPosConstrArrayX[3]=0.2;CHRightFPosConstrArrayX[4]=0.3;

//   CHRightFPosConstrArrayY[0]=-0.15;CHRightFPosConstrArrayY[1]=-0.3;CHRightFPosConstrArrayY[2]=0.4;
//   CHRightFPosConstrArrayY[3]=-0.3;CHRightFPosConstrArrayY[4]=-0.15;

// }

int footConstraintsAsLinearSystem::FindSimilarConstraints(MAL_MATRIX(&A,double),
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
int footConstraintsAsLinearSystem::computeLinearSystem(vector<CH_Point> aVecOfPoints,
						       MAL_MATRIX(&D,double),
						       MAL_MATRIX(&Dc,double),
						       SupportState * Support
						       )
{
  // printf("Entered computeLinearSystem \n");




  double dx,dy,dc,x1,y1,x2,y2;
  unsigned int n = aVecOfPoints.size();
  MAL_MATRIX_RESIZE(D,aVecOfPoints.size(),2);
  MAL_MATRIX_RESIZE(Dc,aVecOfPoints.size(),1);


  // Dump a file to display on scilab .
  // This should be removed during real usage inside a robot.
  if (0)
    {
      ofstream aof;
      aof.open("Constraints-fCSALS.dat",ofstream::app);
      for(unsigned int i=0;i<n-1;i++)
  	{
  	  aof << aVecOfPoints[i].col << " " <<  aVecOfPoints[i].row << " "
  	      << aVecOfPoints[i+1].col << " "  << aVecOfPoints[i+1].row << endl;
  	}
      aof << aVecOfPoints[n-1].col << " " <<  aVecOfPoints[n-1].row << " "
  	  << aVecOfPoints[0].col << " "  << aVecOfPoints[0].row << endl;
      aof.close();
    }

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
      dx = (double)Support->PrwSupportFoot*dx;
      dy = (double)Support->PrwSupportFoot*dy;
      dc = (double)Support->PrwSupportFoot*dc;

      D(i,0) = dx; D(i,1)= dy;
      Dc(i,0) = dc;

      //C is not filled

      //ODEBUG4("D("<<i<<",:): " <<dx<<" "<<dy<<" Dc("<<i<<"): " << dc,"Constraints-fCSALS.dat");
      // ODEBUG4(" Dc("<<i<<"): " << dc,"Constraints-fCSALS.dat");
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
    dx = (double)Support->PrwSupportFoot*dx;
    dy = (double)Support->PrwSupportFoot*dy;
    dc = (double)Support->PrwSupportFoot*dc;

    D(i,0) = dx; D(i,1)= dy;
    Dc(i,0) = dc;

    //C is not filled

    //ODEBUG4("D("<<i<<",:): " <<dx<<" "<<dy<<" Dc("<<i<<"): " << dc,"Constraints-fCSALS.dat");
    // ODEBUG4(" Dc("<<i<<"): " << dc,"Constraints-fCSALS.dat");
  }


  //ODEBUG4(" \n","Constraints-fCSALS.dat");

  // printf("Finished computeLinearSystem \n");

  return 0;
}


int footConstraintsAsLinearSystem::buildLinearConstraintInequalities(deque<FootAbsolutePosition>
								     &LeftFootAbsolutePositions,
								     deque<FootAbsolutePosition>
								     &RightFootAbsolutePositions,
								     deque<LinearConstraintInequalityFreeFeet_t *> &
								     QueueOfLConstraintInequalitiesFreeFeet,
								     deque<LinearConstraintInequalityFreeFeet_t *> &
								     QueueOfFeetPosInequalities,
								     ReferenceAbsoluteVelocity & RefVel,
								     double StartingTime,
								     double m_QP_N,
								     SupportState * Support)
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

  float CHLeftFPosConstrArrayX[5] = {-0.3, -0.2, 0.0, 0.2, 0.3};
  float CHLeftFPosConstrArrayY[5] = {0.2, 0.3, 0.4, 0.3, 0.2};
  float CHRightFPosConstrArrayX[5] = {-0.3, -0.2, 0.0, 0.2, 0.3};
  float CHRightFPosConstrArrayY[5] = {-0.2, -0.3, -0.4, -0.3, -0.2};

  vector<CH_Point> TheConvexHull;


  // Going through the set of generated data for each 5 ms.
  // from this extract a set of linear constraints.
  for(unsigned int i=1;i<=m_QP_N;i++)
    {

      Support->setSupportState(StartingTime, i, RefVel);

      ComputeCH=0;

      TheConvexHull.resize(4);//As for now only ZMP constraints

      //Andremize: theta = 0 as for now
      s_t = 0.0;
      c_t = 1.0;

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
	  TheConvexHull[j].col = lx + ( lxcoefs[j] *
					FootHalfWidth * c_t -
					lycoefs[j] *
					FootHalfHeight * s_t );
	  TheConvexHull[j].row = ly + ( lxcoefs[j] *
					FootHalfWidth * s_t +
					lycoefs[j] *
					FootHalfHeight * c_t );
	}

      if(Support->StateChanged && Support->StepNumber>0)
      	{
      	  //Andremize: theta == 0
      	  lx = 0.0;
      	  ly = 0.0;
      	  //Andremize: Has to be the angle of the previous foot
      	  s_t = 0.0;
      	  c_t = 1.0;

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

      	  LinearConstraintInequalityFreeFeet_t * aLCIFP = new LinearConstraintInequalityFreeFeet_t;

      	  computeLinearSystem(ConvexHullFP, aLCIFP->D, aLCIFP->Dc, Support);

      	  aLCIFP->StepNumber = Support->StepNumber;

      	  QueueOfFeetPosInequalities.push_back(aLCIFP);

      	}

      // Linear Constraint Inequality
      LinearConstraintInequalityFreeFeet_t * aLCI = new LinearConstraintInequalityFreeFeet_t;


      // Building those constraints.
      //ComputeLinearSystem(TheConvexHull, aLCI->A, aLCI->B, aLCI->Center);
      //
      computeLinearSystem(TheConvexHull, aLCI->D, aLCI->Dc, Support);

      //For selection vectors and matrices when computing the constraints
      aLCI->StepNumber = Support->StepNumber;

      // cout<<Support->StepNumber<<" "<<aLCI->StepNumber<<endl;
      // Finding the similar one (i.e. Ai identical).
      //FindSimilarConstraints(aLCI->A,aLCI->SimilarConstraints);

      //aLCI->StartingTime = LeftFootAbsolutePositions[i].time;//???What to do

      QueueOfLConstraintInequalitiesFreeFeet.push_back(aLCI);

    }

  // printf("Left the preview loop \n");

ODEBUG("Size of the 5 ms array: "<< LeftFootAbsolutePositions.size());
ODEBUG("Size of the queue of Linear Constraint Inequalities " << QueueOfLConstraintInequalitiesFreeFeet.size());

 // printf("Leaving buildLinearConstraintInequalities \n");

 return 0;
}

void footConstraintsAsLinearSystem::CallMethod(std::string &Method, std::istringstream &Args)
{
  // TO BE EXTENDED.
}

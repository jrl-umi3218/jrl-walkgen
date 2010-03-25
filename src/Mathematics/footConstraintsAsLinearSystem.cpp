/** \file FootConstraintAsLinearSystem.cpp
    \brief This object build linear constraints based on feet positions.

   Copyright (c) 2005-2009, 
   @author Olivier Stasse
   
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
  
  lLeftFootHalfHeight -= ConstraintOnY;
  lRightFootHalfHeight -= ConstraintOnY;

  lLeftFootHalfWidth -= ConstraintOnX;
  lRightFootHalfWidth -= ConstraintOnX;
  
    
  RESETDEBUG4("Constraints-fCSALS.dat");
 
  // printf("Leaving footConstraintsAsLinearSystem \n");
}

footConstraintsAsLinearSystem::~footConstraintsAsLinearSystem()
{
}

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
						       MAL_MATRIX(&Dc,double)
						       )
{
  // printf("Entered computeLinearSystem \n");

  double dx,dy,dc,x1,y1,x2,y2;
  unsigned int n = aVecOfPoints.size();
  MAL_MATRIX_RESIZE(D,aVecOfPoints.size(),2);
  MAL_MATRIX_RESIZE(Dc,aVecOfPoints.size(),1);


  // // Dump a file to display on scilab .
  // // This should be removed during real usage inside a robot.
  // if (1)
  //   {
  //     ofstream aof;
  //     aof.open("Constraints-fCSALS.dat",ofstream::app);
  //     for(unsigned int i=0;i<n-1;i++)
  // 	{
  // 	  aof << aVecOfPoints[i].col << " " <<  aVecOfPoints[i].row << " "
  // 	      << aVecOfPoints[i+1].col << " "  << aVecOfPoints[i+1].row << endl;
  // 	}
  //     aof << aVecOfPoints[n-1].col << " " <<  aVecOfPoints[n-1].row << " "
  // 	  << aVecOfPoints[0].col << " "  << aVecOfPoints[0].row << endl;
  //     aof.close();
  //   }
  
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
      
      /*symmetrical constraints cannot be achieved without knowledge of the support foot
	dx = -1.0*dx;
	dy = -1.0*dy;
	dc = -1.0*dc;
      */

      D(i,0) = dx; D(i,1)= dy;
      Dc(i,0) = dc;

      //C is not filled

    ODEBUG4("D("<<i<<",:): " <<dx<<" "<<dy,"Constraints-fCSALS.dat");
    ODEBUG4(" Dc("<<i<<"): " << dc,"Constraints-fCSALS.dat");
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
      
    /*symmetrical constraints cannot be achieved without knowledge of the support foot
      dx = -1.0*dx;
      dy = -1.0*dy;
      dc = -1.0*dc;
    */

    D(i,0) = dx; D(i,1)= dy;
    Dc(i,0) = dc;

    //C is not filled

    ODEBUG4("D("<<i<<",:): " <<dx<<" "<<dy,"Constraints-fCSALS.dat");
    ODEBUG4(" Dc("<<i<<"): " << dc,"Constraints-fCSALS.dat");
  }

  
  ODEBUG4(" \n","Constraints-fCSALS.dat");
      
  // printf("Finished computeLinearSystem \n");

  return 0;
}


int footConstraintsAsLinearSystem::buildLinearConstraintInequalities(deque<FootAbsolutePosition> 
								     &LeftFootAbsolutePositions,
								     deque<FootAbsolutePosition> 
								     &RightFootAbsolutePositions,
								     deque<LinearConstraintInequalityFreeFeet_t *> &
								     QueueOfLConstraintInequalitiesFreeFeet,
								     double Ref[3],
								     double StartingTime,
								     double m_QP_N,
								     SupportState * Support)
{

  // printf("Entered buildLinearConstraintInequalities \n");

  ComputeCH=0;
  lx=0.0, ly=0.0;

  float lxcoefs[4] = { 1.0, 1.0, -1.0, -1.0};
  float lycoefs[4] = {-1.0, 1.0,  1.0, -1.0};

  vector<CH_Point> TheConvexHull;

  // Going through the set of generated data for each 5 ms.
  // from this extract a set of linear constraints.
  for(unsigned int i=1;i<=m_QP_N;i++)
    {
      
      Support->setSupportState(StartingTime, i, Ref);

      ComputeCH=0;
      
      TheConvexHull.resize(4);//As for now only ZMP constraints
	
 
      // Which support foot ?
      if (Support->PrwSupportFoot == 1)//Only simple support as for now
	{
	  //should be simplified as now in a local frame
	  lx=0.0;
	  ly=0.0;
		  
	  s_t = 0.0; 
	  c_t = 1.0;
	  for(unsigned j=0;j<4;j++)
	    {
	      TheConvexHull[j].col = lx + 
		( lxcoefs[j] * lLeftFootHalfWidth * c_t -
		  lycoefs[j] * lLeftFootHalfHeight * s_t ); 
	      TheConvexHull[j].row = ly + 
		( lxcoefs[j] * lLeftFootHalfWidth * s_t + 
		  lycoefs[j] * lLeftFootHalfHeight * c_t ); 
	    }
	  ODEBUG("Left support foot");
	}
      else
	{ 
	  lx=0.0;
	  ly=0.0;
		  
	  s_t = 0.0; 
	  c_t = 1.0;
		       
	  for(unsigned j=0;j<4;j++)
	    {
	      TheConvexHull[j].col = lx + ( lxcoefs[j] * 
					    lRightFootHalfWidth * c_t -
					    lycoefs[j] * 
					    lRightFootHalfHeight * s_t );
	      TheConvexHull[j].row = ly + ( lxcoefs[j] * 
					    lRightFootHalfWidth * s_t +
					    lycoefs[j] * 
					    lRightFootHalfHeight * c_t ); 

	    }
	  ODEBUG("Right support foot");
	}
	      
      // Linear Constraint Inequality
      LinearConstraintInequalityFreeFeet_t * aLCI = new LinearConstraintInequalityFreeFeet_t;
      // Building those constraints.
      //ComputeLinearSystem(TheConvexHull, aLCI->A, aLCI->B, aLCI->Center);
      //
      computeLinearSystem(TheConvexHull, aLCI->D, aLCI->Dc);
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

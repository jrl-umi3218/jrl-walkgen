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
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
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

#include <jrlMathTools/jrlConstants.h>

#include <dynamicsJRLJapan/HumanoidSpecificities.h>

#include <walkGenJrl/Mathematics/FootConstraintsAsLinearSystem.h>

using namespace std;
using namespace dynamicsJRLJapan;
using namespace PatternGeneratorJRL;


#if 0
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


FootConstraintsAsLinearSystem::FootConstraintsAsLinearSystem(SimplePluginManager *aSPM,
							     HumanoidSpecificities *aHS) :
  SimplePlugin(aSPM)
{
  m_HS = aHS;
  RESETDEBUG4("Constraints-FCSALS.dat");
}

FootConstraintsAsLinearSystem::~FootConstraintsAsLinearSystem()
{
}

// Assuming that the points are going counter-clockwise
// and that the foot's interior is at the left of the points.
// The result is : A [ Zx(k), Zy(k)]' + B  >=0
int FootConstraintsAsLinearSystem::ComputeLinearSystem(vector<CH_Point> aVecOfPoints, 
						       MAL_MATRIX(&A,double),
						       MAL_MATRIX(&B,double),
						       MAL_VECTOR(&C,double))
{
  double a,b,c;
  unsigned int n = aVecOfPoints.size();
  MAL_MATRIX_RESIZE(A,aVecOfPoints.size(),2);
  MAL_MATRIX_RESIZE(B,aVecOfPoints.size(),1);
  MAL_VECTOR_RESIZE(C,3);

  // Dump a file to display on scilab .
  // This should be removed during real usage inside a robot.
  if (1)
    {
      ofstream aof;
      aof.open("Constraints-FCSALS.dat",ofstream::app);
      for(unsigned int i=0;i<n-1;i++)
	{
	  aof << aVecOfPoints[i].col << " " <<  aVecOfPoints[i].row << " "
	      << aVecOfPoints[i+1].col << " "  << aVecOfPoints[i+1].row << endl;
	}
      aof << aVecOfPoints[n-1].col << " " <<  aVecOfPoints[n-1].row << " "
	  << aVecOfPoints[0].col << " "  << aVecOfPoints[0].row << endl;
      aof.close();
    }
  
  for(unsigned int i=0;i<n-1;i++)
    {
      // Compute center of the convex hull.
      C(0)+= aVecOfPoints[i].col;
      C(1)+= aVecOfPoints[i].row;
      C(2)=0.0;
	
      ODEBUG("(x["<< i << "],y["<<i << "]): " << aVecOfPoints[i].col << " " <<  aVecOfPoints[i].row << " "
	     << aVecOfPoints[i+1].col << " "  << aVecOfPoints[i+1].row );

      if (fabs(aVecOfPoints[i+1].col-aVecOfPoints[i].col)>1e-7)
	{
	  double y1,x1,y2,x2,lmul=-1.0;

	  if (aVecOfPoints[i+1].col < aVecOfPoints[i].col)
	    {
	      lmul=1.0;
	      y2 = aVecOfPoints[i].row;
	      y1 = aVecOfPoints[i+1].row;
	      x2 = aVecOfPoints[i].col;
	      x1 = aVecOfPoints[i+1].col;
	    }
	  else
	    {
	      y2 = aVecOfPoints[i+1].row;
	      y1 = aVecOfPoints[i].row;
	      x2 = aVecOfPoints[i+1].col;
	      x1 = aVecOfPoints[i].col;	 
	    }
	  
	  
	  a = (y2 - y1)/(x2-x1) ;
	  b = (aVecOfPoints[i].row - a * aVecOfPoints[i].col);

	  a = lmul*a;
	  b = lmul*b;
	  c= -lmul;
	  

	}
      else
	{
	  c = 0.0;
	  a = -1.0;	  
	  b = aVecOfPoints[i+1].col;
	  if (aVecOfPoints[i+1].row < aVecOfPoints[i].row)
	    {
	      a=-a;
	      b=-b;
	    }
	}
      
      
      A(i,0) = a; A(i,1)= c;
      B(i,0) = b;

    }

  C(0) /= (double)aVecOfPoints.size();
  C(1) /= (double)aVecOfPoints.size();
  C(2) =0.0;

  ODEBUG("(x["<< n-1 << "],y["<< n-1 << "]): " << aVecOfPoints[n-1].col << " " <<  aVecOfPoints[n-1].row << " "
	 << aVecOfPoints[0].col << " "  << aVecOfPoints[0].row );
  
  if (fabs(aVecOfPoints[0].col-aVecOfPoints[n-1].col)>1e-7)
    {
      double y1,x1,y2,x2,lmul=-1.0;
      
      if (aVecOfPoints[0].col < aVecOfPoints[n-1].col)
	{
	  lmul=1.0;
	  y2 = aVecOfPoints[n-1].row;
	  y1 = aVecOfPoints[0].row;
	  x2 = aVecOfPoints[n-1].col;
	  x1 = aVecOfPoints[0].col;
	}
      else
	{
	  y2 = aVecOfPoints[0].row;
	  y1 = aVecOfPoints[n-1].row;
	  x2 = aVecOfPoints[0].col;
	  x1 = aVecOfPoints[n-1].col;
	  
	}
      
      
      a = (y2 - y1)/(x2-x1) ;
      b = (aVecOfPoints[0].row - a * aVecOfPoints[0].col);
      
      a = lmul*a;
      b = lmul*b;
      c= -lmul;
      
    }
  else
    {
      c = 0.0;
      a = -1.0;	  
      b = aVecOfPoints[0].col;
      if (aVecOfPoints[0].row < aVecOfPoints[n-1].row)
	{
	  a=-a;
	  b=-b;
	}
    }

  
  A(n-1,0) = a; A(n-1,1)= c;
  B(n-1,0) = b;
  
  
  ODEBUG("A: " << A );
  ODEBUG("B: " << B);
      
  return 0;
}

int FootConstraintsAsLinearSystem::BuildLinearConstraintInequalities(deque<FootAbsolutePosition> 
								      &LeftFootAbsolutePositions,
								      deque<FootAbsolutePosition> 
								      &RightFootAbsolutePositions,
								      deque<LinearConstraintInequality_t *> &
								      QueueOfLConstraintInequalities,
								      double ConstraintOnX,
								      double ConstraintOnY)
{
  // Find the convex hull for each of the position,
  // in order to create the corresponding trajectory.
  ComputeConvexHull aCH;
  double lLeftFootHalfWidth,lLeftFootHalfHeight,
    lRightFootHalfWidth,lRightFootHalfHeight,lZ;
  
  // Read humanoid specificities.
  m_HS->GetFootSize(-1,lRightFootHalfWidth,lRightFootHalfHeight,lZ);
  m_HS->GetFootSize(1,lLeftFootHalfWidth,lLeftFootHalfHeight,lZ);

  lRightFootHalfWidth *= 0.5;
  lRightFootHalfHeight *= 0.5;
  lLeftFootHalfWidth *= 0.5;
  lLeftFootHalfHeight *= 0.5;
  
  lLeftFootHalfHeight -= ConstraintOnY;
  lRightFootHalfHeight -= ConstraintOnY;

  lLeftFootHalfWidth -= ConstraintOnX;
  lRightFootHalfWidth -= ConstraintOnX;
  
  if (LeftFootAbsolutePositions.size()!=
      RightFootAbsolutePositions.size())
    return -1;
  
  int State=0; // State for the system 0:start, 1: Right Support Foot, 2: Left Support Foot,
  // 3: Double Support.
  int ComputeCH=0;
  float lx=0.0, ly=0.0;
  float lxcoefs[4] = { 1.0, 1.0, -1.0, -1.0};
  float lycoefs[4] = {-1.0, 1.0,  1.0, -1.0};
  double prev_xmin=1e7, prev_xmax=-1e7, prev_ymin=1e7, prev_ymax=-1e7;
  RESETDEBUG4("ConstraintMax.dat");

  double s_t,c_t;

  // Going through the set of generated data for each 5 ms.
  // from this extract a set of linear constraints.
  for(unsigned int i=0;i<LeftFootAbsolutePositions.size();i++)
    {
      
      ComputeCH=0;
      // First check if we have to compute a convex hull
      if (i==0)
	{
	  ComputeCH = 1;
	  State=3;
	}
      // Double support
      if (LeftFootAbsolutePositions[i].stepType>=10)
	{
	  if (State!=3)
	    ComputeCH=1;
	  State =3;
	}
      else
	{
	  double LiftingThreshold=0.00001;
	  if (LeftFootAbsolutePositions[i].z>LiftingThreshold)
	    {
	      if (State!=2)
		ComputeCH=1;
	      State=2;
	    }
	  else if (RightFootAbsolutePositions[i].z>LiftingThreshold)
	    {
	      if (State!=1)
		ComputeCH=1;
	      State=1;
	    }
	  else if ((RightFootAbsolutePositions[i].z<LiftingThreshold) &&
		   (LeftFootAbsolutePositions[i].z<LiftingThreshold))
	    {
	      if (State!=3)
		ComputeCH=1;
	      State=3;
	    } 

	  
	}

      if (ComputeCH)
	{
	  double xmin=1e7, xmax=-1e7, ymin=1e7, ymax=-1e7;

	  ODEBUG("LeftFootAbsolutePositions[" << i << " ].theta= " << 
		  LeftFootAbsolutePositions[i].theta);

	  ODEBUG("RightFootAbsolutePositions[" << i << " ].theta= " << 
		  RightFootAbsolutePositions[i].theta);
	      
	  vector<CH_Point> TheConvexHull;
	  // Check if we are in a single or double support phase,
	  // by testing the step type. In double support phase
	  // the value is greater than or equal to 10.
	  // In this case, we have to compute the convex hull
	  // of both feet.
	  if (State==3)
	    {
	      vector<CH_Point> aVecOfPoints;
	      
	      aVecOfPoints.resize(8);

	      lx=LeftFootAbsolutePositions[i].x;
	      ly=LeftFootAbsolutePositions[i].y;
	      
	      s_t = sin(LeftFootAbsolutePositions[i].theta*M_PI/180.0); 
	      c_t = cos(LeftFootAbsolutePositions[i].theta*M_PI/180.0);  
	      for(unsigned j=0;j<4;j++)
		{
		  aVecOfPoints[j].col = lx + ( lxcoefs[j] * lLeftFootHalfWidth 
					       * c_t 
					       - lycoefs[j] * 
					       lLeftFootHalfHeight * s_t );
		  aVecOfPoints[j].row = ly + ( lxcoefs[j] * lLeftFootHalfWidth 
					       * s_t 
					       + lycoefs[j] * 
					       lLeftFootHalfHeight * c_t );


		  // Computes the maxima.
		  xmin = aVecOfPoints[j].col < xmin ? aVecOfPoints[j].col : xmin;
		  xmax = aVecOfPoints[j].col > xmax ? aVecOfPoints[j].col : xmax;
		  ymin = aVecOfPoints[j].row < ymin ? aVecOfPoints[j].row : ymin;
		  ymax = aVecOfPoints[j].row > ymax ? aVecOfPoints[j].row : ymax;
		  
		}
	      ODEBUG("State 3-1 " << xmin << " " << xmax << " " << ymin << " " << ymax);
	      lx=RightFootAbsolutePositions[i].x;
	      ly=RightFootAbsolutePositions[i].y;

	      
	      s_t = sin(RightFootAbsolutePositions[i].theta*M_PI/180.0); //+
	      c_t = cos(RightFootAbsolutePositions[i].theta*M_PI/180.0); //+ 
	      
	      ODEBUG("Right Foot: " << lx << " " << ly << " " << RightFootAbsolutePositions[i].theta);
	      for(unsigned j=0;j<4;j++)
		{
		  aVecOfPoints[j+4].col = lx + ( lxcoefs[j] * lRightFootHalfWidth
						 * c_t - lycoefs[j] * 
						 lRightFootHalfHeight * s_t ); 
		  aVecOfPoints[j+4].row = ly + ( lxcoefs[j] * lRightFootHalfWidth
						 * s_t + lycoefs[j] * 
						 lRightFootHalfHeight * c_t );  
		  // Computes the maxima.
		  xmin = aVecOfPoints[j+4].col < xmin ? aVecOfPoints[j+4].col : xmin;
		  xmax = aVecOfPoints[j+4].col > xmax ? aVecOfPoints[j+4].col : xmax;
		  ymin = aVecOfPoints[j+4].row < ymin ? aVecOfPoints[j+4].row : ymin;
		  ymax = aVecOfPoints[j+4].row > ymax ? aVecOfPoints[j+4].row : ymax;
		  
		}

	      ODEBUG("State 3-2" << xmin << " " << xmax << " " << ymin << " " << ymax);
	      aCH.DoComputeConvexHull(aVecOfPoints,TheConvexHull);
	    }
	  // In the second case, it is necessary to compute 
	  // the support foot.
	  else
	    {
	      
	      TheConvexHull.resize(4);
	      
	      // Who is support foot ?
	      if (LeftFootAbsolutePositions[i].z < RightFootAbsolutePositions[i].z)
		{
		  lx=LeftFootAbsolutePositions[i].x;
		  ly=LeftFootAbsolutePositions[i].y;
		  
		  s_t = sin(LeftFootAbsolutePositions[i].theta*M_PI/180.0); 
		  c_t = cos(LeftFootAbsolutePositions[i].theta*M_PI/180.0);
 		  for(unsigned j=0;j<4;j++)
		    {
		      TheConvexHull[j].col = lx + 
			( lxcoefs[j] * lLeftFootHalfWidth * c_t -
			  lycoefs[j] * lLeftFootHalfHeight * s_t ); 
		      TheConvexHull[j].row = ly + 
			( lxcoefs[j] * lLeftFootHalfWidth * s_t + 
			  lycoefs[j] * lLeftFootHalfHeight * c_t ); 
		      // Computes the maxima.
		      xmin = TheConvexHull[j].col < xmin ? TheConvexHull[j].col : xmin;
		      xmax = TheConvexHull[j].col > xmax ? TheConvexHull[j].col : xmax;
		      ymin = TheConvexHull[j].row < ymin ? TheConvexHull[j].row : ymin;
		      ymax = TheConvexHull[j].row > ymax ? TheConvexHull[j].row : ymax;

		    }
		  ODEBUG("Left support foot");
		}
	      else
		{
		  lx=RightFootAbsolutePositions[i].x;
		  ly=RightFootAbsolutePositions[i].y;
		  s_t = sin(RightFootAbsolutePositions[i].theta*M_PI/180.0); 
		  c_t = cos(RightFootAbsolutePositions[i].theta*M_PI/180.0);      
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
		      // Computes the maxima.
		      xmin = TheConvexHull[j].col < xmin ? TheConvexHull[j].col : xmin;
		      xmax = TheConvexHull[j].col > xmax ? TheConvexHull[j].col : xmax;
		      ymin = TheConvexHull[j].row < ymin ? TheConvexHull[j].row : ymin;
		      ymax = TheConvexHull[j].row > ymax ? TheConvexHull[j].row : ymax;

		    }
		  ODEBUG("Right support foot");
		}
	      ODEBUG("State !=3 " << xmin << " " << xmax << " " << ymin << " " << ymax);
	      
	    }

	  // Linear Constraint Inequality
	  LinearConstraintInequality_t * aLCI = new LinearConstraintInequality_t;
	  ComputeLinearSystem(TheConvexHull,aLCI->A, aLCI->B, aLCI->Center);
	  aLCI->StartingTime = LeftFootAbsolutePositions[i].time;
	  if (QueueOfLConstraintInequalities.size()>0)
	    {
	      QueueOfLConstraintInequalities.back()->EndingTime = LeftFootAbsolutePositions[i].time;
	      ODEBUG4( QueueOfLConstraintInequalities.back()->StartingTime << " " <<
		       QueueOfLConstraintInequalities.back()->EndingTime << " " <<
		       prev_xmin << " "  <<
		       prev_xmax << " "  <<
		       prev_ymin << " "  <<
		       prev_ymax
		       ,"ConstraintMax.dat");

	    }
	  ODEBUG("Final " << xmin << " " << xmax << " " << ymin << " " << ymax);
	  prev_xmin = xmin; prev_xmax = xmax;
	  prev_ymin = ymin; prev_ymax = ymax;

	  QueueOfLConstraintInequalities.push_back(aLCI);
	  
	  
	  
	}
      if (i==LeftFootAbsolutePositions.size()-1)
	{
	  if (QueueOfLConstraintInequalities.size()>0)
	    {
	      QueueOfLConstraintInequalities.back()->EndingTime = LeftFootAbsolutePositions[i].time;
	      ODEBUG4( QueueOfLConstraintInequalities.back()->StartingTime << " " <<
		       QueueOfLConstraintInequalities.back()->EndingTime << " " <<
		       prev_xmin << " "  <<
		       prev_xmax << " "  <<
		       prev_ymin << " "  <<
		       prev_ymax
		       ,"ConstraintMax.dat");

	    }
	}
    }

  ODEBUG("Size of the 5 ms array: "<< LeftFootAbsolutePositions.size());
  ODEBUG("Size of the queue of Linear Constraint Inequalities " << QueueOfLConstraintInequalities.size());
  
  return 0;
}

void FootConstraintsAsLinearSystem::CallMethod(std::string &Method, std::istringstream &Args)
{
  // TO BE EXTENDED.
}

/** This object is detecting the collisions between lower leg 
    and the obstacle if there are any, 
    and calculates necessary displacements to avoid the collision


   Copyright (c) 2005-2006, 
   Bjorn Verrelst
   Olivier Stasse,
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation 
   and/or other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
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

#define _DEBUG_
#include <fstream>
#include <MotionGeneration/CollisionDetector.h>


using namespace::PatternGeneratorJRL;

CollisionDetector::CollisionDetector()
{
 
}

CollisionDetector::~CollisionDetector()
{
 
}


void CollisionDetector::SetObstacleCoordinates(ObstaclePar aObstacleInfo)
{
  /*! ObstacleInfo contains x, y, z position of obstacle in worldframe 
    (point taken on the front plan of the obstacle on the floor and in 
    the middel of the width and also the orientation of the obstacle 
    in the X,Y plane of the world and the obstacle dimensions: depth, Width and height 
  */
  
  
  //double ObstacleAngle;
  //double ObstacleWidth, ObstacleHeight, ObstacleDepth;
  double c,s;
  
  MAL_MATRIX_RESIZE(m_ObstaclePoints,3,4);

  m_ObstaclePosition(0) = aObstacleInfo.x;
  m_ObstaclePosition(1) = aObstacleInfo.y;
  m_ObstaclePosition(2) = aObstacleInfo.z;

	
  //	ObstacleAngle = m_ObstacleInfo.theta;
  c = cos(aObstacleInfo.theta*M_PI/180.0);
  s = sin(aObstacleInfo.theta*M_PI/180.0);

	

  //this matrix transformes coordinates in the obstacle frame into the world frame
  m_ObstacleRot(0,0) = c; 	m_ObstacleRot(0,1) =-s;		m_ObstacleRot(0,2) = 0;
  m_ObstacleRot(1,0) = s; 	m_ObstacleRot(1,1) = c;		m_ObstacleRot(1,2) = 0;
  m_ObstacleRot(2,0) = 0; 	m_ObstacleRot(2,1) = 0;		m_ObstacleRot(2,2) = 1;

  //this matrix transformes coordinates in the world frame into the obstacle frame
  m_ObstacleRotInv(0,0) = c; 	m_ObstacleRotInv(0,1) = s;	m_ObstacleRotInv(0,2) = 0;
  m_ObstacleRotInv(1,0) = -s; 	m_ObstacleRotInv(1,1) = c;	m_ObstacleRotInv(1,2) = 0;
  m_ObstacleRotInv(2,0) = 0; 	m_ObstacleRotInv(2,1) = 0;	m_ObstacleRotInv(2,2) = 1;
	
  //ObstacleDepth = ObstacleInfo.d;
  //ObstacleWidth = ObstacleInfo.w;
  //ObstacleHeight = ObstacleInfo.h;
	
  //obstacle points are the four corner points at one side of the obstacle in the obstacle coordinate frame...the other side have the same coordinates, except for the Y coordinate which has opposite sign
  // point 0:
  m_ObstaclePoints(0,0) = 0.0; 	
  m_ObstaclePoints(1,0) = aObstacleInfo.w/2.0;
  m_ObstaclePoints(2,0) = 0.0;
  //point 1:
  m_ObstaclePoints(0,1) = 0.0; 	
  m_ObstaclePoints(1,1) = aObstacleInfo.w/2.0;
  m_ObstaclePoints(2,1) = aObstacleInfo.h;
  //point 2:
  m_ObstaclePoints(0,2) = aObstacleInfo.d; 	
  m_ObstaclePoints(1,2) = aObstacleInfo.w/2.0;
  m_ObstaclePoints(2,2) = aObstacleInfo.h;
  //point 3:
  m_ObstaclePoints(0,3) = aObstacleInfo.d; 	
  m_ObstaclePoints(1,3) = aObstacleInfo.w/2.0;
  m_ObstaclePoints(2,3) = 0.0;
  
  //cout << "I have set the obstacle info in the colission detection class" << endl;
}

void CollisionDetector::WorldFrameToObstacleFrame(MAL_S3_VECTOR(& WorldFrameCoord,double), 
						  MAL_S3_VECTOR(&ObstacleFrameCoord,double))
{ 
  // This function transforms the coordinates of a point 
  // expressed in the world frame to the local coordinates in the obstacle frame
  
  ObstacleFrameCoord = MAL_S3x3_RET_A_by_B(m_ObstacleRotInv , (WorldFrameCoord - m_ObstaclePosition));
  /*
    cout << "X WorldFrameCoord:  " <<  WorldFrameCoord(0,0) << endl;
    cout << "Y WorldFrameCoord:  " <<  WorldFrameCoord(1,0) << endl;
    cout << "Z WorldFrameCoord:  " <<  WorldFrameCoord(2,0) << endl;
    
    cout << "X obstacle position:  " <<  m_ObstaclePosition(0,0) << endl;
    cout << "Y obstacle position:  " <<  m_ObstaclePosition(1,0) << endl;
    cout << "Z obstacle position:  " <<  m_ObstaclePosition(2,0) << endl;
    
    cout << "X obstacle frame coordinate:  " <<  ObstacleFrameCoord(0,0) << endl;
    cout << "Y obstacle frame coordinate:  " <<  ObstacleFrameCoord(1,0) << endl;
    cout << "Z obstacle frame coordinate:  " <<  ObstacleFrameCoord(2,0) << endl;
  */
}

void CollisionDetector::CalcCoordShankLowerLegPoint(MAL_S3_VECTOR( RelCoord,double),
						    MAL_S3_VECTOR( &AbsCoord,double),
						    MAL_VECTOR( LegAngles,double), 
						    MAL_S3x3_MATRIX( WaistRot,double), 
						    MAL_S3_VECTOR( WaistPos,double),
						    int WhichLeg)
{
  MAL_S3x3_MATRIX(Rotation,double);
  MAL_S3_VECTOR(TempCoord,double);
  MAL_S3_VECTOR(Translation,double);
  
  double c,s;
  
  //rotation from lower leg to upper leg  angle 3  frame 3 -> 2
  
  c = cos(LegAngles(3));
  s = sin(LegAngles(3));
  
  Rotation(0,0) = c;       Rotation(0,1) = 0;        Rotation(0,2) = s;
  Rotation(1,0) = 0;       Rotation(1,1) = 1;        Rotation(1,2) = 0;
  Rotation(2,0) =-s;       Rotation(2,1) = 0;        Rotation(2,2) = c;
  
  TempCoord =  MAL_S3x3_RET_A_by_B(Rotation , RelCoord);
  
  
  //translation from knee to hip along Z axis
  
  Translation(0) = 0.0;
  Translation(1) = 0.0;
  Translation(2) = -0.3;
  
  TempCoord = TempCoord + Translation;
  
  
  //rotation from Upperleg to hip  angle 2  frame 2 -> 1
  
  c = cos(LegAngles(2));
  s = sin(LegAngles(2));
  
  Rotation(0,0) = c;       Rotation(0,1) = 0;        Rotation(0,2) = s;
  Rotation(1,0) = 0;       Rotation(1,1) = 1;        Rotation(1,2) = 0;
  Rotation(2,0) =-s;       Rotation(2,1) = 0;        Rotation(2,2) = c;
  
  TempCoord =  MAL_S3x3_RET_A_by_B(Rotation , TempCoord);
  
  //rotation from hip to hip  angle 1  frame  1 -> 0
  
  c = cos(LegAngles(1));
  s = sin(LegAngles(1));
	
  Rotation(0,0) = 1;       Rotation(0,1) = 0;        Rotation(0,2) = 0;
  Rotation(1,0) = 0;       Rotation(1,1) = c;        Rotation(1,2) =-s;
  Rotation(2,0) = 0;       Rotation(2,1) = s;        Rotation(2,2) = c;
	
  TempCoord =  MAL_S3x3_RET_A_by_B(Rotation , TempCoord);
	
	
  //rotation from hip to hip   angle 0  frame  0 -> waistframe orientation
	
  c = cos(LegAngles(0));
  s = sin(LegAngles(0));
	
  Rotation(0,0) = c;       Rotation(0,1) =-s;        Rotation(0,2) = 0;
  Rotation(1,0) = s;       Rotation(1,1) = c;        Rotation(1,2) = 0;
  Rotation(2,0) = 0;       Rotation(2,1) = 0;        Rotation(2,2) = 1;
	
  TempCoord =  MAL_S3x3_RET_A_by_B(Rotation , TempCoord);
	
	
  //translation from hip to waist along x axis
	
	
  Translation(0) = 0.0;
	
  if(WhichLeg>0)
    Translation(1) = 0.06;//this is currenlty hard coded but has to be solved when using functions in dynamic multi body class 
  else
    Translation(1) = -0.06;
	
  Translation(2) = 0.0;
	
  TempCoord = TempCoord + Translation;
	
  //translation and rotation of the waist in the world frame
	
  TempCoord = MAL_S3x3_RET_A_by_B(WaistRot, TempCoord);
  /*	
    cout << "X TempCoord coordinate:  " <<  TempCoord(0,0) << endl;
    cout << "Y TempCoord coordinate:  " <<  TempCoord(1,0) << endl;
    cout << "Z TempCoord coordinate:  " <<  TempCoord(2,0) << endl;
	
    cout << "X WaistPos coordinate:  " <<  WaistPos(0,0) << endl;
    cout << "Y WaistPos coordinate:  " <<  WaistPos(1,0) << endl;
    cout << "Z WaistPos coordinate:  " <<  WaistPos(2,0) << endl;
	
  */
  AbsCoord = TempCoord + WaistPos;
  /*	
    cout << "X AbsCoord coordinate:  " <<  AbsCoord(0,0) << endl;
    cout << "Y AbsCoord coordinate:  " <<  AbsCoord(1,0) << endl;
    cout << "Z AbsCoord coordinate:  " <<  AbsCoord(2,0) << endl;
  */
}

bool CollisionDetector::CollisionTwoLines(vector<double> p1, 
					  vector<double> p2,
					  vector<double> v1, 
					  vector<double> v2)
{
  //this function checks for intersection of two line segments p1p2 and v1v2.
  //since this is a 2D problem the coordinates are the respective planar coordinates
  //it returns true if a collision occurs, else false
  
  double Ap1p2v1, Ap1p2v2, Av1v2p1,Av1v2p2;
  
  
  Ap1p2v1 = (p1[0]*(p2[1]-v1[1])+p2[0]*(v1[1]-p1[1])+v1[0]*(p1[1]-p2[1]));
  Ap1p2v2 = (p1[0]*(p2[1]-v2[1])+p2[0]*(v2[1]-p1[1])+v2[0]*(p1[1]-p2[1]));
  Av1v2p1 = (v1[0]*(v2[1]-p1[1])+v2[0]*(p1[1]-v1[1])+p1[0]*(v1[1]-v2[1]));
  Av1v2p2 = (v1[0]*(v2[1]-p2[1])+v2[0]*(p2[1]-v1[1])+p2[0]*(v1[1]-v2[1]));
  
  
  if ((Ap1p2v1*Ap1p2v2)>0||(Av1v2p1*Av1v2p2)>0)
    return 0;  //collision free
  else
    return 1; //collision occurs

}

bool CollisionDetector::CollisionLineObstaclePlane(MAL_S3_VECTOR(&LegPoint1,double), 
						   MAL_S3_VECTOR(&LegPoint2,double),
						   int PlaneNumber)
{
  /*! This function checks for intersection of a linesegment p1p2 
    of the robot, expressed in the obstacle frame, with one of 
    the three planes of the obstacle: plane 0: front; plane 1: top; plane 2:rear plane
    it returns true if a collision occurs, else false
  
    for the intersection of a linesegment with an obstacle plane, 
    two separate 2D line intersections will be performed: in the XZ plane 
    and the XY plane or YZ plane  depending planenumber 
    indicates a side plan og the obstacle or not. */
  
			

  bool CollisionStatusXZ, CollisionStatusXY,CollisionStatusYZ;
  
  vector<double> p1, p2, v1, v2;
  
  p1.resize(2);	p2.resize(2);	v1.resize(2);	v2.resize(2);
  
  CollisionStatusXY = 1;
  CollisionStatusXZ = 1;
  CollisionStatusYZ = 1;
  
  
  
  if (PlaneNumber==3)
    {
      
      //collisioncheck two lines in the YZplane
      p1[0]=LegPoint1(1);	p1[1]=LegPoint1(2);
      p2[0]=LegPoint2(1);	p2[1]=LegPoint2(2);
      
      v1[0]=m_ObstaclePoints(1,0);	v1[1]=m_ObstaclePoints(2,0);
      v2[0]=m_ObstaclePoints(1,1);	v2[1]=m_ObstaclePoints(2,1);
      
      CollisionStatusYZ = CollisionTwoLines(p1, p2, v1, v2);
      
      //collisioncheck two lines in the XYplane
      p1[0]=LegPoint1(0);	p1[1]=LegPoint1(1);
      p2[0]=LegPoint2(0);	p2[1]=LegPoint2(1);
	
      v1[0]=m_ObstaclePoints(0,0);	v1[1]=m_ObstaclePoints(1,0);
      v2[0]=m_ObstaclePoints(0,3);	v2[1]=m_ObstaclePoints(1,3);
      
      CollisionStatusXY = CollisionTwoLines(p1, p2, v1, v2);
    }
  else if (PlaneNumber==4)
    {
      
      //collisioncheck two lines in the YZplane
      p1[0]=LegPoint1(1);	p1[1]=LegPoint1(2);
      p2[0]=LegPoint2(1);	p2[1]=LegPoint2(2);
      
      v1[0]=-m_ObstaclePoints(1,0);	v1[1]=m_ObstaclePoints(2,0);
      v2[0]=-m_ObstaclePoints(1,1);	v2[1]=m_ObstaclePoints(2,1);
	
      CollisionStatusYZ = CollisionTwoLines(p1, p2, v1, v2);
		
      //collisioncheck two lines in the XYplane
      p1[0]=LegPoint1(0);	p1[1]=LegPoint1(1);
      p2[0]=LegPoint2(0);	p2[1]=LegPoint2(1);
	
      v1[0]=m_ObstaclePoints(0,0);	v1[1]=-m_ObstaclePoints(1,0);
      v2[0]=m_ObstaclePoints(0,3);	v2[1]=-m_ObstaclePoints(1,3);

      CollisionStatusXY = CollisionTwoLines(p1, p2, v1, v2);
    }
  else
    {
		
      //collisioncheck two lines in the XZplane
      p1[0]=LegPoint1(0);	p1[1]=LegPoint1(2);
      p2[0]=LegPoint2(0);	p2[1]=LegPoint2(2);
		
      v1[0]=m_ObstaclePoints(0,0+PlaneNumber);	v1[1]=m_ObstaclePoints(2,0+PlaneNumber);
      v2[0]=m_ObstaclePoints(0,1+PlaneNumber);	v2[1]=m_ObstaclePoints(2,1+PlaneNumber);
	
      CollisionStatusXZ = CollisionTwoLines(p1, p2, v1, v2);
		
		
	
      //collisioncheck two lines in the XYplane
      p1[0]=LegPoint1(0);	p1[1]=LegPoint1(1);
      p2[0]=LegPoint2(0);	p2[1]=LegPoint2(1);
		
      v1[0]=m_ObstaclePoints(0,0+PlaneNumber);	v1[1]=m_ObstaclePoints(1,0+PlaneNumber);
      v2[0]=m_ObstaclePoints(0,0+PlaneNumber);	v2[1]=-m_ObstaclePoints(1,0+PlaneNumber);
	
      CollisionStatusXY = CollisionTwoLines(p1, p2, v1, v2);
		
    }
	
	
  if ((CollisionStatusXZ)&(CollisionStatusXY)&(CollisionStatusYZ))
    return 1; //collision occurs
  else
    return 0; //collision free
	
	

}

bool CollisionDetector::CollisionLineObstacleComplete(MAL_S3_VECTOR(&Point1,double), 
						      MAL_S3_VECTOR(&Point2,double))
{
  
  if ((((Point1(0)>0.0)
	&(Point1(0)<m_ObstaclePoints(0,3)))
       &((Point1(1)>-m_ObstaclePoints(1,3))
	 &(Point1(1)<m_ObstaclePoints(1,3)))
       &((Point1(2)>0.0)
	 &(Point1(2)<m_ObstaclePoints(2,2))))
      &(((Point2(0)>0.0)
	 &(Point2(0)<m_ObstaclePoints(0,3)))
	&((Point2(1)>-m_ObstaclePoints(1,3))
	  &(Point2(1)<m_ObstaclePoints(1,3))) 
	& ((Point2(2)>0.0)
	   &(Point2(2)<m_ObstaclePoints(2,2)))))
    {
      //cout << "line segment completely inside obstacle region" << endl;
      return 1;
      
    }
  else
    {
      for (int i=0;i<5;i++)
	{
	  if (CollisionLineObstaclePlane(Point1,Point2,i))
	    {
	      //cout << "collision by plane with number " << i << endl;
	      return 1;
	    }
	}
      //cout << "no collision " << endl;
      return 0;
    } 
}

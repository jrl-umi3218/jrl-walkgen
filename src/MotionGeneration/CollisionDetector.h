/* This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of relative steps.
   If you want to change the reference trajectories, and the planning
   of the foot, this is the object to modify.

   Copyright (c) 2005-2006,  JRL-Japan, CNRS/AIST
   Francis Keith,
   Bjorn Verrelst 

   All rights reserved.

   Please look at License.txt for more information on license.   
*/

#ifndef _COLLISION_DETECTOR_H_
#define _COLLISION_DETECTOR_H_



#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <vector>
#include <string>


namespace PatternGeneratorJRL
{

  static const double  RadToGrad = 57.295779578; 
  static const double  GradToRad = 0.0174532925; 
  
  struct ObstaclePar_t ;
  typedef struct ObstaclePar_t ObstaclePar;
  /** @ingroup steppingover
      @brief Object to detect collisions between robot leg and obstacle
   */
  class  CollisionDetector
    {
      public :
     
      /// Constructor
      CollisionDetector();

      /// Destructor
      ~CollisionDetector();

      /* !Calculates the absolute coordinates (ref frame) of a point on the 
	 lower legs given in relative coordinates in the locale frame      
	 (whichLeg positive for left leg and negative for right leg). 
	 This function might be replaced by functions from DynamicMultibody
      */
      void CalcCoordShankLowerLegPoint(MAL_S3_VECTOR(RelCoord,double), 
				       MAL_S3_VECTOR(&AbsCoord,double), 
				       MAL_VECTOR(LegAngles,double), 
				       MAL_S3x3_MATRIX(WaistRot,double),
				       MAL_S3_VECTOR(WaistPos,double),
				       int WhichLeg);
      
	


      /*! Set opstacle position in worldframe and  
	the obstacle points in local obstacle frame */
      void SetObstacleCoordinates(ObstaclePar aObstacleInfo);
      
      /*! This function transforms the coordinates of a point 
	expressed in the world frame to the local coordinates 
	in the obstacle frame */
      void WorldFrameToObstacleFrame(MAL_S3_VECTOR(&WorldFrameCoord,double), 
				     MAL_S3_VECTOR(&ObstacleFrameCoord,double));
	
      /*! This function checks for intersection of two line segments 
	p1p2 and v1v2. It returns true if a collision occurs, else false
      */
      bool CollisionTwoLines(std::vector<double> p1, 
			     std::vector<double> p2,
			     std::vector<double> v1, 
			     std::vector<double> v2);

      /*! This function checks for intersection of a linesegment p1p2 
	of the robot, expressed in the obstacle frame, with one of the 
	three planes of the obstacle: plane 0: front; plane 1: top; 
	plane 2:rear plane.
	It returns true if a collision occurs, else false. 
      */
      bool CollisionLineObstaclePlane(MAL_S3_VECTOR(&LegPoint1,double), 
				      MAL_S3_VECTOR(&LegPoint2,double),
				      int PlaneNumber);
	
      /*! This function does the complete check if a linesegment 
	intersects with the obstacle or is completely inside the obstacle
      */
      bool CollisionLineObstacleComplete(MAL_S3_VECTOR( & Point1,double), 
					 MAL_S3_VECTOR( & Point2,double));

    protected:
      
      /*! x, y, z position of obstacle in worldframe 
	(point taken on the front plan of the obstacle 
	on the floor and in the middel of the width) */
      MAL_S3_VECTOR(m_ObstaclePosition,double); 
      
      /*! This is the rotationmatrix from obstacle frame to world frame */
      MAL_S3x3_MATRIX(m_ObstacleRot,double); 
      
      /*! This is the rotationmatrix from world frame to obstacle frame */
      MAL_S3x3_MATRIX(m_ObstacleRotInv,double); 
      
      /*! This contains four points of one side of the obstacle in the 
	local obstacle frame......this frame is positionend in the origin 
	of the obstacle to where Obstacle Position is pointing, 
	the orientation is x following depth, y following width
      */
      MAL_MATRIX(m_ObstaclePoints,double);  
	
	
		
    };





};
#include <MotionGeneration/StepOverPlanner.h>
#endif /* _COLLISION_DETECTOR_H_ */

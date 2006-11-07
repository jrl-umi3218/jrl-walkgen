/** @doc This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of relative steps.
   If you want to change the reference trajectories, and the planning
   of the foot, this is the object to modify.

   CVS Information:
   $Id: ZMPDiscretization.h,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/ZMPDiscretization.h,v $
   $Log: ZMPDiscretization.h,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
   @author Olivier Stasse,Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its contributors 
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

#ifndef _COLLISION_DETECTOR_H_
#define _COLLISION_DETECTOR_H_



#include <VNL/matrix.h>
#include <vector>
#include <string>


namespace PatternGeneratorJRL
{

	const static double RadToGrad = 57.295779578; 
	const static double GradToRad = 0.0174532925; 

  	struct ObstaclePar_t ;
	typedef struct ObstaclePar_t ObstaclePar;
  /// Object to detect collisions between robot leg and obstacle
  class CollisionDetector
    {
      public :
     
      /// Constructor
      CollisionDetector();

      /// Destructor
      ~CollisionDetector();

      ///Calculates the absolute coordinates (ref frame) of a point on the lower legs given in relative coordinates in the locale frame      (whichLeg positive for left leg and negative for right leg). This function might be replaced by functions from DynamicMultibody
	void CalcCoordShankLowerLegPoint(VNL::Matrix<double> RelCoord, VNL::Matrix<double> &AbsCoord, VNL::Matrix<double> LegAngles, VNL::Matrix<double> WaistRot,VNL::Matrix<double> WaistPos,int WhichLeg);

	


	/// Set opstacle position in worldframe and the obstacle points in local obstacle frame
      void SetObstacleCoordinates(ObstaclePar aObstacleInfo);

	///this function transforms the coordinates of a point expressed in the world frame to the local coordinates in the obstacle frame
	void WorldFrameToObstacleFrame(VNL::Matrix<double> WorldFrameCoord, VNL::Matrix<double> &ObstacleFrameCoord);
	
	///this function checks for intersection of two line segments p1p2 and v1v2. It returns true if a collision occurs, else false
	bool CollisionTwoLines(std::vector<double> p1, std::vector<double> p2,std::vector<double> v1, std::vector<double> v2);

	///this function checks for intersection of a linesegment p1p2 of the robot, expressed in the obstacle frame, with one of the three planes of the obstacle: plane 0: front; plane 1: top; plane 2:rear plane //it returns true if a collision occurs, else false
	bool CollisionLineObstaclePlane(VNL::Matrix<double> LegPoint1, VNL::Matrix<double> LegPoint2,int PlaneNumber);
	
	///this function does the complete check if a linesegment intersects with the obstacle or is completely inside the obstacle
	bool CollisionLineObstacleComplete(VNL::Matrix<double> Point1, VNL::Matrix<double> Point2);

	protected:
	
	VNL::Matrix<double> m_ObstaclePosition; //x, y, z position of obstacle in worldframe (point taken on the front plan of the obstacle on the floor and in the middel of the width
	
	VNL::Matrix<double> m_ObstacleRot; //this is the rotationmatrix from obstacle frame to world frame

	VNL::Matrix<double> m_ObstacleRotInv; //this is the rotationmatrix from world frame to obstacle frame


	VNL::Matrix<double> m_ObstaclePoints;  //this contains four points of one side of the obstacle in the local obstacle frame......this frame is positionend in the origin of the obstacle to where Obstacle Position is pointing, the orientation is x following depth, y following width
	
	
		
    };





};
#include <StepOverPlanner.h>
#endif /* _COLLISION_DETECTOR_H_ */

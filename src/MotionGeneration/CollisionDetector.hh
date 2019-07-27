/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Bjorn Verrelst
 * Francois Keith
 * Olivier Stasse
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

/* This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of relative steps.
   If you want to change the reference trajectories, and the planning
   of the foot, this is the object to modify.
*/

#ifndef _COLLISION_DETECTOR_H_
#define _COLLISION_DETECTOR_H_



#include <vector>
#include <string>

#include <Eigen/Dense>

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
    void CalcCoordShankLowerLegPoint(Eigen::Vector3d RelCoord,
                                     Eigen::Vector3d &AbsCoord,
                                     Eigen::VectorXd LegAngles,
                                     Eigen::Matrix3d WaistRot,
                                     Eigen::Vector3d WaistPos,
                                     int WhichLeg);




    /*! Set opstacle position in worldframe and
      the obstacle points in local obstacle frame */
    void SetObstacleCoordinates(ObstaclePar aObstacleInfo);

    /*! This function transforms the coordinates of a point
      expressed in the world frame to the local coordinates
      in the obstacle frame */
    void WorldFrameToObstacleFrame(Eigen::Vector3d &WorldFrameCoord,
                                   Eigen::Vector3d &ObstacleFrameCoord);

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
    bool CollisionLineObstaclePlane(Eigen::Vector3d &LegPoint1,
                                    Eigen::Vector3d &LegPoint2,
                                    int PlaneNumber);

    /*! This function does the complete check if a linesegment
      intersects with the obstacle or is completely inside the obstacle
    */
    bool CollisionLineObstacleComplete(Eigen::Vector3d &Point1,
                                       Eigen::Vector3d &Point2);

  protected:

    /*! x, y, z position of obstacle in worldframe
      (point taken on the front plan of the obstacle
      on the floor and in the middel of the width) */
    Eigen::Vector3d m_ObstaclePosition;

    /*! This is the rotationmatrix from obstacle frame to world frame */
    Eigen::Matrix3d m_ObstacleRot;

    /*! This is the rotationmatrix from world frame to obstacle frame */
    Eigen::Matrix3d m_ObstacleRotInv;

    /*! This contains four points of one side of the obstacle in the
      local obstacle frame......this frame is positionend in the origin
      of the obstacle to where Obstacle Position is pointing,
      the orientation is x following depth, y following width
    */
    Eigen::MatrixXd m_ObstaclePoints;



  };





}
#include <MotionGeneration/StepOverPlanner.hh>
#endif /* _COLLISION_DETECTOR_H_ */

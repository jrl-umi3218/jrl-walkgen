/*
 * door.cpp
 *
 *  Created on: 10 mai 2011
 *      Author: syrine
 */

#include <ZMPRefTrajectoryGeneration/door.hh>

using namespace std;


Door::Door()
{}


void
Door::initialize( double Orientation)
{

  FrameBase_.Orientation = Orientation;

  FrameBase_.InvRx.resize(2,false);
  FrameBase_.InvRx(0) = cos(Orientation);
  FrameBase_.InvRx(1) = -sin(Orientation);
  FrameBase_.InvRy.resize(2,false);
  FrameBase_.InvRy(0) = sin(Orientation);
  FrameBase_.InvRy(1) = cos(Orientation);

  double xp = FrameBase_.Position[X];
  double yp = FrameBase_.Position[Y];

  FrameBase_.InvHp = -FrameBase_.InvRx*xp-FrameBase_.InvRy*yp;

}


void
Door::build_rotation_matrices(double Time, int N, boost_ublas::matrix<double> & R )
{

  double DesVelDoor = 0.0;
  if(Time > 8.0)
    DesVelDoor = M_PI/400.0;

  double CurrentDoorAngle = FrameDoor_.Orientation;
  double Theta;
  for(int j=0;j<N;j++)
    {
      Theta = DesVelDoor*(j+1)*SamplingTime_ + CurrentDoorAngle;
      R(0,j) = sin(Theta);
      R(1,j) = -cos(Theta);
    }

  FrameDoor_.Orientation = CurrentDoorAngle+DesVelDoor*SamplingTime_;

}


Door::frame_s::frame_s()
{
	Orientation = 0.0;
	dOrientation = 0.0;
	ddOrientation = 0.0;
	Position[X] = 0.6;
	Position[Y] = 0.5;
	Position[Z] = 0.0;
}

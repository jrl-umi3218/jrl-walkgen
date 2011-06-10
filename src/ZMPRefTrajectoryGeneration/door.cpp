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

	FrameBase_.Orientation_ = Orientation;

	FrameBase_.InvRx_.resize(2,false);
	FrameBase_.InvRx_(0) = cos(Orientation);
	FrameBase_.InvRx_(1) = -sin(Orientation);
	FrameBase_.InvRy_.resize(2,false);
	FrameBase_.InvRy_(0) = sin(Orientation);
	FrameBase_.InvRy_(1) = cos(Orientation);

    double xp = FrameBase_.Position(X);
    double yp = FrameBase_.Position(Y);
    double zp = FrameBase_.Position(Z);
    FrameBase_.InvHp_ = -FrameBase_.InvRx_*xp-FrameBase_.InvRy_*yp;

}



void
Door::build_door_matrix(double Time, double DesVelDoor, int N, boost_ublas::matrix<double> & R, double RelativAngle, double & Xdoortip, double & Ydoortip )
{

	double Theta;
	for(int j=0;j<N;j++)
	{
		Theta = DesVelDoor*(Time+j)+ RelativAngle;
		R(0,j) = sin(Theta);
		R(1,j) = -cos(Theta);
	}

//	Rrot(0) =  sin(DesVelDoor*(Time)+ RelativAngle);
//	Rrot(1) =  -cos(DesVelDoor*(Time)+ RelativAngle);
//	// L largeur de la porte

	double L = 1.0;
   // door tip position in global frame
	Xdoortip = L*sin( DesVelDoor*(Time))*cos(FrameBase_.Orientation_)- L*cos( DesVelDoor*(Time))*sin(FrameBase_.Orientation_)+FrameBase_.Position(X);
	Ydoortip = L*sin( DesVelDoor*(Time))*sin(FrameBase_.Orientation_)+ L*cos( DesVelDoor*(Time))*cos(FrameBase_.Orientation_)+FrameBase_.Position(Y);
//	cout<<"FrameBase_.Orientation_"<<FrameBase_.Orientation_<< endl;
}


Door::frame_s::frame_s()
{
	Orientation_ = -M_PI/2.0;
	dOrientation_ = 0.0;
	ddOrientation_ = 0.0;
	Position_[X] = 0.6;
	Position_[Y] = 0.5;
	Position_[Z] = 0.0;
}

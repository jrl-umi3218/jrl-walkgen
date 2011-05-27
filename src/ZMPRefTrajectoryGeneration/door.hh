/*
 * door.hh
 *
 *  Created on: 10 mai 2011
 *      Author: syrine
 */

#ifndef DOOR_HH_
#define DOOR_HH_

#include <jrl/mal/matrixabstractlayer.hh>

class Door
{

	//
	// Public types:
	//
public:
	const static unsigned int X = 0;
	const static unsigned int Y = 1;
	const static unsigned int Z = 2;

	//
	// Public methods:
	//
public:

	Door(); //constructeur

	void initialize( double Orientation);

	///
	void build_door_matrix(double Time, double DesVelDoor, int N, boost_ublas::matrix<double> & R, int RelativAngle, double & Xdoortip, double & Ydoortip);
	/// void doormatrices_construct();

	struct frame_s
	{

		//
		// Public methods
		//
	public:

		frame_s();// constructeur de la structure

		inline double const & Position(unsigned int Axis) const
				{ return Position_[Axis]; };
		inline void Position( unsigned int Axis, double Position )
		{ Position_[Axis] = Position; };

		inline double const Orientation() const
			{ return Orientation_; };
			inline void Orientation( const double Orientation )
			{ Orientation_ = Orientation; };

		inline double const dOrientation() const
			{ return dOrientation_; };
			inline void dOrientation( const double dOrientation )
			{ dOrientation_ = dOrientation; };

		inline double const ddOrientation() const
			{ return ddOrientation_; };
			inline void ddOrientation( const double ddOrientation )
			{ ddOrientation_ = ddOrientation; };

		//TODO:
		// inline boost_ublas::vector<double> const Position() const
		//{ return  Postion_[3]; };
        //inline void Postion( const boost_ublas::vector<double> Postion_[3] )
		//{ Postion_[3] = Postion_[3]; };
//
//		 inline boost_ublas::matrix<double> const H() const
//	    { return  H; };
//		inline void H( const boost_ublas::matrix<double> H )
//		{ H = H_; };
//
//		 inline boost_ublas::matrix<double> const Hinv() const
//	    { return  Hinv; };
//		inline void Hinv( const boost_ublas::matrix<double> Hinv )
//		{ Hinv = Hinv; };

		/// \brief Orientation around the vertical axis
		double Orientation_;
		// \brief Rotational velocity around the vertical axis
		double dOrientation_;
		// \brief Rotational acceleration around the vertical axis
		double ddOrientation_;

		/// \brief Position of the origin
		double Position_[3];
//		/// \brief transformation matrix from
//		boost_ublas::matrix<double> H_;
//		/// \brief Inverse of H
//		boost_ublas::matrix<double> Hinv_;

		/// \brief x rotation vector of the inv. transformation matrix InvH
		boost_ublas::vector<double> InvRx_;
		/// \brief y rotation vector of the inv. transformation matrix InvH
		boost_ublas::vector<double> InvRy_;
		/// \brief Translation component vector of InvH matrix
		boost_ublas::vector<double> InvHp_;

	};
	typedef frame_s frame_t;

    /// Definition des matrices correspondant à la porte


	boost_ublas::vector<double> & InvRx()
		{ return FrameBase_.InvRx_; };
	boost_ublas::vector<double> & InvRy()
				{ return FrameBase_.InvRy_; };
	boost_ublas::vector<double> & InvHp()
				{ return FrameBase_.InvHp_; };



	/// \name Matrices defining the evolution
	/// \{
//	inline double const Mass() const
//	{ return Mass_; };
//	inline void Mass( const double Mass )
//	{ Mass_ = Mass; };
//
//	inline double const Inertia() const
//	{ return Inertia_; };
//	inline void Inertia( const double Mass )
//	{ Inertia_ = Inertia; };
	/// \}


	//
	// Private members:
	//
private:

	/// Frame of the door
	frame_t FrameBase_;
	/// Frame of the door
	frame_t FrameDoor_;
//	/// Mass
//	double Mass_;
//	/// Inertia
//	double Inertia_;
	/// Wrist
	/// TODO: Wrist undefined
	frame_t FrameWrist_;


};

#endif /* DOOR_HH_ */

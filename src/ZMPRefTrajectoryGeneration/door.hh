/*
 * door.hh
 *
 *  Created on: 10 mai 2011
 *      Author: syrine
 */

#ifndef DOOR_HH_
#define DOOR_HH_

#include <jrl/mal/matrixabstractlayer.hh>
#include <cmath>

class Door
{

  //
  // Public types:
  //
public:
  const static unsigned int X = 0;
  const static unsigned int Y = 1;
  const static unsigned int Z = 2;

  /// \brief Frame
  struct frame_s
  {

    frame_s();

    /// \brief Orientation around the vertical axis
    double Orientation;
    // \brief Rotational velocity around the vertical axis
    double dOrientation;
    // \brief Rotational acceleration around the vertical axis
    double ddOrientation;

    /// \brief Position of the origin
    double Position[3];

    /// \brief x rotation vector of the inv. transformation matrix InvH
    boost_ublas::vector<double> InvRx;
    /// \brief y rotation vector of the inv. transformation matrix InvH
    boost_ublas::vector<double> InvRy;
    /// \brief Translation component vector of InvH matrix
    boost_ublas::vector<double> InvHp;

  };
  typedef frame_s frame_t;


  //
  // Public methods:
  //
public:

  Door();

  /// \brief
  void initialize( double Orientation );

  /// \brief Build rotation matrices for the whole preview period
  void build_rotation_matrices( double Time, int N, boost_ublas::matrix<double> & R );

  /// \name Accessors
  /// \{
  inline frame_t & FrameBase()
  { return FrameBase_; }
  inline frame_t & FrameDoor()
  { return FrameDoor_; }
  inline frame_t & FrameWrist()
  { return FrameWrist_; }
  inline void SamplingTime( const double SamplingTime )
  { SamplingTime_ = SamplingTime; }
  /// \}


  //
  // Private members:
  //
private:

  /// \brief Frame of the door
  frame_t FrameBase_;
  /// \brief Frame of the door
  frame_t FrameDoor_;
  /// \brief Frame of the wrist
  frame_t FrameWrist_;

  /// \brief Extern sampling time
  double SamplingTime_;

};

#endif /* DOOR_HH_ */

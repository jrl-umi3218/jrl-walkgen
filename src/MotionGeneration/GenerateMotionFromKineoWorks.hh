/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
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

/*! \doc This class can inherited for creating the trajectory 
  of the robot's joints
  given a path provided by KineoWorks. This path can be based on a partial
  model of the robot. The link between this partial model is given by
  an auxiliary file, and the redefinition of the virtual functions of 
  this class.
*/

#ifndef _GENERATE_MOTION_FROM_KINEOWORKS_H_
#define _GENERATE_MOTION_FROM_KINEOWORKS_H_


#include <string>
#include <vector>


#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.hh>
#include <PreviewControl/PreviewControl.hh>

namespace PatternGeneratorJRL
{

  typedef struct s_KWNode
  {
    std::vector<double> Joints;
  } KWNode;

  /*! This object is in charge of reading a path
    provided by KineoWorks and transform it in
    a proper motion for the walking pattern generator.
  */
  class  GenerateMotionFromKineoWorks
  {
  public:

    /*! Constructor */
    GenerateMotionFromKineoWorks();

    /*! Destructor */
    virtual ~GenerateMotionFromKineoWorks();

    /*! Read a file named FileName to create the connection between
      a partial model and the pattern generator.
      Default format is the following:
      \li \c Nb_Of_DOFs_from_the_reduced_model
      \li \c index_0
      \li \c .
      \li \c .
      \li \c .
      \li \c index_[Nb_Of_DOFs_from_the_reduced_model-1]

      If the index is equal to -1, then there is an internal meaning
      which goes beyond the current object.
    */
    virtual int ReadPartialModel(string FileName);

    /*! Read a path from file named FileName.*/
    virtual int ReadKineoWorksPath(string FileName);

    /*! Display model and path */
    void DisplayModelAndPath();

    /*! Create a Upper Body Motion stack based on the way points
      provided by KineoWorks and some outside generation of steps position. */
    void CreateUpperBodyMotion();

    /*! Create a trajectory for COM  */
    void CreateBufferFirstPreview(deque<ZMPPosition> &ZMPRefBuffer);

    /*! Update the link towards the Preview Control object in
      order to simulate the trajectory. */
    void SetPreviewControl(PreviewControl *aPC);

    /*! Create a buffer of Upper Body position based on the KineoWorks
      path and the COM buffer. The main strategy is to find points inside
      the COM buffer which are the closest to the KWs' way points.
      The value between two found points are linearly interpolated. */
    void ComputeUpperBodyPosition
    (deque< KWNode > &UpperBodyPositionsBuffer,
     std::vector<int> &ConversionFromLocalToRobotDOFsIndex);
    
  protected:

    /*! Number of DOFs inside the path provided by KineoWorks. */
    int m_NbOfDOFsFromKW;

    /*! This array maintains the link between the indexes of the path
      read from the KineoWorks path and the robot's model. */
    std::vector<int> m_IndexFromKWToRobot;

    /*! Keep track of the steering method provided by Kineoworks. */
    string m_SteeringMethod;

    /*! The path is a list of Nodes. */
    std::vector<KWNode> m_Path;

    /*! The list of COM position by which the robot is going
     * Should be deduced by using CreateBufferFirstPreview and
     *  with the appropriate buffer of ZMP values.
     */
    std::vector<COMPosition> m_COMBuffer;


    /*! Link to the preview control structure in order to simulate
     *   an Inverted Pendulum trajectory according to the Preview
     *  Control law. */
    PreviewControl * m_PC;

    /*! Time window for the preview control. */
    double m_PreviewControlTime;

    /*! Sampling period of the preview control. */
    double m_SamplingPeriod;

    /*! Size of the preview control buffer m_PreviewControl/m_SamplingPeriod */
    unsigned int m_NL;
  };


}
#endif /* _GENERATE_MOTION_FROM_KINEOWORKS_H_ */

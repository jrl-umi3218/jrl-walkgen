/* This class can inherited for creating the trajectory of the robot's joints
   given a path provided by KineoWorks. This path can be based on a partial
   model of the robot. The link between this partial model is given by
   an auxiliary file, and the redefinition of the virtual functions of this class.

   Copyright (c) 2005-2006, 
   Olivier Stasse,
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
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

#ifndef __PG_GENERATE_MOTION_FROM_KINEOWORKS_H_
#define __PG_GENERATE_MOTION_FROM_KINEOWORKS_H_


#include <string>
#include <vector>

#include <ZMPDiscretization.h>
#include <PreviewControl.h>

namespace PatternGeneratorJRL
{

  typedef struct s_KWNode
  {
    vector<double> Joints;
  } KWNode;

  /*! This object is in charge of reading a path
    provided by KineoWorks and transform it in 
    a proper motion for the walking pattern generator.
  */
  class GenerateMotionFromKineoWorks
    {
    public:

      /*! Constructor */ 
      GenerateMotionFromKineoWorks();

      /*! Destructor */
      virtual ~GenerateMotionFromKineoWorks();

      /*! Read a file named FileName to create the connection between
	a partial model and the pattern generator. 
	Default format is the following:
	Nb_Of_DOFs_from_the_reduced_model
	index_0
	.
	.
	.
	index_[Nb_Of_DOFs_from_the_reduced_model-1]
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
      void ComputeUpperBodyPosition(deque< KWNode > &UpperBodyPositionsBuffer, 
				    vector<int> &ConversionFromLocalToRobotDOFsIndex);
	
    protected:

      /*! Number of DOFs inside the path provided by KineoWorks. */
      int m_NbOfDOFsFromKW;
	
      /*! This array maintains the link between the indexes of the path
	read from the KineoWorks path and the robot's model. */
      std::vector<int> m_IndexFromKWToRobot;

      /*! Keep track of the steering method provided by Kineoworks. */
      string m_SteeringMethod;
	
      /*! The path is a list of Nodes. */
      vector<KWNode> m_Path;
	
      /*! The list of COM position by which the robot is going 
       * Should be deduced by using CreateBufferFirstPreview and
       *  with the appropriate buffer of ZMP values.
       */
      vector<COMPosition> m_COMBuffer;
	
	
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


};
#endif /* _GENERATE_MOTION_FROM_KINEOWORKS_H_ */

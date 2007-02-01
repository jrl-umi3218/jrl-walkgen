/* Computation of the dynamic aspect for a humanoid robot.
  
Copyright (c) 2005-2006, 
@author Olivier Stasse.
   
JRL-Japan, CNRS/AIST

All rights reserved.
   
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:
   
* Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the CNRS and AIST nor the names of its contributors 
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

#ifndef __PG_HUMANOID_DYNAMIC_MULTIBODY_H_
#define __PG_HUMANOID_DYNAMIC_MULTIBODY_H_


#include <DynamicMultiBody.h>
#include <jrlJoint.h>
#include <jrlHumanoidDynamicRobot.h>

namespace PatternGeneratorJRL
{
  /** \brief This class implements the functionnalities specific to a dynamic model for a humanoid.
      
  This includes a direct access to the joints reprensenting the hands, the foot, and the gaze.
  This specific class is the specialization of the generic class CjrlHumanoidDynamicRobot.

  */
  class HumanoidDynamicMultiBody: public DynamicMultiBody,CjrlHumanoidDynamicRobot
    {
    private:
      
      /** \brief Store the Left Hand Joint */
      Joint * m_LeftHandJoint;

      /** \brief Store the Right Hand Joint */
      Joint * m_RightHandJoint;
      
      /** \brief Store the Left Foot Joint */
      Joint * m_LeftFootJoint;

      /** \brief Store the Right Foot Joint */
      Joint * m_RightFootJoint;

      /** \brief Store the Gaze Joint */
      Joint * m_GazeJoint;

      /** \name Gaze related store */

      /** \brief Set the direction of the line. */
      MAL_S3_VECTOR(,double) m_LineVector;

      /** \brief Set the point through which the line is going. */
      MAL_S3_VECTOR(,double) m_LinePoint;
      
      /** @} */
      
      /** \brief Store the computed ZMP */
      MAL_S3_VECTOR(,double) m_ZMP;

    public:
      /** \name Constructors and destructors 
	  @{
      */
      /*! Constructor */
      HumanoidDynamicMultiBody();
      
      /*! Destructor */
      ~HumanoidDynamicMultiBody();

      /** @} */
      
      /** \name jrlHumanoidDynamicRobot Interface */
      
      
      
      /**
	 \name Joints specific to humanoid robots
      */
      
      /**
	 \brief Set the pointer to the left hand joint.
      */
      inline void leftHand(CjrlJoint* inLeftHand)
	{ m_LeftHandJoint = (Joint *)inLeftHand;};
      
      /** 
	  \brief Get a pointer to the left hand.
      */
      inline CjrlJoint* leftHand() const
	{ return m_LeftHandJoint;}
      
      /**
	 \brief Set the pointer to the right hand joint.
      */
      inline void rightHand(CjrlJoint* inRightHand)
	{ m_RightHandJoint = (Joint *)inRightHand;}
      
      /** 
	  \brief Get a pointer to the right hand.
      */
      inline CjrlJoint* rightHand()
	{ return m_RightHandJoint;}
      
      /**
	 \brief Set the pointer to the left foot joint.
      */
      inline void leftFoot(CjrlJoint* inLeftFoot)
	{ m_LeftFootJoint = (Joint *)inLeftFoot;}
      
      /** 
	  \brief Get a pointer to the left foot.
      */
      inline CjrlJoint* leftFoot() const
	{ return m_LeftFootJoint;}
      
      /**
	 \brief Set the pointer to the right foot joint.
      */
      inline void rightFoot(CjrlJoint* inRightFoot)
	{m_RightFootJoint = (Joint *) inRightFoot;}
      
      /** 
	  \brief Get a pointer to the right foot.
      */
      inline CjrlJoint* rightFoot()
	{return m_RightFootJoint;}
      
      /** 
	  \brief Set gaze joint
	  
	  \note  For most humanoid robots, the gaze joint is the head.
      */
      inline void gazeJoint(CjrlJoint* inGazeJoint)
	{ m_GazeJoint = (Joint *)inGazeJoint; }
      
      /**
	 \brief Get gaze joint
      */
      CjrlJoint* gazeJoint()
	{ return m_GazeJoint; }
      
      /**
	 \brief Set the gaze in the local frame of the gaze joint.
	 
	 \note The gaze is defined as a straight line linked to the gaze joint.
	 @param inVector: A 3D vector which define the direction of the gaze.
	 @param inPoint: A 3D point by which the line of direction \a inVector
	 goes through.

	 Those two paramaters defines a line in the head reference frame defining
	 a gaze direction.

      */
      inline void setGaze(const MAL_S3_VECTOR(,double)& inVector, const MAL_S3_VECTOR(,double) & inPoint)
	{ m_LineVector = inVector; m_LinePoint = inPoint;};
      
      /** 
	  \brief Get the gaze orientation in the local frame of the gaze joint.
	  
	  \note Returns the line along which the head is oriented.
	  @return outVector: The direction of the line.
	  @return outPoint: The point by which the line is going through in the head reference frame.

      */
      inline void getGaze(MAL_S3_VECTOR(,double) & outVector, MAL_S3_VECTOR(,double) & outPoint ) const
	{ outVector = m_LineVector; outPoint = m_LinePoint;};
	
      
      /**
	 \@}
      */
      
      /**
	 \name Zero momentum point
      */
      
      /**
	 \brief Compute the coordinates of the Zero Momentum Point.
      */
      const MAL_S3_VECTOR(,double) & zeroMomentumPoint() const;

      /**
	 @}
      */
      
      /** 
      @}
      */
    };
};
#endif

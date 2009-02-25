/* @doc Inverse Kinematics for legs and arms of a canonical
   humanoid robot. The arm are supposed to have 2 links.
   The legs are supposed to have 3 links.
   Please look at the documentation for more information.


   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati, Francois Keith, 
   
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

#ifndef _INVERSE_KINEMATICS_H_
#define _INVERSE_KINEMATICS_H_

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
#include <iostream>

#include <dynamicsJRLJapan/HumanoidSpecificities.h>
#include <walkGenJrl/walkGenJrl_API.h>
using namespace dynamicsJRLJapan;

namespace PatternGeneratorJRL
{
  /** Inverse Kinematics for generic humanoids.
      Some parameters are however specific to the HRP2.
      Please modify the code if needed. 
  */  
  class WALK_GEN_JRL_EXPORT InverseKinematics 
  {
  public:
    /*! Constructor. */
    InverseKinematics(HumanoidSpecificities *aHS);
      
    /*! Destructor */ 
    ~InverseKinematics();
      
      
    /*! Compute InverseKinematics for legs. */
    int ComputeInverseKinematics2ForLegs(MAL_S3x3_MATRIX( ,double)& Body_R,
					 MAL_S3_VECTOR(,double) &Body_P,
					 MAL_S3_VECTOR(,double) &Dt,
					 MAL_S3x3_MATRIX(,double) &Foot_R,
					 MAL_S3_VECTOR(,double) &Foot_P,
					 MAL_VECTOR( ,double) &q);
      
    /*! Compute InverseKinematics for arms. */
    int ComputeInverseKinematicsForArms(double X,
					double Z,
					double &Alpha,
					double &Beta);
      
    /*! Compute Arm swing maximum amplitude. */
    double ComputeXmax(double & lZ);
      
  protected:
      
    double m_KneeAngleBoundCos,m_KneeAngleBound;
    double m_KneeAngleBoundCos1,m_KneeAngleBound1;
    double m_KneeAngleBoundCos2;
      
    double m_FemurLength,m_TibiaLength;
      
    HumanoidSpecificities *m_HS;
  };
};
#endif /*_INVERSE_KINEMATICS_H_ */


/* @doc Inverse Kinematics for legs and arms of a canonical
   humanoid robot. The arm are supposed to have 2 links.
   The legs are supposed to have 3 links.
   Please look at the documentation for more information.

   CVS Information:
   $Id: InverseKinematics.h,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/InverseKinematics.h,v $
   $Log: InverseKinematics.h,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati
   
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

#include <VNL/matrix.h>
#include <iostream>

#include <HumanoidSpecificities.h>

namespace PatternGeneratorJRL
{
  /** Inverse Kinematics for generic humanoids.
      Some parameters are however specific to the HRP2.
      Please modify the code if needed. 
  */  
  class InverseKinematics 
    {
    public:
      /// Constructor.
      InverseKinematics(HumanoidSpecificities *aHS);
      
      /// Destructor
      ~InverseKinematics();
      
      
      /// Compute InverseKinematics for legs.
      int ComputeInverseKinematicsForLegs3(VNL::Matrix<double> Body_R,
					  VNL::Matrix<double> Body_P,
					  VNL::Matrix<double> Dt,
					  VNL::Matrix<double> Foot_R,
					  VNL::Matrix<double> Foot_P,
					  VNL::Matrix<double> &q);
      

      /// Compute InverseKinematics for legs.
      int ComputeInverseKinematics2ForLegs(VNL::Matrix<double> Body_R,
					   VNL::Matrix<double> Body_P,
					   VNL::Matrix<double> Dt,
					   VNL::Matrix<double> Foot_R,
					   VNL::Matrix<double> Foot_P,
					   VNL::Matrix<double> &q);
      
      /// Compute InverseKinematics for arms.
      int ComputeInverseKinematicsForArms(double X,
					  double Z,
					  double &Alpha,
					  double &Beta);
      
      /// Compute Arm swing maximum amplitude.
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


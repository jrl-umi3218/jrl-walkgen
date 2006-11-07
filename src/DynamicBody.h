/* Computation of the dynamical aspect for the body 
   of a model. 
   OS: Almost all the modifications are related to the computation
   of the Resolved Momentum Control.
   
   $Id: DynamicBody.h,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/DynamicBody.h,v $
   $Log: DynamicBody.h,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
   @author Jean Remy Chardonnet,  Abderrahmane Kheddar,  Olivier Stasse,  Ramzi Sellouati
   
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

#ifndef _HRP2_DYNAMIQUE_H_
#define _HRP2_DYNAMIQUE_H_

#include <Body.h>
#include <string>


namespace PatternGeneratorJRL
{

class DynamicBody : public Body
{
  
 public:


  /*
    This relationship does make sense only if we are considering the
    relationship between this body and its mother in a given oriented
    graph.
    q, dq, ddq = articulation position, velocity and acceleration2
    u = torque
    uu, dd = intermediate variables */
  double q, dq, ddq, u, uu, dd, gr, Ir;
  
  /* I = inertie,
     w_I = inertia in reference frame,
     R = orientation,
     Ivv, Iwv, Iww = inertia (p.198, 6.24) */
  matrix3d  w_I,  R,  Ivv,  Iwv,  Iww;


  /* page 46 figure 2.20
     a  = rotation vector,
     b  = translation vector,
     c  = center of mass,
     w_c = center of mass in reference frame,
     p  = position,
     v0 = linear velocity in reference frame,
     dv = linear acceleration,
     w  = angular velocity,
     dw = angular acceleration,
     sv, sw = spatial velocity,
     cv, cw = cross velocity term,
     pph, ppb = v x Iv */
  vector3d a, b, c, w_c, p,
    v0,  dv,  w,  dw,
    sv,  sw,  cv, cw,
    hhv,  hhw,  pph,  ppb;
  
  // Linear and angular momentums.
  vector3d P,L;

  // Mass in the tree structure, cf Kajita IROS2003 p 1647
  double m_tildem;
  double m_tildem_sister;

  // Tilde Center of Mass, 2, cf Kajita IROS2003 p 1647
  vector3d m_tildec;
  vector3d m_tildec_sister;

  // Tilde Inertia matrix , cf Kajita IROS2003 p 1647
  matrix3d m_tildeI, m_tildeI_sister, m_Dsister;

  // Inertia vector for RMC.
  vector3d m_RMC_m;
  vector3d m_RMC_h;
  int sister;
  int child;

  // Constructor.
  DynamicBody();

  // Destructor.
  ~DynamicBody();

  DynamicBody & operator=(const DynamicBody & r);
  DynamicBody & operator=(const Body & r);

};

};
#endif


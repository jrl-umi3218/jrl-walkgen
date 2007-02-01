/* Computation of the dynamical aspect for the body 
   of a model. 
   OS: Almost all the modifications are related to the computation
   of the Resolved Momentum Control.
   
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin
   OS (21/12/2006): removed any reference to non-homogeneous matrix
   library.


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

#ifndef __PG_HRP2_DYNAMIQUE_H_
#define __PG_HRP2_DYNAMIQUE_H_

#include <string>

#include <MatrixAbstractLayer.h>
#include <Body.h>

namespace PatternGeneratorJRL
{

  /*! Fundamental class to store the dynamical information for one body. 
    @ingroup forwardynamics
   */
  class DynamicBody : public Body
  {
    
  public:
    
    
    /*! This relationship does make sense only if we are considering the
      relationship between this body and its mother in a given oriented
      graph.
      q, dq, ddq = articulation position, velocity and acceleration2
      u = torque
      uu, dd = intermediate variables */
    double q, dq, ddq, u, uu, dd, gr, Ir;
    
    /*! Information coded as matrices:
      
    - \a I = inertia,
    - \a w_I = inertia in reference frame,
    - \a R = orientation,
    - \a Ivv, \a Iwv, \a Iww = inertia matrices (p.198, 6.24) 
    */
    MAL_S3x3_MATRIX(,double)  w_I,  R,  Ivv,  Iwv,  Iww;
    

    /*! Here are the physical parameters
      (as defined in Kajita's book page 46 figure 2.20):
      - \a a  = rotation vector,
      - \a b  = translation vector,
      - \a c  = center of mass,
      - \a w_c = center of mass in reference frame,
      - \a p  = position,
      - \a v0 = linear velocity in reference frame,
      - \a dv = linear acceleration,
      - \a w  = angular velocity,
      - \a dw = angular acceleration,
      - \a sv, sw = spatial velocity,
      - \a cv, cw = cross velocity term,
      - \a pph, ppb = v x Iv 
    */
    MAL_S3_VECTOR(,double) a, b, c, w_c, p,
      v0,  dv,  w,  dw,
      sv,  sw,  cv, cw,
      hhv,  hhw,  pph,  ppb;
  
    /*! Linear and angular momentums. */
    MAL_S3_VECTOR(,double) P,L;

    /*!  Mass in the tree structure, cf Kajita IROS2003 p 1647 */
    double m_tildem;
    double m_tildem_sister;

    /*! Tilde Center of Mass, 2, cf Kajita IROS2003 p 1647 */
    MAL_S3_VECTOR(,double) m_tildec;
    MAL_S3_VECTOR(,double) m_tildec_sister;

    /*! Tilde Inertia matrix , cf Kajita IROS2003 p 1647 */
    MAL_S3x3_MATRIX(,double) m_tildeI, 
      m_tildeI_sister, m_Dsister;

    /*! Inertia vector for RMC. */
    MAL_S3_VECTOR(,double) m_RMC_m;
    MAL_S3_VECTOR(,double) m_RMC_h;
    int sister;
    int child;

    /*! Default Constructor. */
    DynamicBody();

    /*! Default Destructor. */
    ~DynamicBody();

    DynamicBody & operator=(const DynamicBody & r);
    DynamicBody & operator=(const Body & r);

  };

};
#endif


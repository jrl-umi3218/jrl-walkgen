/* Computation of the dynamical aspect for the body 
   of a model.
   OS: Almost all the modifications are related to the computation
   of the Resolved Momentum Control.

   $Id: DynamicBody.cpp,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/DynamicBody.cpp,v $
   $Log: DynamicBody.cpp,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin



   Copyright (c) 2005-2006, 
   Jean Remy Chardonnet,
   Abderrahmane Kheddar,
   Olivier Stasse,
   Ramzi Sellouati
   
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

#include <DynamicBody.h>

using namespace PatternGeneratorJRL;

DynamicBody::DynamicBody():Body()
{
  sister =-1;
  child=-1;

  q =0.0;
  dq = 0.0;
  ddq = 0.0;

  // Inertia related.
  w_I.setZero();
  R.setIdentity();
  Ivv.setZero();
  Iwv.setZero();
  Iww.setZero();
  
  a.x = a.y = a.z = 0;
  b.x = b.y = b.z = 0;
  c.x = c.y = c.z = 0;
  w_c.x = w_c.y = w_c.z = 0;
  p.x = p.y = p.z = 0;
  v0.x = v0.y = v0.z = 0;
  dv.x = dv.y = dv.z = 0;
  w.x = w.y = w.z = 0;
  dw.x = dw.y = dw.z = 0;
  sv.x = sv.y = sv.z = 0;
  sw.x = sw.y = sw.z = 0;
  cv.x = cv.y = cv.z = 0;
  cw.x = cw.y = cw.z = 0;
  hhv.x = hhv.y = hhv.z = 0;
  hhw.x = hhw.y = hhw.z = 0;
  pph.x = pph.y = pph.z = 0;
  ppb.x = ppb.y = ppb.z = 0;
  
}

DynamicBody::~DynamicBody()
{

}

DynamicBody & DynamicBody::operator=(const DynamicBody & r)
{
  *((Body *)this)= *((Body *)&r);
  //  (Body)*this= (Body)r;

  // Joint value.
  q = r.q;
  dq = r.dq;
  ddq = r.ddq;

  // Inertia related.
  w_I = r.w_I;
  R = r.R;
  Ivv = r.Ivv;
  Iwv = r.Iwv;
  Iww = r.Iww;

  a = r.a;
  b = r.b;
  c = r.c;
  w_c = r.w_c;
  p = r.p;
  v0 = r.v0;
  dv = r.dv;
  w = r.w;
  dw = r.dw;
  sv = r.sv;
  sw = r.sw;
  cv = r.cv;
  cw = r.cw;
  hhv = r.hhv;
  hhw = r.hhw;
  pph = r.pph;
  ppb = r.ppb;
  
  sister = r.sister;
  child = r.child;
  
  P = r.P;
  L = r.L;
  m_tildem = r.m_tildem;
  m_tildem_sister = r.m_tildem_sister;

  m_tildec = r.m_tildec;
  m_tildec_sister = r.m_tildec_sister;
  m_tildeI = r.m_tildeI;
  m_tildeI_sister = r.m_tildeI_sister;
  m_Dsister = r.m_Dsister;

  m_RMC_m = r.m_RMC_m;
  m_RMC_h = r.m_RMC_h;
  
  return *this;
}


DynamicBody & DynamicBody::operator=(const Body & r)
{
  *((Body *)this) =r ;
  this->c = r.getPositionCoM();
  return *this;
}

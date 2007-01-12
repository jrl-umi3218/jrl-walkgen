/* Computation of the dynamical aspect for the body 
   of a model.
   OS: Almost all the modifications are related to the computation
   of the Resolved Momentum Control.

   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin
   OS (21/12/2006): removed any reference to non-homogeneous matrix
   library.

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

  
  a[0] = a[1] = a[2] = 0;
  b[0] = b[1] = b[2] = 0;
  c[0] = c[1] = c[2] = 0;
  w_c[0] = w_c[1] = w_c[2] = 0;
  p[0] = p[1] = p[2] = 0;
  v0[0] = v0[1] = v0[2] = 0;
  dv[0] = dv[1] = dv[2] = 0;
  w[0] = w[1] = w[2] = 0;
  dw[0] = dw[1] = dw[2] = 0;
  sv[0] = sv[1] = sv[2] = 0;
  sw[0] = sw[1] = sw[2] = 0;
  cv[0] = cv[1] = cv[2] = 0;
  cw[0] = cw[1] = cw[2] = 0;
  hhv[0] = hhv[1] = hhv[2] = 0;
  hhw[0] = hhw[1] = hhw[2] = 0;
  pph[0] = pph[1] = pph[2] = 0;
  ppb[0] = ppb[1] = ppb[2] = 0;
  
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

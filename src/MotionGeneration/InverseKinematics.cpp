/* Inverse Kinematics for legs and arms of a canonical
   humanoid robot. The arm are supposed to have 2 links.
   The legs are supposed to have 3 links.
   Please look at the documentation for more information.

   Copyright (c) 2005-2006, 
   Olivier Stasse,
   Ramzi Sellouati
   Francois Keith, 

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
#include <math.h>
#include <walkGenJrl/MotionGeneration/InverseKinematics.h>
#include "jrlMathTools/jrlConstants.h"


using namespace::PatternGeneratorJRL;

InverseKinematics::InverseKinematics(HumanoidSpecificities *aHS)
{
  m_KneeAngleBound=0.0*M_PI/180.0;
  m_KneeAngleBoundCos=cos(m_KneeAngleBound);
  m_KneeAngleBound1=30.0*M_PI/180.0; //sets a minimum angle for the knee and protects for overstretch
  m_KneeAngleBoundCos1=cos(m_KneeAngleBound1);  //used during inverse kin calculations
  
  m_FemurLength=0.25;
  m_TibiaLength=0.25;
  
  //m_KneeAngleBound=15.0*M_PI/180.0; //sets a minimum angle for the knee and protects for overstretch
  m_KneeAngleBoundCos2= 1.336;//cos(m_KneeAngleBound2);

  m_HS = aHS;
  if (m_HS!=0)
    {
      m_FemurLength = m_HS->GetFemurLength(-1);
      m_TibiaLength = m_HS->GetTibiaLength(-1);
    }
  
}

InverseKinematics::~InverseKinematics()
{
}


int InverseKinematics::ComputeInverseKinematics2ForLegs(MAL_S3x3_MATRIX(,double) &Body_R,
							MAL_S3_VECTOR( ,double) &Body_P,
							MAL_S3_VECTOR( ,double) &Dt,
							MAL_S3x3_MATRIX( ,double) &Foot_R,
							MAL_S3_VECTOR( ,double) &Foot_P,
							MAL_VECTOR( ,double)&q)
{
  double A=m_FemurLength,B=m_TibiaLength,C=0.0,c5=0.0,q6a=0.0;
  MAL_S3_VECTOR( r,double);
  MAL_S3x3_MATRIX( rT,double);
  MAL_S3x3_MATRIX( Foot_Rt,double);
  double NormofDt=0.0;

  // New part for the inverse kinematics specific to the HRP-2
  // robot. The computation of rx, ry and rz is different.

  // We compute the position of the body inside the reference
  // frame of the foot.
  MAL_S3_VECTOR( v,double);
  double theta=0.0, psi=0.0, Cp=0.0;
  float OppSignOfDtY = Dt(1) < 0.0 ? 1.0 : -1.0;


  Foot_Rt = MAL_S3x3_RET_TRANSPOSE(Foot_R);
  v = Body_P - Foot_P;
  v = MAL_S3x3_RET_A_by_B(Foot_Rt , v);
  //  cout << "v : "<< v <<endl;
  r(0) = v(0);
  NormofDt = sqrt(Dt(0)*Dt(0) + Dt(1)*Dt(1) + Dt(2)*Dt(2));
  //  cout << "Norm of Dt: " << NormofDt << endl;
  Cp = sqrt(v(1)*v(1)+v(2)*v(2) - NormofDt * NormofDt);
  psi = OppSignOfDtY * atan2(NormofDt,Cp);

  
  //  cout << "vz: " << v(2,0) << " vy :" << v(1,0) << endl;
  theta = atan2(v(2),v(1));
  
  r(1) = cos(psi+theta)*Cp;

  r(2) = sin(psi+theta)*Cp;

  //  r = rT * (Body_P +  Body_R * Dt - Foot_P);
  C = sqrt(r(0)*r(0)+
	   r(1)*r(1)+
	   r(2)*r(2));
  //C2 =sqrt(C1*C1-D*D);
  c5 = (C*C-A*A-B*B)/(2.0*A*B);

  if (c5>=m_KneeAngleBoundCos)
    {

      q(3)=m_KneeAngleBound;

    }
  else if (c5<=-1.0)
    {
      q(3)= M_PI;
    }
  else 
    {
      q(3)= acos(c5);
    }
  q6a = asin((A/C)*sin(M_PI- q(3)));


  float c,s,cz,sz;

  q(5) = atan2(r(1),r(2));
  if (q(5)>M_PI/2.0)
    {
      q(5) = q(5)-M_PI;
    }
  else if (q(5)<-M_PI/2.0)
    {
      q(5)+= M_PI;
    }

  q(4) = -atan2(r(0), (r(2)<0? -1.0:1.0)*sqrt(r(1)*r(1)+r(2)*r(2) )) - q6a;

  MAL_S3x3_MATRIX(R,double);
  MAL_S3x3_MATRIX(BRt,double);

  BRt = MAL_S3x3_RET_TRANSPOSE(Body_R);
  
  MAL_S3x3_MATRIX( Rroll,double);
  c = cos(-q(5));
  s = sin(-q(5));
  
  Rroll(0,0) = 1.0;   Rroll(0,1) = 0.0;   Rroll(0,2) = 0.0; 
  Rroll(1,0) = 0.0;   Rroll(1,1) = c;   Rroll(1,2) = -s; 
  Rroll(2,0) = 0.0;   Rroll(2,1) = s;   Rroll(2,2) = c; 
  
  
  MAL_S3x3_MATRIX( Rpitch,double);
  c = cos(-q(4)-q(3));
  s = sin(-q(4)-q(3));
  
  Rpitch(0,0) = c;     Rpitch(0,1) = 0;   Rpitch(0,2) = s; 
  Rpitch(1,0) = 0.0;   Rpitch(1,1) = 1;   Rpitch(1,2) = 0; 
  Rpitch(2,0) = -s;    Rpitch(2,1) = 0;   Rpitch(2,2) = c; 
  
  //  cout << " BRt"  << BRt << endl;
  R = MAL_S3x3_RET_A_by_B(BRt, Foot_R );
  MAL_S3x3_MATRIX(Rtmp,double);
  Rtmp = MAL_S3x3_RET_A_by_B(Rroll,Rpitch);
  R = MAL_S3x3_RET_A_by_B(R,Rtmp);

  q(0) = atan2(-R(0,1),R(1,1));
  
  cz = cos(q(0)); sz = sin(q(0));
  q(1) = atan2(R(2,1), -R(0,1)*sz + R(1,1) *cz);
  q(2) = atan2( -R(2,0), R(2,2));
  
  //  exit(0);
  return 0;
}

double InverseKinematics::ComputeXmax(double & Z)
{
  double A=0.25,
    B=0.25;
  double Xmax;
  if (Z<0.0)
    Z = 2*A*cos(15*M_PI/180.0);
  Xmax = sqrt(A*A - (Z - B)*(Z-B));
  return Xmax;
}

int InverseKinematics::ComputeInverseKinematicsForArms(double X,double Z,
						       double &Alpha,
						       double &Beta)
{
  double A=0.25,
    B=0.25;
  
  double C=0.0,Gamma=0.0,Theta=0.0;
  C = sqrt(X*X+Z*Z);
  
  Beta = acos((A*A+B*B-C*C)/(2*A*B))- M_PI;
  Gamma = asin((B*sin(M_PI+Beta))/C);
  Theta = atan2(X,Z);
  Alpha = Gamma - Theta;
  return 0;
}

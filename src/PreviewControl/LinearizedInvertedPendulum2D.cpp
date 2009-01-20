 /* This object simulate a 2D Linearized Inverted Pendulum
    with a control at the jerk level.

   Copyright (c) 2009, 
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
#include <walkGenJrl/PreviewControl/LinearizedInvertedPendulum2D.h>

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; \
                         DebugFile.open(y,ofstream::out); \
                         DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; \
                       DebugFile.open(y,ofstream::app); \
                       DebugFile <<  x << endl; DebugFile.close();}
#else
#define RESETDEBUG4(y) 
#define ODEBUG4(x,y) 
#endif

#define RESETDEBUG6(y) 
#define ODEBUG6(x,y) 

#define RESETDEBUG5(y) { ofstream DebugFile; \
                         DebugFile.open(y,ofstream::out);\
                         DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; \
                       DebugFile.open(y,ofstream::app); \
                       DebugFile << x << endl; \
                       DebugFile.close();}
#define ODEBUG5NOE(x,y) { ofstream DebugFile; \
                          DebugFile.open(y,ofstream::app); \
                          DebugFile << x ; DebugFile.close();}
#if 1
#define ODEBUG(x)
#else
#define ODEBUG(x)  std::cout << "ZMPQPWithConstraint: " << x << endl;
#endif

#define ODEBUG3(x)  std::cout << "ZMPQPWithConstraint: " << x << endl;

using namespace PatternGeneratorJRL;
using namespace std;

LinearizedInvertedPendulum2D::LinearizedInvertedPendulum2D()
{
  m_T =-1.0;
  m_ComHeight = -1.0;
  m_SamplingPeriod = -1.0;
  m_InterpolationInterval = -1;
  MAL_MATRIX_RESIZE(m_A,6,6);
  MAL_MATRIX_RESIZE(m_B,6,1);
  MAL_MATRIX_RESIZE(m_C,2,6);

  MAL_VECTOR_RESIZE(m_xk,6);
  MAL_VECTOR_RESIZE(m_zk,2);

}


LinearizedInvertedPendulum2D::~LinearizedInvertedPendulum2D()
{
}

const double & LinearizedInvertedPendulum2D::GetComHeight() const
{
  return m_ComHeight;
}

void LinearizedInvertedPendulum2D::SetComHeight(const double & aComHeight)
{
  m_ComHeight = aComHeight;
}

const double & LinearizedInvertedPendulum2D::GetSimulationControlPeriod() const
{
  return m_T;
}

void LinearizedInvertedPendulum2D::SetSimulationControlPeriod(const double & aT)
{
  m_T = aT;
  if (m_SamplingPeriod!=0.0)
    {
      double dinterval = m_T /  m_SamplingPeriod;
      m_InterpolationInterval=(int)dinterval;
    }

}

const double & LinearizedInvertedPendulum2D::GetRobotControlPeriod()
{
  return m_SamplingPeriod;
}

void LinearizedInvertedPendulum2D::SetRobotControlPeriod(const double & aT)
{
  m_SamplingPeriod = aT;

  if (m_SamplingPeriod!=0.0)
    {
  
      double dinterval = m_T /  m_SamplingPeriod;
      m_InterpolationInterval=(int)dinterval;
    }
  
}

void LinearizedInvertedPendulum2D::GetState(MAL_VECTOR(,double) &lxk)
{
  lxk = m_xk;
}

int LinearizedInvertedPendulum2D::InitializeSystem()
{
  if (m_T==-1.0)
    return -1;
  
  if (m_ComHeight==-1.0)
    return -2;

  for(int i=0;i<6;i++)
    {
      m_B(i,0) = 0.0;
      m_C(0,i) = 0.0;
      m_C(1,i) = 0.0;
      for(int j=0;j<6;j++)
	m_A(i,j)=0.0;
    }

  m_A(0,0) = 1.0; m_A(0,1) =   m_T; m_A(0,2) = m_T*m_T/2.0;
  m_A(1,0) = 0.0; m_A(1,1) =   1.0; m_A(1,2) = m_T;
  m_A(2,0) = 0.0; m_A(2,1) =   0.0; m_A(2,2) = 1.0;
  m_A(3,3) = 1.0; m_A(3,4) =   m_T; m_A(3,5) = m_T*m_T/2.0;
  m_A(4,3) = 0.0; m_A(4,4) =   1.0; m_A(4,5) = m_T;
  m_A(5,3) = 0.0; m_A(5,4) =   0.0; m_A(5,5) = 1.0;
  

  m_B(0,0) = m_T*m_T*m_T/6.0;
  m_B(1,0) = m_T*m_T/2.0;
  m_B(2,0) = m_T;
  m_B(3,0) = m_T*m_T*m_T/6.0;
  m_B(4,0) = m_T*m_T/2.0;
  m_B(5,0) = m_T;

  
  m_C(0,0) = 1.0;
  m_C(0,1) = 0.0;
  m_C(0,2) = -m_ComHeight/9.81;

  m_C(1,3) = 1.0;
  m_C(1,4) = 0.0;
  m_C(1,5) = -m_ComHeight/9.81;

  for(unsigned int i=0;i<6;i++)
    m_xk[i] = 0.0;
  
  return 0;
}



int LinearizedInvertedPendulum2D::Interpolation(deque<ZMPPosition> &NewFinalZMPPositions,
						deque<COMPosition> &COMPositions,
						deque<ZMPPosition> &ZMPRefPositions,
						int CurrentPosition,
						double CX, double CY)
{

  // Fill the queues with the interpolated CoM values.
  for(int lk=0;lk<m_InterpolationInterval;lk++)
    {
      
      COMPosition aCOMPos;
      double lkSP;
      lkSP = (lk+1) * m_SamplingPeriod;
      
      aCOMPos.x[0] = 
	m_xk[0] + // Position
	lkSP * m_xk[1] +  // Speed
	0.5 * lkSP*lkSP * m_xk[2] +// Acceleration 
	lkSP * lkSP * lkSP * CX /6.0; // Jerk
      
      aCOMPos.x[1] = 
	m_xk[1] + // Speed
	lkSP * m_xk[2] +  // Acceleration
	0.5 * lkSP * lkSP * CX; // Jerk
      
      aCOMPos.x[2] = 
	m_xk[2] +  // Acceleration
	lkSP * CX; // Jerk
      
      aCOMPos.y[0] = 
	m_xk[3] + // Position
	lkSP * m_xk[4] +  // Speed
	0.5 * lkSP*lkSP * m_xk[5] + // Acceleration 
	lkSP * lkSP * lkSP * CY /6.0; // Jerk
      
      aCOMPos.y[1] = 
	m_xk[4] + // Speed
	lkSP * m_xk[5] +  // Acceleration
	0.5 * lkSP * lkSP * CY; // Jerk
      
      aCOMPos.y[2] = 
	m_xk[5] +  // Acceleration
	lkSP * CY; // Jerk
      
      aCOMPos.yaw = ZMPRefPositions[CurrentPosition+lk].theta;      
      
      COMPositions.push_back(aCOMPos);

      // Compute ZMP position and orientation.
      ZMPPosition aZMPPos;
      aZMPPos.px = m_C(0,0) * aCOMPos.x[0] +
	m_C(0,1) * aCOMPos.x[1] + m_C(0,2) * aCOMPos.x[2];
      
      aZMPPos.py = m_C(0,0) * aCOMPos.y[0] +
	m_C(0,1) * aCOMPos.y[1] + m_C(0,2) * aCOMPos.y[2];
      
      aZMPPos.theta = ZMPRefPositions[CurrentPosition+lk].theta;
      aZMPPos.stepType = ZMPRefPositions[CurrentPosition+lk].stepType;
      
      // Put it into the stack.
      NewFinalZMPPositions.push_back(aZMPPos);
      
      ODEBUG4(aCOMPos.x[0] << " " << aCOMPos.x[1] << " " << aCOMPos.x[2] << " " <<
	      aCOMPos.y[0] << " " << aCOMPos.y[1] << " " << aCOMPos.y[2] << " " <<
	      aCOMPos.yaw << " " <<
	      aZMPPos.px << " " << aZMPPos.py <<  " " << aZMPPos.theta << " " << 
	      CX << " " << CY << " " << 
	      lkSP << " " << m_T , "DebugInterpol.dat");
    }
  return 0;
}


int LinearizedInvertedPendulum2D::OneIteration(double CX,double CY)
{
  
  /*! Vector to compute the command applied to the LIPM */
  MAL_VECTOR_DIM(Buk,double,6);

  // Compute the command multiply 
  Buk[0] = CX*m_B(0,0);
  Buk[1] = CX*m_B(1,0);
  Buk[2] = CX*m_B(2,0);
      
  Buk[3] = CY*m_B(3,0);
  Buk[4] = CY*m_B(4,0);
  Buk[5] = CY*m_B(5,0);


  // Simulate the dynamical system
  m_xk = MAL_RET_A_by_B(m_A,m_xk) + Buk ;
  // Modif. from Dimitar: Initially a mistake regarding the ordering.
  MAL_C_eq_A_by_B(m_zk,m_C,m_xk);


  ODEBUG4(m_xk[0] << " " << m_xk[1] << " " << m_xk[2] << " " <<
	  m_xk[3] << " " << m_xk[4] << " " << m_xk[5] << " " <<
	  CX  << " " << CY  << " " << 
	  m_zk[0] << " " << m_zk[1] << " " << 
	  StartingTime,
	  "DebugPBW.dat");

  
  return 0;
}

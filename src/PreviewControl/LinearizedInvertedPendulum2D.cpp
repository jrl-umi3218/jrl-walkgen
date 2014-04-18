/*
 * Copyright 2009, 2010, 
 *
 * Andrei Herdt
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the 
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */

/* This object simulate a 2D Linearized Inverted Pendulum
   with a control at the jerk level. */

//#define _DEBUG_MODE_ON_

#include <iostream>
#include <fstream>

#include <PreviewControl/LinearizedInvertedPendulum2D.hh>
#include <Debug.hh>

using namespace PatternGeneratorJRL;
using namespace std;

LinearizedInvertedPendulum2D::LinearizedInvertedPendulum2D()
{
  m_T =-1.0;
  m_ComHeight = -1.0;
  m_SamplingPeriod = -1.0; 
  m_InterpolationInterval = -1;
  MAL_MATRIX_RESIZE(m_A,3,3);
  MAL_MATRIX_RESIZE(m_B,3,1);
  MAL_MATRIX_RESIZE(m_C,1,3);

  MAL_VECTOR_RESIZE(m_xk,6);
  MAL_VECTOR_RESIZE(m_CoM.x,3);
  MAL_VECTOR_RESIZE(m_CoM.y,3);
  MAL_VECTOR_RESIZE(m_zk,2);

  RESETDEBUG4("Debug2DLIPM.dat");
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


void LinearizedInvertedPendulum2D::GetState(MAL_VECTOR_TYPE(double) &lxk)
{
  //For compability reasons
  m_xk[0] = m_CoM.x[0];m_xk[1] = m_CoM.x[1];m_xk[2] = m_CoM.x[2];
  m_xk[3] = m_CoM.y[0];m_xk[4] = m_CoM.y[1];m_xk[5] = m_CoM.y[2];
  lxk = m_xk;
}


void LinearizedInvertedPendulum2D::setState(MAL_VECTOR_TYPE(double) lxk)
{
  m_xk = lxk;
}


int LinearizedInvertedPendulum2D::InitializeSystem()
{
  if (m_T==-1.0)
    return -1;
  
  if (m_ComHeight==-1.0)
    return -2;

  for(int i=0;i<3;i++)
    {
      m_B(i,0) = 0.0;
      m_C(0,i) = 0.0;
      for(int j=0;j<3;j++)
	m_A(i,j)=0.0;
    }

  m_A(0,0) = 1.0; m_A(0,1) =   m_T; m_A(0,2) = m_T*m_T/2.0;
  m_A(1,0) = 0.0; m_A(1,1) =   1.0; m_A(1,2) = m_T;
  m_A(2,0) = 0.0; m_A(2,1) =   0.0; m_A(2,2) = 1.0;

  m_B(0,0) = m_T*m_T*m_T/6.0;
  m_B(1,0) = m_T*m_T/2.0;
  m_B(2,0) = m_T;
  
  m_C(0,0) = 1.0;
  m_C(0,1) = 0.0;
  m_C(0,2) = -m_ComHeight/9.81;


  return 0;
}



int LinearizedInvertedPendulum2D::Interpolation(deque<COMState> &COMStates,
						deque<ZMPPosition> &ZMPRefPositions,
						int CurrentPosition,
						double CX, double CY)
{
  int lCurrentPosition = CurrentPosition;
  // Fill the queues with the interpolated CoM values.
    
  //TODO: with TestHerdt, it is mandatory to use COMStates.size()-1, or it will crash.
  // Is it the same for the other PG ? Please check.
  int loopEnd = std::min<int>( m_InterpolationInterval, ((int)COMStates.size())-1-CurrentPosition);
  for(int lk=0;lk<=loopEnd;lk++,lCurrentPosition++)
    {
      ODEBUG("lCurrentPosition: "<< lCurrentPosition);
      COMState & aCOMPos = COMStates[lCurrentPosition];
      double lkSP;
      lkSP = (lk+1) * m_SamplingPeriod;
      
      aCOMPos.x[0] = 
	m_CoM.x[0] + // Position
	lkSP * m_CoM.x[1] +  // Speed
	0.5 * lkSP*lkSP * m_CoM.x[2] +// Acceleration
	lkSP * lkSP * lkSP * CX /6.0; // Jerk
      
      aCOMPos.x[1] = 
	m_CoM.x[1] + // Speed
	lkSP * m_CoM.x[2] +  // Acceleration
	0.5 * lkSP * lkSP * CX; // Jerk
      
      aCOMPos.x[2] = 
	m_CoM.x[2] +  // Acceleration
	lkSP * CX; // Jerk
      
      aCOMPos.y[0] = 
	m_CoM.y[0] + // Position
	lkSP * m_CoM.y[1] +  // Speed
	0.5 * lkSP*lkSP * m_CoM.y[2] + // Acceleration
	lkSP * lkSP * lkSP * CY /6.0; // Jerk
      
      aCOMPos.y[1] = 
	m_CoM.y[1] + // Speed
	lkSP * m_CoM.y[2] +  // Acceleration
	0.5 * lkSP * lkSP * CY; // Jerk
      
      aCOMPos.y[2] = 
	m_CoM.y[2] +  // Acceleration
	lkSP * CY; // Jerk
      
      aCOMPos.yaw[0] = ZMPRefPositions[lCurrentPosition].theta;      
      
      aCOMPos.z[0] = m_ComHeight;
      aCOMPos.z[1] = 0;
      aCOMPos.z[2] = 0;
      // Compute ZMP position and orientation.
      ZMPPosition & aZMPPos = ZMPRefPositions[lCurrentPosition];
      aZMPPos.px = m_C(0,0) * aCOMPos.x[0] +
	m_C(0,1) * aCOMPos.x[1] + m_C(0,2) * aCOMPos.x[2];
      
      aZMPPos.py = m_C(0,0) * aCOMPos.y[0] +
	m_C(0,1) * aCOMPos.y[1] + m_C(0,2) * aCOMPos.y[2];
      
            
      ODEBUG4(aCOMPos.x[0] << " " << aCOMPos.x[1] << " " << aCOMPos.x[2] << " " <<
	      aCOMPos.y[0] << " " << aCOMPos.y[1] << " " << aCOMPos.y[2] << " " <<
	      aCOMPos.yaw << " " <<
	      aZMPPos.px << " " << aZMPPos.py <<  " " << aZMPPos.theta << " " <<
	      CX << " " << CY << " " <<
	      lkSP << " " << m_T , "DebugInterpol.dat");
    }
  return 0;
}


com_t LinearizedInvertedPendulum2D::OneIteration(double ux, double uy)
{
  MAL_VECTOR_DIM(Bux,double,3);
  MAL_VECTOR_DIM(Buy,double,3);

  Bux[0] = ux*m_B(0,0);
  Bux[1] = ux*m_B(1,0);
  Bux[2] = ux*m_B(2,0);

  Buy[0] = uy*m_B(0,0);
  Buy[1] = uy*m_B(1,0);
  Buy[2] = uy*m_B(2,0);

  // Simulate the dynamical system
  m_CoM.x = MAL_RET_A_by_B(m_A,m_CoM.x);
  m_CoM.x = m_CoM.x + Bux;
  m_CoM.y = MAL_RET_A_by_B(m_A,m_CoM.y);
  m_CoM.y = m_CoM.y + Buy;
  // Modif. from Dimitar: Initially a mistake regarding the ordering.
  //MAL_C_eq_A_by_B(m_zk,m_C,m_xk);


  ODEBUG4( m_xk[0] << " " << m_xk[1] << " " << m_xk[2] << " " <<
	   m_xk[3] << " " << m_xk[4] << " " << m_xk[5] << " " <<
	   m_CoM.x  << " " << m_CoM.y  << " " <<
	   m_zk[0] << " " << m_zk[1] << " " <<
	   Bux[0] << " " << Bux[1] << " " << Bux[2] << " " <<
	   Buy[0] << " " << Buy[1] << " " << Buy[2] << " " <<
	   m_B(0,0) << " " << m_B(1,0) << " " << m_B(2,0) << " " <<
	   m_B(3,0) << " " << m_B(4,0) << " " << m_B(5,0) << " " ,
	   "Debug2DLIPM.dat");

  
  return m_CoM;
}

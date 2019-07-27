/*
 * Copyright 2010,
 *
 * Andrei Herdt
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
/*! \file PLDPSolver.cpp
  \brief This file implements the optimized QP solver proposed by Dimitar 2009.
*/
#include <iostream>
#include <fstream>
#include <cstring>

#include "portability/gettimeofday.hh"

#ifdef WIN32
# include <Windows.h>
#endif /* WIN32 */

#include <Mathematics/PLDPSolverHerdt.hh>


// Isnan and Isinf
#ifdef WIN32
#include <float.h>
#define isnan _isnan

//definition of isinf for win32
//src:  http://www.gnu.org/software/libtool/manual
// /autoconf/Function-Portability.html
inline int isinf (double x)
{
  return isnan (x - x);
}
#endif /* WIN32 */

#include <Debug.hh>

using namespace PatternGeneratorJRL;
using namespace Optimization::Solver;
using namespace std;

PLDPSolverHerdt::PLDPSolverHerdt(unsigned int CardU,
                                 double *iPu,
                                 double *Px,
                                 double *Pu,
                                 double *iLQ)
{
  m_DebugMode = 3;
  m_HotStart = true;
  m_NoCholesky = true;


  m_InternalTime = 0.0;
  m_tol = 1e-8;
  m_LimitedComputationTime = true;
  m_AmountOfLimitedComputationTime = 0.0025;//0.0013;
  /* Initialize pointers */
  m_CardV = CardU;
  //  m_NbSteps = NumberSteps;

  m_Pu = Pu;
  m_iPu = iPu;
  m_Px = Px;
  m_iPuPx = 0;
  m_Vk = 0;
  m_CstPartOfCostFunction = 0;
  m_UnconstrainedDescentDirection =0;
  m_L =0 ;
  m_iLQ = iLQ;
  m_d = 0;
  m_v1 = m_v2 = m_y = 0;
  m_tmp1 = m_tmp2 = 0;
  m_A = m_b= 0;

  m_ConstraintsValueComputed = 0;

  m_NbMaxOfConstraints = 8*m_CardV;
  m_NbOfConstraints = 0;

  m_DistanceFeetCenters = 0.2;

  m_OptCholesky = new PatternGeneratorJRL::
    OptCholesky(m_NbMaxOfConstraints,2*m_CardV,
                OptCholesky::MODE_FORTRAN);

  string Buffer("InfosPLDP");
  if (m_HotStart)
    Buffer+="HS";
  if (m_LimitedComputationTime)
    Buffer+="LT";
  Buffer+=".dat";
  RESETDEBUG6((char*)Buffer.c_str());
  RESETDEBUG6("ActivatedConstraints.dat");


}

void PLDPSolverHerdt::AllocateMemoryForSolver(unsigned int NumberSteps)
{

  PrecomputeiPuPx(NumberSteps);

  m_Vk = new double[2*(m_CardV+NumberSteps)];
  memset(m_Vk,0,2*(m_CardV+NumberSteps)*sizeof(double));

  m_InitialZMPSolution = new double[2*(m_CardV+NumberSteps)];
  memset(m_InitialZMPSolution,0,2*(m_CardV+NumberSteps)*sizeof(double));

  m_UnconstrainedDescentDirection = new double[2*(m_CardV+NumberSteps)];
  memset(m_UnconstrainedDescentDirection,0,
         2*(m_CardV+NumberSteps)*sizeof(double));

  m_L = new double[m_NbMaxOfConstraints*m_NbMaxOfConstraints];
  m_iL = new double[m_NbMaxOfConstraints*m_NbMaxOfConstraints];

  m_v1 = new double[m_NbMaxOfConstraints];
  m_v2 = new double[m_NbMaxOfConstraints];
  m_tmp1 = new double[m_NbMaxOfConstraints];
  m_tmp2 = new double[m_NbMaxOfConstraints];

  m_y = new double[m_NbMaxOfConstraints];

  m_ConstraintsValueComputed = new bool [m_NbMaxOfConstraints*2];

  m_d = new double[2*(m_CardV+NumberSteps)];
  m_OptCholesky->SetL(m_L);
  m_OptCholesky->SetiL(m_iL);
}

void PLDPSolverHerdt::InitializeSolver(unsigned int /* NumberSteps*/ )
{
#if 0
  // Allocation max:
  // We assume that we might at max. 8 actives constraintes
  // per control time.
  memset(m_v1,0,m_NbMaxOfConstraints*sizeof(double));
  // Same than v1.
  memset(m_v2,0,m_NbMaxOfConstraints*sizeof(double));
  // Same than y.
  memset(m_y,0,m_NbMaxOfConstraints*sizeof(double));

  // Same than L.
  memset(m_L,0,8*m_CardV*sizeof(double));
  // Same than iL.
  memset(m_iL,0,8*m_CardV*sizeof(double));

  // Same for the descent.
  memset(m_d,0,2*(m_CardV+NumberSteps)*sizeof(double));
#endif
  m_ActivatedConstraints.clear();
}

PLDPSolverHerdt::~PLDPSolverHerdt()
{
  if (m_iPuPx!=0)
    delete [] m_iPuPx;

  if (m_Vk!=0)
    delete [] m_Vk;

  if (m_InitialZMPSolution!=0)
    delete [] m_InitialZMPSolution;

  if (m_UnconstrainedDescentDirection!=0)
    delete [] m_UnconstrainedDescentDirection;

  if (m_OptCholesky!=0)
    delete m_OptCholesky;

  if (m_L!=0)
    delete [] m_L;

  if (m_iL!=0)
    delete [] m_iL;

  if (m_v1!=0)
    delete [] m_v1;

  if (m_v2!=0)
    delete [] m_v2;

  if (m_tmp1!=0)
    delete [] m_tmp1;

  if (m_tmp2!=0)
    delete [] m_tmp2;

  if (m_d!=0)
    delete [] m_d;

  if (m_y!=0)
    delete [] m_y;

  if ( m_ConstraintsValueComputed!=0)
    delete [] m_ConstraintsValueComputed;

}

int PLDPSolverHerdt::PrecomputeiPuPx(unsigned int NumberSteps)
{
  m_iPuPx = new double[2*(m_CardV)*6];

  memset(m_iPuPx,0,2*(m_CardV)*6);

  if (m_DebugMode>1)
    {
      ofstream aof;
      aof.open("iPu.dat",ofstream::out);
      for(unsigned int i=0; i<2*(m_CardV+NumberSteps); i++)
        {
          for(unsigned int j=0; j<2*(m_CardV+NumberSteps); j++)
            {
              aof << m_iPu[i*2*(m_CardV+NumberSteps)+j] << " " ;
            }
          aof << endl;
        }
      aof.close();

      aof.open("Pu.dat",ofstream::out);
      for(unsigned int i=0; i<2*(m_CardV+NumberSteps); i++)
        {
          for(unsigned int j=0; j<2*(m_CardV+NumberSteps); j++)
            {
              aof << m_Pu[i*2*(m_CardV+NumberSteps)+j] << " " ;
            }
          aof << endl;
        }
      aof.close();


      aof.open("Px.dat",ofstream::out);
      for(unsigned int i=0; i<m_CardV; i++)
        {
          for(unsigned int j=0; j<3; j++)
            {
              aof << m_Px[i*3+j] << " " ;
            }
          aof << endl;
        }
      aof.close();

      aof.open("isLQ.dat",ofstream::out);
      for(unsigned int i=0; i<2*(m_CardV+NumberSteps); i++)
        {
          for(unsigned int j=0; j<2*(m_CardV+NumberSteps); j++)
            {
              aof << m_iLQ[i*2*(m_CardV+NumberSteps)+j] << " " ;
            }
          aof << endl;
        }
      aof.close();

    }

  // This should be reimplemented with a proper matrix library.
  // will be done in a second time.
  for(unsigned int i=0; i<(m_CardV); i++)
    {
      for(unsigned int j=0; j<3; j++)
        {
          m_iPuPx[i*6+j]= 0.0;
          m_iPuPx[i*6+j+3]= 0.0;
          m_iPuPx[(i+(m_CardV))*6+j]= 0.0;
          m_iPuPx[(i+(m_CardV))*6+j+3]= 0.0;
          for(unsigned int k=0; k<m_CardV; k++)
            {
              double tmp = m_iPu[k*(2*m_CardV+2*NumberSteps)+i] *
                m_Px[k*3+j];

              m_iPuPx[i*6+j]+= tmp;
              //            m_iPuPx[i*6+j+3]+= tmp;
              //            m_iPuPx[(i+m_CardV)*6+j]+= tmp;
              m_iPuPx[(i+(m_CardV))*6+j+3]+= tmp;

            }
        }
    }
  return 0;
}



int PLDPSolverHerdt::
ComputeInitialSolution
(deque<LinearConstraintInequalityFreeFeet_t> &
 QueueOfLConstraintInequalitiesFreeFeet,
 deque<SupportFeet_t> & QueueOfSupportFeet,
 unsigned int NumberSteps,
 double *XkYk)
{
  /*! The initial solution of the problem is given by identical zmp and
    feet positions     that are lying inside the feasible zones defined
    by the constraints on the feet placements.
    U0 = -iPu * Px [Xkt Ykt]t + iPu * InitialZMP
  */

  deque<LinearConstraintInequalityFreeFeet_t>::iterator LCIFF_it;
  LCIFF_it = QueueOfLConstraintInequalitiesFreeFeet.begin();

  //Current support foot coordinates
  deque<SupportFeet_t>::iterator CurSF_it;
  CurSF_it = QueueOfSupportFeet.end();
  CurSF_it--;

  int StepNumber = 0;
  int SupportFoot = CurSF_it->SupportFoot;


  for(unsigned int i=0; i<m_CardV; i++)
    {
      //Change of the previewed support foot
      if(LCIFF_it->StepNumber>StepNumber)
        {
          StepNumber++;
          SupportFoot = -SupportFoot;
        }
      //Initial zmp solutions
      if(SupportFoot == CurSF_it->SupportFoot)
        {
          m_InitialZMPSolution[i] = CurSF_it->x;
          m_InitialZMPSolution[m_CardV+i] = CurSF_it->y;
        }
      else
        {
          m_InitialZMPSolution[i] =
            CurSF_it->x-(double)SupportFoot*
            sin(CurSF_it->theta)*m_DistanceFeetCenters;
          m_InitialZMPSolution[m_CardV+i] =
            CurSF_it->y+(double)SupportFoot*
            cos(CurSF_it->theta)*m_DistanceFeetCenters;
        }
      //Initial feet positions
      m_InitialZMPSolution[2*m_CardV+StepNumber-1] = m_InitialZMPSolution[i];
      m_InitialZMPSolution[2*m_CardV+NumberSteps+StepNumber-1] =
        m_InitialZMPSolution[m_CardV+i];

      LCIFF_it++;
    }

  for(unsigned int i=0; i<m_CardV; i++)
    {
      m_Vk[i] = 0.0;
      m_Vk[i+m_CardV] = 0.0;
      for(unsigned int j=0; j<3; j++)
        m_Vk[i]-= m_iPuPx[i*6+j] *  XkYk[j];

      for(unsigned int j=3; j<6; j++)
        m_Vk[i+m_CardV]-= m_iPuPx[(i+m_CardV)*6+j] *  XkYk[j];

      for(unsigned int j=0; j<m_CardV; j++)
        m_Vk[i]+= m_iPu[j*(2*m_CardV+2*NumberSteps)+i] *
          m_InitialZMPSolution[j];

      for(unsigned int j=0; j<m_CardV; j++)
        m_Vk[i+m_CardV]+= m_iPu[j*(2*m_CardV+2*NumberSteps)+i] *
          m_InitialZMPSolution[j+m_CardV];

    }

  for(unsigned int i=0; i<NumberSteps; i++)
    {
      m_Vk[2*m_CardV+i] = 0.0;
      m_Vk[2*m_CardV+NumberSteps+i] = 0.0;

      for(unsigned int j=0; j<NumberSteps; j++)
        {
          m_Vk[2*m_CardV+i]+=
            m_iPu[(2*m_CardV+i)*(2*m_CardV+2*NumberSteps)+m_CardV+j]
            * m_InitialZMPSolution[j+2*m_CardV];
          m_Vk[2*m_CardV+NumberSteps+i]+=
            m_iPu[(2*m_CardV+NumberSteps+i)*(2*m_CardV+2*NumberSteps)+
                  m_CardV+NumberSteps+j] *
            m_InitialZMPSolution[j+NumberSteps+2*m_CardV];
        }

    }

  if(m_DebugMode>0)
    {
      ofstream aof;
      aof.open("InitialZMPSolution.dat",ofstream::out);
      for(unsigned int i=0; i<2*(m_CardV+NumberSteps); i++)
        aof <<m_InitialZMPSolution[i] << " ";
      aof << endl;
      aof.close();
    }
  return 0;
}

int PLDPSolverHerdt::ForwardSubstitution()
{
  // Compute v2 q (14b) in Dimitrov 2009.
  // First Phase
  // EE^t v2 = v1 <-> LL^t v2 = v1
  // Now solving
  // L y = v1
  unsigned int startIndex=0;
  /*
    if ((m_HotStart==false) ||
    (m_ItNb>0))
    startIndex = m_ActivatedConstraints.size()-1;
  */
  for(unsigned int i=startIndex; i<m_ActivatedConstraints.size(); i++)
    {
      m_y[i] = m_v1[i] ;
      for(unsigned int k=0; k<i; k++)
        m_y[i] += - m_L[i*m_NbMaxOfConstraints+k]*m_y[k];

      if (m_L[i*m_NbMaxOfConstraints+i]!=0.0)
        m_y[i] /= m_L[i*m_NbMaxOfConstraints+i];
    }
  return 0;
}

int PLDPSolverHerdt::BackwardSubstitution()
{
  // Compute v2 q (14b) in Dimitrov 2009.
  // Second phase
  // Now solving
  // LL^t v2 = v1 <-> L y = v1 with L^t v2 = y
  // y solved with first phase.
  // So now we are looking for v2.
  unsigned int SizeOfL = m_ActivatedConstraints.size();
  if (SizeOfL==0)
    return 0;

  ODEBUG("BackwardSubstitution " << m_ItNb);
  for(int i=SizeOfL-1;
      i>=0; i--)
    {
      double tmp=0.0;
      m_v2[i]= m_y[i];
      for(int k=i+1; k<(int)SizeOfL; k++)
        {
          if (k==(int)SizeOfL-1)
            tmp = m_v2[i];

          m_v2[i] -=  m_L[k*m_NbMaxOfConstraints+i]*m_v2[k];
        }
      m_v2[i] = m_v2[i]/m_L[i*m_NbMaxOfConstraints+i];

      tmp = tmp/m_L[i*m_NbMaxOfConstraints+i];
      ODEBUG("BS: m_L[i*m_NbMaxOfConstraints+i]:"<<
             m_L[i*m_NbMaxOfConstraints+i] << " " << m_y[i]);
      ODEBUG("m_v2[" << i<< " ] = "<<m_v2[i] << " " << tmp);
    }
  return 0;
}



int PLDPSolverHerdt::ComputeProjectedDescentDirection(unsigned int NumberSteps)
{

  if(m_DebugMode>1)
    {
      ofstream aof;

      char Buffer[1024];
      sprintf(Buffer,"AC_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int li=0; li<m_ActivatedConstraints.size(); li++)
        aof<< m_ActivatedConstraints[li] << " ";
      aof <<endl;
      aof.close();

      sprintf(Buffer,"E_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int li=0; li<m_ActivatedConstraints.size(); li++)
        {
          unsigned int RowCstMatrix = m_ActivatedConstraints[li];
          for(unsigned int lj=0; lj<2*(m_CardV+NumberSteps); lj++)
            {
              aof << m_A[RowCstMatrix+lj*(m_NbOfConstraints+1)] << " ";
            }
          aof <<endl;
        }
      aof.close();

      sprintf(Buffer,"c_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int li=0; li<2*(m_CardV+NumberSteps); li++)
        {
          aof << m_UnconstrainedDescentDirection[li] << " " ;
        }
      aof << endl;
      aof.close();
    }
  // Compute v1 eq (14a) in Dimitrov 2009
  ODEBUG_NENDL("m_v1(" <<m_ItNb << ")= [");
  unsigned int startIndex=0;
  /*
    if ((m_HotStart==false) ||
    (m_ItNb>0))
    startIndex = m_ActivatedConstraints.size()-1;
  */
  for(unsigned int li=startIndex; li<m_ActivatedConstraints.size(); li++)
    {
      m_v1[li] = 0.0;
      unsigned int RowCstMatrix = m_ActivatedConstraints[li];
      ODEBUG("RowCstMatrix:"<<RowCstMatrix);
      for(unsigned int lj=0; lj<2*(m_CardV+NumberSteps); lj++)
        {
          m_v1[li]+= m_A[RowCstMatrix+lj*(m_NbOfConstraints+1)]*
            m_UnconstrainedDescentDirection[lj];
        }
      ODEBUG_NENDL(m_v1[li]<< " ");
    }
  ODEBUG_NENDL("]" << std::endl);

  if (m_DebugMode>1)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"v1_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int lj=0; lj<m_ActivatedConstraints.size(); lj++)
        aof << m_v1[lj] << endl;
      aof.close();
    }

  // Compute v2 by Forward and backward substitution.
  // v2 = iLt iL E c
  // LtL v2 = c
  // Lt y =c
  ForwardSubstitution();
  if (m_DebugMode>1)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"y_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int lj=0; lj<m_ActivatedConstraints.size(); lj++)
        aof << m_y[lj] << endl;
      aof.close();

    }

  // Solve L v2 = y
  BackwardSubstitution();

  if (m_DebugMode>1)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"v2_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int lj=0; lj<m_ActivatedConstraints.size(); lj++)
        aof << m_v2[lj] << endl;
      aof.close();
    }

  // Compute d
  // d = c - Et v2
  ODEBUG("Size of ActivatedConstraints: "<<
         m_ActivatedConstraints.size());
  for(unsigned int li=0; li<2*(m_CardV+NumberSteps); li++)
    {
      m_d[li] = m_UnconstrainedDescentDirection[li];
      for(unsigned int lj=0; lj<m_ActivatedConstraints.size(); lj++)
        {
          unsigned int RowCstMatrix = m_ActivatedConstraints[lj];
          m_d[li]-= m_A[RowCstMatrix+li*(m_NbOfConstraints+1)]*
            m_v2[lj];
        }
    }

  if (m_DebugMode>1)
    {
      ofstream aof;
      char Buffer[1024];
      sprintf(Buffer,"UDD_%02d.dat",m_ItNb);
      aof.open(Buffer,ofstream::out);
      for(unsigned int lj=0; lj<2*(m_CardV+NumberSteps); lj++)
        aof  << m_d[lj] << " ";
      aof << endl;
      aof.close();
    }
  return 0;
}

double PLDPSolverHerdt::
ComputeAlpha
(vector<unsigned int> & NewActivatedConstraints,
 unsigned int NumberSteps)
{
  double Alpha=10000000.0;
  double *ptA = 0;
  bool TheConstraintIsToBeAdded=false;
  unsigned int TheConstraintToActivate=0;

  for(unsigned li=0; li<m_NbOfConstraints; li++)
    {
      bool ConstraintAlreadyActive=false;

      // Register that this constraint has yet been computed.
      m_ConstraintsValueComputed[li] = false;
      m_ConstraintsValueComputed[li+m_NbOfConstraints] = false;

      // Make sure that this constraint is not already activated.
      for(unsigned int ConstraintIndex=0;
          ConstraintIndex<m_ActivatedConstraints.size();
          ConstraintIndex++)
        {
          if (m_ActivatedConstraints[ConstraintIndex]==li)
            {
              ConstraintAlreadyActive=true;
              break;
            }
        }
      if (ConstraintAlreadyActive)
        continue;

      ptA = m_A + li;

      m_tmp1[li]=0.0;

      //Check if we can not reuse an already computed result
      {
        bool ToBeComputed=true;

        if(ToBeComputed)
          for(unsigned lj=0; lj<2*(m_CardV+NumberSteps); lj++)
            {
              m_tmp1[li]+= *ptA * m_d[lj];
              ptA+=(m_NbOfConstraints+1);
            }
      }

      m_ConstraintsValueComputed[li] = true;

      if (m_tmp1[li]<0.0)
        {
          double lalpha=0.0;
          double *pt2A = m_A + li;
          m_tmp2[li]= -m_b[li];
          // Check if we can not reuse an already computed result
          {
            bool ToBeComputed=true;

            if(ToBeComputed)
              for(unsigned lj=0; lj<2*(m_CardV+NumberSteps); lj++)
                {
                  m_tmp2[li]-= *pt2A * m_Vk[lj];
                  pt2A+=(m_NbOfConstraints+1);
                }
          }

          if (m_tmp2[li]>m_tol)
            {
              std::cerr << "PB ON constraint "<<li
                        << " at time " << m_InternalTime << endl;
              std::cerr << " Check current V k="<<m_ItNb<< endl;
              std::cerr << " should be feasible : " << m_tmp2[li]
                        << " " << -m_v2[li] << endl;
            }
          else if (m_tmp2[li]>0.0)
            m_tmp2[li] = -m_tol;

          lalpha = m_tmp2[li]/m_tmp1[li];

          if (Alpha>lalpha)
            {
              ODEBUG("m_v2[li] : "<< m_v2[li] <<
                     " m_v1[li] : " << m_v1[li] << " "
                     " lalpha: "<< lalpha << " "
                     " Constraint " << li << " of "
                     << m_NbOfConstraints << " constraints.");

              Alpha = lalpha;
              if (Alpha<1)
                {
                  TheConstraintIsToBeAdded = true;
                  TheConstraintToActivate=li;
                }
            }
        }
    }
  ODEBUG("TheConstraintToActivate"<<TheConstraintToActivate);
  if (TheConstraintIsToBeAdded)
    NewActivatedConstraints.push_back(TheConstraintToActivate);

  return Alpha;
}
int PLDPSolverHerdt::
SolveProblem
(deque<LinearConstraintInequalityFreeFeet_t> &
 QueueOfLConstraintInequalitiesFreeFeet,
 deque<SupportFeet_t> & QueueOfSupportFeet,
 double *CstPartOfTheCostFunction,
 unsigned int NbOfConstraints,
 double *LinearPartOfConstraints,
 double *CstPartOfConstraints,
 double *XkYk,
 double *X,
 unsigned int NumberOfRemovedConstraints,
 unsigned int NbRemovedFootCstr,
 bool StartingSequence,
 unsigned int NumberSteps,
 bool CurrentStateChanged,
 double time)
{
  vector<unsigned int> NewActivatedConstraints;
  if (StartingSequence)
    m_InternalTime = 0.0;

  AllocateMemoryForSolver(NumberSteps);
  InitializeSolver(NumberSteps);

  m_A = LinearPartOfConstraints;
  m_b = CstPartOfConstraints;
  m_NbOfConstraints = NbOfConstraints;

  /* Step zero : Algorithm initialization. */
  m_CstPartOfCostFunction = CstPartOfTheCostFunction;

  ODEBUG("State: " << XkYk[0] << " " << XkYk[3] << " " <<
         XkYk[1] << " " << XkYk[4] << " " <<
         XkYk[2] << " " << XkYk[5] << " ");

  ComputeInitialSolution(QueueOfLConstraintInequalitiesFreeFeet,
                         QueueOfSupportFeet, NumberSteps, XkYk);

  ODEBUG("DebugMode:"<<m_DebugMode);

  if (m_DebugMode>1)
    {
      string DebugVkFileName("VkInit.dat");
      WriteCurrentZMPSolution("VkInit.dat",XkYk);

      char Buffer[1024];
      sprintf(Buffer,"InitialSolution_%02f.dat",time);
      ofstream aof;
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0; i<2*(m_CardV+NumberSteps); i++)
        aof <<m_Vk[i] << " ";
      aof << endl;
      aof.close();

      sprintf(Buffer,"iPuPx_%02f.dat",time);
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0; i<2*(m_CardV); i++)
        {
          for(unsigned int j=0; j<6; j++)
            aof << m_iPuPx[i*6+j] << " ";
          aof <<endl;
        }
      aof << endl;
      aof.close();

      sprintf(Buffer,"iPu_%02f.dat",time);
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0; i<2*(m_CardV+NumberSteps); i++)
        {
          for(unsigned int j=0; j<2*(m_CardV+NumberSteps); j++)
            aof << m_iPu[i*2*(m_CardV+NumberSteps)+j] << " ";
          aof <<endl;
        }
      aof << endl;
      aof.close();

      sprintf(Buffer,"A_%02f.dat",time);
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0; i<m_NbOfConstraints; i++)
        {
          for(unsigned int j=0; j<2*(m_CardV+NumberSteps); j++)
            aof << m_A[j*(m_NbOfConstraints+1)+i] << " ";
          aof <<endl;
        }
      aof << endl;
      aof.close();

      aof.open("b.dat",ofstream::out);
      for(unsigned int i=0; i<m_NbOfConstraints; i++)
        {
          aof << m_b[i] << " ";
        }
      aof << endl;
      aof.close();

      sprintf(Buffer,"XkYk_%02f.dat",time);
      aof.open(Buffer,ofstream::out);
      for(unsigned int i=0; i<6; i++)
        {
          aof << XkYk[i] << " ";
        }
      aof << endl;
      aof.close();
    }


  bool ContinueAlgo=true;

  /*! Initialization de cholesky. */
  m_OptCholesky->SetA(LinearPartOfConstraints,m_NbOfConstraints);
  m_OptCholesky->SetToZero();

  struct timeval begin;

  gettimeofday(&begin,0);

  // Hot Start
  if (m_HotStart)
    {
      unsigned int lindex,IndexFootCstr;
      //Shift the constraints
      for(unsigned int i=0; i<m_PreviouslyActivatedConstraints.size(); i++)
        {
          if(m_PreviouslyActivatedConstraints[i] < 4*m_CardV)//ZMP constraints
            {
              lindex=m_PreviouslyActivatedConstraints[i]-
                NumberOfRemovedConstraints;
            }
          else if(m_PreviouslyActivatedConstraints[i] >= 4*m_CardV
                  && CurrentStateChanged == true)//Foot placement constraints
            {
              IndexFootCstr = m_PreviouslyActivatedConstraints[i]-
                NbRemovedFootCstr;
            }

          m_ActivatedConstraints.push_back(lindex);

          if(IndexFootCstr>=4*m_CardV)
            {
              m_ActivatedConstraints.push_back(IndexFootCstr);
            }
        }
      if(m_ActivatedConstraints.size()>0)
        m_OptCholesky->AddActiveConstraints(m_ActivatedConstraints);
    }

  m_PreviouslyActivatedConstraints.clear();

  double alpha=0.0;
  m_ItNb=0;
  while(ContinueAlgo)
    {
      ODEBUG("Iteration Number:" << m_ItNb);
      /* Step one : Compute descent direction. */
      for(unsigned int i=0; i<2*(m_CardV+NumberSteps); i++)
        m_UnconstrainedDescentDirection[i] =
          -m_CstPartOfCostFunction[i] -  m_Vk[i];

      /*! Step two: Compute the projected descent direction. */
      ComputeProjectedDescentDirection(NumberSteps);

      /*! Step three : Compute alpha */
      alpha = ComputeAlpha(NewActivatedConstraints, NumberSteps);

      if (m_DebugMode>1)
        {

          ODEBUG("Alpha:" <<alpha);

          ofstream aof;
          char Buffer[1024];
          sprintf(Buffer,"U_%.3f_%02d.dat",m_InternalTime,m_ItNb);
          aof.open(Buffer,ofstream::out);
          for(unsigned int i=0; i<2*(m_CardV+NumberSteps); i++)
            {
              aof << m_Vk[i] << " " ;
            }
          aof.close();

          sprintf(Buffer,"Zk_%02d.dat",m_ItNb);
          WriteCurrentZMPSolution(Buffer,XkYk);
        }


      if (alpha>=1.0)
        {
          alpha=1.0;
          ContinueAlgo=false;
        }
      if (alpha<0.0)
        {
          std::cerr << "Problem with alpha: should be positive"
                    << std::endl;
          std::cerr << "The initial solution is incorrect: "
                    << m_ItNb << " "
                    << m_InternalTime
                    << std::endl;
          exit(0);
        }

      /*! Compute new solution. */
      for(unsigned int i=0; i<2*(m_CardV+NumberSteps); i++)
        {
          m_Vk[i] = m_Vk[i] + alpha*m_d[i];
        }

      if (ContinueAlgo)
        {
          ODEBUG("Nb of activated constraints: "
                 << NewActivatedConstraints.size());
          m_OptCholesky->AddActiveConstraints(NewActivatedConstraints);
          for(unsigned int i=0; i<NewActivatedConstraints.size(); i++)
            m_ActivatedConstraints.push_back(NewActivatedConstraints[i]);

          NewActivatedConstraints.clear();

          if (m_DebugMode>1)
            {
              ofstream aof;
              char Buffer[1024];
              sprintf(Buffer,"LE_%02d.dat",m_ItNb);
              aof.open(Buffer,ofstream::out);
              for(unsigned int i=0; i<m_ActivatedConstraints.size(); i++)
                {
                  for(unsigned int j=0; j<m_ActivatedConstraints.size(); j++)
                    {
                      aof << m_L[i*m_NbMaxOfConstraints+j] << " ";
                    }
                  aof<<endl;
                }
              aof.close();
              ODEBUG("m_L(0,0)= " << m_L[0]);
              sprintf(Buffer,"alpha_%02d.dat",m_ItNb);
              aof.open(Buffer,ofstream::out);
              aof << alpha << endl;
              aof.close();

            }

        }

      // If limited computation time stop the algorithm.
      if (m_LimitedComputationTime)
        {
          struct timeval current;
          gettimeofday(&current,0);
          double r=(double)(current.tv_sec-begin.tv_sec) +
            0.000001*(current.tv_usec-begin.tv_usec);
          if (r> m_AmountOfLimitedComputationTime)
            {
              ContinueAlgo=false;
            }
        }

      m_ItNb++;
    }

  for(unsigned int i=0; i<2*(m_CardV+NumberSteps); i++)
    X[i] = m_Vk[i];


  if (m_HotStart)
    {
      ODEBUG("AR (" << lTime <<") :" );
      for( unsigned int i=0; i<m_ActivatedConstraints.size(); i++)
        {
          ODEBUG( "( " << m_ActivatedConstraints[i] << " , "
                  << m_v2[i] << " ) ");
          if (m_v2[i]<0.0)
            {
              m_PreviouslyActivatedConstraints.
                push_back(m_ActivatedConstraints[i]);
              ODEBUG(m_ActivatedConstraints[i] << " " );
            }
        }

      ODEBUG( (int)m_ActivatedConstraints.size() -
              (int)m_PreviouslyActivatedConstraints.size() <<  " "
              << m_PreviouslyActivatedConstraints.size() );
      //StoreCurrentZMPSolution(XkYk);

    }

  if (1)
    {
      ofstream aof;
      aof.open("ActivatedConstraints.dat",ofstream::app);
      for(unsigned int i=0; i<320; i++)
        {
          bool FoundConstraint=false;

          if (i<m_NbOfConstraints)
            {
              for(unsigned int j=0; j<m_ActivatedConstraints.size(); j++)
                if (m_ActivatedConstraints[j]==i)
                  {
                    aof << "1 ";
                    FoundConstraint=true;
                    break;
                  }
            }
          if (!FoundConstraint)
            aof << "0 ";

        }
      aof << endl;
      aof.close();
    }

  if ((isnan(X[0])) ||
      (isnan(X[m_CardV])) ||
      (isinf(X[0])) ||
      (isinf(X[m_CardV]))
      )
    {
      cout << "Nan or inf value " << X[0]<< " " << X[m_CardV]
           << " at iteration " << m_ItNb -1 <<endl;
      return -1;
    }

  if (m_DebugMode>1)
    {
      ofstream aof;
      aof.open("NbIt.dat",ofstream::out);
      aof << m_ItNb;
      aof.close();

      aof.open("Pb.dat",ofstream::out);
      aof<<" Number of iterations " << m_ItNb << endl;
      aof.close();

      aof.open("UFinal.dat",ofstream::out);
      for(unsigned int i=0; i<2*(m_CardV+NumberSteps); i++)
        {
          aof << X[i] << " " ;
        }
      aof<<endl;
      aof.close();

      WriteCurrentZMPSolution("FinalSolution.dat",XkYk);
      aof.open("Pb.dat",ofstream::out);
      aof<<" Number of iterations " << m_ItNb << endl;
      aof.close();

    }


  string Buffer("InfosPLDP");
  if (m_HotStart)
    Buffer+="HS";
  if (m_LimitedComputationTime)
    Buffer+="LT";
  Buffer+=".dat";

  ODEBUG6(m_ActivatedConstraints.size() << " "
          << NbOfConstraints << " "
          << m_ActivatedConstraints.size() -
          m_PreviouslyActivatedConstraints.size()
          << " "
          << m_ItNb,(char*)Buffer.c_str());

  m_InternalTime += 0.02;
  return 0;
}


/* Write the current solution for debugging. */
void PLDPSolverHerdt::WriteCurrentZMPSolution(string filename,
                                              double *XkYk)
{
  // The current solution is Vk,
  // but its graphical representation is better understood
  // when transformed in ZMP ref trajectory.
  int lZMP_size=2*m_CardV;
  double* lZMP = new double [lZMP_size];

  ofstream aof;
  aof.open(filename.c_str(),ofstream::out);


  for(unsigned int i=0; i<m_CardV; i++)
    {
      lZMP[i] = 0.0; // X axis
      lZMP[i+m_CardV] =0.0; // Y axis
      for(unsigned int j=0; j<m_CardV; j++)
        {
          lZMP[i] += m_Pu[j*m_CardV+i] * m_Vk[j];
          lZMP[i+m_CardV] += m_Pu[j*m_CardV+i] * m_Vk[j+m_CardV];
        }
      for(unsigned int j=0; j<3; j++)
        {
          lZMP[i] += m_Px[i*3+j] * XkYk[j];
          //     lZMP[i] += m_Px[i*3+j] * XkYk[j+3];
          //     lZMP[i+m_CardV] += m_Px[i*3+j] * XkYk[j];
          lZMP[i+m_CardV] += m_Px[i*3+j] * XkYk[j+3];
        }
      aof << lZMP[i] << " " << lZMP[i+m_CardV] << endl;
    }

  aof.close();
  delete lZMP;
}

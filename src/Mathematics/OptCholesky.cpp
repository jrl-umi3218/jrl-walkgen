/*
 * Copyright 2007, 2008, 2009, 2010,
 *
 * Olivier    Stasse
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
/** \file OptCholesky.cpp
    \brief This object performs a cholesky decomposition optimized for
    the problem to solved. */
#include <math.h>
#include <Mathematics/OptCholesky.hh>
#include <Debug.hh>

using namespace PatternGeneratorJRL;

OptCholesky::OptCholesky(unsigned int lNbMaxOfConstraints,
                         unsigned int lCardU,
                         unsigned int lUpdateMode):
  m_NbMaxOfConstraints(lNbMaxOfConstraints),
  m_CardU(lCardU),
  m_A(0),
  m_L(0),
  m_iL(0),
  m_UpdateMode(lUpdateMode),
  m_NbOfConstraints(0)
{
  InitializeInternalVariables();
}

OptCholesky::~OptCholesky()
{
}


void OptCholesky::InitializeInternalVariables()
{
}

void OptCholesky::SetToZero()
{
#if 0
  if (m_NbMaxOfConstraints!=0)
    {
      if (m_L!=0)
        for(unsigned int i=0;
            i<m_NbMaxOfConstraints * m_NbMaxOfConstraints;
            i++)
          m_L[i]=0.0;


    }
#endif
  m_SetActiveConstraints.clear();

}

void OptCholesky::SetA(double *aA,
                       unsigned int lNbOfConstraints)
{
  m_A = aA;
  m_NbOfConstraints = lNbOfConstraints;
}


int OptCholesky::AddActiveConstraints(vector<unsigned int> & lConstraints)
{
  int r=0;
  for(unsigned int li=0; li<lConstraints.size(); li++)
    {
      r=AddActiveConstraint(lConstraints[li]);
      if (r<0)
        return -((int)(li));
    }
  return r;
}

int OptCholesky::AddActiveConstraint(unsigned int aConstraint)
{
  /* Update set of active constraints */
  m_SetActiveConstraints.push_back(aConstraint);

  int r = 0;
  if (m_UpdateMode==MODE_NORMAL)
    UpdateCholeskyMatrixNormal();
  else if (m_UpdateMode==MODE_FORTRAN)
    UpdateCholeskyMatrixFortran();

  return r;
}

std::size_t OptCholesky::CurrentNumberOfRows()
{
  return m_SetActiveConstraints.size();
}

void OptCholesky::SetL(double *aL)
{
  m_L = aL;
}

void OptCholesky::SetiL(double *aiL)
{
  if (m_iL!=0)
    delete [] m_iL;
  m_iL = aiL;
}

int OptCholesky::UpdateCholeskyMatrixNormal()
{

  if ((m_A==0) | (m_L==0))
    return -1;

  double Mij=0.0;
  std::size_t  IndexNewRowAKAi = 0;
  if (m_SetActiveConstraints.size()>0)
    IndexNewRowAKAi = m_SetActiveConstraints.size()-1;

  double *PointerArow_i = m_A +  m_CardU *
    m_SetActiveConstraints[IndexNewRowAKAi];

  /* Compute Li,j */
  for(int lj=0; lj<(int)m_SetActiveConstraints.size(); lj++)
    {

      /* A value M(i,j) is computed once,
         directly from the matrix A */
      double *Arow_i = PointerArow_i;
      double *Arow_j = m_A + m_CardU * m_SetActiveConstraints[lj];
      Mij=0.0;
      for(int lk=0; lk<(int)m_CardU; lk++)
        {
          Mij+= (*Arow_i++) * (*Arow_j++);
        }

      /* */
      double r = Mij;
      double * ptLik =m_L + IndexNewRowAKAi*m_NbMaxOfConstraints;
      double * ptLjk =m_L + lj*m_NbMaxOfConstraints;

      for(int lk=0; lk<lj; lk++)
        {
          r = r - (*ptLik++)  * (*ptLjk++);
        }
      if (lj!=(int)m_SetActiveConstraints.size()-1)
        m_L[IndexNewRowAKAi*m_NbMaxOfConstraints+lj]=
          r/m_L[lj*m_NbMaxOfConstraints+lj];
      else
        m_L[IndexNewRowAKAi*m_NbMaxOfConstraints+lj] =  sqrt(r);

    }

  return 0;

}

int OptCholesky::UpdateCholeskyMatrixFortran()
{

  if ((m_A==0) | (m_L==0))
    return -1;

  double Mij=0.0;
  std::size_t IndexNewRowAKAi = 0;
  if (m_SetActiveConstraints.size()>0)
    IndexNewRowAKAi = m_SetActiveConstraints.size()-1;

  double *PointerArow_i = m_A +
    m_SetActiveConstraints[IndexNewRowAKAi];

  /* Compute Li,j */
  for(int lj=0; lj<(int)m_SetActiveConstraints.size(); lj++)
    {

      /* A value M(i,j) is computed once,
         directly from the matrix A */
      double *Arow_i = PointerArow_i;
      double *Arow_j = m_A + m_SetActiveConstraints[lj];
      Mij=0.0;
      for(int lk=0; lk<(int)m_CardU; lk++)
        {
          Mij+= (*Arow_i) * (*Arow_j);
          Arow_i+= m_NbOfConstraints+1;
          Arow_j+= m_NbOfConstraints+1;
        }

      /* */
      double r = Mij;
      ODEBUG("r: M("<< m_SetActiveConstraints[IndexNewRowAKAi] << ","
             << m_SetActiveConstraints[lj] <<")="<< r);
      double * ptLik =m_L + IndexNewRowAKAi*m_NbMaxOfConstraints;
      double * ptLjk =m_L + lj*m_NbMaxOfConstraints;

      for(int lk=0; lk<lj; lk++)
        {
          r = r - (*ptLik++)  * (*ptLjk++);
        }
      if (lj!=(int)m_SetActiveConstraints.size()-1)
        m_L[IndexNewRowAKAi*m_NbMaxOfConstraints+lj]=
          r/m_L[lj*m_NbMaxOfConstraints+lj];
      else
        m_L[IndexNewRowAKAi*m_NbMaxOfConstraints+lj] = sqrt(r);

      ODEBUG("m_L(" << IndexNewRowAKAi << "," << lj << ")="
             << m_L[IndexNewRowAKAi*m_NbMaxOfConstraints+lj] );
    }

  return 0;

}

int OptCholesky::ComputeNormalCholeskyOnANormal()
{
  if ((m_A==0) | (m_L==0))
    return -1;
  if (m_NbMaxOfConstraints!=m_CardU)
    return -2;


  double *pA = m_A;
  for(int li=0; li<(int)m_NbMaxOfConstraints; li++)
    {
      for(int lj=0; lj<=li; lj++)
        {

          /* Compute Li,j */

          double r = pA[lj];
          double * ptLik =m_L + li*m_NbMaxOfConstraints;
          double * ptLjk =m_L + lj*m_NbMaxOfConstraints;

          for(int lk=0; lk<lj; lk++)
            {
              r = r - (*ptLik++)  * (*ptLjk++);
            }
          if (lj!=li)
            m_L[li*m_NbMaxOfConstraints+lj]=r/m_L[lj*m_NbMaxOfConstraints+lj];
          else
            m_L[li*m_NbMaxOfConstraints+lj] = sqrt(r);

        }
      pA+=m_NbMaxOfConstraints;
    }
  return 0;

}

int OptCholesky::ComputeInverseCholeskyNormal(int mode)
{
  if (m_iL==0)
    {
      std::cerr << "No memory allocated for iL" << endl;
      return -1;
    }

  std::size_t LocalSize =0;
  if (mode==0)
    LocalSize = m_SetActiveConstraints.size();
  else
    LocalSize = m_NbMaxOfConstraints;

  for(long int lj=(long int)LocalSize-1; lj>=0; lj--)
    {
      double iLljlj=0.0;
      m_iL[lj*m_NbMaxOfConstraints+lj] =
        iLljlj = 1/m_L[lj*m_NbMaxOfConstraints+lj];

      for(long int li=lj+1; li<(long int)LocalSize; li++)
        {

          /* Compute Li,j */
          double r = 0.0;
          double * ptiLik = m_iL + li*m_NbMaxOfConstraints + lj + 1;
          double * ptLjk  = m_L  + (lj+1)*m_NbMaxOfConstraints + lj ;

          for(long int lk=lj+1; lk<(long int)LocalSize; lk++)
            {
              r = r + (*ptiLik++)  * (*ptLjk);
              ptLjk+=m_NbMaxOfConstraints;
            }

          m_iL[li*m_NbMaxOfConstraints+lj]= -iLljlj*r;


        }
    }
  return 0;

}

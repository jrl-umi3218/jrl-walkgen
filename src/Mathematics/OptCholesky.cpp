#include <walkGenJrl/Mathematics/OptCholesky.h>

using namespace PatternGeneratorJRL;

OptCholesky::OptCholesky(unsigned int lNbMaxOfConstraints,
			 unsigned int lCardU):
  m_NbMaxOfConstraints(lNbMaxOfConstraints),
  m_CardU(lCardU),
  m_A(0),
  m_L(0)
{
  if (lCardU<lNbMaxOfConstraints)
    {
      cout << "CardU<lNbMaxOfConstraints thus" << endl;
      cout << "rank(A*A')<NbMaxOfConstraints" << endl;
      cout << "It is very likely" <<endl;
      cout << "that the matrix is *NOT* symmetric definite-positive" <<endl;
    }
  InitializeInternalVariables();
}

OptCholesky::~OptCholesky()
{
  FreeMemory();
}

void OptCholesky::FreeMemory()
{

  if (m_L!=0)
    delete [] m_L ;
}

void OptCholesky::InitializeInternalVariables()
{
  if (m_NbMaxOfConstraints!=0)
    {
      m_L = new double [m_NbMaxOfConstraints * m_NbMaxOfConstraints];
      for(unsigned int i=0;i<m_NbMaxOfConstraints * m_NbMaxOfConstraints;i++)
	m_L[i]=0.0;
    }
}

void OptCholesky::SetA(double *aA)
{
  m_A = aA;
}

int OptCholesky::AddActiveConstraints(vector<unsigned int> & lConstraints)
{
  int r=0;
  for(unsigned int li=0;li<lConstraints.size();li++)
    {
      r=AddActiveConstraint(lConstraints[li]);
      if (r<0)
	return -li;
    }
  return r;
}

int OptCholesky::AddActiveConstraint(unsigned int aConstraint)
{
  /* Update set of active constraints */
  m_SetActiveConstraints.push_back(aConstraint);

  int r = UpdateCholeskyMatrix();
  return r;
}

int OptCholesky::CurrentNumberOfRows()
{
  return m_SetActiveConstraints.size();
}

double * OptCholesky::GetL()
{
  return m_L;
}

int OptCholesky::UpdateCholeskyMatrix()
{

  if (m_A==0)
    return -1;

  double Mij=0.0;
  unsigned int IndexNewRowAKAi = 0;
  if (m_SetActiveConstraints.size()>0)
    IndexNewRowAKAi = m_SetActiveConstraints.size()-1;
  
  double *PointerArow_i = m_A +  m_CardU * 
    m_SetActiveConstraints[IndexNewRowAKAi];

  /* Compute Li,j */
  for(int lj=0;lj<(int)m_SetActiveConstraints.size();lj++)
    {

      /* A value M(i,j) is computed once,
	 directly from the matrix A */      
      double *Arow_i = PointerArow_i;
      double *Arow_j = m_A + m_CardU* m_SetActiveConstraints[lj];
      Mij=0.0;
      for(int lk=0;lk<(int)m_CardU;lk++)
	{
	  Mij+= (*Arow_i++) * (*Arow_j++);
	}

      /* */
      double r = Mij;
      double * ptLik =m_L + IndexNewRowAKAi*m_NbMaxOfConstraints;
      double * ptLjk =m_L + lj*m_NbMaxOfConstraints;

      for(int lk=0;lk<lj;lk++)
	{
	  r = r - (*ptLik++)  * (*ptLjk++);
	}
      if (lj!=(int)m_SetActiveConstraints.size()-1)
	m_L[IndexNewRowAKAi*m_NbMaxOfConstraints+lj]=r/m_L[lj*m_NbMaxOfConstraints+lj];
      else
	m_L[IndexNewRowAKAi*m_NbMaxOfConstraints+lj] = sqrt(r);
      
    }
  
  return 0;
  
}

int OptCholesky::ComputeNormalCholeskyOnA()
{
  if (m_A==0)
    return -1;
  if (m_NbMaxOfConstraints!=m_CardU)
    return -2;

  double *pA = m_A;
  for(int li=0;li<(int)m_NbMaxOfConstraints;li++)
    {
      for(int lj=0;lj<=li;lj++)
	{
	  
	  /* Compute Li,j */
	  
	  double r = pA[lj];
	  double * ptLik =m_L + li*m_NbMaxOfConstraints;
	  double * ptLjk =m_L + lj*m_NbMaxOfConstraints;
	  
	  for(int lk=0;lk<lj;lk++)
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

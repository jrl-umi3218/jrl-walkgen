#include <walkGenJrl/Mathematics/OptCholesky.h>

using namespace PatternGeneratorJRL;

OptCholesky::OptCholesky(unsigned int lNbMaxOfConstraints,
			 unsigned int lCardU):
  m_NbMaxOfConstraints(lNbMaxOfConstraints),
  m_CardU(lCardU),
  m_A(0),
  m_L(0)
{

}

OptCholesky::~OptCholesky()
{
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
    }
}

void OptCholesky::SetA(double *aA)
{
  m_A = aA;
}

void OptCholesky::AddActiveConstraints(vector<unsigned int> & lConstraints)
{
  for(unsigned int li=0;li<lConstraints.size();li++)
    AddActiveConstraint(lConstraints[li]);
}

void OptCholesky::AddActiveConstraint(unsigned int aConstraint)
{
  /* Update set of active constraints */
  m_SetActiveConstraints.push_back(aConstraint);
  
}

void OptCholesky::UpdateCholeskyMatrix()
{
  
  double Mij=0.0;
  unsigned IndexNewRowAKAi = m_SetActiveConstraints.size()-1;
  double *PointerArow_i = m_A +  m_CardU * 
    m_SetActiveConstraints[IndexNewRowAKAi];



  /* Compute Li,j */
  for(unsigned int lj=0;lj<m_SetActiveConstraints.size();lj++)
    {

      /* A value M(i,j) is computed once,
	 directly from the matrix A */      
      double *Arow_i = PointerArow_i;
      double *Arow_j = m_A + m_CardU* m_SetActiveConstraints[lj];

      for(unsigned int lk=0;lk<m_CardU;lk++)
	{
	  Mij+= (*Arow_i++) * (*Arow_j++);
	}

      /* */
      double r = Mij;
      double * ptLik =m_L + IndexNewRowAKAi*m_CardU;
      double * ptLjk =m_L + lj*m_CardU;
      for(unsigned int lk=0;lk<lj-1;lk++)
	{
	  r = r - (*ptLik++)  * (*ptLjk++);
	}
      if (lj!=m_SetActiveConstraints.size()-1)
	m_L[IndexNewRowAKAi*m_CardU+lj]=r/m_L[lj*m_CardU+lj];
      else
	m_L[IndexNewRowAKAi*m_CardU+lj] = sqrt(r);
      
    }
  

  
}

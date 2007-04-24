/** Object to perform preview control on a cart model.

   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati
   
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
#include <fstream>
#include <PreviewControl/PreviewControl.h>

using namespace::PatternGeneratorJRL;

PreviewControl::PreviewControl()
{
  MAL_MATRIX_RESIZE(m_A,3,3);
  MAL_MATRIX_RESIZE(m_B,3,1);
  MAL_MATRIX_RESIZE(m_C,1,3);

  MAL_MATRIX_RESIZE(m_Kx,1,3);
  m_Ks = 0;
}

PreviewControl::~PreviewControl()
{

}


void PreviewControl::ReadPrecomputedFile(string aFileName)
{
  std::ifstream aif;

  aif.open(aFileName.c_str(),std::ifstream::in);
  if (aif.is_open())
    {
      aif >> m_Zc;
      aif >> m_SamplingPeriod;
      aif >> m_PreviewControlTime;
      
      
      float r;
      for(int i=0;i<3;i++)
	{
	  aif >> r;
	  m_Kx(0,i)=r ;
	}

      aif >> r;
      m_Ks=r;

      
      m_SizeOfPreviewWindow = (unsigned int)(m_PreviewControlTime/
					     m_SamplingPeriod);
      MAL_MATRIX_RESIZE(m_F,m_SizeOfPreviewWindow,1);

      for(unsigned int i=0;i<m_SizeOfPreviewWindow;i++)
	{
	  aif >> r;
	  m_F(i,0)=r;
	}
      //      cout << (*m_F) << endl;
      double T = m_SamplingPeriod;
      m_A(0,0) = 1.0; m_A(0,1) =   T; m_A(0,2) = T*T/2.0;
      m_A(1,0) = 0.0; m_A(1,1) = 1.0; m_A(1,2) = T;
      m_A(2,0) = 0.0; m_A(2,1) = 0.0; m_A(2,2) = 1.0;

      m_B(0,0) = T*T*T/6.0;
      m_B(1,0) = T*T/2.0;
      m_B(2,0) = T;

      m_C(0,0) = 1.0;
      m_C(0,1) = 0.0;
      m_C(0,2) = -m_Zc/9.81;

      
      aif.close();
    }
  else 
    cerr << "Unable to open " << aFileName << endl;
    
}

int PreviewControl::OneIterationOfPreview(MAL_MATRIX( &x, double), 
					  MAL_MATRIX(& y, double),
					  double & sxzmp, double & syzmp,
					  deque<PatternGeneratorJRL::ZMPPosition> & ZMPPositions,
					  unsigned int lindex,
					  double & zmpx2, double & zmpy2,
					  bool Simulation)
{

  double ux=0.0, uy=0.0;

  MAL_MATRIX_DIM(r,double,1,1);

  // Compute the command.
  r = MAL_RET_A_by_B(m_Kx,x);
  ux = - r(0,0) + m_Ks * sxzmp ;

  if(ZMPPositions.size()<m_SizeOfPreviewWindow)
    {
      cout << "You've got a problem here " << endl;
      exit(0);
    }
  
  for(unsigned int i=0;i<m_SizeOfPreviewWindow;i++)
    ux += m_F(i,0)* ZMPPositions[lindex+i].px;

  r = MAL_RET_A_by_B(m_Kx, y);

  uy = - r(0,0) + m_Ks * syzmp;
  for(unsigned int i=0;i<m_SizeOfPreviewWindow;i++)
    uy += m_F(i,0)* ZMPPositions[lindex+i].py;
  
  x = MAL_RET_A_by_B(m_A,x) + ux * m_B;
  y = MAL_RET_A_by_B(m_A,y) + uy * m_B;
   
  zmpx2 = (MAL_RET_A_by_B(m_C,x))(0,0);
  zmpy2 = (MAL_RET_A_by_B(m_C,y))(0,0);
  
  if (Simulation)
    {
      sxzmp += (ZMPPositions[lindex].px - zmpx2);
      syzmp += (ZMPPositions[lindex].py - zmpy2);
    }
  

  
  return 0;
}


double PreviewControl::GetHeightOfCoM()
{
  return m_Zc;
}

void PreviewControl::print()
{
  cout << "Zc: " <<  m_Zc <<endl;
  cout << "Sampling Period: " << m_SamplingPeriod <<endl;
  cout << "Preview control time window: "<<m_PreviewControlTime<<endl;
  
  for(int i=0;i<3;i++)
    cout << m_Kx(0,i) << " ";
  cout << endl;
  
  cout << "Ks "<< m_Ks << endl;
  
  cout << "F:"<<endl;
  for(unsigned int i=0;i<m_SizeOfPreviewWindow;i++)
    cout << m_F(i,0) << endl;
  
}

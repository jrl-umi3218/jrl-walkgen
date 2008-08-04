/** This object generate all the values for the foot trajectories,
    and the desired ZMP based on a sequence of steps.

    Copyright (c) 2005-2006, 
    Bjorn Verrelst
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
#define _DEBUG_

#include <fstream>
#include <walkGenJrl/MotionGeneration/WaistHeightVariation.h>

using namespace::PatternGeneratorJRL;

#define ODEBUG3(x) cerr << "WaistHeightVariation :" << x << endl
#if 0
#define ODEBUG(x) cerr << "WaistHeightVariation :" <<  x << endl
#else
#define ODEBUG(x) 
#endif

WaistHeightVariation::WaistHeightVariation()
{

  m_PolynomeHip = new WaistPolynome();

}

WaistHeightVariation::~WaistHeightVariation()
{
  if (m_PolynomeHip!=0)
    delete m_PolynomeHip;
	
}


void WaistHeightVariation::PolyPlanner(deque<COMPosition> &aCOMBuffer,
				       deque<RelativeFootPosition> &aFootHolds,
				       deque<ZMPPosition> aZMPPosition)
{
	

  unsigned int u_start=0;
  int stepnumber =0;
  MAL_VECTOR_DIM(aBoundCond,double,4);
  MAL_VECTOR_FILL(aBoundCond,1);

  vector<double> aTimeDistr;
  aTimeDistr.resize(1);

  m_SamplingPeriod = aZMPPosition[1].time-aZMPPosition[0].time;	
	
  aTimeDistr[0]=aFootHolds[stepnumber].SStime+aFootHolds[stepnumber].DStime;

  /*
  cout << "Time distributed computed from " 
       << m_SamplingPeriod << " " 
       << aFootHolds[stepnumber].SStime << " " << aFootHolds[stepnumber].DStime << endl;
  */
  aBoundCond(0)=aCOMBuffer[0].z[0];
  aBoundCond(1)=0.0;
  aBoundCond(2)=aCOMBuffer[0].z[0]+aFootHolds[stepnumber].DeviationHipHeight;
  aBoundCond(3)=0.0;


  m_PolynomeHip->SetParameters(aBoundCond,aTimeDistr);
  //m_PolynomeHip->print(); 
  aCOMBuffer[u_start].z[0]=aCOMBuffer[0].z[0];//+aCOMBuffer[u_start].z[0];
  aCOMBuffer[u_start].z[1]=0.0;
  aCOMBuffer[u_start].z[2]=0.0;


  for (unsigned int u=1; u<aCOMBuffer.size(); u++) 
    {	
      if ((aFootHolds[stepnumber].sx>0)|(stepnumber==0))
	{	
	  if ((aZMPPosition[u-1].stepType==11)&(std::fabs((double)aZMPPosition[u].stepType)==1))
	  {
	    ODEBUG("Deviation Hip Height: " << aFootHolds[stepnumber].DeviationHipHeight);
	    stepnumber++;
			
	    aTimeDistr[0]=aFootHolds[stepnumber].SStime+aFootHolds[stepnumber].DStime;
		
	    u_start=u;
				
	    aBoundCond(0)=aCOMBuffer[u_start].z[0]+aFootHolds[stepnumber-1].DeviationHipHeight;
	    aBoundCond(1)=0.0;
	    aBoundCond(2)=aCOMBuffer[u_start].z[0]+aFootHolds[stepnumber].DeviationHipHeight;
	    aBoundCond(3)=0.0;
			
 
		
	    m_PolynomeHip->SetParameters(aBoundCond,aTimeDistr);
	    //m_PolynomeHip->print(); 
	    aCOMBuffer[u_start].z[0]=m_PolynomeHip->Compute(0);//+aCOMBuffer[u_start].z[0];
	    aCOMBuffer[u_start].z[1]=(aCOMBuffer[u_start].z[0]-aCOMBuffer[u_start-1].z[0])/m_SamplingPeriod;
	    aCOMBuffer[u_start].z[2]=(aCOMBuffer[u_start].z[1]-aCOMBuffer[u_start-1].z[1])/m_SamplingPeriod;

	  }
	else
	  {
					
	    double LocalTime;
	    LocalTime=(u-u_start)*m_SamplingPeriod;	

	    aCOMBuffer[u].z[0]=m_PolynomeHip->Compute(LocalTime);//+aCOMBuffer[u_start].z[0];
	    aCOMBuffer[u].z[1]=(aCOMBuffer[u].z[0]-aCOMBuffer[u-1].z[0])/m_SamplingPeriod;
	    aCOMBuffer[u].z[2]=(aCOMBuffer[u].z[1]-aCOMBuffer[u-1].z[1])/m_SamplingPeriod;
			
	  }
	}
      else
	{		
	  aCOMBuffer[u].z[0]=aCOMBuffer[u-1].z[0];//+aCOMBuffer[u_start].z[0];
	  aCOMBuffer[u].z[1]=0.0;
	  aCOMBuffer[u].z[2]=0.0;
	}

    }


	
#ifdef _DEBUG_

  //cout << "dumping foot data in StepOverBuffers_1.csv" << endl;
  ofstream aof_Buffers;
  static unsigned char FirstCall=1;
  if (FirstCall)
    {
      aof_Buffers.open("WaistBuffers_1.txt",ofstream::out);
    }
  else 
    {
      aof_Buffers.open("WaistBuffers_1.txt",ofstream::app);
    }
		
  if (FirstCall)
    FirstCall = 0;
	
  for (unsigned int i=0;i<aCOMBuffer.size();i++)
    {	
      if (aof_Buffers.is_open())
	{
	  aof_Buffers << 
	    aCOMBuffer[i].x[0] << " "<< 	
	    aCOMBuffer[i].y[0]<< " " << 
	    aCOMBuffer[i].z[0]<< " " << 
	    endl;
	}
    }
		

  if (aof_Buffers.is_open())
    {
      aof_Buffers.close();
    }
#endif	
 
  //return 1;*/	
};



WaistPolynome::WaistPolynome() :Polynome(4)
{
  // SetParameters(boundCond,timeDistr);
}

void WaistPolynome::SetParameters(MAL_VECTOR( boundCond,double), vector<double> timeDistr)
{
  MAL_MATRIX_DIM(Base,double,4,4);
  MAL_MATRIX_DIM(Temp,double,4,4);

  double T;
  double Ts2,Ts3;
  // double t1s2,t1s3,t1s4,t1s5,t1s6,t1s7;
  // double t2s2,t2s3,t2s4,t2s5,t2s6,t2s7;
  
  

  // t1=timeDistr[0];
  // t2=timeDistr[1];
  // T=timeDistr[2];
  T=timeDistr[0];

  Ts2=T*T;Ts3=T*Ts2;
 
  
  Base(0,0)=1.0;Base(0,1)=0.0;Base(0,2)=0.0;Base(0,3)=0.0;
  Base(1,0)=0.0;Base(1,1)=1.0;Base(1,2)=0.0;Base(1,3)=0.0;
  Base(2,0)=1.0;Base(2,1)=T  ;Base(2,2)=Ts2;Base(2,3)=Ts3;
  Base(3,0)=0.0;Base(3,1)=1.0;Base(3,2)=2.0*T;Base(3,3)=3.0*Ts2;
 
  double detBase;


  detBase=MAL_MATRIX_RET_DETERMINANT(Base,double); 

 
  for (unsigned int i=0;i<boundCond.size();i++)
    {
      Temp=Base;
      for(unsigned int j=0;j<MAL_MATRIX_NB_ROWS(Temp);j++)
	Temp(j,i) = boundCond(j);
      m_Coefficients[i] = MAL_MATRIX_RET_DETERMINANT(Temp,double)/detBase;	
    };

}

WaistPolynome::~WaistPolynome()
{  
}







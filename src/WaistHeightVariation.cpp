/** This object generate all the values for the foot trajectories,
    and the desired ZMP based on a sequence of steps.

    CVS Information: 
    $Id: ZMPDiscretization.cpp,v 1.2 2006-01-18 06:34:58 stasse Exp $
    $Author: stasse $
    $Date: 2006-01-18 06:34:58 $
    $Revision: 1.2 $
    $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/ZMPDiscretization.cpp,v $
    $Log: ZMPDiscretization.cpp,v $
    Revision 1.2  2006-01-18 06:34:58  stasse
    OS: Updated the names of t IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    he contributors, the documentation
    and added a sample file for WalkPlugin


    Copyright (c) 2005-2006, 
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
//#define _DEBUG_
#include <fstream>
#include <WaistHeightVariation.h>


using namespace::PatternGeneratorJRL;

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
	
  m_COMBuffer = aCOMBuffer;
  m_FootHolds = aFootHolds;

  unsigned int u_start=0;
  int stepnumber =0;
  VNL::Vector<double> aBoundCond(4,1);
  vector<double> aTimeDistr;
  aTimeDistr.resize(1);

  m_SamplingPeriod = aZMPPosition[1].time-aZMPPosition[0].time;	
	
  aTimeDistr[0]=m_FootHolds[stepnumber].SStime+m_FootHolds[stepnumber].DStime;

  aBoundCond(0)=m_COMBuffer[0].z[0];
  aBoundCond(1)=0.0;
  aBoundCond(2)=m_COMBuffer[0].z[0]+m_FootHolds[stepnumber].DeviationHipHeight;
  aBoundCond(3)=0.0;


  m_PolynomeHip->SetParameters(aBoundCond,aTimeDistr);
  //m_PolynomeHip->print(); 
  m_COMBuffer[u_start].z[0]=m_COMBuffer[0].z[0];//+m_COMBuffer[u_start].z[0];
  m_COMBuffer[u_start].z[1]=0.0;
  m_COMBuffer[u_start].z[2]=0.0;


  for (unsigned int u=1; u<aCOMBuffer.size(); u++) 
    {	
      if ((m_FootHolds[stepnumber].sx>0)|(stepnumber==0))
	{	if ((aZMPPosition[u-1].stepType==11)&(std::fabs((double)aZMPPosition[u].stepType)==1))
	  {
			
	    stepnumber++;
			
	    aTimeDistr[0]=m_FootHolds[stepnumber].SStime+m_FootHolds[stepnumber].DStime;
		
	    u_start=u;
				
	    aBoundCond(0)=m_COMBuffer[u_start].z[0]+m_FootHolds[stepnumber-1].DeviationHipHeight;
	    aBoundCond(1)=0.0;
	    aBoundCond(2)=m_COMBuffer[u_start].z[0]+m_FootHolds[stepnumber].DeviationHipHeight;
	    aBoundCond(3)=0.0;
			
 
		
	    m_PolynomeHip->SetParameters(aBoundCond,aTimeDistr);
	    //m_PolynomeHip->print(); 
	    m_COMBuffer[u_start].z[0]=m_PolynomeHip->Compute(0);//+m_COMBuffer[u_start].z[0];
	    m_COMBuffer[u_start].z[1]=(m_COMBuffer[u_start].z[0]-m_COMBuffer[u_start-1].z[0])/m_SamplingPeriod;
	    m_COMBuffer[u_start].z[2]=(m_COMBuffer[u_start].z[1]-m_COMBuffer[u_start-1].z[1])/m_SamplingPeriod;

	  }
	else
	  {
					
	    double LocalTime;
	    LocalTime=(u-u_start)*m_SamplingPeriod;	

	    m_COMBuffer[u].z[0]=m_PolynomeHip->Compute(LocalTime);//+m_COMBuffer[u_start].z[0];
	    m_COMBuffer[u].z[1]=(m_COMBuffer[u].z[0]-m_COMBuffer[u-1].z[0])/m_SamplingPeriod;
	    m_COMBuffer[u].z[2]=(m_COMBuffer[u].z[1]-m_COMBuffer[u-1].z[1])/m_SamplingPeriod;
			
	  }
	}
      else
	{		
	  m_COMBuffer[u].z[0]=m_COMBuffer[u-1].z[0];//+m_COMBuffer[u_start].z[0];
	  m_COMBuffer[u].z[1]=0.0;
	  m_COMBuffer[u].z[2]=0.0;
	}

    }


	
  aCOMBuffer = m_COMBuffer;

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
	    m_COMBuffer[i].x[0] << " "<< 	
	    m_COMBuffer[i].y[0]<< " " << 
	    m_COMBuffer[i].z[0]<< " " << 
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

void WaistPolynome::SetParameters(VNL::Vector<double> boundCond, vector<double> timeDistr)
{
  VNL::Matrix<double> Base(4,4),Temp(4,4);

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

  detBase=Determinant(Base); 

  // cout << "determinant of Base matrix: " << detBase << endl;

 
  for (unsigned int i=0;i<boundCond.size();i++)
    {
      Temp=Base;
      Temp.SetColumn(i,boundCond);
      m_Coefficients[i] =Determinant(Temp)/detBase;
	
    };

}

WaistPolynome::~WaistPolynome()
{  
}







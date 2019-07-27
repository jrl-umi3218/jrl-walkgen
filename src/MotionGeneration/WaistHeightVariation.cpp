/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Florent Lamiraux
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

/** This object generate all the values for the foot trajectories,
    and the desired ZMP based on a sequence of steps.
*/
#define _DEBUG_

#include <fstream>
#include "Debug.hh"
#include <MotionGeneration/WaistHeightVariation.hh>

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


  unsigned int u_start=0;
  int stepnumber =0;
  Eigen::Matrix<double,4,1> aBoundCond;
  {
    for(unsigned int i=0; i<aBoundCond.size(); aBoundCond[i++]=1);
  };

  vector<double> aTimeDistr;
  aTimeDistr.resize(1);

  m_SamplingPeriod = aZMPPosition[1].time-aZMPPosition[0].time;

  aTimeDistr[0]=aFootHolds[stepnumber].SStime+aFootHolds[stepnumber].DStime;

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
          if ((aZMPPosition[u-1].stepType==11)&
              (std::fabs((double) aZMPPosition[u].stepType)==1))
            {
              ODEBUG("Deviation Hip Height: "
                     << aFootHolds[stepnumber].DeviationHipHeight);
              stepnumber++;
              
              aTimeDistr[0]=
                aFootHolds[stepnumber].SStime+aFootHolds[stepnumber].DStime;

              u_start=u;

              aBoundCond(0)=aCOMBuffer[u_start].z[0]+
                aFootHolds[stepnumber-1].DeviationHipHeight;
              aBoundCond(1)=0.0;
              aBoundCond(2)=aCOMBuffer[u_start].z[0]
                +aFootHolds[stepnumber].DeviationHipHeight;
              aBoundCond(3)=0.0;



              m_PolynomeHip->SetParameters(aBoundCond,aTimeDistr);
              //m_PolynomeHip->print();
              aCOMBuffer[u_start].z[0]=m_PolynomeHip->Compute(0);
              aCOMBuffer[u_start].z[1]=
                (aCOMBuffer[u_start].z[0]-aCOMBuffer[u_start-1].z[0])
                /m_SamplingPeriod;
              aCOMBuffer[u_start].z[2]=
                (aCOMBuffer[u_start].z[1]-
                 aCOMBuffer[u_start-1].z[1])/m_SamplingPeriod;

            }
          else
            {

              double LocalTime;
              LocalTime=(u-u_start)*m_SamplingPeriod;

              aCOMBuffer[u].z[0]=
                m_PolynomeHip->Compute(LocalTime);//+aCOMBuffer[u_start].z[0];
              aCOMBuffer[u].z[1]=
                (aCOMBuffer[u].z[0]-aCOMBuffer[u-1].z[0])/m_SamplingPeriod;
              aCOMBuffer[u].z[2]=
                (aCOMBuffer[u].z[1]-aCOMBuffer[u-1].z[1])/m_SamplingPeriod;

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

  for (unsigned int i=0; i<aCOMBuffer.size(); i++)
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
}



WaistPolynome::WaistPolynome() :Polynome(4)
{
  // SetParameters(boundCond,timeDistr);
}

void WaistPolynome::SetParameters(Eigen::VectorXd boundCond,
                                  vector<double> timeDistr)
{
  Eigen::Matrix<double,4,4> Base;;
  Eigen::Matrix<double,4,4> Temp;;

  double T;
  double Ts2,Ts3;
  // double t1s2,t1s3,t1s4,t1s5,t1s6,t1s7;
  // double t2s2,t2s3,t2s4,t2s5,t2s6,t2s7;



  // t1=timeDistr[0];
  // t2=timeDistr[1];
  // T=timeDistr[2];
  T=timeDistr[0];

  Ts2=T*T;
  Ts3=T*Ts2;


  Base(0,0)=1.0;
  Base(0,1)=0.0;
  Base(0,2)=0.0;
  Base(0,3)=0.0;
  Base(1,0)=0.0;
  Base(1,1)=1.0;
  Base(1,2)=0.0;
  Base(1,3)=0.0;
  Base(2,0)=1.0;
  Base(2,1)=T  ;
  Base(2,2)=Ts2;
  Base(2,3)=Ts3;
  Base(3,0)=0.0;
  Base(3,1)=1.0;
  Base(3,2)=2.0*T;
  Base(3,3)=3.0*Ts2;

  double detBase;


  detBase=Base.determinant();


  for (unsigned int i=0; i<boundCond.size(); i++)
    {
      Temp=Base;
      for(unsigned int j=0; j<Temp.rows(); j++)
        Temp(j,i) = boundCond(j);
      m_Coefficients[i] = Temp.determinant()/detBase;
    };

}

WaistPolynome::~WaistPolynome()
{
}







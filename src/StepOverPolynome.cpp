/* Polynomes object for generating foot and hip trajectories while stepping over.

CVS Information:
$Id: PolynomeFoot.cpp,v 1.2 2006-01-18 06:34:58 stasse Exp $
$Author: stasse $
$Date: 2006-01-18 06:34:58 $
$Revision: 1.2 $
$Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/PolynomeFoot.cpp,v $
$Log: PolynomeFoot.cpp,v $
Revision 1.2  2006-01-18 06:34:58  stasse
OS: Updated the names of the contributors, the documentation
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
#include <iostream>
#include <VNL/Algo/determinant.h>
#include <VNL/Algo/matrixinverse.h>
#include <VNL/matrix.h>
#include <vector>
#include <VNL/vector.h>
#include <StepOverPolynome.h>

using namespace::std;
using namespace::PatternGeneratorJRL;


StepOverPolynomeFoot::StepOverPolynomeFoot() :Polynome(8)
{
  // SetParameters(boundCond,timeDistr);
}

void StepOverPolynomeFoot::SetParameters(VNL::Vector<double> boundCond, vector<double> timeDistr)
{
  VNL::Matrix<double> Base(8,8),Temp(8,8);

  double t1,t2,T;
  double Ts2,Ts3,Ts4,Ts5,Ts6,Ts7;
  double t1s2,t1s3,t1s4,t1s5,t1s6,t1s7;
  double t2s2,t2s3,t2s4,t2s5,t2s6,t2s7;
  
  

  t1=timeDistr[0];
  t2=timeDistr[1];
  T=timeDistr[2];

  Ts2=T*T;Ts3=T*Ts2;Ts4=T*Ts3;Ts5=T*Ts4;Ts6=T*Ts5;Ts7=T*Ts6;
  t1s2=t1*t1;t1s3=t1*t1s2;t1s4=t1*t1s3;t1s5=t1*t1s4;t1s6=t1*t1s5;t1s7=t1*t1s6;
  t2s2=t2*t2;t2s3=t2*t2s2;t2s4=t2*t2s3;t2s5=t2*t2s4;t2s6=t2*t2s5;t2s7=t2*t2s6;
 
  
  Base(0,0)=1.0;Base(0,1)=0.0;Base(0,2)=0.0;Base(0,3)=0.0;Base(0,4)=0.0;Base(0,5)=0.0;Base(0,6)=0.0;Base(0,7)=0.0;
  Base(1,0)=0.0;Base(1,1)=1.0;Base(1,2)=0.0;Base(1,3)=0.0;Base(1,4)=0.0;Base(1,5)=0.0;Base(1,6)=0.0;Base(1,7)=0.0;
  Base(2,0)=0.0;Base(2,1)=0.0;Base(2,2)=2.0;Base(2,3)=0.0;Base(2,4)=0.0;Base(2,5)=0.0;Base(2,6)=0.0;Base(2,7)=0.0;
  
  Base(3,0)=1.0;Base(3,1)=T  ;Base(3,2)=Ts2;Base(3,3)=Ts3;Base(3,4)=Ts4;Base(3,5)=Ts5;Base(3,6)=Ts6;Base(3,7)=Ts7;
  Base(4,0)=0.0;Base(4,1)=1.0;Base(4,2)=2.0*T;Base(4,3)=3.0*Ts2;Base(4,4)=4.0*Ts3;Base(4,5)=5.0*Ts4;Base(4,6)=6.0*Ts5;Base(4,7)=7.0*Ts6;
  Base(5,0)=0.0;Base(5,1)=0.0;Base(5,2)=2.0;Base(5,3)=6.0*T;Base(5,4)=12.0*Ts2;Base(5,5)=20.0*Ts3;Base(5,6)=30.0*Ts4;Base(5,7)=42.0*Ts5;

  Base(6,0)=1.0;Base(6,1)=t1 ;Base(6,2)=t1s2;Base(6,3)=t1s3;Base(6,4)=t1s4;Base(6,5)=t1s5;Base(6,6)=t1s6;Base(6,7)=t1s7;
  Base(7,0)=1.0;Base(7,1)=t2 ;Base(7,2)=t2s2;Base(7,3)=t2s3;Base(7,4)=t2s4;Base(7,5)=t2s5;Base(7,6)=t2s6;Base(7,7)=t2s7;
 
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

StepOverPolynomeFoot::~StepOverPolynomeFoot()
{  
}

//////////////////////////////////////////////////////////////////////////////////////////////////
StepOverPolynomeFootZtoX::StepOverPolynomeFootZtoX() :Polynome(3)
{
  // SetParameters(boundCond,timeDistr);
}

void StepOverPolynomeFootZtoX::SetParameters(VNL::Vector<double> Zpos, vector<double> Xpos)
{
  VNL::Matrix<double> Base(4,4),Temp(4,4);

  double x1,x2,x3;
  double x1s2,x1s3,x2s2,x2s3,x3s2,x3s3;
    
  
  x1 = Xpos[0];
  x2 = Xpos[1];
  x3 = Xpos[2];

  
  x1s2=x1*x1;	x1s3=x1s2*x1;
  x2s2=x2*x2;	x2s3=x2s2*x2;
  x3s2=x3*x3;	x3s3=x3s2*x3;

 
 
  
  Base(0,0)=1.0;	Base(0,1)=0.0;	Base(0,2)=0.0;	Base(0,3)=0.0;
  Base(1,0)=1.0;	Base(1,1)=x1  ;	Base(1,2)=x1s2;	Base(1,3)=x1s3;
  Base(2,0)=1.0;	Base(2,1)=x2  ;	Base(2,2)=x2s2;	Base(2,3)=x2s3;
  Base(3,0)=1.0;	Base(3,1)=x3  ;	Base(3,2)=x3s2;	Base(3,3)=x3s3;

  
  double detBase;

  detBase=Determinant(Base); 

 
  for (unsigned int i=0;i<Zpos.size();i++)
    {
      Temp=Base;
      Temp.SetColumn(i,Zpos);
      m_Coefficients[i] =Determinant(Temp)/detBase;
    };

}

StepOverPolynomeFootZtoX::~StepOverPolynomeFootZtoX()
{  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

StepOverPolynomeFootXtoTime::StepOverPolynomeFootXtoTime() :Polynome(4)
{
  // SetParameters(boundCond,timeDistr);
}

void StepOverPolynomeFootXtoTime::SetParameters(VNL::Vector<double> Xbound, vector<double> timeDistr)
{
  VNL::Matrix<double> Base(5,5),Temp(5,5);

  double t1,t2,T;
  double Ts2,Ts3,Ts4;
  double t1s2,t1s3,t1s4;
  double t2s2,t2s3,t2s4;
  
  

  t1=timeDistr[0];
  t2=timeDistr[1];
  T=timeDistr[2];

  Ts2=T*T;Ts3=T*Ts2;Ts4=T*Ts3;
  t1s2=t1*t1;t1s3=t1*t1s2;t1s4=t1*t1s3;;
  t2s2=t2*t2;t2s3=t2*t2s2;t2s4=t2*t2s3;


 
  
  Base(0,0)=1.0;Base(0,1)=0.0;Base(0,2)=0.0;Base(0,3)=0.0;Base(0,4)=0.0;
  Base(1,0)=1.0;Base(1,1)=t1  ;Base(1,2)=t1s2;Base(1,3)=t1s3;Base(1,4)=t1s4;
  Base(2,0)=1.0;Base(2,1)=t2  ;Base(2,2)=t2s2;Base(2,3)=t2s3;Base(2,4)=t2s4;
  Base(3,0)=1.0;Base(3,1)=T  ;Base(3,2)=Ts2;Base(3,3)=Ts3;Base(3,4)=Ts4;
  Base(4,0)=0.0;Base(4,1)=1.0;Base(4,2)=2.0*T;Base(4,3)=3.0*Ts2;Base(4,4)=4.0*Ts3;
 


 
  double detBase;

  detBase=Determinant(Base); 

  // cout << "determinant of Base matrix: " << detBase << endl;

 
  for (unsigned int i=0;i<Xbound.size();i++)
    {
      Temp=Base;
      Temp.SetColumn(i,Xbound);
      m_Coefficients[i] =Determinant(Temp)/detBase;
    };

}

StepOverPolynomeFootXtoTime::~StepOverPolynomeFootXtoTime()
{  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



StepOverPolynomeHip4::StepOverPolynomeHip4() :Polynome(4)
{
  // SetParameters(boundCond,timeDistr);
}

void StepOverPolynomeHip4::SetParameters(VNL::Vector<double> boundCond, vector<double> timeDistr)
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

StepOverPolynomeHip4::~StepOverPolynomeHip4()
{  
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



StepOverSpline::StepOverSpline() 
{
  // SetParameters(boundCond,timeDistr);
}

void StepOverSpline::SetParameters(VNL::Vector<double> Points)
{
  VNL::Matrix<double> LhSide(4,4),RhSide(4,1),TempSol(4,1);


 
  number = Points.size();
 
  LhSide.Resize(number,number);
  RhSide.Resize(number,1);
  TempSol.Resize(number,1);
  Coefficients.Resize(4,number);

  for (unsigned int i=0;i<number;i++)
    { 
	
      for (unsigned int j=0;j<number;j++)
	LhSide(i,j)=0.0;
    }
	
  for (unsigned int i=1;i<number-1;i++)
    {
      RhSide(i,0)=3.0*(Points(i+1)-Points(i-1));
    }

  RhSide(0,0)=3.0*(Points(1)-Points(0));
  RhSide(number-1,0)=3.0*(Points(number-1)-Points(number-2));
	
  for (unsigned int i=1;i<number;i++)
    { 
      LhSide(i,i)=4.0;
      LhSide(i,i-1)=1.0;
      LhSide(i-1,i)=1.0;
    }
	
  LhSide(0,0) = 2.0;
  LhSide(number-1,number-1) = 2.0;


  TempSol=SVDInverse(LhSide)*RhSide;
	
  for (unsigned int i=0;i<number-1;i++)
    {
      Coefficients(0,i) = Points(i);
      Coefficients(1,i) = TempSol(i,0);
      Coefficients(2,i) = 3.0*(Points(i+1)-Points(i))-2.0*TempSol(i,0)-TempSol(i+1,0);
      Coefficients(3,i) = 2.0*(Points(i)-Points(i+1))+TempSol(i,0)+TempSol(i+1,0);
    }	
  
}


double StepOverSpline::GetValueSpline(VNL::Vector<double> TimePoints, double CurrentLocalTime) // TimePoints has to be defined [t1=O, t2, t3,....,tnumber]
{
  double ValueSpline =0.0;
  unsigned int numberComp;
  numberComp = TimePoints.size();
  if (!(numberComp=number))
    {
      cout << "NCS: number of intervals is not the same as for the calculation of the spline coefficients !" << endl;
      return 0.0;
    }
 
  for (unsigned int i=0;i<number-1;i++)
    {
      if (CurrentLocalTime<TimePoints(i+1))
	{
	  double Time0to1 =0.0;double Time0to1_2 =0.0;double Time0to1_3 =0.0;
	  Time0to1=(CurrentLocalTime-TimePoints(i))/(TimePoints(i+1)-TimePoints(i));
	  Time0to1_2=Time0to1*Time0to1;
	  Time0to1_3=Time0to1_2*Time0to1; 
	  ValueSpline = Coefficients(0,i)
	    +Coefficients(1,i)*Time0to1
	    +Coefficients(2,i)*Time0to1_2
	    +Coefficients(3,i)*Time0to1_3; 
	  break;
	} //if 
 
    } // for
 
  if (CurrentLocalTime>=TimePoints(number-1)) // if the CurrentLocalTime is out of range it will return the end position if t>tmax 
    {
      ValueSpline = Coefficients(0,(number-1))
	+Coefficients(1,(number-1))
	+Coefficients(2,(number-1))
	+Coefficients(3,(number-1));
      // cout << "NCS: timevalue is beyond the range of given time intervals, last position is returned" << endl;
    }
 
  if (CurrentLocalTime<=TimePoints(0)) 
    // if the CurrentLocalTime is out of range it will return the start position if t<tmin
    {
      ValueSpline = Coefficients(0,0);
      //cout << "NCS: timevalue falls before the range of given time intervals, first position is returned" << endl;
    }
 
  return ValueSpline;
}


void StepOverSpline::print()
{
	
  cout << " Spline Coefficients : " << endl;

  for (unsigned int i=0;i<number-1;i++)
    { 

      cout << "		Spline number " << i+1 << ":" << endl << "		";
      for (unsigned int j=0;j<4;j++)
	cout << Coefficients(j,i) << "		" ;
      cout << endl;
    }	
}




StepOverSpline::~StepOverSpline() 
{
  
}


////////////////////////////////////////////////////////////////////////////////////////////


StepOverClampedCubicSpline::StepOverClampedCubicSpline() 
{
  // SetParameters(boundCond,timeDistr);
}
 
void StepOverClampedCubicSpline::SetParameters(VNL::Vector<double> Points,
					       VNL::Vector<double> TimePoints,
					       VNL::Vector<double> DerivativeEndPoints)
{
  VNL::Matrix<double> LhSide(4,4),RhSide(4,1),TempSol(4,1);
  VNL::Vector<double> h(4);
  number = Points.size()-1;
 
  LhSide.Resize(number+1,number+1);
  RhSide.Resize(number+1,1);
  TempSol.Resize(number+1,1);
  Coefficients.Resize(4,number);
  h.Resize(number);

  for (unsigned int i=0;i<number;i++)
    { 
      h(i)=TimePoints(i+1)-TimePoints(i);
    }

  for (unsigned int i=0;i<number+1;i++)
    { 
      for (unsigned int j=0;j<number+1;j++)
	LhSide(i,j)=0.0;
    }

  for (unsigned int i=1;i<number;i++)
    {
      RhSide(i,0)=3.0/h(i)*(Points(i+1)-Points(i))-3.0/h(i-1)*(Points(i)-Points(i-1));
    }

  RhSide(0,0)=3.0/h(0)*(Points(1)-Points(0))-3.0*DerivativeEndPoints(0);
  RhSide(number,0)=-3.0/h(number-1)*(Points(number)-Points(number-1))+3.0*DerivativeEndPoints(1);

  for (unsigned int i=1;i<number;i++)
    { 
      LhSide(i,i)=2.0*(h(i-1)+h(i));
      LhSide(i,i-1)=h(i-1);
      LhSide(i-1,i)=h(i-1);
    }

  LhSide(0,0) = 2.0*h(0);
 
  LhSide(number,number) = 2.0*h(number-1);
  LhSide(number,number-1) = h(number-1);
  LhSide(number-1,number) = h(number-1);
 
  TempSol=SVDInverse(LhSide)*RhSide;

  for (unsigned int i=0;i<number;i++)
    {
      Coefficients(0,i) = Points(i);
      Coefficients(1,i) = (Points(i+1)-Points(i)-(2.0*TempSol(i,0)+TempSol(i+1,0))*h(i)*h(i)/3.0)/h(i);
      Coefficients(2,i) = TempSol(i,0);
      Coefficients(3,i) = (TempSol(i+1,0)-TempSol(i,0))/(3.0*h(i));
    } 
  
}
 


double StepOverClampedCubicSpline::GetValueSpline(VNL::Vector<double> TimePoints, double CurrentLocalTime)
{
  double ValueSpline =0.0;
  unsigned int numberComp;
  numberComp = TimePoints.size();
  if (!(numberComp=number))
    {
      cout << "CCS: number of intervals is not the same as for the calculation of the spline coefficients !" << endl;
      return 0.0;
    }
 
  for (unsigned int i=0;i<number;i++)
    {
      if (CurrentLocalTime<TimePoints(i+1))
	{
	  double Time0to1 =0.0;double Time0to1_2 =0.0;double Time0to1_3 =0.0;
	  Time0to1=(CurrentLocalTime-TimePoints(i));
	  Time0to1_2=Time0to1*Time0to1;
	  Time0to1_3=Time0to1_2*Time0to1; 
	  ValueSpline = Coefficients(0,i)+Coefficients(1,i)*Time0to1+Coefficients(2,i)*Time0to1_2+Coefficients(3,i)*Time0to1_3; 
	  break;
	} //if 
 
    } // for
 
  if (CurrentLocalTime>=TimePoints(number)) 
    // if the CurrentLocalTime is out of range it will return the end position if t>tmax 
    {
      double Time0to1 =0.0;double Time0to1_2 =0.0;double Time0to1_3 =0.0;
      Time0to1=(TimePoints(number)-TimePoints(number-1));
      Time0to1_2=Time0to1*Time0to1;
      Time0to1_3=Time0to1_2*Time0to1; 

      ValueSpline = Coefficients(0,number-1)+Coefficients(1,number-1)*Time0to1
	+Coefficients(2,number-1)*Time0to1_2+Coefficients(3,number-1)*Time0to1_3;
      // cout << "CCS: timevalue is beyond the range of given time intervals, 
      // last position is returned" << endl;
    }
 
  if (CurrentLocalTime<=TimePoints(0)) 
    // if the CurrentLocalTime is out of range it will return the start position if t<tmin
    {
      ValueSpline = Coefficients(0,0);
      // cout << "CCS: timevalue falls before the range of given time intervals, first position is returned" << endl;
    }
 
  return ValueSpline;
}
 

void StepOverClampedCubicSpline::print()
{
 
  cout << "Clamped Cubic Spline Coefficients : " << endl;
 
  for (unsigned int i=0;i<number;i++)
    { 
 
      cout << "  Spline number " << i+1 << ":" << endl << "  ";
      for (unsigned int j=0;j<4;j++)
	cout << Coefficients(j,i) << "  " ;
      cout << endl;
    } 
}
 
 
 

StepOverClampedCubicSpline::~StepOverClampedCubicSpline() 
{
  
}



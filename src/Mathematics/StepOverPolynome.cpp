/*
 * Copyright 2006, 2007, 2008, 2009, 2010, 
 *
 * Olivier    Stasse
 * Bram       Vanderborght
 * Bjorn      Verrelst
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
/* Polynomes object for generating foot and hip trajectories while stepping over. */
#include <iostream>
#include <vector>

#include <jrl/mal/matrixabstractlayer.hh>

#include <Mathematics/StepOverPolynome.hh>

#include <Debug.hh>

using namespace::std;
using namespace::PatternGeneratorJRL;


StepOverPolynomeFoot::StepOverPolynomeFoot() :Polynome(8)
{
  // SetParameters(boundCond,timeDistr);
}

void StepOverPolynomeFoot::SetParameters(MAL_VECTOR(boundCond,double), 
					 vector<double> timeDistr)
{
  MAL_MATRIX_DIM(Base,double,8,8);
  MAL_MATRIX_DIM(Temp,double,8,8);

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

  detBase=MAL_MATRIX_RET_DETERMINANT(Base,double); 

  // cout << "determinant of Base matrix: " << detBase << endl;

 
  for (unsigned int i=0;i<boundCond.size();i++)
    {
      Temp=Base;
      for(unsigned int j=0;j<MAL_MATRIX_NB_ROWS(Temp);j++)
	Temp(j,i) = boundCond(j);
      m_Coefficients[i] = MAL_MATRIX_RET_DETERMINANT(Temp,double)/detBase;
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

void StepOverPolynomeFootZtoX::SetParameters(MAL_VECTOR(Zpos,double),
					     vector<double> Xpos)
{
  MAL_MATRIX_DIM(Base,double,4,4);
  MAL_MATRIX_DIM(Temp,double,4,4);

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

  detBase=MAL_MATRIX_RET_DETERMINANT(Base,double); 
 
  for (unsigned int i=0;i<Zpos.size();i++)
    {
      Temp=Base;
      for(unsigned int j=0;j<Zpos.size();j++)
	Temp(j,i) = Zpos(j);
      m_Coefficients[i] = MAL_MATRIX_RET_DETERMINANT(Temp,double)/detBase;
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

void StepOverPolynomeFootXtoTime::SetParameters(MAL_VECTOR(Xbound,double), 
						vector<double> timeDistr)
{
  MAL_MATRIX_DIM(Base,double,5,5);
  MAL_MATRIX_DIM(Temp,double,5,5);

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

  detBase=MAL_MATRIX_RET_DETERMINANT(Base,double); 

  // cout << "determinant of Base matrix: " << detBase << endl;

  for (unsigned int i=0;i<Xbound.size();i++)
    {
      Temp=Base;
      for (unsigned int j=0;j<Xbound.size();j++)
	Temp(j,i) = Xbound(j);
      m_Coefficients[i] = MAL_MATRIX_RET_DETERMINANT(Temp,double)/detBase;
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

void StepOverPolynomeHip4::SetParameters(MAL_VECTOR(boundCond,double), 
					 vector<double> timeDistr)
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

  // cout << "determinant of Base matrix: " << detBase << endl;

 
  for (unsigned int i=0;i<boundCond.size();i++)
    {
      Temp=Base;
      for(unsigned int j=0;j<MAL_MATRIX_NB_ROWS(Temp);j++)
	Temp(j,i) = boundCond(j);
      m_Coefficients[i] =MAL_MATRIX_RET_DETERMINANT(Temp,double)/detBase;
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

void StepOverSpline::SetParameters(MAL_VECTOR(Points,double))
{
  MAL_MATRIX_DIM(LhSide,double,4,4);
  MAL_MATRIX_DIM(RhSide,double,4,1);
  MAL_MATRIX_DIM(TempSol,double,4,1);
 
  number = Points.size();
 
  MAL_MATRIX_RESIZE(LhSide,number,number);
  MAL_MATRIX_RESIZE(RhSide,number,1);
  MAL_MATRIX_RESIZE(TempSol,number,1);
  MAL_MATRIX_RESIZE(Coefficients,4,number);

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

  MAL_MATRIX(iLhSide, double);
  MAL_INVERSE(LhSide,iLhSide,double);
  MAL_C_eq_A_by_B(TempSol,iLhSide,RhSide);
	
  for (unsigned int i=0;i<number-1;i++)
    {
      Coefficients(0,i) = Points(i);
      Coefficients(1,i) = TempSol(i,0);
      Coefficients(2,i) = 3.0*(Points(i+1)-Points(i))-2.0*TempSol(i,0)-TempSol(i+1,0);
      Coefficients(3,i) = 2.0*(Points(i)-Points(i+1))+TempSol(i,0)+TempSol(i+1,0);
    }	
  
}

// TimePoints has to be defined [t1=O, t2, t3,....,tnumber]
double StepOverSpline::GetValueSpline(MAL_VECTOR(TimePoints,double), 
				      double CurrentLocalTime) 
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
 
  // if the CurrentLocalTime is out of range 
  // it will return the end position if t>tmax 
  if (CurrentLocalTime>=TimePoints(number-1)) 
    {
      ValueSpline = Coefficients(0,(number-1))
	+ Coefficients(1,(number-1))
	+ Coefficients(2,(number-1))
	+ Coefficients(3,(number-1));
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
 
void StepOverClampedCubicSpline::SetParameters(MAL_VECTOR( Points, double),
					       MAL_VECTOR( TimePoints, double),
					       MAL_VECTOR( DerivativeEndPoints,double))
{
  MAL_MATRIX_DIM( LhSide,double,4,4);
  MAL_MATRIX_DIM( RhSide,double,4,1);
  MAL_MATRIX_DIM( TempSol,double,4,1);
  MAL_VECTOR_DIM( h,double,4);
  number = Points.size()-1;
 
  MAL_MATRIX_RESIZE(LhSide,number+1,number+1);
  MAL_MATRIX_RESIZE(RhSide,number+1,1);
  MAL_MATRIX_RESIZE(TempSol,number+1,1);
  MAL_MATRIX_RESIZE(Coefficients,4,number);
  MAL_VECTOR_RESIZE(h,number);

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

  MAL_MATRIX_DIM(iLhSide,double,
		 MAL_MATRIX_NB_ROWS(LhSide),
		 MAL_MATRIX_NB_COLS(LhSide));
  MAL_INVERSE(LhSide,iLhSide,double);
  MAL_C_eq_A_by_B(TempSol,iLhSide,RhSide);

  for (unsigned int i=0;i<number;i++)
    {
      Coefficients(0,i) = Points(i);
      Coefficients(1,i) = (Points(i+1)-Points(i)-(2.0*TempSol(i,0)+TempSol(i+1,0))
			   *h(i)*h(i)/3.0)/h(i);
      Coefficients(2,i) = TempSol(i,0);
      Coefficients(3,i) = (TempSol(i+1,0)-TempSol(i,0))/(3.0*h(i));
    } 
  
}
 


double StepOverClampedCubicSpline::GetValueSpline(MAL_VECTOR( TimePoints,double), double CurrentLocalTime)
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
	  ValueSpline = Coefficients(0,i)+Coefficients(1,i)*Time0to1+
	    Coefficients(2,i)*Time0to1_2+Coefficients(3,i)*Time0to1_3; 
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



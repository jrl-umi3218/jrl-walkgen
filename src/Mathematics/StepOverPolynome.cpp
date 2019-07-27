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
/* Polynomes object for generating foot and hip trajectories 
while stepping over. */
#include <iostream>
#include <vector>


#include <Mathematics/StepOverPolynome.hh>

#include <Debug.hh>

using namespace::std;
using namespace::PatternGeneratorJRL;


StepOverPolynomeFoot::StepOverPolynomeFoot() :Polynome(8)
{
  // SetParameters(boundCond,timeDistr);
}

void StepOverPolynomeFoot::SetParameters(Eigen::VectorXd boundCond,
                                         vector<double> timeDistr)
{
  Eigen::Matrix<double,8,8> Base;;
  Eigen::Matrix<double,8,8> Temp;;

  double t1,t2,T;
  double Ts2,Ts3,Ts4,Ts5,Ts6,Ts7;
  double t1s2,t1s3,t1s4,t1s5,t1s6,t1s7;
  double t2s2,t2s3,t2s4,t2s5,t2s6,t2s7;



  t1=timeDistr[0];
  t2=timeDistr[1];
  T=timeDistr[2];

  Ts2=T*T;
  Ts3=T*Ts2;
  Ts4=T*Ts3;
  Ts5=T*Ts4;
  Ts6=T*Ts5;
  Ts7=T*Ts6;
  t1s2=t1*t1;
  t1s3=t1*t1s2;
  t1s4=t1*t1s3;
  t1s5=t1*t1s4;
  t1s6=t1*t1s5;
  t1s7=t1*t1s6;
  t2s2=t2*t2;
  t2s3=t2*t2s2;
  t2s4=t2*t2s3;
  t2s5=t2*t2s4;
  t2s6=t2*t2s5;
  t2s7=t2*t2s6;


  Base(0,0)=1.0;
  Base(0,1)=0.0;
  Base(0,2)=0.0;
  Base(0,3)=0.0;
  Base(0,4)=0.0;
  Base(0,5)=0.0;
  Base(0,6)=0.0;
  Base(0,7)=0.0;
  Base(1,0)=0.0;
  Base(1,1)=1.0;
  Base(1,2)=0.0;
  Base(1,3)=0.0;
  Base(1,4)=0.0;
  Base(1,5)=0.0;
  Base(1,6)=0.0;
  Base(1,7)=0.0;
  Base(2,0)=0.0;
  Base(2,1)=0.0;
  Base(2,2)=2.0;
  Base(2,3)=0.0;
  Base(2,4)=0.0;
  Base(2,5)=0.0;
  Base(2,6)=0.0;
  Base(2,7)=0.0;

  Base(3,0)=1.0;
  Base(3,1)=T  ;
  Base(3,2)=Ts2;
  Base(3,3)=Ts3;
  Base(3,4)=Ts4;
  Base(3,5)=Ts5;
  Base(3,6)=Ts6;
  Base(3,7)=Ts7;
  Base(4,0)=0.0;
  Base(4,1)=1.0;
  Base(4,2)=2.0*T;
  Base(4,3)=3.0*Ts2;
  Base(4,4)=4.0*Ts3;
  Base(4,5)=5.0*Ts4;
  Base(4,6)=6.0*Ts5;
  Base(4,7)=7.0*Ts6;
  Base(5,0)=0.0;
  Base(5,1)=0.0;
  Base(5,2)=2.0;
  Base(5,3)=6.0*T;
  Base(5,4)=12.0*Ts2;
  Base(5,5)=20.0*Ts3;
  Base(5,6)=30.0*Ts4;
  Base(5,7)=42.0*Ts5;

  Base(6,0)=1.0;
  Base(6,1)=t1 ;
  Base(6,2)=t1s2;
  Base(6,3)=t1s3;
  Base(6,4)=t1s4;
  Base(6,5)=t1s5;
  Base(6,6)=t1s6;
  Base(6,7)=t1s7;
  Base(7,0)=1.0;
  Base(7,1)=t2 ;
  Base(7,2)=t2s2;
  Base(7,3)=t2s3;
  Base(7,4)=t2s4;
  Base(7,5)=t2s5;
  Base(7,6)=t2s6;
  Base(7,7)=t2s7;

  double detBase;

  detBase=Base.determinant();

  ODEBUG("determinant of Base matrix: " << detBase);


  for (unsigned int i=0; i<boundCond.size(); i++)
    {
      Temp=Base;
      for(unsigned int j=0; j<Temp.rows(); j++)
        Temp(j,i) = boundCond(j);
      m_Coefficients[i] = Temp.determinant()/detBase;
    };

}

StepOverPolynomeFoot::~StepOverPolynomeFoot()
{
}

///////////////////////////////////////////////////////////
StepOverPolynomeFootZtoX::StepOverPolynomeFootZtoX() :Polynome(3)
{
  // SetParameters(boundCond,timeDistr);
}

void StepOverPolynomeFootZtoX::SetParameters(Eigen::VectorXd Zpos,
                                             vector<double> Xpos)
{
  Eigen::Matrix<double,4,4> Base;;
  Eigen::Matrix<double,4,4> Temp;;

  double x1,x2,x3;
  double x1s2,x1s3,x2s2,x2s3,x3s2,x3s3;


  x1 = Xpos[0];
  x2 = Xpos[1];
  x3 = Xpos[2];


  x1s2=x1*x1;
  x1s3=x1s2*x1;
  x2s2=x2*x2;
  x2s3=x2s2*x2;
  x3s2=x3*x3;
  x3s3=x3s2*x3;




  Base(0,0)=1.0;
  Base(0,1)=0.0;
  Base(0,2)=0.0;
  Base(0,3)=0.0;
  Base(1,0)=1.0;
  Base(1,1)=x1  ;
  Base(1,2)=x1s2;
  Base(1,3)=x1s3;
  Base(2,0)=1.0;
  Base(2,1)=x2  ;
  Base(2,2)=x2s2;
  Base(2,3)=x2s3;
  Base(3,0)=1.0;
  Base(3,1)=x3  ;
  Base(3,2)=x3s2;
  Base(3,3)=x3s3;


  double detBase;

  detBase=Base.determinant();

  for (unsigned int i=0; i<Zpos.size(); i++)
    {
      Temp=Base;
      for(unsigned int j=0; j<Zpos.size(); j++)
        Temp(j,i) = Zpos(j);
      m_Coefficients[i] = Temp.determinant()/detBase;
    };

}

StepOverPolynomeFootZtoX::~StepOverPolynomeFootZtoX()
{
}

/////////////////////////////////////////////////////////////////////////

StepOverPolynomeFootXtoTime::StepOverPolynomeFootXtoTime() :Polynome(4)
{
  // SetParameters(boundCond,timeDistr);
}

void StepOverPolynomeFootXtoTime::SetParameters(Eigen::VectorXd Xbound,
                                                vector<double> timeDistr)
{
  Eigen::Matrix<double,5,5> Base;;
  Eigen::Matrix<double,5,5> Temp;;

  double t1,t2,T;
  double Ts2,Ts3,Ts4;
  double t1s2,t1s3,t1s4;
  double t2s2,t2s3,t2s4;

  t1=timeDistr[0];
  t2=timeDistr[1];
  T=timeDistr[2];

  Ts2=T*T;
  Ts3=T*Ts2;
  Ts4=T*Ts3;
  t1s2=t1*t1;
  t1s3=t1*t1s2;
  t1s4=t1*t1s3;;
  t2s2=t2*t2;
  t2s3=t2*t2s2;
  t2s4=t2*t2s3;

  Base(0,0)=1.0;
  Base(0,1)=0.0;
  Base(0,2)=0.0;
  Base(0,3)=0.0;
  Base(0,4)=0.0;
  Base(1,0)=1.0;
  Base(1,1)=t1  ;
  Base(1,2)=t1s2;
  Base(1,3)=t1s3;
  Base(1,4)=t1s4;
  Base(2,0)=1.0;
  Base(2,1)=t2  ;
  Base(2,2)=t2s2;
  Base(2,3)=t2s3;
  Base(2,4)=t2s4;
  Base(3,0)=1.0;
  Base(3,1)=T  ;
  Base(3,2)=Ts2;
  Base(3,3)=Ts3;
  Base(3,4)=Ts4;
  Base(4,0)=0.0;
  Base(4,1)=1.0;
  Base(4,2)=2.0*T;
  Base(4,3)=3.0*Ts2;
  Base(4,4)=4.0*Ts3;

  double detBase;

  detBase=Base.determinant();

  ODEBUG( "determinant of Base matrix: " << detBase );

  for (unsigned int i=0; i<Xbound.size(); i++)
    {
      Temp=Base;
      for (unsigned int j=0; j<Xbound.size(); j++)
        Temp(j,i) = Xbound(j);
      m_Coefficients[i] = Temp.determinant()/detBase;
    };

}

StepOverPolynomeFootXtoTime::~StepOverPolynomeFootXtoTime()
{
}

//////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////



StepOverPolynomeHip4::StepOverPolynomeHip4() :Polynome(4)
{
  // SetParameters(boundCond,timeDistr);
}

void StepOverPolynomeHip4::SetParameters(Eigen::VectorXd boundCond,
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

  // cout << "determinant of Base matrix: " << detBase << endl;


  for (unsigned int i=0; i<boundCond.size(); i++)
    {
      Temp=Base;
      for(unsigned int j=0; j<Temp.rows(); j++)
        Temp(j,i) = boundCond(j);
      m_Coefficients[i] =Temp.determinant()/detBase;
    };

}

StepOverPolynomeHip4::~StepOverPolynomeHip4()
{
}

//////////////////////////////////////////////////////


StepOverSpline::StepOverSpline()
{
  // SetParameters(boundCond,timeDistr);
}

void StepOverSpline::SetParameters(Eigen::VectorXd Points)
{
  Eigen::Matrix<double,4,4> LhSide;;
  Eigen::Matrix<double,4,1> RhSide;;
  Eigen::Matrix<double,4,1> TempSol;;

  m_number = Points.size();

  LhSide.resize(m_number,m_number);
  RhSide.resize(m_number,1);
  TempSol.resize(m_number,1);
  m_Coefficients.resize(4,m_number);

  for (unsigned int i=0; i<m_number; i++)
    {
      for (unsigned int j=0; j<m_number; j++)
        LhSide(i,j)=0.0;
    }

  for (unsigned int i=1; i<m_number-1; i++)
    {
      RhSide(i,0)=3.0*(Points(i+1)-Points(i-1));
    }

  RhSide(0,0)=3.0*(Points(1)-Points(0));
  RhSide(m_number-1,0)=3.0*(Points(m_number-1)-Points(m_number-2));

  for (unsigned int i=1; i<m_number; i++)
    {
      LhSide(i,i)=4.0;
      LhSide(i,i-1)=1.0;
      LhSide(i-1,i)=1.0;
    }

  LhSide(0,0) = 2.0;
  LhSide(m_number-1,m_number-1) = 2.0;

  Eigen::MatrixXd iLhSide;
  iLhSide=LhSide.inverse();
  TempSol=iLhSide+RhSide;

  for (unsigned int i=0; i<m_number-1; i++)
    {
      m_Coefficients(0,i) = Points(i);
      m_Coefficients(1,i) = TempSol(i,0);
      m_Coefficients(2,i) = 3.0*(Points(i+1)-Points(i))-
        2.0*TempSol(i,0)-TempSol(i+1, 0);
      m_Coefficients(3,i) = 2.0*(Points(i)-Points(i+1))+
        TempSol(i,0)+TempSol(i+1,0);
    }

}

// TimePoints has to be defined [t1=O, t2, t3,....,tm_number]
double StepOverSpline::GetValueSpline(Eigen::VectorXd TimePoints,
                                      double CurrentLocalTime)
{
  double ValueSpline =0.0;
  long unsigned int numberComp;
  numberComp = TimePoints.size();
  if (!(numberComp=m_number))
    {
      std::cerr << "NCS: number of intervals is not the same as ";
      std::cerr << "for the calculation of the spline coefficients !" << endl;
      return 0.0;
    }

  for (unsigned int i=0; i<m_number-1; i++)
    {
      if (CurrentLocalTime<TimePoints(i+1))
        {
          double Time0to1 =0.0;
          double Time0to1_2 =0.0;
          double Time0to1_3 =0.0;
          Time0to1=(CurrentLocalTime-TimePoints(i))/
            (TimePoints(i+1)-TimePoints(i));
          Time0to1_2=Time0to1*Time0to1;
          Time0to1_3=Time0to1_2*Time0to1;
          ValueSpline = m_Coefficients(0,i)
            +m_Coefficients(1,i)*Time0to1
            +m_Coefficients(2,i)*Time0to1_2
            +m_Coefficients(3,i)*Time0to1_3;
          break;
        } //if

    } // for

  // if the CurrentLocalTime is out of range
  // it will return the end position if t>tmax
  if (CurrentLocalTime>=TimePoints(m_number-1))
    {
      ValueSpline = m_Coefficients(0,(m_number-1))
        + m_Coefficients(1,(m_number-1))
        + m_Coefficients(2,(m_number-1))
        + m_Coefficients(3,(m_number-1));
      std::cerr
        << "NCS: timevalue is beyond the range of given time intervals,"
        << "last position is returned" << endl;
    }

  if (CurrentLocalTime<=TimePoints(0))
    // if the CurrentLocalTime is out of range it will return
    // the start position if t<tmin
    {
      ValueSpline = m_Coefficients(0,0);
      std::cerr << "NCS: timevalue falls before the range of given"
                << " time intervals, first position is returned" << endl;
    }

  return ValueSpline;
}


void StepOverSpline::print()
{

  cout << " Spline Coefficients : " << endl;

  for (unsigned int i=0; i<m_number-1; i++)
    {

      cout << "             Spline m_number " << i+1 << ":"
           << endl << "            ";
      for (unsigned int j=0; j<4; j++)
        cout << m_Coefficients(j,i) << "          " ;
      cout << endl;
    }
}




StepOverSpline::~StepOverSpline()
{

}


///////////////////////////////////////////////////////////////////////////


StepOverClampedCubicSpline::StepOverClampedCubicSpline()
{
  // SetParameters(boundCond,timeDistr);
}
void StepOverClampedCubicSpline::
SetParameters
(Eigen::VectorXd Points,
 Eigen::VectorXd TimePoints,
 Eigen::VectorXd DerivativeEndPoints)
{
  Eigen::Matrix<double,4,4> LhSide;;
  Eigen::Matrix<double,4,1> RhSide;;
  Eigen::Matrix<double,4,1> TempSol;;
  Eigen::Matrix<double,4,1> h;
  m_number = Points.size()-1;

  LhSide.resize(m_number+1,m_number+1);
  RhSide.resize(m_number+1,1);
  TempSol.resize(m_number+1,1);
  m_Coefficients.resize(4,m_number);
  h.resize(m_number);

  for (unsigned int i=0; i<m_number; i++)
    {
      h(i)=TimePoints(i+1)-TimePoints(i);
    }

  for (unsigned int i=0; i<m_number+1; i++)
    {
      for (unsigned int j=0; j<m_number+1; j++)
        LhSide(i,j)=0.0;
    }

  for (unsigned int i=1; i<m_number; i++)
    {
      RhSide(i,0)=3.0/h(i)*(Points(i+1)-Points(i))-3.0/
        h(i-1)*(Points(i)-Points(i-1));
    }

  RhSide(0,0)=3.0/h(0)*(Points(1)-Points(0))-3.0*DerivativeEndPoints(0);
  RhSide(m_number,0)=-3.0/h(m_number-1)*
    (Points(m_number)-Points(m_number-1))+3.0*DerivativeEndPoints(1);

  for (unsigned int i=1; i<m_number; i++)
    {
      LhSide(i,i)=2.0*(h(i-1)+h(i));
      LhSide(i,i-1)=h(i-1);
      LhSide(i-1,i)=h(i-1);
    }

  LhSide(0,0) = 2.0*h(0);

  LhSide(m_number,m_number) = 2.0*h(m_number-1);
  LhSide(m_number,m_number-1) = h(m_number-1);
  LhSide(m_number-1,m_number) = h(m_number-1);

  Eigen::MatrixXd iLhSide( LhSide.rows(),LhSide.cols());
  iLhSide=LhSide.inverse();
  TempSol=iLhSide+RhSide;

  for (unsigned int i=0; i<m_number; i++)
    {
      m_Coefficients(0,i) = Points(i);
      m_Coefficients(1,i) = (Points(i+1)-Points(i)-
                             (2.0*TempSol(i,0)+TempSol(i+1,0))
                             *h(i)*h(i)/3.0)/h(i);
      m_Coefficients(2,i) = TempSol(i,0);
      m_Coefficients(3,i) = (TempSol(i+1,0)-TempSol(i,0))/(3.0*h(i));
    }

}


double StepOverClampedCubicSpline::
GetValueSpline
(Eigen::VectorXd TimePoints, double CurrentLocalTime)
{
  double ValueSpline =0.0;
  long unsigned int numberComp;
  numberComp = TimePoints.size();
  if (!(numberComp=m_number))
    {
      cout << "CCS: number of intervals is not the same "
           << "as for the calculation of the spline coefficients !"
           << endl;
      return 0.0;
    }

  for (unsigned int i=0; i<m_number; i++)
    {
      if (CurrentLocalTime<TimePoints(i+1))
        {
          double Time0to1 =0.0;
          double Time0to1_2 =0.0;
          double Time0to1_3 =0.0;
          Time0to1=(CurrentLocalTime-TimePoints(i));
          Time0to1_2=Time0to1*Time0to1;
          Time0to1_3=Time0to1_2*Time0to1;
          ValueSpline = m_Coefficients(0,i)+m_Coefficients(1,i)*Time0to1+
            m_Coefficients(2,i)*Time0to1_2+m_Coefficients(3,i)*Time0to1_3;
          break;
        } //if

    } // for

  if (CurrentLocalTime>=TimePoints(m_number))
    // if the CurrentLocalTime is out of range it will return
    //the end position if t>tmax
    {
      double Time0to1 =0.0;
      double Time0to1_2 =0.0;
      double Time0to1_3 =0.0;
      Time0to1=(TimePoints(m_number)-TimePoints(m_number-1));
      Time0to1_2=Time0to1*Time0to1;
      Time0to1_3=Time0to1_2*Time0to1;

      ValueSpline = m_Coefficients(0,m_number-1)+m_Coefficients(1,m_number-1)*
        Time0to1 +  m_Coefficients(2,m_number-1)*
        Time0to1_2+m_Coefficients(3,  m_number-1)*Time0to1_3;
      // cout << "CCS: timevalue is beyond the range of given time intervals,
      // last position is returned" << endl;
    }

  if (CurrentLocalTime<=TimePoints(0))
    // if the CurrentLocalTime is out of range it will
    // return the start position if t<tmin
    {
      ValueSpline = m_Coefficients(0,0);
      // cout << "CCS: timevalue falls before the range
      // of given time intervals, first position is returned" << endl;
    }

  return ValueSpline;
}


void StepOverClampedCubicSpline::print()
{

  cout << "Clamped Cubic Spline m_Coefficients : " << endl;

  for (unsigned int i=0; i<m_number; i++)
    {

      cout << "  Spline m_number " << i+1 << ":" << endl << "  ";
      for (unsigned int j=0; j<4; j++)
        cout << m_Coefficients(j,i) << "  " ;
      cout << endl;
    }
}




StepOverClampedCubicSpline::~StepOverClampedCubicSpline()
{

}

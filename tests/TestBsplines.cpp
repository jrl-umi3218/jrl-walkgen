/*
 * Copyright 2009, 2010, 2014
 *
 * 
 * Olivier  Stasse, Huynh Ngoc Duc
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
/*! \file TestBsplines.cpp
  \brief This Example shows you how to use Bsplines to create a foot trajectory on Z . */
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "Mathematics/Bsplines.hh"

using namespace std;

int main()
{
    PatternGeneratorJRL::BSplinesFoot *X;
    PatternGeneratorJRL::BSplinesFoot *Y;
    PatternGeneratorJRL::BSplinesFoot *Z;

    double tx=0.0;
    double ty=0.0;
    double tz=0.0;

    ofstream myfile;
    myfile.open("TestBsplines.txt");

    //Create the parameters of foot trajectory on X
    double m_FTx = 1.0;
    double m_FPx = 0.209 ;
    double m_IPx = 0.009 ;
    vector<double> m_MPx ;
    vector<double> m_ToMPx ;
    m_MPx.clear();
    m_ToMPx.clear();

    //Create an object for test X
    X = new PatternGeneratorJRL::BSplinesFoot(m_FTx, m_IPx, m_FPx, m_ToMPx, m_MPx);
//    vector<double> cpx ;
//    cpx.push_back(0.2);
//    cpx.push_back(0.2);
//    cpx.push_back(0.2);
//    cpx.push_back(m_FPx);
//    cpx.push_back(m_FPx);
//    cpx.push_back(m_FPx);
//    X->SetControlPoints(cpx);
    double x,dx,ddx;

    X->PrintDegree();
    X->PrintKnotVector();
    X->PrintControlPoints();

    //Create the parameters of foot trajectory on Y
    double m_FTy = 1.0;
    double m_FPy = 0.2;
    double m_IPy = 0.1;
    vector<double> m_MPy = vector<double>(1,5);
    vector<double> m_ToMPy = vector<double>(1,m_FTx/2.0);

    //Create an object for test Y
    Y = new PatternGeneratorJRL::BSplinesFoot(m_FTy, m_IPy, m_FPy, m_ToMPy, m_MPy);
    Y->PrintDegree();
    Y->PrintKnotVector();
    Y->PrintControlPoints();
    double y,dy,ddy;

    //Create the parameters of foot trajectory on Z
    double m_FTz = 1.0;
    double m_FPz = 0.1;
    double m_IPz = 0.2;
    vector<double> m_MPz ;
    m_MPz.push_back(0.35);
    m_MPz.push_back(0.25);
    vector<double> m_ToMPz = vector<double>(2,m_FTx/3.0);
    m_ToMPz[1]=2*m_FTx/3.0;

    //Create an object for test Y
    Z = new PatternGeneratorJRL::BSplinesFoot(m_FTz, m_IPz, m_FPz, m_ToMPz, m_MPz);
    Z->PrintDegree();
    Z->PrintKnotVector();
    Z->PrintControlPoints();
    double z,dz,ddz;

    double dx_(0.0),dy_(0.0),dz_(0.0),ddx_(0.0),ddy_(0.0),ddz_(0.0);
    double x_p(m_IPx),y_p(m_IPy),z_p(m_IPz),dx_p(0.0),dy_p(0.0),dz_p(0.0);
    double dt = X->FT()/1000.0;
    for (int k=0; k<=1000;k++)
    {
        tx=double(k)*X->FT()/1000.0;
        ty=double(k)*X->FT()/1000.0;
        tz=double(k)*X->FT()/1000.0;

        //cout << k << endl;
        //myfile << t << " " << X->ZComputePosition(t) << " " << X->ZComputeVelocity(t)<< " " << X->ZComputeAcc(t)<< endl;
        X->Compute(tx,x,dx,ddx);
        Y->Compute(ty,y,dy,ddy);
        Z->Compute(tz,z,dz,ddz);

        dx_ = (x - x_p) / dt ;
        ddx_ = (dx - dx_p) / dt ;

        dy_ = (y - y_p) / dt ;
        ddy_ = (dy - dy_p) / dt ;

        dz_ = (z - z_p) / dt ;
        ddz_ = (dz - dz_p) / dt ;

        x_p  = x  ;
        y_p  = y  ;
        z_p  = z  ;
        dx_p = dx ;
        dy_p = dy ;
        dz_p = dz ;

        myfile << tx << " " << x << " " << dx << " " << ddx
                     << " " << y << " " << dy << " " << ddy
                     << " " << z << " " << dz << " " << ddz
                     << " " << dx_ << " " << ddx_
                     << " " << dy_ << " " << ddy_
                     << " " << dz_ << " " << ddz_
                     << endl;

        // time - Position - Velocity - Acceleration
        //cout <<  t  << " " << Z->ZComputePosition(t)<<" "<< Z->ZComputeVelocity(t)<< " "<< Z->ZComputeAcc(t)<< endl;
    }
    myfile.close();

    myfile.open("control_point.txt");
    for (int k=0; k<X->GetControlPoints().size();k++)
    {
      myfile << X->GetControlPoints()[k] << " " ;
      myfile << Y->GetControlPoints()[k] << " " ;
      myfile << Z->GetControlPoints()[k] << " " ;
      myfile << endl ;
    }
    myfile.close();

    delete X;
    delete Y;
    delete Z;

    //draw a foot trajectory with the data given from bsplines	
    myfile.open("DrawTestBsplines.gnu");
    myfile << "set term wxt 0" << endl;
    myfile << "plot 'control_point.txt' u 0:1 with points, 'TestBsplines.txt' using 1:2 with lines title 'PosX'"<< endl;
    myfile << "set term wxt 1" << endl;
    myfile << "plot 'TestBsplines.txt' using 1:2 with lines title 'PosX', 'TestBsplines.txt' using 1:3 with lines title 'SpeedX','TestBsplines.txt' using 1:4 with lines title 'AccX'"<< endl;

    myfile << "set term wxt 2" << endl;
    myfile << "plot 'control_point.txt' with points, 'TestBsplines.txt' using 1:5 with lines title 'PosY'"<< endl;
    myfile << "set term wxt 3" << endl;
    myfile << "plot 'TestBsplines.txt' using 1:5 with lines title 'PosY', 'TestBsplines.txt' using 1:6 with lines title 'SpeedY','TestBsplines.txt' using 1:7 with lines title 'AccY'"<< endl;

    myfile << "set term wxt 2" << endl;
    myfile << "plot 'control_point.txt' with points, 'TestBsplines.txt' using 1:8 with lines title 'PosZ'"<< endl;
    myfile << "set term wxt 3" << endl;
    myfile << "plot 'TestBsplines.txt' using 1:8 with lines title 'PosY', 'TestBsplines.txt' using 1:9 with lines title 'SpeedY','TestBsplines.txt' using 1:10 with lines title 'AccY'"<< endl;

    myfile << "set term wxt 4" << endl;
    myfile << "plot 'control_point.txt' with points, 'TestBsplines.txt' using 2:5 with lines title 'PosXY'"<< endl;


    myfile.close();
    
    return 0;
}



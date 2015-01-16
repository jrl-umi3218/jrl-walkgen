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
    double m_FTx = 0.7;
    double m_FPx = 0.54;
    vector<double> m_MPx = vector<double>(1,0.54);
    vector<double> m_ToMPx = vector<double>(1,0.6);

    //Create the parameters of foot trajectory on Y
    double m_FTy = 0.7;
    double m_FPy = 0;
    vector<double> m_MPy = vector<double>(1,0.2);
    vector<double> m_ToMPy = vector<double>(1,m_FTx/2.0);

    //Create the parameters of foot trajectory on Z
    double m_FTz = 0.7;
    double m_FPz = 0.0;
    vector<double> m_MPz = vector<double>(1,0.07);
    vector<double> m_ToMPz = vector<double>(1,m_FTx/2.0);

    //Create an object for test X
    X = new PatternGeneratorJRL::BSplinesFoot(m_FTx, m_FPx, m_ToMPx, m_MPx);
    X->PrintDegree();
    X->PrintKnotVector();
    X->PrintControlPoints();

    //Create an object for test Y
    Y = new PatternGeneratorJRL::BSplinesFoot(m_FTy, m_FPy, m_ToMPy, m_MPy);
    Y->PrintDegree();
    Y->PrintKnotVector();
    Y->PrintControlPoints();

    //Create an object for test Y
    Z = new PatternGeneratorJRL::BSplinesFoot(m_FTz, m_FPz, m_ToMPz, m_MPz);
    Z->PrintDegree();
    Z->PrintKnotVector();
    Z->PrintControlPoints();

    for (int k=1; k<1000;k++)
    {

        tx=double(k)*X->GetKnotVector().back()/1000.0;
        ty=double(k)*Y->GetKnotVector().back()/1000.0;
        tz=double(k)*Z->GetKnotVector().back()/1000.0;

        //cout << k << endl;
        //myfile << t << " " << X->ZComputePosition(t) << " " << X->ZComputeVelocity(t)<< " " << X->ZComputeAcc(t)<< endl;
        myfile << tx << " " << X->Compute(tx) << " " << X->ComputeDerivative(tx)<< " " << X->ComputeSecDerivative(tx)
                     << " " << Y->Compute(ty) << " " << Y->ComputeDerivative(ty)<< " " << Y->ComputeSecDerivative(ty)
                     << " " << Y->Compute(tz) << " " << Z->ComputeDerivative(tz)<< " " << Z->ComputeSecDerivative(tz)
                     << endl;

        // time - Position - Velocity - Acceleration
        //cout <<  t  << " " << Z->ZComputePosition(t)<<" "<< Z->ZComputeVelocity(t)<< " "<< Z->ZComputeAcc(t)<< endl;
    }
    myfile.close();
    delete X;
    delete Y;

    //draw a foot trajectory with the data given from bsplines	
    myfile.open("DrawTestBsplines.gnu");
    myfile << "set term wxt 0" << endl;
    myfile << "plot 'control_point.txt' with points, 'TestBsplines.txt' using 1:2 with lines title 'PosX'"<< endl;
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



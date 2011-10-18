/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010, 
 *
 * Andrei Herdt
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
#include <privatepgtypes.h>

#include <iostream>
#include <fstream>

namespace PatternGeneratorJRL
{

  struct support_state_t & support_state_t::operator =(const support_state_t & aSS)
  {

    Phase  = aSS.Phase;
    Foot  = aSS.Foot;
    NbStepsLeft  = aSS.NbStepsLeft;
    TimeLimit = aSS.TimeLimit;
    StepNumber  = aSS.StepNumber;
    StateChanged = aSS.StateChanged;
    StartTime = aSS.StartTime;
    X = aSS.X;
    Y = aSS.Y;
    Yaw = aSS.Yaw;

    return *this;

  }


  void support_state_t::reset()
  {

    Phase = DS;
    Foot = LEFT;
    NbStepsLeft  = 0;
    TimeLimit = 0.0;
    StepNumber  = 0;
    StateChanged = false;
    NbInstants = 0;
    StartTime = 0.0;
    X = 0.0;
    Y = 0.0;
    Yaw = 0.0;

  }


  support_state_t::support_state_t()
  {

    reset();

  }


  struct com_t & com_t::operator=(const com_t &aCS)
  {

    for(unsigned int i=0;i<3;i++)
      {
	x[i] = aCS.x[i];
	y[i] = aCS.y[i];
	z[i] = aCS.z[i];
      };
    return *this;

  }
      

  void com_t::reset()
  {

    x.resize(3,false);
    y.resize(3,false);
    z.resize(3,false);
    x.clear();
    y.clear();
    z.clear();

  }


  com_t::com_t()
  {

    reset();

  }


  struct trunk_t & trunk_t::operator=(const trunk_t &aTS)
  {

    for(unsigned int i=0;i<3;i++)
      {
        x[i] = aTS.x[i];
        y[i] = aTS.y[i];
        z[i] = aTS.z[i];

        yaw[i] = aTS.yaw[i];
        pitch[i] = aTS.pitch[i];
        roll[i] = aTS.roll[i];
      };
    return *this;

  }


  void trunk_t::reset()
  {

    x.resize(3,false);
    y.resize(3,false);
    z.resize(3,false);
    yaw.resize(3,false);
    pitch.resize(3,false);
    roll.resize(3,false);
    x.clear();
    y.clear();
    z.clear();
    yaw.clear();
    pitch.clear();
    roll.clear();

  }

  trunk_t::trunk_t()
  {

    reset();

  }


  void
  convex_hull_t::rotate( double Angle )
  {

    double XOld, YOld;
    for( unsigned int j=0; j<X.size(); j++ )
      {
        XOld = X[j];
        YOld = Y[j];
        X[j] = ( XOld*cos(Angle) - YOld*sin(Angle) );
        Y[j] = ( XOld*sin(Angle) + YOld*cos(Angle) );
      }

  }


  convex_hull_t::convex_hull_t( int Size )
  {

    resize(Size);
    reset();

  }


  convex_hull_t::convex_hull_t()
  {

  }


  void
  convex_hull_t::reset()
  {

    X.clear();
    Y.clear();

  }


  void
  convex_hull_t::resize( int Size )
  {

    X.resize(Size);
    Y.resize(Size);

  }


  void
  convex_hull_t::set( const double * Xa, const double * Ya )
  {

    for(unsigned i=0; i<X.size(); i++)
      {
        X[i] = Xa[i];
        Y[i] = Ya[i];
      }

  }


  void
  linear_inequality_t::clear()
  {

    D.x.clear();
    D.y.clear();
    dc.clear();

  }


  void
  linear_inequality_t::resize( int NbRows, int NbCols, bool Preserve )
  {

    D.x.resize(NbRows, NbCols, Preserve);
    D.y.resize(NbRows, NbCols, Preserve);
    dc.resize(NbRows, Preserve);

  }


  solution_t::solution_t():
      NbVariables(0),NbConstraints(0),Fail(0),Print(0),
      Solution_vec(0),initialSolution(1),initialConstraint(1),SupportOrientations_deq(0),SupportStates_deq(0),
      ConstrLagr_vec(0),LBoundsLagr_vec(0),UBoundsLagr_vec(0)
  {  }


  void
  solution_t::reset()
  {

    NbVariables =       0;
    NbConstraints =     0;
    Fail =              0;
    Print =             0;

    Solution_vec.resize                 (0,false);
    SupportOrientations_deq.resize      (0);
    TrunkOrientations_deq.resize        (0);
    SupportStates_deq.resize            (0);
    ConstrLagr_vec.resize               (0,false);
    LBoundsLagr_vec.resize              (0,false);
    UBoundsLagr_vec.resize              (0,false);

  }


  void
  solution_t::resize( unsigned int SizeSolution, unsigned int SizeConstraints )
  {
    NbVariables = SizeSolution;
    NbConstraints = SizeConstraints;

    Solution_vec.resize(SizeSolution, false);
    ConstrLagr_vec.resize(SizeConstraints, false);
    LBoundsLagr_vec.resize(SizeSolution, false);
    UBoundsLagr_vec.resize(SizeSolution, false);
  }


  void
  solution_t::dump( const char * FileName )
  {
    std::ofstream aof;
    aof.open(FileName,std::ofstream::out);
    print(aof);
    aof.close();
  }


  void
  solution_t::print( std::ostream & aos )
  {
    aos << "Arrays:" << std::endl
        << "Solution: ";
    for( unsigned int i = 0; i < NbVariables; i++ )
      {aos<<Solution_vec[i]<<" ";}; aos<<std::endl;
  }

  reference_t::reference_t():Global(), Local()
    {    }

  reference_t::reference_t(const reference_t & R):Global(R.Global), Local(R.Local)
    {    }

  reference_t::frame_t::frame_t(): X(0), Y(0), Yaw(0), X_vec(), Y_vec(), Yaw_vec()
    {   }

  reference_t::frame_t::frame_t(const frame_t & F):
      X(F.X), Y(F.X), Yaw(F.Yaw), X_vec(F.X_vec), Y_vec(F.Y_vec), Yaw_vec(F.Yaw_vec)
  {}
}

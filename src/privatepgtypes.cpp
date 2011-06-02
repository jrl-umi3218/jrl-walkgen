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

namespace PatternGeneratorJRL
{

  struct support_state_s & support_state_t::operator =(const support_state_s & aSS)
  {

    Phase  = aSS.Phase;
    Foot  = aSS.Foot;
    StepsLeft  = aSS.StepsLeft;
    TimeLimit = aSS.TimeLimit;
    StepNumber  = aSS.StepNumber;
    StateChanged = aSS.StateChanged;
    StartTime = aSS.StartTime;
    x = aSS.x;
    y = aSS.y;
    yaw = aSS.yaw;

    return *this;

  }


  void support_state_t::reset()
  {

    Phase  = 0;
    Foot  = 0;
    StepsLeft  = 0;
    TimeLimit = 0.0;
    StepNumber  = 0;
    StateChanged = false;
    StartTime = 0.0;
    x = 0.0;
    y = 0.0;
    yaw = 0.0;

  }


  support_state_s::support_state_s()
  {

    reset();

  }


  struct com_s & com_t::operator=(const com_s &aCS)
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


  com_s::com_s()
  {

    reset();

  }


  struct trunk_s & trunk_t::operator=(const trunk_s &aTS)
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

  trunk_s::trunk_s()
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


  convex_hull_s::convex_hull_s( int Size )
  {

    resize(Size);
    reset();

  }


  convex_hull_s::convex_hull_s()
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

}

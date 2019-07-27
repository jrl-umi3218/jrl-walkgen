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
#include <privatepgtypes.hh>

#include <iostream>
#include <fstream>

namespace PatternGeneratorJRL
{

  struct support_state_t &
  support_state_t::operator
  =(const support_state_t & aSS)
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

    for(unsigned int i=0; i<3; i++)
      {
        x[i] = aCS.x[i];
        y[i] = aCS.y[i];
        z[i] = aCS.z[i];
      };
    return *this;

  }


  void com_t::reset()
  {

    x.resize(3);
    y.resize(3);
    z.resize(3);
    x.setZero();
    y.setZero();
    z.setZero();

  }


  com_t::com_t()
  {

    reset();

  }


  struct trunk_t & trunk_t::operator=(const trunk_t &aTS)
  {

    for(unsigned int i=0; i<3; i++)
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

    x.resize(3);
    y.resize(3);
    z.resize(3);
    yaw.resize(3);
    pitch.resize(3);
    roll.resize(3);
    x.setZero();
    y.setZero();
    z.setZero();
    yaw.setZero();
    pitch.setZero();
    roll.setZero();

  }

  trunk_t::trunk_t()
  {

    reset();

  }


  void
  convex_hull_t::rotate( axis_e axis, double angle )
  {

    switch(axis)
      {
      case YAW:
        double xOld, yOld;
        for( unsigned int j=0; j<X_vec.size(); j++ )
          {
            xOld = X_vec[j];
            yOld = Y_vec[j];
            X_vec[j] = ( xOld*cos(angle) - yOld*sin(angle) );
            Y_vec[j] = ( xOld*sin(angle) + yOld*cos(angle) );
          }
        break;
      case PITCH:
        break;
      case ROLL:
        break;
      case X_AXIS:
        break;
      case  Y_AXIS:
        break;
      case Z_AXIS:
        break;
      }

  }


  convex_hull_t::convex_hull_t( unsigned nbVert, unsigned nbIneq ):
    nbIneq_(0), nbVert_(0)
  {
    clear();
    resize( nbVert, nbIneq );
  }


  void
  convex_hull_t::clear()
  {

    X_vec.clear();
    Y_vec.clear();
    Z_vec.clear();
    A_vec.clear();
    B_vec.clear();
    C_vec.clear();
    D_vec.clear();

  }


  void
  convex_hull_t::resize( unsigned nbVert, unsigned nbIneq )
  {

    X_vec.resize(nbVert);
    Y_vec.resize(nbVert);
    Z_vec.resize(nbVert);
    A_vec.resize(nbIneq);
    B_vec.resize(nbIneq);
    C_vec.resize(nbIneq);
    D_vec.resize(nbIneq);

    nbVert_ = nbVert;
    nbIneq_ = nbIneq;

  }


  void
  convex_hull_t::
  set_vertices
  ( const double * X_a, const double * Y_a, const double * Z_a )
  {

    for(unsigned i=0; i<nbVert_; i++)
      {
        X_vec[i] = X_a[i];
        Y_vec[i] = Y_a[i];
        Z_vec[i] = Z_a[i];
      }

  }
  void
  convex_hull_t::
  set_vertices( const double * X_a, const double * Y_a )
  {

    for(unsigned i=0; i<nbVert_; i++)
      {
        X_vec[i] = X_a[i];
        Y_vec[i] = Y_a[i];
      }

  }
  void
  convex_hull_t::
  set_inequalities
  ( const double * A_a,
    const double * B_a,
    const double * C_a,
    const double * D_a )
  {

    for(unsigned i=0; i<nbIneq_; i++)
      {
        A_vec[i] = A_a[i];
        B_vec[i] = B_a[i];
        C_vec[i] = C_a[i];
        D_vec[i] = D_a[i];
      }

  }

  void
  convex_hull_t::cout()
  {

    std::cout<<"Vertices: "<<nbVert_<<std::endl;
    for(unsigned i=0; i<nbVert_; ++i)
      std::cout<<X_vec[i]<<" "<<Y_vec[i]<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Inequalities: "<<nbIneq_<<std::endl;
    for(unsigned i=0; i<nbIneq_; ++i)
      std::cout << A_vec[i] << " "
                << B_vec[i] << " "
                << C_vec[i] << " "
                << D_vec[i] << std::endl;
    std::cout<<std::endl;

  }


  void
  linear_inequality_t::clear()
  {

    D.X_mat.setZero();
    D.Y_mat.setZero();
    D.Z_mat.setZero();
    Dc_vec.setZero();

  }


  void
  linear_inequality_t::resize( int NbRows, int NbCols, bool Preserve )
  {

    D.X_mat.resize(NbRows, NbCols);
    D.Y_mat.resize(NbRows, NbCols);
    D.Z_mat.resize(NbRows, NbCols);
    Dc_vec.resize(NbRows);

  }


  solution_t::solution_t():
    NbVariables(0),NbConstraints(0),Fail(0),Print(0),
    Solution_vec(0),SupportOrientations_deq(0),SupportStates_deq(0),
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
      {
        aos<<Solution_vec[i]<<" ";
      };
    aos<<std::endl;
  }

  reference_t::
  reference_t():
    Global(), Local()
  {    }

  reference_t::
  reference_t(const reference_t & R):
    Global(R.Global), Local(R.Local)
  {    }

  reference_t::frame_t::
  frame_t():
    X(0), Y(0), Yaw(0),
    X_vec(), Y_vec(), Yaw_vec()
  {   }

  reference_t::frame_t::
  frame_t(const frame_t & F):
    X(F.X), Y(F.Y), Yaw(F.Yaw),
    X_vec(F.X_vec), Y_vec(F.Y_vec), Yaw_vec(F.Yaw_vec)
  {}

}

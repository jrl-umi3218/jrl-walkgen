/*
 * Copyright 2010,
 *
 * Andrei  Herdt
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
/* This object provides the finite state machine to determine the support parameters. */

#ifndef _SUPPORT_FSM_
#define _SUPPORT_FSM_

#include <jrl/walkgen/pgtypes.hh>
#include <privatepgtypes.hh>

namespace PatternGeneratorJRL
{
  class  SupportFSM
  {
  public:
    /*! Constructor */
    SupportFSM(const double &SamplingPeriod);

    /*! Destructor */
    ~SupportFSM();

    /*! \brief Initialize the previewed state. */
    void setSupportState(const double &Time, const int &pi,
                         SupportState_t & Support, const ReferenceAbsoluteVelocity & RefVel);

    ///*! \brief Numerical precision */
    double m_eps;

    /*! \brief constants for the durations in the support phases */
    double m_DSDuration, m_SSPeriod, m_DSSSDuration;

    //Number of steps done before DS
    unsigned int m_NbOfStepsSSDS;

  private:

    /*! \Brief Sampling duration */
    double m_T;


    bool m_ReferenceGiven;

    int m_FullDebug;

  };
}

#endif /* _SUPPORT_FSM_ */

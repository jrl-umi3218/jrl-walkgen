/*
 * Copyright 2010, 
 *
 * Andrei  Herdt
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

#ifndef _SUPPORT_STATE_
#define _SUPPORT_STATE_

#include <walkGenJrl/PGTypes.h>

namespace PatternGeneratorJRL
{
  class  SupportState
  {
  public:
    /*! Constructor */
    SupportState(const double &SamplingPeriod);
	
    /*! Destructor */
    ~SupportState();
	
    /*! \brief Initialize the previewed state. */
    void setSupportState(const double &Time, const int &pi,  const ReferenceAbsoluteVelocity & RefVel);
	

	
    ///*! \brief Numerical precision */
    double eps;
	
    /*! \brief constants for the durations in the support phases */
    double DSDuration, SSPeriod, DSSSDuration;

    /*! \brief First support foot */
    int StartSupportFoot;
	
    /*! \brief Current support state */
    int CurrentSupportPhase, CurrentSupportFoot, CurrentStepsLeft;

    int SSSS;

    double CurrentTimeLimit;
	
    /*! \brief Future support state */
    int PrwSupportPhase, PrwSupportFoot, PrwStepsLeft;

    double  PrwTimeLimit;

    bool m_StateChanged;

    int StepNumber;

    //Number of steps done before DS
    unsigned int NbOfStepsSSDS;

 
    /*! \brief Current support state */
    //enum supportType {SS = 1, DS = 0} ;
    //enum footType {Left = 1, Right = -1} ;
	
	
    /*! \brief Support state transitions */
    //enum transitionType {SSDS, DSSS, SSSS, ZERO};
	
    /*! \brief Start time of the previewed support state. */
    //double PreviewedStateStartTime;
	
    /*! \brief Previewed support state */
    //int PreviewedStepsLeft;
    //int PreviewedStepNumber;
	
    /* ! \brief Vector of previewed angles  */
    //MAL_VECTOR(PreviewedAngles,double);
	
    /* ! \brief Translational velocity references  */
    //double TransVelRefX;
    //double TransVelRefY;
	
	
	
    /*! \name Internal state methods. 
      @{
    */
    /* ! \brief Finite state machine  *//*
       void switchState(int &SupportPhase,
       int &SupportFoot,
       int &StepsLeft,
       double &SupportDuration,
       int &StepNumber,
       //MAL_VECTOR(,double) &PreviewedAngles,
       //const double AngVelTrunk,
       const double TransVelRefX,
       const double TransVelRefY);
					*/
	
  private: 
	
    /*! \Brief Sampling duration */
    double m_T;

    /*! \Brief Support state */
    int *SupportPhase, *SupportFoot, *SupportStepsLeft;

    double *SupportTimeLimit;
    void initializePreviewedState();

    int ReferenceGiven;

    int m_FullDebug;

    /*! \name Getter of variables
      @{
    */
    /*! Getter for the support phase */
    //	const int & SupportState::getSupportPhase () const;
	
    /*! Getter for the support foot */
    //	const int & SupportState::getSupportFoot () const;
	
    /*! Getter for the step number */
    //	const int & SupportState::getStepNumber () const;
    
  };
};

#endif /* _SUPPORT_STATE_ */

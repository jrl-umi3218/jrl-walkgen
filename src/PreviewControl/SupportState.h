/* This object provides the finite state machine to determine the support parameters.
 
 Copyright (c) 2010, 
 Andrei Herdt
 
 JRL-Japan, CNRS/AIST
 
 All rights reserved.
 
 Please see License.txt for further information on license.         
 */

#ifndef _SUPPORT_STATE_
#define _SUPPORT_STATE_


class  SupportState
{
public:
	/*! Constructor */
	SupportState();
	
	/*! Destructor */
	~SupportState();
	
	/*! \brief Initialize the previewed state. */
	void setSupportState();
	

	
	/*! \brief Numerical precision */
	double eps;
	
	/*! \brief constants for the durations in the support phases */
	double DSDuration, SSDuration, DSSSDuration;

	/*! \brief First support foot */
	int StartSupportFoot;
	
	/*! \brief Current support state */
	int CurrentSupportPhase, CurrentSupportFoot, CurrentTimeLimit, CurrentStepsLeft;
	
	/*! \brief Future support state */
	int SupportPhase, SupportFoot, TimeLimit, StepsLeft;

	
	/*! \brief Current support state */
	enum supportType {SS = 1, DS = 0} ;
	enum footType {Left = 1, Right = -1} ;
	int CurrentStepsLeft;
	
	/*! \brief Support state transitions */
	enum transitionType {SSDS, DSSS, SSSS, ZERO};
	
	/*! \brief Start time of the previewed support state. */
	double PreviewedStateStartTime;
	
	/*! \brief Previewed support state */
	int PreviewedStepsLeft;
	int PreviewedStepNumber;
	
	/* ! \brief Vector of previewed angles  */
	//MAL_VECTOR(PreviewedAngles,double);
	
	/* ! \brief Translational velocity references  */
	double TransVelRefX;
	double TransVelRefY;
	
	/* ! \brief Equality constraints needed?  */
	bool LastStep;
	bool FirstStep;
	
	
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
	
	/*! \name Getter of variables
	 @{
	 */
	/*! Getter for the support phase */
	const int & SupportState::getSupportPhase () const;
	
	/*! Getter for the support foot */
	const int & SupportState::getSupportFoot () const;
	
	/*! Getter for the step number */
	const int & SupportState::getStepNumber () const;
    
};

#endif /* _SUPPORT_STATE_ */

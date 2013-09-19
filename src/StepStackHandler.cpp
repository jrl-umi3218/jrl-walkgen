/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010, 
 *
 * Francois   Keith
 * Olivier    Stasse
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
/*! This object handle the step stack of the pattern generator.
   It allows also to create automatically stack of steps according to 
   some high level functionnalities.
*/
#include <fstream>
#include <math.h>

#include <jrl/mal/matrixabstractlayer.hh>

#define deg2rad(x) x*M_PI/180.0
#define rad2deg(x) x*180.0/M_PI

#include <Debug.hh>
#include <StepStackHandler.hh>

using namespace::PatternGeneratorJRL;

StepStackHandler::StepStackHandler(SimplePluginManager *lSPM) : SimplePlugin(lSPM)
{
  m_OnLineSteps = false;
  m_SingleSupportTime = 0.0;
  m_DoubleSupportTime = 0.0;
  m_StOvPl = 0;
  m_WalkMode = 0;
  m_KeepLastCorrectSupportFoot=1;
  m_RelativeFootPositions.clear();
  m_TransitionFinishOnLine=false;

  std::string aMethodName[8] = 
    {":walkmode",
     ":singlesupporttime",
     ":doublesupporttime",
     ":supportfoot",
     ":lastsupport",
     ":arc",
     ":addstandardonlinestep",
     ":arccentered"};

  for(int i=0;i<7;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
    }
  RESETDEBUG4("DebugFootPrint.dat");
}

StepStackHandler::~StepStackHandler()
{

}


RelativeFootPosition StepStackHandler::ReturnBackFootPosition()
{
  return m_RelativeFootPositions.back();
}

bool StepStackHandler::ReturnFrontFootPosition(RelativeFootPosition  &aRFP)
{
  if (m_RelativeFootPositions.size()>0)
    {
      aRFP = m_RelativeFootPositions.front();
      return true;
    }
  return false;
}

int StepStackHandler::ReturnStackSize()
{
  return m_RelativeFootPositions.size();
}

void StepStackHandler::CopyRelativeFootPosition(deque<RelativeFootPosition> &lRelativeFootPositions,
						bool PerformClean)
{
  ODEBUG(m_RelativeFootPositions.size());
  lRelativeFootPositions.resize(m_RelativeFootPositions.size());
  for(unsigned int i=0;i<m_RelativeFootPositions.size();i++)
    {
      lRelativeFootPositions[i] = m_RelativeFootPositions[i];
    }
  if (PerformClean)
    m_RelativeFootPositions.clear();
}

void StepStackHandler::SetStepOverPlanner(StepOverPlanner *lStOvPl)
{
  m_StOvPl = lStOvPl;
}

void StepStackHandler::SetWalkMode(int lWalkMode)
{
  m_WalkMode = lWalkMode;
}

int StepStackHandler::GetWalkMode()
{
  return m_WalkMode;
}


void StepStackHandler::ReadStepSequenceAccordingToWalkMode(istringstream &strm)
{
  m_RelativeFootPositions.clear();
  switch (m_WalkMode)
    {
    case 0:
    case 4:
      {	
			
	ODEBUG( "Standard Walk Mode Selected" );
	RelativeFootPosition aFootPosition;
			
	while(!strm.eof())
	  {
	    if (!strm.eof())
	      strm >> aFootPosition.sx;
	    else break;
	    if (!strm.eof())
	      strm >> aFootPosition.sy;
	    else 
	      break;
	    if (!strm.eof())
	      strm >> aFootPosition.theta;
	    else 
	      break;
				
	    aFootPosition.DeviationHipHeight = 0;
	    aFootPosition.SStime=m_SingleSupportTime;
	    aFootPosition.DStime=m_DoubleSupportTime;
	    aFootPosition.stepType=1;
	    ODEBUG4(aFootPosition.sx << " " <<
		    aFootPosition.sy << " " <<
		    aFootPosition.theta << " " << 
		    aFootPosition.SStime << " " << 
		    aFootPosition.DStime << " " << 
		    aFootPosition.DeviationHipHeight << " " ,
		    "DebugGMFKW.dat");
			
	    m_RelativeFootPositions.push_back(aFootPosition);
	    if (aFootPosition.sy>0)
	      m_KeepLastCorrectSupportFoot=-1;
	    else
	      m_KeepLastCorrectSupportFoot=1;
	  }

	ODEBUG("m_RelativeFootPositions: " << m_RelativeFootPositions.size());
	break;
      }
    case 3:
    case 1:
      {	
	ODEBUG4( "Walk Mode with HipHeight Variation Selected","DebugGMFKW.dat" );
	RelativeFootPosition aFootPosition;
	
	ODEBUG4("Inside StepStack Handler","DebugGMFKW.dat");
	while(!strm.eof())
	  {
	    if (!strm.eof())
	      strm >> aFootPosition.sx;
	    else break;
	    if (!strm.eof())
	      strm >> aFootPosition.sy;
	    else break;
	    if (!strm.eof())
	      strm >> aFootPosition.theta;
	    else break;
	    if (!strm.eof())
	      strm >> aFootPosition.DeviationHipHeight;
	    else break;
	    aFootPosition.SStime=m_SingleSupportTime;
	    aFootPosition.DStime=m_DoubleSupportTime;
	    aFootPosition.stepType=1;
	    ODEBUG4(aFootPosition.sx << " " <<
		    aFootPosition.sy << " " <<
		    aFootPosition.theta << " " << 
		    aFootPosition.SStime << " " << 
		    aFootPosition.DStime << " " << 
		    aFootPosition.DeviationHipHeight << " " ,
		    "DebugFootPrint.dat");
	    m_RelativeFootPositions.push_back(aFootPosition);
	    if (aFootPosition.sy>0)
	      m_KeepLastCorrectSupportFoot=-1;
	    else
	      m_KeepLastCorrectSupportFoot=1;
				
	  }
	ODEBUG4("Finito for the reading.  StepStack Handler","DebugGMFKW.dat");
	break;
			
      }
    case 2:
	{	
	  ODEBUG( "Walk Mode with Obstacle StepOver Selected \
                 (obstacle parameters have to be set first, \
                 if not standard dimensions are used)" );
	  //cout << "I am calculating relative positions to negociate obstacle" << endl;
	  m_StOvPl->CalculateFootHolds(m_RelativeFootPositions);
  
	  break;
	}
    // With a varying double support time and a single support time.
    case 5:
      {	
			
	ODEBUG3( "Standard Walk Mode Selected" );
	RelativeFootPosition aFootPosition;
			
	while(!strm.eof())
	  {
	    if (!strm.eof())
	      strm >> aFootPosition.sx;
	    else break;
	    if (!strm.eof())
	      strm >> aFootPosition.sy;
	    else 
	      break;
	    if (!strm.eof())
	      strm >> aFootPosition.theta;
	    else 
	      break;

	    double lSST=0.0, lDST=0.0;
	    if (!strm.eof())
	      strm >> aFootPosition.SStime;
	    else 
	      break;
	    if (!strm.eof())
	      strm >> aFootPosition.DStime;
	    else 
	      break;
				
	    aFootPosition.DeviationHipHeight = 0;
	    aFootPosition.stepType=1;
	    ODEBUG3("FootPositions:" << aFootPosition.sx << " " <<
		    aFootPosition.sy << " " <<
		    aFootPosition.theta << " " << 
		    aFootPosition.SStime << " " << 
		    aFootPosition.DStime << " " << 
		    aFootPosition.DeviationHipHeight << " " );
	
	    ODEBUG4(aFootPosition.sx << " " <<
		    aFootPosition.sy << " " <<
		    aFootPosition.theta << " " << 
		    aFootPosition.SStime << " " << 
		    aFootPosition.DStime << " " << 
		    aFootPosition.DeviationHipHeight << " " ,
		    "DebugGMFKW.dat");
			
	    m_RelativeFootPositions.push_back(aFootPosition);
	    if (aFootPosition.sy>0)
	      m_KeepLastCorrectSupportFoot=-1;
	    else
	      m_KeepLastCorrectSupportFoot=1;
	  }

	ODEBUG("m_RelativeFootPositions: " << m_RelativeFootPositions.size());
	break;
      }

    default: 
      {
	ODEBUG( "PLease select proper walk mode. \
            (0 for normal walking ; \
             1 for walking with waistheight variation ; \
             2 for walking with obstacle stepover)" );
	return;
      }
    }

}

void StepStackHandler::CreateArcInStepStack(  double x,double y, double R,
					      double arc_deg, int SupportFoot)
{
  RelativeFootPosition aFootPosition;
  double StepMax = 0.15;
  double LastStep=0.0;
  double NumberOfStepFloat;
  int NumberOfStep=0;
  double OmegaStep=0;
  double OmegaTotal=deg2rad(arc_deg);
  double LastOmegaStep=0.0;
  int DirectionRay = -1;

  // Compute the ray of the arc.
  R = sqrt(x*x+y*y);

  // Compute the number of steps (in floating point)
  NumberOfStepFloat = OmegaTotal*R/StepMax;

  // Take the integer value of the number of steps.
  NumberOfStep = (int)floor(NumberOfStepFloat);

  // Computes the last step value.
  LastStep = OmegaTotal*R - NumberOfStep * StepMax;
    
  //  OmegaStep = arc_deg/(double)NumberOfStep;
  OmegaStep = StepMax/R;
  LastOmegaStep = OmegaTotal - OmegaStep * NumberOfStep;

  OmegaStep = rad2deg(OmegaStep);
  LastOmegaStep = rad2deg(LastOmegaStep);

  // Handle direction
  if (x<0)
    {
      StepMax = -StepMax;
      LastStep = -LastStep;
      DirectionRay = 1;
    }
  if (y<0)
    {
      OmegaStep= -OmegaStep;
      LastOmegaStep = -LastOmegaStep;
    }

  ODEBUG4(NumberOfStep << " "
	  << LastStep<< " "
	  << arc_deg, "DebugFootPrint.dat");
  

  double Omegakp = 0.0,
    Omegak=0.0;

  for(int i=0;i<NumberOfStep;i++)
    {
      ODEBUG("SupportFoot " << SupportFoot);
      aFootPosition.sx = StepMax;
      aFootPosition.sy = SupportFoot*0.19;
      aFootPosition.theta = OmegaStep;

      {
	MAL_MATRIX_DIM(A,double,2,2);
	MAL_MATRIX_SET_IDENTITY(A);
	MAL_MATRIX_DIM(Ap,double,2,2);
	MAL_MATRIX_SET_IDENTITY(Ap);
	
	Omegakp = Omegak;
	Omegak = Omegak + OmegaStep;
	ODEBUG("Omegak:" << Omegak );
	double c,s;
	c = cos(Omegak*M_PI/180.0);
	s = sin(Omegak*M_PI/180.0);

	// Transpose of the orientation matrix 
	// to get the inverse of the orientation matrix.
	A(0,0) =  c; A(0,1) = s;
	A(1,0) = -s; A(1,1) =  c;

	double cp,sp;
	cp = cos(Omegakp*M_PI/180.0);
	sp = sin(Omegakp*M_PI/180.0);
	
	MAL_VECTOR_DIM(lv,double,2);
	MAL_VECTOR_DIM(lv2,double,2);
	lv(0) = (R+DirectionRay*SupportFoot*0.095)*s - (R-DirectionRay*SupportFoot*0.095)*sp;
	lv(1) = -((R+DirectionRay*SupportFoot*0.095)*c - (R-DirectionRay*SupportFoot*0.095)*cp);
	MAL_C_eq_A_by_B(lv2,A,lv);
	ODEBUG(" X: " << (R+DirectionRay*SupportFoot*0.095)*s << " " << (R-DirectionRay*SupportFoot*0.095)*sp 
		<< " " << StepMax << " " << lv(0) << " " << lv2(0) );
	ODEBUG(" Y: " << (R+DirectionRay*SupportFoot*0.095)*c << " " << (R-DirectionRay*SupportFoot*0.095)*cp 
		<< " " << SupportFoot*0.19 << " " << lv(1) << " " << lv2(1));

	
	aFootPosition.sx = lv2(0);
	aFootPosition.sy = lv2(1);
	
      }
      
      aFootPosition.SStime = m_SingleSupportTime;
      aFootPosition.DStime = m_DoubleSupportTime;
      m_RelativeFootPositions.push_back(aFootPosition);

      ODEBUG4(aFootPosition.sx<< " "
	      << aFootPosition.sy<< " "
	      << aFootPosition.theta,"DebugFootPrint.dat");

      SupportFoot = - SupportFoot;
    }
  if (LastStep!=0.0)
    {
      aFootPosition.sx = LastStep;
      aFootPosition.sy = SupportFoot*0.19;
      aFootPosition.theta = LastOmegaStep;

      {
	MAL_MATRIX_DIM(A,double,2,2);
	MAL_MATRIX_SET_IDENTITY(A);
	
	Omegakp = Omegak;
	Omegak = Omegak + LastOmegaStep;
	ODEBUG( "Omegak:" << Omegak );
	double c,s;
	c = cos(Omegak*M_PI/180.0);
	s = sin(Omegak*M_PI/180.0);
	
	double cp,sp;
	cp = cos(Omegakp*M_PI/180.0);
	sp = sin(Omegakp*M_PI/180.0);
	
	A(0,0) = c;  A(0,1) =s;
	A(1,0) = -s;  A(1,1) = c;
	MAL_VECTOR_DIM(lv,double,2);
	MAL_VECTOR_DIM(lv2,double,2);
	lv(0) = (R+DirectionRay*SupportFoot*0.095)*s - (R-DirectionRay*SupportFoot*0.095)*sp;
	lv(1) = -((R+DirectionRay*SupportFoot*0.095)*c - (R-DirectionRay*SupportFoot*0.095)*cp);
	MAL_C_eq_A_by_B(lv2,A,lv);
	ODEBUG(" X: " << (R+DirectionRay*SupportFoot*0.095)*s << " " << (R-DirectionRay*SupportFoot*0.095)*sp 
		<< " " << lv(0) << " " << lv2(0) );
	ODEBUG(" Y: " << (R+DirectionRay*SupportFoot*0.095)*c << " " << (R-DirectionRay*SupportFoot*0.095)*cp 
		<< " " << lv(1) << " " << lv2(1) );
	
	aFootPosition.sx = lv2(0);
	aFootPosition.sy = lv2(1);
	
      }

      aFootPosition.SStime = m_SingleSupportTime;
      aFootPosition.DStime = m_DoubleSupportTime;
      
      m_RelativeFootPositions.push_back(aFootPosition);

      ODEBUG4(aFootPosition.sx<< " "
	      << aFootPosition.sy<< " "
	      << aFootPosition.theta,"DebugFootPrint.dat");

      SupportFoot = - SupportFoot;
    }
  m_KeepLastCorrectSupportFoot = SupportFoot;
}

void StepStackHandler::CreateArcCenteredInStepStack(  double R,
						      double arc_deg, int SupportFoot)
{
  RelativeFootPosition aFootPosition;
  double StepMax = 0.10;
  //  double StepMax = 0.225;
  double LastStep=0.0;
  double NumberOfStepFloat;
  int NumberOfStep=0;
  double OmegaStep=0;
  double OmegaTotal=deg2rad(arc_deg);
  double LastOmegaStep=0.0;
  double sinOmegaStep, cosOmegaStep;
  // Compute the ray of the arc.

  // Compute the number of steps (in floating point)
  NumberOfStepFloat = OmegaTotal*R/StepMax;

  // Take the integer value of the number of steps.
  NumberOfStep = (int)floor(NumberOfStepFloat);

  // Computes the last step value.
  LastStep = OmegaTotal*R - NumberOfStep * StepMax;
    
  OmegaStep = StepMax/R;
  LastOmegaStep = OmegaTotal - OmegaStep*NumberOfStep;
  
#if 0
  ofstream DebugFile;
  DebugFile.open("/tmp/output.txt",ofstream::out);
  DebugFile << NumberOfStep << " "
	    << OmegaStep << " " 
	    << LastOmegaStep<< " "
	    << arc_deg<< " "
	    << endl;
  DebugFile.close();
#endif

  cosOmegaStep = cos(OmegaStep);
  sinOmegaStep = sin(OmegaStep);
  
  // Make sure that Support Foot is the foot which 
  // does not lead the motion.
  if (SupportFoot*OmegaStep<0.0)
    {
      aFootPosition.sx = 0;
      aFootPosition.sy = -SupportFoot*0.095;
      aFootPosition.theta = 0;
      aFootPosition.SStime = m_SingleSupportTime;
      aFootPosition.DStime = m_DoubleSupportTime;
      
      m_RelativeFootPositions.push_back(aFootPosition);
#if 0
      DebugFile.open("/tmp/output.txt",ofstream::app);
      DebugFile << aFootPosition.sx<< " "
		<< aFootPosition.sy<< " " 
		<< aFootPosition.theta<< " "
		<< endl;
      DebugFile.close();
#endif
      
      SupportFoot=-SupportFoot;
    }

  double S=-SupportFoot*0.095;

  MAL_MATRIX_DIM(Romegastep,double,3,3);
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      if (i==j)
	Romegastep(i,j) =1.0;
      else
	Romegastep(i,j) =0.0;
  Romegastep(0,0) = cosOmegaStep; Romegastep(0,1) = -sinOmegaStep;
  Romegastep(1,0) = sinOmegaStep; Romegastep(1,1) =  cosOmegaStep;

  MAL_MATRIX_DIM(MFNSF,double,3,3);
  MAL_MATRIX_DIM(MFSF,double,3,3);
  MAL_MATRIX_DIM(Romega,double,3,3);
  MAL_MATRIX_DIM(iRomega,double,3,3);
  MAL_MATRIX_DIM(RiR,double,3,3);
  MAL_MATRIX_DIM(FPos,double,3,3);
  MAL_MATRIX_DIM(MSupportFoot,double,3,3);
  MAL_MATRIX_DIM(Mtmp,double,3,3);
  MAL_MATRIX_DIM(Mtmp2,double,3,3);

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      if (i==j)
	{
	  MFNSF(i,j)  = 
	  MFSF(i,j)   = 
	  Romega(i,j) = 
	  Mtmp(i,j)   =  
	  iRomega(i,j)= 1.0;
	}
      else
	{
	  MFNSF(i,j)   =
	  MFSF(i,j)    =
	  Romega(i,j)  =
	  Mtmp(i,j)    =
          Mtmp2(i,j)   =
	  iRomega(i,j) = 0.0;
	}
    

  MFSF(0,2)=-R;
  MFSF(1,2)=-S;
  MFNSF(0,2)=-R;
  MFNSF(1,2)=S;
  MSupportFoot=MFSF; 
  Mtmp(1,2) = 0.19;
#if 0
  DebugFile.open("/tmp/outputNL.txt",ofstream::app);
  DebugFile << MSupportFoot(0,2) << " " 
	    << MSupportFoot(1,2) << endl;
  DebugFile.close();
#endif
  cout << "MSupportFoot  "<< endl << MSupportFoot << endl;
  cout << "Romegastep " << endl << Romegastep << endl;
  for(int i=0;i<NumberOfStep;i++)
    {
      double cosiOmegaStep,siniOmegaStep;

      cosiOmegaStep = cos((i+1)*OmegaStep);
      siniOmegaStep = sin((i+1)*OmegaStep);
      
      Romega(0,0) = cosiOmegaStep;
      Romega(0,1) = -siniOmegaStep;
      Romega(1,0) = siniOmegaStep;
      Romega(1,1) = cosiOmegaStep;
      Romega(0,2) = 0;
      Romega(1,2) = 0;
      
      MAL_MATRIX(lTmp,double);
      MAL_C_eq_A_by_B(lTmp,MSupportFoot,Romegastep);
      MAL_INVERSE(lTmp, RiR,double);
      
      ODEBUG(" Iteration " << i);
      ODEBUG(" Romega " << Romega);
      ODEBUG(" RiR " << RiR);
	
      MAL_C_eq_A_by_B(FPos, Romega, MFNSF);
      ODEBUG("FPos: " << FPos);
      MAL_C_eq_A_by_B(FPos,RiR,FPos);
      ODEBUG("FPos final :");

      aFootPosition.sx = FPos(0,2);
      aFootPosition.sy = FPos(1,2);
      aFootPosition.theta = rad2deg(OmegaStep);
      aFootPosition.SStime = m_SingleSupportTime;
      aFootPosition.DStime = m_DoubleSupportTime;

      m_RelativeFootPositions.push_back(aFootPosition);
      MAL_C_eq_A_by_B(MSupportFoot, Romega,MFNSF);
      
#if 0
      DebugFile.open("/tmp/outputL.txt",ofstream::app);
      DebugFile << MSupportFoot(0,2) << " "
		<< MSupportFoot(1,2) << " "
		<< endl;
      DebugFile.close();

      DebugFile.open("/tmp/output.txt",ofstream::app);
      DebugFile << aFootPosition.sx<< " "
		<< aFootPosition.sy<< " "
		<< aFootPosition.theta<< " "
		<< endl;
      DebugFile.close();
#endif
      aFootPosition.sx = 0;
      aFootPosition.sy = SupportFoot*0.19;
      aFootPosition.theta = 0;
      aFootPosition.SStime = m_SingleSupportTime;
      aFootPosition.DStime = m_DoubleSupportTime;

      m_RelativeFootPositions.push_back(aFootPosition);
#if 0
      DebugFile.open("/tmp/output.txt",ofstream::app);
      DebugFile << aFootPosition.sx<< " "
		<< aFootPosition.sy<< " "
		<< aFootPosition.theta<< " "
		<< endl;
      DebugFile.close();
#endif
      /*
      for(int li=0;li<2;li++)
	for(int lj=0;lj<2;lj++)
	  Mtmp2[li][lj]=MSupportFoot[li][lj];
	  
	  Mtmp2 = Mtmp2*Mtmp;
      */
      MSupportFoot =  MAL_RET_A_by_B( MSupportFoot, Mtmp);
      
#if 0
      DebugFile.open("/tmp/outputNL.txt",ofstream::app);
      DebugFile << MSupportFoot(0,2) << " " 
		<< MSupportFoot(1,2) << endl;
      DebugFile.close();
#endif

    }

  if (LastStep!=0.0)
    {
      double cosiOmegaStep,siniOmegaStep;
      
      cosiOmegaStep = cos(LastOmegaStep+NumberOfStep*OmegaStep);
      siniOmegaStep = sin(LastOmegaStep+NumberOfStep*OmegaStep);
      for(int i=0;i<3;i++)
	for(int j=0;j<3;j++)
	  if (i==j)
	    Romega(i,j) = iRomega(i,j) = 1.0;
	  else
	    Romega(i,j) = iRomega(i,j) = 0.0;
      
      Romega(0,0) = cosiOmegaStep;
      Romega(0,1) = -siniOmegaStep;
      Romega(1,0) = siniOmegaStep;
      Romega(1,1) = cosiOmegaStep;
      Romega(0,2) = 0;
      Romega(1,2) = 0;
      
      double coslOmegaStep,sinlOmegaStep;
      coslOmegaStep = cos(LastOmegaStep);
      sinlOmegaStep = sin(LastOmegaStep);


      iRomega(0,0) = coslOmegaStep;
      iRomega(0,1) = -sinlOmegaStep;
      iRomega(1,0) = sinlOmegaStep;
      iRomega(1,1) = coslOmegaStep;
      
      MAL_MATRIX(lTmp,double);
      MAL_C_eq_A_by_B(lTmp,MSupportFoot,iRomega);
      MAL_INVERSE(lTmp, RiR,double);

      MAL_C_eq_A_by_B(FPos, Romega, MFNSF);
      FPos = MAL_RET_A_by_B(RiR,FPos);

      aFootPosition.sx = FPos(0,2);
      aFootPosition.sy = FPos(1,2);
      aFootPosition.theta = rad2deg(LastOmegaStep);
      aFootPosition.SStime = m_SingleSupportTime;
      aFootPosition.DStime = m_DoubleSupportTime;

      m_RelativeFootPositions.push_back(aFootPosition);
      MAL_C_eq_A_by_B(MSupportFoot,Romega,MFNSF);

#if 0
      DebugFile.open("/tmp/outputL.txt",ofstream::app);
      DebugFile << MSupportFoot(0,2) << " "
		<< MSupportFoot(1,2) << " "
		<< endl;
      DebugFile.close();

      DebugFile.open("/tmp/output.txt",ofstream::app);
      DebugFile << aFootPosition.sx<< " "
		<< aFootPosition.sy<< " "
		<< aFootPosition.theta<< " "
		<< endl;
      DebugFile.close();
#endif
      aFootPosition.sx = 0;
      aFootPosition.sy = SupportFoot*0.19;
      aFootPosition.theta = 0;
      aFootPosition.SStime = m_SingleSupportTime;
      aFootPosition.DStime = m_DoubleSupportTime;

      m_RelativeFootPositions.push_back(aFootPosition);

#if 0
      DebugFile.open("/tmp/output.txt",ofstream::app);
      DebugFile << aFootPosition.sx<< " "
		<< aFootPosition.sy<< " "
		<< aFootPosition.theta<< " "
		<< endl;
      DebugFile.close();
#endif
      MSupportFoot = MAL_RET_A_by_B( MSupportFoot , Mtmp);
     
#if 0
      DebugFile.open("/tmp/outputNL.txt",ofstream::app);
      DebugFile << MSupportFoot(0,2) << " " 
		<< MSupportFoot(1,2) << endl;
      DebugFile.close();
#endif


    }
  m_KeepLastCorrectSupportFoot = -SupportFoot;
}

// Prepare the stack to start a motion on a specific support foot
void StepStackHandler::PrepareForSupportFoot(int SupportFoot)
{
  RelativeFootPosition aFootPosition;
  aFootPosition.sx = 0;
  aFootPosition.sy = SupportFoot*0.095;
  aFootPosition.theta = 0;
  aFootPosition.SStime = m_SingleSupportTime;
  aFootPosition.DStime = m_DoubleSupportTime;

  m_RelativeFootPositions.push_back(aFootPosition);
}

void StepStackHandler::StopOnLineStep()
{
  //  m_OnLineSteps = false;
  m_TransitionFinishOnLine=true;

  // Correct the last support foot before cleaning up the 
  // stack.
  if (m_RelativeFootPositions.size()%2==0)
    m_KeepLastCorrectSupportFoot = -m_KeepLastCorrectSupportFoot;

  m_RelativeFootPositions.clear();

}

void StepStackHandler::StartOnLineStep()
{
  m_OnLineSteps = true;
  ODEBUG("StartOnLineStep(): " << m_RelativeFootPositions.size());
}

bool StepStackHandler::IsOnLineSteppingOn()
{
  return m_OnLineSteps;
}

void StepStackHandler::AddStandardOnLineStep(bool NewStep, 
					     double NewStepX,
					     double NewStepY,
					     double NewTheta)
{
  RelativeFootPosition aFootPosition;
  ODEBUG("m_OnLineSteps: "<<m_OnLineSteps);
  if (!m_OnLineSteps)
    return;
  
  ODEBUG("m_KeepLastCorrectSupportFoot" << m_KeepLastCorrectSupportFoot);
  if (!NewStep)
    {
      aFootPosition.sx = 0;
      aFootPosition.sy = m_KeepLastCorrectSupportFoot*0.19;
      aFootPosition.theta = 0;
      aFootPosition.SStime = m_SingleSupportTime;
      aFootPosition.DStime = m_DoubleSupportTime;
      aFootPosition.stepType = 0;
      
    }
  else
    {

      aFootPosition.sx = NewStepX;
      aFootPosition.sy = NewStepY + m_KeepLastCorrectSupportFoot*0.19;
      aFootPosition.theta = NewTheta;
      aFootPosition.SStime = m_SingleSupportTime;
      aFootPosition.DStime = m_DoubleSupportTime;
      aFootPosition.stepType = 0;
      cout << aFootPosition.sx << " " 
	   << aFootPosition.sy << " " 
	   << aFootPosition.theta << endl;
    }      

  ODEBUG("m_RelativeFootPositions:" << m_RelativeFootPositions.size());
  m_RelativeFootPositions.push_back(aFootPosition);
  ODEBUG("m_RelativeFootPositions:" << m_RelativeFootPositions.size());
  
  m_KeepLastCorrectSupportFoot= - m_KeepLastCorrectSupportFoot;
  
}


bool StepStackHandler::RemoveFirstStepInTheStack()
{
  ODEBUG("RemoveFirstStepInTheStack:: "<< m_RelativeFootPositions.size());
  m_RelativeFootPositions.pop_front();
  if ((m_RelativeFootPositions.size()==0) &&
      m_TransitionFinishOnLine &&
      m_OnLineSteps)
    {
      m_OnLineSteps = false;
      m_TransitionFinishOnLine = false;
      return true;
    }
  return false;
}

void StepStackHandler::AddStepInTheStack(double sx, double sy,
					 double theta, double sstime,
					 double dstime)
{
  RelativeFootPosition aFootPosition;
  aFootPosition.sx = sx;
  aFootPosition.sy = sy;
  aFootPosition.theta = theta;
  aFootPosition.SStime = sstime;
  aFootPosition.DStime = dstime;
  aFootPosition.stepType = 0;  

  m_RelativeFootPositions.push_back(aFootPosition);
}

void StepStackHandler::PushFrontAStepInTheStack(RelativeFootPosition &aRFP)
{
  m_RelativeFootPositions.push_front(aRFP);
}

// Make sure that the previous motion will finish 
// on the last specified correct support foot.
void StepStackHandler::FinishOnTheLastCorrectSupportFoot()
{
  RelativeFootPosition aFootPosition;
  aFootPosition.sx = 0;
  aFootPosition.sy = m_KeepLastCorrectSupportFoot*0.19;
  aFootPosition.theta = 0;
  aFootPosition.SStime = m_SingleSupportTime;
  aFootPosition.DStime = m_DoubleSupportTime;
  aFootPosition.stepType = 0;

  m_RelativeFootPositions.push_back(aFootPosition);
}


void StepStackHandler::SetSingleTimeSupport(double lTSsupport)
{
  m_SingleSupportTime = lTSsupport;
}


double StepStackHandler::GetSingleTimeSupport()
{
  return m_SingleSupportTime;
}

void StepStackHandler::SetDoubleTimeSupport(double lTDsupport)
{
  m_DoubleSupportTime = lTDsupport;
}

double StepStackHandler::GetDoubleTimeSupport()
{
  return m_DoubleSupportTime;
}

void StepStackHandler::m_PartialStepSequence(istringstream &strm)
{
  RelativeFootPosition aFootPosition;
  

  while(!strm.eof())
    {
      if (!strm.eof())
	strm >> aFootPosition.sx;
      else break;
      if (!strm.eof())
	strm >> aFootPosition.sy;
      else 
	break;
      if (!strm.eof())
	strm >> aFootPosition.theta;
      else 
	break;

      aFootPosition.DStime = m_DoubleSupportTime;
      aFootPosition.SStime = m_SingleSupportTime;
      aFootPosition.stepType = 0;
      m_RelativeFootPositions.push_back(aFootPosition);
    }
}

/* Implementation of the plugin methods. */
void StepStackHandler::CallMethod(std::string &Method, std::istringstream &strm)
{
  if (Method==":singlesupporttime")
    {
      strm >> m_SingleSupportTime;
    }
  else if (Method==":doublesupporttime")
    {
      strm >> m_DoubleSupportTime;
    }
  else if (Method==":walkmode")
    {
      strm >> m_WalkMode;
    }
  else if (Method==":supportfoot")
    {
      int SupportFoot=-1;
      strm >> SupportFoot;
      PrepareForSupportFoot(SupportFoot);
    }
  else if (Method==":lastsupport")
    {
      FinishOnTheLastCorrectSupportFoot();
    }
  else if (Method==":addstandardonlinestep")
    {
      double x,y,theta;

      while(!strm.eof())
	{
	  
	  if (!strm.eof())
	    strm >> x;
	  else break;
	  
	  if (!strm.eof())
	    strm >> y;
	  else break;
	  
	  if (!strm.eof())
	    strm >> theta;
	  else break;
	  
	}
      AddStandardOnLineStep(true,x,y,theta);
      
    }
  else if (Method==":arc")
    {
      double x,y,R=0.0,arc_deg;
      int SupportFoot=-1;
      
      
      while(!strm.eof())
	{
	  
	  if (!strm.eof())
	    strm >> x;
	  else break;
	  
	  if (!strm.eof())
	    strm >> y;
	  else break;

	  if (!strm.eof())
	    strm >> arc_deg;
	  else break;
	  
	  if (!strm.eof())
	    strm >> SupportFoot;
	  else break;

	}
      
      
      CreateArcInStepStack(x,y,R,arc_deg,SupportFoot);
    }
  else if (Method==":arccentered")
    {
      double R,arc_deg;
      int SupportFoot=-1;
      ODEBUG4("m_CreateArcCenteredInStepStack 1", "DebugData.txt");
      
      while(!strm.eof())
	{
	  
	  if (!strm.eof())
	    strm >> R;
	  else break;

	  if (!strm.eof())
	    strm >> arc_deg;
	  else break;
	  
	  if (!strm.eof())
	    strm >> SupportFoot;
	  else break;

	}
      ODEBUG4("m_CreateArcCenteredInStepStack 2", "DebugData.txt");
      CreateArcCenteredInStepStack(R,arc_deg,SupportFoot);
      ODEBUG4("m_CreateArcCenteredInStepStack 3", "DebugData.txt");
    }

}

/* This object handle the step stack of the pattern generator.
   It allows also to create automatically stack of steps according to 
   some high level functionnalities.

   CVS Information: 
   $Id $
   $Author $
   $Date$
   $Revision $
   $Source$
   $Log $

   Copyright (c) 2005-2006, 
   Olivier Stasse,
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <fstream>
#include <math.h>

#include <VNL/matrix.h>
#include <VNL/Algo/matrixinverse.h>
#define deg2rad(x) x*M_PI/180.0
#define rad2deg(x) x*180.0/M_PI
#if 0
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "StepStackHandler: " << x << endl; DebugFile.close();}
#else
#define ODEBUG4(x,y) 
#endif
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "SSH: " << x << endl; DebugFile.close();}

#if 1
#define ODEBUG(x)
#else
#define ODEBUG(x)  std::cout << x << endl;
#endif

#define ODEBUG3(x) std::cout << x << endl;

#include <StepStackHandler.h>
#include <VNL/matrix.h>

using namespace::PatternGeneratorJRL;

StepStackHandler::StepStackHandler()
{
  m_OnLineSteps = false;
  m_TSsupport = 0.0;
  m_TDsupport = 0.0;
  m_StOvPl = 0;
  m_WalkMode = 0;
  m_KeepLastCorrectSupportFoot=1;
  m_RelativeFootPositions.clear();
  m_TransitionFinishOnLine=false;
}

StepStackHandler::~StepStackHandler()
{

}


RelativeFootPosition StepStackHandler::ReturnBackFootPosition()
{
  return m_RelativeFootPositions.back();
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
				
	    aFootPosition.SStime=m_TSsupport;
	    aFootPosition.DStime=m_TDsupport;
	    aFootPosition.stepType=1;
			
	    m_RelativeFootPositions.push_back(aFootPosition);
	  }
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
	    aFootPosition.SStime=m_TSsupport;
	    aFootPosition.DStime=m_TDsupport;
	    aFootPosition.stepType=1;
	    ODEBUG4(aFootPosition.sx << " " <<
		    aFootPosition.sy << " " <<
		    aFootPosition.theta << " " << 
		    aFootPosition.DeviationHipHeight << " " << li++,
		    "DebugGMFKW.dat");
	    m_RelativeFootPositions.push_back(aFootPosition);
				
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
    }
  if (y<0)
    {
      OmegaStep= -OmegaStep;
      LastOmegaStep = -LastOmegaStep;
    }
#if 1
  ofstream DebugFile;
  DebugFile.open("/tmp/output.txt",ofstream::app);
  DebugFile << NumberOfStep << " "
	    << LastStep<< " "
	    << arc_deg<< " "
	    << endl;
  DebugFile.close();
#endif
  
  for(int i=0;i<NumberOfStep;i++)
    {
      aFootPosition.sx = StepMax;
      aFootPosition.sy = SupportFoot*0.19;
      aFootPosition.theta = OmegaStep;
      aFootPosition.SStime = m_TSsupport;
      aFootPosition.DStime = m_TDsupport;
      m_RelativeFootPositions.push_back(aFootPosition);

#if 1
      DebugFile.open("/tmp/output.txt",ofstream::app);
      DebugFile << aFootPosition.sx<< " "
		<< aFootPosition.sy<< " "
		<< aFootPosition.theta<< " "
		<< endl;
      DebugFile.close();
#endif

      SupportFoot = - SupportFoot;
    }
  if (LastStep!=0.0)
    {
      aFootPosition.sx = LastStep;
      aFootPosition.sy = SupportFoot*0.19;
      aFootPosition.theta = LastOmegaStep;
      aFootPosition.SStime = m_TSsupport;
      aFootPosition.DStime = m_TDsupport;
      
      m_RelativeFootPositions.push_back(aFootPosition);
#if 1
      DebugFile.open("/tmp/output.txt",ofstream::app);
      DebugFile << aFootPosition.sx<< " "
		<< aFootPosition.sy<< " "
		<< aFootPosition.theta<< " "
		<< endl;
      DebugFile.close();
#endif

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
      aFootPosition.SStime = m_TSsupport;
      aFootPosition.DStime = m_TDsupport;
      
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

  VNL::Matrix<double> Romegastep(3,3);
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      if (i==j)
	Romegastep(i,j) =1.0;
      else
	Romegastep(i,j) =0.0;
  Romegastep[0][0] = cosOmegaStep; Romegastep[0][1] = -sinOmegaStep;
  Romegastep[1][0] = sinOmegaStep; Romegastep[1][1] =  cosOmegaStep;

  VNL::Matrix<double> MFNSF(3,3), MFSF(3,3),Romega(3,3),iRomega(3,3),RiR(3,3), FPos(3,3),
    MSupportFoot(3,3),Mtmp(3,3), Mtmp2(3,3);
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
    

  MFSF[0][2]=-R;
  MFSF[1][2]=-S;
  MFNSF[0][2]=-R;
  MFNSF[1][2]=S;
  MSupportFoot=MFSF; 
  Mtmp[1][2] = 0.19;
#if 0
  DebugFile.open("/tmp/outputNL.txt",ofstream::app);
  DebugFile << MSupportFoot[0][2] << " " 
	    << MSupportFoot[1][2] << endl;
  DebugFile.close();
#endif
  
  for(int i=0;i<NumberOfStep;i++)
    {
      double cosiOmegaStep,siniOmegaStep;

      cosiOmegaStep = cos((i+1)*OmegaStep);
      siniOmegaStep = sin((i+1)*OmegaStep);
      
      Romega[0][0] = cosiOmegaStep;
      Romega[0][1] = -siniOmegaStep;
      Romega[1][0] = siniOmegaStep;
      Romega[1][1] = cosiOmegaStep;
      Romega[0][2] = 0;
      Romega[1][2] = 0;

      RiR = VNL::MatrixInverse<double> (MSupportFoot* Romegastep);
      FPos = (RiR * ( Romega * MFNSF));

      aFootPosition.sx = FPos[0][2];
      aFootPosition.sy = FPos[1][2];
      aFootPosition.theta = rad2deg(OmegaStep);
      aFootPosition.SStime = m_TSsupport;
      aFootPosition.DStime = m_TDsupport;

      m_RelativeFootPositions.push_back(aFootPosition);
      MSupportFoot = Romega*MFNSF;
      
#if 0
      DebugFile.open("/tmp/outputL.txt",ofstream::app);
      DebugFile << MSupportFoot[0][2] << " "
		<< MSupportFoot[1][2] << " "
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
      aFootPosition.SStime = m_TSsupport;
      aFootPosition.DStime = m_TDsupport;

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
      MSupportFoot = MSupportFoot * Mtmp;
      
#if 0
      DebugFile.open("/tmp/outputNL.txt",ofstream::app);
      DebugFile << MSupportFoot[0][2] << " " 
		<< MSupportFoot[1][2] << endl;
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
      
      Romega[0][0] = cosiOmegaStep;
      Romega[0][1] = -siniOmegaStep;
      Romega[1][0] = siniOmegaStep;
      Romega[1][1] = cosiOmegaStep;
      Romega[0][2] = 0;
      Romega[1][2] = 0;
      
      double coslOmegaStep,sinlOmegaStep;
      coslOmegaStep = cos(LastOmegaStep);
      sinlOmegaStep = sin(LastOmegaStep);


      iRomega[0][0] = coslOmegaStep;
      iRomega[0][1] = -sinlOmegaStep;
      iRomega[1][0] = sinlOmegaStep;
      iRomega[1][1] = coslOmegaStep;
      
      RiR = VNL::MatrixInverse<double> (MSupportFoot*iRomega);
      FPos = (RiR * ( Romega * MFNSF));

      aFootPosition.sx = FPos[0][2];
      aFootPosition.sy = FPos[1][2];
      aFootPosition.theta = rad2deg(LastOmegaStep);
      aFootPosition.SStime = m_TSsupport;
      aFootPosition.DStime = m_TDsupport;

      m_RelativeFootPositions.push_back(aFootPosition);
      MSupportFoot = Romega*MFNSF;

#if 0
      DebugFile.open("/tmp/outputL.txt",ofstream::app);
      DebugFile << MSupportFoot[0][2] << " "
		<< MSupportFoot[1][2] << " "
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
      aFootPosition.SStime = m_TSsupport;
      aFootPosition.DStime = m_TDsupport;

      m_RelativeFootPositions.push_back(aFootPosition);

#if 0
      DebugFile.open("/tmp/output.txt",ofstream::app);
      DebugFile << aFootPosition.sx<< " "
		<< aFootPosition.sy<< " "
		<< aFootPosition.theta<< " "
		<< endl;
      DebugFile.close();
#endif
      MSupportFoot = MSupportFoot * Mtmp;
     
#if 0
      DebugFile.open("/tmp/outputNL.txt",ofstream::app);
      DebugFile << MSupportFoot[0][2] << " " 
		<< MSupportFoot[1][2] << endl;
      DebugFile.close();
#endif


    }
  m_KeepLastCorrectSupportFoot = -SupportFoot;
}

// Prepare the stack to start a motion on a specific support foot
void StepStackHandler::PrepareForSupportFoot(int SupportFoot)
{
  cout << "PrepareForSupportFoot " << endl;
  RelativeFootPosition aFootPosition;
  aFootPosition.sx = 0;
  aFootPosition.sy = SupportFoot*0.095;
  aFootPosition.theta = 0;
  aFootPosition.SStime = m_TSsupport;
  aFootPosition.DStime = m_TDsupport;

  m_RelativeFootPositions.push_back(aFootPosition);
}

void StepStackHandler::m_PrepareForSupportFoot(istringstream &strm)
{
  int SupportFoot=-1;
  while(!strm.eof())
    {
      if (!strm.eof())
	strm >> SupportFoot;
      else break;
    }  
  PrepareForSupportFoot(SupportFoot);
  
}

void StepStackHandler::StopOnLineStep()
{
  //  m_OnLineSteps = false;
  m_TransitionFinishOnLine=true;

  m_RelativeFootPositions.clear();


}

void StepStackHandler::StartOnLineStep()
{
  if (!m_OnLineSteps)
    m_RelativeFootPositions.clear();

  m_OnLineSteps = true;

  RelativeFootPosition aFootPosition;
  aFootPosition.sx = 0;
  aFootPosition.sy = m_KeepLastCorrectSupportFoot*0.095;
  aFootPosition.theta = 0;
  aFootPosition.SStime = m_TSsupport;
  aFootPosition.DStime = m_TDsupport;
  aFootPosition.stepType = 0;
  
  m_RelativeFootPositions.push_back(aFootPosition);

  m_KeepLastCorrectSupportFoot= -   m_KeepLastCorrectSupportFoot;
  for(int i=0;i<3;i++)
    {
      aFootPosition.sx = 0;
      aFootPosition.sy = m_KeepLastCorrectSupportFoot*0.19;
      aFootPosition.theta = 0;
      aFootPosition.SStime = m_TSsupport;
      aFootPosition.DStime = m_TDsupport;
      aFootPosition.stepType = 0;
  
      m_RelativeFootPositions.push_back(aFootPosition);
	
      m_KeepLastCorrectSupportFoot= -   m_KeepLastCorrectSupportFoot;
    }
  ODEBUG3("StartOnLineStep(): " << m_RelativeFootPositions.size());
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
  if (!m_OnLineSteps)
    return;
  
  if (!NewStep)
    {
      aFootPosition.sx = 0;
      aFootPosition.sy = m_KeepLastCorrectSupportFoot*0.19;
      aFootPosition.theta = 0;
      aFootPosition.SStime = m_TSsupport;
      aFootPosition.DStime = m_TDsupport;
      aFootPosition.stepType = 0;
      
    }
  else
    {

      aFootPosition.sx = NewStepX;
      aFootPosition.sy = NewStepY + m_KeepLastCorrectSupportFoot*0.19;;
      aFootPosition.theta = NewTheta;
      aFootPosition.SStime = m_TSsupport;
      aFootPosition.DStime = m_TDsupport;
      aFootPosition.stepType = 0;
      cout << aFootPosition.sx << " " 
	   << aFootPosition.sy << " " 
	   << aFootPosition.theta << endl;
    }      

  m_RelativeFootPositions.push_back(aFootPosition);
  
  m_KeepLastCorrectSupportFoot= -   m_KeepLastCorrectSupportFoot;
  
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

// Make sure that the previous motion will finish 
// on the last specified correct support foot.
void StepStackHandler::FinishOnTheLastCorrectSupportFoot()
{
  RelativeFootPosition aFootPosition;
  aFootPosition.sx = 0;
  aFootPosition.sy = m_KeepLastCorrectSupportFoot*0.19;
  aFootPosition.theta = 0;
  aFootPosition.SStime = m_TSsupport;
  aFootPosition.DStime = m_TDsupport;
  aFootPosition.stepType = 0;

  m_RelativeFootPositions.push_back(aFootPosition);
}


void StepStackHandler::SetSingleTimeSupport(double lTSsupport)
{
  m_TSsupport = lTSsupport;
}


double StepStackHandler::GetSingleTimeSupport()
{
  return m_TSsupport;
}

void StepStackHandler::SetDoubleTimeSupport(double lTDsupport)
{
  m_TDsupport = lTDsupport;
}

double StepStackHandler::GetDoubleTimeSupport()
{
  return m_TDsupport;
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

      aFootPosition.DStime = m_TDsupport;
      aFootPosition.SStime = m_TSsupport;
      aFootPosition.stepType = 0;
      m_RelativeFootPositions.push_back(aFootPosition);
    }
}


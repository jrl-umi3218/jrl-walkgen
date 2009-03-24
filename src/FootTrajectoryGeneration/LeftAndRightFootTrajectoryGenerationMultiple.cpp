/* @doc This object generate all the values for the foot trajectories.
   @ingroup foottrajectorygeneration
   @endgroup


   Copyright (c) 2007, 
   @author Francois Keith, Olivier Stasse,

   $Id$
   
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
#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "LeftAndRightFootTrajectoryGenerationMultiple :" << x << endl
#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#if 0
#define ODEBUG(x) cerr << "LeftAndRightFootTrajectoryGenerationMultiple :" <<  x << endl
#else
#define ODEBUG(x)
#endif

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); \
    DebugFile << "PGI: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1
#else
#define RESETDEBUG4(y)
#define ODEBUG4(x,y)
#endif

#define ODEBUG6(x,y)

/*! Walking Pattern Generator inclusion */
#include <walkGenJrl/FootTrajectoryGeneration/LeftAndRightFootTrajectoryGenerationMultiple.h>

using namespace PatternGeneratorJRL;

LeftAndRightFootTrajectoryGenerationMultiple::LeftAndRightFootTrajectoryGenerationMultiple(SimplePluginManager *lSPM,
											   dynamicsJRLJapan::HumanoidSpecificities * lHS) : SimplePlugin(lSPM)
{
  m_HS = lHS;

  m_LeftFootTrajectory = new FootTrajectoryGenerationMultiple(lSPM,m_HS);
  m_RightFootTrajectory = new FootTrajectoryGenerationMultiple(lSPM,m_HS);

  string aMethodName[4] = 
    {":omega",":stepheight", ":singlesupporttime",":doublesupporttime"};

  for (int i=0;i<4;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
    }

}

LeftAndRightFootTrajectoryGenerationMultiple::~LeftAndRightFootTrajectoryGenerationMultiple()
{

  if (m_LeftFootTrajectory!=0)
    delete m_LeftFootTrajectory;

  if (m_RightFootTrajectory!=0)
    delete m_RightFootTrajectory;

}

/*! Handling methods for the plugin mecanism. */
void LeftAndRightFootTrajectoryGenerationMultiple::CallMethod(std::string & Method, std::istringstream &strm)
{
  if (Method==":omega")
    {
      strm >> m_Omega;
    }
  else if (Method==":stepheight")
    {
      strm >> m_StepHeight;
    }
  else if (Method==":singlesupporttime")
    {
      strm >> m_SingleSupportTime;
    }
  else if (Method==":doublesupporttime")
    {
      strm >> m_DoubleSupportTime;
    }
}

void LeftAndRightFootTrajectoryGenerationMultiple::SetAnInterval(unsigned int IntervalIndex,
								 FootTrajectoryGenerationMultiple * aFTGM,
								 FootAbsolutePosition &FootInitialPosition,
								 FootAbsolutePosition &FootFinalPosition)
{

  ODEBUG("Set interval " << IntervalIndex << " : " << m_DeltaTj[IntervalIndex] << " X: ("
	  << FootFinalPosition.x << "," << FootInitialPosition.x << "," << FootInitialPosition.dx << ")("
	  << FootFinalPosition.y << "," << FootInitialPosition.y << "," << FootInitialPosition.dy << ")("
	  << FootFinalPosition.z << "," << FootInitialPosition.z << "," << FootInitialPosition.dz << ")");

  // Init the first interval. 
  // X axis.
  aFTGM->SetParametersWithInitPosInitSpeed(IntervalIndex,
					   FootTrajectoryGenerationStandard::X_AXIS,
					   m_DeltaTj[IntervalIndex],
					   FootFinalPosition.x, 
					   FootInitialPosition.x, 
					   FootInitialPosition.dx);
  // Y axis.
  aFTGM->SetParametersWithInitPosInitSpeed(IntervalIndex,
					   FootTrajectoryGenerationStandard::Y_AXIS,
					   m_DeltaTj[IntervalIndex],
					   FootFinalPosition.y, 
					   FootInitialPosition.y, 
					   FootInitialPosition.dy);

  // Z axis.
  aFTGM->SetParametersWithInitPosInitSpeed(IntervalIndex,
					   FootTrajectoryGenerationStandard::Z_AXIS,
					   m_DeltaTj[IntervalIndex],
					   FootFinalPosition.z, 
					   FootInitialPosition.z, 
					   FootInitialPosition.dz);

  // THETA
  aFTGM->SetParametersWithInitPosInitSpeed(IntervalIndex,
					   FootTrajectoryGenerationStandard::THETA_AXIS,
					   m_DeltaTj[IntervalIndex],
					   FootFinalPosition.theta, 
					   FootInitialPosition.theta, 
					   FootInitialPosition.dtheta);
  
  // Omega 
  aFTGM->SetParametersWithInitPosInitSpeed(IntervalIndex,
					   FootTrajectoryGenerationStandard::OMEGA_AXIS,
					   m_DeltaTj[IntervalIndex],
					   FootFinalPosition.omega, 
					   FootInitialPosition.omega, 
					   FootInitialPosition.domega);
  
  // Omega 2
  aFTGM->SetParametersWithInitPosInitSpeed(IntervalIndex,
					   FootTrajectoryGenerationStandard::OMEGA2_AXIS,
					   m_DeltaTj[IntervalIndex],
					   FootFinalPosition.omega2, 
					   FootInitialPosition.omega2, 
					   FootInitialPosition.domega2);
}

void LeftAndRightFootTrajectoryGenerationMultiple::
InitializeFromRelativeSteps(deque<RelativeFootPosition> &RelativeFootPositions,
			    FootAbsolutePosition &LeftFootInitialPosition,
			    FootAbsolutePosition &RightFootInitialPosition,
			    deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions,
			    bool IgnoreFirst, bool Continuity)
{

  /*! Makes sure the size of the SupportFootAbsolutePositions is the same than
   the relative foot positions. */
  if (SupportFootAbsoluteFootPositions.size()!=
      RelativeFootPositions.size())
    SupportFootAbsoluteFootPositions.resize(RelativeFootPositions.size());

  unsigned int lNbOfIntervals = RelativeFootPositions.size(); 
  /*! It is assumed that a set of relative positions for the support foot
    are given as an input. */
  deque<FootAbsolutePosition> AbsoluteFootPositions;

  /*! Those two variables are needed to compute intermediate 
    initial positions for the feet. */
  FootAbsolutePosition LeftFootTmpInitPos,RightFootTmpInitPos;
  /*! Those two variables are needed to compute intermediate 
    final positions for the feet. */
  FootAbsolutePosition LeftFootTmpFinalPos,RightFootTmpFinalPos;
  
  AbsoluteFootPositions.resize(lNbOfIntervals);
  lNbOfIntervals = 2*lNbOfIntervals+1;
  
  /*! Resize the Left and Right foot trajectories. */
  m_LeftFootTrajectory->SetNumberOfIntervals(lNbOfIntervals);
  m_RightFootTrajectory->SetNumberOfIntervals(lNbOfIntervals);
  ODEBUG("resize left and right foot trajectories: " << lNbOfIntervals);

  /*! Compute the absolute coordinates of the steps.  */
  double CurrentAbsTheta=0.0,c=0.0,s=0.0;
  MAL_MATRIX_DIM(MM,double,2,2);
  MAL_MATRIX_DIM(CurrentSupportFootPosition,double,3,3);
  MAL_MATRIX_SET_IDENTITY(CurrentSupportFootPosition);
  MAL_MATRIX_DIM(Orientation,double,2,2);
  MAL_MATRIX_SET_IDENTITY(Orientation);
  MAL_MATRIX_DIM(v,double,2,1);
  MAL_MATRIX_DIM(v2,double,2,1);
  
  if (m_DeltaTj.size()!=lNbOfIntervals)
    m_DeltaTj.resize(lNbOfIntervals);

  /*! Who is the first support foot. */
  int SupportFoot=1; // Left
  
  if ( 
      // The flying foot is on the left, thus the support foot is on the right.
      // and this is not the beginning of the stepping.
      ((RelativeFootPositions[0].sy>0) && (IgnoreFirst==false)) ||
      // There is no flying foot because this is the beginning of the stepping.
      ((RelativeFootPositions[0].sy<0) && (IgnoreFirst==true))
      )
    {
      ODEBUG("Detect support foot on the right.");
      SupportFoot=-1; 
      CurrentAbsTheta = RightFootInitialPosition.theta;
      v2(0,0) = RightFootInitialPosition.x;
      v2(1,0) = RightFootInitialPosition.y;
    }
  else
    {
      ODEBUG("Detect support foot on the left.");
      CurrentAbsTheta = LeftFootInitialPosition.theta;
      v2(0,0) = LeftFootInitialPosition.x;
      v2(1,0) = LeftFootInitialPosition.y;
      
    }
  ODEBUG("Support Foot : " << v2(0,0) << " " << v2(1,0) << " " << CurrentAbsTheta);
  
  // Initial Position of the current support foot.
  c = cos(CurrentAbsTheta*M_PI/180.0);
  s = sin(CurrentAbsTheta*M_PI/180.0);
  MM(0,0) = Orientation(0,0) = c;      MM(0,1) = Orientation(0,1) = -s;
  MM(1,0) = Orientation(1,0) = s;      MM(1,1) = Orientation(1,1) = c;

  for(int k=0;k<2;k++)
    for(int l=0;l<2;l++)
      CurrentSupportFootPosition(k,l) = MM(k,l);
  
  for(int k=0;k<2;k++)
    CurrentSupportFootPosition(k,2) = v2(k,0);
  
  
  /*! Initialize the temporary initial position. */
  
  LeftFootTmpInitPos = LeftFootInitialPosition;
  RightFootTmpInitPos = RightFootInitialPosition;

  /* Keep track of the interval index once this is
     for single support, once for double support */
  int IntervalIndex=0;
  ODEBUG("LeftFootTmpInitPos.x " << LeftFootTmpInitPos.x << endl << 
	  "LeftFootTmpInitPos.y " << LeftFootTmpInitPos.y << endl << 
	  "LeftFootTmpInitPos.z " << LeftFootTmpInitPos.z << endl << 
	  "LeftFootTmpInitPos.dx " << LeftFootTmpInitPos.dx << endl << 
	  "LeftFootTmpInitPos.dy " << LeftFootTmpInitPos.dy << endl << 
	  "LeftFootTmpInitPos.dz " << LeftFootTmpInitPos.dz << endl );

  ODEBUG("RightFootTmpInitPos.x " << RightFootTmpInitPos.x << endl << 
	  "RightFootTmpInitPos.y " << RightFootTmpInitPos.y << endl << 
	  "RightFootTmpInitPos.z " << RightFootTmpInitPos.z << endl << 
	  "RightFootTmpInitPos.dx " << RightFootTmpInitPos.dx << endl << 
	  "RightFootTmpInitPos.dy " << RightFootTmpInitPos.dy << endl << 
	  "RightFootTmpInitPos.dz " << RightFootTmpInitPos.dz << endl );

  ODEBUG("CurrentSupportFootPosition: " << CurrentSupportFootPosition);
  ODEBUG("RelativeFootPositions: " << RelativeFootPositions.size());
  for(unsigned int i=0;i<RelativeFootPositions.size();i++)
    {
      if (i!=0)
	{
	  /*! At this stage the phase of double support is deal with */
	  ODEBUG("Double support phase");
	  LeftFootTmpInitPos.z = 0;
	  SetAnInterval(IntervalIndex,m_LeftFootTrajectory,
			LeftFootTmpInitPos,
			LeftFootTmpInitPos);
	  RightFootTmpInitPos.z = 0;
	  SetAnInterval(IntervalIndex,m_RightFootTrajectory,
			RightFootTmpInitPos,
			RightFootTmpInitPos);
	  IntervalIndex++;

	  
	  ODEBUG("It: " << i << " Double support Phase :" << endl <<
		  "\t Init Left: ( " << 
		  LeftFootTmpInitPos.x << " , " << 
		  LeftFootTmpInitPos.y << " ) " << 
		  endl << "Right : ( " << 
		  RightFootTmpInitPos.x << " , " << 
		  RightFootTmpInitPos.y << " ) " << endl <<
		  "\t Final Left: ( " << 
		  LeftFootTmpFinalPos.x << " , " << 
		  LeftFootTmpFinalPos.y << " ) " << 
		  endl << "Right : ( " << 
		  RightFootTmpFinalPos.x << " , " << 
		  RightFootTmpFinalPos.y << " ) " << endl <<
		  "\t RelativeFootPosition: ( " <<
		  RelativeFootPositions[i].sx << " , " <<
		  RelativeFootPositions[i].sy << " , " <<
		  RelativeFootPositions[i].sx << " , " <<
		  RelativeFootPositions[i].theta << " )");
	}
	      
      /*! Compute Orientation matrix related to the relative orientation
	of the support foot */
      c = cos(RelativeFootPositions[i].theta*M_PI/180.0);
      s = sin(RelativeFootPositions[i].theta*M_PI/180.0);
      MM(0,0) = c;      MM(0,1) = -s;
      MM(1,0) = s;      MM(1,1) = c;
      
      /*! Update the orientation */	  
      CurrentAbsTheta+= RelativeFootPositions[i].theta;
      CurrentAbsTheta = fmod(CurrentAbsTheta,180.0);
	
      /*! Extract the current absolute orientation matrix. */
      for(int k=0;k<2;k++)
	for(int l=0;l<2;l++)
	  Orientation(k,l) = CurrentSupportFootPosition(k,l);
	
      /*! Put in a vector form the translation of the relative foot. */
      v(0,0) = RelativeFootPositions[i].sx;
      v(1,0) = RelativeFootPositions[i].sy;

      /*! Compute the new orientation of the foot vector. */
      Orientation = MAL_RET_A_by_B(MM , Orientation);
      v2 = MAL_RET_A_by_B(Orientation, v);
	
      /*! Update the world coordinates of the support foot. */
      if ((!IgnoreFirst) || (i>0))
	{
	  for(int k=0;k<2;k++)
	    for(int l=0;l<2;l++)
	      CurrentSupportFootPosition(k,l) = Orientation(k,l);
	  
	  for(int k=0;k<2;k++)
	    CurrentSupportFootPosition(k,2) += v2(k,0);
	}

      AbsoluteFootPositions[i].x = CurrentSupportFootPosition(0,2);
      AbsoluteFootPositions[i].y = CurrentSupportFootPosition(1,2);
      AbsoluteFootPositions[i].theta = CurrentAbsTheta;
      
      ODEBUG("CSFP:" << CurrentSupportFootPosition(0,2) << " " << CurrentSupportFootPosition(1,2));

      /*! We deal with the single support phase,
	i.e. the target of the next single support phase
	is the current target of the swinging foot. */
      if ((!IgnoreFirst) || (i>0))
	{
	  if (SupportFoot==1)
	    {
	      /*! The current support foot is the left one.*/
	      RightFootTmpFinalPos.x = CurrentSupportFootPosition(0,2);
	      RightFootTmpFinalPos.y = CurrentSupportFootPosition(1,2);
	      RightFootTmpFinalPos.z = m_StepHeight;
	      RightFootTmpFinalPos.theta = CurrentAbsTheta;
	      RightFootTmpFinalPos.omega = m_Omega;
	      RightFootTmpFinalPos.dx = 0.0;
	      RightFootTmpFinalPos.dy = 0.0;
	      RightFootTmpFinalPos.dz = 0.0;
	      RightFootTmpFinalPos.dtheta = 0.0;
	      RightFootTmpFinalPos.domega = 0.0;
	      
	      LeftFootTmpFinalPos = LeftFootTmpInitPos;
	      LeftFootTmpFinalPos.z = 0.0;
	    }
	  else
	    {
	      /*! The current support foot is the right one.*/
	      LeftFootTmpFinalPos.x = CurrentSupportFootPosition(0,2);
	      LeftFootTmpFinalPos.y = CurrentSupportFootPosition(1,2);
	      LeftFootTmpFinalPos.z = m_StepHeight;
	      LeftFootTmpFinalPos.theta = CurrentAbsTheta;
	      LeftFootTmpFinalPos.omega = m_Omega;
	      LeftFootTmpFinalPos.dx = 0.0;
	      LeftFootTmpFinalPos.dy = 0.0;
	      LeftFootTmpFinalPos.dz = 0.0;
	      LeftFootTmpFinalPos.dtheta = 0.0;
	      LeftFootTmpFinalPos.domega = 0.0;

	      RightFootTmpFinalPos = RightFootTmpInitPos;
	      RightFootTmpFinalPos.z = 0.0;
	      
	    }
	}
      else
	{
	  LeftFootTmpFinalPos = LeftFootTmpInitPos;
	  LeftFootTmpFinalPos.z = 0.0;
	  LeftFootTmpFinalPos.dx = LeftFootTmpInitPos.dx = 0.0;
	  LeftFootTmpFinalPos.dy = LeftFootTmpInitPos.dy =0.0;
	  LeftFootTmpFinalPos.dz = LeftFootTmpInitPos.dz =0.0;
	      

	  RightFootTmpFinalPos = RightFootTmpInitPos;
	  RightFootTmpFinalPos.z = 0.0;
	  RightFootTmpFinalPos.dx = RightFootTmpInitPos.dx = 0.0;
	  RightFootTmpFinalPos.dy = RightFootTmpInitPos.dy =0.0;
	  RightFootTmpFinalPos.dz = RightFootTmpInitPos.dz =0.0;
	  
	}
      
      if ((i!=0)|| (Continuity))
	{
	  /* Initialize properly the interval in single support phase */
	  ODEBUG("Single support phase");
	  SetAnInterval(IntervalIndex,m_LeftFootTrajectory,
			LeftFootTmpInitPos,
			LeftFootTmpFinalPos);
	  
	  ODEBUG("LeftInit: ( " << LeftFootTmpInitPos.x << " , " 
		 << LeftFootTmpInitPos.y << " , " 
		 << LeftFootTmpInitPos.z << " ) ( " 
		 << LeftFootTmpInitPos.dx << " , " 
		 << LeftFootTmpInitPos.dy << " , " 
		 << LeftFootTmpInitPos.dz << " ) "
		 << endl << "LeftFinal : ( " 
		 << LeftFootTmpFinalPos.x << " , " 
		 << LeftFootTmpFinalPos.y << " , " 
		 << LeftFootTmpFinalPos.z << " ) ( " 
		 << LeftFootTmpFinalPos.dx << " , " 
		 << LeftFootTmpFinalPos.dy << " , " 
		 << LeftFootTmpFinalPos.dz << " ) " );
	  
	  SetAnInterval(IntervalIndex,m_RightFootTrajectory,
			RightFootTmpInitPos,
			RightFootTmpFinalPos);

	  ODEBUG("RightInit: ( " << RightFootTmpInitPos.x << " , " 
		 << RightFootTmpInitPos.y << " , " 
		 << RightFootTmpInitPos.z << " ) ( " 
		 << RightFootTmpInitPos.dx << " , " 
		 << RightFootTmpInitPos.dy << " , " 
		 << RightFootTmpInitPos.dz << " ) "
		 << endl << "RightFinal : ( " 
		 << RightFootTmpFinalPos.x << " , " 
		 << RightFootTmpFinalPos.y << " , " 
		 << RightFootTmpFinalPos.z << " ) ( " 
		 << RightFootTmpFinalPos.dx << " , " 
		 << RightFootTmpFinalPos.dy << " , " 
		 << RightFootTmpFinalPos.dz << " ) " );
	  
	  // Switch from single support to double support.
	  IntervalIndex++;
	}

      if ((!Continuity) && ((i==0) || (i==RelativeFootPositions.size()-1)))
	  {
	    /*! At this stage the phase of double support is deal with */
	    unsigned int limitk=1;

	    /*! If we are at the end a second double support phase has to be added. */
	    if (i==RelativeFootPositions.size()-1)
		limitk=2;

	    for(unsigned int lk=0;lk<limitk;lk++)
	      {
		LeftFootTmpFinalPos.z = 0;
		SetAnInterval(IntervalIndex,m_LeftFootTrajectory,
			      LeftFootTmpFinalPos,
			      LeftFootTmpFinalPos);
		RightFootTmpFinalPos.z = 0;
		SetAnInterval(IntervalIndex,m_RightFootTrajectory,
			      RightFootTmpFinalPos,
			      RightFootTmpFinalPos);
		IntervalIndex++;
	      }
	  }


      /* The final position become the new initial position */
      LeftFootTmpInitPos = LeftFootTmpFinalPos;
      RightFootTmpInitPos = RightFootTmpFinalPos;
      
      
      /* Populate the set of support foot absolute positions */
      SupportFootAbsoluteFootPositions[i].x = CurrentSupportFootPosition(0,2);
      SupportFootAbsoluteFootPositions[i].y = CurrentSupportFootPosition(1,2);
      SupportFootAbsoluteFootPositions[i].theta = CurrentAbsTheta;

      if ((!IgnoreFirst) || (i>0))
	SupportFoot=-SupportFoot;
    }
}

void LeftAndRightFootTrajectoryGenerationMultiple::
ComputeAbsoluteStepsFromRelativeSteps(deque<RelativeFootPosition> &RelativeFootPositions,
				      FootAbsolutePosition &LeftFootInitialPosition,
				      FootAbsolutePosition &RightFootInitialPosition,
				      deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions)
{
  FootAbsolutePosition aSupportFootAbsolutePosition;

  if (RelativeFootPositions[0].sy>0) 
    {
      // The flying foot is on the left, thus the support foot is on the right.
      ODEBUG("Detect support foot on the right.");
      aSupportFootAbsolutePosition = RightFootInitialPosition;
    }
  else
    {
      ODEBUG("Detect support foot on the left.");
      aSupportFootAbsolutePosition = LeftFootInitialPosition;      
    }  
  ComputeAbsoluteStepsFromRelativeSteps(RelativeFootPositions,
					aSupportFootAbsolutePosition,
					SupportFootAbsoluteFootPositions);

}
void LeftAndRightFootTrajectoryGenerationMultiple::
ComputeAbsoluteStepsFromRelativeSteps(deque<RelativeFootPosition> &RelativeFootPositions,
				      FootAbsolutePosition &SupportFootInitialAbsolutePosition,
				      deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions)
{
  /*! Makes sure the size of the SupportFootAbsolutePositions is the same than
   the relative foot positions. */
  if (SupportFootAbsoluteFootPositions.size()!=
      RelativeFootPositions.size())
    SupportFootAbsoluteFootPositions.resize(RelativeFootPositions.size());

  unsigned int lNbOfIntervals = RelativeFootPositions.size(); 
  /*! It is assumed that a set of relative positions for the support foot
    are given as an input. */
  deque<FootAbsolutePosition> AbsoluteFootPositions;
  
  AbsoluteFootPositions.resize(lNbOfIntervals);
  lNbOfIntervals = 2*lNbOfIntervals+1;
  
  /*! Compute the absolute coordinates of the steps.  */
  double CurrentAbsTheta=0.0,c=0.0,s=0.0;
  MAL_MATRIX_DIM(MM,double,2,2);
  MAL_MATRIX_DIM(CurrentSupportFootPosition,double,3,3);
  MAL_MATRIX_SET_IDENTITY(CurrentSupportFootPosition);
  MAL_MATRIX_DIM(Orientation,double,2,2);
  MAL_MATRIX_SET_IDENTITY(Orientation);
  MAL_MATRIX_DIM(v,double,2,1);
  MAL_MATRIX_DIM(v2,double,2,1);
  
  ODEBUG("Detect support foot on the right.");
  CurrentAbsTheta = SupportFootInitialAbsolutePosition.theta;
  v2(0,0) = SupportFootInitialAbsolutePosition.x;
  v2(1,0) = SupportFootInitialAbsolutePosition.y;
  
  // Initial Position of the current support foot.
  c = cos(CurrentAbsTheta*M_PI/180.0);
  s = sin(CurrentAbsTheta*M_PI/180.0);
  MM(0,0) = Orientation(0,0) = c;      MM(0,1) = Orientation(0,1) = -s;
  MM(1,0) = Orientation(1,0) = s;      MM(1,1) = Orientation(1,1) = c;

  for(int k=0;k<2;k++)
    for(int l=0;l<2;l++)
      CurrentSupportFootPosition(k,l) = MM(k,l);
  
  for(int k=0;k<2;k++)
    CurrentSupportFootPosition(k,2) = v2(k,0);
  
  /* Keep track of the interval index once this is
     for single support, once for double support */

  ODEBUG("CurrentSupportFootPosition: " << CurrentSupportFootPosition);
  ODEBUG("RelativeFootPositions: " << RelativeFootPositions.size());

  for(unsigned int i=0;i<RelativeFootPositions.size();i++)
    {

      ODEBUG( i << " : " << 
	       RelativeFootPositions[i].sx << " " << 
	       RelativeFootPositions[i].sy);
      
      ODEBUG("Double support phase -- Left: ( " << 
	      LeftFootTmpInitPos.x << " , " << 
	      LeftFootTmpInitPos.y << " ) " << "Right : ( " << 
	      RightFootTmpInitPos.x << " , " << 
	      RightFootTmpInitPos.y << " ) " );
	      
      /*! Compute Orientation matrix related to the relative orientation
	of the support foot */
      c = cos(RelativeFootPositions[i].theta*M_PI/180.0);
      s = sin(RelativeFootPositions[i].theta*M_PI/180.0);
      MM(0,0) = c;      MM(0,1) = -s;
      MM(1,0) = s;      MM(1,1) = c;
	
      /*! Update the orientation */
      CurrentAbsTheta+= RelativeFootPositions[i].theta;
      CurrentAbsTheta = fmod(CurrentAbsTheta,180.0);
	
      /*! Extract the current absolute orientation matrix. */
      for(int k=0;k<2;k++)
	for(int l=0;l<2;l++)
	  Orientation(k,l) = CurrentSupportFootPosition(k,l);
	
      /*! Put in a vector form the translation of the relative foot. */
      v(0,0) = RelativeFootPositions[i].sx;
      v(1,0) = RelativeFootPositions[i].sy;

      /*! Compute the new orientation of the foot vector. */
      Orientation = MAL_RET_A_by_B(MM , Orientation);
      v2 = MAL_RET_A_by_B(Orientation, v);
	
      /*! Update the world coordinates of the support foot. */
      for(int k=0;k<2;k++)
	for(int l=0;l<2;l++)
	  CurrentSupportFootPosition(k,l) = Orientation(k,l);
	
      for(int k=0;k<2;k++)
	CurrentSupportFootPosition(k,2) += v2(k,0);

      AbsoluteFootPositions[i].x = CurrentSupportFootPosition(0,2);
      AbsoluteFootPositions[i].y = CurrentSupportFootPosition(1,2);
      AbsoluteFootPositions[i].theta = CurrentAbsTheta;
      
      ODEBUG("CSFP:" << CurrentSupportFootPosition(0,2) << " " << CurrentSupportFootPosition(1,2));
            
      /* Populate the set of support foot absolute positions */
      SupportFootAbsoluteFootPositions[i].x = CurrentSupportFootPosition(0,2);
      SupportFootAbsoluteFootPositions[i].y = CurrentSupportFootPosition(1,2);
      SupportFootAbsoluteFootPositions[i].theta = CurrentAbsTheta;

    }
}

void LeftAndRightFootTrajectoryGenerationMultiple::
ChangeRelStepsFromAbsSteps(deque<RelativeFootPosition> &RelativeFootPositions,
 			   FootAbsolutePosition &SupportFootInitialPosition,
			   deque<FootAbsolutePosition> &SupportFootAbsoluteFootPositions,
			   unsigned int ChangedInterval)
{
  if (ChangedInterval>=SupportFootAbsoluteFootPositions.size())
    {
      ODEBUG3("Pb: ChangedInterval is after the size of absolute foot stack.");
      return; 
    }

  MAL_S3x3_MATRIX(KM1,double);
  MAL_S3x3_MATRIX_SET_IDENTITY(KM1);
  MAL_S3x3_MATRIX(K,double);
  MAL_S3x3_MATRIX_SET_IDENTITY(K);
  MAL_S3x3_MATRIX(KP1,double);
  MAL_S3x3_MATRIX_SET_IDENTITY(KP1);
  
  double thetakm1,xkm1,ykm1,c,s;

  // Change the previous relative position.
  if (ChangedInterval==0)
    {
      thetakm1= SupportFootInitialPosition.theta;
      xkm1 = SupportFootInitialPosition.x;
      ykm1 = SupportFootInitialPosition.y;
    }
  else 
    {
      thetakm1= SupportFootAbsoluteFootPositions[ChangedInterval-1].theta;
      xkm1 = SupportFootAbsoluteFootPositions[ChangedInterval-1].x;
      ykm1 = SupportFootAbsoluteFootPositions[ChangedInterval-1].y;
    }
  ODEBUG("Changed interval: " << ChangedInterval);
  ODEBUG("K-1 position: " << xkm1 << " " << ykm1 << " " << thetakm1 );
  c = cos(thetakm1*M_PI/180.0);
  s = sin(thetakm1*M_PI/180.0);
  KM1(0,0) = c;      KM1(0,1) = -s; KM1(0,2) = xkm1;
  KM1(1,0) = s;      KM1(1,1) = c;  KM1(1,2) = ykm1;

  double thetak,xk,yk;
  thetak= SupportFootAbsoluteFootPositions[ChangedInterval].theta;
  xk = SupportFootAbsoluteFootPositions[ChangedInterval].x;
  yk = SupportFootAbsoluteFootPositions[ChangedInterval].y;

  ODEBUG("K position: " << xk << " " << yk << " " << thetak );
  c = cos(thetak*M_PI/180.0);
  s = sin(thetak*M_PI/180.0);
  K(0,0) = c;      K(0,1) = -s; K(0,2) = xk;
  K(1,0) = s;      K(1,1) = c;  K(1,2) = yk;
  
  MAL_S3x3_MATRIX(iKM1,double);
  MAL_S3x3_INVERSE(KM1,iKM1,double);
  MAL_S3x3_MATRIX(relMotionM1,double);

  MAL_S3x3_C_eq_A_by_B(relMotionM1,iKM1,K);

  RelativeFootPositions[ChangedInterval].sx = relMotionM1(0,2);
  RelativeFootPositions[ChangedInterval].sy = relMotionM1(1,2);
  RelativeFootPositions[ChangedInterval].theta = atan2(relMotionM1(1,0),relMotionM1(0,0));

  double thetakp1,xkp1,ykp1;

  // Change the next relative position
  if (ChangedInterval<SupportFootAbsoluteFootPositions.size()-1)
    {
      thetakp1= SupportFootAbsoluteFootPositions[ChangedInterval+1].theta;
      xkp1 = SupportFootAbsoluteFootPositions[ChangedInterval+1].x;
      ykp1 = SupportFootAbsoluteFootPositions[ChangedInterval+1].y;
      
      c = cos(thetakp1*M_PI/180.0);
      s = sin(thetakp1*M_PI/180.0);
      KP1(0,0) = c;      KP1(0,1) = -s; KP1(0,2) = xkp1;
      KP1(1,0) = s;      KP1(1,1) = c;  KP1(1,2) = ykp1;
      
      MAL_S3x3_MATRIX(iK,double);
      MAL_S3x3_INVERSE(K,iK,double);
      MAL_S3x3_MATRIX(relMotionP1,double);
  
      MAL_S3x3_C_eq_A_by_B(relMotionP1,iK,KP1);
      
      RelativeFootPositions[ChangedInterval+1].sx = relMotionP1(0,2);
      RelativeFootPositions[ChangedInterval+1].sy = relMotionP1(1,2);
      RelativeFootPositions[ChangedInterval+1].theta = atan2(relMotionP1(1,0),relMotionP1(0,0));
      
    }

  ODEBUG("KP1 position: " << xkp1 << " " << ykp1 << " " << thetakp1 );
  ODEBUG("Changed intervals : " << ChangedInterval-1 << " " << ChangedInterval << " " << ChangedInterval + 1);
}

void LeftAndRightFootTrajectoryGenerationMultiple::ComputeAnAbsoluteFootPosition(int LeftOrRight,
										 double time,
										 FootAbsolutePosition & aFAP)
{

  ODEBUG("Left (1) or right (-1) : " <<  LeftOrRight);

  if (LeftOrRight==1)
    m_LeftFootTrajectory->Compute(time,aFAP);
  else 
    m_RightFootTrajectory->Compute(time,aFAP);

}

void LeftAndRightFootTrajectoryGenerationMultiple::ComputeAnAbsoluteFootPosition(int LeftOrRight,
										 double time,
										 FootAbsolutePosition & aFAP,
										 unsigned int IndexInterval)
{

  ODEBUG(this << " " << m_LeftFootTrajectory << " " << m_RightFootTrajectory);

  if (LeftOrRight==1)
    m_LeftFootTrajectory->Compute(time,aFAP,IndexInterval);
  else 
    m_RightFootTrajectory->Compute(time,aFAP,IndexInterval);

}

void LeftAndRightFootTrajectoryGenerationMultiple::SetDeltaTj(vector<double> &aDeltaTj)
{
  ODEBUG("SetDeltaTj :" << aDeltaTj.size() << " " << aDeltaTj[0]);

  m_DeltaTj = aDeltaTj;
  if (m_LeftFootTrajectory!=0)
    m_LeftFootTrajectory->SetTimeIntervals(m_DeltaTj);
  if (m_RightFootTrajectory!=0)
    m_RightFootTrajectory->SetTimeIntervals(m_DeltaTj);
    
}

void LeftAndRightFootTrajectoryGenerationMultiple::GetDeltaTj(vector<double> &aDeltaTj)
{
  aDeltaTj = m_DeltaTj;
}

void LeftAndRightFootTrajectoryGenerationMultiple::SetStepHeight(double aStepHeight)
{
  m_StepHeight = aStepHeight;
}

double LeftAndRightFootTrajectoryGenerationMultiple::GetStepHeight()
{
  return m_StepHeight;
}

double LeftAndRightFootTrajectoryGenerationMultiple::GetAbsoluteTimeReference()
{
  double res=0.0;
  double LeftATR=0.0,RightATR=0.0;
  if (m_LeftFootTrajectory!=0)
    LeftATR = m_LeftFootTrajectory->GetAbsoluteTimeReference();
  if (m_RightFootTrajectory!=0)
    RightATR = m_RightFootTrajectory->GetAbsoluteTimeReference();
  if (LeftATR!=RightATR)
    res=-1;
  else
    res = LeftATR;
  
  return res;
}

void LeftAndRightFootTrajectoryGenerationMultiple::SetAbsoluteTimeReference(double anATR)
{
  if (m_LeftFootTrajectory!=0)
    m_LeftFootTrajectory->SetAbsoluteTimeReference(anATR);
  if (m_RightFootTrajectory!=0)
    m_RightFootTrajectory->SetAbsoluteTimeReference(anATR);
}

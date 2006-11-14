/** This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of steps.

   CVS Information: 
   $Id: ZMPDiscretization.cpp,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/ZMPDiscretization.cpp,v $
   $Log: ZMPDiscretization.cpp,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of t IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   he contributors, the documentation
   and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
   Bjorn Verrelst
   Olivier Stasse,
   Ramzi Sellouati
   
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
//#define _DEBUG_
#include <fstream>
#include <StepOverPlanner.h>


using namespace::PatternGeneratorJRL;

StepOverPlanner::StepOverPlanner(ObstaclePar &ObstacleParameters,
				 HumanoidSpecificities *aHS)
{

  m_HS = aHS;
  // Get information specific to the humanoid.
  double lWidth,lHeight;
  double AnklePosition[3];

  if (m_HS!=0)
    {
      m_HS->GetFootSize(-1,lWidth,lHeight);
      m_HS->GetAnklePosition(-1,AnklePosition);
      m_AnkleSoilDistance = AnklePosition[2];
      m_tipToAnkle = lWidth-AnklePosition[0];
      m_heelToAnkle = m_AnkleSoilDistance;	
  
    }
  else 
    {
      lWidth = 0.2; lHeight=0.15;
      cerr << "WARNING: no object with humanoid specificities properly defined." << endl;
      m_AnkleSoilDistance = 0.1;
      m_tipToAnkle = 0.1;
      m_heelToAnkle = 0.1;
  
    }

  m_CollDet =0;

  SetObstacleInformation(ObstacleParameters);

		
  m_soleToAnkle = m_AnkleSoilDistance;
	
  m_heelDistAfter = 0.0;
  m_tipDistBefore = 0.0;
	
  m_nominalStepLenght = 0.2;
  m_nominalStepWidth = 0.19;


  //this angle is used to limit the position during feasibility in douyble support over the obstacle
  m_KneeAngleBound=15.0*M_PI/180.0;
	
  m_Tsingle = 0.78; 
  m_Tdble = 0.02;

  m_TsingleStepOver = 1.5;
  m_TdbleStepOver = 0.04;

		
  m_TsingleStepOverBeforeAfter = m_TsingleStepOver*1.0;
  m_TdbleStepOverBeforeAfter = m_TdbleStepOver*1.0;

	
  m_WaistRotationStepOver = 25.0;
	
	
  m_PolynomeStepOverHipStep2 = new StepOverPolynomeHip4();

  m_PolynomeStepOverHipRotation = new StepOverPolynomeHip4();	
	
  m_ClampedCubicSplineStepOverFootX = new StepOverClampedCubicSpline ;
  m_ClampedCubicSplineStepOverFootY = new StepOverClampedCubicSpline ;
  m_ClampedCubicSplineStepOverFootZ = new StepOverClampedCubicSpline ;
  m_ClampedCubicSplineStepOverFootOmega = new StepOverClampedCubicSpline ;
  m_ClampedCubicSplineStepOverFootOmegaImpact = new StepOverClampedCubicSpline ;		
	
  m_DMB = 0;
  m_PC = 0;
  m_IK = 0;
  m_ZMPDiscr = 0;
	
  m_Dt.Resize(3,1);
  // Displacement between the hip and RLINK2
  // WARNING : Specific to HRP2 !
  m_Dt(0,0) = 0.0;
  m_Dt(1,0) = 0.035;
  m_Dt(2,0) = 0.0;
	
  m_StaticToTheLeftHip.Resize(3,1);
  m_StaticToTheRightHip.Resize(3,1);
  // Displacement between the COM and the waist
  // WARNING : Specific to HRP2 !
  m_DiffBetweenComAndWaist = -0.1656;//-0.145184;
	
	
  m_StaticToTheLeftHip(0,0) = 0.0;
  m_StaticToTheLeftHip(1,0) = 0.06;
  m_StaticToTheLeftHip(2,0) = m_DiffBetweenComAndWaist;
	
  m_StaticToTheRightHip(0,0) = 0.0;
  m_StaticToTheRightHip(1,0) = -0.06;
  m_StaticToTheRightHip(2,0) = m_DiffBetweenComAndWaist;

  // defining the points on the shank to set the boundary lines of the leg layout
  // for the values of the coordinates see paper guan san IROS 2004 
  // 'feasibility of humanoid robots stepping over obstacles'

  double RadiusKnee;
  double Angle1,Angle2;
	
  RadiusKnee=0.118;
  Angle1=60.0/180.0*M_PI;
  Angle2=30.0/180.0*M_PI;
	
  m_LegLayoutPoint.Resize(3,7);
	
  //point1	
  m_LegLayoutPoint(0,0)= RadiusKnee*cos(Angle1);
  m_LegLayoutPoint(1,0)= 0.0;
  m_LegLayoutPoint(2,0)= RadiusKnee*sin(Angle1);

  //point2
  m_LegLayoutPoint(0,1)= RadiusKnee*cos(Angle2);
  m_LegLayoutPoint(1,1)= 0.0;
  m_LegLayoutPoint(2,1)= RadiusKnee*sin(Angle2);

  //point3
  m_LegLayoutPoint(0,2)= RadiusKnee;
  m_LegLayoutPoint(1,2)= 0.0;
  m_LegLayoutPoint(2,2)= 0.0;
	
  //point4
  m_LegLayoutPoint(0,3)= 0.0772;
  m_LegLayoutPoint(1,3)= 0.0;
  m_LegLayoutPoint(2,3)=-0.2253;

  //point5
  m_LegLayoutPoint(0,4)=-0.0772;
  m_LegLayoutPoint(1,4)= 0.0;
  m_LegLayoutPoint(2,4)=-0.2492;

  //point6
  m_LegLayoutPoint(0,5)=-0.0163;
  m_LegLayoutPoint(1,5)= 0.0;
  m_LegLayoutPoint(2,5)=-0.0939;

  //point7
  m_LegLayoutPoint(0,6)=-0.0322;
  m_LegLayoutPoint(1,6)= 0.0;
  m_LegLayoutPoint(2,6)=-0.0183;

  m_TimeDistrFactor.resize(4);
	
  m_TimeDistrFactor[0]=2.0;
  m_TimeDistrFactor[1]=3.7;
  m_TimeDistrFactor[2]=1.0;
  m_TimeDistrFactor[3]=3.0;	
     	
  m_DeltaStepOverCOMHeightMax = 0.0; 

  
}

StepOverPlanner::~StepOverPlanner()
{
		
  if (m_PolynomeStepOverHipRotation!=0)
    delete m_PolynomeStepOverHipRotation;

  if (m_PolynomeStepOverHipStep2!=0)
    delete m_PolynomeStepOverHipStep2;

  if (m_ClampedCubicSplineStepOverFootX!=0)
    delete m_ClampedCubicSplineStepOverFootX;

  if (m_ClampedCubicSplineStepOverFootY!=0)
    delete m_ClampedCubicSplineStepOverFootY;

  if (m_ClampedCubicSplineStepOverFootZ!=0)
    delete m_ClampedCubicSplineStepOverFootZ;
	
  if (m_ClampedCubicSplineStepOverFootOmega!=0)
    delete m_ClampedCubicSplineStepOverFootOmega;

  if (m_ClampedCubicSplineStepOverFootOmegaImpact!=0)
    delete m_ClampedCubicSplineStepOverFootOmegaImpact;

  if (m_CollDet!=0)
    delete m_CollDet;
}

void StepOverPlanner::CalculateFootHolds(deque<RelativeFootPosition> &aFootHolds)
{
	
  m_FootHolds.clear();

  m_Tsingle=m_ZMPDiscr->GetTSingleSupport();
  m_Tdble=m_ZMPDiscr->GetTDoubleSupport();

  /// Returns the double support time.
  float GetTDoubleSupport();	
	
	


		 
	 
  DoubleSupportFeasibility(); //perform this function to set m_StepOverStepLenght and  m_StepOverHipHeight;
	 

  double ankleDistToObstacle;

  ankleDistToObstacle=m_StepOverStepLenght-m_heelToAnkle-m_heelDistAfter-m_ObstacleParameters.d;
	  
  double  footDistLeftToMove;
	  
  footDistLeftToMove=m_ObstacleParameters.x-ankleDistToObstacle;

	  

  double numberOfSteps;
	    
  numberOfSteps=floor(footDistLeftToMove/m_nominalStepLenght);
	 
  double walkStepLenght;

  walkStepLenght=m_nominalStepLenght+(footDistLeftToMove-m_nominalStepLenght*numberOfSteps)/numberOfSteps;
	 
  cout << "Obstacle height with safety boundary:" << m_ObstacleParameters.h << endl;
  cout << "Obstacle thickness with safety boundary:" << m_ObstacleParameters.d << endl;
  cout << "Distance to Obstacle:" << m_ObstacleParameters.x << endl;
  cout << "StepOver steplenght:" << m_StepOverStepLenght << endl;
  cout << "StepOver COMHeight:" << m_StepOverHipHeight << endl;
  cout << "Ankle distance in front of the obstacle:" <<  ankleDistToObstacle << endl;
  cout << "number of Steps before the obstacle:" << numberOfSteps << endl;
  cout << "Steplenght during walking:" << walkStepLenght << endl;

  RelativeFootPosition tempPos;
	 
  tempPos.sx=0.0;
  tempPos.sy=-m_nominalStepWidth/2.0;
  tempPos.theta=0.0; 
  tempPos.SStime=0.0; 
  tempPos.DStime=m_Tdble/2.0; 
	 
  tempPos.stepType=1;
	

  m_FootHolds.push_back(tempPos);

	    
  for (int i=0;i<numberOfSteps-1;i++)
    {
      tempPos.sx=walkStepLenght;
      tempPos.sy=(-1.0)*(tempPos.sy)/std::fabs((double)tempPos.sy)*m_nominalStepWidth;
      tempPos.theta=0.0;
      tempPos.SStime=m_Tsingle; 
      tempPos.DStime=m_Tdble; 
      tempPos.stepType=1;
      m_FootHolds.push_back(tempPos);
    };

  // one step before stepover obstacle

  tempPos.sx=walkStepLenght;
  tempPos.sy=(-1.0)*(tempPos.sy)/std::fabs((double)tempPos.sy)*m_nominalStepWidth;
  tempPos.theta=0.0;
  tempPos.SStime=m_TsingleStepOverBeforeAfter; 
  tempPos.DStime=m_TdbleStepOverBeforeAfter; 
  tempPos.stepType=2;
  m_FootHolds.push_back(tempPos);
	 
  // first leg over the obsacle

  tempPos.sx=m_StepOverStepLenght;
  tempPos.sy=(-1.0)*(tempPos.sy)/std::fabs((double)tempPos.sy)*m_nominalStepWidth;
  //*cos(m_WaistRotationStepOver*M_PI/180.0);
  tempPos.theta=0.0;
  tempPos.SStime=m_TsingleStepOver; 
  tempPos.DStime=m_Tdble; 
  tempPos.stepType=3;
  m_FootHolds.push_back(tempPos);

  // second leg over the obsacle

  tempPos.sx=m_nominalStepLenght;
  tempPos.sy=(-1.0)*(tempPos.sy)/std::fabs((double)tempPos.sy)*m_nominalStepWidth;
  //*cos(m_WaistRotationStepOver*M_PI/180.0);
  tempPos.theta=0.0;
  tempPos.SStime=m_TsingleStepOver; 
  tempPos.DStime=m_TdbleStepOver; 
  tempPos.stepType=4;
  m_FootHolds.push_back(tempPos);
	
  //one step after the obstacle stepping over
  tempPos.sx=m_nominalStepLenght;
  tempPos.sy=(-1.0)*(tempPos.sy)/std::fabs((double)tempPos.sy)*m_nominalStepWidth;
  tempPos.theta=0.0;
  tempPos.SStime=m_TsingleStepOverBeforeAfter; 
  tempPos.DStime=m_TdbleStepOverBeforeAfter; 
  tempPos.stepType=5;
  m_FootHolds.push_back(tempPos);

  //one extra regular step
  tempPos.sx=m_nominalStepLenght;
  tempPos.sy=(-1.0)*(tempPos.sy)/std::fabs((double)tempPos.sy)*m_nominalStepWidth;
  tempPos.theta=0.0;
  tempPos.SStime=m_Tsingle; 
  tempPos.DStime=m_Tdble; 
	
  tempPos.stepType=1;
  m_FootHolds.push_back(tempPos);
		
  //last step
  tempPos.sx=0;
  tempPos.sy=(-1.0)*(tempPos.sy)/std::fabs((double)tempPos.sy)*m_nominalStepWidth;
  tempPos.theta=0.0;
  tempPos.SStime=m_Tsingle; 
  tempPos.DStime=m_Tdble; 
  tempPos.stepType=1;
  m_FootHolds.push_back(tempPos);  
		
  aFootHolds=m_FootHolds;
		
}

void StepOverPlanner::DoubleSupportFeasibility()
{
  double StepOverStepWidth;
  double StepOverStepLenght, StepOverStepLenghtMin, StepOverStepLenghtMax; 
  double StepOverCOMHeight, StepOverCOMHeightMin, StepOverCOMHeightMax; 
  double OrientationHipToObstacle, OrientationFeetToObstacle = 0.0, OmegaAngleFeet = 0.0;
  //this is the factor determining aproximately the COM position due to preview control during double support
  double DoubleSupportCOMPosFactor;  


  int EvaluationNumber = 10;
  double IncrementStepLenght, IncrementCOMHeight;

  VNL::Matrix<double> Body_R(3,3), Body_P(3,1);
  VNL::Matrix<double> Foot_R(3,3), Foot_P(3,1);
  float c,s,co,so;
  VNL::Matrix<double> ToTheHip(3,1);
  VNL::Matrix<double> LeftLegAngles(6,1), RightLegAngles(6,1); 
	

	
  VNL::Matrix<double> AnkleBeforeObst(3,1), AnkleAfterObst(3,1);
  VNL::Matrix<double> TempCOMPosition(3,1), Temp(3,3);
	
  COMPosition aCOMPosition;

  VNL::Matrix<double> PointOnLeg(3,1);
  VNL::Matrix<double> AbsCoord(3,1),AbsCoord1(3,1),AbsCoord2(3,1);
  VNL::Matrix<double> LegAngles(6,1);
  VNL::Matrix<double> WaistRot(3,3);
  VNL::Matrix<double> WaistPos(3,1);
  VNL::Matrix<double> ObstFrameCoord(3,1),ObstFrameCoord1(3,1),ObstFrameCoord2(3,1);
	
  bool CollisionStatus, FinalCollisionStatus;
	

  StepOverStepLenghtMin = m_ObstacleParameters.d + m_heelToAnkle + m_tipToAnkle + m_heelDistAfter + m_tipDistBefore;
  StepOverStepLenghtMax = 0.6;

  StepOverCOMHeightMin = 0.4- m_DiffBetweenComAndWaist + m_soleToAnkle;//0.4 - m_DiffBetweenComAndWaist + m_soleToAnkle;0.6 * cos(90.0*M_PI/180.0/2.0) 
  StepOverCOMHeightMax =0.75-m_DeltaStepOverCOMHeightMax;//m_NominalCOMStepHeight;//0.6 * cos(m_KneeAngleBound/2.0) - m_DiffBetweenComAndWaist + m_soleToAnkle;

  IncrementStepLenght = double ((StepOverStepLenghtMax  - StepOverStepLenghtMin)/((EvaluationNumber)));
  IncrementCOMHeight  = double ((StepOverCOMHeightMax   - StepOverCOMHeightMin)/((EvaluationNumber)));
 
  OrientationHipToObstacle = 0.0*M_PI/180.0; 
  ///this angle can be used to extend the steplength during stepover but currently it is set to 0 convinience
	
	
  DoubleSupportCOMPosFactor = 0.50; ///this parameter should be evaluated and checked and in the end to be retreieved from a table containing these values for different step situations ...for which a first round of preview control has been performed
  CollisionStatus = 1;
  FinalCollisionStatus = 1;

  ///we suppose that both feet have the same orentation with respect to the obstacle

  for (int i=0;i<EvaluationNumber+1;i++)
    {
      for (int j=0;j<EvaluationNumber+1;j++)
	{	

			
	
	  StepOverStepLenght = StepOverStepLenghtMin + i*IncrementStepLenght;
	  StepOverCOMHeight = StepOverCOMHeightMax - (double(j*IncrementCOMHeight));
	  StepOverStepWidth = m_nominalStepWidth;

							
	  //cout << "StepOverStepcd ../Lenght: " << StepOverStepLenght << " StepOverStepWidth: " << StepOverStepWidth << " StepOverCOMHeight: " << StepOverCOMHeight << endl;

	  //coordinates ankles in obstacle frame  
	  //assuming the left foot is in front of the obstacle
	  //and that in the Y direction of the obstacle the feet are symmetrical with respect to the obstacle origin

	  AnkleBeforeObst(0,0) =-( StepOverStepLenght-m_heelToAnkle-m_heelDistAfter-m_ObstacleParameters.d);
	  AnkleBeforeObst(1,0) = StepOverStepWidth/2.0;
	  AnkleBeforeObst(2,0) = m_soleToAnkle;

	  AnkleAfterObst(0,0) = AnkleBeforeObst(0,0) +StepOverStepLenght;
	  AnkleAfterObst(1,0) = -StepOverStepWidth/2.0;
	  AnkleAfterObst(2,0) = m_soleToAnkle;
	  		

	  //position left foot in front of the obstacle to world frame coordinates
	  Foot_P = m_ObstaclePosition + m_ObstacleRot * AnkleBeforeObst;

	
	  TempCOMPosition(0,0) = AnkleBeforeObst(0,0)+ DoubleSupportCOMPosFactor * StepOverStepLenght;
	  TempCOMPosition(1,0) = 0.0; //suppose the preview control sets Y coordinate in the middel of the dubbel support 
	  TempCOMPosition(2,0) = StepOverCOMHeight;

	  //to worldframe 
	  TempCOMPosition = m_ObstaclePosition + m_ObstacleRot * TempCOMPosition;
			
	  aCOMPosition.x[0] = TempCOMPosition(0,0);
	  aCOMPosition.y[0] = TempCOMPosition(1,0);
	  aCOMPosition.z[0] = TempCOMPosition(2,0);
	
	  aCOMPosition.theta = - m_WaistRotationStepOver;//m_ObstacleParameters.theta + OrientationHipToObstacle;  
	
			
	  c = cos(aCOMPosition.theta*M_PI/180.0);
	  s = sin(aCOMPosition.theta*M_PI/180.0);
	
	  // COM Orientation
	  Body_R(0,0) = c;       Body_R(0,1) = -s;       Body_R(0,2) = 0;
	  Body_R(1,0) = s;       Body_R(1,1) =  c;       Body_R(1,2) = 0;
	  Body_R(2,0) = 0;       Body_R(2,1) = 0;        Body_R(2,2) = 1;
		
	  // COM position
	  ToTheHip = Body_R * m_StaticToTheLeftHip;
	  Body_P(0,0) = aCOMPosition.x[0] + ToTheHip(0,0) ;
	  Body_P(1,0) = aCOMPosition.y[0] + ToTheHip(1,0);
	  Body_P(2,0) = aCOMPosition.z[0] + ToTheHip(2,0);
	
	
	  // Left Foot.
			
	  c = cos(m_ObstacleParameters.theta*M_PI/180.0);
	  s = sin(m_ObstacleParameters.theta*M_PI/180.0);
	  co = cos(0.0*M_PI/180.0);  //at the moment the feet stand flat on the ground when in double support phase
	  so = sin(0.0*M_PI/180.0);
				
	  Foot_R(0,0) = c*co;       Foot_R(0,1) = -s;       Foot_R(0,2) = c*so;
	  Foot_R(1,0) = s*co;       Foot_R(1,1) =  c;       Foot_R(1,2) = s*so;
	  Foot_R(2,0) = -so;        Foot_R(2,1) = 0;        Foot_R(2,2) = co;
	
			
	
	  // Compute the inverse kinematics.
	  m_IK->ComputeInverseKinematics2ForLegs(Body_R,
						 Body_P,
						 m_Dt,
						 Foot_R,
						 Foot_P,
						 LeftLegAngles);
	
	  // RIGHT FOOT //
	  m_Dt(1,0) = -m_Dt(1,0);
			
				
	  // Right Foot.
	  c = cos(OrientationFeetToObstacle*M_PI/180.0);
	  s = sin(OrientationFeetToObstacle*M_PI/180.0);
	  co = cos(OmegaAngleFeet*M_PI/180.0);
	  so = sin(OmegaAngleFeet*M_PI/180.0);
	
	  // Orientation
	  Foot_R(0,0) = c*co;       Foot_R(0,1) = -s;       Foot_R(0,2) = c*so;
	  Foot_R(1,0) = s*co;       Foot_R(1,1) =  c;       Foot_R(1,2) = s*so;
	  Foot_R(2,0) =  -so;       Foot_R(2,1) =  0;       Foot_R(2,2) = co;
			
	  // position
	  Foot_P = m_ObstaclePosition + m_ObstacleRot * AnkleAfterObst;

	
	  // COM position
	  ToTheHip = Body_R * m_StaticToTheRightHip;
	  Body_P(0,0) = aCOMPosition.x[0] + ToTheHip(0,0) ;
	  Body_P(1,0) = aCOMPosition.y[0] + ToTheHip(1,0);
	  Body_P(2,0) = aCOMPosition.z[0] + ToTheHip(2,0);
			
	  m_IK->ComputeInverseKinematics2ForLegs(Body_R,
						 Body_P,
						 m_Dt,
						 Foot_R,
						 Foot_P,
						 RightLegAngles);
	  m_Dt(1,0) = -m_Dt(1,0);

			

	  ///TO DO a check on all the maximum values for the angles after the inverse kinematics....or implement a check in the inverskinematics claas itself...at this moments there is only a protection against knee overstretch built in		
	  if (!((LeftLegAngles(3,0)<m_KneeAngleBound)||(RightLegAngles(3,0)<m_KneeAngleBound)))
	    {
				
	      WaistPos(0,0) = aCOMPosition.x[0];
	      WaistPos(1,0) = aCOMPosition.y[0];
	      WaistPos(2,0) = aCOMPosition.z[0]+m_DiffBetweenComAndWaist;
				
	      WaistRot = Body_R;
				
				//check collision : for the leg in front of the obstacle only lines (points 1, 2, 3, 4) on the shin
				//		    for the leg behind the obstacle only lines (point 5, 6, 7) on the calf 
	      CollisionStatus = 0;
	      FinalCollisionStatus = 0;
				
				//leg in front of the obstacle (for now always left leg)
	      for (unsigned int k=0;k<3;k++)
		{	
		  PointOnLeg = m_LegLayoutPoint.GetNColumns(k,1);
		  m_CollDet->CalcCoordShankLowerLegPoint(PointOnLeg,AbsCoord,LeftLegAngles,WaistRot,WaistPos,1);
		  m_CollDet->WorldFrameToObstacleFrame(AbsCoord, ObstFrameCoord1);
		  PointOnLeg = m_LegLayoutPoint.GetNColumns(k+1,1);
		  m_CollDet->CalcCoordShankLowerLegPoint(PointOnLeg,AbsCoord,LeftLegAngles,WaistRot,WaistPos,1);
		  m_CollDet->WorldFrameToObstacleFrame(AbsCoord, ObstFrameCoord2);
		  CollisionStatus = m_CollDet->CollisionLineObstacleComplete(ObstFrameCoord1,ObstFrameCoord2);
		  //cout << "collision status for line with starting point " << k+1 << " is : " << CollisionStatus << endl;
		  FinalCollisionStatus = FinalCollisionStatus || CollisionStatus;
		}
	      for (unsigned int k=4;k<6;k++)
		{	
		  PointOnLeg = m_LegLayoutPoint.GetNColumns(k,1);
		  m_CollDet->CalcCoordShankLowerLegPoint(PointOnLeg,AbsCoord,LeftLegAngles,WaistRot,WaistPos,1);
		  m_CollDet->WorldFrameToObstacleFrame(AbsCoord, ObstFrameCoord1);
		  PointOnLeg = m_LegLayoutPoint.GetNColumns(k+1,1);
		  m_CollDet->CalcCoordShankLowerLegPoint(PointOnLeg,AbsCoord,LeftLegAngles,WaistRot,WaistPos,1);
		  m_CollDet->WorldFrameToObstacleFrame(AbsCoord, ObstFrameCoord2);
		  CollisionStatus = m_CollDet->CollisionLineObstacleComplete(ObstFrameCoord1,ObstFrameCoord2);
		  //cout << "collision status for line with starting point " << k+1 << " is : " << CollisionStatus << endl;
		  FinalCollisionStatus = FinalCollisionStatus || CollisionStatus;
		}
	    }
	  //cout << "FinalCollisionStatus is " << FinalCollisionStatus << endl;
	  if (!FinalCollisionStatus)
	    break;
			
	}
      if (!FinalCollisionStatus)
	{	
	  m_StepOverStepLenght = StepOverStepLenght;
	  m_StepOverHipHeight = StepOverCOMHeight;
				//cout << "feasibility selected StepOverStepLenght : " << StepOverStepLenght << " and StepOverCOMHeight : " << StepOverCOMHeight << endl;
	  //	cout << "while the nominal steplength is : " << m_nominalStepLenght << " and the nominal COMHeight is " << m_NominalCOMStepHeight << endl;;
	  break;
	}
    }
}

void StepOverPlanner::PolyPlanner(deque<COMPosition> &aCOMBuffer, 
				  deque<FootAbsolutePosition> & aLeftFootBuffer, 
				  deque<FootAbsolutePosition> & aRightFootBuffer,
				  deque<ZMPPosition> & aZMPPositions)
{
  m_RightFootBuffer = aRightFootBuffer;
  m_LeftFootBuffer = aLeftFootBuffer; 
  m_COMBuffer = aCOMBuffer;
  m_ZMPPositions = aZMPPositions;	
	
	
	
  m_ModulationSupportCoefficient = 0.9; //m_ZMPDiscr->GetModulationSupportCoefficient();
	
  m_StartStepOver = 0;
  m_StartDoubleSupp = 0;
  m_StartSecondStep = 0;
  m_EndStepOver = 0;

  m_WhileSpecialSteps = false;
  m_StartPrevStepOver = 0;
  m_EndPrevStepOver = 0;

  m_StartAfterStepOver = 0;
  m_EndAfterStepOver = 0;

  for (unsigned int u=0; u<m_LeftFootBuffer.size(); u++) 
    {	
      if ((std::fabs((double)m_LeftFootBuffer[u].stepType)==2)&(m_StartPrevStepOver==0))
	{
	  m_StartPrevStepOver = u;
	  m_WhileSpecialSteps = true;
	}
      if ((std::fabs((double)m_LeftFootBuffer[u].stepType)==13)&(m_EndPrevStepOver==0))
	m_EndPrevStepOver = u;
      if ((std::fabs((double)m_LeftFootBuffer[u].stepType)==3)&(m_StartStepOver==0))
	m_StartStepOver = u;
      if ((m_LeftFootBuffer[u].stepType==14)&(m_StartDoubleSupp==0))
	m_StartDoubleSupp = u;	
      if ((std::fabs((double)m_LeftFootBuffer[u].stepType)==4)&(m_StartSecondStep==0))
	m_StartSecondStep = u;
      if ((m_LeftFootBuffer[u].stepType==15)&(m_EndStepOver==0))
	m_EndStepOver = u;
      if ((std::fabs((double)m_LeftFootBuffer[u].stepType)==5)&(m_StartAfterStepOver==0))
	m_StartAfterStepOver = u;
      if ((std::fabs((double)m_LeftFootBuffer[u].stepType)==11)&(m_EndAfterStepOver==0)&(m_WhileSpecialSteps==true))
	{	m_EndAfterStepOver = u;	
			
	m_WhileSpecialSteps = false;
	break;
	}
    }
	
  if(m_LeftFootBuffer[m_StartStepOver].stepType > 0)
    {
      m_WaistRotationStepOver = -m_WaistRotationStepOver;
    }
	
  PolyPlannerHip();

  if(m_LeftFootBuffer[m_StartStepOver].stepType > 0)
    {
      m_WhoIsFirst = -1;
      PolyPlannerFirstStep(m_LeftFootBuffer);
      PolyPlannerSecondStep(m_RightFootBuffer);
    }
  else
    {		m_WhoIsFirst = +1;	
    PolyPlannerFirstStep(m_RightFootBuffer);
    PolyPlannerSecondStep(m_LeftFootBuffer);
    }
	
  aRightFootBuffer = m_RightFootBuffer;
  aLeftFootBuffer = m_LeftFootBuffer;
  aCOMBuffer = m_COMBuffer;
  aZMPPositions =	m_ZMPPositions;		
	


#ifdef _DEBUG_

  //cout << "dumping foot data in StepOverBuffers_1.csv" << endl;
  ofstream aof_StepOverBuffers;
  static unsigned char FirstCall=1;
  if (FirstCall)
    {
      aof_StepOverBuffers.open("StepOverBuffers_1.csv",ofstream::out);
    }
  else 
    {
      aof_StepOverBuffers.open("StepOverBuffers_1.csv",ofstream::app);
    }
		
  if (FirstCall)
    FirstCall = 0;
	
  for (unsigned int i=0;i<m_LeftFootBuffer.size();i++)
    {	
      if (aof_StepOverBuffers.is_open())
	{
	  aof_StepOverBuffers << 
	    m_LeftFootBuffer[i].time << " " << 
				//m_ZMPBuffer[i].px << " "<< 	
				//m_ZMPBuffer[i].py<< " " << 
	    m_COMBuffer[i].x[0] << " "<< 	
	    m_COMBuffer[i].y[0]<< " " << 
	    m_COMBuffer[i].z[0]<< " " << 
	    m_LeftFootBuffer[i].stepType << " " << 
	    m_LeftFootBuffer[i].x << " " << 
	    m_LeftFootBuffer[i].y << " " << 
	    m_LeftFootBuffer[i].z << " " << 
	    m_LeftFootBuffer[i].omega << " " << 
	    m_RightFootBuffer[i].stepType << " " << 
	    m_RightFootBuffer[i].x << " " << 
	    m_RightFootBuffer[i].y << " " << 
	    m_RightFootBuffer[i].z << " " << 
	    m_RightFootBuffer[i].omega << " " << 
	    endl;
	}
    }
		

  if (aof_StepOverBuffers.is_open())
    {
      aof_StepOverBuffers.close();
    }
#endif	
 
  //return 1;*/	
};


void StepOverPlanner::PolyPlannerFirstStep(deque<FootAbsolutePosition> &aStepOverFootBuffer)
{
 
  VNL::Vector<double> aBoundCondZ(8,1),aBoundCondY(8,1),aBoundCondX(8,1),aBoundCondOmega(8,1); ; 	
	
  double StepTime;
  double StepLenght;
  double Omega1,Omega2,OmegaImpact;
  double xOffset,zOffset;
  double Point1X, Point1Y, Point1Z;
  double Point2X, Point2Y, Point2Z;
  double Point3Z;
	
  StepTime = aStepOverFootBuffer[m_StartDoubleSupp].time-aStepOverFootBuffer[m_StartStepOver].time; 
  StepLenght = aStepOverFootBuffer[m_StartDoubleSupp].x-aStepOverFootBuffer[m_StartStepOver].x; 
	
  xOffset=0.00;
  zOffset=0.0;

  Omega1=0.0;
  Omega2=0.0;
  OmegaImpact=-2.0;

  //m_ModulationSupportCoefficient=0.8;// MOET ERGENS ANDERS GEDEFINIEERD WORDEN	

  //for now it is only in the 2D and with the obstacle perpendicular to absolute x direction
	
  Point1X = StepLenght-m_heelToAnkle-m_ObstacleParameters.d-xOffset-m_tipToAnkle*cos(Omega1*M_PI/180.0);
  Point1Y = 0.00;
  Point1Z = m_ObstacleParameters.h-m_tipToAnkle*sin(Omega1*M_PI/180.0);
	
  Point2X = StepLenght-m_heelToAnkle+xOffset+m_heelToAnkle*cos(Omega2*M_PI/180.0);
  Point2Y = 0.00;
  Point2Z = m_ObstacleParameters.h-m_tipToAnkle*sin(Omega2*M_PI/180.0);
	
  Point3Z= Point1Z+0.04;// m_ObstacleParameters.h+zOffset+0.04+m_tipToAnkle*sin(Omega2*M_PI/180.0);


  vector<double> aTimeDistr,aTimeDistrModulated;
  double ModulatedStepTime = StepTime * m_ModulationSupportCoefficient;
  double LiftOffTime = (StepTime-ModulatedStepTime)*0.5;
  double TouchDownTime = StepTime-(StepTime-ModulatedStepTime)*0.5;

  aTimeDistr.resize(3);
	


	
  aTimeDistr[0]=m_TimeDistrFactor[0]*StepTime/5.0;
  aTimeDistr[1]=m_TimeDistrFactor[1]*StepTime/5.0;
  aTimeDistr[2]=StepTime;


  //this time schedule is used for the X and Y coordinate of the foot in order to make sure the foot lifts the ground (Z) before moving the X and Y direction
	
  aTimeDistrModulated.resize(3);
	
  aTimeDistrModulated[0]=aTimeDistr[0]-LiftOffTime;
  aTimeDistrModulated[1]=aTimeDistr[1]-LiftOffTime;
  aTimeDistrModulated[2]=aTimeDistr[2]-2.0*LiftOffTime;

  VNL::Vector<double> ZfootPos(5),TimeIntervalsZ(5),ZfootSpeedBound(2) ;
  double PreviousSpeedZ,EndSpeedZ,SpeedAccZ,IntermediateZAcc;
  vector<double> SpeedWeightZ;

  ZfootSpeedBound(0)=0.0;
  ZfootSpeedBound(1)=0.0;


	
  int NumberIntermediate = 0,NumberIntermediate2 = 0,Counter =0,CounterTemp =0;
  double IntermediateTimeStep;

  NumberIntermediate = 10;
  NumberIntermediate2 = 20;
	
	
  ZfootPos.Resize(2+ 3*NumberIntermediate);
  TimeIntervalsZ.Resize(2+3*NumberIntermediate);
  SpeedWeightZ.resize(NumberIntermediate);
    

	

	
  ZfootPos(0) = 0.0;
  ZfootPos(1) = Point1Z;
	
  TimeIntervalsZ(0) = 0.0;
  TimeIntervalsZ(1) = aTimeDistr[0];
	
	

  //from point1Z going up to top 	
	
	
  IntermediateTimeStep = (aTimeDistr[1]-aTimeDistr[0])/2.0/(NumberIntermediate);

	
  SpeedAccZ = 0.0;
  IntermediateZAcc = 0.0;
  PreviousSpeedZ = (Point1Z)/(aTimeDistr[0]);
  EndSpeedZ = 0.0;

	
  Counter = 1;
	
  for (int i=1;i<=NumberIntermediate;i++)
    {
      SpeedWeightZ[i-1] = (EndSpeedZ-PreviousSpeedZ)*((double (i-1)/(double (NumberIntermediate))))+PreviousSpeedZ;
      //	cout << "SpeedWeightZ[i-1]" << SpeedWeightZ[i-1] << endl;
      SpeedAccZ = SpeedAccZ + SpeedWeightZ[i-1];
    }
  for (int i=1;i<=NumberIntermediate;i++)
    {
      IntermediateZAcc = IntermediateZAcc + (Point3Z-Point1Z)*SpeedWeightZ[i-1]/SpeedAccZ;
      ZfootPos(Counter+i) =  IntermediateZAcc + ZfootPos(Counter);
		
      TimeIntervalsZ(Counter+i) = TimeIntervalsZ(Counter)+i*IntermediateTimeStep;
      CounterTemp = i;
    }
	
	
	
  //from top going down to point2Z 	
  SpeedAccZ = 0.0;
  IntermediateZAcc = 0.0;
  PreviousSpeedZ = 0.0 ;
  EndSpeedZ =(-Point2Z)/(aTimeDistr[2]-aTimeDistr[1]);
  Counter = CounterTemp + Counter;

  for (int i=1;i<=NumberIntermediate;i++)
    {
      SpeedWeightZ[i-1] = (EndSpeedZ-PreviousSpeedZ)*((double (i-1)/(double (NumberIntermediate))))+PreviousSpeedZ;
      SpeedAccZ = SpeedAccZ + SpeedWeightZ[i-1];
    }
  for (int i=1;i<=NumberIntermediate;i++)
    {
      IntermediateZAcc = IntermediateZAcc + (Point2Z-Point3Z)*SpeedWeightZ[i-1]/SpeedAccZ;
      ZfootPos(Counter+i) =  IntermediateZAcc + ZfootPos(Counter);
		
      TimeIntervalsZ(Counter+i) = TimeIntervalsZ(Counter)+i*IntermediateTimeStep;
      CounterTemp = i;
    }
	
  //going down from point2Z to the ground with smooth velocity profile at touch down
	
  IntermediateTimeStep = (aTimeDistr[2]-aTimeDistr[1])/(NumberIntermediate);
	
	
	
  SpeedAccZ = 0.0;
  IntermediateZAcc = 0.0;
  PreviousSpeedZ = (-Point2Z)/(aTimeDistr[2]-aTimeDistr[1]);
  EndSpeedZ = ZfootSpeedBound(1);
  Counter = CounterTemp + Counter;


  for (int i=1;i<=NumberIntermediate;i++)
    {
      SpeedWeightZ[i-1] = (EndSpeedZ-PreviousSpeedZ)*pow((double (i-1)/(double (NumberIntermediate))),1)+PreviousSpeedZ;
      SpeedAccZ = SpeedAccZ + SpeedWeightZ[i-1];
    }
	
  for (int i=1;i<=NumberIntermediate;i++)
    {
      IntermediateZAcc = IntermediateZAcc + (-Point2Z)*SpeedWeightZ[i-1]/SpeedAccZ;
      ZfootPos(Counter+i) =  IntermediateZAcc + ZfootPos(Counter);
		
      TimeIntervalsZ(Counter+i) = TimeIntervalsZ(Counter)+i*IntermediateTimeStep;

    }


  m_ClampedCubicSplineStepOverFootZ->SetParameters(ZfootPos,TimeIntervalsZ,ZfootSpeedBound);
 

  VNL::Vector<double> XfootPos(4),TimeIntervalsX(4),XfootSpeedBound(2) ;
       
       
  XfootPos(0) = 0.0;
  XfootPos(1) = Point1X;
  XfootPos(2) = Point2X;
  XfootPos(3) = StepLenght;

  TimeIntervalsX(0) = 0.0;
  TimeIntervalsX(1) = aTimeDistrModulated[0];
  TimeIntervalsX(2) = aTimeDistrModulated[1];
  TimeIntervalsX(3) = aTimeDistrModulated[2];

  XfootSpeedBound(0)=0.0;
  XfootSpeedBound(1)=0.0;

  m_ClampedCubicSplineStepOverFootX->SetParameters(XfootPos,TimeIntervalsX,XfootSpeedBound);
  
  VNL::Vector<double> OmegafootPos(4),TimeIntervalsOmega(4),OmegafootSpeedBound(2);

  OmegafootPos(0) = 0.0;
  OmegafootPos(1) = OmegaImpact*1.0/3.0;
  OmegafootPos(2) = OmegaImpact*2.0/3.0;
  OmegafootPos(3) = OmegaImpact;

  TimeIntervalsOmega(0) = 0.0;
  TimeIntervalsOmega(1) = aTimeDistrModulated[0];
  TimeIntervalsOmega(2) = aTimeDistrModulated[1];
  TimeIntervalsOmega(3) = aTimeDistrModulated[2];

  OmegafootSpeedBound(0)=0.0;
  OmegafootSpeedBound(1)=0.0;

  m_ClampedCubicSplineStepOverFootOmega->SetParameters(OmegafootPos,TimeIntervalsOmega,OmegafootSpeedBound);
 
  VNL::Vector<double> OmegaImpactfootPos(2),TimeIntervalsOmegaImpact(2),OmegaImpactfootSpeedBound(2);

  OmegaImpactfootPos(0) = OmegaImpact;
  OmegaImpactfootPos(1) = 0.0;

  TimeIntervalsOmegaImpact(0) = 0.0;
  TimeIntervalsOmegaImpact(1) = LiftOffTime;

  OmegaImpactfootSpeedBound(0)=0.0;
  OmegaImpactfootSpeedBound(1)=0.0;

  m_ClampedCubicSplineStepOverFootOmegaImpact->SetParameters(OmegaImpactfootPos,TimeIntervalsOmegaImpact,OmegaImpactfootSpeedBound);
 


  vector<double> aTimeDistrModulatedYSide;
  VNL::Vector<double> aBoundCondYSide(5);
	 
  aTimeDistrModulatedYSide.resize(2);

  aTimeDistrModulatedYSide[0]=3.0*(StepTime-2.0*LiftOffTime)/5.0;
  aTimeDistrModulatedYSide[1]=StepTime-2.0*LiftOffTime;


  //this time schedule is used for the X and Y coordinate of the foot in order to make sure the foot lifts the ground (Z) before moving the X and Y direction
		
	

  VNL::Vector<double> YfootPos(4),TimeIntervalsY(4),YfootSpeedBound(2) ;

  {
    if (m_ObstacleParameters.h> 0.20)
      {
	//when the obstacle is to high an auto collision occurs at the hip joint 
	// so the foot is turned a little inwaqrds to avoid this
	YfootPos(0) = 0.0;
	YfootPos(1) = m_WhoIsFirst*0.01*(m_ObstacleParameters.h-0.20)/0.05;
	YfootPos(2) = m_WhoIsFirst*0.07*(m_ObstacleParameters.h-0.20)/0.05;
	YfootPos(3) = 0.0;
      }
    else
      {
	YfootPos(0) = 0.0;
	YfootPos(1) = 0.0;
	YfootPos(2) = 0.0;
	YfootPos(3) = 0.0;
      }
	
    TimeIntervalsY(0) = 0.0;
    TimeIntervalsY(1) = aTimeDistrModulated[0];
    TimeIntervalsY(2) = aTimeDistrModulated[1];
    TimeIntervalsY(3) = aTimeDistrModulated[2];
	
    YfootSpeedBound(0)=0.0;
    YfootSpeedBound(1)=0.0;
	
    m_ClampedCubicSplineStepOverFootY->SetParameters(YfootPos,TimeIntervalsY,YfootSpeedBound);
  }

  //update the footbuffers with the new calculated polynomials for stepping over
  unsigned int diff = m_StartDoubleSupp-m_StartStepOver;
  double LocalTime;
  int aStart = m_StartStepOver;
  //double temp;
	
  for (unsigned int i=0;i<=diff;i++)
    {	
      LocalTime=(i)*m_SamplingPeriod;	
		
      if (LocalTime<LiftOffTime)
	{
	  aStepOverFootBuffer[i+aStart].x=aStepOverFootBuffer[aStart].x;
	  aStepOverFootBuffer[i+aStart].y=aStepOverFootBuffer[aStart].y;
	  aStepOverFootBuffer[i+aStart].theta=aStepOverFootBuffer[aStart].theta;
	  aStepOverFootBuffer[i+aStart].omega=aStepOverFootBuffer[aStart].omega;
	}
      else if (LocalTime>=TouchDownTime)
	{
	  aStepOverFootBuffer[i+aStart].x=aStepOverFootBuffer[i+aStart-1].x;
	  aStepOverFootBuffer[i+aStart].y=aStepOverFootBuffer[i+aStart-1].y;
	  aStepOverFootBuffer[i+aStart].theta=aStepOverFootBuffer[i+aStart-1].theta;
	  aStepOverFootBuffer[i+aStart].omega=m_ClampedCubicSplineStepOverFootOmegaImpact->GetValueSpline(TimeIntervalsOmegaImpact,LocalTime-TouchDownTime)+aStepOverFootBuffer[aStart].omega;
	}
      else 
	{	
	  aStepOverFootBuffer[i+aStart].x = m_ClampedCubicSplineStepOverFootX->GetValueSpline(TimeIntervalsX,LocalTime-LiftOffTime)+aStepOverFootBuffer[aStart].x;	
	  aStepOverFootBuffer[i+aStart].y = m_ClampedCubicSplineStepOverFootY->GetValueSpline(TimeIntervalsY,LocalTime-LiftOffTime)+aStepOverFootBuffer[aStart].y;	
	  aStepOverFootBuffer[i+aStart].omega=m_ClampedCubicSplineStepOverFootOmega->GetValueSpline(TimeIntervalsOmega,LocalTime-LiftOffTime)+aStepOverFootBuffer[aStart].omega;
	  aStepOverFootBuffer[i+aStart].theta=aStepOverFootBuffer[aStart].theta;	
	}
      aStepOverFootBuffer[i+aStart].z = m_ClampedCubicSplineStepOverFootZ->GetValueSpline(TimeIntervalsZ,LocalTime)+aStepOverFootBuffer[aStart].z;	
    }

};

void StepOverPlanner::PolyPlannerSecondStep(deque<FootAbsolutePosition> &aStepOverFootBuffer)
{
 
  VNL::Vector<double> aBoundCondZ(8,1),aBoundCondY(8,1),aBoundCondX(8,1), aBoundCondOmega(8,1);
	
  double StepTime;
  double StepLenght;
  double Omega1,Omega2,OmegaImpact;
  double xOffset,zOffset;
  double Point1X,Point1Y,Point1Z;
  double Point2X,Point2Y,Point2Z;
  double Point3Z;
	
  StepTime = aStepOverFootBuffer[m_EndStepOver].time-aStepOverFootBuffer[m_StartSecondStep].time; 
  StepLenght = aStepOverFootBuffer[m_EndStepOver].x-aStepOverFootBuffer[m_StartSecondStep].x; 
	
  xOffset=0.0;
  zOffset=0.0;

  Omega1=120.0*m_ObstacleParameters.h;    //in degrees
  Omega2=120.0*m_ObstacleParameters.h;
  OmegaImpact=-2.0;

  Point1X = m_StepOverStepLenght-m_heelToAnkle-m_ObstacleParameters.d-xOffset-m_tipToAnkle*cos(Omega1*M_PI/180.0);
  Point1Y = 0.0;
  Point1Z = m_ObstacleParameters.h+m_tipToAnkle*sin(Omega1*M_PI/180.0);
	
  Point2X = m_StepOverStepLenght-m_heelToAnkle+xOffset+m_heelToAnkle*cos(Omega2*M_PI/180.0);
  Point2Y = 0.0;
  Point2Z = Point1Z;// m_ObstacleParameters.h+0.04;//-m_tipToAnkle*sin(Omega2*M_PI/180.0);
	

	


  vector<double> aTimeDistr,aTimeDistrModulated;
  double ModulatedStepTime = StepTime * m_ModulationSupportCoefficient;
  double LiftOffTime = (StepTime-ModulatedStepTime)*0.5;
  double TouchDownTime = StepTime-(StepTime-ModulatedStepTime)*0.5;

  aTimeDistr.resize(4);
	
  aTimeDistr[0]=m_TimeDistrFactor[2]*StepTime/5.0;
  //aTimeDistr[1]=1.8*StepTime/5.0;
  aTimeDistr[1]=m_TimeDistrFactor[3]*StepTime/5.0;
  aTimeDistr[2]=StepTime;

  //this time schedule is used for the X and Y coordinate of the foot in order to make sure the foot lifts the ground (Z) before moving the X and Y direction
	
  aTimeDistrModulated.resize(3);
	
  aTimeDistrModulated[0]=aTimeDistr[0]-LiftOffTime;
  aTimeDistrModulated[1]=aTimeDistr[1]-LiftOffTime;
  aTimeDistrModulated[2]=aTimeDistr[2]-2.0*LiftOffTime;

  VNL::Vector<double> ZfootPos(10),TimeIntervalsZ(10),ZfootSpeedBound(2) ;

        
  ZfootSpeedBound(0)=0.0;
  ZfootSpeedBound(1)=0.0;
	
  int NumberIntermediate = 0,Counter =0;
  double IntermediateTimeStep;

	
	
  NumberIntermediate = 10;
  IntermediateTimeStep = (aTimeDistr[1]-aTimeDistr[0])/(NumberIntermediate+1);	

  ZfootPos.Resize(4+NumberIntermediate);
  TimeIntervalsZ.Resize(4+NumberIntermediate);

  Point3Z= Point1Z+0.01;

  ZfootPos(0) = 0.0;
  ZfootPos(1) = Point1Z;
	
  TimeIntervalsZ(0) = 0.0;
  TimeIntervalsZ(1) = aTimeDistr[0];

  for (int i=1;i<=NumberIntermediate;i++)
    {
      ZfootPos(1+i) = Point3Z;
      TimeIntervalsZ(1+i) = aTimeDistr[0]+i*IntermediateTimeStep;
      Counter = i;
    }
  ZfootPos(1+Counter+1) = Point2Z;
  ZfootPos(1+Counter+2) = 0.0;
  TimeIntervalsZ(1+Counter+1) = aTimeDistr[1];
  TimeIntervalsZ(1+Counter+2) = aTimeDistr[2];



  m_ClampedCubicSplineStepOverFootZ->SetParameters(ZfootPos,TimeIntervalsZ,ZfootSpeedBound);

  VNL::Vector<double> XfootPos(4),TimeIntervalsX(4),XfootSpeedBound(2) ;

  XfootSpeedBound(0)=0.0;
  XfootSpeedBound(1)=0.0;

  NumberIntermediate = 10;
  IntermediateTimeStep = (aTimeDistrModulated[2]-aTimeDistrModulated[1])/(NumberIntermediate+1);	

  XfootPos.Resize(4+NumberIntermediate);
  TimeIntervalsX.Resize(4+NumberIntermediate);

  //Use of speed to weight the extra points for the last interval on X to prevent overshoot of the spline on X
  double PreviousSpeedX,EndSpeedX,SpeedAccX;
  vector<double> SpeedWeightX;
	
  SpeedWeightX.resize(NumberIntermediate);
  SpeedAccX = 0.0;
  PreviousSpeedX = (Point2X-Point1X)/(aTimeDistrModulated[1]-aTimeDistrModulated[0]);
  EndSpeedX = XfootSpeedBound(1);
  for (int i=1;i<=NumberIntermediate;i++)
    {
      SpeedWeightX[i] = (EndSpeedX-PreviousSpeedX)*i/(NumberIntermediate+1)+PreviousSpeedX;
      SpeedAccX = SpeedAccX+SpeedWeightX[i];
    }
	

  XfootPos(0) = 0.0;
  XfootPos(1) = Point1X;
  XfootPos(2) = Point2X;
	
  TimeIntervalsX(0) = 0.0;
  TimeIntervalsX(1) = aTimeDistrModulated[0];
  TimeIntervalsX(2) = aTimeDistrModulated[1];

  for (int i=1;i<=NumberIntermediate;i++)
    {
      XfootPos(2+i) = XfootPos(2+i-1)+(StepLenght-Point2X)*SpeedWeightX[i]/SpeedAccX;
      TimeIntervalsX(2+i) = aTimeDistrModulated[1]+i*IntermediateTimeStep;
      Counter = i;
    }
  XfootPos(2+Counter+1) = StepLenght;
  TimeIntervalsX(2+Counter+1) = aTimeDistrModulated[2];
	

  m_ClampedCubicSplineStepOverFootX->SetParameters(XfootPos,TimeIntervalsX,XfootSpeedBound);
 
  VNL::Vector<double> OmegafootPos(4),TimeIntervalsOmega(4),OmegafootSpeedBound(2);

  OmegafootSpeedBound(0)=0.0;
  OmegafootSpeedBound(1)=0.0;

	

	
	
  NumberIntermediate = 10;
  IntermediateTimeStep = (aTimeDistrModulated[1]-aTimeDistrModulated[0])/(NumberIntermediate+1);	

  OmegafootPos.Resize(4+NumberIntermediate);
  TimeIntervalsOmega.Resize(4+NumberIntermediate);

  double Omega3;

  Omega3= Omega1+1;

  OmegafootPos(0) = 0.0;
  OmegafootPos(1) = Omega1;
	
  TimeIntervalsOmega(0) = 0.0;
  TimeIntervalsOmega(1) = aTimeDistrModulated[0];

  for (int i=1;i<=NumberIntermediate;i++)
    {
      OmegafootPos(1+i) = Omega3;
      TimeIntervalsOmega(1+i) = aTimeDistrModulated[0]+i*IntermediateTimeStep;
      Counter = i;
    }
  OmegafootPos(1+Counter+1) = Omega2;
  OmegafootPos(1+Counter+2) = OmegaImpact;
  TimeIntervalsOmega(1+Counter+1) = aTimeDistrModulated[1];
  TimeIntervalsOmega(1+Counter+2) = aTimeDistrModulated[2];


  /*

    OmegafootPos(0) = 0.0;
    OmegafootPos(1) = Omega1;
    OmegafootPos(2) = Omega2;
    OmegafootPos(3) = 0.0;

    TimeIntervalsOmega(0) = 0.0;
    TimeIntervalsOmega(1) = aTimeDistrModulated[0];
    TimeIntervalsOmega(2) = aTimeDistrModulated[1];
    TimeIntervalsOmega(3) = aTimeDistrModulated[2];
  */
  m_ClampedCubicSplineStepOverFootOmega->SetParameters(OmegafootPos,TimeIntervalsOmega,OmegafootSpeedBound);
 
  VNL::Vector<double> OmegaImpactfootPos(2),TimeIntervalsOmegaImpact(2),OmegaImpactfootSpeedBound(2);

  OmegaImpactfootPos(0) = OmegaImpact;
  OmegaImpactfootPos(1) = 0.0;

  TimeIntervalsOmegaImpact(0) = 0.0;
  TimeIntervalsOmegaImpact(1) = LiftOffTime;

  OmegaImpactfootSpeedBound(0)=0.0;
  OmegaImpactfootSpeedBound(1)=0.0;

  m_ClampedCubicSplineStepOverFootOmegaImpact->SetParameters(OmegaImpactfootPos,TimeIntervalsOmegaImpact,OmegaImpactfootSpeedBound);
 

  VNL::Vector<double> YfootPos(4),TimeIntervalsY(4),YfootSpeedBound(2) ;

	
  YfootPos(0) = 0.0;
  YfootPos(1) = 0.0;
  YfootPos(2) = 0.0;
  YfootPos(3) = 0.0;
	

  TimeIntervalsY(0) = 0.0;
  TimeIntervalsY(1) = aTimeDistrModulated[0];
  TimeIntervalsY(2) = aTimeDistrModulated[1];
  TimeIntervalsY(3) = aTimeDistrModulated[2];

  YfootSpeedBound(0)=0.0;
  YfootSpeedBound(1)=0.0;

  m_ClampedCubicSplineStepOverFootY->SetParameters(YfootPos,TimeIntervalsY,YfootSpeedBound);

  //update the footbuffers with the new calculated polynomials for stepping over
  unsigned int diff = m_EndStepOver-m_StartSecondStep;
  double LocalTime;
  int  aStart = m_StartSecondStep;
	
  for (unsigned int i=0;i<=diff;i++)
    {	
      LocalTime=(i)*m_SamplingPeriod;	
		
      if (LocalTime<LiftOffTime)
	{
	  aStepOverFootBuffer[i+aStart].x=aStepOverFootBuffer[aStart].x;
	  aStepOverFootBuffer[i+aStart].y=aStepOverFootBuffer[aStart].y;
	  aStepOverFootBuffer[i+aStart].theta=aStepOverFootBuffer[aStart].theta;
	  aStepOverFootBuffer[i+aStart].omega=aStepOverFootBuffer[aStart].omega;
	}
      else if (LocalTime>=TouchDownTime)
	{
	  aStepOverFootBuffer[i+aStart].x=aStepOverFootBuffer[i+aStart-1].x;
	  aStepOverFootBuffer[i+aStart].y=aStepOverFootBuffer[i+aStart-1].y;
	  aStepOverFootBuffer[i+aStart].theta=aStepOverFootBuffer[i+aStart-1].theta;
	  aStepOverFootBuffer[i+aStart].omega=m_ClampedCubicSplineStepOverFootOmegaImpact->GetValueSpline(TimeIntervalsOmegaImpact,LocalTime-TouchDownTime)+aStepOverFootBuffer[aStart].omega;
	}
      else 
	{
	  aStepOverFootBuffer[i+aStart].x = m_ClampedCubicSplineStepOverFootX->GetValueSpline(TimeIntervalsX,LocalTime-LiftOffTime)+aStepOverFootBuffer[aStart].x;	
	  aStepOverFootBuffer[i+aStart].y = m_ClampedCubicSplineStepOverFootY->GetValueSpline(TimeIntervalsY,LocalTime-LiftOffTime)+aStepOverFootBuffer[aStart].y;	
	  //aStepOverFootBuffer[i+aStart].y=m_PolynomeStepOverY->Compute(LocalTime-LiftOffTime)+aStepOverFootBuffer[aStart].y;
	  aStepOverFootBuffer[i+aStart].omega=m_ClampedCubicSplineStepOverFootOmega->GetValueSpline(TimeIntervalsOmega,LocalTime-LiftOffTime)+aStepOverFootBuffer[aStart].omega;
	  aStepOverFootBuffer[i+aStart].theta=aStepOverFootBuffer[aStart].theta;	
	}
      aStepOverFootBuffer[i+aStart].z = m_ClampedCubicSplineStepOverFootZ->GetValueSpline(TimeIntervalsZ,LocalTime)+aStepOverFootBuffer[aStart].z;	
    }

};



void StepOverPlanner::PolyPlannerHip()
{
  VNL::Vector<double> aBoundCond(4,1); 	
	
  double StepTime;
  double HeightDifference;
	
 
  StepTime = m_LeftFootBuffer[m_EndPrevStepOver].time-m_LeftFootBuffer[m_StartPrevStepOver].time; 
  //StepTime = m_LeftFootBuffer[m_StartDoubleSupp].time-m_LeftFootBuffer[m_StartStepOver].time; 
  //we take foot buffer since this contains the time course
  HeightDifference =m_StepOverHipHeight-m_COMBuffer[m_StartPrevStepOver].z[0];

  vector<double> aTimeDistr;
       
  //update the COMbuffers with the new calculated polynomials 
  unsigned int diff = int ((m_StartDoubleSupp-m_StartStepOver));

  double LocalTime;
  int aStart = m_StartStepOver;
	
  aTimeDistr.resize(1);
  aTimeDistr[0]=diff*m_SamplingPeriod;




		
  aBoundCond(0)=0.0;
  aBoundCond(1)=0.0;
  aBoundCond(2)=HeightDifference;
  aBoundCond(3)=0.0;
	  
	 
  m_PolynomeStepOverHipStep2->SetParameters(aBoundCond,aTimeDistr);
	
  aBoundCond(0)=0.0;
  aBoundCond(1)=0.0;
  aBoundCond(2)=m_WaistRotationStepOver;
  aBoundCond(3)=0.0;
	  
	 
  m_PolynomeStepOverHipRotation->SetParameters(aBoundCond,aTimeDistr);

  for (unsigned int i=0;i<=diff;i++)
    {	
      LocalTime=(i)*m_SamplingPeriod;	
      {
	m_COMBuffer[i+aStart].z[0]=m_PolynomeStepOverHipStep2->Compute(LocalTime)+m_COMBuffer[aStart].z[0];

	m_COMBuffer[i+aStart].z[1]=(m_COMBuffer[i+aStart].z[0]-m_COMBuffer[i+aStart-1].z[0])/m_SamplingPeriod;
	m_COMBuffer[i+aStart].z[2]=(m_COMBuffer[i+aStart].z[1]-m_COMBuffer[i+aStart-1].z[1])/m_SamplingPeriod;

	m_COMBuffer[i+aStart].theta=m_PolynomeStepOverHipRotation->Compute(LocalTime)+m_COMBuffer[aStart].theta;			
      }
		
		
    }
	

  //during double support stepping over
  diff = m_StartSecondStep-m_StartDoubleSupp;
  aStart = m_StartDoubleSupp;

  for (unsigned int i=0;i<=diff;i++)
    {
      m_COMBuffer[i+aStart].z[0]=m_COMBuffer[i+aStart-1].z[0];

      m_COMBuffer[i+aStart].z[1]=(m_COMBuffer[i+aStart].z[0]-m_COMBuffer[i+aStart-1].z[0])/m_SamplingPeriod;
      m_COMBuffer[i+aStart].z[2]=(m_COMBuffer[i+aStart].z[1]-m_COMBuffer[i+aStart-1].z[1])/m_SamplingPeriod;

      m_COMBuffer[i+aStart].theta=m_COMBuffer[i+aStart-1].theta;
    }
	

  //update the COMbuffers with the new calculated polynomials 
  diff = int ((m_EndStepOver-m_StartSecondStep));


  StepTime = diff*m_SamplingPeriod; 
  aTimeDistr.resize(1);	
  aTimeDistr[0]=StepTime;

  aBoundCond(0)=0.0;
  aBoundCond(1)=0.0;
  aBoundCond(2)=-HeightDifference;
  aBoundCond(3)=0.0;
	  
  m_PolynomeStepOverHipStep2->SetParameters(aBoundCond,aTimeDistr);
	

  aBoundCond(0)=0.0;
  aBoundCond(1)=0.0;
  aBoundCond(2)=-m_WaistRotationStepOver;
  aBoundCond(3)=0.0;
	  
	 
  m_PolynomeStepOverHipRotation->SetParameters(aBoundCond,aTimeDistr);
	
  for (unsigned int i=0;i<=diff;i++)
    {	
      LocalTime=(i)*m_SamplingPeriod;	
      {
	m_COMBuffer[i+aStart].z[0]=m_PolynomeStepOverHipStep2->Compute(LocalTime)+m_COMBuffer[aStart].z[0];

	m_COMBuffer[i+aStart].z[1]=(m_COMBuffer[i+aStart].z[0]-m_COMBuffer[i+aStart-1].z[0])/m_SamplingPeriod;
	m_COMBuffer[i+aStart].z[2]=(m_COMBuffer[i+aStart].z[1]-m_COMBuffer[i+aStart-1].z[1])/m_SamplingPeriod;

	m_COMBuffer[i+aStart].theta=m_PolynomeStepOverHipRotation->Compute(LocalTime)+m_COMBuffer[aStart].theta;
      }
    }

}


void StepOverPlanner::SetExtraBuffer(deque<COMPosition> aExtraCOMBuffer,
				     deque<FootAbsolutePosition> aExtraRightFootBuffer, 
				     deque<FootAbsolutePosition> aExtraLeftFootBuffer)
{
  m_ExtraCOMBuffer=aExtraCOMBuffer;
  m_ExtraRightFootBuffer = aExtraRightFootBuffer;
  m_ExtraLeftFootBuffer = aExtraLeftFootBuffer;
}



void StepOverPlanner::GetExtraBuffer(deque<COMPosition> &aExtraCOMBuffer,
				     deque<FootAbsolutePosition> &aExtraRightFootBuffer, 
				     deque<FootAbsolutePosition> &aExtraLeftFootBuffer)
{
  aExtraCOMBuffer = m_ExtraCOMBuffer;
  aExtraRightFootBuffer = m_ExtraRightFootBuffer;
  aExtraLeftFootBuffer = m_ExtraLeftFootBuffer;
}

void StepOverPlanner::SetFootBuffers(deque<FootAbsolutePosition> aLeftFootBuffer, 
				     deque<FootAbsolutePosition> aRightFootBuffer)
{
  m_RightFootBuffer = aRightFootBuffer;
  m_LeftFootBuffer = aLeftFootBuffer;
}

void StepOverPlanner::GetFootBuffers(deque<FootAbsolutePosition> & aRightFootBuffer, 
				     deque<FootAbsolutePosition> & aLeftFootBuffer)
{
  aRightFootBuffer = m_RightFootBuffer;
  aLeftFootBuffer = m_LeftFootBuffer;
}

void StepOverPlanner::SetObstacleInformation(ObstaclePar ObstacleParameters)
{

  //add safety boundaries to the obstacle , the safety bounderies at the moment are chosen
  //but they can vary in the fuuter in function of the vision information uncertainty

  double safeBoundWidth=0.03;
  double safeBoundHeight=0.03;
  double safeBoundDepth=0.03;


  ObstacleParameters.h+=safeBoundHeight;
  ObstacleParameters.w+=2.0*safeBoundWidth;
  ObstacleParameters.d+=2.0*safeBoundDepth;
  ObstacleParameters.x+=-safeBoundDepth;

  //m_obstacles is visible and requered in the rest of the class 
  m_ObstacleParameters = ObstacleParameters;
       
  m_ObstaclePosition.Resize(3,1);
  m_ObstacleRot.Resize(3,3);
  m_ObstacleRotInv.Resize(3,3);	
	

  m_ObstaclePosition(0,0) = m_ObstacleParameters.x;
  m_ObstaclePosition(1,0) = m_ObstacleParameters.y;
  m_ObstaclePosition(2,0) = m_ObstacleParameters.z;

  double c,s;

  c = cos(m_ObstacleParameters.theta*M_PI/180.0);
  s = sin(m_ObstacleParameters.theta*M_PI/180.0);

	

  //this matrix transformes coordinates in the obstacle frame into the world frame
  m_ObstacleRot(0,0) = c; 	m_ObstacleRot(0,1) =-s;		m_ObstacleRot(0,2) = 0;
  m_ObstacleRot(1,0) = s; 	m_ObstacleRot(1,1) = c;		m_ObstacleRot(1,2) = 0;
  m_ObstacleRot(2,0) = 0; 	m_ObstacleRot(2,1) = 0;		m_ObstacleRot(2,2) = 1;

  //this matrix transformes coordinates in the world frame into the obstacle frame
  m_ObstacleRotInv(0,0) = c; 	m_ObstacleRotInv(0,1) = s;	m_ObstacleRotInv(0,2) = 0;
  m_ObstacleRotInv(1,0) = -s; 	m_ObstacleRotInv(1,1) = c;	m_ObstacleRotInv(1,2) = 0;
  m_ObstacleRotInv(2,0) = 0; 	m_ObstacleRotInv(2,1) = 0;	m_ObstacleRotInv(2,2) = 1;

  m_CollDet = new CollisionDetector();
  m_CollDet->SetObstacleCoordinates(m_ObstacleParameters);

}



void StepOverPlanner::SetPreviewControl(PreviewControl *aPC)
{
  m_PC = aPC;
  m_SamplingPeriod = m_PC->SamplingPeriod();
  m_PreviewControlTime = m_PC->PreviewControlTime();
  m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);
  m_NominalCOMStepHeight = m_PC->GetHeightOfCoM();
}

void StepOverPlanner::SetZMPDiscretization(ZMPDiscretization *aZMPDiscr)
{
  m_ZMPDiscr = aZMPDiscr;
}


void StepOverPlanner::SetDynamicMultiBodyModel(DynamicMultiBody *aDMB)
{
  m_DMB = aDMB;
  for(int i=0;i<m_DMB->NbOfLinks();i++)
    m_DMB->Setdq(i,0.0);

}

void StepOverPlanner::SetInverseKinematics(InverseKinematics *anIK)
{
  m_IK = anIK;
}

void StepOverPlanner::TimeDistributeFactor(vector<double> &TimeDistrFactor)
{
	
  for (unsigned int i=0;i<TimeDistrFactor.size();i++)
    {
      m_TimeDistrFactor[i] = TimeDistrFactor[i];
	
    }

}

void StepOverPlanner::SetDeltaStepOverCOMHeightMax(double aDeltaStepOverCOMHeightMax)
{
	
  m_DeltaStepOverCOMHeightMax = aDeltaStepOverCOMHeightMax;
}


void StepOverPlanner::CreateBufferFirstPreview(deque<COMPosition> &m_COMBuffer,
					       deque<ZMPPosition> &m_ZMPBuffer, 
					       deque<ZMPPosition> &m_ZMPRefBuffer)
{
  deque<ZMPPosition> aFIFOZMPRefPositions;
  VNL::Matrix<double> aPC1x,aPC1y; 
  double aSxzmp, aSyzmp;
  double aZmpx2, aZmpy2;
	
  //initialize ZMP FIFO
  for (unsigned int i=0;i<m_NL;i++)
    aFIFOZMPRefPositions.push_back(m_ZMPRefBuffer[i]);
	
  //use accumulated zmp error  of preview control so far
  aSxzmp = 0.0;//m_sxzmp;
  aSyzmp = 0.0;//m_syzmp;

  aPC1x.Resize(3,1);  aPC1y.Resize(3,1);

  aPC1x(0,0)= 0;    aPC1x(1,0)= 0;    aPC1x(2,0)= 0;
  aPC1y(0,0)= 0;    aPC1y(1,0)= 0;    aPC1y(2,0)= 0;

  //create the extra COMbuffer
	
#ifdef _DEBUG_
  ofstream aof_COMBuffer;
  static unsigned char FirstCall=1;
  if (FirstCall)
    {
      aof_COMBuffer.open("CartCOMBuffer_1.dat",ofstream::out);
    }
  else 
    {
      aof_COMBuffer.open("CartCOMBuffer_1.dat",ofstream::app);
    }
		
  if (FirstCall)
    FirstCall = 0;
#endif

  for (unsigned int i=0;i<m_ZMPRefBuffer.size()-m_NL;i++)
    {	
	
      aFIFOZMPRefPositions.push_back(m_ZMPRefBuffer[i+m_NL]);
		
      m_PC->OneIterationOfPreview(aPC1x,aPC1y,
				  aSxzmp,aSyzmp,
				  aFIFOZMPRefPositions,0,
				  aZmpx2, aZmpy2, true);
		
      for(unsigned j=0;j<3;j++)
	{
	  m_COMBuffer[i].x[j] = aPC1x(j,0);
	  m_COMBuffer[i].y[j] = aPC1y(j,0);
	}

      m_ZMPBuffer[i].px=aZmpx2;
      m_ZMPBuffer[i].py=aZmpy2;

      m_COMBuffer[i].theta = m_ZMPRefBuffer[i].theta;
		
      aFIFOZMPRefPositions.pop_front();
	
		
#ifdef _DEBUG_
      if (aof_COMBuffer.is_open())
	{
	  aof_COMBuffer << m_ZMPRefBuffer[i].time << " " << m_ZMPRefBuffer[i].px << " "<< m_COMBuffer[i].x[0]<< " " << m_COMBuffer[i].y[0] << endl;
	}
#endif
		
    }

	
#ifdef _DEBUG_
  if (aof_COMBuffer.is_open())
    {
      aof_COMBuffer.close();
    }
#endif 
  	
}







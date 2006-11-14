/* This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of steps.

   CVS Information: 
   $Id: ZMPDiscretization.cpp,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/ZMPDiscretization.cpp,v $
   $Log: ZMPDiscretization.cpp,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
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
#define _DEBUG_
#include <fstream>
#include <StepOverPlanner.h>


using namespace::PatternGeneratorJRL;

StepOverPlanner::StepOverPlanner(HumanoidSpecificites *aHS)
{
  m_HS = aHS;
  // Get information specific to the humanoid.
  double lWidth,lHeight;
  double AnklePosition[3];

  if (m_HS!=0)
    {
      m_HS->GetFootSize(-1,lWidth,lHeight);
      m_HS->GetAnklePosition(-1,AnklePosition);
      m_tipToAnkle = lWidth-AnklePosition[0];
      m_heelToAnkle = AnklePosition[2];	
  
    }
  else 
    {
      lWidth = 0.2; lHeight=0.15;
      cerr << "WARNING: no object with humanoid specificities properly defined." << endl;
      m_tipToAnkle = 0.1;
      m_heelToAnkle = 0.1;
  
    }  
  
  m_heelDistAfter=0.0;
  
  m_nominalStepLenght=0.2;
  m_nominalStepWidth=0.19;
  
  m_PolynomeStepOverX = new StepOverPolynomeFoot();
  m_PolynomeStepOverY = new StepOverPolynomeFoot();
  m_PolynomeStepOverZ = new StepOverPolynomeFoot();
  
  m_DMB = 0;
  m_PC = 0;
  m_IK = 0;
}

StepOverPlanner::~StepOverPlanner()
{
  if (m_PolynomeStepOverX!=0)
    delete m_PolynomeStepOverX;
  
  if (m_PolynomeStepOverY!=0)
    delete m_PolynomeStepOverY;
  
  if (m_PolynomeStepOverZ!=0)
    delete m_PolynomeStepOverZ;
}

void StepOverPlanner::CalculateFootHolds(vector<RelativeFootPosition> &aFootHolds,ObstaclePar &ObstacleParameters)
{
	 
  cout << "The obstacle parameters are" << endl;
  cout << "Obstacle height:" << ObstacleParameters.h << endl;
  cout << "Obstacle width:" << ObstacleParameters.w << endl;

  //add safety boundaries to the obstacle , the safety bounderies at the moment are chosen
  //but they can vary in the fuuter in function of the vision information uncertainty

  double safeBoundWidth=0.02;
  double safeBoundHeight=0.02;
	  
  ObstacleParameters.h+=safeBoundHeight;
  ObstacleParameters.w+=2.0*safeBoundWidth;

  //m_obstacles is visible and requered in the rest of the class 
  m_ObstacleParameters = ObstacleParameters;
        	  
  cout << "The obstacle parameters dimensions including safety boundary are" << endl;
  cout << "Obstacle height:" << ObstacleParameters.h << endl;
  cout << "Obstacle width:" << ObstacleParameters.w << endl;


  double sx=0.2,sy=0.19;
	
	  
  //at this moment we will assume the robot standing orthoganal to the obstacle 
  //thus only foothold calculation in the x-direction

  //calculation of the feasibility during double support over the obstacle 
  //this results in a stepwidth and hipheight during double support over the obstacle
  //and since the foot is positioned right after the obstacles safety margin the position
  //of the foot infront of the obstacle is known. 

  // double stepOverStepLenght,stepOverHipHeight;

  //the following normally is calculated with the stepover feasibility tool
	  
  m_StepOverStepLenght=0.35;
  m_StepOverHipHeight=0.65;

  double ankleDistToObstacle;

  ankleDistToObstacle=m_StepOverStepLenght-m_heelToAnkle-m_heelDistAfter-ObstacleParameters.w;
	  
  double  footDistLeftToMove;
	  
  footDistLeftToMove=ObstacleParameters.x-ankleDistToObstacle;

	  

  double numberOfSteps;
	    
  numberOfSteps=floor(footDistLeftToMove/m_nominalStepLenght);
	 
  double walkStepLenght;

  walkStepLenght=m_nominalStepLenght+(footDistLeftToMove-m_nominalStepLenght*numberOfSteps)/numberOfSteps;
	  
  cout << "Distance to Obstacle:" << ObstacleParameters.x << endl;
  cout << "Step Over steplenght:" << m_StepOverStepLenght << endl;
  cout << "Ankle distance in front of the obstacle:" <<  ankleDistToObstacle << endl;
  cout << "number of Steps before the obstacle:" << numberOfSteps << endl;
  cout << "Steplenght during walking:" << walkStepLenght << endl;

	
  RelativeFootPosition tempPos;
	 
  tempPos.sx=0.0;
  tempPos.sy=-m_nominalStepWidth/2;
  tempPos.theta=0.0; 
  tempPos.stepType=1;
	

  m_FootHolds.push_back(tempPos);

	    
  for (int i=0;i<numberOfSteps-1;i++)
    {
      tempPos.sx=walkStepLenght;
      tempPos.sy=(-1.0)*(tempPos.sy)/fabs(tempPos.sy)*m_nominalStepWidth;
      tempPos.theta=0.0;
      tempPos.stepType=1;
      m_FootHolds.push_back(tempPos);
    };

  // one step before stepover obstacle

  tempPos.sx=walkStepLenght;
  tempPos.sy=(-1.0)*(tempPos.sy)/fabs(tempPos.sy)*m_nominalStepWidth;
  tempPos.theta=0.0;
  tempPos.stepType=2;
  m_FootHolds.push_back(tempPos);
	 
  // first leg over the obsacle

  tempPos.sx=m_StepOverStepLenght;
  tempPos.sy=(-1.0)*(tempPos.sy)/fabs(tempPos.sy)*m_nominalStepWidth;
  tempPos.theta=0.0;
  tempPos.stepType=3;
  m_FootHolds.push_back(tempPos);

  // second leg over the obsacle

  tempPos.sx=m_nominalStepLenght;
  tempPos.sy=(-1.0)*(tempPos.sy)/fabs(tempPos.sy)*m_nominalStepWidth;
  tempPos.theta=0.0;
  tempPos.stepType=4;
  m_FootHolds.push_back(tempPos);

  //one step after the obstacle stepping over
  tempPos.sx=m_nominalStepLenght;
  tempPos.sy=(-1.0)*(tempPos.sy)/fabs(tempPos.sy)*m_nominalStepWidth;
  tempPos.theta=0.0;
  tempPos.stepType=5;
  m_FootHolds.push_back(tempPos);

  //one extra regular step
  tempPos.sx=m_nominalStepLenght;
  tempPos.sy=(-1.0)*(tempPos.sy)/fabs(tempPos.sy)*m_nominalStepWidth;
  tempPos.theta=0.0;
  tempPos.stepType=1;
  m_FootHolds.push_back(tempPos);
	
  //last step
  tempPos.sx=0;
  tempPos.sy=(-1.0)*(tempPos.sy)/fabs(tempPos.sy)*m_nominalStepWidth;
  tempPos.theta=0.0;
  tempPos.stepType=1;
  m_FootHolds.push_back(tempPos);  
	
  aFootHolds=m_FootHolds;
	
};

void StepOverPlanner::PolyPlanner()
{
  cout << "ik ben in globale de polyplanner" <<  endl;
	
  bool StepOverPlanned=false;
	
	
  unsigned int u=0;

  for (unsigned int i=0; i<m_LeftFootBuffer.size(); i++) 
    {
      if (fabs(m_LeftFootBuffer[i].stepType)==3)
	break;
    }	
  m_StartStepOver = i;
  m_StartDoubleSupp = 0;
  m_StartSecondStep = 0;
  for (unsigned int u=StartStepOver; u<m_LeftFootBuffer.size(); u++) 
    {	

      aExtraZMPRefBuffer.push_back(ZMPPositions[u+i]);
      aExtraRightFootBuffer.push_back(RightFootPositions[u+i]);
      aExtraLeftFootBuffer.push_back(LeftFootPositions[u+i]);
		
			
      if ((m_ExtraLeftFootBuffer[u].stepType==14)&(m_StartDoubleSupp==0))
	m_StartDoubleSupp = u;	
      if ((fabs(m_ExtraLeftFootBuffer[u].stepType)==4)&(m_StartSecondStep==0))
	m_StartSecondStep = u;
      if ((m_ExtraLeftFootBuffer[u].stepType==15))
	{	
	  m_EndStepOver = u;
	  break;
	}	
    }
	
  if(m_LeftFootBuffer[m_StartStepOver].stepType > 0)
    {
      PolyPlannerFirstStep(m_LeftFootBuffer);
      PolyPlannerSecondStep(m_RightFootBuffer);
    }
  else
    {
      PolyPlannerFirstStep(m_RightFootBuffer);
      PolyPlannerSecondStep(m_LeftFootBuffer);
    }
		
  /*
  //retrieve some information from the buffers to split the two steps and get sampletime
		
  m_ExtraBufferLength = m_ExtraLeftFootBuffer.size();
  m_SampleTime = m_ExtraLeftFootBuffer[1].time-m_ExtraLeftFootBuffer[0].time;

  m_StartDoubleSupp = 0;
  m_StartSecondStep = 0;
	
		
  for (unsigned int i=0;i<m_ExtraBufferLength;i++)
  {	
  if ((m_ExtraLeftFootBuffer[i].stepType==14)&(m_StartDoubleSupp==0))
  m_StartDoubleSupp = i;	
  if ((fabs(m_ExtraLeftFootBuffer[i].stepType)==4)&(m_StartSecondStep==0))
  m_StartSecondStep = i;
  i++;
  }
  /	if(m_ExtraLeftFootBuffer[0].stepType > 0)
  {
  PolyPlannerFirstStep(m_ExtraLeftFootBuffer);
  PolyPlannerSecondStep(m_ExtraRightFootBuffer);
  }
  else
  {
  PolyPlannerFirstStep(m_ExtraRightFootBuffer);
  PolyPlannerSecondStep(m_ExtraLeftFootBuffer);
  }

  cout << "startsecondstep"<<m_StartSecondStep<<endl;


  #ifdef _DEBUG_
  ofstream aof_ExtraBuffers;
  static unsigned char FirstCall=1;
  if (FirstCall)
  {
  aof_ExtraBuffers.open("ExtraBuffers_1.dat",ofstream::out);
  }
  else 
  {
  aof_ExtraBuffers.open("ExtraBuffers_1.dat",ofstream::app);
  }
		
  if (FirstCall)
  FirstCall = 0;
	
  for (unsigned int i=0;i<m_ExtraCOMBuffer.size();i++)
  {	
  if (aof_ExtraBuffers.is_open())
  {
  aof_ExtraBuffers << 
  m_ExtraLeftFootBuffer[i].time << " " << 
  m_ExtraCOMBuffer[i].x[i] << " "<< 	
  m_ExtraCOMBuffer[i].y[i]<< " " << 
  m_ExtraLeftFootBuffer[i].stepType << " " << 
  m_ExtraLeftFootBuffer[i].x << " " << 
  m_ExtraLeftFootBuffer[i].y << " " << 
  m_ExtraLeftFootBuffer[i].z << " " << 
  m_ExtraRightFootBuffer[i].stepType << " " << 
  m_ExtraRightFootBuffer[i].x << " " << 
  m_ExtraRightFootBuffer[i].y << " " << 
  m_ExtraRightFootBuffer[i].z << " " << 
  endl;
  }
  }
		

  if (aof_ExtraBuffers.is_open())
  {
  aof_ExtraBuffers.close();
  }
  #endif	
 
  //return 1;*/	
};

void StepOverPlanner::PolyPlannerFirstStep(vector<FootAbsolutePosition> &aFirstStepOverFootBuffer)
{
 
  VNL::Vector<double> aBoundCondZ(8,1),aBoundCondY(8,1),aBoundCondX(8,1); 	
  vector<double> aTimeDistr;
  double StepTime;
  double StepLenght;
  double Omega1,Omega2;
  double xOffset,zOffset;
  double Point1X,Point1Z;
  double Point2X,Point2Z;
	
  StepTime = aFirstStepOverFootBuffer[m_StartDoubleSupp-1].time-aFirstStepOverFootBuffer[0].time; 
  StepLenght = aFirstStepOverFootBuffer[m_StartDoubleSupp-1].x-aFirstStepOverFootBuffer[0].x; 
	
  xOffset=0.01;
  zOffset=0.01;

  Omega1=0.0;
  Omega2=0.0;

  //for now it is only in the 2D and with the obstacle perpendicular to absolute x direction
		
  Point1X = StepLenght-m_heelToAnkle-m_heelDistAfter-m_ObstacleParameters.w-xOffset-m_tipToAnkle*cos(Omega1);
  Point1Z = m_ObstacleParameters.h+zOffset-m_tipToAnkle*sin(Omega1);
	
  Point2X = StepLenght-m_heelToAnkle-m_heelDistAfter+xOffset+m_heelToAnkle*cos(Omega2);
  Point2Z = m_ObstacleParameters.h+zOffset+m_tipToAnkle*sin(Omega2);

	


  aTimeDistr.resize(3);
	
  aTimeDistr[0]=StepTime/3.0;
  aTimeDistr[1]=2.0*StepTime/3.0;
  aTimeDistr[2]=StepTime;
		
	  
	  
	
	 
	
  cout << "steplenght during step 3"<<StepLenght << endl;
		
  aBoundCondX(0)=0.0;
  aBoundCondX(1)=0.0;
  aBoundCondX(2)=0.0;
  aBoundCondX(3)=StepLenght;
  aBoundCondX(4)=0.0;
  aBoundCondX(5)=0.0;
  aBoundCondX(6)=Point1X;//StepLenght/3.0;
  aBoundCondX(7)=Point2X;//2.0*StepLenght/3.0;
  m_PolynomeStepOverX->SetParameters(aBoundCondX,aTimeDistr);
  m_PolynomeStepOverX->print(); 

  aBoundCondY(0)=0.0;
  aBoundCondY(1)=0.0;
  aBoundCondY(2)=0.0;
  aBoundCondY(3)=0.0;
  aBoundCondY(4)=0.0;
  aBoundCondY(5)=0.0;
  aBoundCondY(6)=0.0;
  aBoundCondY(7)=0.0;
  m_PolynomeStepOverY->SetParameters(aBoundCondY,aTimeDistr);
  m_PolynomeStepOverY->print(); 


  aBoundCondZ(0)=0.0;
  aBoundCondZ(1)=0.0;
  aBoundCondZ(2)=0.0;
  aBoundCondZ(3)=0.0;
  aBoundCondZ(4)=0.0;
  aBoundCondZ(5)=0.0;
  aBoundCondZ(6)=Point1Z;
  aBoundCondZ(7)=Point2Z;
  m_PolynomeStepOverZ->SetParameters(aBoundCondZ,aTimeDistr);
  m_PolynomeStepOverZ->print();       

  //update the footbuffers with the new calculated polynomials for stepping over
  unsigned int t = 0;
  double LocalTime;
	
  while(aFirstStepOverFootBuffer[t].stepType==3)
    {	
      LocalTime=(t)*m_SampleTime;	
      UpdatePosition(aFirstStepOverFootBuffer[t],aFirstStepOverFootBuffer[0],LocalTime);
      t++;
    }


	
};

void StepOverPlanner::PolyPlannerSecondStep(vector<FootAbsolutePosition> &aSecondStepOverFootBuffer)
{
 
  VNL::Vector<double> aBoundCondZ(8,1),aBoundCondY(8,1),aBoundCondX(8,1); 	
  vector<double> aTimeDistr;
  double StepTime;
  double StepLenght;
  double Omega1,Omega2;
  double xOffset,zOffset;
  double Point1X,Point1Z;
  double Point2X,Point2Z;
	
  StepTime = aSecondStepOverFootBuffer[m_ExtraBufferLength-1].time-aSecondStepOverFootBuffer[m_StartSecondStep].time; 
  StepLenght = aSecondStepOverFootBuffer[m_ExtraBufferLength-1].x-aSecondStepOverFootBuffer[m_StartSecondStep].x; 
	
  cout << "steplenght during step 4"<<StepLenght << endl;
  cout << "steptime during step 4"<<StepTime << endl;
  cout << " m_ExtraBufferLength during step 4"<<aSecondStepOverFootBuffer[m_ExtraBufferLength-1].time << endl;
  cout << " m_StartSecondStep during step 4"<<aSecondStepOverFootBuffer[m_ExtraBufferLength].time << endl;

  xOffset=0.01;
  zOffset=0.01;

  Omega1=0.0;
  Omega2=0.0;

  //for now it is only in the 2D and with the obstacle perpendicular to absolute x direction
		
  Point1X = m_StepOverStepLenght-m_heelToAnkle-m_heelDistAfter-m_ObstacleParameters.w-xOffset-m_tipToAnkle*cos(Omega1);
  Point1Z = m_ObstacleParameters.h+zOffset-m_tipToAnkle*sin(Omega1);
	
  Point2X = m_StepOverStepLenght-m_heelToAnkle-m_heelDistAfter+xOffset+m_heelToAnkle*cos(Omega2);
  Point2Z = m_ObstacleParameters.h+zOffset+m_tipToAnkle*sin(Omega2);

	


  aTimeDistr.resize(3);
	
  aTimeDistr[0]=StepTime/3.0;
  aTimeDistr[1]=2.0*StepTime/3.0;
  aTimeDistr[2]=StepTime;
		 	 		
  aBoundCondX(0)=0.0;
  aBoundCondX(1)=0.0;
  aBoundCondX(2)=0.0;
  aBoundCondX(3)=StepLenght;
  aBoundCondX(4)=0.0;
  aBoundCondX(5)=0.0;
  aBoundCondX(6)=Point1X;//StepLenght/3.0;
  aBoundCondX(7)=Point2X;//2.0*StepLenght/3.0;
  m_PolynomeStepOverX->SetParameters(aBoundCondX,aTimeDistr);
  m_PolynomeStepOverX->print(); 

  aBoundCondY(0)=0.0;
  aBoundCondY(1)=0.0;
  aBoundCondY(2)=0.0;
  aBoundCondY(3)=0.0;
  aBoundCondY(4)=0.0;
  aBoundCondY(5)=0.0;
  aBoundCondY(6)=0.0;
  aBoundCondY(7)=0.0;
  m_PolynomeStepOverY->SetParameters(aBoundCondY,aTimeDistr);
  m_PolynomeStepOverY->print(); 


  aBoundCondZ(0)=0.0;
  aBoundCondZ(1)=0.0;
  aBoundCondZ(2)=0.0;
  aBoundCondZ(3)=0.0;
  aBoundCondZ(4)=0.0;
  aBoundCondZ(5)=0.0;
  aBoundCondZ(6)=Point1Z;
  aBoundCondZ(7)=Point2Z;
  m_PolynomeStepOverZ->SetParameters(aBoundCondZ,aTimeDistr);
  m_PolynomeStepOverZ->print();       

  //update the footbuffers with the new calculated polynomials for stepping over
  unsigned int t = m_StartSecondStep;
  double LocalTime;
	
  while(aSecondStepOverFootBuffer[t].stepType==4)
    {	
      LocalTime=(t-m_StartSecondStep)*m_SampleTime;	
      UpdatePosition(aSecondStepOverFootBuffer[t],aSecondStepOverFootBuffer[m_StartSecondStep],LocalTime);
      t++;
    }


	
};



void StepOverPlanner::UpdatePosition(FootAbsolutePosition &aFootPositions,FootAbsolutePosition &iniFootPositions,double LocalTime)
{
  aFootPositions.x=
    m_PolynomeStepOverX->Compute(LocalTime)+iniFootPositions.x;
	
  aFootPositions.y=
    m_PolynomeStepOverY->Compute(LocalTime)+iniFootPositions.y;
	
  aFootPositions.z=
    m_PolynomeStepOverZ->Compute(LocalTime)+iniFootPositions.z;
	
}


void StepOverPlanner::SetExtraBuffer(vector<COMPosition> aExtraCOMBuffer,vector<FootAbsolutePosition> aExtraRightFootBuffer, vector<FootAbsolutePosition> aExtraLeftFootBuffer)
{
  m_ExtraCOMBuffer=aExtraCOMBuffer;
  m_ExtraRightFootBuffer = aExtraRightFootBuffer;
  m_ExtraLeftFootBuffer = aExtraLeftFootBuffer;
}



void StepOverPlanner::GetExtraBuffer(vector<COMPosition> &aExtraCOMBuffer,vector<FootAbsolutePosition> &aExtraRightFootBuffer, vector<FootAbsolutePosition> &aExtraLeftFootBuffer)
{
  aExtraCOMBuffer = m_ExtraCOMBuffer;
  aExtraRightFootBuffer = m_ExtraRightFootBuffer;
  aExtraLeftFootBuffer = m_ExtraLeftFootBuffer;
}

void StepOverPlanner::SetFootBuffers(vector<FootAbsolutePosition> aLeftFootBuffer, vector<FootAbsolutePosition> aRightFootBuffer)
{
  m_RightFootBuffer = aRightFootBuffer;
  m_LeftFootBuffer = aLeftFootBuffer;
}

void StepOverPlanner::GetFootBuffers(vector<FootAbsolutePosition> & aRightFootBuffer, vector<FootAbsolutePosition> & aLeftFootBuffer)
{
  aRightFootBuffer = m_RightFootBuffer;
  aLeftFootBuffer = m_LeftFootBuffer;
}


void StepOverPlanner::SetPreviewControl(PreviewControl *aPC)
{
  m_PC = aPC;
  m_SamplingPeriod = m_PC->SamplingPeriod();
  m_PreviewControlTime = m_PC->PreviewControlTime();
  m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);
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

void StepOverPlanner::CreateBufferFirstPreview(vector<COMPosition> &m_COMBuffer,vector<ZMPPosition> &m_ZMPBuffer, vector<ZMPPosition> &m_ZMPRefBuffer)

{

  cout << "I am making buffer with COM Positions" << endl;
 
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
	
      aFIFOZMPRefPositions.push_back(m_ZMPRefBuffer[i+m_NL+1]);
		
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
	  aof_COMBuffer << m_ZMPRefBuffer[i].time << " " 
			<< m_ZMPRefBuffer[i].px << " "
			<< m_COMBuffer[i].x[0]<< " " 
			<< m_COMBuffer[i].y[0] << endl;
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

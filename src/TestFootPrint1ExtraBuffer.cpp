#include <sys/time.h>
#include <time.h>
#include <ZMPPreviewControlWithMultiBodyZMP.h>
#include <ZMPDiscretization.h>
#include <PreviewControl.h>
#include <InverseKinematics.h>
#include <DynamicMultiBody.h>
#include <fstream>
#include <StepOverPlanner.h>

using namespace::PatternGeneratorJRL;

int main(void)
{

  double sx=0.2,sy=0.19;
#if 0
  RelativeFootPosition TabFoots[7] = {
    { 0.0, -sy/2, 0.0},
    {  sx,  sy, 0.0},
    {  sx, -sy, 0.0},
    {  sx,  sy, 0.0},
    {  sx, -sy, 0.0},
    {  sx,  sy, 0.0},
    { 0.0, -sy, 0.0}
    
  };
#endif

  RelativeFootPosition TabFoots2[12] = {
    { 0.0, -sy/2, 0.0},
    {  sx,  sy, 0.0},
    {  sx, -sy, 10.0},
    {  sx,  sy, 10.0},
    {  sx, -sy, 10.0},
    {  sx,  sy, 10.0},
    {  sx, -sy, 10.0},
    {  sx,  sy, 10.0},
    {  sx, -sy, 10.0},
    {  sx,  sy, 10.0},
    {  sx, -sy, 10.0},
    { 0.0,  sy, 0.0}
    
  };

ObstaclePar ObstaclePars;

  ObstaclePars.x=1;
  ObstaclePars.y=1;
  ObstaclePars.theta=0;
  ObstaclePars.h=0.1;
  ObstaclePars.w=0.05;
  

  vector<ZMPPosition> ZMPPositions;
  vector<FootAbsolutePosition> FootAbsolutePositions;
  vector<RelativeFootPosition> RelativeFootPositions;
  vector<FootAbsolutePosition> LeftFootPositions,RightFootPositions;
  vector<FootAbsolutePosition> LeftHandPositions,RightHandPositions;

  ZMPPreviewControlWithMultiBodyZMP *aZMPpcwmbz=0;
  DynamicMultiBody *aDMB=0,*a2DMB=0,*a3DMB=0;
  ZMPDiscretization *aZMPDiscretization = 0;
  PreviewControl *aPC = 0;
  InverseKinematics *anIK = 0;

  StepOverPlanner *aStOvPl=0;

  aDMB = new DynamicMultiBody();
  a2DMB = new DynamicMultiBody();
  a3DMB = new DynamicMultiBody();
  string HRP2JRLpath = "../../etc/HRP2JRL/";
  string HRP2JRLfilename = "HRP2JRLmain.wrl";
  string HRP2fullpath= "./";
  string HRP2fullfilename = "HRP2main_full.wrl";

  aDMB->parserVRML(HRP2JRLpath,HRP2JRLfilename,"");
  a2DMB->parserVRML(HRP2JRLpath,HRP2JRLfilename,"");
  a3DMB->parserVRML(HRP2JRLpath,HRP2JRLfilename,"");  

  //aDMB->parserVRML(HRP2fullpath,HRP2fullfilename,"");
  //  aDMB->afficherLiaisons();

  aZMPDiscretization = new ZMPDiscretization();
  aPC = new PreviewControl;
  anIK = new InverseKinematics();

  

  string PCParameters="PreviewControlParameters.ini";
  aPC->ReadPrecomputedFile(PCParameters);
  double SamplingPeriod = aPC->SamplingPeriod();
  double PreviewControlTime = aPC->PreviewControlTime();
  unsigned int NL = (unsigned int)(PreviewControlTime/SamplingPeriod);

  aStOvPl = new StepOverPlanner();
  aStOvPl->SetPreviewControl(aPC);
  aStOvPl->SetDynamicMultiBodyModel(aDMB);
  aStOvPl->SetInverseKinematics(anIK);

  aZMPpcwmbz = new ZMPPreviewControlWithMultiBodyZMP();

  aZMPpcwmbz->SetPreviewControl(aPC);
  aZMPpcwmbz->SetDynamicMultiBodyModel(aDMB);
  aZMPpcwmbz->SetInverseKinematics(anIK);

  
  
  ofstream arof, alof;
  alof.open("LeftLegJointAngles.dat",ofstream::out);
  arof.open("RightLegJointAngles.dat",ofstream::out);

   aStOvPl->CalculateFootHolds(RelativeFootPositions,ObstaclePars);
 
/* // First sequence of steps.
#if 0
  for(int i=0;i<7;i++)
    RelativeFootPositions.push_back(TabFoots[i]);
#else
  for(int i=0;i<12;i++)
    RelativeFootPositions.push_back(TabFoots2[i]);
#endif
*/
  double Xmax;
  double ZARM=-1.0;
  Xmax = anIK->ComputeXmax(ZARM); // Laaaaaazzzzzyyyyy guy...
  cout << "XMAX " << Xmax << endl;

  aZMPDiscretization->GetZMPDiscretization(ZMPPositions,
					   FootAbsolutePositions,
					   RelativeFootPositions,
					   LeftFootPositions,
					   RightFootPositions,
					   LeftHandPositions,
					   RightHandPositions,
					   Xmax);
  
  string ZMPPos1="ZMPPositions_1.dat";
  string FootPos1="FootPositions_1.dat";
  aZMPDiscretization->DumpDataFiles(ZMPPos1, FootPos1,
				    ZMPPositions, FootAbsolutePositions);
  
  vector<COMPosition> aCOMBuffer;
  vector<ZMPPosition> aZMPBuffer;
 
  aCOMBuffer.resize(RightFootPositions.size());
  aZMPBuffer.resize(RightFootPositions.size());
	
  aStOvPl->CreateBufferFirstPreview(aCOMBuffer,aZMPBuffer,ZMPPositions);
  aStOvPl->SetFootBuffers(LeftFootPositions,RightFootPositions);

  ofstream aofpos;
  aofpos.open("Pos_1.dat",ofstream::out);

  ofstream aofpos2;
  aofpos2.open("Pos_2.dat",ofstream::out);

  ofstream aofspeed;
  aofspeed.open("Speed_1.dat",ofstream::out);

  ofstream aofspeed2;
  aofspeed2.open("Speed_2.dat",ofstream::out);

  ofstream aof_errorspeed;
  aof_errorspeed.open("ErrorSpeed_1.dat",ofstream::out);

  ofstream aof_FootError;
  aof_FootError.open("FootError_1.dat", ofstream::out);

  ofstream aof_COMPos;
  aof_COMPos.open("COMPositions_1.dat", ofstream::out);

  ofstream aof_Momentum;
  aof_Momentum.open("Momentums.dat", ofstream::out);

  ofstream aof_GlobalMomentum;
  aof_GlobalMomentum.open("GlobalMomentums.dat", ofstream::out);

  ofstream aof_dq;
  aof_dq.open("dq.dat", ofstream::out);

  ofstream aof_ZMPRMB;
  aof_ZMPRMB.open("ZMPMBRecomputed.dat", ofstream::out);

  ofstream aof_ARMANGLES;
  aof_ARMANGLES.open("ArmAngles.dat", ofstream::out);

  ofstream aof_ARMX;
  aof_ARMX.open("ArmX.dat", ofstream::out);

  ofstream aof_Finaldqleg;
  aof_Finaldqleg.open("Finaldqleg.dat", ofstream::out);

  
  vector3d ErrorAccumulation;
  ErrorAccumulation.x = 0 ;
  ErrorAccumulation.y = 0 ;
  ErrorAccumulation.z = 0 ;

  for(int i=0;i<aDMB->NbOfLinks();i++)
    {
      aDMB->Setdq(i,0.0);
      aDMB->Setv(i,0.0);
      aDMB->Setw(i,0.0);
      a2DMB->Setdq(i,0.0);
      a2DMB->Setv(i,0.0);
      a2DMB->Setw(i,0.0);
      a3DMB->Setdq(i,0.0);
      a3DMB->Setv(i,0.0);
      a3DMB->Setw(i,0.0);
    }
 
cout<<"hello"<<endl;



cout<<"before setup"<<endl;
  int Lindex=0;
  // Read NL informations from ZMPRefPositions.
  aZMPpcwmbz->Setup(ZMPPositions,
		    LeftFootPositions,
		    RightFootPositions);

  Lindex+=NL;
  
  VNL::Matrix<double> prev_qr(6,1), prev_ql(6,1);
  VNL::Matrix<double> prev_qArmr(7,1), prev_qArml(7,1);
  VNL::Matrix<double> Finalq(41,1);
  vector3d prev_P,prev_L;
  prev_P.x = prev_P.y = prev_P.z =
    prev_L.x = prev_L.y = prev_L.z = 0.0;
  int LINKSFORRLEG[6] = { 0, 1, 2, 3,  4, 5};
  int LINKSFORLLEG[6] = { 6, 7, 8, 9, 10, 11};
  int LINKSFORRARM[7] = { 16, 17, 18, 19, 20, 21, 22};
  int LINKSFORLARM[7] = { 23, 24, 25, 26, 27, 28, 29};
  
  VNL::Matrix<double> dql(6,1), dqr(6,1);
  VNL::Matrix<double> dqArml(7,1), dqArmr(7,1);
  // Resolved Momentum Control parameters.
  vector<int> LeftLeg, RightLeg,FreeJoints;
  LeftLeg.resize(6);
  RightLeg.resize(6);
  for(unsigned int i=0;i<6;i++)
    {
      LeftLeg[i] = LINKSFORLLEG[i];
      RightLeg[i] = LINKSFORRLEG[i];
    }

  FreeJoints.resize(a2DMB->NbOfLinks()-
		    LeftLeg.size() - RightLeg.size());

  for(int i=12;i<a2DMB->NbOfLinks();i++)
    {
      FreeJoints[i-12]=i;
    }
  cout << "Number of free joints " << FreeJoints.size() <<endl;
  
   VNL::Matrix<double> S(4,6);
  VNL::Matrix<double>  PLref(6,1);
  matrix3d FinalWaistOrientation;
  matrix3d Ro;
  FinalWaistOrientation.setIdentity();
  // 

  struct timeval time_start,time_end;
  gettimeofday(&time_start,0);
  double GainX = Xmax/0.2;
  cout << "GainX " << GainX << endl;
	
  

  bool StepOverPlanned=false;
  vector<COMPosition> aExtraCOMBuffer;
  vector<ZMPPosition> aExtraZMPBuffer;
  vector<ZMPPosition> aExtraZMPRefBuffer;
  vector<FootAbsolutePosition> aExtraRightFootBuffer, aExtraLeftFootBuffer;

  
  for(unsigned int i=NL;i<ZMPPositions.size()-NL;i++)
    {
      VNL::Matrix<double> lqr(6,1),lql(6,1);
      VNL::Matrix<double> lqArmr(7,1),lqArml(7,1);
      COMPosition aCOMPosition;
      	
      	 
 
     if ((fabs(LeftFootPositions[i].stepType)==3)&&(!StepOverPlanned))
	{
		//determine lenght of extra buffer
		unsigned int u=0;
		
		//create a buffer of ZMP references required to calculate the extra COM buffer by first stage of preview control
		while (ZMPPositions[u+i].stepType!=15)
		{	
			aExtraZMPRefBuffer.push_back(ZMPPositions[u+i]);
			aExtraRightFootBuffer.push_back(RightFootPositions[u+i]);
			aExtraLeftFootBuffer.push_back(LeftFootPositions[u+i]);
			u++;
		}
		//add NL extra ZMPRefPositions for the preview control
		for (unsigned int v=0;v<NL;v++)
			aExtraZMPRefBuffer.push_back(ZMPPositions[u+i+v]);

		aExtraCOMBuffer.resize(u);
		aExtraZMPBuffer.resize(u);
		aZMPpcwmbz->CreateExtraCOMBuffer(aExtraCOMBuffer,aExtraZMPBuffer,aExtraZMPRefBuffer);
		
		/*for (unsigned int v=0;v<aExtraCOMBuffer.size();v++)
		{
			cout << "aExtraCOMBuffer" << "" << aExtraCOMBuffer[v].x[0] <<endl;
		}
		*/

		aStOvPl->SetExtraBuffer(aExtraCOMBuffer,aExtraRightFootBuffer, aExtraLeftFootBuffer);
				 
		/*float aTsingle,aTdouble;  
		float aSamplingPeriod;  
		unsigned int nPointsSingle,nPointsDouble;

		aTsingle = aZMPDiscretization->GetTSingleSupport();
		aTdouble = aZMPDiscretization->GetTDoubleSupport();
		aSamplingPeriod = aZMPDiscretization->GetSamplingPeriod();
		nPointsSingle = (unsigned int)(aTsingle/aSamplingPeriod);
		nPointsDouble = (unsigned int)(aTdouble/aSamplingPeriod);
		cout << "Ik ben aan het plannen" << RightFootPositions[i+nPointsSingle].x-RightFootPositions[i].x << endl;
		*/

		aStOvPl->PolyPlanner();

		aStOvPl->GetExtraBuffer(aExtraCOMBuffer,aExtraRightFootBuffer,aExtraLeftFootBuffer);
			
		for (unsigned int v=0;v<aExtraLeftFootBuffer.size();v++)
		{
			RightFootPositions[i+v]=aExtraRightFootBuffer[v];
			LeftFootPositions[i+v]=aExtraLeftFootBuffer[v];
		}
		
		
		int aStepType;
		unsigned int t=i;
		double LocalTime; 
		aStepType=ZMPPositions[i].stepType;

		
	


		StepOverPlanned=!StepOverPlanned;

				
	}

           

      aZMPpcwmbz->OneGlobalStepOfControl(LeftFootPositions[i],
					 RightFootPositions[i],
					 ZMPPositions[i+NL],
					 lqr,lql,
					 aCOMPosition);
	  
      // Compute the arm angles.
      double Alpha,Beta;
      //Temporary variables
      double TempXL,TempXR,TempCos,TempSin;
      TempCos = cos(aCOMPosition.theta*M_PI/180.0);
      TempSin = sin(aCOMPosition.theta*M_PI/180.0);

      TempXL = TempCos * (RightFootPositions[i-NL].x  - aCOMPosition.x[0]) + TempSin * (RightFootPositions[i-NL].y  - aCOMPosition.y[0]);

      anIK->ComputeInverseKinematicsForArms(TempXL*GainX,
					    ZARM,
					    Alpha,
					    Beta);
      lqArml(0,0)=Alpha;
      lqArml(1,0)=10.0*M_PI/180.0;
      lqArml(2,0)= 0.0;
      lqArml(3,0)= Beta;
      lqArml(4,0)= 0.0;
      lqArml(5,0)= 0.0;
      lqArml(6,0)= 10.0*M_PI/180.0;

      TempXR = TempCos * (LeftFootPositions[i-NL].x  - aCOMPosition.x[0]) + TempSin * (LeftFootPositions[i-NL].y  - aCOMPosition.y[0]);

      anIK->ComputeInverseKinematicsForArms(TempXR*GainX,
					    ZARM,
					    Alpha,
					    Beta);
      lqArmr(0,0)=Alpha;
      lqArmr(1,0)=-10.0*M_PI/180.0;
      lqArmr(2,0)= 0.0;
      lqArmr(3,0)= Beta;
      lqArmr(4,0)= 0.0;
      lqArmr(5,0)= 0.0;
      lqArmr(6,0)= 10.0*M_PI/180.0;;

	
      for(unsigned int j=0;j<6;j++)
	{
	  a2DMB->Setq(LINKSFORLLEG[j],lql(j,0));
	  a2DMB->Setq(LINKSFORRLEG[j],lqr(j,0));
	  
	  if (i!=NL)
	    {
	      dql(j,0) = (lql(j,0)-prev_ql(j,0))/SamplingPeriod;
	      dqr(j,0) = (lqr(j,0)-prev_qr(j,0))/SamplingPeriod;
	    }
	  else
	    {
	      dql(j,0) = 0;
	      dqr(j,0) = 0;
	    }
	  
	  a2DMB->Setdq(LINKSFORLLEG[j],dql(j,0));
	  a2DMB->Setdq(LINKSFORRLEG[j],dqr(j,0));
	  prev_ql(j,0) = lql(j,0);
	  prev_qr(j,0) = lqr(j,0);
	}

      for(unsigned int j=0;j<7;j++)
	{
	  a2DMB->Setq(LINKSFORLARM[j],lqArml(j,0));
	  a2DMB->Setq(LINKSFORRARM[j],lqArmr(j,0));
	  
	  if (i!=NL)
	    {
	      dqArml(j,0) = (lqArml(j,0)-prev_qArml(j,0))/SamplingPeriod;
	      dqArmr(j,0) = (lqArmr(j,0)-prev_qArmr(j,0))/SamplingPeriod;
	    }
	  else
	    {
	      dqArml(j,0) = 0;
	      dqArmr(j,0) = 0;
	    }
	  
	  a2DMB->Setdq(LINKSFORLARM[j],dqArml(j,0));
	  a2DMB->Setdq(LINKSFORRARM[j],dqArmr(j,0));
	  prev_qArml(j,0) = lqArml(j,0);
	  prev_qArmr(j,0) = lqArmr(j,0);
	}

      vector3d WaistPosition,WaistVelocity;
      
      WaistPosition.x = aCOMPosition.x[0];
      WaistPosition.y = aCOMPosition.y[0];
      WaistPosition.z = aCOMPosition.omega-0.705;
      
      WaistVelocity.x = aCOMPosition.x[1];
      WaistVelocity.y = aCOMPosition.y[1];
      WaistVelocity.z = 0.0;
      
      matrix3d Body_Rm3d;
      double c = cos(aCOMPosition.theta*M_PI/180.0);
      double s = sin(aCOMPosition.theta*M_PI/180.0);

      Body_Rm3d.m[0] = c;       Body_Rm3d.m[1] = -s;       Body_Rm3d.m[2] = 0;
      Body_Rm3d.m[3] = s;       Body_Rm3d.m[4] =  c;       Body_Rm3d.m[5] = 0;
      Body_Rm3d.m[6] = 0;       Body_Rm3d.m[7] = 0;        Body_Rm3d.m[8] = 1;

      a2DMB->ForwardVelocity(WaistPosition,Body_Rm3d,WaistVelocity);
      
      double ZMPmultibody[2];
      vector3d P,L,  dP, dL;
      a2DMB->GetPandL(P,L);
      
      // Approximate first order derivative of linear and angular momentum.
      dP = (P - prev_P)/SamplingPeriod;
      dL = (L - prev_L)/SamplingPeriod;
      prev_P = P;
      prev_L = L;
      // Compute ZMP
      double ZMPz = 0.0;
      vector3d a2COMPosition;      
      a2DMB->CalculateZMP(ZMPmultibody[0],ZMPmultibody[1],
      			  dP, dL, ZMPz);
      a2COMPosition = a2DMB->getPositionCoM();

      // Computes dTheta for the legs.


      //cout << "Iteration Number " << i << endl;
      //      cout << XiBdThetaFree <<endl;
      //      cout << "***********************************"<<endl;


      
      /*      cout << "COMPosition (" 
	   << FinalCOMPos.x << " "
	   << FinalCOMPos.y << " " 
	   << FinalCOMPos.z << endl;
      */
      if(0)
	{
	  if(aof_Finaldqleg.is_open())
	    {
	      for(unsigned int i=0;i<6;i++)
		aof_Finaldqleg << dql(i,0) <<" ";

	      for(unsigned int i=0;i<6;i++)
		aof_Finaldqleg << dqr(i,0) <<" ";
	      aof_Finaldqleg << endl;
	      
	    }
	  if (aof_ARMX.is_open())
	    {
	      aof_ARMX << TempXL*GainX << " " ;
	      aof_ARMX << TempXR*GainX << endl ;
	      
	    }
	  if (aof_ARMANGLES.is_open())
	    {
	      for(unsigned int i=0;i<7;i++)
		aof_ARMANGLES << lqArml(i,0) << " ";

	      for(unsigned int i=0;i<7;i++)
		aof_ARMANGLES << lqArmr(i,0) <<" ";

	      aof_ARMANGLES << endl;
	    }

	  if (aof_ZMPRMB.is_open())
	    {
	      aof_ZMPRMB <<  ZMPPositions[i-NL].time << " "  <<
		ZMPmultibody[0] << " " << ZMPmultibody[1] << " " <<
		P.x << " " << P.y << " " << P.z << " " <<
		L.x << " " << L.y << " " << L.z << endl;
	    }
	

	  if (alof.is_open())
	    {
	      for(unsigned int j=0;j<6;j++)
		{
		  alof << lql(j,0) << " ";
		}
	      alof<<endl;
	    }


	  if (arof.is_open())
	    {
	      for(unsigned int j=0;j<6;j++)
		arof << lqr(j,0) << " ";
	      arof<<endl;
	    }


	  if (aof_errorspeed.is_open())
	    {
	      vector3d lv = aDMB->Getw(1)- aDMB->Getw(4);
	      ErrorAccumulation += lv;
	      aof_errorspeed << ErrorAccumulation.x << " " 
			     << ErrorAccumulation.y << " " 
			     << ErrorAccumulation.z << endl;
	    }

	  if (aofspeed.is_open())
	    {
	  
	      aofspeed << ZMPPositions[i-NL].time << " " ;
	      for(int j=0;j<aDMB->NbOfLinks();j++)
		{
		  
		  vector3d avec = aDMB->Getv(j);
		  aofspeed << avec.x << " " <<  avec.y << " "  << avec.z << " " ;
		  avec = aDMB->Getw(j);
		  aofspeed << avec.x << " " <<  avec.y << " "  << avec.z << " " ;
		}

	      aofspeed << endl;

	    }

	  if (aofspeed2.is_open())
	    {
	      aofspeed2 << ZMPPositions[i-NL].time << " " ;
	      for(int j=0;j<a2DMB->NbOfLinks();j++)
		{
		  
		  vector3d avec = a2DMB->Getv(j);
		  aofspeed2 << avec.x << " " <<  avec.y << " "  << avec.z << " " ;
		  avec = a2DMB->Getw(j);
		  aofspeed2 << avec.x << " " <<  avec.y << " "  << avec.z << " " ;
		}

	      aofspeed2 << endl;

	    }

	  if (aofpos.is_open())
	    {
	      for(int j=0;j<aDMB->NbOfLinks();j++)
		{
		  vector3d avec = aDMB->Getp(j);
		  aofpos << avec.x << " " <<  avec.y << " "  << avec.z << " " ;
		  if (j==5) 
		    {
		      aof_FootError << RightFootPositions[i].x - avec.x << " " ;
		      aof_FootError << RightFootPositions[i].y - avec.y << " " ;
		      aof_FootError << RightFootPositions[i].z + 0.105 - avec.z << " ";
		    }
		  if (j==11)
		    {
		      aof_FootError << LeftFootPositions[i].x - avec.x << " ";
		      aof_FootError << LeftFootPositions[i].y - avec.y << " ";
		      aof_FootError << LeftFootPositions[i].z + 0.105 - avec.z << " ";
		    }
		}
	      aof_FootError<<endl;

	      aofpos << endl;
	    }
      

	  if (aofpos2.is_open())
	    {
	      for(int j=0;j<a2DMB->NbOfLinks();j++)
		{
		  vector3d avec = a2DMB->Getp(j);
		  aofpos2 << avec.x << " " <<  avec.y << " "  << avec.z << " " ;
		}
	      aofpos2 << endl;
	    }

	  if (aof_Momentum.is_open())
	    {
	      aof_Momentum << ZMPPositions[i-NL].time << " ";
	      for(int j=0;j<aDMB->NbOfLinks();j++)
		{
		  vector3d avec = a2DMB->GetL(j);
		  aof_Momentum << avec.x << " " <<  avec.y << " "  << avec.z << " " ;
		  avec = a2DMB->GetP(j);
		  aof_Momentum << avec.x << " " <<  avec.y << " "  << avec.z << " " ;
		}
	      aof_Momentum<<endl;
	    }
      
	  if (aof_GlobalMomentum.is_open())
	    {
	      vector3d lP,lL;
	      aDMB->GetPandL(lP,lL);
	      aof_GlobalMomentum << lP.x << " " << lP.y << " " << lP.z <<" ";
	      aof_GlobalMomentum << lL.x << " " << lL.y << " " << lL.z <<" ";
	      aof_GlobalMomentum << endl;
	    }
	      
	  if (aof_COMPos.is_open())
	    {
	      aof_COMPos << ZMPPositions[i-NL].time << " " 
			 << aCOMPosition.x[0] << " " << aCOMPosition.y[0] << " "
			 << aCOMPosition.x[1] << " " << aCOMPosition.y[1] << " 0.68"<< endl;
	    }

	  if (aof_dq.is_open())
	    {
	      for(int j=0;j<a2DMB->NbOfLinks();j++)
		aof_dq << a2DMB->Getdq(j) << " ";
	      aof_dq << endl;
	    }
	}
    }
  
  string LAbsPos = "LeftFootAbsPos_1.dat";
  string RAbsPos = "RightFootAbsPos_1.dat";
  aZMPDiscretization->DumpFootAbsolutePosition(LAbsPos, LeftFootPositions);
  aZMPDiscretization->DumpFootAbsolutePosition(RAbsPos, RightFootPositions);

  gettimeofday(&time_end,0);
  double GlobalTime = time_end.tv_sec - time_start.tv_sec +
    0.000001 * (time_end.tv_sec - time_start.tv_sec );
  double Average = GlobalTime / (ZMPPositions.size()-2*NL);
  cout << "Total Time consumption: "<< GlobalTime << " Average consumption time: " << Average << endl;

  if (aof_ARMX.is_open())
    aof_ARMX.close();

  if (aof_ARMANGLES.is_open())
    aof_ARMANGLES.close();

  if (aof_errorspeed.is_open())
    aof_errorspeed.close();

  if (aofspeed.is_open())
    aofspeed.close();

  if (aofspeed2.is_open())
    aofspeed2.close();
  
  if (aofpos.is_open())
    aofpos.close();

  if (aofpos2.is_open())
    aofpos2.close();

  if (aof_FootError.is_open())
    aof_FootError.close();

  if (aof_COMPos.is_open())
    aof_COMPos.close();

  if (aof_ZMPRMB.is_open())
    aof_ZMPRMB.close();
  
  if (aof_Finaldqleg.is_open())
    aof_Finaldqleg.close();

  // Test of Preview control.
  if (anIK!=0)
    delete anIK;
  
  if (aof_Momentum.is_open())
    aof_Momentum.close();

  if (aof_GlobalMomentum.is_open())
    aof_GlobalMomentum.close();

  if (aof_dq.is_open())
    aof_dq.close();

  if (aZMPDiscretization!=0)
    delete aZMPDiscretization;

  if (aPC!=0)
    delete aPC;
    
}

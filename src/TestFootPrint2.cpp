#include <sys/time.h>
#include <time.h>
#include <ZMPPreviewControlWithMultiBodyZMP.h>
#include <ZMPDiscretization.h>
#include <PreviewControl.h>
#include <InverseKinematics.h>
#include <DynamicMultiBody.h>
#include <fstream>
#include <StepOverPlanner.h>
#include <CollisionDetector.h>
#include <WaistHeightVariation.h>
#include <UpperBodyMotion.h>

using namespace::PatternGeneratorJRL;

int main(void)
{

  double sx=0.2,sy=0.19;
  int walkmode = 0;

  RelativeFootPosition TabFoots[7] = {
    { 0.0, -sy/2, 0.0},
    {  sx,  sy, 0.0},
    {  sx, -sy, 0.0},
    {  sx,  sy, 0.0},
    {  sx, -sy, 0.0},
    {  sx,  sy, 0.0},
    { 0.0, -sy, 0.0}
    
  };

#if 0
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
#endif

  // Obstacle parameters
  ObstaclePar ObstaclePars;

  ObstaclePars.x=1.0;
  ObstaclePars.y=0.0;
  ObstaclePars.z=0.0;
  ObstaclePars.theta= 0.0;
  ObstaclePars.h=.02;
  ObstaclePars.w=1.0;
  ObstaclePars.d=.05;
  

  deque<ZMPPosition> ZMPPositions;
  deque<FootAbsolutePosition> FootAbsolutePositions;
  deque<RelativeFootPosition> RelativeFootPositions;
  deque<FootAbsolutePosition> LeftFootPositions,RightFootPositions;
  deque<FootAbsolutePosition> LeftHandPositions,RightHandPositions;
  
  ZMPPreviewControlWithMultiBodyZMP *aZMPpcwmbz=0;
  DynamicMultiBody *aDMB=0,*a2DMB=0,*a3DMB=0;
  ZMPDiscretization *aZMPDiscretization = 0;
  PreviewControl *aPC = 0;
  InverseKinematics *anIK = 0;

  StepOverPlanner *aStOvPl=0;
  WaistHeightVariation *aWaistPlanner=0;
  UpperBodyMotion *aUpBody=0;


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

  aStOvPl = new StepOverPlanner(ObstaclePars);
  aStOvPl->SetPreviewControl(aPC);
  aStOvPl->SetDynamicMultiBodyModel(aDMB);
  aStOvPl->SetInverseKinematics(anIK);
  aStOvPl->SetZMPDiscretization(aZMPDiscretization);
  
  aWaistPlanner = new WaistHeightVariation();
  aUpBody = new UpperBodyMotion();
  double DiffBetweenComAndWaist = -0.1656;

  aZMPpcwmbz = new ZMPPreviewControlWithMultiBodyZMP();

  aZMPpcwmbz->SetPreviewControl(aPC);
  aZMPpcwmbz->SetDynamicMultiBodyModel(aDMB);
  aZMPpcwmbz->SetInverseKinematics(anIK);
  
  ofstream arof, alof;
  alof.open("LeftLegJointAngles.dat",ofstream::out);
  arof.open("RightLegJointAngles.dat",ofstream::out);

  double Xmax=0;
  // First sequence of steps.
#if 1
  for(int i=0;i<7;i++)
    RelativeFootPositions.push_back(TabFoots[i]);
#else
  for(int i=0;i<12;i++)
    RelativeFootPositions.push_back(TabFoots2[i]);
#endif

  VNL::Vector<double> lSCOM(3,0.0);
  PatternGeneratorJRL::FootAbsolutePosition InitLFPos, InitRFPos;
  bzero(&InitLFPos,sizeof(InitLFPos));
  bzero(&InitRFPos,sizeof(InitRFPos));

  aZMPDiscretization->GetZMPDiscretization(ZMPPositions,
					   FootAbsolutePositions,
					   RelativeFootPositions,
					   LeftFootPositions,
					   RightFootPositions,
					   LeftHandPositions,
					   RightHandPositions,
					   Xmax,lSCOM,InitLFPos,InitRFPos);
  
  string ZMPPos1="ZMPPositions_1.dat";
  string FootPos1="FootPositions_1.dat";
  aZMPDiscretization->DumpDataFiles(ZMPPos1, FootPos1,
				    ZMPPositions, FootAbsolutePositions);

  
  string LAbsPos = "LeftFootAbsPos_1.dat";
  string RAbsPos = "RightFootAbsPos_1.dat";
  aZMPDiscretization->DumpFootAbsolutePosition(LAbsPos, LeftFootPositions);
  aZMPDiscretization->DumpFootAbsolutePosition(RAbsPos, RightFootPositions);

  string BodyDat = "UpperBodyDataFile.dat";
  int LenghtDataArray;
	

  LenghtDataArray = LeftFootPositions.size();

  aUpBody->GenerateDataFile(BodyDat,LenghtDataArray);

  VNL::Matrix<double> UpperBodyAnglesBuffer;
  UpperBodyAnglesBuffer.Resize(LenghtDataArray,28);

  aUpBody->ReadDataFile(BodyDat,UpperBodyAnglesBuffer);

	


  deque<COMPosition> aCOMBuffer;
  deque<ZMPPosition> aZMPBuffer;
 
  aCOMBuffer.resize(RightFootPositions.size());
  aZMPBuffer.resize(RightFootPositions.size());
 
  //fill buffer with preliminary COM height information
  
 
  for(unsigned int i=0;i<aCOMBuffer.size();i++)
    {
      aCOMBuffer[i].z[0] = aPC->GetHeightOfCoM();
      aCOMBuffer[i].z[1] = 0.0;
      aCOMBuffer[i].z[2] = 0.0;
    }
	
  aStOvPl->CreateBufferFirstPreview(aCOMBuffer,aZMPBuffer,ZMPPositions);
 
  if (walkmode == 1)	
    aWaistPlanner->PolyPlanner(aCOMBuffer,RelativeFootPositions,ZMPPositions);
  else if (walkmode == 2)	
    aStOvPl->PolyPlanner(aCOMBuffer,LeftFootPositions,RightFootPositions,ZMPPositions);

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

  ofstream aof_XiThetaFree;
  aof_XiThetaFree.open("XiThetaFree.dat", ofstream::out);

  ofstream aof_XiLeftFoot;
  aof_XiLeftFoot.open("XiLeftFoot.dat", ofstream::out);

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


  VNL::Matrix<double> BodyAnglesIni(40,1,0.0);
  //  BodyAnglesIni.Resize(40,1);

  for(int i=0;i<28;i++)
    BodyAnglesIni(12+i,0)= UpperBodyAnglesBuffer(0,i);

  int Lindex=0;
  // Read NL informations from ZMPRefPositions.
  aZMPpcwmbz->Setup(ZMPPositions,
		    aCOMBuffer,
		    LeftFootPositions,
		    RightFootPositions,
		    BodyAnglesIni);
  Lindex+=NL;
  
  VNL::Matrix<double> prev_qr(6,1,0.0), prev_ql(6,1,0.0);
  VNL::Matrix<double> Finalq(41,1,0.0);
  vector3d prev_P,prev_L;
  prev_P.x = prev_P.y = prev_P.z =
    prev_L.x = prev_L.y = prev_L.z = 0.0;
  int LINKSFORRLEG[6] = { 0, 1, 2, 3,  4, 5};
  int LINKSFORLLEG[6] = { 6, 7, 8, 9, 10, 11};
  VNL::Matrix<double> dql(6,1,0.0), dqr(6,1,0.0);

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
  
  VNL::Matrix<double> S(4,6,0.0);
  VNL::Matrix<double> XiLeftFootRef(6,1,0.0),
    XiRightFootRef(6,1),
    XiBdThetaFreeRef,
    XiBdThetaFree,
    PLref(6,1),
    dqlf(6,1),dqrf(6,1);
  double lMasse = a2DMB->getMasse();
  double lZc = aPC->GetHeightOfCoM();
  int NbOfFreeJoints;
  vector3d FinalWaistPosition;
  matrix3d FinalWaistOrientation;
  matrix3d Ro;
  FinalWaistOrientation.setIdentity();
  // 

  struct timeval time_start,time_end;
  gettimeofday(&time_start,0);
  for(unsigned int i=NL;i<ZMPPositions.size()-NL;i++)
    {
      
      VNL::Matrix<double> lqr(6,1,0.0),lql(6,1,0.0);
      VNL::Matrix<double> lqArmr(7,1,0.0),lqArml(7,1,0.0);
      VNL::Matrix<double> UpperBodyAngles(28,1,0.0);
      
      COMPosition aCOMPosition;
      
      for(unsigned int j=0;j<28;j++) 
	UpperBodyAngles(j,0) = UpperBodyAnglesBuffer(i,j);

      
      aCOMPosition =  aCOMBuffer[i]; //aCOMMPosition is first given for the first stage of preview
      //but the returned aCOMPosition is the one of NL steps ago
      if (i==799)
	cout << "i =799"<< " " << aCOMPosition.z[0]<<endl;

      aZMPpcwmbz->OneGlobalStepOfControl(LeftFootPositions[i],
					 RightFootPositions[i],
					 ZMPPositions[i+NL],
					 lqr,lql,
					 aCOMPosition,
					 UpperBodyAngles);
	  
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
      
      //cout << WaistVelocity.x << " "<< WaistVelocity.y << endl;
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
      vector3d a2COMPosition;      
      //      a2DMB->CalculateZMP(ZMPmultibody[0],ZMPmultibody[1],
      //			  dP, dL, ZMPz);
      a2COMPosition = a2DMB->getPositionCoM();

      a2DMB->InertiaMatricesforRMCFirstStep();
      a2DMB->InertiaMatricesforRMCSecondStep();


      a2DMB->BuildSplittedInertialMatrices(LeftLeg,RightLeg,
					   1, FreeJoints);

      double Kv=lMasse, Kp=0.0; // Coefficients for the PD controller of the reference value ...

      //      cout << aCOMPosition.x[0] << " " << a2COMPosition.x << " " << aCOMPosition.x[0] - a2COMPosition.x << endl;
      PLref(0,0) = lMasse * Kp * (aCOMPosition.x[0] - a2COMPosition.x)
	+ Kv * aCOMPosition.x[1];
      PLref(1,0) = lMasse * Kp * (aCOMPosition.y[0] - a2COMPosition.y)
	+ Kv * aCOMPosition.y[1];      
      PLref(2,0) = lMasse * Kp * (lZc - a2COMPosition.z);
      PLref(3,0) = PLref(4,0)= PLref(5,0)= 0.0;

      // From bodyinfo.h
      int LLEG_JOINT5=11;
      int RLEG_JOINT5=5;
      vector3d lv,lw;
      lv = a2DMB->Getv(LLEG_JOINT5);
      lw = a2DMB->Getw(LLEG_JOINT5);
      XiLeftFootRef(0,0) = lv.x;
      XiLeftFootRef(1,0) = lv.y;
      XiLeftFootRef(2,0) = lv.z;
      XiLeftFootRef(3,0) = lw.x;
      XiLeftFootRef(4,0) = lw.y;
      XiLeftFootRef(5,0) = lw.z;
      
      lv = a2DMB->Getv(RLEG_JOINT5);
      lw = a2DMB->Getw(RLEG_JOINT5);
      XiRightFootRef(0,0) = lv.x;
      XiRightFootRef(1,0) = lv.y;
      XiRightFootRef(2,0) = lv.z;
      XiRightFootRef(3,0) = lw.x;
      XiRightFootRef(4,0) = lw.y;
      XiRightFootRef(5,0) = lw.z;

      NbOfFreeJoints = FreeJoints.size();
      
      S(0,0) = 1.0;
      S(1,1) = 1.0;
      S(2,2) = 1.0;
      S(3,5) = 1.0;

      XiBdThetaFreeRef.Resize(6 + NbOfFreeJoints,1);
#if 0
      XiBdThetaFreeRef.Zero();
      if (i==NL)
	{
	  XiBdThetaFreeRef(0,0) = WaistVelocity.x;
	  XiBdThetaFreeRef(1,0) = WaistVelocity.y;
	  XiBdThetaFreeRef(2,0) = WaistVelocity.z;
	}
      else 
	{
	  XiBdThetaFreeRef(0,0) = XiBdThetaFree(0,0);
	  XiBdThetaFreeRef(1,0) = XiBdThetaFree(1,0);
	  XiBdThetaFreeRef(2,0) = XiBdThetaFree(2,0);
	  XiBdThetaFreeRef(3,0) = XiBdThetaFree(3,0);
	  XiBdThetaFreeRef(4,0) = XiBdThetaFree(4,0);
	  XiBdThetaFreeRef(5,0) = XiBdThetaFree(5,0);
	}
#endif
#if 1
      //      XiBdThetaFreeRef.Zero();
      vector3d Wvin2DMB, Wwin2DMB;
      Wvin2DMB = a2DMB->GetvBody(1);
      Wwin2DMB = a2DMB->GetwBody(1);
      XiBdThetaFreeRef(0,0) = Wvin2DMB.x;
      XiBdThetaFreeRef(1,0) = Wvin2DMB.y;
      XiBdThetaFreeRef(2,0) = Wvin2DMB.z;
      XiBdThetaFreeRef(3,0) = Wwin2DMB.x;
      XiBdThetaFreeRef(4,0) = Wwin2DMB.y;
      XiBdThetaFreeRef(5,0) = Wwin2DMB.z;

#endif

      
      a2DMB->BuildLinearSystemForRMC(PLref,
				     XiLeftFootRef,
				     XiRightFootRef,
				     NbOfFreeJoints,
				     S,
				     XiBdThetaFreeRef,
				     XiBdThetaFree,dqlf,dqrf);

      // Integration of the speed returned by Build Linear System For RMC.

      // initialization
      if (i==NL)
	{
	  FinalWaistPosition.x = 0;
	  FinalWaistPosition.y = 0;
	  FinalWaistPosition.z = aCOMPosition.omega;
	}
	      
      // Integrates for the legs.
      for(unsigned int j=0;j<6;j++)
	{
	  if (i==NL)
	    {
	      a3DMB->Setq(LINKSFORLLEG[j],lql(j,0));
	      a3DMB->Setq(LINKSFORRLEG[j],lqr(j,0));
	      Finalq(LINKSFORLLEG[j],0)= lql(j,0);
	      Finalq(LINKSFORRLEG[j],0)= lqr(j,0);
	      
	    }
	  else
	    {
	      Finalq(LINKSFORLLEG[j],0) += dqlf(j,0)*SamplingPeriod;
	      Finalq(LINKSFORRLEG[j],0) += dqrf(j,0)*SamplingPeriod;
	      a3DMB->Setq(LINKSFORLLEG[j],Finalq(LINKSFORLLEG[j],0));
	      a3DMB->Setq(LINKSFORRLEG[j],Finalq(LINKSFORRLEG[j],0));
	    }
	  
	  a3DMB->Setdq(LINKSFORLLEG[j],dqlf(j,0));
	  a3DMB->Setdq(LINKSFORRLEG[j],dqrf(j,0));
	}

      // Integrates for the free joints.
      for(int j=13;j<a3DMB->NbOfLinks();j++)
	{
	  a3DMB->Setdq(j,XiBdThetaFree(j-6,0));
	  a3DMB->Setq(j,Finalq(j,0));
	  Finalq(j,0) += XiBdThetaFree(j-6,0)*SamplingPeriod; /* 7 + j -13 = j-5 */
	}
      
      vector3d FWw;
      vector3d FWv;
      FWv.x= XiBdThetaFree(0,0);
      FWv.y= XiBdThetaFree(1,0);
      FWv.z= XiBdThetaFree(2,0);
      
      FWw.x= XiBdThetaFree(3,0);
      FWw.y= XiBdThetaFree(4,0);
      FWw.z= XiBdThetaFree(5,0);
      double th = FWw.norm()*0.005;
      
      FWw.normalize();
      matrix3d lRo, wedge;
      wedge[0*3+0] =   0.0;wedge[0*3+1]= -FWw.z; wedge[0*3+2]=  FWw.y;	// Cross product
      wedge[1*3+0] = FWw.z;wedge[1*3+1]=    0.0; wedge[1*3+2]= -FWw.x;
      wedge[2*3+0] =-FWw.y;wedge[2*3+1]=  FWw.x; wedge[2*3+2]=    0.0;

      Ro.setIdentity();
      Ro = Ro + wedge * sin(th) + (wedge * wedge) * (1-cos(th));
      
      Body_Rm3d = Ro;

      a3DMB->ForwardVelocity(FinalWaistPosition,Body_Rm3d,FWv);

      FinalWaistPosition += FWv*0.005;
      FinalWaistOrientation *= Ro;
      a3DMB->SetRBody(1,FinalWaistOrientation);

      vector3d FinalP, FinalL;
      a3DMB->GetPandL(FinalP,FinalL);
      
      cout << FinalP.x << " " << FinalP.y << " " << FinalP.z << " " 
	   << FinalL.x << " " << FinalL.y << " " << FinalL.z << endl;
      // Computes dTheta for the legs.


      //cout << "Iteration Number " << i << endl;
      //      cout << XiBdThetaFree <<endl;
      //      cout << "***********************************"<<endl;


      
      /*      cout << "COMPosition (" 
	   << FinalCOMPos.x << " "
	   << FinalCOMPos.y << " " 
	   << FinalCOMPos.z << endl;
      */
      if(1)
	{
	  if(aof_Finaldqleg.is_open())
	    {
	      for(unsigned int i=0;i<6;i++)
		aof_Finaldqleg << dqrf(i,0) <<" ";

	      for(unsigned int i=0;i<6;i++)
		aof_Finaldqleg << dqlf(i,0) <<" ";
	      aof_Finaldqleg << endl;
	      
	    }

	  if(aof_XiLeftFoot.is_open())
	    {
	      aof_XiLeftFoot <<  XiLeftFootRef(0,0) << " " <<
		                 XiLeftFootRef(1,0) << " " <<
		                 XiLeftFootRef(2,0) << " " <<
                                 XiLeftFootRef(3,0) << " " <<
                                 XiLeftFootRef(4,0) << " " <<
                                 XiLeftFootRef(5,0) << " " << endl;
	    }
	  if(aof_XiThetaFree.is_open())
	    {
	      for(unsigned int j=0;j<XiBdThetaFree.Rows();j++)
		{
		  aof_XiThetaFree << XiBdThetaFree(j,0) << " " ;
		}
	      aof_XiThetaFree << endl;
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
			 << aCOMPosition.x[1] << " " << aCOMPosition.y[1] << endl;
	    }

	  if (aof_dq.is_open())
	    {
	      for(int j=0;j<a2DMB->NbOfLinks();j++)
		aof_dq << a2DMB->Getdq(j) << " ";
	      aof_dq << endl;
	    }
	}
    }
  gettimeofday(&time_end,0);
  double GlobalTime = time_end.tv_sec - time_start.tv_sec +
    0.000001 * (time_end.tv_sec - time_start.tv_sec );
  double Average = GlobalTime / (ZMPPositions.size()-2*NL);
  cout << "Total Time consumption: "<< GlobalTime << " Average consumption time: " << Average << endl;

 if(aof_XiLeftFoot.is_open())
    aof_XiLeftFoot.close();

 if(aof_XiThetaFree.is_open())
    aof_XiThetaFree.close();

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

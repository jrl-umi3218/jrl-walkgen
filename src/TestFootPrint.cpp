#include <ZMPDiscretization.h>
#include <PreviewControl.h>
#include <InverseKinematics.h>
#include <DynamicMultiBody.h>
#include <fstream>

using namespace::PatternGeneratorJRL;

int main(void)
{

  double sx=0.2,sy=0.19;
  RelativeFootPosition TabFoots[7] = {
    { 0.0, -sy/2, 0.0},
    {  sx,  sy, 0.0},
    {  sx, -sy, 0.0},
    {  sx,  sy, 0.0},
    {  sx, -sy, 0.0},
    {  sx,  sy, 0.0},
    { 0.0, -sy, 0.0}
    
  };

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
  

  vector<ZMPPosition> ZMPPositions;
  vector<FootAbsolutePosition> FootAbsolutePositions;
  vector<RelativeFootPosition> RelativeFootPositions;
  vector<FootAbsolutePosition> LeftFootPositions,RightFootPositions;
  InverseKinematics *anIK;

  DynamicMultiBody *aDMB=0;
  aDMB = new DynamicMultiBody();
  string HRP2JRLpath = "../../etc/HRP2JRL/";
  string HRP2JRLfilename = "HRP2JRLmain.wrl";
  string HRP2fullpath= "./";
  string HRP2fullfilename = "HRP2main_full.wrl";
  
  aDMB->parserVRML(HRP2JRLpath,HRP2JRLfilename,"");
  //aDMB->parserVRML(HRP2fullpath,HRP2fullfilename,"");
  aDMB->afficherLiaisons();
  ZMPDiscretization *MonBeauFootPrint;
  MonBeauFootPrint = new ZMPDiscretization();



  PreviewControl *aPC = new PreviewControl;

  string PCParameters="PreviewControlParameters.ini";
  aPC->ReadPrecomputedFile(PCParameters);
  double SamplingPeriod = aPC->SamplingPeriod();
  double PreviewControlTime = aPC->PreviewControlTime();
  unsigned int NL = (unsigned int)(PreviewControlTime/SamplingPeriod);

  dMatrix x(3,1),y(3,1);
  x(1,1)= 0;   x(2,1)= 0;   x(3,1)= 0;
  y(1,1)= 0;   y(2,1)= 0;   y(3,1)= 0;

  double sxzmp=0.0,syzmp=0.0;
  double zmpx2=0.0,zmpy2=0.0;

  vector<ZMPPosition> ZMP2Positions;
  vector<COMPosition> COMPositions;
  dMatrix Body_R(3,3), Body_P(3,1);
  dMatrix Foot_R(3,3), Foot_P(3,1);
  float c,s;
  int LINKSFORRLEG[6] = { 0, 1, 2, 3,  4, 5};
  int LINKSFORLLEG[6] = { 6, 7, 8, 9, 10, 11};
  dMatrix Dt(3,1),ql(6,1),qr(6,1),prev_ql(6,1),prev_qr(6,1);
  dMatrix dql(6,1), dqr(6,1);

  // Momentum vectors.
  vector3d P,L, prev_P, prev_L, dP, dL;

  // For computing ZMP multi body
  double ZMPmultibody[2];
  double ZMPz=0.0;
  
  vector3d COMFromMB;

  ofstream arof, alof;
  alof.open("LeftLegJointAngles.dat",ofstream::out);
  arof.open("RightLegJointAngles.dat",ofstream::out);

  anIK = new InverseKinematics();

  // First sequence of steps.

  for(int i=0;i<7;i++)
    RelativeFootPositions.push_back(TabFoots[i]);


  
  MonBeauFootPrint->GetZMPDiscretization(ZMPPositions,
					 FootAbsolutePositions,
					 RelativeFootPositions,
					 LeftFootPositions,
					 RightFootPositions);
  
  string ZMPPos1="ZMPPositions_1.dat";
  string FootPos1="FootPositions_1.dat";
  MonBeauFootPrint->DumpDataFiles(ZMPPos1, FootPos1,
				 ZMPPositions, FootAbsolutePositions);
  string LAbsPos = "LeftFootAbsPos_1.dat";
  string RAbsPos = "RightFootAbsPos_1.dat";
  MonBeauFootPrint->DumpFootAbsolutePosition(LAbsPos, LeftFootPositions);
  MonBeauFootPrint->DumpFootAbsolutePosition(RAbsPos, RightFootPositions);

  ofstream aofpos;
  aofpos.open("Pos_1.dat",ofstream::out);

  ofstream aofspeed;
  aofspeed.open("Speed_1.dat",ofstream::out);

  ofstream aof_errorspeed;
  aof_errorspeed.open("ErrorSpeed_1.dat",ofstream::out);

  ofstream aof_FootError;
  aof_FootError.open("FootError_1.dat", ofstream::out);

  ofstream aof_ZMPMB;
  aof_ZMPMB.open("ZMPMB.dat", ofstream::out);

  double DiffBetweenComAndWaist = -0.145184;
  vector3d ErrorAccumulation;
  ErrorAccumulation.x = 0 ;
  ErrorAccumulation.y = 0 ;
  ErrorAccumulation.z = 0 ;
  for(unsigned int i=1;i<=6;i++)
    prev_ql(i,1) = prev_qr(i,1) = dql(i,1) = dqr(i,1) = 0.0;

  for(int i=0;i<aDMB->NbOfLinks();i++)
    aDMB->Setdq(i,0.0);

  for(unsigned int i=0;i<ZMPPositions.size()-NL;i++)
    {
      // Displacement between the hip and RLINK2
      Dt(1,1) = 0;
      Dt(2,1) = 0.035;
      Dt(3,1) = 0.0;
  
      // From the waist to the hip. 
      dMatrix StaticToTheHip(3,1),ToTheHip(3,1);
      StaticToTheHip(1,1) = 0.0;
      StaticToTheHip(2,1) = 0.06;
      StaticToTheHip(3,1) = DiffBetweenComAndWaist;

      aPC->OneIterationOfPreview(x,y,sxzmp,syzmp,
			    ZMPPositions,i,
			    zmpx2, zmpy2, true);
      ZMPPosition azmpp;
      azmpp.px  = zmpx2;
      azmpp.py  = zmpy2;
      azmpp.time = ZMPPositions[i].time;
      ZMP2Positions.push_back(azmpp);
      
      COMPosition acomp;
      for(unsigned j=1;j<=3;j++)
	acomp.x[j-1] = x(j,1);

      for(unsigned j=1;j<=3;j++)
	acomp.y[j-1] = y(j,1);
      COMPositions.push_back(acomp);

      c = cos(ZMPPositions[i].theta*M_PI/180.0);
      s = sin(ZMPPositions[i].theta*M_PI/180.0);
      
      // COM Orientation
      Body_R(1,1) = c;       Body_R(1,2) = -s;       Body_R(1,3) = 0;
      Body_R(2,1) = s;       Body_R(2,2) =  c;       Body_R(2,3) = 0;
      Body_R(3,1) = 0;       Body_R(3,2) = 0;        Body_R(3,3) = 1;

      // COM position
      ToTheHip = Body_R * StaticToTheHip;
      Body_P(1,1)=acomp.x[0] + ToTheHip(1,1) ;
      Body_P(2,1)=acomp.y[0] + ToTheHip(2,1);
      Body_P(3,1)=aPC->GetHeightOfCoM() + ToTheHip(3,1);


      // Left Foot.
      c = cos(LeftFootPositions[i].theta*M_PI/180.0);
      s = sin(LeftFootPositions[i].theta*M_PI/180.0);

      //      cout << ZMPPositions[i].theta << " " << LeftFootPositions[i].theta;
      // Orientation
      Foot_R(1,1) = c;       Foot_R(1,2) = -s;       Foot_R(1,3) = 0;
      Foot_R(2,1) = s;       Foot_R(2,2) =  c;       Foot_R(2,3) = 0;
      Foot_R(3,1) = 0;       Foot_R(3,2) = 0;        Foot_R(3,3) = 1;

      // position
      Foot_P(1,1)=LeftFootPositions[i].x;
      Foot_P(2,1)=LeftFootPositions[i].y;
      Foot_P(3,1)=LeftFootPositions[i].z+0.105;

      anIK->ComputeInverseKinematics2(Body_R,
				     Body_P,
				     Dt,
				     Foot_R,
				     Foot_P,
				     ql);
      for(unsigned int j=0;j<6;j++)
	{
	  aDMB->Setq(LINKSFORLLEG[j],ql(j+1,1));
	  if (i!=0)
	    dql(j+1,1) = (ql(j+1,1)-prev_ql(j+1,1))/SamplingPeriod;
	  aDMB->Setdq(LINKSFORLLEG[j],dql(j+1,1));
	  prev_ql(j+1,1) = ql(j+1,1);
	}
      
      if (alof.is_open())
	{
	  for(unsigned int j=1;j<=6;j++)
	    {
	      alof << ql(j,1) << " ";
	    }
	  alof<<endl;
	}


      Dt(2,1) = -Dt(2,1);
      StaticToTheHip(2,1) = - StaticToTheHip(2,1);
      ToTheHip = Body_R * StaticToTheHip;

      // Right Foot.
      c = cos(RightFootPositions[i].theta*M_PI/180.0);
      s = sin(RightFootPositions[i].theta*M_PI/180.0);
      // Orientation
      Foot_R(1,1) = c;       Foot_R(1,2) = -s;       Foot_R(1,3) = 0;
      Foot_R(2,1) = s;       Foot_R(2,2) =  c;       Foot_R(2,3) = 0;
      Foot_R(3,1) = 0;       Foot_R(3,2) = 0;        Foot_R(3,3) = 1;

      // position
      Foot_P(1,1)=RightFootPositions[i].x;
      Foot_P(2,1)=RightFootPositions[i].y;
      Foot_P(3,1)=RightFootPositions[i].z+0.105;

      // COM position
      Body_P(1,1)=acomp.x[0] + ToTheHip(1,1);
      Body_P(2,1)=acomp.y[0] + ToTheHip(2,1);
      Body_P(3,1)=aPC->GetHeightOfCoM() + ToTheHip(3,1);

      anIK->ComputeInverseKinematics2(Body_R,
				      Body_P,
				      Dt,
				      Foot_R,
				      Foot_P,
				      qr);

      for(unsigned int j=0;j<6;j++)
	{
	  aDMB->Setq(LINKSFORRLEG[j],qr(j+1,1));
	  if (i!=0)
	    dqr(j+1,1) = (qr(j+1,1)-prev_qr(j+1,1))/SamplingPeriod;
	  aDMB->Setdq(LINKSFORRLEG[j],dqr(j+1,1));
	  prev_qr(j+1,1) = qr(j+1,1);
	}
      
      if (arof.is_open())
	{
	  for(unsigned int j=1;j<=6;j++)
	    arof << qr(j,1) << " ";
	  arof<<endl;
	}
	    
      Dt(2,1) = -Dt(2,1);


      //      cout << endl;

      // Update the Dynamic Multi Body Model.

      // Update the velocity.
      int lCOMi = COMPositions.size();
      vector3d WaistPosition;
      vector3d WaistVelocity;
      WaistVelocity.x = COMPositions[lCOMi-1].x[1];
      WaistVelocity.y = COMPositions[lCOMi-1].y[1];
      WaistVelocity.z = 0;
      
      WaistPosition.x = COMPositions[lCOMi-1].x[0];
      WaistPosition.y = COMPositions[lCOMi-1].y[0];
      WaistPosition.z = aPC->GetHeightOfCoM() + ToTheHip(3,1) - 0.705; // Compensate for the static translation.
      
      aDMB->ForwardVelocity(WaistPosition,WaistVelocity);

      aDMB->GetPandL(P,L);
      if (i!=0)
	{
	  dP = (P - prev_P)/SamplingPeriod;
	  dL = (L - prev_L)/SamplingPeriod;
	  aDMB->CalculateZMP(ZMPmultibody[0],ZMPmultibody[1],
			     dP, dL, ZMPz);

	  COMFromMB = aDMB->getPositionCoM();
	  if (aof_ZMPMB.is_open())
	    {
	      aof_ZMPMB << ZMPmultibody[0] << " " << ZMPmultibody[1] << " " 
			<< COMFromMB.x << " " << COMFromMB.y << " " << COMFromMB.z << endl;
	    }
	}
      prev_P = P;
      prev_L = L;

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
	  
	  for(int j=0;j<aDMB->NbOfLinks();j++)
	    {
	      vector3d avec = aDMB->Getv(j);
	      aofspeed << avec.x << " " <<  avec.y << " "  << avec.z << " " ;
	      avec = aDMB->Getw(j);
	      aofspeed << avec.x << " " <<  avec.y << " "  << avec.z << " " ;
	    }

	  aofspeed << endl;

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
	      
      
    }

  if (aof_errorspeed.is_open())
    aof_errorspeed.close();

  if (aofspeed.is_open())
    aofspeed.close();
  
  if (aofpos.is_open())
    aofpos.close();

  if (aof_FootError.is_open())
    aof_FootError.close();

  if (aof_ZMPMB.is_open())
    aof_ZMPMB.close();

  ofstream aof;
  aof.open("ZMP2Positions_1.dat",ofstream::out);
  if (aof.is_open())
    {
      for(unsigned int i=0;i<ZMP2Positions.size();i++)
	{
	  aof << ZMP2Positions[i].time << " " << ZMP2Positions[i].px << " " << ZMP2Positions[i].py << " 0.0" << endl;
	}
      aof.close();
    }

  aof.open("COMPositions_1.dat",ofstream::out);
  if (aof.is_open())
    {
      for(unsigned int i=0;i<COMPositions.size();i++)
	{
	  for(unsigned int j=0;j<3;j++)
	    aof << COMPositions[i].x[j] << " ";

	  for(unsigned int j=0;j<3;j++)
	    aof << COMPositions[i].y[j] << " " ;

	  aof << aPC->GetHeightOfCoM()<< " ";
	  aof<<endl;
	}
      aof.close();
    }


  exit(0);
  // Second sequence of steps.
  sxzmp=0.0,syzmp=0.0;
  zmpx2=0.0,zmpy2=0.0;
  x(1,1)= 0;   x(2,1)= 0;   x(3,1)= 0;
  y(1,1)= 0;   y(2,1)= 0;   y(3,1)= 0;

  ZMP2Positions.clear();
  COMPositions.clear();
  RelativeFootPositions.clear();
  for(int i=0;i<12;i++)
    RelativeFootPositions.push_back(TabFoots2[i]);

  FootAbsolutePositions.clear();
  ZMPPositions.clear();
  LeftFootPositions.clear();
  RightFootPositions.clear();
  cout << "ZMPPositions size: " << ZMPPositions.size() << endl;
  MonBeauFootPrint->GetZMPDiscretization(ZMPPositions,
					 FootAbsolutePositions,
					 RelativeFootPositions,
					 LeftFootPositions,
					 RightFootPositions);

  string ZMPPos2="ZMPPositions_2.dat";
  string FootPos2="FootPositions_2.dat";
  MonBeauFootPrint->DumpDataFiles(ZMPPos2, FootPos2,
				 ZMPPositions, FootAbsolutePositions);

  LAbsPos = "LeftFootAbsPos_2.dat";
  RAbsPos = "RightFootAbsPos_2.dat";
  MonBeauFootPrint->DumpFootAbsolutePosition(LAbsPos, LeftFootPositions);
  MonBeauFootPrint->DumpFootAbsolutePosition(RAbsPos, RightFootPositions);


  for(unsigned int i=0;i<ZMPPositions.size()-NL;i++)
    {
      aPC->OneIterationOfPreview(x,y,sxzmp,syzmp,
				 ZMPPositions,i,
				 zmpx2, zmpy2, true);
      ZMPPosition azmpp;
      azmpp.px  = zmpx2;
      azmpp.py  = zmpy2;
      azmpp.time = ZMPPositions[i].time;
      ZMP2Positions.push_back(azmpp);
      
      COMPosition acomp;
      for(unsigned j=1;j<=3;j++)
	acomp.x[j-1] = x(j,1);

      for(unsigned j=1;j<=3;j++)
	acomp.y[j-1] = y(j,1);
      COMPositions.push_back(acomp);

      c = cos(ZMPPositions[i].theta*M_PI/180.0);
      s = sin(ZMPPositions[i].theta*M_PI/180.0);
      
      // COM Orientation
      Body_R(1,1) = c;       Body_R(1,2) = -s;       Body_R(1,3) = 0;
      Body_R(2,1) = s;       Body_R(2,2) =  c;       Body_R(2,3) = 0;
      Body_R(3,1) = 0;       Body_R(3,2) = 0;        Body_R(3,3) = 1;

      // COM position
      Body_P(1,1)=acomp.x[0];
      Body_P(2,1)=acomp.y[0];
      Body_P(3,1)=aPC->GetHeightOfCoM();
      
      // Left Foot.
      c = cos(LeftFootPositions[i].theta*M_PI/180.0);
      s = sin(LeftFootPositions[i].theta*M_PI/180.0);

      //      cout << ZMPPositions[i].theta << " " << LeftFootPositions[i].theta;
      // Orientation
      Foot_R(1,1) = c;       Foot_R(1,2) = -s;       Foot_R(1,3) = 0;
      Foot_R(2,1) = s;       Foot_R(2,2) =  c;       Foot_R(2,3) = 0;
      Foot_R(3,1) = 0;       Foot_R(3,2) = 0;        Foot_R(3,3) = 1;

      // position
      Foot_P(1,1)=LeftFootPositions[i].x;
      Foot_P(2,1)=LeftFootPositions[i].y;
      Foot_P(3,1)=LeftFootPositions[i].z+0.105;

      anIK->ComputeInverseKinematics(Body_R,
				     Body_P,
				     Dt,
				     Foot_R,
				     Foot_P,
				     ql);
      
      if (alof.is_open())
	{
	  for(unsigned int j=1;j<=6;j++)
	    {
	      alof << ql(j,1) << " ";
	    }
	  alof<<endl;
	}


      Dt(2,1) = -Dt(2,1);

      // Right Foot.
      c = cos(RightFootPositions[i].theta*M_PI/180.0);
      s = sin(RightFootPositions[i].theta*M_PI/180.0);
      // Orientation
      Foot_R(1,1) = c;       Foot_R(1,2) = -s;       Foot_R(1,3) = 0;
      Foot_R(2,1) = s;       Foot_R(2,2) =  c;       Foot_R(2,3) = 0;
      Foot_R(3,1) = 0;       Foot_R(3,2) = 0;        Foot_R(3,3) = 1;

      // position
      Foot_P(1,1)=RightFootPositions[i].x;
      Foot_P(2,1)=RightFootPositions[i].y;
      Foot_P(3,1)=RightFootPositions[i].z+0.105;

      anIK->ComputeInverseKinematics(Body_R,
				     Body_P,
				     Dt,
				     Foot_R,
				     Foot_P,
				     qr);
      
      if (arof.is_open())
	{
	  for(unsigned int j=1;j<=6;j++)
	    arof << qr(j,1) << " ";
	  arof<<endl;
	}
	    
      Dt(2,1) = -Dt(2,1);
      //      cout << endl;
    }

  if (alof.is_open())
    alof.close();

  if (arof.is_open())
    arof.close();

  aof.open("ZMP2Positions_2.dat",ofstream::out);
  if (aof.is_open())
    {
      for(unsigned int i=0;i<ZMP2Positions.size();i++)
	{
	  aof << ZMP2Positions[i].time << " " << ZMP2Positions[i].px << " " << ZMP2Positions[i].py << " 0.0" << endl;
	}
      aof.close();
    }

  aof.open("COMPositions_2.dat",ofstream::out);
  if (aof.is_open())
    {
      for(unsigned int i=0;i<COMPositions.size();i++)
	{
	  for(unsigned int j=0;j<3;j++)
	    aof << COMPositions[i].x[j] << " ";

	  for(unsigned int j=0;j<3;j++)
	    aof << COMPositions[i].y[j] << " " ;

	  aof << aPC->GetHeightOfCoM()<< " ";
	  aof<<endl;
	}
      aof.close();
    }
  
  // Test of Preview control.
  if (anIK!=0)
    delete anIK;

  if (MonBeauFootPrint!=0)
    delete MonBeauFootPrint;

  if (aPC!=0)
    delete aPC;
    
}

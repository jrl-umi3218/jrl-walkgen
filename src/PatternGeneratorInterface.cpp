#include <fstream>
#include <PatternGeneratorInterface.h>
#include <sys/time.h>
#include <time.h>

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "PatternGeneratorInterface :" << x << endl
#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#if 0
#define ODEBUG(x) cerr << "PatternGeneratorInterface :" <<  x << endl
#else
#define ODEBUG(x) 
#endif

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1 
#else
#define RESETDEBUG4(y) 
#define ODEBUG4(x,y)
#endif

#define ODEBUG6(x,y)

namespace PatternGeneratorJRL {
  PatternGeneratorInterface::PatternGeneratorInterface(istringstream &strm)    
  {

    string PCParameters;
    strm >> PCParameters;

    string HRP2JRLpath;// = "/home/stasse/OpenHRP/etc/HRP2JRL/";
    strm >> HRP2JRLpath;

    string HRP2JRLfilename;// = "HRP2JRLmain.wrl";
    strm >> HRP2JRLfilename;

    string HumanoidSpecificitiesFileName;
    strm >> HumanoidSpecificitiesFileName;
    // Load the specific data of the humanoid.
    cout << "Load the specific data of the humanoid " << endl;
    m_HS = new HumanoidSpecificities();

    string aHumanoidName;
    m_HS->ReadXML(HumanoidSpecificitiesFileName,aHumanoidName);
    m_HS->Display();
    cout << "Here at the end"  << endl;

    m_BoolPBWAlgo = 0;
    // Initialize (if needed) debugging actions.
    ODEBUG4("Step 0","DebugPGI.txt");
    m_StepStackHandler = 0;
    m_dt = 0.005;
    m_DOF = 40;
    RESETDEBUG4("DebugDataWPDisplay.txt");

    RESETDEBUG4("DebugDataqrDisplay.txt");
    RESETDEBUG4("DebugDataqlDisplay.txt");
    RESETDEBUG4("DebugDataZMPMB1Display.txt");

    RESETDEBUG4("DebugDatadqrDisplay.txt");
    RESETDEBUG4("DebugDatadqlDisplay.txt");
    RESETDEBUG4("DebugDataUBDisplay.txt");
    RESETDEBUG4("DebugDatadUBDisplay.txt");

    RESETDEBUG4("DebugData.txt");
    RESETDEBUG4("DebugPGI.txt");
    RESETDEBUG5("DebugDataLong.txt");
    RESETDEBUG4("DebugDataZMPTargetZ.dat");
    RESETDEBUG4("LeftLegAngle.txt");
    RESETDEBUG4("RightLegAngle.txt");
    ODEBUG4("Step 1","DebugPGI.txt");
    RESETDEBUG4("DebugDataIKArms.txt");
    RESETDEBUG4("DebugDataOnLine.txt");
    RESETDEBUG4("DebugDataWaistYaw.dat");
    RESETDEBUG5("DebugGMFKW.dat");

    m_ObstacleDetected = false;

    // Initialization of obstacle parameters informations.	
    m_ObstaclePars.x=1.0;
    m_ObstaclePars.y=0.0;
    m_ObstaclePars.z=0.0;
    m_ObstaclePars.theta=0.0;
    m_ObstaclePars.h=0.05;
    m_ObstaclePars.w=1.0;
    m_ObstaclePars.d=0.05;
		
    m_prev_qr.Resize(6,1);
    m_prev_dqr.Resize(6,1);
    m_prev_ql.Resize(6,1);
    m_prev_dql.Resize(6,1);

    /* For debug purposes. */
    m_Debug_prev_qr.Resize(6,1);
    m_Debug_prev_dqr.Resize(6,1);
    m_Debug_prev_ql.Resize(6,1);
    m_Debug_prev_dql.Resize(6,1);

    m_Debug_prev_qr_RefState.Resize(6,1);
    m_Debug_prev_ql_RefState.Resize(6,1);

    m_Debug_prev_UpperBodyAngles.Resize(28,1);

    
    

    // Initialization of the  computed
    // leg's articular speed.
    m_dqr.Resize(6,1);
    m_dql.Resize(6,1);

    for(int i=0;i<6;i++)
      {
	m_dqr(i,0) = 0.0;
	m_dql(i,0) = 0.0;
      }


    m_ZMPShift.resize(4);
    m_ZMPShift[0] = 0.02;
    m_ZMPShift[1] = 0.07;
    m_ZMPShift[2] = 0.02;
    m_ZMPShift[3] = 0.02;

    m_TimeDistrFactor.resize(4);
    m_TimeDistrFactor[0]=2.0;
    m_TimeDistrFactor[1]=3.7;
    m_TimeDistrFactor[2]=1.0;
    m_TimeDistrFactor[3]=3.0;

    m_UpperBodyMotion.resize(3);
    m_UpperBodyMotion[0]=0.0;
    m_UpperBodyMotion[1]=0.0;
    m_UpperBodyMotion[2]=0.0;

    m_MotionAbsPos.Resize(4,4);
    m_MotionAbsOrientation.Resize(4,4);
    m_WaistAbsPos.Resize(4,4);
    m_WaistRelativePos.Resize(4,4);
  
    for(int i=0;i<4;i++)
      {
	for(int j=0;j<4;j++)
	  if (i==j)
	    m_MotionAbsPos(i,j) =
	      m_MotionAbsOrientation(i,j) =
	      m_WaistAbsPos(i,j) =
	      m_WaistRelativePos(i,j) = 
	      1.0;
	  else
	    m_MotionAbsPos(i,j) =
	      m_MotionAbsOrientation(i,j) =
	      m_WaistAbsPos(i,j) =
	      m_WaistRelativePos(i,j) = 
	      0.0;
      
      }

    m_AbsLinearVelocity.Resize(4,1);
    m_AbsAngularVelocity.Resize(4,1); 

    m_CurrentJointValues.resize(m_DOF);
    m_DeltaFeasibilityLimit =0.0;

    ODEBUG4("Step 2","DebugPGI.txt");	
    m_ZMPD = new ZMPDiscretization("",m_HS);
    m_IK = new InverseKinematics(m_HS);
	
    m_ZARM = -1.0;
    m_GainFactor = 1.7;	
	
    //  string PCParameters="/home/stasse/OpenHRP/PatternGeneratorJRL/src/PreviewControlParameters.ini";
    m_PC = new PreviewControl();
    ofstream DebugFile;
    ofstream DebugFileLong;
    ofstream DebugFileUpperBody;
#if _DEBUG_4_ACTIVATED_ 
    DebugFileUpperBody.open("UpperBodyAngles.txt",ofstream::out);
    DebugFileUpperBody.close();
    DebugFileLong.open("DebugZMPFinale.txt",ofstream::out);	
    DebugFileLong.close();
#endif
    m_PC->ReadPrecomputedFile(PCParameters);
    m_Zc = m_PC->GetHeightOfCoM();
    m_count = 0;
	
    // Object to generate Motion from KineoWorks.
    m_GMFKW = new GenerateMotionFromKineoWorks();
    m_GMFKW->SetPreviewControl(m_PC);
  
    m_DMB = new DynamicMultiBody();
	
    m_2DMB = new DynamicMultiBody();
	
    ODEBUG4("Step 3","DebugPGI.txt");	
    m_DMB->parserVRML(HRP2JRLpath,HRP2JRLfilename,"");
    m_2DMB->parserVRML(HRP2JRLpath,HRP2JRLfilename,"");
    //m_DiffBetweenComAndWaist =-0.145184;//-0.1703 ;0.0136923 0.00380861 0.157698
    m_DiffBetweenComAndWaist[0] = -0.0136923;
    m_DiffBetweenComAndWaist[1] = -0.00380861;
    m_DiffBetweenComAndWaist[2] = -0.15769;
    m_ZMPpcwmbz = new ZMPPreviewControlWithMultiBodyZMP(m_HS);
    m_ZMPpcwmbz->SetPreviewControl(m_PC);
    m_ZMPpcwmbz->SetDynamicMultiBodyModel(m_DMB);
    m_ZMPpcwmbz->SetInverseKinematics(m_IK);
	
    m_StOvPl = new StepOverPlanner(m_ObstaclePars,m_HS);
    m_StOvPl->SetPreviewControl(m_PC);
    m_StOvPl->SetDynamicMultiBodyModel(m_DMB);
    m_StOvPl->SetInverseKinematics(m_IK);
    m_StOvPl->SetZMPDiscretization(m_ZMPD);

    ODEBUG4("Step 4","DebugPGI.txt");

    m_WaistPlanner = new WaistHeightVariation();
    m_UpBody = new UpperBodyMotion();
    m_StepStackHandler = new StepStackHandler();
    m_StepStackHandler->SetStepOverPlanner(m_StOvPl);

    m_SamplingPeriod = m_PC->SamplingPeriod();
    m_PreviewControlTime = m_PC->PreviewControlTime();
    m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);
    m_DebugMode = 0;
	
    m_StepStackHandler->SetWalkMode(0);
    
    m_TSsupport = 0.78;
    m_TDsupport = 0.02;
	
    m_FirstPrint = true;
    m_FirstRead = true;
    ODEBUG4("Step 5","DebugPGI.txt");

#if _DEBUG_4_ACTIVATED_  
    DebugFile.open("DebugDataCOM.txt",ofstream::out);
    DebugFile.close();
#endif
    m_NewStepX = 0.0;
    m_NewStepY = 0.0;
    m_NewTheta = 0.0;
    m_NewStep = false;
    m_ShouldBeRunning = false;

    m_ConstraintOnX = 0.04;
    m_ConstraintOnY = 0.04;

    m_QP_T = 0.02;
    m_QP_N = 75;


  }

  PatternGeneratorInterface::~PatternGeneratorInterface()
  {
    ODEBUG4("Destructor: Start","DebugPGI.txt");
    if (m_UpBody!=0)
      delete m_UpBody;	
    ODEBUG4("Destructor: did m_UpperBodyMotion","DebugPGI.txt");

    if (m_WaistPlanner!=0)
      delete m_WaistPlanner;	
    ODEBUG4("Destructor: did m_WaistPlanner","DebugPGI.txt");

    if (m_StOvPl!=0)
      delete m_StOvPl;	
    ODEBUG4("Destructor: did m_StOvPl","DebugPGI.txt");

    if (m_ZMPpcwmbz!=0)
      delete m_ZMPpcwmbz;
    ODEBUG4("Destructor: did m_ZMPpcwmbz","DebugPGI.txt");


    if (m_2DMB!=0)
      delete m_2DMB;
    ODEBUG4("Destructor: did m_2DMB","DebugPGI.txt");	

    if (m_DMB!=0)
      delete m_DMB;
    ODEBUG4("Destructor: did m_DMB","DebugPGI.txt");	

    if (m_GMFKW!=0)
      delete m_GMFKW;
    ODEBUG4("Destructor: did m_GMKFW","DebugPGI.txt");	

    if (m_PC!=0)
      delete m_PC;
    ODEBUG4("Destructor: did m_PC","DebugPGI.txt");

    if (m_IK!=0)
      delete m_IK;
    ODEBUG4("Destructor: did m_IK","DebugPGI.txt");

    if (m_ZMPD!=0)
      delete m_ZMPD;
    ODEBUG4("Destructor: did m_ZMPD","DebugPGI.txt");

  
  }

  void PatternGeneratorInterface::m_SetObstacleParameters(istringstream &strm)
  {
 	
    m_ObstacleDetected=false;
    bool ReadObstacleParameters = false;
 
    ODEBUG( "I am reading the obstacle parameters" << " ");	
  
    while(!strm.eof())
      {
	if (!strm.eof())
	  {
	    strm >> m_ObstaclePars.x;
	    ODEBUG("obstacle position x:" << " "<< m_ObstaclePars.x );
	  }
	else 
	  break;
	if (!strm.eof())
	  {
	    strm >> m_ObstaclePars.y;
	    ODEBUG( "obstacle position y:" << " "<< m_ObstaclePars.y );
	  }
	else 
	  break;
	if (!strm.eof())
	  {
	    strm >> m_ObstaclePars.z;
	    ODEBUG( "obstacle position z:" << " "<< m_ObstaclePars.z );
	  }
	else 
	  break;
	if (!strm.eof())
	  {
	    strm >> m_ObstaclePars.theta;
	    ODEBUG( "obstacle orientation:" << " "<< m_ObstaclePars.theta );
	  }
	else 
	  break;
	if (!strm.eof())
	  {
	    strm >> m_ObstaclePars.h;
	    ODEBUG( "obstacle height:" << " "<< m_ObstaclePars.h );
	  }
	else 
	  break;
	if (!strm.eof())
	  {
	    strm >> m_ObstaclePars.w;
	    ODEBUG( "obstacle width:" << " "<< m_ObstaclePars.w );
	  }
	else 
	  break;
	if (!strm.eof())
	  {
	    strm >> m_ObstaclePars.d;
	    ODEBUG( "obstacle depth:" << " "<< m_ObstaclePars.d );
	  }
	else 
	  break;
	if (!strm.eof())
	  {
	    strm >> m_ObstacleDetected;
	    ODEBUG( "m_ObstacleDetected:" << " "<< m_ObstacleDetected );
	    ReadObstacleParameters = true;
	    break;
	  }
	else 
	  {
	    cout << "Not enough inputs for completion of obstacle information structure!" << endl;
	    break;
	  }
		
      }
	
  }

  void PatternGeneratorInterface::m_SetArmParameters(istringstream &strm)
  {
    ODEBUG("Arm Parameters");
    while(!strm.eof())
      {
	if (!strm.eof())
	  {
	    strm >> m_GainFactor;
	  }
	else break;
      }
  }

  void PatternGeneratorInterface::m_SetZMPShiftParameters(istringstream &strm)
  {
    ODEBUG("SetZMPShitParameters");
    while(!strm.eof())
      {
	if (!strm.eof())
	  {
	    strm >> m_ZMPShift[0];
		
	  }
	else break;
	if (!strm.eof())
	  {
	    strm >> m_ZMPShift[1];

	  }
	else break;
	if (!strm.eof())
	  {
	    strm >> m_ZMPShift[2];

	  }
	else break;
	if (!strm.eof())
	  {
	    strm >> m_ZMPShift[3];

	  }
	else break;
      }
  }

  void PatternGeneratorInterface::m_WhichWalkMode(istringstream &strm)
  {
    while(!strm.eof())
      {
	if (!strm.eof())
	  {
	    int lWalkMode;
	    strm >> lWalkMode;
	    m_StepStackHandler->SetWalkMode(lWalkMode);
	  }
	else break;
      }
  }

  void PatternGeneratorInterface::m_SetLimitsFeasibility(istringstream &strm)
  {
    while(!strm.eof())
      {
	if (!strm.eof())
	  {
	    strm >> m_DeltaFeasibilityLimit;
	  }
	else break;
      }
  }


  void PatternGeneratorInterface::m_StepSequence(istringstream &strm)
  {

    ODEBUG6("Step Sequence","DebugGMFKW.dat");
    ofstream DebugFile;

    // Read the data inside strm.	

	
    switch (m_StepStackHandler->GetWalkMode())
      {
      case 0:
      case 4:
      case 3:
      case 1:
	{
	  m_StepStackHandler->ReadStepSequenceAccordingToWalkMode(strm);
	  break;
	}
      case 2:
	{	
	  ODEBUG( "Walk Mode with Obstacle StepOver Selected \
                 (obstacle parameters have to be set first, \
                 if not standard dimensions are used)" );
	  m_StOvPl->SetObstacleInformation(m_ObstaclePars);
	  m_StOvPl->SetDeltaStepOverCOMHeightMax(m_DeltaFeasibilityLimit);
	  //cout << "I am calculating relative positions to negociate obstacle" << endl;

	  // Update stack of relative foot by using StpOvPl.
	  m_StepStackHandler->ReadStepSequenceAccordingToWalkMode(strm);

	  break;
	}
      default: 
	{
	  ODEBUG3( "PLease select proper walk mode. \
            (0 for normal walking ; \
             1 for walking with waistheight variation ; \
             2 for walking with obstacle stepover)" );
	  return;
	}
      }

    ODEBUG6("Will start the last sequence.","DebugGMFWK.dat");
    FinishAndRealizeStepSequence();
  }
  
  void PatternGeneratorInterface::EvaluateStartingCOM(VNL::Matrix<double>  & BodyAnglesIni,
						      VNL::Vector<double>  & lStartingCOMPosition,
						      FootAbsolutePosition & InitLeftFootAbsPos,
						      FootAbsolutePosition & InitRightFootAbsPos)
  {
    m_ZMPpcwmbz->EvaluateStartingCoM(BodyAnglesIni,lStartingCOMPosition,
				     InitLeftFootAbsPos, InitRightFootAbsPos);

    
  }

  void PatternGeneratorInterface::EvaluateStartingCOM(VNL::Matrix<double>  & BodyAnglesIni,
						      VNL::Vector<double>  & lStartingCOMPosition,
						      VNL::Vector<double>  & lStartingWaistPosition,
						      FootAbsolutePosition & InitLeftFootAbsPos,
						      FootAbsolutePosition & InitRightFootAbsPos)
  {
    m_ZMPpcwmbz->EvaluateStartingCoM(BodyAnglesIni,lStartingCOMPosition, lStartingWaistPosition,
				     InitLeftFootAbsPos, InitRightFootAbsPos);

    
  }

  void PatternGeneratorInterface::CommonInitializationOfWalking(VNL::Vector<double>  & lStartingCOMPosition,
								VNL::Matrix<double>  & BodyAnglesIni,
								FootAbsolutePosition & InitLeftFootAbsPos, 
								FootAbsolutePosition & InitRightFootAbsPos,
							        deque<RelativeFootPosition> & lRelativeFootPositions,
								vector<double> & lCurrentJointValues,
								bool ClearStepStackHandler)
  {
    m_FootAbsolutePositions.clear();
    m_ZMPPositions.clear();
    m_LeftFootPositions.clear();
    m_RightFootPositions.clear();

    lCurrentJointValues.resize(m_CurrentJointValues.size());
  
    for(unsigned int i=0;i<m_CurrentJointValues.size();i++)
      lCurrentJointValues[i] = m_CurrentJointValues[i];


    BodyAnglesIni.Resize(m_DOF,1);


    for(int j=0; j<m_DOF;j++)
      {
	BodyAnglesIni[j][0] = lCurrentJointValues[j];
      }

    // Very important for a proper initialization of the COM.
    m_ZMPpcwmbz->EvaluateStartingCoM(BodyAnglesIni,lStartingCOMPosition,
				     InitLeftFootAbsPos, InitRightFootAbsPos);
  
    m_StepStackHandler->CopyRelativeFootPosition(lRelativeFootPositions,ClearStepStackHandler);

    if (0)
      {
	
	ofstream aof;
	aof.open("/tmp/output.txt", ofstream::out);
	if (aof.is_open())
	  {
	    for(unsigned int i=0;i<lRelativeFootPositions.size();i++)
	      {
		aof << lRelativeFootPositions[i].sx <<" " 
		    << lRelativeFootPositions[i].sy <<" " 
		    << lRelativeFootPositions[i].theta 
		    << endl;
	      }
	  }
      }

  }

  void PatternGeneratorInterface::m_StartOnLineStepSequencing(istringstream &strm2)
  {
    StartOnLineStepSequencing();

  }

  void PatternGeneratorInterface::StartOnLineStepSequencing()
  {
    VNL::Vector<double> lStartingCOMPosition(3,0.0);	
    VNL::Matrix<double> BodyAnglesIni;
    FootAbsolutePosition InitLeftFootAbsPos, InitRightFootAbsPos;
    deque<RelativeFootPosition> lRelativeFootPositions;
    vector<double> lCurrentJointValues;

    ODEBUG("StartOnLineStepSequencing - 1 ");
    m_StepStackHandler->StartOnLineStep();

    ODEBUG("StartOnLineStepSequencing - 2 ");
    CommonInitializationOfWalking(lStartingCOMPosition,
				  BodyAnglesIni,
				  InitLeftFootAbsPos, InitRightFootAbsPos,
				  lRelativeFootPositions,lCurrentJointValues,false);

    ODEBUG("StartOnLineStepSequencing - 3 " << lStartingCOMPosition << " "
	    << lRelativeFootPositions.size() 
	    );

    int NbOfStepsToRemoveFromTheStack=m_ZMPD->InitOnLine(m_ZMPPositions,
							 m_LeftFootPositions,
							 m_RightFootPositions,
							 InitLeftFootAbsPos,
							 InitRightFootAbsPos,
							 lRelativeFootPositions,
							 lStartingCOMPosition );

    // Keep the last one to be removed at the next insertion.
    for(int i=0;i<NbOfStepsToRemoveFromTheStack-1;i++)
      m_StepStackHandler->RemoveFirstStepInTheStack();


    // Initialization of the COM buffer.
    m_COMBuffer.resize(m_ZMPPositions.size());
    for(unsigned int i=0;i<m_COMBuffer.size();i++)
      {
	m_COMBuffer[i].z[0] = m_PC->GetHeightOfCoM();
	m_COMBuffer[i].z[1] = 0.0;
	m_COMBuffer[i].z[2] = 0.0;

	m_COMBuffer[i].omega = 0.0;
	m_COMBuffer[i].hip = 0.0;
      
      }
    
    // Initialization of the upper body motion.
    m_UpperBodyPositionsBuffer.resize(m_ZMPPositions.size());
    for(unsigned int i=0;i<m_UpperBodyPositionsBuffer.size();i++)
      {
	m_UpperBodyPositionsBuffer[i].Joints.resize(28);
	
	if (i==0)
	  {
	    for(unsigned int j=0;j<28;j++)
	      m_UpperBodyPositionsBuffer[i].Joints[j] = lCurrentJointValues[j+12];
	  }
	// Initialize the upper body motion to the current stored value.
	for(unsigned int j=0;j<28;j++)
	  m_UpperBodyPositionsBuffer[i].Joints[j] = m_UpperBodyPositionsBuffer[0].Joints[j];
      }
    
    // Initialization of the first preview.
    for(int j=0; j<m_DOF;j++)
      {
	BodyAnglesIni[j][0] = lCurrentJointValues[j];
	ODEBUG4(BodyAnglesIni[j][0],"DebugDataOnLine.txt");
      }

    m_ZMPpcwmbz->Setup(m_ZMPPositions,
		       m_COMBuffer,
		       m_LeftFootPositions,
		       m_RightFootPositions,
		       BodyAnglesIni);
    
    m_ShouldBeRunning=true;
    ODEBUG("StartOnLineStepSequencing - 4 "
	    << m_ZMPPositions.size() << " "
	    << m_LeftFootPositions.size() << " "
	    << m_RightFootPositions.size() << " ");
  }
  
  void PatternGeneratorInterface::StopOnLineStepSequencing()
  {
    m_StepStackHandler->StopOnLineStep();
  }

  void PatternGeneratorInterface::m_StopOnLineStepSequencing(istringstream &strm2)
  {
    StopOnLineStepSequencing();
  }

 void PatternGeneratorInterface::FinishAndRealizeStepSequence()
  { 
    VNL::Vector<double> lStartingCOMPosition(3,0.0);	
    VNL::Matrix<double> BodyAnglesIni;
    FootAbsolutePosition InitLeftFootAbsPos, InitRightFootAbsPos;
    struct timeval begin, end, time1, time2, time3, time4, time5, time6;
    
    gettimeofday(&begin,0);

    ODEBUG6("FinishAndRealizeStepSequence() - 1","DebugGMFKW.dat");
    m_Xmax = m_IK->ComputeXmax(m_ZARM); // Laaaaaazzzzzyyyyy guy...

    vector<double> lCurrentJointValues;
    m_ZMPD->SetZMPShift(m_ZMPShift);

    deque<RelativeFootPosition> lRelativeFootPositions;
    CommonInitializationOfWalking(lStartingCOMPosition,
				  BodyAnglesIni,
				  InitLeftFootAbsPos, InitRightFootAbsPos,
				  lRelativeFootPositions,lCurrentJointValues,true);
    

    ODEBUG6("Size of lRelativeFootPositions :" << lRelativeFootPositions.size(),"DebugGMFKW.dat");
    // Create the appropriate arrays
    m_ZMPD->GetZMPDiscretization(m_ZMPPositions,
				 m_FootAbsolutePositions,
				 lRelativeFootPositions,
				 m_LeftFootPositions,
				 m_RightFootPositions,
				 m_LeftHandPositions,
				 m_RightHandPositions,
				 m_Xmax, lStartingCOMPosition,
				 InitLeftFootAbsPos,
				 InitRightFootAbsPos);

    deque<ZMPPosition> aZMPBuffer;

    // Option : Use Wieber06's algorithm to compute a new ZMP 
    // profil. Suppose to preempt the first stage of control.
    deque<ZMPPosition> NewZMPPositions;

    cout << "COM_Buffer size: " << m_COMBuffer.size() << endl;
    if (m_BoolPBWAlgo)
      {
	m_ZMPD->BuildZMPTrajectoryFromFootTrajectory(m_LeftFootPositions,
						     m_RightFootPositions,
						     m_ZMPPositions,
						     NewZMPPositions,
						     m_COMBuffer,
						     m_ConstraintOnX,
						     m_ConstraintOnY,
						     m_QP_T,
						     m_QP_N);
	if (m_ZMPPositions.size()!=NewZMPPositions.size())
	  {
	    cout << "Problem here between m_ZMPPositions and new zmp positions" << endl;
	    cout << m_ZMPPositions.size() << " " << NewZMPPositions.size() << endl;
	  }

	for(unsigned int i=0;i<m_ZMPPositions.size();i++)
	  m_ZMPPositions[i] = NewZMPPositions[i];

	
      }
    else
      {
	m_COMBuffer.clear();
	m_COMBuffer.resize(m_RightFootPositions.size());
      }

    aZMPBuffer.resize(m_RightFootPositions.size());

    ODEBUG6("FinishAndRealizeStepSequence() - 3 ","DebugGMFKW.dat");

    gettimeofday(&time2,0);
    //this function calculates a buffer with COM values after a first preview round,
    // currently required to calculate the arm swing before "onglobal step of control"
    // in order to take the arm swing motion into account in the second preview loop
    if (m_StepStackHandler->GetWalkMode()==2)
      m_StOvPl->CreateBufferFirstPreview(m_COMBuffer,aZMPBuffer,m_ZMPPositions);

	
    ODEBUG6("FinishAndRealizeStepSequence() - 4 ","DebugGMFKW.dat");
    for(unsigned int i=0;i<m_COMBuffer.size();i++)
      {
	m_COMBuffer[i].z[0] = m_PC->GetHeightOfCoM();
	m_COMBuffer[i].z[1] = 0.0;
	m_COMBuffer[i].z[2] = 0.0;

	m_COMBuffer[i].omega = 0.0;
	m_COMBuffer[i].hip = 0.0;
      
      }
    
    if (0)
      {
	m_ZMPD->DumpDataFiles("/tmp/ZMPSetup.dat",
			      "/tmp/LeftFootAbsolutePosSetup.dat",
			      m_ZMPPositions,
			      m_LeftFootPositions);
	
	
	m_ZMPD->DumpDataFiles("/tmp/ZMPSetup.dat",
			      "/tmp/RightFootAbsolutePosSetup.dat",
			      m_ZMPPositions,
			      m_RightFootPositions);
      }


  

    // read upperbody data which has to be included in the patterngeneration second preview loop stability
    string BodyDat = "UpperBodyDataFile.dat";

    ODEBUG6("FinishAndRealizeStepSequence() - 5 ","DebugGMFKW.dat");
    // Link the current trajectory and GenerateMotionFromKineoWorks.
  
    // Specify the buffer.

    m_UpperBodyPositionsBuffer.clear();
   

    if (m_StepStackHandler->GetWalkMode()==3)
      {
	ODEBUG6("Before Starting GMFKW","DebugGMFKW.dat");
	m_GMFKW->CreateBufferFirstPreview(m_ZMPPositions);
      
	// Map the path found by KineoWorks onto the ZMP buffer.
	m_ConversionForUpperBodyFromLocalIndexToRobotDOFs.clear();
      
	m_GMFKW->ComputeUpperBodyPosition(m_UpperBodyPositionsBuffer,
					  m_ConversionForUpperBodyFromLocalIndexToRobotDOFs);
	ODEBUG6("After GMFKW","DebugGMFKW.dat");
      }
    else 
      {
	// Create the stack of upper body motion.
	m_UpperBodyPositionsBuffer.resize(m_ZMPPositions.size());


	for(unsigned int i=0;i<m_UpperBodyPositionsBuffer.size();i++)
	  {
	    m_UpperBodyPositionsBuffer[i].Joints.resize(28);

	    if (i==0)
	      {
		for(unsigned int j=0;j<28;j++)
		  m_UpperBodyPositionsBuffer[i].Joints[j] = lCurrentJointValues[j+12];
	      }
	    // Initialize the upper body motion to the current stored value.
	    for(unsigned int j=0;j<28;j++)
	      m_UpperBodyPositionsBuffer[i].Joints[j] = m_UpperBodyPositionsBuffer[0].Joints[j];
	  }
      

	// Create the conversion array.
	m_ConversionForUpperBodyFromLocalIndexToRobotDOFs.resize(m_DOF);
	for(unsigned int i=0;i<m_ConversionForUpperBodyFromLocalIndexToRobotDOFs.size();i++)
	  m_ConversionForUpperBodyFromLocalIndexToRobotDOFs[i] = i;
      }

    ODEBUG6("FinishAndRealizeStepSequence() - 6 ","DebugCG.ctx");
    
    for(unsigned int j=0;
	j<m_UpperBodyPositionsBuffer[0].Joints.size();
	j++)
      {
 	BodyAnglesIni(m_ConversionForUpperBodyFromLocalIndexToRobotDOFs[j]+12,0) =
	  m_UpperBodyPositionsBuffer[0].Joints[j];	  
      }
	
    gettimeofday(&time3,0);    
    ODEBUG6("FinishAndRealizeStepSequence() - 7 ","DebugGMFKW.dat");

    // Very important, you have to make sure that the correct COM position is 
    // set inside this buffer.
    // X and Y  will be defined by the PG, but the height has to be specified.
    // by default it should be Zc.
    // If you want to change use modewalk 2.
    for(unsigned int i=0;i<m_COMBuffer.size();i++)
      {
	m_COMBuffer[i].z[0] = m_PC->GetHeightOfCoM();
	m_COMBuffer[i].z[1] = m_COMBuffer[i].z[2] = 0.0;
      }

    if (m_StepStackHandler->GetWalkMode()==0) 
      {
	for(unsigned int i=0;i<m_ZMPPositions.size()-m_NL;i++)
	  m_COMBuffer[i].theta = m_ZMPPositions[i+m_NL-1].theta;
      }

    if ((m_StepStackHandler->GetWalkMode()==1)	||
	(m_StepStackHandler->GetWalkMode()==3)	)
      {
	m_WaistPlanner->PolyPlanner(m_COMBuffer,lRelativeFootPositions,m_ZMPPositions);
      }

    if(m_StepStackHandler->GetWalkMode()==2)
      {
	m_StOvPl->TimeDistributeFactor(m_TimeDistrFactor);
	m_StOvPl->PolyPlanner(m_COMBuffer,m_LeftFootPositions,m_RightFootPositions,m_ZMPPositions);
      }

#if _DEBUG_4_ACTIVATED_
    ofstream DebugFile;
    DebugFile.open("DebugDataCOM.txt",ofstream::app);
    for(unsigned int i=0;i<m_COMBuffer.size();i++)
      {

	for(int j=0;j<3;j++)
	  {
	    DebugFile << m_COMBuffer[i].x[j] << " ";
	  }

	for(int j=0;j<3;j++)
	  {
	    DebugFile << m_COMBuffer[i].y[j] << " ";
	  }

	for(int j=0;j<3;j++)
	  {
	    DebugFile << m_COMBuffer[i].z[j] << " ";
	  }

	DebugFile << m_COMBuffer[i].omega << " "
		  << m_COMBuffer[i].theta << " "
		  << m_COMBuffer[i].hip;

	DebugFile << endl;
      }

    DebugFile.close();
  
#endif 

    gettimeofday(&time4,0);
    ODEBUG6("FinishAndRealizeStepSequence() - 8 ","DebugGMFKW.dat");
    // Read NL informations from ZMPRefPositions.
    int localWalkMode = m_StepStackHandler->GetWalkMode();
    if (localWalkMode==0)
      {
	m_ZMPpcwmbz->SetupFirstPhase(m_ZMPPositions,
				     m_COMBuffer,
				     m_LeftFootPositions,
				     m_RightFootPositions,
				     BodyAnglesIni);

	gettimeofday(&time5,0);
	for(unsigned int i=0;i<m_NL;i++)
	  {
	    VNL::Matrix<double> qArmr(7,1,0.0), qArml(7,1,0.0);
	    COMPosition aCOMPosition;
	    
	    
	    m_ZMPpcwmbz->SetupIterativePhase(m_ZMPPositions,
					     m_COMBuffer,
					     m_LeftFootPositions,
					     m_RightFootPositions,
					     BodyAnglesIni,i);
	    
	    // Take the new COM computed by the first stage of Preview control.
	    aCOMPosition = m_ZMPpcwmbz->GetLastCOMFromFirstStage();
	    
	    m_COMBuffer[i+1].x[0] = aCOMPosition.x[0];
	    m_COMBuffer[i+1].y[0] = aCOMPosition.y[0];
	    m_COMBuffer[i+1].omega = aCOMPosition.omega;
	    m_COMBuffer[i+1].theta = aCOMPosition.theta;
	    
	    // Compute Upper body heuristic according to the COM.
	    ComputeUpperBodyHeuristicForNormalWalking(qArmr, qArml,
						      aCOMPosition, 
						      InitRightFootAbsPos, 
						      InitLeftFootAbsPos);
	    
	    for(unsigned int j=0;j<28;j++)
	      m_UpperBodyPositionsBuffer[i].Joints[j]=BodyAnglesIni(j+12,0);


	    for(unsigned int j=0;j<7;j++)
	      {
		BodyAnglesIni(j+16,0) = qArmr(j,0);
		BodyAnglesIni(j+23,0) = qArml(j,0);
	      }

	  }

      }
    else 
      m_ZMPpcwmbz->Setup(m_ZMPPositions,
			 m_COMBuffer,
			 m_LeftFootPositions,
			 m_RightFootPositions,
			 BodyAnglesIni);

    gettimeofday(&time6,0);
    ODEBUG("FinishAndRealizeStepSequence() - 9 ");
	
    m_ZMPpcwmbz->GetDifferenceBetweenComAndWaist(  m_DiffBetweenComAndWaist);
    m_count = 0;
    ODEBUG("FinishAndRealizeStepSequence() - 10 ");
    
    m_ShouldBeRunning = true;
    gettimeofday(&end,0);
    ODEBUG3(endl << 
	    "Step 1 : "<<  time1.tv_sec-begin.tv_sec + 0.000001 * (time1.tv_usec -begin.tv_usec) << endl << 
	    "Step 2 : "<<  time2.tv_sec-time1.tv_sec + 0.000001 * (time2.tv_usec -time1.tv_usec) << endl << 
	    "Step 3 : "<<  time3.tv_sec-time2.tv_sec + 0.000001 * (time3.tv_usec -time2.tv_usec) << endl << 
	    "Step 4 : "<<  time4.tv_sec-time3.tv_sec + 0.000001 * (time4.tv_usec -time3.tv_usec) << endl << 
	    "Step 5 : "<<  time5.tv_sec-time4.tv_sec + 0.000001 * (time5.tv_usec -time4.tv_usec) << endl << 
	    "Step 6 : "<<  time6.tv_sec-time5.tv_sec + 0.000001 * (time6.tv_usec -time5.tv_usec) << endl << 
	    "Total time : "<< end.tv_sec-begin.tv_sec + 0.000001 * (end.tv_usec -begin.tv_usec) );
  }


  void PatternGeneratorInterface::m_ReadFileFromKineoWorks(istringstream &strm)
  {
  
    string aPartialModel="PartialModel.dat";
    string aKWPath="KWBarPath.pth";

    strm >> aPartialModel;
    strm >> aKWPath;
  
    ODEBUG6("Went through m_ReadFileFromKineoWorks(istringstream &strm)","DebugGMFKW.dat");
    if (m_GMFKW->ReadPartialModel(aPartialModel)<0)
      cerr<< "Error while reading partial model " << endl;

    if (m_GMFKW->ReadKineoWorksPath(aKWPath)<0)
      cerr<< "Error while reading the path " << endl;	
    ODEBUG6("Went before DisplayModel and PAth m_ReadFileFromKineoWorks(istringstream &strm)","DebugGMFKW.dat");
  
    m_GMFKW->DisplayModelAndPath();
    ODEBUG6("Fini..","DebugGMFKW.dat");
  }

  int PatternGeneratorInterface::ParseCmd(istringstream &strm)
  {
    string aCmd;
    strm >> aCmd;
    ODEBUG("PGI:ParseCmd: Commande: " << aCmd);
    if (aCmd==":omega")
      m_SetOmega(strm);
    
    else if (aCmd==":stepheight")
      m_SetStepHeight(strm);
    
    else if (aCmd==":singlesupporttime")
      m_SetSingleSupportTime(strm);

    else if (aCmd==":doublesupporttime")
      m_SetDoubleSupportTime(strm);

    else if (aCmd==":armparameters")
      m_SetArmParameters(strm);

    else if (aCmd==":LimitsFeasibility")
      m_SetLimitsFeasibility(strm);
    
    else if (aCmd==":ZMPShiftParameters")
      m_SetZMPShiftParameters(strm);

    else if (aCmd==":TimeDistributionParameters")
      m_SetTimeDistrParameters(strm);

    else if (aCmd==":UpperBodyMotionParameters")
      m_SetUpperBodyMotionParameters(strm);

    else if (aCmd==":stepseq")
      m_StepSequence(strm);
    
    else if (aCmd==":stepseq")
      m_StepSequence(strm);

    else if (aCmd==":walkmode")
      m_WhichWalkMode(strm);

    else if (aCmd==":lastsupport")
      m_FinishOnTheLastCorrectSupportFoot(strm);

    else if (aCmd==":supportfoot")
      m_PrepareForSupportFoot(strm);

    else if (aCmd==":finish")
      m_FinishAndRealizeStepSequence(strm);

    else if (aCmd==":arccentered")
      m_CreateArcCenteredInStepStack(strm);

    else if (aCmd==":arc")
      m_CreateArcInStepStack(strm);

    else if (aCmd==":StartOnLineStepSequencing")
      m_StartOnLineStepSequencing(strm);

    else if (aCmd==":StopOnLineStepSequencing")
      m_StopOnLineStepSequencing(strm);
    
    else if (aCmd==":readfilefromkw")
      m_ReadFileFromKineoWorks(strm);

    else if (aCmd==":setpbwconstraint")
      m_SetPBWConstraint(strm);

    else if (aCmd==":SetAlgoForZmpTrajectory")
      m_SetAlgoForZMPTraj(strm);
    return 0;
  }

  void PatternGeneratorInterface::m_SetAlgoForZMPTraj(istringstream &strm)
  {
    string ZMPTrajAlgo;
    strm >> ZMPTrajAlgo;
    if (ZMPTrajAlgo=="PBW")
      {
	m_BoolPBWAlgo=1;
	
      }
    else if (ZMPTrajAlgo=="Kajita")
      {
	m_BoolPBWAlgo=0;
      }
    
  }
  void PatternGeneratorInterface::m_SetPBWConstraint(istringstream &strm)
  {
    string PBWCmd;
    strm >> PBWCmd;
    if (PBWCmd=="XY")
      {
	strm >> m_ConstraintOnX;
	strm >> m_ConstraintOnY;
	cout << "Constraint On X: " << m_ConstraintOnX << " Constraint On Y: " << m_ConstraintOnY << endl;
      } 
    else if (PBWCmd=="T")
      {
	strm >> m_QP_T;
	cout << "Sampling for the QP " << m_QP_T <<endl;
      }
    else if (PBWCmd=="N")
      {
	strm >> m_QP_N;
	cout << "Preview window for the QP " << m_QP_N << endl;
      }
  }
  void PatternGeneratorInterface::m_SetOmega(istringstream &strm)
  {
    double anOmega;
    strm >> anOmega;
    ODEBUG("OMEGA :" << anOmega);
    if (m_ZMPD!=0)
      m_ZMPD->SetOmega(anOmega);
  }

  void PatternGeneratorInterface::m_SetStepHeight(istringstream &strm)
  {
    float StepHeight;
    strm >> StepHeight;
    ODEBUG("StepHeight :" << StepHeight);
    if (m_ZMPD!=0)
      m_ZMPD->SetStepHeight(StepHeight);
  }

  void PatternGeneratorInterface::m_SetSingleSupportTime(istringstream &strm)
  {
    //float TSsupport;
    strm >> m_TSsupport;
    ODEBUG("SingleSupportTime :" << m_TSsupport);
    if (m_ZMPD!=0)
      m_ZMPD->SetTSingleSupport(m_TSsupport);

    if (m_StepStackHandler!=0)
      m_StepStackHandler->SetSingleTimeSupport(m_TSsupport);

  }

  void PatternGeneratorInterface::m_SetDoubleSupportTime(istringstream &strm)
  {
	
    strm >> m_TDsupport;
    ODEBUG("DoubleSupportTime :" << m_TDsupport);
    if (m_ZMPD!=0)
      m_ZMPD->SetTDoubleSupport(m_TDsupport);

    if (m_StepStackHandler!=0)
      m_StepStackHandler->SetDoubleTimeSupport(m_TDsupport);

  }

  void PatternGeneratorInterface::m_SetUpperBodyMotionParameters(istringstream &strm)
  {
    ODEBUG("Upper Body Motion Parameters");
    while(!strm.eof())
      {
	if (!strm.eof())
	  {
	    strm >> m_UpperBodyMotion[0];
	  }
	else break;
	if (!strm.eof())
	  {
	    strm >> m_UpperBodyMotion[1];
	  }
	else break;
	if (!strm.eof())
	  {
	    strm >> m_UpperBodyMotion[2];
	  }
	else break;
      }
  }


  void PatternGeneratorInterface::ComputeUpperBodyHeuristicForNormalWalking(    VNL::Matrix<double> &qArmr, 
										VNL::Matrix<double> &qArml,
										COMPosition aCOMPosition,
										FootAbsolutePosition RFP,
										FootAbsolutePosition LFP )
  {


    m_Xmax = m_IK->ComputeXmax(m_ZARM);
    
    double GainX = m_GainFactor * m_Xmax/0.2;
    
    // Arms Motion : Heuristic based.
    double Alpha,Beta;
    //Temporary variables
    double TempXL,TempXR,TempCos,TempSin,
      TempARight,TempALeft;

    // Compute the position of the hand according to the 
    // leg.
    TempCos = cos(aCOMPosition.theta*M_PI/180.0);
    TempSin = sin(aCOMPosition.theta*M_PI/180.0);
    
    TempXR = TempCos * (RFP.x  - aCOMPosition.x[0]) + 
      TempSin * (RFP.y  - aCOMPosition.y[0]);
    TempXL = TempCos * (LFP.x  - aCOMPosition.x[0]) + 
      TempSin * (LFP.y  - aCOMPosition.y[0]);
    
    ODEBUG4("COM:" << aCOMPosition.x[0] << " " << aCOMPosition.y[0],"DebugDataIKArms.txt");

    TempARight = TempXR*-1.0;
    TempALeft = TempXL*-1.0;

    ODEBUG4("Values:" << TempALeft * GainX << " " << m_ZARM << " " << m_Xmax,"DebugDataIKArms.txt");
    // Compute angles using inverse kinematics and the computed hand position.
    m_IK->ComputeInverseKinematicsForArms(TempALeft * GainX,
					  m_ZARM,
					  Alpha,
					  Beta);

    ODEBUG6("ComputeHeuristicArm: Step 2 ","DebugDataIKArms.txt");
    qArml(0,0)=Alpha;
    qArml(1,0)=10.0*M_PI/180.0;
    qArml(2,0)= 0.0;
    qArml(3,0)= Beta;
    qArml(4,0)= 0.0;
    qArml(5,0)= 0.0;
    qArml(6,0)= 10.0*M_PI/180.0;
      
    ODEBUG4( "IK Left arm p:" << qArml(0,0)<< " " <<  qArml(1,0)  << " " << qArml(2,0) 
	     << " " << qArml(3,0) << "  " << qArml(4,0) << " " << qArml(5,0), "DebugDataIKArms.txt" );
		
		
    m_IK->ComputeInverseKinematicsForArms(TempARight * GainX,
					  m_ZARM,
					  Alpha,
					  Beta);
    qArmr(0,0)=Alpha;
    qArmr(1,0)=-10.0*M_PI/180.0;
    qArmr(2,0)= 0.0;
    qArmr(3,0)= Beta;
    qArmr(4,0)= 0.0;
    qArmr(5,0)= 0.0;
    qArmr(6,0)= 10.0*M_PI/180.0;;

    ODEBUG4( "IK Right arm p:" << qArmr(0,0)<< " " <<  qArmr(1,0)  << " " << qArmr(2,0) 
	     << " " << qArmr(3,0) << "  " << qArmr(4,0) << " " << qArmr(5,0), "DebugDataIKArms.txt" );


  }  
  bool PatternGeneratorInterface::RunOneStepOfTheControlLoop(VNL::Matrix<double> &qr,
							     VNL::Matrix<double> &ql,
							     VNL::Matrix<double> &UpperBodyAngles,
							     VNL::Matrix<double> &ZMPTarget,
							     COMPosition & outWaistPosition
							     )
						    
  {
    
    long int u=0;
    if ((!m_ShouldBeRunning) ||
	(m_ZMPPositions.size()< 2*m_NL))
      return false;

    double qWaistYaw=0.0;
    VNL::Matrix<double> qArmr(7,1,0.0), qArml(7,1,0.0);
    VNL::Matrix<double> FutureUpperBodyAngles(28,1,0.0);
    VNL::Matrix<double> Futureqr(6,1,0.0),Futureql(6,1,0.0);
    VNL::Matrix<double> FutureBodyAttitude(3,3,0.0);
    COMPosition COMPositionFromPC1,finalCOMPosition;



    // New scheme:
    // Update the queue of ZMP ref
    m_ZMPpcwmbz->UpdateTheZMPRefQueue(m_ZMPPositions[2*m_NL-1]);

    if (m_StepStackHandler->GetWalkMode()==0)
      {
	m_COMBuffer[m_NL-1].theta = m_ZMPPositions[m_NL-1].theta;
      }
    COMPositionFromPC1 = m_COMBuffer[m_NL-1];

    // Compute the first stage to get the COM evaluation from the first stage of control.
    m_ZMPpcwmbz->FirstStageOfControl(m_LeftFootPositions[m_NL-1],
				     m_RightFootPositions[m_NL-1],
				     COMPositionFromPC1,
				     Futureql,Futureqr,FutureBodyAttitude);
	
    ODEBUG4("RunOneStepOfTheControlLoop: Step 0 " << u << " " << m_count << " " << m_COMBuffer.size() << 
	    " " << m_ZMPPositions.size()<< " NL:" << m_NL,"DebugData.txt");
    
    ODEBUG4("COM from RunOneStep: " << m_COMBuffer.size() << " "
	    << m_COMBuffer[m_NL-1].x[0] << " " 
	    << m_COMBuffer[m_NL-1].y[0], "DebugData.txt");


    
    // Stepping over and wakl mode are done in three phases

    // First generate the upper body motion according to Kajita's heuristic 
    if(m_StepStackHandler->GetWalkMode()<3)
      {
	ComputeUpperBodyHeuristicForNormalWalking(    qArmr, 
						      qArml,
						      COMPositionFromPC1,
						      m_RightFootPositions[m_NL-1],
						      m_LeftFootPositions[m_NL-1]
						      );
	// Initialize properly the futur upper body angles.
	// Basically we take what is given by the user,
	// and just modify the arms.
	for(unsigned int i=0;i<FutureUpperBodyAngles.Rows();i++)
	  FutureUpperBodyAngles(i,0) = UpperBodyAngles(i,0);
      }
    
    // For stepping over modify the waist position and
    // according to parameters the arms motion.
    if(m_StepStackHandler->GetWalkMode()==2)
      {
	///this angle is introduced to rotate the upperbody when the waist is rotated during stepover
	qWaistYaw = -m_COMBuffer[m_NL-1].theta*M_PI/180.0;
	ODEBUG4(qWaistYaw,"DebugDataWaistYaw.dat");
	//this is not correct yet since it uses COMPositionFromPC1.theta which also changes when turning....
	//it will be modified in the near future	
	//include waistrotation in dynamic model for second preview correction
			
	  
	UpperBodyAngles(0,0) = qWaistYaw;	
				
	if (m_UpperBodyMotion[0]!=0)
	  {
	    UpperBodyAngles(1,0) = m_UpperBodyMotion[0]*fabs(qWaistYaw);
				
	  }
	if (m_UpperBodyMotion[1]!=0)
	  {
	    qArmr(0,0)=qArmr(0,0)-m_UpperBodyMotion[1]*fabs(qWaistYaw);
	    qArml(0,0)=qArml(0,0)-m_UpperBodyMotion[1]*fabs(qWaistYaw);
	  }

	if (m_UpperBodyMotion[2]!=0)
	  {
				
	    COMPositionFromPC1.omega = m_UpperBodyMotion[2]*fabs(COMPositionFromPC1.theta);
	  }
		
      } 
    
      
    // Third compute future arm motion.
    if (m_StepStackHandler->GetWalkMode()<3)
      {
	//update upperbody buffer with new angles		 	
	for(unsigned int j=0;j<7;j++) 
	  {
	    // For the future m_NL state .
	    FutureUpperBodyAngles(j+4,0) =
	      m_UpperBodyPositionsBuffer[m_NL-1].Joints[j+4] = qArmr(j,0);
	    FutureUpperBodyAngles(j+11,0) = 
	      m_UpperBodyPositionsBuffer[m_NL-1].Joints[j+11] = qArml(j,0);	
	      
	    // For the current state.
	    UpperBodyAngles(j+4,0) = m_UpperBodyPositionsBuffer[0].Joints[j+4];
	    UpperBodyAngles(j+11,0) = m_UpperBodyPositionsBuffer[0].Joints[j+11];
	  }
		
      }

    // Do differently if the user specify the upper body motion.
    if (m_StepStackHandler->GetWalkMode()==4)
      {


	for(unsigned int j=0;j<28;j++) 
	  {
	    m_UpperBodyPositionsBuffer[m_NL-1].Joints[j] = 
	      FutureUpperBodyAngles(j,0) = m_CurrentJointValues[j+12];
	    // For the current state.
	    UpperBodyAngles(j,0) = m_UpperBodyPositionsBuffer[0].Joints[j];
	  }
      }

     
    // Also in the case of planning.
    if (m_StepStackHandler->GetWalkMode()==3)
      {
	for(unsigned int j=0;
	    j<m_UpperBodyPositionsBuffer[m_NL-1].Joints.size();
	    j++)
	  {
	    int r=m_ConversionForUpperBodyFromLocalIndexToRobotDOFs[j];
	    FutureUpperBodyAngles(r,0) =
	      m_UpperBodyPositionsBuffer[m_NL-1].Joints[j];
	    UpperBodyAngles(r,0) =
	      m_UpperBodyPositionsBuffer[0].Joints[j];	  
	      
	  }
      }

#if 0

    // MAJOR FLAW in putting this computation here 
    // If the upper body has to be computed on the m_NL-1 state of the COM
    // then it should be computed by the result of the COM.
    // which should be computed BEFORE.
    // The correct way is to split the first stage and the second stage of control.
   

    // The call which computes the current state of the robot's legs qr, ql,
    // based on the futur which is updated by providing:
    // the k+NL left and right foot position, the k+NL COM position,
    // the k+NL FutureUpperBodyAngles.
    // Be aware, the current COM Position is send back in COMPositionFromPC1.
    m_ZMPpcwmbz->OneGlobalStepOfControl(m_LeftFootPositions[m_NL-1],
					m_RightFootPositions[m_NL-1],
					m_ZMPPositions[2*m_NL-1],
					qr,ql,
					COMPositionFromPC1,FutureUpperBodyAngles);
#else
    // New scheme : 
    // Evaluate the multibody ZMP including upper body motion
    // derived from the first stage of control.
    m_ZMPpcwmbz->EvaluateMultiBodyZMP(Futureql,Futureqr,FutureUpperBodyAngles,FutureBodyAttitude,-1);
    // And compute the second stage of control.
    m_ZMPpcwmbz->SecondStageOfControl(qr,ql,finalCOMPosition);
#endif	

    ODEBUG4( ql(0,0)*180.0/M_PI << " " <<  ql(1,0)*180.0/M_PI << " " << ql(2,0)*180.0/M_PI << " " << 
	     ql(3,0)*180.0/M_PI << "  " << ql(4,0)*180.0/M_PI << " " << ql(5,0)*180.0/M_PI, "LeftLegAngle.txt" );
    ODEBUG4( qr(0,0)*180.0/M_PI << " " <<  qr(1,0)*180.0/M_PI << " " << qr(2,0)*180.0/M_PI << " " << 
	     qr(3,0)*180.0/M_PI << "  " << qr(4,0)*180.0/M_PI << " " << qr(5,0)*180.0/M_PI, "RightLegAngle.txt" );
    ODEBUG4( finalCOMPosition.x[0] << " " << finalCOMPosition.y[0] << " " << 
	     m_ZMPPositions[0 + 2*m_NL-1].px << " " << 
	     m_LeftFootPositions[m_NL-1].x << " " << m_RightFootPositions[m_NL-1].x, "ZMPPCW.txt");
     
    for(int j=0;j<6;j++)
      {
	m_dql(j,0) = (ql(j,0)-m_prev_ql(j,0))/m_SamplingPeriod;
	m_dqr(j,0) = (qr(j,0)-m_prev_qr(j,0))/m_SamplingPeriod;
	  
	m_prev_ql(j,0) = ql(j,0);
	m_prev_qr(j,0) = qr(j,0);

	// Try to deal with the first speed which might be problematic. 
	// Indeed m_prev_ql and m_prev_qr are set to zero whereas they
	// should be set to the initial positions. 
	// For convenience the speed is set to zero.
	if (m_count==0)
	  {
	    m_dql(j,0) = 0;
	    m_dqr(j,0) = 0;
	  }
      }

    m_COMBuffer[0] = finalCOMPosition;  
		
    //retrieve correct angles at instant m_count for sending to the robot motors 
    //also lazy right now by keeping variables qArmr and so
      			 	

    for(unsigned int j=0;j<7;j++) 
      {
	qArmr(j,0) = UpperBodyAngles(j+4,0);
	qArml(j,0) = UpperBodyAngles(j+11,0);	
      }
                
    qWaistYaw = UpperBodyAngles(0,0);	

    m_count++;

    VNL::Matrix<double> FinalDesiredCOMPose = m_ZMPpcwmbz->GetFinalDesiredCOMPose();			
    VNL::Vector<double> PosOfWaistInCOMF = m_ZMPpcwmbz->GetCurrentPositionofWaistInCOMFrame();
    VNL::Vector<double> AbsWaist;
    AbsWaist = FinalDesiredCOMPose * PosOfWaistInCOMF;
    
    outWaistPosition = finalCOMPosition;
    outWaistPosition.x[0] =  AbsWaist[0];
    outWaistPosition.y[0] =  AbsWaist[1];
    outWaistPosition.z[0] =  AbsWaist[2];

    double temp1;
    double temp2;
    double temp3;
    temp1 = m_ZMPPositions[0].px - outWaistPosition.x[0];
    temp2 = m_ZMPPositions[0].py - outWaistPosition.y[0];
    temp3 = finalCOMPosition.theta*M_PI/180.0;

    ZMPTarget(0,0) = cos(temp3)*temp1+sin(temp3)*temp2 ;
    ZMPTarget(1,0) = -sin(temp3)*temp1+cos(temp3)*temp2;
    ZMPTarget(2,0) = -finalCOMPosition.z[0] - m_DiffBetweenComAndWaist[2];
      
    ODEBUG4(finalCOMPosition.z[0] << " " << m_DiffBetweenComAndWaist[2] << " " << ZMPTarget(2,0),
	    "DebugDataZMPTargetZ.dat");
    m_ZMPPositions.pop_front();
    m_COMBuffer.pop_front();
    m_LeftFootPositions.pop_front();
    m_RightFootPositions.pop_front();
    m_UpperBodyPositionsBuffer.pop_front();

    m_CurrentWaistState = outWaistPosition;

    bool UpdateAbsMotionOrNot = false;

    //    if ((u=(m_count - (m_ZMPPositions.size()-2*m_NL)))>=0)
    if ((u=m_ZMPPositions.size()-2*m_NL)==0)
      {
	if (m_StepStackHandler->IsOnLineSteppingOn())
	  {
	    // CAREFULL: we assume that this sequence will create temporary
	    // two relative foot steps inside the StepStackHandler.
	    // The first step in the stack is the previous one.
	    // The second step is the new one.
	    RelativeFootPosition lRelativeFootPositions;
	    // Add a new step inside the stack.
	    m_StepStackHandler->AddStandardOnLineStep(m_NewStep, m_NewStepX, m_NewStepY, m_NewTheta);
	    m_NewStep = false;

	    // Returns the newly created foot step.
	    lRelativeFootPositions = m_StepStackHandler->ReturnBackFootPosition();
	    
	    // Remove the previous step.
	    bool EndSequence = m_StepStackHandler->RemoveFirstStepInTheStack();
	    
	    // Create the new values for the ZMP and feet trajectories queues.
	    int beforesize = m_ZMPPositions.size();
	    m_ZMPD->OnLine(lRelativeFootPositions,
			   m_ZMPPositions,
			   m_LeftFootPositions,
			   m_RightFootPositions,
			   EndSequence);
	    int aftersize = m_ZMPPositions.size();

	    // Create the new values for the COM and Upper body positions queues.
	    ExpandCOMAndUpperBodyPositionsQueues(aftersize-beforesize);

	  }
	else
	  {
	    //	cout << "Sorry not enough information" << endl;
	    m_ShouldBeRunning = false;
	    UpdateAbsMotionOrNot = true;
	    ODEBUG("m_count " << m_count <<
		    " m_ZMPPositions.size() " << m_ZMPPositions.size() << 
		    " u : " << u);
	  }
      }

    // Update the absolute position of the robot.
    UpdateAbsolutePosition(UpdateAbsMotionOrNot);
    return true;
  }

  void PatternGeneratorInterface::DebugControlLoop(VNL::Matrix<double> &qr,
						   VNL::Matrix<double> &ql,
						   VNL::Matrix<double> &UpperBodyAngles,
						   COMPosition aWaistPosition,
						   int localindex)
  {
    
    if (m_ZMPPositions.size()-2*m_NL<0)
      return;


    VNL::Matrix<double> dqlRefState(6,1,0.0), dqrRefState(6,1,0.0);
    VNL::Matrix<double> qArmr(7,1,0.0), qArml(7,1,0.0);

    int LINKSFORRLEG[6] = { 0, 1, 2, 3,  4, 5};
    int LINKSFORLLEG[6] = { 6, 7, 8, 9, 10, 11};
    //    int LINKSFORRARM[6] = { 16, 17, 18, 19, 20, 21};
    //    int LINKSFORLARM[6] = { 23, 24, 25, 26, 27, 28};

    VNL::Matrix<double> dql(6,1,0.0), dqr(6,1,0.0);
    VNL::Matrix<double> dqal(6,1,0.0), dqar(6,1,0.0);
    VNL::Matrix<double> ddql(6,1,0.0), ddqr(6,1,0.0);
    VNL::Matrix<double> ddqal(6,1,0.0), ddqar(6,1,0.0);

    VNL::Matrix<double> dUpperBodyAngles(28,1,0.0);

    double ZMPmultibody[2]={0.0,0.0};
    vector3d P(0,0,0),L(0,0,0),  dP(0,0,0), dL(0,0,0);

    double qWaistYaw=0.0,dqWaistYaw=0.0,ddqWaistYaw;
    //get real angles and ZMP
	
    // Computes New Multybody ZMP.  and angular velocity of real angles
    //upperbody angles
    //	m_2DMB->Setq(LINKSFORUPPERBODY[0],qWaistYaw);
    //	m_2DMB->Setq(LINKSFORUPPERBODY[1],0.0);
	
    if (m_count!=1)
      {
	dqWaistYaw = (qWaistYaw-m_Debug_prev_qWaistYaw)/m_SamplingPeriod;
	if (m_count>2)
	  ddqWaistYaw = (dqWaistYaw-m_Debug_prev_dqWaistYaw)/m_SamplingPeriod;
	else
	  ddqWaistYaw = 0.0;
      }
    else
      {
	dqWaistYaw = 0.0;
	ddqWaistYaw = 0.0;
      }
			
			
			
    // Update the Dynamic multi body model with upperbody motion
    for(int j=0;j<28;j++)
      {
	      
	m_2DMB->Setq(12+j,UpperBodyAngles(j,0));
	if (m_count>1)
	  dUpperBodyAngles(j,0) = (UpperBodyAngles(j,0)- m_Debug_prev_UpperBodyAngles(j,0))/m_SamplingPeriod;
	else
	  dUpperBodyAngles(j,0) = 0.0;
	m_2DMB->Setdq(12+j,dUpperBodyAngles(j,0));
	m_Debug_prev_UpperBodyAngles(j,0) = UpperBodyAngles(j,0);
      }
	
		
    //	m_2DMB->Setdq(LINKSFORUPPERBODY[0],dqWaistYaw);
    //	m_2DMB->Setdq(LINKSFORUPPERBODY[1],0.0);
	
    m_Debug_prev_qWaistYaw = qWaistYaw;
    m_Debug_prev_dqWaistYaw = dqWaistYaw;
		
    for(unsigned int j=0;j<6;j++)
      {
	m_2DMB->Setq(LINKSFORLLEG[j],ql(j,0));
	m_2DMB->Setq(LINKSFORRLEG[j],qr(j,0));
				
	if (m_count>1)
	  {
	    dql(j,0) = (ql(j,0)-m_Debug_prev_ql(j,0))/m_SamplingPeriod;
	    ddql(j,0) = (dql(j,0)-m_Debug_prev_dql(j,0))/m_SamplingPeriod;
	    dqr(j,0) = (qr(j,0)-m_Debug_prev_qr(j,0))/m_SamplingPeriod;
	    ddqr(j,0) = (dqr(j,0)-m_Debug_prev_dqr(j,0))/m_SamplingPeriod;


		  
	    //dqlRefState(j,0) = (qlRefState(j,0)-m_prev_ql_RefState(j,0))/m_SamplingPeriod;
	    //dqrRefState(j,0) = (qrRefState(j,0)-m_prev_qr_RefState(j,0))/m_SamplingPeriod;
					
	  }
	else
	  {
	    dql(j,0) = 0.0;
	    ddql(j,0) = 0.0;
	    dqr(j,0) = 0.0;
	    ddqr(j,0) = 0.0;
	
	    dqlRefState(j,0) = 0.0;
	    dqrRefState(j,0) = 0.0;
	  }
		
			
			
	m_2DMB->Setdq(LINKSFORLLEG[j],dql(j,0));
	m_2DMB->Setdq(LINKSFORRLEG[j],dqr(j,0));
				
	m_Debug_prev_ql(j,0) = ql(j,0);
	m_Debug_prev_dql(j,0) = dql(j,0);
	m_Debug_prev_qr(j,0) = qr(j,0);
	m_Debug_prev_dqr(j,0) = dqr(j,0);
	
	//m_prev_ql_RefState(j,0) = qlRefState(j,0);
	//	      m_Debug_prev_qr_RefState(j,0) = qrRefState(j,0);
				

			
      }
    ODEBUG4( ql(0,0)<< " " <<  ql(1,0)  << " " << ql(2,0) << " " << ql(3,0) << "  " << ql(4,0) << " " << ql(5,0),
	    "DebugDataqlDisplay.txt" );
    ODEBUG4( qr(0,0)<< " " <<  qr(1,0)  << " " << qr(2,0) << " " 
	     << qr(3,0) << "  " << qr(4,0) << " " << qr(5,0), "DebugDataqrDisplay.txt" );


    ODEBUG4( dql(0,0)<< " " <<  dql(1,0)  << " " << dql(2,0) << " " << dql(3,0) << "  " << dql(4,0) << " " << dql(5,0),
	     "DebugDatadqlDisplay.txt" );
    ODEBUG4( dqr(0,0)<< " " <<  dqr(1,0)  << " " << dqr(2,0) << " " 
	     << dqr(3,0) << "  " << dqr(4,0) << " " << dqr(5,0), "DebugDatadqrDisplay.txt" );

    ODEBUG4( dql(0,0)<< " " <<  dql(1,0)  << " " << dql(2,0) << " " << dql(3,0) << "  " << dql(4,0) << " " << dql(5,0),
	     "DebugDatadqlDisplay.txt" );
    
    ODEBUG4( UpperBodyAngles(0,0) << " " <<
	     UpperBodyAngles(1,0) << " " <<
	     UpperBodyAngles(2,0) << " " <<
	     UpperBodyAngles(3,0) << " " <<
	     UpperBodyAngles(4,0) << " " <<
	     UpperBodyAngles(5,0) << " " <<
	     UpperBodyAngles(6,0) << " " <<
	     UpperBodyAngles(7,0) << " " <<
	     UpperBodyAngles(8,0) << " " <<
	     UpperBodyAngles(9,0) << " " <<
	     UpperBodyAngles(10,0) << " " <<
	     UpperBodyAngles(11,0) << " " <<
	     UpperBodyAngles(12,0) << " " <<
	     UpperBodyAngles(13,0) << " " <<
	     UpperBodyAngles(14,0) << " " <<
	     UpperBodyAngles(15,0) << " " <<
	     UpperBodyAngles(16,0) << " " <<
	     UpperBodyAngles(17,0) << " " <<
	     UpperBodyAngles(18,0) << " " <<
	     UpperBodyAngles(19,0) << " " <<
	     UpperBodyAngles(20,0) << " " <<
	     UpperBodyAngles(21,0) << " " <<
	     UpperBodyAngles(22,0) << " " <<
	     UpperBodyAngles(23,0) << " " <<
	     UpperBodyAngles(24,0) << " " <<
	     UpperBodyAngles(25,0) << " " <<
	     UpperBodyAngles(26,0) << " " <<
	     UpperBodyAngles(27,0) << " " 
	     , "DebugDataUBDisplay.txt" );
    ODEBUG4( dUpperBodyAngles(0,0) << " " <<
	     dUpperBodyAngles(1,0) << " " <<
	     dUpperBodyAngles(2,0) << " " <<
	     dUpperBodyAngles(3,0) << " " <<
	     dUpperBodyAngles(4,0) << " " <<
	     dUpperBodyAngles(5,0) << " " <<
	     dUpperBodyAngles(6,0) << " " <<
	     dUpperBodyAngles(7,0) << " " <<
	     dUpperBodyAngles(8,0) << " " <<
	     dUpperBodyAngles(9,0) << " " <<
	     dUpperBodyAngles(10,0) << " " <<
	     dUpperBodyAngles(11,0) << " " <<
	     dUpperBodyAngles(12,0) << " " <<
	     dUpperBodyAngles(13,0) << " " <<
	     dUpperBodyAngles(14,0) << " " <<
	     dUpperBodyAngles(15,0) << " " <<
	     dUpperBodyAngles(16,0) << " " <<
	     dUpperBodyAngles(17,0) << " " <<
	     dUpperBodyAngles(18,0) << " " <<
	     dUpperBodyAngles(19,0) << " " <<
	     dUpperBodyAngles(20,0) << " " <<
	     dUpperBodyAngles(21,0) << " " <<
	     dUpperBodyAngles(22,0) << " " <<
	     dUpperBodyAngles(23,0) << " " <<
	     dUpperBodyAngles(24,0) << " " <<
	     dUpperBodyAngles(25,0) << " " <<
	     dUpperBodyAngles(26,0) << " " <<
	     dUpperBodyAngles(27,0) << " ", 
	     "DebugDatadUBDisplay.txt" );



	
	
    vector3d WaistPosition,WaistVelocity,WaistAngularVelocity;
    matrix3d Body_Rm3d;

    VNL::Matrix<double> FinalDesiredCOMPose = m_ZMPpcwmbz->GetFinalDesiredCOMPose();			

    WaistPosition.x = aWaistPosition.x[0];
    WaistPosition.y = aWaistPosition.y[0];
    WaistPosition.z = aWaistPosition.z[0]-0.705; //aCOMPosition.hip-0.705;

    WaistVelocity.x = aWaistPosition.x[1];
    WaistVelocity.y = aWaistPosition.y[1];
    WaistVelocity.z = aWaistPosition.z[1];

    // COM Orientation
    for(int li=0;li<3;li++)
      for(int lj=0;lj<3;lj++)
	Body_Rm3d.m[li*3+lj] = FinalDesiredCOMPose(li,lj);

	
    WaistAngularVelocity.x = 0;
    WaistAngularVelocity.y = 0;
    WaistAngularVelocity.z = 0;//(lCOMTheta - m_prev_Zaxis_Angle)/m_SamplingPeriod;
	
    //m_2DMB->ForwardVelocity(WaistPosition,WaistVelocity,WaistAngularVelocity);
    ODEBUG4( WaistPosition.x << " " << WaistPosition.y << " " << WaistPosition.z << " " << 
	     WaistVelocity.x << " " << WaistVelocity.y << " " << WaistVelocity.z << " ", "DebugDataWPDisplay.txt");

    m_2DMB->ForwardVelocity(WaistPosition,Body_Rm3d,WaistVelocity);

    if (m_count>1)
      {
	m_2DMB->GetPandL(P,L);
	
	if (m_count>2)
	  {
	    if (m_count==3)
	      {
		// For the first iteration, we assume that the previous values were the same.
		m_Debug_prev_P = P;
		m_Debug_prev_L = L;
		
	      }
	    
	    // Approximate first order derivative of linear and angular momentum.
	    dP = (P - m_Debug_prev_P)/m_SamplingPeriod;
	    dL = (L - m_Debug_prev_L)/m_SamplingPeriod;
	  }
      }

    m_Debug_prev_P = P;
    m_Debug_prev_L = L;
    // Compute ZMP
    double ZMPz=0.0;
    m_2DMB->CalculateZMP(ZMPmultibody[0],ZMPmultibody[1],
			 dP, dL, ZMPz);
    ODEBUG4( m_count << " " << ZMPmultibody[0] << " " << ZMPmultibody[1] << " " 
	     << P.x << " " << P.y << " " << P.z << " " 
	     << L.x << " " << L.y << " " << P.z << " " 
	     << dP.x << " " << dP.y << " " << dP.z <<" " 
	     << dL.x << " " << dL.y << " " << dL.z ,"DebugDataZMPMB1Display.txt");
    

    vector3d _2DMBCoM = m_2DMB->getPositionCoM();


    ofstream DebugFileLong, DebugFileUpperBody;
    DebugFileLong.open("DebugDataLong.txt",ofstream::app);
    DebugFileUpperBody.open("UpperBodyAngles.txt",ofstream::app);
    if (m_FirstPrint)
      {   
	DebugFileLong << "time" << "\t" 
		      << "ZMPdesiredX"<< "\t" 
		      << "ZMPdesiredY"<< "\t" 
		      << "ZMPMultiBodyX"<< "\t" 
		      << "ZMPMultiBodyY"<< "\t" 
	              << "COMrecomputedX" << "\t"
	              << "COMrecomputedY" << "\t"
		      << "COMPositionX"<< "\t" 
		      << "COMPositionY"<< "\t" 
		      << "COMPositionZ"<< "\t" 	
		      << "COMVelocityX"<< "\t" 
		      << "COMVelocityY"<< "\t" 
		      << "COMVelocityZ"<< "\t" 
		      << "COMOrientation"<< "\t"
		      << "LeftFootPositionsX"<< "\t" 
		      << "LeftFootPositionsY"<< "\t" 
		      << "LeftFootPositionsZ"<< "\t" 
		      << "RightFootPositionsX" << "\t" 
		      << "RightFootPositionsY" << "\t" 
		      << "RightFootPositionsZ" << "\t";
	// 20 lines

	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << "LeftLegAngleJoint:" << i << "\t"; 
	    DebugFileLong << "LeftLegAngleRealJoint:" << i << "\t"; 
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << "RightLegAngleJoint:" << i << "\t"; 
	    DebugFileLong << "RightLegAngleRealJoint:" << i << "\t"; 
	  }
	DebugFileLong << "UpperbodyAngleYaw" << "\t"; 
	// 45 = 20 + 25 lines
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << "LeftArmAngleJoint:" << i << "\t"; 
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << "RightArmAngleJoint:" << i << "\t" ;
	  }
				
	// 57 = 45 + 12 lines
	// angular velocities
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << "LeftLegAngularVelocityJoint:" << i << "\t"; 
	    DebugFileLong << "LeftLegAngularVelocityRealJoint:" << i << "\t"; 
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << "RightLegAngularVelocityJoint:" << i << "\t"; 
	    DebugFileLong << "RightLegAngularVelocityRealJoint:" << i << "\t"; 
	  }
	DebugFileLong << "UpperbodyAngularVelocityYaw" << "\t"; 
	// 82 = 57 + 25 lines
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << "LeftArmAngularVelocityJoint:" << i << "\t"; 
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << "RightArmAngularVelocityJoint:" << i << "\t" ;
	  }
	// angular accelerations
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << "LeftLegAngularAccelerationJoint:" << i << "\t" ;
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << "RightLegAngularAccelerationJoint:" << i << "\t"; 
	  }
	DebugFileLong << "UpperbodyAngularAccelerationYaw" << "\t" ;
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << "LeftArmAngularAccelerationJoint:" << i << "\t";
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << "RightArmAngularAccelerationJoint:" << i << "\t"; 
	  }
	DebugFileLong << "WaistPositionX" << "\t" 
		      << "WaistPositionY" << "\t" 
		      << "WaistPositionZ" << "\t" 
		      << "WaistOrientation0" << "\t" 
		      << "WaistOrientation1" << "\t" 
		      << "WaistOrientation2" << "\t" 
		      << "RightFootPositionsOmega" << "\t"
		      << "LeftFootPositionsOmega" << "\t"
		      << endl;


	DebugFileUpperBody << "Chest_Joint0"<< "\t" 
			   << "Chest_Joint1"<< "\t" 
			   << "Head_Joint0"<< "\t" 
			   << "Head_Joint1"<< "\t" 
			   << "Rarm_Joint0"<< "\t" 
			   << "Rarm_Joint1"<< "\t" 
			   << "Rarm_Joint2"<< "\t" 
			   << "Rarm_Joint3"<< "\t" 
			   << "Rarm_Joint4"<< "\t" 
			   << "Rarm_Joint5"<< "\t" 
			   << "Rarm_Joint6"<< "\t" 
			   << "Larm_Joint0"<< "\t"
			   << "Larm_Joint1"<< "\t"
			   << "Larm_Joint2"<< "\t"
			   << "Larm_Joint3"<< "\t"
			   << "Larm_Joint4"<< "\t"
			   << "Larm_Joint5"<< "\t"
			   << "Larm_Joint6"<< "\t"
			   << "Rhand_Joint0"<< "\t" 
			   << "Rhand_Joint1"<< "\t" 
			   << "Rhand_Joint2"<< "\t" 
			   << "Rhand_Joint3"<< "\t" 
			   << "Rhand_Joint4"<< "\t" 
			   << "Lhand_Joint0"<< "\t"
			   << "Lhand_Joint1"<< "\t"
			   << "Lhand_Joint2"<< "\t"
			   << "Lhand_Joint3"<< "\t"
			   << "Lhand_Joint4"<< "\t"
			   << endl;
				
				
	m_FirstPrint = false;
      }
    else 
      {
	DebugFileLong << m_count *m_SamplingPeriod << "\t" 
		      << m_ZMPPositions[0].px << "\t" 
		      << m_ZMPPositions[0].py << "\t" 
		      << ZMPmultibody[0] << "\t" 
		      << ZMPmultibody[1] << "\t" 
	              << _2DMBCoM.x << "\t" 
		      << _2DMBCoM.y << "\t" 
		      << FinalDesiredCOMPose(0,3) <<  "\t"  
		      << FinalDesiredCOMPose(1,3) <<  "\t"  
		      << FinalDesiredCOMPose(2,3) <<  "\t"  	
		      << WaistVelocity.x <<  "\t"  
		      << WaistVelocity.y <<  "\t"  
		      << WaistVelocity.z <<  "\t"  
		      << aWaistPosition.theta <<  "\t" 
		      << m_LeftFootPositions[0].x <<  "\t"  
		      << m_LeftFootPositions[0].y <<  "\t"  
		      << m_LeftFootPositions[0].z <<  "\t"  
		      << m_RightFootPositions[0].x <<  "\t" 
		      << m_RightFootPositions[0].y <<  "\t"  
		      << m_RightFootPositions[0].z <<  "\t";  
	// 20 lines angles
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << ql(i,0)*180.0/M_PI<<  "\t"  ;
	    //DebugFileLong << qlRefState(i,0)*180.0/M_PI<<  "\t"  ;
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << qr(i,0)*180.0/M_PI<<  "\t" ; 
	    //DebugFileLong << qrRefState(i,0)*180.0/M_PI<<  "\t"  ;
	  }
	DebugFileLong << qWaistYaw*180.0/M_PI <<  "\t" ; 
	// 35 lines = 20 +15
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << UpperBodyAngles(11+i,0)*180.0/M_PI<<  "\t" ;
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << UpperBodyAngles(4+i,0)*180.0/M_PI <<  "\t" ; 
	  }
				
	// 47 lines = 35 + 12 
	// angular velocities
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << dql(i,0)*180.0/M_PI<<  "\t" ;
	    DebugFileLong << dqlRefState(i,0)*180.0/M_PI<<  "\t" ;
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << dqr(i,0)*180.0/M_PI<<  "\t"  ;
	    DebugFileLong << dqrRefState(i,0)*180.0/M_PI<<  "\t" ;
	  }

	DebugFileLong << dqWaistYaw*180.0/M_PI <<  "\t" ; 
	// 73 lines = 47 + 25
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << dUpperBodyAngles(11+i,0)*180.0/M_PI<<  "\t" ; 
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << dUpperBodyAngles(4+i,0)*180.0/M_PI<<  "\t" ;
	  }
	// angular accelerations
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << ddql(i,0)*180.0/M_PI <<  "\t" ; 
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << ddqr(i,0)*180.0/M_PI <<  "\t" ; 
	  }
	DebugFileLong << ddqWaistYaw*180.0/M_PI <<  "\t" ; 
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << ddqal(i,0)*180.0/M_PI <<  "\t"  ;
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << ddqar(i,0)*180.0/M_PI <<  "\t"  ;
	  }
	// 110 lines = 73 + 37 
	DebugFileLong << m_LeftFootPositions[0].omega <<  "\t"  
		      << m_RightFootPositions[0].omega <<  "\t" 
		      << endl;
	
	for(unsigned int j=0;j<28;j++) 
	  {
	    DebugFileUpperBody << UpperBodyAngles(j,0) << "\t"  ;
	  }
	DebugFileUpperBody << endl;
      }
    DebugFileLong.close();
    DebugFileUpperBody.close();
			 


  }

  void PatternGeneratorInterface::m_SetTimeDistrParameters(istringstream &strm)
  {
    ODEBUG("SetTimeDistrParameters");
    while(!strm.eof())
      {
	if (!strm.eof())
	  {
	    strm >> m_TimeDistrFactor[0];
	  }
	else break;
	if (!strm.eof())
	  {
	    strm >> m_TimeDistrFactor[1];
	  
	  }
	else break;
	if (!strm.eof())
	  {
	    strm >> m_TimeDistrFactor[2];

	  }
	else break;
	if (!strm.eof())
	  {
	    strm >> m_TimeDistrFactor[3];
	  
	  }
	else break;
      }
  }
  
  void PatternGeneratorInterface::SetCurrentJointValues(VNL::Matrix<double> lCurrentJointValues)
  {
    if(lCurrentJointValues.Rows()!=m_CurrentJointValues.size())
      m_CurrentJointValues.resize(lCurrentJointValues.Rows());
    
    for(unsigned int i=0;i<lCurrentJointValues.Rows();i++)
      m_CurrentJointValues[i] = lCurrentJointValues[i][0];

  }

  void PatternGeneratorInterface::m_FinishOnTheLastCorrectSupportFoot(istringstream &strm)
  {
    m_StepStackHandler->FinishOnTheLastCorrectSupportFoot();
    
  }

  void PatternGeneratorInterface::m_FinishAndRealizeStepSequence(istringstream &strm)
  {
    int Synchronize=1;
    
    while(!strm.eof())
      {
	if (!strm.eof())
	  strm >> Synchronize;
	else 
	  break;
      }
    FinishAndRealizeStepSequence();
  }

  void PatternGeneratorInterface::m_CreateArcInStepStack(istringstream &strm)
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
    
    
    m_StepStackHandler->CreateArcInStepStack(x,y,R,arc_deg,SupportFoot);
  }

  void PatternGeneratorInterface::m_CreateArcCenteredInStepStack(istringstream &strm)
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
    m_StepStackHandler->CreateArcCenteredInStepStack(R,arc_deg,SupportFoot);
    ODEBUG4("m_CreateArcCenteredInStepStack 3", "DebugData.txt");
    
  }
  
  int PatternGeneratorInterface::GetWalkMode()
  {
    return m_StepStackHandler->GetWalkMode();
  }

  void PatternGeneratorInterface::m_PrepareForSupportFoot(istringstream & strm)
  {
    if (m_StepStackHandler!=0)
      m_StepStackHandler->m_PrepareForSupportFoot(strm);
  }

  void PatternGeneratorInterface::m_PartialStepSequence(istringstream &strm)
  {
    if (m_StepStackHandler!=0)
      m_StepStackHandler->m_PartialStepSequence(strm);
  }

  void PatternGeneratorInterface::GetLegJointVelocity(VNL::Matrix<double> & dqr,
						      VNL::Matrix<double> & dql)
  {
    for(int i=0;i<6;i++)
      {
	dqr(i,0) = m_dqr(i,0);
	dql(i,0) = m_dql(i,0);
      }
  }
  
  void PatternGeneratorInterface::ExpandCOMAndUpperBodyPositionsQueues(int aNumber)
  {
    COMPosition aCOMPos;
    KWNode anUpperBodyPos;

    for(int i=0;i<aNumber;i++)
      {
	// Add COM value set at a default value.
	aCOMPos.z[0] = m_PC->GetHeightOfCoM();
	aCOMPos.z[1] = 0.0;
	aCOMPos.z[2] = 0.0;

	aCOMPos.omega = 0.0;
	aCOMPos.hip = 0.0;
	m_COMBuffer.push_back(aCOMPos);

	// Add UpperBody Position set at a default value.
       
      }
    if (m_StepStackHandler->GetWalkMode()!=3)
      {
	
	for(int i=0;i<aNumber;i++)
	  {
	    anUpperBodyPos.Joints.resize(28);

	    for(unsigned int j=0;j<28;j++)
	      anUpperBodyPos.Joints[j] = m_CurrentJointValues[j+12];

	    m_UpperBodyPositionsBuffer.push_back(anUpperBodyPos);
	  }

      }
  }



  void PatternGeneratorInterface::AddOnLineStep(double X, double Y, double Theta)
  {
    m_NewStep = true;
    m_NewStepX = X;
    m_NewStepY = Y;
    m_NewTheta = Theta;
  }

  void PatternGeneratorInterface::UpdateAbsolutePosition(bool UpdateAbsMotionOrNot)
  {
    // Compute relative, absolution position and speed.
    m_WaistRelativePos[3][0] = 0.0;
    m_WaistRelativePos[3][1] = 0.0;
    m_WaistRelativePos[3][2] = 0.0;
    m_WaistRelativePos[3][3] = 1.0;
  
    double thetarad = m_CurrentWaistState.theta*M_PI/180.0;
    double c = cos(thetarad);
    double s = sin(thetarad);
  
    m_WaistRelativePos[0][0] = c; m_WaistRelativePos[0][1]=-s; m_WaistRelativePos[0][2] = 0;
    m_WaistRelativePos[1][0] = s; m_WaistRelativePos[1][1]= c; m_WaistRelativePos[1][2] = 0;
    m_WaistRelativePos[2][0] = 0; m_WaistRelativePos[2][1]= 0; m_WaistRelativePos[2][2] = 1;
    m_WaistRelativePos[3][3] = 1;
  
    VNL::Matrix<double> RelativeLinearVelocity(4,1);
    RelativeLinearVelocity[0][0] =  m_CurrentWaistState.x[1];
    RelativeLinearVelocity[1][0] =  m_CurrentWaistState.y[1];
    RelativeLinearVelocity[2][0] =  m_CurrentWaistState.z[0];
    RelativeLinearVelocity[3][0] =  1.0;
  
    VNL::Matrix<double> RelativeLinearAcc(4,1);
    RelativeLinearAcc[0][0] =  m_CurrentWaistState.x[2];
    RelativeLinearAcc[1][0] =  m_CurrentWaistState.y[2];
    RelativeLinearAcc[2][0] =  0.0;
    RelativeLinearAcc[3][0] =  1.0;
  
    m_AbsLinearVelocity = m_MotionAbsOrientation * RelativeLinearVelocity;
    m_AbsLinearAcc = m_MotionAbsOrientation * RelativeLinearAcc;
  
    m_WaistRelativePos[0][3] = m_CurrentWaistState.x[0];
    m_WaistRelativePos[1][3] = m_CurrentWaistState.y[0];
    m_WaistRelativePos[2][3] = m_CurrentWaistState.z[0];
  
    VNL::Matrix<double> prevWaistAbsPos = m_WaistAbsPos;
  
    m_WaistAbsPos = m_MotionAbsPos * m_WaistRelativePos;
  
    /*
      cout << "Motion Abs Pos " << m_MotionAbsPos<< endl;
      cout << "Waist Relative Pos " << m_WaistRelativePos<< endl;
      cout << "Waist Abs Pos " << m_WaistAbsPos << endl;
    */
    m_AbsAngularVelocity[0][0] = 0.0;
    m_AbsAngularVelocity[1][0] = 0.0;
  
    if (m_count!=0)
      m_AbsAngularVelocity[2][0] = (m_AbsMotionTheta + thetarad - m_AbsTheta )/m_dt;
    else
      m_AbsAngularVelocity[2][0] = 0.0;
  
    m_AbsAngularVelocity[3][0] = 1.0;
    //      cout << "m_AbsAngularVelocity " << m_AbsAngularVelocity<< endl;
    m_AbsTheta = fmod(m_AbsMotionTheta + thetarad,2*M_PI);

    if (UpdateAbsMotionOrNot)
      {
	m_MotionAbsPos = m_WaistAbsPos;
	// The position is supposed at the ground level
	m_MotionAbsPos[2][3] = 0.0;
	m_AbsMotionTheta = m_AbsTheta;
      }
  
#if 0
    aof_WaistAbsPos.open("/tmp/WaistOrientation.dat",ofstream::app);
    TransformQuaternion aTQ;
    getWaistPositionAndOrientation(aTQ);
    aof_WaistAbsPos << aTQ.qw << " "
		    << aTQ.qx << " "
		    << aTQ.qy << " "
		    << aTQ.qz << " "
		    << m_AbsTheta << " "
		    << m_CurrentWaistState.theta << " " 
		    <<endl;
    aof_WaistAbsPos.close(); 
  
  
    aof_WaistAbsPos.open("/tmp/WaistAbsPositionPG.dat",ofstream::app);
    aof_WaistAbsPos << m_WaistAbsPos[0][3] << " "
		    << m_WaistAbsPos[1][3] << " "
		    << m_WaistAbsPos[2][3] << " "
		    << m_AbsLinearVelocity[0][0] << " "
		    << m_AbsLinearVelocity[1][0] << " " 
		    << (m_WaistAbsPos[0][3] - prevWaistAbsPos[0][3])/m_dt << " " 
		    << (m_WaistAbsPos[1][3] - prevWaistAbsPos[1][3])/m_odt << " " 
		    << m_AbsAngularVelocity[2][0] << " "
		    <<endl;
    aof_WaistAbsPos.close(); 
  
    if (m_count%6==0)
      {
	aof_WaistAbsPos.open("/tmp/SensorsSimu.dat",ofstream::app);
	for(int li=0;li<3;li++)
	  aof_WaistAbsPos << rs->rate[0][li] << " ";
	for(int li=0;li<3;li++)
	  aof_WaistAbsPos << rs->accel[0][li] << " ";
	aof_WaistAbsPos <<  endl;
	aof_WaistAbsPos.close(); 
      
	aof_WaistAbsPos.open("/tmp/Sensors2.dat",ofstream::app);
	for(int li=0;li<2;li++)
	  aof_WaistAbsPos << "0.0 ";
	aof_WaistAbsPos << m_AbsAngularVelocity[2][0] << " ";
	for(int li=0;li<3;li++)
	  aof_WaistAbsPos << "0.0 ";
	aof_WaistAbsPos <<  endl;
	aof_WaistAbsPos.close(); 
      
	aof_WaistAbsPos.open("/tmp/Control.dat",ofstream::app);
	aof_WaistAbsPos << m_AbsLinearAcc[0][0] << " "
			<< m_AbsLinearAcc[1][0] << " "
			<< "0.0 " << endl;
	aof_WaistAbsPos.close(); 
      
	aof_WaistAbsPos.open("/tmp/WaistVelocity.dat",ofstream::app);
	aof_WaistAbsPos << m_AbsLinearVelocity[0][0]<< " "
			<< m_AbsLinearVelocity[1][0]<< endl;
	aof_WaistAbsPos.close(); 
      
	aof_WaistAbsPos.open("/tmp/WaistOrientation.dat",ofstream::app);
	TransformQuaternion aTQ;
	getWaistPositionAndOrientation(aTQ);
	aof_WaistAbsPos << aTQ.qw << " " 
			<< aTQ.qx << " "
			<< aTQ.qy << " "
			<< aTQ.qz << endl;
	aof_WaistAbsPos.close(); 
      
      
      }
#endif

  }

  void PatternGeneratorInterface::getWaistPositionMatrix(VNL::Matrix<double> &lWaistAbsPos)
  {
    lWaistAbsPos = m_WaistAbsPos;
  }

  void PatternGeneratorInterface::getWaistPositionAndOrientation(double aTQ[7], double &Orientation)
  {
    // Position
    aTQ[0] = m_WaistAbsPos[0][3];
    aTQ[1] = m_WaistAbsPos[1][3];
    aTQ[2] = m_WaistAbsPos[2][3];
    
    
    // Carefull : Extremly specific to the pattern generator.
    double cx,cy,cz, sx,sy,sz;
    cx = 0; cy = 0; cz = cos(0.5*m_AbsTheta);
    sx = 0; sy = 0; sz = sin(0.5*m_AbsTheta);
    aTQ[3] = 0;
    aTQ[4] = 0;
    aTQ[5] = sz;
    aTQ[6] = cz;
    Orientation = m_AbsTheta;
  }

  void PatternGeneratorInterface::setWaistPositionAndOrientation(double aTQ[7])
  {
    // Position
    m_WaistAbsPos[0][3] = aTQ[0];
    m_WaistAbsPos[1][3] = aTQ[1];
    m_WaistAbsPos[2][3] = aTQ[2];
    double _x = aTQ[3];
    double _y = aTQ[4];
    double _z = aTQ[5];
    double _r = aTQ[6];
    
    
    double x2 = _x * _x;
    double y2 = _y * _y;
    double z2 = _z * _z;
    double r2 = _r * _r;
    m_WaistAbsPos[0][0] = r2 + x2 - y2 - z2;         // fill diagonal terms
    m_WaistAbsPos[1][1] = r2 - x2 + y2 - z2;
    m_WaistAbsPos[2][2] = r2 - x2 - y2 + z2;
    double xy = _x * _y;
    double yz = _y * _z;
    double zx = _z * _x;
    double rx = _r * _x;
    double ry = _r * _y;
    double rz = _r * _z;
    m_WaistAbsPos[0][1] = 2 * (xy - rz);             // fill off diagonal terms
    m_WaistAbsPos[0][2] = 2 * (zx + ry);
    m_WaistAbsPos[1][0] = 2 * (xy + rz);
    m_WaistAbsPos[1][2] = 2 * (yz - rx);
    m_WaistAbsPos[2][0] = 2 * (zx - ry);
    m_WaistAbsPos[2][1] = 2 * (yz + rx);
    
  }

  void PatternGeneratorInterface::getWaistVelocity(double & dx,
						   double & dy,
						   double & omega)
  {
    dx = m_AbsLinearVelocity[0][0];
    dy = m_AbsLinearVelocity[1][0];
    omega = m_AbsAngularVelocity[2][0];
  }
}



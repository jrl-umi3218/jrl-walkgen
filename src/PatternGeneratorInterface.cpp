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
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); \
    DebugFile << "PGI: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1 
#else
#define RESETDEBUG4(y) 
#define ODEBUG4(x,y)
#endif

#define ODEBUG6(x,y)

namespace PatternGeneratorJRL {

  PatternGeneratorInterface::PatternGeneratorInterface(istringstream &strm)    
  {

    ODEBUG4("Step 0","DebugPGI.txt");
	
    // Initialization of the parameters directory and files.
    string PCParametersFileName;
    strm >> PCParametersFileName;

    string HumanoidVRMLFileDirectory;
    strm >> HumanoidVRMLFileDirectory;

    string  HumanoidVRMLFileName;
    strm >>  HumanoidVRMLFileName;

    string HumanoidSpecificitiesFileName;
    strm >> HumanoidSpecificitiesFileName;
     
    string LinkJointRank;
    strm >> LinkJointRank;

    // Initialization for debugging.
    RESETDEBUG4("DebugDataWPDisplay.txt");

    RESETDEBUG4("DebugDataqrDisplay.txt");
    RESETDEBUG4("DebugDataqlDisplay.txt");
    RESETDEBUG4("DebugDataZMPMB1Display.txt");
    
    RESETDEBUG4("DebugDataCOMForHeuristic.txt");
    RESETDEBUG4("DebugDataqArmsHeuristic.txt");
	
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

    RESETDEBUG4("UpperBodyAngles.txt");
    RESETDEBUG4("DebugZMPFinale.txt");
    
    RESETDEBUG4("DebugGMFKW.dat");
    RESETDEBUG4("DebugDataCOM.txt");

    RESETDEBUG4("DDCC.dat");
    RESETDEBUG4("DDCV.dat");
    RESETDEBUG4("DebugDataWaist.dat");
    // End if Initialization

    m_ObstacleDetected = false;
    
    // Initialization of obstacle parameters informations.	
    m_ObstaclePars.x=1.0;
    m_ObstaclePars.y=0.0;
    m_ObstaclePars.z=0.0;
    m_ObstaclePars.theta=0.0;
    m_ObstaclePars.h=0.05;
    m_ObstaclePars.w=1.0;
    m_ObstaclePars.d=0.05;

    m_TimeDistrFactor.resize(4);
    m_TimeDistrFactor[0]=2.0;
    m_TimeDistrFactor[1]=3.7;
    m_TimeDistrFactor[2]=1.0;
    m_TimeDistrFactor[3]=3.0;

    m_DeltaFeasibilityLimit =0.0;
    
    ODEBUG("Beginning of Object Instanciation ");
    // Initialization of the fundamental objects.
    ObjectsInstanciation(HumanoidSpecificitiesFileName);
    ODEBUG("End of Object Instanciation ");
    // Initialize their relationships.
	   
    ODEBUG("Beginning of Inter Object Relation Initialization " );
    InterObjectRelationInitialization(PCParametersFileName,
				      HumanoidVRMLFileDirectory,
				      HumanoidVRMLFileName,
				      LinkJointRank);
    ODEBUG("End of Inter Object Relation Initialization ");
    // Initialization of the strategy for ZMP ref trajectory generation.
    m_BoolPBWAlgo = 0;

    // Initialize (if needed) debugging actions.
    m_dt = 0.005;
    m_DOF = m_DMB->numberDof();
    m_UpperBodyJoints = m_HumanoidDynamicRobot->
      getHumanoidSpecificities()->GetUpperBodyJoints();
    m_NbOfUpperBodyJoints = m_UpperBodyJoints.size();

    m_SamplingPeriod = m_PC->SamplingPeriod();
    m_PreviewControlTime = m_PC->PreviewControlTime();
    m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);
    
		

    /* For debug purposes. */
    MAL_VECTOR_RESIZE(m_Debug_prev_qr,6);
    MAL_VECTOR_RESIZE(m_Debug_prev_dqr,6);
    MAL_VECTOR_RESIZE(m_Debug_prev_ql,6);
    MAL_VECTOR_RESIZE(m_Debug_prev_dql,6);

    MAL_VECTOR_RESIZE(m_Debug_prev_qr_RefState,6);
    MAL_VECTOR_RESIZE(m_Debug_prev_ql_RefState,6);

    MAL_VECTOR_RESIZE(m_Debug_prev_UpperBodyAngles,28);
    
    m_ZMPShift.resize(4);
    m_ZMPShift[0] = 0.02;
    m_ZMPShift[1] = 0.07;
    m_ZMPShift[2] = 0.02;
    m_ZMPShift[3] = 0.02;



  
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


    ODEBUG4("Step 2","DebugPGI.txt");	
	
    //  string PCParameters="/home/stasse/OpenHRP/PatternGeneratorJRL/src/PreviewControlParameters.ini";

    ofstream DebugFile;
    ofstream DebugFileLong;
    ofstream DebugFileUpperBody;
    m_count = 0;
	
    ODEBUG4("Step 3","DebugPGI.txt");	

    ODEBUG4("Step 4","DebugPGI.txt");
    
    m_TSsupport = 0.78;
    m_TDsupport = 0.02;
	
    m_FirstPrint = true;
    m_FirstRead = true;
    ODEBUG4("Step 5","DebugPGI.txt");

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

  void PatternGeneratorInterface::ObjectsInstanciation(string & HumanoidSpecificitiesFileName)
  {
    // Create fundamental objects to make the WPG runs.

    // INFO: This where you should instanciate your own 
    // INFO: implementation of CjrlDynamicRobot.
    m_DMB = new DynamicMultiBody();
    m_DMB->setComputeZMP(true);
    m_DMB->setComputeBackwardDynamics(false);
    // INFO: This where you should instanciate your own
    // INFO: implementation of CjrlHumanoidDynamicRobot
    m_HumanoidDynamicRobot= new HumanoidDynamicMultiBody(m_DMB,HumanoidSpecificitiesFileName);

    // INFO: This where you should instanciate your own
    // INFO: object for Com and Foot realization.
    // INFO: The default one is based on a geometrical approach.
    m_ComAndFootRealization = new ComAndFootRealizationByGeometry(this);

    // ZMP reference and Foot trajectory planner.
    m_ZMPD = new ZMPDiscretization("",m_HumanoidDynamicRobot->getHumanoidSpecificities());

    // Preview control for a 3D Linear inverse pendulum
    m_PC = new PreviewControl();

    // Object to generate Motion from KineoWorks.
    m_GMFKW = new GenerateMotionFromKineoWorks();

    // Object to have a Dynamic multibody robot model.
    // for the second preview loop.

    // Object to investiguate the result of the second preview loop.
    m_2DMB = new DynamicMultiBody();
    m_2DMB->setComputeZMP(true);
    m_2DMB->setComputeBackwardDynamics(false);
    
    // INFO: This where you should instanciate your own
    // INFO: implementation of CjrlHumanoidDynamicRobot
    m_2HumanoidDynamicRobot= new HumanoidDynamicMultiBody(m_2DMB,HumanoidSpecificitiesFileName);

    // Stack of steps handler.
    m_StepStackHandler = new StepStackHandler();

    // Stepping over planner.
    m_StOvPl = new StepOverPlanner(m_ObstaclePars,
				   m_HumanoidDynamicRobot->getHumanoidSpecificities());

    // The object to realize the global stage of preview control.
    m_ZMPpcwmbz = new ZMPPreviewControlWithMultiBodyZMP();

    // End of the creation of the fundamental objects.

  }

  void PatternGeneratorInterface::InterObjectRelationInitialization(string & PCParametersFileName,
								    string & HumanoidVRMLFileDirectory,
								    string & HumanoidVRMLFileName,
								    string & HumanoidLinkJointRank)
  {
    // Initialize the Preview Control.
    m_PC->ReadPrecomputedFile(PCParametersFileName);
    m_Zc = m_PC->GetHeightOfCoM();

    // Initialize the Preview Control general object.
    m_ZMPpcwmbz->SetPreviewControl(m_PC);
    m_ZMPpcwmbz->setHumanoidDynamicRobot(m_HumanoidDynamicRobot);
    m_ZMPpcwmbz->setComAndFootRealization(m_ComAndFootRealization);

    // The motion generator based on a Kineoworks pathway is given here.
    m_GMFKW->SetPreviewControl(m_PC);

    // Read the robot VRML file model.
    m_DMB->parserVRML(HumanoidVRMLFileDirectory,
		      HumanoidVRMLFileName,
		      HumanoidLinkJointRank.c_str());

    m_2DMB->parserVRML(HumanoidVRMLFileDirectory,
		       HumanoidVRMLFileName,
		       HumanoidLinkJointRank.c_str());
    
    m_DMB->SetTimeStep(m_PC->SamplingPeriod());
    m_2DMB->SetTimeStep(m_PC->SamplingPeriod());

    // The link between specific Humanoid information 
    // and joint of the dynamic multi robot
    // can be done only with the VRML file has been read (obvious isn't it ? ) 
    
    m_HumanoidDynamicRobot->LinkBetweenJointsAndEndEffectorSemantic();
    m_2HumanoidDynamicRobot->LinkBetweenJointsAndEndEffectorSemantic();

    m_ComAndFootRealization->setHumanoidDynamicRobot(m_HumanoidDynamicRobot);

    m_ComAndFootRealization->SetHeightOfTheCoM(m_PC->GetHeightOfCoM());

    m_ComAndFootRealization->setSamplingPeriod(m_PC->SamplingPeriod());

    m_ComAndFootRealization->SetStepStackHandler(m_StepStackHandler);

    m_ComAndFootRealization->Initialization();

    m_StOvPl->SetPreviewControl(m_PC);
    m_StOvPl->SetDynamicMultiBodyModel(m_DMB);
    m_StOvPl->SetZMPDiscretization(m_ZMPD);


    m_StepStackHandler->SetStepOverPlanner(m_StOvPl);	
    m_StepStackHandler->SetWalkMode(0);
    // End of the initialization of the fundamental object.
    
  }

  PatternGeneratorInterface::~PatternGeneratorInterface()
  {

    
    ODEBUG4("Destructor: Start","DebugPGI.txt");
    
    if (m_StOvPl!=0)
      delete m_StOvPl;
    ODEBUG4("Destructor: did m_StOvPl","DebugPGI.txt");
    
    if (m_StepStackHandler!=0)
      delete m_StepStackHandler;
    ODEBUG4("Destructor: did m_StepStackHandler","DebugPGI.txt");

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
	    // TODO : forward this to ComAndFootRealization.
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
	  ODEBUG("Juste before the reading of the step sequence ");
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
    ODEBUG("Just before starting to Finish and RealizeStepSequence()");
    FinishAndRealizeStepSequence();
  }
  
  void PatternGeneratorInterface::EvaluateStartingCOM(MAL_VECTOR(  & Configuration,double),
						      MAL_S3_VECTOR(  & lStartingCOMPosition,double))
  {
    m_HumanoidDynamicRobot->currentConfiguration(Configuration);
    m_DMB->setComputeCoM(true);
    m_DMB->computeForwardKinematics();
    lStartingCOMPosition = m_DMB->positionCenterOfMass();
  }


  void PatternGeneratorInterface::CommonInitializationOfWalking(MAL_S3_VECTOR(  & lStartingCOMPosition,double),
								MAL_VECTOR(  & BodyAnglesIni,double),
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

    lCurrentJointValues.resize(m_CurrentActuatedJointValues.size());
  
    for(unsigned int i=0;i<m_CurrentActuatedJointValues.size();i++)
      lCurrentJointValues[i] = m_CurrentActuatedJointValues[i];

    m_DOF = m_CurrentActuatedJointValues.size();
    MAL_VECTOR_RESIZE(BodyAnglesIni,m_CurrentActuatedJointValues.size());
        
    for(int j=0; j<m_DOF;j++)
      {
	BodyAnglesIni(j) = lCurrentJointValues[j];
      }
    
    // Copy the relative foot position from the stack handler to here.
    m_StepStackHandler->CopyRelativeFootPosition(lRelativeFootPositions,ClearStepStackHandler);

    // Initialize consequently the ComAndFoot Realization object.
    m_ZMPpcwmbz->EvaluateStartingCoM(BodyAnglesIni,lStartingCOMPosition,
				     InitLeftFootAbsPos, InitRightFootAbsPos);
    ODEBUG("StartingCOMPosition: " << lStartingCOMPosition);
    // We also initialize the iteration number inside DMB.
    m_DMB->ResetIterationNumber();
    
    ODEBUG( "CommonInitializationOfWalking " << BodyAnglesIni );
    
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
    MAL_S3_VECTOR( lStartingCOMPosition,double);	
    MAL_VECTOR( BodyAnglesIni,double);

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

    ODEBUG("StartOnLineStepSequencing - 3 " 
	   << lStartingCOMPosition << " "
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

	m_COMBuffer[i].pitch = 0.0;
	m_COMBuffer[i].roll = 0.0;
      
      }
    
    // Initialization of the upper body motion.
    m_UpperBodyPositionsBuffer.resize(m_ZMPPositions.size());
    for(unsigned int i=0;i<m_UpperBodyPositionsBuffer.size();i++)
      {
	m_UpperBodyPositionsBuffer[i].Joints.resize(m_UpperBodyJoints.size());
	
	if (i==0)
	  {
	    for(unsigned int j=0;j<m_NbOfUpperBodyJoints;j++)
	      m_UpperBodyPositionsBuffer[i].Joints[j] = 
		lCurrentJointValues[m_UpperBodyJoints[j]];
	  }
	// Initialize the upper body motion to the current stored value.
	for(unsigned int j=0;j<m_NbOfUpperBodyJoints;j++)
	  m_UpperBodyPositionsBuffer[i].Joints[j] = m_UpperBodyPositionsBuffer[0].Joints[j];
      }
    
    // Initialization of the first preview.
    for(int j=0; j<m_DOF;j++)
      {
	BodyAnglesIni(j) = lCurrentJointValues[j];
	ODEBUG4(BodyAnglesIni(j),"DebugDataOnLine.txt");
      }

    m_ZMPpcwmbz->Setup(m_ZMPPositions,
		       m_COMBuffer,
		       m_LeftFootPositions,
		       m_RightFootPositions);
    
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
    ODEBUG("PGI-Start");
    MAL_S3_VECTOR(lStartingCOMPosition,double);	
    MAL_VECTOR( BodyAnglesIni,double);
    FootAbsolutePosition InitLeftFootAbsPos, InitRightFootAbsPos;
    struct timeval begin, end, time1, time2, time3, time4, time5, time6;
  
    gettimeofday(&begin,0);
  
    ODEBUG6("FinishAndRealizeStepSequence() - 1","DebugGMFKW.dat");

    vector<double> lCurrentJointValues;
    m_ZMPD->SetZMPShift(m_ZMPShift);

    MAL_VECTOR(,double) lCurrentConfiguration;
    lCurrentConfiguration = m_HumanoidDynamicRobot->currentConfiguration();  
    ODEBUG("lCurrent Configuration :" << lCurrentConfiguration );

    deque<RelativeFootPosition> lRelativeFootPositions;
    CommonInitializationOfWalking(lStartingCOMPosition,
				  BodyAnglesIni,
				  InitLeftFootAbsPos, InitRightFootAbsPos,
				  lRelativeFootPositions,lCurrentJointValues,true);
    ODEBUG( "Pass through here " );
    lCurrentConfiguration(0) = 0.0;
    lCurrentConfiguration(1) = 0.0;
    lCurrentConfiguration(2) = 0.0;
    lCurrentConfiguration(3) = 0.0;
    lCurrentConfiguration(4) = 0.0;
    lCurrentConfiguration(5) = 0.0;
    m_HumanoidDynamicRobot->currentConfiguration(lCurrentConfiguration);  
    
    ODEBUG6("Size of lRelativeFootPositions :" << lRelativeFootPositions.size(),"DebugGMFKW.dat");


    ODEBUG("CurrentZMPNeutralPosition: " 
	   << CurrentZMPNeutralPosition[0] 
	   << " "
	   << CurrentZMPNeutralPosition[1] );
    
    // Create the ZMP reference.
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

    ODEBUG("First m_ZMPPositions" << m_ZMPPositions[0].px << " " << m_ZMPPositions[0].py);
    deque<ZMPPosition> aZMPBuffer;

    gettimeofday(&time1,0);
    // Option : Use Wieber06's algorithm to compute a new ZMP 
    // profil. Suppose to preempt the first stage of control.
    deque<ZMPPosition> NewZMPPositions;

    ODEBUG( "COM_Buffer size: " << m_COMBuffer.size());

    
    if (m_BoolPBWAlgo)
      {
	ODEBUG("PBW algo set on");
	ODEBUG("Size of COMBuffer: " << m_COMBuffer.size());
	m_COMBuffer.clear();
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

	m_COMBuffer[i].pitch = 0.0;
	m_COMBuffer[i].roll = 0.0;
      
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
		for(unsigned int j=0;j<m_NbOfUpperBodyJoints;j++)
		  m_UpperBodyPositionsBuffer[i].Joints[j] = 
		    lCurrentJointValues[m_UpperBodyJoints[j]];
	      }
	    // Initialize the upper body motion to the current stored value.
	    for(unsigned int j=0;j<m_NbOfUpperBodyJoints;j++)
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
	BodyAnglesIni(m_ConversionForUpperBodyFromLocalIndexToRobotDOFs[j]) =
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

    if ((m_StepStackHandler->GetWalkMode()==0) ||
	(m_StepStackHandler->GetWalkMode()==4))
      {
	for(unsigned int i=0;i<m_ZMPPositions.size()-m_NL;i++)
	  //	  m_COMBuffer[i].theta = m_ZMPPositions[i+m_NL-1].theta;
	  m_COMBuffer[i].yaw = m_ZMPPositions[i].theta;
      }


    ODEBUG("m_ZMPPositions : " << m_ZMPPositions.size() << endl <<
	    "m_COMBuffer : " << m_COMBuffer.size() << endl);
    if(m_StepStackHandler->GetWalkMode()==2)
      {
	m_StOvPl->TimeDistributeFactor(m_TimeDistrFactor);
	m_StOvPl->PolyPlanner(m_COMBuffer,m_LeftFootPositions,m_RightFootPositions,m_ZMPPositions);
      }

    gettimeofday(&time4,0);
    ODEBUG6("FinishAndRealizeStepSequence() - 8 ","DebugGMFKW.dat");
    // Read NL informations from ZMPRefPositions.
    int localWalkMode = m_StepStackHandler->GetWalkMode();
    if ((localWalkMode==0) ||
	(localWalkMode==3))
      {
	m_ZMPpcwmbz->SetupFirstPhase(m_ZMPPositions,
				     m_COMBuffer,
				     m_LeftFootPositions,
				     m_RightFootPositions);

	gettimeofday(&time5,0);

	MAL_VECTOR(,double) CurrentConfiguration;
	CurrentConfiguration = m_HumanoidDynamicRobot->currentConfiguration();  
	MAL_VECTOR(,double) CurrentVelocity = m_HumanoidDynamicRobot->currentVelocity();
	
	for(unsigned int i=0;i<m_NL;i++)
	  {
	    COMPosition aCOMPosition;
	    
	    ODEBUG("PGI: " << i <<  " " 
		   << m_COMBuffer[i].x[0] << " "
		   << m_COMBuffer[i].y[0] << " "
		   << m_COMBuffer[i].z[0]);
	    
	    
	    m_ZMPpcwmbz->SetupIterativePhase(m_ZMPPositions,
					     m_COMBuffer,
					     m_LeftFootPositions,
					     m_RightFootPositions,
					     CurrentConfiguration,
					     CurrentVelocity,
					     i);
	    
	    // Take the new COM computed by the first stage of Preview control.
	    if (m_BoolPBWAlgo == 0)
	      {
		aCOMPosition = m_ZMPpcwmbz->GetLastCOMFromFirstStage();
		
		m_COMBuffer[i+1].x[0] = aCOMPosition.x[0];
		m_COMBuffer[i+1].y[0] = aCOMPosition.y[0];
		m_COMBuffer[i+1].pitch = aCOMPosition.pitch;
		//		m_COMBuffer[i+1].theta = aCOMPosition.theta;
	      }
	    else 
	      aCOMPosition = m_COMBuffer[i];
	    
	  }
	
      }
    else 
      {
	gettimeofday(&time5,0);
	m_ZMPpcwmbz->Setup(m_ZMPPositions,
			   m_COMBuffer,
			   m_LeftFootPositions,
			   m_RightFootPositions);
      }
    
    gettimeofday(&time6,0);
    ODEBUG("FinishAndRealizeStepSequence() - 9 ");
    
    m_count = 0;
    ODEBUG("FinishAndRealizeStepSequence() - 10 ");
    
    m_ShouldBeRunning = true;
    gettimeofday(&end,0);
    ODEBUG(endl << 
	    "Step 1 : "<<  time1.tv_sec-begin.tv_sec + 
	   0.000001 * (time1.tv_usec -begin.tv_usec) << endl << 
	    "Step 2 : "<<  time2.tv_sec-time1.tv_sec + 
	   0.000001 * (time2.tv_usec -time1.tv_usec) << endl << 
		"Step 3 : "<<  time3.tv_sec-time2.tv_sec + 
	   0.000001 * (time3.tv_usec -time2.tv_usec) << endl << 
		"Step 4 : "<<  time4.tv_sec-time3.tv_sec + 
	   0.000001 * (time4.tv_usec -time3.tv_usec) << endl << 
		"Step 5 : "<<  time5.tv_sec-time4.tv_sec + 
	   0.000001 * (time5.tv_usec -time4.tv_usec) << endl << 
		"Step 6 : "<<  time6.tv_sec-time5.tv_sec + 
	   0.000001 * (time6.tv_usec -time5.tv_usec) << endl << 
		"Total time : "<< end.tv_sec-begin.tv_sec + 
	   0.000001 * (end.tv_usec -begin.tv_usec) );
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
    ODEBUG6("Went before DisplayModel and PAth m_ReadFileFromKineoWorks(istringstream &strm)",
	    "DebugGMFKW.dat");
  
    //    m_GMFKW->DisplayModelAndPath();
    ODEBUG6("Fini..","DebugGMFKW.dat");
  }

  int PatternGeneratorInterface::ParseCmd(istringstream &strm)
  {
    string aCmd;
    strm >> aCmd;

    
    if (CallMethod(aCmd,strm))
      {
	ODEBUG("Method " << aCmd << " found and handled.");
	return 0;
      }

    ODEBUG("PGI:ParseCmd: Commande: " << aCmd);
    if (aCmd==":omega")
      m_SetOmega(strm);
    
    else if (aCmd==":stepheight")
      m_SetStepHeight(strm);
    
    else if (aCmd==":singlesupporttime")
      m_SetSingleSupportTime(strm);

    else if (aCmd==":doublesupporttime")
      m_SetDoubleSupportTime(strm);

    else if (aCmd==":LimitsFeasibility")
      m_SetLimitsFeasibility(strm);
    
    else if (aCmd==":ZMPShiftParameters")
      m_SetZMPShiftParameters(strm);

    else if (aCmd==":TimeDistributionParameters")
      m_SetTimeDistrParameters(strm);

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
	m_ZMPpcwmbz->SetAlgorithmForZMPAndCoMTrajectoryGeneration(ZMPPreviewControlWithMultiBodyZMP::
								  ZMPCOM_TRAJECTORY_WIEBER);
      }
    else if (ZMPTrajAlgo=="Kajita")
      {
	m_BoolPBWAlgo=0;
	m_ZMPpcwmbz->SetAlgorithmForZMPAndCoMTrajectoryGeneration(ZMPPreviewControlWithMultiBodyZMP::
								  ZMPCOM_TRAJECTORY_KAJITA);

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
	cout << "Constraint On X: " << m_ConstraintOnX 
	     << " Constraint On Y: " << m_ConstraintOnY << endl;
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
      }
  }



  bool PatternGeneratorInterface::RunOneStepOfTheControlLoop(MAL_VECTOR(,double) & CurrentConfiguration,
							     MAL_VECTOR(,double) & CurrentVelocity,
							     MAL_VECTOR( &ZMPTarget,double))
    
  {
    
    long int u=0;
    if ((!m_ShouldBeRunning) ||
	(m_ZMPPositions.size()< 2*m_NL+1))
      {
	ODEBUG(" m_ShoulBeRunning " << m_ShouldBeRunning << endl <<
		" m_ZMPPositions " << m_ZMPPositions.size() << endl <<
		" 2*m_NL+1 " << 2*m_NL+1 << endl);
		
	return false;
      }
    
    double qWaistYaw=0.0;
    COMPosition COMPositionFromPC1,finalCOMPosition;
    
    // New scheme:
    // Update the queue of ZMP ref
    m_ZMPpcwmbz->UpdateTheZMPRefQueue(m_ZMPPositions[2*m_NL]);
    
    if ((m_StepStackHandler->GetWalkMode()==0) ||
	(m_StepStackHandler->GetWalkMode()==4))
      {
	m_COMBuffer[m_NL].yaw = m_ZMPPositions[m_NL].theta;
      }
    //    COMPositionFromPC1 = m_COMBuffer[m_NL];
    finalCOMPosition =  m_COMBuffer[m_NL];

    ODEBUG("ZMP : " << m_ZMPPositions[0].px 
	   << " " << m_ZMPPositions[0].py 
	   << " " << m_ZMPPositions[2*m_NL].px 
	   << " " << m_ZMPPositions[2*m_NL].py );

    ODEBUG(m_count << " before-CurrentConfiguration " << CurrentConfiguration);
    m_ZMPpcwmbz->OneGlobalStepOfControl(m_LeftFootPositions[m_NL],
					m_RightFootPositions[m_NL],
					m_ZMPPositions[2*m_NL],
					finalCOMPosition,
					CurrentConfiguration,
					CurrentVelocity);
    ODEBUG(m_count << " CurrentConfiguration " << CurrentConfiguration);
    for(unsigned int i=0;i<m_CurrentActuatedJointValues.size();i++)
      {
	Joint * aJoint = (Joint *)m_DMB->GetJointFromVRMLID(i);
	if (aJoint !=0)
	  m_CurrentActuatedJointValues[i] = aJoint->quantity();
      }
    
    
    /*
      if (m_count==0)
        {
          for(unsigned int i=0;i<m_CurrentActuatedJointValues.size();i++)
	    cout << m_CurrentActuatedJointValues[i] << " " ;
	  cout << endl;
	}
    */
    m_COMBuffer[0] = finalCOMPosition;  

    m_count++;

    // Compute the waist position in the current motion global reference frame.
    //     MAL_S4x4_MATRIX( FinalDesiredCOMPose,double);
    //     FinalDesiredCOMPose = m_ZMPpcwmbz->GetFinalDesiredCOMPose();			
    //     ODEBUG3("FinalDesiredCOMPose :" << FinalDesiredCOMPose);
    MAL_S4x4_MATRIX( PosOfWaistInCOMF,double);
    PosOfWaistInCOMF = m_ZMPpcwmbz->GetCurrentPositionofWaistInCOMFrame();
    // MAL_S4x4_MATRIX( AbsWaist,double);
    //     MAL_S4x4_C_eq_A_by_B(AbsWaist ,FinalDesiredCOMPose , PosOfWaistInCOMF);
    //     ODEBUG3("AbsWaist " << AbsWaist);
    //     ODEBUG3("PosOfWaistInCOMF " << PosOfWaistInCOMF);
    //     ODEBUG3("Configuration(0-2): " << 
    //  	    CurrentConfiguration(0) << " " <<
    //  	    CurrentConfiguration(1) << " " <<
    //  	    CurrentConfiguration(2));

    COMPosition outWaistPosition;
    outWaistPosition = finalCOMPosition;
    outWaistPosition.x[0] =  CurrentConfiguration(0);
    outWaistPosition.y[0] =  CurrentConfiguration(1);
    outWaistPosition.z[0] =  CurrentConfiguration(2);
    
    // In case we are at the end of the motion
    double CurrentZMPNeutralPosition[2];
    CurrentZMPNeutralPosition[0] = m_ZMPPositions[0].px;
    CurrentZMPNeutralPosition[1] = m_ZMPPositions[0].py;
    
    double temp1;
    double temp2;
    double temp3;
    temp1 = m_ZMPPositions[0].px - outWaistPosition.x[0];
    temp2 = m_ZMPPositions[0].py - outWaistPosition.y[0];
    temp3 = finalCOMPosition.yaw*M_PI/180.0;
    
    ZMPTarget(0) = cos(temp3)*temp1+sin(temp3)*temp2 ;
    ZMPTarget(1) = -sin(temp3)*temp1+cos(temp3)*temp2;
    ZMPTarget(2) = -finalCOMPosition.z[0] - PosOfWaistInCOMF[2];
    
    ODEBUG4(finalCOMPosition.z[0] << " " << PosOfWaitInCOMF[2] << " " << ZMPTarget(2,0),
	    "DebugDataZMPTargetZ.dat");
    m_ZMPPositions.pop_front();
    m_COMBuffer.pop_front();
    m_LeftFootPositions.pop_front();
    m_RightFootPositions.pop_front();
    m_UpperBodyPositionsBuffer.pop_front();
		
    m_CurrentWaistState = outWaistPosition;
    ODEBUG4("CurrentWaistState: " 
	    << m_CurrentWaistState.x[0] << " " 
	    << m_CurrentWaistState.y[0] << " " 
	    << m_CurrentWaistState.z[0] << " ","DebugDataWaist.dat" );

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

	    // Specifies the next starting ZMP position    
	    //	    m_ZMPD->setZMPNeutralPosition(CurrentZMPNeutralPosition);

	    ODEBUG("m_count " << m_count <<
		   " m_ZMPPositions.size() " << m_ZMPPositions.size() << 
		   " u : " << u);
	  }

	/*	
	ODEBUG3("CurrentActuatedJointValues at the end: " );
	for(unsigned int i=0;i<m_CurrentActuatedJointValues.size();i++)
	  cout << m_CurrentActuatedJointValues[i] << " " ;
	cout << endl;
	*/
	ODEBUG4("*** TAG *** " , "DebugDataIK.dat");

      }

    // Update the absolute position of the robot.
    // to be done only when the robot has finish a motion.
    UpdateAbsolutePosition(UpdateAbsMotionOrNot);
    return true;
  }

																													     
  
  void PatternGeneratorInterface::DebugControlLoop(MAL_VECTOR(,double) & CurrentConfiguration,
						   MAL_VECTOR(,double) & CurrentVelocity,
						   int localindex)
  {
    
    if (m_ZMPPositions.size()-2*m_NL<0)
      return;

    MAL_VECTOR_DIM(dqlRefState,double,6);
    MAL_VECTOR_DIM(dqrRefState,double,6);
    MAL_VECTOR_DIM(qArmr,double,7);
    MAL_VECTOR_DIM(qArml,double,7);

    int LINKSFORRLEG[6] = { 0, 1, 2, 3,  4, 5};
    int LINKSFORLLEG[6] = { 6, 7, 8, 9, 10, 11};
    //    int LINKSFORRARM[6] = { 16, 17, 18, 19, 20, 21};
    //    int LINKSFORLARM[6] = { 23, 24, 25, 26, 27, 28};

    MAL_VECTOR_DIM( dql,double,6); 
    MAL_VECTOR_DIM( dqr,double,6);
    MAL_VECTOR_DIM( dqal,double,6);
    MAL_VECTOR_DIM( dqar,double,6);
    MAL_VECTOR_DIM( ddql,double,6);
    MAL_VECTOR_DIM( ddqr,double,6);
    MAL_VECTOR_DIM( ddqal,double,6);
    MAL_VECTOR_DIM( ddqar,double,6);
    
    MAL_VECTOR_DIM( dUpperBodyAngles,double,m_NbOfUpperBodyJoints);
    
    MAL_S3_VECTOR(ZMPmultibody,double);

    
    if (m_count>1)
      {
	for(int i=0;i<6;i++)
	  {
	    dql[i] = CurrentVelocity[i+6+6];
	    dqr[i] = CurrentVelocity[i+6];
	  }
      }
    else 
      {
	for(int i=0;i<6;i++)
	  {
	    dql[i] = 0.0;
	    dqr[i] = 0.0;
	  }
      }

    double qWaistYaw=0.0,dqWaistYaw=0.0,ddqWaistYaw;
    //get real angles and ZMP
    
    // Computes New Multybody ZMP.  and angular velocity of real angles
    //upperbody angles
    //	m_2DMB->Setq(LINKSFORUPPERBODY[0],qWaistYaw);
    //	m_2DMB->Setq(LINKSFORUPPERBODY[1],0.0);
    
    if (m_count>1)
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
    
#if 0
    // Update the Dynamic multi body model with upperbody motion
    for(int j=0;j<m_NbOfUpperBodyJoints;j++)
      {
	
	// Hard coded function to check HRP-2 .
	// Free flyer + the current degree of freedoms.
	m_2DMB->Setq(m_UpperBodyJoints[j],
		     CurrentConfiguration(m_UpperBodyJoints[j]));
	if (m_count>1)
	  dUpperBodyAngles(j) = (CurrentConfiguration(m_UpperBodyJoints[j])- 
				 m_Debug_prev_UpperBodyAngles(j))/m_SamplingPeriod;
	else
	  dUpperBodyAngles(j) = 0.0;
	m_2DMB->Setdq(m_UpperBodyJoints[j],dUpperBodyAngles(j));
	m_Debug_prev_UpperBodyAngles(j) = CurrentConfiguration(m_UpperBodyJoints[j]);
      }
#endif
    
    //	m_2DMB->Setdq(LINKSFORUPPERBODY[0],dqWaistYaw);
    //	m_2DMB->Setdq(LINKSFORUPPERBODY[1],0.0);
    
    m_Debug_prev_qWaistYaw = qWaistYaw;
    m_Debug_prev_dqWaistYaw = dqWaistYaw;
    
    ODEBUG4( CurrentConfiguration(12+0)<< " " <<  
	     CurrentConfiguration(12+1)  << " " 
	     << CurrentConfiguration(12+2) << " " 
	     << CurrentConfiguration(12+3) << "  " << 
	     CurrentConfiguration(12+4) << " " << 
	     CurrentConfiguration(12+5),
	     "DebugDataqlDisplay.txt" );
    ODEBUG4( qr(0)<< " " <<  qr(1)  << " " << qr(2) << " " 
	     << qr(3) << "  " << qr(4) << " " 
	     << qr(5), "DebugDataqrDisplay.txt" );
    ODEBUG4( dql(0)<< " " <<  dql(1)  << " " << 
	     dql(2) << " " << dql(3) << "  " << 
	     dql(4) << " " << dql(5),
	     "DebugDatadqlDisplay.txt" );
    ODEBUG4( dqr(0) << " " << dqr(1) << " " << 
	     dqr(2) << " " << dqr(3) << "  " << 
	     dqr(4) << " " << dqr(5), "DebugDatadqrDisplay.txt" );
    ODEBUG4( dql(0) << " " << dql(1) << " " << 
	     dql(2) << " " << dql(3) << "  " << dql(4) << " " << dql(5),
	     "DebugDatadqlDisplay.txt" );
    ODEBUG4( UpperBodyAngles(0) << " " <<
	     UpperBodyAngles(1) << " " <<
	     UpperBodyAngles(2) << " " <<
	     UpperBodyAngles(3) << " " <<
	     UpperBodyAngles(4) << " " <<
	     UpperBodyAngles(5) << " " <<
	     UpperBodyAngles(6) << " " <<
	     UpperBodyAngles(7) << " " <<
	     UpperBodyAngles(8) << " " <<
	     UpperBodyAngles(9) << " " <<
	     UpperBodyAngles(10) << " " <<
	     UpperBodyAngles(11) << " " <<
	     UpperBodyAngles(12) << " " <<
	     UpperBodyAngles(13) << " " <<
	     UpperBodyAngles(14) << " " <<
	     UpperBodyAngles(15) << " " <<
	     UpperBodyAngles(16) << " " <<
	     UpperBodyAngles(17) << " " <<
	     UpperBodyAngles(18) << " " <<
	     UpperBodyAngles(19) << " " <<
	     UpperBodyAngles(20) << " " <<
	     UpperBodyAngles(21) << " " <<
	     UpperBodyAngles(22) << " " <<
	     UpperBodyAngles(23) << " " <<
	     UpperBodyAngles(24) << " " <<
	     UpperBodyAngles(25) << " " <<
	     UpperBodyAngles(26) << " " <<
	     UpperBodyAngles(27) << " " 
	     , "DebugDataUBDisplay.txt" );
    ODEBUG4( dUpperBodyAngles(0) << " " <<
	     dUpperBodyAngles(1) << " " <<
	     dUpperBodyAngles(2) << " " <<
	     dUpperBodyAngles(3) << " " <<
	     dUpperBodyAngles(4) << " " <<
	     dUpperBodyAngles(5) << " " <<
	     dUpperBodyAngles(6) << " " <<
	     dUpperBodyAngles(7) << " " <<
	     dUpperBodyAngles(8) << " " <<
	     dUpperBodyAngles(9) << " " <<
	     dUpperBodyAngles(10) << " " <<
	     dUpperBodyAngles(11) << " " <<
	     dUpperBodyAngles(12) << " " <<
	     dUpperBodyAngles(13) << " " <<
	     dUpperBodyAngles(14) << " " <<
	     dUpperBodyAngles(15) << " " <<
	     dUpperBodyAngles(16) << " " <<
	     dUpperBodyAngles(17) << " " <<
	     dUpperBodyAngles(18) << " " <<
	     dUpperBodyAngles(19) << " " <<
	     dUpperBodyAngles(20) << " " <<
	     dUpperBodyAngles(21) << " " <<
	     dUpperBodyAngles(22) << " " <<
	     dUpperBodyAngles(23) << " " <<
	     dUpperBodyAngles(24) << " " <<
	     dUpperBodyAngles(25) << " " <<
	     dUpperBodyAngles(26) << " " <<
	     dUpperBodyAngles(27) << " ", 
	     "DebugDatadUBDisplay.txt" );

    MAL_S3_VECTOR(WaistPosition,double);
    MAL_S3_VECTOR(WaistVelocity,double);
    MAL_S3_VECTOR(WaistAngularVelocity,double);
    MAL_S3x3_MATRIX(Body_Rm3d,double);

    MAL_S4x4_MATRIX( FinalDesiredCOMPose,double);
    FinalDesiredCOMPose= m_ZMPpcwmbz->GetFinalDesiredCOMPose();			

    WaistPosition[0] = CurrentConfiguration(0);
    WaistPosition[1] = CurrentConfiguration(1);
    WaistPosition[2] = CurrentConfiguration(2); //aCOMPosition.hip-0.705;

    WaistVelocity[0] = CurrentVelocity(0);
    WaistVelocity[1] = CurrentVelocity(1);
    WaistVelocity[2] = CurrentVelocity(2);

    // COM Orientation
    for(int li=0;li<3;li++)
      for(int lj=0;lj<3;lj++)
	Body_Rm3d(li,lj) = FinalDesiredCOMPose(li,lj);

	
    WaistAngularVelocity[0] = 0;
    WaistAngularVelocity[1] = 0;
    WaistAngularVelocity[2] = 0;//(lCOMTheta - m_prev_Zaxis_Angle)/m_SamplingPeriod;
	
    //m_2DMB->ForwardVelocity(WaistPosition,WaistVelocity,WaistAngularVelocity);
    ODEBUG4( WaistPosition[0] << " " << WaistPosition[1] << " " << WaistPosition[2] << " " << 
	     WaistVelocity[0] << " " << WaistVelocity[1] << " " << WaistVelocity[2] << " ", "DebugDataWPDisplay.txt");
    
    ODEBUG4(CurrentConfiguration,"DDCC.dat");
    ODEBUG4(CurrentVelocity,"DDCV.dat");
      
    m_2HumanoidDynamicRobot->currentConfiguration(CurrentConfiguration);
    m_2HumanoidDynamicRobot->currentVelocity(CurrentVelocity);

    m_2HumanoidDynamicRobot->computeForwardKinematics();
    ZMPmultibody = m_2HumanoidDynamicRobot->zeroMomentumPoint();

    ODEBUG4( m_count << " " << ZMPmultibody[0] << " " << ZMPmultibody[1],
	     "DebugDataZMPMB1Display.txt");
    
    MAL_S3_VECTOR( _2DMBCoM,double);
    _2DMBCoM= m_2DMB->getPositionCoM();


    ofstream DebugFileLong, DebugFileUpperBody;
    DebugFileLong.open("DebugDataLong.txt",ofstream::app);
    int lindex=0;
    if (m_FirstPrint)
      {  
	DebugFileLong << lindex++ << "-time" << "\t"                   //  1
		      << lindex++ << "-ZMPdesiredX"<< "\t"             //  2
		      << lindex++ << "-ZMPdesiredY"<< "\t"             //  3
		      << lindex++ << "-ZMPMultiBodyX"<< "\t"           //  4
		      << lindex++ << "-ZMPMultiBodyY"<< "\t"           //  5
		      << lindex++ << "-COMrecomputedX" << "\t"         //  6
		      << lindex++ << "-COMrecomputedY" << "\t"         //  7
		      << lindex++ << "-COMPositionX"<< "\t"            //  8
		      << lindex++ << "-COMPositionY"<< "\t"            //  9
		      << lindex++ << "-COMPositionZ"<< "\t" 	       // 10
		      << lindex++ << "-COMVelocityX"<< "\t"            // 11
		      << lindex++ << "-COMVelocityY"<< "\t"            // 12
		      << lindex++ << "-COMVelocityZ"<< "\t"            // 13
		      << lindex++ << "-COMOrientation"<< "\t"          // 14
		      << lindex++ << "-LeftFootPositionsX"<< "\t"      // 15
		      << lindex++ << "-LeftFootPositionsY"<< "\t"      // 16
		      << lindex++ << "-LeftFootPositionsZ"<< "\t"      // 17
		      << lindex++ << "-RightFootPositionsX" << "\t"    // 18
		      << lindex++ << "-RightFootPositionsY" << "\t"    // 19
		      << lindex++ << "-RightFootPositionsZ" << "\t";   // 20
	// 20 cols

	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << lindex++ << "-LeftLegAngleJoint:" << i << "\t"; 
	    DebugFileLong << lindex++ << "-LeftLegAngleRealJoint:" << i << "\t"; 
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << lindex++ << "-RightLegAngleJoint:" << i << "\t"; 
	    DebugFileLong << lindex++ << "-RightLegAngleRealJoint:" << i << "\t"; 
	  }
	DebugFileLong << lindex++ << "-UpperbodyAngleYaw" << "\t"; 

	// 45 = 20 + 25 cols
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << lindex++ << "-LeftArmAngleJoint:" << i << "\t"; 
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << lindex++ << "-RightArmAngleJoint:" << i << "\t" ;
	  }
				
	// 57 = 45 + 12 cols
	// angular velocities
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << lindex++ << "-LeftLegAngularVelocityJoint:" << i << "\t"; 
	    DebugFileLong << lindex++ << "-LeftLegAngularVelocityRealJoint:" << i << "\t"; 
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << lindex++ << "-RightLegAngularVelocityJoint:" << i << "\t"; 
	    DebugFileLong << lindex++ << "-RightLegAngularVelocityRealJoint:" << i << "\t"; 
	  }
	DebugFileLong << lindex++ << "-UpperbodyAngularVelocityYaw" << "\t"; 
	// 82 = 57 + 25 cols
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << lindex++ << "-LeftArmAngularVelocityJoint:" << i << "\t"; 
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << lindex++ << "-RightArmAngularVelocityJoint:" << i << "\t" ;
	  }
	// 94 = 82 + 12 cols
	// angular accelerations
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << lindex++ << "-LeftLegAngularAccelerationJoint:" << i << "\t" ;
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << lindex++ << "-RightLegAngularAccelerationJoint:" << i << "\t"; 
	  }
	DebugFileLong << lindex++ << "-UpperbodyAngularAccelerationYaw" << "\t" ;
	// 107  = 94 + 13 cols
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << lindex++ << "-LeftArmAngularAccelerationJoint:" << i << "\t";
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << lindex++ << "-RightArmAngularAccelerationJoint:" << i << "\t"; 
	  }
	// 119 = 107 + 12 cols
	DebugFileLong << lindex++ << "-WaistPositionX" << "\t" 
		      << lindex++ << "-WaistPositionY" << "\t" 
		      << lindex++ << "-WaistPositionZ" << "\t" 
		      << lindex++ << "-WaistOrientation0" << "\t" 
		      << lindex++ << "-WaistOrientation1" << "\t" 
		      << lindex++ << "-WaistOrientation2" << "\t" 
		      << lindex++ << "-RightFootPositionsOmega" << "\t"
		      << lindex++ << "-LeftFootPositionsOmega" << "\t"
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
	ODEBUG(m_ZMPPositions[0].px << " "  << m_ZMPPositions[0].py);
 
	DebugFileLong << m_count *m_SamplingPeriod << "\t" 
		      << m_ZMPPositions[0].px << "\t" 
		      << m_ZMPPositions[0].py << "\t" 
		      << ZMPmultibody[0] << "\t" 
		      << ZMPmultibody[1] << "\t" 
		      << _2DMBCoM[0] << "\t" 
		      << _2DMBCoM[1] << "\t" 
		      << FinalDesiredCOMPose(0,3) <<  "\t"  
		      << FinalDesiredCOMPose(1,3) <<  "\t"  
		      << FinalDesiredCOMPose(2,3) <<  "\t"  	
		      << WaistVelocity[0] <<  "\t"  
		      << WaistVelocity[1] <<  "\t"  
		      << WaistVelocity[2] <<  "\t"  
		      << CurrentConfiguration(3) <<  "\t" 
		      << m_LeftFootPositions[0].x <<  "\t"  
		      << m_LeftFootPositions[0].y <<  "\t"  
		      << m_LeftFootPositions[0].z <<  "\t"  
		      << m_RightFootPositions[0].x <<  "\t" 
		      << m_RightFootPositions[0].y <<  "\t"  
		      << m_RightFootPositions[0].z <<  "\t";  
	// 20 lines angles
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << CurrentConfiguration(12+i)*180.0/M_PI<<  "\t"  ;
	    DebugFileLong << CurrentConfiguration(12+i)*180.0/M_PI<<  "\t"  ;
	    //DebugFileLong << qlRefState(i)*180.0/M_PI<<  "\t"  ;
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << CurrentConfiguration(6+i)*180.0/M_PI<<  "\t" ; 
	    DebugFileLong << CurrentConfiguration(6+i)*180.0/M_PI<<  "\t" ; 
	    //DebugFileLong << qrRefState(i)*180.0/M_PI<<  "\t"  ;
	  }
	DebugFileLong << qWaistYaw*180.0/M_PI <<  "\t" ; 
	// 45 lines = 20 +25
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << CurrentConfiguration(6+16+i)*180.0/M_PI<<  "\t" ;
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << CurrentConfiguration(6+23+i)*180.0/M_PI <<  "\t" ; 
	  }
				
	// 57 lines = 45 + 12 
	// angular velocities
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << dql(i)*180.0/M_PI<<  "\t" ;
	    DebugFileLong << dqlRefState(i)*180.0/M_PI<<  "\t" ;
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << dqr(i)*180.0/M_PI<<  "\t"  ;
	    DebugFileLong << dqrRefState(i)*180.0/M_PI<<  "\t" ;
	  }

	DebugFileLong << dqWaistYaw*180.0/M_PI <<  "\t" ; 
	// 82 lines = 57 + 25
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << dUpperBodyAngles(11+i)*180.0/M_PI<<  "\t" ; 
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << dUpperBodyAngles(4+i)*180.0/M_PI<<  "\t" ;
	  }
	// 94 = 82 + 12 cols
	// angular accelerations
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << ddql(i)*180.0/M_PI <<  "\t" ; 
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << ddqr(i)*180.0/M_PI <<  "\t" ; 
	  }
	DebugFileLong << ddqWaistYaw*180.0/M_PI <<  "\t" ; 
	// 107 = 94 + 13 cols
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << ddqal(i)*180.0/M_PI <<  "\t"  ;
	  }
	for(unsigned int i=0;i<6;i++)
	  {
	    DebugFileLong << ddqar(i)*180.0/M_PI <<  "\t"  ;
	  }
	// 119 = 107 + 12 cols
	double rfpo=0.0,lfpo=0.0;
	DebugFileLong << CurrentConfiguration(0) << "\t"
		      << CurrentConfiguration(1) << "\t"
		      << CurrentConfiguration(2) << "\t"
		      << CurrentConfiguration(3) << "\t"
		      << CurrentConfiguration(4) << "\t"
		      << CurrentConfiguration(5) << "\t"
		      << rfpo << "\t" 
		      << lfpo 
		      << endl;
	// 127  =119 + 8
	for(unsigned int j=0;j<m_NbOfUpperBodyJoints;j++) 
	  {
	    DebugFileUpperBody << CurrentConfiguration(m_UpperBodyJoints[j]) << "\t"  ;
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
  
  void PatternGeneratorInterface::SetCurrentJointValues(MAL_VECTOR( & lCurrentJointValues,double))
  {
    if(MAL_VECTOR_SIZE(lCurrentJointValues)!=m_CurrentActuatedJointValues.size())
      m_CurrentActuatedJointValues.resize(MAL_VECTOR_SIZE(lCurrentJointValues));
    
    for(unsigned int i=0;i<MAL_VECTOR_SIZE(lCurrentJointValues);i++)
      {
	m_CurrentActuatedJointValues[i] = lCurrentJointValues(i);
      }
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

  void PatternGeneratorInterface::GetLegJointVelocity(MAL_VECTOR( & dqr,double),
						      MAL_VECTOR( & dql,double))
  {

    // TO DO: take the joint specific to the legs
    // and create the appropriate vector.
    for(int i=0;i<6;i++)
      {
	dqr(i) = m_dqr(i);
	dql(i) = m_dql(i);
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
	
	aCOMPos.pitch = 0.0;
	aCOMPos.roll = 0.0;
	m_COMBuffer.push_back(aCOMPos);
	
	// Add UpperBody Position set at a default value.	
      }

    if (m_StepStackHandler->GetWalkMode()!=3)
      {
	
	for(int i=0;i<aNumber;i++)
	  {
	    // SPECIFIC TO A ROBOT ...
	    anUpperBodyPos.Joints.resize(28);
	    
	    for(unsigned int j=0;j<28;j++)
	      anUpperBodyPos.Joints[j] = m_CurrentActuatedJointValues[j+12];
	    
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
    m_WaistRelativePos(3,0) = 0.0;
    m_WaistRelativePos(3,1) = 0.0;
    m_WaistRelativePos(3,2) = 0.0;
    m_WaistRelativePos(3,3) = 1.0;
  
    double thetarad = m_CurrentWaistState.yaw*M_PI/180.0;
    double c = cos(thetarad);
    double s = sin(thetarad);
  
    m_WaistRelativePos(0,0) = c; m_WaistRelativePos(0,1)=-s; m_WaistRelativePos(0,2) = 0;
    m_WaistRelativePos(1,0) = s; m_WaistRelativePos(1,1)= c; m_WaistRelativePos(1,2) = 0;
    m_WaistRelativePos(2,0) = 0; m_WaistRelativePos(2,1)= 0; m_WaistRelativePos(2,2) = 1;
    m_WaistRelativePos(3,3) = 1;
  
    MAL_S4_VECTOR( RelativeLinearVelocity,double);
    RelativeLinearVelocity(0) =  m_CurrentWaistState.x[1];
    RelativeLinearVelocity(1) =  m_CurrentWaistState.y[1];
    RelativeLinearVelocity(2) =  m_CurrentWaistState.z[0];
    RelativeLinearVelocity(3) =  1.0;
  
    MAL_S4_VECTOR( RelativeLinearAcc,double);
    RelativeLinearAcc(0) =  m_CurrentWaistState.x[2];
    RelativeLinearAcc(1) =  m_CurrentWaistState.y[2];
    RelativeLinearAcc(2) =  0.0;
    RelativeLinearAcc(3) =  1.0;
    
    MAL_S4x4_C_eq_A_by_B(m_AbsLinearVelocity, 
			 m_MotionAbsOrientation,
			 RelativeLinearVelocity);
    MAL_S4x4_C_eq_A_by_B(m_AbsLinearAcc, 
			 m_MotionAbsOrientation , 
			 RelativeLinearAcc);
    
    m_WaistRelativePos(0,3) = m_CurrentWaistState.x[0];
    m_WaistRelativePos(1,3) = m_CurrentWaistState.y[0];
    m_WaistRelativePos(2,3) = m_CurrentWaistState.z[0];
    
    MAL_S4x4_MATRIX( prevWaistAbsPos,double);
    prevWaistAbsPos = m_WaistAbsPos;
  
    MAL_S4x4_C_eq_A_by_B(m_WaistAbsPos, m_MotionAbsPos , m_WaistRelativePos);
    
    ODEBUG("Motion Abs Pos " << m_MotionAbsPos);
    ODEBUG("Waist Relative Pos " << m_WaistRelativePos);
    ODEBUG("Waist Abs Pos " << m_WaistAbsPos);

    m_AbsAngularVelocity(0) = 0.0;
    m_AbsAngularVelocity(1) = 0.0;
    
    if (m_count!=0)
      m_AbsAngularVelocity(2) = (m_AbsMotionTheta + thetarad - m_AbsTheta )/m_dt;
    else
      m_AbsAngularVelocity(2) = 0.0;
    
    m_AbsAngularVelocity(3) = 1.0;
    //      cout << "m_AbsAngularVelocity " << m_AbsAngularVelocity<< endl;
    m_AbsTheta = fmod(m_AbsMotionTheta + thetarad,2*M_PI);
    
    if (UpdateAbsMotionOrNot)
      {
	m_MotionAbsPos = m_WaistAbsPos;
	// The position is supposed at the ground level
	m_MotionAbsPos(2,3) = 0.0;
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
    aof_WaistAbsPos << m_WaistAbsPos(0,3) << " "
		    << m_WaistAbsPos(1,3) << " "
		    << m_WaistAbsPos(2,3) << " "
		    << m_AbsLinearVelocity[0,0] << " "
		    << m_AbsLinearVelocity[1,0] << " " 
		    << (m_WaistAbsPos(0,3] - prevWaistAbsPos(0,3])/m_dt << " " 
		    << (m_WaistAbsPos(1,3] - prevWaistAbsPos(1,3])/m_odt << " " 
		    << m_AbsAngularVelocity[2,0] << " "
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
  
  void PatternGeneratorInterface::getWaistPositionMatrix(MAL_S4x4_MATRIX( &lWaistAbsPos,double))
  {
    lWaistAbsPos = m_WaistAbsPos;
  }
  
  void PatternGeneratorInterface::getWaistPositionAndOrientation(double aTQ[7], double &Orientation)
  {
    // Position
    aTQ[0] = m_WaistAbsPos(0,3);
    aTQ[1] = m_WaistAbsPos(1,3);
    aTQ[2] = m_WaistAbsPos(2,3);
    
    
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
    m_WaistAbsPos(0,3) = aTQ[0];
    m_WaistAbsPos(1,3) = aTQ[1];
    m_WaistAbsPos(2,3) = aTQ[2];
    double _x = aTQ[3];
    double _y = aTQ[4];
    double _z = aTQ[5];
    double _r = aTQ[6];
    
    
    double x2 = _x * _x;
    double y2 = _y * _y;
    double z2 = _z * _z;
    double r2 = _r * _r;
    m_WaistAbsPos(0,0) = r2 + x2 - y2 - z2;         // fill diagonal terms
    m_WaistAbsPos(1,1) = r2 - x2 + y2 - z2;
    m_WaistAbsPos(2,2) = r2 - x2 - y2 + z2;
    double xy = _x * _y;
    double yz = _y * _z;
    double zx = _z * _x;
    double rx = _r * _x;
    double ry = _r * _y;
    double rz = _r * _z;
    m_WaistAbsPos(0,1) = 2 * (xy - rz);             // fill off diagonal terms
    m_WaistAbsPos(0,2) = 2 * (zx + ry);
    m_WaistAbsPos(1,0) = 2 * (xy + rz);
    m_WaistAbsPos(1,2) = 2 * (yz - rx);
    m_WaistAbsPos(2,0) = 2 * (zx - ry);
    m_WaistAbsPos(2,1) = 2 * (yz + rx);
    
  }
  
  void PatternGeneratorInterface::getWaistVelocity(double & dx,
						   double & dy,
						   double & omega)
  {
    dx = m_AbsLinearVelocity(0);
    dy = m_AbsLinearVelocity(1);
    omega = m_AbsAngularVelocity(2);
  }


}



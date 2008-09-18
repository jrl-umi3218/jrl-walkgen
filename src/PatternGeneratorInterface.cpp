#include <fstream>
#include <sys/time.h>
#include <time.h>

#include <walkGenJrl/PatternGeneratorInterface.h>

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
    m_AlgorithmforZMPCOM = ZMPCOM_KAJITA_2003;

    // Initialize (if needed) debugging actions.
    m_dt = 0.005;
    m_DOF = m_HumanoidDynamicRobot->numberDof();

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
	    MAL_S4x4_MATRIX_ACCESS_I_J(m_MotionAbsPos, i,j) =
	      MAL_S4x4_MATRIX_ACCESS_I_J(m_MotionAbsOrientation, i,j) =
	      MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, i,j) =
	      MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, i,j) =
	      1.0;
	  else
	    MAL_S4x4_MATRIX_ACCESS_I_J(m_MotionAbsPos, i,j) =
	      MAL_S4x4_MATRIX_ACCESS_I_J(m_MotionAbsOrientation, i,j) =
	      MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, i,j) =
	      MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, i,j) =
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

    m_AbsMotionTheta = 0;
    m_InternalClock = 0.0;

  }

  void PatternGeneratorInterface::ObjectsInstanciation(string & HumanoidSpecificitiesFileName)
  {
    // Create fundamental objects to make the WPG runs.

    // INFO: This where you should instanciate your own
    // INFO: implementation of CjrlHumanoidDynamicRobot
    m_HumanoidDynamicRobot= new HumanoidDynamicMultiBody();
    m_HumanoidDynamicRobot->SetHumanoidSpecificitiesFile(HumanoidSpecificitiesFileName);
    m_HumanoidDynamicRobot->setComputeZMP(true);
    m_HumanoidDynamicRobot->setComputeBackwardDynamics(false);

    // INFO: This where you should instanciate your own
    // INFO: object for Com and Foot realization.
    // INFO: The default one is based on a geometrical approach.
    m_ComAndFootRealization = new ComAndFootRealizationByGeometry(this);

    // Creates the foot trajectory generator.
    m_FeetTrajectoryGenerator = new LeftAndRightFootTrajectoryGenerationMultiple(this,
										 m_HumanoidDynamicRobot->getHumanoidSpecificities());

    // ZMP reference and Foot trajectory planner (Preview control method from Kajita2003)
    m_ZMPD = new ZMPDiscretization(this,"",m_HumanoidDynamicRobot->getHumanoidSpecificities());

    // ZMP and CoM generation using the method proposed in Wieber2006.
    m_ZMPQP = new ZMPQPWithConstraint(this,"",m_HumanoidDynamicRobot->getHumanoidSpecificities());

    // ZMP and CoM generation using the analytical method proposed in Morisawa2007.
    m_ZMPM = new AnalyticalMorisawaCompact(this);
    m_ZMPM->SetHumanoidSpecificities(m_HumanoidDynamicRobot->getHumanoidSpecificities());

    // Preview control for a 3D Linear inverse pendulum
    m_PC = new PreviewControl();

    // Object to generate Motion from KineoWorks.
    m_GMFKW = new GenerateMotionFromKineoWorks();

    // Object to have a Dynamic multibody robot model.
    // for the second preview loop.

    // Object to investiguate the result of the second preview loop.
    // INFO: This where you should instanciate your own
    // INFO: implementation of CjrlHumanoidDynamicRobot
    m_2HumanoidDynamicRobot= new HumanoidDynamicMultiBody();
    m_2HumanoidDynamicRobot->SetHumanoidSpecificitiesFile(HumanoidSpecificitiesFileName);
    m_2HumanoidDynamicRobot->setComputeZMP(true);
    m_2HumanoidDynamicRobot->setComputeBackwardDynamics(false);
    

    // Stack of steps handler.
    m_StepStackHandler = new StepStackHandler(this);

    // Stepping over planner.
    m_StOvPl = new StepOverPlanner(m_ObstaclePars,
				   m_HumanoidDynamicRobot->getHumanoidSpecificities());


    // The creation of the double stage preview control manager.
    m_DoubleStagePCStrategy = new DoubleStagePreviewControlStrategy(this);
    m_DoubleStagePCStrategy->SetBufferPositions(&m_ZMPPositions,
						&m_COMBuffer,
						&m_LeftFootPositions,
						&m_RightFootPositions);

    
    m_CoMAndFootOnlyStrategy = new CoMAndFootOnlyStrategy(this);
    m_CoMAndFootOnlyStrategy->SetBufferPositions(&m_ZMPPositions,
						 &m_COMBuffer,
						 &m_LeftFootPositions,
						 &m_RightFootPositions);

    // Default handler :DoubleStagePreviewControl.
    m_GlobalStrategyManager = m_DoubleStagePCStrategy;

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
    m_DoubleStagePCStrategy->InitInterObjects(m_PC,
					      m_HumanoidDynamicRobot,
					      m_ComAndFootRealization,
					      m_StepStackHandler);
        
    m_CoMAndFootOnlyStrategy->InitInterObjects(m_PC,
					       m_HumanoidDynamicRobot,
					       m_ComAndFootRealization,
					       m_StepStackHandler);

    // Initialize the ZMP trajectory generator.
    m_ZMPD->SetSamplingPeriod(m_PC->SamplingPeriod());
    m_ZMPD->SetTimeWindowPreviewControl(m_PC->PreviewControlTime());
    m_ZMPD->SetPreviewControl(m_PC);

    m_ZMPQP->SetSamplingPeriod(m_PC->SamplingPeriod());
    m_ZMPQP->SetTimeWindowPreviewControl(m_PC->PreviewControlTime());
    
    m_ZMPM->SetSamplingPeriod(m_PC->SamplingPeriod());
    m_ZMPM->SetTimeWindowPreviewControl(m_PC->PreviewControlTime());
    m_ZMPM->SetFeetTrajectoryGenerator(m_FeetTrajectoryGenerator);

    // The motion generator based on a Kineoworks pathway is given here.
    m_GMFKW->SetPreviewControl(m_PC);

    // Read the robot VRML file model.
    m_HumanoidDynamicRobot->parserVRML(HumanoidVRMLFileDirectory,
				       HumanoidVRMLFileName,
				       HumanoidLinkJointRank.c_str());

    m_2HumanoidDynamicRobot->parserVRML(HumanoidVRMLFileDirectory,
					HumanoidVRMLFileName,
					HumanoidLinkJointRank.c_str());

    m_HumanoidDynamicRobot->SetTimeStep(m_PC->SamplingPeriod());
    m_2HumanoidDynamicRobot->SetTimeStep(m_PC->SamplingPeriod());

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
    m_StOvPl->SetDynamicMultiBodyModel(m_HumanoidDynamicRobot);
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

    if (m_2HumanoidDynamicRobot!=0)
      delete m_2HumanoidDynamicRobot;
    ODEBUG4("Destructor: did m_2HumanoidDynamicRobot","DebugPGI.txt");

    if (m_HumanoidDynamicRobot!=0)
      delete m_HumanoidDynamicRobot;
    ODEBUG4("Destructor: did m_HumanoidDynamicRobot","DebugPGI.txt");

    if (m_GMFKW!=0)
      delete m_GMFKW;
    ODEBUG4("Destructor: did m_GMKFW","DebugPGI.txt");

    if (m_PC!=0)
      delete m_PC;
    ODEBUG4("Destructor: did m_PC","DebugPGI.txt");

    if (m_ZMPD!=0)
      delete m_ZMPD;
    ODEBUG4("Destructor: did m_ZMPD","DebugPGI.txt");

    if (m_ZMPQP!=0)
      delete m_ZMPQP;
    ODEBUG4("Destructor: did m_ZMPQP","DebugPGI.txt");

    if (m_ZMPM!=0)
      delete m_ZMPM;
    ODEBUG4("Destructor: did m_ZMPM","DebugPGI.txt");

    if (m_ComAndFootRealization!=0)
      delete m_ComAndFootRealization;

    if (m_FeetTrajectoryGenerator!=0)
      delete m_FeetTrajectoryGenerator;

    if (m_DoubleStagePCStrategy!=0)
      delete m_DoubleStagePCStrategy;

    if (m_CoMAndFootOnlyStrategy!=0)
      delete m_CoMAndFootOnlyStrategy;
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
  
  void PatternGeneratorInterface::ReadSequenceOfSteps(istringstream &strm)
  {
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
	  ODEBUG3( "Please select proper walk mode. \
            (0 for normal walking ; \
             1 for walking with waistheight variation ; \
             2 for walking with obstacle stepover)" );
	  return;
	}
      }
    ODEBUG("Just before starting to Finish and RealizeStepSequence()");

  }

  void PatternGeneratorInterface::m_StepSequence(istringstream &strm)
  {

    ODEBUG4("Step Sequence","DebugGMFKW.dat");
    ofstream DebugFile;
    ReadSequenceOfSteps(strm);
    ODEBUG4("After reading Step Sequence","DebugGMFKW.dat");
    FinishAndRealizeStepSequence();
    ODEBUG4("After finish and realize Step Sequence","DebugGMFKW.dat");
  }

  void PatternGeneratorInterface::EvaluateStartingCOM(MAL_VECTOR(  & Configuration,double),
						      MAL_S3_VECTOR(  & lStartingCOMPosition,double))
  {
    m_HumanoidDynamicRobot->currentConfiguration(Configuration);
    m_HumanoidDynamicRobot->setComputeCoM(true);
    m_HumanoidDynamicRobot->computeForwardKinematics();
    lStartingCOMPosition = m_HumanoidDynamicRobot->positionCenterOfMass();
  }


  void PatternGeneratorInterface::CommonInitializationOfWalking(COMPosition  & lStartingCOMPosition,
								MAL_VECTOR(  & BodyAnglesIni,double),
								FootAbsolutePosition & InitLeftFootAbsPos,
								FootAbsolutePosition & InitRightFootAbsPos,
							        deque<RelativeFootPosition> & lRelativeFootPositions,
								vector<double> & lCurrentJointValues,
								bool ClearStepStackHandler)
  {
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
    ODEBUG4("Size of lRelativeFootPositions: " << lRelativeFootPositions.size(),"DebugData.txt");
    for(unsigned int i=0;i<lRelativeFootPositions.size();i++)
      {
	ODEBUG(lRelativeFootPositions[i].sx << " " << 
		lRelativeFootPositions[i].sy << " " <<  
		lRelativeFootPositions[i].theta );

      }

    // Initialize consequently the ComAndFoot Realization object.
    m_GlobalStrategyManager->EvaluateStartingState(BodyAnglesIni,lStartingCOMPosition,
						   InitLeftFootAbsPos, InitRightFootAbsPos);

    ODEBUG("StartingCOMPosition: " << lStartingCOMPosition.x[0] 
	    << " "  << lStartingCOMPosition.y[0]
	    << " "  << lStartingCOMPosition.z[0]);
    ODEBUG4("StartingCOMPosition: " << lStartingCOMPosition.x[0] << " "  << lStartingCOMPosition.y[0],"DebugData.txt");
    // We also initialize the iteration number inside DMB.
    m_HumanoidDynamicRobot->ResetIterationNumber();

    ODEBUG4("CommonInitializationOfWalking " << BodyAnglesIni,"DebugData.txt" );

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
    m_InternalClock = 0.0;
    ReadSequenceOfSteps(strm2);
    StartOnLineStepSequencing();
  }

  void PatternGeneratorInterface::StartOnLineStepSequencing()
  {
    COMPosition lStartingCOMPosition;
    MAL_VECTOR( BodyAnglesIni,double);

    FootAbsolutePosition InitLeftFootAbsPos, InitRightFootAbsPos;
    memset(&InitLeftFootAbsPos,0,sizeof(InitLeftFootAbsPos));
    memset(&InitRightFootAbsPos,0,sizeof(InitRightFootAbsPos));
    
    deque<RelativeFootPosition> lRelativeFootPositions;
    vector<double> lCurrentJointValues;

    ODEBUG("StartOnLineStepSequencing - 1 ");
    m_StepStackHandler->StartOnLineStep();


    CommonInitializationOfWalking(lStartingCOMPosition,
				  BodyAnglesIni,
				  InitLeftFootAbsPos, InitRightFootAbsPos,
				  lRelativeFootPositions,lCurrentJointValues,false);

    
    ODEBUG("StartOnLineStepSequencing - 3 "
	   << lStartingCOMPosition.x[0] << " "
	   << lRelativeFootPositions.size()
	   );

    int NbOfStepsToRemoveFromTheStack=0;
    if (m_AlgorithmforZMPCOM==ZMPCOM_KAJITA_2003)
      {
	ODEBUG("ZMPCOM KAJITA 2003 - 2 ");
	NbOfStepsToRemoveFromTheStack=m_ZMPD->InitOnLine(m_ZMPPositions,
							 m_COMBuffer,
							 m_LeftFootPositions,
							 m_RightFootPositions,
							 InitLeftFootAbsPos,
							 InitRightFootAbsPos,
							 lRelativeFootPositions,
							 lStartingCOMPosition );

      }
    else if (m_AlgorithmforZMPCOM==ZMPCOM_MORISAWA_2007)
      {
	m_ZMPM->SetCurrentTime(m_InternalClock);
	NbOfStepsToRemoveFromTheStack = m_ZMPM->InitOnLine(m_ZMPPositions,
							   m_COMBuffer,
							   m_LeftFootPositions,
							   m_RightFootPositions,
							   InitLeftFootAbsPos,
							   InitRightFootAbsPos,
							   lRelativeFootPositions,
							   lStartingCOMPosition );
	ODEBUG("After Initializing the Analytical Morisawa part. " << m_LeftFootPositions.size()
		<< " " << m_RightFootPositions.size());
      }

    // Keep the last one to be removed at the next insertion.
    for(int i=0;i<NbOfStepsToRemoveFromTheStack-1;i++)
      m_StepStackHandler->RemoveFirstStepInTheStack();

    // Initialization of the first preview.
    for(int j=0; j<m_DOF;j++)
      {
	BodyAnglesIni(j) = lCurrentJointValues[j];
	ODEBUG4(BodyAnglesIni(j),"DebugDataOnLine.txt");
      }

    m_GlobalStrategyManager->Setup(m_ZMPPositions,
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
    COMPosition lStartingCOMPosition;	
    MAL_VECTOR( BodyAnglesIni,double);
    FootAbsolutePosition InitLeftFootAbsPos, InitRightFootAbsPos;
    struct timeval begin, end, time4, time5;

    gettimeofday(&begin,0);

    ODEBUG4("FinishAndRealizeStepSequence() - 1","DebugGMFKW.dat");

    vector<double> lCurrentJointValues;
    m_ZMPD->SetZMPShift(m_ZMPShift);

    MAL_VECTOR(,double) lCurrentConfiguration;

    lCurrentConfiguration = m_HumanoidDynamicRobot->currentConfiguration();  
    ODEBUG4("lCurrent Configuration :" << lCurrentConfiguration,"DebugGMFKW.dat" );

    deque<RelativeFootPosition> lRelativeFootPositions;
    CommonInitializationOfWalking(lStartingCOMPosition,
				  BodyAnglesIni,
				  InitLeftFootAbsPos, InitRightFootAbsPos,
				  lRelativeFootPositions,lCurrentJointValues,true);
    ODEBUG4( "Pass through here ","DebugGMFKW.dat" );
    lCurrentConfiguration(0) = 0.0;
    lCurrentConfiguration(1) = 0.0;
    lCurrentConfiguration(2) = 0.0;
    lCurrentConfiguration(3) = 0.0;
    lCurrentConfiguration(4) = 0.0;
    lCurrentConfiguration(5) = 0.0;
    m_HumanoidDynamicRobot->currentConfiguration(lCurrentConfiguration);

    ODEBUG4("Size of lRelativeFootPositions :" << lRelativeFootPositions.size(),"DebugGMFKW.dat");
    
    // Create the ZMP reference.
    CreateZMPReferences(lRelativeFootPositions,
			lStartingCOMPosition,
			InitLeftFootAbsPos,
			InitRightFootAbsPos);

    ODEBUG("First m_ZMPPositions" << m_ZMPPositions[0].px << " " << m_ZMPPositions[0].py);
    deque<ZMPPosition> aZMPBuffer;

    // Option : Use Wieber06's algorithm to compute a new ZMP
    // profil. Suppose to preempt the first stage of control.
    aZMPBuffer.resize(m_RightFootPositions.size());

    // this function calculates a buffer with COM values after a first preview round,
    // currently required to calculate the arm swing before "onglobal step of control"
    // in order to take the arm swing motion into account in the second preview loop
    if (m_StepStackHandler->GetWalkMode()==2)
      m_StOvPl->CreateBufferFirstPreview(m_COMBuffer,aZMPBuffer,m_ZMPPositions);


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

    ODEBUG4("FinishAndRealizeStepSequence() - 5 ","DebugGMFKW.dat");
    // Link the current trajectory and GenerateMotionFromKineoWorks.

    // Very important, you have to make sure that the correct COM position is
    // set inside this buffer.
    // X and Y  will be defined by the PG, but the height has to be specified.
    // by default it should be Zc.
    // If you want to change use modewalk 2.

    ODEBUG4("FinishAndRealizeStepSequence() - 6 ","DebugGMFKW.dat");
    ODEBUG4("m_ZMPPositions : " << m_ZMPPositions.size() << endl <<
	    " m_LeftFootPositions: " << m_LeftFootPositions.size()<< endl <<
	    " m_RightFootPositions: " << m_RightFootPositions.size()<< endl <<
	    " m_TimeDistrFactor" << m_TimeDistrFactor.size() << endl <<
	    "m_COMBuffer : " << m_COMBuffer.size() << endl,"DebugGMFKW.dat");
    if(m_StepStackHandler->GetWalkMode()==2)
      {
	ODEBUG4("FinishAndRealizeStepSequence() - 6.25 ","DebugGMFKW.dat");
	m_StOvPl->TimeDistributeFactor(m_TimeDistrFactor);
	ODEBUG4("FinishAndRealizeStepSequence() - 6.5 ","DebugGMFKW.dat");
	m_StOvPl->PolyPlanner(m_COMBuffer,m_LeftFootPositions,m_RightFootPositions,m_ZMPPositions);
	ODEBUG4("FinishAndRealizeStepSequence() - 6.75 ","DebugGMFKW.dat");
      }

    gettimeofday(&time4,0);
    ODEBUG4("FinishAndRealizeStepSequence() - 7 ","DebugGMFKW.dat");
    // Read NL informations from ZMPRefPositions.
    m_GlobalStrategyManager->Setup(m_ZMPPositions,
				   m_COMBuffer,
				   m_LeftFootPositions,
				   m_RightFootPositions);

    gettimeofday(&time5,0);

    m_count = 0;
    ODEBUG("FinishAndRealizeStepSequence() - 8 ");

    m_ShouldBeRunning = true;

    m_InternalClock = 0.0;

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

    ODEBUG("PARSECMD");
    if (CallMethod(aCmd,strm))
      {
	ODEBUG("Method " << aCmd << " found and handled.");
      }

    ODEBUG("PGI:ParseCmd: Commande: " << aCmd);

    if (aCmd==":LimitsFeasibility")
      m_SetLimitsFeasibility(strm);

    else if (aCmd==":ZMPShiftParameters")
      m_SetZMPShiftParameters(strm);

    else if (aCmd==":TimeDistributionParameters")
      m_SetTimeDistrParameters(strm);

    else if (aCmd==":stepseq")
      m_StepSequence(strm);

    else if (aCmd==":finish")
      m_FinishAndRealizeStepSequence(strm);

    else if (aCmd==":StartOnLineStepSequencing")
      m_StartOnLineStepSequencing(strm);

    else if (aCmd==":StopOnLineStepSequencing")
      m_StopOnLineStepSequencing(strm);

    else if (aCmd==":readfilefromkw")
      m_ReadFileFromKineoWorks(strm);

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
	m_AlgorithmforZMPCOM = ZMPCOM_WIEBER_2006;
	m_GlobalStrategyManager = m_DoubleStagePCStrategy;
      }
    else if (ZMPTrajAlgo=="Kajita")
      {
	m_AlgorithmforZMPCOM = ZMPCOM_KAJITA_2003;
	m_GlobalStrategyManager = m_DoubleStagePCStrategy;
      }
    else if (ZMPTrajAlgo=="Morisawa")
      {
	m_AlgorithmforZMPCOM = ZMPCOM_MORISAWA_2007;
	m_GlobalStrategyManager = m_CoMAndFootOnlyStrategy;
	m_CoMAndFootOnlyStrategy->SetTheLimitOfTheBuffer(m_ZMPM->ReturnOptimalTimeToRegenerateAStep());
      }
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
							     MAL_VECTOR(,double) & CurrentAcceleration,
							     MAL_VECTOR( &ZMPTarget,double))
  {
    COMPosition finalCOMPosition;
    FootAbsolutePosition LeftFootPosition,RightFootPosition;

    return RunOneStepOfTheControlLoop(CurrentConfiguration,
				      CurrentVelocity,
				      CurrentAcceleration,
				      ZMPTarget,
				      finalCOMPosition,
				      LeftFootPosition,
				      RightFootPosition);
    
  }

  bool PatternGeneratorInterface::RunOneStepOfTheControlLoop(FootAbsolutePosition &LeftFootPosition,
							     FootAbsolutePosition &RightFootPosition,
							     ZMPPosition &ZMPRefPos,
							     COMPosition &COMRefPos)
  {
    MAL_VECTOR(,double)  CurrentConfiguration;
    MAL_VECTOR(,double)  CurrentVelocity;
    MAL_VECTOR(,double)  CurrentAcceleration;
    MAL_VECTOR( ZMPTarget,double);
    bool r=false;

    r = RunOneStepOfTheControlLoop(CurrentConfiguration,
				   CurrentVelocity,
				   CurrentAcceleration,
				   ZMPTarget,
				   COMRefPos,
				   LeftFootPosition,
				   RightFootPosition);

    bzero(&ZMPRefPos,sizeof(ZMPPosition));
    ZMPRefPos.px = ZMPTarget(0);
    ZMPRefPos.py = ZMPTarget(1);
    return r;
  }


  bool PatternGeneratorInterface::RunOneStepOfTheControlLoop(MAL_VECTOR(,double) & CurrentConfiguration,
							     MAL_VECTOR(,double) & CurrentVelocity,
							     MAL_VECTOR(,double) & CurrentAcceleration,
							     MAL_VECTOR( &ZMPTarget,double),
							     COMPosition &finalCOMPosition,
							     FootAbsolutePosition &LeftFootPosition, 
							     FootAbsolutePosition &RightFootPosition )


  {

    m_InternalClock+=m_SamplingPeriod;

    if ((!m_ShouldBeRunning) ||
	(m_GlobalStrategyManager->EndOfMotion()<0))
      {
	ODEBUG(" m_ShoulBeRunning " << m_ShouldBeRunning << endl <<
		" m_ZMPPositions " << m_ZMPPositions.size() << endl <<
		" 2*m_NL+1 " << 2*m_NL+1 << endl);

	return false;
      }

    if (m_StepStackHandler->IsOnLineSteppingOn())
      {
	ODEBUG("On Line Stepping: ON!");
	// ********* WARNING THIS IS THE TIME CONSUMING PART *******************
	if (m_AlgorithmforZMPCOM==ZMPCOM_WIEBER_2006)
	  {
	  }
	else if (m_AlgorithmforZMPCOM==ZMPCOM_KAJITA_2003)
	  {
	    m_ZMPD->OnLine(m_InternalClock,
			   m_ZMPPositions,
			   m_COMBuffer,
			   m_LeftFootPositions,			   
			   m_RightFootPositions);
	  }
	else if (m_AlgorithmforZMPCOM==ZMPCOM_MORISAWA_2007)
	  {
	    m_ZMPM->OnLine(m_InternalClock,
			   m_ZMPPositions,
			   m_COMBuffer,
			   m_LeftFootPositions,
			   m_RightFootPositions);
	  }
      }
    

    m_GlobalStrategyManager->OneGlobalStepOfControl(LeftFootPosition,
						    RightFootPosition,
						    ZMPTarget,
						    finalCOMPosition,
						    CurrentConfiguration,
						    CurrentVelocity,
						    CurrentAcceleration);

    
    // New scheme:
    // Update the queue of ZMP ref
    m_count++;
    
    // Update the waist state, it is assumed that the waist is the free flyer 
    // Depending on the strategy used to generate the CoM trajectory
    // this can be empty.

    m_CurrentWaistState.x[0]  = CurrentConfiguration[0];
    m_CurrentWaistState.y[0]  = CurrentConfiguration[1];
    m_CurrentWaistState.z[0]  = CurrentConfiguration[2];
    m_CurrentWaistState.roll  = CurrentConfiguration[3];
    m_CurrentWaistState.pitch = CurrentConfiguration[4];
    m_CurrentWaistState.yaw   = CurrentConfiguration[5];

    m_CurrentWaistState.x[1]  = CurrentVelocity[0];
    m_CurrentWaistState.y[1]  = CurrentVelocity[1];
    m_CurrentWaistState.z[1]  = CurrentVelocity[2];

    ODEBUG4("CurrentWaistState: " 
	    << m_CurrentWaistState.x[0] << " " 
	    << m_CurrentWaistState.y[0] << " " 
	    << m_CurrentWaistState.z[0] << " "
	    << m_CurrentWaistState.roll << " "
	    << m_CurrentWaistState.pitch << " "
	    << m_CurrentWaistState.yaw,
	    "DebugDataWaist.dat" );

    bool UpdateAbsMotionOrNot = false;

    //    if ((u=(m_count - (m_ZMPPositions.size()-2*m_NL)))>=0)
    if (m_GlobalStrategyManager->EndOfMotion()==GlobalStrategyManager::NEW_STEP_NEEDED)
      {
	ODEBUG("NEW STEP NEEDED" << m_InternalClock/m_SamplingPeriod << " Internal Clock :" << m_InternalClock);
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


	    ODEBUG("Asking a new step");
	    
	    // ********* WARNING THIS IS THE TIME CONSUMING PART *******************
	    if (m_AlgorithmforZMPCOM==ZMPCOM_WIEBER_2006)
	      {
	      }
	    else if (m_AlgorithmforZMPCOM==ZMPCOM_KAJITA_2003)
	      {
		m_ZMPD->OnLineAddFoot(lRelativeFootPositions,
				      m_ZMPPositions,
				      m_COMBuffer,
				      m_LeftFootPositions,			   
				      m_RightFootPositions,
				      EndSequence);
	      }
	    else if (m_AlgorithmforZMPCOM==ZMPCOM_MORISAWA_2007)
	      {
		ODEBUG("Putting a new step SX: " << 
			lRelativeFootPositions.sx << " SY: " 
			<< lRelativeFootPositions.sy );
		m_ZMPM->SetCurrentTime(m_InternalClock);
		m_ZMPM->OnLineAddFoot(lRelativeFootPositions,
				      m_ZMPPositions,
				      m_COMBuffer,
				      m_LeftFootPositions,
				      m_RightFootPositions,
				      EndSequence);
		ODEBUG("Left and Right foot positions queues: " 
			<< m_LeftFootPositions.size() << " " 
			<< m_RightFootPositions.size() );
	      }
	    // ************* THIS HAS TO FIT INSIDE THE control step time  ***********

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
						   MAL_VECTOR(,double) & CurrentAcceleration,
						   int localindex)
  {

    if (m_ZMPPositions.size()-2*m_NL<0)
      return;

    MAL_VECTOR_DIM(dqlRefState,double,6);
    MAL_VECTOR_DIM(dqrRefState,double,6);
    MAL_VECTOR_DIM(qArmr,double,7);
    MAL_VECTOR_DIM(qArml,double,7);

    //    int LINKSFORRLEG[6] = { 0, 1, 2, 3,  4, 5};
    //    int LINKSFORLLEG[6] = { 6, 7, 8, 9, 10, 11};
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
    //	m_2HumanoidDynamicRobot->Setq(LINKSFORUPPERBODY[0],qWaistYaw);
    //	m_2HumanoidDynamicRobot->Setq(LINKSFORUPPERBODY[1],0.0);

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


    //	m_2HumanoidDynamicRobot->Setdq(LINKSFORUPPERBODY[0],dqWaistYaw);
    //	m_2HumanoidDynamicRobot->Setdq(LINKSFORUPPERBODY[1],0.0);

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

    MAL_S3_VECTOR(WaistPosition,double);
    MAL_S3_VECTOR(WaistVelocity,double);
    MAL_S3_VECTOR(WaistAngularVelocity,double);
    MAL_S3x3_MATRIX(Body_Rm3d,double);

    MAL_S4x4_MATRIX( FinalDesiredCOMPose,double);
    //    FinalDesiredCOMPose= m_ZMPpcwmbz->GetFinalDesiredCOMPose();

    WaistPosition[0] = CurrentConfiguration(0);
    WaistPosition[1] = CurrentConfiguration(1);
    WaistPosition[2] = CurrentConfiguration(2); //aCOMPosition.hip-0.705;

    WaistVelocity[0] = CurrentVelocity(0);
    WaistVelocity[1] = CurrentVelocity(1);
    WaistVelocity[2] = CurrentVelocity(2);

    // COM Orientation
    for(int li=0;li<3;li++)
      for(int lj=0;lj<3;lj++)
	MAL_S3x3_MATRIX_ACCESS_I_J(Body_Rm3d, li,lj) =
	  MAL_S4x4_MATRIX_ACCESS_I_J(FinalDesiredCOMPose, li,lj);


    WaistAngularVelocity[0] = 0;
    WaistAngularVelocity[1] = 0;
    WaistAngularVelocity[2] = 0;//(lCOMTheta - m_prev_Zaxis_Angle)/m_SamplingPeriod;

    //m_2HumanoidDynamicRobot->ForwardVelocity(WaistPosition,WaistVelocity,WaistAngularVelocity);
    ODEBUG4( WaistPosition[0] << " " << WaistPosition[1] << " " << WaistPosition[2] << " " <<
	     WaistVelocity[0] << " " << WaistVelocity[1] << " " << WaistVelocity[2] << " ", "DebugDataWPDisplay.txt");

    ODEBUG4(CurrentConfiguration,"DDCC.dat");
    ODEBUG4(CurrentVelocity,"DDCV.dat");

    m_2HumanoidDynamicRobot->currentConfiguration(CurrentConfiguration);
    m_2HumanoidDynamicRobot->currentVelocity(CurrentVelocity);
    m_2HumanoidDynamicRobot->currentAcceleration(CurrentAcceleration);

    m_2HumanoidDynamicRobot->computeForwardKinematics();
    ZMPmultibody = m_2HumanoidDynamicRobot->zeroMomentumPoint();

    ODEBUG4( m_count << " " << ZMPmultibody[0] << " " << ZMPmultibody[1],
	     "DebugDataZMPMB1Display.txt");

    MAL_S3_VECTOR( _2DMBCoM,double);
    _2DMBCoM= m_2HumanoidDynamicRobot->getPositionCoM();


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
		      << MAL_S4x4_MATRIX_ACCESS_I_J(FinalDesiredCOMPose, 0,3) <<  "\t"
		      << MAL_S4x4_MATRIX_ACCESS_I_J(FinalDesiredCOMPose, 1,3) <<  "\t"
		      << MAL_S4x4_MATRIX_ACCESS_I_J(FinalDesiredCOMPose, 2,3) <<  "\t"
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
// 	for(unsigned int i=0;i<6;i++)
// 	  {
// 	    DebugFileLong << dUpperBodyAngles(11+i)*180.0/M_PI<<  "\t" ;
// 	  }
// 	for(unsigned int i=0;i<6;i++)
// 	  {
// 	    DebugFileLong << dUpperBodyAngles(4+i)*180.0/M_PI<<  "\t" ;
// 	  }
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


  int PatternGeneratorInterface::GetWalkMode()
  {
    return m_StepStackHandler->GetWalkMode();
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

  void PatternGeneratorInterface::ExpandCOMPositionsQueues(int aNumber)
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
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 3,0) = 0.0;
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 3,1) = 0.0;
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 3,2) = 0.0;
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 3,3) = 1.0;

    double thetarad = m_CurrentWaistState.yaw;
    double c = cos(thetarad);
    double s = sin(thetarad);

    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 0,0) = c;
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 0,1)=-s;
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 0,2) = 0;

    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 1,0) = s;
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 1,1)= c;
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 1,2) = 0;

    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 2,0) = 0;
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 2,1)= 0;
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 2,2) = 1;

    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 3,3) = 1;

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

    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 0,3) = m_CurrentWaistState.x[0];
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 1,3) = m_CurrentWaistState.y[0];
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistRelativePos, 2,3) = m_CurrentWaistState.z[0];

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
	MAL_S4x4_MATRIX_ACCESS_I_J(m_MotionAbsPos, 2,3) = 0.0;
	m_AbsMotionTheta = m_AbsTheta;
      }

  }

  void PatternGeneratorInterface::getWaistPositionMatrix(MAL_S4x4_MATRIX( &lWaistAbsPos,double))
  {
    lWaistAbsPos = m_WaistAbsPos;
  }

  void PatternGeneratorInterface::getWaistPositionAndOrientation(double aTQ[7], double &Orientation)
  {
    // Position
    aTQ[0] = MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 0,3);
    aTQ[1] = MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 1,3);
    aTQ[2] = MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 2,3);


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
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 0,3) = aTQ[0];
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 1,3) = aTQ[1];
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 2,3) = aTQ[2];
    double _x = aTQ[3];
    double _y = aTQ[4];
    double _z = aTQ[5];
    double _r = aTQ[6];


    double x2 = _x * _x;
    double y2 = _y * _y;
    double z2 = _z * _z;
    double r2 = _r * _r;
    // fill diagonal terms
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 0,0) = r2 + x2 - y2 - z2;
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 1,1) = r2 - x2 + y2 - z2;
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 2,2) = r2 - x2 - y2 + z2;
    double xy = _x * _y;
    double yz = _y * _z;
    double zx = _z * _x;
    double rx = _r * _x;
    double ry = _r * _y;
    double rz = _r * _z;
    // fill off diagonal terms
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 0,1) = 2 * (xy - rz);
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 0,2) = 2 * (zx + ry);
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 1,0) = 2 * (xy + rz);
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 1,2) = 2 * (yz - rx);
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 2,0) = 2 * (zx - ry);
    MAL_S4x4_MATRIX_ACCESS_I_J(m_WaistAbsPos, 2,1) = 2 * (yz + rx);

  }

  void PatternGeneratorInterface::getWaistVelocity(double & dx,
						   double & dy,
						   double & omega)
  {
    dx = m_AbsLinearVelocity(0);
    dy = m_AbsLinearVelocity(1);
    omega = m_AbsAngularVelocity(2);
  }


  int PatternGeneratorInterface::ChangeOnLineStep(double time,
						  FootAbsolutePosition & aFootAbsolutePosition)
  {
    /* Compute the index of the interval which will be modified. */
    if (m_AlgorithmforZMPCOM==ZMPCOM_MORISAWA_2007)
      {
	m_ZMPM->SetCurrentTime(m_InternalClock);
	m_ZMPM->OnLineFootChange(time,
				 aFootAbsolutePosition,
				 m_ZMPPositions,
				 m_COMBuffer,
				 m_LeftFootPositions,
				 m_RightFootPositions,
				 m_StepStackHandler);
	return 0;
      }
    return -1;
  }

  int PatternGeneratorInterface::CreateZMPReferences(deque<RelativeFootPosition> &lRelativeFootPositions,
						     COMPosition &lStartingCOMPosition,
						     FootAbsolutePosition &InitLeftFootAbsPos, 
						     FootAbsolutePosition &InitRightFootAbsPos)
  {
    if (m_AlgorithmforZMPCOM==ZMPCOM_WIEBER_2006)
      {
	m_COMBuffer.clear();
	m_ZMPQP->GetZMPDiscretization(m_ZMPPositions,
				      m_COMBuffer,
				      lRelativeFootPositions,
				      m_LeftFootPositions,
				      m_RightFootPositions,
				      m_Xmax, lStartingCOMPosition,
				      InitLeftFootAbsPos,
				      InitRightFootAbsPos);      
	//m_ZMPQP->GetComBuffer(m_COMBuffer);
      }    
    else if (m_AlgorithmforZMPCOM==ZMPCOM_KAJITA_2003)
      {
	m_ZMPD->GetZMPDiscretization(m_ZMPPositions,
				     m_COMBuffer,
				     lRelativeFootPositions,
				     m_LeftFootPositions,
				     m_RightFootPositions,
				     m_Xmax, lStartingCOMPosition,
				     InitLeftFootAbsPos,
				     InitRightFootAbsPos);
	//	m_COMBuffer.clear();
	//	m_COMBuffer.resize(m_RightFootPositions.size());
      }
    else if (m_AlgorithmforZMPCOM==ZMPCOM_MORISAWA_2007)
      {
	ODEBUG("ZMPCOM_MORISAWA_2007");
	m_ZMPM->GetZMPDiscretization(m_ZMPPositions,
				     m_COMBuffer,
				     lRelativeFootPositions,
				     m_LeftFootPositions,
				     m_RightFootPositions,
				     m_Xmax, lStartingCOMPosition,
				     InitLeftFootAbsPos,
				     InitRightFootAbsPos);

	ODEBUG("ZMPCOM_MORISAWA_2007 " << m_ZMPPositions.size() );
	//	m_ZMPM->GetComBuffer(m_COMBuffer);
      }
    return 0;
  }
  
  void PatternGeneratorInterface::AddStepInStack(double dx, double dy, double theta)
  {
    if (m_StepStackHandler!=0)
      {
	m_StepStackHandler->AddStepInTheStack(dx,dy,theta,m_TSsupport, m_TDsupport);
      }
  }
  
}



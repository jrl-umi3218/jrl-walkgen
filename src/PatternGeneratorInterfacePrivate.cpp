/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Andrei     Herdt
 * Fumio      Kanehiro
 * Francois   Keith
 * Florent    Lamiraux
 * Benallegue Mehdi
 * Alireza    Nakhaei
 * Mathieu    Poirier
 * Olivier    Stasse
 * Eiichi     Yoshida
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/* \doc This object is the interface to the walking gait
   generation architecture. */
#include <fstream>
#include <time.h>
#include <fenv.h>

#include "portability/gettimeofday.hh"
#include "portability/bzero.hh"

#ifdef WIN32
# include <Windows.h>
#endif /* WIN32 */

#include <patterngeneratorinterfaceprivate.hh>
#include <Debug.hh>

namespace PatternGeneratorJRL
{

  PatternGeneratorInterfacePrivate::
  PatternGeneratorInterfacePrivate(PinocchioRobot *aPinocchioRobotRobot)
    : PatternGeneratorInterface(aPinocchioRobotRobot),SimplePlugin(this)
  {
    AllowFPE();
    m_PinocchioRobot = aPinocchioRobotRobot;

    ODEBUG4("Step 0","DebugPGI.txt");

    // Initialization for debugging.
    // End if Initialization

    m_ObstacleDetected = false;
    m_AutoFirstStep = false;
    m_feedBackControl = false;

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
    ObjectsInstanciation();
    ODEBUG("End of Object Instanciation ");
    // Initialize their relationships.

    ODEBUG("Beginning of Inter Object Relation Initialization " );
    InterObjectRelationInitialization();
    ODEBUG("End of Inter Object Relation Initialization ");
    // Initialization of the strategy for ZMP ref trajectory generation.
    m_AlgorithmforZMPCOM = ZMPCOM_KAJITA_2003;

    // Initialize (if needed) debugging actions.
    m_dt = 0.005;
    //m_DOF = m_HumanoidDynamicRobot->numberDof();
    m_DOF = m_PinocchioRobot->numberDof() ;

    m_SamplingPeriod = m_PC->SamplingPeriod();
    m_PreviewControlTime = m_PC->PreviewControlTime();
    if(m_SamplingPeriod==0)
      m_NL = 0;
    else
      m_NL = (unsigned int)(m_PreviewControlTime/m_SamplingPeriod);

    /* For debug purposes. */
    m_Debug_prev_qr.resize(6);
    m_Debug_prev_dqr.resize(6);
    m_Debug_prev_ql.resize(6);
    m_Debug_prev_dql.resize(6);

    m_Debug_prev_qr_RefState.resize(6);
    m_Debug_prev_ql_RefState.resize(6);

    m_Debug_prev_UpperBodyAngles.resize(28);

    m_ZMPShift.resize(4);
    m_ZMPShift[0] = 0.02;
    m_ZMPShift[1] = 0.07;
    m_ZMPShift[2] = 0.02;
    m_ZMPShift[3] = 0.02;

    for(int i=0; i<4; i++)
      {
        for(int j=0; j<4; j++)
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

    //RESETDEBUG5("DebugHerdt.txt");
    //RESETDEBUG5("DebugDataCoMZMP.dat");
    m_NewStepX = 0.0;
    m_NewStepY = 0.0;
    m_NewTheta = 0.0;
    m_NewStep = false;
    m_ShouldBeRunning = false;
    m_Running = false;

    m_AbsMotionTheta = 0;
    m_InternalClock = 0.0;

    for(unsigned int i=0; i<3; i++)
      m_ZMPInitialPoint(i)=0.0;
    m_ZMPInitialPointSet = false;

    RegisterPluginMethods();
  }

  void PatternGeneratorInterfacePrivate::AllowFPE()
  {
    feenableexcept(FE_INVALID|FE_DIVBYZERO|FE_OVERFLOW);
  }

  void PatternGeneratorInterfacePrivate::RegisterPluginMethods()
  {
#define number_of_method 18
    std::string aMethodName[number_of_method] =
      {
       ":LimitsFeasibility",
       ":ZMPShiftParameters",
       ":TimeDistributionParameters",
       ":stepseq",
       ":stepstairseq",
       ":finish",
       ":StartOnLineStepSequencing",
       ":StopOnLineStepSequencing",
       ":readfilefromkw",
       ":SetAlgoForZmpTrajectory",
       ":SetAutoFirstStep",
       ":ChangeNextStep",
       ":samplingperiod",
       ":HerdtOnline",
       ":NaveauOnline",
       ":setVelReference",
       ":setCoMPerturbationForce",
       ":feedBackControl"
      };

    for(int i=0; i<number_of_method; i++)
      {
        if (!SimplePlugin::RegisterMethod(aMethodName[i]))
          {
            std::cerr << "Unable to register " << aMethodName << std::endl;
          }
        else
          {
            ODEBUG("Succeed in registering " << aMethodName[i]);
          }
      }

  }
  void PatternGeneratorInterfacePrivate::ObjectsInstanciation()
  {
    // Create fundamental objects to make the WPG runs.

    // INFO: This where you should instanciate your own
    // INFO: object for Com and Foot realization.
    // INFO: The default one is based on a geometrical approach.
#if USE_QUADPROG==1
    m_ComAndFootRealization.resize(4);
#else
    m_ComAndFootRealization.resize(3);
#endif
    m_ComAndFootRealization[0] = new ComAndFootRealizationByGeometry(this);

    // Creates the foot trajectory generator.
    m_FeetTrajectoryGenerator = new
      LeftAndRightFootTrajectoryGenerationMultiple
      (this,m_PinocchioRobot->leftFoot());

    // ZMP reference and Foot trajectory planner
    // (Preview control method from Kajita2003)
    m_ZMPD = new ZMPDiscretization(this,"",m_PinocchioRobot);

    // ZMP and CoM generation using the method proposed in Wieber2006.
    m_ZMPQP = new ZMPQPWithConstraint(this,"",m_PinocchioRobot);

    // ZMP and CoM generation using the method proposed in Dimitrov2008.
    m_ZMPCQPFF = new ZMPConstrainedQPFastFormulation(this,"",m_PinocchioRobot);

#if USE_QUADPROG==1
    m_ZMPVRSQP = new ZMPVelocityReferencedSQP(this,"",m_PinocchioRobot);
    m_ComAndFootRealization[3] = m_ZMPVRSQP->getComAndFootRealization();
#endif

    // ZMP and CoM generation using the method proposed in Herdt2010.
    m_ZMPVRQP = new ZMPVelocityReferencedQP(this,"",m_PinocchioRobot);
    m_ComAndFootRealization[1] = m_ZMPVRQP->getComAndFootRealization();

    // ZMP and CoM generation using the analytical method proposed in
    // Morisawa2007.
    m_ZMPM = new AnalyticalMorisawaCompact(this,m_PinocchioRobot);
    m_ZMPM->SetHumanoidSpecificities(m_PinocchioRobot);
    m_ComAndFootRealization[2] = m_ZMPM->getComAndFootRealization();

    // Preview control for a 3D Linear inverse pendulum
    m_PC = new PreviewControl
      (this,OptimalControllerSolver::MODE_WITHOUT_INITIALPOS,true);

    // Object to generate Motion from KineoWorks.
    m_GMFKW = new GenerateMotionFromKineoWorks();

    // Object to h ave a Dynamic multibody robot model.
    // for the second preview loop.


    // Stack of steps handler.
    m_StepStackHandler = new StepStackHandler(this);

    // Stepping over planner.
    m_StOvPl = new StepOverPlanner(m_ObstaclePars,
                                   m_PinocchioRobot);


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

  void PatternGeneratorInterfacePrivate::InterObjectRelationInitialization()
  {
    // Initialize the Preview Control.
    m_Zc = m_PC->GetHeightOfCoM();

    // Initialize the Preview Control general object.
    m_DoubleStagePCStrategy->InitInterObjects(m_PinocchioRobot,
                                              m_ComAndFootRealization[0],
                                              m_StepStackHandler);

    m_CoMAndFootOnlyStrategy->InitInterObjects(m_PinocchioRobot,
                                               m_ComAndFootRealization,
                                               m_StepStackHandler);

    // Initialize the ZMP trajectory generator.
    m_ZMPD->SetSamplingPeriod(m_PC->SamplingPeriod());
    m_ZMPD->SetTimeWindowPreviewControl(m_PC->PreviewControlTime());
    //m_ZMPD->SetPreviewControl(m_PC);

    m_ZMPQP->SetSamplingPeriod(m_PC->SamplingPeriod());
    m_ZMPQP->SetTimeWindowPreviewControl(m_PC->PreviewControlTime());
    //m_ZMPQP->SetPreviewControl(m_PC);

    m_ZMPCQPFF->SetSamplingPeriod(m_PC->SamplingPeriod());
    m_ZMPCQPFF->SetTimeWindowPreviewControl(m_PC->PreviewControlTime());
    m_ZMPCQPFF->SetPreviewControl(m_PC);

#if USE_QUADPROG==1
    m_ZMPVRSQP->SetTimeWindowPreviewControl(m_PC->PreviewControlTime());
#endif

    m_ZMPVRQP->SetTimeWindowPreviewControl(m_PC->PreviewControlTime());
    //m_ZMPVRQP->SetPreviewControl(m_PC);
    //m_ZMPVRQP->SetSamplingPeriod(m_PC->SamplingPeriod());

    //m_ZMPCQPFF->SetSamplingPeriod(m_PC->SamplingPeriod());
    //m_ZMPCQPFF->SetTimeWindowPreviewControl(m_PC->PreviewControlTime());


    m_ZMPM->SetSamplingPeriod(m_PC->SamplingPeriod());
    m_ZMPM->SetTimeWindowPreviewControl(m_PC->PreviewControlTime());
    m_ZMPM->SetFeetTrajectoryGenerator(m_FeetTrajectoryGenerator);

    // The motion generator based on a Kineoworks pathway is given here.
    m_GMFKW->SetPreviewControl(m_PC);

    // Read the robot VRML file model.

    m_ComAndFootRealization[0]->setPinocchioRobot(m_PinocchioRobot);
    m_ComAndFootRealization[0]->SetHeightOfTheCoM(m_PC->GetHeightOfCoM());
    m_ComAndFootRealization[0]->setSamplingPeriod(m_PC->SamplingPeriod());
    m_ComAndFootRealization[0]->Initialization();

    for(vector<ComAndFootRealization *>::iterator
          CFR_it = m_ComAndFootRealization.begin() ;
        CFR_it!=m_ComAndFootRealization.end() ; ++CFR_it)
      (*CFR_it)->SetStepStackHandler(m_StepStackHandler);

    m_StOvPl->SetPreviewControl(m_PC);
    m_StOvPl->SetDynamicMultiBodyModel(m_PinocchioRobot);
    m_StOvPl->SetZMPDiscretization(m_ZMPD);


    m_StepStackHandler->SetStepOverPlanner(m_StOvPl);
    m_StepStackHandler->SetWalkMode(0);
    // End of the initialization of the fundamental object.

  }

  PatternGeneratorInterfacePrivate::~PatternGeneratorInterfacePrivate()
  {


    ODEBUG4("Destructor: Start","DebugPGI.txt");

    if (m_StOvPl!=0)
      delete m_StOvPl;
    ODEBUG4("Destructor: did m_StOvPl","DebugPGI.txt");

    if (m_StepStackHandler!=0)
      delete m_StepStackHandler;
    ODEBUG4("Destructor: did m_StepStackHandler","DebugPGI.txt");

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

    //if (m_ZMPCQPFF!=0)
    //  delete m_ZMPCQPFF;
    //ODEBUG4("Destructor: did m_ZMPQP","DebugPGI.txt");

#if USE_QUADPROG==1
    if(m_ZMPVRSQP!=0)
      delete m_ZMPVRSQP;
    ODEBUG4("Destructor: did m_ZMPVRSQP","DebugPGI.txt");
#endif

    if (m_ZMPVRQP!=0)
      delete m_ZMPVRQP;
    ODEBUG4("Destructor: did m_ZMPVRQP","DebugPGI.txt");

    if (m_ZMPM!=0)
      delete m_ZMPM;
    ODEBUG4("Destructor: did m_ZMPM","DebugPGI.txt");

    if (m_ComAndFootRealization[0]!=0)
      delete (m_ComAndFootRealization[0]);

    if (m_FeetTrajectoryGenerator!=0)
      delete m_FeetTrajectoryGenerator;

    if (m_DoubleStagePCStrategy!=0)
      delete m_DoubleStagePCStrategy;

    if (m_CoMAndFootOnlyStrategy!=0)
      delete m_CoMAndFootOnlyStrategy;


  }


  void PatternGeneratorInterfacePrivate::
  m_SetZMPShiftParameters(istringstream &strm)
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


  void PatternGeneratorInterfacePrivate::
  m_SetLimitsFeasibility(istringstream &strm)
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

  void PatternGeneratorInterfacePrivate::
  ReadSequenceOfSteps(istringstream &strm)
  {
    // Read the data inside strm.
    switch (m_StepStackHandler->GetWalkMode())
      {
      case 0:
      case 4:
      case 5:
      case 6:
        {
          m_StepStackHandler->ReadStepSequenceAccordingToWalkMode(strm);
          break;
        }
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

          // Update stack of relative foot by using StpOvPl.
          m_StepStackHandler->ReadStepSequenceAccordingToWalkMode(strm);

          break;
        }
      default:
        {
          std::cerr<<  "Please select proper walk mode. \
               (0 for normal walking ; \
                1 for walking with waistheight variation ; \
                2 for walking with obstacle stepover)"
                   << std::endl;
          return;
        }
      }
    ODEBUG("Just before starting to Finish and RealizeStepSequence()");

  }

  void PatternGeneratorInterfacePrivate::
  setVelReference(istringstream &strm)
  {
#ifdef DEBUG
    std::cout << __PRETTY_FUNCTION__ << " setVelReference"
              << strm.str() << std::endl;
#endif // DEBUG
    // Read the data inside strm.
#if USE_QUADPROG==1
    m_ZMPVRQP->Reference(strm);
    istringstream strm2(strm.str());
    m_ZMPVRSQP->Reference(strm2);
#endif
  }

  void PatternGeneratorInterfacePrivate::
  setCoMPerturbationForce(istringstream &strm)
  {
    // Read the data inside strm.
    m_ZMPVRQP->setCoMPerturbationForce(strm);
    istringstream strm2(strm.str());
#if USE_QUADPROG==1
    m_ZMPVRSQP->setCoMPerturbationForce(strm2);
#endif
  }


  void PatternGeneratorInterfacePrivate::initOnlineHerdt()
  {

    // TODO : The common part has to be shared,
    // and the specific part send back to the algorithm implementation.

    COMState lStartingCOMState;
    memset(&lStartingCOMState,0,sizeof(COMState));
    Eigen::Vector3d lStartingZMPPosition;;

    FootAbsolutePosition InitLeftFootAbsPos, InitRightFootAbsPos;
    memset(&InitLeftFootAbsPos,0,sizeof(InitLeftFootAbsPos));
    memset(&InitRightFootAbsPos,0,sizeof(InitRightFootAbsPos));

    Eigen::Matrix<double, 6, 1> lStartingWaistPose;

    EvaluateStartingState(lStartingCOMState,
                          lStartingZMPPosition,
                          lStartingWaistPose,
                          InitLeftFootAbsPos,
                          InitRightFootAbsPos);

    deque<RelativeFootPosition> RelativeFootPositions;
    m_ZMPVRQP->SetCurrentTime(m_InternalClock);

    m_ZMPVRQP->InitOnLine(m_ZMPPositions,
                          m_COMBuffer,
                          m_LeftFootPositions,
                          m_RightFootPositions,
                          InitLeftFootAbsPos,
                          InitRightFootAbsPos,
                          RelativeFootPositions,
                          lStartingCOMState,
                          lStartingZMPPosition);

    m_GlobalStrategyManager->Setup(m_ZMPPositions,
                                   m_COMBuffer,
                                   m_LeftFootPositions,
                                   m_RightFootPositions);

    m_ShouldBeRunning=true;
  }

  void PatternGeneratorInterfacePrivate::initOnlineNaveau()
  {

    // TODO : The common part has to be shared,
    // and the specific part send back to the algorithm implementation.

    COMState lStartingCOMState;
    memset(&lStartingCOMState,0,sizeof(COMState));
    Eigen::Vector3d lStartingZMPPosition;;

    FootAbsolutePosition InitLeftFootAbsPos, InitRightFootAbsPos;
    memset(&InitLeftFootAbsPos,0,sizeof(InitLeftFootAbsPos));
    memset(&InitRightFootAbsPos,0,sizeof(InitRightFootAbsPos));

    Eigen::Matrix<double, 6, 1> lStartingWaistPose;

    EvaluateStartingState(lStartingCOMState,
                          lStartingZMPPosition,
                          lStartingWaistPose,
                          InitLeftFootAbsPos,
                          InitRightFootAbsPos);

    deque<RelativeFootPosition> RelativeFootPositions;
#if USE_QUADPROG==1
    m_ZMPVRSQP->SetCurrentTime(m_InternalClock);

    m_ZMPVRSQP->InitOnLine(m_ZMPPositions,
                           m_COMBuffer,
                           m_LeftFootPositions,
                           m_RightFootPositions,
                           InitLeftFootAbsPos,
                           InitRightFootAbsPos,
                           RelativeFootPositions,
                           lStartingCOMState,
                           lStartingZMPPosition);
#endif
    m_GlobalStrategyManager->Setup(m_ZMPPositions,
                                   m_COMBuffer,
                                   m_LeftFootPositions,
                                   m_RightFootPositions);

    m_ShouldBeRunning=true;
  }

  void PatternGeneratorInterfacePrivate::
  m_StepSequence(istringstream &strm)
  {

    ODEBUG("Step Sequence");
    ofstream DebugFile;
    ReadSequenceOfSteps(strm);
    ODEBUG("After reading Step Sequence");
    FinishAndRealizeStepSequence();
    ODEBUG("After finish and realize Step Sequence");
  }

  void PatternGeneratorInterfacePrivate::
  m_StepStairSequence(istringstream &strm)
  {

    ODEBUG("Step Sequence");
    ofstream DebugFile;
    m_StepStackHandler->ReadStepStairSequenceAccordingToWalkMode(strm);

    ODEBUG("After reading Step Stair Sequence");
    FinishAndRealizeStepSequence();
    ODEBUG("After finish and realize Step Stair Sequence");
  }


  void PatternGeneratorInterfacePrivate::
  EvaluateStartingCOM
  (Eigen::VectorXd &Configuration,
   Eigen::Vector3d &lStartingCOMState)
  {
    Eigen::VectorXd Velocity=    m_PinocchioRobot->currentRPYVelocity();
    Velocity.setZero();

    m_PinocchioRobot->currentRPYConfiguration(Configuration);
    m_PinocchioRobot->currentRPYVelocity(Velocity);
    m_PinocchioRobot->computeForwardKinematics();
    m_PinocchioRobot->positionCenterOfMass(lStartingCOMState);

  }

  void PatternGeneratorInterfacePrivate::
  EvaluateStartingState
  (COMState  & lStartingCOMState,
   Eigen::Vector3d & lStartingZMPPosition,
   Eigen::Matrix<double, 6, 1> & lStartingWaistPose,
   FootAbsolutePosition & InitLeftFootAbsPos,
   FootAbsolutePosition & InitRightFootAbsPos)
  {
    Eigen::VectorXd lBodyInit;
    lBodyInit.resize(m_CurrentActuatedJointValues.size());

    for(unsigned int j=0;
        j<m_CurrentActuatedJointValues.size();
        j++)
      {
        lBodyInit(j) = m_CurrentActuatedJointValues[j];
      }

    m_GlobalStrategyManager->
      EvaluateStartingState
      (lBodyInit,
       lStartingCOMState,
       lStartingZMPPosition,
       lStartingWaistPose,
       InitLeftFootAbsPos,
       InitRightFootAbsPos);

    ostringstream osscomheightcmd;
    osscomheightcmd << ":comheight "
                    << lStartingCOMState.z[0];
    string atmp = osscomheightcmd.str();
    istringstream isscomheightcmd(atmp);
    ParseCmd(isscomheightcmd);

  }

  // This method assumes that we still are using the ZMP
  // someday it should go out.
  void PatternGeneratorInterfacePrivate::
  AutomaticallyAddFirstStep
  (deque<RelativeFootPosition> & lRelativeFootPositions,
   FootAbsolutePosition & InitLeftFootAbsPos,
   FootAbsolutePosition & InitRightFootAbsPos,
   COMState &lStartingCOMState)
  {
    Eigen::Matrix3d InitPos;
    Eigen::Matrix3d CoMPos;

    double coscomyaw, sincomyaw;
    coscomyaw = cos(lStartingCOMState.yaw[0]);
    sincomyaw = sin(lStartingCOMState.yaw[0]);

    CoMPos(0,0) = coscomyaw;
    CoMPos(0,1) = -sincomyaw;
    CoMPos(0,2) = lStartingCOMState.x[0];

    CoMPos(1,0) = sincomyaw;
    CoMPos(1,1) =  coscomyaw;
    CoMPos(1,2) = lStartingCOMState.y[0];

    CoMPos(2,0) = 0.0;
    CoMPos(2,1) = 0.0;
    CoMPos(2,2) = 1.0;

    ODEBUG("InitLeftFoot:" <<  InitLeftFootAbsPos.x
           << " " << InitLeftFootAbsPos.y
           << " " <<InitLeftFootAbsPos.theta);

    ODEBUG("InitRightFoot:" <<  InitRightFootAbsPos.x
           << " " << InitRightFootAbsPos.y
           << " " <<InitRightFootAbsPos.theta);

    // First step targets the left
    // then the robot should move towards the right.
    double lsx,lsy,ltheta;
    if (lRelativeFootPositions[0].sy > 0 )
      {
        lsx = InitRightFootAbsPos.x;
        lsy = InitRightFootAbsPos.y;
        ltheta = InitRightFootAbsPos.theta;
      }
    // First step targets the right
    // then the robot should move towards the left.
    else
      {
        lsx = InitLeftFootAbsPos.x;
        lsy = InitLeftFootAbsPos.y;
        ltheta = InitLeftFootAbsPos.theta;
      }

    double cosinitfoottheta, sininitfoottheta;
    cosinitfoottheta = cos(ltheta);
    sininitfoottheta = sin(ltheta);

    InitPos(0,0) = cosinitfoottheta;
    InitPos(0,1) = -sininitfoottheta;
    InitPos(0,2) = lsx;

    InitPos(1,0) = sininitfoottheta;
    InitPos(1,1) =  cosinitfoottheta;
    InitPos(1,2) = lsy;

    InitPos(2,0) = 0.0;
    InitPos(2,1) = 0.0;
    InitPos(2,2) = 1.0;

    ODEBUG("InitPos:" << InitPos);
    ODEBUG("CoMPos: " << CoMPos);

    Eigen::Matrix3d iCoMPos;
    iCoMPos=iCoMPos.inverse();
    Eigen::Matrix3d InitialMotion;

    // Compute the rigid motion from the CoM to the next support foot.
    InitialMotion=iCoMPos+InitPos;

    // Create from the rigid motion the step to be added to the list of steps.
    RelativeFootPosition aRFP;
    memset(&aRFP,0,sizeof(aRFP));
    aRFP.sx = InitialMotion(0,2);
    aRFP.sy = InitialMotion(1,2);
    aRFP.theta = atan2(InitialMotion(1,0),InitialMotion(0,0));
    ODEBUG("Initial motion: " << InitialMotion);
    ODEBUG("lRelativeFootPositions:"<<lRelativeFootPositions.size());
    ODEBUG("AutomaticallyAddFirstStep: "
           << aRFP.sx << " "
           << aRFP.sy << " "
           <<aRFP.theta);

    lRelativeFootPositions.push_front(aRFP);

  }

  void PatternGeneratorInterfacePrivate::
  CommonInitializationOfWalking
  (COMState  & lStartingCOMState,
   Eigen::Vector3d & lStartingZMPPosition,
   Eigen::VectorXd &BodyAnglesIni,
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

    for(unsigned int i=0; i<m_CurrentActuatedJointValues.size(); i++)
      lCurrentJointValues[i] = m_CurrentActuatedJointValues[i];

    m_DOF = (int)m_CurrentActuatedJointValues.size();
    BodyAnglesIni.resize(m_CurrentActuatedJointValues.size());

    for(int j=0; j<m_DOF; j++)
      {
        BodyAnglesIni(j) = lCurrentJointValues[j];
      }

    // Copy the relative foot position from the stack handler to here.
    m_StepStackHandler->
      CopyRelativeFootPosition
      (lRelativeFootPositions,ClearStepStackHandler);

    for(unsigned int i=0; i<lRelativeFootPositions.size(); i++)
      {
        ODEBUG(lRelativeFootPositions[i].sx << " " <<
               lRelativeFootPositions[i].sy << " " <<
               lRelativeFootPositions[i].sz << " " <<
               lRelativeFootPositions[i].theta );

      }


    // Initialize consequently the ComAndFoot Realization object.
    Eigen::Matrix<double, 6, 1> lStartingWaistPose;
    m_GlobalStrategyManager->
      EvaluateStartingState(BodyAnglesIni,
                            lStartingCOMState,
                            lStartingZMPPosition,
                            lStartingWaistPose,
                            InitLeftFootAbsPos, InitRightFootAbsPos);

    // Add the first step automatically when the corresponding
    // option is set on.
    if (m_AutoFirstStep)
      {
        AutomaticallyAddFirstStep(lRelativeFootPositions,
                                  InitLeftFootAbsPos,
                                  InitRightFootAbsPos,
                                  lStartingCOMState);
        if (!ClearStepStackHandler)
          {
            m_StepStackHandler->
              PushFrontAStepInTheStack
              (lRelativeFootPositions[0]);
            ODEBUG("Push a position in stack of steps:"<<
                   lRelativeFootPositions[0].sx << " " <<
                   lRelativeFootPositions[0].sy << " " <<
                   lRelativeFootPositions[0].sz << " " <<
                   lRelativeFootPositions[0].theta);
          }
      }

    ODEBUG("StartingCOMState: " << lStartingCOMState.x[0]
           << " "  << lStartingCOMState.y[0]
           << " "  << lStartingCOMState.z[0]);
    // We also initialize the iteration number inside DMB.
    //std::cerr << "You have to implement a reset iteration number." << endl;
    //    string aProperty("ResetIteration"),aValue("any");
    //    m_HumanoidDynamicRobot->setProperty(aProperty,aValue);
    // TODO check if an iteration number is needed in the PinocchioRobot class
    if (0)
      {

        ofstream aof;
        aof.open("/tmp/output.txt", ofstream::out);
        if (aof.is_open())
          {
            for(unsigned int i=0; i<lRelativeFootPositions.size(); i++)
              {
                aof << lRelativeFootPositions[i].sx <<" "
                    << lRelativeFootPositions[i].sy <<" "
                    << lRelativeFootPositions[i].theta
                    << endl;
              }
          }
      }

  }


  void PatternGeneratorInterfacePrivate::StartOnLineStepSequencing()
  {
    COMState lStartingCOMState;
    memset(&lStartingCOMState,0,sizeof(COMState));
    Eigen::Vector3d lStartingZMPPosition;
    Eigen::VectorXd BodyAnglesIni;

    FootAbsolutePosition InitLeftFootAbsPos, InitRightFootAbsPos;
    memset(&InitLeftFootAbsPos,0,sizeof(InitLeftFootAbsPos));
    memset(&InitRightFootAbsPos,0,sizeof(InitRightFootAbsPos));

    deque<RelativeFootPosition> lRelativeFootPositions;
    vector<double> lCurrentJointValues;

    ODEBUG("StartOnLineStepSequencing - 1 ");
    m_StepStackHandler->StartOnLineStep();


    CommonInitializationOfWalking
      (lStartingCOMState,
       lStartingZMPPosition,
       BodyAnglesIni,
       InitLeftFootAbsPos, InitRightFootAbsPos,
       lRelativeFootPositions,lCurrentJointValues,false);


    if (m_ZMPInitialPointSet)
      {
        for(unsigned int i=0; i<3; i++)
          lStartingZMPPosition(i) = m_ZMPInitialPoint(i);
      }


    ODEBUG("StartOnLineStepSequencing - 3 "
           << lStartingCOMState.x[0] << " "
           << lStartingCOMState.y[0] << " "
           << lStartingCOMState.z[0] << " "
           << lRelativeFootPositions.size()
           );
    ODEBUG("ZMPInitialPoint OnLine" << lStartingZMPPosition(0)  << " "
           << lStartingZMPPosition(1)  << " " << lStartingZMPPosition(2) );
    std::size_t NbOfStepsToRemoveFromTheStack=0;
    if (m_AlgorithmforZMPCOM==ZMPCOM_KAJITA_2003)
      {
        ODEBUG("ZMPCOM KAJITA 2003 - 2 ");

        NbOfStepsToRemoveFromTheStack=
          m_ZMPD->InitOnLine
          (m_ZMPPositions,
           m_COMBuffer,
           m_LeftFootPositions,
           m_RightFootPositions,
           InitLeftFootAbsPos,
           InitRightFootAbsPos,
           lRelativeFootPositions,
           lStartingCOMState,
           lStartingZMPPosition);

      }
    else if (m_AlgorithmforZMPCOM==ZMPCOM_MORISAWA_2007)
      {
        m_COMBuffer.clear();
        m_ZMPM->SetCurrentTime(m_InternalClock);

        NbOfStepsToRemoveFromTheStack =
          m_ZMPM->InitOnLine
          (m_ZMPPositions,
           m_COMBuffer,
           m_LeftFootPositions,
           m_RightFootPositions,
           InitLeftFootAbsPos,
           InitRightFootAbsPos,
           lRelativeFootPositions,
           lStartingCOMState,
           lStartingZMPPosition );

        ODEBUG("After Initializing the Analytical Morisawa part. "
               << m_LeftFootPositions.size()
               << " " << m_RightFootPositions.size());
      }
    // Keep the last one to be removed at the next insertion.
    for(std::size_t i=0; i<NbOfStepsToRemoveFromTheStack-1; i++)
      m_StepStackHandler->RemoveFirstStepInTheStack();

    // Initialization of the first preview.
    for(int j=0; j<m_DOF; j++)
      {
        BodyAnglesIni(j) = lCurrentJointValues[j];
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

  void PatternGeneratorInterfacePrivate::
  StopOnLineStepSequencing()
  {
    m_StepStackHandler->StopOnLineStep();
  }

  void PatternGeneratorInterfacePrivate::
  FinishAndRealizeStepSequence()
  {
    ODEBUG("PGI-Start");
    COMState lStartingCOMState;
    Eigen::Vector3d lStartingZMPPosition;
    Eigen::VectorXd BodyAnglesIni;
    FootAbsolutePosition InitLeftFootAbsPos, InitRightFootAbsPos;
    struct timeval begin, end, time4, time5;

    gettimeofday(&begin,0);

    ODEBUG("FinishAndRealizeStepSequence() - 1");

    vector<double> lCurrentJointValues;
    m_ZMPD->SetZMPShift(m_ZMPShift);

    Eigen::VectorXd lCurrentConfiguration;

    lCurrentConfiguration = m_PinocchioRobot->currentRPYConfiguration();

    deque<RelativeFootPosition> lRelativeFootPositions;
    CommonInitializationOfWalking
      (lStartingCOMState,
       lStartingZMPPosition,
       BodyAnglesIni,
       InitLeftFootAbsPos, InitRightFootAbsPos,
       lRelativeFootPositions,lCurrentJointValues,true);

    ODEBUG("lStartingCOMState: "
           << lStartingCOMState.x[0] << " "
           << lStartingCOMState.y[0] << " "
           << lStartingCOMState.z[0] );

    ODEBUG( "Pass through here ");
    lCurrentConfiguration(0) = 0.0;
    lCurrentConfiguration(1) = 0.0;
    lCurrentConfiguration(2) = 0.0;
    lCurrentConfiguration(3) = 0.0;
    lCurrentConfiguration(4) = 0.0;
    lCurrentConfiguration(5) = 0.0;
    m_PinocchioRobot->currentRPYConfiguration(lCurrentConfiguration);

    ODEBUG("Size of lRelativeFootPositions :"
           << lRelativeFootPositions.size());
    // cout <<"Size of lRelativeFootPositions :"
    // << lRelativeFootPositions.size() << endl;
    ODEBUG("ZMPInitialPoint"
           << lStartingZMPPosition(0)  << " "
           << lStartingZMPPosition(1)  << " "
           << lStartingZMPPosition(2) );

    ODEBUG("COMBuffer: " << m_COMBuffer.size() );

    // Create the ZMP reference.
    CreateZMPReferences(lRelativeFootPositions,
                        lStartingCOMState,
                        lStartingZMPPosition,
                        InitLeftFootAbsPos,
                        InitRightFootAbsPos);


    ODEBUG("First m_ZMPPositions"
           << m_ZMPPositions[0].px << " "
           << m_ZMPPositions[0].py);
    deque<ZMPPosition> aZMPBuffer;

    // Option : Use Wieber06's algorithm to compute a new ZMP
    // profil. Suppose to preempt the first stage of control.
    aZMPBuffer.resize(m_RightFootPositions.size());

    // this function calculates a buffer with COM values
    // after a first preview round,
    // currently required to calculate the arm swing
    // before "onglobal step of control"
    // in order to take the arm swing motion into
    // account in the second preview loop
    if (m_StepStackHandler->GetWalkMode()==2)
      m_StOvPl->CreateBufferFirstPreview
        (m_COMBuffer,aZMPBuffer,m_ZMPPositions);


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
        m_StOvPl->PolyPlanner
          (m_COMBuffer,m_LeftFootPositions,
           m_RightFootPositions,m_ZMPPositions);
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
    ODEBUG("FinishAndRealizeStepSequence() - 8 - COMBuffer: "
           << m_COMBuffer.size());

    m_ShouldBeRunning = true;

    m_InternalClock = 0.0;

    gettimeofday(&end,0);
  }


  void PatternGeneratorInterfacePrivate::
  m_ReadFileFromKineoWorks(istringstream &strm)
  {

    string aPartialModel="PartialModel.dat";
    string aKWPath="KWBarPath.pth";

    strm >> aPartialModel;
    strm >> aKWPath;

    ODEBUG6("Went through m_ReadFileFromKineoWorks"
            "istringstream &strm)","DebugGMFKW.dat");
    if (m_GMFKW->ReadPartialModel(aPartialModel)<0)
      cerr<< "Error while reading partial model " << endl;

    if (m_GMFKW->ReadKineoWorksPath(aKWPath)<0)
      cerr<< "Error while reading the path " << endl;
    ODEBUG6("Went before DisplayModel and PAth "
            "m_ReadFileFromKineoWorks(istringstream &strm)",
            "DebugGMFKW.dat");

    //    m_GMFKW->DisplayModelAndPath();
    ODEBUG6("Fini..","DebugGMFKW.dat");
  }

  int PatternGeneratorInterfacePrivate::ParseCmd(istringstream &strm)
  {
    string aCmd;
    strm >> aCmd;

    ODEBUG("PARSECMD");

    if (SimplePluginManager::CallMethod(aCmd,strm))
      {
        ODEBUG("Method " << aCmd << " found and handled.");
      }

    return 0;
  }
  void PatternGeneratorInterfacePrivate::
  ChangeOnLineStep(istringstream &strm,double &newtime)
  {
    if (m_AlgorithmforZMPCOM==ZMPCOM_MORISAWA_2007)
      {
        FootAbsolutePosition aFAP;
        double ltime = (double)m_ZMPM->GetTSingleSupport();
        strm >> aFAP.x;
        strm >> aFAP.y;
        strm >> aFAP.theta;
        strm >> aFAP.z;

        ChangeOnLineStep(ltime,aFAP,newtime);
      }
  }

  void PatternGeneratorInterfacePrivate::CallMethod(string &aCmd,
                                                    istringstream &strm)
  {

    ODEBUG("PGI:ParseCmd: Commande: " << aCmd);

    if (aCmd==":ChangeNextStep")
      {
        double nt;
        ChangeOnLineStep(strm,nt);
      }
    else if (aCmd==":samplingperiod")
      {
        double sp;
        strm >> sp;
        m_SamplingPeriod = sp;
      }

    else if (aCmd==":LimitsFeasibility")
      m_SetLimitsFeasibility(strm);

    else if (aCmd==":ZMPShiftParameters")
      m_SetZMPShiftParameters(strm);

    else if (aCmd==":TimeDistributionParameters")
      m_SetTimeDistrParameters(strm);

    else if (aCmd==":stepseq")
      m_StepSequence(strm);

    else if (aCmd==":stepstairseq")
      m_StepStairSequence(strm);

    else if (aCmd==":finish")
      m_FinishAndRealizeStepSequence(strm);

    else if (aCmd==":StartOnLineStepSequencing")
      {
        m_InternalClock = 0.0;
        ReadSequenceOfSteps(strm);
        StartOnLineStepSequencing();
      }
    else if (aCmd==":StopOnLineStepSequencing")
      StopOnLineStepSequencing();

    else if (aCmd==":setVelReference")
      {
        //m_InternalClock = 0.0;
        setVelReference(strm);
      }

    else if (aCmd==":HerdtOnline")
      {
        m_InternalClock = 0.0;
        initOnlineHerdt();
        printf("Online Herdt\n");
        //ODEBUG4("InitOnLine","DebugHerdt.txt");
      }
    else if (aCmd==":NaveauOnline")
      {
        m_InternalClock = 0.0;
        initOnlineNaveau();
        printf("Online Naveau\n");
        //ODEBUG4("InitOnLine","DebugNaveau.txt");
      }
    else if(aCmd==":feedBackControl")
      {
        std::string lFeedBack;
        strm>> lFeedBack;
        if (lFeedBack=="true")
          m_feedBackControl = true ;
        else  if (lFeedBack=="false")
          m_feedBackControl = false;
        ODEBUG("feedBackControl: " << m_feedBackControl);
      }
    else if (aCmd==":setCoMPerturbationForce")
      {
        setCoMPerturbationForce(strm);
      }

    else if (aCmd==":readfilefromkw")
      m_ReadFileFromKineoWorks(strm);

    else if (aCmd==":SetAlgoForZmpTrajectory")
      {
        m_SetAlgoForZMPTraj(strm);
      }

    else if (aCmd==":SetAutoFirstStep")
      {
        std::string lAutoFirstStep;
        strm>> lAutoFirstStep;
        if (lAutoFirstStep=="true")
          m_AutoFirstStep=true;
        else  if (lAutoFirstStep=="false")
          m_AutoFirstStep=false;
        ODEBUG("SetAutoFirstStep: " << m_AutoFirstStep);
      }

  }

  void PatternGeneratorInterfacePrivate::
  m_SetAlgoForZMPTraj(istringstream &strm)
  {
    string ZMPTrajAlgo;
    strm >> ZMPTrajAlgo;
    ODEBUG("ZMPTrajAlgo: " << ZMPTrajAlgo);
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
        m_CoMAndFootOnlyStrategy->SetTheLimitOfTheBuffer
          (m_ZMPM->ReturnOptimalTimeToRegenerateAStep());
      }
    else if (ZMPTrajAlgo=="Dimitrov")
      {
        m_AlgorithmforZMPCOM = ZMPCOM_DIMITROV_2008;
        m_GlobalStrategyManager = m_DoubleStagePCStrategy;
        cout << "DIMITROV" << endl;
      }
    else if (ZMPTrajAlgo=="Herdt")
      {
        m_AlgorithmforZMPCOM = ZMPCOM_HERDT_2010;
        m_GlobalStrategyManager = m_CoMAndFootOnlyStrategy;
        m_CoMAndFootOnlyStrategy->SetTheLimitOfTheBuffer(0);
        cout << "Herdt" << endl;
      }
    else if (ZMPTrajAlgo=="Naveau")
      {
        m_AlgorithmforZMPCOM = ZMPCOM_NAVEAU_2015;
        m_GlobalStrategyManager = m_CoMAndFootOnlyStrategy;
        m_CoMAndFootOnlyStrategy->SetTheLimitOfTheBuffer(0);
        cout << "Naveau" << endl;
      }
  }

  void PatternGeneratorInterfacePrivate::
  m_SetUpperBodyMotionParameters(istringstream &strm)
  {
    ODEBUG("Upper Body Motion Parameters");
    while(!strm.eof())
      {
      }
  }

  bool PatternGeneratorInterfacePrivate::
  RunOneStepOfTheControlLoop
  (Eigen::VectorXd & CurrentConfiguration,
   Eigen::VectorXd & CurrentVelocity,
   Eigen::VectorXd & CurrentAcceleration,
   Eigen::VectorXd &ZMPTarget)
  {
    COMState finalCOMState;
    FootAbsolutePosition LeftFootPosition,RightFootPosition;

    m_Running = RunOneStepOfTheControlLoop(CurrentConfiguration,
                                           CurrentVelocity,
                                           CurrentAcceleration,
                                           ZMPTarget,
                                           finalCOMState,
                                           LeftFootPosition,
                                           RightFootPosition);
    return m_Running;
  }

  bool PatternGeneratorInterfacePrivate::
  RunOneStepOfTheControlLoop
  (FootAbsolutePosition &LeftFootPosition,
   FootAbsolutePosition &RightFootPosition,
   ZMPPosition &ZMPRefPos,
   COMPosition &COMRefPos)
  {
    Eigen::VectorXd  CurrentConfiguration;
    Eigen::VectorXd  CurrentVelocity;
    Eigen::VectorXd  CurrentAcceleration;
    Eigen::VectorXd ZMPTarget;
    COMState aCOMRefState;

    m_Running = RunOneStepOfTheControlLoop(CurrentConfiguration,
                                           CurrentVelocity,
                                           CurrentAcceleration,
                                           ZMPTarget,
                                           aCOMRefState,
                                           LeftFootPosition,
                                           RightFootPosition);

    COMRefPos = aCOMRefState;
    bzero(&ZMPRefPos,sizeof(ZMPPosition));
    ZMPRefPos.px = ZMPTarget(0);
    ZMPRefPos.py = ZMPTarget(1);
    return m_Running;
  }


  bool PatternGeneratorInterfacePrivate::
  RunOneStepOfTheControlLoop
  (Eigen::VectorXd & CurrentConfiguration,
   Eigen::VectorXd & CurrentVelocity,
   Eigen::VectorXd & CurrentAcceleration,
   Eigen::VectorXd &ZMPTarget,
   COMPosition &finalCOMPosition,
   FootAbsolutePosition &LeftFootPosition,
   FootAbsolutePosition &RightFootPosition )
  {
    COMState aCOMState;
    m_Running = RunOneStepOfTheControlLoop(CurrentConfiguration,
                                           CurrentVelocity,
                                           CurrentAcceleration,
                                           ZMPTarget,
                                           aCOMState,
                                           LeftFootPosition,
                                           RightFootPosition);
    finalCOMPosition = aCOMState;
    return m_Running;
  }

  bool PatternGeneratorInterfacePrivate::
  RunOneStepOfTheControlLoop
  (Eigen::VectorXd & CurrentConfiguration,
   Eigen::VectorXd & CurrentVelocity,
   Eigen::VectorXd & CurrentAcceleration,
   Eigen::VectorXd &ZMPTarget,
   COMState &finalCOMState,
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
        ODEBUG("m_ShouldBeRunning : "<< m_ShouldBeRunning << endl <<
               "m_GlobalStrategyManager: "
               << m_GlobalStrategyManager->EndOfMotion());

        m_Running = false;
        return m_Running;//Andremize
      }
    ODEBUG("Internal clock:" << m_InternalClock);

    m_Running = true;

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
            ODEBUG("InternalClock:" <<m_InternalClock  <<
                   " SamplingPeriod: "<<m_SamplingPeriod);

            m_ZMPM->OnLine(m_InternalClock,
                           m_ZMPPositions,
                           m_COMBuffer,
                           m_LeftFootPositions,
                           m_RightFootPositions);
          }
      }
    else
      /* Check if we are not in an ending phase generated on-line */
      {
        if ((m_AlgorithmforZMPCOM==ZMPCOM_MORISAWA_2007) &&
            (m_ZMPM->GetOnLineMode()))
          {
            m_ZMPM->OnLine(m_InternalClock,
                           m_ZMPPositions,
                           m_COMBuffer,
                           m_LeftFootPositions,
                           m_RightFootPositions);
          }
      }

    if (m_AlgorithmforZMPCOM==ZMPCOM_HERDT_2010)
      {
        ODEBUG("InternalClock:" <<m_InternalClock  <<
               " SamplingPeriod: "<<m_SamplingPeriod);
        m_ZMPVRQP->OnLine(m_InternalClock,
                          m_ZMPPositions,
                          m_COMBuffer,
                          m_LeftFootPositions,
                          m_RightFootPositions);
        m_Running = m_ZMPVRQP->Running();
      }

#if USE_QUADPROG
    if (m_AlgorithmforZMPCOM==ZMPCOM_NAVEAU_2015)
      {
        if(m_feedBackControl)
          {
            m_ZMPVRSQP->UpdateCoM(ZMPTarget,
                                  finalCOMState);
            //cout << "com feedbacked" << endl;
          }

        ODEBUG("InternalClock:" <<m_InternalClock  <<
               " SamplingPeriod: "<<m_SamplingPeriod);

        m_ZMPVRSQP->OnLine(m_InternalClock,
                           m_ZMPPositions,
                           m_COMBuffer,
                           m_LeftFootPositions,
                           m_RightFootPositions);
        m_Running = m_ZMPVRSQP->Running();

        m_ZMPVRSQP->UpdateCurrentPos(m_ZMPPositions[0],
                                     m_COMBuffer[0],
                                     m_LeftFootPositions[0],
                                     m_RightFootPositions[0]);
      }
#endif

    m_GlobalStrategyManager->OneGlobalStepOfControl(LeftFootPosition,
                                                    RightFootPosition,
                                                    ZMPTarget,
                                                    finalCOMState,
                                                    CurrentConfiguration,
                                                    CurrentVelocity,
                                                    CurrentAcceleration);

    ODEBUG("finalCOMState: "  <<
           finalCOMState.x[0] << " " <<
           finalCOMState.x[1] << " " <<
           finalCOMState.x[2] << " " <<
           finalCOMState.y[0] << " " <<
           finalCOMState.y[1] << " " <<
           finalCOMState.y[2] << " " <<
           finalCOMState.z[0] << " " <<
           finalCOMState.z[1] << " " <<
           finalCOMState.z[2] << " " <<
           finalCOMState.yaw[0] << " " <<
           finalCOMState.yaw[1] << " " <<
           finalCOMState.yaw[2] << " " <<
           finalCOMState.pitch[0] << " " <<
           finalCOMState.pitch[1] << " " <<
           finalCOMState.pitch[2] << " " <<
           finalCOMState.roll[0] << " " <<
           finalCOMState.roll[1] << " " <<
           finalCOMState.roll[2] << " " );

    // New scheme:
    // Update the queue of ZMP ref
    m_count++;

    // Update the waist state, it is assumed that the waist is the free flyer
    // Depending on the strategy used to generate the CoM trajectory
    // this can be empty.

    m_CurrentWaistState.x[0]  = CurrentConfiguration[0];
    m_CurrentWaistState.y[0]  = CurrentConfiguration[1];
    m_CurrentWaistState.z[0]  = CurrentConfiguration[2];
    m_CurrentWaistState.roll[0]  = CurrentConfiguration[3];
    m_CurrentWaistState.pitch[0] = CurrentConfiguration[4];
    m_CurrentWaistState.yaw[0]   = CurrentConfiguration[5];

    m_CurrentWaistState.x[1]  = CurrentVelocity[0];
    m_CurrentWaistState.y[1]  = CurrentVelocity[1];
    m_CurrentWaistState.z[1]  = CurrentVelocity[2];

    ODEBUG4("CurrentWaistState: "
            << m_CurrentWaistState.x[0] << " "
            << m_CurrentWaistState.y[0] << " "
            << m_CurrentWaistState.z[0] << " "
            << m_CurrentWaistState.roll[0] << " "
            << m_CurrentWaistState.pitch[0] << " "
            << m_CurrentWaistState.yaw[0],
            "DebugDataWaist.dat" );
    bool UpdateAbsMotionOrNot = false;

    //    if ((u=(m_count - (m_ZMPPositions.size()-2*m_NL)))>=0)

    if (m_GlobalStrategyManager->EndOfMotion()==
        GlobalStrategyManager::NEW_STEP_NEEDED)
      {
        ODEBUG("NEW STEP NEEDED" << m_InternalClock/m_SamplingPeriod
               << " Internal Clock :" << m_InternalClock);
        if (m_StepStackHandler->IsOnLineSteppingOn())
          {
            ODEBUG("Add a step");
            // CAREFULL: we assume that this sequence will create a
            // a new foot steps at the back of the queue handled
            // by the StepStackHandler.
            // Then we have two foot steps: the last one put inside the preview,
            // and the new one.
            RelativeFootPosition lRelativeFootPositions;
            // Add a new step inside the stack.
            if (m_StepStackHandler->ReturnStackSize()<=1)
              {
                m_StepStackHandler->
                  AddStandardOnLineStep(m_NewStep,
                                        m_NewStepX,
                                        m_NewStepY,
                                        m_NewTheta);
                m_NewStep = false;
              }

            // Remove the first step of the queue.
            bool EndSequence = m_StepStackHandler->
              RemoveFirstStepInTheStack();
            ODEBUG("EndSequence:" <<EndSequence);
            // Returns the front foot step in the step stack handler
            // which is not yet
            // in the preview control queue.
            bool EnoughSteps= m_StepStackHandler->
              ReturnFrontFootPosition(lRelativeFootPositions);
            if ((!EnoughSteps)&& (!EndSequence))
              {
                std::cerr << "You don't have enough steps in the"
                          << " step stack handler."
                          << std::endl;
                std::cerr << "And this is not an end sequence."
                          << std::endl;
              }


            ODEBUG(" EnoughSteps: " << EnoughSteps << endl <<
                   " EndSequence:" << EndSequence << endl);

            if (!EndSequence)
              {
                // *** WARNING THIS IS THE TIME CONSUMING PART ***
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
              }
            else if (EndSequence)
              {
                ODEBUG("End Sequence");
                if (m_AlgorithmforZMPCOM==ZMPCOM_WIEBER_2006)
                  {
                  }
                else if (m_AlgorithmforZMPCOM==ZMPCOM_KAJITA_2003)
                  {
                    m_ZMPD->EndPhaseOfTheWalking(m_ZMPPositions,
                                                 m_COMBuffer,
                                                 m_LeftFootPositions,
                                                 m_RightFootPositions);
                  }
                else if (m_AlgorithmforZMPCOM==ZMPCOM_MORISAWA_2007)
                  {
                    ODEBUG("Putting a new step SX: " <<
                           lRelativeFootPositions.sx << " SY: "
                           << lRelativeFootPositions.sy );
                    m_ZMPM->SetCurrentTime(m_InternalClock);
                    m_ZMPM->EndPhaseOfTheWalking(m_ZMPPositions,
                                                 m_COMBuffer,
                                                 m_LeftFootPositions,
                                                 m_RightFootPositions);
                    ODEBUG("("<<m_InternalClock << ")");
                    ODEBUG("Left and Right foot positions queues: "
                           << m_LeftFootPositions.size() << " "
                           << m_RightFootPositions.size() );
                  }
              }
            // **** THIS HAS TO FIT INSIDE THE control step time  ****

          }
        else
          {
            //    cout << "Sorry not enough information" << endl;
            m_ShouldBeRunning = false;
            UpdateAbsMotionOrNot = true;

            if ((m_AlgorithmforZMPCOM==ZMPCOM_MORISAWA_2007) &&
                (m_ZMPM->GetOnLineMode()))
              {
                m_ShouldBeRunning = true;
              }

            ODEBUG("Finished the walking pattern generator ("
                   <<m_InternalClock << ")");
          }

        ODEBUG4("*** TAG *** ", "DebugDataIK.dat");

      }

    // Update the absolute position of the robot.
    // to be done only when the robot has finish a motion.
    UpdateAbsolutePosition(UpdateAbsMotionOrNot);
    ODEBUG("Return true");
    return m_Running;
  }



  void PatternGeneratorInterfacePrivate::
  m_SetTimeDistrParameters(istringstream &strm)
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

  void PatternGeneratorInterfacePrivate::
  SetCurrentJointValues(Eigen::VectorXd &lCurrentJointValues)
  {
    if((std::size_t)lCurrentJointValues.size()!=
       m_CurrentActuatedJointValues.size())
      m_CurrentActuatedJointValues.resize(lCurrentJointValues.size());

    for(unsigned int i=0; i<lCurrentJointValues.size(); i++)
      {
        m_CurrentActuatedJointValues[i] = lCurrentJointValues(i);
      }
  }


  void PatternGeneratorInterfacePrivate::
  m_FinishAndRealizeStepSequence(istringstream &strm)
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


  int PatternGeneratorInterfacePrivate::GetWalkMode() const
  {
    return m_StepStackHandler->GetWalkMode();
  }


  void PatternGeneratorInterfacePrivate::
  m_PartialStepSequence(istringstream &strm)
  {
    if (m_StepStackHandler!=0)
      m_StepStackHandler->m_PartialStepSequence(strm);
  }

  void PatternGeneratorInterfacePrivate::
  GetLegJointVelocity
  (Eigen::VectorXd &dqr,
   Eigen::VectorXd &dql) const
  {

    // TO DO: take the joint specific to the legs
    // and create the appropriate vector.
    for(int i=0; i<6; i++)
      {
        dqr(i) = m_dqr(i);
        dql(i) = m_dql(i);
      }
  }

  void PatternGeneratorInterfacePrivate::ExpandCOMPositionsQueues(int aNumber)
  {
    COMState aCOMPos;
    KWNode anUpperBodyPos;

    for(int i=0; i<aNumber; i++)
      {
        // Add COM value set at a default value.
        aCOMPos.z[0] = m_PC->GetHeightOfCoM();
        aCOMPos.z[1] = 0.0;
        aCOMPos.z[2] = 0.0;

        aCOMPos.pitch[0] = 0.0;
        aCOMPos.roll[0] = 0.0;
        m_COMBuffer.push_back(aCOMPos);

        // Add UpperBody Position set at a default value.
      }

  }



  void PatternGeneratorInterfacePrivate::
  AddOnLineStep
  (double X, double Y, double Theta)
  {
    m_NewStep = true;
    m_NewStepX = X;
    m_NewStepY = Y;
    m_NewTheta = Theta;
  }

  void PatternGeneratorInterfacePrivate::
  UpdateAbsolutePosition
  (bool UpdateAbsMotionOrNot)
  {
    // Compute relative, absolution position and speed.
    m_WaistRelativePos(3,0) = 0.0;
    m_WaistRelativePos(3,1) = 0.0;
    m_WaistRelativePos(3,2) = 0.0;
    m_WaistRelativePos(3,3) = 1.0;

    double thetarad = m_CurrentWaistState.yaw[0];
    double c = cos(thetarad);
    double s = sin(thetarad);

    m_WaistRelativePos(0,0) = c;
    m_WaistRelativePos(0,1)=-s;
    m_WaistRelativePos(0,2) = 0;

    m_WaistRelativePos(1,0) = s;
    m_WaistRelativePos(1,1)= c;
    m_WaistRelativePos(1,2) = 0;

    m_WaistRelativePos(2,0) = 0;
    m_WaistRelativePos(2,1)= 0;
    m_WaistRelativePos(2,2) = 1;

    m_WaistRelativePos(3,3) = 1;

    Eigen::Vector4d RelativeLinearVelocity;
    RelativeLinearVelocity(0) =  m_CurrentWaistState.x[1];
    RelativeLinearVelocity(1) =  m_CurrentWaistState.y[1];
    RelativeLinearVelocity(2) =  m_CurrentWaistState.z[0];
    RelativeLinearVelocity(3) =  1.0;

    Eigen::Vector4d RelativeLinearAcc;
    RelativeLinearAcc(0) =  m_CurrentWaistState.x[2];
    RelativeLinearAcc(1) =  m_CurrentWaistState.y[2];
    RelativeLinearAcc(2) =  0.0;
    RelativeLinearAcc(3) =  1.0;

    m_AbsLinearVelocity=
      m_MotionAbsOrientation*
      RelativeLinearVelocity;
    m_AbsLinearAcc=
      m_MotionAbsOrientation*
      RelativeLinearAcc;

    m_WaistRelativePos(0,3) = m_CurrentWaistState.x[0];
    m_WaistRelativePos(1,3) = m_CurrentWaistState.y[0];
    m_WaistRelativePos(2,3) = m_CurrentWaistState.z[0];

    Eigen::Matrix4d prevWaistAbsPos;
    prevWaistAbsPos = m_WaistAbsPos;

    m_WaistAbsPos= m_MotionAbsPos * m_WaistRelativePos;

    ODEBUG("Motion Abs Pos " << m_MotionAbsPos);
    ODEBUG("Waist Relative Pos " << m_WaistRelativePos);
    ODEBUG("Waist Abs Pos " << m_WaistAbsPos);

    m_AbsAngularVelocity(0) = 0.0;
    m_AbsAngularVelocity(1) = 0.0;

    if (m_count!=0)
      m_AbsAngularVelocity(2) =
        (m_AbsMotionTheta + thetarad - m_AbsTheta )/m_dt;
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

  }

  void PatternGeneratorInterfacePrivate::
  getWaistPositionMatrix
  (Eigen::Matrix4d &lWaistAbsPos) const
  {
    lWaistAbsPos = m_WaistAbsPos;
  }

  //TODO test me
  void PatternGeneratorInterfacePrivate::
  getWaistPositionAndOrientation
  (double aTQ[7], double &Orientation) const
  {
    // Position
    aTQ[0] = m_WaistAbsPos(0,3);
    aTQ[1] = m_WaistAbsPos(1,3);
    aTQ[2] = m_WaistAbsPos(2,3);


    // Carefull : Extremly specific to the pattern generator.
    double /*cx,cy,*/ cz, /*sx,sy,*/ sz;
    /*cx = 0; cy = 0;*/ cz = cos(0.5*m_AbsTheta);
    /*sx = 0; sy = 0;*/ sz = sin(0.5*m_AbsTheta);
    aTQ[3] = 0;
    aTQ[4] = 0;
    aTQ[5] = sz;
    aTQ[6] = cz;
    Orientation = m_AbsTheta;
  }

  void PatternGeneratorInterfacePrivate::
  setWaistPositionAndOrientation
  (double aTQ[7])
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
    // fill diagonal terms
    m_WaistAbsPos(0,0) = r2 + x2 - y2 - z2;
    m_WaistAbsPos(1,1) = r2 - x2 + y2 - z2;
    m_WaistAbsPos(2,2) = r2 - x2 - y2 + z2;
    double xy = _x * _y;
    double yz = _y * _z;
    double zx = _z * _x;
    double rx = _r * _x;
    double ry = _r * _y;
    double rz = _r * _z;
    // fill off diagonal terms
    m_WaistAbsPos(0,1) = 2 * (xy - rz);
    m_WaistAbsPos(0,2) = 2 * (zx + ry);
    m_WaistAbsPos(1,0) = 2 * (xy + rz);
    m_WaistAbsPos(1,2) = 2 * (yz - rx);
    m_WaistAbsPos(2,0) = 2 * (zx - ry);
    m_WaistAbsPos(2,1) = 2 * (yz + rx);

  }

  void PatternGeneratorInterfacePrivate::
  getWaistVelocity
  (double & dx,
   double & dy,
   double & omega) const
  {
    dx = m_AbsLinearVelocity(0);
    dy = m_AbsLinearVelocity(1);
    omega = m_AbsAngularVelocity(2);
  }

  void PatternGeneratorInterfacePrivate::
  setVelocityReference
  (double x,
   double y,
   double yaw)
  {
    m_ZMPVRQP->Reference(x,y,yaw);
#if USE_QUADPROG
    m_ZMPVRSQP->Reference(x,y,yaw);
#endif
  }

  void PatternGeneratorInterfacePrivate::
  setCoMPerturbationForce
  (double x,
   double y)
  {
    m_ZMPVRQP->setCoMPerturbationForce(x,y);
#if USE_QUADPROG
    m_ZMPVRSQP->setCoMPerturbationForce(x,y);
#endif
  }

  int PatternGeneratorInterfacePrivate::
  ChangeOnLineStep
  (double time,
   FootAbsolutePosition & aFootAbsolutePosition,
   double &newtime)
  {
    /* Compute the index of the interval which will be modified. */
    if (m_AlgorithmforZMPCOM==ZMPCOM_MORISAWA_2007)
      {
        m_ZMPM->SetCurrentTime(m_InternalClock);
        m_ZMPM->OnLineFootChange(m_InternalClock+time,
                                 aFootAbsolutePosition,
                                 m_ZMPPositions,
                                 m_COMBuffer,
                                 m_LeftFootPositions,
                                 m_RightFootPositions,
                                 m_StepStackHandler);
        vector<double> lDj;
        m_FeetTrajectoryGenerator->GetDeltaTj(lDj);
        ODEBUG("lDj[0] = " << lDj(0));
        newtime = lDj[0];
        return 0;
      }
    return -1;
  }

  int PatternGeneratorInterfacePrivate::
  CreateZMPReferences
  (deque<RelativeFootPosition> &lRelativeFootPositions,
   COMState &lStartingCOMState,
   Eigen::Vector3d & lStartingZMPPosition,
   FootAbsolutePosition &InitLeftFootAbsPos,
   FootAbsolutePosition &InitRightFootAbsPos)
  {
    if (m_AlgorithmforZMPCOM==ZMPCOM_WIEBER_2006)
      {
        ODEBUG("ZMPCOM_WIEBER_2006 " << m_ZMPPositions.size() );
        m_COMBuffer.clear();
        m_ZMPM->SetCurrentTime(m_InternalClock);
        m_ZMPQP->GetZMPDiscretization(m_ZMPPositions,
                                      m_COMBuffer,
                                      lRelativeFootPositions,
                                      m_LeftFootPositions,
                                      m_RightFootPositions,
                                      m_Xmax, lStartingCOMState,
                                      lStartingZMPPosition,
                                      InitLeftFootAbsPos,
                                      InitRightFootAbsPos);
      }
    // else if (m_AlgorithmforZMPCOM==ZMPCOM_DIMITROV_2008)
    //   {
    //        ODEBUG("ZMPCOM_DIMITROV_2008 " << m_ZMPPositions.size() );
    //        m_COMBuffer.clear();
    //        m_ZMPCQPFF->GetZMPDiscretization(m_ZMPPositions,
    //                                     m_COMBuffer,
    //                                     lRelativeFootPositions,
    //                                     m_LeftFootPositions,
    //                                     m_RightFootPositions,
    //                                     m_Xmax, lStartingCOMState,
    //                                     lStartingZMPPosition,
    //                                     InitLeftFootAbsPos,
    //                                     InitRightFootAbsPos);
    //   }

    else if (m_AlgorithmforZMPCOM==ZMPCOM_HERDT_2010)
      {
        ODEBUG("ZMPCOM_HERDT_2010 " << m_ZMPPositions.size() );
        m_COMBuffer.clear();
        m_ZMPM->SetCurrentTime(m_InternalClock);
        m_ZMPVRQP->GetZMPDiscretization(m_ZMPPositions,
                                        m_COMBuffer,
                                        lRelativeFootPositions,
                                        m_LeftFootPositions,
                                        m_RightFootPositions,
                                        m_Xmax, lStartingCOMState,
                                        lStartingZMPPosition,
                                        InitLeftFootAbsPos,
                                        InitRightFootAbsPos);
      }
#if USE_QUADPROG==1
    else if (m_AlgorithmforZMPCOM==ZMPCOM_NAVEAU_2015)
      {
        ODEBUG("ZMPCOM_NAVEAU_2015 " << m_ZMPPositions.size() );
        m_COMBuffer.clear();
        m_ZMPM->SetCurrentTime(m_InternalClock);
        m_ZMPVRSQP->GetZMPDiscretization(m_ZMPPositions,
                                         m_COMBuffer,
                                         lRelativeFootPositions,
                                         m_LeftFootPositions,
                                         m_RightFootPositions,
                                         m_Xmax, lStartingCOMState,
                                         lStartingZMPPosition,
                                         InitLeftFootAbsPos,
                                         InitRightFootAbsPos);
      }
#endif
    else if (m_AlgorithmforZMPCOM==ZMPCOM_KAJITA_2003)
      {
        ODEBUG("ZMPCOM_KAJITA_2003 " << m_ZMPPositions.size() );
        m_ZMPD->SetCurrentTime(m_InternalClock);
        m_ZMPD->GetZMPDiscretization(m_ZMPPositions,
                                     m_COMBuffer,
                                     lRelativeFootPositions,
                                     m_LeftFootPositions,
                                     m_RightFootPositions,
                                     m_Xmax, lStartingCOMState,
                                     lStartingZMPPosition,
                                     InitLeftFootAbsPos,
                                     InitRightFootAbsPos);
        //      m_COMBuffer.clear();
        //      m_COMBuffer.resize(m_RightFootPositions.size());
      }
    else if (m_AlgorithmforZMPCOM==ZMPCOM_MORISAWA_2007)
      {
        ODEBUG("ZMPCOM_MORISAWA_2007");
        m_COMBuffer.clear();
        m_ZMPM->SetCurrentTime(m_InternalClock);
        m_ZMPM->GetZMPDiscretization(m_ZMPPositions,
                                     m_COMBuffer,
                                     lRelativeFootPositions,
                                     m_LeftFootPositions,
                                     m_RightFootPositions,
                                     m_Xmax, lStartingCOMState,
                                     lStartingZMPPosition,
                                     InitLeftFootAbsPos,
                                     InitRightFootAbsPos);

        ODEBUG("ZMPCOM_MORISAWA_2007 " << m_ZMPPositions.size() );
      }
    return 0;
  }

  void PatternGeneratorInterfacePrivate::
  AddStepInStack
  (double dx, double dy, double theta)
  {
    if (m_StepStackHandler!=0)
      {
        m_StepStackHandler->AddStepInTheStack
          (dx,dy,theta,m_TSsupport, m_TDsupport);
      }
  }

  void PatternGeneratorInterfacePrivate::
  setZMPInitialPoint(Eigen::Vector3d & lZMPInitialPoint)
  {
    m_ZMPInitialPoint = lZMPInitialPoint;
    m_ZMPInitialPointSet = true;
  }

  void PatternGeneratorInterfacePrivate::
  getZMPInitialPoint(Eigen::Vector3d & lZMPInitialPoint) const
  {
    lZMPInitialPoint = m_ZMPInitialPoint;
  }


  PatternGeneratorInterface *
  patternGeneratorInterfaceFactory
  (PinocchioRobot * aRobot)
  {
    return new PatternGeneratorInterfacePrivate(aRobot);
  }



}

/*
 * Copyright 2008, 2009, 2010, 2014, 2015
 *
 * Torea Foissotte
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of jrl-walkgen.
 * jrl-walkgen is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * jrl-walkgen is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with jrl-walkgen. If not, see <http://www.gnu.org/licenses/>.
 *
 * Research carried out within the scope of the
 * Joint Japanese-French Robotics Laboratory (JRL)
 */
/* This object generates the reference value for the
   ZMP based on a polynomail representation
   of the ZMP following
   "Experimentation of Humanoid Walking Allowing Immediate
   Modification of Foot Place Based on Analytical Solution"
   Morisawa, Harada, Kajita, Nakaoka, Fujiwara, Kanehiro, Hirukawa,
   ICRA 2007, 3989--39994
*/

#include <Debug.hh>
#include <fstream>
#include <ZMPRefTrajectoryGeneration/AnalyticalMorisawaCompact.hh>
#include <iomanip>

#include <boost/version.hpp>

typedef double doublereal;
typedef int integer;

extern "C"
{
  extern int dgesvx_( char *, char *, /* 0 FACT TRANS */
                      integer *, integer *,  /* 2 N NHRS */
                      doublereal *, integer *, /* 4 A LDA */
                      doublereal *, integer *, /* 6 AF LDAF */
                      integer *, /* 7 IPIV */
                      char *, /* 8 EQUED */
                      doublereal *, /* 9 R */
                      doublereal *, /* 10 C */
                      doublereal *, /* 11 B */
                      integer *, /* 12 LDB */
                      doublereal *, /* 13 X */
                      integer *, /* 14 LDX */
                      doublereal *, /* 15 RCOND */
                      doublereal *, /* 16 FERR */
                      doublereal *, /* 17 BERR */
                      doublereal *, /* 18 WORK */
                      integer *, /* 19 IWORK */
                      integer * /* 20 INFO */
                      );

#if BOOST_VERSION >=104000
  extern void dgetrf_( integer * m, /* M */
                       integer * n, /* N */
                       doublereal * A, /* A */
                       integer * lda, /* LDA */
                       integer * ipiv, /* IPIV */
                       integer *info /* info */
                       );
#endif
}

namespace PatternGeneratorJRL
{


  AnalyticalMorisawaCompact::
  AnalyticalMorisawaCompact
  (SimplePluginManager *lSPM,
   PinocchioRobot *aPR)
    : AnalyticalMorisawaAbstract(lSPM)
  {

    RegisterMethods();
    m_OnLineMode=false;
    m_EndPhase = false;

    memset(&m_CTIPX,0,sizeof(m_CTIPX));
    memset(&m_CTIPY,0,sizeof(m_CTIPY));

    m_OnLineChangeStepMode = ABSOLUTE_FRAME;
    m_PR = aPR;
    m_FeetTrajectoryGenerator =
      m_BackUpm_FeetTrajectoryGenerator = 0;

    m_NeedToReset = true;
    m_AbsoluteTimeReference = 0.0;

    m_PreviewControl =
      new PreviewControl
      (lSPM,
       OptimalControllerSolver::MODE_WITH_INITIALPOS,
       true);

    /*! Dynamic allocation of the analytical trajectories for the ZMP 
      and the COG */
    m_AnalyticalZMPCoGTrajectoryX = new AnalyticalZMPCOGTrajectory(7);
    m_AnalyticalZMPCoGTrajectoryY = new AnalyticalZMPCOGTrajectory(7);
    // m_AnalyticalZMPCoGTrajectoryZ = new AnalyticalZMPCOGTrajectory(7);

    /*! Dynamic allocation of the filters. */
    m_FilterXaxisByPC =
      new FilteringAnalyticalTrajectoryByPreviewControl
      (lSPM,
       m_AnalyticalZMPCoGTrajectoryX,
       m_PreviewControl);
    
    m_FilterYaxisByPC =
      new FilteringAnalyticalTrajectoryByPreviewControl
      (lSPM,
       m_AnalyticalZMPCoGTrajectoryY,
       m_PreviewControl);
    
    m_kajitaDynamicFilter = new DynamicFilter(lSPM,m_PR);

    m_VerboseLevel=0;

    m_NewStepInTheStackOfAbsolutePosition = false;

    m_FilteringActivate = true;

    m_CoMbsplinesZ = new BSplinesFoot() ;
    m_ZMPpolynomeZ = new Polynome3(0.0,0.0);
    DFpreviewWindowSize_ = 0.0 ;

    RESETDEBUG4("Test.dat");
  }


  AnalyticalMorisawaCompact::~AnalyticalMorisawaCompact()
  {
    if (m_VerboseLevel>2)
      {
        // Display the clock for some part of the code.
        cout << "Part of the foot position computation + queue handling."
             << endl;
        m_Clock1.Display();
        cout << "Part of the foot change landing position" << endl;
        m_Clock2.Display();
        cout << "Part on the analytical ZMP COG trajectories and "
             << "foot polynomial computation"
             << endl;
        m_Clock3.Display();
      }

    string Filename("Clock1.dat");
    m_Clock1.RecordDataBuffer(Filename);
    Filename = "Clock2.dat";
    m_Clock2.RecordDataBuffer(Filename);
    Filename = "Clock3.dat";
    m_Clock3.RecordDataBuffer(Filename);

    if (m_AnalyticalZMPCoGTrajectoryX!=0)
      delete m_AnalyticalZMPCoGTrajectoryX;
    ODEBUG4("Destructor: did AnalyticalZMPCoGTrajectoryX","DebugPGI.txt");

    if (m_AnalyticalZMPCoGTrajectoryY!=0)
      delete m_AnalyticalZMPCoGTrajectoryY;
    ODEBUG4("Destructor: did AnalyticalZMPCoGTrajectoryY","DebugPGI.txt");

    if (m_FilterXaxisByPC!=0)
      delete m_FilterXaxisByPC;

    ODEBUG4("Destructor: did FilterXaxisByPC","DebugPGI.txt");

    if (m_FilterYaxisByPC!=0)
      delete m_FilterYaxisByPC;

    ODEBUG4("Destructor: did FilterYaxisByPC","DebugPGI.txt");

    if (m_PreviewControl!=0)
      delete m_PreviewControl;

    if (m_BackUpm_FeetTrajectoryGenerator!=0)
      delete m_BackUpm_FeetTrajectoryGenerator;
    ODEBUG4("Destructor: did PreviewControl","DebugPGI.txt");
  }


  bool AnalyticalMorisawaCompact::InitializeBasicVariables()
  {
    m_NumberOfIntervals = 2 * m_NumberOfStepsInAdvance+1;

    // Compute the temporal intervals.
    m_DeltaTj.resize(m_NumberOfIntervals);
    m_Omegaj.resize(m_NumberOfIntervals);
    m_StepTypes.resize(m_NumberOfIntervals);


    m_DeltaTj[0]=m_Tsingle*3.0;

    m_StepTypes[0] = DOUBLE_SUPPORT;
    for(int i=1; i<m_NumberOfIntervals; i++)
      {
        if (i%2==0)
          {
            m_DeltaTj[i] = m_Tsingle;
            m_StepTypes[i] = SINGLE_SUPPORT;
          }
        else
          {
            m_DeltaTj[i] = m_Tdble;
            m_StepTypes[i] = DOUBLE_SUPPORT;
          }
      }
    m_DeltaTj[m_NumberOfIntervals-1]=m_Tsingle*3.0;
    m_StepTypes[m_NumberOfIntervals-1]=DOUBLE_SUPPORT;
    ComputePreviewControlTimeWindow();

    if (m_VerboseLevel>=2)
      {
        double total=0;
        for(int i=0; i<m_NumberOfIntervals; i++)
          {
            cout << setprecision(12) << m_DeltaTj[i] << " ";
            total+= m_DeltaTj[i];
          }
        cout << " total: " << total <<endl;
      }

    // Specify the degrees corresponding to the given interval.
    m_PolynomialDegrees.resize(m_NumberOfIntervals);
    m_PolynomialDegrees[0] = 4;
    m_PolynomialDegrees[m_NumberOfIntervals-1] = 4;
    for(int i=1; i<m_NumberOfIntervals-1; i++)
      m_PolynomialDegrees[i] = 3;

    /*! Dynamic allocation for the foot trajectory. */
    if(m_FeetTrajectoryGenerator!=0)
      {
        m_FeetTrajectoryGenerator->SetDeltaTj(m_DeltaTj);
      }
    return true;
  }

  void AnalyticalMorisawaCompact::ComputePolynomialWeights()
  {
    Eigen::MatrixXd iZ;
    iZ=m_Z.inverse();

    // Compute the weights.
    m_y=iZ+m_w;

    if (m_VerboseLevel>=2)
      {
        std::ofstream ofs;
        ofs.open("YMatrix.dat",ofstream::out);
        ofs.precision(10);

        for(unsigned int i=0; i<m_y.size(); i++)
          {
            ofs << m_y[i]<< " ";
          }
        ofs << endl;
        ofs.close();
      }

  }

  void AnalyticalMorisawaCompact::ResetTheResolutionOfThePolynomial()
  {
    long int SizeOfZ = m_Z.rows();

    m_AF.resize(SizeOfZ,2*SizeOfZ);
    m_IPIV.resize(SizeOfZ);
    m_AF.setZero();
    m_IPIV.setZero();

    m_NeedToReset = true;
  }

  void AnalyticalMorisawaCompact::ComputePolynomialWeights2()
  {
    int SizeOfZ = (int)m_Z.rows(),
      LDA,LDAF,LDB;
    int NRHS = 1;

    char EQUED='N';

    Eigen::MatrixXd tZ;
    tZ = m_Z.transpose();


    Eigen::VectorXd lR;
    lR.resize(SizeOfZ);
    Eigen::VectorXd lC;
    lC.resize(SizeOfZ);

    m_y.resize(SizeOfZ);
    //Eigen::VectorXd m_X;
    //b m_X.resize(SizeOfZ);
    LDA = SizeOfZ;
    LDAF = SizeOfZ;
    LDB = SizeOfZ;

    double lRCOND;

    Eigen::VectorXd lFERR, lBERR;
    lFERR.resize(SizeOfZ);
    lBERR.resize(SizeOfZ);

    int lwork = 4* SizeOfZ;
    double *work = new double[lwork];
    int *iwork = new int[SizeOfZ];
    int lsizeofx= SizeOfZ;
    int info=0;

    if (m_NeedToReset)
      {
        m_AF = m_Z.transpose();
        dgetrf_(&SizeOfZ, /* M */
                &SizeOfZ, /* N here M=N=SizeOfZ */
                &m_AF(0), /* A */
                &SizeOfZ, /* Leading dimension cf before */
                &m_IPIV(0), /* IPIV */
                &info /* info */
                );
        m_NeedToReset = false;
      }

    char lF[2]="F";
    char lN[2]="N";
    dgesvx_(lF, /* Specify that AF and IPIV should be used. */
            lN, /* A * X = B */
            &SizeOfZ, /* Size of A */
            &NRHS, /*Nb of columns for X et B */
            &tZ(0), /* Access to A */
            &LDA, /* Leading size of A */
            &m_AF(0),
            &LDAF,
            &m_IPIV(0),
            &EQUED,
            &lR(0),
            &lC(0),
            &m_w(0),
            &LDB,
            &m_y(0),
            &lsizeofx,
            &lRCOND,
            &lFERR(0),
            &lBERR(0),
            work,
            iwork,
            &info
            );

    // Compute the weights.
    // m_y=iZ+m_w;

    if (m_VerboseLevel>=2)
      {
        std::ofstream ofs;
        ofs.open("YMatrix.dat",ofstream::out);
        ofs.precision(10);

        for(unsigned int i=0; i<m_y.size(); i++)
          {
            ofs << m_y[i]<< " ";
          }
        ofs << endl;
        ofs.close();
      }

    delete [] work ;
    delete [] iwork ;
  }


  int AnalyticalMorisawaCompact::
  BuildAndSolveCOMZMPForASetOfSteps
  (Eigen::Matrix3d &lStartingCOMState,
   FootAbsolutePosition &LeftFootInitialPosition,
   FootAbsolutePosition &RightFootInitialPosition,
   bool IgnoreFirstRelativeFoot,
   bool DoNotPrepareLastFoot)
  {

    if (m_RelativeFootPositions.size()==0)
      return -2;

    int NbSteps = (int)m_RelativeFootPositions.size();
    int NbOfIntervals=2*NbSteps+1;

    SetNumberOfStepsInAdvance(NbSteps);
    InitializeBasicVariables();

    vector<double> * lCoMZ;
    vector<double> * lZMPZ;

    lCoMZ = & m_CTIPX.CoMZ;
    lZMPZ = & m_CTIPX.ZMPZ;

    lCoMZ->resize(NbOfIntervals);
    lZMPZ->resize(NbOfIntervals);
    for(int i=0; i<NbOfIntervals; i++)
      {
        (*lCoMZ)[i] = lStartingCOMState(2,0);
        (*lZMPZ)[i] = 0.0;
      }

    /*! Build the Z Matrix. */
    BuildingTheZMatrix(*lCoMZ,*lZMPZ);

    ResetTheResolutionOfThePolynomial();

    /*! Initialize correctly the analytical trajectories. */
    m_AnalyticalZMPCoGTrajectoryX->SetNumberOfIntervals(NbOfIntervals);
    m_AnalyticalZMPCoGTrajectoryY->SetNumberOfIntervals(NbOfIntervals);

    m_AnalyticalZMPCoGTrajectoryX->SetPolynomialDegrees(m_PolynomialDegrees);
    m_AnalyticalZMPCoGTrajectoryY->SetPolynomialDegrees(m_PolynomialDegrees);

    m_AnalyticalZMPCoGTrajectoryX->
      SetStartingTimeIntervalsAndHeightVariation(m_DeltaTj,m_Omegaj);
    m_AnalyticalZMPCoGTrajectoryY->
      SetStartingTimeIntervalsAndHeightVariation(m_DeltaTj,m_Omegaj);

    /* Build the profil for the X and Y axis. */
    double InitialCoMX=0.0;
    double InitialCoMSpeedX=0.0;
    double FinalCoMPosX=0.6;
    vector<double> * lZMPX=0;
    lZMPX = &m_CTIPX.ZMPProfil;

    lZMPX->resize(NbOfIntervals);

    double InitialCoMY=0.0;
    double InitialCoMSpeedY=0.0;
    double FinalCoMPosY=0.0;
    vector<double> * lZMPY=0;
    lZMPY = &m_CTIPY.ZMPProfil;

    lZMPY->resize(NbOfIntervals);

    (*lZMPX)[0] = lStartingCOMState(0,0);
    (*lZMPY)[0] = lStartingCOMState(1,0);

    /*! Extract the set of initial conditions relevant for
      computing the analytical trajectories. */
    InitialCoMX = (*lZMPX)[0];
    InitialCoMSpeedX = lStartingCOMState(0,1);
    InitialCoMY = (*lZMPY)[0];
    InitialCoMSpeedY = lStartingCOMState(1,1);

    /*! Extract the set of absolute coordinates for the foot position. */
    if (m_FeetTrajectoryGenerator!=0)
      {
        m_FeetTrajectoryGenerator->SetDeltaTj(m_DeltaTj);
        m_FeetTrajectoryGenerator->InitializeFromRelativeSteps
          (m_RelativeFootPositions,
           LeftFootInitialPosition,
           RightFootInitialPosition,
           m_AbsoluteSupportFootPositions,
           IgnoreFirstRelativeFoot, false);
        unsigned int i=0,j=1;

        for(i=0,j=1; i<m_AbsoluteSupportFootPositions.size(); i++,j+=2)
          {
            (*lZMPX)[j] = m_AbsoluteSupportFootPositions[i].x;
            (*lZMPX)[j+1] = m_AbsoluteSupportFootPositions[i].x;

            (*lZMPY)[j] = m_AbsoluteSupportFootPositions[i].y;
            (*lZMPY)[j+1] = m_AbsoluteSupportFootPositions[i].y;
          }


        // Strategy for the final CoM pos: middle of the segment
        // between the two final steps, in order to be statically stable.
        unsigned int lindex = (unsigned int)
          (m_AbsoluteSupportFootPositions.size()-1);

        if (DoNotPrepareLastFoot)
          FinalCoMPosX = m_AbsoluteSupportFootPositions[lindex].x;
        else
          FinalCoMPosX = 0.5 *(m_AbsoluteSupportFootPositions[lindex-1].x +
                               m_AbsoluteSupportFootPositions[lindex].x);
        if (DoNotPrepareLastFoot)
          (*lZMPX)[j-2] = (*lZMPX)[j-1] =
            m_AbsoluteSupportFootPositions[lindex].x;
        else
          (*lZMPX)[j-2] = (*lZMPX)[j-1] = FinalCoMPosX;

        if (DoNotPrepareLastFoot)
          FinalCoMPosY = m_AbsoluteSupportFootPositions[lindex].y;
        else
          FinalCoMPosY = 0.5 *(m_AbsoluteSupportFootPositions[lindex-1].y +
                               m_AbsoluteSupportFootPositions[lindex].y);

        if (DoNotPrepareLastFoot)
          (*lZMPY)[j-2] = (*lZMPY)[j-1] =
            m_AbsoluteSupportFootPositions[lindex].y;
        else
          (*lZMPY)[j-2] = (*lZMPY)[j-1] = FinalCoMPosY;
      }
    else
      {
        std::cerr << " Feet Trajectory Generator NOT INITIALIZED"
                  << std::endl;
        return -1;
      }

    /*! Build 3rd order polynomials. */
    for(int i=1; i<NbOfIntervals-1; i++)
      {
        m_AnalyticalZMPCoGTrajectoryX->
          Building3rdOrderPolynomial(i,(*lZMPX)[i-1],
                                     (*lZMPX)[i]);
        m_AnalyticalZMPCoGTrajectoryY->
          Building3rdOrderPolynomial(i,(*lZMPY)[i-1],
                                     (*lZMPY)[i]);
      }

    // Block for X trajectory
    m_CTIPX.InitialCoM = InitialCoMX;
    m_CTIPX.InitialCoMSpeed = InitialCoMSpeedX;
    m_CTIPX.FinalCoMPos = FinalCoMPosX;
    //m_CTIPX.ZMPProfil = lZMPX;
    // m_CTIPX.ZMPZ = lZMPZ;
    // m_CTIPX.CoMZ = lCoMZ;
    ComputeTrajectory(m_CTIPX,*m_AnalyticalZMPCoGTrajectoryX);

    // Block for Y trajectory.
    m_CTIPY.InitialCoM = InitialCoMY;
    m_CTIPY.InitialCoMSpeed = InitialCoMSpeedY;
    m_CTIPY.FinalCoMPos = FinalCoMPosY;
    //m_CTIPY.ZMPProfil = lZMPY;
    m_CTIPY.ZMPZ = *lZMPZ;
    m_CTIPY.CoMZ = *lCoMZ;
    ComputeTrajectory(m_CTIPY,*m_AnalyticalZMPCoGTrajectoryY);


    return 0;

  }

  void AnalyticalMorisawaCompact::
  GetZMPDiscretization
  (deque<ZMPPosition> &
   ZMPPositions,
   deque<COMState> & COMStates,
   deque<RelativeFootPosition> &RelativeFootPositions,
   deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
   deque<FootAbsolutePosition> &RightFootAbsolutePositions,
   double,
   COMState & lStartingCOMState,
   Eigen::Vector3d &,
   FootAbsolutePosition & InitLeftFootAbsolutePosition,
   FootAbsolutePosition & InitRightFootAbsolutePosition)
  {
    // INITIALIZE FEET POSITIONS:
    // --------------------------
    Eigen::Vector3d lAnklePositionRight,lAnklePositionLeft;
    PRFoot *LeftFoot, *RightFoot;
    LeftFoot = m_PR->leftFoot();
    if (LeftFoot==0)
      LTHROW("No left foot");

    RightFoot = m_PR->rightFoot();
    if (RightFoot==0)
      LTHROW("No right foot");

    lAnklePositionLeft = LeftFoot->anklePosition ;
    lAnklePositionRight = RightFoot->anklePosition ;

    Eigen::Matrix4d CurPosWICF_homogeneous ;
    CurPosWICF_homogeneous = m_kajitaDynamicFilter->getComAndFootRealization()
      ->GetCurrentPositionofWaistInCOMFrame();

    m_RelativeFootPositions = RelativeFootPositions;
    /* This part computes the CoM and ZMP trajectory giving 
       the foot position information.
       It also creates the analytical feet trajectories.
    */
    Eigen::Matrix3d lMStartingCOMState;

    lMStartingCOMState(0,0)= lStartingCOMState.x[0];
    lMStartingCOMState(1,0)= lStartingCOMState.y[0];
    lMStartingCOMState(2,0)= lStartingCOMState.z[0];

    m_InitialPoseCoMHeight = lMStartingCOMState(2,0);

    for(unsigned int i=0; i<3; i++)
      {
        for(unsigned int j=1; j<3; j++)
          lMStartingCOMState(i,j)= 0.0;
      }

    int r=0;
    if ((r=BuildAndSolveCOMZMPForASetOfSteps(lMStartingCOMState,
                                             InitLeftFootAbsolutePosition,
                                             InitRightFootAbsolutePosition,
                                             true,false))<0)
      {
        switch(r)
          {
          case (-1):
            LTHROW("Error: Humanoid Specificities not initialized. ");
            break;
          case (-2):
            LTHROW("Error: Relative Foot Size" );
            break;
          }
        return;
      }

    /*! Set the current time reference for the analytical trajectory. */
    double TimeShift = m_Tsingle*2;
    m_AbsoluteTimeReference = m_CurrentTime-TimeShift;
    m_AnalyticalZMPCoGTrajectoryX->
      SetAbsoluteTimeReference(m_AbsoluteTimeReference);
    m_AnalyticalZMPCoGTrajectoryY->
      SetAbsoluteTimeReference(m_AbsoluteTimeReference);
    m_FeetTrajectoryGenerator->
      SetAbsoluteTimeReference(m_AbsoluteTimeReference);

    /*! Compute the total size of the array related to the steps. */
    FillQueues(m_CurrentTime,m_CurrentTime+m_PreviewControlTime-TimeShift,
               ZMPPositions, COMStates,LeftFootAbsolutePositions,
               RightFootAbsolutePositions);

    bool filterOn_ = true ;
    if(filterOn_)
      {
        /*! initialize the dynamic filter */
        unsigned int n =(unsigned int) COMStates.size();
        double KajitaPCpreviewWindow = 1.6 ;
        m_kajitaDynamicFilter->init( m_SamplingPeriod,
                                     m_SamplingPeriod,
                                     n*m_SamplingPeriod,
                                     m_PreviewControlTime-
                                     TimeShift+KajitaPCpreviewWindow,
                                     KajitaPCpreviewWindow,
                                     lStartingCOMState );
        /*! Set the upper body trajectory */
        Eigen::VectorXd UpperConfig = m_PR->currentRPYConfiguration() ;
        Eigen::VectorXd UpperVel    = m_PR->currentRPYVelocity() ;
        Eigen::VectorXd UpperAcc    = m_PR->currentRPYAcceleration() ;
        // carry the weight in front of him
        //        UpperConfig(18)= 0.0 ;            // CHEST_JOINT0
        //        UpperConfig(19)= 0.015 ;          // CHEST_JOINT1
        //        UpperConfig(20)= 0.0 ;            // HEAD_JOINT0
        //        UpperConfig(21)= 0.0 ;            // HEAD_JOINT1
        //        UpperConfig(22)= -0.108210414 ;   // RARM_JOINT0
        //        UpperConfig(23)= 0.0383972435 ;   // RARM_JOINT1
        //        UpperConfig(24)= 0.474729557 ;    // RARM_JOINT2
        //        UpperConfig(25)= -1.41720735 ;    // RARM_JOINT3
        //        UpperConfig(26)= 1.45385927 ;     // RARM_JOINT4
        //        UpperConfig(27)= 0.509636142 ;    // RARM_JOINT5
        //        UpperConfig(28)= 0.174532925 ;    // RARM_JOINT6
        //        UpperConfig(29)= -0.108210414 ;   // LARM_JOINT0
        //        UpperConfig(30)= -0.129154365 ;   // LARM_JOINT1
        //        UpperConfig(31)= -0.333357887 ;   // LARM_JOINT2
        //        UpperConfig(32)= -1.41720735 ;    // LARM_JOINT3
        //        UpperConfig(33)= 1.45385927 ;     // LARM_JOINT4
        //        UpperConfig(34)= -0.193731547 ;   // LARM_JOINT5
        //        UpperConfig(35)= 0.174532925 ;    // LARM_JOINT6

        //    // carry the weight over the head
        //    UpperConfig(18)= 0.0 ;            // CHEST_JOINT0
        //    UpperConfig(19)= 0.015 ;          // CHEST_JOINT1
        //    UpperConfig(20)= 0.0 ;            // HEAD_JOINT0
        //    UpperConfig(21)= 0.0 ;            // HEAD_JOINT1
        //    UpperConfig(22)= -1.4678219 ;     // RARM_JOINT0
        //    UpperConfig(23)= 0.0366519143 ;   // RARM_JOINT1
        //    UpperConfig(24)= 0.541052068 ;    // RARM_JOINT2
        //    UpperConfig(25)= -1.69296937 ;    // RARM_JOINT3
        //    UpperConfig(26)= 1.56556034 ;     // RARM_JOINT4
        //    UpperConfig(27)= 0.584685299 ;    // RARM_JOINT5
        //    UpperConfig(28)= 0.174532925 ;    // RARM_JOINT6
        //    UpperConfig(29)= -1.4678219 ;     // LARM_JOINT0
        //    UpperConfig(30)= -0.0366519143 ;  // LARM_JOINT1
        //    UpperConfig(31)= -0.541052068 ;   // LARM_JOINT2
        //    UpperConfig(32)= -1.69296937 ;    // LARM_JOINT3
        //    UpperConfig(33)= -1.56556034 ;     // LARM_JOINT4
        //    UpperConfig(34)= 0.584685299 ;    // LARM_JOINT5
        //    UpperConfig(35)= 0.174532925 ;    // LARM_JOINT6

        for(unsigned int i = 18 ; i < 35 ; ++i)
          {
            UpperConfig(i)=m_PR->currentRPYConfiguration()(i);
            UpperVel(i)=0.0;
            UpperAcc(i)=0.0;
          }

        m_kajitaDynamicFilter->setRobotUpperPart(UpperConfig,UpperVel,UpperAcc);

        /*! Add "KajitaPCpreviewWindow" second to the buffers for fitering */
        ZMPPosition lastZMP = ZMPPositions.back();
        COMState lastCoM = COMStates.back();
        FootAbsolutePosition lastLF = LeftFootAbsolutePositions.back();
        FootAbsolutePosition lastRF = RightFootAbsolutePositions.back();
        for (unsigned int i = 0  ;
             i < KajitaPCpreviewWindow/m_SamplingPeriod ;
             ++i)
          {
            ZMPPositions.push_back(lastZMP);
            COMStates.push_back(lastCoM);
            LeftFootAbsolutePositions.push_back(lastLF);
            RightFootAbsolutePositions.push_back(lastRF);
          }

        for (unsigned int i = 0 ; i < COMStates.size() ; ++i )
          {
            COMStates[i].roll[0]  = 180/M_PI* COMStates[i].roll[0] ;
            COMStates[i].pitch[0] = 180/M_PI* COMStates[i].pitch[0] ;
            COMStates[i].yaw[0]   = 180/M_PI* COMStates[i].yaw[0] ;
          }

        // Filter the trajectory
        deque<COMState> outputDeltaCOMTraj_deq (n) ;
        m_kajitaDynamicFilter->
          OffLinefilter
          (COMStates,
           ZMPPositions,
           LeftFootAbsolutePositions,
           RightFootAbsolutePositions,
           vector< Eigen::VectorXd > (1,UpperConfig),
           vector< Eigen::VectorXd > (1,UpperVel),
           vector< Eigen::VectorXd > (1,UpperAcc),
           outputDeltaCOMTraj_deq);
        
#ifdef DEBUG
        m_kajitaDynamicFilter->Debug(COMStates,LeftFootAbsolutePositions,
                                     RightFootAbsolutePositions,
                                     COMStates,ZMPPositions,
                                     LeftFootAbsolutePositions,
                                     RightFootAbsolutePositions,
                                     outputDeltaCOMTraj_deq);
#endif
        vector <vector<double> > filteredZMPMB (n, vector<double> (2,0.0)) ;
        for (unsigned int i = 0 ; i < n ; ++i)
          {
            for(int j=0; j<3; j++)
              {
                COMStates[i].x[j] += outputDeltaCOMTraj_deq[i].x[j] ;
                COMStates[i].y[j] += outputDeltaCOMTraj_deq[i].y[j] ;
              }
          }

        for (unsigned int i = 0  ;
             i < KajitaPCpreviewWindow/m_SamplingPeriod ;
             ++i)
          {
            ZMPPositions.pop_back();
            COMStates.pop_back();
            LeftFootAbsolutePositions.pop_back();
            RightFootAbsolutePositions.pop_back();
          }
      }
    // End the Filtering

    m_UpperTimeLimitToUpdateStacks = m_CurrentTime;
    for(int i=0; i<m_NumberOfIntervals; i++)
      {
        m_UpperTimeLimitToUpdateStacks += m_DeltaTj[i];
      }
  }

  std::size_t AnalyticalMorisawaCompact::
  InitOnLine
  (deque<ZMPPosition> &
   FinalZMPPositions,
   deque<COMState> & FinalCoMPositions,
   deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
   deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
   FootAbsolutePosition & InitLeftFootAbsolutePosition,
   FootAbsolutePosition & InitRightFootAbsolutePosition,
   deque<RelativeFootPosition> &RelativeFootPositions,
   COMState & lStartingCOMState,
   Eigen::Vector3d &)
  {
    m_OnLineMode = true;
    m_RelativeFootPositions.clear();

    unsigned int r = (unsigned int)RelativeFootPositions.size();
    unsigned int maxrelsteps = r < 3 ? r : 3;

    // INITIALIZE FEET POSITIONS:
    // --------------------------
    Eigen::Vector3d lAnklePositionRight,lAnklePositionLeft;
    PRFoot *LeftFoot, *RightFoot;
    LeftFoot = m_PR->leftFoot();
    if (LeftFoot==0)
      LTHROW("No left foot");

    RightFoot = m_PR->rightFoot();
    if (RightFoot==0)
      LTHROW("No right foot");

    lAnklePositionLeft  = LeftFoot->anklePosition ;
    lAnklePositionRight = RightFoot->anklePosition ;

    Eigen::Matrix4d CurPosWICF_homogeneous ;
    CurPosWICF_homogeneous =
      m_kajitaDynamicFilter->getComAndFootRealization()
      ->GetCurrentPositionofWaistInCOMFrame();

    InitLeftFootAbsolutePosition.x +=  lAnklePositionLeft(0)  ;
    InitLeftFootAbsolutePosition.y +=  lAnklePositionLeft(1)  ;
    InitLeftFootAbsolutePosition.z +=  lAnklePositionLeft(2)  ;
    InitRightFootAbsolutePosition.x += lAnklePositionRight(0) ;
    InitRightFootAbsolutePosition.y += lAnklePositionRight(1) ;
    InitRightFootAbsolutePosition.z += lAnklePositionRight(2) ;

    // INITIALIZE THE COM
    // ------------------
    Eigen::Matrix3d lMStartingCOMState;

    lMStartingCOMState(0,0)= lStartingCOMState.x[0];
    lMStartingCOMState(1,0)= lStartingCOMState.y[0];
    lMStartingCOMState(2,0)= lStartingCOMState.z[0];

    m_InitialPoseCoMHeight = lStartingCOMState.z[0];

    for(unsigned int i=0; i<3; i++)
      {
        for(unsigned int j=1; j<3; j++)
          lMStartingCOMState(i,j)= 0.0;
      }

    for(unsigned int i=0; i< maxrelsteps; i++)
      m_RelativeFootPositions.push_back(RelativeFootPositions[i]);

    if (m_RelativeFootPositions[0].sy < 0)
      m_AbsoluteCurrentSupportFootPosition = InitRightFootAbsolutePosition;
    else
      m_AbsoluteCurrentSupportFootPosition = InitLeftFootAbsolutePosition;

    /* This part computes the CoM and ZMP trajectory 
       giving the foot position information.
       It also creates the analytical feet trajectories.
    */
    if (BuildAndSolveCOMZMPForASetOfSteps(lMStartingCOMState,
                                          InitLeftFootAbsolutePosition,
                                          InitRightFootAbsolutePosition,
                                          true,true)<0)
      {
        LTHROW("Error: Humanoid Specificities not initialized. ");
      }

    m_AbsoluteTimeReference = m_CurrentTime-m_Tsingle*2;
    m_AnalyticalZMPCoGTrajectoryX->
      SetAbsoluteTimeReference(m_AbsoluteTimeReference);
    m_AnalyticalZMPCoGTrajectoryY->
      SetAbsoluteTimeReference(m_AbsoluteTimeReference);
    m_FeetTrajectoryGenerator->
      SetAbsoluteTimeReference(m_AbsoluteTimeReference);

    /* Current strategy : add 2 values, and update at each iteration the stack.
       When the limit is reached, and the stack exhausted this method is called
       again.  */
    FillQueues(m_CurrentTime,
               m_CurrentTime+2*m_SamplingPeriod,
               FinalZMPPositions,
               FinalCoMPositions,
               FinalLeftFootAbsolutePositions,
               FinalRightFootAbsolutePositions);

    for(unsigned i=0; i<FinalCoMPositions.size(); ++i)
      {
        FinalCoMPositions[i].z[0]=lStartingCOMState.z[0];
        FinalCoMPositions[i].z[1]=lStartingCOMState.z[1];
        FinalCoMPositions[i].z[2]=lStartingCOMState.z[2];
      }

    /*! Recompute time when a new step should be added. */
    m_UpperTimeLimitToUpdateStacks = m_AbsoluteTimeReference + m_DeltaTj[0] +
      m_Tdble + 0.45 * m_Tsingle;

    double previewWindowSize = 0.8 ;
    double controlWindowSize = 0.005 ;
    double interpolationPeriod = 0.05 ;
    DFpreviewWindowSize_ = previewWindowSize - interpolationPeriod ; //second
    m_kajitaDynamicFilter->getComAndFootRealization()->ShiftFoot(false);
    m_kajitaDynamicFilter->init( m_SamplingPeriod,
                                 interpolationPeriod,
                                 controlWindowSize,
                                 previewWindowSize,
                                 DFpreviewWindowSize_,
                                 lStartingCOMState );


    return (int) m_RelativeFootPositions.size();

  }


  void AnalyticalMorisawaCompact::
  OnLine(double time,
         deque<ZMPPosition> & FinalZMPPositions,
         deque<COMState> & FinalCOMStates,
         deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
         deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)
  {
    unsigned int lIndexInterval;
    if (time<m_UpperTimeLimitToUpdateStacks)
      {
        if (m_AnalyticalZMPCoGTrajectoryX->
            GetIntervalIndexFromTime(time,
                                     lIndexInterval))
          {

            ZMPPosition aZMPPos;
            memset(&aZMPPos,0,sizeof(aZMPPos));
            COMState aCOMPos;
            memset(&aCOMPos,0,sizeof(aCOMPos));

            if (m_FilteringActivate)
              {
                double FZmpX=0, FComX=0,FComdX=0;

                // Should we filter ?
                bool r = m_FilterXaxisByPC->
                  UpdateOneStep(time,FZmpX, FComX, FComdX);
                if (r)
                  {
                    double FZmpY=0, FComY=0,FComdY=0;
                    // Yes we should.
                    m_FilterYaxisByPC->UpdateOneStep(time,FZmpY, FComY, FComdY);

                    /*! Feed the ZMPPositions. */
                    aZMPPos.px = FZmpX;
                    aZMPPos.py = FZmpY;

                    /*! Feed the COMStates. */
                    aCOMPos.x[0] = FComX;
                    aCOMPos.x[1] = FComdX;
                    aCOMPos.y[0] = FComY;
                    aCOMPos.y[1] = FComdY;
                  }
              }


            /*! Feed the ZMPPositions. */
            double lZMPPosx=0.0,lZMPPosy=0.0;
            m_AnalyticalZMPCoGTrajectoryX->
              ComputeZMP(time,lZMPPosx,lIndexInterval);
            aZMPPos.px += lZMPPosx;
            m_AnalyticalZMPCoGTrajectoryY->
              ComputeZMP(time,lZMPPosy,lIndexInterval);
            aZMPPos.py += lZMPPosy;
            FinalZMPPositions.push_back(aZMPPos);

            /*! Feed the COMStates. */
            double lCOMPosx=0.0, lCOMPosdx=0.0;
            double lCOMPosy=0.0, lCOMPosdy=0.0;
            m_AnalyticalZMPCoGTrajectoryX->
              ComputeCOM(time,lCOMPosx,lIndexInterval);
            m_AnalyticalZMPCoGTrajectoryX->
              ComputeCOMSpeed(time,lCOMPosdx,lIndexInterval);
            m_AnalyticalZMPCoGTrajectoryY->
              ComputeCOM(time,lCOMPosy,lIndexInterval);
            m_AnalyticalZMPCoGTrajectoryY->
              ComputeCOMSpeed(time,lCOMPosdy,lIndexInterval);
            aCOMPos.x[0] += lCOMPosx;
            aCOMPos.x[1] += lCOMPosdx;
            aCOMPos.y[0] += lCOMPosy;
            aCOMPos.y[1] += lCOMPosdy;
            aCOMPos.z[0] = m_InitialPoseCoMHeight;
            FinalCOMStates.push_back(aCOMPos);
            /*! Feed the FootPositions. */


            /*! Left */
            FootAbsolutePosition LeftFootAbsPos;
            memset(&LeftFootAbsPos,0,sizeof(LeftFootAbsPos));
            m_FeetTrajectoryGenerator->
              ComputeAnAbsoluteFootPosition
              (1,time,LeftFootAbsPos,
               lIndexInterval);
            FinalLeftFootAbsolutePositions.push_back(LeftFootAbsPos);

            /*! Right */
            FootAbsolutePosition RightFootAbsPos;
            memset(&RightFootAbsPos,0,sizeof(RightFootAbsPos));
            m_FeetTrajectoryGenerator->
              ComputeAnAbsoluteFootPosition
              (-1,time,
               RightFootAbsPos,lIndexInterval);
            FinalRightFootAbsolutePositions.push_back(RightFootAbsPos);

          }
      }
    else
      {
        /*! We reached the end of the trajectory generated
          and no foot steps have been added. */
        m_OnLineMode = false;
      }
  }

  void AnalyticalMorisawaCompact::
  OnLineAddFoot(RelativeFootPosition &
                NewRelativeFootPosition,
                deque<ZMPPosition> & FinalZMPPositions,
                deque<COMState> & FinalCoMPositions,
                deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
                deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
                bool )
  {
    ODEBUG("****************** Begin OnLineAddFoot **************************");
    unsigned int StartingIndexInterval;
    m_AnalyticalZMPCoGTrajectoryX->
      GetIntervalIndexFromTime(m_CurrentTime,
                               StartingIndexInterval);

    unsigned int IndexInterval = (unsigned int)(m_CTIPX.ZMPProfil.size()-1);

    /* If the interval detected is not a double support interval,
       a shift is done to chose the earliest double support interval. */
    vector<unsigned int> IndexLastZMPProfil;
    IndexLastZMPProfil.resize(1);
    IndexLastZMPProfil[0] = IndexInterval;

    // The strategy is simple: we trigger a false modification of the last
    // step and call change landing position, just after updating the stack
    // of relative foot positions.

    m_Clock1.StartTiming();

    // Update the stack of relative foot positions.
    m_RelativeFootPositions.pop_front();
    m_RelativeFootPositions.push_back(NewRelativeFootPosition);

    deque<FootAbsolutePosition> aQAFP;

    m_FeetTrajectoryGenerator->
      ComputeAbsoluteStepsFromRelativeSteps
      (m_RelativeFootPositions,
       FinalLeftFootAbsolutePositions[0],
       FinalRightFootAbsolutePositions[0],
       aQAFP);

    vector<FootAbsolutePosition> aNewFootAbsPos;
    aNewFootAbsPos.resize(1);
    aNewFootAbsPos[0]=aQAFP.back();

    if (FinalLeftFootAbsolutePositions[0].z==0.0)
      m_AbsoluteCurrentSupportFootPosition = FinalLeftFootAbsolutePositions[0];
    else
      m_AbsoluteCurrentSupportFootPosition = FinalRightFootAbsolutePositions[0];

    m_AbsoluteSupportFootPositions.pop_front();
    m_AbsoluteSupportFootPositions.push_back(aNewFootAbsPos[0]);

    /* Indicates that the step has to be taken into account appropriatly
       to compute the trajectory. */
    m_NewStepInTheStackOfAbsolutePosition = true;

    m_Clock1.StopTiming();
    m_Clock1.IncIteration(1);

    m_Clock2.StartTiming();

    /* Realize the foot changing position during the last interval */
    bool lResetFilters = false;
    bool lTemporalShift = false;
    bool lAddingAFootStep = true;
    ChangeFootLandingPosition(m_CurrentTime,
                              IndexLastZMPProfil,
                              aNewFootAbsPos,
                              *m_AnalyticalZMPCoGTrajectoryX,
                              m_CTIPX,
                              *m_AnalyticalZMPCoGTrajectoryY,
                              m_CTIPY,
                              lTemporalShift,lResetFilters,
                              0, lAddingAFootStep);

    /* Indicates that the step has been taken into account appropriatly
       in computing the trajectory. */
    m_NewStepInTheStackOfAbsolutePosition = false;

    m_Clock2.StopTiming();
    m_Clock2.IncIteration(1);

    m_Clock3.StartTiming();

    /* Current strategy : add 2 values, and update at each iteration the stack.
       When the limit is reached, and the stack exhausted this method is called 
       again. */
    FillQueues(m_AbsoluteTimeReference,
               m_AbsoluteTimeReference+2*m_SamplingPeriod,
               FinalZMPPositions,
               FinalCoMPositions,
               FinalLeftFootAbsolutePositions,
               FinalRightFootAbsolutePositions);

    for(unsigned i=0; i<FinalCoMPositions.size(); ++i)
      {
        FinalCoMPositions[i].z[0]=m_InitialPoseCoMHeight;
        FinalCoMPositions[i].z[1]=0.0;
        FinalCoMPositions[i].z[2]=0.0;
      }

    m_Clock3.StopTiming();
    m_Clock3.IncIteration();

    /* Update the time at which the stack should not be updated anymore */
    m_UpperTimeLimitToUpdateStacks = m_AbsoluteTimeReference + m_DeltaTj[0] +
      m_Tdble + 0.45 * m_Tsingle;
    ODEBUG("****************** End OnLineAddFoot **************************");
  }



  void AnalyticalMorisawaCompact::ComputeW(double InitialCoMPos,
                                           double InitialCoMSpeed,
                                           vector<double> &ZMPPosSequence,
                                           double FinalCoMPos,
                                           AnalyticalZMPCOGTrajectory & aAZCT)
  {
    unsigned int lindex = 0;

    // Again assuming that the number of unknowns
    // is based on a 4- (m-2) 3 -4 th order polynomials.
    m_w.resize( 2 * m_NumberOfIntervals + 6);

    // Initial CoM Position
    m_w(lindex)= InitialCoMPos - ZMPPosSequence[0];
    lindex++;
    // Initial CoM Speed
    m_w(lindex) = InitialCoMSpeed;
    lindex++;

    // Just be carefull:
    // at iteration j, we still build m_w for the
    // interval j-1.
    for(int j=1; j<m_NumberOfIntervals; j++)
      {
        // Takes back the polynomial needed to compute m_w.
        Polynome *aPolynomeNext,*aPolynome;
        aAZCT.GetFromListOfCOGPolynomials(j,aPolynomeNext);
        vector<double> NextCoeffsFromCOG, CoeffsFromCOG;
        aPolynomeNext->GetCoefficients(NextCoeffsFromCOG);
        if (j==1)
          {
            m_w[lindex] = NextCoeffsFromCOG[0] - ZMPPosSequence[0];
            lindex++;
            m_w[lindex] = NextCoeffsFromCOG[1];
            lindex++;
            m_w[lindex] = 0;//ZMPPosSequence[0] - ZMPPosSequence[0];
            lindex++;
            m_w[lindex] = 0;
            lindex++;
          }
        else
          {
            aAZCT.GetFromListOfCOGPolynomials(j-1,aPolynome);
            aPolynome->GetCoefficients(CoeffsFromCOG);
            double r1=0.0,r2=0.0;
            double deltat1=1.0,deltat2=1.0;
            for (unsigned int k=0; k<CoeffsFromCOG.size(); k++)
              {
                r1+= CoeffsFromCOG[k]* deltat1;
                deltat1 *= m_DeltaTj[j-1];
                if (k>0)
                  {
                    r2+= k * CoeffsFromCOG[k]* deltat2;
                    deltat2 *= m_DeltaTj[j-1];
                  }
              }
            if (j!=m_NumberOfIntervals-1)
              {
                m_w[lindex] = NextCoeffsFromCOG[0] - r1;
              }
            else
              {
                m_w[lindex] = ZMPPosSequence[j-1] - r1;
              }
            lindex++;
            if (j!=m_NumberOfIntervals-1)
              {
                m_w[lindex] = NextCoeffsFromCOG[1] - r2;
              }
            else
              {
                m_w[lindex] = - r2;
              }

            lindex++;
          }
      }

    // Compute the last part of w for the last interval.

    // Constraints on the final value of the CoM position
    if (m_NumberOfIntervals>1)
      {
        m_w[lindex] = FinalCoMPos - ZMPPosSequence[m_NumberOfIntervals-2];
        lindex++;
        // and the CoM speed.
        m_w[lindex] = 0;
        lindex++;

        // Position of the ZMP.
        m_w[lindex] = FinalCoMPos - ZMPPosSequence[m_NumberOfIntervals-2];
        lindex++;
        // and the ZMP speed.
        m_w[lindex] = 0.0;
        lindex++;
      }

    if (m_VerboseLevel>=2)
      {
        std::ofstream ofs;
        ofs.open("WCompactMatrix.dat",ofstream::out);
        ofs.precision(10);

        for(unsigned int i=0; i<m_w.size(); i++)
          {
            ofs << m_w[i]<< " ";
          }
        ofs << endl;
        ofs.close();
      }

  }

  void AnalyticalMorisawaCompact::ComputeZ1( unsigned int &rowindex)
  {
    double SquareOmega0 = m_Omegaj[0] * m_Omegaj[0];

    // First Row: Connection of the position of the CoM
    double c0,s0;
    double Deltat = m_DeltaTj[0] * m_DeltaTj[0];

    // A_2^(0) coefficient + A_1^(0) express as a function of A_2^(0)
    m_Z(rowindex,0) = Deltat + 2.0 / SquareOmega0;
    // A3^(0) coefficient
    Deltat *= m_DeltaTj[0];
    m_Z(rowindex,1) = Deltat + m_DeltaTj[0] * 6.0 / SquareOmega0;
    // A4^(0) coefficient
    Deltat *= m_DeltaTj[0];
    m_Z(rowindex,2) = Deltat;
    m_Z(rowindex,3) = c0 = cosh(m_Omegaj[0] * m_DeltaTj[0]);
    m_Z(rowindex,4) = s0 = sinh(m_Omegaj[0] * m_DeltaTj[0]);
    m_Z(rowindex,5) = -1.0;
    rowindex++;

    // Second Row : Connection of the velocity of the CoM
    Deltat = m_DeltaTj[0];
    m_Z(rowindex,0) = 2 * Deltat;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,1) = 3 * Deltat + 6.0 / SquareOmega0;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,2) = 4 * Deltat;
    m_Z(rowindex,3) = m_Omegaj[0] * s0;
    m_Z(rowindex,4) = m_Omegaj[0] * c0;
    m_Z(rowindex,6) = -m_Omegaj[0];
    rowindex++;

    // Third Row : Terminal condition for the ZMP position
    Deltat = m_DeltaTj[0] * m_DeltaTj[0];
    m_Z(rowindex,0) = Deltat;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,1) = Deltat;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,2) = Deltat - 12 * m_DeltaTj[0] * m_DeltaTj[0]/ SquareOmega0;
    rowindex++;

    // Fourth Row : Terminal velocity for the ZMP position
    Deltat = m_DeltaTj[0];
    m_Z(rowindex,0) = 2.0 * Deltat;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,1) = 3.0 * Deltat;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,2) = 4 * Deltat - 24.0 * m_DeltaTj[0] / SquareOmega0;
    rowindex++;

  }

  void AnalyticalMorisawaCompact::ComputeZj(unsigned int intervalindex,
                                            unsigned int &colindex,
                                            unsigned int &rowindex)
  {
    // First row : Connection of the position of the CoM
    double Omegaj=m_Omegaj[intervalindex];
    double Omegam=m_Omegaj[m_NumberOfIntervals-1];


    double c0=0.0,s0=0.0;
    m_Z(rowindex,colindex) = c0 = cosh(Omegaj * m_DeltaTj[intervalindex]);
    m_Z(rowindex,colindex+1) = s0 = sinh(Omegaj * m_DeltaTj[intervalindex]);

    if ((int)intervalindex!=m_NumberOfIntervals-2)
      {
        m_Z(rowindex,colindex+2) = -1.0;
      }
    else
      {
        m_Z(rowindex,colindex+2) = -2.0/(Omegam*Omegam);
        m_Z(rowindex,colindex+5) = -1.0;
      }
    rowindex++;

    // Second row : Connection of the velocity of the CoM
    m_Z(rowindex,colindex) = Omegaj * s0;
    m_Z(rowindex,colindex+1) = Omegaj * c0;
    if ((int)intervalindex!=m_NumberOfIntervals-2)
      m_Z(rowindex,colindex+3) = -Omegaj;
    else
      {
        m_Z(rowindex,colindex+3) = -6.0/(Omegam*Omegam);
        m_Z(rowindex,colindex+6) = -Omegam;
      }
    rowindex++;
  }

  void AnalyticalMorisawaCompact::ComputeZm(unsigned intervalindex,
                                            unsigned int &colindex,
                                            unsigned int &rowindex)
  {
    double Omegam=m_Omegaj[m_NumberOfIntervals-1];
    double SquareOmegam = Omegam*Omegam;

    // First row : Connection of the position of the CoM
    double Deltat = m_DeltaTj[intervalindex];
    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex) = Deltat + 2.0 /SquareOmegam;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+1) = Deltat + m_DeltaTj[intervalindex] *
      6.0 /SquareOmegam;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = Deltat;

    double c0=0.0,s0=0.0;
    m_Z(rowindex,colindex+3) = c0 = cosh(Omegam * m_DeltaTj[intervalindex]);
    m_Z(rowindex,colindex+4) = s0 = sinh(Omegam * m_DeltaTj[intervalindex]);
    rowindex++;

    // Second row : Connection of the velocity of the CoM
    Deltat = m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex) = 2*Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+1) = 3*Deltat + 6.0 / SquareOmegam;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = 4*Deltat;

    m_Z(rowindex,colindex+3) = Omegam * s0;
    m_Z(rowindex,colindex+4) = Omegam * c0;
    rowindex++;

    // Third row: Terminal condition for the ZMP position
    Deltat = m_DeltaTj[intervalindex]* m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex) = Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+1) = Deltat;

    Deltat*= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = Deltat - 12 * m_DeltaTj[intervalindex] *
      m_DeltaTj[intervalindex]/ SquareOmegam;

    rowindex++;


    // Fourth row: Terminal velocity for the ZMP position
    Deltat = m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex) = 2 * Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+1) = 3 * Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = 4 * Deltat - 24.0 * m_DeltaTj[intervalindex] /
      SquareOmegam;

    rowindex++;


  }
  void AnalyticalMorisawaCompact::BuildingTheZMatrix(vector<double> &lCoM,
                                                     vector<double> &lZMP )
  {

    if (((int)lCoM.size()!=m_NumberOfIntervals)
        || ((int)lZMP.size()!=m_NumberOfIntervals))
      return;

    for(unsigned int i=0; i<lCoM.size(); i++)
      {
        m_Omegaj[i]=sqrt(9.86/ (lCoM[i] - lZMP[i]));
      }
    BuildingTheZMatrix();
  }

  void AnalyticalMorisawaCompact::BuildingTheZMatrix()
  {
    unsigned NbRows, NbCols;
    unsigned int rowindex=0;
    unsigned int colindex=0;

    NbRows = 2+4+2*(m_NumberOfIntervals-2)+4;
    NbCols = 2*m_NumberOfIntervals + 6;
    m_Z.resize(NbRows,NbCols);

    // Initial condition for the COG position and the velocity
    double SquareOmega0 = m_Omegaj[0]*m_Omegaj[0];

    {
      for(unsigned int i=0; i<m_Z.rows(); i++)
        for(unsigned int j=0; j<m_Z.cols(); j++)
          m_Z(i,j)=0.0;
    };
    m_Z(0,0) = 2.0/SquareOmega0;
    m_Z(0,3) = 1.0;
    rowindex++;
    m_Z(1,1) = 6.0/SquareOmega0;
    m_Z(1,4) = m_Omegaj[0];
    rowindex++;

    // Compute Z1
    ComputeZ1(rowindex);
    colindex+= m_PolynomialDegrees[0]+1;

    // Computing Zj.
    for(int i=1; i<m_NumberOfIntervals-1; ++i)
      {
        ComputeZj(i,colindex,rowindex);
        colindex+= 2;
      }

    // Computing Zm
    ComputeZm(m_NumberOfIntervals-1,
              colindex,rowindex);

    if (m_VerboseLevel>=2)
      {
        std::ofstream ofs;
        ofs.open("ZCompactMatrix.dat",ofstream::out);
        ofs.precision(10);
        for(unsigned int i=0; i<m_Z.rows(); i++)
          {
            for(unsigned int j=0; j<m_Z.cols(); j++)
              {
                ofs << m_Z(i,j) << " ";
              }
            ofs << endl;

          }
        ofs.close();
      }

  }

  void AnalyticalMorisawaCompact::
  TransfertTheCoefficientsToTrajectories
  (AnalyticalZMPCOGTrajectory &aAZCT,
   vector<double> &lCoMZ,
   vector<double> &lZMPZ,
   double &lZMPInit,
   double &lZMPEnd,
   bool )
  {
    vector<double> lV;
    vector<double> lW;

    // Set the starting point and the height variation.
    aAZCT.SetStartingTimeIntervalsAndHeightVariation(m_DeltaTj,m_Omegaj);

    unsigned int lindex = 0;

    // Weights.
    lV.resize(m_NumberOfIntervals);

    Polynome * aPolynome=0;
    aAZCT.GetFromListOfCOGPolynomials(0,aPolynome);
    vector<double> coeff;
    coeff.resize(m_PolynomialDegrees[0]+1);

    for(unsigned int k=2; k<=m_PolynomialDegrees[0]; k++)
      {
        coeff[k] = m_y[lindex++];
      }
    coeff[1] = coeff[3] * 6.0/(m_Omegaj[0]*m_Omegaj[0]);
    coeff[0] = lZMPInit + coeff[2] * 2.0/(m_Omegaj[0]*m_Omegaj[0]);

    aPolynome->SetCoefficients(coeff);

    lW.resize(m_NumberOfIntervals);


    for(int i=0; i<m_NumberOfIntervals-1; i++)
      {
        lV[i] = m_y[lindex++];
        lW[i] = m_y[lindex++];
      }

    for(unsigned int k=2; k<=m_PolynomialDegrees[m_NumberOfIntervals-1]; k++)
      {
        coeff[k] = m_y[lindex++];
      }
    coeff[1] = coeff[3] * 6.0/(m_Omegaj[m_NumberOfIntervals
                                        -1]*m_Omegaj[m_NumberOfIntervals-1]);
    coeff[0] = lZMPEnd + coeff[2] * 2.0/
      (m_Omegaj[m_NumberOfIntervals-1]*m_Omegaj[m_NumberOfIntervals-1]);


    aAZCT.GetFromListOfCOGPolynomials(m_NumberOfIntervals-1,aPolynome);
    aPolynome->SetCoefficients(coeff);

    lV[m_NumberOfIntervals-1] = m_y[lindex++];
    lW[m_NumberOfIntervals-1] = m_y[lindex++];

    // Set the hyperbolic weights.
    aAZCT.SetCoGHyperbolicCoefficients(lV,lW);

    // Compute the ZMP weights from the CoG's ones:

    // for the first interval
    aAZCT.TransfertOneIntervalCoefficientsFromCOGTrajectoryToZMPOne(0,lCoMZ[0],
                                                                    lZMPZ[0]);

    // and the last interval
    aAZCT.TransfertOneIntervalCoefficientsFromCOGTrajectoryToZMPOne
      (m_NumberOfIntervals-1,
       lCoMZ[m_NumberOfIntervals-1],
       lZMPZ[m_NumberOfIntervals-1]);
  }

  void AnalyticalMorisawaCompact::
  ComputeTrajectory
  (CompactTrajectoryInstanceParameters &aCTIP,
   AnalyticalZMPCOGTrajectory &aAZCT)
  {
    ComputeW(aCTIP.InitialCoM,
             aCTIP.InitialCoMSpeed,
             aCTIP.ZMPProfil,
             aCTIP.FinalCoMPos,
             aAZCT);
    ComputePolynomialWeights2();
    TransfertTheCoefficientsToTrajectories
      (aAZCT,
       aCTIP.CoMZ,
       aCTIP.ZMPZ,
       aCTIP.ZMPProfil[0],
       aCTIP.ZMPProfil[m_NumberOfIntervals-1],
       false);
    
  }


  int AnalyticalMorisawaCompact::
  TimeChange
  (double LocalTime,
   unsigned int IndexStep,
   unsigned int &IndexStartingInterval,
   double &FinalTime,
   double &NewTj)
  {

    // The Index Step can be equal to m_NumberOfIntervals.
    if ((int)IndexStep<m_NumberOfIntervals)

      if (m_StepTypes[IndexStep]!=DOUBLE_SUPPORT)
        {
          LTHROW("ERROR WRONG FOOT TYPE. ");
        }

    FinalTime = 0.0;
    for(unsigned int j=0; j<m_DeltaTj.size(); j++)
      FinalTime+=m_DeltaTj[j];

    /* Find from which interval we are starting. */
    m_AnalyticalZMPCoGTrajectoryX->
      GetIntervalIndexFromTime
      (LocalTime+m_AbsoluteTimeReference,IndexStartingInterval);

    double reftime=0.0;
    for(unsigned int j=0; j<IndexStartingInterval; j++)
      reftime+=m_DeltaTj[j];

    NewTj = m_DeltaTj[IndexStartingInterval] - LocalTime + reftime;

    /* Not enough time to change foot localisation
       if this is the next step which should be changed
       and we went over half the current interval.
    */
    if ((NewTj<m_Tsingle*0.5) &&
        (IndexStep==IndexStartingInterval+1))
      {
        return ERROR_TOO_LATE_FOR_MODIFICATION;
      }

    return 0;
  }

  void AnalyticalMorisawaCompact::NewTimeIntervals(unsigned int
                                                   IndexStartingInterval,
                                                   double NewTime)
  {

    /* Build the new time interval. */
    m_DeltaTj[0] = NewTime;
    m_StepTypes[0] = m_StepTypes[IndexStartingInterval];
    for(int i=1; i<m_NumberOfIntervals; i++)
      {
        if (m_StepTypes[i-1]==DOUBLE_SUPPORT)
          {
            m_DeltaTj[i] = m_Tsingle;
            m_StepTypes[i] = SINGLE_SUPPORT;
          }
        else
          {
            m_DeltaTj[i] = m_Tdble;
            m_StepTypes[i] = DOUBLE_SUPPORT;
          }
      }

    m_DeltaTj[m_NumberOfIntervals-1]=m_Tsingle*3.0;
    m_StepTypes[m_NumberOfIntervals-1] = DOUBLE_SUPPORT;
    ComputePreviewControlTimeWindow();

  }

  void AnalyticalMorisawaCompact::
  ConstraintsChange
  (double,
   FluctuationParameters FPX,
   FluctuationParameters FPY,
   CompactTrajectoryInstanceParameters &aCTIPX,
   CompactTrajectoryInstanceParameters &aCTIPY,
   unsigned int IndexStartingInterval,
   StepStackHandler *aStepStackHandler)
  {
    if (IndexStartingInterval!=0)
      {
        /* Shift the current value of the profil. */
        int i;
        unsigned int j;
        for(i=IndexStartingInterval,j=0; i<m_NumberOfIntervals; i++,j++)
          {
            /* Shift the ZMP profil */
            aCTIPX.ZMPProfil[j] = aCTIPX.ZMPProfil[i];
            aCTIPY.ZMPProfil[j] = aCTIPY.ZMPProfil[i];
          }

        /* Add value from the provided steps stack.
           BE CAREFUL: There is a modification on the initial value
           depending if a m_AbsoluteSupportFootPositions has been updated
           or not.
           If m_AbsoluteFootPositions has not been updated then
           a demand for a new step will be triggered.
        */

        unsigned int k = 0;
        if (m_NewStepInTheStackOfAbsolutePosition)
          k = (i-3)/2;
        else
          k = (i-1)/2;

        for(;
            (k < m_AbsoluteSupportFootPositions.size()) &&
              (j< m_CTIPX.ZMPProfil.size()); k++,j+=2)
          {

            aCTIPX.ZMPProfil[j] = m_AbsoluteSupportFootPositions[k].x;
            aCTIPY.ZMPProfil[j] = m_AbsoluteSupportFootPositions[k].y;

            if ((j+1)<m_CTIPX.ZMPProfil.size())
              {
                aCTIPX.ZMPProfil[j+1] = m_AbsoluteSupportFootPositions[k].x;
                aCTIPY.ZMPProfil[j+1] = m_AbsoluteSupportFootPositions[k].y;
              }

          }
        /* Complete the ZMP profil when no other step is available,
           and if there is a StepStack Handler available.
        */
        if (aStepStackHandler!=0)
          {
            /* Compute the number of steps needed. */
            int NeededSteps = (int)((aCTIPX.ZMPProfil.size()-j+1)/2);
            long int r;

            /* Test if there is enough step in the stack of.
               We have to remove one, because there is 
               still the last foot added.
            */
            if ((r=(long int)aStepStackHandler->ReturnStackSize()-1-
                 (long int)NeededSteps)<0)
              {
                bool lNewStep=false;
                double NewStepX=0.0,NewStepY=0.0,NewStepTheta=0.0;
                for(int li=0; li<-r; li++)
                  {

                    aStepStackHandler->AddStandardOnLineStep(lNewStep,
                                                             NewStepX,
                                                             NewStepY,
                                                             NewStepTheta);
                  }
              }

            /* Takes the number of Relative Foot Positions needed. */
            deque<RelativeFootPosition> lRelativeFootPositions;
            aStepStackHandler->CopyRelativeFootPosition
              (lRelativeFootPositions,false);

            /*! Remove the first step still in the stack. */
            lRelativeFootPositions.pop_front();

            deque<FootAbsolutePosition> lAbsoluteSupportFootPositions;
            int lLastIndex = (int)(m_AbsoluteSupportFootPositions.size()-1);
            m_FeetTrajectoryGenerator->
              ComputeAbsoluteStepsFromRelativeSteps
              (lRelativeFootPositions,
               m_AbsoluteSupportFootPositions[lLastIndex],
               lAbsoluteSupportFootPositions);

            /* Add the necessary absolute support foot positions. */
            for(int li=0;
                (li<NeededSteps)&& (j< m_CTIPX.ZMPProfil.size());
                li++,j+=2)
              {

                aCTIPX.ZMPProfil[j] = lAbsoluteSupportFootPositions[li].x;
                aCTIPY.ZMPProfil[j] = lAbsoluteSupportFootPositions[li].y;

                if ((j+1)<m_CTIPX.ZMPProfil.size())
                  {
                    aCTIPX.ZMPProfil[j+1] = lAbsoluteSupportFootPositions[li].x;
                    aCTIPY.ZMPProfil[j+1] = lAbsoluteSupportFootPositions[li].y;
                  }
              }

            /* Add the relative foot position inside the internal 
               stack as well as
               the absolute foot position. It also removes the steps
               inside the StepStackHandler object taken into account.*/
            for(int li=0; li<NeededSteps; li++)
              {
                m_RelativeFootPositions.push_back(lRelativeFootPositions[li]);
                m_AbsoluteSupportFootPositions.
                  push_back(lAbsoluteSupportFootPositions[li]);
                aStepStackHandler->RemoveFirstStepInTheStack();
              }
            /*! Remove the corresponding step from the stack of 
              relative and absolute foot positions. */
            for(unsigned int li=0; li<IndexStartingInterval/2; li++)
              {
                m_RelativeFootPositions.pop_front();
                m_AbsoluteSupportFootPositions.pop_front();
              }

          }

      }

    /* Compute the current value of the initial
       and final CoM to be feed to the new system. */
    aCTIPX.InitialCoM = FPX.CoMInit;
    aCTIPX.InitialCoMSpeed = FPX.CoMSpeedInit;
    aCTIPX.FinalCoMPos = aCTIPX.ZMPProfil[m_NumberOfIntervals-1];

    aCTIPY.InitialCoM = FPY.CoMInit;
    aCTIPY.InitialCoMSpeed = FPY.CoMSpeedInit;
    aCTIPY.FinalCoMPos = aCTIPY.ZMPProfil[m_NumberOfIntervals-1];


  }

  double AnalyticalMorisawaCompact::
  TimeCompensationForZMPFluctuation
  (FluctuationParameters &aFP,
   double DeltaTInit)
  {
    double r=0.0;
    double DeltaTNew=0.0;
    if (fabs(aFP.CoMSpeedNew)<1e-7)
      aFP.CoMSpeedNew = 0.0;
    if (fabs(aFP.ZMPSpeedNew)<1e-7)
      aFP.ZMPSpeedNew = 0.0;
    if (fabs(aFP.CoMSpeedInit)<1e-7)
      aFP.CoMSpeedInit = 0.0;
    if (fabs(aFP.ZMPSpeedInit)<1e-7)
      aFP.ZMPSpeedInit = 0.0;
    if (fabs(aFP.CoMInit)<1e-7)
      aFP.CoMInit = 0.0;
    if (fabs(aFP.ZMPInit)<1e-7)
      aFP.ZMPInit = 0.0;
    if (fabs(aFP.CoMNew)<1e-7)
      aFP.CoMNew = 0.0;
    if (fabs(aFP.ZMPNew)<1e-7)
      aFP.ZMPNew = 0.0;


    double rden= ( m_Omegaj[0]*(aFP.CoMInit - aFP.ZMPInit) +
                   (aFP.CoMSpeedInit - aFP.ZMPSpeedInit) );
    if (fabs(rden)<1e-5)
      rden = 0.0;

    double rnum = (m_Omegaj[0] * ( aFP.CoMNew - aFP.ZMPNew) +
                   (aFP.CoMSpeedNew - aFP.ZMPSpeedNew));

    if (rden==0.0)
      r=0.0;
    else r = rnum/rden;

    ;
    /*    r2 = ( m_Omegaj[0]*(aFP.CoMInit - aFP.ZMPInit) 
          + (aFP.CoMSpeedInit - aFP.ZMPSpeedInit) )/
          (m_Omegaj[0] * ( aFP.CoMNew - aFP.ZMPNew) + 
          (aFP.CoMSpeedNew - aFP.ZMPSpeedNew)); */

    if (r<0.0)
      DeltaTNew = DeltaTInit + m_Tsingle*0.5;
    else if (r>0)
      DeltaTNew = DeltaTInit + log(r)/m_Omegaj[0];
    else if (r==0.0)
      DeltaTNew = DeltaTInit;

    return DeltaTNew;

  }

  void AnalyticalMorisawaCompact::
  ChangeZMPProfil
  (vector<unsigned int> &
   IndexStep,
   vector<FootAbsolutePosition> &NewFootAbsPos,
   CompactTrajectoryInstanceParameters &aCTIPX,
   CompactTrajectoryInstanceParameters &aCTIPY)
  {

    /* Change in the constraints, i.e. modify 
       aCTIPX and aCTIPY appropriatly . */
    for(unsigned int i=0; i<IndexStep.size(); i++)
      {
        unsigned int lIndexStep=IndexStep[i];
        unsigned int lIndexForFootPrintInterval = 0;

        if (IndexStep.size()==NewFootAbsPos.size()*2)
          lIndexForFootPrintInterval = i/2;
        else
          {
            lIndexForFootPrintInterval = i/2;
            if (lIndexForFootPrintInterval>=NewFootAbsPos.size())
              lIndexForFootPrintInterval=(unsigned int)(NewFootAbsPos.size()-1);
          }

        if (lIndexStep<aCTIPX.ZMPProfil.size())
          {
            aCTIPX.ZMPProfil[lIndexStep] =
              NewFootAbsPos[lIndexForFootPrintInterval].x;
            aCTIPY.ZMPProfil[lIndexStep] =
              NewFootAbsPos[lIndexForFootPrintInterval].y;
          }
        if (lIndexStep<aCTIPX.ZMPProfil.size()-1)
          {
            aCTIPX.ZMPProfil[lIndexStep+1] =
              NewFootAbsPos[lIndexForFootPrintInterval].x;
            aCTIPY.ZMPProfil[lIndexStep+1] =
              NewFootAbsPos[lIndexForFootPrintInterval].y;
          }

        /* If the end condition has been changed... */
        if ((int)lIndexStep+1==m_NumberOfIntervals-1)
          {
            aCTIPX.FinalCoMPos = NewFootAbsPos[lIndexForFootPrintInterval].x;
            aCTIPY.FinalCoMPos = NewFootAbsPos[lIndexForFootPrintInterval].y;
          }
      }
  }

  int AnalyticalMorisawaCompact::
  ChangeFootLandingPosition
  (double t,
   vector<unsigned int> & IndexStep,
   vector<FootAbsolutePosition> & NewFootAbsPos)
  {
    int r=0;
    r=ChangeFootLandingPosition(t,IndexStep,
                                NewFootAbsPos,
                                *m_AnalyticalZMPCoGTrajectoryX,
                                m_CTIPX,
                                *m_AnalyticalZMPCoGTrajectoryY,
                                m_CTIPY,true,true,0,false);
    return r;
  }

  int AnalyticalMorisawaCompact::
  ChangeFootLandingPosition
  (double t,
   vector<unsigned int> & IndexStep,
   vector<FootAbsolutePosition> & NewFootAbsPos,
   AnalyticalZMPCOGTrajectory &aAZCTX,
   CompactTrajectoryInstanceParameters &aCTIPX,
   AnalyticalZMPCOGTrajectory &aAZCTY,
   CompactTrajectoryInstanceParameters &aCTIPY,
   bool TemporalShift,
   bool ResetFilters,
   StepStackHandler * aStepStackHandler,
   bool AddingAFootStep)
  {
    double LocalTime = t - m_AbsoluteTimeReference;
    double FinalTime=0.0;
    unsigned int IndexStartingInterval=0;
    int RetourTC=0;

    double NewTj=0.0;
    FluctuationParameters aFPX,aFPY;
    double TCX=0.0, TCY=0.0, TCMax=0.0;

    /* Perform First Time Change i.e. recomputing the proper Tj */
    if ((RetourTC=TimeChange(LocalTime,
                             IndexStep[0],
                             IndexStartingInterval,
                             FinalTime,
                             NewTj))<0)
      {
        LTHROW("Time Change not possible");
        // return RetourTC;
      }

    //displayDeltaTj(cerr);

    /* Store the current position and speed of each foot. */
    FootAbsolutePosition InitAbsLeftFootPos,InitAbsRightFootPos;

    m_FeetTrajectoryGenerator->
      ComputeAnAbsoluteFootPosition(1,t,
                                    InitAbsLeftFootPos);
    m_FeetTrajectoryGenerator->
      ComputeAnAbsoluteFootPosition(-1,t,
                                    InitAbsRightFootPos);
    
    /* ! This part of the code is not used if we are just trying to add
       a foot step. */
    // if ((int)IndexStep[0]<m_NumberOfIntervals)
    if (!AddingAFootStep)
      {
        /* Compute the time of maximal fluctuation 
           for the initial solution along the X axis.*/
        aAZCTX.FluctuationMaximal();
        aAZCTX.ComputeCOM(t,aFPX.CoMInit,IndexStartingInterval);

        aAZCTX.ComputeCOMSpeed(t,aFPX.CoMSpeedInit);
        aAZCTX.ComputeZMP(t,aFPX.ZMPInit,IndexStartingInterval);

        aAZCTX.ComputeZMPSpeed(t,aFPX.ZMPSpeedInit);

        /* Compute the time of maximal fluctuation 
           for the initial solution along the Y axis.*/
        aAZCTY.ComputeCOM(t,aFPY.CoMInit,IndexStartingInterval);
        aAZCTY.ComputeCOMSpeed(t,aFPY.CoMSpeedInit);
        aAZCTY.ComputeZMP(t,aFPY.ZMPInit,IndexStartingInterval);

        aAZCTY.ComputeZMPSpeed(t,aFPY.ZMPSpeedInit);

        /* Adapt the ZMP profil of aCTPIX and aCTPIY
           according to IndexStep */
        ChangeZMPProfil(IndexStep,NewFootAbsPos,
                        aCTIPX,aCTIPY);

        /* Recompute the coefficient of the ZMP/COG trajectories objects. */
        for(int i=1; i<m_NumberOfIntervals-1; i++)
          {
            aAZCTX.Building3rdOrderPolynomial
              (i,aCTIPX.ZMPProfil[i-1],aCTIPX.ZMPProfil[i]);
            aAZCTY.Building3rdOrderPolynomial
              (i,aCTIPY.ZMPProfil[i-1],aCTIPY.ZMPProfil[i]);
          }

        /* Compute the trajectories */
        ComputeTrajectory(aCTIPY,aAZCTY);
        ComputeTrajectory(aCTIPX,aAZCTX);

        aAZCTX.ComputeCOM(t,aFPX.CoMNew);

        aAZCTX.ComputeCOMSpeed(t,aFPX.CoMSpeedNew);
        aAZCTX.ComputeZMP(t,aFPX.ZMPNew);
        aAZCTX.ComputeZMPSpeed(t,aFPX.ZMPSpeedNew);

        aAZCTY.ComputeCOM(t,aFPY.CoMNew);
        aAZCTY.ComputeCOMSpeed(t,aFPY.CoMSpeedNew);
        aAZCTY.ComputeZMP(t,aFPY.ZMPNew);
        aAZCTY.ComputeZMPSpeed(t,aFPY.ZMPSpeedNew);

        TCX = TimeCompensationForZMPFluctuation(aFPX,NewTj);
        TCY = TimeCompensationForZMPFluctuation(aFPY,NewTj);

        TCMax = TCX < TCY ? TCY : TCX;
      }
    else
      {
        // For a proper initialization of the analytical trajectories
        // through constraint changes,
        // the Fluctuation structure has to be changed
        // approriatly.
        aAZCTX.ComputeCOM(t,aFPX.CoMInit);
        aAZCTX.ComputeCOMSpeed(t,aFPX.CoMSpeedInit);
        aAZCTX.ComputeZMP(t,aFPX.ZMPInit,IndexStartingInterval);
        aAZCTX.ComputeZMPSpeed(t,aFPX.ZMPSpeedInit);

        aAZCTY.ComputeCOM(t,aFPY.CoMInit);
        aAZCTY.ComputeCOMSpeed(t,aFPY.CoMSpeedInit);
        aAZCTY.ComputeZMP(t,aFPY.ZMPInit,IndexStartingInterval);
        aAZCTY.ComputeZMPSpeed(t,aFPY.ZMPSpeedInit);


        aAZCTX.ComputeCOM(t,aFPX.CoMNew);

        aAZCTX.ComputeCOMSpeed(t,aFPX.CoMSpeedNew);
        aAZCTX.ComputeZMP(t,aFPX.ZMPNew);
        aAZCTX.ComputeZMPSpeed(t,aFPX.ZMPSpeedNew);

        aAZCTY.ComputeCOM(t,aFPY.CoMNew);
        aAZCTY.ComputeCOMSpeed(t,aFPY.CoMSpeedNew);
        aAZCTY.ComputeZMP(t,aFPY.ZMPNew);
        aAZCTY.ComputeZMPSpeed(t,aFPY.ZMPSpeedNew);

        if (m_StepTypes[IndexStartingInterval]==SINGLE_SUPPORT)
          TCMax = m_Tsingle-m_SamplingPeriod;
        else
          TCMax = m_Tdble-m_SamplingPeriod;
      }


    /************ PERFORM THE TIME INTERVAL MODIFICATION ****************/

    if (TemporalShift)
      {
        NewTimeIntervals(IndexStartingInterval,TCMax); //TCMax
      }
    else
      {
        NewTimeIntervals(IndexStartingInterval,NewTj);
      }


    /*! Extract the set of absolute coordinates for the foot position,
      and recompute the feet trajectory accordingly. */
    if (m_FeetTrajectoryGenerator!=0)
      {

        m_FeetTrajectoryGenerator->SetDeltaTj(m_DeltaTj);

        /* Modify the feet trajectory */
        ODEBUG("***** Begin Change Foot Landing Position **************");
        m_FeetTrajectoryGenerator->ComputeAbsoluteStepsFromRelativeSteps
          (m_RelativeFootPositions,
           InitAbsLeftFootPos,InitAbsRightFootPos,
           m_AbsoluteSupportFootPositions);
        
        ODEBUG("***** End of Change Foot Landing Position *************");
      }

    /* Shift the ZMP profil, the initial
       and final condition, and update the queue of foot prints. */
    ConstraintsChange(LocalTime,
                      aFPX,aFPY,
                      aCTIPX,aCTIPY,
                      IndexStartingInterval,
                      aStepStackHandler);

    m_FeetTrajectoryGenerator->InitializeFromRelativeSteps
      (m_RelativeFootPositions,
       InitAbsLeftFootPos,
       InitAbsRightFootPos,
       m_AbsoluteSupportFootPositions,
       false,true);
    
    // Initialize and modify the aAZCT trajectories' Tj and Omegaj.
    aAZCTX.SetStartingTimeIntervalsAndHeightVariation(m_DeltaTj,m_Omegaj);
    aAZCTY.SetStartingTimeIntervalsAndHeightVariation(m_DeltaTj,m_Omegaj);

    /* Recompute the coefficient of the ZMP/COG trajectories objects. */
    for(int i=1; i<m_NumberOfIntervals-1; i++)
      {
        aAZCTX.Building3rdOrderPolynomial
          (i,aCTIPX.ZMPProfil[i-1],aCTIPX.ZMPProfil[i]);
        aAZCTY.Building3rdOrderPolynomial
          (i,aCTIPY.ZMPProfil[i-1],aCTIPY.ZMPProfil[i]);
      }

    aAZCTX.SetAbsoluteTimeReference(t);
    aAZCTY.SetAbsoluteTimeReference(t);

    /* Build the Z matrix */
    BuildingTheZMatrix();
    ResetTheResolutionOfThePolynomial();

    /* Compute the trajectories for ZMP and CoM */
    ComputeTrajectory(aCTIPY,aAZCTY);
    ComputeTrajectory(aCTIPX,aAZCTX);

    m_FeetTrajectoryGenerator->SetAbsoluteTimeReference(t);
    m_AbsoluteTimeReference = t;

    /* Reset the filters */
    // Preparing the filtering out of the feet.
    if (m_FilteringActivate && ResetFilters)
      {
        m_FilterXaxisByPC->FillInWholeBuffer(aFPX.ZMPInit,m_DeltaTj[0]);
        m_FilterYaxisByPC->FillInWholeBuffer(aFPY.ZMPInit,m_DeltaTj[0]);
      }
    return 0;
  }

  void AnalyticalMorisawaCompact::StringErrorMessage(int ErrorIndex,
                                                     string &ErrorMessage)
  {
    switch(ErrorIndex)
      {
      case(0):
        ErrorMessage="No error";
        break;
      case(-1):
        ErrorMessage="Wrong foot type";
        break;
      case(-2):
        ErrorMessage="Modification is asked after the time limit";
        break;
      default:
        ErrorMessage="Unknown error";
        break;
      }
  }

  int AnalyticalMorisawaCompact::SetPreviewControl(PreviewControl *
                                                   aPreviewControl)
  {
    m_PreviewControl = aPreviewControl;
    m_FilterXaxisByPC->SetPreviewControl(aPreviewControl);
    m_FilterYaxisByPC->SetPreviewControl(aPreviewControl);

    return 0;
  }

  PreviewControl * AnalyticalMorisawaCompact::GetPreviewControl()
  {
    return m_PreviewControl;
  }

  void AnalyticalMorisawaCompact::
  FilterOutOrthogonalDirection
  (AnalyticalZMPCOGTrajectory & aAZCT,
   CompactTrajectoryInstanceParameters &aCTIP,
   deque<double> & ZMPTrajectory,
   deque<double> & CoGTrajectory)
  {
    /* Initiliazing the Preview Control according to the trajectory
       to filter. */
    double lAbsoluteTimeReference = aAZCT.GetAbsoluteTimeReference();
    Eigen::MatrixXd x(3,1);

    /*! Initialize the state vector used by the preview controller */
    x(0,0) = 0.0;//aAZCT.ComputeCOM(lAbsoluteTimeReference,x(0,0));
    x(1,0) = 0.0;//aAZCT.ComputeCOMSpeed(lAbsoluteTimeReference,x(1,0));
    x(2,0) = 0.0;

    /*! Initializing variables needed to compute the state vector */
    double lsxzmp = 0.0;
    double lxzmp = 0.0;

    /*! Preview window of the ZMP ref positions */
    double PreviewWindowTime = m_PreviewControl->PreviewControlTime();
    deque<double> FIFOZMPRefPositions;
    RESETDEBUG4("ProfilZMPError.dat");
    for(double lx=0; lx<m_DeltaTj[0]+2*PreviewWindowTime; lx+=m_SamplingPeriod)
      {
        double r=0.0;
        if (lx<m_DeltaTj[0])
          {
            double lZMP;
            aAZCT.ComputeZMP(lAbsoluteTimeReference+lx,lZMP);
            r = aCTIP.ZMPProfil[0] - lZMP;
          }

        FIFOZMPRefPositions.push_back(r);
        ODEBUG4(r,"ProfilZMPError.dat");
      }


    unsigned int lindex=0;
    lsxzmp = 0.0;
    for(double lx=0; lx<m_DeltaTj[0]+PreviewWindowTime; lx+= m_SamplingPeriod)
      {
        m_PreviewControl->
          OneIterationOfPreview1D(x,lsxzmp,FIFOZMPRefPositions,lindex,
                                  lxzmp,false);
        ZMPTrajectory.push_back(lxzmp);
        CoGTrajectory.push_back(x(0,0));
        lindex++;
        lsxzmp = 0.0;

      }
  }


  void AnalyticalMorisawaCompact::
  SetFeetTrajectoryGenerator
  (LeftAndRightFootTrajectoryGenerationMultiple *
   aFeetTrajectoryGenerator)
  {
    m_FeetTrajectoryGenerator = aFeetTrajectoryGenerator;
    if (m_BackUpm_FeetTrajectoryGenerator==0)
      m_BackUpm_FeetTrajectoryGenerator=
        new LeftAndRightFootTrajectoryGenerationMultiple
        (m_FeetTrajectoryGenerator->getSimplePluginManager(),
         m_FeetTrajectoryGenerator->getFoot());

  }

  LeftAndRightFootTrajectoryGenerationMultiple *
  AnalyticalMorisawaCompact::GetFeetTrajectoryGenerator()
  {
    return m_FeetTrajectoryGenerator;
  }

  int AnalyticalMorisawaCompact::
  OnLineFootChange
  (double time,
   FootAbsolutePosition &aFootAbsolutePosition,
   deque<ZMPPosition> & ZMPPositions,
   deque<COMState> & CoMPositions,
   deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
   deque<FootAbsolutePosition> &RightFootAbsolutePositions,
   StepStackHandler *aStepStackHandler)
  {
    deque<FootAbsolutePosition> NewFeetAbsolutePosition;
    NewFeetAbsolutePosition.push_back(aFootAbsolutePosition);
    return OnLineFootChanges(time,
                             NewFeetAbsolutePosition,
                             ZMPPositions,
                             CoMPositions,
                             LeftFootAbsolutePositions,
                             RightFootAbsolutePositions,
                             aStepStackHandler);

  }

  int AnalyticalMorisawaCompact::
  OnLineFootChanges
  (double time,
   deque<FootAbsolutePosition> &aFootAbsolutePosition,
   deque<ZMPPosition> & ZMPPositions,
   deque<COMState> & CoMPositions,
   deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
   deque<FootAbsolutePosition> &RightFootAbsolutePositions,
   StepStackHandler *aStepStackHandler)
  {
    
    ODEBUG("***** Begin OnLineFootChange *****");
    int IndexInterval=-1;

    /* Trying to find the index interval where the change should operate. */
    double TimeReference = m_AbsoluteTimeReference;
    if (time> m_AbsoluteTimeReference)
      {
        for(unsigned int i=0; i<m_DeltaTj.size(); i++)
          {
            if (time < TimeReference + m_DeltaTj[i])
              {
                IndexInterval = (int)i;
                break;
              }
            TimeReference += m_DeltaTj[i];
          }
      }

    if (IndexInterval==-1)
      {
        cerr << "time :" << time << endl
             << "m_AbsoluteTimeReference : " << m_AbsoluteTimeReference << endl
             << "m_AbsoluteTimeReference + sum of DTj: " << TimeReference;
        LTHROW("The time reference is not in the preview window ");
        return -1;
      }

    /* If the interval detected is not a double support interval,
       a shift is done to chose the earliest double support interval. */
    if (m_StepTypes[IndexInterval]!=DOUBLE_SUPPORT)
      {

        if (IndexInterval!=0)
          IndexInterval-=1;
        else
          IndexInterval+=1;
      }
    else
      {
        ODEBUG("Already on a DOUBLE_SUPPORT PHASE:" << IndexInterval);
      }

    if (IndexInterval==-1)
      {
        LTHROW("No feasible double support interval found.");
        return -1;
      }

    /* Backup data structures */
    FootAbsolutePosition BackUpm_AbsoluteCurrentSupportFootPosition =
      m_AbsoluteCurrentSupportFootPosition;
    deque<FootAbsolutePosition> BackUpm_AbsoluteSupportFootPositions =
      m_AbsoluteSupportFootPositions;
    deque<RelativeFootPosition> BackUpm_RelativeFootPositions =
      m_RelativeFootPositions;
    *m_BackUpm_FeetTrajectoryGenerator = *m_FeetTrajectoryGenerator;

    /* Find the corresponding interval in the stack of foot steps*/
    unsigned int lChangedIntervalFoot = (IndexInterval-1)/2;

    /* Which foot is the support one ? */
    if (LeftFootAbsolutePositions[0].stepType < 0)
      m_AbsoluteCurrentSupportFootPosition = LeftFootAbsolutePositions[0];
    else
      m_AbsoluteCurrentSupportFootPosition = RightFootAbsolutePositions[0];

    vector<unsigned int> IndexIntervals;
    vector<FootAbsolutePosition> NewRelFootAbsolutePositions;

    /*! Recompute relative or absolute foot-steps positions 
      depending on the mode. */
    if (m_OnLineChangeStepMode==ABSOLUTE_FRAME)
      {
        IndexIntervals.resize(1);
        IndexIntervals[0] = IndexInterval;

        for(unsigned int i=0; i<aFootAbsolutePosition.size(); i++)
          m_AbsoluteSupportFootPositions[i+lChangedIntervalFoot] =
            aFootAbsolutePosition[i];

        /* From the new foot landing position computes the new relative set of
           positions. */
        m_FeetTrajectoryGenerator->
          ChangeRelStepsFromAbsSteps
          (m_RelativeFootPositions,
           m_AbsoluteCurrentSupportFootPosition,
           m_AbsoluteSupportFootPositions,
           lChangedIntervalFoot);

        NewRelFootAbsolutePositions.resize(aFootAbsolutePosition.size());
        for(unsigned int i=0; i<aFootAbsolutePosition.size(); i++)
          NewRelFootAbsolutePositions[i] = aFootAbsolutePosition[i];
      }
    else if (m_OnLineChangeStepMode==RELATIVE_FRAME)
      {
        IndexIntervals.resize(m_NumberOfIntervals-IndexInterval);

        NewRelFootAbsolutePositions.resize(m_RelativeFootPositions.size()
                                           -lChangedIntervalFoot);

        for(int j=IndexInterval,k=0; k<(int)IndexIntervals.size(); j++,k++)
          {
            IndexIntervals[k] = j;
          }

        // In this mode the frame is relative to previous local modification...
        for(unsigned int i=0; i<aFootAbsolutePosition.size(); i++)
          {
            m_RelativeFootPositions[lChangedIntervalFoot+i].sx +=
              aFootAbsolutePosition[i].x;
            m_RelativeFootPositions[lChangedIntervalFoot+i].sy +=
              aFootAbsolutePosition[i].y;
            m_RelativeFootPositions[lChangedIntervalFoot+i].sz +=
              aFootAbsolutePosition[i].z;
            m_RelativeFootPositions[lChangedIntervalFoot+i].theta +=
              aFootAbsolutePosition[i].theta;
          }

        deque<FootAbsolutePosition> lAbsoluteSupportFootPositions;
        m_FeetTrajectoryGenerator->
          ComputeAbsoluteStepsFromRelativeSteps
          (m_RelativeFootPositions,
           LeftFootAbsolutePositions[0],
           RightFootAbsolutePositions[0],
           lAbsoluteSupportFootPositions);
        
        for(unsigned int j=0,k=lChangedIntervalFoot;
            j<NewRelFootAbsolutePositions.size(); j++,k++)
          {
            NewRelFootAbsolutePositions[j] = lAbsoluteSupportFootPositions[k];
          }
        for(unsigned int k=lChangedIntervalFoot;
            k<m_AbsoluteSupportFootPositions.size(); k++)
          {
            m_AbsoluteSupportFootPositions[k] =
              lAbsoluteSupportFootPositions[k];
          }

      }


    ODEBUG("*** End Change foot position *** ");
    /* Change the foot landing position. */
    try
      {
        ChangeFootLandingPosition(m_CurrentTime,
                                  IndexIntervals,
                                  NewRelFootAbsolutePositions,
                                  *m_AnalyticalZMPCoGTrajectoryX,
                                  m_CTIPX,
                                  *m_AnalyticalZMPCoGTrajectoryY,
                                  m_CTIPY,true,true,
                                  aStepStackHandler,false);
      }
    catch(exception &e)
      {
        /*! Put back the foot steps to their original states */
        m_AbsoluteCurrentSupportFootPosition=
          BackUpm_AbsoluteCurrentSupportFootPosition;
        m_AbsoluteSupportFootPositions =
          BackUpm_AbsoluteSupportFootPositions;
        m_RelativeFootPositions =
          BackUpm_RelativeFootPositions;

        /*! Same for the feet trajectories */
        *m_FeetTrajectoryGenerator = *m_BackUpm_FeetTrajectoryGenerator;

        std::cerr << "Unable to change the step ( "
                  << aFootAbsolutePosition[0].x << " , "
                  << aFootAbsolutePosition[0].y << " , "
                  << aFootAbsolutePosition[0].theta << " ) "
                  << std::endl;
        throw e;
      }

    // *** Very important:
    // we assume that on the on-line mode we have two values ahead inside
    // the stack. As the change will operate from the current time
    // the stacks are cleared.
    // ***
    ZMPPositions.clear();
    CoMPositions.clear();
    LeftFootAbsolutePositions.clear();
    RightFootAbsolutePositions.clear();

    /*! Compute next time where a foot-step should be added. */
    m_UpperTimeLimitToUpdateStacks = m_AbsoluteTimeReference + m_DeltaTj[0] +
      m_Tdble + 0.45 * m_Tsingle;

    /*! Put 2 iterations of the new trajectories in the queues */
    FillQueues(m_AbsoluteTimeReference,
               m_AbsoluteTimeReference+2*m_SamplingPeriod,
               ZMPPositions,CoMPositions, LeftFootAbsolutePositions,
               RightFootAbsolutePositions);
    ODEBUG("***** End OnLineFootChange *****");
    return 0;
  }

  /*! \brief Return the time at which it is optimal to 
    regenerate a step in online mode.
    This time is given in the size of the Left Foot Positions 
    queue under which such
    new step has to be generate.
  */
  int AnalyticalMorisawaCompact::ReturnOptimalTimeToRegenerateAStep()
  {
    int r=1;
    return r;
  }

  void AnalyticalMorisawaCompact::
  EndPhaseOfTheWalking
  (deque<ZMPPosition>
   &FinalZMPPositions,
   deque<COMState> &FinalCoMPositions,
   deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
   deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)
  {

    m_OnLineMode = true;
    bool DoNotPrepareLastFoot = false;
    int NbSteps = (int)m_RelativeFootPositions.size();
    int NbOfIntervals=2*NbSteps+1;

    /* Update the relative and absolute foot positions. */
    m_RelativeFootPositions.pop_front();

    ODEBUG("***** Begin EndPhaseOfTheWalking *****");
    // Strategy for the final CoM pos: middle of the segment
    // between the two final steps, in order to be statically stable.
    unsigned int lindex = (unsigned int)
      (m_AbsoluteSupportFootPositions.size()-1);
    vector<double> * lZMPX=0;
    double FinalCoMPosX=0.6;

    vector<double> * lZMPY=0;
    double FinalCoMPosY=0.0;

    /*! Prepare end condition for CoM along X axis */
    if (DoNotPrepareLastFoot)
      FinalCoMPosX = m_AbsoluteSupportFootPositions[lindex].x;
    else
      FinalCoMPosX = 0.5 *(m_AbsoluteSupportFootPositions[lindex-1].x +
                           m_AbsoluteSupportFootPositions[lindex].x);
    m_CTIPX.FinalCoMPos = FinalCoMPosX;

    /*! Prepare end condition for ZMP along X axis */
    lZMPX = &m_CTIPX.ZMPProfil;
    unsigned int j=(unsigned int)lZMPX->size();
    if (DoNotPrepareLastFoot)
      (*lZMPX)[j-2] = (*lZMPX)[j-1] = m_AbsoluteSupportFootPositions[lindex].x;
    else
      (*lZMPX)[j-2] = (*lZMPX)[j-1] = FinalCoMPosX;

    /*! Build 3rd order polynomials. */
    for(int i=1; i<NbOfIntervals-1; i++)
      {
        m_AnalyticalZMPCoGTrajectoryX->
          Building3rdOrderPolynomial(i,(*lZMPX)[i-1],
                                     (*lZMPX)[i]);
      }

    /*! Compute trajectory for CoM along X axis. */
    ComputeTrajectory(m_CTIPX,*m_AnalyticalZMPCoGTrajectoryX);

    /*! Prepare end condition for CoM along Y axis */
    if (DoNotPrepareLastFoot)
      FinalCoMPosY = m_AbsoluteSupportFootPositions[lindex].y;
    else
      FinalCoMPosY = 0.5 *(m_AbsoluteSupportFootPositions[lindex-1].y +
                           m_AbsoluteSupportFootPositions[lindex].y);
    m_CTIPY.FinalCoMPos = FinalCoMPosY;

    /*! Prepare end condition for ZMP along Y axis */
    lZMPY = &m_CTIPY.ZMPProfil;
    if (DoNotPrepareLastFoot)
      (*lZMPY)[j-2] = (*lZMPY)[j-1] = m_AbsoluteSupportFootPositions[lindex].y;
    else
      (*lZMPY)[j-2] = (*lZMPY)[j-1] = FinalCoMPosY;

    /*! Build 3rd order polynomials. */
    for(int i=1; i<NbOfIntervals-1; i++)
      {
        m_AnalyticalZMPCoGTrajectoryY->
          Building3rdOrderPolynomial(i,(*lZMPY)[i-1],
                                     (*lZMPY)[i]);
      }

    /*! Compute the analytical trajectory*/
    ComputeTrajectory(m_CTIPY,*m_AnalyticalZMPCoGTrajectoryY);

    /* Specify when a new step should be asked for. */
    m_UpperTimeLimitToUpdateStacks = m_AbsoluteTimeReference +
      m_PreviewControlTime;

    /* Put two positions from the new polynomials in the queues. */
    FillQueues(m_CurrentTime,
               m_CurrentTime+2*m_SamplingPeriod,
               FinalZMPPositions,
               FinalCoMPositions,
               FinalLeftFootAbsolutePositions,
               FinalRightFootAbsolutePositions);

    m_EndPhase=true;
    ODEBUG("**** End of EndPhaseOfTheWalking *******");
  }

  void AnalyticalMorisawaCompact::RegisterMethods()
  {
    std::string aMethodName[2]=
      {":onlinechangestepframe",":setRobotUpperPart"};

    for(int i=0; i<1; i++)
      {
        if (!RegisterMethod(aMethodName[i]))
          {
            std::cerr << "Unable to register " << aMethodName[i] << std::endl;
          }
        else
          {
            ODEBUG("Succeed in registering " << aMethodName[i]);
          }
      }
  }


  void AnalyticalMorisawaCompact::CallMethod(std::string & Method,
                                             std::istringstream &strm)
  {
    if (Method==":onlinechangestepframe")
      {
        std::string aws;
        if (strm.good())
          {
            strm >> aws;
            if (aws=="absolute")
              m_OnLineChangeStepMode = ABSOLUTE_FRAME;
            else if (aws=="relative")
              m_OnLineChangeStepMode = RELATIVE_FRAME;
          }
      }
    else if (Method==":filtering")
      {
        std::string aws;
        if (strm.good())
          {
            strm >> aws;
            if (aws=="activate")
              m_FilteringActivate = true;
            else if (aws=="deactivate")
              m_FilteringActivate = false;
          }
      }
    else if (Method==":setRobotUpperPart")
      {
        //      Eigen::VectorXd configuration ;
        //      if (strm.good())
        //      {
        //        strm >> configuration;
        //        m_kajitaDynamicFilter->setRobotUpperPart(configuration);
        //      }
      }

    ZMPRefTrajectoryGeneration::CallMethod(Method,strm);
  }

  void AnalyticalMorisawaCompact::PropagateAbsoluteReferenceTime(double x)
  {
    m_AbsoluteTimeReference = x;
    m_AnalyticalZMPCoGTrajectoryX->SetAbsoluteTimeReference(x);
    m_AnalyticalZMPCoGTrajectoryY->SetAbsoluteTimeReference(x);
    m_FeetTrajectoryGenerator->SetAbsoluteTimeReference(x);
  }

  void AnalyticalMorisawaCompact::
  FillQueues
  (double samplingPeriod,
   double StartingTime,
   double EndTime,
   deque<ZMPPosition> & FinalZMPPositions,
   deque<COMState> & FinalCoMPositions,
   deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
   deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions)
  {
    unsigned int lIndexInterval,lPrevIndexInterval;
    m_AnalyticalZMPCoGTrajectoryX->
      GetIntervalIndexFromTime(m_AbsoluteTimeReference,
                               lIndexInterval);
    lPrevIndexInterval = lIndexInterval;

    /*! Fill in the stacks: minimal strategy only 1 reference. */
    for(double t=StartingTime; t<=EndTime; t+= samplingPeriod)
      {
        m_AnalyticalZMPCoGTrajectoryX->
          GetIntervalIndexFromTime(t,lIndexInterval,
                                   lPrevIndexInterval);
        
        /*! Feed the ZMPPositions. */
        ZMPPosition aZMPPos;
        if (!m_AnalyticalZMPCoGTrajectoryX->
            ComputeZMP(t,aZMPPos.px,lIndexInterval))
          LTHROW("Unable to compute ZMP along X-Axis in EndPhaseOfWalking");

        if (!m_AnalyticalZMPCoGTrajectoryY->
            ComputeZMP(t,aZMPPos.py,lIndexInterval))
          LTHROW("Unable to compute ZMP along Y-Axis in EndPhaseOfWalking");

        ComputeZMPz(t,aZMPPos,lIndexInterval);

        FinalZMPPositions.push_back(aZMPPos);

        /*! Feed the FootPositions. */

        /*! Left */
        FootAbsolutePosition LeftFootAbsPos;
        memset(&LeftFootAbsPos,0,sizeof(LeftFootAbsPos));
        if (!m_FeetTrajectoryGenerator->
            ComputeAnAbsoluteFootPosition
            (1,t,
             LeftFootAbsPos,lIndexInterval))
          {
            LTHROW("Unable to compute left foot position in EndPhaseOfWalking");
          }
        FinalLeftFootAbsolutePositions.push_back(LeftFootAbsPos);

        /*! Right */
        FootAbsolutePosition RightFootAbsPos;
        memset(&RightFootAbsPos,0,sizeof(RightFootAbsPos));
        if (!m_FeetTrajectoryGenerator->
            ComputeAnAbsoluteFootPosition
            (-1,t,
             RightFootAbsPos,lIndexInterval))
          {
            LTHROW("Unable to compute right foot"
                   " position in EndPhaseOfWalking");
          }
        FinalRightFootAbsolutePositions.push_back(RightFootAbsPos);

        /*! Feed the COMStates. */
        COMState aCOMPos;
        memset(&aCOMPos,0,sizeof(aCOMPos));
        if (!m_AnalyticalZMPCoGTrajectoryX->
            ComputeCOM(t,aCOMPos.x[0],lIndexInterval))
          {
            LTHROW("COM out of bound along X axis.");
          }
        m_AnalyticalZMPCoGTrajectoryX->
          ComputeCOMSpeed(t,aCOMPos.x[1],lIndexInterval);
        m_AnalyticalZMPCoGTrajectoryX->ComputeCOMAcceleration(t,aCOMPos.x[2],
                                                              lIndexInterval);

        if (!m_AnalyticalZMPCoGTrajectoryY->
            ComputeCOM(t,aCOMPos.y[0],lIndexInterval))
          {
            LTHROW("COM out of bound along Y axis.");
          }
        m_AnalyticalZMPCoGTrajectoryY->
          ComputeCOMSpeed(t,aCOMPos.y[1],lIndexInterval);
        m_AnalyticalZMPCoGTrajectoryY->ComputeCOMAcceleration(t,aCOMPos.y[2],
                                                              lIndexInterval);

        ComputeCoMz(t, lIndexInterval, aCOMPos, FinalCoMPositions);

        aCOMPos.yaw[0] = 0.5*(LeftFootAbsPos.theta + RightFootAbsPos.theta);
        aCOMPos.yaw[1] = 0.5*(LeftFootAbsPos.dtheta + RightFootAbsPos.dtheta);
        aCOMPos.yaw[2] = 0.5*(LeftFootAbsPos.ddtheta + RightFootAbsPos.ddtheta);

        FinalCoMPositions.push_back(aCOMPos);

        ODEBUG4(aZMPPos.px << " " << aZMPPos.py << " " <<
                aCOMPos.x[0] << " " << aCOMPos.y[0] << " " << aCOMPos.z[0]
                <<" "<<
                LeftFootAbsPos.x << " " << LeftFootAbsPos.y << " " <<
                LeftFootAbsPos.z << " " <<
                RightFootAbsPos.x << " " << RightFootAbsPos.y << " " <<
                RightFootAbsPos.z << " "
                <<
                samplingPeriod,"Test.dat");
      }
  }

  void AnalyticalMorisawaCompact::ComputeCoMz(COMState & CoM,
                                              FootAbsolutePosition & LeftFoot,
                                              FootAbsolutePosition & )
  {

    CoM.z[0] = ( (LeftFoot.z + 0.7) + (LeftFoot.z + 0.85) + (LeftFoot.z + 0.7) +
                 (LeftFoot.z + 0.85) ) * 0.25 ;
    CoM.z[0] = ( (LeftFoot.dz + 0.7) + (LeftFoot.dz + 0.85) +
                 (LeftFoot.dz + 0.7) + (LeftFoot.dz + 0.85) ) * 0.25 ;
    CoM.z[0] = ( (LeftFoot.ddz + 0.7) + (LeftFoot.ddz + 0.85) +
                 (LeftFoot.ddz + 0.7) + (LeftFoot.ddz + 0.85) ) * 0.25 ;

  }

  void AnalyticalMorisawaCompact::
  ComputeCoMz
  (double t,
   unsigned int lIndexInterval,
   COMState &CoM,
   deque<COMState> & FinalCoMPositions)
  {
    double* CoMz = CoM.z ;
    double moving_time = m_RelativeFootPositions[0].SStime +
      m_RelativeFootPositions[0].DStime;
    unsigned int Index = lIndexInterval/2 ;

    // absFootz_0, the z axis is expressed in the waist frame
    // we choose the left one by default, the foot are supposed to be symetrical
    // we use it pass the ankle position to the fot position
    PRFoot *aFoot = m_PR->leftFoot() ;
    if (aFoot==0)
      LTHROW("No foot");
    Eigen::Vector3d corrZ ;
    corrZ = aFoot->anklePosition ;
    corrZ(2) = 0; //foot height no more equal to ankle height; TODO :
    // remove corrZ

    // after the final step we keep the same position for a while
    if( Index >= m_AbsoluteSupportFootPositions.size() )
      {
        if(FinalCoMPositions.size()==0)
          {
            CoMz[0] = m_InitialPoseCoMHeight +
              m_AbsoluteSupportFootPositions[Index].z -
              corrZ(2);
            CoMz[1] = 0.0 ;
            CoMz[2] = 0.0 ;
            return ;
          }

        COMState LastCoM = FinalCoMPositions.back();
        double higherPoseCoMz = m_InitialPoseCoMHeight +
          m_AbsoluteSupportFootPositions.back().z - corrZ(2);
        double ft = m_RelativeFootPositions.back().SStime-
          (t-Index*moving_time) ;

        m_CoMbsplinesZ->SetParameters
          (ft,LastCoM.z[0],higherPoseCoMz,
           vector<double>(), vector<double>(), LastCoM.z[1], LastCoM.z[2]) ;
        m_CoMbsplinesZ->Compute(m_SamplingPeriod,CoMz[0],CoMz[1],CoMz[2]) ;
        return ;
      }
    //    cout << "INDEX = " << Index << endl ;
    //    cout << "lIndexInterval = " << lIndexInterval << endl ;

    double sx = m_RelativeFootPositions[Index].sx ;
    double sy = m_RelativeFootPositions[Index].sy ;
    double sz = m_RelativeFootPositions[Index].sz ;
    double SStime = m_RelativeFootPositions[Index].SStime ;
    double DStime = m_RelativeFootPositions[Index].DStime ;
    double initCoMheight = m_InitialPoseCoMHeight +
      m_AbsoluteSupportFootPositions[Index].z - corrZ(2);
    double lowerCoMheight = 0.95*m_InitialPoseCoMHeight +
      m_AbsoluteSupportFootPositions[Index].z - corrZ(2);
    double FinalTime(0.0), InitPos(0.0), InitSpeed(0.0), InitAcc(0.0),
      FinalPos(0.0), interpolationTime(0.0);
    vector<double> MP ;
    MP.clear() ;
    vector<double> ToMP ;
    ToMP.clear() ;

    COMState LastCoM ;
    if (FinalCoMPositions.size()!=0)
      LastCoM = FinalCoMPositions.back() ;
    else
      {
        LastCoM.z[0] = initCoMheight ;
        LastCoM.z[1] = 0.0 ;
        LastCoM.z[2] = 0.0 ;
      }

    // we start analyze since 2nd step
    if ( Index == 0 )
      {
        sx = m_RelativeFootPositions[Index+1].sx ;
        sy = m_RelativeFootPositions[Index+1].sy ;
        sz = m_RelativeFootPositions[Index+1].sz ;
        FinalTime = moving_time ;
        interpolationTime = t-Index*moving_time ;
        FinalPos = initCoMheight ;
        FinalTime = SStime - interpolationTime ;
        if(sx*sx+sy*sy > 0.22*0.22 && sz*sz+sz*sz < 0.00001 && sx*sx+sx*sx >
           0.00001)
          {
            FinalPos = lowerCoMheight ;
          }
        InitPos   = LastCoM.z[0];
        InitSpeed = LastCoM.z[1];
        InitAcc   = LastCoM.z[2];
        m_CoMbsplinesZ->SetParameters(FinalTime,InitPos,FinalPos,ToMP,MP,
                                      InitSpeed,
                                      InitAcc) ;
        m_CoMbsplinesZ->Compute(m_SamplingPeriod,CoMz[0],CoMz[1],CoMz[2]) ;
        return ;
      }

    // variables that parameterize the trajectory of the CoM in z
    double deltaZ ;
    // double static CoMzpre = CoMz;
    double up=0.20,upRight = 0.9,upLeft = 0.0;
    double down = 0.4, downRight = 0.90, downLeft = 0.1;

    // some variables renaming which improve the readibility
    double absFootz_0 = m_AbsoluteSupportFootPositions[Index].z - corrZ(2);
    double absFootz_1 = m_AbsoluteSupportFootPositions[Index-1].z - corrZ(2);
    double absFootz_2 = 0.0 ;
    if (Index >1)
      {
        absFootz_2 = m_AbsoluteSupportFootPositions[Index-2].z - corrZ(2);
      }

    // climbing
    // put first leg on the stairs with decrease of CoM //up// of stair height
    // the CoM line will decrease between an //upLeft to upRight
    // interval of SStime.
    // the CoM line will go up between an //upLeft1 to upRight1//
    // interval of SStime while 2nd leg moving up on the stairs.
    if (absFootz_0 > absFootz_1) // first leg
      {
        deltaZ = absFootz_0 - absFootz_1;
        if (Index>1)
          InitPos = m_InitialPoseCoMHeight + absFootz_2 -
            up*(absFootz_1 - absFootz_2) ;
        else // Special case: starting the motion.
          InitPos = m_InitialPoseCoMHeight ;

        InitSpeed = 0.0 ;
        FinalPos = m_InitialPoseCoMHeight + absFootz_1 - up*deltaZ;

        interpolationTime = t-Index*moving_time - upLeft*SStime ;
        FinalTime = (upRight-upLeft)*SStime - interpolationTime ;
      }
    else if (absFootz_0 == absFootz_1
             && m_RelativeFootPositions[Index-1].sz > 0) // 2nd leg
      {
        deltaZ = (absFootz_0 - absFootz_2 );
        InitPos = m_InitialPoseCoMHeight + absFootz_2 - up*deltaZ ;
        InitSpeed = 0.0 ;
        FinalPos = m_InitialPoseCoMHeight + absFootz_0 ;

        interpolationTime = t-Index*moving_time - upLeft*SStime ;
        FinalTime = (upRight-upLeft)*SStime - interpolationTime;
      }
    // going down
    // the CoM line will decrease an //1+down// stair height
    // between an //downLeft to downRight// interval of SStime while
    // moving first leg down
    // put the 2nd leg down while standing up the CoM.
    else if (absFootz_0 < absFootz_1 )
      {
        deltaZ = absFootz_1 - absFootz_0;
        if (Index>1)
          InitPos = m_InitialPoseCoMHeight + absFootz_1 -
            down*(absFootz_2 - absFootz_1) ;
        else // Special case: starting the motion.
          InitPos = m_InitialPoseCoMHeight ;

        InitSpeed = 0.0 ;
        FinalPos = m_InitialPoseCoMHeight +  absFootz_0 - down*deltaZ ;

        interpolationTime = t-Index*moving_time - downLeft*SStime ;
        FinalTime = (downRight-downLeft)*SStime - interpolationTime;
      }
    else if (absFootz_0 == absFootz_1
             && m_RelativeFootPositions[Index-1].sz < 0) //second leg
      {
        deltaZ = absFootz_2 - absFootz_0;
        InitPos = m_InitialPoseCoMHeight + absFootz_0 - down*deltaZ ;
        InitSpeed = 0.0 ;
        FinalPos = m_InitialPoseCoMHeight + absFootz_0 ;
        interpolationTime = t-Index*moving_time - downLeft*SStime ;
        FinalTime = (downRight-downLeft)*SStime - interpolationTime ;
      }
    // normal walking
    // the com stay on the horizotal plane at the initial com altitude
    else
      {
        InitSpeed = 0.0 ;
        interpolationTime = t-Index*moving_time-SStime ;
        FinalTime = moving_time - interpolationTime ;
        InitPos = initCoMheight ;
        FinalPos = initCoMheight ;

        if(sx*sx+sy*sy > 0.22*0.22 && sz*sz+sz*sz < 0.00001 &&
           sx*sx+sx*sx > 0.00001)
          {
            if(LastCoM.z[0] >= lowerCoMheight + 0.00001
               || LastCoM.z[0] <= lowerCoMheight - 0.00001)
              {
                FinalTime = DStime - interpolationTime;
                FinalPos = lowerCoMheight ;
              }
            else
              {
                InitPos = lowerCoMheight ;
                FinalPos = lowerCoMheight ;
              }
          }
        else if(LastCoM.z[0] >= initCoMheight + 0.00001
                || LastCoM.z[0] <= initCoMheight - 0.00001)
          {
            FinalTime = SStime - interpolationTime;
            InitPos = lowerCoMheight ;
            FinalPos = initCoMheight ;
          }
        else
          {
            InitPos = initCoMheight ;
            FinalPos = initCoMheight ;
          }
      }

    //    cout << "relative position : "
    //         << sx << " "
    //         << sy << " "
    //         << dx << " "
    //         << dy << " "
    //         << SStime << " "
    //         << m_AbsoluteSupportFootPositions[Index].time << " "
    //         << m_AbsoluteSupportFootPositions[Index-1].time << " " << endl ;
    InitPos = LastCoM.z[0];
    InitSpeed = LastCoM.z[1];
    InitAcc = LastCoM.z[2];
    m_CoMbsplinesZ->SetParameters(FinalTime,InitPos,FinalPos,ToMP,MP,InitSpeed,
                                  InitAcc) ;
    m_CoMbsplinesZ->Compute(m_SamplingPeriod,CoMz[0],CoMz[1],CoMz[2]) ;
  }

  void AnalyticalMorisawaCompact::ComputeZMPz(double t,
                                              ZMPPosition &ZMPz,
                                              unsigned int IndexInterval)
  {
    // absFootz_0, the z axis is expressed in the waist frame
    // we choose the left one by default, the foot are supposed to be symetrical
    // we use it pass the ankle position to the fot position
    PRFoot *aFoot = m_PR->leftFoot() ;
    if (aFoot==0)
      LTHROW("No foot");
    Eigen::Vector3d corrZ ;
    corrZ = aFoot->anklePosition ;
    corrZ(2) = 0.0 ;
    bool sinple_support = (IndexInterval % 2) == 0 ;
    double moving_time = m_RelativeFootPositions[0].SStime +
      m_RelativeFootPositions[0].DStime;
    unsigned int stepNumber = int(t/moving_time) ;

    // we start analyze since 2nd step
    // after the final step we keep the same position for a while
    // when a foot fly the zmp.pz is the supportFoot.z
    if( stepNumber >= m_AbsoluteSupportFootPositions.size() )
      {
        ZMPz.pz = m_AbsoluteSupportFootPositions.back().z - corrZ(2) ;
        return ;
      }
    else if ( stepNumber < 1 )
      {
        ZMPz.pz = m_AbsoluteSupportFootPositions.front().z - corrZ(2);
      }
    else if ( t <= moving_time || sinple_support)
      {
        ZMPz.pz = m_AbsoluteSupportFootPositions[stepNumber-1].z - corrZ(2) ;
        return ;
      }
    else
      {
        double absFootz_0 = m_AbsoluteSupportFootPositions[stepNumber].z
          - corrZ(2) ;
        double absFootz_1 = m_AbsoluteSupportFootPositions[stepNumber-1].z
          - corrZ(2) ;
        m_ZMPpolynomeZ->
          SetParametersWithInitPosInitSpeed
          (m_RelativeFootPositions[0].DStime,
           absFootz_0,absFootz_1,0.0);
        ZMPz.pz = m_ZMPpolynomeZ->Compute(t-stepNumber*moving_time
                                          -m_RelativeFootPositions[0].SStime);
      }
    return ;
  }

  void AnalyticalMorisawaCompact::
  FillQueues
  (double StartingTime,
   double EndTime,
   deque<ZMPPosition> & FinalZMPPositions,
   deque<COMState> & FinalCoMPositions,
   deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
   deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions)
  {
    FillQueues(m_SamplingPeriod, StartingTime, EndTime, FinalZMPPositions,
               FinalCoMPositions, FinalLeftFootAbsolutePositions,
               FinalRightFootAbsolutePositions);
  }
}

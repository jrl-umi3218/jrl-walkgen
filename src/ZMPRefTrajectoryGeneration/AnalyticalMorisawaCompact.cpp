/* This object generate the reference value for the
   ZMP based on a polynomail representation
   of the ZMP following 
   "Experimentation of Humanoid Walking Allowing Immediate
   Modification of Foot Place Based on Analytical Solution"
   Morisawa, Harada, Kajita, Nakaoka, Fujiwara, Kanehiro, Hirukawa, 
   ICRA 2007, 3989--39994

   Copyright (c) 2007, 
   Olivier Stasse,
   
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

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "AnalyticalMorisawaCompact :" << x << endl
#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#if 0
#define ODEBUG(x) cerr << "AnalyticalMorisawaCompact :" <<  x << endl
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

#include <fstream>
#include <walkGenJrl/ZMPRefTrajectoryGeneration/AnalyticalMorisawaCompact.h>

typedef double doublereal;
typedef int integer;

extern "C"
{
  extern int dgesvx_( char *, char *, /* 0 FACT TRANS */
		     integer * , integer *, /* 2 N NHRS */
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
#if 0
  extern void dgetrf_( integer * m, /* M */
		       integer * n, /* N */
		       doublereal * A, /* A */
		       integer *  lda, /* LDA */
		       integer * ipiv, /* IPIV */
		       integer *info /* info */
		       ); 
#endif
}

namespace PatternGeneratorJRL
{


  AnalyticalMorisawaCompact::AnalyticalMorisawaCompact(SimplePluginManager *lSPM)
    : AnalyticalMorisawaAbstract(lSPM)
  {
    ODEBUG("Constructor");

    memset(&m_CTIPX,0,sizeof(m_CTIPX));
    memset(&m_CTIPY,0,sizeof(m_CTIPY));

    m_HS = 0;
    m_FeetTrajectoryGenerator = 0;

    m_NeedToReset = true;
    m_AbsoluteTimeReference = 0.0;

    /*! Dynamic allocation of the analytical trajectories for the ZMP and the COG */
    m_AnalyticalZMPCoGTrajectoryX = new AnalyticalZMPCOGTrajectory(7);
    m_AnalyticalZMPCoGTrajectoryY = new AnalyticalZMPCOGTrajectory(7);

    m_VerboseLevel=0;
    
    m_NewStepInTheStackOfAbsolutePosition = false;

    
    RESETDEBUG4("Test.dat");
  }


  AnalyticalMorisawaCompact::~AnalyticalMorisawaCompact()
  {
    // Display the clock for some part of the code.
    cout << "Part of the foot position computation + queue handling." << endl;
    m_Clock1.Display();
    cout << "Part of the foot change landing position" << endl;
    m_Clock2.Display();
    cout << "Part on the analytical ZMP COG trajectories and foot polynomial computation" << endl;
    m_Clock3.Display();

    if (m_AnalyticalZMPCoGTrajectoryX!=0)
      delete m_AnalyticalZMPCoGTrajectoryX;

    if (m_AnalyticalZMPCoGTrajectoryY!=0)
      delete m_AnalyticalZMPCoGTrajectoryY;
  }


  bool AnalyticalMorisawaCompact::InitializeBasicVariables()
  {
    m_NumberOfIntervals = 2 * m_NumberOfStepsInAdvance+1;

    // Compute the temporal intervals.
    m_DeltaTj.resize(m_NumberOfIntervals);
    m_Omegaj.resize(m_NumberOfIntervals);
    m_StepTypes.resize(m_NumberOfIntervals);

    
    m_DeltaTj[0]=m_Tsingle*3.0;
    //m_DeltaTj[0]=m_Tsingle*1.0;
    m_StepTypes[0] = DOUBLE_SUPPORT;
    for(unsigned int i=1;i<m_NumberOfIntervals;i++)
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
    ODEBUG("PreviewControlTime:" << m_PreviewControlTime << " " << m_Tsingle << " " << m_Tdble);
    
    if (m_VerboseLevel>=2)
      {
	for(unsigned int i=0;i<m_NumberOfIntervals;i++)
	  cout << m_DeltaTj[i] << " ";
	cout << endl;
      }

    // Specify the degrees corresponding to the given interval.
    m_PolynomialDegrees.resize(m_NumberOfIntervals);
    m_PolynomialDegrees[0] = 4;
    m_PolynomialDegrees[m_NumberOfIntervals-1] = 4;
    for(unsigned int i=1;i<m_NumberOfIntervals-1;i++)
      m_PolynomialDegrees[i] = 3;    
    

    /*! Dynamic allocation for the foot trajectory. */
    if(m_FeetTrajectoryGenerator!=0)
      {
	m_FeetTrajectoryGenerator->SetDeltaTj(m_DeltaTj);
      }
    
  }




  void AnalyticalMorisawaCompact::ComputePolynomialWeights()
  {
    MAL_MATRIX(,double) iZ;
    MAL_INVERSE(m_Z,iZ,double);

    // Compute the weights.
    MAL_C_eq_A_by_B(m_y,iZ,m_w);

    if (m_VerboseLevel>=2)
      {
	std::ofstream ofs;
	ofs.open("YMatrix.dat",ofstream::out);
	ofs.precision(10);
      
	for(unsigned int i=0;i<MAL_VECTOR_SIZE(m_y);i++)
	  {
	    ofs << m_y[i]<< " ";
	  }
	ofs << endl;
	ofs.close();
      }

  }

  void AnalyticalMorisawaCompact::ResetTheResolutionOfThePolynomial()
  {
    int SizeOfZ = MAL_MATRIX_NB_ROWS(m_Z);
      
    MAL_MATRIX_RESIZE(m_AF,SizeOfZ,2*SizeOfZ);
    MAL_VECTOR_RESIZE(m_IPIV,SizeOfZ);

    MAL_MATRIX_FILL(m_AF,0);
    MAL_VECTOR_FILL(m_IPIV,0);

    m_NeedToReset = true;
  }

  void AnalyticalMorisawaCompact::ComputePolynomialWeights2()
  {
    int SizeOfZ = MAL_MATRIX_NB_ROWS(m_Z),
      LDA,LDAF,LDB;
    int NRHS = 1;
    
    char EQUED='N';

    MAL_MATRIX(,double) tZ;
    tZ = MAL_RET_TRANSPOSE(m_Z);
    

    MAL_VECTOR(,double) lR;
    MAL_VECTOR_RESIZE(lR,SizeOfZ);
    MAL_VECTOR(,double) lC;
    MAL_VECTOR_RESIZE( lC,SizeOfZ);

    MAL_VECTOR_RESIZE(m_y,SizeOfZ);
    //MAL_VECTOR(,double) m_X;
    //b    MAL_VECTOR_RESIZE(m_X,SizeOfZ);
    LDA =  SizeOfZ;
    LDAF = SizeOfZ;
    LDB = SizeOfZ;

    double lRCOND;

    MAL_VECTOR(,double) lFERR, lBERR;
    MAL_VECTOR_RESIZE(lFERR,SizeOfZ);
    MAL_VECTOR_RESIZE(lBERR,SizeOfZ);

    int lwork = 4* SizeOfZ;
    double *work = new double[lwork];
    int *iwork = new int[SizeOfZ];
    int lsizeofx= SizeOfZ;
    int info=0;

    if (m_NeedToReset)
      {
	m_AF = MAL_RET_TRANSPOSE(m_Z);	
	dgetrf_(&SizeOfZ, /* M */
	       &SizeOfZ, /* N here M=N=SizeOfZ */
	       MAL_RET_MATRIX_DATABLOCK(m_AF), /* A */
	       &SizeOfZ, /* Leading dimension cf before */
	       MAL_RET_VECTOR_DATABLOCK(m_IPIV), /* IPIV */ 
	       &info /* info */
		);
	m_NeedToReset = false;
      }
    
    dgesvx_("F", /* Specify that AF and IPIV should be used. */
	    "N", /* A * X = B */
	    &SizeOfZ, /* Size of A */
	    &NRHS, /*Nb of columns for X et B */
	    MAL_RET_MATRIX_DATABLOCK(tZ), /* Access to A */
	    &LDA, /* Leading size of A */
	    MAL_RET_MATRIX_DATABLOCK(m_AF),
	    &LDAF,
	    MAL_RET_VECTOR_DATABLOCK(m_IPIV),
	    &EQUED,
	    MAL_RET_VECTOR_DATABLOCK(lR),
	    MAL_RET_VECTOR_DATABLOCK(lC),
	    MAL_RET_VECTOR_DATABLOCK(m_w),
	    &LDB,
	    MAL_RET_VECTOR_DATABLOCK(m_y),
	    &lsizeofx,
	    &lRCOND,
	    MAL_RET_VECTOR_DATABLOCK(lFERR),
	    MAL_RET_VECTOR_DATABLOCK(lBERR),
	    work,
	    iwork,
	    &info
	    );	   
	   
    // Compute the weights.
    //    MAL_C_eq_A_by_B(m_y,iZ,m_w);

    if (m_VerboseLevel>=2)
      {
	std::ofstream ofs;
	ofs.open("YMatrix.dat",ofstream::out);
	ofs.precision(10);
      
	for(unsigned int i=0;i<MAL_VECTOR_SIZE(m_y);i++)
	  {
	    ofs << m_y[i]<< " ";
	  }
	ofs << endl;
	ofs.close();
      }

  }

 
  int AnalyticalMorisawaCompact::BuildAndSolveCOMZMPForASetOfSteps(MAL_S3x3_MATRIX(& lStartingCOMPosition,double),
								   FootAbsolutePosition &LeftFootInitialPosition,
								   FootAbsolutePosition &RightFootInitialPosition,
								   bool IgnoreFirstRelativeFoot,
								   bool DoNotPrepareLastFoot)
  {
    
    if (m_RelativeFootPositions.size()==0)
      return -2;

    int NbSteps = m_RelativeFootPositions.size();
    int NbOfIntervals=2*NbSteps+1;

    ODEBUG("Number of steps in advance: "<< NbSteps);
    SetNumberOfStepsInAdvance(NbSteps);
    InitializeBasicVariables();
    
    vector<double> * lCoMZ;
    vector<double> * lZMPZ;
    if (m_CTIPX.CoMZ!=0)
      lCoMZ = m_CTIPX.CoMZ;
    else 
      lCoMZ = new vector<double>;

    if (m_CTIPX.ZMPZ!=0)
      lZMPZ = m_CTIPX.ZMPZ;
    else
      lZMPZ = new vector<double>;

    lCoMZ->resize(NbOfIntervals);
    lZMPZ->resize(NbOfIntervals);
    for(unsigned int i=0;i<NbOfIntervals;i++)
      {
	(*lCoMZ)[i] = lStartingCOMPosition(2,0); 
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
    
    m_AnalyticalZMPCoGTrajectoryX->SetStartingTimeIntervalsAndHeightVariation(m_DeltaTj,m_Omegaj);
    m_AnalyticalZMPCoGTrajectoryY->SetStartingTimeIntervalsAndHeightVariation(m_DeltaTj,m_Omegaj);

    /* Build the profil for the X and Y axis. */
    double InitialCoMX=0.0;
    double InitialCoMSpeedX=0.0;
    double FinalCoMPosX=0.6;
    vector<double> * lZMPX;
    if (m_CTIPX.ZMPProfil!=0)
      lZMPX = m_CTIPX.ZMPProfil;
    else
      lZMPX = new vector<double>;

    ODEBUG("NbOfIntervals: " << NbOfIntervals);
    lZMPX->resize(NbOfIntervals);
    
    double InitialCoMY=0.0;
    double InitialCoMSpeedY=0.0;
    double FinalCoMPosY=0.0;
    vector<double> * lZMPY;
    if (m_CTIPY.ZMPProfil!=0)
      lZMPY = m_CTIPY.ZMPProfil;
    else
      lZMPY = new vector<double>;
	
    lZMPY->resize(NbOfIntervals);
  
    (*lZMPX)[0] = lStartingCOMPosition(0,0); 
    (*lZMPY)[0] = lStartingCOMPosition(1,0);
    ODEBUG(" m_CTIPY COM : " << (*lZMPY)[0]);

    /*! Extract the set of initial conditions relevant for 
      computing the analytical trajectories. */
    InitialCoMX = (*lZMPX)[0];
    InitialCoMSpeedX = lStartingCOMPosition(0,1);
    InitialCoMY = (*lZMPY)[0];
    InitialCoMSpeedY = lStartingCOMPosition(1,1);
    
    /*! Extract the set of absolute coordinates for the foot position. */
    if (m_FeetTrajectoryGenerator!=0)
      {
	ODEBUG("Initialize Feet Trajectory Generator");
	m_FeetTrajectoryGenerator->SetDeltaTj(m_DeltaTj);

	m_FeetTrajectoryGenerator->InitializeFromRelativeSteps(m_RelativeFootPositions,
							       LeftFootInitialPosition,
							       RightFootInitialPosition,
							       m_AbsoluteSupportFootPositions,
							       IgnoreFirstRelativeFoot, false);
	unsigned int i=0,j=1;

	for(i=0,j=1;i<m_AbsoluteSupportFootPositions.size();i++,j+=2)
	  {
	    (*lZMPX)[j] = m_AbsoluteSupportFootPositions[i].x;
	    (*lZMPX)[j+1] = m_AbsoluteSupportFootPositions[i].x;

	    (*lZMPY)[j] = m_AbsoluteSupportFootPositions[i].y;
	    (*lZMPY)[j+1] = m_AbsoluteSupportFootPositions[i].y;	    
	  }

	FootAbsolutePosition LeftFootFinalPosition;
	FootAbsolutePosition RightFootFinalPosition;
								 
	// Strategy for the final CoM pos: middle of the segment
	// between the two final steps, in order to be statically stable.
	unsigned int lindex = m_AbsoluteSupportFootPositions.size()-1;

	if (DoNotPrepareLastFoot)
	  FinalCoMPosX = m_AbsoluteSupportFootPositions[lindex].x;
	else
	  FinalCoMPosX = 0.5 *(m_AbsoluteSupportFootPositions[lindex-1].x + 
			     m_AbsoluteSupportFootPositions[lindex].x);
	ODEBUG("j: "<< j);

	if (DoNotPrepareLastFoot)
	  (*lZMPX)[j-2] = (*lZMPX)[j-1] = m_AbsoluteSupportFootPositions[lindex].x;
	else
	  (*lZMPX)[j-2] = (*lZMPX)[j-1] = FinalCoMPosX;

	if (DoNotPrepareLastFoot)
	  FinalCoMPosY = m_AbsoluteSupportFootPositions[lindex].y;
	else
	  FinalCoMPosY = 0.5 *(m_AbsoluteSupportFootPositions[lindex-1].y + 
			     m_AbsoluteSupportFootPositions[lindex].y);

	if (DoNotPrepareLastFoot)
	  (*lZMPY)[j-2] = (*lZMPY)[j-1] = m_AbsoluteSupportFootPositions[lindex].y;
	else
	  (*lZMPY)[j-2] = (*lZMPY)[j-1] = FinalCoMPosY;

	
    
      }
    else 
      {
	ODEBUG3(" Feet Trajectory Generator NOT INITIALIZED");
	return -1;
      }

    /*! Build 3rd order polynomials. */
    for(unsigned int i=1;i<NbOfIntervals-1;i++)
      {
	m_AnalyticalZMPCoGTrajectoryX->Building3rdOrderPolynomial(i,(*lZMPX)[i-1],(*lZMPX)[i]);
	m_AnalyticalZMPCoGTrajectoryY->Building3rdOrderPolynomial(i,(*lZMPY)[i-1],(*lZMPY)[i]);
      }

    // Block for X trajectory
    m_CTIPX.InitialCoM = InitialCoMX;
    m_CTIPX.InitialCoMSpeed = InitialCoMSpeedX;
    m_CTIPX.FinalCoMPos = FinalCoMPosX;
    m_CTIPX.ZMPProfil = lZMPX;
    m_CTIPX.ZMPZ = lZMPZ;
    m_CTIPX.CoMZ = lCoMZ;
    ComputeTrajectory(m_CTIPX,*m_AnalyticalZMPCoGTrajectoryX);
	  
    // Block for Y trajectory.
    m_CTIPY.InitialCoM = InitialCoMY;
    m_CTIPY.InitialCoMSpeed = InitialCoMSpeedY;
    m_CTIPY.FinalCoMPos = FinalCoMPosY;
    m_CTIPY.ZMPProfil = lZMPY;
    m_CTIPY.ZMPZ = lZMPZ;
    m_CTIPY.CoMZ = lCoMZ;
    ComputeTrajectory(m_CTIPY,*m_AnalyticalZMPCoGTrajectoryY);

    
    return 0;

  }

  void AnalyticalMorisawaCompact::GetZMPDiscretization(deque<ZMPPosition> & ZMPPositions,
						       deque<COMPosition> & COMPositions,
						       deque<RelativeFootPosition> &RelativeFootPositions,
						       deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
						       deque<FootAbsolutePosition> &RightFootAbsolutePositions,
						       double Xmax,
						       COMPosition & lStartingCOMPosition,
						       FootAbsolutePosition & InitLeftFootAbsolutePosition,
						       FootAbsolutePosition & InitRightFootAbsolutePosition)
  {

    m_RelativeFootPositions = RelativeFootPositions;
    /* This part computes the CoM and ZMP trajectory giving the foot position information. 
       It also creates the analytical feet trajectories.
     */
    MAL_S3x3_MATRIX(lMStartingCOMPosition,double);

    lMStartingCOMPosition(0,0)= lStartingCOMPosition.x[0];
    lMStartingCOMPosition(1,0)= lStartingCOMPosition.y[0];
    lMStartingCOMPosition(2,0)= lStartingCOMPosition.z[0];
    for(unsigned int i=0;i<3;i++)
      {
	for(unsigned int j=1;j<3;j++)
	  lMStartingCOMPosition(i,j)= 0.0;
      }
    
    int r=0;
    if ((r=BuildAndSolveCOMZMPForASetOfSteps(lMStartingCOMPosition,
					     InitLeftFootAbsolutePosition,
					     InitRightFootAbsolutePosition,
					     true,false))<0)
      {
	switch(r)
	  {
	  case (-1):
	    cerr << "Error: Humanoid Specificities not initialized. " << endl;
	    break;
	  case (-2):
	    cerr << "Error: Relative Foot Size" << endl;
	    break;
	  }
	return;
      }

    /*! Compute the total size of the array related with the steps. */
    ODEBUG("m_SampligPeriod: " << m_SamplingPeriod);
    ODEBUG("m_PreviewControlTime: " << m_PreviewControlTime);

    double lSizeOfArray = m_PreviewControlTime / m_SamplingPeriod;
    ODEBUG(lSizeOfArray);

    for(double t=0.0; t<m_PreviewControlTime; t+= 0.005)
      {
	/*! Feed the ZMPPositions. */
	ZMPPosition aZMPPos;
        m_AnalyticalZMPCoGTrajectoryX->ComputeZMP(t,aZMPPos.px);
	m_AnalyticalZMPCoGTrajectoryY->ComputeZMP(t,aZMPPos.py);
	ZMPPositions.push_back(aZMPPos);

	/*! Feed the COMPositions. */
	COMPosition aCOMPos;
	memset(&aCOMPos,0,sizeof(aCOMPos));
	m_AnalyticalZMPCoGTrajectoryX->ComputeCOM(t,aCOMPos.x[0]);
	m_AnalyticalZMPCoGTrajectoryY->ComputeCOM(t,aCOMPos.y[0]);
	COMPositions.push_back(aCOMPos);

	/*! Feed the FootPositions. */

	/*! Left */
	FootAbsolutePosition LeftFootAbsPos;
	m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(1,t,LeftFootAbsPos);
	LeftFootAbsolutePositions.push_back(LeftFootAbsPos);

	/*! Right */
	FootAbsolutePosition RightFootAbsPos;
	m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(-1,t,RightFootAbsPos);
	RightFootAbsolutePositions.push_back(RightFootAbsPos);

	ODEBUG4(aZMPPos.px << " " << aZMPPos.py << " " << aCOMPos.x[0] << " " << aCOMPos.y[0] << " " << 
		LeftFootAbsPos.x << " " << LeftFootAbsPos.y << " " << LeftFootAbsPos.z << " " << 
		RightFootAbsPos.x << " " << RightFootAbsPos.y << " " << RightFootAbsPos.z << " " ,"Test.dat");
      }

  }

  int AnalyticalMorisawaCompact::InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
					    deque<COMPosition> & CoMPositions,
					    deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
					    deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
					    FootAbsolutePosition & InitLeftFootAbsolutePosition,
					    FootAbsolutePosition & InitRightFootAbsolutePosition,
					    deque<RelativeFootPosition> &RelativeFootPositions,
					    COMPosition & lStartingCOMPosition)
  {
    ODEBUG("Begin InitOnLine");
    m_RelativeFootPositions.clear();

    unsigned int r = RelativeFootPositions.size();
    unsigned int maxrelsteps = r < 3 ? r : 3;
    ODEBUG("Number of relative steps: "<< maxrelsteps);
    MAL_S3x3_MATRIX(lMStartingCOMPosition,double);

    lMStartingCOMPosition(0,0)= lStartingCOMPosition.x[0];
    lMStartingCOMPosition(1,0)= lStartingCOMPosition.y[0];
    lMStartingCOMPosition(2,0)= lStartingCOMPosition.z[0];
    for(unsigned int i=0;i<3;i++)
      {
	for(unsigned int j=1;j<3;j++)
	  lMStartingCOMPosition(i,j)= 0.0;
      }
    
    for(int i=0;i< maxrelsteps;i++)
      m_RelativeFootPositions.push_back(RelativeFootPositions[i]);
    
    if (m_RelativeFootPositions[0].sy < 0)
      m_AbsoluteCurrentSupportFootPosition = InitRightFootAbsolutePosition;
    else 
      m_AbsoluteCurrentSupportFootPosition = InitLeftFootAbsolutePosition;

    ODEBUG("InitLeftFootAbsolutePosition: " << InitLeftFootAbsolutePosition.x << " " 
	    << InitLeftFootAbsolutePosition.y);

    ODEBUG("InitRightFootAbsolutePosition: " << InitRightFootAbsolutePosition.x << " " 
	    << InitRightFootAbsolutePosition.y);


    /* This part computes the CoM and ZMP trajectory giving the foot position information. 
       It also creates the analytical feet trajectories.
     */
    if (BuildAndSolveCOMZMPForASetOfSteps(lMStartingCOMPosition,
					  InitLeftFootAbsolutePosition,
					  InitRightFootAbsolutePosition,
					  true,true)<0)
      {
	cerr<< "Error: Humanoid Specificities not initialized. " << endl;
	return -1;
      }
    

    
    double lSizeOfArray = m_PreviewControlTime / m_SamplingPeriod;
    ODEBUG(lSizeOfArray);
    
    ODEBUG( " t1: " << m_Tsingle*2 << " t2: " << 4*m_Tsingle+m_Tdble <<
	     " t1: " << m_Tsingle*2/m_SamplingPeriod << " t2: " << (4*m_Tsingle+m_Tdble)/m_SamplingPeriod );

    m_AbsoluteTimeReference = m_CurrentTime-m_Tsingle*2;
    m_AnalyticalZMPCoGTrajectoryX->SetAbsoluteTimeReference(m_AbsoluteTimeReference);
    m_AnalyticalZMPCoGTrajectoryY->SetAbsoluteTimeReference(m_AbsoluteTimeReference);
    m_FeetTrajectoryGenerator->SetAbsoluteTimeReference(m_AbsoluteTimeReference);

    ODEBUG("Interval Test.dat : begin : " << m_CurrentTime << 
	   " end : " << m_Tsingle+2*m_Tdble);

    //    for(double t=0; t<m_Tsingle+2*m_Tdble; t+= 0.005)

    for(double t=m_CurrentTime; 
	t<m_CurrentTime+2*m_SamplingPeriod; 
	t+= m_SamplingPeriod)
      {
	/*! Feed the ZMPPositions. */
	ZMPPosition aZMPPos;
        m_AnalyticalZMPCoGTrajectoryX->ComputeZMP(t,aZMPPos.px);
	m_AnalyticalZMPCoGTrajectoryY->ComputeZMP(t,aZMPPos.py);
	FinalZMPPositions.push_back(aZMPPos);

	/*! Feed the COMPositions. */
	COMPosition aCOMPos;
	memset(&aCOMPos,0,sizeof(aCOMPos));
	m_AnalyticalZMPCoGTrajectoryX->ComputeCOM(t,aCOMPos.x[0]);
	m_AnalyticalZMPCoGTrajectoryY->ComputeCOM(t,aCOMPos.y[0]);
	CoMPositions.push_back(aCOMPos);

	/*! Feed the FootPositions. */

	/*! Left */
	ODEBUG("t: "<< t);
	FootAbsolutePosition LeftFootAbsPos;
	m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(1,t,LeftFootAbsPos);
	FinalLeftFootAbsolutePositions.push_back(LeftFootAbsPos);
	ODEBUG("Test.dat|LFA.xy " << LeftFootAbsPos.x << " " << LeftFootAbsPos.y);
	/*! Right */
	FootAbsolutePosition RightFootAbsPos;
	m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(-1,t,RightFootAbsPos);
	FinalRightFootAbsolutePositions.push_back(RightFootAbsPos);
	
	ODEBUG4(aZMPPos.px << " " << aZMPPos.py << " " << aCOMPos.x[0] << " " << aCOMPos.y[0] << " " << 
		LeftFootAbsPos.x << " " << LeftFootAbsPos.y << " " << LeftFootAbsPos.z << " " << 
		RightFootAbsPos.x << " " << RightFootAbsPos.y << " " << RightFootAbsPos.z << " " ,"Test.dat");
      }
    m_UpperTimeLimitToUpdateStacks = m_CurrentTime + m_Tsingle+m_Tdble;
#if 0
    ODEBUG3("Interval Valid_001.dat : begin : " << m_AbsoluteTimeReference << 
	    " end : " << m_AbsoluteTimeReference+m_PreviewControlTime);
    RESETDEBUG4("Valid_001.dat");
    for(double t=m_AbsoluteTimeReference; 
	t<m_AbsoluteTimeReference+m_PreviewControlTime; 
	t+= m_SamplingPeriod)
      {
	/*! Feed the ZMPPositions. */
	ZMPPosition aZMPPos;
        m_AnalyticalZMPCoGTrajectoryX->ComputeZMP(t,aZMPPos.px);
	m_AnalyticalZMPCoGTrajectoryY->ComputeZMP(t,aZMPPos.py);

	/*! Feed the COMPositions. */
	COMPosition aCOMPos;
	memset(&aCOMPos,0,sizeof(aCOMPos));
	m_AnalyticalZMPCoGTrajectoryX->ComputeCOM(t,aCOMPos.x[0]);
	m_AnalyticalZMPCoGTrajectoryY->ComputeCOM(t,aCOMPos.y[0]);

	/*! Feed the FootPositions. */

	/*! Left */
	FootAbsolutePosition LeftFootAbsPos;
	m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(1,t,LeftFootAbsPos);
	ODEBUG("Valid_001.dat|LFA.xy " << t << " | " << LeftFootAbsPos.x << " " << LeftFootAbsPos.y);
	/*! Right */
	FootAbsolutePosition RightFootAbsPos;
	m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(-1,t,RightFootAbsPos);
	
	ODEBUG4(t << " " << 
		aZMPPos.px << " " << aZMPPos.py << " " << 
		aCOMPos.x[0] << " " << aCOMPos.y[0] << " " << 
		LeftFootAbsPos.x << " " << LeftFootAbsPos.y << " " << LeftFootAbsPos.z << " " << 
		RightFootAbsPos.x << " " << RightFootAbsPos.y << " " << RightFootAbsPos.z << " " ,"Valid_001.dat");
      }
#endif
    
    
    ODEBUG("End of InitOnLine : Size of relative foot positions: " << m_RelativeFootPositions.size());

    return m_RelativeFootPositions.size();
  }


  void AnalyticalMorisawaCompact::OnLine(double time,
					 deque<ZMPPosition> & FinalZMPPositions,				     
					 deque<COMPosition> & FinalCOMPositions,
					 deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
					 deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)
  {
    unsigned int lIndexInterval;
    ODEBUG(" m_AnalyticalZMPCoGTrajectoryX " << m_AnalyticalZMPCoGTrajectoryX);
    if (time<m_UpperTimeLimitToUpdateStacks) 
      {
	if (m_AnalyticalZMPCoGTrajectoryX->GetIntervalIndexFromTime(time,lIndexInterval))
	  {
	    
	    /*! Feed the ZMPPositions. */
	    ZMPPosition aZMPPos;
	    m_AnalyticalZMPCoGTrajectoryX->ComputeZMP(time,aZMPPos.px,lIndexInterval);
	    m_AnalyticalZMPCoGTrajectoryY->ComputeZMP(time,aZMPPos.py,lIndexInterval);
	    FinalZMPPositions.push_back(aZMPPos);
	    
	    /*! Feed the COMPositions. */
	    COMPosition aCOMPos;
	    memset(&aCOMPos,0,sizeof(aCOMPos));
	    m_AnalyticalZMPCoGTrajectoryX->ComputeCOM(time,aCOMPos.x[0],lIndexInterval);
	    m_AnalyticalZMPCoGTrajectoryY->ComputeCOM(time,aCOMPos.y[0],lIndexInterval);
	    FinalCOMPositions.push_back(aCOMPos);
	    /*! Feed the FootPositions. */
    
	    /*! Left */
	    FootAbsolutePosition LeftFootAbsPos;
	    memset(&LeftFootAbsPos,0,sizeof(LeftFootAbsPos));
	    m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(1,time,LeftFootAbsPos,lIndexInterval);
	    FinalLeftFootAbsolutePositions.push_back(LeftFootAbsPos);
    
	    /*! Right */
	    FootAbsolutePosition RightFootAbsPos;
	    memset(&RightFootAbsPos,0,sizeof(RightFootAbsPos));
	    m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(-1,time,RightFootAbsPos,lIndexInterval);
	    FinalRightFootAbsolutePositions.push_back(RightFootAbsPos);
	  }
      }
  }

  void AnalyticalMorisawaCompact::OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
						deque<ZMPPosition> & FinalZMPPositions,				     
						deque<COMPosition> & FinalCoMPositions,				     
						deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
						deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
						bool EndSequence)
  {
    ODEBUG("****************** Begin OnLineAddFoot **************************");

    double NewPosX, NewPosY;
    unsigned int IndexLastZMPProfil = m_CTIPX.ZMPProfil->size();//m_CTIPX.ZMPProfil->size()-2;

    // The strategy is simple: we trigger a false modification of the last
    // step and call change landing position, just after updating the stack
    // of relative foot positions.

    m_Clock1.StartTiming();

    // Update the stack of relative foot positions.
    m_RelativeFootPositions.pop_front();
    m_RelativeFootPositions.push_back(NewRelativeFootPosition);    

    ODEBUG("Beginning of OnLine : Size of relative foot positions: " << m_RelativeFootPositions.size());
    deque<FootAbsolutePosition> aQAFP;
    ODEBUG("Current LeftFootAbsolutePosition: " << FinalLeftFootAbsolutePositions[0].x  << " " 
	    << " " << FinalLeftFootAbsolutePositions[0].y 
	    << " " << FinalLeftFootAbsolutePositions[0].dx 
	    << " " << FinalLeftFootAbsolutePositions[0].dy);
    ODEBUG("Current RightFootAbsolutePosition: " << FinalRightFootAbsolutePositions[0].x  << " " 
	    << " " << FinalRightFootAbsolutePositions[0].y
	    << " " << FinalRightFootAbsolutePositions[0].dx
	    << " " << FinalRightFootAbsolutePositions[0].dy);
    ODEBUG("New relativeFootPositions when adding a foot.");

    m_FeetTrajectoryGenerator->ComputeAbsoluteStepsFromRelativeSteps(m_RelativeFootPositions,
								     FinalLeftFootAbsolutePositions[0],
								     FinalRightFootAbsolutePositions[0],
								     aQAFP);
								     
    FootAbsolutePosition aNewFootAbsPos=aQAFP.back();
    
    ODEBUG("New Foot Position: " << aNewFootAbsPos.x << " " << aNewFootAbsPos.y );
    ODEBUG("Current Time: " << m_CurrentTime );
    m_AbsoluteCurrentSupportFootPosition = m_AbsoluteSupportFootPositions[0];

    m_AbsoluteSupportFootPositions.pop_front();
    m_AbsoluteSupportFootPositions.push_back(aNewFootAbsPos);    

    /* Indicates that the step has to be taken into account appropriatly
       to compute the trajectory. */
    m_NewStepInTheStackOfAbsolutePosition = true;

    m_Clock1.StopTiming();
    m_Clock1.IncIteration();
    
    m_Clock2.StartTiming();

    ChangeFootLandingPosition(m_CurrentTime,
			      IndexLastZMPProfil,
			      aNewFootAbsPos,
			      *m_AnalyticalZMPCoGTrajectoryX,
			      m_CTIPX,
			      *m_AnalyticalZMPCoGTrajectoryY,
			      m_CTIPY,false);

    /* Indicates that the step has been taken into account appropriatly
       in computing the trajectory. */
    m_NewStepInTheStackOfAbsolutePosition = false;

    m_Clock2.StopTiming();
    m_Clock2.IncIteration();

    m_Clock3.StartTiming();
    /*! Feed the sequence with the new trajectory. */
    
    unsigned int lIndexInterval,lPrevIndexInterval;
    m_AnalyticalZMPCoGTrajectoryX->GetIntervalIndexFromTime(m_AbsoluteTimeReference,lIndexInterval);
    lPrevIndexInterval = lIndexInterval;

    /* Current strategy : add 2 values, and update at each iteration the stack.
       When the limit is reached, and the stack exhausted this method is called again.
     */
    //    for(double t=m_AbsoluteTimeReference; t<m_AbsoluteTimeReference+m_Tsingle+m_Tdble; t+= 0.005)
    for(double t=m_AbsoluteTimeReference; t<m_AbsoluteTimeReference+2*m_SamplingPeriod; t+= m_SamplingPeriod)
      {
	m_AnalyticalZMPCoGTrajectoryX->GetIntervalIndexFromTime(t,lIndexInterval,lPrevIndexInterval);

	/*! Feed the ZMPPositions. */
	ZMPPosition aZMPPos;
        m_AnalyticalZMPCoGTrajectoryX->ComputeZMP(t,aZMPPos.px,lIndexInterval);
	m_AnalyticalZMPCoGTrajectoryY->ComputeZMP(t,aZMPPos.py,lIndexInterval);
	FinalZMPPositions.push_back(aZMPPos);

		
	/*! Feed the COMPositions. */
	COMPosition aCOMPos;
	memset(&aCOMPos,0,sizeof(aCOMPos));
	m_AnalyticalZMPCoGTrajectoryX->ComputeCOM(t,aCOMPos.x[0],lIndexInterval);
	m_AnalyticalZMPCoGTrajectoryY->ComputeCOM(t,aCOMPos.y[0],lIndexInterval);
	FinalCoMPositions.push_back(aCOMPos);
	/*! Feed the FootPositions. */

	/*! Left */
	FootAbsolutePosition LeftFootAbsPos;
	memset(&LeftFootAbsPos,0,sizeof(LeftFootAbsPos));
	m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(1,t,LeftFootAbsPos,lIndexInterval);
	FinalLeftFootAbsolutePositions.push_back(LeftFootAbsPos);

	/*! Right */
	FootAbsolutePosition RightFootAbsPos;
	memset(&RightFootAbsPos,0,sizeof(RightFootAbsPos));
	m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(-1,t,RightFootAbsPos,lIndexInterval);
	FinalRightFootAbsolutePositions.push_back(RightFootAbsPos);

	ODEBUG4(aZMPPos.px << " " << aZMPPos.py << " " << aCOMPos.x[0] << " " << aCOMPos.y[0] << " " << 
		LeftFootAbsPos.x << " " << LeftFootAbsPos.y << " " << LeftFootAbsPos.z << " " << 
		RightFootAbsPos.x << " " << RightFootAbsPos.y << " " << RightFootAbsPos.z << " " ,"Test.dat");
      }
    m_Clock3.StopTiming();
    m_Clock3.IncIteration();

    /* Update the time at which the stack should not be updated anymore */
    m_UpperTimeLimitToUpdateStacks = m_AbsoluteTimeReference + m_Tsingle+ m_Tdble;    
#if 0
    static unsigned int NbOfTests = 2;
    char BufferFileName[1024];
    sprintf(BufferFileName,"Valid_%03d.dat",NbOfTests++);
    RESETDEBUG4(BufferFileName);
    ODEBUG3(BufferFileName);
    for(double t=m_AbsoluteTimeReference; t<m_AbsoluteTimeReference+m_PreviewControlTime; t+= 0.005)
      {
	/*! Feed the ZMPPositions. */
	ZMPPosition aZMPPos;
        m_AnalyticalZMPCoGTrajectoryX->ComputeZMP(t,aZMPPos.px);
	m_AnalyticalZMPCoGTrajectoryY->ComputeZMP(t,aZMPPos.py);

	/*! Feed the COMPositions. */
	COMPosition aCOMPos;
	memset(&aCOMPos,0,sizeof(aCOMPos));
	m_AnalyticalZMPCoGTrajectoryX->ComputeCOM(t,aCOMPos.x[0]);
	m_AnalyticalZMPCoGTrajectoryY->ComputeCOM(t,aCOMPos.y[0]);

	/*! Feed the FootPositions. */

	/*! Left */
	FootAbsolutePosition LeftFootAbsPos;
	m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(1,t,LeftFootAbsPos);
	ODEBUG(BufferFileName << " " << t << " | " << LeftFootAbsPos.x << " " << LeftFootAbsPos.y);

	/*! Right */
	FootAbsolutePosition RightFootAbsPos;
	m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(-1,t,RightFootAbsPos);
	
	ODEBUG4(t << " " << 
		aZMPPos.px << " " << aZMPPos.py << " " << aCOMPos.x[0] << " " << aCOMPos.y[0] << " " << 
		LeftFootAbsPos.x << " " << LeftFootAbsPos.y << " " << LeftFootAbsPos.z << " " << 
		RightFootAbsPos.x << " " << RightFootAbsPos.y << " " << RightFootAbsPos.z << " " ,BufferFileName);
      }
#endif

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
    MAL_VECTOR_RESIZE(m_w, 2 * m_NumberOfIntervals + 6);
  
    // Initial CoM Position
    ODEBUG("Ligne 1: " << InitialCoMPos << " " << ZMPPosSequence[0] );
    m_w(lindex)= InitialCoMPos - ZMPPosSequence[0];
    lindex++;
    // Initial CoM Speed
    ODEBUG("Ligne 2: " << InitialCoMSpeed );
    m_w(lindex) = InitialCoMSpeed;
    lindex++;

    // Just be carefull:
    // at iteration j, we still build m_w for the 
    // interval j-1.
    for(unsigned int j=1;j<m_NumberOfIntervals;j++)
      {
	// Takes back the polynomial needed to compute m_w.
	Polynome *aPolynomeNext,*aPolynome;
	aAZCT.GetFromListOfCOGPolynomials(j,aPolynomeNext);
	vector<double> NextCoeffsFromCOG, CoeffsFromCOG;
	aPolynomeNext->GetCoefficients(NextCoeffsFromCOG);
	if (j==1)
	  {
	    ODEBUG("Ligne "<< lindex << ": " << NextCoeffsFromCOG[0]<< " " <<  ZMPPosSequence[0] );
	    m_w[lindex] = NextCoeffsFromCOG[0] - ZMPPosSequence[0];
	    lindex++;
	    ODEBUG("Ligne "<< lindex << ": " << NextCoeffsFromCOG[1] );
	    m_w[lindex] = NextCoeffsFromCOG[1];
	    lindex++;
	    ODEBUG("Ligne "<< lindex << ": 0.0");
	    m_w[lindex] = 0;//ZMPPosSequence[0] - ZMPPosSequence[0];
	    lindex++;
	    ODEBUG("Ligne "<< lindex << ": 0.0");
	    m_w[lindex] = 0;
	    lindex++;
	  }
	else
	  {
	    aAZCT.GetFromListOfCOGPolynomials(j-1,aPolynome);
	    aPolynome->GetCoefficients(CoeffsFromCOG);
	    double r1=0.0,r2=0.0;
	    double deltat1=1.0,deltat2=1.0;
	    ODEBUG("Inside ComputeCompacteW: " << j-1 << " " << m_DeltaTj[j-1]);
	    for (unsigned int k=0;k<CoeffsFromCOG.size();k++)
	      {
		ODEBUG(" COMcoeffs : " << k << " : " << CoeffsFromCOG[k]);
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
		ODEBUG("m_w: " << m_w[lindex] << " r1: " << r1 << " NextCoeffsFromCOG[0] " << NextCoeffsFromCOG[0]);
	      }
	    else 
	      {
		m_w[lindex] = ZMPPosSequence[j-1] - r1;
		ODEBUG("m_w: " << m_w[lindex] << " r1: " << r1 << " ZMPPosSequence[" <<j-1<< "] "
			<< ZMPPosSequence[j-1]);
	      }
	    lindex++;
	    if (j!=m_NumberOfIntervals-1)
	      {
		m_w[lindex] = NextCoeffsFromCOG[1] - r2;
		ODEBUG("m_w: " << m_w[lindex] << " r2: " << r2 << " NextCoeffsFromCOG[1] " << NextCoeffsFromCOG[1]);
	      }
	    else
	      {
		m_w[lindex] = - r2;
		ODEBUG("m_w: " << m_w[lindex] << " -r2: " << -r2);
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
      
	for(unsigned int i=0;i<MAL_VECTOR_SIZE(m_w);i++)
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
    double SquareOmegaj=Omegaj*Omegaj;
    double Omegam=m_Omegaj[m_NumberOfIntervals-1];
    
    
    double Deltat = m_DeltaTj[intervalindex];  
    double c0=0.0,s0=0.0;
    m_Z(rowindex,colindex) = c0 = cosh(Omegaj * m_DeltaTj[intervalindex]);
    m_Z(rowindex,colindex+1) = s0 = sinh(Omegaj * m_DeltaTj[intervalindex]);

    if (intervalindex!=m_NumberOfIntervals-2)
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
    if (intervalindex!=m_NumberOfIntervals-2)
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
    m_Z(rowindex,colindex+1) = Deltat + m_DeltaTj[intervalindex] * 6.0 /SquareOmegam;

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
    m_Z(rowindex,colindex+2) = Deltat - 12 * m_DeltaTj[intervalindex] * m_DeltaTj[intervalindex]/ SquareOmegam;

    rowindex++;
  
  
    // Fourth row: Terminal velocity for the ZMP position
    Deltat = m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex) = 2 * Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+1) = 3 * Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = 4 * Deltat - 24.0 * m_DeltaTj[intervalindex] / SquareOmegam;

    rowindex++;
  

  }
  void AnalyticalMorisawaCompact::BuildingTheZMatrix(vector<double> &lCoM, vector<double> &lZMP )
  {

    if ((lCoM.size()!=m_NumberOfIntervals) || (lZMP.size()!=m_NumberOfIntervals))
      return;

    for(unsigned int i=0;i<lCoM.size();i++)
      {
	m_Omegaj[i]=sqrt(9.86/ (lCoM[i] - lZMP[i]));
      }
    BuildingTheZMatrix();
  }
  
  void AnalyticalMorisawaCompact::BuildingTheZMatrix()
  {
    unsigned NbRows, NbCols;
    unsigned int i,j;
    unsigned int rowindex=0;
    unsigned int colindex=0;

    NbRows = 2+4+2*(m_NumberOfIntervals-2)+4;
    NbCols = 2*m_NumberOfIntervals + 6;
    ODEBUG( "Rows: " << NbRows << " Cols: " << NbCols);
    MAL_MATRIX_RESIZE(m_Z,NbRows,NbCols);

    // Initial condition for the COG position and the velocity 
    double c0=0.0,s0= 0.0;
    double SquareOmega0 = m_Omegaj[0]*m_Omegaj[0];
  
    MAL_MATRIX_FILL(m_Z,0.0);
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
    for(int i=1;i<m_NumberOfIntervals-1;++i)
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
	for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(m_Z);i++)
	  {
	    for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(m_Z);j++)
	      {
		ofs << m_Z(i,j) << " ";
	      }
	    ofs << endl;
	    
	  }
	ofs.close();
      }
  
  }

  void AnalyticalMorisawaCompact::TransfertTheCoefficientsToTrajectories(AnalyticalZMPCOGTrajectory &aAZCT,
									 vector<double> &lCoMZ, 
									 vector<double> &lZMPZ,
									 double &lZMPInit,
									 double &lZMPEnd,
									 bool InitializeAZCT)
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
  
    for(unsigned int k=2;k<=m_PolynomialDegrees[0];k++)
      {
	coeff[k] = m_y[lindex++];
      }
    coeff[1] = coeff[3] * 6.0/(m_Omegaj[0]*m_Omegaj[0]);
    coeff[0] = lZMPInit + coeff[2] * 2.0/(m_Omegaj[0]*m_Omegaj[0]);
  
    aPolynome->SetCoefficients(coeff);

    lW.resize(m_NumberOfIntervals);


    for(unsigned int i=0;i<m_NumberOfIntervals-1;i++)
      {
	lV[i] = m_y[lindex++];
	lW[i] = m_y[lindex++];
      }

    for(unsigned int k=2;k<=m_PolynomialDegrees[m_NumberOfIntervals-1];k++)
      {
	coeff[k] = m_y[lindex++];
      }
    coeff[1] = coeff[3] * 6.0/(m_Omegaj[m_NumberOfIntervals-1]*m_Omegaj[m_NumberOfIntervals-1]);
    coeff[0] = lZMPEnd + coeff[2] * 2.0/(m_Omegaj[m_NumberOfIntervals-1]*m_Omegaj[m_NumberOfIntervals-1]);


    aAZCT.GetFromListOfCOGPolynomials(m_NumberOfIntervals-1,aPolynome);
    aPolynome->SetCoefficients(coeff);

    lV[m_NumberOfIntervals-1] = m_y[lindex++];
    lW[m_NumberOfIntervals-1] = m_y[lindex++];

    // Set the hyperbolic weights.
    aAZCT.SetCoGHyperbolicCoefficients(lV,lW);

    // Compute the ZMP weights from the CoG's ones:

    // for the first interval
    aAZCT.TransfertOneIntervalCoefficientsFromCOGTrajectoryToZMPOne(0,lCoMZ[0],lZMPZ[0]);
  
    // and the last interval
    aAZCT.TransfertOneIntervalCoefficientsFromCOGTrajectoryToZMPOne(m_NumberOfIntervals-1,
								    lCoMZ[m_NumberOfIntervals-1],
								    lZMPZ[m_NumberOfIntervals-1]);
  }

  void AnalyticalMorisawaCompact::ComputeTrajectory(CompactTrajectoryInstanceParameters &aCTIP,
						    AnalyticalZMPCOGTrajectory &aAZCT)
  {
    ComputeW(aCTIP.InitialCoM,
	     aCTIP.InitialCoMSpeed,
	     *aCTIP.ZMPProfil,
	     aCTIP.FinalCoMPos,
	     aAZCT);
    ComputePolynomialWeights2();
    TransfertTheCoefficientsToTrajectories(aAZCT,
					   *aCTIP.CoMZ,
					   *aCTIP.ZMPZ,
					   (*aCTIP.ZMPProfil)[0], 
					   (*aCTIP.ZMPProfil)[m_NumberOfIntervals-1],
					   false);  
    
  }


  int AnalyticalMorisawaCompact::TimeChange(double LocalTime,
					    unsigned int IndexStep,
					    unsigned int &IndexStartingInterval,
					    double &FinalTime,
					    double &NewTj)
  {

    // The Index Step can be equal to m_NumberOfIntervals.
    if (IndexStep<m_NumberOfIntervals)
      
      if (m_StepTypes[IndexStep]!=DOUBLE_SUPPORT)
	{
	  ODEBUG3("ERROR WRONG FOOT TYPE");
	  return ERROR_WRONG_FOOT_TYPE;
	}

    FinalTime = 0.0;
    for(unsigned int j=0;j<m_DeltaTj.size();j++)
      FinalTime+=m_DeltaTj[j];

    /* Find from which interval we are starting. */
    double reftime=0.0;
    
    for(unsigned int j=0;j<m_DeltaTj.size();j++)
      {

	if ((LocalTime>=reftime) && (LocalTime<=reftime+m_DeltaTj[j]))
	  {
	    IndexStartingInterval = j;
	    break;
	  }
	reftime+=m_DeltaTj[j];
      }

    NewTj = m_DeltaTj[IndexStartingInterval] - LocalTime  + reftime;
    
    /* Not enough time to change foot localisation 
       if this is the next step which should be changed
       and we went over half the current interval. 
     */
    if ((NewTj<m_Tsingle*0.5) &&
	(IndexStep==IndexStartingInterval+1))
      {
	ODEBUG(" NewTj : " << NewTj << " LT: " << LocalTime << " "<< reftime);
	return ERROR_TOO_LATE_FOR_MODIFICATION;
      }

    return 0;
  }

  void AnalyticalMorisawaCompact::NewTimeIntervals(unsigned int IndexStartingInterval,
						   double NewTime)
  {

    /* Build the new time interval. */
    m_DeltaTj[0] = NewTime;
    m_StepTypes[0] = m_StepTypes[IndexStartingInterval];

    for(unsigned int i=1;i<m_NumberOfIntervals;i++)
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
    ComputePreviewControlTimeWindow();

  }

  void AnalyticalMorisawaCompact::ConstraintsChange(double LocalTime,
						    FluctuationParameters FPX,
						    FluctuationParameters FPY,
						    CompactTrajectoryInstanceParameters &aCTIPX,
						    CompactTrajectoryInstanceParameters &aCTIPY,
						    unsigned int IndexStartingInterval,
						    StepStackHandler *aStepStackHandler)
  {
    ODEBUG("ConstraintsChange: " << IndexStartingInterval << " " << aCTIPX.ZMPProfil->size());

    if (IndexStartingInterval!=0)
      {
	/* Shift the current value of the profil. */
	unsigned int i,j;
	for(i=IndexStartingInterval,j=0;i<m_NumberOfIntervals;i++,j++)
	  {
	    /* Shift the ZMP profil */
	    (*aCTIPX.ZMPProfil)[j] = (*aCTIPX.ZMPProfil)[i];
	    (*aCTIPY.ZMPProfil)[j] = (*aCTIPY.ZMPProfil)[i];
	  }
	ODEBUG("ConstraintsChange: " << j << " " 
		<< m_AbsoluteSupportFootPositions.size()  << " "
		<< (*aCTIPX.ZMPProfil).size() << " " 
		<< (*aCTIPY.ZMPProfil).size()  );
		    
	/* Add value from the provided steps stack. 
	   BE CAREFUL: There is a modification on the initial value
	   depending if a m_AbsoluteSupportFootPositions has been updated
	   or not. 
	   If m_AbsoluteFootPositions has not been updated then
	   a demand for a new step will triggered.
	 */
	
	unsigned int k = 0;

	if (m_NewStepInTheStackOfAbsolutePosition)
	  k = (i-3)/2;
	else 
	  k = (i-1)/2;

	ODEBUG("Starting value for k:" << k);
	for(;
	    (k < m_AbsoluteSupportFootPositions.size()) &&
	      (j< m_CTIPX.ZMPProfil->size());k++,j+=2)
	  {
		    
	    (*aCTIPX.ZMPProfil)[j] = m_AbsoluteSupportFootPositions[k].x;
	    (*aCTIPY.ZMPProfil)[j] = m_AbsoluteSupportFootPositions[k].y;
	    
	    if ((j+1)<m_CTIPX.ZMPProfil->size())
	      {
		(*aCTIPX.ZMPProfil)[j+1] = m_AbsoluteSupportFootPositions[k].x;
		(*aCTIPY.ZMPProfil)[j+1] = m_AbsoluteSupportFootPositions[k].y;
	      }

	  }
	/* Complete the ZMP profil when no other step is available,
	   and if there is a StepStack Handler available.
	 */
	if (aStepStackHandler!=0)
	  {
	    ODEBUG("Start the generation of steps");
	    /* Compute the number of steps needed. */
	    int NeededSteps = (aCTIPX.ZMPProfil->size()-j+1)/2;
	    int r;
	    /* Test if there is enough step in the stack of. 
	       We have to remove one, because there is still the last foot added.
	     */
	    if ((r=aStepStackHandler->ReturnStackSize()-1-NeededSteps)<0)
	      {
		bool lNewStep=false;
		double NewStepX=0.0,NewStepY=0.0,NewStepTheta=0.0;
		for(int li=0;li<-r;li++)
		  {
		    aStepStackHandler->AddStandardOnLineStep(lNewStep,
							     NewStepX,
							     NewStepY,
							     NewStepTheta);
		  }		
	      }
	    ODEBUG(" NeededSteps : " << NeededSteps << 
		    "  " <<aStepStackHandler->ReturnStackSize() << 
		    " r: " << r);
	    bool EndSequence = aStepStackHandler->RemoveFirstStepInTheStack();

	    /* Takes the number of Relative Foot Positions needed. */
	    deque<RelativeFootPosition> lRelativeFootPositions;
	    aStepStackHandler->CopyRelativeFootPosition(lRelativeFootPositions,false);
	    
	    deque<FootAbsolutePosition> lAbsoluteSupportFootPositions;
	    int lLastIndex = m_AbsoluteSupportFootPositions.size()-1;
	    m_FeetTrajectoryGenerator->ComputeAbsoluteStepsFromRelativeSteps(lRelativeFootPositions,
									     m_AbsoluteSupportFootPositions[lLastIndex],
									     lAbsoluteSupportFootPositions);
	    
	    /* Add the absolute support foot positions necessary. */
	    for(int li=0;(li<NeededSteps)&& (j< m_CTIPX.ZMPProfil->size());li++)
	      {
		
		(*aCTIPX.ZMPProfil)[j] = lAbsoluteSupportFootPositions[li].x;
		(*aCTIPY.ZMPProfil)[j] = lAbsoluteSupportFootPositions[li].y;
		
		if ((j+1)<m_CTIPX.ZMPProfil->size())
		  {
		    (*aCTIPX.ZMPProfil)[j+1] = lAbsoluteSupportFootPositions[li].x;
		    (*aCTIPY.ZMPProfil)[j+1] = lAbsoluteSupportFootPositions[li].y;
		  }		
	      }
						  
	    /* Add the relative foot position inside the internal stack as well as 
	       the absolute foot position. It also removes the steps
	       inside the StepStackHandler object taken into account.*/
	    for(int li=0;li<NeededSteps;li++)
	      {
		m_RelativeFootPositions.push_back(lRelativeFootPositions[li]);    
		m_AbsoluteSupportFootPositions.push_back(lAbsoluteSupportFootPositions[li]);
		aStepStackHandler->RemoveFirstStepInTheStack();
	      }
	    /*! Remove the corresponding step from the stack of relative and absolute
	      foot positions. */
	    ODEBUG("IndexStartingInterval: " <<IndexStartingInterval << " " << IndexStartingInterval/2);
	    for(int li=0;li<IndexStartingInterval/2;li++)
	      {
		m_RelativeFootPositions.pop_front();    
		m_AbsoluteSupportFootPositions.pop_front();
	      }
	    ODEBUG("End the generation of steps");
	  }
	
	ODEBUG("Finished at index: j:" << j << " i:" << i << " k:" << k );
      }
    //     for(unsigned int li=0;li<(*aCTIPX.ZMPProfil).size();li++)
    //       {
    // 	ODEBUG3("After - ZMPProfil along X-Y axis: " << li 
    // 		<< " " << (*aCTIPX.ZMPProfil)[li] 
    // 		<< " " << (*aCTIPY.ZMPProfil)[li]
    // 		<< " " << m_DeltaTj[li] << " " << m_StepTypes[li] 
    // 		<< " ( " << SINGLE_SUPPORT << " , " << DOUBLE_SUPPORT << " ) " );
    //       }
    /* Compute the current value of the initial 
       and final CoM to be feed to the new system. */
    aCTIPX.InitialCoM = FPX.CoMInit;
    aCTIPX.InitialCoMSpeed = FPX.CoMSpeedInit;
    aCTIPX.FinalCoMPos = (*aCTIPX.ZMPProfil)[m_NumberOfIntervals-1];
    ODEBUG("InitialCoMPos " << aCTIPX.InitialCoM << 
	    " InitialCoMSpeed " << aCTIPX.InitialCoMSpeed << 
	    " FinalCoMPos " << aCTIPX.FinalCoMPos
	    );
    aCTIPY.InitialCoM = FPY.CoMInit;
    aCTIPY.InitialCoMSpeed = FPY.CoMSpeedInit;
    aCTIPY.FinalCoMPos = (*aCTIPY.ZMPProfil)[m_NumberOfIntervals-1];
    ODEBUG("InitialCoM " << aCTIPY.InitialCoM << 
	    " InitialCoMSpeed " << aCTIPY.InitialCoMSpeed << 
	    " FinalCoMPos " << aCTIPY.FinalCoMPos
	    );
	
    
  }

  double AnalyticalMorisawaCompact::TimeCompensationForZMPFluctuation(FluctuationParameters &aFP,
								      double DeltaTInit)
  {
    double r,r2;
    double DeltaTNew;

    ODEBUG( aFP.CoMInit  << " " << aFP.ZMPInit << " " << aFP.CoMSpeedInit  << " " << aFP.ZMPSpeedInit ); 
    ODEBUG( aFP.CoMNew  << " " << aFP.ZMPNew << " " << aFP.CoMSpeedNew  << " " << aFP.ZMPSpeedNew ); 
    ODEBUG("m_Omagej[0]:" << m_Omegaj[0] );
    r = (m_Omegaj[0] * ( aFP.CoMNew - aFP.ZMPNew) + (aFP.CoMSpeedNew - aFP.ZMPSpeedNew))/
      ( m_Omegaj[0]*(aFP.CoMInit - aFP.ZMPInit) + (aFP.CoMSpeedInit - aFP.ZMPSpeedInit) )
      ;
    r2 = ( m_Omegaj[0]*(aFP.CoMInit - aFP.ZMPInit) + (aFP.CoMSpeedInit - aFP.ZMPSpeedInit) )/
      (m_Omegaj[0] * ( aFP.CoMNew - aFP.ZMPNew) + (aFP.CoMSpeedNew - aFP.ZMPSpeedNew));
      
    ODEBUG("Fluctuation: " << r );
    if (r<=0.0)
      DeltaTNew = DeltaTInit + m_Tsingle*0.5;
    else
      DeltaTNew = DeltaTInit + log(r)/m_Omegaj[0];

    ODEBUG("DeltaTInit: " << DeltaTInit << " DeltaTNew : " 
	   << DeltaTNew << " DeltaDeltaT: " 
	   << DeltaTNew - DeltaTInit ); 
#if 0
    if (DeltaTNew<0.2*m_Tsingle)
      DeltaTNew = 0.2*m_Tsingle;
    if (DeltaTNew>DeltaTInit + m_Tsingle*0.5)
      DeltaTNew = DeltaTInit + m_Tsingle*0.5;
#endif
    return DeltaTNew;

  }

  void AnalyticalMorisawaCompact::ChangeZMPProfil(unsigned int IndexStep,
						  FootAbsolutePosition &NewFootAbsPos,
						  CompactTrajectoryInstanceParameters &aCTIPX,
						  CompactTrajectoryInstanceParameters &aCTIPY)
  {
    /* Change in the constraints, i.e. modify aCTIPX and aCTIPY appropriatly . */
    
    if (IndexStep<aCTIPX.ZMPProfil->size())
      {
	(*aCTIPX.ZMPProfil)[IndexStep] = NewFootAbsPos.x;
	(*aCTIPY.ZMPProfil)[IndexStep] = NewFootAbsPos.y;
      }
    if (IndexStep<aCTIPX.ZMPProfil->size()-1)
      {
	(*aCTIPX.ZMPProfil)[IndexStep+1] = NewFootAbsPos.x;
	(*aCTIPY.ZMPProfil)[IndexStep+1] = NewFootAbsPos.y;
      }

    
    /* If the end condition has been changed... */
    if (IndexStep+1==m_NumberOfIntervals-1)
      {
	aCTIPX.FinalCoMPos = NewFootAbsPos.x;
	aCTIPY.FinalCoMPos = NewFootAbsPos.y;
      }
  }

  int AnalyticalMorisawaCompact::ChangeFootLandingPosition(double t,
							   unsigned int IndexStep,
							   FootAbsolutePosition & NewFootAbsPos)
  {
    ChangeFootLandingPosition(t,IndexStep,
			      NewFootAbsPos,
			      *m_AnalyticalZMPCoGTrajectoryX,
			      m_CTIPX,
			      *m_AnalyticalZMPCoGTrajectoryY,
			      m_CTIPY,true);
  }
							   
  int AnalyticalMorisawaCompact::ChangeFootLandingPosition(double t,
							   unsigned int IndexStep,
							   FootAbsolutePosition & NewFootAbsPos,
							   AnalyticalZMPCOGTrajectory &aAZCTX,
							   CompactTrajectoryInstanceParameters &aCTIPX,
							   AnalyticalZMPCOGTrajectory &aAZCTY,
							   CompactTrajectoryInstanceParameters &aCTIPY,
							   bool TemporalShift,
							   StepStackHandler * aStepStackHandler)
  {
    double LocalTime = t - m_AbsoluteTimeReference;
    double FinalTime;
    unsigned int IndexStartingInterval;
    int RetourTC;

    double Tmax,NewTj;
    FluctuationParameters aFPX,aFPY;
    double TCX, TCY, TCMax;

    /* Perform First Time Change i.e. recomputing the proper Tj */
    if ((RetourTC=TimeChange(LocalTime,
			     IndexStep,
			     IndexStartingInterval,
			     FinalTime,
			     NewTj))<0)
      return RetourTC;

    ODEBUG("LocalTime: " << LocalTime + m_AbsoluteTimeReference 
	    << " m_AbsoluteTimeReference : " << m_AbsoluteTimeReference);

    /* Store the current position and speed of each foot. */
    FootAbsolutePosition InitAbsLeftFootPos,InitAbsRightFootPos;
    
    m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(1,t,InitAbsLeftFootPos);
    m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(-1,t,InitAbsRightFootPos);

    ODEBUG("Index Starting Interval :" << IndexStartingInterval << " " << IndexStep);

    /* ! This part of the code is not used if we are just trying to add
       a foot step. */
    if (IndexStep<m_NumberOfIntervals)
      {
	/* Compute the time of maximal fluctuation for the initial solution along the X axis.*/
	Tmax = aAZCTX.FluctuationMaximal();
	ODEBUG("Tmax X Init :" << Tmax );
	aAZCTX.ComputeCOM(t,aFPX.CoMInit,IndexStartingInterval);
	aAZCTX.ComputeCOMSpeed(t,aFPX.CoMSpeedInit);
	aAZCTX.ComputeZMP(t,aFPX.ZMPInit,IndexStartingInterval);
	aAZCTX.ComputeZMPSpeed(t,aFPX.ZMPSpeedInit);
	
	/* Compute the time of maximal fluctuation for the initial solution along the Y axis.*/
	Tmax = aAZCTY.FluctuationMaximal();
	ODEBUG("Tmax Y Init :" << Tmax );
	aAZCTY.ComputeCOM(t,aFPY.CoMInit,IndexStartingInterval);
	aAZCTY.ComputeCOMSpeed(t,aFPY.CoMSpeedInit);
	ODEBUG("COM Y: " << aFPY.CoMInit);
	aAZCTY.ComputeZMP(t,aFPY.ZMPInit,IndexStartingInterval);
	ODEBUG("ZMP Y: " << aFPY.ZMPInit);
	aAZCTY.ComputeZMPSpeed(t,aFPY.ZMPSpeedInit);
	
	
	/* Adapt the ZMP profil of aCTPIX and aCTPIY 
	   according to IndexStep */
	ChangeZMPProfil(IndexStep,NewFootAbsPos,
		       aCTIPX,aCTIPY);
		
	/* Recompute the coefficient of the ZMP/COG trajectories objects. */
	for(unsigned int i=1;i<m_NumberOfIntervals-1;i++)
	  {
	    aAZCTX.Building3rdOrderPolynomial(i,(*aCTIPX.ZMPProfil)[i-1],(*aCTIPX.ZMPProfil)[i]);
	    aAZCTY.Building3rdOrderPolynomial(i,(*aCTIPY.ZMPProfil)[i-1],(*aCTIPY.ZMPProfil)[i]);
	    ODEBUG( (*aCTIPX.ZMPProfil)[i-1] << " " << (*aCTIPX.ZMPProfil)[i] << " " << 
		    (*aCTIPY.ZMPProfil)[i-1] << " " << (*aCTIPY.ZMPProfil)[i]);
	  }
	
	/* Compute the trajectories */
	ComputeTrajectory(aCTIPY,aAZCTY);
	ComputeTrajectory(aCTIPX,aAZCTX);
	
	aAZCTX.ComputeCOM(t,aFPX.CoMNew,IndexStartingInterval);
	ODEBUG( "FPX.COMInit :" << aFPX.CoMInit << " FPX.CoMNew: " << aFPX.CoMNew);
	aAZCTX.ComputeCOMSpeed(t,aFPX.CoMSpeedNew);
	aAZCTX.ComputeZMP(t,aFPX.ZMPNew,IndexStartingInterval);
	aAZCTX.ComputeZMPSpeed(Tmax,aFPX.ZMPSpeedNew);
	
	aAZCTY.ComputeCOM(t,aFPY.CoMNew,IndexStartingInterval);
	ODEBUG("FPY.COMInit :" << aFPY.CoMInit << " FPY.CoMNew: " << aFPY.CoMNew );
	aAZCTY.ComputeCOMSpeed(t,aFPY.CoMSpeedNew);
	aAZCTY.ComputeZMP(t,aFPY.ZMPNew,IndexStartingInterval);
	aAZCTY.ComputeZMPSpeed(t,aFPY.ZMPSpeedNew);
	
	TCX = TimeCompensationForZMPFluctuation(aFPX,NewTj);
	TCY = TimeCompensationForZMPFluctuation(aFPY,NewTj); 
	
	TCMax = TCX < TCY ? TCY : TCX;
	ODEBUG("TCX : "<< TCX << " TCY: " << TCY << " TCMax : " << TCMax << " DeltaTj[0] : " << m_DeltaTj[0]);
      }
    else
      {
	// For a proper initialization of the analytical trajectories
	// through constraint change the Fluctuation structure has to be changed
	// approriatly.
	aAZCTX.ComputeCOM(t,aFPX.CoMInit);
	aAZCTX.ComputeCOMSpeed(t,aFPX.CoMSpeedInit);

	aAZCTY.ComputeCOM(t,aFPY.CoMInit);
	aAZCTY.ComputeCOMSpeed(t,aFPY.CoMSpeedInit);
	
	
	TCX = m_Tsingle-m_SamplingPeriod;
      }


    /************ PERFORM THE TIME INTERVAL MODIFICATION ****************/

    if (TemporalShift)
      {
	NewTimeIntervals(IndexStartingInterval,TCX); //TCMax
      }
    else 
      {
	NewTimeIntervals(IndexStartingInterval,NewTj);
      }
	

    /*! Extract the set of absolute coordinates for the foot position,
      and recompute the feet trajectory accordingly.
     */
    if (m_FeetTrajectoryGenerator!=0)
      {
	
	ODEBUG("ChangeFootLandingPosition: " << m_CurrentTime);
	m_FeetTrajectoryGenerator->SetDeltaTj(m_DeltaTj);
	/* Modify the feet trajectory */	
	ODEBUG("***************** From Change Foot Landing Position ********************");
	/*
	m_FeetTrajectoryGenerator->InitializeFromRelativeSteps(m_RelativeFootPositions,
							       InitAbsLeftFootPos,
							       InitAbsRightFootPos,
							       m_AbsoluteSupportFootPositions,
							       false,true);
	*/
	m_FeetTrajectoryGenerator->ComputeAbsoluteStepsFromRelativeSteps(m_RelativeFootPositions,
									 InitAbsLeftFootPos,
									 InitAbsRightFootPos,
									 m_AbsoluteSupportFootPositions);
		
	ODEBUG("***************** End of Change Foot Landing Position ********************");
      }

    /* Shift the ZMP profil, the initial 
       and final condition. */
    ConstraintsChange(LocalTime,
		      aFPX,aFPY,
		      aCTIPX,aCTIPY,
		      IndexStartingInterval,
		      aStepStackHandler);
    
    m_FeetTrajectoryGenerator->InitializeFromRelativeSteps(m_RelativeFootPositions,
							   InitAbsLeftFootPos,
							   InitAbsRightFootPos,
							   m_AbsoluteSupportFootPositions,
							   false,true);
    
    // Initialize and modify the aAZCT trajectories' Tj and Omegaj.
    aAZCTX.SetStartingTimeIntervalsAndHeightVariation(m_DeltaTj,m_Omegaj);
    aAZCTY.SetStartingTimeIntervalsAndHeightVariation(m_DeltaTj,m_Omegaj);
    
    /* Recompute the coefficient of the ZMP/COG trajectories objects. */
    for(unsigned int i=1;i<m_NumberOfIntervals-1;i++)
      {
	aAZCTX.Building3rdOrderPolynomial(i,(*aCTIPX.ZMPProfil)[i-1],(*aCTIPX.ZMPProfil)[i]);
	aAZCTY.Building3rdOrderPolynomial(i,(*aCTIPY.ZMPProfil)[i-1],(*aCTIPY.ZMPProfil)[i]);
      }

    ODEBUG("ChangeFootLandingPosition: SetAbsoluteTimeReference to t:" << t);
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
    return 0;
  }

  void AnalyticalMorisawaCompact::StringErrorMessage(int ErrorIndex, string &ErrorMessage)
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
  
  int AnalyticalMorisawaCompact::SetPreviewControl(PreviewControl * aPreviewControl)
  {
    m_PreviewControl = aPreviewControl;
    return 0;
  }

  PreviewControl * AnalyticalMorisawaCompact::GetPreviewControl()
  {
    return m_PreviewControl;
  }

  void AnalyticalMorisawaCompact::FilterOutOrthogonalDirection(AnalyticalZMPCOGTrajectory & aAZCT,
							       CompactTrajectoryInstanceParameters &aCTIP,
							       deque<double> & ZMPTrajectory,
							       deque<double> & CoGTrajectory)
  {
    /* Initiliazing the Preview Control according to the trajectory 
       to filter. */
    double lAbsoluteTimeReference = aAZCT.GetAbsoluteTimeReference();
    MAL_MATRIX_DIM(x,double,3,1);

    /*! Initialize the state vector used by the preview controller */
    x(0,0) = 0.0;//aAZCT.ComputeCOM(lAbsoluteTimeReference,x(0,0));
    x(1,0) = 0.0;//aAZCT.ComputeCOMSpeed(lAbsoluteTimeReference,x(1,0));
    x(2,0) = 0.0;
    
    /*! Initializing variables needed to compute the state vector */
    double lsxzmp = 0.0;    
    double lxzmp = 0.0;
    /*
    aAZCT.ComputeZMP(lAbsoluteTimeReference,lxzmp);
    lxzmp = (*aCTIP.ZMPProfil)[0] - lxzmp;
    */
    /*! Preview window of the ZMP ref positions */
    double PreviewWindowTime = m_PreviewControl->PreviewControlTime();
    deque<double> FIFOZMPRefPositions;
    RESETDEBUG4("ProfilZMPError.dat");
    for(double lx=0;lx<m_DeltaTj[0]+2*PreviewWindowTime;lx+=m_SamplingPeriod)
      {
	double r=0.0,lZMP;
	if (lx<m_DeltaTj[0])
	  {
	    double lZMP;
	    aAZCT.ComputeZMP(lAbsoluteTimeReference+lx,lZMP);
	    r = (*aCTIP.ZMPProfil)[0] - lZMP;
	  }
	
	FIFOZMPRefPositions.push_back(r);
	ODEBUG4(r,"ProfilZMPError.dat");
      }
    

    unsigned int lindex=0;
    lsxzmp = 0.0;
    for(double lx=0;lx<m_DeltaTj[0]+PreviewWindowTime;lx+= m_SamplingPeriod)
      {
	m_PreviewControl->OneIterationOfPreview1D(x,lsxzmp,FIFOZMPRefPositions,lindex,
				      lxzmp,false);
	ZMPTrajectory.push_back(lxzmp);
	CoGTrajectory.push_back(x(0,0));
	lindex++;
	lsxzmp = 0.0;
	
      }
  }

 
  void AnalyticalMorisawaCompact::SetFeetTrajectoryGenerator(LeftAndRightFootTrajectoryGenerationMultiple * 
							     aFeetTrajectoryGenerator)
  {
    m_FeetTrajectoryGenerator = aFeetTrajectoryGenerator;
  }

  LeftAndRightFootTrajectoryGenerationMultiple * AnalyticalMorisawaCompact::GetFeetTrajectoryGenerator()
  {
    return m_FeetTrajectoryGenerator;
  }

  int AnalyticalMorisawaCompact::OnLineFootChange(double time,
						  FootAbsolutePosition &aFootAbsolutePosition,
						  deque<ZMPPosition> & ZMPPositions,			     
						  deque<COMPosition> & CoMPositions,
						  deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
						  deque<FootAbsolutePosition> &RightFootAbsolutePositions,
						  StepStackHandler *aStepStackHandler)
  {
    ODEBUG("****************** Begin OnLineFootChange **************************");
    int IndexInterval=-1;
    
    /* Trying to find the index interval where the change should operate. */
    double TimeReference = m_AbsoluteTimeReference;
    if (time> m_AbsoluteTimeReference)
      {
	for(unsigned int i=0;i<m_DeltaTj.size();i++)
	  {
	    if (time < TimeReference + m_DeltaTj[i])
	      {
		IndexInterval = (int)i;
		break;
	      }
	    TimeReference += m_DeltaTj[i];
	  }
      }

    /* If the interval detected is not a double support interval,
       a shift is done to chose the earliest double support interval. */
    if (m_StepTypes[IndexInterval]!=DOUBLE_SUPPORT)
      {
	if (IndexInterval!=0)
	  IndexInterval-=1;
	else
	  IndexInterval=1;
      }

    if (IndexInterval==-1)
      return -1;

    /* Change the foot */
    unsigned int lChangedIntervalFoot = (IndexInterval-1)/2;
    if (LeftFootAbsolutePositions[0].z==0.0)
      m_AbsoluteCurrentSupportFootPosition = LeftFootAbsolutePositions[0];
    else 
      m_AbsoluteCurrentSupportFootPosition = RightFootAbsolutePositions[0];

    m_AbsoluteSupportFootPositions[lChangedIntervalFoot] = aFootAbsolutePosition;

    ODEBUG("Current LeftFootAbsolutePosition: " << LeftFootAbsolutePositions[0].x  << " " 
	    << " " << LeftFootAbsolutePositions[0].y 
	    << " " << LeftFootAbsolutePositions[0].dx 
	    << " " << LeftFootAbsolutePositions[0].dy);
    ODEBUG("Current RightFootAbsolutePosition: " << RightFootAbsolutePositions[0].x  << " " 
	    << " " << RightFootAbsolutePositions[0].y
	    << " " << RightFootAbsolutePositions[0].dx
	    << " " << RightFootAbsolutePositions[0].dy);
    
    ODEBUG("*** Begin Change foot position *** " << m_RelativeFootPositions.size());

    /* From the new foot landing position computes the new relative set of positions. */
    m_FeetTrajectoryGenerator->ChangeRelStepsFromAbsSteps(m_RelativeFootPositions,
							  m_AbsoluteCurrentSupportFootPosition,
							  m_AbsoluteSupportFootPositions,
							  lChangedIntervalFoot);
    ODEBUG("After changingRelative foot steps");
	
    ODEBUG("*** End Change foot position *** ");
    /* Change the foot landing position. */
    ChangeFootLandingPosition(m_CurrentTime,
			      IndexInterval,
			      aFootAbsolutePosition,
			      *m_AnalyticalZMPCoGTrajectoryX,
			      m_CTIPX,
			      *m_AnalyticalZMPCoGTrajectoryY,
			      m_CTIPY,true,
			      aStepStackHandler);
    
    double LocalTimeFrame = m_CurrentTime - m_AbsoluteTimeReference;
    //unsigned int ControlIndex = (unsigned int)((time - m_CurrentTime)/0.005);


    // ***  Very important: 
    // we assume that on the on-line mode we have two values ahead inside
    // the stack. As the change will operate from the current time
    // the stacks are cleared.
    // ***
    ZMPPositions.clear();
    CoMPositions.clear();
    LeftFootAbsolutePositions.clear();
    RightFootAbsolutePositions.clear();
    m_UpperTimeLimitToUpdateStacks = m_AbsoluteTimeReference + m_DeltaTj[0] + m_Tdble;    
    ODEBUG("m_UpperTimeLimitToUpdateStacks "<< m_UpperTimeLimitToUpdateStacks << 
	    " m_AbsoluteTimeReference: " <<m_AbsoluteTimeReference << 
	    " m_DeltaTj[0] " << m_DeltaTj[0] << " m_Tdble: " << m_Tdble);
 
    for(double t=m_AbsoluteTimeReference; t<m_AbsoluteTimeReference+2*m_SamplingPeriod; t+= 0.005)
      {
	/*! Feed the ZMPPositions. */
	ZMPPosition aZMPPos;
        m_AnalyticalZMPCoGTrajectoryX->ComputeZMP(t,aZMPPos.px);
	m_AnalyticalZMPCoGTrajectoryY->ComputeZMP(t,aZMPPos.py);
	ZMPPositions.push_back(aZMPPos);

	/*! Feed the COMPositions. */
	COMPosition aCOMPos;
	memset(&aCOMPos,0,sizeof(aCOMPos));
	m_AnalyticalZMPCoGTrajectoryX->ComputeCOM(t,aCOMPos.x[0]);
	m_AnalyticalZMPCoGTrajectoryY->ComputeCOM(t,aCOMPos.y[0]);
	CoMPositions.push_back(aCOMPos);

	/*! Feed the FootPositions. */

	/*! Left */
	FootAbsolutePosition LeftFootAbsPos;
	m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(1,t,LeftFootAbsPos);
	LeftFootAbsolutePositions.push_back(LeftFootAbsPos);

	/*! Right */
	FootAbsolutePosition RightFootAbsPos;
	m_FeetTrajectoryGenerator->ComputeAnAbsoluteFootPosition(-1,t,RightFootAbsPos);
	RightFootAbsolutePositions.push_back(RightFootAbsPos);

	ODEBUG4(aZMPPos.px << " " << aZMPPos.py << " " << aCOMPos.x[0] << " " << aCOMPos.y[0] << " " << 
		LeftFootAbsPos.x << " " << LeftFootAbsPos.y << " " << LeftFootAbsPos.z << " " << 
		RightFootAbsPos.x << " " << RightFootAbsPos.y << " " << RightFootAbsPos.z << " " ,"Test.dat");

      }
    ODEBUG("****************** End OnLineFootChange **************************");
    return 0;
  }

  /*! \brief Return the time at which it is optimal to regenerate a step in online mode. 
    This time is given in the size of the Left Foot Positions queue under which such
    new step has to be generate.
   */
  int AnalyticalMorisawaCompact::ReturnOptimalTimeToRegenerateAStep()
  {
    //    int r = 1+(int)((4 * m_Tsingle + m_Tdble) / m_SamplingPeriod);
    int r=1;
    return r;
  }
        
}


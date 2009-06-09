/* \file AnalyticalMorisawaFull.h
   \brief 
   Copyright (c) 2007, 
   Olivier Stasse,
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   see license.txt for more information on license.
*/

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "AnalyticalMorisawaFull :" << x << endl
#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#if 0
#define ODEBUG(x) cerr << "AnalyticalMorisawaFull :" <<  x << endl
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
#include <walkGenJrl/ZMPRefTrajectoryGeneration/AnalyticalMorisawaFull.h>

typedef double doublereal;
typedef int integer;

extern "C"
{
  extern int dgesvx_( char *, char *, /* FACT TRANS */
		     integer * , integer *, /* N NHRS */
		     doublereal *, integer *, /* A LDA */
		     doublereal *, integer *, /* AF LDAF */
		     integer *, /* IPIV */
		     char *, /* EQUED */
		     doublereal *, /* R */
		     doublereal *, /* C */
		     doublereal *, /* B */
		     integer *, /* LDB */
		     doublereal *, /* X */
		     integer *, /* LDX */
		     doublereal *, /* RCOND */
		     doublereal *, /* FERR */
		     doublereal *, /* BERR */
		     doublereal *, /* WORK */
		     integer *, /* IWORK */
		     integer * /* INFO */
                 );
}
namespace PatternGeneratorJRL
{


  AnalyticalMorisawaFull::AnalyticalMorisawaFull(SimplePluginManager *lSPM)
    : AnalyticalMorisawaAbstract(lSPM)
  {
  }


  AnalyticalMorisawaFull::~AnalyticalMorisawaFull()
  {

  }


  bool AnalyticalMorisawaFull::InitializeBasicVariables()
  {
    m_NumberOfIntervals = 2 * m_NumberOfStepsInAdvance+1;

    // Compute the temporal intervals.
    m_DeltaTj.resize(m_NumberOfIntervals);
    m_Omegaj.resize(m_NumberOfIntervals);

    for(int i=0;i<m_NumberOfIntervals;i++)
      {
	if (i%2==0)
	  m_DeltaTj[i] = m_Tsingle;
	else
	  m_DeltaTj[i] = m_Tdble;
      }
    m_DeltaTj[m_NumberOfIntervals-1]=m_Tsingle*3.0;

    if (m_VerboseLevel>=2)
      {
	for(int i=0;i<m_NumberOfIntervals;i++)
	  cout << m_DeltaTj[i] << " ";
	cout << endl;
      }

    // Specify the degrees corresponding to the given interval.
    m_PolynomialDegrees.resize(m_NumberOfIntervals);
    m_PolynomialDegrees[0] = 4;
    m_PolynomialDegrees[m_NumberOfIntervals-1] = 4;
    for(int i=1;i<m_NumberOfIntervals-1;i++)
      m_PolynomialDegrees[i] = 3;    
    
    return true;
  }

  void AnalyticalMorisawaFull::ComputeZ1(unsigned int &rowindex)
  {
    double SquareOmega0 = m_Omegaj[0] * m_Omegaj[0];

    // First Row: Connection of the position of the CoM 
    double c0,s0;
    double Deltat = m_DeltaTj[0];
    m_Z(rowindex,0) = 1.0;
    m_Z(rowindex,1) = Deltat;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,2) = Deltat;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,3) = Deltat;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,4) = Deltat;
    m_Z(rowindex,5) = c0 = cosh(m_Omegaj[0] * m_DeltaTj[0]);
    m_Z(rowindex,6) = s0 = sinh(m_Omegaj[0] * m_DeltaTj[0]);
    m_Z(rowindex,7) = -1.0;
    m_Z(rowindex,11) = -1.0;
    rowindex++;

    // Second Row : Connection of the velocity of the CoM
    Deltat = m_DeltaTj[0];
    m_Z(rowindex,1) = 1.0;
    m_Z(rowindex,2) = 2 * Deltat;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,3) = 3 * Deltat;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,4) = 4 * Deltat;
    m_Z(rowindex,5) = m_Omegaj[0] * s0;
    m_Z(rowindex,6) = m_Omegaj[0] * c0;
    m_Z(rowindex,8) = -1.0;
    m_Z(rowindex,12) = -m_Omegaj[0];
    rowindex++;
  

    // Third Row : Terminal condition for the ZMP position
    Deltat = m_DeltaTj[0];
    m_Z(rowindex,0) = 1.0;
    m_Z(rowindex,1) = Deltat;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,2) = Deltat - 2.0 / SquareOmega0;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,3) = Deltat - 6.0 * m_DeltaTj[0] / SquareOmega0;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,4) = Deltat - 12 * m_DeltaTj[0] * m_DeltaTj[0]/ SquareOmega0;
    rowindex++;
  
    // Fourth Row : Terminal velocity for the ZMP position
    Deltat = m_DeltaTj[0];
    m_Z(rowindex,1) = 1.0;
    m_Z(rowindex,2) = 2.0 * Deltat;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,3) = 3.0 * Deltat - 6.0 / SquareOmega0;
    Deltat*= m_DeltaTj[0];
    m_Z(rowindex,4) = 4 * Deltat - 24.0 * m_DeltaTj[0] / SquareOmega0;;
    rowindex++;
  
    // Fifth Row : Initial condition for the ZMP position
    m_Z(rowindex,0) = 1.0;
    m_Z(rowindex,2) = -2.0 / SquareOmega0;
    rowindex++;

    // Sixth Row : Initial velocity for the ZMP position
    m_Z(rowindex,1) = 1.0;
    m_Z(rowindex,3) = -6.0 / SquareOmega0;
    rowindex++;
  }

  void AnalyticalMorisawaFull::ComputeZj(unsigned int intervalindex, 
				     unsigned int &colindex,
				     unsigned int &rowindex)
  {
    // First row : Connection of the position of the CoM 
    double Omegaj = m_Omegaj[intervalindex];
    double SquareOmegaj=Omegaj*Omegaj;
    
    double Deltat = m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex) = 1;

    m_Z(rowindex,colindex+1) = Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+3) = Deltat;
  
    double c0=0.0,s0=0.0;
    m_Z(rowindex,colindex+4) = c0 = cosh(Omegaj * m_DeltaTj[intervalindex]);
    m_Z(rowindex,colindex+5) = s0 = sinh(Omegaj * m_DeltaTj[intervalindex]);
    m_Z(rowindex,colindex+6) = -1.0;
    if ((int)intervalindex!=m_NumberOfIntervals-2)
      m_Z(rowindex,colindex+10) = -1.0;
    else
      m_Z(rowindex,colindex+11) = -1.0;
    rowindex++;
  
    // Second row : Connection of the velocity of the CoM
    m_Z(rowindex,colindex+1) = 1;
    Deltat = m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = 2*Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+3) = 3*Deltat;
  
    m_Z(rowindex,colindex+4) = Omegaj * s0;
    m_Z(rowindex,colindex+5) = Omegaj * c0;
    m_Z(rowindex,colindex+7) = -1.0;
    if ((int)intervalindex!=m_NumberOfIntervals-2)
      m_Z(rowindex,colindex+11) = -Omegaj;
    else
      m_Z(rowindex,colindex+12) = -m_Omegaj[m_NumberOfIntervals-1];


    rowindex++;

    // Third row: Terminal condition for the ZMP position
    m_Z(rowindex,colindex) = 1;
    Deltat = m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+1) = Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = Deltat - 2.0 / SquareOmegaj;;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+3) = Deltat  - 6.0 * m_DeltaTj[intervalindex]/SquareOmegaj;
    rowindex++;
  
  
    // Fourth row: Terminal velocity for the ZMP position
    m_Z(rowindex,colindex+1) = 1;
    Deltat = m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = 2 * Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+3) = 3 * Deltat - 6.0 / SquareOmegaj;
    rowindex++;
  
  
    // Fifth row: Initial condition for the ZMP position
    m_Z(rowindex,colindex) = 1;
    m_Z(rowindex,colindex+2) = -2.0 / SquareOmegaj;
    rowindex++;

    // Sixth row: Initial velocity for the ZMP position
    m_Z(rowindex,colindex+1) = 1;
    m_Z(rowindex,colindex+3) =  - 6.0 / SquareOmegaj;
    rowindex++;
  
  
  }	

  void AnalyticalMorisawaFull::ComputeZm(unsigned int intervalindex, 
				     unsigned int &colindex, 
				     unsigned int &rowindex)
  {
    double Omegam=m_Omegaj[intervalindex];
    double SquareOmegam = Omegam*Omegam;

    // First row : Connection of the position of the CoM 
    double Deltat = m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex) = 1;

    m_Z(rowindex,colindex+1) = Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+3) = Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+4) = Deltat;
  
    double c0=0.0,s0=0.0;
    m_Z(rowindex,colindex+5) = c0 = cosh(Omegam * m_DeltaTj[intervalindex]);
    m_Z(rowindex,colindex+6) = s0 = sinh(Omegam * m_DeltaTj[intervalindex]);  
    rowindex++;
  
    // Second row : Connection of the velocity of the CoM
    m_Z(rowindex,colindex+1) = 1;
    Deltat = m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = 2*Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+3) = 3*Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+4) = 4*Deltat;

    m_Z(rowindex,colindex+5) = Omegam * s0;
    m_Z(rowindex,colindex+6) = Omegam * c0;
    rowindex++;

    // Third row: Terminal condition for the ZMP position
    m_Z(rowindex,colindex) = 1;
    Deltat = m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+1) = Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = Deltat - 2.0 / SquareOmegam;;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+3) = Deltat - 6.0 * m_DeltaTj[intervalindex]/SquareOmegam;

    Deltat*= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+4) = Deltat - 12 * m_DeltaTj[intervalindex] * m_DeltaTj[intervalindex]/ SquareOmegam;

    rowindex++;
  
  
    // Fourth row: Terminal velocity for the ZMP position
    m_Z(rowindex,colindex+1) = 1;
    Deltat = m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+2) = 2 * Deltat;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+3) = 3 * Deltat - 6.0 / SquareOmegam;

    Deltat *= m_DeltaTj[intervalindex];
    m_Z(rowindex,colindex+4) = 4 * Deltat - 24.0 * m_DeltaTj[intervalindex] / SquareOmegam;

    rowindex++;
  
  
    // Fifth row: Initial condition for the ZMP position
    m_Z(rowindex,colindex) = 1;
    m_Z(rowindex,colindex+2) = - 2.0 / SquareOmegam;
    rowindex++;

    // Sixth row: Initial condition for the ZMP velocity.
    m_Z(rowindex,colindex+1) = 1;
    m_Z(rowindex,colindex+3) =  - 6.0 / SquareOmegam;
    rowindex++;

  }

  void AnalyticalMorisawaFull::BuildingTheZMatrix(vector<double> &lCoM, vector<double> &lZMP )
  {

    if ((lCoM.size()!=(unsigned int)m_NumberOfIntervals) || (lZMP.size()!=(unsigned int)m_NumberOfIntervals))
      return;

    for(unsigned int i=0;i<lCoM.size();i++)
      {
	m_Omegaj[i]=sqrt(9.86/ (lCoM[i] - lZMP[i]));
      }
    unsigned NbRows, NbCols;
    unsigned int rowindex=0;
    unsigned int colindex=0;

    NbRows = 6*m_NumberOfIntervals+2;
    NbCols = 14+(m_NumberOfIntervals-2)*6;
    ODEBUG( "Rows: " << NbRows << " Cols: " << NbCols);
    MAL_MATRIX_RESIZE(m_Z,NbRows,NbCols);

    // Initial condition for the COG position and the velocity 
    MAL_MATRIX_FILL(m_Z,0.0);
    m_Z(0,0) = m_Z(0,5) = 1.0; 
    rowindex++;
    m_Z(1,1) = m_Z(1,6) = 1.0;
    rowindex++;


    // Compute Z1
    ComputeZ1(rowindex);
    colindex+= m_PolynomialDegrees[0]+3;

    // Computing Zj.
    for(int i=1;i<m_NumberOfIntervals-1;++i)
      {
	ComputeZj(i,colindex,rowindex);
	colindex+= m_PolynomialDegrees[i]+3;
      }

    // Computing Zm
    ComputeZm(m_NumberOfIntervals-1,
	      colindex,rowindex);
  
    if (m_VerboseLevel>=2)
      {
	std::ofstream ofs;
	ofs.open("ZMatrix.dat",ofstream::out);
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

  void AnalyticalMorisawaFull::ComputeW(double InitialCoMPos,
					double InitialCoMSpeed,
					vector<double> &ZMPPosSequence,
					double FinalCoMPos,
					AnalyticalZMPCOGTrajectory & aAZCT)
  {
    MAL_VECTOR_RESIZE(m_w,6 * m_NumberOfIntervals + 2);
    // Initical COM position.
    m_w(0) = InitialCoMPos;
    m_w(1) = InitialCoMSpeed;

    // For each Position the same sequence
    for(int i=0;i<m_NumberOfIntervals;i++)
      {
	unsigned int lindex=6*i+2;
	if (i!=m_NumberOfIntervals-1)
	  {
	    // Continuity regarding the COM
	    // position and its velocity
	    m_w(lindex  )= 0.0; 
	    m_w(lindex+1)= 0.0;
	  }
	else
	  {
	    // Save for the last interval.
	    m_w(lindex  )= ZMPPosSequence[i];
	    m_w(lindex+1)= 0.0;	  
	  }
	// Final condition
	m_w(lindex+2)= ZMPPosSequence[i];
	m_w(lindex+3)= 0.0;
      
	// Initial condition
	if (i==0)
	  // If this is the first interval we 
	  // stay where we are.
	  m_w(lindex+4)= ZMPPosSequence[i];
	else 
	  // Otherwise we start from the 
	  // previous value.
	  m_w(lindex+4)= ZMPPosSequence[i-1];

	m_w(lindex+5)= 0.0;
      }
  
    if (m_VerboseLevel>=2)
      {
	std::ofstream ofs;
	ofs.open("WMatrix.dat",ofstream::out);
	ofs.precision(10);
      
	for(unsigned int i=0;i<MAL_VECTOR_SIZE(m_w);i++)
	  {
	    ofs << m_w[i]<< " ";
	  }
	ofs << endl;
	ofs.close();
      }
  }


  void AnalyticalMorisawaFull::ComputePolynomialWeights()
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

  void AnalyticalMorisawaFull::ComputePolynomialWeights2()
  {
    int SizeOfZ = MAL_MATRIX_NB_ROWS(m_Z),
      LDA,LDAF,LDB;
    int NRHS = 1;
    
    char EQUED;

    MAL_MATRIX(,double) tZ;
    tZ = MAL_RET_TRANSPOSE(m_Z);
    
    MAL_VECTOR(,double) m_AF;

    MAL_VECTOR_RESIZE(m_AF,SizeOfZ*2*SizeOfZ);

    MAL_VECTOR(,integer) m_IPIV;
    MAL_VECTOR_RESIZE(m_IPIV,SizeOfZ);

    MAL_VECTOR(,double) m_R;
    MAL_VECTOR_RESIZE(m_R,SizeOfZ);
    MAL_VECTOR(,double) m_C;
    MAL_VECTOR_RESIZE(m_C,SizeOfZ);
    MAL_VECTOR(,double) m_B;
    MAL_VECTOR_RESIZE(m_B,SizeOfZ);

    MAL_VECTOR_RESIZE(m_y,SizeOfZ);
    //MAL_VECTOR(,double) m_X;
    //b    MAL_VECTOR_RESIZE(m_X,SizeOfZ);

    LDA =  SizeOfZ;
    LDAF = SizeOfZ;
    LDB = SizeOfZ;

    double lRCOND;

    MAL_VECTOR(,double) m_FERR, m_BERR;
    MAL_VECTOR_RESIZE(m_FERR,SizeOfZ);
    MAL_VECTOR_RESIZE(m_BERR,SizeOfZ);

    int lwork = 4* SizeOfZ;
    double *work = new double[lwork];
    int *iwork = new int[SizeOfZ];
    int lsizeofx;
    int info=0;
    char lE[2]="E";
    char lN[2]="N";
    dgesvx_(lE, /* Ask for equilibration, copied to AF and factorization */
	   lN, /* A * X = B */
	   &SizeOfZ, /* Size of A */
	   &NRHS, /*Nb of columns for X et B */
	   MAL_RET_MATRIX_DATABLOCK(tZ), /* Access to A */
	   &LDA, /* Leading size of A */
	   MAL_RET_VECTOR_DATABLOCK(m_AF),
	   &LDAF,
	   MAL_RET_VECTOR_DATABLOCK(m_IPIV),
	   &EQUED,
	   MAL_RET_VECTOR_DATABLOCK(m_R),
	   MAL_RET_VECTOR_DATABLOCK(m_C),
	   MAL_RET_VECTOR_DATABLOCK(m_w),
	   &LDB,
	   MAL_RET_VECTOR_DATABLOCK(m_y),
	   &lsizeofx,
	   &lRCOND,
	   MAL_RET_VECTOR_DATABLOCK(m_FERR),
	   MAL_RET_VECTOR_DATABLOCK(m_BERR),
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

  void AnalyticalMorisawaFull::GetZMPDiscretization(deque<ZMPPosition> & ZMPPositions,
						    deque<COMPosition> & COMPositions,
						    deque<RelativeFootPosition> &RelativeFootPositions,
						    deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
						    deque<FootAbsolutePosition> &RightFootAbsolutePositions,
						    double Xmax,
						    COMPosition & lStartingCOMPosition,
						    MAL_S3_VECTOR(&,double) lStartingZMPPosition,
						    FootAbsolutePosition & InitLeftFootAbsolutePosition,
						    FootAbsolutePosition & InitRightFootAbsolutePosition)
  {
    cout << "To be implemented" << endl;
  }

  int AnalyticalMorisawaFull::InitOnLine(deque<ZMPPosition> & FinalZMPPositions,
					 deque<COMPosition> & COMPositions,
					 deque<FootAbsolutePosition> & FinalLeftFootAbsolutePositions,
					 deque<FootAbsolutePosition> & FinalRightFootAbsolutePositions,
					 FootAbsolutePosition & InitLeftFootAbsolutePosition,
					 FootAbsolutePosition & InitRightFootAbsolutePosition,
					 deque<RelativeFootPosition> &RelativeFootPositions,
					 COMPosition & lStartingCOMPosition,
					 MAL_S3_VECTOR(&,double) lStartingZMPPosition)
  {
    cout << "To be implemented" << endl;
    return 0;
  }

  void AnalyticalMorisawaFull::OnLine(double time,
				      deque<ZMPPosition> & FinalZMPPositions,				     
				      deque<COMPosition> & FinalCOMPositions,
				      deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
				      deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions)
  {
    cout << "To be implemented" << endl;
  }
  
  void AnalyticalMorisawaFull::OnLineAddFoot(RelativeFootPosition & NewRelativeFootPosition,
					     deque<ZMPPosition> & FinalZMPPositions,				     
					     deque<COMPosition> & FinalCOMPositions,
					     deque<FootAbsolutePosition> &FinalLeftFootAbsolutePositions,
					     deque<FootAbsolutePosition> &FinalRightFootAbsolutePositions,
					     bool EndSequence)
  {
    cout << "To be implemented" << endl;
  }

  void AnalyticalMorisawaFull::TransfertTheCoefficientsToTrajectories(AnalyticalZMPCOGTrajectory &aAZCT,
								      vector<double> &lCoMZ, 
								      vector<double> &lZMPZ,
								      double &lZMPInit,
								      double &lZMPEnd,
								      bool InitializeaAZCT)
  {
    if (InitializeaAZCT)
      {
	// Set the number of intervals.
	aAZCT.SetNumberOfIntervals(m_NumberOfIntervals);
      
	// Transfert the degrees of each polynome.
	aAZCT.SetPolynomialDegrees(m_PolynomialDegrees);
      }

    // Set the starting point and the height variation.
    aAZCT.SetStartingTimeIntervalsAndHeightVariation(m_DeltaTj,m_Omegaj);

    unsigned int lindex = 0;
    vector<double> lV,lW;


    // Weights.
    lV.resize(m_NumberOfIntervals);
    lW.resize(m_NumberOfIntervals);

    for(int i=0;i<m_NumberOfIntervals;i++)
      {
	Polynome * aPolynome=0;
	aAZCT.GetFromListOfCOGPolynomials(i,aPolynome);
	vector<double> coeff;
	coeff.resize(m_PolynomialDegrees[i]+1);

	for(unsigned int k=0;k<=m_PolynomialDegrees[i];k++)
	  {
	    coeff[k] = m_y[lindex++];
	  }

	aPolynome->SetCoefficients(coeff);

	lV[i] = m_y[lindex++];
	lW[i] = m_y[lindex++];

      }
  
    // Set the hyperbolic weights.
    aAZCT.SetCoGHyperbolicCoefficients(lV,lW);

    // Compute the ZMP weights from the CoG's ones.
    aAZCT.TransfertCoefficientsFromCOGTrajectoryToZMPOne(lCoMZ,lZMPZ);

  }
  
  int AnalyticalMorisawaFull::OnLineFootChange(double time, 
					       FootAbsolutePosition &aFootAbsolutePosition,
					       deque<ZMPPosition> & ZMPPositions,			     
					       deque<COMPosition> & CoMPositions,
					       deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
					       deque<FootAbsolutePosition> &RightFootAbsolutePositions,
					       StepStackHandler *aStepStackHandler)
  {
    return -1;
  }

  /*! \brief Method to stop walking.
    @param[out] ZMPPositions: The queue of ZMP reference positions.
    @param[out] CoMPositions: The queue of COM reference positions.
    @param[out] FinalLeftFootAbsolutePositions: The queue of left foot absolute positions.
	@param[out] FinalRightFootAbsolutePositions: The queue of right foot absolute positions.
  */
  void AnalyticalMorisawaFull::EndPhaseOfTheWalking(deque<ZMPPosition> &ZMPPositions,
						    deque<COMPosition> &FinalCOMPositions,
						    deque<FootAbsolutePosition> &LeftFootAbsolutePositions,
						    deque<FootAbsolutePosition> &RightFootAbsolutePositions)
  {
  }
  

  /*! \brief Return the time at which it is optimal to regenerate a step in online mode. 
   */
  int AnalyticalMorisawaFull::ReturnOptimalTimeToRegenerateAStep()
  {
    return 0;
  }
      

}

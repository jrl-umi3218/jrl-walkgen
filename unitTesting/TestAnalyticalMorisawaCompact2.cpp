/******************************************************************/
/* Program to test X,Y ZMP and COM trajectories generation.       */
/* This program test more particulary a sudden change.            */
/*                                                                */
/* Date: 11/09/2007                                               */
/*                                                                */
/* (c) JRL, CNRS/AIST, Olivier STASSE                             */
/*                                                                */
/******************************************************************/
#include <fstream>
#ifdef UNIX
#include <sys/time.h>
#endif /*UNIX*/
#ifdef WIN32
inline int random() {return rand();}
#endif /*WIN32*/
#include <time.h>

#include <walkGenJrl/Mathematics/Polynome.h>
#include <walkGenJrl/SimplePluginManager.h>
#include <walkGenJrl/ZMPRefTrajectoryGeneration/AnalyticalMorisawaCompact.h>


#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "TestAnalyticalMorisawaCompact2 :" << x << endl
#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "TAMC2: " << x << endl; DebugFile.close();}
#if 0
#define ODEBUG(x) cerr << "TestAnalyticalMorisawaCompac2t :" <<  x << endl
#else
#define ODEBUG(x)
#endif

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); \
    DebugFile << "TAMC2: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1
#else
#define RESETDEBUG4(y)
#define ODEBUG4(x,y)
#endif

#define ODEBUG6(x,y)

void OutputInAFile(string FileName,  
		   PatternGeneratorJRL::AnalyticalZMPCOGTrajectory aAZCT,
		   double TMax,
		   double TStep,
		   double LocalTime)
{
  // Write the output into a file.
  ofstream aof;
  aof.open((char *)FileName.c_str(),ofstream::out);

  for(double t=LocalTime;t<TMax;t+=TStep )
    {
      double CoM,ZMP;
      aof << t << " " ;
      if (aAZCT.ComputeCOM(t,CoM))
	aof << CoM;
      else aof << "false";
      aof << " ";
      if (aAZCT.ComputeZMP(t,ZMP))
	aof << ZMP;
      else aof << "false";
      
      aof << endl;
    }
  aof.close();
}

void OutputInAFileWithCorrections(string FileName,  
				  PatternGeneratorJRL::AnalyticalZMPCOGTrajectory aAZCT,
				  double TMax,
				  double TStep,
				  double LocalTime,
				  deque<double> ZMPPositions,
				  deque<double> COGPositions)
{
  // Write the output into a file.
  ofstream aof;
  aof.open((char *)FileName.c_str(),ofstream::out);

  unsigned int li=0;
  for(double t=LocalTime;t<TMax;t+=TStep,li++)
    {
      double CoM,ZMP;
      aof << t << " " ;
      
      if (aAZCT.ComputeCOM(t,CoM))
	{
	  if (li< COGPositions.size())
	    CoM += COGPositions[li];

	  aof << CoM;
	}
      else aof << "false";
      aof << " ";
      if (aAZCT.ComputeZMP(t,ZMP))
	{
	  if (li< ZMPPositions.size())
	    ZMP += ZMPPositions[li];
	  aof << ZMP;
	}
      else aof << "false";
      
      aof << endl;
    }
  aof.close();
}

int main(void)
{
	
  if (argc!=2)
    {
      cerr << " This program takes 2 arguments: " << endl;
      cerr << "./TestAnalyticalMorisawaCompact2 PATH_TO_PC_PARAMS_FILE "<< endl;
	  exit(-1);
    }	
  
  PatternGeneratorJRL::SimplePluginManager aSPM;
  PatternGeneratorJRL::AnalyticalMorisawaCompact aAM(&aSPM);
  PatternGeneratorJRL::PreviewControl aPC;
  
  string PCFileName=argv[1];
  aPC.ReadPrecomputedFile(PCFileName);
  aAM.SetPreviewControl(&aPC);
  double TS=0.7,TD=0.1;
  double ControlStep = 0.01;
  aAM.SetSamplingPeriod(ControlStep);
  int NbSteps = 3;
  int lVerboseMode = 1;
  unsigned int NbOfIntervals=2*NbSteps+1;

  deque<double> ZMPPositions,COGPositions;

  /* Initialization of the analytical resolution  */
  aAM.SetTSingleSupport(TS);
  aAM.SetTDoubleSupport(TD);

  aAM.SetNumberOfStepsInAdvance(NbSteps);
  aAM.InitializeBasicVariables();
  
  /* Define the height trajectory. */
  vector<double> lCoMZ;
  vector<double> lZMPZ;
  lCoMZ.resize(NbOfIntervals);
  lZMPZ.resize(NbOfIntervals);
  for(unsigned int i=0;i<NbOfIntervals;i++)
    {
      //      lCoMZ[i] = 0.68;
      lCoMZ[i] = 0.814;
      lZMPZ[i] = 0.0;
    }

  aAM.BuildingTheZMatrix(lCoMZ,lZMPZ);
  vector<double> OmegajCoefficients;
  aAM.GetOmegaj(OmegajCoefficients);

  vector<double> lDeltaTj;
  aAM.GetDeltaTj(lDeltaTj);

  /* Initialization for the sagital and orthogonal trajectories */
  double InitialCoMX=0.0;
  double InitialCoMSpeedX=0.0;
  double FinalCoMPosX=0.6;
  vector<double> lZMPX;
  lZMPX.resize(NbOfIntervals);

  double InitialCoMY=0.0;
  double InitialCoMSpeedY=0.0;
  double FinalCoMPosY=0.0;
  vector<double> lZMPY;
  lZMPY.resize(NbOfIntervals);

  /* Initialization of the analytical sagital and orthogonal
     trajectories */
  PatternGeneratorJRL::AnalyticalZMPCOGTrajectory 
    aAZCTX(NbOfIntervals),aAZCTY(NbOfIntervals);

  vector<unsigned int> lPolynomialDegrees;
  aAM.GetPolynomialDegrees(lPolynomialDegrees);
  aAZCTX.SetPolynomialDegrees(lPolynomialDegrees);
  aAZCTY.SetPolynomialDegrees(lPolynomialDegrees);


  unsigned int NbIterationLimit = 2;
  struct timeval timestart, timestartit, timeend,
    time3rdorder,timePolynomial,timeTransfert,timeChangeStep;
  double Acc3rdorder=0.0,
    AccComputeW=0.0, 
    AccComputePolynomial=0.0,
    AccTransfert=0.0,
    AccTimeIt = 0.0,
    AccChangeStep = 0.0;

  PatternGeneratorJRL::CompactTrajectoryInstanceParameters aCTIPX, aCTIPY;
  double laddx,incZMPX = 0.0, incZMPY = 0.0;
  
  gettimeofday(&timestart,0);
  
  aAM.ResetTheResolutionOfThePolynomial();

  double LocalTime;
  double LocalTimeReference;

  LocalTime = 0.0;
  LocalTimeReference = 0.0;

  incZMPX = 0.0;
  incZMPY = 0.0;
  for(unsigned int NbIteration=0;NbIteration<NbIterationLimit;NbIteration++)
    {
      
      if (NbIteration%2==0)
	{
	  
	  LocalTime = 0.0;
	  LocalTimeReference = 0.0;
	  
	  incZMPX = 0.0;
	  incZMPY = 0.0;
  
	  lZMPX[0] = 0.0;
	  for(unsigned int i=1;i<NbOfIntervals-1;i+=2)
	    {
	      laddx= 0.1 * ((double)random())/(double)RAND_MAX;
	      laddx = 0.0;
	      incZMPX+=0.2 +laddx;
	      
	      lZMPX[i]=incZMPX;
	      lZMPX[i+1]=incZMPX;
	      
	    }  
	  FinalCoMPosX = lZMPX[NbOfIntervals-1];
	  
	  lZMPY[0] = 0.0;
	  double lsign=-1.0;
	  for(unsigned int i=1;i<NbOfIntervals-1;i+=2)
	    {
	      if (i==1)
		incZMPY+=lsign*0.09;
	      else
		incZMPY+=lsign*0.18;
	      
	      laddx= 0.5 * incZMPY*((double)random())/(double)RAND_MAX;
	      laddx = 0.0;
	      incZMPY += laddx;
	      
	      lZMPY[i]=incZMPY;
	      lZMPY[i+1]=incZMPY;
	      lsign=-1.0*lsign;
	    }  
	  FinalCoMPosY = lZMPY[NbOfIntervals-1];
	  aAM.SetDeltaTj(lDeltaTj);
	    
	}
      
      
      gettimeofday(&timestartit,0);

      if (NbIteration%2==1)
	{
	  if (lVerboseMode>5)
	    {
	      cout << "TAMCP2: Before" << endl << "X\tY"<<endl;
	      for(unsigned int i=0;i<(*aCTIPX.ZMPProfil).size();i++) 
		{
		  cout << (*aCTIPX.ZMPProfil)[i] << "\t"
		       << (*aCTIPY.ZMPProfil)[i] << endl; 
		}
	    }

	  int r;
	  PatternGeneratorJRL::FootAbsolutePosition aFAP;
	  aFAP.x = 0.7; aFAP.y = -0.09;

	  if ((r=aAM.ChangeFootLandingPosition(LocalTime,
					       5,aFAP,
					       aAZCTX,
					       aCTIPX,
					       aAZCTY,
					       aCTIPY,
					       true))<0)
	    {
	      string ErrorMessage;
	      aAM.StringErrorMessage(r,ErrorMessage);
	      cout << ErrorMessage<< endl;
	    }
	  else
	    {
	      double lCoMPosX, lCoMSpeedX;
	      aAZCTX.ComputeCOM(LocalTime,lCoMPosX);
	      aAZCTX.ComputeCOMSpeed(LocalTime,lCoMSpeedX);
	      if (0)
		cout << "Initial CoM : " << lCoMPosX << " Initial Speed: " << lCoMSpeedX << endl;
	      
	      /* New phase: Correct the fluctuation in the orthogonal axis */

	      aAM.FilterOutOrthogonalDirection(aAZCTY, aCTIPY, ZMPPositions, COGPositions);
	      
	      if(1)
		{
		  ofstream laof;
		  laof.open("AdditionnalZMP.dat",ofstream::out);
		  for(unsigned int li=0;li<ZMPPositions.size();li++)
		    {
		      laof << ZMPPositions[li] << " " << COGPositions[li]<< endl;
		    }
		  laof.close();
		}
	    }

	  gettimeofday(&timeChangeStep,0);
	  AccChangeStep += (timeChangeStep.tv_sec - timestartit.tv_sec ) 
	    + 0.000001 * (timeChangeStep.tv_usec - timestartit.tv_usec) ;
		
	}
      else 
	{

	  aAM.ResetTheResolutionOfThePolynomial();
	  aAM.BuildingTheZMatrix(lCoMZ,lZMPZ);

	  aAZCTX.SetStartingTimeIntervalsAndHeightVariation(lDeltaTj,OmegajCoefficients);
	  aAZCTY.SetStartingTimeIntervalsAndHeightVariation(lDeltaTj,OmegajCoefficients);

	  for(unsigned int i=1;i<NbOfIntervals-1;i++)
	    {
	      aAZCTX.Building3rdOrderPolynomial(i,lZMPX[i-1],lZMPX[i]);
	      aAZCTY.Building3rdOrderPolynomial(i,lZMPY[i-1],lZMPY[i]);
	    }
	  gettimeofday(&timePolynomial,0);

	  // Block for X trajectory
	  aCTIPX.InitialCoM = InitialCoMX;
	  aCTIPX.InitialCoMSpeed = InitialCoMSpeedX;
	  aCTIPX.FinalCoMPos = FinalCoMPosX;
	  aCTIPX.ZMPProfil = &lZMPX;
	  aCTIPX.ZMPZ = &lZMPZ;
	  aCTIPX.CoMZ = &lCoMZ;
	  aAM.ComputeTrajectory(aCTIPX,aAZCTX);
	  
	  // Block for Y trajectory.
	  aCTIPY.InitialCoM = InitialCoMY;
	  aCTIPY.InitialCoMSpeed = InitialCoMSpeedY;
	  aCTIPY.FinalCoMPos = FinalCoMPosY;
	  aCTIPY.ZMPProfil = &lZMPY;
	  aCTIPY.ZMPZ = &lZMPZ;
	  aCTIPY.CoMZ = &lCoMZ;
	  aAM.ComputeTrajectory(aCTIPY,aAZCTY);
	  

	  gettimeofday(&time3rdorder,0);

	  Acc3rdorder += (time3rdorder.tv_sec -timestartit.tv_sec) 
	    + 0.000001 * (time3rdorder.tv_usec -timestartit.tv_usec) ;
	  ODEBUG("End 3 rd solving ===========");  
      

	  AccTransfert += (time3rdorder.tv_sec - timePolynomial.tv_sec ) 
	    + 0.000001 * (time3rdorder.tv_usec - timePolynomial.tv_usec) ;

	  // Compute accumulated time in part of the algorithms.
	  AccComputePolynomial += (timePolynomial.tv_sec - timestartit.tv_sec ) 
	    + 0.000001 * (timePolynomial.tv_usec - timestartit.tv_usec) ;
	  
	  AccComputeW  +=  (time3rdorder.tv_sec - timePolynomial.tv_sec ) 
	    + 0.000001 * (timestartit.tv_usec - timestartit.tv_usec) ;
	}

      gettimeofday(&timeTransfert,0);
      

      AccTimeIt += (timeTransfert.tv_sec - timestartit.tv_sec ) 
	+ 0.000001 * (timeTransfert.tv_usec - timestartit.tv_usec) ;

	    
      // Save the trajectory
      if (1)
      {
	char Buffer[1024];
	
	sprintf(Buffer,"AnalyticalZMPCOGTrajectoryX_%03d.dat",NbIteration);
	string XTrajectoryFileName=Buffer;
	
	sprintf(Buffer,"AnalyticalZMPCOGTrajectoryY_%03d.dat",NbIteration);
	string YTrajectoryFileName=Buffer;
	
	double TMax = 0; 
	for(unsigned int i=0;i<lDeltaTj.size();i++)
	  TMax+= lDeltaTj[i];
	TMax+=LocalTime;
	double TStep=ControlStep;
	
	OutputInAFile(XTrajectoryFileName,aAZCTX,TMax, TStep,LocalTime);
	//	OutputInAFile(XTrajectoryFileName,aAZCTX,TMax, TStep, 0.0);
	
	OutputInAFileWithCorrections(YTrajectoryFileName,aAZCTY,TMax, TStep,LocalTime,
				     ZMPPositions,COGPositions);
	//OutputInAFile(YTrajectoryFileName,aAZCTY,TMax, TStep, 0.0);

      }

      // Update the current clock.
      LocalTime += 2.4;
      LocalTimeReference = LocalTime;
      

    }
      
  gettimeofday(&timeend,0);

  if (lVerboseMode>0)
    {
      double NbOfSeconds = (timeend.tv_sec - timestart.tv_sec) + 0.000001 * (timeend.tv_usec - timestart.tv_usec);
      cout << "Time for one iteration (stats over "<< NbIterationLimit << " iterations ): " <<
	NbOfSeconds/(double)NbIterationLimit << " " << AccTimeIt / (double)NbIterationLimit << endl;
      cout << "Time for 3rd order:" << Acc3rdorder / (double)NbIterationLimit<< endl;
      cout << "Time for computing W:" << AccComputeW / (double)(0.5 * NbIterationLimit)<< endl;
      cout << "Time for computing polynomial:" << AccComputePolynomial / (double)NbIterationLimit<< endl;
      cout << "Time for computing transfert:" << AccTransfert /(double) NbIterationLimit<< endl;
      cout << "Time for changing a step:" << AccChangeStep /(double) (0.5*NbIterationLimit) << endl;
    }
      
  string XTrajectoryFileName="AnalyticalZMPCOGTrajectoryX.dat";
  string YTrajectoryFileName="AnalyticalZMPCOGTrajectoryY.dat";

  
  
  double TMax = 0; 
  for(unsigned int i=0;i<lDeltaTj.size();i++)
    TMax+= lDeltaTj[i];
  TMax+=LocalTime;

  ofstream aof;
  aof.open("AnalyticalZMPCOGCompactCoeffsX.dat",ofstream::out);
  aof << aAZCTX;
  aof.close();

  aof.open("AnalyticalZMPCOGCompactCoeffsY.dat",ofstream::out);
  aof << aAZCTY;
  aof.close();

  aof.open("AMC2.dat",ofstream::out);
  aof << aAM;
  aof.close();

}

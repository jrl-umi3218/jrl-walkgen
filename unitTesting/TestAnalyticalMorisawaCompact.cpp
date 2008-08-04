#include <fstream>
#include <sys/time.h>
#include <time.h>

#include <walkGenJrl/Mathematics/Polynome.h>
#include <walkGenJrl/SimplePluginManager.h>
#include <walkGenJrl/ZMPRefTrajectoryGeneration/AnalyticalMorisawaCompact.h>


#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "TestAnalyticalMorisawaCompact :" << x << endl
#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#if 0
#define ODEBUG(x) cerr << "TestAnalyticalMorisawaCompact :" <<  x << endl
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


int main(void)
{
  PatternGeneratorJRL::SimplePluginManager aSPM;
  PatternGeneratorJRL::AnalyticalMorisawaCompact aAM(&aSPM);
  
  double TS=0.7,TD=0.1;
  double ControlStep = 0.005;
  int NbSteps = 3;
  int lVerboseMode = 0;
  int NbOfIntervals=2*NbSteps+1;

  aAM.SetTSingleSupport(TS);
  aAM.SetTDoubleSupport(TD);

  aAM.SetNumberOfStepsInAdvance(NbSteps);
  aAM.InitializeBasicVariables();

  vector<double> lDeltaTj;
  aAM.GetDeltaTj(lDeltaTj);
  lDeltaTj[0] =0.4;
  aAM.SetDeltaTj(lDeltaTj);
  
  vector<double> lCoMZ;
  vector<double> lZMPZ;
  lCoMZ.resize(NbOfIntervals);
  lZMPZ.resize(NbOfIntervals);
  for(unsigned int i=0;i<NbOfIntervals;i++)
    {
      lCoMZ[i] = 0.814;
      lZMPZ[i] = 0.0;
    }

  aAM.BuildingTheZMatrix(lCoMZ,lZMPZ);
  vector<double> OmegajCoefficients;
  aAM.GetOmegaj(OmegajCoefficients);


  double InitialCoMX=0.0;
  double InitialCoMSpeedX=0.0;
  double FinalCoMPosX=0.6;

  if (1)
    {
      InitialCoMX = 0.0172326;
      InitialCoMSpeedX=0.0799363;
    }
  vector<double> lZMPX;
  lZMPX.resize(NbOfIntervals);
  double incZMPX=0.0;

  PatternGeneratorJRL::AnalyticalZMPCOGTrajectory aAZCT(NbOfIntervals);
  vector<unsigned int> lPolynomialDegrees;
  aAM.GetPolynomialDegrees(lPolynomialDegrees);
  aAZCT.SetPolynomialDegrees(lPolynomialDegrees);

  if (lVerboseMode>5)
    {
      cout << "Tj: "<< lDeltaTj.size() << endl;
      for(unsigned int li=0;li<lDeltaTj.size();li++)
	cout << lDeltaTj[li] << " " ;
      cout << endl;
    }

  unsigned int NbIterationLimit = 1;
  struct timeval timestart, timestartit, timeend,
    time3rdorder,timeComputeW,timePolynomial,timeTransfert;
  double Acc3rdorder=0.0,
    AccComputeW=0.0, 
    AccComputePolynomial=0.0,
    AccTransfert=0.0;
    

  lZMPX[0] = 0.0;
  double lsign=-1.0;
  for(unsigned int i=1;i<NbOfIntervals-1;i+=2)
    {
      incZMPX+=0.2;

      lZMPX[i]=incZMPX;
      lZMPX[i+1]=incZMPX;
    }  
  FinalCoMPosX = lZMPX[NbOfIntervals-1];
  gettimeofday(&timestart,0);

  double laddx;
  for(unsigned int NbIteration=0;NbIteration<NbIterationLimit;NbIteration++)
    {
      aAM.ResetTheResolutionOfThePolynomial();  
      incZMPX = 0.0;
      for(unsigned int i=1;i<NbOfIntervals-1;i+=2)
	{
	  laddx= 0.1 * ((double)random())/(double)RAND_MAX;
	  laddx = 0.0;
	  incZMPX+=0.2 +laddx;
	  
	  if (i==1)
	    {
	      lZMPX[1] = 0.35;
	      lZMPX[2] = 0.35;
	    }
	  else
	    {
	      lZMPX[i]=incZMPX;
	      lZMPX[i+1]=incZMPX;
	    }
	}  
      FinalCoMPosX = lZMPX[NbOfIntervals-1];
      gettimeofday(&timestartit,0);
      aAZCT.SetStartingTimeIntervalsAndHeightVariation(lDeltaTj,OmegajCoefficients);
            
      ODEBUG("Start 3 rd solving ===========");
      for(unsigned int i=1;i<NbOfIntervals-1;i++)
	aAZCT.Building3rdOrderPolynomial(i,lZMPX[i-1],lZMPX[i]);

      gettimeofday(&time3rdorder,0);
      Acc3rdorder += (time3rdorder.tv_sec -timestartit.tv_sec) 
	+ 0.000001 * (time3rdorder.tv_usec -timestartit.tv_usec) ;
      ODEBUG("End 3 rd solving ===========");  
      
      
      if (lVerboseMode>=2)
	cout << "Compute W: " << endl;
      aAM.ComputeW(InitialCoMX,
		   InitialCoMSpeedX,
		   lZMPX,
		   FinalCoMPosX,
		   aAZCT);
      gettimeofday(&timeComputeW,0);
      AccComputeW += (timeComputeW.tv_sec - time3rdorder.tv_sec) 
	+ 0.000001 * (timeComputeW.tv_usec -time3rdorder.tv_usec) ;
      
      if (lVerboseMode>=2)
	cout << "Compute Polynomial weights begin " << endl;
      
      aAM.ComputePolynomialWeights2();
      gettimeofday(&timePolynomial,0);
      AccComputePolynomial += (timePolynomial.tv_sec - timeComputeW.tv_sec ) 
	+ 0.000001 * (timePolynomial.tv_usec - timeComputeW.tv_usec) ;

      if (lVerboseMode>=2)
	cout << "Compute Polynomial weights end" << endl;
      /*
      vector<double> PolynomialWeights;
      aAM.GetPolynomialWeights(PolynomialWeights);


     aAM.ComputePolynomialWeights();
      if (lVerboseMode>=2)
	{
	  for(unsigned int r =0; r< MAL_VECTOR_SIZE(PolynomialWeights);r++)
	    cout << PolynomialWeights[r] << " ";
	  cout << endl;
	}
      */
      aAM.TransfertTheCoefficientsToTrajectories(aAZCT,lCoMZ,lZMPZ,
						 lZMPX[0], lZMPX[NbOfIntervals-1],false);  
      gettimeofday(&timeTransfert,0);
      AccTransfert += (timeTransfert.tv_sec - timePolynomial.tv_sec ) 
	+ 0.000001 * (timeTransfert.tv_usec - timePolynomial.tv_usec) ;

    }
  gettimeofday(&timeend,0);

  if (lVerboseMode>0)
    {
      double NbOfSeconds = (timeend.tv_sec - timestart.tv_sec) + 0.000001 * (timeend.tv_usec - timestart.tv_usec);
      cout << "Time for one iteration (stats over "<< NbIterationLimit << " iterations ): " <<
	NbOfSeconds/(double)NbIterationLimit << endl;
      cout << "Time for 3rd order:" << Acc3rdorder / (double)NbIterationLimit<< endl;
      cout << "Time for computing W:" << AccComputeW / (double)NbIterationLimit<< endl;
      cout << "Time for computing polynomial:" << AccComputePolynomial / (double)NbIterationLimit<< endl;
      cout << "Time for computing transfert:" << AccTransfert /(double) NbIterationLimit<< endl;
    }
  // Write the output into a file.
  ofstream aof;
  aof.open("AnalyticalCompactZMPCOGTrajectory.dat",ofstream::out);

  for(double t=0;t< NbSteps * (TS+TD) + 3*TS; t+=ControlStep)
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

  aof.open("AnalyticalZMPCOGCompactCoeffs.dat",ofstream::out);
  aof << aAZCT;
  aof.close();
    
  aof.open("AMC1.dat",ofstream::out);
  aof << aAM;
  aof.close();

  double CoMPosX,CoMSpeedX;
  aAZCT.ComputeCOM(0.0,CoMPosX);
  aAZCT.ComputeCOMSpeed(0.0,CoMSpeedX);
  cout << " Initial Pos : " << CoMPosX<< " "
       << " Initial Speed: " << CoMSpeedX<< endl;
}

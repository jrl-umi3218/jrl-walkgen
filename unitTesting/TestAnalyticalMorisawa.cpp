/******************************************************************/
/* Program to test X,Y ZMP and COM trajectories generation.       */
/* This program test measures the time needed.                    */
/*                                                                */
/* Date: 11/07/2007                                               */
/*                                                                */
/* (c) JRL, CNRS/AIST, Olivier STASSE                             */
/*                                                                */
/******************************************************************/
#include <fstream>
#include <walkGenJrl/SimplePluginManager.h>
#include <walkGenJrl/ZMPRefTrajectoryGeneration/AnalyticalMorisawaFull.h>

int main(void)
{
  PatternGeneratorJRL::SimplePluginManager aSPM;
  PatternGeneratorJRL::AnalyticalMorisawaFull aAM(&aSPM);
  
  double TS=0.7,TD=0.1;
  double ControlStep = 0.005;
  int NbSteps = 3;
  int lVerboseMode = 3;
  int NbOfIntervals=2*NbSteps+1;

  aAM.SetTSingleSupport(TS);
  aAM.SetTDoubleSupport(TD);

  aAM.SetNumberOfStepsInAdvance(NbSteps);
  aAM.InitializeBasicVariables();


  PatternGeneratorJRL::AnalyticalZMPCOGTrajectory aAZCT(aAM.GetNumberOfIntervals());
  
  vector<double> lCoMZ;
  vector<double> lZMPZ;
  lCoMZ.resize(NbOfIntervals);
  lZMPZ.resize(NbOfIntervals);
  for(unsigned int i=0;i<NbOfIntervals;i++)
    {
      lCoMZ[i] = 0.68;
      lZMPZ[i] = 0.0;
    }

  aAM.BuildingTheZMatrix(lCoMZ,lZMPZ);
  
  double InitialCoMX=0.0;
  double InitialCoMSpeedX=0.0;
  double FinalCoMPosX=0.6;
  vector<double> lZMPX;
  lZMPX.resize(NbOfIntervals);
  double incZMPX=0.0;
  lZMPX[0] = 0.0;
  for(unsigned int i=1;i<NbOfIntervals-1;i+=2)
    {
      incZMPX+=0.2;
	  
      lZMPX[i]=incZMPX;
      lZMPX[i+1]=incZMPX;
    }

  if (lVerboseMode>=2)
    cout << "Compute W: " << endl;

  aAM.ComputeW(InitialCoMX,
	       InitialCoMSpeedX,
	       lZMPX,
	       FinalCoMPosX,
	       aAZCT);

  if (lVerboseMode>=2)
    cout << "Compute Polynomial weights begin " << endl;

  aAM.ComputePolynomialWeights();

  if (lVerboseMode>=2)
    cout << "Compute Polynomial weights end" << endl;

  vector<double> PolynomialWeights;
  aAM.GetPolynomialWeights(PolynomialWeights);
  if (lVerboseMode>=2)
    {
      for(unsigned int r =0; r< MAL_VECTOR_SIZE(PolynomialWeights);r++)
	cout << PolynomialWeights[r] << " ";
      cout << endl;
    }
  
  // Transfert the weights computed from AnalyticalMorisawa to
  // an analytical version of ZMP and COG trajectories.
  aAM.TransfertTheCoefficientsToTrajectories(aAZCT,lCoMZ,lZMPZ,
					     lZMPX[0], lZMPX[NbOfIntervals-1], 
					     true);
  

  // Write the output into a file.
  ofstream aof;
  aof.open("AnalyticalZMPCOGTrajectory.dat",ofstream::out);

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

  aof.open("AnalyticalZMPCOGCoeffs.dat",ofstream::out);
  aof << aAZCT;
  aof.close();

  // Verification part for the compact form.
  // This is trying to simulate what should the WCompactMatrix 
  // from the non-compact solution (validated).

  MAL_VECTOR(,double) VerifW;
  MAL_VECTOR_RESIZE(VerifW,2 * NbOfIntervals +6);

  PatternGeneratorJRL::Polynome * aPolynome, *aPolynomeNext;
  vector<double> COMCoeffs, COMCoeffsNext;
  vector<double> lDeltaTj;

  aAM.GetDeltaTj(lDeltaTj);
      
  // First part.
  VerifW(0)= 0.0;
  VerifW(1)= 0.0;
  aAZCT.GetFromListOfCOGPolynomials(1,aPolynomeNext);
  
  aPolynomeNext->GetCoefficients(COMCoeffsNext);
  VerifW(2) = COMCoeffsNext[0];
  VerifW(3) = COMCoeffsNext[1];
  VerifW(4) = 0;
  VerifW(5) = 0;
  unsigned int lindex=6;
  for(unsigned int i=2;i<NbOfIntervals;i++)
    {
      aAZCT.GetFromListOfCOGPolynomials(i-1,aPolynome);
      aAZCT.GetFromListOfCOGPolynomials(i,aPolynomeNext);
      aPolynome->GetCoefficients(COMCoeffs);
      aPolynomeNext->GetCoefficients(COMCoeffsNext);
      double r1=0.0,deltat=1.0;
      cout << "lDeltaTj: " << i-1 << " " << lDeltaTj[i-1] << endl;
      for(unsigned int j=0;j<COMCoeffs.size();j++)
	{
	  r1+= COMCoeffs[j]*deltat;
	  deltat *= lDeltaTj[i-1];
	  cout << " COMcoeffs : " << j << " : " << COMCoeffs[j] << endl;
	}

      if (i!=NbOfIntervals-1)
	VerifW(lindex++) = COMCoeffsNext[0] - r1;
      else 
	VerifW(lindex++) = lZMPX[NbOfIntervals-1] - r1;
      cout << "r1: " << r1 << " COMCoeffsNext[0] " << COMCoeffsNext[0] << endl;
      
      r1=0.0;deltat=1.0;
      for(unsigned int j=1;j<COMCoeffs.size();j++)
	{
	  r1+= j * COMCoeffs[j]* deltat;
	  deltat *= lDeltaTj[i-1];
	}

      if (i!=NbOfIntervals-1)
	VerifW(lindex++) = COMCoeffsNext[1] - r1;
      else 
	VerifW(lindex++) = - r1;
	    
      cout << "r2: " << r1 << " COMCoeffsNext[1] " << COMCoeffsNext[1] << endl;
    }
  VerifW(lindex++) = 0.0;
  VerifW(lindex++) = 0.0;
  VerifW(lindex++) = 0.0;
  VerifW(lindex++) = 0.0;

  aof.open("VerificationWCompactMatrix.dat" ,ofstream::out);
  for(unsigned int i=0;i<2*NbOfIntervals+6;i++)
    aof << VerifW(i) << " ";
  aof << endl;
  aof.close();
  
  
  
}

/* Olivier Stasse
 * (c) 2005-2009
 * 
 */

#include "CommonTools.h"

using namespace::PatternGeneratorJRL;
using namespace std;

void Herdt(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);  

  {
    istringstream strm2(":setVelReference  0.2 0.0 0.0");
    aPGI.ParseCmd(strm2);
  }
  {
    istringstream strm2(":singlesupporttime 0.7");
    aPGI.ParseCmd(strm2);
  }
  {
    istringstream strm2(":SetAlgoForZmpTrajectory Herdt");
    aPGI.ParseCmd(strm2);
  }

  if (0)
  {
    istringstream strm2(":setdimitrovconstraint XY 0.07 0.05");
    aPGI.ParseCmd(strm2);
  }


  {
    istringstream strm2(":stepseq 0.0 -0.105 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0 \
                     0.0 -0.21 0.0");
    aPGI.ParseCmd(strm2);
  }

}

void Herdt_Stop(PatternGeneratorInterface &aPGI)
{
  {
    istringstream strm2(":setVelReference  0.0 0.0 0.0");
    aPGI.ParseCmd(strm2);
  }
}


void HerdtOnline(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);

  {
    istringstream strm2(":SetAlgoForZmpTrajectory Herdt");
    aPGI.ParseCmd(strm2);

  }
  {
    istringstream strm2(":singlesupporttime 0.7");
    aPGI.ParseCmd(strm2);
  }
  {
    istringstream strm2(":HerdtOnline 0.2 0.0 0.1");
    aPGI.ParseCmd(strm2);
  }
}


int main(int argc, char *argv[])
{
  unsigned int TestProfil=PROFIL_HERDT_ONLINE;

  string VRMLPath;
  string VRMLFileName;
  string SpecificitiesFileName;
  string LinkJointRank;

  MAL_VECTOR_DIM(CurrentConfiguration,double,46);
  MAL_VECTOR_DIM(CurrentVelocity,double,46);
  MAL_VECTOR_DIM(CurrentAcceleration,double,46);
  MAL_VECTOR_DIM(PreviousConfiguration,double,46) ;
  MAL_VECTOR_DIM(PreviousVelocity,double,46);
  MAL_VECTOR_DIM(PreviousAcceleration,double,46);

  getOptions(argc,argv,
	     VRMLPath, VRMLFileName,
	     SpecificitiesFileName, LinkJointRank,
	     TestProfil);
  // Humanoids
  CjrlHumanoidDynamicRobot * aHDR = 0, * aDebugHDR = 0;
  // Create Pattern Generator Interface
  PatternGeneratorInterface * aPGI=0;

  // Instanciate and initialize.
  string RobotFileName = VRMLPath + VRMLFileName;
  CreateAndInitializeHumanoidRobot(RobotFileName,
				   SpecificitiesFileName,
				   LinkJointRank,
				   aHDR, aDebugHDR, aPGI);

  // Specify the walking mode: here the default one.
  istringstream strm2(":walkmode 0");
  aPGI->ParseCmd(strm2);

  
  MAL_VECTOR_DIM(ZMPTarget,double,3);
    
  //COMPosition CurrentWaistPosition;
  struct timeval begin,end,startingtime;
  unsigned long int NbOfIt=0, NbOfItToCompute=0;

  COMPosition finalCOMPosition;
  FootAbsolutePosition LeftFootPosition;
  FootAbsolutePosition RightFootPosition;


  bool DebugFGPI = true;
  bool DebugZMP2 = true;
  unsigned int PGIInterface = 0;
  
  double TimeProfile[200*620];
  bool bTimeProfile=true;
  double TimeProfileTS[200*620];
  unsigned int TimeProfileIndex = 0;
  unsigned int TimeProfileUpperLimit=200*620;

  ofstream aofzmpmb2;
  if (DebugZMP2)
    aofzmpmb2.open("ZMPMBSTAGE2.dat",ofstream::out);


  ofstream aof;
  if (DebugFGPI)
    {
      aof.open("TestFGPI_description.dat",ofstream::out);
      string Titles[26] =
	{ "Time",
	  "Com X",
	  "Com Y" ,
	  "Com Z" ,
	  "Com Yaw",
	  "Com dX" ,
	  "Com dY" ,
	  "Com dZ" ,
	  "ZMP X (waist ref.)" ,
	  "ZMP Y (waist ref.)" ,
	  "Left Foot X" ,
	  "Left Foot Y" ,
	  "Left Foot Z" ,
	  "Left Foot Theta" ,
	  "Left Foot Omega" ,
	  "Left Foot Omega2" ,
	  "Right Foot X" ,
	  "Right Foot Y" ,
	  "Right Foot Z" ,
	  "Right Foot Theta" ,
	  "Right Foot Omega" ,
	  "Right Foot Omega2" ,
	  "ZMP X (world ref.)" ,
	  "ZMP Y (world ref.)" ,
	  "Waist X (world ref.)" ,
	  "Waist Y (world ref.)" };
      for(unsigned int i=0;i<26;i++)
	aof << i+1 << ". " <<Titles[i] <<std::endl;
	
      aof.close();
      aof.open("TestFGPI.dat",ofstream::out);
    }

  double totaltime=0,maxtime=0;
  double totaltimemodif=0, timemodif = 0;
  double totaltimeinplanning=0;
  unsigned long int nbofmodifs=0;
  
  gettimeofday(&startingtime,0);
  // Number of sequences added.
  unsigned int lNbItMax = 1;
  
  for (unsigned int lNbIt=0;lNbIt<lNbItMax;lNbIt++)
    {
      //StrangeStartingPosition(*aPGI);
      cout << "<===============================================================>"<<endl;
      cout << "Iteration nb: " << lNbIt << endl;
      gettimeofday(&begin,0);
      switch (TestProfil) 
	{

	case PROFIL_HERDT:
	  Herdt(*aPGI);
	  break;
	  
	case PROFIL_HERDT_ONLINE:
	  HerdtOnline(*aPGI);
	  break;

	default:
	  break;
	};

	
      // Should generate the same than the one previous (but shorter to specify).

      gettimeofday(&end,0);
      double ltime = end.tv_sec-begin.tv_sec + 0.000001 * (end.tv_usec - begin.tv_usec);
      totaltimeinplanning+=ltime;

      aDebugHDR->currentConfiguration(PreviousConfiguration);	      
      aDebugHDR->currentVelocity(PreviousVelocity);
      aDebugHDR->currentAcceleration(PreviousAcceleration);
      aDebugHDR->computeForwardKinematics();

      bool ok = true;
      while(ok)
	{
	  gettimeofday(&begin,0);
	  
	  if (PGIInterface==0)
	    {
	      ok = aPGI->RunOneStepOfTheControlLoop(CurrentConfiguration,
						    CurrentVelocity,
						    CurrentAcceleration,
						    ZMPTarget,
						    finalCOMPosition,
						    LeftFootPosition,
						    RightFootPosition);
	    }
	  else if (PGIInterface==1)
	    {
	      ok = aPGI->RunOneStepOfTheControlLoop(CurrentConfiguration,
						    CurrentVelocity,
						    CurrentAcceleration,
						    ZMPTarget);
	    }

	  gettimeofday(&end,0);
	  double ltime = end.tv_sec-begin.tv_sec + 0.000001 * (end.tv_usec - begin.tv_usec);
	  if (maxtime<ltime)
	    maxtime = ltime;
	  NbOfIt++;
	  
	  if (ltime>0.000300)
	    {
	      totaltime += ltime;
	      NbOfItToCompute++;
	    }
	  
	  PreviousConfiguration = CurrentConfiguration;	  
	  PreviousVelocity = CurrentVelocity;
	  PreviousAcceleration = CurrentAcceleration;
	  
	  timemodif =0;
	  if (TestProfil==PROFIL_HERDT_ONLINE)
	    {
	      if (NbOfIt>3*200) /* Stop after 3 seconds the on-line stepping */
		{
		  Herdt_Stop(*aPGI);
		  if(NbOfIt > 5*200)
		    ok=false;
		}
	    }

	  TimeProfile[TimeProfileIndex] = ltime + timemodif;
	  TimeProfileTS[TimeProfileIndex] = begin.tv_sec + 0.000001 * begin.tv_usec;
	  TimeProfileIndex++;
	  if (TimeProfileIndex>TimeProfileUpperLimit)
	    TimeProfileIndex = 0;

	  if (DebugFGPI)
	    {
	      aof << NbOfIt*0.005 << " " 
		  << finalCOMPosition.x[0] << " "
		  << finalCOMPosition.y[0] << " " 
		  << finalCOMPosition.z[0] << " "
		  << finalCOMPosition.yaw << " "
		  << finalCOMPosition.x[1] << " "
		  << finalCOMPosition.y[1] << " " 
		  << finalCOMPosition.z[1] << " "
		  << ZMPTarget(0) << " " << ZMPTarget(1) << " " 
		  << LeftFootPosition.x  << " " << LeftFootPosition.y  << " " 
		  << LeftFootPosition.z  << " " << LeftFootPosition.theta  << " "  
		  << LeftFootPosition.omega  << " " << LeftFootPosition.omega2  << " "
		  << RightFootPosition.x << " " << RightFootPosition.y << " " 
		  << RightFootPosition.z << " " << RightFootPosition.theta << " " 
		  << RightFootPosition.omega  << " " << RightFootPosition.omega2  << " "
		  << ZMPTarget(0)*cos(CurrentConfiguration(5)) - 
		ZMPTarget(1)*sin(CurrentConfiguration(5))
		+CurrentConfiguration(0) << " " 
		  << ZMPTarget(0)*sin(CurrentConfiguration(5)) + 
		ZMPTarget(1)*cos(CurrentConfiguration(5))
		+CurrentConfiguration(1) << " "
		  << CurrentConfiguration(0) << " " 
		  << CurrentConfiguration(1) << " " 
		  << begin.tv_sec + 0.000001 * begin.tv_usec 
		  << endl;
	    }
	  
	}

      cout << "End of iteration " << lNbIt << endl;
      cout << "<===============================================================>"<<endl;
    }


  aofzmpmb2.close();

  if (bTimeProfile)
  {
    ofstream lProfileOutput("TimeProfile.dat",ofstream::out);
    double dST = startingtime.tv_sec + 0.000001 * startingtime.tv_usec;
    for(unsigned int i=0;i<TimeProfileIndex;i++)
      lProfileOutput << " " <<	TimeProfileTS[i] - dST
		     << " " << TimeProfile[i] << std::endl;

    lProfileOutput.close();
  }


  if (DebugFGPI)
    aof.close();

  
  delete aPGI;

  cout << "Number of iterations " << NbOfIt << " " << NbOfItToCompute << endl;
  cout << "Time consumption: " << (double)totaltime/(double)NbOfItToCompute 
       << " max time: " << maxtime <<endl;
  cout << "Time for modif: " << (double)totaltimemodif/(double)nbofmodifs 
       <<  " nb of modifs: " << nbofmodifs << endl ;
  cout << "Time on ZMP ref planning (Kajita policy): " 
       << totaltimeinplanning<< " " 
       << totaltimeinplanning*4/(double)NbOfIt<< endl;

}

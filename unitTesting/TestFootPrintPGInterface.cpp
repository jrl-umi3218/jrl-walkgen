#include <sys/time.h>
#include <time.h>
#include <sstream>
#include <fstream>
#include <PatternGeneratorInterface.h>

using namespace::PatternGeneratorJRL;

void CommonInitialization(PatternGeneratorInterface &aPGI)
{
  const char lBuffer[9][256] =
    {":omega 0.0",
     ":stepheight 0.07",
     ":singlesupporttime 0.78",
     ":doublesupporttime 0.02",
     ":armparameters 0.5",
     ":LimitsFeasibility 0.0",
     ":ZMPShiftParameters 0.015 0.015 0.015 0.015",
     ":TimeDistributionParameters 2.0 3.7 1.7 3.0",
     ":UpperBodyMotionParameters -0.1 -1.0 0.0"
    };
  
  for(int i=0;i<9;i++)
    {
      std::istringstream strm(lBuffer[i]);
      aPGI.ParseCmd(strm);
    }
  

}

void StraightWalking(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  
  {
    istringstream strm2(":stepseq 0.0 -0.095 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0 \
                     0.0 0.19 0.0");
    aPGI.ParseCmd(strm2);
  }

}


void StraightWalkingPBW(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":SetAlgoForZmpTrajectory PBW");
    aPGI.ParseCmd(strm2);
  }

  if (0)
  {
    istringstream strm2(":setpbwconstraint XY 0.07 0.05");
    aPGI.ParseCmd(strm2);
  }

  {
    istringstream strm2(":stepseq 0.0 -0.095 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0 \
                     0.2 0.19 0.0 \
                     0.0 -0.19 0.0");
    aPGI.ParseCmd(strm2);
  }

}

void CurvedWalkingPBW(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":SetAlgoForZmpTrajectory PBW");
    aPGI.ParseCmd(strm2);
  }

  if (0)
    {
      istringstream strm2(":setpbwconstraint XY 0.07 0.05");
      aPGI.ParseCmd(strm2);
    }
  
  {
    istringstream strm2(":stepseq 0.0 -0.095 0.0 \
                     0.2 0.19 10.0  \
                     0.2 -0.19 10.0 \
                     0.2 0.19 10.0  \
                     0.2 -0.19 10.0 \
                     0.2 0.19 10.0 \
                     0.0 -0.19 0.0");
    aPGI.ParseCmd(strm2);
  }

}


void CurvedWalkingPBW2(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":SetAlgoForZmpTrajectory PBW");
    aPGI.ParseCmd(strm2);
  }

  if (0)
    {
      istringstream strm2(":setpbwconstraint XY 0.07 0.05");
      aPGI.ParseCmd(strm2);
    }
  
  {
    istringstream strm2(":stepseq 0.0 -0.095 0.0 \
                     0.2 0.19 10.0  \
                     0.2 -0.19 10.0 \
                     0.2 0.19 10.0  \
                     0.2 -0.19 10.0 \
                     0.2 0.19 10.0 \
                     0.2 -0.19 -10.0 \
                     0.2 0.19 -10.0  \
                     0.2 -0.19 -10.0 \
                     0.2 0.19 -10.0 \
                     0.2 -0.19 -10.0 \
                     0.0 0.19 0.0");
    aPGI.ParseCmd(strm2);
  }

}

void ShortStraightWalking(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":stepseq 0.0 -0.095 0.0 \
                     0.2  0.19 0.0 \
                     0.2 -0.19 0.0 \
                     0.2  0.19 0.0 \
                     0.2 -0.19 0.0 \
                     0.2  0.19 0.0 \
                     0.0 -0.19 0.0");
    aPGI.ParseCmd(strm2);
  }

}

void Turn90DegreesWalking(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":stepseq 0.0 -0.095 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 10.0 \
                     0.2 0.19 10.0  \
                     0.2 -0.19 10.0 \
                     0.2 0.19 10.0  \
                     0.2 -0.19 10.0 \
                     0.2 0.19 10.0  \
                     0.2 -0.19 10.0 \
                     0.2 0.19 10.0  \
                     0.2 -0.19 10.0 \
                     0.0 0.19 0.0");
    aPGI.ParseCmd(strm2);
  }
}

void TurningOnTheCircleTowardsTheCenter(PatternGeneratorInterface &aPGI)
{

  CommonInitialization(aPGI);


  {
     istringstream strm2(":arccentered 0.75 360.0 -1");
     aPGI.ParseCmd(strm2);
  }
   
  {
     istringstream strm2(":finish");
     aPGI.ParseCmd(strm2);
  }

}

void TurningOnTheCircle(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);

   {
     istringstream strm2(":supportfoot 1");
     aPGI.ParseCmd(strm2);
  }

  {
     istringstream strm2(":arc 0.0 0.75 30.0 -1");
     aPGI.ParseCmd(strm2);
  }
  
  {
     istringstream strm2(":lastsupport");
     aPGI.ParseCmd(strm2);
  }

  {
     istringstream strm2(":finish");
     aPGI.ParseCmd(strm2);
  }


}

void StartOnLineWalking(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":StartOnLineStepSequencing");
    aPGI.ParseCmd(strm2);
  }
}

void StopOnLineWalking(PatternGeneratorInterface &aPGI)
{
  istringstream strm2(":StopOnLineStepSequencing");
  aPGI.ParseCmd(strm2);
}

void KineoWorks(PatternGeneratorInterface &aPGI)
{

  CommonInitialization(aPGI);
  {
    istringstream strm2(":walkmode 3");
    aPGI.ParseCmd(strm2);
  }

  {
    istringstream strm2(":readfilefromkw PartialModel.dat KWBarPath.pth");
    aPGI.ParseCmd(strm2);
  }
  {
    
    istringstream strm2(":stepseq 0.0 -0.095 0.0 0.0 0.2 0.19 0.0 0.0 0.2 -0.19 0.0 0.0 0.2 0.19 0.0 0.0 0.2 -0.19 0.0 0.0 0.2 0.19 0.0 -0.05 0.2 -0.19 0.0 -0.10 0.2 0.19 0.0 -0.15 0.2 -0.19 0.0 -0.2 0.2 +0.19 0.0 -0.2 0.2 -0.19 0.0 -0.2 0.2 +0.19 0.0 -0.2 0.2 -0.19 0.0 -0.15 0.2 +0.19 0.0 -0.07 0.2 -0.19 0.0 0.0 0.0 0.19 0.0 0.0");
    aPGI.ParseCmd(strm2);
  }

}

int main(int argc, char *argv[])
{
  if (argc!=6)
    {
      cerr << " This program takes 5 arguments: " << endl;
      cerr << "./TestHumanoidDynamicRobot PATH_TO_PC_PARAMS_FILE PATH_TO_VRML_FILE VRML_FILE_NAME PATH_TO_SPECIFICITIES_XML LINK_JOINT_RANK" << endl;
      exit(-1);
    }	

  string PCParametersFile = argv[1];
  string VRMLPath=argv[2];
  string VRMLFileName=argv[3];
  string SpecificitiesFileName = argv[4];
  string LinkJointRank = argv[5];
  string Global=PCParametersFile;
  Global+= " ";
  Global+=VRMLPath;
  Global+= " ";
  Global+= VRMLFileName;
  Global+= " ";
  Global+=SpecificitiesFileName;
  Global+= " ";
  Global+=LinkJointRank;

  std::istringstream strm(Global);
  //("./data/PreviewControlParameters.ini ../../etc/HRP2JRL/ HRP2JRLmain.wrl ./data/HRP2Specificities.xml");
  PatternGeneratorInterface * aPGI;

  aPGI = new PatternGeneratorInterface(strm);

  //  cout << "before PGI " << endl;
  // Initial position;
#if 0 // With previous half-sitting value
  double dInitPos[40] = { 
    0.0, 0.0, -20.0, 40.0, -20.0, 0.0, 0.0, 0.0, -20.0, 40.0, -20.0, 0.0,  // legs

    0.0, 0.0, -23.7, 6.6, // chest and head

    27.0, -5.0, -4.0, -87.0, -4.0, -16.0, 20.0, // right arm
    15.0,  10.0, 0.0, -20.0, 0.0, 0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  };
#endif

  // Nicolas position + New half sitting for the legs
#if 0
  double dInitPos[40] = { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, -23.7, 6.6, // chest and head

    27.0, -5.0, -4.0, -87.0, -4.0, -16.0, 20.0, // right arm
    15.0,  10.0, 0.0, -20.0, 0.0, 0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  };
#endif

  // Test to compare with previous PG.
#if 0
  double dInitPos[40] = { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // right arm
    0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // left arm 

    0.0, 0.0, 0.0, 0.0, 0.0, // right hand
    0.0, 0.0, 0.0, 0.0, 0.0  // left hand
  };
#endif

  // Normal position - New half sitting 
#if 1 
  double dInitPos[40] = { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // left arm 

    -10.0, 10.0, -10.0, 10.0, -10.0  // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  };
#endif

  // This is a vector corresponding to the DOFs actuated of the robot.
  MAL_VECTOR_DIM(InitialPosition,double,40);
  //MAL_VECTOR_DIM(CurrentPosition,double,40);
  for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
    InitialPosition(i) = dInitPos[i]*M_PI/180.0;
  aPGI->SetCurrentJointValues(InitialPosition);

  // Specify the walking mode: here the default one.
  if (1)
  {
    istringstream strm2(":walkmode 0");
    aPGI->ParseCmd(strm2);
  }

  // This is a vector corresponding to ALL the DOFS of the robot:
  // free flyer + actuated DOFS.
  MAL_VECTOR_DIM(CurrentConfiguration,double,46);
  MAL_VECTOR_DIM(CurrentVelocity,double,46);
  MAL_VECTOR_DIM(PreviousConfiguration,double,46) ;
  MAL_VECTOR_DIM(PreviousVelocity,double,46);
  for(int i=0;i<6;i++)
    {
      PreviousConfiguration[i] = 
	PreviousVelocity[i] =0.0;
    }

  for(int i=6;i<46;i++)
    {
      PreviousConfiguration[i] = InitialPosition[i-6];
      PreviousVelocity[i] = 0.0;
    }

  MAL_VECTOR_DIM(ZMPTarget,double,3);
  
    
  //COMPosition CurrentWaistPosition;
  struct timeval begin,end;
  unsigned long int NbOfIt=0;
  gettimeofday(&begin,0);

  for (unsigned int lNbIt=0;lNbIt<2;lNbIt++)
    {
      // ShortStraightWalking(*aPGI);
      CurvedWalkingPBW2(*aPGI);
      // KineoWorks(*aPGI);
      // StraightWalking(*aPGI);
      //      CurvedWalkingPBW(*PGI);
      // StraightWalkingPBW(*aPGI);
      // Turn90DegreesWalking(aPGI);
      // TurningOnTheCircle(*aPGI); 
      
      // Should generate the same than the one previous (but shorter to specify).

      // TurningOnTheCircleTowardsTheCenter(*aPGI);
      // TurningOnTheCircleTowardsTheCenter(aPGI);
      // StartOnLineWalking(*aPGI);
      
      while(aPGI->RunOneStepOfTheControlLoop(CurrentConfiguration,
					     CurrentVelocity,
					     ZMPTarget))
	{
	  NbOfIt++;
	  // cout << "LocalIndex :" << NbOfIt << endl;
	  // Record the angular values generated by the PG.
	  // aPGI->SetCurrentJointValues(CurrentPosition);
	  
	  aPGI->DebugControlLoop(PreviousConfiguration,PreviousVelocity,NbOfIt);
	  PreviousConfiguration = CurrentConfiguration;
	  PreviousVelocity = CurrentVelocity;
	  if (NbOfIt>30*200) /* Stop after 30 seconds the on-line stepping */
	    {
	      StopOnLineWalking(*aPGI);
	    }
	}
    }
  gettimeofday(&end,0);
  double ltime = end.tv_sec-begin.tv_sec + 0.000001 * (end.tv_usec - begin.tv_usec);
  cout << "Number of iterations " << NbOfIt << endl;
  cout << "Time consumption: " << (double)ltime/(double)NbOfIt << endl;

}

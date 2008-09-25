#include <sys/time.h>
#include <time.h>
#include <sstream>
#include <fstream>
#include <walkGenJrl/PatternGeneratorInterface.h>

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


void TestNewPG1(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);

  {
    istringstream strm2(":walkmode 0");
    aPGI.ParseCmd(strm2);
  }

  {
    istringstream strm2(":stepseq 0.0 -0.095  0.0 \
                                 -0.2  0.19   0.0 \
                                 -0.2 -0.19   0.0 \
                                 -0.2  0.19   0.0 \
                                 -0.2 -0.19   0.0 \
                                 -0.1  0.19 -10.0 \
                                  0.0 -0.19 -10.0 \
                                  0.0  0.19 -10.0 \
                                  0.0 -0.19 -10.0 \
                                  0.0  0.19 -10.0 \
                                  0.0 -0.19 -10.0 \
                                  0.0  0.19 -10.0 \
                                  0.0 -0.19 -10.0 \
                                  0.0  0.19 -10.0 \
                                  0.0 -0.19   0.0");
    aPGI.ParseCmd(strm2);
  }
}

void TestNewPG2(PatternGeneratorInterface &aPGI)
{
  {
    istringstream strm2(":stepseq 0.0 -0.095   0.0 \
                                  0.0  0.19  -10.0 \
                                  0.0 -0.19  -10.0 \
                                  0.0  0.19  -10.0 \
                                  0.0 -0.19  -10.0 \
                                  0.0  0.19  -10.0 \
                                  0.0 -0.19  -10.0 \
                                  0.0  0.19  -10.0 \
                                  0.0 -0.19  -10.0 \
                                  0.0  0.19  -10.0 \
                                  0.0 -0.19  -10.0 \
                                  0.0  0.19  -10.0 \
                                  0.0 -0.19  -10.0 \
                                  0.0  0.19  -10.0 \
                                  0.0 -0.19    0.0");
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
    istringstream strm2(":stepseq 0.0 -0.105 0.0 \
                     0.2  0.21 0.0 \
                     0.2 -0.21 0.0 \
                     0.2  0.21 0.0 \
                     0.2 -0.21 0.0 \
                     0.2  0.21 0.0 \
                     0.0 -0.21 0.0");
    
    aPGI.ParseCmd(strm2);
  }
}

void AnalyticalShortStraightWalking(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
    aPGI.ParseCmd(strm2);
  }

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
    /*    istringstream strm2(":StartOnLineStepSequencing 0.0 -0.095 0.0 \
                     0.2 0.19 0.0 \				   \
                     0.2 -0.19 0.0");*/
    istringstream strm2(":StartOnLineStepSequencing 0.0 0.105 0.0 \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0 \
                     0.2 -0.21 0.0 ");

    aPGI.ParseCmd(strm2);
  }
}

void StartSimuOnLineWalking(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    /*    istringstream strm2(":StartOnLineStepSequencing 0.0 -0.095 0.0 \
                     0.2 0.19 0.0 \				   \
                     0.2 -0.19 0.0");*/
    istringstream strm2(":stepseq 0.0 0.105 0.0 \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0 \
                     0.2 -0.21 0.0 \
                     0.0 0.21 0.0 \
                     0.0 -0.21 0.0 \
                     0.0 0.21 0.0 \
                     0.0 -0.21 0.0 \
                     0.0 0.21 0.0 ");

    aPGI.ParseCmd(strm2);
  }
}

void StartAnalyticalOnLineWalking(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);

  {
    istringstream strm2(":SetAlgoForZmpTrajectory Morisawa");
    aPGI.ParseCmd(strm2);
  }

  {
    istringstream strm2(":StartOnLineStepSequencing 0.0 -0.095 0.0 \
                     0.2 0.19 0.0 \
                     0.2 -0.19 0.0");
    aPGI.ParseCmd(strm2);
  }
}

void StrangeStartingPosition(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);

  {
    istringstream strm2(":SetAlgoForZmpTrajectory Kajita");
    aPGI.ParseCmd(strm2);
  }

  {
    istringstream strm2(":stepseq 0.075 0.125 7.16197 0.15 -0.25 14.3239");
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

void SteppingOver(PatternGeneratorInterface &aPGI)
{
  const char lBuffer[3][256] =
    { ":walkmode 2",
      ":UpperBodyMotionParameters -0.1 -1.0 0.0",
      ":stepseq 0.0 -0.095 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0  0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.0 0.19 0.0"
    };
  
  for(int i=0;i<3;i++)
    {
      std::istringstream strm(lBuffer[i]);
      aPGI.ParseCmd(strm);
    }
}

int main(int argc, char *argv[])
{
#if 1

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
#else 
  string PCParametersFile("/home/stasse/src/OpenHRP/JRL/src/PatternGeneratorJRL_underdev/src/data/PreviewControlParameters.ini");
  string VRMLPath("/home/stasse/src/OpenHRP/etc/HRP2JRL/");
  string VRMLFileName("HRP2JRLmain.wrl");
  string SpecificitiesFileName("/home/stasse/src/OpenHRP/JRL/src/PatternGeneratorJRL_underdev/src/data/HRP2Specificities.xml");
  string LinkJointRank("/home/stasse/src/OpenHRP/JRL/src/PatternGeneratorJRL_underdev/src/data/HRP2LinkJointRank.xml");
#endif
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
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // left arm 

    -10.0, 10.0, -10.0, 10.0, -10.0  // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  };
#endif

#if 0
// Strange position 1
  double dInitPos[40] = 
    {
      14.323945,  -6.0363396,  -13.459409,    44.02602,  -30.566611,    6.0363396,
      0.0000001,   7.4859801,  -27.663319,    44.65489,  -16.991579,   -7.4859801,
      0.,    0.,    0.,    0.,    
      12.397718,  -10.000004,    0.,  -29.618538,    0.,    0.,    10.0,
      16.536364,   10.000004,    0.,  -29.828011,    0.,    0.,    10.0,

      -10.0,  10.0, -10.0,  10,   -10.0, 
      -10.0,  10.0, -10.0,  10.0, -10.0 
    };

  double dInitPos[40] = 
    {
      -7.16197, -7.69299, -16.1787, 44.5201, -28.3415,  7.69299, 
      7.16197,   5.74946, -31.3668, 44.1057, -12.7389, -5.74946,

      0., 0., 0., 0., 

      12.622 , -10, 0, -29.678 , 0, 0, 10, 
      16.7091,  10, 0, -29.7841, 0, 0, 10, 
      
      -10.0,  10.0, -10.0,  10,   -10.0, 
      -10.0,  10.0, -10.0,  10.0, -10.0 
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
  MAL_VECTOR_DIM(CurrentAcceleration,double,46);
  MAL_VECTOR_DIM(PreviousConfiguration,double,46) ;
  MAL_VECTOR_DIM(PreviousVelocity,double,46);
  MAL_VECTOR_DIM(PreviousAcceleration,double,46);
  for(int i=0;i<6;i++)
    {
      PreviousConfiguration[i] = 
	PreviousVelocity[i] = 
	PreviousAcceleration[i] = 0.0;
    }

  for(int i=6;i<46;i++)
    {
      PreviousConfiguration[i] = InitialPosition[i-6];
      PreviousVelocity[i] = 
	PreviousAcceleration[i] = 0.0;
    }

  MAL_VECTOR_DIM(ZMPTarget,double,3);
  
    
  //COMPosition CurrentWaistPosition;
  struct timeval begin,end,beginmodif,endmodif;
  unsigned long int NbOfIt=0, NbOfItToCompute=0;


  bool TestChangeFoot = true;

  COMPosition finalCOMPosition;
  FootAbsolutePosition LeftFootPosition;
  FootAbsolutePosition RightFootPosition;
  ofstream aof,aofq;
  aofq.open("TestConfiguration.dat",ofstream::out);
  aof.open("TestFGPI.dat",ofstream::out);
  double totaltime=0,maxtime=0;
  double timemodif = 0;

  for (unsigned int lNbIt=0;lNbIt<1;lNbIt++)
    {
      //StrangeStartingPosition(*aPGI);
      /*
      if (lNbIt==1)
	TestNewPG1(*aPGI);
      else if (lNbIt==2)
	TestNewPG2(*aPGI);
      */
      // SteppingOver(*aPGI);
      //ShortStraightWalking(*aPGI);
      // CurvedWalkingPBW2(*aPGI);
      // KineoWorks(*aPGI);
      // StraightWalking(*aPGI);
      
      // AnalyticalShortStraightWalking(*aPGI);
      // CurvedWalkingPBW(*PGI);
      // StraightWalkingPBW(*aPGI);
      // Turn90DegreesWalking(aPGI);
      // TurningOnTheCircle(*aPGI); 
      
      // Should generate the same than the one previous (but shorter to specify).

      // TurningOnTheCircleTowardsTheCenter(*aPGI);
      // TurningOnTheCircleTowardsTheCenter(aPGI);
      // StartAnalyticalOnLineWalking(*aPGI);
      StartOnLineWalking(*aPGI);
      //StartSimuOnLineWalking(*aPGI);

      bool ok = true;
      while(ok)
	{

	  gettimeofday(&begin,0);
#if 1
	  ok = aPGI->RunOneStepOfTheControlLoop(CurrentConfiguration,
						CurrentVelocity,
						CurrentAcceleration,
						ZMPTarget,
						finalCOMPosition,
						LeftFootPosition,
						RightFootPosition);
#else
	  ok = aPGI->RunOneStepOfTheControlLoop(CurrentConfiguration,
						CurrentVelocity,
						CurrentAcceleration,
						ZMPTarget);
#endif
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
	  /*
	  aPGI->DebugControlLoop(PreviousConfiguration,
				 PreviousVelocity,
				 PreviousAcceleration,
				 NbOfIt);
	  */
	  PreviousConfiguration = CurrentConfiguration;
	  PreviousVelocity = CurrentVelocity;
	  PreviousAcceleration = CurrentAcceleration;
	  
	  
	  if ((NbOfIt>5*200) && 
	      TestChangeFoot)
	    {
	      FootAbsolutePosition aFAP;
	      aFAP.x=0.2;
	      aFAP.y=0.0;
	      gettimeofday(&beginmodif,0);
	      //aPGI->ChangeOnLineStep(15.5,aFAP);
	      istringstream strm2(":parsecmd :addstandardonlinestep 0.2 0.0 0.0");
	      aPGI->ParseCmd(strm2);
	      gettimeofday(&endmodif,0);
	      timemodif = endmodif.tv_sec-beginmodif.tv_sec + 0.000001 * (endmodif.tv_usec - beginmodif.tv_usec);
	      TestChangeFoot=false;
	    }

	  if (NbOfIt>10*200) /* Stop after 30 seconds the on-line stepping */
	    {
	      StopOnLineWalking(*aPGI);
	    }

	  aof << NbOfIt*0.005 << " " 
	      << finalCOMPosition.x[0] << " "
	      << finalCOMPosition.y[0] << " " 
	      << finalCOMPosition.z[0] << " "
	      << ZMPTarget(0) << " " << ZMPTarget(1) << " " 
	      << LeftFootPosition.x  << " " << LeftFootPosition.y  << " " << LeftFootPosition.z  << " "
	      << RightFootPosition.x << " " << RightFootPosition.y << " " << RightFootPosition.z << " " 
	      << ZMPTarget(0)+CurrentConfiguration(0) << " " 
	      << ZMPTarget(1)+CurrentConfiguration(1) << " " << endl;
	  for(unsigned int k=0;k<30;k++)
	    {
	      aofq << CurrentConfiguration[k+6]*180/M_PI << " ";
	    }
	  aofq << endl;

	}
    }
  aofq.close();
  aof.close();

  delete aPGI;

  cout << "Number of iterations " << NbOfIt << " " << NbOfItToCompute << endl;
  cout << "Time consumption: " << (double)totaltime/(double)NbOfItToCompute << " max time: " << maxtime <<endl;
  cout << "Time for modif: " << timemodif << endl;

}

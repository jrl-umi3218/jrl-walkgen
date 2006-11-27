#include <sys/time.h>
#include <time.h>
#include <sstream>
#include <fstream>
#include <PatternGeneratorInterface.h>

using namespace::PatternGeneratorJRL;

void CommonInitialization(PatternGeneratorInterface &aPGI)
{
  {
    std::istringstream strm2(":omega 0.0");
    aPGI.ParseCmd(strm2);
  }

  {
    istringstream strm2(":stepheight 0.07");
    aPGI.ParseCmd(strm2);
  }

  {
    istringstream strm2(":singlesupporttime 0.78");
    aPGI.ParseCmd(strm2);
  }

  {
    istringstream strm2(":doublesupporttime 0.02");
    aPGI.ParseCmd(strm2);
  }
  
  {
    istringstream strm2(":armparameters 0.5");
    aPGI.ParseCmd(strm2);
  }

  {
    istringstream strm2(":LimitsFeasibility 0.0");
    aPGI.ParseCmd(strm2);
  }
  
  {
    istringstream strm2(":ZMPShiftParameters 0.015 0.015 0.015 0.015");
    aPGI.ParseCmd(strm2);
  }

  {
    istringstream strm2(":TimeDistributionParameters 2.0 3.7 1.7 3.0");
    aPGI.ParseCmd(strm2);
  }
  
  {
    istringstream strm2(":UpperBodyMotionParameters -0.1 -1.0 0.0");
    aPGI.ParseCmd(strm2);
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
     istringstream strm2(":arc 0.0 0.75 90.0 -1");
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

int main(void)
{

  cout << "Starting point " << endl;
  std::istringstream strm("PreviewControlParameters.ini ../../etc/HRP2JRL/ HRP2JRLmain.wrl HRP2Specificities.xml");
  PatternGeneratorInterface * aPGI;
  aPGI = new PatternGeneratorInterface(strm);

  cout << "before PGI " << endl;
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

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  };
#endif

  VNL::Matrix<double> InitialPosition(40,1,0.0),CurrentPosition(40,1,0.0);
  for(unsigned int i=0;i<InitialPosition.Rows();i++)
    InitialPosition[i][0] = dInitPos[i]*M_PI/180.0;

  aPGI->SetCurrentJointValues(InitialPosition);
  

  // Specify the walking mode: here the default one.
  {
    istringstream strm2(":walkmode 0");
    aPGI->ParseCmd(strm2);
  }



  VNL::Matrix<double> qr(6,1,0.0),ql(6,1,0.0),
    UpperBodyAngles(28,1,0.0),
    ZMPTarget(3,1,0.0);
  
  for(unsigned int i=0;i<UpperBodyAngles.Rows();i++)
    UpperBodyAngles[i][0] = dInitPos[i+12]*M_PI/180.0;
    
  ofstream aof;
  aof.open("DDO.dat",ofstream::out);
  COMPosition CurrentWaistPosition;
  cout << "Reached here" << endl;

  ShortStraightWalking(*aPGI);
  //Turn90DegreesWalking(aPGI);
  //TurningOnTheCircle(*aPGI); // Should generate the same than the one previous (but shorter to specify).
  //  TurningOnTheCircleTowardsTheCenter(*aPGI);
  //TurningOnTheCircleTowardsTheCenter(aPGI);
  //StartOnLineWalking(*aPGI);

  cout << "Reached here 2" << endl;
  struct timeval begin,end;
  unsigned long int NbOfIt=0;
  gettimeofday(&begin,0);
  while(aPGI->RunOneStepOfTheControlLoop(qr,ql,UpperBodyAngles,ZMPTarget,CurrentWaistPosition))
    {
      NbOfIt++;
      // cout << "LocalIndex :" << NbOfIt << endl;
      // Record the angular values generated by the PG.
#if 0
      int i;

      for(i=0;i<6;i++)
	aof << qr(i,0)*180.0/M_PI << " ";
      for(i=0;i<6;i++)
	aof << ql(i,0)*180.0/M_PI << " ";
      for(i=0;i<28;i++)
	aof << UpperBodyAngles(i,0)*180.0/M_PI << " ";
      for(i=0;i<3;i++)
	aof << ZMPTarget(i,0) << " ";
      aof << CurrentWaistPosition.x[0] << " "
	  << CurrentWaistPosition.y[0] << " "
	  << CurrentWaistPosition.z[0] << " ";
      aof <<endl;
#endif
      // Update the current values (just in case some body modifies the values of the upper body.)
      // Here we feed back the values generated by the PG.
      for(unsigned int i=0;i<6;i++)
	{
	  CurrentPosition[i][0] = qr(i,0);
	  CurrentPosition[i+6][0] = ql(i,0);
	}
      for(unsigned int i=0;i<40-12;i++)
	CurrentPosition(i+12,0) = UpperBodyAngles(i,0);
      
      aPGI->SetCurrentJointValues(CurrentPosition);
      aPGI->DebugControlLoop(qr,ql,UpperBodyAngles,CurrentWaistPosition,NbOfIt);
      
      if (NbOfIt>30*200) /* Stop after 30 seconds the on-line stepping */
	{
	  StopOnLineWalking(*aPGI);
	}

    }

  gettimeofday(&end,0);
  double ltime = end.tv_sec-begin.tv_sec + 0.000001 * (end.tv_usec - begin.tv_usec);
  cout << "Number of iterations " << NbOfIt << endl;
  cout << "Time consumption: " << (double)ltime/(double)NbOfIt << endl;
  aof.close();
}

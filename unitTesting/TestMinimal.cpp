#include <sstream>
#include <jrlMathTools/jrlConstants.h>
#include <walkGenJrl/PatternGeneratorInterface.h>

using namespace::PatternGeneratorJRL;

void StraightWalking(PatternGeneratorInterface &aPGI)
{
  const char lBuffer[5][256] =
    {":omega 0.0",
     ":stepheight 0.07",
     ":singlesupporttime 0.78",
     ":doublesupporttime 0.02",
     ":armparameters 0.5"};

  for(int i=0;i<5;i++)
    {
      std::istringstream strm(lBuffer[i]);
      aPGI.ParseCmd(strm);
    }

  istringstream strm2(":stepseq 0.0 -0.095 0.0 \
                     0.2 0.19 0.0	       \
                     0.2 -0.19 0.0	       \
                     0.2 0.19 0.0	       \
                     0.2 -0.19 0.0	       \
                     0.2 0.19 0.0	       \
                     0.2 -0.19 0.0	       \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0 \
                     0.2 0.19 0.0  \
                     0.2 -0.19 0.0		\
                     0.0 0.19 0.0");
  aPGI.ParseCmd(strm2);

}

int main(int argc, char *argv[])
{

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

  PatternGeneratorInterface * aPGI;

  aPGI = new PatternGeneratorInterface(strm);

  // Normal position - New half sitting 
  double dInitPos[40] = { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // left arm 

    -10.0, 10.0, -10.0, 10.0, -10.0  // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  };

  // This is a vector corresponding to the DOFs actuated of the robot.
  MAL_VECTOR_DIM(InitialPosition,double,40);
  MAL_VECTOR_DIM(CurrentPosition,double,40);
  for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
    InitialPosition(i) = dInitPos[i]*M_PI/180.0;
  aPGI->SetCurrentJointValues(InitialPosition);


  // This is a vector corresponding to ALL the DOFS of the robot:
  // free flyer + actuated DOFS.
  MAL_VECTOR_DIM(CurrentConfiguration,double,46);
  MAL_VECTOR_DIM(CurrentVelocity,double,46);
  MAL_VECTOR_DIM(CurrentAcceleration,double,46);
  MAL_VECTOR_DIM(PreviousConfiguration,double,46) ;
  MAL_VECTOR_DIM(PreviousVelocity,double,46);
  MAL_VECTOR_DIM(PreviousAcceleration,double,46);
  MAL_VECTOR_DIM(ZMPTarget,double,3);
  int NbOfIt=0;

  StraightWalking(*aPGI);
  while(aPGI->RunOneStepOfTheControlLoop(CurrentConfiguration,
					 CurrentVelocity,
					 CurrentAcceleration,
					     ZMPTarget))
    {
      NbOfIt++;
      aPGI->DebugControlLoop(PreviousConfiguration,PreviousVelocity,PreviousAcceleration,NbOfIt);
      PreviousConfiguration = CurrentConfiguration;
      PreviousVelocity = CurrentVelocity;
      PreviousAcceleration = CurrentAcceleration;
    }

}

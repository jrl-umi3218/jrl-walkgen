
/* Olivier Stasse
 * (c) 2005-2009
 * 
 */

#ifdef UNIX
#include <sys/time.h>
#include <stdlib.h>
#endif /*UNIX*/

#ifdef WIN32
#include <Windows.h>
#include "TimeUtilsWindows.h"
#endif /*WIN32*/

#include <time.h>
#include <sstream>
#include <fstream>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

#ifdef WITH_HRP2DYNAMICS
#include <hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h>
#endif

#include <walkGenJrl/PatternGeneratorInterface.h>

#include "TestFootPrintPGInterfaceData.h"

using namespace::PatternGeneratorJRL;
using namespace std;


void CommonInitialization(PatternGeneratorInterface &aPGI)
{
  const char lBuffer[12][256] =
    {":samplingperiod 0.005",
     ":previewcontroltime 1.6",
     ":comheight 0.814",
     ":omega 0.0",
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
    istringstream strm2(":stepseq 0.0 -0.105 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.0 0.21 0.0");
    aPGI.ParseCmd(strm2);
  }

}

void PbFlorentSeq1(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":stepseq 0 -0.1 0 \
0.203885 0.214762 2.27277 \
0.210865 -0.208919 5.07812 \
0.209585 0.189649 5.68817 \
0.211002 -0.208084 5.02252 \
0.206673 0.213288 3.90856 \
0.215084 -0.204415 2.77733 \
0.203042 0.217162 1.75895 \
0.218517 -0.201212 0.846647 \
0.219977 0.200271 -0.0129191 \
0.201539 -0.218175 -0.887889 \
0.216723 0.203487 -1.85508 \
0.205132 -0.214262 -2.9997 \
0.212108 0.207836 -4.40306 \
0.210196 -0.188306 -6.08503 \
0.185675 0.213231 -7.85383 \
0.215002 -0.182611 -9.1292 \
0.183249 0.214462 -9.15803 \
0.212975 -0.186052 -7.79003 \
0.189799 0.208778 -5.68383 \
0.206109 -0.214158 -3.56669 \
0.216837 0.202638 -1.79747 \
0.200882 -0.219409 -0.506198 \
0.0136717 0.2 -0.00157708 \
0 -0.2 0");
    aPGI.ParseCmd(strm2);
  }
}

void PbFlorentSeq2(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":stepseq \
                                 0 -0.1 0 \
				-0.0512076 0.207328 -1.15414 \
				-0.0473172 -0.218623 -1.15414 \
				-0.0515644 0.21034 -1.15414 \
				-0.0475332 -0.215615 -1.15414 \
				-0.0516395 0.203345 -1.15414 \
				-0.0476688 -0.217615 -1.15414 \
				-0.0517348 0.201344 -1.15414 \
				-0.0477237 -0.219617 -1.15414 \
				-0.0517494 0.21934 -1.15414 \
				-0.047698 -0.201621 -1.15414 \
				-0.0516832 0.217337 -1.15414 \
				-0.0475915 -0.203622 -1.15414 \
				-0.0515365 0.215339 -1.15414 \
				-0.0474046 -0.205617 -1.15414 \
				-0.0513094 0.213348 -1.15414 \
				-0.0471374 -0.207603 -1.15414 \
				-0.0510024 0.216368 -1.15414 \
				-0.0466898 -0.214575 -1.15414 \
				-0.0506158 0.214402 -1.15414 \
				-0.0462637 -0.216533 -1.15414 \
				-0.0501503 0.217453 -1.15414 \
				-0.0456584 -0.223471 -1.15414 \
				-0.0366673 0.212742 1.62857 \
				-0.0360079 -0.201543 4.21944 \
				-0.0154622 0.279811 4.21944 \
				-0.0300936 -0.217751 4.21944 \
				-0.00928506 0.283157 4.21944 \
				-0.0236871 -0.219869 4.21944 \
				-0.00231593 0.290546 4.21944 \
				-0.0169269 -0.202959 4.21944 \
				0.00493436 0.296941 4.21944 \
				-0.00995958 -0.202061 4.21944 \
				0.0119489 0.297324 4.21944 \
				-0.00293587 -0.202195 4.21944 \
				0.0189437 0.296673 4.21944 \
				0.00399208 -0.203358 4.21944 \
				0.0257673 0.295003 4.21944 \
				0.0106743 -0.200526 4.21944 \
				0.031904 0.287363 4.21944 \
				0.016966 -0.203651 4.21944 \
				0.0379487 0.283784 4.21944 \
				0.141438 -0.212069 3.6834 \
				0.204562 0.216453 2.64204 \
				0.200635 -0.218747 -0.366254 \
				0.216228 0.204108 -2.13008 \
				0.206583 -0.212425 -3.87382 \
				0.187966 0.211947 -6.61811 \
				0.219749 -0.17341 -12.4824 \
				0.146814 0.240465 -26.5643 \
				0.247166 -0.119114 -37.1489 \
				0.163211 0.222722 -19.7198 \
				0.208825 -0.213706 -5.15242 \
				0.0285368 0.200005 -0.0337318 \
				0 -0.2 0 ");
    aPGI.ParseCmd(strm2);
  }
      
}


void PbFlorentSeq3(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":stepseq \
				0 -0.1 0 \
				0.206454 0.189754 3.86836 \
				0.174414 -0.222668 13.578 \
				0.225916 0.166888 16.7012 \
				0.173605 -0.22035 14.1845 \
				0.215782 0.183337 9.55516 \
				0.211335 -0.206707 4.86378 \
				0.217278 0.205158 -1.51671 \
				0.213124 -0.179685 -10.3377 \
				0.0326875 0.233626 -1.98501 \
				0.0384266 -0.200022 -1.98501 \
				0.0290562 0.271027 -1.98501 \
				0.0359251 -0.217685 -1.98501 \
				0.0264753 0.273276 -1.98501 \
				0.0332679 -0.200527 -1.98501 \
				0.0235717 0.280338 -1.98501 \
				0.0304676 -0.218558 -1.98501 \
				0.0207049 0.282209 -1.98501 \
				0.0275378 -0.201788 -1.98501 \
				0.0175423 0.288873 -1.98501 \
				0.0244924 -0.200224 -1.98501 \
				0.0144446 0.29033 -1.98501 \
				0.021346 -0.218876 -1.98501 \
				0.0112534 0.291569 -1.98501 \
				0.0181139 -0.202748 -1.98501 \
				0.00781095 0.297581 -1.98501 \
				0.0148115 -0.201847 -1.98501 \
				0.00447927 0.298367 -1.98501 \
				0.0114545 -0.201177 -1.98501 \
				0.00110116 0.29892 -1.98501 \
				0.00805928 -0.200741 -1.98501 \
				-0.00230717 0.299239 -1.98501 \
				0.00464196 -0.200541 -1.98501 \
				-0.00572937 0.29932 -1.98501 \
				0.00117482 -0.20335 -1.27115 \
				0 0.2 0 ");
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
    istringstream strm2(":stepseq 0.0 -0.105  0.0 \
                                 -0.2  0.21   0.0 \
                                 -0.2 -0.21   0.0 \
                                 -0.2  0.21   0.0 \
                                 -0.2 -0.21   0.0 \
                                 -0.1  0.21 -10.0 \
                                  0.0 -0.21 -10.0 \
                                  0.0  0.21 -10.0 \
                                  0.0 -0.21 -10.0 \
                                  0.0  0.21 -10.0 \
                                  0.0 -0.21 -10.0 \
                                  0.0  0.21 -10.0 \
                                  0.0 -0.21 -10.0 \
                                  0.0  0.21 -10.0 \
                                  0.0 -0.21   0.0");
    aPGI.ParseCmd(strm2);
  }
}

void TestNewPG2(PatternGeneratorInterface &aPGI)
{
  {
    istringstream strm2(":stepseq 0.0 -0.105   0.0 \
                                  0.0  0.21  -10.0 \
                                  0.0 -0.21  -10.0 \
                                  0.0  0.21  -10.0 \
                                  0.0 -0.21  -10.0 \
                                  0.0  0.21  -10.0 \
                                  0.0 -0.21  -10.0 \
                                  0.0  0.21  -10.0 \
                                  0.0 -0.21  -10.0 \
                                  0.0  0.21  -10.0 \
                                  0.0 -0.21  -10.0 \
                                  0.0  0.21  -10.0 \
                                  0.0 -0.21  -10.0 \
                                  0.0  0.21  -10.0 \
                                  0.0 -0.21    0.0");
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

void StraightWalkingDimitrov(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":SetAlgoForZmpTrajectory Dimitrov");
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

void CurvedWalkingDimitrov(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":SetAlgoForZmpTrajectory Dimitrov");
    aPGI.ParseCmd(strm2);
  }

  if (0)
    {
      istringstream strm2(":setpbwconstraint XY 0.07 0.05");
      aPGI.ParseCmd(strm2);
    }
  
  {
    istringstream strm2(":stepseq 0.0 -0.105 0.0 \
                     0.2 0.21 10.0  \
                     0.2 -0.21 10.0 \
                     0.2 0.21 10.0  \
                     0.2 -0.21 10.0 \
                     0.2 0.21 10.0 \
                     0.2 -0.21 -10.0 \
                     0.2 0.21 -10.0  \
                     0.2 -0.21 -10.0 \
                     0.2 0.21 -10.0 \
                     0.2 -0.21 -10.0 \
                     0.0 0.21 0.0");
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
    istringstream strm2(":stepseq 0.0 -0.105 0.0 \
                     0.2 0.21 10.0  \
                     0.2 -0.21 10.0 \
                     0.2 0.21 10.0  \
                     0.2 -0.21 10.0 \
                     0.2 0.21 10.0 \
                     0.0 -0.21 0.0");
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
    istringstream strm2(":stepseq 0.0 -0.105 0.0 \
                     0.2 0.21 10.0  \
                     0.2 -0.21 10.0 \
                     0.2 0.21 10.0  \
                     0.2 -0.21 10.0 \
                     0.2 0.21 10.0 \
                     0.2 -0.21 -10.0 \
                     0.2 0.21 -10.0  \
                     0.2 -0.21 -10.0 \
                     0.2 0.21 -10.0 \
                     0.2 -0.21 -10.0 \
                     0.0 0.21 0.0");
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

void ShortStraightWalkingOneStage(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":SetAlgoForZmpTrajectory KajitaOneStage");
    aPGI.ParseCmd(strm2);
  }

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

void Turn90DegreesWalking(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
  {
    istringstream strm2(":stepseq 0.0 -0.105 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 10.0 \
                     0.2 0.21 10.0  \
                     0.2 -0.21 10.0 \
                     0.2 0.21 10.0  \
                     0.2 -0.21 10.0 \
                     0.2 0.21 10.0  \
                     0.2 -0.21 10.0 \
                     0.2 0.21 10.0  \
                     0.2 -0.21 10.0 \
                     0.0 0.21 0.0");
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
    /*    istringstream strm2(":StartOnLineStepSequencing 0.0 -0.105 0.0 \
                     0.2 0.21 0.0 \				   \
                     0.2 -0.21 0.0");*/
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
    /*    istringstream strm2(":StartOnLineStepSequencing 0.0 -0.105 0.0 \
                     0.2 0.21 0.0 \				   \
                     0.2 -0.21 0.0");*/
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
    istringstream strm2(":onlinechangestepframe relative");
    aPGI.ParseCmd(strm2);
  }

  {
    istringstream strm2(":SetAutoFirstStep false");
    aPGI.ParseCmd(strm2);
  }

  {
    istringstream strm2(":StartOnLineStepSequencing 0.0 -0.095 0.0 \
                     0.0 0.19 0.0 \
                     0.0 -0.19 0.0 \
                     0.0 0.19 0.0");
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
    
    istringstream strm2(":stepseq 0.0 -0.105 0.0 0.0 0.2 \
                          0.21 0.0 0.0 0.2 -0.21 0.0 0.0 0.2 \
                          0.21 0.0 0.0 0.2 -0.21 0.0 0.0 0.2 \
                          0.21 0.0 -0.05 0.2 -0.21 0.0 -0.10 0.2 \
                          0.21 0.0 -0.15 0.2 -0.21 0.0 -0.2 0.2 \
                         +0.21 0.0 -0.2 0.2 -0.21 0.0 -0.2 0.2 \
                         +0.21 0.0 -0.2 0.2 -0.21 0.0 -0.15 0.2 \
                         +0.21 0.0 -0.07 0.2 -0.21 0.0 0.0 0.0 \
                          0.21 0.0 0.0");
    aPGI.ParseCmd(strm2);
  }

}

void SteppingOver(PatternGeneratorInterface &aPGI)
{
  const char lBuffer[3][256] =
    { ":walkmode 2",
      ":UpperBodyMotionParameters -0.1 -1.0 0.0",
      ":stepseq 0.0 -0.105 0.0 0.2 0.21 0.0 0.2 -0.21 0.0 0.2 \
                0.21 0.0 0.2 -0.21 0.0 0.2 0.21 0.0 0.2 \
               -0.21 0.0  0.2 0.21 0.0 0.2 -0.21 0.0 0.2 \
                0.21 0.0 0.2 -0.21 0.0 0.0 0.21 0.0"
    };
  
  for(int i=0;i<3;i++)
    {
      std::istringstream strm(lBuffer[i]);
      aPGI.ParseCmd(strm);
    }
}


int main(int argc, char *argv[])
{
  //  unsigned int TestProfil=PROFIL_STRAIGHT_WALKING;
  unsigned int TestProfil=PROFIL_ANALYTICAL_ONLINE_WALKING;
  string PCParametersFile;
  string VRMLPath;
  string VRMLFileName;
  string SpecificitiesFileName;
  string LinkJointRank;

  
  if (argc!=6)
    {
      const char *openhrphome="OPENHRPHOME";
      char *value = 0;
      value = getenv(openhrphome);
      if (value==0)
	{
	  cerr << " This program takes 5 arguments: " << endl;
	  cerr << "./TestFootPrintPGInterface \
                         PATH_TO_PC_PARAMS_FILE	  \
                         PATH_TO_VRML_FILE	   \
                         VRML_FILE_NAME		   \
                         PATH_TO_SPECIFICITIES_XML \
                         LINK_JOINT_RANK" << endl;
	  exit(-1);
	}
      else
	{
	  PCParametersFile = value;
	  PCParametersFile += "Controller/IOserver/robot/HRP2JRL/etc/";
	  PCParametersFile +="PreviewControlParameters.ini";
	  VRMLPath=value;
	  VRMLPath+="Controller/IOserver/robot/HRP2JRL/model/";
	  VRMLFileName="HRP2JRLmain.wrl";
	  SpecificitiesFileName = value;
	  SpecificitiesFileName +="Controller/IOserver/robot/HRP2JRL/etc/";
	  SpecificitiesFileName += "HRP2Specificities.xml";
	  LinkJointRank = value;
	  LinkJointRank += "Controller/IOserver/robot/HRP2JRL/etc/";
	  LinkJointRank += "HRP2LinkJointRank.xml";
	  
	  if (argc==2)
	    {
	      TestProfil=atoi(argv[1]);
	      cout << "Profil: " << ProfilesNames[TestProfil] << endl;
	    }
	      
	}
    }	
  else 
    {
      PCParametersFile = argv[1];
      VRMLPath=argv[2];
      VRMLFileName=argv[3];
      SpecificitiesFileName = argv[4];
      LinkJointRank = argv[5];
    }


  // Creating the humanoid robot.
  CjrlHumanoidDynamicRobot * aHDR = 0;
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;

#ifndef WITH_HRP2DYNAMICS
  aHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
#else
  aHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
#endif

  // Parsing the file.
  string RobotFileName = VRMLPath + VRMLFileName;
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,
					 LinkJointRank,
					 SpecificitiesFileName);

  // Create Pattern Generator Interface
  PatternGeneratorInterface * aPGI;
  aPGI = patternGeneratorInterfaceFactory(aHDR);

  bool conversiontoradneeded=true;
  
  double * dInitPos = InitialPoses[HALF_SITTING_2008];
  
  // This is a vector corresponding to the DOFs actuated of the robot.
  MAL_VECTOR_DIM(InitialPosition,double,40);
  //MAL_VECTOR_DIM(CurrentPosition,double,40);
  if (conversiontoradneeded)
    for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
      InitialPosition(i) = dInitPos[i]*M_PI/180.0;
  else
    for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
      InitialPosition(i) = dInitPos[i];
  aPGI->SetCurrentJointValues(InitialPosition);

  // Specify the walking mode: here the default one.
  istringstream strm2(":walkmode 0");
  aPGI->ParseCmd(strm2);

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
  struct timeval begin,end,startingtime;
  unsigned long int NbOfIt=0, NbOfItToCompute=0;


  bool TestChangeFoot = true;

  unsigned int NbStepsModified = 0;

  COMPosition finalCOMPosition;
  FootAbsolutePosition LeftFootPosition;
  FootAbsolutePosition RightFootPosition;

  bool DebugConfiguration = true;
  bool DebugFGPI = true;
  unsigned int PGIInterface = 0;
  
  double TimeProfile[200*620];
  double TimeProfileTS[200*620];
  unsigned int TimeProfileIndex = 0;
  unsigned int TimeProfileUpperLimit=200*620;

  ofstream aofq;
  if (DebugConfiguration)
    {
      aofq.open("TestConfiguration.dat",ofstream::out);
      if (aofq.is_open())
	{
	  for(unsigned int k=0;k<30;k++)
	    {
	      aofq << dInitPos[k] << " ";
	    }
	  aofq << endl;
	}

    }

  ofstream aof;
  if (DebugFGPI)
    {
      aof.open("TestFGPI_description.dat",ofstream::out);
      string Titles[25] =
	{ "Time",
	  "Com X",
	  "Com Y" ,
	  "Com Z" ,
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
      for(unsigned int i=0;i<25;i++)
	aof << i+1 << ". " <<Titles[i] <<std::endl;
	
      aof.close();
      aof.open("TestFGPI.dat",ofstream::out);
    }

  double totaltime=0,maxtime=0;
  double totaltimemodif=0, timemodif = 0;
  double totaltimeinplanning=0;
  double newtime=0,deltatime=0;
  unsigned long int nbofmodifs=0;
  
  gettimeofday(&startingtime,0);
  for (unsigned int lNbIt=0;lNbIt<1;lNbIt++)
    {
      //StrangeStartingPosition(*aPGI);
      
      gettimeofday(&begin,0);
      switch (TestProfil) 
	{

	case PROFIL_PB_FLORENT:
	  if (lNbIt==0)
	    PbFlorentSeq1(*aPGI);
	  else if (lNbIt==1)
	    PbFlorentSeq2(*aPGI);
	  else if (lNbIt==2)
	    PbFlorentSeq3(*aPGI);  
	    break;

	case PROFIL_STEPPING_OVER:
	  SteppingOver(*aPGI);
	  break;

	case PROFIL_SHORT_STRAIGHT_WALKING:
	  ShortStraightWalking(*aPGI);
	  break;

	case PROFIL_SHORT_STRAIGHT_WALKING_ONE_STAGE:
	  ShortStraightWalking(*aPGI);
	  break;	  

	case PROFIL_CURVED_WALKING_PBW2:
	  CurvedWalkingPBW2(*aPGI);
	  break;

	case PROFIL_KINEOWORKS:
	  KineoWorks(*aPGI);
	  break;

	case PROFIL_STRAIGHT_WALKING:
	  StraightWalking(*aPGI);
	  break;

	case PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING:
	  AnalyticalShortStraightWalking(*aPGI);
	  break;

	case PROFIL_CURVED_WALKING_PBW:
	  CurvedWalkingPBW(*aPGI);
	  break;

	case PROFIL_STRAIGHT_WALKING_DIMITROV:
	  StraightWalkingDimitrov(*aPGI);
	  break;

	case PROFIL_CURVED_WALKING_DIMITROV:
	  CurvedWalkingDimitrov(*aPGI);
	  break;

	case PROFIL_TURN_90D:
	  Turn90DegreesWalking(*aPGI);
	  break;

	case PROFIL_TURNING_ON_THE_CIRCLE:
	  TurningOnTheCircle(*aPGI);
	  break;

	case PROFIL_TURNING_ON_THE_CIRCLE_TOWARDS_THE_CENTER:
	  TurningOnTheCircleTowardsTheCenter(*aPGI);
	  break;

	case PROFIL_ANALYTICAL_ONLINE_WALKING:
	  StartAnalyticalOnLineWalking(*aPGI);
	  break;

	case PROFIL_ONLINE_WALKING:
	  StartOnLineWalking(*aPGI);
	  break;
	  
	case PROFIL_SIMU_ONLINE_WALKING:
	  StartSimuOnLineWalking(*aPGI);

	default:
	  break;
	};

      // Should generate the same than the one previous (but shorter to specify).

      //StartSimuOnLineWalking(*aPGI);
      gettimeofday(&end,0);
      double ltime = end.tv_sec-begin.tv_sec + 0.000001 * (end.tv_usec - begin.tv_usec);
      totaltimeinplanning+=ltime;
      
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
	  
	  /* aPGI->DebugControlLoop(PreviousConfiguration,
				 PreviousVelocity,
				 PreviousAcceleration,
				 NbOfIt);  */
	  PreviousConfiguration = CurrentConfiguration;
	  PreviousVelocity = CurrentVelocity;
	  PreviousAcceleration = CurrentAcceleration;
	  
	  timemodif =0;
			      
	  if (TestProfil==PROFIL_ANALYTICAL_ONLINE_WALKING)
	    {
	      if (NbOfIt>50*200) /* Stop after 30 seconds the on-line stepping */
		{
		  StopOnLineWalking(*aPGI);
		}
	      else{
		
		//if ((NbOfIt>(8.82*200)) && 
		//		double triggertime = 9.64*200 + deltatime*200;
		//double triggertime = 12.44*200 + deltatime*200;
		// double triggertime = 2.455*200 + deltatime*200;
		double triggertime = 9.64*200 + deltatime*200;
		if ((NbOfIt>triggertime) && 
		    TestChangeFoot)
		  {
		    struct timeval beginmodif,endmodif;
		    PatternGeneratorJRL::FootAbsolutePosition aFAP;
		    //aFAP.x=0.2;
		    //aFAP.y=-0.09;
		    if (nbofmodifs<NBOFPREDEFONLINEFOOTSTEPS)
		      {
			aFAP.x = OnLineFootSteps[nbofmodifs][0];
			aFAP.y = OnLineFootSteps[nbofmodifs][1];
			aFAP.theta = OnLineFootSteps[nbofmodifs][2];
		      }
		    else
		      {
			aFAP.x=0.1;
			aFAP.y=0.0;
			aFAP.theta=5.0;
		      }
		    gettimeofday(&beginmodif,0);
		    aPGI->ChangeOnLineStep(0.805,aFAP,newtime);
		    deltatime += newtime+0.025;
		    // std::cout << "trigger time: "<< triggertime/200.0 << std::endl;
		    // std::cout << "delta time: " << deltatime << " newtime: " << newtime << std::endl;
		    //istringstream strm2(":parsecmd :addstandardonlinestep 0.2 0.0 0.0");
		    //aPGI->ParseCmd(strm2);
		    gettimeofday(&endmodif,0);
		    timemodif = endmodif.tv_sec-beginmodif.tv_sec + 
		      0.000001 * (endmodif.tv_usec - beginmodif.tv_usec);
		    totaltimemodif += timemodif;
		    nbofmodifs++;
		    TestChangeFoot=true;
		    NbStepsModified++;
		    if (NbStepsModified==360)
		      TestChangeFoot=false;
		  }
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
		  << ZMPTarget(0)+CurrentConfiguration(0) << " " 
		  << ZMPTarget(1)+CurrentConfiguration(1) << " "
		  << CurrentConfiguration(0) << " " 
		  << CurrentConfiguration(1) << " " 
		  << begin.tv_sec + 0.000001 * begin.tv_usec 
		  << endl;
	    }
	  
	  if (DebugConfiguration)
	    {
	      for(unsigned int k=0;k<30;k++)
		{
		  aofq << CurrentConfiguration[k+6]*180/M_PI << " ";
		}
	      aofq << endl;
	    }
	}
    }

  {
    ofstream lProfileOutput("TimeProfile.dat",ofstream::out);
    double dST = startingtime.tv_sec + 0.000001 * startingtime.tv_usec;
    for(unsigned int i=0;i<TimeProfileIndex;i++)
      lProfileOutput << " " <<	TimeProfileTS[i] - dST
		     << " " << TimeProfile[i] << std::endl;

    lProfileOutput.close();
  }
  if (DebugConfiguration)
    aofq.close();

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

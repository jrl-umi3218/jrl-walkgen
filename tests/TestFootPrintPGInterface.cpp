/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010, 
 *
 * Mehdi    Benallegue 
 * Andrei   Herdt
 * Francois Keith
 * Mathieu  Poirier
 * Olivier  Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the 
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */

#include "portability/gettimeofday.hh"

#ifdef WIN32
# include <Windows.h>
#endif /* WIN32 */

#include <time.h>
#include <sstream>
#include <fstream>

#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl/dynamics/dynamicsfactory.hh>

#ifdef WITH_HRP2DYNAMICS
#include <hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h>
#endif

#include <jrl/walkgen/patterngeneratorinterface.hh>

#include "TestFootPrintPGInterfaceData.h"

using namespace::PatternGeneratorJRL;
using namespace std;

double InitialPoses[7][40] = {

  // 1- With previous half-sitting value
  { 
    0.0, 0.0, -20.0, 40.0, -20.0, 0.0, 0.0, 0.0, -20.0, 40.0, -20.0, 0.0,  // legs

    0.0, 0.0, -23.7, 6.6, // chest and head

    27.0, -5.0, -4.0, -87.0, -4.0, -16.0, 20.0, // right arm
    15.0,  10.0, 0.0, -20.0, 0.0, 0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  },
  // 2- Nicolas position + New half sitting for the legs
  { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, -23.7, 6.6, // chest and head

    27.0, -5.0, -4.0, -87.0, -4.0, -16.0, 20.0, // right arm
    15.0,  10.0, 0.0, -20.0, 0.0, 0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  },
  // 3- Test for comparison with PG v1.x
  { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // right arm
    0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // left arm 

    0.0, 0.0, 0.0, 0.0, 0.0, // right hand
    0.0, 0.0, 0.0, 0.0, 0.0  // left hand
  },
  // 4- New Half sitting
  { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // left arm 

    -10.0, 10.0, -10.0, 10.0, -10.0,  // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  },
  // 5- Position for interaction
  {
    0.0 ,  0.0 , -26.0 ,  50.0 , -24 ,   0.0,  
    0.0 ,  0.0 , -26.0 ,  50.0 , -24 ,   0.0 , // legs
    0.0 ,  0.0 ,  // chest
    0.0 ,  0.0 , // head 

    10.0, -18.0, 0.0, -100.0, -18.0, 0.0, 10.0, // right arm  
    10.0,  18.0, 0.0, -100.0,  18.0, 0.0, 10.0, 
    -10.000004 ,  10.000004 , -10.000004 ,  10.000004 , -10.000004 , // right hand 
    -10.000004 ,  10.000004 , -10.000004 ,  10.000004 , -10.000004 // left hand
  },
  // 6- Initial position for model building 1,
  {
    14.323945,  -6.0363396,  -13.459409,    44.02602,  -30.566611,    6.0363396,
    0.0000001,   7.4859801,  -27.663319,    44.65489,  -16.991579,   -7.4859801,
    0.,    0.,    0.,    0.,    
    12.397718,  -10.000004,    0.,  -29.618538,    0.,    0.,    10.0,
    16.536364,   10.000004,    0.,  -29.828011,    0.,    0.,    10.0,
    
    -10.0,  10.0, -10.0,  10,   -10.0, 
    -10.0,  10.0, -10.0,  10.0, -10.0 
  },
  // 7- Initial position for model buiding 2
  {
    -7.16197, -7.69299, -16.1787, 44.5201, -28.3415,  7.69299, 
    7.16197,   5.74946, -31.3668, 44.1057, -12.7389, -5.74946,
    
    0., 0., 0., 0., 
    
    12.622 , -10, 0, -29.678 , 0, 0, 10, 
    16.7091,  10, 0, -29.7841, 0, 0, 10, 
    
    -10.0,  10.0, -10.0,  10,   -10.0, 
    -10.0,  10.0, -10.0,  10.0, -10.0 
  }

};



//      ":comheight 0.807727",
void CommonInitialization(PatternGeneratorInterface &aPGI)
{
  const char lBuffer[12][256] =
    {":comheight 0.8078",
     ":samplingperiod 0.005",
     ":previewcontroltime 1.6",
     ":omega 0.0",
     ":stepheight 0.07",
     ":singlesupporttime 0.7",
     ":doublesupporttime 0.1",
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
  // Evaluate current state of the robot in the PG.
  COMState   lStartingCOMPosition;
  MAL_S3_VECTOR_TYPE(double)  lStartingZMPPosition;
  MAL_VECTOR_TYPE(double)  lStartingWaistPose;
  FootAbsolutePosition  InitLeftFootAbsPos;
  FootAbsolutePosition  InitRightFootAbsPos;

  aPGI.EvaluateStartingState(lStartingCOMPosition,
			     lStartingZMPPosition,
			     lStartingWaistPose,
			     InitLeftFootAbsPos,
			     InitRightFootAbsPos);
      

}

void StraightWalking(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
   {
    istringstream strm2(":SetAlgoForZmpTrajectory Kajita");
    aPGI.ParseCmd(strm2);
  }
 
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
    istringstream strm2(":SetAlgoForZmpTrajectory Kajita");
    aPGI.ParseCmd(strm2);
  }

  {
    /*
    istringstream strm2(":stepseq 0 -0.1 0 \
0.203885 0.214762 2.27277		   \
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
    */
istringstream strm2(":stepseq 0 0.1 0 \
	-0.0398822	-0.232351	4.6646 \
	-0.0261703	0.199677	4.6646 \
	-0.0471999	-0.256672	4.6646 \
	-0.0305785	0.200634	4.6646 \
	-0.0507024	-0.245393	4.6646 \
	-0.0339626	0.197227	4.6646 \
	-0.0527259	-0.228579	4.6646 \
	-0.0362332	0.199282	4.6646 \
	-0.0540087	-0.21638	4.6646 \
	-0.0373302	0.196611	4.6646 \
	-0.0536928	-0.199019	4.6646 \
	-0.0372245	0.204021	4.6646 \
	-0.0529848	-0.196642	4.6646 \
	-0.0355124	0.2163	4.6646 \
	-0.000858977	-0.204807	 0.0767924 \
0 0.2 0");
    aPGI.ParseCmd(strm2);
  }
}

void PbFlorentSeq2(PatternGeneratorInterface &aPGI)
{
  CommonInitialization(aPGI);
   {
    istringstream strm2(":SetAlgoForZmpTrajectory Kajita");
    aPGI.ParseCmd(strm2);
  }

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
    istringstream strm2(":SetAlgoForZmpTrajectory Kajita");
    aPGI.ParseCmd(strm2);
  }

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
    istringstream strm2(":HerdtOnline 0.2 0.0 0.0");
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
  // unsigned int TestProfil=PROFIL_STRAIGHT_WALKING;
  // unsigned int TestProfil=PROFIL_ANALYTICAL_ONLINE_WALKING;
   unsigned int TestProfil=PROFIL_HERDT_ONLINE;
  //unsigned int TestProfil = PROFIL_STRAIGHT_WALKING_DIMITROV;

  string PCParametersFile;
  string VRMLPath;
  string VRMLFileName;
  string SpecificitiesFileName;
  string LinkJointRank;

  
  if (argc!=5)
    {
      const char *openhrphome="OPENHRPHOME";
      char *value = 0;
      value = getenv(openhrphome);
      if (value==0)
	{
	  cerr << " This program takes 4 arguments: " << endl;
	  cerr << "./TestFootPrintPGInterface \
                         PATH_TO_VRML_FILE	   \
                         VRML_FILE_NAME		   \
                         PATH_TO_SPECIFICITIES_XML \
                         LINK_JOINT_RANK" << endl;
	  exit(-1);
	}
      else
	{
	  VRMLPath=value;
	  //VRMLPath+="/Controller/IOserver/robot/HRP2JRL/model/";
	  VRMLFileName="HRP2JRLmain.wrl";
	  SpecificitiesFileName = value;
	  //SpecificitiesFileName +="/Controller/IOserver/robot/HRP2JRL/etc/";
	  SpecificitiesFileName += "/HRP2Specificities.xml";
	  LinkJointRank = value;
	  //LinkJointRank += "/Controller/IOserver/robot/HRP2JRL/etc/";
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
      VRMLPath=argv[1];
      VRMLFileName=argv[2];
      SpecificitiesFileName = argv[3];
      LinkJointRank = argv[4];
    }


  // Creating the humanoid robot.
  CjrlHumanoidDynamicRobot * aHDR = 0, * aDebugHDR = 0;
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;

#ifndef WITH_HRP2DYNAMICS
  aHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
  aDebugHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
#else
# ifdef WIN32
#pragma message ( " -- !!! -- Warning : Compiled with HRP2DYNAMICS !" )
# else
#warning "Compiled with HRP2DYNAMICS !"
# endif
  Chrp2OptHumanoidDynamicRobot *aHRP2HDR= new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
  aHDR = aHRP2HDR;
  aDebugHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
#endif

  // Parsing the file.
  string RobotFileName = VRMLPath + VRMLFileName;
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,
					 LinkJointRank,
					 SpecificitiesFileName);

  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aDebugHDR,RobotFileName,
					 LinkJointRank,
					 SpecificitiesFileName);

  
  // Create Pattern Generator Interface
  PatternGeneratorInterface * aPGI;
  aPGI = patternGeneratorInterfaceFactory(aHDR);

  bool conversiontoradneeded=true;
  
  //  double * dInitPos = InitialPoses[INTERACTION_2008];
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
  
  string inProperty[5]={"TimeStep","ComputeAcceleration",
			"ComputeBackwardDynamics", "ComputeZMP",
			"ResetIteration"};
  string inValue[5]={"0.005","false","false","true","true"};
  
  for(unsigned int i=0;i<5;i++)
    aDebugHDR->setProperty(inProperty[i],
			   inValue[i]);
  
    
  //COMPosition CurrentWaistPosition;
  struct timeval begin,end,startingtime;
  unsigned long int NbOfIt=0, NbOfItToCompute=0;


  bool TestChangeFoot = true;

  unsigned int NbStepsModified = 0;

  COMPosition finalCOMPosition;
  FootAbsolutePosition LeftFootPosition;
  FootAbsolutePosition RightFootPosition;


  bool DebugFGPI = true;
  bool DebugZMP2 = true;
  bool DebugConfiguration = false;
  unsigned int PGIInterface = 0;
  
  double TimeProfile[200*620];
  bool bTimeProfile=true;
  double TimeProfileTS[200*620];
  unsigned int TimeProfileIndex = 0;
  unsigned int TimeProfileUpperLimit=200*620;

  ofstream aofzmpmb2;
  if (DebugZMP2)
    aofzmpmb2.open("ZMPMBSTAGE2.dat",ofstream::out);

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
  // Number of sequences added.
  unsigned int lNbItMax = 1;
  
  if (TestProfil==PROFIL_PB_FLORENT)
    lNbItMax = 3;

  for (unsigned int lNbIt=0;lNbIt<lNbItMax;lNbIt++)
    {
      //StrangeStartingPosition(*aPGI);
      cout << "<===============================================================>"<<endl;
      cout << "Iteration nb: " << lNbIt << endl;
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

	case PROFIL_HERDT:
	  Herdt(*aPGI);
	  break;
	  
	case PROFIL_HERDT_ONLINE:
	  HerdtOnline(*aPGI);
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
	  
	  if (NbOfIt>3)
	    {
	      if (TestProfil==PROFIL_SHORT_STRAIGHT_WALKING)
		{
		  aDebugHDR->currentConfiguration(PreviousConfiguration);	      
		  aDebugHDR->currentVelocity(PreviousVelocity);
		  aDebugHDR->currentAcceleration(PreviousAcceleration);
		  aDebugHDR->computeForwardKinematics();
		  
		  MAL_S3_VECTOR_TYPE(double) ZMPmultibody;
		  ZMPmultibody = aDebugHDR->zeroMomentumPoint();
		  if (DebugZMP2)
		    {
		      if (aofzmpmb2.is_open())
			{
			  aofzmpmb2 << ZMPmultibody[0] << " " << ZMPmultibody[1] << endl;
			}
		    }
		}
	    }
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
		  << ZMPTarget(0)*cos(CurrentConfiguration(5)) - ZMPTarget(1)*sin(CurrentConfiguration(5))+CurrentConfiguration(0) << " " 
		  << ZMPTarget(0)*sin(CurrentConfiguration(5)) + ZMPTarget(1)*cos(CurrentConfiguration(5))+CurrentConfiguration(1) << " "
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

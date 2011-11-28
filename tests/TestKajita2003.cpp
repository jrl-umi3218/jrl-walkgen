/*
 * Copyright 2010, 
 *
 * Olivier Stasse
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
/* \file This file tests A. Herdt's walking algorithm for
 * automatic foot placement giving an instantaneous CoM velocity reference.
 */
#include "CommonTools.hh"
#include "TestObject.hh"

#include <Debug.hh>
#include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>

using namespace::PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;
using namespace std;

enum Profiles_t {
  PROFIL_CIRCLE,                 // 1
  PROFIL_STRAIGHT_WALKING,              // 2
  PROFIL_PB_FLORENT_SEQ1,               // 3
  PROFIL_PB_FLORENT_SEQ2                // 4
};

class TestKajita2003: public TestObject
{

private:
public:
  TestKajita2003(int argc, char *argv[], string &aString, int TestProfile):
    TestObject(argc,argv,aString)
  {
    m_TestProfile = TestProfile;
  };


protected:

  virtual void SpecializedRobotConstructor(   CjrlHumanoidDynamicRobot *& aHDR,
					      CjrlHumanoidDynamicRobot *& aDebugHDR)
  {
    dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
    Chrp2OptHumanoidDynamicRobot *aHRP2HDR= new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
    aHDR = aHRP2HDR;
    aDebugHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
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


  void chooseTestProfile()
  {
    
    switch(m_TestProfile)
      {

      case PROFIL_CIRCLE:
	TurningOnTheCircle(*m_PGI);
	break;
      case PROFIL_STRAIGHT_WALKING:
	StraightWalking(*m_PGI);
	break;
      case PROFIL_PB_FLORENT_SEQ1:
	PbFlorentSeq1(*m_PGI);
	break;
      case PROFIL_PB_FLORENT_SEQ2:
	PbFlorentSeq2(*m_PGI);
	break;
      default:
	throw("No correct test profile");
	break;
      }
  }
  
  void generateEvent()
  {
  }
};

int PerformTests(int argc, char *argv[])
{

 std::string TestNames[4] = {  "TestKajita2003StraightWalking",
			       "TestKajita2003Circle",
                               "TestKajita2003PbFlorentSeq1",
                               "TestKajita2003PbFlorentSeq2"};
  int TestProfiles[4] = { PROFIL_STRAIGHT_WALKING,
			  PROFIL_CIRCLE,
                          PROFIL_PB_FLORENT_SEQ1,
                          PROFIL_PB_FLORENT_SEQ2};

  for (unsigned int i=0;i<4;i++)
    {
      TestKajita2003 aTK2003(argc,argv,
			    TestNames[i],
			    TestProfiles[i]);
      aTK2003.init();
      try
	{
	  if (!aTK2003.doTest(std::cout))
	    {
	      cout << "Failed test " << i << endl;
	      return -1;
	    }
	  else
	    cout << "Passed test " << i << endl;
	}
      catch (const char * astr)
	{ cerr << "Failed on following error " << astr << std::endl;
	  return -1; }
    }
  return 0;
}

int main(int argc, char *argv[])
{
  try
    {
      return PerformTests(argc,argv);
    }
  catch (const std::string& msg)
    {
      std::cerr << msg << std::endl;
    }
  return 1;
}



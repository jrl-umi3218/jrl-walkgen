/*
 * Copyright 2013, 
 *
 * Mauricio Garcia
 *
 * LAAS, CRNS
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
 */
/* \file This file tests A. Herdt's walking algorithm for
 * automatic foot placement giving an instantaneous CoM velocity reference.
 */
#include "CommonTools.hh"
#include "TestObject.hh"

using namespace::PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;
using namespace std;

enum Profiles_t {
  PROFIL_VS_ONLINE_WALKING                 // 1
};

class TestVS2013: public TestObject
{

private:
public:
  TestVS2013(int argc, char *argv[], string &aString, int PGIInterface, int TestProfile):
    TestObject(argc,argv,aString,PGIInterface)
  {
    m_TestProfile = TestProfile;
  };
  
  
protected:
  
  void startOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);
    
    {
      istringstream strm2(":SetAlgoForZmpTrajectory Garcia");
      aPGI.ParseCmd(strm2);
      
    }
    {
      istringstream strm2(":singlesupporttime 0.7");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":setNumberOfLandMarks 4");
      aPGI.ParseCmd(strm2);
    }
    {
      //istringstream strm2(":cameraintrinsicparameters 391.181 0.0 248.988 0.0 390.983 245.235 0.0 0.0 1.0");
      //aPGI.ParseCmd(strm2);
    }
    {
      //istringstream strm2(":setLandMarksPositions 5.0 -0.5 0.5 5 0.5 0.5 5.0 0.5 1.5 5.0 -0.5 1.5");
      //   istringstream strm2(":setLandMarksPositions 4.191465 0.412066 1.834407 4.442912 1.326137 1.773185 3.917841 1.345496 -0.094307 3.666394 0.431425 -0.033085 ");
      istringstream strm2(":setLandMarksPositions 4.191465 0.412066 1.834407 4.442912 1.326137 1.773185 3.917841 1.345496 -0.094307 3.666394 0.431425 -0.033085 ");
      aPGI.ParseCmd(strm2);
    }
    {
      //istringstream strm2(":setDesiredAngle 0.5236"); // 30 degree
      //istringstream strm2(":setDesiredAngle 0.1745"); // 10 degree
      istringstream strm2(":setDesiredAngle 0.0");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":setAngleErrorGain 0.05");
      aPGI.ParseCmd(strm2);
    }
    {
      // Position of final landmarks
      //istringstream strm2(":setFinalLandMarks 300.0005 296.0929 401.0587 285.3273 431.3475 39.6418 313.0407 29.1062"); // (,)
      istringstream strm2(":setFinalLandMarks 0.13041 0.13008 0.38875 0.10254 0.46618 -0.52584 0.16374 -0.55278 0.13041 0.13008");


      //istringstream strm2(":setFinalLandMarks 1.5146 0.4996 0.8232 0.3933 0.8232 -0.0326 1.5146 -0.0415"); // (2,1,30deg)
      //istringstream strm2(":setFinalLandMarks 0.7417 0.3428 0.3534 0.3220 0.3534 -0.0267 0.7417 -0.0284"); // (2,1,10deg)
      //istringstream strm2(":setFinalLandMarks 0.5000 0.3078 0.1667 0.3078 0.1667 -0.0255 0.5000 .0.0255"); //(2,1,0)
      //istringstream strm2(":setFinalLandMarks 0.16667 0.30779 -0.16667 0.30779 -0.16667 -0.025539 0.16667 -0.025539"); // (2,0,0)
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":VSOnline 1");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":numberstepsbeforestop 2");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":setfeetconstraint XY 0.02 0.02");
      aPGI.ParseCmd(strm2);
    }

  }

  void stopOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setWalk 0");
      aPGI.ParseCmd(strm2);
    }
  }

  void chooseTestProfile()
  {
    
    switch(m_TestProfile)
      {

      case PROFIL_VS_ONLINE_WALKING:
	startOnLineWalking(*m_PGI);
	break;
      default:
	throw("No correct test profile");
	break;
      }
  }
  
  void generateEvent()
  {
    unsigned int StoppingTime =75.0*200;

    if (m_OneStep.NbOfIt>StoppingTime) 
      {
	stopOnLineWalking(*m_PGI);
      }
  }
};

int PerformTests(int argc, char *argv[])
{

  std::string TestNames[1] = { "TestVS2013OnLine"};
  int TestProfiles[1] = { PROFIL_VS_ONLINE_WALKING};
  int PGIInterface = 2;

  for (unsigned int i=0;i<1;i++)
    {
      TestVS2013 aTVS2013(argc,argv,
			  TestNames[i],
			  PGIInterface,			  
			  TestProfiles[i]);
      aTVS2013.init();
      try
	{
	  if (!aTVS2013.doTest(std::cout))
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



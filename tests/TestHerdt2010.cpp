/*
 * Copyright 2010, 
 *
 * Andrei Herdt
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
#include "Debug.hh"
#include "CommonTools.hh"
#include "TestObject.hh"

using namespace::PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;
using namespace std;

enum Profiles_t {
  PROFIL_HERDT_ONLINE_WALKING                 // 1
};

class TestHerdt2010: public TestObject
{

private:
public:
  TestHerdt2010(int argc, char *argv[], string &aString, int TestProfile):
    TestObject(argc,argv,aString)
  {
    m_TestProfile = TestProfile;
  };
  
  
  
protected:

  

  
  void startOnLineWalking(PatternGeneratorInterface &aPGI)
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
      istringstream strm2(":doublesupporttime 0.1");
      aPGI.ParseCmd(strm2);
    }
    {
      //istringstream strm2(":HerdtOnline");
      istringstream strm2(":HerdtOnline 0.2 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":numberstepsbeforestop 2");
      aPGI.ParseCmd(strm2);
    }
  }

  void startTurningLeft(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.2 0.0 6.0832");
      aPGI.ParseCmd(strm2);
    }
  }
  void startTurningRight(PatternGeneratorInterface &aPGI)
  {
    {
      //istringstream strm2(":setVelReference  0.2 0.0 -0.2");
      istringstream strm2(":setVelReference  0.2 0.0 -6.0832");
      aPGI.ParseCmd(strm2);
    }
  }

  void startTurningLeftOnSpot(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 0.0 10.0");
      aPGI.ParseCmd(strm2);
    }
  }

  void startTurningRightOnSpot(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 0.0 -10.");
      aPGI.ParseCmd(strm2);
    }
  }
  void stop(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
  }
  void walkForward(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.2 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
  }
  void walkSidewards(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 0.2 0.0");
      aPGI.ParseCmd(strm2);
    }
  }

  void stopOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 0.0 0.0");
      aPGI.ParseCmd(strm2);
      istringstream strm3(":stoppg");
      aPGI.ParseCmd(strm3);
    }
  }

  void chooseTestProfile()
  {
    
    switch(m_TestProfile)
      {

      case PROFIL_HERDT_ONLINE_WALKING:
	startOnLineWalking(*m_PGI);
	break;
      default:
	throw("No correct test profile");
	break;
      }
  }
  
  void generateEvent()
  {

    typedef void (TestHerdt2010::* localeventHandler_t)(PatternGeneratorInterface &);

    struct localEvent 
    {
      unsigned time;
      localeventHandler_t Handler ;
    };

    #define localNbOfEvents 12
    struct localEvent events [localNbOfEvents] =
      { { 5*200,&TestHerdt2010::walkForward},
        {10*200,&TestHerdt2010::walkSidewards},
        {25*200,&TestHerdt2010::startTurningRightOnSpot},
        {35*200,&TestHerdt2010::walkForward},
        {45*200,&TestHerdt2010::startTurningLeftOnSpot},
        {55*200,&TestHerdt2010::walkForward},
        {65*200,&TestHerdt2010::startTurningRightOnSpot},
        {75*200,&TestHerdt2010::walkForward},
        {85*200,&TestHerdt2010::startTurningLeft},
	{95*200,&TestHerdt2010::startTurningRight},
	{105*200,&TestHerdt2010::stop},
	{110*200,&TestHerdt2010::stopOnLineWalking}};
    
    // Test when triggering event.
    for(unsigned int i=0;i<localNbOfEvents;i++)
      { 
	if ( m_OneStep.NbOfIt==events[i].time)
	  {
            ODEBUG3("********* GENERATE EVENT ***********");
	    (this->*(events[i].Handler))(*m_PGI);
	  }
      }
  }
};

int PerformTests(int argc, char *argv[])
{

  std::string TestNames[1] = { "TestHerdt2010OnLine"};
  int TestProfiles[1] = { PROFIL_HERDT_ONLINE_WALKING};

  for (unsigned int i=0;i<1;i++)
    {
      TestHerdt2010 aTH2010(argc,argv,
			    TestNames[i],
			    TestProfiles[i]);
      aTH2010.init();
      try
	{
	  if (!aTH2010.doTest(std::cout))
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


